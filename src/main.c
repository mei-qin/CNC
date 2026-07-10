#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <stdatomic.h>
#include "global_def.h"
#include "axis_ctrl.h"
#include "kinematics.h"
#include "smc_api.h"
#include "macro_eval.h"   // 宏变量环境初始化
#include "sim_drive.h"    // g_sim_rt_cycle (RT cycle 诊断)

// ============================================================================
// 动态轴坐标信息构建（安全 snprintf 拼接，防溢出）
// ============================================================================
static const char g_axis_letters[] = "XYZABCUVW";

static void build_axis_info_str(char* buf, int buf_size) {
    int offset = 0;
    buf[0] = '\0';
    for(int i = 0; g_axis_letters[i] != '\0'; i++) {
        char letter = g_axis_letters[i];
        if(!SMC_IsAxisConfigured(letter)) continue;
        double pos = SMC_GetLogicalPos(letter);
        int idx = g_axis_map[letter - 'A'];
        const char *unit = (g_axis[idx].axis_type == 1) ? "\xc2\xb0" : "mm";
        int remaining = buf_size - offset;
        if(remaining <= 1) break;
        int written = snprintf(buf + offset, remaining,
                                "%c:%8.3f %s  ", letter, pos, unit);
        if(written < 0 || written >= remaining) break;
        offset += written;
    }
}

// ============================================================================
// 辅助函数：阻塞等待运动完成，并实时刷新动态状态栏
// ============================================================================

// 诊断: cycle 速率计算 (每次 print_status_line 调用时计算增量)
// 显示格式: cyc_rt=N (delta_cycles / delta_ms ≈ kHz)
static uint64_t g_last_cycle = 0;
static double   g_last_ts_sec = 0.0;
static int      g_sim_debug = -1;   // -1=未初始化, 0=简洁, 1=详细诊断

extern _Atomic int g_sys_alarm_state;

static double monotonic_ts_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static void print_status_line(void) {
    char status_str[16];
    char axis_info[256];
    SMC_GetSystemStatusStr(status_str, sizeof(status_str));
    build_axis_info_str(axis_info, sizeof(axis_info));
    int q_cnt = SMC_GetQueueCount();

    // 首次调用时读 env (默认简洁模式)
    if (g_sim_debug < 0) {
        const char *env = getenv("SIM_RT_DEBUG");
        g_sim_debug = env ? atoi(env) : 0;
    }

    // RT cycle 速率 (轻量监控, 默认显示)
    uint64_t cur_cycle = atomic_load(&g_sim_rt_cycle);
    double   cur_ts    = monotonic_ts_sec();
    double   dt        = cur_ts - g_last_ts_sec;
    double   cyc_khz   = (dt > 1e-6) ? (cur_cycle - g_last_cycle) / dt / 1000.0 : 0.0;
    g_last_cycle = cur_cycle;
    g_last_ts_sec = cur_ts;

    int alarm = atomic_load(&g_sys_alarm_state);

    if (g_sim_debug) {
        // 详细诊断模式 (SIM_RT_DEBUG=1): 显示 S 曲线 phase + feedhold 状态机
        int    ph     = g_interpolator.current_phase;
        double vt     = g_interpolator.virtual_time_ms;
        double T7     = g_interpolator.T7;
        int    is_mv  = g_interpolator.is_moving;
        double tscal  = g_interpolator.time_scale;
        int    hstate = g_interpolator.hold_state;     // 0=NORMAL 1=BRAKING 2=PAUSED 3=RESUMING
        int    pausereq = g_interpolator.pause_request;
        int    op_ready = g_all_axis_op_ready;

        printf("\r\033[K[%-5s] Q:%3d | cyc=%llu (%.2fkHz) ALM=%d | "
               "ph=%d vt=%.1f/%.1f mv=%d ts=%.2f hs=%d pr=%d op=%d | %s",
               status_str, q_cnt,
               (unsigned long long)cur_cycle, cyc_khz, alarm,
               ph, vt, T7, is_mv, tscal, hstate, pausereq, op_ready, axis_info);
    } else {
        // 简洁模式 (默认): Q + cycle 速率 + 报警 + 坐标
        printf("\r\033[K[%-5s] Q:%3d | cyc=%llu (%.2fkHz) ALM=%d | %s",
               status_str, q_cnt,
               (unsigned long long)cur_cycle, cyc_khz, alarm, axis_info);
    }
    fflush(stdout);
}

void wait_and_print_status() {
    usleep(100000);
    int timeout_s = 0;
    while(SMC_IsParserRunning() || !SMC_IsMotionDone()) {
        print_status_line();
        usleep(50000);
        if (++timeout_s > 1200) {  /* 50ms * 1200 = 60s */
            printf("\n[警告] 等待运动完成超时 (60s), 强制返回菜单\n");
            break;
        }
    }
    print_status_line();
    printf("\n");
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("用法: sudo %s <EtherCAT网卡名> (例如: sudo ./cnc_core eth0)\n", argv[0]);
        printf("      sudo %s sim                  (纯软件仿真模式, 无需硬件)\n", argv[0]);
        return 0;
    }

    // ---- 仿真模式检测 ----
    if (strcmp(argv[1], "sim") == 0) {
        g_sim_mode = 1;
        printf("[SIM] *** 纯软件仿真模式已激活 ***\n");
    }

    printf("\n==============================================\n");
    printf("     SMC 五轴高端数控系统内核 (V2.0) \n");
    if (g_sim_mode) printf("     [SIMULATION MODE - 无真实硬件]\n");
    printf("==============================================\n");

    // ==========================================
    // 1. 系统底层内存与互斥锁初始化
    // ==========================================
    axis_sys_init();

    // ==========================================
    // 1.1 激光切割子系统初始化 (Phase A)
    // 默认 slave_id = -1 即"未配置激光"; RT 路径会安全跳过 PDO 输出
    // 用户接激光硬件时通过 SMC_ConfigLaserIO/DOBits/DIBits/AOChannels/Range 显式启用
    // ==========================================
    laser_ctrl_init();
    SMC_ConfigLaserIO(0, 0, 0);  // [临时] sim 模式下启用激光字段输出

    // ==========================================
    // 1.5 宏变量环境初始化（#500-#999 永久区清零，避免上电随机值）
    // ==========================================
    Macro_Init();

    // ==========================================
    // 2. 硬件拓扑配置（字母化 API，底层自动分配房间号）
    // ==========================================
    SMC_ConfigAxisTopology("X", 0, 5, 0);
    SMC_ConfigAxisTopology("Y", 1, 3, 4); // Y轴双驱 (主站3, 从站4)
    SMC_ConfigAxisTopology("Z", 0, 6, 0);
    SMC_ConfigAxisTopology("C", 0, 1, 0);
    SMC_ConfigAxisTopology("B", 0, 2, 0);

    // ==========================================
    // 3. 脉冲当量配置 (1 单位对应多少个脉冲)
    // ==========================================
    SMC_ConfigPulsePerUnit('X', 10000.0);   // 1 mm = 10000 脉冲
    SMC_ConfigPulsePerUnit('Y', 10000.0);   // 1 mm = 10000 脉冲
    SMC_ConfigPulsePerUnit('Z', 1000.0);    // 1 mm = 1000 脉冲
    SMC_ConfigPulsePerUnit('C', 2777.7778); // 1 度 = 2777.8 脉冲
    SMC_ConfigPulsePerUnit('B', 2777.7778); // 1 度 = 2777.8 脉冲

    // ==========================================
    // 4. 动力学约束与量纲护城河配置
    // 参数: (轴字母, 类型[0线/1旋], 最大速度, 最大加速度, 最大减速度, 等效半径)
    // ==========================================
    SMC_ConfigAxisDynamics('X', 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics('Y', 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics('Z', 0, 30.0, 100.0, 100.0, 0.0); // Z轴带主轴较重，参数保守

    // 旋转轴 (速度 度/s, 加速度 度/s^2, 等效半径 mm)
    SMC_ConfigAxisDynamics('C', 1, 18.0, 72.0, 72.0, 50.0);
    SMC_ConfigAxisDynamics('B', 1, 18.0, 72.0, 72.0, 80.0);

    // ==========================================
    // 4.5 启动通用五轴运动学引擎 (Kinematics Chain)
    // ==========================================
    // 假设：刀具长度 150mm，上方 B 轴旋转中心离下方 C 轴旋转中心的高度差为 200mm
    double tool_off[3]  = {0.0, 0.0, 150.0};
    double pivot_off[3] = {0.0, 0.0, 200.0};

    // 配置为 Head-Head 构型：
    // 第 1 旋转轴：B 轴 (映射表找索引)，绕 Y 轴旋转 (1)
    // 第 2 旋转轴：C 轴 (映射表找索引)，绕 Z 轴旋转 (2)
    SMC_ConfigKinematicsOffset(150.0, 0.0, 0.0, 200.0); // 供记录用
    SMC_ConfigKinematics(KIN_HEAD_HEAD, 
                         g_axis_map['B'-'A'], 1, 
                         g_axis_map['C'-'A'], 2, 
                         tool_off, pivot_off);

    // ==========================================
    // 5. 规划器参数与安全预警配置
    // ==========================================
    SMC_ConfigPlannerParams(0.05, 500.0);
    SMC_ConfigGantrySyncAlarm('Y', 1, 1000, 8000, 100); // Y 轴龙门同步防撕裂
    SMC_ConfigSoftLimit('Z', 1, -500.0, 200.0);          // Z 轴防撞软限位 (放宽以适应RTCP逆解)

    // ==========================================
    // 6. 启动 EtherCAT 内核
    // ==========================================
    if (SMC_InitAndStart(argv[1]) != 0) {
        return -1;
    }

    while(!g_all_axis_op_ready){
        osal_usleep(100000);
    }

    printf("\n[系统] 上电物理原点已锚定 (G53 机械坐标建立)。\n");

    // 默认给当前位置设一个 G54 工件原点，方便直接跑代码
    SMC_SetZero(SMC_AXIS_ALL);
    sleep(1);
    printf("[系统] G54 逻辑坐标系初始化完毕！准备接受操作员指令。\n");

    // ==========================================
    // 7. 交互式控制台主循环
    // ==========================================
    int choice = -1;
    while (1)
    {
        printf("\n----------------------------------------------\n");
        printf("  1. 回归原点 (回到 G54 工件坐标 0,0,0,0,0)\n");
        printf("  2. 设定原点 (将当前位置锚定为新 G54 原点)\n");
        printf("  3. 相对点动 (智能量纲 JOG 面板)\n");
        printf("  4. 加工图纸 (运行 G 代码文件)\n");
        printf("  5. 刹车测试 (运行中途触发 Feedhold 暂停)\n");
        printf("  0. 关机退出\n");
        printf("----------------------------------------------\n");
        printf("请输入操作编号 (0-5): ");

        if (scanf("%d", &choice) != 1) {
            while(getchar() != '\n');
            continue;
        }

        switch (choice)
        {
            case 1:
                printf("\n[执行] 正在全轴联动回归 G54 原点...\n");
                SMC_GoZero(SMC_AXIS_ALL, 1500.0);
                wait_and_print_status();
                break;

            case 2:
                printf("\n[执行] 正在将当前物理位置设定为全新的 G54 零点...\n");
                SMC_SetZero(SMC_AXIS_ALL);
                printf("[成功] 坐标系偏移矩阵已更新！\n");
                break;

            case 3:
            {
                char ax = 'X';
                double dist = 0.0, spd = 1000.0;
                printf("\n--- 智能量纲 JOG 面板 ---\n");
                printf("请选择轴 (X/Y/Z/A/B, *=全轴): ");
                scanf(" %c", &ax);
                ax = toupper((unsigned char)ax);

                // 智能单位提示
                if (ax == 'A' || ax == 'B') {
                    printf("请输入移动角度 (度, 正负代表方向): ");
                    scanf("%lf", &dist);
                    printf("请输入移动速度 (度/s): ");
                    scanf("%lf", &spd);
                } else if (ax >= 'X' && ax <= 'Z') {
                    printf("请输入移动距离 (mm, 正负代表方向): ");
                    scanf("%lf", &dist);
                    printf("请输入移动速度 (mm/s): ");
                    scanf("%lf", &spd);
                } else if (ax == '*') {
                    printf("全轴测试，请输入统一位移数值: ");
                    scanf("%lf", &dist);
                    printf("请输入统一目标速度: ");
                    scanf("%lf", &spd);
                } else {
                    printf("[错误] 无效轴字母 '%c'\n", ax);
                    break;
                }

                printf("\n[执行] 下发 JOG 指令: 轴[%c] 移动 %.3f, 速度 %.1f...\n", ax, dist, spd);
                SMC_MoveRelative(ax, dist, spd);
                wait_and_print_status();
                break;
            }

            case 4:
            {
                char filename[64];
                printf("\n请输入要运行的 G代码 文件名 (如 test.txt): ");
                scanf("%s", filename);
                printf("[执行] 开始后台加工图纸 %s...\n", filename);
                if (SMC_RunGCodeFile(filename) == 0) {
                    wait_and_print_status();
                    printf("\n[成功] 加工完美结束！零件已产出！\n");
                }
                break;
            }

            case 5:
                printf("\n[执行] 启动加工 test.txt，并在 3 秒后触发平滑刹车暂停测试...\n");
                if (SMC_RunGCodeFile("test.txt") == 0) {

                    for(int i=0; i<30; i++) {
                        print_status_line();
                        usleep(100000);
                    }

                    printf("\n\n[干预] 收到进给保持(Feedhold)指令！系统正在平滑刹车...\n");
                    SMC_PauseProcessing();

                    for(int i=0; i<30; i++) {
                        print_status_line();
                        usleep(100000);
                    }

                    printf("\n\n[干预] 收到恢复(Resume)指令！系统正在平滑加速恢复轨迹...\n");
                    SMC_ResumeProcessing();

                    wait_and_print_status();
                }
                break;

            case 0:
                printf("\n[退出] 收到关机指令，准备断开伺服动力...\n");
                SMC_Close();
                return 0;

            default:
                printf("\n[错误] 无效的指令编号！\n");
                break;
        }
    }

    return 0;
}
