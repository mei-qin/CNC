#include "smc_api.h"
#include "global_def.h"
#include "ecat_core.h"
#include "axis_ctrl.h"
#include "gcode_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>

// 底层轴房间号自动分配计数器
static int g_allocated_axis_count = 0;

// 字母 → 底层索引查表（内部工具函数）
// 返回 -1 表示轴未配置
static int axis_lookup(char axis_letter) {
    char c = toupper((unsigned char)axis_letter);
    if(c < 'A' || c > 'Z') return -1;
    return g_axis_map[c - 'A'];
}

// ================== 系统管理 ==================
int SMC_InitAndStart(const char *netif_name)
{
    printf("[SMC_API] 正在初始化运动控制内核...\n");

    if (!osal_thread_create_rt(&thread_rt, 128000, &ecat_thread_rt, NULL)) {
        printf("[SMC_API] 实时控制线程创建失败！\n"); return -1;
    }
    if (!osal_thread_create(&thread_chk, 128000, &ecat_thread_chk, NULL)) {
        printf("[SMC_API] 故障检查线程创建失败！\n"); return -1;
    }
    if(!osal_thread_create(&thread_parser, 128000, &parser_thread_func, NULL)){
        printf("[SMC_API] G-code解析线程创建失败！\n"); return -1;
    }

    ecat_bringup((char*)netif_name);

    printf("[SMC_API] 等待伺服全轴使能并进入 CSP 同步...\n");
    int timeout = 50; // 最多等5秒
    while (!g_all_axis_op_ready && timeout > 0) {
        osal_usleep(100000);
        timeout--;
    }
    if (timeout == 0) {
        printf("[SMC_API] 伺服使能超时！请检查硬件。\n"); return -1;
    }

    // 给系统时钟收敛留时间
    osal_usleep(1000000);
    printf("[SMC_API] 内核启动完毕，伺服就绪！\n");
    return 0;
}

void SMC_Close(void)
{
    printf("\n[SMC_API] 收到关闭请求，触发优雅下电时序...\n");

    // 请求 RT 线程进入优雅下电状态机（抱闸闭合 + CiA402 降级）
    dorun = 2;

    // 阻塞等待 RT 线程完成下电状态机（最多 3 秒）
    int wait_cycles = 30;
    while (dorun != 0 && wait_cycles > 0) {
        osal_usleep(100000);
        wait_cycles--;
    }
    if (dorun != 0) {
        printf("[SMC_API] 优雅下电超时（3s），强制降级！\n");
        dorun = 0;
        osal_usleep(200000);
    } else {
        printf("[SMC_API] RT 线程优雅下电完成。\n");
    }

    // 降级 EtherCAT 状态机：OP → SAFE_OP → INIT
    ctx.slavelist[0].state = EC_STATE_SAFE_OP;
    ecx_writestate(&ctx, 0);
    if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
        printf("[ECAT警告] 切换SAFE_OP失败，但继续执行降级流程\n");
    }
    ctx.slavelist[0].state = EC_STATE_INIT;
    ecx_writestate(&ctx, 0);
    if (ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE) != EC_STATE_INIT) {
        printf("[ECAT警告] 切换INIT失败，但继续关闭主站\n");
    }
    ecx_close(&ctx);
    printf("[SMC_API] 系统已安全关闭！\n");
}

// ================== 轴配置 ==================

// @Context: Non-RealTime Background Thread（仅初始化阶段调用）
// @Thread-Safety: 写入 g_axis[] 和 g_axis_map[]，仅允许初始化阶段调用
// axis_name 首字母决定映射槽位，底层自动分配连续的数组房间号
int SMC_ConfigAxisTopology(const char* axis_name, int is_dual_drive, int master_id, int slave_id){
    if(axis_name == NULL || axis_name[0] == '\0') {
        printf("[SMC_API] 轴名不能为空！\n");
        return -1;
    }

    // 检查房间号是否用尽
    if(g_allocated_axis_count >= AXIS_NUM) {
        printf("[SMC_API] 轴房间号已满(%d/%d)，无法继续分配 '%s'！\n",
               g_allocated_axis_count, AXIS_NUM, axis_name);
        return -1;
    }

    // 提取首字母并校验
    char letter = toupper((unsigned char)axis_name[0]);
    if(letter < 'A' || letter > 'Z') {
        printf("[SMC_API] 轴名首字母 '%c' 不在 A-Z 范围内！\n", axis_name[0]);
        return -1;
    }

    // 检查该字母的映射槽是否已被占用
    if(g_axis_map[letter - 'A'] >= 0) {
        printf("[SMC_API] 字母 '%c' 已被轴 '%s' 占用，拒绝重复映射！\n",
               letter, g_axis[g_axis_map[letter - 'A']].axis_name);
        return -1;
    }

    // 自动分配房间号
    int axis_idx = g_allocated_axis_count;

    // 写入轴名
    strncpy(g_axis[axis_idx].axis_name, axis_name, sizeof(g_axis[axis_idx].axis_name) - 1);
    g_axis[axis_idx].axis_name[sizeof(g_axis[axis_idx].axis_name) - 1] = '\0';

    // 注册字母 → 底层索引映射
    g_axis_map[letter - 'A'] = axis_idx;

    // 拓扑赋值
    if(is_dual_drive){
        g_axis[axis_idx].slave_ids[0] = master_id;
        g_axis[axis_idx].slave_ids[1] = slave_id;
        g_axis[axis_idx].slave_count  = 2;
        printf("[SMC_API] 配置 '%s' → 房间[%d], 字母 '%c', 双驱 主ID:%d 从ID:%d\n",
               axis_name, axis_idx, letter, master_id, slave_id);
    } else {
        g_axis[axis_idx].slave_ids[0] = master_id;
        g_axis[axis_idx].slave_count  = 1;
        printf("[SMC_API] 配置 '%s' → 房间[%d], 字母 '%c', 单驱 ID:%d\n",
               axis_name, axis_idx, letter, master_id);
    }

    g_allocated_axis_count++;
    return 0;
}

int SMC_ConfigSoftLimit(char axis_letter, int enable, double neg_limit_mm, double pos_limit_mm){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置软限位！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    g_axis[idx].enable_soft_limit = enable;
    g_axis[idx].soft_limit_neg    = neg_limit_mm;
    g_axis[idx].soft_limit_pos    = pos_limit_mm;
    return 0;
}

int SMC_ConfigGantrySyncAlarm(char axis_letter, int enable, int32_t tolerance_pulse, int32_t max_error_pulse, int time_ms){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置龙门同步报警！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    g_axis[idx].enable_sync_alarm    = enable;
    g_axis[idx].sync_tolerance_pulse = tolerance_pulse;
    g_axis[idx].sync_max_err_pulse   = max_error_pulse;
    g_axis[idx].sync_err_time_ms     = time_ms;
    g_axis[idx]._current_sync_timer  = 0;
    return 0;
}

void SMC_ConfigPulsePerUnit(char axis_letter, double pulse_per_unit){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置脉冲当量！\n", toupper((unsigned char)axis_letter));
        return;
    }
    g_axis[idx].pulse_per_unit = pulse_per_unit;
}

// @Thread-Safety: Requires atomic operations or lock-free design.
// 写入 g_axis[] 全局数组，仅允许在初始化阶段（加工停止时）调用。
int SMC_ConfigAxisDynamics(char axis_letter, int type, double max_v, double max_a, double max_d, double equivalent_radius){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API ERROR] 轴 '%c' 未配置！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    if(g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API ERROR] 系统运行中（队列未空或插补器未静止），禁止修改轴动力学参数！\n");
        return -1;
    }
    if(max_v <= 0.0 || max_a <= 0.0 || max_d <= 0.0) {
        printf("[SMC_API ERROR] %s 动力学参数必须大于0！Vmax=%.1f, Amax=%.1f, Dmax=%.1f\n",
               g_axis[idx].axis_name, max_v, max_a, max_d);
        return -1;
    }
    if(type == 1 && equivalent_radius <= 0.0) {
        printf("[SMC_API ERROR] %s 为旋转轴，equivalent_radius 必须大于0！\n",
               g_axis[idx].axis_name);
        return -1;
    }
    g_axis[idx].axis_type          = type;
    g_axis[idx].equivalent_radius  = equivalent_radius;
    __sync_synchronize();
    g_axis[idx].max_speed = max_v;
    g_axis[idx].max_acc   = max_a;
    g_axis[idx].max_dec   = max_d;
    __sync_synchronize();
    printf("[SMC_API] %s 动力学: type=%d, Vmax=%.1f, Amax=%.1f, Dmax=%.1f, eq_radius=%.2f\n",
           g_axis[idx].axis_name, type, max_v, max_a, max_d, equivalent_radius);
    return 0;
}

// @Thread-Safety: Requires atomic operations or lock-free design.
int SMC_ConfigPlannerParams(double tolerance, double max_centripetal_acc){
    if(g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API ERROR] 系统运行中（队列未空或插补器未静止），禁止修改规划器参数！\n");
        return -1;
    }
    if(tolerance <= 0.0 || max_centripetal_acc <= 0.0) {
        printf("[SMC_API ERROR] 规划器参数必须大于0！tolerance=%.4f, centripetal_acc=%.1f\n",
               tolerance, max_centripetal_acc);
        return -1;
    }
    __sync_synchronize();
    g_planner_config.corner_tolerance    = tolerance;
    g_planner_config.max_centripetal_acc = max_centripetal_acc;
    __sync_synchronize();
    printf("[SMC_API] 规划器参数: tolerance=%.4f, centripetal_acc=%.1f\n",
           tolerance, max_centripetal_acc);
    return 0;
}

// ================== 坐标与状态 ==================
double SMC_GetLogicalPos(char axis_letter) {
    if(axis_letter == SMC_AXIS_ALL) return 0.0;
    int idx = axis_lookup(axis_letter);
    if(idx < 0) return 0.0;
    return g_coord_mgr.current_logical_pos[idx];
}

int SMC_IsParserRunning() { return g_parser_ctrl.is_running; }
int SMC_IsMotionDone() { return is_trajectory_finished(); }
int SMC_GetQueueCount() {
    return (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
}

// @Thread-Safety: 只读 g_axis_map[]，初始化后不变
int SMC_IsAxisConfigured(char axis_letter) {
    char c = toupper((unsigned char)axis_letter);
    if(c < 'A' || c > 'Z') return 0;
    return (g_axis_map[c - 'A'] >= 0) ? 1 : 0;
}

// @Thread-Safety: 使用 atomic_load_explicit 读取原子变量，
// 普通 int 变量 (is_paused/is_running) 为 UI 显示容许短暂撕裂，不会触发总线错误
void SMC_GetSystemStatusStr(char* out_str, int max_len) {
    if(out_str == NULL || max_len <= 0) return;
    if(atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) == 1) {
        snprintf(out_str, max_len, "ALARM");
    } else if(g_parser_ctrl.is_paused == 1) {
        snprintf(out_str, max_len, "HOLD");
    } else if(g_parser_ctrl.is_running == 1 ||
              atomic_load_explicit(&g_interpolator.is_moving, memory_order_acquire) == 1) {
        snprintf(out_str, max_len, "RUN");
    } else {
        snprintf(out_str, max_len, "IDLE");
    }
}

// ================== 运动控制 ==================
void SMC_SetZero(char axis_letter) {
    if(axis_letter == SMC_AXIS_ALL) { api_set_zero(AXIS_ALL); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设零！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_set_zero(idx);
}

void SMC_MoveRelative(char axis_letter, double distance, double speed) {
    if(axis_letter == SMC_AXIS_ALL) { api_move_relative(AXIS_ALL, distance, speed); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法点动！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_move_relative(idx, distance, speed);
}

void SMC_GoZero(char axis_letter, double speed) {
    if(axis_letter == SMC_AXIS_ALL) { api_go_zero(AXIS_ALL, speed); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法回零！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_go_zero(idx, speed);
}

// ================== G代码加工 ==================
int SMC_RunGCodeFile(const char *filepath) {
    if (g_parser_ctrl.is_running) return -1;
    strncpy(g_parser_ctrl.filepath, filepath, sizeof(g_parser_ctrl.filepath)-1);
    g_parser_ctrl.abort_request = 0;
    g_parser_ctrl.is_paused = 0;
    g_parser_ctrl.is_running = 1;
    return 0;
}

void SMC_AbortProcessing() { g_parser_ctrl.abort_request = 1; }

void SMC_PauseProcessing(){
    g_parser_ctrl.is_paused = 1;
    api_motion_pause();
}

void SMC_ResumeProcessing(){
    g_parser_ctrl.is_paused = 0;
    api_motion_resume();
}
