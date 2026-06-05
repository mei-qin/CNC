#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "global_def.h"
#include "axis_ctrl.h"
#include "smc_api.h" 

// ============================================================================
// 辅助函数：阻塞等待运动完成，并实时刷新坐标大屏
// ============================================================================
void wait_and_print_status() {
    usleep(100000); // 稍微延时，等待指令进入队列
    
    while(SMC_IsParserRunning() || !SMC_IsMotionDone()) {
        double px = SMC_GetLogicalPos(SMC_AXIS_X);
        double py = SMC_GetLogicalPos(SMC_AXIS_Y);
        double pz = SMC_GetLogicalPos(SMC_AXIS_Z);
        double pa = SMC_GetLogicalPos(SMC_AXIS_A);
        double pb = SMC_GetLogicalPos(SMC_AXIS_B);
        int q_cnt = SMC_GetQueueCount();
        
        // 大屏区分量纲：XYZ 为 mm，AB 为度(°)
        printf("\r[监控大屏] 队列:%3d | X:%8.3f mm  Y:%8.3f mm  Z:%8.3f mm  A:%8.3f °  B:%8.3f °  ", 
               q_cnt, px, py, pz, pa, pb);
        fflush(stdout);
        usleep(50000); // 50ms 刷新率 (20FPS)
    }
    
    // 运动彻底结束，打印最终坐标
    printf("\r[监控大屏] 队列:%3d | X:%8.3f mm  Y:%8.3f mm  Z:%8.3f mm  A:%8.3f °  B:%8.3f °  ", 
           0, 
           SMC_GetLogicalPos(SMC_AXIS_X), SMC_GetLogicalPos(SMC_AXIS_Y), 
           SMC_GetLogicalPos(SMC_AXIS_Z), SMC_GetLogicalPos(SMC_AXIS_A),
           SMC_GetLogicalPos(SMC_AXIS_B));
    printf("\n[提示] 运动已安全完成！\n");
}

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("用法: sudo %s <EtherCAT网卡名> (例如: sudo ./cnc_core eth0)\n", argv[0]); 
        return 0;
    }

    printf("\n==============================================\n");
    printf("     🚀 SMC 五轴高端数控系统内核 (V2.0) \n");
    printf("==============================================\n");

    // ==========================================
    // 1. 系统底层内存与互斥锁初始化
    // ==========================================
    axis_sys_init();
    
    // ==========================================
    // 2. 硬件拓扑与总线映射配置
    // ==========================================
    SMC_ConfigAxisTopology(SMC_AXIS_X, "X轴", 0, 5, 0);
    SMC_ConfigAxisTopology(SMC_AXIS_Y, "Y轴", 1, 3, 4); // Y轴双驱 (主站3, 从站4)
    SMC_ConfigAxisTopology(SMC_AXIS_Z, "Z轴", 0, 6, 0);
    SMC_ConfigAxisTopology(SMC_AXIS_A, "B轴", 0, 1, 0);
    SMC_ConfigAxisTopology(SMC_AXIS_B, "C轴", 0, 2, 0);

    // ==========================================
    // 3. 脉冲当量配置 (1 单位对应多少个脉冲)
    // ==========================================
    SMC_ConfigPulsePerUnit(SMC_AXIS_X, 10000.0); // 1 mm = 10000 脉冲
    SMC_ConfigPulsePerUnit(SMC_AXIS_Y, 10000.0); // 1 mm = 10000 脉冲
    SMC_ConfigPulsePerUnit(SMC_AXIS_Z, 1000.0);  // 1 mm = 1000 脉冲
    SMC_ConfigPulsePerUnit(SMC_AXIS_A, 2777.7778); // 1 度 = 1000 脉冲
    SMC_ConfigPulsePerUnit(SMC_AXIS_B, 2777.7778); // 1 度 = 1000 脉冲

    // ==========================================
    // 4. 动力学约束与量纲护城河配置 (极其核心)
    // 参数: (轴号, 类型[0线/1旋], 最大速度, 最大加速度, 最大减速度, 等效半径)
    // ==========================================
    // 线性轴 (速度 mm/s, 加速度 mm/s^2)
    SMC_ConfigAxisDynamics(SMC_AXIS_X, 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics(SMC_AXIS_Y, 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics(SMC_AXIS_Z, 0, 30.0, 100.0, 100.0, 0.0); // Z轴带主轴较重，参数保守
    
    // 旋转轴 (速度 度/s, 加速度 度/s^2, 等效半径 mm/deg)
    // 假设 A 轴挂载的工件半径约 50mm，B 轴旋转盘半径约 80mm
    SMC_ConfigAxisDynamics(SMC_AXIS_A, 1, 18.0, 72.0, 72.0, 50.0); 
    SMC_ConfigAxisDynamics(SMC_AXIS_B, 1, 18.0, 72.0, 72.0, 80.0);

    // ==========================================
    // 5. 规划器参数与安全预警配置
    // ==========================================
    SMC_ConfigPlannerParams(0.05, 500.0); // G64: 允许偏离 0.05mm, 向心加速度上限 500 mm/s^2
    SMC_ConfigGantrySyncAlarm(SMC_AXIS_Y, 1, 1000, 8000, 100); // Y 轴龙门同步防撕裂
    SMC_ConfigSoftLimit(SMC_AXIS_Z, 1, -500.0, 15.0);          // Z 轴防撞软限位

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
            while(getchar() != '\n'); // 清除错误输入缓冲
            continue;
        }

        switch (choice) 
        {
            case 1:
                printf("\n[执行] 正在全轴联动回归 G54 原点...\n");
                SMC_GoZero(SMC_AXIS_ALL, 1500.0); // 统一按 1500 mm/min 回归
                wait_and_print_status();
                break;

            case 2:
                printf("\n[执行] 正在将当前物理位置设定为全新的 G54 零点...\n");
                SMC_SetZero(SMC_AXIS_ALL);
                printf("[成功] 坐标系偏移矩阵已更新！\n");
                break;

            case 3:
            {
                int ax = 0;
                double dist = 0.0, spd = 1000.0;
                printf("\n--- 智能量纲 JOG 面板 ---\n");
                printf("请选择轴号 (0:X, 1:Y, 2:Z, 3:A, 4:B, -1:全轴): ");
                scanf("%d", &ax);
                
                // 智能单位提示
                if (ax == SMC_AXIS_A || ax == SMC_AXIS_B) {
                    printf("请输入移动角度 (度°, 正负代表方向): ");
                    scanf("%lf", &dist);
                    printf("请输入移动速度 (度°/s): ");
                    scanf("%lf", &spd);
                } else if (ax >= SMC_AXIS_X && ax <= SMC_AXIS_Z) {
                    printf("请输入移动距离 (mm, 正负代表方向): ");
                    scanf("%lf", &dist);
                    printf("请输入移动速度 (mm/s): ");
                    scanf("%lf", &spd);
                } else {
                    printf("全轴测试，请输入统一位移数值: ");
                    scanf("%lf", &dist);
                    printf("请输入统一目标速度: ");
                    scanf("%lf", &spd);
                }
                
                printf("\n[执行] 下发 JOG 指令: 轴[%d] 移动 %.3f, 速度 %.1f...\n", ax, dist, spd);
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
                    printf("\n[成功] ★ 加工完美结束！零件已产出！★\n");
                }
                break;
            }

            case 5:
                printf("\n[执行] 启动加工 test.txt，并在 3 秒后触发平滑刹车暂停测试...\n");
                if (SMC_RunGCodeFile("test.txt") == 0) {
                    
                    // 正常跑 3 秒
                    for(int i=0; i<30; i++) {
                        printf("\r[正常加工] X:%8.3f Y:%8.3f A:%8.3f ", 
                               SMC_GetLogicalPos(0), SMC_GetLogicalPos(1), SMC_GetLogicalPos(3));
                        fflush(stdout);
                        usleep(100000);
                    }
                    
                    // 触发暂停！体验基于虚拟时间轴的平滑减速
                    printf("\n\n⚠️ [干预] 收到进给保持(Feedhold)指令！系统正在平滑刹车...\n");
                    SMC_PauseProcessing(); 
                    
                    // 停滞观察 3 秒
                    for(int i=0; i<30; i++) {
                        printf("\r[暂停停稳] X:%8.3f Y:%8.3f (机床锁定中) ", 
                               SMC_GetLogicalPos(0), SMC_GetLogicalPos(1));
                        fflush(stdout);
                        usleep(100000);
                    }
                    
                    // 恢复运行
                    printf("\n\n✅ [干预] 收到恢复(Resume)指令！系统正在平滑加速恢复轨迹...\n");
                    SMC_ResumeProcessing();
                    
                    // 把剩下的跑完
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