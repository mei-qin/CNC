#ifndef SMC_API_H
#define SMC_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 全轴操作通配符（用于 SetZero/GoZero/MoveRelative 等支持全轴的 API）
#define SMC_AXIS_ALL '*'

// =======================================================
// 系统生命周期管理 API
// =======================================================
// 初始化并启动系统 (包含 EtherCAT 组网、线程启动、伺服上电握手)
// 阻塞函数，直到全轴就绪返回 0，失败返回 -1
int SMC_InitAndStart(const char *netif_name);

// 安全关闭系统 (伺服优雅下电，释放网卡)
void SMC_Close(void);

// =======================================================
// 轴配置 API
// =======================================================
// 1.配置轴的从站拓扑结构（单驱/双驱）和名称
// axis_name: 轴名称，首字母须为 A-Z（如 "X轴"、"Y"）
// 底层自动分配数组房间号，用户无需关心索引
// is_dual_drive: 0=单驱，1=双驱
// master_id/slave_id: 双驱时分别指定主从ID
int SMC_ConfigAxisTopology(const char* axis_name, int is_dual_drive, int master_id, int slave_id);

// 2.配置轴的软件限位参数
int SMC_ConfigSoftLimit(char axis_letter, int enable, double neg_limit_mm, double pos_limit_mm);

// 3.配置轴的龙门同步报警参数
int SMC_ConfigGantrySyncAlarm(char axis_letter, int enable, int32_t tolerance_pulse, int32_t max_error_pulse, int time_ms);


// =======================================================
// 坐标与状态获取 API
// =======================================================
// 获取指定轴的当前逻辑坐标 (单位: 脉冲)
double SMC_GetLogicalPos(char axis_letter);

// 获取 G 代码解析器状态 (1:正在解析/加工中, 0:空闲)
int SMC_IsParserRunning(void);

// 判断底层运动是否彻底结束 (队列空 且 插补器停止)
int SMC_IsMotionDone(void);

// 获取底层 FIFO 队列堆积的指令数量 (用于UI进度条)
int SMC_GetQueueCount(void);


// =======================================================
// 运动控制 API
// =======================================================
// 设定原点 (工件坐标系 G54)，axis_letter='*' 全轴归零
void SMC_SetZero(char axis_letter);

// 单轴相对点动 (JOG)，axis_letter='*' 全轴联动
void SMC_MoveRelative(char axis_letter, double distance, double speed);

// 多轴联动回归原点，axis_letter='*' 全轴联动
void SMC_GoZero(char axis_letter, double speed);

// =======================================================
// G 代码加工 API
// =======================================================
// 启动后台加工 (非阻塞，下发后立刻返回)
int SMC_RunGCodeFile(const char *filepath);

// 暂停加工 (Hold)
void SMC_PauseProcessing(void);

// 恢复加工 (Resume)
void SMC_ResumeProcessing(void);

// 紧急中止 (Abort)
void SMC_AbortProcessing(void);

// 配置轴的脉冲/单位 (例如 脉冲/mm 或 脉冲/度)，用于位置/速度换算
void SMC_ConfigPulsePerUnit(char axis_letter, double pulse_per_unit);

// 配置轴动力学参数（类型、最大速度、最大加减速、旋转轴等效半径）
// equivalent_radius: 仅旋转轴有效（mm/deg），线性轴传 0.0
int SMC_ConfigAxisDynamics(char axis_letter, int type, double max_v, double max_a, double max_d, double equivalent_radius);

// 配置规划器参数（G64 拐角容差、最大向心加速度）
int SMC_ConfigPlannerParams(double tolerance, double max_centripetal_acc);


#ifdef __cplusplus
}
#endif

#endif // SMC_API_H
