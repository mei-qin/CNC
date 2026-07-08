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

// 查询指定轴字母是否已配置映射 (1=已映射, 0=未配置)
int SMC_IsAxisConfigured(char axis_letter);

// 获取系统当前运行状态字符串 (ALARM / HOLD / RUN / IDLE)
// out_str: 输出缓冲区, max_len: 缓冲区最大长度(含 '\0')
void SMC_GetSystemStatusStr(char* out_str, int max_len);

// =======================================================
// 辅助状态查询 API (反映 RT 实际执行到的状态, 非parser 解析到的"将来")
// =======================================================
// 查询主轴模态与转速 (镜像 RT spindle_mode_rt/spindle_rpm_rt)
// *mode: 0=off(M5), 1=CW(M3), 2=CCW(M4)
// *rpm:  最近有效 S 值
// 返回: 0=成功, -1=轴未就绪
int SMC_GetSpindleState(int *mode, double *rpm);

// 查询冷却液 bit flags (镜像 RT coolant_state_rt)
// *state: bit0=flood(M8), bit1=mist(M7); 0=off/1=flood/2=mist/3=both
// 返回: 0=成功, -1=轴未就绪
int SMC_GetCoolantState(int *state);

// 查询当前刀号 (镜像 RT current_tool_id_rt, 由最近一次 M6 切换)
// 返回: 0=成功, -1=轴未就绪
int SMC_GetCurrentTool(int *tool_id);

// 控制 M1 可选停开关 (HMI 用)
// enable: 0=禁用 M1 (默认, M1 no-op), 1=M1 等价 M0
// 返回: 0=成功
int SMC_SetOptionalStopEnable(int enable);


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

// 配置运动学偏置参数（BC 双摆头: 刀具长度 + B→C 旋转中心偏置）
// tool_len:    刀具长度 (mm), 沿主轴 -Z 方向
// pivot_x/y/z: C 轴旋转中心到 B 轴旋转中心的物理偏置 (mm)
void SMC_ConfigKinematicsOffset(double tool_len, double pivot_x, double pivot_y, double pivot_z);

// 配置通用五轴运动学构型 (Head-Head / Table-Table / Mixed)
// type:        构型枚举 (KIN_HEAD_HEAD / KIN_TABLE_TABLE / KIN_MIXED)
// r1_idx/axis: 第 1 旋转轴的底层索引和旋转维度 (0=X,1=Y,2=Z)
// r2_idx/axis: 第 2 旋转轴的底层索引和旋转维度
// tool_off:    主轴面到刀尖的偏置 [3] (mm)
// pivot_off:   两旋转中心之间的偏置 [3] (mm)
void SMC_ConfigKinematics(int type,
                          int r1_idx, int r1_axis,
                          int r2_idx, int r2_axis,
                          double tool_off[3], double pivot_off[3]);

// =======================================================
// 仿真驱动器 API (仅 g_sim_mode==1 时有效, 真实硬件模式返回错误)
// =======================================================
// 故障注入: 把指定轴指定 motor 推入 CiA402 FAULT 态 (SW_ERROR 置位)
// axis_letter:   轴字母 ('X'/'Y'/'Z'/'B'/'C')
// slave_subidx:  双驱轴的 motor 索引 (0=主, 1=从; 单驱轴只能 0)
// 返回: 0=成功, -1=参数越界/轴未配置, -2=非 sim 模式
int SMC_InjectAxisFault(char axis_letter, int slave_subidx);

// 配置 sim 一阶伺服模型的滞后系数 alpha
// alpha: (0, 1) 范围, 默认 0.2; 越小跟随误差越大 (alpha=0.05 时易触发硬停)
// 返回: 0=成功, -1=参数越界/轴未配置, -2=非 sim 模式
int SMC_ConfigSimDynamics(char axis_letter, double alpha);


#ifdef __cplusplus
}
#endif

#endif // SMC_API_H
