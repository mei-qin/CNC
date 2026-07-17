#ifndef SMC_API_H
#define SMC_API_H

#include <stdint.h>
#include "laser_ctrl.h"      // LaserCouplePoint_t (SMC_ConfigLaserCoupleTable 参数)
#include "smc_protocol.h"    // SmcGetProgramStructureRes (P0-b v2)

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
// P2-A: 实时倍率系统 (Feed/Rapid/Spindle Override + Mode Flags)
// =======================================================
// 设计原则:
//   - 完全复用现有 time_scale feedhold 机制 (ecat_core.c ms_budget 计算)
//   - RT 单写者字段 (g_interpolator.feed_override_ratio 等), 非 RT 写, RT 读
//   - v1 范围: feed/rapid [0..100]%, spindle [0..120]%; feed/rapid 超 100% clamp 到 100%
//
// SMC_SetOverride 语义 (mask/value 模式, 与位寄存器同):
//   feed_pct/rapid_pct/spindle_pct: -1=不改, 0..120=设置值 (clamp 后通过 out 返回)
//   mode_mask: 要修改的 mode_flags 位掩码 (0=不改任何位)
//   mode_value: mask 标识的位写入 0 或 1 (其他位不变)
//   out_*: 全部非 NULL 时回读 clamp 后的实际生效值 (UI 旋钮位置同步用)
// 返回: 0=成功 (clamp 不视为错误), -1=参数全 -1 且 mask=0 (no-op)
int SMC_SetOverride(int feed_pct, int rapid_pct, int spindle_pct,
                    uint16_t mode_mask, uint16_t mode_value,
                    int *out_feed, int *out_rapid, int *out_spindle,
                    uint16_t *out_mode);

// SMC_GetOverride 语义:
//   反映 RT 当前生效的 override + mode_flags (RT 单写者 → 此处单读者)
//   HMI 60Hz 调用安全 (与 SMC_GetSpindleState 同模式)
// 返回: 0=成功, -1=参数全 NULL
int SMC_GetOverride(int *feed_pct, int *rapid_pct, int *spindle_pct,
                    uint16_t *mode_flags);


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

// =======================================================
// P0-b v2: LoadProgram / RunLoadedProgram 分离 API
// =======================================================
// 典型流程: LoadProgram (preview) → GetProgramStructure → 操作员检查 → RunLoadedProgram
//
// 加载程序到 preview cache (parser 跑完但不进 motion queue, RT 不消费)
// 调用后 parser_thread 异步跑, g_program_load_done=1 时表示完成
// filepath: G 代码绝对路径 (Linux 格式, Windows 端通过 SDK 的 TranslatePathForWSL 转换)
// 返回: 0=已启动, -1=parser 正忙 (is_running=1), -2=filepath 空或越界
int SMC_LoadProgram(const char *filepath);

// 执行已加载的程序 (LoadProgram 必须先完成, g_program_load_done=1)
// 内部重新解析同一 filepath, 这次进 motion queue, RT 实际执行
// 返回: 0=已启动, -1=LoadProgram 未完成或已 running, -2=filepath 为空
int SMC_RunLoadedProgram(void);

// 查询当前程序结构元数据
//   未加载时 ret_code=-1, 其他字段为零/哨兵值
//   load 完成 (g_program_load_done=1) 后所有字段有效
//   run 进行中 estimated_time_ms 可能不准 (T_total 边解析边累加)
// 返回: 0=成功 (即使未加载也返回 0, 通过 ret_code 字段区分), -1=out 为空
int SMC_GetProgramStructure(SmcGetProgramStructureRes *out);

// =======================================================
// P1-b: ClearAlarm (清除系统报警)
// =======================================================
// 触发 RT alarm_reset_request, RT 在安全点 (queue 空 + 驱动就绪) 时实际清。
// 异步语义: 调用立即返回, RT 清完通过 event stream (code 0x0041) 通知 UI。
// 注意: 若 parser 正在跑 (is_running=1) 拒绝, 要求先 AbortProcessing (防撞刀)。
// 返回: 0=请求已提交, -1=parser 正在跑 (先 AbortProcessing), -2=轴未就绪
int SMC_ClearAlarm(void);

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


// =======================================================
// 激光切割子系统 API (Phase A 安全地基)
// =======================================================
// 配置时序: 必须在 SMC_InitAndStart 之前调, 否则 RT 线程已启动后修改无效
// 默认状态 (laser_ctrl_init): 所有 slave_id = -1, 即"未配置激光"
// RT 线程在 slave_id < 0 时安全跳过 PDO 输出, 不影响主轴/伺服逻辑

// 1.配置激光 I/O 拓扑 (3 个 EtherCAT 从站, 任一可传 -1 表示未配置)
// do_slave_id: 数字输出从站 (16-bit: enable/shutter/gas×3/alarm_lamp)
// ao_slave_id: 模拟输出从站 (2-ch 16-bit: 功率/频率)
// di_slave_id: 安全互锁输入从站 (16-bit: door/estop/laser_alm/water×2/gas_press)
// 返回: 0=成功, -1=参数越界
int SMC_ConfigLaserIO(int do_slave_id, int ao_slave_id, int di_slave_id);

// 2.配置 DO bit 偏移 (0-15)
int SMC_ConfigLaserDOBits(uint8_t b_enable, uint8_t b_shutter,
                          uint8_t b_gas_n2, uint8_t b_gas_o2, uint8_t b_gas_air,
                          uint8_t b_alarm_lamp);

// 3.配置 DI bit 偏移 (0-15)
int SMC_ConfigLaserDIBits(uint8_t b_door, uint8_t b_estop, uint8_t b_laser_alm,
                          uint8_t b_water_t, uint8_t b_water_f, uint8_t b_gas_p);

// 4.配置 AO 通道偏移
int SMC_ConfigLaserAOChannels(uint8_t ch_power, uint8_t ch_freq);

// 5.配置激光器物理量程
// power_max_w: 满量程功率 (W), 默认 3000
// freq_max_hz: 满量程频率 (Hz), 默认 5000
// power_min_w: 起辉功率下限 (W), 默认 50 (Phase A 暂未在 RT 强制)
int SMC_ConfigLaserRange(double power_max_w, double freq_max_hz, double power_min_w);

// 6.配置功率-速度耦合 (Phase B1)
// mode: 0=off (默认, 直接输出 P_base), 1=查表耦合
// v_thresh_mm_s: 低速阈值, v_current 低于此值时强制 P=0 (默认 5.0)
int SMC_ConfigLaserCoupling(int mode, double v_thresh_mm_s);

// 7.配置功率-速度耦合表 (Phase B1)
// points: 采样点数组 (按 v_mm_s 单调不减排序)
// count:  采样点数 (1..16)
// 默认表 (laser_ctrl_init): 线性 v=0→0, v=50 mm/s→1.0
// 用户自定义示例 (起弧/切割/饱和 三段):
//   LaserCouplePoint_t t[3] = {{0,0}, {5,0.3}, {20,1.0}};
//   SMC_ConfigLaserCoupleTable(t, 3);
int SMC_ConfigLaserCoupleTable(const LaserCouplePoint_t *points, int count);

// 6.查询激光器完整状态 (HMI 用, 镜像 RT 单写者字段 + 段级派生 + 加工统计)
// struct-based out param: 字段过多 (14 个), out param 模式不可读, 改用 Res 结构体直接 fill
// (参考 SMC_GetProgramStructure 模式). SmcGetLaserStateRes 定义在 smc_protocol.h.
// @Thread-Safety: g_laser_rt / g_interpolator 均为 RT 单写者, 此处 acquire 读.
//   int/double 字段对齐天然原子; pierce_count / laser_on_time_ms 64-bit 字段
//   在 32-bit 平台可能撕裂, HMI 容忍 1ms 级滞后 (统计字段非安全关键).
// 返回: 0=成功, -1=out 为空或激光未配置 (do_slave_id<0)
int SMC_GetLaserState(SmcGetLaserStateRes *out);


// =======================================================
// P0-3: Safe Z Lift (紧急抬升避让)
// =======================================================
// 设计原则:
//   - 报警时 (emergency_kill 同周期) 自动把 Z 抬到配置的 G53 绝对安全高度
//     防激光头停在工件上方 0.5~2mm 处, 操作员开门处理时碰撞
//   - 抬升由 RT 独立子状态机执行 (不走段消费环, 不走 time_scale)
//     修改 g_axis[z].current_cmd_pos, PDO 写入循环自动传播 (单驱/双驱天然适配)
//   - 配置时序: 必须在 SMC_InitAndStart 之前调 (与 ConfigAxisDynamics 同语义)
//
// 配置 API:
//   z_letter:        Z 轴字母 (如 'Z'), 经 axis_lookup 解析
//   safe_z_mm:       G53 绝对机床坐标目标 (推荐 +50mm; 必须 ≤ Z 软限位正限)
//   lift_speed_mm_s: 抬升速度 (默认 20.0; 越大越快但驱动器扭矩要求越高)
//   auto_on_alarm:   0=仅手动 API 触发, 1=报警路径自动触发 + 手动仍可用
// 返回: 0=成功, -1=轴未配置/参数非法/旋转轴, -2=safe_z 超软限位 (event 0x0044)
int SMC_ConfigSafeLiftZ(char z_letter, double safe_z_mm,
                        double lift_speed_mm_s, int auto_on_alarm);

// 手动触发抬升 (idempotent: PENDING/RUNNING/DONE 中再调为 no-op, 返回 0)
// 不需 alarm 上下文, 加工中也可调用作为紧急避让
// 返回: 0=请求已提交, -1=未配置
int SMC_SafeLiftZ(void);

// 取消抬升 (仅 PENDING/DONE 状态有效, RUNNING 拒绝避免 Z 卡在中间高度)
// 返回: 0=已取消, -1=正在抬升中拒绝 (避免撞刀)
int SMC_CancelSafeLiftZ(void);

// 查询抬升状态 (HMI 用)
//   *out_state: 0=IDLE, 1=PENDING, 2=RUNNING, 3=DONE
//   *out_progress_mm: 已抬升距离 (current_z - start_z), 可为 NULL
// 返回: 0=成功, -1=未配置
int SMC_GetSafeLiftState(int *out_state, double *out_progress_mm);


// =======================================================
// P0-1: 工业级回零 (G28 / SMC_HomeAxis / SMC_HomeAll)
// =======================================================
// 设计原则:
//   - 5 状态机: IDLE → PENDING → RUNNING → DONE / FAULT
//   - RT 仅维护状态字段 + 冻结 queue (不阻塞); Non-RT (parser_thread) 跑阻塞 axis_homing()
//   - method 35 (软件法, v1) / method 1-19 (硬件, v2 预留)
//   - 串行顺序默认 Z → X → Y → B → C, 可配置
//   - 部分成功 all-or-nothing: HomeAll 第 N 轴失败回滚前 N-1 轴
//   - 与 SafeLift/JOG 三功能互斥 (cycle 头检测)
//   - 配置时序: 必须在 SMC_InitAndStart 之前调

// 配置单轴回零参数 (init 阶段)
// axis_letter:   轴字母
// method:        35 (v1 唯一支持), 1-19 v1 拒绝 (-3)
// search_speed:  method 1-19 寻零速度 mm/s (v1 忽略)
// creep_speed:   method 1-19 蠕动速度 mm/s (v1 忽略)
// direction:     +1 / -1 (method 1-19 寻零方向)
// timeout_ms:    超时 (默认 10000), 大行程轴可设 30000
// 返回: 0=ok, -1=轴未配置/参数非法/运行中, -3=method 不支持
int SMC_ConfigHoming(char axis_letter, int method, double search_speed,
                     double creep_speed, int direction, int timeout_ms);

// 配置回零顺序 (init 阶段)
// order_letters: 轴字母字符串, 如 "ZXYBC" (Z 先, C 后)
// 返回: 0=ok, -1=参数非法, -2=未配置的轴
int SMC_ConfigHomingAll(const char *order_letters);

// 单轴回零 (异步, 立即返回)
// 返回: 0=已提交, -1=未配置/parser 正在跑/与其他子状态机冲突
int SMC_HomeAxis(char axis_letter);

// 全轴串行回零 (异步, 立即返回; 内部按 order 顺序, all-or-nothing 回滚)
// 返回: 0=已提交, -1=未配置/parser 正在跑/与其他子状态机冲突
int SMC_HomeAll(void);

// 取消回零 (仅 PENDING/DONE, RUNNING 拒绝)
// 返回: 0=已取消, -1=未配置/正在 RUNNING 拒绝
int SMC_CancelHoming(void);

// 查询回零状态 (HMI 用)
//   *out_state: 0=IDLE, 1=PENDING, 2=RUNNING, 3=DONE, 4=FAULT
//   *out_axis_idx: 当前回零轴 (-1=HomeAll 顺序模式)
//   *out_progress_pct: 进度 0.0-1.0 (HomeAll 模式按轴数计算)
// 返回: 0=ok, -1=未配置
int SMC_GetHomingState(int *out_state, int *out_axis_idx, double *out_progress_pct);


// =======================================================
// P0-1: JOG 模式 (method 35 前置依赖 — 手动定位参考位)
// =======================================================
// 设计原则:
//   - 持续运动直到 SMC_JogStop 或撞软限位
//   - 不切 op-mode (复用 CSP), RT 子状态机直接写 current_cmd_pos 增量
//   - 与 SafeLift/Homing 互斥
//   - 软限位撞停自停 + event 0x000B

// 启动 JOG (持续运动)
//   direction: +1 正向 / -1 负向
//   speed_mm_s: JOG 速度 (典型 10-50 mm/s)
// 返回: 0=已启动, -1=轴未配置/运动中/homing 中/safelift 中, -2=方向非法
int SMC_JogStart(char axis_letter, int direction, double speed_mm_s);

// 停止 JOG (零差, 直接停, 不做减速规划)
//   axis_letter: '*' 停所有轴
// 返回: 0=已停止, -1=未在 JOG
int SMC_JogStop(char axis_letter);

// JOG 中实时位置查询 (UI 显示用, 等价于 SMC_GetLogicalPos 但语义清晰)
double SMC_JogGetPos(char axis_letter);


#ifdef __cplusplus
}
#endif

#endif // SMC_API_H
