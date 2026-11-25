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

// 2.1 P0: 配置旋转轴 G90 最短路径解缠绕 (wrap-to-nearest)
// 仅对 axis_type==1 的轴有意义; 线性轴调用返回 -1
int SMC_ConfigRotWrapEnable(char axis_letter, int enable);

// 2.2 P0: 配置 RTCP 奇异点检测阈值 (R_max = 物理步长/逻辑步长 峰值)
// rmax <= 0 显式禁用检测; 默认 30.0 (编译期 g_rtcp_singularity_rmax)
int SMC_ConfigSingularityRmax(double rmax);

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

// 中止程序解析 (仅停解析, 不停车)。
// 仅置 g_parser_ctrl.abort_request: parser 停止解析并退出 (is_running=0),
// 但 **不清运动队列** — 已入队的段由 RT 继续走完 (队列 1024 段, 短程序在按 Stop 时
// 往往已全部入队, 表现为 "按了停止还在走")。暂停中调用会同时解除暂停 (走完残余段)。
// 程序被中止时 parser 退出路径复位程序级模态 (M3 出光/冷却液/刀补等回安全默认)。
// 调用方: SMC_EmergencyStop ①、api_alarm_reset 联动、RPC abort 命令。
// (OPC UA CNC.Program.Stop / System.Reset 的"停"走 SMC_StopProgram。)
// 需要"停住并丢弃队列": 程序 Stop 用 SMC_StopProgram (不报警); 异常用 SMC_EmergencyStop
// + SMC_ClearAlarm。
void SMC_AbortProcessing(void);

// 程序 Stop (2026-11-25): 刹停 + 丢弃残余段 + 关工艺输出, 不产生报警、无需复位。
// 工业控制器 "程序停止/复位" 语义 — 与软急停的区别只有三点: 不进报警态、不抬 Z、不用复位。
// 异步: 请求后 RT 立即减速 (~200ms 到 HOLD_PAUSED), 在安全点丢弃全队列并推事件 0x0035;
//   期间 Machine.Status = PAUSE, 完成后回 IDLE, 再次 RunLoadedProgram 即从头重跑。
// 轴停在按下时的位置附近, 坐标不回零; 中止的程序不可"继续" (要续切走断点续切)。
// 返回: 恒 0 (不可拒; parser 未在解析时无事可做, 同样返回 0)
int SMC_StopProgram(void);

// =======================================================
// P0-b v2: LoadProgram / RunLoadedProgram 分离 API
// =======================================================
// 典型流程: LoadProgram (preview) → GetProgramStructure → 操作员检查 → RunLoadedProgram
//
// 加载程序到 preview cache (parser 跑完但不进 motion queue, RT 不消费)
// 调用后 parser_thread 异步跑, g_program_load_done=1 时表示完成
// filepath: G 代码绝对路径 (Linux 格式, Windows 端通过 SDK 的 TranslatePathForWSL 转换)
// 返回: 0=已启动, -1=parser 正忙 (is_running=1), -2=filepath 空或越界,
//       -3=文件不可达 (fail-fast, 2026-09-09; 上一成功加载不受影响)
// 异步失败 (读取中断等): keep-last-good 恢复上一成功程序 (事件 0x003A)
int SMC_LoadProgram(const char *filepath);

// 执行已加载的程序 (两种可启动状态):
//   (a) 已加载待运行: LoadProgram 已完成, g_program_load_done=1
//   (b) 加工完成态: 上次运行已结束 (M30/EOF 或被 Stop 中止) — 同一程序从头重跑,
//       要求程序文件自加载后未被修改 (size + mtime 与加载时一致)
// 内部重新解析同一 filepath, 这次进 motion queue, RT 实际执行
// 注意: 不是断点续切 (续切走 SMC_ResumeSearch/ResumeStart)。程序被 Stop/中止时 parser
//       复位程序级模态 (出光/冷却液/刀补/G68.2 等, 同 M30), 但运动模态不复位 —
//       被中止的程序若未显式给出 G90/G94/平面/坐标系, 重跑会沿用中止时的运动模态。
// 返回: 0=已启动,
//       -1=无已加载程序 (从未成功 LoadProgram),
//       -2=parser 正在运行 / 上次 Stop 尚未执行完 / 运动队列有残余段 /
//          (已加载态下) filepath 为空,
//       -3=加工完成态但程序文件自加载后已变更/不可访问 → 须重新 LoadProgram
int SMC_RunLoadedProgram(void);

// 查询当前程序结构元数据
//   未加载时 ret_code=-1, 其他字段为零/哨兵值
//   load 完成 (g_program_load_done=1) 后所有字段有效
//   run 进行中 estimated_time_ms 可能不准 (T_total 边解析边累加)
// 返回: 0=成功 (即使未加载也返回 0, 通过 ret_code 字段区分), -1=out 为空
int SMC_GetProgramStructure(SmcGetProgramStructureRes *out);

// =======================================================
// 断点续切 Phase 1 (2026-09-08): ResumeSearch / ResumeStart / GetResumeState
// =======================================================
// 典型流程: (中断发生后) GetResumeState/preview 历史选定边界行 → ResumeSearch
//           (阻塞校验+冻结) → 操作员确认 → ResumeStart (前导重定位+续切)
//
// 断点续切第一步: 校验 target_line 是 G00 边界行 (轮廓进近行) 并 SEARCH 重放
// 建立模态。阻塞至重放完成 (毫秒~百毫秒级, 5s 超时兜底)。
// 前置: LoadProgram 已完成 + SafeLiftZ 已配置 (前导依赖) + 文件/WCS 指纹一致。
// 返回: 0=边界已冻结 (可 ResumeStart),
//       -1=parser 忙, -2=未 LoadProgram/指纹不符, -3=行存在但非 G00 边界,
//       -4=SafeLiftZ 未配置, -5=行不存在 (或 M30 前/被中止), -6=搜索超时
int SMC_ResumeSearch(int32_t target_line);

// 断点续切第二步: 从冻结边界起跑。parser 从头重放, 已执行区间被跳段丢弃,
// 边界行拦截生成重定位前导 (激光模态恢复 + SafeZ 抬升 → 安全高度平移 → Z 下降)。
// 返回: 0=已启动, -1=未冻结 (先 ResumeSearch), -2=parser 忙, -3=指纹不符 (冻结已作废)
int SMC_ResumeStart(void);

// 查询续切状态 (ready 位 / SEARCH 结果码 / 冻结行号与目标位姿)
void SMC_GetResumeState(SmcGetResumeRes *out);

// =======================================================
// 论文1 IV-E (2026-09-16): G93 jerk-bounded 转移剖面配置 (RPC 0x0082)
// =======================================================
// enable=1: 相邻不同 v_cmd 严格块边界构造 jerk 受限转移 (加速→exact 超速模式 /
//           减速→debt 签名时长模式); enable=0: 完全退化 IV-C 纯匀速 (A/B 对照开关)
// 返回: 0=已生效, -1=ε 参数非法
int SMC_ConfigG93Jerk(int enable, double eps_v_pct, double eps_t_pct);

// =======================================================
// 论文1 B0′ 消融 + N22 实时性 (2026-09-17, 复审 P06/P12)
// =======================================================
// SMC_ConfigG93Ablation (RPC 0x0083): f_inherit=1 禁用 IV-B 时间再分配,
// 每微段继承整块进给字 (τ_b=n_b·T_b, 基线 B0 可执行复现)。会话级开关,
// 不持久化 (server 重启即回默认 OFF)。返回: 0=已生效, -2=程序运行中拒绝
int SMC_ConfigG93Ablation(int f_inherit);

// SMC_ConfigG93Posthoc (RPC 0x0085, 论文1 M1 替代设计): enable=1 关闭 IV-D
// 的先验长度重标定, 段按标称 T 执行并在段末按实测速度累积赤字补偿到下一段
// ("先测量再修正"的可执行复现)。会话级开关。返回: 0=已生效, -2=运行中拒绝
int SMC_ConfigG93Posthoc(int enable);

// SMC_ConfigG93Ratio (RPC 0x0086, 论文1 V2 仿真战役): ratio_off=1 关闭 IV-D
// 长度比重标定 (Naive 基线 T'=T, 使 v_act/v_cmd ≡ L_act/L_prog = γ), 用于
// E4 γ 扫描的 Naive vs Proposed 实测对照。会话级开关。返回: 0=已生效, -2=运行中拒绝
int SMC_ConfigG93Ratio(int ratio_off);

// SMC_ConfigG93StrictOff (RPC 0x0087, 论文1 S3 planner-rate-limiting 战役):
// strict_off=1 关闭 G93 强一致豁免, 下游 planner 对每段执行完整短板效应限幅
// (基线 B2 的可执行复现, E3 机制实证); strict_off=0 恢复 Ours 强一致路径 (出厂默认)。
// 会话级开关。返回: 0=已生效, -2=运行中拒绝切换
int SMC_ConfigG93StrictOff(int strict_off);

// SMC_ConfigG93TipBasis (RPC 0x0088, 论文1 S10-H1 长度基消融):
// tip_basis=1 把 G93 需求速率的长度基从机械轴等效长度 L^M 换成刀尖欧氏长度
// L^P (旋转轴折算不计), 使第三步.5 可行性门低估绑定轴需求 m=L^M/L^P 倍并
// 放行本应判不可行的段 (绑定轴突破硬限速); tip_basis=0 = IV-A 物理基 (出厂默认)。
// 会话级开关。返回: 0=已生效, -2=运行中拒绝切换
int SMC_ConfigG93TipBasis(int tip_basis);

// SMC_GetEnqueueStats (RPC 0x0084): enqueue 临界区耗时统计 (论文 V-C 实测)。
// reset=1 读后清零。单位: μs; strict 路径 = is_g93_strict 入队 (含剖面预计算)。
void SMC_GetEnqueueStats(int reset, uint64_t *count_all, uint64_t *count_strict,
                         double *mean_us_all, double *mean_us_strict,
                         double *max_us_strict);

// =======================================================
// P1-b: ClearAlarm (清除系统报警)
// =======================================================
// 触发 RT alarm_reset_request, RT 在安全点 (queue 空 + 驱动就绪) 时实际清。
// 异步语义: 调用立即返回, RT 清完通过 event stream (code 0x0041) 通知 UI。
// 注意: 若 parser 正在跑 (is_running=1) 拒绝, 要求先 AbortProcessing (防撞刀)。
// 返回: 0=请求已提交, -1=parser 正在跑 (先 AbortProcessing), -2=轴未就绪
int SMC_ClearAlarm(void);

// P0-A (2026-07-21): 软急停一站式 API (ISO 13850 软件层).
// @Context: Non-RealTime (RPC server / UI 通过 SDK)
// @Thread-Safety: rpc_server 单线程消费 socket, 调用自然序列化
// 原子序列 (顺序敏感):
//   1. SMC_AbortProcessing - 清 parser 队列
//   2. g_sys_alarm_state=1 - 触发 RT HOLD_BRAKING 平滑刹车
//   3. g_laser_rt.emergency_kill=1 - laser_rt_safety_gate 关光闸/DO
//   4. SMC_SafeLiftZ() (若 auto_on_alarm=1) - Z 抬升防撞工件
//   5. EventLogger LASER/0x0011 "急停软线"
// reason_code: 0=UI 手动, 1=外部系统(SDK/脚本), 2-9 保留
// 返回 0=已触发 (急停不可拒, 恒返回 0)
int SMC_EmergencyStop(int reason_code, const char *message);

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

// P1 (2026-09-04): 配置双摆头链序 (G43.5 姿态解算 + 逆解装配共用)
// 0 = R1 外层 (已交付默认, B外C内刀具同轴) / 1 = R2 外层 (典型 C外B内腕)
// 须在 SMC_ConfigKinematics 之后调; 非法值 (非 0/1) 忽略并 WARN
void SMC_ConfigKinematicsChain(int chain_order);

// =======================================================
// 仿真驱动器 API (仅 g_sim_mode==1 时有效, 真实硬件模式返回错误)
// =======================================================
// 故障注入: 把指定轴指定 motor 推入 CiA402 FAULT 态 (SW_ERROR 置位)
// axis_letter:   轴字母 ('X'/'Y'/'Z'/'B'/'C')
// slave_subidx:  双驱轴的 motor 索引 (0=主, 1=从; 单驱轴只能 0)
// 返回: 0=成功, -1=参数越界/轴未配置, -2=非 sim 模式
int SMC_InjectAxisFault(char axis_letter, int slave_subidx);

// B2 (2026-07-23): sim 模式注入双驱龙门静态差
// 仅 g_sim_mode==1 时有效, 用于 B2 pre-align 算法 sim 验证
// 直接给 slave motor pos 加 offset_pulse, 模拟机械梁倾斜 / encoder 零点漂移
// axis_letter:   轴字母 (必须是 slave_count==2 双驱轴)
// offset_pulse:  加到 slave motor pos 的偏移量 (正/负)
// 返回: 0=成功, -1=非双驱/参数越界, -2=非 sim 模式
int SMC_InjectGantryOffset(char axis_letter, int32_t offset_pulse);

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

// P0-2 (2026-09-09): 配置互锁链规则表 (软 PLC 地基, 数据驱动互锁)
// count: 0..LASER_IL_MAX; 0 = 回退默认 6 路链 (bit_di_* 固定映射)
// 校验: bit<=15, mask 非 0 且不含 bit15 (0x8000 保留 system_alarm)
// 返回: 0=成功, -1=参数越界
int SMC_ConfigLaserInterlock(const LaserIlRule_t *rules, int count);

// P0-3a (2026-09-09): DI 注入/强制 (P1.5-2 起 sim+实机均有效)
//   sim: 传感器模拟 (互锁链可测); 实机: DI 强制 (覆盖 PDO 物理读数,
//   调机隔离故障点; 实机开关推 LASER 事件 0x0015 审计)
// pattern: bit=1 该位异常 (如 0x0008=水温); enable: 1=使能 0=清除
// 返回: 0=成功
int SMC_InjectLaserDi(uint16_t pattern, uint16_t enable);

// Phase 1 (2026-11-23): 机械轴限位映射 sim 测试通道 (任务书: 不新增实机强制接口)
//   按站注入 0x60FD 输入字 + 有效标志; word/valid 单原子打包, 诊断查询读同一样本
//   slave_id: 目标站号 (跨轴拓扑寻址, 与轴字母无关); valid=0 时 word 保留作证据回显
// 返回: 0=成功, -1=站号不在当前拓扑, -2=非 sim 模式
int SMC_InjectSimLimitInput(int slave_id, int valid, uint32_t word);

// Phase 1 (2026-11-23): 限位映射诊断查询 (纯 RPC 线程, 零 RT 参与)
//   逐 轴×方向: runtime 配置表 (kernel_init_hw apply) + sim 从站样本 → limit_map_eval
//   out_entries: AXIS_NUM*2 条目 (轴主序, + 在前 − 在后), 结构见 smc_protocol.h
// 返回: 0=成功
int SMC_QueryLimitDiag(SmcLimitDiagEntry_t *out_entries);

// P1.5-2 (2026-09-15): PLC 用户变量强制 (输出强制语义, 周期末覆写)
// name: 变量名 (大小写不敏感); name 空且 on=0 = 清除全部; 仅 RUNNING 有效
// 返回: 0=成功, -1=变量不存在, -3=非 RUNNING
int SMC_PlcForceVar(const char *name, double val, int on);

// P0-4a (2026-09-10): I/O 端子 PDO 布局 (byte 偏移/宽度, 选型适配)
// di/do: 起始字节 + 宽度 1/2; ao: 通道 0 起始字节 + 每通道字节 2/4
// reserved: 2 字节保留 (协议对齐), 必须填 0; 传 NULL 跳过检查
// 返回: 0=成功, -1=参数越界
int SMC_ConfigLaserPdoLayout(uint8_t di_off, uint8_t di_bytes,
                             uint8_t do_off, uint8_t do_bytes,
                             uint8_t ao_off, uint8_t ao_stride,
                             const uint8_t *reserved);

// P1.5-1 (2026-09-14): 配置外围 AI 模拟输入通道 (水温/气压/流量传感器)
// RT 周期采样: PDO raw 16-bit → 0..fs[ch] 工程量 → PLC 系统变量 G_AI0..3
// fs[i] <= 0 = 该通道未配置 (G_AIx 恒 0); count=0 整体关闭
// 返回: 0=成功, -1=参数越界
int SMC_ConfigPeriphAi(int slave_id, int byte_off, int stride, int count,
                       const double *fs);

// P1.5-1 (2026-09-14): sim-only 注入传感器工程量 (G_AIx 呈现值)
// 返回: 0=成功, -1=通道越界, -2=非 sim 模式
int SMC_InjectPeriphAi(int ch, double val);

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

// P0-2 (2026-09-03): 配置辅助气压 AO 通道 + 量程 (M13 P<bar> 配套)
// ch 0-3; max_bar<=0 拒绝 (未配置即 M13 一律报警, 防呆)
int SMC_ConfigLaserGas(uint8_t ch_ao_pressure, double pressure_max_bar);

// P1-5 (2026-09-03): 微连接段功率系数 (M77 P<ratio> / OPC UA 配套), (0,1]
int SMC_ConfigLaserMicroJoint(double mj_ratio);

// P1-4 (2026-09-03): 渐进穿孔台阶表 (M76 P<级数> 配套), count 1..8
int SMC_ConfigLaserPierceTable(const LaserPierceStep_t *steps, int count);

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
//   - method 35 (v1.5, 当前唯一可用): RT 运动块把轴走回零点, 不依赖 home switch,
//               不做 SDO; 不重锚 homing_shift (零点恒为上电锚定点)
//               目标: G53 下为机械零点; G54..G59 下为当前 WCS 零点的 G53 投影
//   - method 1-19 (CiA402 硬件回零, 需 home switch): 非 sim 分支已有实现,
//               但 SMC_ConfigHoming 配置层拒绝 (-3), 当前不可达
//   - 默认 enabled=0: 未配置时 SMC_HomeAxis/HomeAll/HomeOrder 一律返回 -1。
//     配置来源仅两条: (a) RPC 0x005B/0x005C; (b) 启动参数 "rpc_server <iface> hw"
//     (kernel_init_hw 按机床参数档案逐轴配置)。无 "hw" 的 kernel_init 不配回零。
//   - 串行顺序默认 Z → X → Y → B → C, 可配置
//   - 部分成功 all-or-nothing: HomeAll 第 N 轴失败回滚前 N-1 轴
//   - 与 SafeLift/JOG 三功能互斥 (cycle 头检测)
//   - 配置时机: init 阶段 (SMC_InitAndStart 之前) 或 parser 空闲期 (RPC 运行期),
//     parser 运行中或轨迹未结束时拒绝修改

// 配置单轴回零参数
// axis_letter:   轴字母
// method:        35 (当前唯一支持), 1-19 拒绝 (-3)
// search_speed:  method 35 回零速度 mm/s (>0 固定速度; 0 = RT 用轴 max_speed)
// creep_speed:   >0 时记录, method 35 不使用; <=0 时存 1.0
// direction:     +1 / -1 (必须显式给出; method 35 的运动方向由零点位置决定, 此值不参与)
// timeout_ms:    超时 [1000, 60000], 大行程轴可设 30000
// 返回: 0=ok, -1=轴未配置/参数非法/系统运行中, -3=method 不支持
int SMC_ConfigHoming(char axis_letter, int method, double search_speed,
                     double creep_speed, int direction, int timeout_ms);

// 配置回零顺序 (init 阶段)
// order_letters: 轴字母字符串, 如 "ZXYBC" (Z 先, C 后)
// 返回: 0=ok, -1=参数非法, -2=未配置的轴
// v1 wrapper: 不改 auto_on_init (保持 g_homing_cfg.auto_on_init 现状, 默认 0)
int SMC_ConfigHomingAll(const char *order_letters);

// B4 (2026-07-23): v2 wrapper, 加 auto_on_init 参数
// auto_on_init: 0=不自动回零 (默认), 1=SMC_InitAndStart 末尾自动调 axis_homing_multi
//   注: auto_on_init 仅在 SMC_InitAndStart 时检查, 此后修改无效 (与 enabled 同语义)
//   失败时进 alarm, 操作员手动介入 (不自动重试, 防掩盖硬件问题)
// 返回: 0=ok, -1=参数非法/运行中, -2=未配置的轴
int SMC_ConfigHomingAllEx(const char *order_letters, int auto_on_init);

// B2 (2026-07-23): 配置双驱龙门轴 pre-align 锚定前预对中参数
// 触发条件: axis_homing 双驱分支 step⑤ 前, 若 |p_master - p_slave| > tol_pulse
//          则先走 CSP 临时对中运动到 (p_m+p_s)/2, 收敛后再触发 CiA402 homing
// 解决痛点: method 35 软件法把当前位置标为零, 若主从存在静态机械差,
//          该差会被永久固化到 homing_shift, 导致龙门梁"先天歪斜" + gantry_sync 误报
// 配置时序: 必须在 SMC_InitAndStart 之前调 (与 SMC_ConfigHoming 同语义)
//
// axis_letter:  轴字母 (必须是 slave_count==2 双驱轴)
// tol_pulse:    触发阈值脉冲数 (0=跳过 pre-align; 推荐值 = pulse_per_mm × 0.05mm)
// timeout_ms:   pre-align 收敛超时 [500, 30000], 默认 3000; 超时则 FAULT + event 0x000D
// 返回: 0=ok, -1=轴未配置/单驱轴/运行中/参数非法
int SMC_ConfigGantryAlign(char axis_letter, int32_t tol_pulse, int timeout_ms);

// 单轴回零 (异步, 立即返回)
// 返回: 0=已提交, -1=未配置/parser 正在跑/与其他子状态机冲突
int SMC_HomeAxis(char axis_letter);

// 全轴串行回零 (异步, 立即返回; 内部按 order 顺序, all-or-nothing 回滚)
// 返回: 0=已提交, -1=未配置/parser 正在跑/与其他子状态机冲突
int SMC_HomeAll(void);

// 子集串行回零 (异步, 立即返回; 按字母序回零, 如 "BC" 旋转回零)
// 与 HomeAll 同一 worker/回滚机制; 任一轴未配置回零则整体拒绝
// 返回: 0=已提交, -1=未配置/轴未配置/系统运行中/冲突/重入
int SMC_HomeOrder(const char *axis_letters);

// v1.5 (2026-08-28): 当前位置设为原点 (main 时代 SetZero 语义)
// 把当前激活工件坐标系 (G54..G59; G53 拒绝) 零点定义在当前物理位置。
// 纯 WCS 偏置写入, 无运动; 之后回零 (m35) 物理回到这里。
// axis_letter: 单轴字母 或 SMC_AXIS_ALL (-1) 全轴
// 返回: 0=成功, -1=G53 模态/轴未配置/运行中/冲突
int SMC_SetOriginHere(char axis_letter);

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
