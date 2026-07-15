#ifndef AXIS_CFG_H
#define AXIS_CFG_H

#include <stdint.h>
#include <string.h>
#include <stdatomic.h>
typedef int32_t int32;
typedef uint16_t uint16;
/************************ 核心宏定义 ************************/
#define AXIS_NUM         5      // 五轴核心宏！改此值可灵活增减轴数
#define EC_TIMEOUTMON    500     // SOEM超时时间
#define NSEC_PER_SEC     1000000000
#define CYCLE_TIME_NS    1000000  // 1ms实时周期，五轴共用

/************************ CiA402 控制字（台达B3-E标准） ************************/
#define CW_FAULT_RESET   0x0080
#define CW_SHUTDOWN      0x0006
#define CW_SWITCH_ON     0x0007
#define CW_ENABLE_OP     0x000F
#define CW_PP_TRIGGER    0x001F   // bit4上升沿触发
#define CW_CSP_ENABLE    0x004F 

/************************ CiA402 状态字（掩码+标准状态） ************************/
#define SW_MASK          0x006F   // CiA402状态字有效位掩码
#define SW_FAULT_CLEAR   0x0000   // 故障位(bit3)清零
#define SW_SHUTDOWN_RDY  0x0021   // 关机就绪
#define SW_SWITCHED_ON   0x0023   // 伺服上电完成
#define SW_OP_ENABLED    0x0027   // 操作使能就绪
#define SW_TARGET_REACH  0x0400   // 目标位置到达

/************************ PDO 字节偏移（台达B3-E标准，所有轴统一） ************************/
#define PDO_CW_BYTE0     0
#define PDO_CW_BYTE1     1
#define PDO_POS_BYTE0    2
#define PDO_POS_BYTE1    3
#define PDO_POS_BYTE2    4
#define PDO_POS_BYTE3    5
#define PDO_SW_BYTE0     0
#define PDO_SW_BYTE1     1
#define PDO_FOLLOW_BYTE0 6   // 0x60F4 跟随误差（32位，小端）
#define PDO_FOLLOW_BYTE1 7
#define PDO_FOLLOW_BYTE2 8
#define PDO_FOLLOW_BYTE3 9

// 补全所有未定义的对象字典标识符
#define OD6060_MODE_OF_OPERATION 0x6060  // 操作模式
#define OD607A_TARGET_POSITION   0x607A  // 目标位置（32位有符号）
#define OD6081_MAX_SPEED         0x6081  // 最大速度（32位无符号）
#define OD6083_ACCELERATION      0x6083  // 加速时间（32位无符号）
#define OD6084_DECELERATION      0x6084  // 减速时间（32位无符号）
#define OD6040_CONTROL_WORD      0x6040  // 控制字（备用）
#define OD6041_STATUS_WORD       0x6041  // 状态字（备用）
#define OD60FF_TARGET_SPEED      0x60FF  // 目标速度（32位有符号）


// 状态字位掩码
#define SW_READY_FUNC_START  0x0001  // bit0：准备功能启动
#define SW_SERVO_READY       0x0002  // bit1：伺服准备完成
#define SW_SERVO_ENABLE      0x0004  // bit2：伺服使能
#define SW_ERROR             0x0008  // bit3：异常信号
#define SW_MAIN_POWER_ON     0x0010  // bit4：入力侧供电
#define SW_EMERGENCY_STOP    0x0020  // bit5：紧急停止
#define SW_READY_FUNC_OFF    0x0040  // bit6：准备功能关闭
#define SW_WARNING           0x0080  // bit7：警告信号
#define SW_REMOTE_CTRL       0x0200  // bit9：远程控制
#define SW_TARGET_REACH      0x0400  // bit10：目标到达

// 核心状态组合宏（便于上层判断）
#define SW_IS_NORMAL         ((SW_ERROR == 0) && (SW_EMERGENCY_STOP == 0)) // 无异常无急停
#define SW_IS_ENABLED        (SW_SERVO_ENABLE == 1)                        // 伺服使能
#define SW_IS_TARGET_REACH   (SW_TARGET_REACH == 1)                        // 目标到达

// ========== CSP模式相关宏 ==========
// CiA402模式值
#define CSP_MODE           0x08    // CSP模式（循环同步位置）
#define PP_MODE            0x01    // PP模式（点位，原有）

// CSP轨迹生成参数
#define CSP_AMPLITUDE      10000   // 轨迹幅值（脉冲）
#define CSP_FREQUENCY      1.0f    // 轨迹频率（Hz）
#define CSP_OFFSET         0       // 轨迹偏移（脉冲）

//#define LOGICAL_AXIS_NUM 5
#define AXIS_ALL -1

#define MAX_SLAVES_PER_AXIS 2
#define MIN_FEED_SPEED  0.5   // 合成速度下限（mm/s），低于此值钳制
#define MIN_FEED_ACC   10.0   // 合成加速度下限（mm/s^2），防止龟速蠕动
#define MIN_FEED_DEC   10.0   // 合成减速度下限（mm/s^2）
#define QUEUE_SIZE 1024 // 命令队列大小

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG_TO_RAD (M_PI / 180.0)

/************************ 命令类型宏 ************************/
#define CMD_TYPE_MOTION  0   // 运动段（G00/G01/G02/G03）
#define CMD_TYPE_MCODE   1   // M代码段（辅助功能）

// ---- P0-b v1: TrajectorySegment_t.motion_type 字段枚举 ----
// parser case 'G' 设置, 用于 UI 轨迹上色 + 段类型识别
#define MOTION_TYPE_RAPID    0   // G00 快速移动
#define MOTION_TYPE_LINEAR   1   // G01 直线插补
#define MOTION_TYPE_ARC_CW   2   // G02 顺时针圆弧
#define MOTION_TYPE_ARC_CCW  3   // G03 逆时针圆弧
#define MOTION_TYPE_NURBS    4   // NURBS/B-Spline 平滑段
#define MOTION_TYPE_OTHER    0xFF // 其他 (固定循环展开段/未知)

// ---- Laser Phase B4: TrajectorySegment_t.seg_flags 字段 bit 定义 ----
// parser case 'M' M72-M75 modal 设置, 用于 UI 区分引线/微连接段
//   M72/M73 包裹的段 → SEG_FLAG_LEAD_IN 置位
//   M74/M75 包裹的段 → SEG_FLAG_MICRO_JOINT 置位
// 两者可叠加 (M72 后又 M74, 段 flags = LEAD_IN|MICRO_JOINT)
// bit2-7 保留 (未来 pierce / over-burn / tab 等)
#define SEG_FLAG_LEAD_IN     0x01   // bit0: 引线段 (CAM 输出, UI 虚线橙)
#define SEG_FLAG_MICRO_JOINT 0x02   // bit1: 微连接段 (CAM 输出, UI 虚线紫)

#define MCODE_WAIT_TIMEOUT_MS  5000  // M代码等待绝对超时（ms），防止队列死锁

/************************ 跟随误差监控参数 ************************/
#define FOLLOW_ERR_MAX_PULSE     5000   // 跟随误差硬限（脉冲），超过立即停机
#define FOLLOW_ERR_WARN_PULSE    3000   // 跟随误差警告阈值（脉冲）
#define FOLLOW_ERR_WARN_TIME_MS  200    // 警告阈值持续时长（ms），超过则停机

/************************ 实时线程环形日志缓冲 ************************/
#define RT_LOG_BUF_SIZE  64
#define RT_LOG_MSG_LEN   128

typedef struct {
    char buffer[RT_LOG_BUF_SIZE][RT_LOG_MSG_LEN];
    volatile int head;
    volatile int tail;
} RtLog_t;


/************************ 轴参数结构体（单轴所有参数独立封装） ************************/
// 每个轴的独立配置/状态，五轴通过数组管理
typedef struct {
    // ① 静态配置（初始化赋值，运行中不变）
    int slave_id; 
    int slave_ids[MAX_SLAVES_PER_AXIS];  // 关联的从站ID数组（支持多从站）
    int slave_count;      // 有效的从站数量（<= MAX_SLAVES_PER_AXIS）
    char axis_name[16];    // 轴名（例如 "X","Y","Z","A","B"）
    int32 pp_target_pos;   // PP模式目标位置（脉冲/PUU，32位有符号）
    int32 csp_base_pos;    // CSP模式的基准位置（用于相对轨迹计算）
    float csp_speed;       // CSP模式速度，单位为 脉冲/ms（浮点）
   


    // ② 动态状态（运行时实时更新，每个轴独立）
    int cia_step;          // CiA402状态机步骤（0-6）
    int cia_step_delay;    // 步骤延时计数器
    int pp_trigger_sent;   // PP触发标志（0=未触发，1=已触发）
    int is_op_ready;       // 操作使能就绪（0=否，1=是）
    int is_target_reach;   // 目标到达（0=否，1=是）
    int is_error;          // 故障标志（0=无错，1=故障）
    int state;
    int is_enabled;
    int32_t target_pos;    // 目标位置（脉冲），由上层命令或轨迹生成器设置
    int32_t actual_pos;    // 实际位置（脉冲），从驱动器或编码器反馈

    int32_t home_offset[MAX_SLAVES_PER_AXIS]; // 归零/原点偏移（每个从站）
    double current_cmd_pos; // 当前命令位置（以工程单位表示，用于UI/控制）
    double pulse_per_unit; // 脉冲/单位（如 脉冲/mm 或 脉冲/度），用于位置/速度换算

    // ⑤ 仿真存根字段 (仅 g_sim_mode == 1 时使用)
    // sim_actual_pos 已迁移到 sim_drive.c 的 g_sim_axis[].motor[].pos
    // (一阶低通推算, 不再等于 sim_target_pos, 可产生真实跟随误差)
    uint16_t sim_cmd_cw;       // 上周期 RT 线程发送的控制字
    int32_t sim_target_pos;    // 上周期 RT 线程发送的目标位置 (脉冲)

    // ④ 轴动力学参数（由 SMC_ConfigAxisDynamics 配置）
    int axis_type;        // 0: 线性轴 (Linear), 1: 旋转轴 (Rotary)
    double max_speed;     // 单轴最大允许速度 (mm/s 或 deg/s)
    double max_acc;       // 单轴最大允许加速度 (mm/s^2 或 deg/s^2)
    double max_dec;       // 单轴最大允许减速度 (mm/s^2 或 deg/s^2)
    double max_jerk;      // 单轴最大加加速度 (mm/s^3 或 deg/s^3)，默认 5000.0
    double equivalent_radius; // 旋转轴的物理半径，单位: mm (用于弧长换算: mm = deg * (PI/180) * radius)

    // ③  软件限位参数（可选，视驱动器支持情况而定）
    int enable_soft_limit;   // 软件限位使能（0=否，1=是）
    double soft_limit_pos;  // 软件限位位置- 正向限位
    double soft_limit_neg;  // 软件限位位置- 负向限位

    //
    int enable_sync_alarm;    // 同步报警使能（0=否，1=是）
    int32_t sync_tolerance_pulse; // 同步容差（脉冲），用于判断同步异常
    int32_t sync_max_err_pulse;   // 同步最大误差（脉冲），超过则判定为同步故障
    int sync_err_time_ms;       // 同步误差持续时间（ms），超过则判定为同步故障
    int _current_sync_timer;    // 同步误差计时器（ms），用于跟踪同步异常持续时间

    int32_t _follow_err_timer;  // 跟随误差警告持续时间计时器（ms）
    
} AxisCtrl_t;

/************************ 规划器全局参数 ************************/
typedef struct {
    double corner_tolerance;      // G64 拐角容差 (mm)
    double max_centripetal_acc;   // G64 最大向心加速度 (mm/s^2)
} PlannerConfig_t;

extern PlannerConfig_t g_planner_config;


typedef enum{
    HOLD_NORMAL=0,
    HOLD_BRAKING,
    HOLD_PAUSED,
    HOLD_RESUMING //

}FeedHoldState_t;

typedef struct{
    _Atomic int is_moving;

    double start_pos[AXIS_NUM];
    double target_pos[AXIS_NUM];
    double dir_vec[AXIS_NUM];

    double current_pos[AXIS_NUM];
    double v_max,v_start,v_end;

    double virtual_time_ms;
    double time_scale;
    FeedHoldState_t hold_state;
    int pause_request;
    _Atomic int alarm_reset_request;

    double total_distance;

    // ---- 7段式 S 曲线绝对解析参数（由 planner 预计算，RT 线程只读）----
    // T[n] = 第 n 阶段结束时的累计虚拟时间（ms），T7 = 总时长
    double T1, T2, T3, T4, T5, T6, T7;
    // v[n] = 第 n 阶段入口处的瞬时速度（mm/ms）
    double v0, v1, v2, v3, v4, v5, v6;
    // s[n] = 第 n 阶段入口处的累计位移（mm）
    double s0, s1, s2, s3, s4, s5, s6;
    // 各阶段控制量：j=jerk(mm/ms^3), a=加速度(mm/ms^2)
    double j1, a2, j3, j5, a6, j7;

    double v_target;
    double acc;
    double dec;

    double v_current;  // 当前周期的真实物理瞬时速度 (mm/ms),由 S 曲线 7 段解析求得
    int current_phase; // S 曲线当前 phase (1..7),跨周期缓存,以减少 if-else 链分支预测失败
    double phase_T_curr; // 当前 phase 下边界 T_{phase-1} (phase=1 时为 0),用于 dt = t - _T_curr
    double phase_T_next; // 当前 phase 上边界 T_phase (phase=7 时为 T7),用于前向 phase 推进判定

    int is_waiting_mcode;       // M代码等待屏障标志
    int32_t mcode_wait_timer;   // M代码非阻塞延时计数器（ms）
    int current_mcode;          // 当前等待中的M代码编号

    // ---- P1': 辅助状态机 RT 镜像 (RT 单写者, 消费 M 代码段时从 seg 同步) ----
    // parser 端 g_state 的对应 modal 字段先快照到 TrajectorySegment_t,
    // RT 消费时拷到这里, 供 sim CSV trace / 未来 HMI RPC 读出。
    int    spindle_mode_rt;     // 0=off(M5), 1=CW(M3), 2=CCW(M4)
    double spindle_rpm_rt;      // S 代码最近值 (rpm)
    int    coolant_state_rt;    // bit flags: bit0=flood(M8), bit1=mist(M7); 0/1/2/3 见 g_state 注释
    int    current_tool_id_rt;  // T 代码当前刀号 (M6 切换后)

    // ---- P0-Laser: RT 镜像 (RT 单写者, 消费 seg 时同步) ----
    // 与 spindle_*_rt / coolant_state_rt 同语义, 供 sim CSV trace / 未来 HMI 读出.
    // 写入点: ecat_thread_rt 段消费环 seg 拷出后, 由 laser_rt_apply_aux() 推进.
    // Phase B2: G04 dwell 等 M 段携带参数 (M64 段 p_value=dwell_ms)
    double mcode_p_value_ms;   // 段消费时从 seg.p_value 同步, switch case 64 读

    int    laser_enable_rt;
    int    laser_shutter_rt;
    double laser_power_w_rt;
    double laser_freq_hz_rt;
    int    gas_select_rt;

    // ---- P0-c: 实时光标 (RT 消费段时记录, snapshot 镜像, UI 高亮当前段) ----
    // current_seg_id_rt: 段加载时从 seg.seg_id 拷贝 (ecat_thread_rt 段消费环).
    //                    UI 据此在 G 代码编辑器高亮当前行 + 在轨迹上标记当前段。
    //                    静止时 (is_moving=0) 保留上一段 id, UI 可显示"刚执行完"。
    uint64_t current_seg_id_rt;

    // ---- P0-Laser-Q: 段级工艺标记镜像 (与 current_seg_id_rt 同步) ----
    // M 段 + 运动段消费时都从 seg.seg_flags 拷贝, HMI 据此判断:
    //   "当前段是不是引线/微连接" + "穿孔 dwell 期间是不是 lead_in 上下文"
    // 段级快照原则 (B1 教训): 走段消费环同步, 不走全局读 g_state.laser_seg_flags.
    uint8_t  current_seg_flags_rt;

}Interpolator_t;

/*
 * Interpolator_t 字段说明（绝对解析式 S 曲线版）
 * RT 线程通过 virtual_time_ms 与 T1~T7 比较确定当前阶段，
 * 使用预计算的 v[n]/s[n]/控制量 进行无状态绝对解析求 s。
 * T1=加加速结束, T2=匀加速结束, T3=减加速结束, T4=匀速结束,
 * T5=加减速结束, T6=匀减速结束, T7=减减速结束=总时长。
 */

// ================================================================
// 坐标系枚举 — 必须在 TrajectorySegment_t 之前定义
// (TrajectorySegment_t.active_wcs / CoordManager_t.current_coord 都用此类型)
// ================================================================
typedef enum{

    COORD_G53=0,
    COORD_G54=1,
    COORD_G55=2,
    COORD_G56=3,
    COORD_G57=4,
    COORD_G58=5,
    COORD_G59=6,

}CoordSystem_t;

typedef struct{
    double target_pos[AXIS_NUM];
    _Atomic int is_ready;
    int32_t speed;

    int cmd_type;       // CMD_TYPE_MOTION 或 CMD_TYPE_MCODE
    // ---- P0-b v1: 段元数据 (UI 预览/光标用, 不参与 RT 插补决策) ----
    // seg_id:    axis_ctrl.c 入队时 atomic fetch_add g_seg_id_counter, 全局唯一单调递增.
    //            UI 用此 ID 做丢帧检测 (相邻段 +1) + 实时光标定位 (P0-c 用).
    // line_no:   parser 入口设 = g_current_program->lines[g_pc].line_no (1-based 源行号).
    //            UI G 代码编辑器据此高亮当前执行行.
    // motion_type: parser case 'G' 设 (0=G00/1=G01/2=G02/3=G03/4=NURBS/0xFF=OTHER).
    //              UI 用此字段给轨迹上色 (G00 灰/G01 蓝/G02-G03 绿/NURBS 紫).
    // seg_flags: Laser B4 段级工艺标记 (SEG_FLAG_LEAD_IN/MICRO_JOINT), parser M72-M75 modal.
    //            UI 据此区分引线/微连接段 (虚线渲染), 不影响 RT 插补/激光功率.
    //            注: 加此 1B 字段复用 motion_type 后的 padding, 结构体总大小不变.
    uint64_t seg_id;
    int32_t  line_no;
    uint8_t  motion_type;
    uint8_t  seg_flags;
    int is_fillet;      // 几何锁定标识(0/1)：1=圆弧子段、G93微段、B-样条透传段。标记为1时，禁止 Planner 对其进行 G64 拐角二次抹圆篡改。
    int is_g93_strict;  // 1=G93 强一致性段: 纯匀速,planner 不得 S 曲线限幅
    int is_rtcp_active; // 1=RTCP 路径产生段(经 Kinematics_Inverse 物理逆解);
                        // 仅元数据: 供 Trace 日志分类与未来度量扩展,
                        // 不参与插补决策 (target_pos 已是物理关节坐标)
    // 工件坐标系索引 (G53..G59)，由 Parser 在 push 时从 g_state.modal_wcs 盖章。
    // RT 线程在消费本段时据此更新 g_coord_mgr.current_coord，避免 parser/RT 时序错位
    // 导致 UI 显示与宏系统变量 #5001+ 跳变到"未来坐标系"。
    // 见 CLAUDE.md 红线 #2: 段内 target_pos 已是机械绝对坐标，本字段仅用于 UI/宏显示侧。
    CoordSystem_t active_wcs;
    // WCS 偏置向量快照 (H-1 修复): 段入队时刻 work_offsets[wcs-1] 的值拷贝。
    // RT 线程消费段时拷到 g_coord_mgr.active_offset, 用于 current_logical_pos 推导。
    // 目的: 杜绝 parser 解析中途 `#5221=..` / G10 L2 改 work_offsets 时,
    // RT 用新偏置推旧位置的瞬间撕裂。G53 段此向量全 0。
    double wcs_offset_snap[AXIS_NUM];
    int m_code;         // M代码编号（如 3=M03, 5=M05）
    double s_value;     // S值（如主轴转速）
    double p_value;     // P参数（激光功率、宏程序参数等）
    double q_value;     // Q参数
    double r_value;     // R参数
    // ---- P1': 辅助状态机快照 (parser 入队时从 g_state 拷贝) ----
    // RT 消费 M 代码段时同步到 g_interpolator 的 _rt 镜像字段。
    int    aux_spindle_mode;    // 0=off, 1=CW, 2=CCW
    double aux_spindle_rpm;     // rpm
    int    aux_coolant;         // bit flags: bit0=flood, bit1=mist (与 g_state.coolant_state 同语义)
    int    aux_tool_id;         // 当前刀号

    // ---- P0-Laser: 激光辅助状态机快照 (parser → seg → RT 同步链路) ----
    // parser 入队时从 g_state 拷贝; RT 消费 seg 时同步到 laser_*_rt + g_laser_rt.
    // 每段都拷 (含运动段), 保证激光开/关与运动段 1ms 边界严格对齐.
    int    aux_laser_enable;       // 0=off, 1=on (与 M3/M5 联动)
    int    aux_laser_shutter;      // 0=off, 1=on (M62/M63 同步)
    double aux_laser_power_w;      // M67 E<n> 设置 (W)
    double aux_laser_freq_hz;      // M68 E<n> 设置 (Hz)
    int    aux_gas_select;         // 0=off, 1=N2, 2=O2, 3=Air (M10/M11/M12)
    // ---- Phase B1: 段级耦合配置快照 (修复架构 BUG) ----
    // 历史设计错误: 把 coupling_mode 作为全局运行时状态, RT 每 cycle 读 g_laser_cfg.
    //   后果: parser 在 RT 第一次读之前就把 mode 改回 (M70 P1 → M70 P0 顺序覆盖),
    //         RT 全程看到 mode=0.
    // 正确设计: 段级快照, parser 入队时把当时的 mode 冻结到 seg, RT 消费段时同步.
    //   保证段执行期间用入队时的 mode, 不受后续 M70 写入影响.
    int    aux_laser_coupling_mode;   // 入队时 g_laser_cfg.coupling_mode 快照
    double aux_laser_v_thresh;        // 入队时 g_laser_cfg.v_thresh_mm_s 快照

    double total_distance;
    double dir_vec[AXIS_NUM];
    //double dir_x,dir_y,dir_z; // 运动方向单位向量
    
    double v_max;
    double v_start;
    double v_end;

    // ---- 7段式 S 曲线绝对解析参数（由 planner 预计算）----
    double T1, T2, T3, T4, T5, T6, T7;
    double v0, v1, v2, v3, v4, v5, v6;
    double s0, s1, s2, s3, s4, s5, s6;
    double j1, a2, j3, j5, a6, j7;
    double T_total;  // = T7，总时长（ms）

    double v_target;
    double acc;
    double dec;
    double jerk;
}TrajectorySegment_t;


/* ================================================================
 * CommandQueue_t — Hybrid Concurrency 环形队列 (Cache-Line Isolated)
 *
 * 并发模型 (混合并发):
 *   ✦ RT 线程 (ecat_thread_rt): 绝对 Lock-Free 消费者
 *     - 仅 acquire 读 is_ready, release 写 read_tail
 *     - 永不接触 queue_spinlock,保证 1ms 硬实时不受任何锁影响
 *     - 优先级反转免疫: 高优先级 RT 不会被低优先级后台线程卡住
 *
 *   ✦ 后台线程 (Parser / BSpline / Planner / Watchdog): queue_spinlock 互斥
 *     - 所有修改 buffer 内容 + 推进 write_head 的操作必须持锁
 *     - 包括 fillet 的内存平移 (dst=src+K 批量 memcpy 风格拷贝)
 *     - 生产者 push: spin-wait 获取锁,临界区极短 (µs 级)
 *     - Planner: try-lock 避让,失败立即返回 (不阻塞生产者)
 *     - Watchdog: spin-wait 获取锁,保证 force_flush 必然成功
 *
 * 设计契约 (不可打破的不变量):
 *   1. queue_spinlock 持有期间: write_head 不会变化 (其他后台写者全部阻塞)
 *   2. RT 线程消费侧: is_ready=1 (release) → is_ready==1 (acquire)
 *      建立 happens-before,RT 线程通过此屏障看到所有 buffer 修改
 *   3. read_tail 单写者: 仅 RT 线程写入,其他线程只读
 *
 * Happens-before 链 (生产者数据 → RT 可见):
 *   producer write buffer[N] (relaxed)
 *     → producer release queue_spinlock
 *     → planner acquire queue_spinlock
 *     → planner write is_ready=1 (release)
 *     → RT acquire is_ready
 *     → RT read buffer[N]
 *   由 C11 传递性,所有 relaxed 写都被正确发布。
 *
 * Cache-Line 隔离 (_Alignas(64)):
 *   消除 write_head / read_tail / queue_spinlock 之间的 False-Sharing。
 * ================================================================
 */
typedef struct {
    TrajectorySegment_t buffer[QUEUE_SIZE];

    /* 生产者 (Parser / BSpline) 独占写入 —— 自身 cache line */
    _Alignas(64) _Atomic int write_head;

    /* 消费者 (RT 线程) 独占写入 —— 自身 cache line */
    _Alignas(64) _Atomic int read_tail;

    /* 后台线程互斥锁 (producer/planner/watchdog 共用) —— 自身 cache line */
    _Alignas(64) atomic_flag queue_spinlock;
} CommandQueue_t;

typedef struct{

    CoordSystem_t current_coord;   // 当前坐标系 (RT 线程为唯一写者: 消费段时 = seg.active_wcs)
    double work_offsets[6][AXIS_NUM]; // 各坐标系的工件坐标偏置（mm或度），例如 word_offsets[1] 就是 G54 的偏置
                                   // NOTE: 此表 RT 不再读 — RT 用 active_offset 推导逻辑坐标。
                                   //       仅 parser / SMC API / Macro 读, parser 可直接写。
    double work_offsets_ext[48][AXIS_NUM]; // P5': G54.1 P1-P48 扩展 WCS 偏置 (Fanuc 标准 48 组)
                                   // 写入路径: macro_eval.c #7001-#7948, 或未来 G10 L20 Pn
    double active_offset[AXIS_NUM]; // 当前生效偏置向量 (RT 单写者, = seg.wcs_offset_snap)
                                    // current_logical_pos = current_g53_pos - active_offset
    double current_g53_pos[AXIS_NUM]; // 当前G53坐标位置（机床坐标），实时更新用于UI显示和坐标转换
    double current_logical_pos[AXIS_NUM];// 当前逻辑坐标位置（相对于当前坐标系），实时更新用于UI显示和控制

}CoordManager_t;

#endif // AXIS_CFG_H
