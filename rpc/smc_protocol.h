#ifndef SMC_PROTOCOL_H
#define SMC_PROTOCOL_H

/* =====================================================================
 *  smc_protocol.h  ——  CNC Core RPC 共享通讯协议 (全量映射 smc_api.h)
 *
 *  作用: CNC Core (Linux/纯C) 与 CAM/HMI (Windows或Linux/C++) 之间
 *        通过自定义二进制 TCP 协议交换的"信封"与"载荷"定义。
 *
 *  设计原则:
 *    1. 纯 C 语法, 同时被 rpc_server.c (C) 与 SmcControllerSdk (C++) 包含。
 *    2. #pragma pack(push, 1) 强制一字节对齐, 防止跨编译器/跨位宽结构体空洞。
 *    3. 所有整数字段使用 stdint.h 精确位宽 (uint16_t/int32_t/...),
 *       SDK 公开方法可用 int 接收, 隐式转换到 int32_t 入 wire。
 *    4. 字符串字段使用定长数组 (netif_name/axis_name/filepath/status_str),
 *       C 端无需做长度前缀解析, 接收端再强制 '\0' 终结防越界。
 *    5. 函数原本返回 void 时, 不定义 Res 结构 (响应 data_len = 0);
 *       原本返回 int 时, Res 仅含 int32_t ret_code, 与协议层 err_code 解耦。
 *
 *  字节序: 默认双方同字节序 (x86/ARM 小端)。跨序部署需在 SmcReqHeader
 *          加 magic 字段并对多字节字段做 ntohl/htonl。当前版本暂不引入。
 * ===================================================================== */

#include <stdint.h>

/* AXIS_NUM 兼容 (P0-b v2): SMC_ProgramStructure_t.bbox_min/max 用。
 * CNC Core 端 axis_cfg.h 已定义, 此处允许外部预定义 (Windows SDK 包含本头时 fallback 5)。 */
#ifndef AXIS_NUM
#define AXIS_NUM 5
#endif

#ifdef __cplusplus
extern "C" {
#endif

#pragma pack(push, 1)

/* ============================================================
 * 1. 公共常量
 * ============================================================ */
#define SMC_MAX_PAYLOAD_BYTES    4096   /* 单包 payload 防御性上限 */
#define SMC_AXIS_ALL             '*'    /* 全轴通配符, 与 smc_api.h 一致 */

#define SMC_NETIF_NAME_MAX_LEN   32     /* 网卡名 e.g. "eth0" */
#define SMC_AXIS_NAME_MAX_LEN    32     /* 轴名 e.g. "X轴", "B" */
#define SMC_FILEPATH_MAX_LEN     256    /* G 代码文件绝对路径 */
#define SMC_STATUS_STR_MAX_LEN   32     /* 系统状态字符串缓冲 */

/* ============================================================
 * 1.5 激光 P-v 耦合查表共享类型 (P0-Laser-ConfigRPC 移入)
 * 原 laser_ctrl.h 本地定义, 因 SDK 端 SMC_ConfigLaserCoupleTable 需要可见,
 * 移到协议头让 C 端 + C++ SDK 共享同一布局.
 * pack(1) 下 16B (2×double 无 padding), 与原 laser_ctrl.h 自然对齐布局一致.
 * ============================================================ */
#define LASER_COUPLE_TABLE_MAX  16      /* P-v 耦合表最大采样点数 */

typedef struct {
    double v_mm_s;   /* 速度采样点 (mm/s), 单调不减 */
    double ratio;    /* 输出比例 (0-1) */
} LaserCouplePoint_t;

/* ============================================================
 * 2. 命令类型枚举
 *    按功能段划分并预留空号, 便于未来追加新 API 而不打乱编号。
 * ============================================================ */
typedef enum {
    /* --- 系统生命周期 0x0001 ~ 0x000F --- */
    SMC_CMD_INIT_AND_START           = 0x0001,
    SMC_CMD_CLOSE                    = 0x0002,

    /* --- 轴配置 0x0010 ~ 0x001F --- */
    SMC_CMD_CONFIG_AXIS_TOPOLOGY     = 0x0010,
    SMC_CMD_CONFIG_SOFT_LIMIT        = 0x0011,
    SMC_CMD_CONFIG_GANTRY_SYNC_ALARM = 0x0012,
    SMC_CMD_CONFIG_PULSE_PER_UNIT    = 0x0013,
    SMC_CMD_CONFIG_AXIS_DYNAMICS     = 0x0014,
    SMC_CMD_CONFIG_PLANNER_PARAMS    = 0x0015,
    SMC_CMD_CONFIG_KINEMATICS_OFFSET = 0x0016,
    SMC_CMD_CONFIG_KINEMATICS        = 0x0017,
    SMC_CMD_INJECT_AXIS_FAULT        = 0x0018,  /* sim 模式故障注入 */
    SMC_CMD_CONFIG_SIM_DYNAMICS      = 0x0019,  /* sim 一阶伺服 alpha 配置 */

    /* --- 坐标与状态查询 0x0020 ~ 0x002F --- */
    SMC_CMD_GET_LOGICAL_POS          = 0x0020,
    SMC_CMD_IS_PARSER_RUNNING        = 0x0021,
    SMC_CMD_IS_MOTION_DONE           = 0x0022,
    SMC_CMD_GET_QUEUE_COUNT          = 0x0023,
    SMC_CMD_IS_AXIS_CONFIGURED       = 0x0024,
    SMC_CMD_GET_SYSTEM_STATUS        = 0x0025,
    SMC_CMD_GET_SPINDLE_STATE        = 0x0026,  /* 主轴模态 + 转速查询 */
    SMC_CMD_GET_COOLANT_STATE        = 0x0027,  /* 冷却液 bit flags 查询 */
    SMC_CMD_GET_CURRENT_TOOL         = 0x0028,  /* 当前刀号查询 */
    SMC_CMD_SET_OPTIONAL_STOP_ENABLE = 0x0029,  /* M1 可选停开关 */
    SMC_CMD_SUBSCRIBE                = 0x002A,  /* P0-a: 订阅状态推送通道 (9528 端口握手用) */
    SMC_CMD_PREVIEW_SUBSCRIBE        = 0x002B,  /* P0-b v1: 订阅段流推送通道 (9529 端口握手用) */
    SMC_CMD_LOAD_PROGRAM             = 0x002C,  /* P0-b v2: 加载程序 (仅 preview, 不执行) */
    SMC_CMD_RUN_LOADED_PROGRAM       = 0x002D,  /* P0-b v2: 执行已加载程序 */
    SMC_CMD_GET_PROGRAM_STRUCTURE    = 0x002E,  /* P0-b v2: 查询程序结构元数据 */
    SMC_CMD_CLEAR_ALARM              = 0x002F,  /* P1-b: 清除系统报警 (触发 RT alarm_reset_request) */
    SMC_CMD_EVENT_SUBSCRIBE          = 0x0030,  /* P1-b: 订阅事件流推送通道 (9530 端口握手用) */

    /* --- 运动控制 0x0030 ~ 0x003F --- */
    SMC_CMD_SET_ZERO                 = 0x0030,
    SMC_CMD_MOVE_RELATIVE            = 0x0031,
    SMC_CMD_GO_ZERO                  = 0x0032,
    SMC_CMD_GET_LASER_STATE          = 0x0033,  /* P0-Laser-Q: 激光器完整状态查询 (含统计) */
    SMC_CMD_SET_OVERRIDE             = 0x0034,  /* P2-A: 实时倍率 + 模式标志设置 */
    SMC_CMD_GET_OVERRIDE             = 0x0035,  /* P2-A: 实时倍率 + 模式标志查询 */

    /* --- G 代码加工 0x0040 ~ 0x004F --- */
    SMC_CMD_RUN_GCODE_FILE           = 0x0040,
    SMC_CMD_PAUSE_PROCESSING         = 0x0041,
    SMC_CMD_RESUME_PROCESSING        = 0x0042,
    SMC_CMD_ABORT_PROCESSING         = 0x0043,

    /* --- 激光配置 0x0050 ~ 0x005F (P0-Laser-ConfigRPC) --- */
    SMC_CMD_CONFIG_LASER_IO          = 0x0050,  /* 配置 3 个 EtherCAT 从站 id */
    SMC_CMD_CONFIG_LASER_DO_BITS     = 0x0051,  /* 配置 DO bit 偏移 (6 位) */
    SMC_CMD_CONFIG_LASER_DI_BITS     = 0x0052,  /* 配置 DI bit 偏移 (6 位) */
    SMC_CMD_CONFIG_LASER_AO_CHANNELS = 0x0053,  /* 配置 AO 通道偏移 (2 通道) */
    SMC_CMD_CONFIG_LASER_RANGE       = 0x0054,  /* 配置功率/频率量程 */
    SMC_CMD_CONFIG_LASER_COUPLING    = 0x0055,  /* 配置 P-v 耦合开关 + 低速阈值 */
    SMC_CMD_CONFIG_LASER_COUPLE_TABLE= 0x0056,  /* 配置 P-v 耦合查表 (固定 16 槽) */

    /* --- Safe Z Lift 0x0057 ~ 0x005A (P0-3) --- */
    SMC_CMD_CONFIG_SAFE_LIFT         = 0x0057,  /* 配置抬升参数 (init 阶段) */
    SMC_CMD_SAFE_LIFT_TRIGGER        = 0x0058,  /* 手动触发抬升 (idempotent) */
    SMC_CMD_SAFE_LIFT_CANCEL         = 0x0059,  /* 取消抬升 (仅 PENDING/DONE) */
    SMC_CMD_GET_SAFE_LIFT            = 0x005A,  /* 查询抬升状态 */

    /* --- Homing 0x005B ~ 0x005F (P0-1) --- */
    SMC_CMD_CONFIG_HOMING_AXIS       = 0x005B,  /* 配置单轴回零参数 */
    SMC_CMD_CONFIG_HOMING_ORDER      = 0x005C,  /* 配置回零顺序 "ZXYBC" */
    SMC_CMD_HOMING_TRIGGER           = 0x005D,  /* 触发回零 (axis_letter 或 '\0'=All) */
    SMC_CMD_HOMING_CANCEL            = 0x005E,  /* 取消回零 (仅 PENDING/DONE) */
    SMC_CMD_GET_HOMING               = 0x005F,  /* 查询回零状态 */

    /* --- JOG 0x0060 ~ 0x0061 (P0-1, method 35 前置) --- */
    SMC_CMD_JOG_START                = 0x0060,  /* 启动 JOG (axis + direction + speed) */
    SMC_CMD_JOG_STOP                 = 0x0061,  /* 停止 JOG (axis 或 '*') */

    /* --- Emergency Stop 0x0062 (P0-A, ISO 13850 软件层) --- */
    SMC_CMD_EMERGENCY_STOP           = 0x0062,  /* 软急停一站式触发 (原子序列) */

    /* --- Gantry Pre-Align 0x0063 (B2, 2026-07-23) --- */
    SMC_CMD_CONFIG_GANTRY_ALIGN      = 0x0063,  /* 配置双驱龙门轴 pre-align 锚定前预对中 */

    /* --- Gantry Mock 0x0064 (B2 sim-only, 测试用) --- */
    SMC_CMD_INJECT_GANTRY_OFFSET     = 0x0064,  /* sim 模式注入主从静态差 (验证 pre-align) */

    /* --- Homing Order v2 0x0065 (B4, 2026-07-23) --- */
    SMC_CMD_CONFIG_HOMING_ORDER_EX   = 0x0065,  /* v2: order + auto_on_init (旧 0x005C 不变) */

    /* --- Set Origin Here 0x0066 (v1.5, 2026-08-28) --- */
    SMC_CMD_SET_ORIGIN_HERE          = 0x0066,  /* 当前位设为激活 WCS 零点 (main 时代 SetZero 语义) */
} SmcCmdType;

/* ============================================================
 * 3. 协议层错误码
 *    正数/0 = 成功; 负数 = 协议层/调用层失败。
 *    业务函数的返回值 (如 SMC_RunGCodeFile 的 int) 不复用此空间,
 *    放进各自 Response Payload, 二者职责分离。
 * ============================================================ */
typedef enum {
    SMC_OK              =  0,
    SMC_ERR_UNKNOWN_CMD = -1,   /* 未知 cmd_type */
    SMC_ERR_PARAM       = -2,   /* payload 长度不足或字段非法 */
    SMC_ERR_SOCKET      = -3,   /* recv/send 失败或对端关闭 */
    SMC_ERR_INTERNAL    = -4,   /* 内存分配/流失步等内部错误 */
} SmcErrCode;

/* ============================================================
 * 4. 包头 (一切命令通用, 定长)
 *    一次完整请求 = SmcReqHeader + Payload[data_len]
 *    一次完整响应 = SmcResHeader + Payload[data_len]
 * ============================================================ */
typedef struct {
    uint16_t cmd_type;     /* SmcCmdType */
    uint16_t data_len;     /* 紧随其后的 payload 字节数 */
} SmcReqHeader;            /* 4 字节 */

typedef struct {
    int32_t  err_code;     /* SmcErrCode, SMC_OK 表示业务成功 */
    uint32_t data_len;     /* 紧随其后的 payload 字节数 */
} SmcResHeader;            /* 8 字节 */

/* ============================================================
 * 5. 业务 Payload (与 smc_api.h 全量对应)
 * ============================================================ */

/* ----- 系统生命周期 ----- */
typedef struct {
    char netif_name[SMC_NETIF_NAME_MAX_LEN];
} SmcInitAndStartReq;

typedef struct {
    int32_t ret_code;
} SmcInitAndStartRes;

/* SMC_CLOSE: 无 Req, 无 Res (业务 void 返回) */

/* ----- 轴配置 ----- */
typedef struct {
    char    axis_name[SMC_AXIS_NAME_MAX_LEN];
    int32_t is_dual_drive;
    int32_t master_id;
    int32_t slave_id;
} SmcConfigAxisTopologyReq;
typedef struct {
    int32_t ret_code;
} SmcConfigAxisTopologyRes;

typedef struct {
    char    axis_letter;
    int32_t enable;
    double  neg_limit_mm;
    double  pos_limit_mm;
} SmcConfigSoftLimitReq;
typedef struct {
    int32_t ret_code;
} SmcConfigSoftLimitRes;

typedef struct {
    char    axis_letter;
    int32_t enable;
    int32_t tolerance_pulse;
    int32_t max_error_pulse;
    int32_t time_ms;
} SmcConfigGantrySyncAlarmReq;
typedef struct {
    int32_t ret_code;
} SmcConfigGantrySyncAlarmRes;

typedef struct {
    char   axis_letter;
    double pulse_per_unit;
} SmcConfigPulsePerUnitReq;
/* SMC_CONFIG_PULSE_PER_UNIT: void 返回, 无 Res */

typedef struct {
    char    axis_letter;
    int32_t type;
    double  max_v;
    double  max_a;
    double  max_d;
    double  equivalent_radius;
} SmcConfigAxisDynamicsReq;
typedef struct {
    int32_t ret_code;
} SmcConfigAxisDynamicsRes;

typedef struct {
    double tolerance;
    double max_centripetal_acc;
} SmcConfigPlannerParamsReq;
typedef struct {
    int32_t ret_code;
} SmcConfigPlannerParamsRes;

typedef struct {
    double tool_len;
    double pivot_x;
    double pivot_y;
    double pivot_z;
} SmcConfigKinematicsOffsetReq;
/* SMC_CONFIG_KINEMATICS_OFFSET: void 返回, 无 Res */

typedef struct {
    int32_t type;
    int32_t r1_idx;
    int32_t r1_axis;
    int32_t r2_idx;
    int32_t r2_axis;
    double  tool_off[3];
    double  pivot_off[3];
} SmcConfigKinematicsReq;
/* SMC_CONFIG_KINEMATICS: void 返回, 无 Res */

/* ----- 仿真驱动器 (sim 模式专用) ----- */
typedef struct {
    char    axis_letter;
    uint8_t slave_subidx;   /* 0=主 motor, 1=从 motor (双驱轴) */
} SmcInjectAxisFaultReq;
typedef struct {
    int32_t ret_code;       /* 0=成功, -1=参数越界, -2=非 sim 模式 */
} SmcInjectAxisFaultRes;

typedef struct {
    char    axis_letter;
    double  alpha;          /* 一阶伺服系数 (0,1), 默认 0.2 */
} SmcConfigSimDynamicsReq;
typedef struct {
    int32_t ret_code;
} SmcConfigSimDynamicsRes;

/* ----- 坐标与状态查询 ----- */
typedef struct {
    char axis_letter;
} SmcGetLogicalPosReq;
typedef struct {
    double position;       /* 单位: 脉冲 (机械绝对当量) */
} SmcGetLogicalPosRes;

/* SMC_IS_PARSER_RUNNING: 无 Req */
typedef struct {
    int32_t running;       /* 1=解析/加工中, 0=空闲 */
} SmcIsParserRunningRes;

/* SMC_IS_MOTION_DONE: 无 Req */
typedef struct {
    int32_t done;          /* 1=运动结束, 0=进行中 */
} SmcIsMotionDoneRes;

/* SMC_GET_QUEUE_COUNT: 无 Req */
typedef struct {
    int32_t count;         /* FIFO 堆积指令数 */
} SmcGetQueueCountRes;

typedef struct {
    char axis_letter;
} SmcIsAxisConfiguredReq;
typedef struct {
    int32_t configured;    /* 1=已映射, 0=未配置 */
} SmcIsAxisConfiguredRes;

/* SMC_GET_SYSTEM_STATUS_STR: 无 Req */
typedef struct {
    char status_str[SMC_STATUS_STR_MAX_LEN];  /* "ALARM"/"HOLD"/"RUN"/"IDLE" */
} SmcGetSystemStatusRes;

/* SMC_GET_SPINDLE_STATE: 无 Req */
typedef struct {
    int32_t mode;       /* 0=off(M5), 1=CW(M3), 2=CCW(M4) */
    double  rpm;        /* 最近有效 S 值 */
    int32_t ret_code;   /* 0=ok, -1=轴未就绪 */
} SmcGetSpindleStateRes;

/* SMC_GET_COOLANT_STATE: 无 Req */
typedef struct {
    int32_t state;      /* bit0=flood(M8), bit1=mist(M7); 0/1/2/3 */
    int32_t ret_code;
} SmcGetCoolantStateRes;

/* SMC_GET_CURRENT_TOOL: 无 Req */
typedef struct {
    int32_t tool_id;    /* 当前刀号 (最近 M6 切换值) */
    int32_t ret_code;
} SmcGetCurrentToolRes;

/* SMC_GET_LASER_STATE: 无 Req
 * 镜像 g_laser_rt (RT 单写者) + g_interpolator 派生字段, 共 14 字段.
 * 字段顺序: 状态 → 统计 → ret_code (与 SpindleStateRes ret_code 末置一致).
 * pack(1) 下 ~75B, 远低于 SMC_MAX_PAYLOAD_BYTES (4096). */
typedef struct {
    /* ---- 状态字段 (镜像 g_laser_rt, RT 单写者) ---- */
    int32_t  enable;            /* 0/1, 激光器主使能 (M3=1, M5=0) */
    int32_t  shutter;           /* 0/1, 激光闸 (M62=1, M63=0) */
    double   power_w;           /* 当前输出功率 W (P-v 耦合后) */
    double   freq_hz;           /* 频率 Hz */
    int32_t  gas_select;        /* 0=off, 1=N2, 2=O2, 3=Air */
    uint16_t interlock;         /* 互锁位图: bit0=door, bit1=estop_soft, bit2=laser_alm,
                                   bit3=water_temp, bit4=water_flow, bit5=gas_press,
                                   bit15=system_alarm; 0=正常 */
    int32_t  emergency_kill;    /* 急停锁存 (1=已锁存, 需 alarm_reset 路径调 laser_rt_reset) */
    double   P_base_w;          /* 基准功率 (M67 设定, 不被 P-v 耦合覆盖) */
    double   v_actual_mm_s;     /* 当前周期瞬时速度 mm/s (RT 写, 耦合 update 推进) */
    int32_t  coupling_mode_rt;  /* 当前段耦合模式 (0=off, 1=查表) */
    int32_t  is_piercing;       /* 是否在 G04 穿孔 dwell 中 (派生: is_waiting_mcode && current_mcode==64) */
    uint8_t  current_seg_flags; /* 当前段工艺标记: bit0=lead_in, bit1=micro_joint */
    /* ---- 加工统计 (RT 累计, 跨程序不清零) ---- */
    int32_t  pierce_count;      /* 累计穿孔次数 (M64 段完成 ++) */
    int64_t  laser_on_time_ms;  /* 累计激光开启时间 ms (enable&&shutter&&!ekill 时累加) */
    /* ---- 返回码 ---- */
    int32_t  ret_code;          /* 0=ok, -1=激光未配置 (do_slave_id<0) */
} SmcGetLaserStateRes;

typedef struct {
    int32_t enable;     /* 0=禁用 M1, 1=M1 等价 M0 */
} SmcSetOptionalStopEnableReq;
typedef struct {
    int32_t ret_code;
} SmcSetOptionalStopEnableRes;

/* ============================================================
 * P2-A: 实时倍率系统 mode_flags 位定义 (与 g_interpolator.mode_flags 一致)
 * UI/HMI 通过 SMC_SetOverride mask/value 模式修改, RT 每 cycle 读.
 * ============================================================ */
#define SMC_MODE_SINGLE_BLOCK     0x0001   /* bit0: 单段模式 (每段完成自动 feedhold, 等 Cycle Start) */
#define SMC_MODE_DRY_RUN          0x0002   /* bit1: 空运行 (G01/G02/G03 走 rapid_override 通道,
                                            *        spindle/coolant/laser 输出强制 0 - 工业安全 prove-out) */
#define SMC_MODE_OPTIONAL_STOP    0x0004   /* bit2: M1 可选停使能 (复用现有 SMC_SetOptionalStopEnable 语义,
                                            *        v1 保留位, 通过 SMC_SetOverride 写入不同步到 g_optional_stop_enabled,
                                            *        UI 调用 SMC_SetOptionalStopEnable 才生效) */
#define SMC_MODE_BLOCK_SKIP_VERIFY 0x0008  /* bit3: 跳步验证 (v1 预留位, / 跳步号校验, 不影响 RT) */
#define SMC_MODE_OVERRIDE_PERSIST  0x0010  /* bit4: M30/M2 程序结束时不重置 override 旋钮 (默认重置) */
/* bit5..15 预留 */

/* SMC_SET_OVERRIDE (0x0034): mask/value 模式修改倍率 + 模式
 * -1 = 不改 (允许部分修改), 0..120 = 设置值 (clamp 后通过 Res 回读)
 * 全部 -1 且 mask=0 视为 no-op, 返回 ret_code=-1 */
typedef struct {
    int32_t  feed_pct;        /* -1=不改, [0..100]% (v1 锁 100, 超 100 clamp) */
    int32_t  rapid_pct;       /* -1=不改, [0..100]% */
    int32_t  spindle_pct;     /* -1=不改, [0..120]% (50-120% 工业惯例, v1 允许 0%) */
    uint16_t mode_mask;       /* 要修改的 mode_flags 位 (0=不改任何位) */
    uint16_t mode_value;      /* mask 标识的位写入 0 或 1 */
} SmcSetOverrideReq;

typedef struct {
    int32_t  ret_code;            /* 0=ok, -1=no-op (全 -1 且 mask=0) */
    int32_t  actual_feed_pct;     /* clamp 后的实际生效值 */
    int32_t  actual_rapid_pct;
    int32_t  actual_spindle_pct;
    uint16_t actual_mode_flags;
    uint16_t _pad16;
} SmcSetOverrideRes;

/* SMC_GET_OVERRIDE (0x0035): 无 Req, Res 回读 RT 当前生效值 */
typedef struct {
    int32_t  feed_pct;
    int32_t  rapid_pct;
    int32_t  spindle_pct;
    uint16_t mode_flags;
    uint16_t _pad16;
    int32_t  ret_code;            /* 0=ok (永远成功, 即使参数 NULL 也返回 -1) */
} SmcGetOverrideRes;

/* ----- 运动控制 ----- */
typedef struct {
    char axis_letter;      /* '*' 全轴归零 */
} SmcSetZeroReq;
/* SMC_SET_ZERO: void 返回, 无 Res */

typedef struct {
    char   axis_letter;    /* '*' 全轴联动 */
    double distance;       /* mm */
    double speed;          /* 速度参数, 透明转发 */
} SmcMoveRelativeReq;
/* SMC_MOVE_RELATIVE: void 返回, 无 Res */

typedef struct {
    char   axis_letter;    /* '*' 全轴联动 */
    double speed;
} SmcGoZeroReq;
/* SMC_GO_ZERO: void 返回, 无 Res */

/* ----- G 代码加工 ----- */
typedef struct {
    char filepath[SMC_FILEPATH_MAX_LEN];
} SmcRunGCodeFileReq;
typedef struct {
    int32_t ret_code;      /* SMC_RunGCodeFile 原返回值 */
} SmcRunGCodeFileRes;

/* SMC_PAUSE_PROCESSING / RESUME_PROCESSING / ABORT_PROCESSING: 无 Req, 无 Res */

/* ----- P0-b v2: LoadProgram / RunLoadedProgram / GetProgramStructure ----- */
typedef struct {
    char filepath[SMC_FILEPATH_MAX_LEN];
} SmcLoadProgramReq;
typedef struct {
    int32_t ret_code;   /* 0=ok, -1=is_running (parser 忙), -2=Program_Load 失败 */
} SmcLoadProgramRes;

/* SMC_RUN_LOADED_PROGRAM: 无 Req */
typedef struct {
    int32_t ret_code;   /* 0=ok, -1=load_done!=1 (LoadProgram 未完成), -2=is_running */
} SmcRunLoadedProgramRes;

/* SMC_GET_PROGRAM_STRUCTURE: 无 Req, payload 直接是 SMC_ProgramStructure_t */
typedef struct {
    int32_t  ret_code;            /* 0=ok, -1=未加载 */
    char     filepath[SMC_FILEPATH_MAX_LEN];
    int32_t  is_loaded;           /* 0=未加载, 1=loaded(preview done 待 run), 2=running, 3=done */
    int32_t  total_lines;         /* g_current_program->num_lines (源码行数) */
    int32_t  total_segments;      /* PreviewStreamer_GetWriteSeq() (本程序段数, 累计跨程序) */
    int32_t  num_o_labels;        /* g_current_program->num_o_labels */
    int32_t  num_n_labels;        /* g_current_program->num_n_labels */
    uint64_t first_seg_id;        /* 本程序 load 阶段首段 seg_id */
    uint64_t last_seg_id;         /* 本程序 load 阶段末段 seg_id */
    double   estimated_time_ms;   /* sum T_total (run 模式有效, preview=0) */
    double   bbox_min[AXIS_NUM];  /* 边界框 min (load + run 都更新, 哨兵 ±1e18) */
    double   bbox_max[AXIS_NUM];
} SmcGetProgramStructureRes;

/* ----- P1-b: ClearAlarm + Event stream ----- */
/* SMC_CLEAR_ALARM: 无 Req */
typedef struct {
    int32_t ret_code;   /* 0=请求已提交 (实际清在 RT 异步), -1=轴未就绪 */
} SmcClearAlarmRes;

/* SMC_EVENT_SUBSCRIBE: SmcReqHeader + payload{int32 freq_hz, uint64 from_seq} */
/* SubscribeAck 与 SmcEventFrameHeader 同尺寸 16B, 便于 client 复用读缓冲 */
typedef struct {
    uint32_t magic;            /* SMC_EVENT_ACK_MAGIC = 0x45564E4B ("EVNK") */
    uint32_t version;          /* SMC_EVENT_VERSION */
    uint32_t max_per_tick;     /* server 单帧最多事件数 (= EVENT_READ_MAX=32) */
    uint32_t event_size_bytes; /* sizeof(SmcEvent_t) = 88, client 据此分配缓冲 */
} SmcEventAck;

/* 事件帧头 (16B) + N × SmcEvent_t (88B each)
 * CRC32 覆盖: event_count 字段 + events payload (与 preview 帧同模式) */
typedef struct {
    uint32_t magic;            /* SMC_EVENT_MAGIC = 0x45564E54 ("EVNT") */
    uint32_t version;          /* SMC_EVENT_VERSION */
    uint32_t event_count;      /* 本帧事件数 (1..EVENT_READ_MAX) */
    uint32_t crc32;            /* CRC32(event_count + events) */
} SmcEventFrameHeader;

/* ============================================================
 * 激光配置 Payload (0x0050-0x0056, P0-Laser-ConfigRPC)
 * 7 组 Req/Res, Res 统一仅含 int32_t ret_code (与 SMC_ConfigLaser* 返回值对齐)
 * CoupleTable Req 固定 16 槽 (count 指示实际有效数), pack(1) 260B
 * ============================================================ */

/* SMC_CONFIG_LASER_IO: 3 个 EtherCAT 从站 id, -1=未配置 */
typedef struct {
    int32_t do_slave_id;    /* 数字输出从站 */
    int32_t ao_slave_id;    /* 模拟输出从站 */
    int32_t di_slave_id;    /* 安全互锁输入从站 */
} SmcConfigLaserIOReq;
typedef struct { int32_t ret_code; } SmcConfigLaserIORes;

/* SMC_CONFIG_LASER_DO_BITS: 6 个 DO bit 偏移 (0-15) */
typedef struct {
    uint8_t b_enable;
    uint8_t b_shutter;
    uint8_t b_gas_n2;
    uint8_t b_gas_o2;
    uint8_t b_gas_air;
    uint8_t b_alarm_lamp;
} SmcConfigLaserDOBitsReq;     /* pack(1) 6B */
typedef struct { int32_t ret_code; } SmcConfigLaserDOBitsRes;

/* SMC_CONFIG_LASER_DI_BITS: 6 个 DI bit 偏移 (0-15) */
typedef struct {
    uint8_t b_door;
    uint8_t b_estop;
    uint8_t b_laser_alm;
    uint8_t b_water_t;
    uint8_t b_water_f;
    uint8_t b_gas_p;
} SmcConfigLaserDIBitsReq;
typedef struct { int32_t ret_code; } SmcConfigLaserDIBitsRes;

/* SMC_CONFIG_LASER_AO_CHANNELS: 2 个 AO 通道偏移 */
typedef struct {
    uint8_t ch_power;
    uint8_t ch_freq;
} SmcConfigLaserAOChannelsReq;
typedef struct { int32_t ret_code; } SmcConfigLaserAOChannelsRes;

/* SMC_CONFIG_LASER_RANGE: 物理量程 (功率/频率 W/Hz) */
typedef struct {
    double power_max_w;     /* 满量程功率, 默认 3000 */
    double freq_max_hz;     /* 满量程频率, 默认 5000 */
    double power_min_w;     /* 起辉功率下限, 默认 50 */
} SmcConfigLaserRangeReq;
typedef struct { int32_t ret_code; } SmcConfigLaserRangeRes;

/* SMC_CONFIG_LASER_COUPLING: P-v 耦合开关 + 低速阈值 */
typedef struct {
    int32_t mode;           /* 0=off (默认), 1=查表耦合 */
    double  v_thresh_mm_s;  /* 低速阈值 mm/s, 默认 5.0 */
} SmcConfigLaserCouplingReq;
typedef struct { int32_t ret_code; } SmcConfigLaserCouplingRes;

/* SMC_CONFIG_LASER_COUPLE_TABLE: P-v 耦合查表 (固定 16 槽)
 * count: 实际有效采样点数 (1..16), 超范围 SMC_ConfigLaserCoupleTable 返回 -1
 * points: 固定 16 槽, 未用槽为零值 (count 之后的内容被忽略)
 * pack(1) 下 4 + 16*16 = 260B, 远低于 SMC_MAX_PAYLOAD_BYTES */
typedef struct {
    int32_t            count;
    LaserCouplePoint_t points[LASER_COUPLE_TABLE_MAX];
} SmcConfigLaserCoupleTableReq;
typedef struct { int32_t ret_code; } SmcConfigLaserCoupleTableRes;

/* ============================================================
 * P0-3: Safe Z Lift
 * ============================================================ */

/* SMC_CONFIG_SAFE_LIFT (0x0057): 配置抬升参数 (init 阶段, InitAndStart 之前调)
 *   z_letter:        Z 轴字母 ASCII ('Z'=0x5A, 大小写不敏感由 API 端 toupper 处理)
 *   safe_z_mm:       G53 绝对目标 (mm)
 *   lift_speed_mm_s: 抬升速度 (默认 20.0)
 *   auto_on_alarm:   0/1 报警路径自动触发开关
 * pack(1) 下 1 + 3(pad) + 8 + 8 + 4 = 24B
 */
typedef struct {
    uint8_t z_letter;          /* 'Z' (大小写不敏感) */
    uint8_t _pad[3];           /* 4B 对齐 */
    double  safe_z_mm;
    double  lift_speed_mm_s;
    int32_t auto_on_alarm;
} SmcConfigSafeLiftReq;
typedef struct { int32_t ret_code; } SmcConfigSafeLiftRes;

/* SMC_SAFE_LIFT_TRIGGER (0x0058): 手动触发抬升 (idempotent) — 无 payload */
typedef struct { int32_t ret_code; } SmcSafeLiftTriggerRes;

/* SMC_SAFE_LIFT_CANCEL (0x0059): 取消抬升 — 无 payload */
typedef struct { int32_t ret_code; } SmcSafeLiftCancelRes;

/* SMC_GET_SAFE_LIFT (0x005A): 查询抬升状态
 *   state:      0=IDLE, 1=PENDING, 2=RUNNING, 3=DONE
 *   progress_mm: 已抬升距离 (current_z - start_z), DONE 时 = 总抬升量
 *   z_target_mm / z_current_mm: HMI 显示用
 *   enabled: 0=未配置 (整个 SafeLift 禁用), 1=已配置
 * pack(1) 实际 = 4×int32(ret_code,state,enabled,_pad=16B) + 3×double(progress,z_target,z_current=24B) = 40B
 *   (旧注释误算为 36B, 漏算 state 字段; 客户端必须以 40B 解包, 否则 unpack 报 buffer too short)
 */
typedef struct {
    int32_t ret_code;
    int32_t state;
    int32_t enabled;
    int32_t _pad;
    double  progress_mm;
    double  z_target_mm;
    double  z_current_mm;
} SmcGetSafeLiftRes;

/* ============================================================
 * P0-1: Homing + JOG
 * ============================================================ */

/* SMC_CONFIG_HOMING_AXIS (0x005B): 单轴回零配置 (init 阶段) */
typedef struct {
    uint8_t z_letter;
    uint8_t _pad[3];
    int32_t method;           /* 35 (v1) / 1-19 (v2 预留, v1 拒绝 -3) */
    double  search_speed_mm_s;
    double  creep_speed_mm_s;
    int32_t direction;        /* +1 / -1 */
    int32_t timeout_ms;
} SmcConfigHomingAxisReq;
typedef struct { int32_t ret_code; } SmcConfigHomingAxisRes;

/* SMC_CONFIG_HOMING_ORDER (0x005C): 全局回零顺序配置 */
#define SMC_HOMING_ORDER_MAX_LEN  16  /* "ZXYBC" + 余量 */
typedef struct {
    char order_letters[SMC_HOMING_ORDER_MAX_LEN];
} SmcConfigHomingOrderReq;
typedef struct { int32_t ret_code; } SmcConfigHomingOrderRes;

/* SMC_CONFIG_HOMING_ORDER_EX (0x0065) — B4 (2026-07-23) v2: order + auto_on_init */
/* 与 0x005C 区别: 加 auto_on_init 字段. 旧 client 继续用 0x005C (auto_on_init 保持现状) */
/* auto_on_init: 0=不自动 (默认), 1=SMC_InitAndStart 末尾自动回零 */
typedef struct {
    char    order_letters[SMC_HOMING_ORDER_MAX_LEN];
    int32_t auto_on_init;   /* 0/1 */
} SmcConfigHomingOrderExReq;
typedef struct { int32_t ret_code; } SmcConfigHomingOrderExRes;

/* SMC_SET_ORIGIN_HERE (0x0066) — v1.5 (2026-08-28): 当前位设为激活 WCS 零点 */
/* main 时代 SetZero 语义: work_offsets[当前WCS] = 当前 G53 坐标 (纯偏置写, 无运动) */
/* 之后回零 (m35) 物理回到该点。G53 模态拒绝 (ret_code=-1) */
typedef struct {
    uint8_t z_letter;   /* 轴字母; '\0' = 全部轴 */
    uint8_t _pad[3];
} SmcSetOriginHereReq;
typedef struct { int32_t ret_code; } SmcSetOriginHereRes;

/* SMC_HOMING_TRIGGER (0x005D): 触发回零 */
typedef struct {
    uint8_t z_letter;   /* 轴字母, '\0' = HomeAll */
    uint8_t _pad[3];
} SmcHomingTriggerReq;
typedef struct { int32_t ret_code; } SmcHomingTriggerRes;

/* SMC_HOMING_CANCEL (0x005E): 无 Req */
typedef struct { int32_t ret_code; } SmcHomingCancelRes;

/* SMC_GET_HOMING (0x005F): 查询回零状态 */
typedef struct {
    int32_t ret_code;
    int32_t state;        /* 0/1/2/3/4 */
    int32_t axis_idx;     /* 当前回零轴 (-1=HomeAll) */
    int32_t enabled;
    int32_t _pad;
    double  progress_pct; /* 0.0-1.0 */
} SmcGetHomingRes;

/* SMC_JOG_START (0x0060) */
typedef struct {
    uint8_t z_letter;
    uint8_t _pad[3];
    int32_t direction;    /* +1 / -1 */
    double  speed_mm_s;
} SmcJogStartReq;
typedef struct { int32_t ret_code; } SmcJogStartRes;

/* SMC_JOG_STOP (0x0061) */
typedef struct {
    uint8_t z_letter;   /* '*' = 全停 */
    uint8_t _pad[3];
} SmcJogStopReq;
typedef struct { int32_t ret_code; } SmcJogStopRes;

/* SMC_EMERGENCY_STOP (0x0062) — P0-A 软急停 */
/* reason_code: 0=UI 手动, 1=外部系统, 2-9 保留 */
/* message: UTF-8 null 终结, 超长截断, 64B */
typedef struct {
    int32_t reason_code;
    char    message[64];
} SmcEmergencyStopReq;
typedef struct {
    int32_t ret_code;   /* 恒 0=已触发 (急停不可拒) */
} SmcEmergencyStopRes;

/* SMC_CONFIG_GANTRY_ALIGN (0x0063) — B2 (2026-07-23) 双驱龙门 pre-align 配置 */
/* 配置时序: SMC_InitAndStart 之前 (与 SMC_CONFIG_HOMING_AXIS 同语义) */
/* 仅对 slave_count==2 双驱轴生效; 触发阈值 tol_pulse=0 表示跳过 pre-align */
typedef struct {
    uint8_t z_letter;          /* 轴字母 */
    uint8_t _pad[3];
    int32_t tol_pulse;         /* 触发阈值脉冲数 (0=跳过; 推荐 pulse_per_mm × 0.05mm) */
    int32_t timeout_ms;        /* 收敛超时 [500, 30000], 默认 3000 */
} SmcConfigGantryAlignReq;
typedef struct {
    int32_t ret_code;          /* 0=ok, -1=未配置/单驱/运行中/参数非法 */
} SmcConfigGantryAlignRes;

/* SMC_INJECT_GANTRY_OFFSET (0x0064) — B2 (2026-07-23) sim 模式 mock 注入 */
/* 仅 g_sim_mode==1 有效; 实机返回 -2 */
/* 测试场景: 注入主从静态差 → 触发 homing → 验证 pre-align 触发与收敛 */
typedef struct {
    uint8_t z_letter;          /* 轴字母 (必须是 slave_count==2 双驱轴) */
    uint8_t _pad[3];
    int32_t offset_pulse;      /* 加到 slave motor pos 的偏移量 (正/负均可) */
} SmcInjectGantryOffsetReq;
typedef struct {
    int32_t ret_code;          /* 0=ok, -1=非双驱/参数越界, -2=非 sim 模式 */
} SmcInjectGantryOffsetRes;

#pragma pack(pop)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SMC_PROTOCOL_H */
