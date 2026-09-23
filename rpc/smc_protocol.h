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

/* ---- P1-4 (2026-09-03): 渐进穿孔台阶表 (M76 P<级数> 逐级驱动) ---- */
#define LASER_PIERCE_TABLE_MAX  8       /* 台阶上限 (对齐 movecontrol .ld 三级渐进典型值) */

typedef struct {
    double  power_w;   /* 本级功率 W (>=0) */
    double  freq_hz;   /* 本级频率 Hz (>=0) */
    int32_t dwell_ms;  /* 本级停留 ms (>=1) */
} LaserPierceStep_t;   /* pack(1) 下 20B */

/* ---- P0-2 (2026-09-09): 激光互锁链规则表 (软 PLC 地基, 数据驱动互锁) ----
 * 对标整机厂梯形图互锁链: il_count==0 回退内核默认 6 路链;
 * 1..LASER_IL_MAX 条规则由 safety_gate 每 cycle 评估。
 * mask 约束: 不占用 bit15 (0x8000 保留 system_alarm 联动)。 */
#define LASER_IL_MAX  8        /* 规则条数上限 (RT 1ms 循环评估预算内) */

typedef struct {
    uint8_t  bit;      /* DI 位偏移 (0-15), 该位=1 视为异常 */
    uint8_t  reserved; /* 对齐填充, 必须填 0 */
    uint16_t mask;     /* 异常时 interlock_status 置位码 (bit0-14) */
} LaserIlRule_t;       /* pack(1) 下 4B */

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

    /* --- Laser P0/P1 (2026-09-03): 0x0067-0x0069 --- */
    SMC_CMD_CONFIG_LASER_GAS         = 0x0067,  /* P0-2: 气压 AO 通道 + 量程 (M13 P<bar> 配套) */
    SMC_CMD_CONFIG_LASER_MICRO_JOINT = 0x0068,  /* P1-5: 微连接段功率系数 mj_ratio */
    SMC_CMD_CONFIG_LASER_PIERCE_TABLE = 0x0069, /* P1-4: 渐进穿孔台阶表 (M76 P<级数> 配套) */

    /* --- 断点续切 Phase 1 (2026-09-08): 0x0070 ~ 0x0072 --- */
    SMC_CMD_RESUME_SEARCH            = 0x0070, /* 校验目标行 G00 边界 + SEARCH 重放建立模态 (阻塞至完成) */
    SMC_CMD_RESUME_START             = 0x0071, /* 从冻结边界起跑 (重定位前导 + 激光模态恢复 + 续切) */
    SMC_CMD_GET_RESUME               = 0x0072, /* 查询 resume 状态 (ready/target_line/target_pos) */

    /* --- 软 PLC 地基 P0 (2026-09-09): 0x0073 ~ 0x0074 --- */
    SMC_CMD_INJECT_LASER_DI          = 0x0073, /* sim-only: 注入 16-bit DI 图案 (互锁链 sim 可测) */
    SMC_CMD_CONFIG_LASER_INTERLOCK   = 0x0074, /* 互锁链规则表 (0 条 = 回退默认 6 路链) */

    /* --- P0-4a 参数化骨架 (2026-09-10): 0x0075 --- */
    SMC_CMD_CONFIG_LASER_PDO_LAYOUT  = 0x0075, /* I/O 端子 PDO 布局 (byte 偏移/宽度, 选型适配) */

    /* --- P1 软 PLC (2026-09-11): 0x0076 ~ 0x007A --- */
    SMC_CMD_PLC_DOWNLOAD             = 0x0076, /* 程序下载+编译 (返回诊断行号; 不启动) */
    SMC_CMD_PLC_UPLOAD               = 0x0077, /* 程序文本上传 (P1-4 实装) */
    SMC_CMD_PLC_CTRL                 = 0x0078, /* 启动/停止/复位 (P1-2 实装) */
    SMC_CMD_PLC_GET_STATE            = 0x0079, /* 状态查询 (state/变量数/语句数/文本长) */
    SMC_CMD_PLC_READ_VARS            = 0x007A, /* 变量表快照 (P1-2 实装) */

    /* --- P1-3 (2026-09-12): 0x007B sim-only 测试钩子 --- */
    SMC_CMD_PLC_TEST_HOOK            = 0x007B, /* 冻结/恢复 heartbeat 发布 (看门狗验证) */

    /* --- P1.5-1 (2026-09-14): AI 模拟输入通道 --- */
    SMC_CMD_CONFIG_PERIPH_AI          = 0x007C, /* AI 从站 PDO 布局 + 每通道满量程 */
    SMC_CMD_INJECT_PERIPH_AI          = 0x007D, /* sim-only: 注入传感器工程量 (G_AIx) */

    /* --- P1.5-2 (2026-09-15): Force 强制 --- */
    SMC_CMD_PLC_FORCE_VAR             = 0x007E, /* PLC 用户变量强制 (输出强制语义) */

    /* --- P1.5-3 (2026-09-15): Trace 变量示波器 --- */
    SMC_CMD_PLC_TRACE_CONFIG          = 0x007F, /* 通道/分频/触发配置 */
    SMC_CMD_PLC_TRACE_CTRL            = 0x0080, /* arm / stop 定格 / abort / 查询 */
    SMC_CMD_PLC_TRACE_READ            = 0x0081, /* DONE 后分页导出样本 */

    /* --- G93 jerk-bounded 转移剖面 (论文1 IV-E, 2026-09-16): 0x0082 --- */
    SMC_CMD_CONFIG_G93_JERK           = 0x0082, /* 配置转移模式 (enable/ε_v%/ε_t%; enable=0 退化 IV-C) */

    /* --- 论文1 B0′ 消融 + N22 实时性 (2026-09-17): 0x0083 / 0x0084 --- */
    SMC_CMD_CONFIG_G93_ABLATION       = 0x0083, /* B0′ F-inheritance 消融 (f_inherit; 1=τ=n·T, 仅实验) */
    SMC_CMD_QUERY_ENQUEUE_STATS       = 0x0084, /* enqueue 临界区耗时统计 (count/mean/max μs, 可读后清零) */

    /* --- 论文1 M1 替代设计: 事后修正 (2026-09-18): 0x0085 --- */
    SMC_CMD_CONFIG_G93_POSTHOC        = 0x0085, /* M1 post-hoc 速度赤字 carry-over (1=启用, 0=本文构造) */
    SMC_CMD_CONFIG_G93_RATIO          = 0x0086, /* V2 战役: IV-D 长度比重标定 关闭 (1=Naive T'=T, 仅实验) */
    SMC_CMD_CONFIG_G93_STRICT_OFF     = 0x0087, /* S3 planner-rate-limiting: 1=关闭强一致豁免(B2 下游限速), 0=Ours(默认) */
    SMC_CMD_CONFIG_G93_TIP_BASIS      = 0x0088, /* S10-H1 RTCP 长度基消融: 1=刀尖欧氏长度 L^P (忽略旋转轴折算, 门看不到机械轴真实需求), 0=机械轴等效长度 L^M (Ours 默认) */
    SMC_CMD_QUERY_BLOCK_TIMING        = 0x0089, /* 论文1 S10: 控制器内 block 级计时查询 (op: 0=读, 1=清零后读) */

    /* --- V13.1 (2026-09-22): G93 语义变换事件 (geometry-policy / M1 因果态) --- */
    SMC_CMD_QUERY_G93_SEMANTIC_EVENTS = 0x008A, /* 事件环形缓冲分页查询 (op: 0=读, 1=复位后读) */
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
    double   gas_pressure_bar;  /* P0-2: 辅助气压设定值 bar (M13 P, 0=未设定) */
    uint16_t interlock;         /* 互锁位图: bit0=door, bit1=estop_soft, bit2=laser_alm,
                                   bit3=water_temp, bit4=water_flow, bit5=gas_press,
                                   bit15=system_alarm; 0=正常 */
    int32_t  emergency_kill;    /* 急停锁存 (1=已锁存, 需 alarm_reset 路径调 laser_rt_reset) */
    double   P_base_w;          /* 基准功率 (M67 设定, 不被 P-v 耦合覆盖) */
    double   v_actual_mm_s;     /* 当前周期瞬时速度 mm/s (RT 写, 耦合 update 推进) */
    int32_t  coupling_mode_rt;  /* 当前段耦合模式 (0=off, 1=查表) */
    int32_t  is_piercing;       /* 是否在穿孔中 (派生: is_waiting_mcode && current_mcode∈{64,76}; 64=G04 dwell, 76=渐进) */
    uint8_t  current_seg_flags; /* 当前段工艺标记: bit0=lead_in, bit1=micro_joint */
    int32_t  hold_gate_off;     /* P0-1: feedhold 刹车/暂停中光闸强制关 (0/1, 不锁存 RESUMING 即清) */
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

/* ----- 断点续切 Phase 1 (2026-09-08) ----- */
typedef struct {
    int32_t target_line;   /* 目标边界物理行号 (line_no; 必须是含 G00 运动的行, 即轮廓进近行) */
} SmcResumeSearchReq;
typedef struct {
    int32_t ret_code;      /* 0=ok(边界已冻结, 可 RESUME_START),
                            * -1=parser 忙, -2=未 LoadProgram/文件或 WCS 指纹不符,
                            * -3=行存在但非 G00 边界, -4=SafeLiftZ 未配置 (前导依赖),
                            * -5=行不存在, -6=搜索超时 (5s) */
    int32_t target_line;   /* 回显冻结的行号 */
    double  target_pos[AXIS_NUM];  /* 边界 G00 机械目标位姿 (G53, ret_code=0 时有效) */
} SmcResumeSearchRes;

/* SMC_RESUME_START: 无 Req */
typedef struct {
    int32_t ret_code;      /* 0=ok, -1=resume_ready!=1 (未冻结), -2=parser 忙, -3=指纹不符(已作废冻结) */
} SmcResumeStartRes;

/* SMC_GET_RESUME: 无 Req */
typedef struct {
    int32_t ret_code;      /* 恒 0 */
    int32_t resume_ready; /* 0/1 = g_program_resume_ready */
    int32_t search_result;/* g_program_resume_result (0=pending,1=ok,2=非边界,3=行不存在) */
    int32_t target_line;  /* g_parser_ctrl.resume_target_line */
    double  target_pos[AXIS_NUM]; /* SEARCH 冻结位姿 */
} SmcGetResumeRes;

/* ----- G93 jerk-bounded 转移剖面 (论文1 IV-E, 2026-09-16): 0x0082 ----- */
/* SMC_CONFIG_G93_JERK: pack(1) 下 1+3pad + 2×double = 20B */
typedef struct {
    uint8_t  enable;       /* 1=转移模式生效 (加速边界 exact / 减速边界 debt), 0=完全退化 IV-C */
    uint8_t  _pad[3];
    double   eps_v_pct;    /* exact 模式瞬时超速上限 (% v_cmd, 默认 5.0) */
    double   eps_t_pct;    /* debt 模式时长修正上限 (% T, 默认 2.0) */
} SmcConfigG93JerkReq;
typedef struct { int32_t ret_code; } SmcConfigG93JerkRes;

/* ----- 论文1 B0′ 消融 + N22 实时性 (2026-09-17): 0x0083 / 0x0084 ----- */
/* SMC_CONFIG_G93_ABLATION: pack(1) 下 1+3pad = 4B */
typedef struct {
    uint8_t  f_inherit;    /* 1=B0′ F-inheritance (每微段继承整块进给字, τ=n·T), 0=IV-B 正常分配 */
    uint8_t  _pad[3];
} SmcConfigG93AblationReq;
typedef struct { int32_t ret_code; } SmcConfigG93AblationRes;

/* SMC_CONFIG_G93_POSTHOC (0x0085, 论文1 M1): 复用同构 4B 请求体 */
typedef struct {
    uint8_t  enable;       /* 1=M1 事后修正 (段末测 v_act, 速度赤字 carry-over 到下一段), 0=本文 IV-D 构造 */
    uint8_t  _pad[3];
} SmcConfigG93PosthocReq;
typedef struct { int32_t ret_code; } SmcConfigG93PosthocRes;

/* SMC_CONFIG_G93_RATIO (0x0086, 论文1 V2 仿真战役): 复用同构 4B 请求体 */
typedef struct {
    uint8_t  ratio_off;    /* 1=Naive 基线 (T'=T, 关闭 IV-D 长度比重标定), 0=IV-D 生效 (生产默认) */
    uint8_t  _pad[3];
} SmcConfigG93RatioReq;
typedef struct { int32_t ret_code; } SmcConfigG93RatioRes;

/* SMC_CONFIG_G93_STRICT_OFF (0x0087, 论文1 S3 planner-rate-limiting): 复用同构 4B 请求体 */
typedef struct {
    uint8_t  strict_off;   /* 1=B2 基线 (关闭 G93 强一致豁免, 下游 planner 正常短板限幅), 0=Ours (默认) */
    uint8_t  _pad[3];
} SmcConfigG93StrictOffReq;
typedef struct { int32_t ret_code; } SmcConfigG93StrictOffRes;

/* SMC_CONFIG_G93_TIP_BASIS (0x0088, 论文1 S10-H1 长度基消融): 复用同构 4B 请求体 */
typedef struct {
    uint8_t  tip_basis;    /* 1=-H1 基线 (需求速率按刀尖欧氏长度 L^P 反推, 旋转轴折算不计), 0=IV-A 物理基 L^M (生产默认) */
    uint8_t  _pad[3];
} SmcConfigG93TipBasisReq;
typedef struct { int32_t ret_code; } SmcConfigG93TipBasisRes;

/* ============================================================
 * SMC_QUERY_BLOCK_TIMING (0x0089, 论文1 S10, 2026-09-20)
 *
 * 动机: 此前的时长口径全部来自 CSV trace 行数 (cy[i_end]-cy[i0])。但 sim_engine
 *       落盘受 should_log_this_cycle 闸门影响, 窗口两端都会丢行 ⇒ 恒少 1。
 *       本 RPC 直接读控制器内由 RT 周期计数器 `cycle` 打的 block 起止刻度。
 *
 * 约定: block_end_cycle_exclusive = 末微段完成周期 + 1
 *       block_elapsed_cycles      = end_exclusive - start   (单周期 block 得 1)
 *
 * 三种时间严格分开 (H5 要求):
 *   virtual_elapsed_ms        = T_virtual (语义时间, 被倍率/feedhold 缩放)
 *   controller_elapsed_cycles = T_ctrl 的周期数 (真实流逝, 含 feedhold)
 *   host_runtime_ms           = T_host  —— 不在本结构, 由宿主墙钟测
 *
 * Req pack(1) 下 1+3pad = 4B; Res pack(1) 下 = 196B (V13.1 前 184B, 尾部追加
 *        block_plan_sum_ms + plan_segments_loaded —— 旧宿主脚本需同步更新)
 * Python: struct.unpack("<IiIQQQIIIdddQQQddQdQdQdQQQdI", resp[:196])
 * ============================================================ */
typedef struct {
    uint8_t  op;           /* 0=只读, 1=读前清零 (下一 run 从零累计) */
    uint8_t  _pad[3];
} SmcBlockTimingReq;

typedef struct {
    int32_t  ret_code;                  /* 恒 0 */

    /* ---- block 级 (当前 / 最后一个) ---- */
    uint32_t block_uid;
    int32_t  block_line_no;
    uint64_t block_start_cycle;
    uint64_t block_end_cycle_exclusive;
    uint64_t block_elapsed_cycles;
    uint32_t block_stall_cycles;        /* block 在飞但虚拟时间未推进的周期数 (期望 0) */
    uint32_t segments_generated;        /* parser 声明生成 */
    uint32_t segments_completed;        /* RT 实际跑完 */
    double   block_budget_cmd_ms;       /* 指令 block 时长 (ms) */
    double   block_budget_sum_ms;       /* 实际下发微段预算之和 (ms) */
    double   block_virtual_ms;          /* 本 block 推进的虚拟时间 (ms), 见 inc/block_timing.h */

    /* ---- 程序级 ---- */
    uint64_t ctrl_start_cycle;
    uint64_t ctrl_end_cycle_exclusive;
    uint64_t controller_elapsed_cycles;
    double   controller_elapsed_ms;
    double   virtual_elapsed_ms;
    uint64_t hold_active_cycles;
    double   hold_active_ms;                /* = hold_active_cycles * Ts */
    uint64_t hold_frozen_cycles;            /* 虚拟时间完全不推进的周期 (advance==0) */
    double   override_eff_sum;              /* Σ eff_scale (仅 is_moving 周期) */
    uint64_t override_eff_cycles;           /* 上面求和的周期数 */
    double   virtual_model_ms;              /* Σ(hold ? 0 : Ts*eff_scale) 声明式模型 */
    uint64_t segments_completed_total;
    uint64_t blocks_closed;
    uint64_t rt_cycle_counter;          /* 读数时刻的 RT 全局 cycle */

    /* ---- V13.1 H3 (2026-09-22): planner 最终目标 (T_plan, 独立于 T_sem) ---- */
    double   block_plan_sum_ms;         /* 本 block 实际装载微段的 Σ T_total (ms), RT 装载时累计 */
    uint32_t plan_segments_loaded;      /* 实际装载微段数 (对照 segments_generated) */
} SmcBlockTimingRes;

/* ============================================================
 * SMC_QUERY_G93_SEMANTIC_EVENTS (0x008A, V13.1, 2026-09-22)
 *
 * 职责: G93 语义变换证据 (reference channel ≠ observation channel 的
 *       controller-native 侧)。事件由刀补引擎在**真正计算几何策略**的
 *       位置记录 (cutter_comp.c 三处策略分叉), 记录的是控制器实际产出,
 *       严禁由宿主按 γ·T_decl 反构 observed value。
 *
 * 策略枚举映射 (控制器实际配置开关, 非 V13.1 新增逻辑):
 *   RATE_PRESERVE = IV-D "Ours" 默认:  T' = T·γ, γ = L_act/L_prog
 *   TIME_IDENTITY = Naive (0x0086 ratio_off): T' = T (不变量破坏基线)
 *   POSTHOC       = M1   (0x0085):      T' = L_prog/v_eff, 速度赤字 carry-over
 *
 * M1/S7b 字段: m1_* 仅 policy=POSTHOC 时有意义 (其余策略记 0)。
 *   u_pre = v_eff/v_cmd 是块前因果态, T_decl/T_actual 只能作 execution
 *   diagnostic, 不得冒充 u_b。
 *
 * 缓冲: 控制器内 256 条环形 (SMC_G93_SEM_EVENT_CAP), 单写者 = parser 线程
 *   (刀补引擎), 读者 = RPC 线程。宿主应在 run 结束后查询 (与 0x0084/0x0089
 *   同一使用模式); run 进行中读取可能得到撕裂条目, 由事件 seq 连续性校验。
 *   超出容量的最老事件被驱逐, dropped_events 暴露驱逐数 (channel validity)。
 *
 * 分页: SMC_RESP_BUF_LEN=512B 硬约束, 单事件 pack(1)=100B ⇒ 每页 4 条,
 *   请求携带 start_seq (1-based, 全局递增事件号)。
 *
 * Req pack(1) 下 12B; Res pack(1) 下定长 16+4×100 = 416B
 * ============================================================ */
#define SMC_G93_SEM_EVENT_CAP            256   /* 控制器内环形容量 */
#define SMC_G93_SEM_EVENT_MAX_PER_RSP    4     /* 512B resp_buf 约束下的每页条数 */

typedef enum {
    G93_SEM_POLICY_NONE           = 0,
    G93_SEM_POLICY_RATE_PRESERVE  = 1,  /* IV-D 默认: T' = T·γ */
    G93_SEM_POLICY_TIME_IDENTITY  = 2,  /* Naive (ratio_off): T' = T */
    G93_SEM_POLICY_POSTHOC        = 3   /* M1 (posthoc): T' = L_prog/v_eff */
} G93SemPolicy_t;

typedef struct {
    uint32_t seq;             /* 全局递增事件号 (1-based, 复位后重新从 1 起) */
    int32_t  source_line;     /* G 代码源行号 (记录时刻 parser 的 g_current_line_no) */
    double   T_in_ms;         /* 输入时间预算 T_in (ms) */
    double   L_prog_mm;       /* 编程轮廓长度 (记录点控制器实际使用的量纲) */
    double   L_act_mm;        /* 实际执行长度 */
    double   gamma;           /* = L_act/L_prog (控制器记录点真实计算值) */
    int32_t  policy_id;       /* G93SemPolicy_t */
    double   T_policy_ms;     /* 控制器 native 策略输出 (实际下发的预算, ms) */
    double   m1_debt_before;  /* S7b: 本事件修正前速度赤字 (mm/s) */
    double   m1_v_cmd;        /* S7b: v_cmd = L_prog/T_in (mm/s) */
    double   m1_v_eff;        /* S7b: v_eff = v_cmd + debt_before */
    double   m1_u_pre;        /* S7b: u_pre = v_eff/v_cmd (正式 S7b 的 u_b) */
    double   m1_debt_after;   /* 本事件修正后速度赤字 */
} SmcG93SemEvent;             /* pack(1) 下 = 100B */

typedef struct {
    uint8_t  op;              /* 0=读取, 1=复位计数器 (响应体为复位后状态) */
    uint8_t  _pad[3];
    uint32_t start_seq;       /* 请求起始事件 seq (1-based; 早于最老可用则服务端钳到最老) */
    uint32_t max_count;       /* 请求条数 (服务端钳到 SMC_G93_SEM_EVENT_MAX_PER_RSP) */
} SmcG93SemEventsReq;         /* pack(1) 下 = 12B */

typedef struct {
    int32_t  ret_code;        /* 恒 0 */
    uint32_t total_count;     /* 累计记录事件数 (monotonic, 复位清零) */
    uint32_t dropped_events;  /* 环形驱逐数 = (total > CAP) ? total - CAP : 0 */
    uint32_t returned_count;  /* 本页实际拷贝条数 (events[] 前 returned_count 个有效) */
    SmcG93SemEvent events[SMC_G93_SEM_EVENT_MAX_PER_RSP];
} SmcG93SemEventsRes;         /* pack(1) 下 = 16 + 400 = 416B */


/* SMC_QUERY_ENQUEUE_STATS: Req pack(1) 下 1+3pad = 4B;
 * Res pack(1) 下 4+4pad+2×u64+4×double = 56B */
typedef struct {
    uint8_t  reset;        /* 1=读后清零 (下一 run 从零累计) */
    uint8_t  _pad[3];
} SmcEnqueueStatsReq;
typedef struct {
    int32_t  ret_code;      /* 恒 0 */
    uint32_t _pad;
    uint64_t count_all;     /* 全部入队次数 (G93+G94) */
    uint64_t count_strict;  /* G93 严格入队次数 (含 IV-C/IV-E 剖面预计算) */
    double   mean_us_all;   /* 全部入队平均耗时 (μs/次) */
    double   mean_us_strict;/* 严格入队平均耗时 (μs/次) — 论文 V-C 头条 */
    double   max_us_strict; /* 严格入队最大耗时 (μs/次) */
    double   reserved;      /* 预留对齐 (0) */
} SmcEnqueueStatsRes;

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

/* SMC_CONFIG_LASER_GAS (0x0067, P0-2 2026-09-03): 气压 AO 通道 + 量程
 * 气压 AO 不受 can_emit 门 (关光后吹气继续), 配套 M13 P<bar>。
 * pack(1) 下 1 + 7(pad) + 8 = 16B */
typedef struct {
    uint8_t ch_ao_pressure;    /* 气压通道号 (0-3) */
    uint8_t _pad[7];           /* 8B 对齐 (后随 double) */
    double  pressure_max_bar;  /* 量程 bar, >0 生效 */
} SmcConfigLaserGasReq;
typedef struct { int32_t ret_code; } SmcConfigLaserGasRes;

/* SMC_CONFIG_LASER_MICRO_JOINT (0x0068, P1-5 2026-09-03): 微连接段功率系数
 * SEG_FLAG_MICRO_JOINT 置位段 coupling ratio × mj_ratio; 配套 M77 P<ratio> */
typedef struct {
    double mj_ratio;   /* (0, 1], 默认 1.0 = 不衰减 */
} SmcConfigLaserMicroJointReq;
typedef struct { int32_t ret_code; } SmcConfigLaserMicroJointRes;

/* SMC_CONFIG_LASER_PIERCE_TABLE (0x0069, P1-4 2026-09-03): 渐进穿孔台阶表
 * count: 1..LASER_PIERCE_TABLE_MAX; 配套 M76 P<级数> (级数 <= count)。
 * pack(1) 下 4 + 8×20 = 164B */
typedef struct {
    int32_t           count;
    LaserPierceStep_t steps[LASER_PIERCE_TABLE_MAX];
} SmcConfigLaserPierceTableReq;
typedef struct { int32_t ret_code; } SmcConfigLaserPierceTableRes;

/* SMC_INJECT_LASER_DI (0x0073, P0-3a 2026-09-09): sim-only DI 注入
 * 使能后 laser_hw_read_di 返回 pattern, safety_gate 互锁链在 WSL2 sim 可测。
 * pack(1) 下 4B */
typedef struct {
    uint16_t pattern;   /* 注入 DI 图案 (bit=1 该位异常), 如 0x0008=水温 */
    uint16_t enable;    /* 1=使能注入, 0=清除 (回恒 0 无异常) */
} SmcInjectLaserDiReq;
typedef struct {
    int32_t ret_code;   /* 0=ok, -2=非 sim 模式 */
} SmcInjectLaserDiRes;

/* SMC_CONFIG_LASER_INTERLOCK (0x0074, P0-2 2026-09-09): 互锁链规则表
 * count: 0..LASER_IL_MAX (0 = 回退默认 6 路链, 等价不配置)
 * 校验: bit<=15, mask!=0 且不含 0x8000, reserved==0
 * pack(1) 下 4 + 8×4 = 36B */
typedef struct {
    int32_t       count;
    LaserIlRule_t rules[LASER_IL_MAX];
} SmcConfigLaserInterlockReq;
typedef struct { int32_t ret_code; } SmcConfigLaserInterlockRes;

/* SMC_CONFIG_LASER_PDO_LAYOUT (0x0075, P0-4a 2026-09-10): I/O 端子 PDO 布局
 * 选型适配参数 — 换端子型号 = 改这 6 个数 + slave_id, 内核代码不动。
 * 校验: *_bytes ∈ {1,2}, ao_stride ∈ {2,4}, *_byte_off <= 63
 * 默认 (EL 级 16-bit 端子): 全 0 偏移 + di/do 2 字节 + stride 2 */
typedef struct {
    uint8_t di_byte_off;    /* DI PDO 起始字节 */
    uint8_t di_bytes;       /* DI 宽度 1/2 (8 点端子配 1, 防 IOmap 越界读邻站) */
    uint8_t do_byte_off;    /* DO PDO 起始字节 */
    uint8_t do_bytes;       /* DO 宽度 1/2 */
    uint8_t ao_byte_off;    /* AO 通道 0 起始字节 */
    uint8_t ao_stride;      /* AO 每通道字节 2/4 */
    uint8_t reserved[2];    /* 必须 0 */
} SmcConfigLaserPdoLayoutReq;   /* pack(1) 下 8B */
typedef struct { int32_t ret_code; } SmcConfigLaserPdoLayoutRes;

/* ============================================================
 * P1 软 PLC (2026-09-11): 0x0076 ~ 0x007A
 * ============================================================ */
#define SMC_PLC_TEXT_MAX      4080   /* 程序文本上限 (8B 头 + 4080 ≤ 4096 payload 限额;
                                      * 与 plc_core.h PLC_PROG_TEXT_MAX 同步改) */
#define SMC_PLC_DIAG_MSG_LEN  64
#define SMC_PLC_DIAG_IN_RESP  5      /* 响应内诊断条数上限 (352B < 512 响应缓冲) */

/* SMC_PLC_DOWNLOAD (0x0076): Req = 8B 头 + 变长文本 (data_len = 8 + text_len)
 * 文本不要求 NUL 终结; CRC-32 (IEEE, 与 Python zlib.crc32 一致)。 */
typedef struct {
    int32_t  text_len;      /* 1..SMC_PLC_TEXT_MAX */
    uint32_t crc32;
} SmcPlcDownloadReqHead;    /* 之后紧跟 char text[text_len] */

/* 编译诊断 (布局与 plc_lang.h PlcDiag_t 一致, 二进制兼容直接 memcpy) */
typedef struct {
    int32_t line;
    char    msg[SMC_PLC_DIAG_MSG_LEN];
} SmcPlcDiag_t;             /* pack(1) 下 68B */

typedef struct {
    int32_t ret_code;       /* 0=编译通过(未启动) -1=编译错误(见 diags)
                             * -2=超长/参数非法 -3=CRC 不符 -4=运行中拒绝 */
    int32_t n_diag_total;   /* 诊断总数 (可能 > included) */
    int32_t n_diag_included;
    SmcPlcDiag_t diags[SMC_PLC_DIAG_IN_RESP];
} SmcPlcDownloadRes;        /* pack(1) 下 12 + 5×68 = 352B */

/* SMC_PLC_CTRL (0x0078, P1-2): action 0=stop 1=start 2=reset */
typedef struct { int32_t action; } SmcPlcCtrlReq;
typedef struct { int32_t ret_code; } SmcPlcCtrlRes;   /* 0=ok, -1=状态不允许 */

/* SMC_PLC_GET_STATE (0x0079, P1-2 扩展): 前 16B 与 P1-1 兼容 (追加字段) */
typedef struct {
    int32_t  state;         /* PlcRunState: 0=EMPTY 1=STOPPED 2=RUNNING 3=ERROR */
    int32_t  n_vars;        /* 用户变量数 */
    int32_t  n_stmts;       /* 顶层语句数 */
    int32_t  text_len;
    uint32_t heartbeat;     /* 已完成扫描周期数 */
    uint32_t cycle_us;      /* 最近周期执行耗时 (µs) */
    int32_t  err_line;      /* 运行期错误行 (0=无) */
    char     err_msg[64];
} SmcPlcGetStateRes;        /* pack(1) 下 92B */

/* SMC_PLC_READ_VARS (0x007A, P1-2): 变量快照分页 (用户变量 + 3 个 F 系统变量) */
#define SMC_PLC_VARS_PER_PAGE  10
typedef struct { int32_t offset; } SmcPlcReadVarsReq;  /* 起始条目 (0 起) */
typedef struct {
    char     name[32];
    uint8_t  type;          /* PlcType */
    uint8_t  cls;           /* 0=user 2=sys_F */
    uint8_t  fb;            /* PlcFbKind */
    uint8_t  forced;        /* P1.5-2: 1 = 该变量当前被强制 (原 reserved 位升级) */
    double   value;         /* BOOL 0/1, 数值统一 double */
} SmcPlcVarSnap_t;          /* pack(1) 下 44B */
typedef struct {
    int32_t ret_code;
    int32_t n_total;
    int32_t n_included;
    int32_t next_offset;    /* -1=末页 */
    SmcPlcVarSnap_t vars[SMC_PLC_VARS_PER_PAGE];
} SmcPlcReadVarsRes;        /* pack(1) 下 16 + 440 = 456B */

/* SMC_PLC_TEST_HOOK (0x007B, P1-3 2026-09-12): sim-only 看门狗验证钩子
 * freeze_hb=1: 扫描线程继续跑但停止递增/发布 heartbeat (模拟调度饥饿) */
typedef struct { int32_t freeze_hb; } SmcPlcTestHookReq;
typedef struct { int32_t ret_code; } SmcPlcTestHookRes;   /* 0=ok, -2=非 sim */

/* SMC_PLC_FORCE_VAR (0x007E, P1.5-2 2026-09-15): PLC 用户变量强制
 * 输出强制语义: 程序写被周期末覆写, watch/F 区呈现强制值。
 * name 空 + on=0 = 清除全部强制; 仅 RUNNING 有效 (-3), 重启自动清零。
 * pack(1) 下 48B */
typedef struct {
    char     name[32];      /* 变量名 (大小写不敏感) */
    double   val;           /* 强制值 (按声明类型转换/钳制) */
    int32_t  on;            /* 1=强制 0=解除 */
    int32_t  reserved;      /* 必须 0 */
} SmcPlcForceVarReq;
typedef struct { int32_t ret_code; } SmcPlcForceVarRes;
/* ret: 0=ok, -1=变量不存在, -3=非 RUNNING */

/* ============================================================
 * P1.5-3 (2026-09-15): Trace 变量示波器 0x007F ~ 0x0081
 * 采样: PLC 扫描线程每 div 周期一点 (10ms×div), 1000 点环形,
 * pre 500 / post 500; 通道可为任意 PLC 变量 (含 G_/F_ 系统变量)。
 * ============================================================ */
#define SMC_PLC_TRACE_CH_MAX  8     /* 与 plc_trace.h PLC_TRACE_CH_MAX 同步 */

/* SMC_PLC_TRACE_CONFIG (0x007F): 通道/分频/触发配置 (须在 arm 前)
 * pack(1) 下 16+8+256 = 280B */
typedef struct {
    int32_t  n_ch;              /* 1-8 */
    int32_t  div;               /* 采样分频 1-100 (1=10ms/点) */
    int32_t  trig_kind;         /* 0=手动 1=>= 2=<= 3=上穿 4=下穿 */
    int32_t  trig_ch;           /* 触发通道 0-7 */
    double   trig_threshold;
    char     names[SMC_PLC_TRACE_CH_MAX][32];  /* 前 n_ch 有效, 变量名 */
} SmcPlcTraceConfigReq;
typedef struct { int32_t ret_code; } SmcPlcTraceConfigRes;
/* ret: 0=ok, -1=参数越界(含空名), -2=录制中拒改 */

/* SMC_PLC_TRACE_CTRL (0x0080): 1=arm 0=stop 定格 2=abort 清数据 3=仅查询 */
typedef struct { int32_t action; } SmcPlcTraceCtrlReq;
typedef struct {
    int32_t ret_code;       /* arm: 0/-1 通道名不可解析/-3 非RUNNING;
                             * stop: 0/-1 非录制态; abort/查询: 0 */
    int32_t state;          /* 0=IDLE 1=ARMED 2=TRIGGED 3=DONE */
    int32_t n_samples;      /* 可导出样本数 = min(累计, 1000) */
} SmcPlcTraceCtrlRes;       /* 12B */

/* SMC_PLC_TRACE_READ (0x0081): DONE 态分页导出 (时间升序) */
#define SMC_PLC_TRACE_PER_PAGE  6
typedef struct {
    uint32_t seq;           /* arm 起递增 */
    uint32_t t_ms;          /* heartbeat×10 */
    double   v[SMC_PLC_TRACE_CH_MAX];
} SmcPlcTraceSample_t;      /* pack(1) 下 72B */
typedef struct { int32_t offset; } SmcPlcTraceReadReq;
typedef struct {
    int32_t ret_code;       /* 0=ok, -2=offset 越界, -3=须 DONE 态 */
    int32_t n_total;
    int32_t n_included;
    int32_t next_offset;    /* -1=末页 */
    SmcPlcTraceSample_t samples[SMC_PLC_TRACE_PER_PAGE];
} SmcPlcTraceReadRes;       /* pack(1) 下 16 + 432 = 448B */

/* SMC_CONFIG_PERIPH_AI (0x007C, P1.5-1 2026-09-14): AI 通道配置
 * 数据链: RT periph_ai_rt_poll 读 PDO raw → 0..fs[ch] 工程量 → G 区 ai[4]
 *         → PLC 系统变量 G_AI0..G_AI3 (REAL)。fs<=0 = 该通道未配 (恒 0)。
 * pack(1) 下 8 + 32 = 40B */
#define PERIPH_AI_PROTO_MAX  4    /* 与 periph_ai.h PERIPH_AI_CH_MAX 同步 */
typedef struct {
    int32_t  slave_id;            /* AI 从站 (-1 = 未配置) */
    uint8_t  byte_off;            /* 通道 0 起始字节 (0-63) */
    uint8_t  stride;              /* 每通道字节 2/4 */
    uint8_t  count;               /* 通道数 0-4 */
    uint8_t  reserved;            /* 必须 0 */
    double   fs[PERIPH_AI_PROTO_MAX];   /* 每通道满量程工程值 */
} SmcConfigPeriphAiReq;
typedef struct { int32_t ret_code; } SmcConfigPeriphAiRes;   /* 0=ok, -1=越界 */

/* SMC_INJECT_PERIPH_AI (0x007D, P1.5-1): sim-only 传感器值注入
 * pack(1) 下 8 + 8 = 16B */
typedef struct {
    uint8_t  ch;                  /* 0-3 */
    uint8_t  reserved[7];         /* 0 */
    double   val;                 /* 工程量 (G_AIx 呈现值) */
} SmcInjectPeriphAiReq;
typedef struct { int32_t ret_code; } SmcInjectPeriphAiRes;   /* 0=ok, -1=ch 越界, -2=非 sim */

/* SMC_PLC_UPLOAD (0x0077, P1-4 2026-09-12): 程序文本分页上传
 * (文本 ≤4080B > 512B 响应缓冲, 按 0x007A 同模式分页) */
#define SMC_PLC_UPLOAD_CHUNK  480
typedef struct { int32_t offset; } SmcPlcUploadReq;
typedef struct {
    int32_t ret_code;       /* 0=ok, -1=槽内无文本, -2=offset 越界 */
    int32_t total_len;
    int32_t n_included;     /* 本次 chunk 字节数 */
    int32_t next_offset;    /* -1=末页 */
    char    chunk[SMC_PLC_UPLOAD_CHUNK];
} SmcPlcUploadRes;          /* 16 + n_included (≤496B) */

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
