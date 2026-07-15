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

#pragma pack(pop)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SMC_PROTOCOL_H */
