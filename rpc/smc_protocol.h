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

    /* --- G 代码加工 0x0040 ~ 0x004F --- */
    SMC_CMD_RUN_GCODE_FILE           = 0x0040,
    SMC_CMD_PAUSE_PROCESSING         = 0x0041,
    SMC_CMD_RESUME_PROCESSING        = 0x0042,
    SMC_CMD_ABORT_PROCESSING         = 0x0043,
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

#pragma pack(pop)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SMC_PROTOCOL_H */
