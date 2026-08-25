/* =====================================================================
 *  opcua_server.c  ——  OPC UA Server (契约 P4: 对外标准通道)
 *
 *  实现 movecontrol/doc/CNC_OPCUA_地址空间契约.md 定义的地址空间:
 *    ns=1;s=CNC.Machine.Status / CNC.Program.* / CNC.Alarm.* /
 *         CNC.ActualSpeed / CNC.{Feed,Rapid,Spindle}Override /
 *         CNC.Axis.<axis>.<attr> (轴对象 Browse 动态发现)
 *  方法: CNC.Program.Load/Start/Pause/Resume/Stop, CNC.System.Reset,
 *        CNC.Jog.Move — 回调直接调既有 SMC_* API (零运动逻辑重写)。
 *
 *  设计要点:
 *    - 单后台线程 (模仿 rpc_event_server 先例), UA_Server_run_iterate 驱动
 *    - 变量节点全部 DataSource 回调: 读现拉 SnapshotHub (seqlock) /
 *      g_axis / g_parser_ctrl / EventLogger, 无缓存一致性问题
 *    - 写回调仅 3 个 override 节点, 钳制后调 SMC_SetOverride (-1=不改语义)
 *    - 契约单位换算: ActualSpeed mm/s→m/s, Override pct→0~1.5 比率
 *    - SecurityPolicy None + 匿名 (与 movecontrol P2 客户端现状匹配,
 *      生产环境再启用 UA TLS, 见契约 §8)
 *
 *  @Context: 全模块 Non-RealTime Background Thread (独立 pthread)
 *  @Safe: malloc/阻塞 I/O/printf 均允许; 不触碰 RT 路径
 * ===================================================================== */

#include "opcua_server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <pthread.h>
#include <stdatomic.h>
#include <unistd.h>
#include <time.h>
#include <ctype.h>
#include <sys/stat.h>

#include "open62541.h"

#include "global_def.h"        /* g_axis/g_axis_map/g_parser_ctrl/g_current_line_no/... */
#include "snapshot_hub.h"      /* SnapshotHub_ReadLatest */
#include "event_logger.h"      /* EventLogger_ReadSince / GetWriteSeq */
#include "smc_api.h"           /* SMC_* API */
#include "axis_cfg.h"          /* AXIS_NUM / FeedHoldState_t */
#include "smc_protocol.h"      /* SMC_MODE_SINGLE_BLOCK */
#include "gcode_parser.h"      /* ParserControl_t */

/* ---- 配置常量 ---- */
#define OPCUA_PORT              4840
#define OPCUA_NAMESPACE_INDEX   1
#define OPCUA_ALARM_MAX         8      /* Alarm.List 最多条数 (契约未定上限, 8 条够 HMI 显示) */
#define OPCUA_ALARM_SCAN_WIN    256    /* 报警扫描窗口 (最近 N 条事件) */
#define OPCUA_RESET_WAIT_MS     1000   /* System.Reset 等 parser 退出的上限 */

/* 程序根目录 (Syntec 生态同款: 共享文件系统 + 路径触发加载):
 * 上位把 .nc 写入共享目录 (SMB), Load 传 Windows 形式路径/裸文件名时
 * 取 basename 在此目录下回退查找。环境变量 SMC_PROGRAM_DIR 可覆盖。 */
#define CNC_PROGRAM_DIR_DEFAULT "/home/meiqin/nc"

/* ---- 契约机器状态枚举 (契约 §1) ---- */
#define CNC_ST_IDLE    0
#define CNC_ST_RUN     1
#define CNC_ST_PAUSE   2
#define CNC_ST_ALARM   3
#define CNC_ST_RESET   4
#define CNC_ST_HOMING  5

/* ---- 契约操作模式枚举 (契约 §1) ---- */
#define CNC_MODE_AUTO  0
#define CNC_MODE_STEP  1

/* =====================================================================
 *  节点上下文 (静态池, 注册期分配, 运行期只读)
 * ===================================================================== */

typedef enum {
    OV_MACHINE_STATUS = 0,
    OV_MACHINE_MODE,
    OV_MACHINE_POWERON,
    OV_MACHINE_DATETIME,
    OV_PROGRAM_NAME,
    OV_PROGRAM_MAINNAME,
    OV_PROGRAM_CURRENTLINE,
    OV_PROGRAM_LINECOUNT,
    OV_ALARM_COUNT,
    OV_ALARM_LIST,
    OV_ACTUAL_SPEED,
    OV_FEED_OVERRIDE,
    OV_RAPID_OVERRIDE,
    OV_SPINDLE_OVERRIDE,
    OV_AXIS_ACTUAL_POS,
    OV_AXIS_CMD_POS,
    OV_AXIS_MECH_POS,
    OV_AXIS_PROG_POS,
    OV_AXIS_REMAIN_DIST,
    OV_AXIS_ACTIVE,
    OV_HOME_STATE,
    OV_HOME_PROGRESS,
    OV_KIND_COUNT
} OpcVarKind;

typedef enum {
    OM_PROGRAM_LOAD = 0,
    OM_PROGRAM_START,
    OM_PROGRAM_PAUSE,
    OM_PROGRAM_RESUME,
    OM_PROGRAM_STOP,
    OM_SYSTEM_RESET,
    OM_SYSTEM_ESTOP,
    OM_HOME_ALL,
    OM_HOME_AXIS,
    OM_JOG_MOVE,
    OM_JOG_MOVE_INC
} OpcMethodKind;

typedef struct {
    uint8_t kind;      /* OpcVarKind / OpcMethodKind */
    int8_t  axis_idx;  /* 轴节点: g_axis 索引; 非轴节点 -1 */
} OpcNodeCtx;

/* 静态上下文池: 注册期线性分配 (当前方法 11 个 / 变量 ~44 个, 留余量) */
static OpcNodeCtx g_var_ctx_pool[96];
static int        g_var_ctx_n;
static OpcNodeCtx g_mtd_ctx_pool[16];
static int        g_mtd_ctx_n;

static OpcNodeCtx *var_ctx_new(uint8_t kind, int axis_idx)
{
    if ((size_t)g_var_ctx_n >= sizeof(g_var_ctx_pool) / sizeof(g_var_ctx_pool[0]))
        return NULL;
    OpcNodeCtx *c = &g_var_ctx_pool[g_var_ctx_n++];
    c->kind = kind;
    c->axis_idx = (int8_t)axis_idx;
    return c;
}

static OpcNodeCtx *mtd_ctx_new(uint8_t kind)
{
    if ((size_t)g_mtd_ctx_n >= sizeof(g_mtd_ctx_pool) / sizeof(g_mtd_ctx_pool[0]))
        return NULL;
    OpcNodeCtx *c = &g_mtd_ctx_pool[g_mtd_ctx_n++];
    c->kind = kind;
    c->axis_idx = -1;
    return c;
}

/* =====================================================================
 *  线程生命周期
 * ===================================================================== */

static _Atomic int        g_opcua_running;
static pthread_t          g_opcua_tid;
static UA_Server         *g_opcua_server;

/* =====================================================================
 *  业务辅助 (全部 Non-RT 安全读路径)
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread
 * @Safe: SnapshotHub seqlock 读 + SMC 查询 API, 可阻塞。
 * 状态派生优先级: ALARM > HOMING > PAUSE > RUN > IDLE (契约枚举) */
static int32_t derive_machine_status(void)
{
    SMC_Snapshot_t snap = {0};   /* 兜底清零: 1000 次重试全落在写窗口时防 UB */
    SnapshotHub_ReadLatest(&snap);   /* -1 时 *out 亦是最后一次拷贝, best-effort */

    if (snap.sys_alarm_state == 1)
        return CNC_ST_ALARM;

    int hstate = 0, haxis = 0;
    double hprog = 0.0;
    if (SMC_GetHomingState(&hstate, &haxis, &hprog) == 0 && (hstate == 1 || hstate == 2))
        return CNC_ST_HOMING;   /* PENDING / RUNNING */

    if (snap.parser_is_paused || snap.hold_state == HOLD_PAUSED)
        return CNC_ST_PAUSE;

    if (snap.parser_is_running || snap.is_moving)
        return CNC_ST_RUN;

    return CNC_ST_IDLE;
}

/* basename("路径") — 同时容忍 '/' 与 '\\' (Windows 客户端传入的路径) */
static const char *path_basename(const char *path)
{
    const char *p = strrchr(path, '/');
    const char *q = strrchr(path, '\\');
    if (q && (!p || q > p)) p = q;
    return p ? p + 1 : path;
}

/* Windows 盘符路径 → WSL 路径宽容转换 (仅当以 "X:" 开头才转换)
 * "D:\\a\\b.nc" / "D:/a/b.nc" → "/mnt/d/a/b.nc"; 已是 "/" 开头原样拷贝 */
static void path_to_wsl(const char *in, char *out, size_t outsz)
{
    if (in[0] == '\0') { out[0] = '\0'; return; }

    if (isalpha((unsigned char)in[0]) && in[1] == ':') {
        size_t o = 0;
        o += (size_t)snprintf(out, outsz, "/mnt/%c", (char)tolower((unsigned char)in[0]));
        for (size_t i = 2; in[i] != '\0' && o + 1 < outsz; i++) {
            char c = (in[i] == '\\') ? '/' : in[i];
            out[o++] = c;
        }
        out[o] = '\0';
        return;
    }
    snprintf(out, outsz, "%s", in);
}

/* 程序路径解析 (Syntec 生态对等: 共享文件系统 + 路径触发):
 *   1. 输入先经 path_to_wsl 规范化 (盘符 → /mnt/…)
 *   2. Linux 绝对路径且文件存在 → 原样
 *   3. 否则取 basename → 程序根目录 (SMC_PROGRAM_DIR 或默认) 下查找
 *      (上位经 SMB 共享写入的文件按文件名命中, 传 Windows 路径/UNC/
 *       裸文件名均可)
 *   4. 都未命中 → 返回规范化路径 (SMC_LoadProgram 报错, 路径明确) */
static void resolve_program_path(const char *in, char *out, size_t outsz)
{
    static const char *prog_dir;      /* 首次调用缓存 */
    if (!prog_dir) {
        const char *env = getenv("SMC_PROGRAM_DIR");
        prog_dir = (env && env[0]) ? env : CNC_PROGRAM_DIR_DEFAULT;
    }

    char norm[512];
    path_to_wsl(in, norm, sizeof(norm));

    struct stat st;
    if (norm[0] == '/' && stat(norm, &st) == 0) {
        snprintf(out, outsz, "%s", norm);
        return;
    }

    const char *base = path_basename(norm);
    if (base[0] != '\0' && base[0] != '/') {
        char cand[512];
        snprintf(cand, sizeof(cand), "%s/%s", prog_dir, base);
        if (stat(cand, &st) == 0) {
            snprintf(out, outsz, "%s", cand);
            return;
        }
    }

    snprintf(out, outsz, "%s", norm);
}

/* Alarm.List 缓存: EventLogger 无新事件时跳过重扫 (读回调可能 10ms 级采样) */
static char     g_alarm_cache[OPCUA_ALARM_MAX][96];
static int      g_alarm_cache_n;
static uint64_t g_alarm_cache_seq;

/* @Context: Non-RealTime Background Thread
 * @Safe: EventLogger 多读者安全。扫最近 OPCUA_ALARM_SCAN_WIN 条事件,
 *        收集 severity>=ALARM 的最后 OPCUA_ALARM_MAX 条, 格式
 *        "code|severity|source|text" (契约 §3)。
 * @return 缓存内报警条数 (0=无报警) */
static int build_alarm_list(void)
{
    uint64_t wseq = EventLogger_GetWriteSeq();
    if (wseq == g_alarm_cache_seq)
        return g_alarm_cache_n;

    g_alarm_cache_n = 0;
    g_alarm_cache_seq = wseq;

    uint64_t from = (wseq > OPCUA_ALARM_SCAN_WIN) ? (wseq - OPCUA_ALARM_SCAN_WIN) : 0;
    SmcEvent_t ev[EVENT_READ_MAX];
    while (from < wseq) {
        uint64_t next = 0;
        int got = EventLogger_ReadSince(from, ev, EVENT_READ_MAX, &next);
        if (got <= 0) break;   /* 覆盖/异常, 停止扫描 */
        for (int i = 0; i < got; i++) {
            if (ev[i].severity < SEVERITY_ALARM) continue;
            if (g_alarm_cache_n == OPCUA_ALARM_MAX) {
                /* 滚动: 丢最旧, 留最新 */
                memmove(g_alarm_cache[0], g_alarm_cache[1],
                        sizeof(g_alarm_cache[0]) * (OPCUA_ALARM_MAX - 1));
                g_alarm_cache_n--;
            }
            snprintf(g_alarm_cache[g_alarm_cache_n], sizeof(g_alarm_cache[0]),
                     "%u|%u|%u|%s",
                     (unsigned)ev[i].code, (unsigned)ev[i].severity,
                     (unsigned)ev[i].source, ev[i].message);
            g_alarm_cache_n++;
        }
        if (next <= from) break;
        from = next;
    }
    return g_alarm_cache_n;
}

/* ISO8601 UTC 时间串 */
static void format_datetime(char *out, size_t outsz)
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    gmtime_r(&ts.tv_sec, &tm);
    snprintf(out, outsz, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/* 开机时长 "12h30m" (契约 §1) */
static void format_poweron(char *out, size_t outsz, const SMC_Snapshot_t *snap)
{
    uint64_t total_min = (uint64_t)(snap->uptime_ms / 60000.0);
    snprintf(out, outsz, "%lluh%llum",
             (unsigned long long)(total_min / 60),
             (unsigned long long)(total_min % 60));
}

/* =====================================================================
 *  DataSource 读/写回调
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread (server 线程 Read/采样)
 * @Safe: SnapshotHub seqlock 读 + 静态缓存读, 无阻塞无分配语义
 *        (setScalarCopy/setArrayCopy 内部分配由 server 统一释放)。
 * 错误约定 (v1.5.3): 状态码写入 value->status, 返回值仅用于日志。 */
static UA_StatusCode
opcua_ds_read(UA_Server *server, const UA_NodeId *sessionId, void *sessionContext,
              const UA_NodeId *nodeId, void *nodeContext,
              UA_Boolean includeSourceTimeStamp, const UA_NumericRange *range,
              UA_DataValue *value)
{
    (void)server; (void)sessionId; (void)sessionContext; (void)nodeId; (void)range;
    OpcNodeCtx *ctx = (OpcNodeCtx *)nodeContext;
    if (!ctx) {
        value->hasStatus = true;
        value->status = UA_STATUSCODE_BADINTERNALERROR;
        return UA_STATUSCODE_GOOD;
    }

    /* 大多数节点要快照; 一次性取一份 (seqlock 保证一致性) */
    SMC_Snapshot_t snap = {0};
    SnapshotHub_ReadLatest(&snap);

    UA_StatusCode sc = UA_STATUSCODE_GOOD;
    char buf[160];

    switch ((OpcVarKind)ctx->kind) {

    case OV_MACHINE_STATUS: {
        UA_Int32 v = (UA_Int32)derive_machine_status();
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }
    case OV_MACHINE_MODE: {
        /* 契约 §1: 0=AUTO 1=STEP...; 映射 single_block 位 → STEP */
        UA_Int32 v = (snap.mode_flags & SMC_MODE_SINGLE_BLOCK) ? CNC_MODE_STEP : CNC_MODE_AUTO;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }
    case OV_MACHINE_POWERON: {
        format_poweron(buf, sizeof(buf), &snap);
        UA_String s = UA_STRING(buf);
        sc = UA_Variant_setScalarCopy(&value->value, &s, &UA_TYPES[UA_TYPES_STRING]);
        break;
    }
    case OV_MACHINE_DATETIME: {
        format_datetime(buf, sizeof(buf));
        UA_String s = UA_STRING(buf);
        sc = UA_Variant_setScalarCopy(&value->value, &s, &UA_TYPES[UA_TYPES_STRING]);
        break;
    }

    case OV_PROGRAM_NAME:
    case OV_PROGRAM_MAINNAME: {
        /* 加载/运行中显示程序文件名, 空闲时空串 (契约未定义空态哨兵) */
        const char *base = path_basename(g_parser_ctrl.filepath);
        UA_String s = UA_STRING((char *)base);
        sc = UA_Variant_setScalarCopy(&value->value, &s, &UA_TYPES[UA_TYPES_STRING]);
        break;
    }
    case OV_PROGRAM_CURRENTLINE: {
        UA_Int32 v = g_current_line_no;   /* parser 写, 1-based */
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }
    case OV_PROGRAM_LINECOUNT: {
        UA_Int32 v = g_program_total_lines;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }

    case OV_ALARM_COUNT: {
        UA_Int32 v = (UA_Int32)build_alarm_list();
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }
    case OV_ALARM_LIST: {
        int n = build_alarm_list();
        if (n > 0) {
            UA_String arr[OPCUA_ALARM_MAX];
            for (int i = 0; i < n; i++)
                arr[i] = UA_STRING(g_alarm_cache[i]);
            /* 深拷贝, 栈数组指向静态缓存无需释放 */
            sc = UA_Variant_setArrayCopy(&value->value, arr, (size_t)n,
                                         &UA_TYPES[UA_TYPES_STRING]);
        } else {
            /* 无报警: 带 type 的 0 长度 String 数组 (哨兵, clear 时掩码安全)。
             * 不能返回全空 variant — open62541 检查 "空值只允许 BaseDataType"
             * (dataType=String 会 BadInternalError)。valueRank=-2 (Any) 下
             * 哨兵数组通过 typecheck (实证 mini 复现)。 */
            UA_Variant_setArray(&value->value, UA_EMPTY_ARRAY_SENTINEL, 0,
                                &UA_TYPES[UA_TYPES_STRING]);
            sc = UA_STATUSCODE_GOOD;
        }
        break;
    }

    case OV_ACTUAL_SPEED: {
        /* 契约单位 m/s = snapshot mm/s ÷ 1000 */
        UA_Double v = snap.v_current_mm_s / 1000.0;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_FEED_OVERRIDE: {
        UA_Double v = snap.feed_override_pct / 100.0;   /* pct → 0~1.5 比率 */
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_RAPID_OVERRIDE: {
        UA_Double v = snap.rapid_override_pct / 100.0;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_SPINDLE_OVERRIDE: {
        UA_Double v = snap.spindle_override_pct / 100.0;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }

    case OV_AXIS_ACTUAL_POS:
    case OV_AXIS_CMD_POS:
    case OV_AXIS_MECH_POS: {
        /* v1: 反馈位置字段尚未回写 (sim/实机均指令坐标), 三者同源 machine_pos;
         * 实机编码器反馈接入后 ACTUAL 切换 g_axis[i].actual_pos / pulse_per_unit */
        UA_Double v = (ctx->axis_idx >= 0 && ctx->axis_idx < AXIS_NUM)
                      ? snap.machine_pos[ctx->axis_idx] : 0.0;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_AXIS_PROG_POS: {
        UA_Double v = (ctx->axis_idx >= 0 && ctx->axis_idx < AXIS_NUM)
                      ? snap.logical_pos[ctx->axis_idx] : 0.0;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_AXIS_REMAIN_DIST: {
        UA_Double v = 0.0;
        if (ctx->axis_idx >= 0 && ctx->axis_idx < AXIS_NUM)
            v = snap.target_pos[ctx->axis_idx] - snap.machine_pos[ctx->axis_idx];
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }
    case OV_AXIS_ACTIVE: {
        UA_Boolean v = (ctx->axis_idx >= 0 && ctx->axis_idx < AXIS_NUM)
                       ? (g_axis[ctx->axis_idx].is_enabled != 0) : false;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_BOOLEAN]);
        break;
    }

    case OV_HOME_STATE: {
        /* 与 SMC_GetHomingState 同源: 0=IDLE 1=PENDING 2=RUNNING 3=DONE 4=FAULT
         * (Machine.Status 派生 HOMING 态亦用此状态) */
        int hstate = 0, haxis = 0;
        double hprog = 0.0;
        SMC_GetHomingState(&hstate, &haxis, &hprog);
        UA_Int32 v = (UA_Int32)hstate;
        sc = UA_Variant_setScalarCopy(&value->value, &v, &UA_TYPES[UA_TYPES_INT32]);
        break;
    }
    case OV_HOME_PROGRESS: {
        int hstate = 0, haxis = 0;
        double hprog = 0.0;
        SMC_GetHomingState(&hstate, &haxis, &hprog);
        sc = UA_Variant_setScalarCopy(&value->value, &hprog, &UA_TYPES[UA_TYPES_DOUBLE]);
        break;
    }

    default: {
        value->hasStatus = true;
        value->status = UA_STATUSCODE_BADINTERNALERROR;
        return UA_STATUSCODE_GOOD;
    }
    }

    if (sc != UA_STATUSCODE_GOOD) {
        value->hasStatus = true;
        value->status = sc;
        return UA_STATUSCODE_GOOD;
    }
    value->hasValue = true;
    if (includeSourceTimeStamp) {
        value->hasSourceTimestamp = true;
        value->sourceTimestamp = UA_DateTime_now();
    }
    return UA_STATUSCODE_GOOD;
}

/* @Context: Non-RealTime Background Thread (server 线程 Write)
 * @Safe: 调 SMC_SetOverride (非 RT 写 RT 单写者字段, 项目既有约定)。
 * 仅 3 个 override 节点可写 (accessLevel 已限制, 此处按 kind 二次防线)。 */
static UA_StatusCode
opcua_ds_write(UA_Server *server, const UA_NodeId *sessionId, void *sessionContext,
               const UA_NodeId *nodeId, void *nodeContext,
               const UA_NumericRange *range, const UA_DataValue *value)
{
    (void)server; (void)sessionId; (void)sessionContext; (void)nodeId; (void)range;
    OpcNodeCtx *ctx = (OpcNodeCtx *)nodeContext;
    if (!ctx || !value || !value->hasValue ||
        !UA_Variant_hasScalarType(&value->value, &UA_TYPES[UA_TYPES_DOUBLE])) {
        return UA_STATUSCODE_BADTYPEMISMATCH;
    }

    UA_Double ratio = *(UA_Double *)value->value.data;
    int pct = (int)(ratio * 100.0);   /* 契约比率 → pct */

    switch ((OpcVarKind)ctx->kind) {
    case OV_FEED_OVERRIDE:
        printf("[opcua] write FeedOverride %.3f -> SMC_SetOverride(%d)\n", ratio, pct);
        SMC_SetOverride(pct, -1, -1, 0, 0, NULL, NULL, NULL, NULL);
        return UA_STATUSCODE_GOOD;
    case OV_RAPID_OVERRIDE:
        SMC_SetOverride(-1, pct, -1, 0, 0, NULL, NULL, NULL, NULL);
        return UA_STATUSCODE_GOOD;
    case OV_SPINDLE_OVERRIDE:
        SMC_SetOverride(-1, -1, pct, 0, 0, NULL, NULL, NULL, NULL);
        return UA_STATUSCODE_GOOD;
    default:
        return UA_STATUSCODE_BADNOTWRITABLE;
    }
}

/* =====================================================================
 *  方法回调 (契约 §6, 返回 Boolean = 对应 SMC_* 成功)
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread (server 线程 MethodCall)
 * @Safe: SMC_* API 非阻塞设计; System.Reset 最多阻塞 OPCUA_RESET_WAIT_MS。 */
static UA_StatusCode
opcua_method_cb(UA_Server *server, const UA_NodeId *sessionId, void *sessionContext,
                const UA_NodeId *methodId, void *methodContext,
                const UA_NodeId *objectId, void *objectContext,
                size_t inputSize, const UA_Variant *input,
                size_t outputSize, UA_Variant *output)
{
    (void)server; (void)sessionId; (void)sessionContext;
    (void)methodId; (void)objectId; (void)objectContext; (void)outputSize;
    OpcNodeCtx *ctx = (OpcNodeCtx *)methodContext;
    if (!ctx) return UA_STATUSCODE_BADINTERNALERROR;

    UA_Boolean ok = false;
    char path[512];
    char axis_letter = 'X';
    UA_Int32 jog_dir = 0;
    UA_Double jog_speed = 0.0;
    UA_Double jog_distance = 0.0;

    switch ((OpcMethodKind)ctx->kind) {

    case OM_PROGRAM_LOAD:
        /* in: String filePath → Boolean。宽容转换 Windows 盘符路径 → /mnt */
        if (inputSize != 1 ||
            !UA_Variant_hasScalarType(&input[0], &UA_TYPES[UA_TYPES_STRING]))
            return UA_STATUSCODE_BADINVALIDARGUMENT;
        {
            UA_String *s = (UA_String *)input[0].data;
            size_t len = (s && s->length < sizeof(path) - 1) ? s->length : 0;
            if (s && s->data && len > 0) {
                memcpy(path, s->data, len);
                path[len] = '\0';
            } else {
                path[0] = '\0';
            }
        }
        {
            char resolved[512];
            resolve_program_path(path, resolved, sizeof(resolved));
            printf("[opcua] Program.Load: \"%s\" -> \"%s\"\n", path, resolved);
            ok = (SMC_LoadProgram(resolved) == 0);
        }
        break;

    case OM_PROGRAM_START:
        ok = (SMC_RunLoadedProgram() == 0);
        break;

    case OM_PROGRAM_PAUSE:
        SMC_PauseProcessing();      /* void, 恒成功 */
        ok = true;
        break;

    case OM_PROGRAM_RESUME:
        SMC_ResumeProcessing();
        ok = true;
        break;

    case OM_PROGRAM_STOP:
        SMC_AbortProcessing();
        ok = true;
        break;

    case OM_SYSTEM_RESET:
        /* 工业语义: 停 + 清报警。ClearAlarm 在 parser running 时拒绝 (-1),
         * 故先 Abort 再等 parser 退出 (上限 OPCUA_RESET_WAIT_MS)。 */
        if (SMC_IsParserRunning()) {
            SMC_AbortProcessing();
            for (int i = 0; i < OPCUA_RESET_WAIT_MS / 10; i++) {
                if (!SMC_IsParserRunning()) break;
                usleep(10000);
            }
        }
        ok = (SMC_ClearAlarm() == 0);
        if (ok) {
            /* 等 RT 清理真正完成 (alarm_reset_request 1→0): RT 清理路径会强停
             * JOG/homing/队列 (ecat_core "报警必停车"), 若在提交后立即返回,
             * 上层 Reset 后立刻点动会被延迟执行的清理吞掉 (竞态窗口)。
             * 上限 5s 覆盖 SafeLiftZ 自动抬升时长 (hw profile: 报警时 Z 先抬
             * 完 alarm_reset 才被 RT 放行清零, 如 50mm@20mm/s = 2.5s)。
             * Reset 返回 = 系统真正回到可操作态。 */
            for (int i = 0; i < 500; i++) {
                if (atomic_load_explicit(&g_interpolator.alarm_reset_request,
                                         memory_order_acquire) == 0)
                    break;
                usleep(10000);
            }
        }
        break;

    case OM_SYSTEM_ESTOP:
        /* 软急停 (ISO 13850 软件层): Abort + 软停机刹车 + 激光 kill +
         * SafeLift (若配 auto_on_alarm)。不可拒, 恒成功。
         * reason=1 (外部系统/上位软件触发)。恢复走 System.Reset。 */
        printf("[opcua] System.EStop triggered by client\n");
        SMC_EmergencyStop(1, "OPC UA CNC.System.EStop");
        ok = true;
        break;

    case OM_HOME_ALL:
        /* 全轴串行回零 (Z 优先, all-or-nothing 回滚)。异步提交, 进度看
         * CNC.Home.State / CNC.Home.Progress。未配置/冲突时返回 false。 */
        ok = (SMC_HomeAll() == 0);
        break;

    case OM_HOME_AXIS:
        /* 单轴回零。in: String axis (轴字母) */
        if (inputSize != 1 ||
            !UA_Variant_hasScalarType(&input[0], &UA_TYPES[UA_TYPES_STRING]))
            return UA_STATUSCODE_BADINVALIDARGUMENT;
        {
            UA_String *s = (UA_String *)input[0].data;
            if (s && s->length > 0 && s->data)
                axis_letter = (char)toupper((unsigned char)((char *)s->data)[0]);
        }
        ok = (SMC_HomeAxis(axis_letter) == 0);
        break;

    case OM_JOG_MOVE:
        /* in: String axis, Int32 dir(-1/0/1), Double speed */
        if (inputSize != 3 ||
            !UA_Variant_hasScalarType(&input[0], &UA_TYPES[UA_TYPES_STRING]) ||
            !UA_Variant_hasScalarType(&input[1], &UA_TYPES[UA_TYPES_INT32]) ||
            !UA_Variant_hasScalarType(&input[2], &UA_TYPES[UA_TYPES_DOUBLE]))
            return UA_STATUSCODE_BADINVALIDARGUMENT;
        {
            UA_String *s = (UA_String *)input[0].data;
            if (s && s->length > 0 && s->data)
                axis_letter = (char)toupper((unsigned char)((char *)s->data)[0]);
            jog_dir   = *(UA_Int32 *)input[1].data;
            jog_speed = *(UA_Double *)input[2].data;
        }
        if (jog_dir == 0)
            ok = (SMC_JogStop(axis_letter) == 0);
        else
            ok = (SMC_JogStart(axis_letter, (int)jog_dir, jog_speed) == 0);
        break;

    case OM_JOG_MOVE_INC: {
        /* 增量寸动: in String axis, Int32 dir(±1), Double distance(>0),
         * Double speed(>0, mm/s)。精确走 distance mm — 底层复用
         * SMC_MoveRelative (与 RPC MoveRelative 同源, plan_cursor 基准),
         * 零新运动学路径。 */
        if (inputSize != 4 ||
            !UA_Variant_hasScalarType(&input[0], &UA_TYPES[UA_TYPES_STRING]) ||
            !UA_Variant_hasScalarType(&input[1], &UA_TYPES[UA_TYPES_INT32]) ||
            !UA_Variant_hasScalarType(&input[2], &UA_TYPES[UA_TYPES_DOUBLE]) ||
            !UA_Variant_hasScalarType(&input[3], &UA_TYPES[UA_TYPES_DOUBLE]))
            return UA_STATUSCODE_BADINVALIDARGUMENT;
        {
            UA_String *s = (UA_String *)input[0].data;
            if (s && s->length > 0 && s->data)
                axis_letter = (char)toupper((unsigned char)((char *)s->data)[0]);
            jog_dir      = *(UA_Int32 *)input[1].data;
            jog_distance = *(UA_Double *)input[2].data;
            jog_speed    = *(UA_Double *)input[3].data;
        }
        /* 边界校验: SMC_MoveRelative 为 void fire-and-forget (轴不存在只
         * printf 后静默忽略), 越界入参必须在方法层拒收 */
        {
            int aidx = (axis_letter >= 'A' && axis_letter <= 'Z')
                           ? g_axis_map[axis_letter - 'A'] : -1;
            if (aidx < 0 || aidx >= AXIS_NUM
                || (jog_dir != 1 && jog_dir != -1)
                || !(jog_distance > 0.0) || !(jog_speed > 0.0)) {
                ok = false;
                break;
            }
        }
        /* 与 SMC_JogStart 同源互斥: Homing/SafeLift/parser 活动期拒绝。
         * SMC_MoveRelative 自身无互斥检查, 手动段混入自动流程队列是
         * 硬伤, 此守卫不可省。 */
        if (g_interpolator.homing_state != 0
            || g_interpolator.safe_lift_state != 0
            || SMC_IsParserRunning()) {
            ok = false;
            break;
        }
        /* 连续 JOG 进行中则幂等全停: ALL 路径恒返回 0, 恢复 time_scale
         * 并同步 plan_cursor 到当前实际位置 (P0-1 hotfix #2) — 保证增量
         * 基准是此刻位置而非 JOG 前旧值。空闲态调用无害 (幂等)。 */
        SMC_JogStop(SMC_AXIS_ALL);
        /* 有符号距离 = distance × dir; void 返回, 与 RPC 层 fire-and-forget
         * 语义对等。连续两次增量: plan_cursor 已推进, 距离正确累加。 */
        SMC_MoveRelative(axis_letter, jog_distance * (double)jog_dir, jog_speed);
        ok = true;
        break;
    }

    return UA_Variant_setScalarCopy(output, &ok, &UA_TYPES[UA_TYPES_BOOLEAN]);
}

/* =====================================================================
 *  地址空间构建
 * ===================================================================== */

/* 变量节点注册 helper: ns=1;s=<id> 挂 <parent> 下 (HasComponent) */
static UA_StatusCode
add_var(UA_Server *s, const char *id_suffix, const UA_NodeId parent,
        const char *browse_name, const UA_NodeId *data_type,
        uint8_t kind, int axis_idx, UA_Boolean writable, UA_Int32 value_rank)
{
    OpcNodeCtx *ctx = var_ctx_new(kind, axis_idx);
    if (!ctx) return UA_STATUSCODE_BADINTERNALERROR;

    char nodeid_str[128];
    snprintf(nodeid_str, sizeof(nodeid_str), "CNC.%s", id_suffix);

    UA_VariableAttributes attr;
    UA_VariableAttributes_init(&attr);
    attr.displayName = UA_LOCALIZEDTEXT("en-US", (char *)browse_name);
    attr.description = UA_LOCALIZEDTEXT("en-US", (char *)browse_name);
    attr.dataType = *data_type;
    attr.valueRank = value_rank;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ |
                       (writable ? UA_ACCESSLEVELMASK_WRITE : 0);

    const UA_DataSource ds = { opcua_ds_read, opcua_ds_write };
    UA_StatusCode sc = UA_Server_addDataSourceVariableNode(
        s,
        UA_NODEID_STRING(OPCUA_NAMESPACE_INDEX, nodeid_str),
        parent,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(OPCUA_NAMESPACE_INDEX, (char *)browse_name),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
        attr, ds, ctx, NULL);
    if (sc != UA_STATUSCODE_GOOD)
        fprintf(stderr, "[opcua] add var node %s fail: 0x%08x\n", nodeid_str, sc);
    return sc;
}

/* 方法节点注册 helper: ns=1;s=CNC.<id> 挂 <parent> 对象下。
 * 入参由 caller 用 arg_in_* 构造数组传入 (无参传 NULL/0), 出参恒 Boolean。 */
static UA_StatusCode
add_method(UA_Server *s, const char *id_suffix, const UA_NodeId parent,
           const char *browse_name, const char *desc, uint8_t kind,
           const UA_Argument *in_args, size_t n_in)
{
    OpcNodeCtx *ctx = mtd_ctx_new(kind);
    if (!ctx) {
        fprintf(stderr, "[opcua] add method %s fail: ctx pool exhausted\n", id_suffix);
        return UA_STATUSCODE_BADINTERNALERROR;
    }

    char nodeid_str[128];
    snprintf(nodeid_str, sizeof(nodeid_str), "CNC.%s", id_suffix);

    UA_MethodAttributes ma;
    UA_MethodAttributes_init(&ma);
    ma.displayName = UA_LOCALIZEDTEXT("en-US", (char *)browse_name);
    ma.description = UA_LOCALIZEDTEXT("en-US", (char *)(desc ? desc : browse_name));
    ma.executable = true;
    ma.userExecutable = true;

    UA_Argument out_arg;
    UA_Argument_init(&out_arg);
    out_arg.name = UA_STRING((char *)"result");
    out_arg.description = UA_LOCALIZEDTEXT("en-US", (char *)"true on success");
    out_arg.dataType = UA_TYPES[UA_TYPES_BOOLEAN].typeId;
    out_arg.valueRank = -1;

    UA_StatusCode sc = UA_Server_addMethodNode(
        s,
        UA_NODEID_STRING(OPCUA_NAMESPACE_INDEX, nodeid_str),
        parent,
        UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
        UA_QUALIFIEDNAME(OPCUA_NAMESPACE_INDEX, (char *)browse_name),
        ma, opcua_method_cb,
        n_in, in_args, 1, &out_arg,
        ctx, NULL);
    if (sc != UA_STATUSCODE_GOOD)
        fprintf(stderr, "[opcua] add method node %s fail: 0x%08x\n", nodeid_str, sc);
    return sc;
}

/* 标量入参构造 (add_method 配套) */
static UA_Argument arg_in_string(const char *name, const char *desc)
{
    UA_Argument a;
    UA_Argument_init(&a);
    a.name = UA_STRING((char *)name);
    a.description = UA_LOCALIZEDTEXT("en-US", (char *)desc);
    a.dataType = UA_TYPES[UA_TYPES_STRING].typeId;
    a.valueRank = -1;
    return a;
}
static UA_Argument arg_in_int32(const char *name, const char *desc)
{
    UA_Argument a;
    UA_Argument_init(&a);
    a.name = UA_STRING((char *)name);
    a.description = UA_LOCALIZEDTEXT("en-US", (char *)desc);
    a.dataType = UA_TYPES[UA_TYPES_INT32].typeId;
    a.valueRank = -1;
    return a;
}
static UA_Argument arg_in_double(const char *name, const char *desc)
{
    UA_Argument a;
    UA_Argument_init(&a);
    a.name = UA_STRING((char *)name);
    a.description = UA_LOCALIZEDTEXT("en-US", (char *)desc);
    a.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
    a.valueRank = -1;
    return a;
}

/* 对象节点注册 helper: nodeid_full 即完整 id (如 "CNC" / "CNC.Axis") */
static UA_NodeId
add_obj(UA_Server *s, const char *nodeid_full, const UA_NodeId parent,
        const UA_NodeId ref_type, const char *browse_name)
{
    UA_ObjectAttributes attr;
    UA_ObjectAttributes_init(&attr);
    attr.displayName = UA_LOCALIZEDTEXT("en-US", (char *)browse_name);

    UA_NodeId out = UA_NODEID_NULL;
    UA_Server_addObjectNode(
        s,
        UA_NODEID_STRING(OPCUA_NAMESPACE_INDEX, (char *)nodeid_full),
        parent, ref_type,
        UA_QUALIFIEDNAME(OPCUA_NAMESPACE_INDEX, (char *)browse_name),
        UA_NODEID_NUMERIC(0, UA_NS0ID_BASEOBJECTTYPE),
        attr, NULL, &out);
    return out;
}

/* @Context: Non-RealTime Background Thread (server 线程启动期一次)
 * 按 kernel_init 已配置的轴拓扑 (g_axis_map) 创建轴对象与属性节点。
 * 客户端 ScanAxesAndSubscribe Browse "ns=1;s=CNC.Axis" 只取 Object 子节点,
 * browseName 即轴名 — 与本函数创建结构一致。 */
static void build_axis_nodes(UA_Server *s, const UA_NodeId axis_parent)
{
    static const struct { const char *attr; uint8_t kind; } attrs[] = {
        { "ActualPosition",   OV_AXIS_ACTUAL_POS  },
        { "CommandPosition",  OV_AXIS_CMD_POS     },
        { "MechanicalPosition", OV_AXIS_MECH_POS  },
        { "ProgramPosition",  OV_AXIS_PROG_POS    },
        { "RemainingDistance", OV_AXIS_REMAIN_DIST },
        { "Active",           OV_AXIS_ACTIVE      },
    };

    for (int letter = 0; letter < 26; letter++) {
        int idx = g_axis_map[letter];
        if (idx < 0 || idx >= AXIS_NUM) continue;

        const char *axis_name = g_axis[idx].axis_name;   /* 如 "X" */
        if (axis_name[0] == '\0') continue;

        char axis_obj_id[32];
        snprintf(axis_obj_id, sizeof(axis_obj_id), "CNC.Axis.%s", axis_name);
        UA_NodeId axis_obj = add_obj(s, axis_obj_id, axis_parent,
                                     UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT),
                                     axis_name);

        for (size_t k = 0; k < sizeof(attrs) / sizeof(attrs[0]); k++) {
            const UA_NodeId *dt =
                (attrs[k].kind == OV_AXIS_ACTIVE)
                    ? &UA_TYPES[UA_TYPES_BOOLEAN].typeId
                    : &UA_TYPES[UA_TYPES_DOUBLE].typeId;
            char leaf[64];
            snprintf(leaf, sizeof(leaf), "Axis.%s.%s", axis_name, attrs[k].attr);
            UA_StatusCode sc = add_var(s, leaf, axis_obj, attrs[k].attr,
                                       dt, attrs[k].kind, idx, false, -1);
            if (sc != UA_STATUSCODE_GOOD)
                fprintf(stderr, "[opcua] add axis var %s fail: 0x%08x\n", leaf, sc);
        }
        printf("[opcua] axis object exposed: %s (idx=%d)\n", axis_name, idx);
    }
}

/* @Context: Non-RealTime Background Thread (server 线程启动期一次) */
static void build_address_space(UA_Server *s)
{
    const UA_NodeId ns_cnc = UA_NODEID_STRING(OPCUA_NAMESPACE_INDEX, "CNC");

    /* 根对象 CNC 挂 Objects 文件夹 (Organizes) */
    add_obj(s, "CNC", UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
            UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES), "CNC");

    /* ---- Machine 对象 ---- */
    UA_NodeId ns_machine = add_obj(s, "CNC.Machine", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Machine");
    add_var(s, "Machine.Status",      ns_machine, "Status",      &UA_TYPES[UA_TYPES_INT32].typeId,   OV_MACHINE_STATUS,    -1, false, -1);
    add_var(s, "Machine.Mode",        ns_machine, "Mode",        &UA_TYPES[UA_TYPES_INT32].typeId,   OV_MACHINE_MODE,      -1, false, -1);
    add_var(s, "Machine.PowerOnSpan", ns_machine, "PowerOnSpan", &UA_TYPES[UA_TYPES_STRING].typeId,  OV_MACHINE_POWERON,   -1, false, -1);
    add_var(s, "Machine.DateTime",    ns_machine, "DateTime",    &UA_TYPES[UA_TYPES_STRING].typeId,  OV_MACHINE_DATETIME,  -1, false, -1);

    /* ---- Program 对象 (变量 + 5 方法) ---- */
    UA_NodeId ns_program = add_obj(s, "CNC.Program", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Program");
    add_var(s, "Program.Name",        ns_program, "Name",        &UA_TYPES[UA_TYPES_STRING].typeId,  OV_PROGRAM_NAME,        -1, false, -1);
    add_var(s, "Program.MainName",    ns_program, "MainName",    &UA_TYPES[UA_TYPES_STRING].typeId,  OV_PROGRAM_MAINNAME,    -1, false, -1);
    add_var(s, "Program.CurrentLine", ns_program, "CurrentLine", &UA_TYPES[UA_TYPES_INT32].typeId,   OV_PROGRAM_CURRENTLINE, -1, false, -1);
    add_var(s, "Program.LineCount",   ns_program, "LineCount",   &UA_TYPES[UA_TYPES_INT32].typeId,   OV_PROGRAM_LINECOUNT,   -1, false, -1);

    add_method(s, "Program.Start",  ns_program, "Start",  "cycle start", OM_PROGRAM_START,  NULL, 0);
    add_method(s, "Program.Pause",  ns_program, "Pause",  "feedhold",    OM_PROGRAM_PAUSE,  NULL, 0);
    add_method(s, "Program.Resume", ns_program, "Resume", "resume",      OM_PROGRAM_RESUME, NULL, 0);
    add_method(s, "Program.Stop",   ns_program, "Stop",   "abort",       OM_PROGRAM_STOP,   NULL, 0);

    /* Program.Load(String filePath): G 代码路径 (Linux 路径; Windows 盘符
     * 路径自动转 /mnt, 仅 WSL2 部署有意义) */
    {
        UA_Argument in = arg_in_string("filePath", "G-code file path");
        add_method(s, "Program.Load", ns_program, "Load", "Load G-code file (preview)",
                   OM_PROGRAM_LOAD, &in, 1);
    }

    /* ---- Alarm 对象 ---- */
    UA_NodeId ns_alarm = add_obj(s, "CNC.Alarm", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Alarm");
    add_var(s, "Alarm.Count", ns_alarm, "Count", &UA_TYPES[UA_TYPES_INT32].typeId,  OV_ALARM_COUNT, -1, false, -1);
    /* valueRank=-2 (Any): open62541 addNode 静态 typecheck 拒绝 valueRank=1
     * (一维数组) 且无 attr.value 初值的 DataSource 节点 (BadTypeMismatch)。
     * -2 是规范合法的宽松声明, 客户端按值类型 (String[]) 解析, 不受影响。 */
    add_var(s, "Alarm.List",  ns_alarm, "List",  &UA_TYPES[UA_TYPES_STRING].typeId, OV_ALARM_LIST,  -1, false, -2);

    /* ---- System / Home / Jog 对象 ---- */
    UA_NodeId ns_system = add_obj(s, "CNC.System", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "System");
    add_method(s, "System.Reset", ns_system, "Reset",
               "stop + clear alarm", OM_SYSTEM_RESET, NULL, 0);
    /* 软急停 (ISO 13850 软件层): Abort + 软停机刹车 + 激光 kill + SafeLift;
     * 不可拒恒 true, 恢复走 System.Reset */
    add_method(s, "System.EStop", ns_system, "EStop",
               "software emergency stop (recover via Reset)", OM_SYSTEM_ESTOP, NULL, 0);

    /* Home 对象: 回零状态/进度 + 单轴/全轴回零 (实机 G53 基准) */
    UA_NodeId ns_home = add_obj(s, "CNC.Home", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Home");
    add_var(s, "Home.State",    ns_home, "State",    &UA_TYPES[UA_TYPES_INT32].typeId,  OV_HOME_STATE,    -1, false, -1);
    add_var(s, "Home.Progress", ns_home, "Progress", &UA_TYPES[UA_TYPES_DOUBLE].typeId, OV_HOME_PROGRESS, -1, false, -1);
    add_method(s, "Home.All", ns_home, "All",
               "home all axes (serial, Z first)", OM_HOME_ALL, NULL, 0);
    {
        UA_Argument in = arg_in_string("axis", "axis letter");
        add_method(s, "Home.Axis", ns_home, "Axis",
                   "home single axis", OM_HOME_AXIS, &in, 1);
    }

    UA_NodeId ns_jog = add_obj(s, "CNC.Jog", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Jog");
    {
        UA_Argument in[3] = {
            arg_in_string("axis",   "axis letter"),
            arg_in_int32("dir",     "-1/0/1 (0=stop)"),
            arg_in_double("speed",  "mm/s"),
        };
        add_method(s, "Jog.Move", ns_jog, "Move", "JOG move", OM_JOG_MOVE, in, 3);
    }
    {
        UA_Argument in[4] = {
            arg_in_string("axis",     "axis letter"),
            arg_in_int32("dir",       "-1/+1 direction"),
            arg_in_double("distance", "travel distance (mm, > 0)"),
            arg_in_double("speed",    "mm/s (> 0)"),
        };
        add_method(s, "Jog.MoveInc", ns_jog, "MoveInc", "incremental JOG move",
                   OM_JOG_MOVE_INC, in, 4);
    }

    /* ---- 速度/倍率 (直挂 CNC 下) ---- */
    add_var(s, "ActualSpeed",    ns_cnc, "ActualSpeed",    &UA_TYPES[UA_TYPES_DOUBLE].typeId, OV_ACTUAL_SPEED,    -1, false, -1);
    add_var(s, "FeedOverride",   ns_cnc, "FeedOverride",   &UA_TYPES[UA_TYPES_DOUBLE].typeId, OV_FEED_OVERRIDE,   -1, true,  -1);
    add_var(s, "RapidOverride",  ns_cnc, "RapidOverride",  &UA_TYPES[UA_TYPES_DOUBLE].typeId, OV_RAPID_OVERRIDE,  -1, true,  -1);
    add_var(s, "SpindleOverride",ns_cnc, "SpindleOverride",&UA_TYPES[UA_TYPES_DOUBLE].typeId, OV_SPINDLE_OVERRIDE,-1, true,  -1);

    /* ---- 轴对象 (按 kernel_init 配置的拓扑动态创建) ---- */
    UA_NodeId ns_axis = add_obj(s, "CNC.Axis", ns_cnc,
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASCOMPONENT), "Axis");
    build_axis_nodes(s, ns_axis);
}

/* =====================================================================
 *  线程与生命周期
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread (opcua server 主线程)
 * @Safe: open62541 内部 select 循环 + DataSource 回调 (Non-RT 读路径)。 */
static void *opcua_thread_func(void *arg)
{
    (void)arg;

    g_opcua_server = UA_Server_new();
    if (!g_opcua_server) {
        fprintf(stderr, "[opcua] UA_Server_new failed\n");
        atomic_store(&g_opcua_running, 0);
        return NULL;
    }

    UA_ServerConfig *config = UA_Server_getConfig(g_opcua_server);
    /* SecurityPolicy None + 匿名 + 4840 (契约 §8: 默认不加密);
     * bind 全接口 — Windows movecontrol 经 WSL2 localhost 转发可达 */
    UA_StatusCode sc = UA_ServerConfig_setMinimal(config, OPCUA_PORT, NULL);
    if (sc != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] setMinimal fail: 0x%08x\n", sc);
        UA_Server_delete(g_opcua_server);
        g_opcua_server = NULL;
        atomic_store(&g_opcua_running, 0);
        return NULL;
    }

    build_address_space(g_opcua_server);

    /* 程序根目录确保存在 (上位 SMB 共享写入目标; 已存在则 no-op) */
    {
        const char *env = getenv("SMC_PROGRAM_DIR");
        const char *dir = (env && env[0]) ? env : CNC_PROGRAM_DIR_DEFAULT;
        mkdir(dir, 0755);
    }

    sc = UA_Server_run_startup(g_opcua_server);
    if (sc != UA_STATUSCODE_GOOD) {
        fprintf(stderr, "[opcua] run_startup fail: 0x%08x (port %d busy?)\n", sc, OPCUA_PORT);
        UA_Server_delete(g_opcua_server);
        g_opcua_server = NULL;
        atomic_store(&g_opcua_running, 0);
        return NULL;
    }

    printf("[opcua] OPC UA server listening on opc.tcp://0.0.0.0:%d (ns=1, no security)\n",
           OPCUA_PORT);

    while (atomic_load_explicit(&g_opcua_running, memory_order_acquire)) {
        /* block=true: 内部 select 等待网络事件, 空闲不耗 CPU;
         * stop 时最多延迟一个 iterate 周期退出 */
        UA_Server_run_iterate(g_opcua_server, true);
    }

    UA_Server_run_shutdown(g_opcua_server);
    UA_Server_delete(g_opcua_server);
    g_opcua_server = NULL;
    printf("[opcua] server stopped\n");
    return NULL;
}

int opcua_server_start(void)
{
    if (atomic_load_explicit(&g_opcua_running, memory_order_acquire))
        return 0;

    atomic_store_explicit(&g_opcua_running, 1, memory_order_release);
    if (pthread_create(&g_opcua_tid, NULL, opcua_thread_func, NULL) != 0) {
        perror("[opcua] pthread_create");
        atomic_store_explicit(&g_opcua_running, 0, memory_order_release);
        return -1;
    }
    pthread_detach(g_opcua_tid);
    return 0;
}

void opcua_server_stop(void)
{
    if (!atomic_load_explicit(&g_opcua_running, memory_order_acquire))
        return;
    atomic_store_explicit(&g_opcua_running, 0, memory_order_release);
    /* detached 线程, 不 join; run_iterate 唤醒后自行退出清理 */
}
