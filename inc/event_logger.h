#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

/* =====================================================================
 *  event_logger.h  ——  P1-b 事件/报警流推送中心
 *
 *  定位:
 *    把 11 个 alarm 触发点 + 程序生命周期事件结构化记录, 经 rpc_event_server
 *    端口 9530 推送给 UI。UI 据此实现"报警历史窗口" + 故障诊断。
 *
 *  并发模型 (SPSC 历史 ring + 多 reader, 与 PreviewStreamer 同模式):
 *    单写者: 任意线程 (RT/parser/SMC_API), EventLogger_Push 内部 memcpy + atomic store
 *    多读者: client_thread, EventLogger_ReadSince 拉取 [from_seq, latest)
 *
 *  RT 安全:
 *    EventLogger_Push 仅 memcpy 88B + 1 atomic store + strncpy 64B 常量,
 *    无锁无 malloc 无 printf, 1ms 周期内可忽略开销。
 *
 *  与 rt_log 的关系 (互补):
 *    rt_log (ecat_core.c:47-56): RT 调试日志, 64 条 × 128B, drain 到 stdout, 不外发
 *    EventLogger: 业务事件流, 1024 events × 88B, 推送给 UI
 *
 *  Event code 规范见 plan 文件 radiant-mixing-reddy.md "Event code 规范" 表。
 * ===================================================================== */

#include <stdint.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 容量与限流 ---- */
#define EVENT_RING_CAPACITY    1024   /* 历史环容事件数, 1024 × 88B ≈ 88KB */
#define EVENT_READ_MAX         32     /* 单次 ReadSince 最多事件数 (server 单帧 ≤32) */
#define SMC_EVENT_MSG_LEN      64     /* message 字段定长 */

/* ---- C1 (2026-07-24) 故障持久化 ---- */
#define EVENT_PERSIST_MAGIC      0x474C5645u   /* "EVLG" little-endian */
#define EVENT_PERSIST_VERSION    1u
#define EVENT_PERSIST_PERIOD_MS  100           /* 落盘线程周期 */
#define EVENT_PERSIST_DIR        "event_logs"  /* 默认目录 (相对工作路径) */
#define EVENT_PERSIST_FILENAME   "event_log_%Y%m%d_%H%M%S.bin"
#define EVENT_PERSIST_MAX_AGE_D  7             /* 清理超过 7 天的旧文件 */

/* ---- 帧标识 ---- */
#define SMC_EVENT_MAGIC        0x45564E54u   /* "EVNT" little-endian */
#define SMC_EVENT_ACK_MAGIC    0x45564E4Bu   /* "EVNK" - Event Ack */
#define SMC_EVENT_VERSION      1u

/* ---- severity 枚举 ---- */
#define SEVERITY_INFO     1
#define SEVERITY_WARN     2
#define SEVERITY_ALARM    3
#define SEVERITY_FATAL    4

/* ---- source 枚举 ---- */
#define SOURCE_PARSER     1
#define SOURCE_RT         2
#define SOURCE_LASER      3
#define SOURCE_DRIVE      4
#define SOURCE_PLC        5
#define SOURCE_MANUAL     6

/* ---- event code 规范 (按 source 分段) ----
 * DRIVE=4:   0x0001 sync err / 0x0002 SW_ERROR / 0x0003 follow err hard / 0x0004 follow err warn
 * LASER=3:   0x0010 safety door / 0x0011 estop soft / 0x0012 laser ALM / 0x0013 water/gas
 * PARSER=1:  0x0020 M3/M4 rpm<=0 / 0x0021 M67/M68 oob / 0x0022 G04 neg / 0x0023 M70/M71 oob
 *            0x0030 LoadProgram start / 0x0031 LoadProgram done
 *            0x0032 RunLoadedProgram start / 0x0033 program done (M30) / 0x0034 abort
 * MANUAL=6:  0x0040 ClearAlarm req / 0x0041 alarm cleared (RT confirmed)
 */

#pragma pack(push, 1)
typedef struct {
    uint64_t timestamp_ms;     /* cycle*1ms 全局硬时间 (与 snapshot.uptime_ms 一致) */
    uint64_t event_seq;        /* EventLogger 内部递增 (Push 时原子 fetch_add) */
    uint8_t  severity;         /* SEVERITY_* */
    uint8_t  source;           /* SOURCE_* */
    uint16_t code;             /* source-specific event code (见上表) */
    int32_t  value;            /* 可选值 (e.g. follow_err 脉冲数, alarm 位图) */
    char     message[SMC_EVENT_MSG_LEN];  /* UTF-8 null 终结, 不够 64B 用 strncpy 填 0 */
} SmcEvent_t;   /* 88B packed */
#pragma pack(pop)

/* ---- C1 (2026-07-24) 故障持久化文件头 ----
 * 文件布局: EventLogFileHeader_t (32B) + SmcEvent_t × CAP (90112B) ≈ 88KB 固定
 * ring 结构: 文件 slot [seq % CAP] 与内存 ring 同构, 落盘/回放共用偏移公式
 * 多会话隔离: 每次启动新建带时间戳文件, 旧文件保留 EVENT_PERSIST_MAX_AGE_D 天 */
#pragma pack(push, 1)
typedef struct {
    uint32_t magic;            /* EVENT_PERSIST_MAGIC = "EVLG" */
    uint32_t version;          /* EVENT_PERSIST_VERSION = 1 */
    uint32_t capacity;         /* EVENT_RING_CAPACITY = 1024 */
    uint32_t reserved;         /* 0, 未来扩展 */
    uint64_t write_seq;        /* 当前总写入事件数 (与 g_event_write_seq 同语义) */
    uint64_t boot_id;          /* 启动 ID (每次启动递增, UI 区分不同会话) */
} EventLogFileHeader_t;        /* 32B packed */
#pragma pack(pop)

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime (main 启动早期, RT/parser 启动前)
 * @Safe: 静态 ring 清零 + atomic init。幂等。 */
int  EventLogger_Init(void);

/* @Context: 任意线程 (RT / parser / SMC_API 后台)
 * @Safe: memcpy 88B + atomic store + strncpy 64B。无锁无 malloc 无 printf。
 * @note  RT 端调用: message 必须是常量字符串或预分配, 不能 sprintf/malloc。
 *        Non-RT 端可以 snprintf 后传 buffer (但需 caller 保证 buffer 生存期)。
 *        strncpy 在 Push 内部完成 (取前 63 字节 + 强制 null 终结)。 */
void EventLogger_Push(uint8_t severity, uint8_t source, uint16_t code,
                       int32_t value, const char *message);

/* @Context: Non-RealTime Background Thread (event_server client_thread)
 * @Safe: 多 reader 并发安全。memcpy + atomic load。
 *
 * @param from_seq     读取起始 seq (= client 上次 next_seq)
 * @param out_buf      输出缓冲 (caller 分配, 至少 max_count 个 SmcEvent_t)
 * @param max_count    out_buf 容量 (≤ EVENT_READ_MAX=32)
 * @param out_next_seq 输出: 下次 ReadSince 应传的 from_seq
 *
 * @return >0: 实际读到的事件数
 *          0: 无新事件
 *         -1: from_seq 过期已被覆盖 (from_seq + CAP <= write_seq), client 应报错重连 */
int  EventLogger_ReadSince(uint64_t from_seq,
                            SmcEvent_t *out_buf,
                            int max_count,
                            uint64_t *out_next_seq);

/* @Context: 任意线程
 * @return 当前总写入事件数 (= 下一事件将分配的 seq) */
uint64_t EventLogger_GetWriteSeq(void);

/* =====================================================================
 *  C1 (2026-07-24) 故障持久化
 *
 *  设计:
 *    1. Init 时扫描 EVENT_PERSIST_DIR 下所有 event_log_*.bin, 按时间戳取最新
 *       → 回放最后 CAP 条到内存 ring + 设 atomic write_seq
 *    2. 新建带时间戳文件作为本次会话写入目标
 *    3. 后台落盘线程 (100ms 周期) 增量 fwrite + fsync
 *    4. StopPersistThread flush 最后一批 + 关闭文件
 *
 *  RT 安全:
 *    - 落盘线程独立 Non-RT, 不影响 RT/parser
 *    - 读 ring 用 atomic seq + memcpy 88B (与 ReadSince 同模式)
 *    - 落盘失败 (磁盘满/权限) 降级为内存 only + warn 日志, 不阻塞系统
 *
 *  文件管理:
 *    - 每次启动新建文件 (时间戳路径), 不覆盖旧文件
 *    - Init 时清理超过 EVENT_PERSIST_MAX_AGE_D (7) 天的旧文件
 *    - 文件大小固定 ~88KB (header 32B + 1024 × 88B)
 * ===================================================================== */

/* @Context: Non-RealTime (SMC_InitAndStart 内调用)
 * @Safe: 文件 I/O + 线程创建. 调用前 EventLogger_Init 必须已完成 (回放逻辑在 Init 内).
 * dir_path: 持久化目录 (NULL=用默认 EVENT_PERSIST_DIR; 不存在则创建)
 * out_boot_id: 返回本次启动的 boot_id (UI 可用于区分会话)
 * 返回: 0=成功, -1=目录创建失败, -2=文件打开失败 (降级内存 only) */
int  EventLogger_StartPersistThread(const char *dir_path, uint64_t *out_boot_id);

/* @Context: Non-RealTime (SMC_Close 内调用)
 * @Safe: 阻塞 flush 最后一批 + join 落盘线程 + fclose. 必须在 RT 线程退出后调.
 * 返回: 0=成功, -1=未启动 */
int  EventLogger_StopPersistThread(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* EVENT_LOGGER_H */
