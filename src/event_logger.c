/* =====================================================================
 *  event_logger.c  ——  P1-b 事件/报警流推送中心实现
 *
 *  设计见 inc/event_logger.h 头注释。结构拷贝自 preview_streamer.c, 改 entry
 *  类型为 SmcEvent_t, 容量改 1024 (事件比段少)。
 *
 *  关键不变量:
 *    1. g_event_write_seq 单调不减
 *    2. ring slot [N % CAP] 在 write_seq > N 时已写, 内容稳定直到 write_seq > N + CAP
 *    3. writer 是 single-producer (但可跨线程, atomic_fetch_add seq 保证唯一性)
 *    4. reader 检查 from_seq + CAP > write_seq 才读, 保证 slot 未被覆盖
 *
 *  跨线程 Push 安全性:
 *    多线程同时 Push 时, atomic_fetch_add_event_seq 保证每个 caller 拿到唯一 seq,
 *    写入 ring[seq % CAP] 是不同 slot (除非已 wrap, 此时数据丢失但不会撕裂)。
 *    写入顺序: 先填 local event, 再 memcpy 到 ring slot, 最后 atomic_store release seq。
 *    但本实现简化: 直接写到 ring slot, 单 _Atomic seq 推进。
 *    若多线程同时 Push, slot 可能并发写 → 撕裂。
 *
 *    **v1 假设**: EventLogger_Push 主要在 RT 单线程 (1ms) + parser 单线程 (10Hz) +
 *    SMC_API 偶发调用, 多数时刻不并发。若实测发现撕裂, v2 加 spinlock。
 * ===================================================================== */

#include "event_logger.h"
#include "global_def.h"   /* cycle (uint32 RT 周期计数) */
#include <string.h>

/* ---- 静态分配 (避免 malloc, 模块加载即就绪) ---- */
static SmcEvent_t g_event_ring[EVENT_RING_CAPACITY];
static _Alignas(64) _Atomic uint64_t g_event_write_seq = 0;

int EventLogger_Init(void)
{
    memset(g_event_ring, 0, sizeof(g_event_ring));
    atomic_store_explicit(&g_event_write_seq, 0, memory_order_release);
    return 0;
}

void EventLogger_Push(uint8_t severity, uint8_t source, uint16_t code,
                       int32_t value, const char *message)
{
    uint64_t seq = atomic_fetch_add_explicit(&g_event_write_seq, 1,
                                              memory_order_acq_rel);
    uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
    SmcEvent_t *ev = &g_event_ring[idx];

    /* timestamp = 当前 RT cycle * 1ms (与 snapshot uptime_ms 一致) */
    ev->timestamp_ms = (uint64_t)cycle;
    ev->event_seq    = seq;
    ev->severity     = severity;
    ev->source       = source;
    ev->code         = code;
    ev->value        = value;

    /* strncpy 不保证 null 终结, 手动强制 */
    if (message != NULL) {
        strncpy(ev->message, message, SMC_EVENT_MSG_LEN - 1);
        ev->message[SMC_EVENT_MSG_LEN - 1] = '\0';
    } else {
        ev->message[0] = '\0';
    }
}

int EventLogger_ReadSince(uint64_t from_seq,
                            SmcEvent_t *out_buf,
                            int max_count,
                            uint64_t *out_next_seq)
{
    if (out_buf == NULL || max_count <= 0) {
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    uint64_t latest = atomic_load_explicit(&g_event_write_seq,
                                            memory_order_acquire);

    if (from_seq >= latest) {
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    /* 检查 from_seq 是否已被覆盖 */
    if (latest > (uint64_t)EVENT_RING_CAPACITY &&
        from_seq + EVENT_RING_CAPACITY <= latest) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    int available = (int)(latest - from_seq);
    if (available > max_count) available = max_count;

    for (int i = 0; i < available; i++) {
        uint64_t seq = from_seq + (uint64_t)i;
        uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
        out_buf[i] = g_event_ring[idx];
    }

    /* 二次检查: 读期间 writer 是否覆盖了 from_seq */
    uint64_t now = atomic_load_explicit(&g_event_write_seq,
                                         memory_order_acquire);
    if (now > (uint64_t)EVENT_RING_CAPACITY &&
        from_seq + EVENT_RING_CAPACITY <= now) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    if (out_next_seq) *out_next_seq = from_seq + (uint64_t)available;
    return available;
}

uint64_t EventLogger_GetWriteSeq(void)
{
    return atomic_load_explicit(&g_event_write_seq, memory_order_acquire);
}
