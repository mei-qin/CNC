/* =====================================================================
 *  preview_streamer.c  ——  P0-b v1 段流推送中心实现
 *
 *  设计见 inc/preview_streamer.h 头注释。
 *
 *  关键不变量:
 *    1. g_preview_write_seq 单调不减 (writer 推进, reader 只读)
 *    2. ring slot [N % CAP] 在 write_seq > N 时已被写过, 内容稳定直到 write_seq 推到 N+CAP
 *    3. writer 是 single-producer, ring slot 不会被并发写
 *    4. reader 检查 from_seq + CAP > write_seq 才读, 保证 slot 未被覆盖
 *
 *  内存序:
 *    Push: memcpy (relaxed) + atomic_store(write_seq, release)
 *    ReadSince: atomic_load(write_seq, acquire) + memcpy + 再次 atomic_load 验证
 *    release/acquire 配对确保 reader 看到 ring slot 内容之前 write_seq 已更新
 * ===================================================================== */

#include "preview_streamer.h"
#include <string.h>

/* ---- 静态分配 (避免 malloc, 模块加载即就绪) ----
 * _Alignas(64) 让 write_seq 独占 cache line, 减少 false sharing。
 * ring 本身不 align cache line (太大会浪费内存, slot 边界不齐不影响正确性)。 */
static TrajectorySegment_t g_preview_ring[PREVIEW_RING_CAPACITY];
static _Alignas(64) _Atomic uint64_t g_preview_write_seq = 0;

int PreviewStreamer_Init(void)
{
    memset(g_preview_ring, 0, sizeof(g_preview_ring));
    atomic_store_explicit(&g_preview_write_seq, 0, memory_order_release);
    return 0;
}

void PreviewStreamer_Push(const TrajectorySegment_t *seg)
{
    if (seg == NULL) return;

    uint64_t seq = atomic_load_explicit(&g_preview_write_seq, memory_order_relaxed);
    uint32_t idx = (uint32_t)(seq % PREVIEW_RING_CAPACITY);

    /* 结构体赋值 = memcpy (TrajectorySegment_t 大但无 _Atomic 字段, 直接赋值安全) */
    g_preview_ring[idx] = *seg;

    /* release: 保证上面的 memcpy 在 write_seq 推进之前对 reader 可见 */
    atomic_store_explicit(&g_preview_write_seq, seq + 1, memory_order_release);
}

int PreviewStreamer_ReadSince(uint64_t from_seq,
                                TrajectorySegment_t *out_buf,
                                int max_count,
                                uint64_t *out_next_seq)
{
    if (out_buf == NULL || max_count <= 0) {
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    /* acquire: 看到 write_seq 之前, ring slot 的 memcpy 已对 reader 可见 */
    uint64_t latest = atomic_load_explicit(&g_preview_write_seq, memory_order_acquire);

    if (from_seq >= latest) {
        /* 无新段 */
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    /* 检查 from_seq 是否已被覆盖。
     * 有效范围: [latest - CAP, latest), 即 from_seq + CAP > latest 才有效。
     * 注意 latest >= CAP 时才可能溢出, 否则 from_seq 一定有效 (从 0 开始)。 */
    if (latest > (uint64_t)PREVIEW_RING_CAPACITY &&
        from_seq + PREVIEW_RING_CAPACITY <= latest) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    /* 实际可读段数: min(latest - from_seq, max_count) */
    int available = (int)(latest - from_seq);
    if (available > max_count) available = max_count;

    /* 拷贝段到 out_buf */
    for (int i = 0; i < available; i++) {
        uint64_t seq = from_seq + (uint64_t)i;
        uint32_t idx = (uint32_t)(seq % PREVIEW_RING_CAPACITY);
        out_buf[i] = g_preview_ring[idx];
    }

    /* 二次检查: 读期间 writer 是否覆盖了 from_seq?
     * 极端情况: writer 在本函数执行期间推了 CAP+ 段, from_seq slot 已被覆盖。
     * 此时 out_buf 内容不可信, 返回 -1 让 client 重连。
     * 实际触发条件: writer 速率 >> reader 速率 + CAP 太小, v1 不优化。 */
    uint64_t now = atomic_load_explicit(&g_preview_write_seq, memory_order_acquire);
    if (now > (uint64_t)PREVIEW_RING_CAPACITY &&
        from_seq + PREVIEW_RING_CAPACITY <= now) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    if (out_next_seq) *out_next_seq = from_seq + (uint64_t)available;
    return available;
}

uint64_t PreviewStreamer_GetWriteSeq(void)
{
    return atomic_load_explicit(&g_preview_write_seq, memory_order_acquire);
}
