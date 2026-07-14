#ifndef PREVIEW_STREAMER_H
#define PREVIEW_STREAMER_H

/* =====================================================================
 *  preview_streamer.h  ——  P0-b v1 段流推送中心
 *
 *  定位:
 *    parser 入队的每个 TrajectorySegment_t 在进 motion queue 之前, fork
 *    一份副本到本模块的环形缓冲。外部 UI client (经 rpc_preview_server
 *    端口 9529) 按各自订阅 seq 拉取段流, 用于轨迹预览 + 实时光标。
 *
 *  并发模型 (SPSC 历史 ring + 多 reader):
 *    单写者: Non-RT Background Thread (parser_thread, axis_ctrl hook 内)
 *      ring[write_seq % CAP] = *seg; write_seq++;   (release)
 *    多读者: Non-RT Background Thread (push server client_thread)
 *      ReadSince(from_seq, buf, max) 拷贝 [from_seq, min(latest, from_seq+max))
 *
 *  历史 ring vs SnapshotHub seqlock 的差异:
 *    SnapshotHub: client 拉最新一帧 (不关心历史)
 *    PreviewStreamer: client 拉段序列, 晚到 client 要从 from_seq=0 拿全历史
 *
 *  覆盖策略:
 *    ring 容量 PREVIEW_RING_CAPACITY=8192 段 (~2MB @ 256B/seg)。
 *    writer 持续推进, 覆盖最旧段。reader 落后 CAP 段以上时 ReadSince 返回 -1,
 *    client 应报错重连 (或调大 CAP, v2 加分块拉取 API)。
 *
 *  与 motion queue 的关系:
 *    本模块只读副本 (fork), 完全不影响 CommandQueue 行为。即使本模块崩溃,
 *    CNC 加工不受影响。
 * ===================================================================== */

#include <stdint.h>
#include <stdatomic.h>
#include "axis_cfg.h"   /* TrajectorySegment_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 容量与限流 ---- */
#define PREVIEW_RING_CAPACITY   8192   /* 历史环容段数, ~2MB @ 256B/seg */
#define PREVIEW_READ_MAX        16     /* 单次 ReadSince 最多段数 (server 单帧 ≤16) */

/* ---- 帧标识 (与 SmcPreviewFrameHeader.magic 一致) ---- */
#define SMC_PREVIEW_MAGIC       0x53524556u   /* "PREV" little-endian */
#define SMC_PREVIEW_ACK_MAGIC   0x50524146u   /* "PRAK" - Preview Ack */
#define SMC_PREVIEW_VERSION     2u   /* v2: TrajectorySegment_t 加 seg_flags (Laser B4, 2026-07-14). 结构体大小不变 560B (复用 motion_type 后 padding), 字段布局变, 旧 client fail-fast. */

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread (main 启动早期, RT/parser 启动前)
 * @Safe: 静态分配 ring 清零 + atomic init。幂等。 */
int  PreviewStreamer_Init(void);

/* @Context: Non-RealTime Background Thread (parser_thread / bspline_thread)
 * @Safe: 1 次 memcpy + 1 次 atomic store。无锁, 不感知 reader。
 *         绝不在 1ms Hard-RT 线程调用 (TrajectorySegment_t 太大, memcpy 影响抖动)。
 *         api_push_trajectory_impl 在 Non-RT 后台线程调用本函数。
 * @param seg 待推段 (caller 通常从 CommandQueue.buffer[head] 直接传址) */
void PreviewStreamer_Push(const TrajectorySegment_t *seg);

/* @Context: Non-RealTime Background Thread (push server client_thread)
 * @Safe: 多 reader 并发安全。memcpy + atomic load。
 *
 * @param from_seq   读取起始 seq (= client 上次读到的 next_seq)
 * @param out_buf    输出缓冲 (caller 分配, 至少 max_count 个 TrajectorySegment_t)
 * @param max_count  out_buf 容量 (建议 ≤ PREVIEW_READ_MAX=16)
 * @param out_next_seq 输出: 下次 ReadSince 应传的 from_seq (= from_seq + 实际段数)
 *
 * @return >0: 实际读到的段数 (拷到 out_buf[0..n-1])
 *          0: 无新段 (from_seq >= write_seq)
 *         -1: from_seq 过期 (from_seq + CAP <= write_seq, 已被覆盖)。
 *             client 应报错或重连 (重连时 from_seq=0 拿当前全历史) */
int  PreviewStreamer_ReadSince(uint64_t from_seq,
                                TrajectorySegment_t *out_buf,
                                int max_count,
                                uint64_t *out_next_seq);

/* @Context: 任意线程
 * @return 当前总写入段数 (= 下一段将分配的 seq)。client 用此估算 client lag。 */
uint64_t PreviewStreamer_GetWriteSeq(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PREVIEW_STREAMER_H */
