#ifndef RPC_PREVIEW_SERVER_H
#define RPC_PREVIEW_SERVER_H

/* =====================================================================
 *  rpc_preview_server.h  ——  P0-b v1 段流推送服务端 (端口 9529)
 *
 *  定位:
 *    独立于 9527 RPC (req-resp) 和 9528 push (snapshot) 的段流推送通道。
 *    parser 入队的每个 TrajectorySegment_t 副本写入 PreviewStreamer 历史 ring,
 *    本服务端 accept 多个 UI client, 每客户端一个线程按订阅 freq + from_seq
 *    拉取段批量推送。用于 UI 轨迹预览 + G 代码编辑器行高亮 + 实时光标 (P0-c)。
 *
 *  协议:
 *    1. client TCP connect 9529
 *    2. client 发 SmcReqHeader{cmd=SMC_CMD_PREVIEW_SUBSCRIBE, data_len=12} +
 *                   {int32 freq_hz, uint64 from_seq}
 *       from_seq=0 表示从头要全部历史段
 *    3. server 回 SmcPreviewAck (16B)
 *    4. server 按 freq_hz 周期性推 [SmcPreviewFrameHeader + N × TrajectorySegment_t]
 *       N ≤ PREVIEW_READ_MAX (16)
 *
 *  帧布局 (单帧多段, 减少 TCP 帧头开销):
 *    [SmcPreviewFrameHeader 16B][N × TrajectorySegment_t (~256B each)]
 *    CRC32 覆盖: SmcPreviewFrameHeader[8..12] (seg_count) + segments
 *
 *  跨平台:
 *    本头文件只声明 SMC_PREVIEW_PORT / SmcPreviewFrameHeader / SmcPreviewAck /
 *    函数原型, 不含任何平台特定类型。Linux 服务端实现于 rpc/rpc_preview_server.c,
 *    Windows C++ SDK 可包含本头文件解析帧。
 * ===================================================================== */

#include <stdint.h>
#include "preview_streamer.h"   /* PREVIEW_READ_MAX, SMC_PREVIEW_* */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 推送通道端口 (9527 RPC, 9528 snapshot push, 9529 preview push) ---- */
#define SMC_PREVIEW_PORT     9529
#define SMC_PREVIEW_BACKLOG  8

/* ---- 客户端订阅频率上下限 (与 9528 push 通道一致) ---- */
#define SMC_PREVIEW_FREQ_MIN     1
#define SMC_PREVIEW_FREQ_MAX     200
#define SMC_PREVIEW_FREQ_DEFAULT 60

#pragma pack(push, 1)

/* Ack 帧 (16B, 与 SmcPreviewFrameHeader 同尺寸便于 client 复用缓冲)
 *
 * 流程: client 发 subscribe req → server 回此 Ack → server 开始周期推送
 *
 * 用途: 让 client 知道 server 接受的参数 + 段大小 + 容量, 用于 buffer 分配。 */
typedef struct {
    uint32_t magic;            /* SMC_PREVIEW_ACK_MAGIC = 0x50524146 */
    uint32_t version;          /* SMC_PREVIEW_VERSION */
    uint32_t max_per_tick;     /* server 单帧最多段数 (= PREVIEW_READ_MAX) */
    uint32_t seg_size_bytes;   /* sizeof(TrajectorySegment_t), client 据此分配缓冲 */
} SmcPreviewAck;

/* 推送帧头 (16B)
 *
 * 紧随其后是 seg_count 个 TrajectorySegment_t。
 * crc32 覆盖范围: SmcPreviewFrameHeader[8..12] (即 seg_count 字段) + 整个 segments。
 * 即 crc32(header.seg_count + segments)。
 *
 * 注意: magic 和 version 不在 CRC 范围内 (常量, 不需校验)。
 *       校验 crc 时 client 取 header[8..12] 这 4 字节 + segments 重算。 */
typedef struct {
    uint32_t magic;            /* SMC_PREVIEW_MAGIC = 0x53524556 */
    uint32_t version;          /* SMC_PREVIEW_VERSION */
    uint32_t seg_count;        /* 本帧段数 (1..PREVIEW_READ_MAX) */
    uint32_t crc32;            /* CRC32(seg_count_field + segments_payload) */
} SmcPreviewFrameHeader;

#pragma pack(pop)

/* ---- API ---- */

/* @Context: Non-RealTime (rpc_server.c main, kernel_init 之后, accept 循环之前)
 * @Safe: 创建 accept_thread 后立即返回, 不阻塞主线程。
 * @return 0=成功; -1=socket/bind/listen/pthread_create 失败。
 *         失败时仅 9527 RPC + 9528 snapshot push 可用, 主流程不应 fatal。 */
int  rpc_preview_server_start(void);

/* @Context: Non-RealTime (SMC_Close 路径或 main 退出时)
 * @Safe: 关闭 listen fd, 所有 client_thread 在下次 send 失败后自然退出。 */
void rpc_preview_server_stop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RPC_PREVIEW_SERVER_H */
