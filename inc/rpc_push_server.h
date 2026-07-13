#ifndef RPC_PUSH_SERVER_H
#define RPC_PUSH_SERVER_H

/* =====================================================================
 *  rpc_push_server.h  ——  P0-a 推送通道服务端 (端口 9528)
 *
 *  定位:
 *    独立于 9527 RPC (req-resp) 的状态推送通道。RT 线程把 SMC_Snapshot_t
 *    写入 SnapshotHub seqlock, 本服务端 accept 多个 UI client, 每客户端
 *    一个线程按订阅 freq (最高 200Hz) 拉取最新帧并 TCP 推送。
 *
 *  协议:
 *    1. client TCP connect 9528
 *    2. client 发 SmcReqHeader{cmd_type=SMC_CMD_SUBSCRIBE, data_len=4} + int32_t freq_hz
 *    3. server 立即推一帧 (force_log 借鉴, 不等下个 tick)
 *    4. server 按 freq_hz 周期性推 SmcPushFrameHeader + SMC_Snapshot_t
 *    5. client 可随时关闭, server 检测到 send 返回 -1 时清理线程
 *
 *  帧布局 (无分片, 单帧 ~272B):
 *    [SmcPushFrameHeader 16B] [SMC_Snapshot_t ~200B]
 *
 *  线程模型:
 *    accept_thread (后台) - accept 循环, 每来 client 创建 client_thread
 *    client_thread (每客户端) - ReadLatest + send_frame + usleep(freq)
 *
 *  跨平台:
 *    本头文件只声明 SMC_PUSH_PORT / SmcPushFrameHeader / 函数原型, 不含
 *    任何平台特定类型。Linux 服务端实现于 rpc/rpc_push_server.c,
 *    Windows C++ SDK 可包含本头文件解析帧。
 * ===================================================================== */

#include <stdint.h>
#include "snapshot_hub.h"   /* SMC_Snapshot_t, SMC_SNAPSHOT_MAGIC/VERSION */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 推送通道端口 (9527 是现有 RPC, 9528 是推送) ---- */
#define SMC_PUSH_PORT      9528
#define SMC_PUSH_BACKLOG   8

/* ---- 客户端订阅频率上下限 ---- */
#define SMC_PUSH_FREQ_MIN  1
#define SMC_PUSH_FREQ_MAX  200
#define SMC_PUSH_FREQ_DEFAULT  60

/* ---- Ack 帧 magic (区分 SubscribeAck 与 SmcPushFrameHeader) ----
 * 与 SMC_SNAPSHOT_MAGIC (0x534E4150) 同族, 仅末字节不同 (0x4B vs 0x50),
 * 便于 client 用同一个 16B 读缓冲区, 看 magic 路由解析。 */
#define SMC_ACK_MAGIC      0x534E414Bu   /* "SNAK" 同族标识 */

#pragma pack(push, 1)

/* 推送帧头 (16B)
 * 紧随其后是 SMC_Snapshot_t payload (长度 = payload_len 字段值)。
 * crc32 覆盖范围: SmcPushFrameHeader 前 12 字节 (magic+version+payload_len)
 *                 + 整个 payload。即 crc32(header[0..12] + payload)。
 * 客户端校验: 重算 crc32 比对 crc 字段, 不匹配则丢帧并告警。 */
typedef struct {
    uint32_t magic;         /* SMC_SNAPSHOT_MAGIC = 0x534E4150 ("SNAP") */
    uint32_t version;       /* SMC_SNAPSHOT_VERSION, 字段表变更必须 bump */
    uint32_t payload_len;   /* sizeof(SMC_Snapshot_t), 客户端按此 recvn */
    uint32_t crc32;         /* CRC32(header[0..12] + payload) */
} SmcPushFrameHeader;

/* SubscribeAck 帧 (16B, 与 SmcPushFrameHeader 同尺寸)
 *
 * 流程: client 发 SmcReqHeader{cmd=SUBSCRIBE, freq} → server 立即回此 Ack
 *       → server 开始周期性推 SmcPushFrameHeader + SMC_Snapshot_t
 *
 * 用途: 让 client 知道 server 实际接受的 freq (可能被 clamp 到默认),
 *       区分"接受原值"和"被 clamp"。若 freq 被修正, actual_freq_hz != 请求值。
 *       client 不应答, 直接进入帧循环。 */
typedef struct {
    uint32_t magic;          /* SMC_ACK_MAGIC = 0x534E414B */
    uint32_t version;        /* SMC_SNAPSHOT_VERSION, 与后续推送帧版本一致 */
    uint32_t actual_freq_hz; /* server 实际接受的 freq (1..200, 可能被 clamp) */
    uint32_t reserved;       /* 0, 未来扩展 (server capability bitmap 等) */
} SubscribeAck;

#pragma pack(pop)

/* ---- API ---- */

/* @Context: Non-RealTime (rpc_server.c main, kernel_init 之后, accept 循环之前)
 * @Safe: 创建 accept_thread 后立即返回, 不阻塞主线程。
 * @return 0=成功; -1=socket/bind/listen/pthread_create 失败。
 *         失败时仅 9527 RPC 可用, 主流程不应 fatal。 */
int  rpc_push_server_start(void);

/* @Context: Non-RealTime (SMC_Close 路径或 main 退出时)
 * @Safe: 关闭 listen fd, 所有 client_thread 在下次 send 失败后自然退出。
 *        不强制 join, 避免卡死主退出流程。 */
void rpc_push_server_stop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RPC_PUSH_SERVER_H */
