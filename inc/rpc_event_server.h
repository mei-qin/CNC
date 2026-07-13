#ifndef RPC_EVENT_SERVER_H
#define RPC_EVENT_SERVER_H

/* =====================================================================
 *  rpc_event_server.h  ——  P1-b 事件流推送服务端 (端口 9530)
 *
 *  定位:
 *    独立于 9527 RPC (req-resp) / 9528 snapshot push / 9529 preview push 的
 *    事件流推送通道。EventLogger 收集 11 个 alarm 触发点 + 程序生命周期事件,
 *    本服务端 accept 多个 UI client, 每客户端一个线程按订阅 freq + from_seq
 *    拉取事件批量推送。UI 据此实现"报警历史窗口" + 故障诊断。
 *
 *  协议 (与 9529 preview 同模式):
 *    1. client TCP connect 9530
 *    2. client 发 SmcReqHeader{cmd=SMC_CMD_EVENT_SUBSCRIBE, data_len=12} +
 *                   {int32 freq_hz, uint64 from_seq}
 *    3. server 回 SmcEventAck (16B)
 *    4. server 按 freq_hz 周期性推 [SmcEventFrameHeader + N × SmcEvent_t]
 *       N ≤ EVENT_READ_MAX (32)
 *
 *  跨平台: 同 rpc_preview_server.h, 头文件只声明常量/struct/函数原型。
 * ===================================================================== */

#include <stdint.h>
#include "event_logger.h"   /* SmcEvent_t, EVENT_READ_MAX, SMC_EVENT_* */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 推送通道端口 (9527 RPC, 9528 snapshot, 9529 preview, 9530 event) ---- */
#define SMC_EVENT_PORT      9530
#define SMC_EVENT_BACKLOG   8

/* ---- 客户端订阅频率上下限 ---- */
#define SMC_EVENT_FREQ_MIN     1
#define SMC_EVENT_FREQ_MAX     60       /* 事件比段少, 上限 60Hz 够 */
#define SMC_EVENT_FREQ_DEFAULT 10       /* 默认 10Hz, 报警突发可每帧多事件 */

/* ---- API ---- */
int  rpc_event_server_start(void);
void rpc_event_server_stop(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* RPC_EVENT_SERVER_H */
