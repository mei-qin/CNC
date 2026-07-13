#ifndef PREVIEW_SUBSCRIBER_H
#define PREVIEW_SUBSCRIBER_H

/* =====================================================================
 *  preview_subscriber.h  ——  P0-b v1 段流推送客户端 SDK (C++ 跨平台)
 *
 *  定位:
 *    给 UI / CAM / 数据采集端用的段流订阅客户端。connect 后 start,
 *    后台线程持续接收 SmcPreviewFrameHeader + N × TrajectorySegment_t 帧,
 *    校验 magic/version/CRC32 后调用户回调 (批量段)。
 *
 *  与 SnapshotSubscriber 的差异:
 *    - 端口 9529 (vs 9528)
 *    - 回调签名: 批量段 (const TrajectorySegment_t* + count) vs 单一 snapshot
 *    - 段流是历史重放型 (晚到 client 从 from_seq=0 拿全部)
 *    - 帧格式: header + N × segment vs header + 1 snapshot
 *
 *  用法示例:
 *    PreviewSubscriber sub;
 *    sub.SetCallback([](const TrajectorySegment_t* segs, size_t n) {
 *        for (size_t i = 0; i < n; i++) {
 *            printf("seg_id=%llu line=%d type=%u\n",
 *                   segs[i].seg_id, segs[i].line_no, segs[i].motion_type);
 *        }
 *    });
 *    if (!sub.Connect("127.0.0.1", 9529, 60, 0)) return 1;   // from_seq=0 拿全部历史
 *    sub.Start();
 * ===================================================================== */

#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

#include "rpc_preview_server.h"   /* SMC_PREVIEW_PORT, SmcPreviewFrameHeader, SmcPreviewAck */
#include "preview_streamer.h"     /* PREVIEW_READ_MAX, SMC_PREVIEW_* */
#include "axis_cfg.h"             /* TrajectorySegment_t */
#include "smc_protocol.h"         /* SMC_CMD_PREVIEW_SUBSCRIBE, SmcReqHeader */

/* 跨平台 socket 句柄 (与 SnapshotSubscriber 同风格) */
#ifdef _WIN32
    #ifndef _WIN32_LEAN_AND_MEAN
        #define _WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using pv_socket_t   = SOCKET;
    #define PV_INVALID_SOCKET  INVALID_SOCKET
    #define PV_CLOSE_SOCKET(s) closesocket(s)
    #define PV_SHUT_RDWR       SD_BOTH
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    using pv_socket_t   = int;
    #define PV_INVALID_SOCKET  (-1)
    #define PV_CLOSE_SOCKET(s) close(s)
    #define PV_SHUT_RDWR       SHUT_RDWR
#endif

class PreviewSubscriber
{
public:
    /* 段批量到达回调 (在 recv_thread 中调用, 必须线程安全 + 非阻塞)
     * segs 生存期: 仅在回调期间有效, 回调返回后会被复用, 用户需 memcpy 保存 */
    using SegmentCallback = std::function<void(const TrajectorySegment_t* segs, size_t count)>;

    using ErrorCallback = std::function<void(const std::string& reason)>;

    PreviewSubscriber();
    ~PreviewSubscriber();

    /* 禁拷贝 */
    PreviewSubscriber(const PreviewSubscriber&)            = delete;
    PreviewSubscriber& operator=(const PreviewSubscriber&) = delete;

    /* ---------- 连接管理 ---------- */

    /* 连接 9529 + 发送 subscribe req
     *   ip:        CNC Core 主机
     *   port:      默认 SMC_PREVIEW_PORT (9529)
     *   freq_hz:   订阅频率 [1, 200], 越界服务端用默认 60
     *   from_seq:  起始 seq, 0=拿全部历史 (推荐), N=从段 N 开始 (跳过历史)
     *   返回 true=ack 已收到, 服务端即将推送 */
    bool Connect(const std::string& ip,
                 int port = SMC_PREVIEW_PORT,
                 int freq_hz = SMC_PREVIEW_FREQ_DEFAULT,
                 uint64_t from_seq = 0);

    void Disconnect();
    bool IsConnected() const;

    /* ---------- 回调注册 ---------- */
    void SetCallback(SegmentCallback cb);
    void SetErrorCallback(ErrorCallback cb);

    /* ---------- 接收线程控制 ---------- */
    bool Start();
    void Stop();
    bool IsRunning() const { return running_.load(std::memory_order_acquire) != 0; }

    /* ---------- 统计 ---------- */
    struct Stats {
        uint64_t frames;       /* 收到的帧数 */
        uint64_t segments;     /* 收到的段总数 */
        uint64_t crc_fail;     /* CRC 失败次数 */
        uint64_t bytes_recv;   /* 累计字节 */
        uint64_t last_seg_id;  /* 最新段的 seg_id */
    };
    Stats GetStats() const;

private:
    void RecvLoop();
    int  RecvAll(void* buf, size_t n);

    static uint32_t Crc32Bufs(const void* b1, size_t n1,
                               const void* b2, size_t n2);

    pv_socket_t       sock_fd_;
    SegmentCallback   cb_;
    ErrorCallback     err_cb_;

    std::atomic<int>    running_;
    std::thread         recv_thread_;

    /* 统计 */
    std::atomic<uint64_t> frames_;
    std::atomic<uint64_t> segments_;
    std::atomic<uint64_t> crc_fail_;
    std::atomic<uint64_t> bytes_recv_;
    std::atomic<uint64_t> last_seg_id_;
};

#endif /* PREVIEW_SUBSCRIBER_H */
