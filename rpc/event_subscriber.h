#ifndef EVENT_SUBSCRIBER_H
#define EVENT_SUBSCRIBER_H

/* =====================================================================
 *  event_subscriber.h  ——  P1-b 事件流推送客户端 SDK (C++ 跨平台)
 *
 *  定位: UI / CAM / 故障诊断端订阅事件流。connect 后 start, 后台线程持续
 *    接收 SmcEventFrameHeader + N × SmcEvent_t 帧, 校验 magic/version/CRC32
 *    后调用户回调 (批量事件)。
 *
 *  与 PreviewSubscriber 同模式, 仅帧类型 (SmcEvent_t vs TrajectorySegment_t)
 *    和端口 (9530 vs 9529) 不同。
 *
 *  用法:
 *    EventSubscriber sub;
 *    sub.SetCallback([](const SmcEvent_t* evs, size_t n) {
 *        for (size_t i = 0; i < n; i++) {
 *            printf("[%llu] sev=%d src=%d code=0x%04X msg=%s\n",
 *                   evs[i].timestamp_ms, evs[i].severity, evs[i].source,
 *                   evs[i].code, evs[i].message);
 *        }
 *    });
 *    if (!sub.Connect("127.0.0.1", 9530, 10, 0)) return 1;  // from_seq=0 拿全部历史
 *    sub.Start();
 * ===================================================================== */

#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

#include "rpc_event_server.h"   /* SMC_EVENT_PORT, SMC_EVENT_FREQ_DEFAULT */
#include "event_logger.h"       /* SmcEvent_t, EVENT_READ_MAX, SMC_EVENT_* */
#include "smc_protocol.h"       /* SMC_CMD_EVENT_SUBSCRIBE, SmcReqHeader, SmcEventAck, SmcEventFrameHeader */

#ifdef _WIN32
    #ifndef _WIN32_LEAN_AND_MEAN
        #define _WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using ev_socket_t   = SOCKET;
    #define EV_INVALID_SOCKET  INVALID_SOCKET
    #define EV_CLOSE_SOCKET(s) closesocket(s)
    #define EV_SHUT_RDWR       SD_BOTH
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    using ev_socket_t   = int;
    #define EV_INVALID_SOCKET  (-1)
    #define EV_CLOSE_SOCKET(s) close(s)
    #define EV_SHUT_RDWR       SHUT_RDWR
#endif

class EventSubscriber
{
public:
    using EventCallback = std::function<void(const SmcEvent_t* evs, size_t count)>;
    using ErrorCallback = std::function<void(const std::string& reason)>;

    EventSubscriber();
    ~EventSubscriber();

    EventSubscriber(const EventSubscriber&)            = delete;
    EventSubscriber& operator=(const EventSubscriber&) = delete;

    /* 连接 9530 + 发送 subscribe req
     *   ip:        CNC Core 主机
     *   port:      默认 SMC_EVENT_PORT (9530)
     *   freq_hz:   订阅频率 [1, 60], 越界服务端用默认 10
     *   from_seq:  起始 seq, 0=拿全部历史 (推荐), N=跳过历史 */
    bool Connect(const std::string& ip,
                 int port = SMC_EVENT_PORT,
                 int freq_hz = SMC_EVENT_FREQ_DEFAULT,
                 uint64_t from_seq = 0);

    void Disconnect();
    bool IsConnected() const;

    void SetCallback(EventCallback cb);
    void SetErrorCallback(ErrorCallback cb);

    bool Start();
    void Stop();
    bool IsRunning() const { return running_.load(std::memory_order_acquire) != 0; }

    struct Stats {
        uint64_t frames;
        uint64_t events;
        uint64_t crc_fail;
        uint64_t bytes_recv;
        uint64_t last_event_seq;
    };
    Stats GetStats() const;

private:
    void RecvLoop();
    int  RecvAll(void* buf, size_t n);

    static uint32_t Crc32Bufs(const void* b1, size_t n1,
                               const void* b2, size_t n2);

    ev_socket_t       sock_fd_;
    EventCallback     cb_;
    ErrorCallback     err_cb_;

    std::atomic<int>    running_;
    std::thread         recv_thread_;

    std::atomic<uint64_t> frames_;
    std::atomic<uint64_t> events_;
    std::atomic<uint64_t> crc_fail_;
    std::atomic<uint64_t> bytes_recv_;
    std::atomic<uint64_t> last_event_seq_;
};

#endif /* EVENT_SUBSCRIBER_H */
