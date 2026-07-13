#ifndef SNAPSHOT_SUBSCRIBER_H
#define SNAPSHOT_SUBSCRIBER_H

/* =====================================================================
 *  snapshot_subscriber.h  ——  P0-a 推送通道客户端 SDK (C++ 跨平台)
 *
 *  定位:
 *    给 UI / CAM / 数据采集端用的状态推送订阅客户端。connect 后 start,
 *    后台线程持续接收 SMC_Snapshot_t 帧, 校验 magic/version/CRC32 后
 *    调用户回调。回调在 recv_thread 中执行, 用户需保证线程安全
 *    (典型做法: 回调里 push 到一个 lock-free 队列, UI 线程消费)。
 *
 *  协议: 详见 inc/rpc_push_server.h
 *
 *  跨平台:
 *    Windows (Winsock2) / Linux (POSIX), 与 SmcControllerSdk.h 同风格。
 *    Windows 构造时 WSAStartup (引用计数, 多实例安全), 析构时 WSACleanup。
 *
 *  线程安全:
 *    单实例: Start/Stop/Connect 可在主线程调, 回调在 recv_thread 调。
 *    多实例: 每实例独立 socket + 独立 recv_thread, 互不影响。
 *
 *  用法示例:
 *    SnapshotSubscriber sub;
 *    sub.SetCallback([](const SMC_Snapshot_t& s) {
 *        printf("cycle=%u pos=(%.2f,%.2f,%.2f)\n",
 *               s.cycle, s.machine_pos[0], s.machine_pos[1], s.machine_pos[2]);
 *    });
 *    if (!sub.Connect("127.0.0.1", 9528, 60)) { return 1; }
 *    sub.Start();
 *    // ... 主线程做别的事 ...
 *    sub.Stop();
 * ===================================================================== */

#include <string>
#include <cstdint>
#include <atomic>
#include <thread>
#include <functional>
#include <mutex>

#include "rpc_push_server.h"   /* SMC_PUSH_PORT, SmcPushFrameHeader */
#include "snapshot_hub.h"      /* SMC_Snapshot_t */
#include "smc_protocol.h"      /* SMC_CMD_SUBSCRIBE, SmcReqHeader */

/* 跨平台 socket 句柄类型 (与 SmcControllerSdk.h 一致) */
#ifdef _WIN32
    #ifndef _WIN32_LEAN_AND_MEAN
        #define _WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using snap_socket_t   = SOCKET;
    #define SNAP_INVALID_SOCKET  INVALID_SOCKET
    #define SNAP_CLOSE_SOCKET(s) closesocket(s)
    #define SNAP_SHUT_RDWR       SD_BOTH
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    using snap_socket_t   = int;
    #define SNAP_INVALID_SOCKET  (-1)
    #define SNAP_CLOSE_SOCKET(s) close(s)
    #define SNAP_SHUT_RDWR       SHUT_RDWR
#endif

class SnapshotSubscriber
{
public:
    /* 帧到达回调 (在 recv_thread 中调用, 必须线程安全 + 非阻塞) */
    using Callback = std::function<void(const SMC_Snapshot_t&)>;

    /* 错误回调 (连接断开 / CRC 持续失败 / 协议错时调) */
    using ErrorCallback = std::function<void(const std::string& reason)>;

    SnapshotSubscriber();
    ~SnapshotSubscriber();

    /* 禁拷贝 (内含 socket fd + 线程) */
    SnapshotSubscriber(const SnapshotSubscriber&)            = delete;
    SnapshotSubscriber& operator=(const SnapshotSubscriber&) = delete;

    /* ---------- 连接管理 ---------- */

    /* 连接推送端口 + 发送 subscribe req
     *   ip:       CNC Core 主机 (WSL2 场景通常 127.0.0.1 或 WSL2 IP)
     *   port:     默认 SMC_PUSH_PORT (9528)
     *   freq_hz:  订阅频率 [1, 200], 越界服务端用默认 60
     *   返回 true=subscribe req 已发送, 待 Start() 启动接收线程 */
    bool Connect(const std::string& ip,
                 int port = SMC_PUSH_PORT,
                 int freq_hz = SMC_PUSH_FREQ_DEFAULT);

    /* 主动断开 + 停止接收线程 */
    void Disconnect();

    bool IsConnected() const;

    /* ---------- 回调注册 (Connect 前调) ---------- */
    void SetCallback(Callback cb);
    void SetErrorCallback(ErrorCallback cb);

    /* ---------- 接收线程控制 ---------- */

    /* 启动 recv_thread, 开始接收帧并调回调。
     * 必须在 Connect + SetCallback 之后调。
     * 返回 false=线程创建失败 */
    bool Start();

    /* 停止 recv_thread (shutdown socket 解除 recv 阻塞) + join */
    void Stop();

    bool IsRunning() const { return running_.load(std::memory_order_acquire) != 0; }

    /* ---------- 统计 ---------- */
    struct Stats {
        uint64_t frames;       /* 成功收到的帧数 */
        uint64_t crc_fail;     /* CRC 校验失败的帧数 (丢弃) */
        uint64_t seq_gaps;     /* snapshot_seq 不连续次数 (丢帧检测) */
        uint64_t bytes_recv;   /* 累计接收字节数 (含帧头) */
    };
    Stats GetStats() const;

private:
    /* recv_thread 入口 */
    void RecvLoop();

    /* 阻塞收满 n 字节, 返回 0=成功 -1=对端关闭/错误 */
    int  RecvAll(void* buf, size_t n);

    /* CRC32 (与服务端 rpc_push_server.c 一致, zlib 兼容) */
    static uint32_t Crc32Bufs(const void* b1, size_t n1,
                               const void* b2, size_t n2);

    snap_socket_t       sock_fd_;
    Callback            cb_;
    ErrorCallback       err_cb_;

    std::atomic<int>    running_;
    std::thread         recv_thread_;

    /* 统计 */
    std::atomic<uint64_t> frames_;
    std::atomic<uint64_t> crc_fail_;
    std::atomic<uint64_t> seq_gaps_;
    std::atomic<uint64_t> bytes_recv_;
    uint64_t              last_seq_;
};

#endif /* SNAPSHOT_SUBSCRIBER_H */
