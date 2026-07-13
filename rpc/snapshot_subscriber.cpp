/* =====================================================================
 *  snapshot_subscriber.cpp  ——  P0-a 推送通道客户端 SDK 实现
 *
 *  详见 rpc/snapshot_subscriber.h 头注释。
 *
 *  线程模型:
 *    主线程: Connect / SetCallback / Start / Stop
 *    recv_thread: RecvLoop (阻塞 recv, 收到帧调 cb_)
 *
 *  退出路径:
 *    Stop() 设 running_=0 → shutdown(sock, SHUT_RDWR) 解除 recv 阻塞 →
 *    RecvLoop 退出循环 → join
 * ===================================================================== */

#include "snapshot_subscriber.h"

#include <cstring>
#include <cstdio>

#ifdef _WIN32
    #ifndef _WIN32_LEAN_AND_MEAN
        #define _WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #include <errno.h>
#endif

/* =====================================================================
 *  WSAStartup 引用计数 (Windows, 多实例安全)
 * ===================================================================== */
#ifdef _WIN32
static std::atomic<int> g_wsa_refcount{0};
static void wsa_startup() {
    if (g_wsa_refcount.fetch_add(1, std::memory_order_relaxed) == 0) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    }
}
static void wsa_cleanup() {
    if (g_wsa_refcount.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        WSACleanup();
    }
}
#else
static void wsa_startup() {}
static void wsa_cleanup() {}
#endif

/* =====================================================================
 *  CRC32 表 (zlib 兼容, C++11 magic statics 线程安全初始化)
 * ===================================================================== */
struct Crc32Table {
    uint32_t data[256];
    Crc32Table() {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            data[i] = c;
        }
    }
};
static const Crc32Table& get_crc32_table() {
    static Crc32Table t;   /* C++11 magic statics: 首次调用线程安全初始化 */
    return t;
}

uint32_t SnapshotSubscriber::Crc32Bufs(const void* b1, size_t n1,
                                         const void* b2, size_t n2)
{
    const Crc32Table& tbl = get_crc32_table();
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t* p1 = (const uint8_t*)b1;
    for (size_t i = 0; i < n1; i++) {
        crc = tbl.data[(crc ^ p1[i]) & 0xFFu] ^ (crc >> 8);
    }
    if (b2 && n2) {
        const uint8_t* p2 = (const uint8_t*)b2;
        for (size_t i = 0; i < n2; i++) {
            crc = tbl.data[(crc ^ p2[i]) & 0xFFu] ^ (crc >> 8);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* =====================================================================
 *  构造 / 析构
 * ===================================================================== */
SnapshotSubscriber::SnapshotSubscriber()
    : sock_fd_(SNAP_INVALID_SOCKET),
      running_(0),
      frames_(0), crc_fail_(0), seq_gaps_(0), bytes_recv_(0),
      last_seq_(0)
{
    wsa_startup();
}

SnapshotSubscriber::~SnapshotSubscriber()
{
    Stop();
    Disconnect();
    wsa_cleanup();
}

/* =====================================================================
 *  连接管理
 * ===================================================================== */
bool SnapshotSubscriber::Connect(const std::string& ip, int port, int freq_hz)
{
    if (sock_fd_ != SNAP_INVALID_SOCKET) {
        /* 已连接, 先断开 */
        Disconnect();
    }

    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ == SNAP_INVALID_SOCKET) {
        if (err_cb_) err_cb_("socket() failed");
        return false;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
        if (err_cb_) err_cb_("inet_pton failed for " + ip);
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }

    if (connect(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        if (err_cb_) err_cb_("connect failed to " + ip + ":" + std::to_string(port));
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }

    /* 发送 subscribe req: SmcReqHeader{cmd, data_len=4} + int32_t freq */
    SmcReqHeader req_hdr;
    req_hdr.cmd_type = SMC_CMD_SUBSCRIBE;
    req_hdr.data_len = 4;
    int32_t freq = freq_hz;

    if (send(sock_fd_, (const char*)&req_hdr, sizeof(req_hdr), 0) != sizeof(req_hdr) ||
        send(sock_fd_, (const char*)&freq, sizeof(freq), 0) != sizeof(freq)) {
        if (err_cb_) err_cb_("send subscribe req failed");
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }

    /* Bug #7: 读 SubscribeAck, 验证 server 实际接受的 freq */
    SubscribeAck ack;
    if (RecvAll(&ack, sizeof(ack)) != 0) {
        if (err_cb_) err_cb_("recv SubscribeAck failed");
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }
    if (ack.magic != SMC_ACK_MAGIC) {
        if (err_cb_) err_cb_("ack magic mismatch: " + std::to_string(ack.magic));
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }
    if (ack.version != SMC_SNAPSHOT_VERSION) {
        if (err_cb_) err_cb_("ack version mismatch: " + std::to_string(ack.version) +
                             " vs client " + std::to_string(SMC_SNAPSHOT_VERSION));
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
        return false;
    }
    if ((int)ack.actual_freq_hz != freq_hz) {
        if (err_cb_) err_cb_("freq clamped by server: requested " +
                             std::to_string(freq_hz) + " actual " +
                             std::to_string(ack.actual_freq_hz));
    }

    return true;
}

void SnapshotSubscriber::Disconnect()
{
    if (sock_fd_ != SNAP_INVALID_SOCKET) {
        /* shutdown 让 RecvLoop 的 recv 立即返回, 然后 close */
        shutdown(sock_fd_, SNAP_SHUT_RDWR);
        SNAP_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = SNAP_INVALID_SOCKET;
    }
}

bool SnapshotSubscriber::IsConnected() const
{
    return sock_fd_ != SNAP_INVALID_SOCKET;
}

/* =====================================================================
 *  回调注册
 * ===================================================================== */
void SnapshotSubscriber::SetCallback(Callback cb) { cb_ = std::move(cb); }
void SnapshotSubscriber::SetErrorCallback(ErrorCallback cb) { err_cb_ = std::move(cb); }

/* =====================================================================
 *  接收线程控制
 * ===================================================================== */
bool SnapshotSubscriber::Start()
{
    if (running_.load(std::memory_order_acquire) != 0) return true;   /* 已启动 */
    if (sock_fd_ == SNAP_INVALID_SOCKET) {
        if (err_cb_) err_cb_("Start called before Connect");
        return false;
    }

    running_.store(1, std::memory_order_release);
    try {
        recv_thread_ = std::thread(&SnapshotSubscriber::RecvLoop, this);
    } catch (const std::exception& e) {
        running_.store(0, std::memory_order_release);
        if (err_cb_) err_cb_(std::string("thread create failed: ") + e.what());
        return false;
    }
    return true;
}

void SnapshotSubscriber::Stop()
{
    if (running_.load(std::memory_order_acquire) == 0) return;
    running_.store(0, std::memory_order_release);

    /* shutdown socket 让 RecvLoop 的 recv 立即返回 (不依赖 recv 超时) */
    if (sock_fd_ != SNAP_INVALID_SOCKET) {
        shutdown(sock_fd_, SNAP_SHUT_RDWR);
    }

    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

SnapshotSubscriber::Stats SnapshotSubscriber::GetStats() const
{
    Stats s;
    s.frames    = frames_.load(std::memory_order_relaxed);
    s.crc_fail  = crc_fail_.load(std::memory_order_relaxed);
    s.seq_gaps  = seq_gaps_.load(std::memory_order_relaxed);
    s.bytes_recv = bytes_recv_.load(std::memory_order_relaxed);
    return s;
}

/* =====================================================================
 *  RecvAll: 阻塞收满 n 字节
 * ===================================================================== */
int SnapshotSubscriber::RecvAll(void* buf, size_t n)
{
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n) {
#ifdef _WIN32
        int r = recv(sock_fd_, (char*)(p + got), (int)(n - got), 0);
        if (r == 0) return -1;             /* 对端关闭 */
        if (r < 0) return -1;              /* WSAEINTR 等不重试 (Stop 已 shutdown) */
#else
        ssize_t r = recv(sock_fd_, p + got, n - got, 0);
        if (r == 0) return -1;
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
#endif
        got += (size_t)r;
        bytes_recv_.fetch_add((size_t)r, std::memory_order_relaxed);
    }
    return 0;
}

/* =====================================================================
 *  RecvLoop: 接收线程主体
 * ===================================================================== */
void SnapshotSubscriber::RecvLoop()
{
    SmcPushFrameHeader hdr;
    SMC_Snapshot_t     snap;

    while (running_.load(std::memory_order_acquire) != 0) {
        /* ---- 读帧头 16B ---- */
        if (RecvAll(&hdr, sizeof(hdr)) != 0) {
            if (running_.load(std::memory_order_acquire) == 0) break;   /* Stop 触发 */
            if (err_cb_) err_cb_("recv header failed, disconnected");
            break;
        }

        if (hdr.magic != SMC_SNAPSHOT_MAGIC) {
            if (err_cb_) err_cb_("magic mismatch: " + std::to_string(hdr.magic));
            /* 帧同步丢失, 尝试跳过 payload 重新对齐 */
            if (hdr.payload_len > 0 && hdr.payload_len <= sizeof(SMC_Snapshot_t) * 2) {
                SMC_Snapshot_t dummy;
                RecvAll(&dummy, hdr.payload_len);
            }
            continue;
        }
        if (hdr.version != SMC_SNAPSHOT_VERSION) {
            if (err_cb_) err_cb_("version mismatch: " + std::to_string(hdr.version));
            break;
        }
        if (hdr.payload_len != sizeof(SMC_Snapshot_t)) {
            if (err_cb_) err_cb_("payload_len mismatch: " + std::to_string(hdr.payload_len) +
                                 " vs expected " + std::to_string(sizeof(SMC_Snapshot_t)));
            continue;
        }

        /* ---- 读 payload ---- */
        if (RecvAll(&snap, sizeof(snap)) != 0) {
            if (err_cb_) err_cb_("recv payload failed, disconnected");
            break;
        }

        /* ---- CRC32 校验 ---- */
        uint32_t calc = Crc32Bufs(&hdr.magic, 12, &snap, sizeof(snap));
        if (calc != hdr.crc32) {
            crc_fail_.fetch_add(1, std::memory_order_relaxed);
            if (err_cb_) err_cb_("CRC mismatch: calc=" + std::to_string(calc) +
                                 " frame=" + std::to_string(hdr.crc32));
            continue;   /* 丢帧, 不调 cb */
        }

        /* ---- 丢帧检测 (相邻 seq 应 +2) ---- */
        if (last_seq_ != 0 && snap.snapshot_seq != last_seq_ + 2) {
            seq_gaps_.fetch_add(1, std::memory_order_relaxed);
        }
        last_seq_ = snap.snapshot_seq;

        frames_.fetch_add(1, std::memory_order_relaxed);

        /* ---- 调用户回调 (在 recv_thread 中执行, 必须非阻塞) ---- */
        if (cb_) {
            try {
                cb_(snap);
            } catch (const std::exception& e) {
                if (err_cb_) err_cb_(std::string("callback threw: ") + e.what());
            } catch (...) {
                if (err_cb_) err_cb_("callback threw unknown exception");
            }
        }
    }

    running_.store(0, std::memory_order_release);
}
