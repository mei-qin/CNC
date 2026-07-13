/* =====================================================================
 *  preview_subscriber.cpp  ——  P0-b v1 段流推送客户端 SDK 实现
 *
 *  详见 rpc/preview_subscriber.h 头注释。
 * ===================================================================== */

#include "preview_subscriber.h"

#include <cstring>
#include <cstdio>
#include <vector>

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
 *  WSAStartup 引用计数 (Windows)
 * ===================================================================== */
#ifdef _WIN32
static std::atomic<int> g_pv_wsa_refcount{0};
static void wsa_startup() {
    if (g_pv_wsa_refcount.fetch_add(1, std::memory_order_relaxed) == 0) {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    }
}
static void wsa_cleanup() {
    if (g_pv_wsa_refcount.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        WSACleanup();
    }
}
#else
static void wsa_startup() {}
static void wsa_cleanup() {}
#endif

/* =====================================================================
 *  CRC32 (与服务端 rpc_preview_server.c 一致, zlib 兼容)
 * ===================================================================== */
struct PvCrc32Table {
    uint32_t data[256];
    PvCrc32Table() {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) {
                c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            }
            data[i] = c;
        }
    }
};
static const PvCrc32Table& get_pv_crc32_table() {
    static PvCrc32Table t;
    return t;
}

uint32_t PreviewSubscriber::Crc32Bufs(const void* b1, size_t n1,
                                        const void* b2, size_t n2)
{
    const PvCrc32Table& tbl = get_pv_crc32_table();
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
PreviewSubscriber::PreviewSubscriber()
    : sock_fd_(PV_INVALID_SOCKET),
      running_(0),
      frames_(0), segments_(0), crc_fail_(0), bytes_recv_(0), last_seg_id_(0)
{
    wsa_startup();
}

PreviewSubscriber::~PreviewSubscriber()
{
    Stop();
    Disconnect();
    wsa_cleanup();
}

/* =====================================================================
 *  连接管理
 * ===================================================================== */
bool PreviewSubscriber::Connect(const std::string& ip, int port, int freq_hz, uint64_t from_seq)
{
    if (sock_fd_ != PV_INVALID_SOCKET) {
        Disconnect();
    }

    sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ == PV_INVALID_SOCKET) {
        if (err_cb_) err_cb_("socket() failed");
        return false;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
        if (err_cb_) err_cb_("inet_pton failed for " + ip);
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }

    if (connect(sock_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        if (err_cb_) err_cb_("connect failed to " + ip + ":" + std::to_string(port));
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }

    /* 发送 subscribe req: SmcReqHeader{cmd, data_len=12} + int32 freq + uint64 from_seq */
    SmcReqHeader req_hdr;
    req_hdr.cmd_type = SMC_CMD_PREVIEW_SUBSCRIBE;
    req_hdr.data_len = 12;   /* int32 freq + uint64 from_seq */

    /* 注意字节序: 结构体打包后 little-endian 一致 (两端 x86 LE) */
    if (send(sock_fd_, (const char*)&req_hdr, sizeof(req_hdr), 0) != sizeof(req_hdr) ||
        send(sock_fd_, (const char*)&freq_hz, sizeof(int32_t), 0) != sizeof(int32_t) ||
        send(sock_fd_, (const char*)&from_seq, sizeof(uint64_t), 0) != sizeof(uint64_t)) {
        if (err_cb_) err_cb_("send subscribe req failed");
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }

    /* 读 SmcPreviewAck */
    SmcPreviewAck ack;
    if (RecvAll(&ack, sizeof(ack)) != 0) {
        if (err_cb_) err_cb_("recv SmcPreviewAck failed");
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }
    if (ack.magic != SMC_PREVIEW_ACK_MAGIC) {
        if (err_cb_) err_cb_("ack magic mismatch: " + std::to_string(ack.magic));
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }
    if (ack.version != SMC_PREVIEW_VERSION) {
        if (err_cb_) err_cb_("ack version mismatch: " + std::to_string(ack.version) +
                             " vs client " + std::to_string(SMC_PREVIEW_VERSION));
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }
    /* ack.seg_size_bytes 客户端用于校验服务端 sizeof(TrajectorySegment_t) 与本地一致 */
    if (ack.seg_size_bytes != sizeof(TrajectorySegment_t)) {
        if (err_cb_) err_cb_("seg_size mismatch: server=" + std::to_string(ack.seg_size_bytes) +
                             " client=" + std::to_string(sizeof(TrajectorySegment_t)));
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
        return false;
    }

    return true;
}

void PreviewSubscriber::Disconnect()
{
    if (sock_fd_ != PV_INVALID_SOCKET) {
        shutdown(sock_fd_, PV_SHUT_RDWR);
        PV_CLOSE_SOCKET(sock_fd_);
        sock_fd_ = PV_INVALID_SOCKET;
    }
}

bool PreviewSubscriber::IsConnected() const
{
    return sock_fd_ != PV_INVALID_SOCKET;
}

/* =====================================================================
 *  回调注册
 * ===================================================================== */
void PreviewSubscriber::SetCallback(SegmentCallback cb) { cb_ = std::move(cb); }
void PreviewSubscriber::SetErrorCallback(ErrorCallback cb) { err_cb_ = std::move(cb); }

/* =====================================================================
 *  接收线程控制
 * ===================================================================== */
bool PreviewSubscriber::Start()
{
    if (running_.load(std::memory_order_acquire) != 0) return true;
    if (sock_fd_ == PV_INVALID_SOCKET) {
        if (err_cb_) err_cb_("Start called before Connect");
        return false;
    }

    running_.store(1, std::memory_order_release);
    try {
        recv_thread_ = std::thread(&PreviewSubscriber::RecvLoop, this);
    } catch (const std::exception& e) {
        running_.store(0, std::memory_order_release);
        if (err_cb_) err_cb_(std::string("thread create failed: ") + e.what());
        return false;
    }
    return true;
}

void PreviewSubscriber::Stop()
{
    if (running_.load(std::memory_order_acquire) == 0) return;
    running_.store(0, std::memory_order_release);

    if (sock_fd_ != PV_INVALID_SOCKET) {
        shutdown(sock_fd_, PV_SHUT_RDWR);
    }
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

PreviewSubscriber::Stats PreviewSubscriber::GetStats() const
{
    Stats s;
    s.frames      = frames_.load(std::memory_order_relaxed);
    s.segments    = segments_.load(std::memory_order_relaxed);
    s.crc_fail    = crc_fail_.load(std::memory_order_relaxed);
    s.bytes_recv  = bytes_recv_.load(std::memory_order_relaxed);
    s.last_seg_id = last_seg_id_.load(std::memory_order_relaxed);
    return s;
}

/* =====================================================================
 *  RecvAll: 阻塞收满 n 字节
 * ===================================================================== */
int PreviewSubscriber::RecvAll(void* buf, size_t n)
{
    uint8_t* p = (uint8_t*)buf;
    size_t got = 0;
    while (got < n) {
#ifdef _WIN32
        int r = recv(sock_fd_, (char*)(p + got), (int)(n - got), 0);
        if (r == 0) return -1;
        if (r < 0) return -1;
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
void PreviewSubscriber::RecvLoop()
{
    /* 最大单帧段数 = PREVIEW_READ_MAX (16), 每段 sizeof(TrajectorySegment_t) */
    std::vector<TrajectorySegment_t> seg_buf(PREVIEW_READ_MAX);

    while (running_.load(std::memory_order_acquire) != 0) {
        /* ---- 读帧头 16B ---- */
        SmcPreviewFrameHeader hdr;
        if (RecvAll(&hdr, sizeof(hdr)) != 0) {
            if (running_.load(std::memory_order_acquire) == 0) break;
            if (err_cb_) err_cb_("recv frame header failed, disconnected");
            break;
        }

        if (hdr.magic != SMC_PREVIEW_MAGIC) {
            if (err_cb_) err_cb_("magic mismatch: " + std::to_string(hdr.magic));
            break;
        }
        if (hdr.version != SMC_PREVIEW_VERSION) {
            if (err_cb_) err_cb_("version mismatch: " + std::to_string(hdr.version));
            break;
        }
        if (hdr.seg_count == 0 || hdr.seg_count > PREVIEW_READ_MAX) {
            if (err_cb_) err_cb_("seg_count out of range: " + std::to_string(hdr.seg_count));
            continue;
        }

        /* ---- 读 segments ---- */
        size_t seg_bytes = (size_t)hdr.seg_count * sizeof(TrajectorySegment_t);
        if (RecvAll(seg_buf.data(), seg_bytes) != 0) {
            if (err_cb_) err_cb_("recv segments failed, disconnected");
            break;
        }

        /* ---- CRC32 校验 (覆盖 seg_count 字段 + segments) ---- */
        uint32_t calc = Crc32Bufs(&hdr.seg_count, 4, seg_buf.data(), seg_bytes);
        if (calc != hdr.crc32) {
            crc_fail_.fetch_add(1, std::memory_order_relaxed);
            if (err_cb_) err_cb_("CRC mismatch: calc=" + std::to_string(calc) +
                                 " frame=" + std::to_string(hdr.crc32));
            continue;   /* 丢帧 */
        }

        frames_.fetch_add(1, std::memory_order_relaxed);
        segments_.fetch_add(hdr.seg_count, std::memory_order_relaxed);
        last_seg_id_.store(seg_buf[hdr.seg_count - 1].seg_id, std::memory_order_relaxed);

        /* ---- 调用户回调 ---- */
        if (cb_) {
            try {
                cb_(seg_buf.data(), hdr.seg_count);
            } catch (const std::exception& e) {
                if (err_cb_) err_cb_(std::string("callback threw: ") + e.what());
            } catch (...) {
                if (err_cb_) err_cb_("callback threw unknown exception");
            }
        }
    }

    running_.store(0, std::memory_order_release);
}
