/* =====================================================================
 *  rpc_push_server.c  ——  P0-a 推送通道服务端实现 (Linux, 端口 9528)
 *
 *  详见 inc/rpc_push_server.h 头注释。
 *
 *  线程模型:
 *    accept_thread (单实例) - bind/listen 9528, accept 循环
 *      每来 client: recv subscribe req → 创建 ClientContext_t → pthread_create
 *    client_thread (每客户端) - 立即推一帧 → 循环 ReadLatest + send + usleep(freq)
 *
 *  资源管理:
 *    ClientContext_t 由 client_thread free 自己 (pthread_detach)
 *    listen_fd 由 rpc_push_server_stop 关闭, accept_thread 自然退出
 *
 *  失败模式 (详见 plan):
 *    send 返回 -1 (EPIPE/ECONNRESET/SO_SNDTIMEO) → client_thread 退出
 *    recv subscribe req 5s 超时 → close fd
 *    SnapshotHub_ReadLatest 持续返回 -1 → 用旧帧 best-effort 推送
 *
 *  CRC32:
 *    自实现表驱动 (多项式 0xEDB88320, 与 zlib/zip/png 兼容)。
 *    避免引入 -lz 依赖, Makefile 不需改 LDFLAGS。
 * ===================================================================== */

#include "axis_cfg.h"       /* AXIS_NUM 权威值 (在 snapshot_hub.h 之前包含, 避免触发 fallback) */
#include "rpc_push_server.h"
#include "snapshot_hub.h"
#include "smc_protocol.h"   /* SMC_CMD_SUBSCRIBE, SmcReqHeader */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* =====================================================================
 *  CRC32 (zlib 兼容, 表驱动)
 * ===================================================================== */
static uint32_t g_crc32_table[256];
static int      g_crc32_initialized = 0;

static void crc32_init_once(void)
{
    if (g_crc32_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        g_crc32_table[i] = c;
    }
    g_crc32_initialized = 1;
}

/* 双缓冲 CRC32: 覆盖 b1[0..n1) + b2[0..n2), 与 zlib crc32 兼容 */
static uint32_t crc32_bufs(const void *b1, size_t n1,
                            const void *b2, size_t n2)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p1 = (const uint8_t *)b1;
    for (size_t i = 0; i < n1; i++) {
        crc = g_crc32_table[(crc ^ p1[i]) & 0xFFu] ^ (crc >> 8);
    }
    if (b2 && n2) {
        const uint8_t *p2 = (const uint8_t *)b2;
        for (size_t i = 0; i < n2; i++) {
            crc = g_crc32_table[(crc ^ p2[i]) & 0xFFu] ^ (crc >> 8);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* =====================================================================
 *  TCP IO 辅助 (与 rpc_server.c:54-86 同名函数等价, 此处独立实现避免修改
 *  原 static 函数可见性)
 * ===================================================================== */
static int recv_all(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    size_t  got = 0;
    while (got < n) {
        ssize_t r = recv(fd, p + got, n - got, 0);
        if (r == 0) return -1;             /* 对端正常关闭 */
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += (size_t)r;
    }
    return 0;
}

static int send_all(int fd, const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t  sent = 0;
    while (sent < n) {
        ssize_t s = send(fd, p + sent, n - sent, 0);
        if (s < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (s == 0) return -1;
        sent += (size_t)s;
    }
    return 0;
}

/* =====================================================================
 *  帧发送: 构造 SmcPushFrameHeader + 计算 CRC32 + 发送 header + payload
 * ===================================================================== */
static int send_frame(int fd, const SMC_Snapshot_t *snap)
{
    SmcPushFrameHeader hdr;
    hdr.magic       = SMC_SNAPSHOT_MAGIC;
    hdr.version     = SMC_SNAPSHOT_VERSION;
    hdr.payload_len = (uint32_t)sizeof(SMC_Snapshot_t);
    /* CRC32 覆盖 header[0..12] (magic+version+payload_len) + payload */
    hdr.crc32 = crc32_bufs(&hdr.magic, 12, snap, sizeof(SMC_Snapshot_t));

    if (send_all(fd, &hdr, sizeof(hdr)) != 0) return -1;
    if (send_all(fd, snap,  sizeof(SMC_Snapshot_t)) != 0) return -1;
    return 0;
}

/* =====================================================================
 *  Client 上下文 + 推送线程
 * ===================================================================== */
typedef struct {
    int      fd;
    int      freq_hz;
    _Atomic int running;
    pthread_t tid;
} ClientContext_t;

/* @Context: Non-RealTime Background Thread (每客户端一个)
 * @Safe: 可阻塞 send_all (SO_SNDTIMEO=2s 上限)。pthread_detach 自我清理。 */
static void *client_thread(void *arg)
{
    ClientContext_t *ctx = (ClientContext_t *)arg;
    SMC_Snapshot_t snap;

    /* force_log 借鉴: 立即推一帧 (不等下个 tick), client 连上的瞬间就看到当前状态 */
    if (SnapshotHub_ReadLatest(&snap) == 0) {
        if (send_frame(ctx->fd, &snap) < 0) {
            printf("[push] client fd=%d 首帧发送失败, 断开\n", ctx->fd);
            goto cleanup;
        }
    }

    while (atomic_load_explicit(&ctx->running, memory_order_acquire)) {
        /* ReadLatest 持续返回 -1 时用旧帧 best-effort 推送 (写者持续忙极罕见) */
        SnapshotHub_ReadLatest(&snap);
        if (send_frame(ctx->fd, &snap) < 0) {
            printf("[push] client fd=%d send 失败, 断开\n", ctx->fd);
            goto cleanup;
        }
        usleep((useconds_t)(1000000 / ctx->freq_hz));
    }

cleanup:
    atomic_store_explicit(&ctx->running, 0, memory_order_release);
    close(ctx->fd);
    printf("[push] client fd=%d disconnected, freq=%d\n", ctx->fd, ctx->freq_hz);
    free(ctx);
    return NULL;
}

/* =====================================================================
 *  Accept 主线程
 * ===================================================================== */
static int              g_push_listen_fd = -1;
static _Atomic int      g_push_running = 0;
static pthread_t        g_push_accept_tid;

/* @Context: Non-RealTime Background Thread (单实例) */
static void *accept_thread(void *arg)
{
    (void)arg;

    crc32_init_once();

    g_push_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_push_listen_fd < 0) {
        perror("[push] socket");
        return NULL;
    }
    int opt = 1;
    setsockopt(g_push_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);   /* WSL2 ↔ Windows 跨边界必需 */
    addr.sin_port        = htons(SMC_PUSH_PORT);

    if (bind(g_push_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[push] bind");
        close(g_push_listen_fd);
        g_push_listen_fd = -1;
        return NULL;
    }
    if (listen(g_push_listen_fd, SMC_PUSH_BACKLOG) < 0) {
        perror("[push] listen");
        close(g_push_listen_fd);
        g_push_listen_fd = -1;
        return NULL;
    }
    printf("[push] listening on 0.0.0.0:%d ...\n", SMC_PUSH_PORT);

    while (atomic_load_explicit(&g_push_running, memory_order_acquire)) {
        int cfd = accept(g_push_listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            /* listen_fd 被关闭 (rpc_push_server_stop) → EBADF/ENOTSOCK, 退出循环 */
            if (!atomic_load_explicit(&g_push_running, memory_order_acquire)) break;
            perror("[push] accept");
            usleep(1000);   /* 防止 accept 紧密失败烧 CPU */
            continue;
        }
        printf("[push] client connected, fd=%d\n", cfd);

        /* 接收 subscribe req: SmcReqHeader{cmd_type, data_len} + (可选) int32_t freq */
        struct timeval rtv = { .tv_sec = 5, .tv_usec = 0 };   /* 5s 接收超时 */
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));

        SmcReqHeader req_hdr;
        int32_t      freq_hz = SMC_PUSH_FREQ_DEFAULT;
        int          parse_ok = 0;

        if (recv_all(cfd, &req_hdr, sizeof(req_hdr)) == 0) {
            if (req_hdr.cmd_type == SMC_CMD_SUBSCRIBE) {
                if (req_hdr.data_len >= 4) {
                    if (recv_all(cfd, &freq_hz, sizeof(int32_t)) == 0) {
                        parse_ok = 1;
                    }
                } else if (req_hdr.data_len == 0) {
                    /* 用默认 freq */
                    parse_ok = 1;
                } else {
                    /* data_len 1-3, 异常, 读完丢弃 */
                    uint8_t buf[4];
                    recv_all(cfd, buf, req_hdr.data_len);
                    parse_ok = 1;   /* 容忍, 用默认 freq */
                }
            } else {
                printf("[push] client fd=%d cmd_type=0x%04X 非 SUBSCRIBE, 断开\n",
                       cfd, req_hdr.cmd_type);
            }
        }

        if (!parse_ok) {
            printf("[push] client fd=%d subscribe req 接收失败/超时, 断开\n", cfd);
            close(cfd);
            continue;
        }

        /* freq 范围校验, 越界用默认 */
        if (freq_hz < SMC_PUSH_FREQ_MIN || freq_hz > SMC_PUSH_FREQ_MAX) {
            printf("[push] client fd=%d freq=%d 越界, 用默认 %d\n",
                   cfd, freq_hz, SMC_PUSH_FREQ_DEFAULT);
            freq_hz = SMC_PUSH_FREQ_DEFAULT;
        }

        /* send 超时 2s 防客户端不读导致 send 阻塞卡死线程 */
        struct timeval stv = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        /* 发 SubscribeAck 让 client 知道实际接受的 freq (区分"接受"和"被 clamp") */
        SubscribeAck ack;
        ack.magic          = SMC_ACK_MAGIC;
        ack.version        = SMC_SNAPSHOT_VERSION;
        ack.actual_freq_hz = (uint32_t)freq_hz;
        ack.reserved       = 0;
        if (send_all(cfd, &ack, sizeof(ack)) != 0) {
            printf("[push] client fd=%d send ack 失败, 断开\n", cfd);
            close(cfd);
            continue;
        }

        /* 创建 client_thread */
        ClientContext_t *ctx = (ClientContext_t *)calloc(1, sizeof(ClientContext_t));
        if (ctx == NULL) {
            close(cfd);
            continue;
        }
        ctx->fd = cfd;
        ctx->freq_hz = freq_hz;
        atomic_store_explicit(&ctx->running, 1, memory_order_release);

        if (pthread_create(&ctx->tid, NULL, client_thread, ctx) != 0) {
            perror("[push] pthread_create client_thread");
            close(cfd);
            free(ctx);
            continue;
        }
        pthread_detach(ctx->tid);   /* 自我清理, 不需 join */
        printf("[push] client fd=%d subscribed, freq=%d\n", cfd, freq_hz);
    }

    if (g_push_listen_fd >= 0) {
        close(g_push_listen_fd);
        g_push_listen_fd = -1;
    }
    printf("[push] accept_thread exited\n");
    return NULL;
}

/* =====================================================================
 *  公共 API
 * ===================================================================== */

/* @Context: Non-RealTime (rpc_server.c main, kernel_init 之后) */
int rpc_push_server_start(void)
{
    if (atomic_load_explicit(&g_push_running, memory_order_acquire)) {
        return 0;   /* 已启动, 幂等 */
    }
    atomic_store_explicit(&g_push_running, 1, memory_order_release);

    if (pthread_create(&g_push_accept_tid, NULL, accept_thread, NULL) != 0) {
        perror("[push] pthread_create accept_thread");
        atomic_store_explicit(&g_push_running, 0, memory_order_release);
        return -1;
    }
    pthread_detach(g_push_accept_tid);   /* 不需 join, 进程退出时自然终止 */
    return 0;
}

/* @Context: Non-RealTime (SMC_Close 路径或进程退出) */
void rpc_push_server_stop(void)
{
    if (!atomic_load_explicit(&g_push_running, memory_order_acquire)) {
        return;
    }
    atomic_store_explicit(&g_push_running, 0, memory_order_release);

    /* 关闭 listen_fd 让 accept() 返回错误退出循环 */
    if (g_push_listen_fd >= 0) {
        close(g_push_listen_fd);
        g_push_listen_fd = -1;
    }
    /* client_thread 在下次 send 失败后自然退出 (SO_SNDTIMEO 2s 上限) */
    /* 不强制 join, 避免卡死主退出流程 */
}
