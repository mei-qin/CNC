/* =====================================================================
 *  rpc_event_server.c  ——  P1-b 事件流推送服务端实现 (Linux, 端口 9530)
 *
 *  结构拷贝自 rpc_preview_server.c, 改帧类型 SmcEvent_t。
 * ===================================================================== */

#include "rpc_event_server.h"
#include "event_logger.h"
#include "smc_protocol.h"   /* SMC_CMD_EVENT_SUBSCRIBE, SmcReqHeader, SmcEventAck, SmcEventFrameHeader */

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
 *  CRC32 (与 rpc_push/preview_server.c 同实现)
 * ===================================================================== */
static uint32_t g_ev_crc32_table[256];
static int      g_ev_crc32_initialized = 0;

static void crc32_init_once(void)
{
    if (g_ev_crc32_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        g_ev_crc32_table[i] = c;
    }
    g_ev_crc32_initialized = 1;
}

static uint32_t crc32_bufs(const void *b1, size_t n1,
                            const void *b2, size_t n2)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p1 = (const uint8_t *)b1;
    for (size_t i = 0; i < n1; i++) {
        crc = g_ev_crc32_table[(crc ^ p1[i]) & 0xFFu] ^ (crc >> 8);
    }
    if (b2 && n2) {
        const uint8_t *p2 = (const uint8_t *)b2;
        for (size_t i = 0; i < n2; i++) {
            crc = g_ev_crc32_table[(crc ^ p2[i]) & 0xFFu] ^ (crc >> 8);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* =====================================================================
 *  TCP IO 辅助
 * ===================================================================== */
static int recv_all(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    size_t  got = 0;
    while (got < n) {
        ssize_t r = recv(fd, p + got, n - got, 0);
        if (r == 0) return -1;
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
 *  帧发送: SmcEventFrameHeader + N × SmcEvent_t + CRC32
 * ===================================================================== */
static int send_frame(int fd, const SmcEvent_t *events, uint32_t count)
{
    SmcEventFrameHeader hdr;
    hdr.magic       = SMC_EVENT_MAGIC;
    hdr.version     = SMC_EVENT_VERSION;
    hdr.event_count = count;
    size_t ev_bytes = count * sizeof(SmcEvent_t);
    hdr.crc32 = crc32_bufs(&hdr.event_count, 4, events, ev_bytes);

    if (send_all(fd, &hdr, sizeof(hdr)) != 0) return -1;
    if (count > 0) {
        if (send_all(fd, events, ev_bytes) != 0) return -1;
    }
    return 0;
}

/* =====================================================================
 *  Client 上下文 + 推送线程
 * ===================================================================== */
typedef struct {
    int      fd;
    int      freq_hz;
    uint64_t next_seq;
    _Atomic int running;
    pthread_t tid;
} ClientContext_t;

static void *client_thread(void *arg)
{
    ClientContext_t *ctx = (ClientContext_t *)arg;
    SmcEvent_t buf[EVENT_READ_MAX];

    while (atomic_load_explicit(&ctx->running, memory_order_acquire)) {
        uint64_t next_seq_after = ctx->next_seq;
        int n = EventLogger_ReadSince(ctx->next_seq, buf, EVENT_READ_MAX,
                                       &next_seq_after);

        if (n < 0) {
            /* client lag 太大, 跳到最新 */
            printf("[event] client fd=%d lag, skip to latest\n", ctx->fd);
            ctx->next_seq = EventLogger_GetWriteSeq();
            continue;
        }

        if (n > 0) {
            if (send_frame(ctx->fd, buf, (uint32_t)n) < 0) {
                printf("[event] client fd=%d send 失败, 断开\n", ctx->fd);
                break;
            }
            ctx->next_seq = next_seq_after;
        }

        usleep((useconds_t)(1000000 / ctx->freq_hz));
    }

    atomic_store_explicit(&ctx->running, 0, memory_order_release);
    close(ctx->fd);
    printf("[event] client fd=%d disconnected, freq=%d\n", ctx->fd, ctx->freq_hz);
    free(ctx);
    return NULL;
}

/* =====================================================================
 *  Accept 主线程
 * ===================================================================== */
static int          g_ev_listen_fd = -1;
static _Atomic int  g_ev_running = 0;
static pthread_t    g_ev_accept_tid;

static void *accept_thread(void *arg)
{
    (void)arg;
    crc32_init_once();

    g_ev_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_ev_listen_fd < 0) { perror("[event] socket"); return NULL; }
    int opt = 1;
    setsockopt(g_ev_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(SMC_EVENT_PORT);

    if (bind(g_ev_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[event] bind"); close(g_ev_listen_fd); g_ev_listen_fd = -1; return NULL;
    }
    if (listen(g_ev_listen_fd, SMC_EVENT_BACKLOG) < 0) {
        perror("[event] listen"); close(g_ev_listen_fd); g_ev_listen_fd = -1; return NULL;
    }
    printf("[event] listening on 0.0.0.0:%d ...\n", SMC_EVENT_PORT);

    while (atomic_load_explicit(&g_ev_running, memory_order_acquire)) {
        int cfd = accept(g_ev_listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            if (!atomic_load_explicit(&g_ev_running, memory_order_acquire)) break;
            perror("[event] accept");
            usleep(1000);
            continue;
        }
        printf("[event] client connected, fd=%d\n", cfd);

        /* SmcReqHeader + {int32 freq, uint64 from_seq} = 4 + 12 = 16 bytes */
        struct timeval rtv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));

        SmcReqHeader req_hdr;
        int32_t      freq_hz = SMC_EVENT_FREQ_DEFAULT;
        uint64_t     from_seq = 0;
        int          parse_ok = 0;

        if (recv_all(cfd, &req_hdr, sizeof(req_hdr)) == 0 &&
            req_hdr.cmd_type == SMC_CMD_EVENT_SUBSCRIBE) {
            if (req_hdr.data_len >= 12) {
                if (recv_all(cfd, &freq_hz, sizeof(int32_t)) == 0 &&
                    recv_all(cfd, &from_seq, sizeof(uint64_t)) == 0) {
                    parse_ok = 1;
                }
            } else if (req_hdr.data_len == 0) {
                parse_ok = 1;
            } else {
                uint8_t tmp[12];
                recv_all(cfd, tmp, req_hdr.data_len);
                parse_ok = 1;
            }
        }

        if (!parse_ok) {
            printf("[event] client fd=%d subscribe req 失败, 断开\n", cfd);
            close(cfd);
            continue;
        }

        if (freq_hz < SMC_EVENT_FREQ_MIN || freq_hz > SMC_EVENT_FREQ_MAX) {
            printf("[event] client fd=%d freq=%d 越界, 用默认 %d\n",
                   cfd, freq_hz, SMC_EVENT_FREQ_DEFAULT);
            freq_hz = SMC_EVENT_FREQ_DEFAULT;
        }

        uint64_t cur_seq = EventLogger_GetWriteSeq();
        if (from_seq > cur_seq) {
            printf("[event] client fd=%d from_seq=%llu > cur=%llu, clamp\n",
                   cfd, (unsigned long long)from_seq, (unsigned long long)cur_seq);
            from_seq = cur_seq;
        }

        struct timeval stv = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        /* 发 SmcEventAck */
        SmcEventAck ack;
        ack.magic          = SMC_EVENT_ACK_MAGIC;
        ack.version        = SMC_EVENT_VERSION;
        ack.max_per_tick   = EVENT_READ_MAX;
        ack.event_size_bytes = (uint32_t)sizeof(SmcEvent_t);
        if (send_all(cfd, &ack, sizeof(ack)) != 0) {
            printf("[event] client fd=%d send ack 失败, 断开\n", cfd);
            close(cfd);
            continue;
        }

        ClientContext_t *ctx = (ClientContext_t *)calloc(1, sizeof(ClientContext_t));
        if (ctx == NULL) { close(cfd); continue; }
        ctx->fd = cfd;
        ctx->freq_hz = freq_hz;
        ctx->next_seq = from_seq;
        atomic_store_explicit(&ctx->running, 1, memory_order_release);

        if (pthread_create(&ctx->tid, NULL, client_thread, ctx) != 0) {
            perror("[event] pthread_create");
            close(cfd); free(ctx); continue;
        }
        pthread_detach(ctx->tid);
        printf("[event] client fd=%d subscribed, freq=%d from_seq=%llu\n",
               cfd, freq_hz, (unsigned long long)from_seq);
    }

    if (g_ev_listen_fd >= 0) {
        close(g_ev_listen_fd);
        g_ev_listen_fd = -1;
    }
    printf("[event] accept_thread exited\n");
    return NULL;
}

int rpc_event_server_start(void)
{
    if (atomic_load_explicit(&g_ev_running, memory_order_acquire)) return 0;
    atomic_store_explicit(&g_ev_running, 1, memory_order_release);

    if (pthread_create(&g_ev_accept_tid, NULL, accept_thread, NULL) != 0) {
        perror("[event] pthread_create accept_thread");
        atomic_store_explicit(&g_ev_running, 0, memory_order_release);
        return -1;
    }
    pthread_detach(g_ev_accept_tid);
    return 0;
}

void rpc_event_server_stop(void)
{
    if (!atomic_load_explicit(&g_ev_running, memory_order_acquire)) return;
    atomic_store_explicit(&g_ev_running, 0, memory_order_release);
    if (g_ev_listen_fd >= 0) {
        close(g_ev_listen_fd);
        g_ev_listen_fd = -1;
    }
}
