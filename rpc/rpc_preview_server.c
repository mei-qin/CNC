/* =====================================================================
 *  rpc_preview_server.c  ——  P0-b v1 段流推送服务端实现 (Linux, 端口 9529)
 *
 *  详见 inc/rpc_preview_server.h 头注释。
 *
 *  线程模型 (与 rpc_push_server.c 一致, 改帧格式):
 *    accept_thread (单实例) - bind/listen 9529, accept 循环
 *      每来 client: recv subscribe req → 发 SmcPreviewAck → 创建 ClientContext_t → pthread_create
 *    client_thread (每客户端) - 循环 ReadSince + 单帧多段打包 + send + usleep(freq)
 *
 *  关键差异 vs rpc_push_server.c:
 *    1. subscribe req payload 是 12B (int32 freq + uint64 from_seq), 不是 4B
 *    2. ack 多了 max_per_tick + seg_size_bytes 字段
 *    3. 单帧多段 (≤ PREVIEW_READ_MAX=16), 减少 TCP 帧头开销
 *    4. CRC32 范围: header[8..12] + segments (不是 header[0..12] + payload)
 *    5. client_thread 维护 next_seq 状态, 跨 tick 持续读
 * ===================================================================== */

#include "axis_cfg.h"           /* AXIS_NUM 权威值 */
#include "rpc_preview_server.h"
#include "preview_streamer.h"
#include "smc_protocol.h"        /* SMC_CMD_PREVIEW_SUBSCRIBE, SmcReqHeader */

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
 *  CRC32 (与 rpc_push_server.c 同实现, zlib 兼容)
 * ===================================================================== */
static uint32_t g_pv_crc32_table[256];
static int      g_pv_crc32_initialized = 0;

static void crc32_init_once(void)
{
    if (g_pv_crc32_initialized) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        g_pv_crc32_table[i] = c;
    }
    g_pv_crc32_initialized = 1;
}

static uint32_t crc32_bufs(const void *b1, size_t n1,
                            const void *b2, size_t n2)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p1 = (const uint8_t *)b1;
    for (size_t i = 0; i < n1; i++) {
        crc = g_pv_crc32_table[(crc ^ p1[i]) & 0xFFu] ^ (crc >> 8);
    }
    if (b2 && n2) {
        const uint8_t *p2 = (const uint8_t *)b2;
        for (size_t i = 0; i < n2; i++) {
            crc = g_pv_crc32_table[(crc ^ p2[i]) & 0xFFu] ^ (crc >> 8);
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
 *  帧发送: 构造 SmcPreviewFrameHeader + 计算 CRC32 + 发送 header + segments
 * ===================================================================== */
static int send_frame(int fd, const TrajectorySegment_t *segs, uint32_t count)
{
    SmcPreviewFrameHeader hdr;
    hdr.magic     = SMC_PREVIEW_MAGIC;
    hdr.version   = SMC_PREVIEW_VERSION;
    hdr.seg_count = count;
    /* CRC32 覆盖: seg_count 字段 (header[8..12]) + segments */
    size_t seg_bytes = count * sizeof(TrajectorySegment_t);
    hdr.crc32 = crc32_bufs(&hdr.seg_count, 4, segs, seg_bytes);

    if (send_all(fd, &hdr, sizeof(hdr)) != 0) return -1;
    if (count > 0) {
        if (send_all(fd, segs, seg_bytes) != 0) return -1;
    }
    return 0;
}

/* =====================================================================
 *  Client 上下文 + 推送线程
 * ===================================================================== */
typedef struct {
    int      fd;
    int      freq_hz;
    uint64_t next_seq;        /* 下次 ReadSince 起始 seq (跨 tick 持续) */
    _Atomic int running;
    pthread_t tid;
} ClientContext_t;

/* @Context: Non-RealTime Background Thread (每客户端一个) */
static void *client_thread(void *arg)
{
    ClientContext_t *ctx = (ClientContext_t *)arg;
    TrajectorySegment_t buf[PREVIEW_READ_MAX];

    while (atomic_load_explicit(&ctx->running, memory_order_acquire)) {
        uint64_t next_seq_after = ctx->next_seq;
        int n = PreviewStreamer_ReadSince(ctx->next_seq, buf, PREVIEW_READ_MAX,
                                           &next_seq_after);

        if (n < 0) {
            /* client lag 太大 (落后 > RING_CAPACITY), 数据已被覆盖。
             * 简单策略: 跳到最新 (丢弃中间段), 继续推送。
             * UI 端可通过 seg_id 不连续检测到丢段并告警。 */
            printf("[preview] client fd=%d lag, skip to latest\n", ctx->fd);
            ctx->next_seq = PreviewStreamer_GetWriteSeq();
            continue;
        }

        if (n > 0) {
            /* 即使 n==1 也立即推 (避免延迟) */
            if (send_frame(ctx->fd, buf, (uint32_t)n) < 0) {
                printf("[preview] client fd=%d send 失败, 断开\n", ctx->fd);
                break;
            }
            ctx->next_seq = next_seq_after;
        }

        usleep((useconds_t)(1000000 / ctx->freq_hz));
    }

    atomic_store_explicit(&ctx->running, 0, memory_order_release);
    close(ctx->fd);
    printf("[preview] client fd=%d disconnected, freq=%d\n", ctx->fd, ctx->freq_hz);
    free(ctx);
    return NULL;
}

/* =====================================================================
 *  Accept 主线程
 * ===================================================================== */
static int          g_pv_listen_fd = -1;
static _Atomic int  g_pv_running = 0;
static pthread_t    g_pv_accept_tid;

static void *accept_thread(void *arg)
{
    (void)arg;

    crc32_init_once();

    g_pv_listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (g_pv_listen_fd < 0) {
        perror("[preview] socket");
        return NULL;
    }
    int opt = 1;
    setsockopt(g_pv_listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(SMC_PREVIEW_PORT);

    if (bind(g_pv_listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[preview] bind");
        close(g_pv_listen_fd);
        g_pv_listen_fd = -1;
        return NULL;
    }
    if (listen(g_pv_listen_fd, SMC_PREVIEW_BACKLOG) < 0) {
        perror("[preview] listen");
        close(g_pv_listen_fd);
        g_pv_listen_fd = -1;
        return NULL;
    }
    printf("[preview] listening on 0.0.0.0:%d ...\n", SMC_PREVIEW_PORT);

    while (atomic_load_explicit(&g_pv_running, memory_order_acquire)) {
        int cfd = accept(g_pv_listen_fd, NULL, NULL);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            if (!atomic_load_explicit(&g_pv_running, memory_order_acquire)) break;
            perror("[preview] accept");
            usleep(1000);
            continue;
        }
        printf("[preview] client connected, fd=%d\n", cfd);

        /* 接收 subscribe req: SmcReqHeader + {int32 freq_hz, uint64 from_seq} */
        struct timeval rtv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));

        SmcReqHeader req_hdr;
        int32_t      freq_hz = 60;       /* 默认 60Hz */
        uint64_t     from_seq = 0;        /* 默认从头要全部历史 */
        int          parse_ok = 0;

        if (recv_all(cfd, &req_hdr, sizeof(req_hdr)) == 0) {
            if (req_hdr.cmd_type == SMC_CMD_PREVIEW_SUBSCRIBE) {
                if (req_hdr.data_len >= 12) {
                    /* 读 freq (4B) + from_seq (8B), 顺序与 client pack 一致 */
                    if (recv_all(cfd, &freq_hz, sizeof(int32_t)) == 0 &&
                        recv_all(cfd, &from_seq, sizeof(uint64_t)) == 0) {
                        parse_ok = 1;
                    }
                } else if (req_hdr.data_len == 0) {
                    parse_ok = 1;   /* 用全默认 */
                } else {
                    uint8_t tmp[12];
                    recv_all(cfd, tmp, req_hdr.data_len);
                    parse_ok = 1;
                }
            } else {
                printf("[preview] client fd=%d cmd_type=0x%04X 非 PREVIEW_SUBSCRIBE, 断开\n",
                       cfd, req_hdr.cmd_type);
            }
        }

        if (!parse_ok) {
            printf("[preview] client fd=%d subscribe req 失败, 断开\n", cfd);
            close(cfd);
            continue;
        }

        /* freq 范围校验 */
        if (freq_hz < SMC_PREVIEW_FREQ_MIN || freq_hz > SMC_PREVIEW_FREQ_MAX) {
            printf("[preview] client fd=%d freq=%d 越界, 用默认 60\n", cfd, freq_hz);
            freq_hz = SMC_PREVIEW_FREQ_DEFAULT;
        }

        /* from_seq 校验: 不能超过当前 write_seq (否则没意义) */
        uint64_t cur_seq = PreviewStreamer_GetWriteSeq();
        if (from_seq > cur_seq) {
            printf("[preview] client fd=%d from_seq=%llu > cur=%llu, clamp\n",
                   cfd, (unsigned long long)from_seq, (unsigned long long)cur_seq);
            from_seq = cur_seq;
        }

        /* send 超时 2s */
        struct timeval stv = { .tv_sec = 2, .tv_usec = 0 };
        setsockopt(cfd, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

        /* 发 SmcPreviewAck */
        SmcPreviewAck ack;
        ack.magic          = SMC_PREVIEW_ACK_MAGIC;
        ack.version        = SMC_PREVIEW_VERSION;
        ack.max_per_tick   = PREVIEW_READ_MAX;
        ack.seg_size_bytes = (uint32_t)sizeof(TrajectorySegment_t);
        if (send_all(cfd, &ack, sizeof(ack)) != 0) {
            printf("[preview] client fd=%d send ack 失败, 断开\n", cfd);
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
        ctx->next_seq = from_seq;
        atomic_store_explicit(&ctx->running, 1, memory_order_release);

        if (pthread_create(&ctx->tid, NULL, client_thread, ctx) != 0) {
            perror("[preview] pthread_create");
            close(cfd);
            free(ctx);
            continue;
        }
        pthread_detach(ctx->tid);
        printf("[preview] client fd=%d subscribed, freq=%d from_seq=%llu\n",
               cfd, freq_hz, (unsigned long long)from_seq);
    }

    if (g_pv_listen_fd >= 0) {
        close(g_pv_listen_fd);
        g_pv_listen_fd = -1;
    }
    printf("[preview] accept_thread exited\n");
    return NULL;
}

/* =====================================================================
 *  公共 API
 * ===================================================================== */

int rpc_preview_server_start(void)
{
    if (atomic_load_explicit(&g_pv_running, memory_order_acquire)) {
        return 0;
    }
    atomic_store_explicit(&g_pv_running, 1, memory_order_release);

    if (pthread_create(&g_pv_accept_tid, NULL, accept_thread, NULL) != 0) {
        perror("[preview] pthread_create accept_thread");
        atomic_store_explicit(&g_pv_running, 0, memory_order_release);
        return -1;
    }
    pthread_detach(g_pv_accept_tid);
    return 0;
}

void rpc_preview_server_stop(void)
{
    if (!atomic_load_explicit(&g_pv_running, memory_order_acquire)) {
        return;
    }
    atomic_store_explicit(&g_pv_running, 0, memory_order_release);

    if (g_pv_listen_fd >= 0) {
        close(g_pv_listen_fd);
        g_pv_listen_fd = -1;
    }
}
