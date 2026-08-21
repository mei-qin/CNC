/* =====================================================================
 *  event_logger.c  ——  P1-b 事件/报警流推送中心实现
 *
 *  设计见 inc/event_logger.h 头注释。结构拷贝自 preview_streamer.c, 改 entry
 *  类型为 SmcEvent_t, 容量改 1024 (事件比段少)。
 *
 *  关键不变量:
 *    1. g_event_write_seq 单调不减
 *    2. ring slot [N % CAP] 在 write_seq > N 时已写, 内容稳定直到 write_seq > N + CAP
 *    3. writer 是 single-producer (但可跨线程, atomic_fetch_add seq 保证唯一性)
 *    4. reader 检查 from_seq + CAP > write_seq 才读, 保证 slot 未被覆盖
 *
 *  跨线程 Push 安全性:
 *    多线程同时 Push 时, atomic_fetch_add_event_seq 保证每个 caller 拿到唯一 seq,
 *    写入 ring[seq % CAP] 是不同 slot (除非已 wrap, 此时数据丢失但不会撕裂)。
 *    写入顺序: 先填 local event, 再 memcpy 到 ring slot, 最后 atomic_store release seq。
 *    但本实现简化: 直接写到 ring slot, 单 _Atomic seq 推进。
 *    若多线程同时 Push, slot 可能并发写 → 撕裂。
 *
 *    **v1 假设**: EventLogger_Push 主要在 RT 单线程 (1ms) + parser 单线程 (10Hz) +
 *    SMC_API 偶发调用, 多数时刻不并发。若实测发现撕裂, v2 加 spinlock。
 *
 *  C1 (2026-07-24) 故障持久化:
 *    StartPersistThread 扫描目录找最新 event_log_*.bin → 回放 → 新建时间戳文件 →
 *    启动后台落盘线程 (100ms 周期, 增量 fwrite + fsync).
 *    StopPersistThread flush 最后一批 + join 线程 + fclose.
 *    落盘失败降级内存 only (warn 日志, 不阻塞系统).
 * ===================================================================== */

#include "event_logger.h"
#include "global_def.h"   /* cycle (uint32 RT 周期计数) */
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

/* ---- 静态分配 (避免 malloc, 模块加载即就绪) ---- */
static SmcEvent_t g_event_ring[EVENT_RING_CAPACITY];
static _Alignas(64) _Atomic uint64_t g_event_write_seq = 0;

/* ---- C1 持久化全局 (仅 Non-RT 落盘线程访问, 无锁) ---- */
static FILE *g_persist_fp = NULL;
static EventLogFileHeader_t g_persist_header;
static _Atomic int g_persist_running = 0;
static pthread_t g_persist_thread;
static uint64_t g_persist_last_seq = 0;
static uint64_t g_boot_id_counter = 0;
static char g_persist_dir[256] = {0};
static char g_persist_current_file[512] = {0};

int EventLogger_Init(void)
{
    memset(g_event_ring, 0, sizeof(g_event_ring));
    atomic_store_explicit(&g_event_write_seq, 0, memory_order_release);

    /* C1: 重置持久化状态 (实际启动在 StartPersistThread) */
    g_persist_fp = NULL;
    memset(&g_persist_header, 0, sizeof(g_persist_header));
    atomic_store_explicit(&g_persist_running, 0, memory_order_release);
    g_persist_last_seq = 0;
    return 0;
}

void EventLogger_Push(uint8_t severity, uint8_t source, uint16_t code,
                       int32_t value, const char *message)
{
    uint64_t seq = atomic_fetch_add_explicit(&g_event_write_seq, 1,
                                              memory_order_acq_rel);
    uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
    SmcEvent_t *ev = &g_event_ring[idx];

    /* timestamp = 当前 RT cycle * 1ms (与 snapshot uptime_ms 一致) */
    ev->timestamp_ms = (uint64_t)cycle;
    ev->event_seq    = seq;
    ev->severity     = severity;
    ev->source       = source;
    ev->code         = code;
    ev->value        = value;

    /* strncpy 不保证 null 终结, 手动强制 */
    if (message != NULL) {
        strncpy(ev->message, message, SMC_EVENT_MSG_LEN - 1);
        ev->message[SMC_EVENT_MSG_LEN - 1] = '\0';
    } else {
        ev->message[0] = '\0';
    }
}

int EventLogger_ReadSince(uint64_t from_seq,
                            SmcEvent_t *out_buf,
                            int max_count,
                            uint64_t *out_next_seq)
{
    if (out_buf == NULL || max_count <= 0) {
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    uint64_t latest = atomic_load_explicit(&g_event_write_seq,
                                            memory_order_acquire);

    if (from_seq >= latest) {
        if (out_next_seq) *out_next_seq = from_seq;
        return 0;
    }

    /* 检查 from_seq 是否已被覆盖 */
    if (latest > (uint64_t)EVENT_RING_CAPACITY &&
        from_seq + EVENT_RING_CAPACITY <= latest) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    int available = (int)(latest - from_seq);
    if (available > max_count) available = max_count;

    for (int i = 0; i < available; i++) {
        uint64_t seq = from_seq + (uint64_t)i;
        uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
        out_buf[i] = g_event_ring[idx];
    }

    /* 二次检查: 读期间 writer 是否覆盖了 from_seq */
    uint64_t now = atomic_load_explicit(&g_event_write_seq,
                                         memory_order_acquire);
    if (now > (uint64_t)EVENT_RING_CAPACITY &&
        from_seq + EVENT_RING_CAPACITY <= now) {
        if (out_next_seq) *out_next_seq = from_seq;
        return -1;
    }

    if (out_next_seq) *out_next_seq = from_seq + (uint64_t)available;
    return available;
}

uint64_t EventLogger_GetWriteSeq(void)
{
    return atomic_load_explicit(&g_event_write_seq, memory_order_acquire);
}

/* =====================================================================
 *  C1 持久化实现
 * ===================================================================== */

/* 扫描 dir_path 下所有 "event_log_*.bin" 文件, 按 mtime 取最新写入 out_path */
static int find_latest_persist_file(const char *dir_path, char *out_path, size_t out_size)
{
    DIR *d = opendir(dir_path);
    if (d == NULL) return -1;

    char best_path[512] = {0};
    time_t best_mtime = 0;
    struct dirent *ent;
    while ((ent = readdir(d)) != NULL) {
        /* 匹配 "event_log_*.bin" 前缀 + 后缀 */
        if (strncmp(ent->d_name, "event_log_", 10) != 0) continue;
        size_t nlen = strlen(ent->d_name);
        if (nlen < 5 || strcmp(ent->d_name + nlen - 4, ".bin") != 0) continue;

        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dir_path, ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0) continue;
        if (st.st_mtime > best_mtime) {
            best_mtime = st.st_mtime;
            strncpy(best_path, full, sizeof(best_path) - 1);
        }
    }
    closedir(d);

    if (best_path[0] == '\0') return -1;  /* 未找到 */
    strncpy(out_path, best_path, out_size - 1);
    out_path[out_size - 1] = '\0';
    return 0;
}

/* 清理 dir_path 下超过 max_age_days 天的 event_log_*.bin 文件 */
static void cleanup_old_persist_files(const char *dir_path, int max_age_days)
{
    DIR *d = opendir(dir_path);
    if (d == NULL) return;

    time_t now = time(NULL);
    struct dirent *ent;
    int cleaned = 0;
    while ((ent = readdir(d)) != NULL) {
        if (strncmp(ent->d_name, "event_log_", 10) != 0) continue;
        size_t nlen = strlen(ent->d_name);
        if (nlen < 5 || strcmp(ent->d_name + nlen - 4, ".bin") != 0) continue;

        char full[512];
        snprintf(full, sizeof(full), "%s/%s", dir_path, ent->d_name);
        struct stat st;
        if (stat(full, &st) != 0) continue;
        if (now - st.st_mtime > (time_t)max_age_days * 86400) {
            if (unlink(full) == 0) cleaned++;
        }
    }
    closedir(d);
    if (cleaned > 0) {
        printf("[EventLogger] 清理 %d 个超过 %d 天的旧文件\n", cleaned, max_age_days);
    }
}

/* 从 file_path 回放历史 events 到内存 ring + 设 write_seq */
static int replay_persist_file(const char *file_path)
{
    FILE *fp = fopen(file_path, "rb");
    if (fp == NULL) return -1;

    EventLogFileHeader_t hdr;
    if (fread(&hdr, sizeof(hdr), 1, fp) != 1) {
        fclose(fp);
        return -1;
    }

    /* magic/version/capacity 校验 */
    if (hdr.magic != EVENT_PERSIST_MAGIC ||
        hdr.version != EVENT_PERSIST_VERSION ||
        hdr.capacity != EVENT_RING_CAPACITY) {
        printf("[EventLogger] 回放文件 %s 损坏 (magic=0x%X ver=%d cap=%d), 跳过\n",
               file_path, hdr.magic, hdr.version, hdr.capacity);
        fclose(fp);
        return -1;
    }

    /* 回填最后 min(write_seq, CAP) 条到 ring + 设 atomic write_seq */
    uint64_t total = hdr.write_seq;
    uint64_t start = (total > EVENT_RING_CAPACITY) ? (total - EVENT_RING_CAPACITY) : 0;
    int replayed = 0;
    for (uint64_t seq = start; seq < total; seq++) {
        uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
        long offset = (long)sizeof(EventLogFileHeader_t) + (long)idx * sizeof(SmcEvent_t);
        if (fseek(fp, offset, SEEK_SET) != 0) break;
        if (fread(&g_event_ring[idx], sizeof(SmcEvent_t), 1, fp) != 1) break;
        replayed++;
    }
    atomic_store_explicit(&g_event_write_seq, total, memory_order_release);
    printf("[EventLogger] 从 %s 回放 %d 条事件 (write_seq=%llu)\n",
           file_path, replayed, (unsigned long long)total);
    fclose(fp);
    return 0;
}

/* 生成时间戳文件名: <dir>/event_log_YYYYMMDD_HHMMSS.bin */
static void generate_timestamp_path(const char *dir_path,
                                     char *out_buf, size_t out_size)
{
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    char fname[64];
    strftime(fname, sizeof(fname), "event_log_%Y%m%d_%H%M%S.bin", tm);
    snprintf(out_buf, out_size, "%s/%s", dir_path, fname);
}

/* 落盘线程主体: 100ms 周期, 增量 fwrite + fsync */
static void *event_persist_thread_func(void *arg)
{
    (void)arg;
    while (atomic_load_explicit(&g_persist_running, memory_order_acquire)) {
        osal_usleep(EVENT_PERSIST_PERIOD_MS * 1000);

        uint64_t current = EventLogger_GetWriteSeq();
        if (current == g_persist_last_seq) continue;

        /* 增量写入 [last, current) 的 events (跳过已被 ring 覆盖的) */
        for (uint64_t seq = g_persist_last_seq; seq < current; seq++) {
            if (current - seq > EVENT_RING_CAPACITY) continue;  /* 已被覆盖 */
            uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
            long offset = (long)sizeof(EventLogFileHeader_t) +
                          (long)idx * sizeof(SmcEvent_t);
            if (fseek(g_persist_fp, offset, SEEK_SET) != 0) break;
            if (fwrite(&g_event_ring[idx], sizeof(SmcEvent_t), 1, g_persist_fp) != 1) break;
        }

        /* 更新 header.write_seq + 写 header + fsync */
        g_persist_header.write_seq = current;
        fseek(g_persist_fp, 0, SEEK_SET);
        fwrite(&g_persist_header, sizeof(g_persist_header), 1, g_persist_fp);
        fflush(g_persist_fp);
        fsync(fileno(g_persist_fp));

        g_persist_last_seq = current;
    }
    return NULL;
}

int EventLogger_StartPersistThread(const char *dir_path, uint64_t *out_boot_id)
{
    /* 防御: 重复调用 */
    if (atomic_load_explicit(&g_persist_running, memory_order_acquire)) {
        printf("[EventLogger] 持久化线程已在运行, 跳过\n");
        return -1;
    }

    /* 默认目录处理 */
    const char *dir = (dir_path != NULL) ? dir_path : EVENT_PERSIST_DIR;
    strncpy(g_persist_dir, dir, sizeof(g_persist_dir) - 1);

    /* 创建目录 (不存在时) */
    struct stat st;
    if (stat(g_persist_dir, &st) != 0) {
        if (mkdir(g_persist_dir, 0755) != 0 && errno != EEXIST) {
            printf("[EventLogger] 创建目录 %s 失败: %s (降级内存 only)\n",
                   g_persist_dir, strerror(errno));
            return -1;
        }
    }

    /* 扫描最新文件 → 回放 */
    char latest[512];
    if (find_latest_persist_file(g_persist_dir, latest, sizeof(latest)) == 0) {
        replay_persist_file(latest);
    }

    /* 清理旧文件 */
    cleanup_old_persist_files(g_persist_dir, EVENT_PERSIST_MAX_AGE_D);

    /* 生成新文件名 + 打开 */
    generate_timestamp_path(g_persist_dir, g_persist_current_file,
                             sizeof(g_persist_current_file));
    g_persist_fp = fopen(g_persist_current_file, "wb+");
    if (g_persist_fp == NULL) {
        printf("[EventLogger] 打开 %s 失败: %s (降级内存 only)\n",
               g_persist_current_file, strerror(errno));
        return -2;
    }

    /* 写 empty header (write_seq 已被回放设为历史值, boot_id 递增) */
    g_boot_id_counter++;
    g_persist_header.magic = EVENT_PERSIST_MAGIC;
    g_persist_header.version = EVENT_PERSIST_VERSION;
    g_persist_header.capacity = EVENT_RING_CAPACITY;
    g_persist_header.reserved = 0;
    g_persist_header.write_seq = EventLogger_GetWriteSeq();
    g_persist_header.boot_id = g_boot_id_counter;
    fwrite(&g_persist_header, sizeof(g_persist_header), 1, g_persist_fp);
    fflush(g_persist_fp);
    fsync(fileno(g_persist_fp));

    /* 预分配文件大小 (header + CAP × 88B), 避免 fseek 越界 */
    long file_size = (long)sizeof(EventLogFileHeader_t) +
                     (long)EVENT_RING_CAPACITY * sizeof(SmcEvent_t);
    fseek(g_persist_fp, file_size - 1, SEEK_SET);
    fputc(0, g_persist_fp);
    fflush(g_persist_fp);

    g_persist_last_seq = g_persist_header.write_seq;

    /* 启动落盘线程 (用 pthread, 与 sim_engine 落盘线程同模式) */
    atomic_store_explicit(&g_persist_running, 1, memory_order_release);
    if (pthread_create(&g_persist_thread, NULL,
                        event_persist_thread_func, NULL) != 0) {
        printf("[EventLogger] 落盘线程创建失败 (降级内存 only)\n");
        atomic_store_explicit(&g_persist_running, 0, memory_order_release);
        fclose(g_persist_fp);
        g_persist_fp = NULL;
        return -1;
    }

    if (out_boot_id) *out_boot_id = g_boot_id_counter;
    printf("[EventLogger] 持久化启动: file=%s boot_id=%llu write_seq=%llu\n",
           g_persist_current_file,
           (unsigned long long)g_boot_id_counter,
           (unsigned long long)g_persist_header.write_seq);
    return 0;
}

int EventLogger_StopPersistThread(void)
{
    if (!atomic_load_explicit(&g_persist_running, memory_order_acquire)) {
        return -1;
    }

    /* 通知线程退出 */
    atomic_store_explicit(&g_persist_running, 0, memory_order_release);

    /* join 线程 (pthread, 与 sim_engine_finish 同模式) */
    pthread_join(g_persist_thread, NULL);

    /* flush 最后一批 (与线程主体同逻辑) */
    if (g_persist_fp != NULL) {
        uint64_t current = EventLogger_GetWriteSeq();
        for (uint64_t seq = g_persist_last_seq; seq < current; seq++) {
            if (current - seq > EVENT_RING_CAPACITY) continue;
            uint32_t idx = (uint32_t)(seq % EVENT_RING_CAPACITY);
            long offset = (long)sizeof(EventLogFileHeader_t) +
                          (long)idx * sizeof(SmcEvent_t);
            if (fseek(g_persist_fp, offset, SEEK_SET) != 0) break;
            if (fwrite(&g_event_ring[idx], sizeof(SmcEvent_t), 1, g_persist_fp) != 1) break;
        }
        g_persist_header.write_seq = current;
        fseek(g_persist_fp, 0, SEEK_SET);
        fwrite(&g_persist_header, sizeof(g_persist_header), 1, g_persist_fp);
        fflush(g_persist_fp);
        fsync(fileno(g_persist_fp));
        fclose(g_persist_fp);
        g_persist_fp = NULL;
        printf("[EventLogger] 持久化停止: final write_seq=%llu\n",
               (unsigned long long)current);
    }
    return 0;
}
