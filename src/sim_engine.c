#include "sim_engine.h"
#include "global_def.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

// ================== 全局实例 ==================
sim_logger_t g_sim_logger;

// ================== 初始化 ==================
// @Context: Non-RealTime Background Thread (初始化阶段)
// @Safe: Math functions, blocking, and I/O are allowed.
int sim_engine_init(const char *output_path, int use_binary)
{
    memset(&g_sim_logger, 0, sizeof(sim_logger_t));

    g_sim_logger.capacity = SIM_BUF_CAPACITY;
    size_t buf_size = (size_t)SIM_BUF_CAPACITY * sizeof(sim_trace_record_t);

    // 分配双缓冲 (每个 64 MB, 共 128 MB)
    g_sim_logger.bufs[0] = (sim_trace_record_t *)malloc(buf_size);
    g_sim_logger.bufs[1] = (sim_trace_record_t *)malloc(buf_size);
    if (!g_sim_logger.bufs[0] || !g_sim_logger.bufs[1]) {
        printf("[SimEngine] 内存分配失败! 需要 %zu MB x 2\n", buf_size / (1024 * 1024));
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }
    memset(g_sim_logger.bufs[0], 0, buf_size);
    memset(g_sim_logger.bufs[1], 0, buf_size);

    // 原子变量初始化
    atomic_store_explicit(&g_sim_logger.counts[0], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.counts[1], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.active_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.flush_pending[0], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.flush_pending[1], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.total_records, 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.dropped_records, 0, memory_order_relaxed);

    g_sim_logger.use_binary = use_binary;
    g_sim_logger.file_record_count = 0;
    g_sim_logger.running = 1;
    strncpy(g_sim_logger.output_path, output_path,
            sizeof(g_sim_logger.output_path) - 1);

    // 创建输出文件
    g_sim_logger.fp = fopen(output_path, use_binary ? "wb" : "w");
    if (!g_sim_logger.fp) {
        printf("[SimEngine] 无法创建输出文件: %s\n", output_path);
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }

    // 写入文件头
    if (use_binary) {
        sim_file_header_t header;
        memset(&header, 0, SIM_FILE_HEADER_SIZE);
        memcpy(header.magic, "SIM1", 4);
        header.version     = 1;
        header.axis_count  = AXIS_NUM;
        header.record_size = (uint32_t)sizeof(sim_trace_record_t);
        header.record_count = 0;

        for (int i = 0; i < AXIS_NUM && i < 8; i++) {
            strncpy(header.axis_names[i], g_axis[i].axis_name, 15);
            header.axis_names[i][15] = '\0';
        }

        fwrite(&header, SIM_FILE_HEADER_SIZE, 1, g_sim_logger.fp);
        fflush(g_sim_logger.fp);

        // 记录 record_count 字段在文件中的偏移 (关闭时回填)
        g_sim_logger.header_offset = (long)offsetof(sim_file_header_t, record_count);
    } else {
        // CSV 表头 (新增 stage_id 列, 位于 cycle 与 time 之间)
        fprintf(g_sim_logger.fp, "cycle,stage_id,virtual_time_ms");
        for (int i = 0; i < AXIS_NUM; i++)
            fprintf(g_sim_logger.fp, ",%s", g_axis[i].axis_name);
        fprintf(g_sim_logger.fp, ",v_target\n");
        fflush(g_sim_logger.fp);
    }

    // 初始化信号量 (初始值 0, RT 线程 post 时唤醒)
    if (sem_init(&g_sim_logger.flush_sem, 0, 0) != 0) {
        printf("[SimEngine] 信号量初始化失败!\n");
        fclose(g_sim_logger.fp);
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }

    printf("[SimEngine] 双缓冲采集器就绪 (容量=%d x2, %zu MB/buf, 格式=%s, 文件=%s)\n",
           SIM_BUF_CAPACITY, buf_size / (1024 * 1024),
           use_binary ? "BINARY" : "CSV", output_path);
    return 0;
}

// ================== 启动落盘线程 ==================
// @Context: Non-RealTime Background Thread
int sim_engine_start(void)
{
    if (pthread_create(&g_sim_logger.flush_thread, NULL,
                        sim_flush_thread_func, &g_sim_logger) != 0) {
        printf("[SimEngine] 落盘线程创建失败!\n");
        return -1;
    }
    printf("[SimEngine] 落盘线程已启动\n");
    return 0;
}

// ================== 落盘线程 ==================
// @Context: Non-RealTime Background Thread
// @Safe: Math functions, blocking, and I/O are allowed.
//
// 阻塞在 sem_wait, 被 RT 线程的 sem_post 唤醒后:
// 1. 扫描两个缓冲的 flush_pending 标志
// 2. 对标记为 1 的缓冲: 读取 counts[i] 条记录, fwrite 整块落盘
// 3. 重置 flush_pending[i] = 0 (允许 RT 线程再次使用该缓冲)
//
// 关闭时: RT 线程设置 running=0 并 sem_post, 本线程排空残余后退出。
void *sim_flush_thread_func(void *arg)
{
    sim_logger_t *L = (sim_logger_t *)arg;

    while (1) {
        sem_wait(&L->flush_sem);

        int did_flush = 0;

        for (int i = 0; i < 2; i++) {
            if (!atomic_load_explicit(&L->flush_pending[i], memory_order_acquire))
                continue;

            uint32_t cnt = atomic_load_explicit(&L->counts[i], memory_order_acquire);
            if (cnt == 0) {
                atomic_store_explicit(&L->flush_pending[i], 0, memory_order_release);
                continue;
            }

            if (L->use_binary) {
                // 二进制落盘: 单次 fwrite 写入整块 (极高速)
                size_t written = fwrite(L->bufs[i], sizeof(sim_trace_record_t),
                                        cnt, L->fp);
                L->file_record_count += written;
            } else {
                // CSV 落盘: 逐行格式化 (较慢但可读, 用于调试)
                // 输出列序: cycle, stage_id, virtual_time_ms, x/y/z/b/c, v_target
                for (uint32_t j = 0; j < cnt; j++) {
                    sim_trace_record_t *r = &L->bufs[i][j];
                    fprintf(L->fp, "%" PRIu64 ",%d,%.3f",
                            r->cycle, r->stage_id, r->virtual_time_ms);
                    for (int a = 0; a < AXIS_NUM; a++)
                        fprintf(L->fp, ",%.6f", r->pos[a]);
                    fprintf(L->fp, ",%.6f\n", r->v_target);
                }
                L->file_record_count += cnt;
            }

            fflush(L->fp);
            atomic_store_explicit(&L->flush_pending[i], 0, memory_order_release);
            did_flush = 1;
        }

        // 关闭条件: running=0 且本轮无数据可刷
        if (!L->running && !did_flush) break;
    }

    return NULL;
}

// ================== 停止并回填 ==================
// @Context: Non-RealTime Background Thread (仿真结束后调用)
// @Safe: Math functions, blocking, and I/O are allowed.
void sim_engine_finish(void)
{
    if (!g_sim_logger.running) return;

    // 1. 将活跃缓冲的残余数据标记为待落盘
    int idx = atomic_load_explicit(&g_sim_logger.active_idx, memory_order_relaxed);
    uint32_t cnt = atomic_load_explicit(&g_sim_logger.counts[idx], memory_order_acquire);
    if (cnt > 0) {
        atomic_store_explicit(&g_sim_logger.flush_pending[idx], 1, memory_order_release);
    }

    // 2. 通知落盘线程退出 (先标记, 再唤醒)
    g_sim_logger.running = 0;
    sem_post(&g_sim_logger.flush_sem);

    // 3. 等待落盘线程完成
    pthread_join(g_sim_logger.flush_thread, NULL);

    // 4. 二进制模式: 回填文件头的 record_count
    if (g_sim_logger.use_binary && g_sim_logger.fp) {
        fseek(g_sim_logger.fp, g_sim_logger.header_offset, SEEK_SET);
        fwrite(&g_sim_logger.file_record_count, sizeof(uint64_t), 1, g_sim_logger.fp);
    }

    // 5. 关闭文件
    if (g_sim_logger.fp) {
        fflush(g_sim_logger.fp);
        fclose(g_sim_logger.fp);
        g_sim_logger.fp = NULL;
    }

    // 6. 释放内存
    free(g_sim_logger.bufs[0]);
    free(g_sim_logger.bufs[1]);
    g_sim_logger.bufs[0] = NULL;
    g_sim_logger.bufs[1] = NULL;

    sem_destroy(&g_sim_logger.flush_sem);

    printf("[SimEngine] 轨迹落盘完成, 共 %" PRIu64 " 条记录, 文件: %s\n",
           g_sim_logger.file_record_count, g_sim_logger.output_path);

    uint64_t dropped = atomic_load_explicit(&g_sim_logger.dropped_records, memory_order_relaxed);
    if (dropped > 0) {
        printf("[SimEngine] 警告: 丢弃 %" PRIu64 " 条记录 (落盘速度不足)\n", dropped);
    }
}
