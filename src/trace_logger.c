#include "trace_logger.h"
#include "global_def.h"
#include "soem/soem.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>

// ================== 全局队列 ==================
TraceQueue_t g_trace_queue;

static volatile int trace_shutdown_request = 0;

// ================== 初始化 ==================
// @Context: Non-RealTime Background Thread (初始化阶段)
void TraceLogger_Init(void)
{
    memset(&g_trace_queue, 0, sizeof(TraceQueue_t));
    trace_shutdown_request = 0;
    printf("[Trace] 无锁探针队列初始化完成 (容量=%d)\n", TRACE_RING_SIZE);
}

// ================== 启动落盘线程 ==================
// @Context: Non-RealTime Background Thread
int TraceLogger_StartThread(void)
{
    static OSAL_THREAD_HANDLE thread_trace;
    if (!osal_thread_create(&thread_trace, 64000, &TraceLogger_ThreadFunc, NULL)) {
        printf("[Trace] 落盘线程创建失败！\n");
        return -1;
    }
    printf("[Trace] 落盘线程已启动\n");
    return 0;
}

// ================== 停止 ==================
// @Context: Non-RealTime Background Thread (SMC_Close 内调用)
void TraceLogger_StopThread(void)
{
    trace_shutdown_request = 1;
    // 给落盘线程时间排空残余数据
    osal_usleep(500000); // 500ms
}

// ================== 生产者：RT 线程 (lock-free) ==================
//
// SPSC: 仅 RT 线程写 head，仅 logger 线程写 tail。
// 使用 acquire/release ordering:
//   - Push: release store on head (数据写入先于 head 更新可见)
//   - 满检查: acquire load on tail (读最新的 tail)
//   - 队列满时丢弃 — 绝不阻塞 RT 线程
//
// @Context: 1ms Hard-RT Thread
// @Danger: 无阻塞, 无 printf, 无 malloc。最坏情况 O(1) 丢弃。
void TraceLogger_Push(int32_t cycle, double virtual_time_ms,
                       double x, double y, double z,
                       double b, double c, double v_target)
{
    int head = atomic_load_explicit(&g_trace_queue.head, memory_order_relaxed);
    int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_acquire);
    int next = (head + 1) % TRACE_RING_SIZE;

    if (next == tail) return;   // 队列满 → 丢弃 (不阻塞 RT 线程)

    TraceEntry_t *entry = &g_trace_queue.buffer[head];
    entry->cycle           = cycle;
    entry->virtual_time_ms = virtual_time_ms;
    entry->x_mm            = x;
    entry->y_mm            = y;
    entry->z_mm            = z;
    entry->b_deg           = b;
    entry->c_deg           = c;
    entry->v_target        = v_target;

    atomic_store_explicit(&g_trace_queue.head, next, memory_order_release);
}

// ================== 消费者：落盘线程 ==================
//
// @Context: Non-RealTime Background Thread
// @Safe: Math functions, blocking, and I/O are allowed.
//
// 每 100ms 醒来，排空队列写入 CSV。
// 收到 shutdown 请求后排空残余数据再退出。
OSAL_THREAD_FUNC TraceLogger_ThreadFunc(void *arg)
{
    (void)arg;

    FILE *fp = fopen("cnc_trace_log.csv", "w");
    if (fp == NULL) {
        printf("[Trace] 无法创建 cnc_trace_log.csv！\n");
        return NULL;
    }

    // CSV 表头
    fprintf(fp, "cycle,virtual_time_ms,x_mm,y_mm,z_mm,b_deg,c_deg,v_target\n");
    fflush(fp);

    char line[TRACE_CSV_BUF];
    int written_count = 0;
    int flush_counter = 0;

    while (!trace_shutdown_request) {
        osal_usleep(100000); // 100ms 周期

        int flushed_this_cycle = 0;

        while (1) {
            int head = atomic_load_explicit(&g_trace_queue.head, memory_order_acquire);
            int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_relaxed);

            if (tail == head) break; // 队列空

            TraceEntry_t *entry = &g_trace_queue.buffer[tail];
            int len = snprintf(line, sizeof(line),
                               "%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                               entry->cycle,
                               entry->virtual_time_ms,
                               entry->x_mm,
                               entry->y_mm,
                               entry->z_mm,
                               entry->b_deg,
                               entry->c_deg,
                               entry->v_target);
            if (len > 0 && len < (int)sizeof(line)) {
                fputs(line, fp);
                written_count++;
                flushed_this_cycle++;
            }

            int next = (tail + 1) % TRACE_RING_SIZE;
            atomic_store_explicit(&g_trace_queue.tail, next, memory_order_release);
        }

        // 每 10 次循环或有数据时 flush
        flush_counter++;
        if (flushed_this_cycle > 0 && flush_counter >= 10) {
            fflush(fp);
            flush_counter = 0;
        }
    }

    // ---- 关机排空: 把残余数据写完 ----
    while (1) {
        int head = atomic_load_explicit(&g_trace_queue.head, memory_order_acquire);
        int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_relaxed);
        if (tail == head) break;

        TraceEntry_t *entry = &g_trace_queue.buffer[tail];
        fprintf(fp, "%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                entry->cycle, entry->virtual_time_ms,
                entry->x_mm, entry->y_mm, entry->z_mm,
                entry->b_deg, entry->c_deg, entry->v_target);

        int next = (tail + 1) % TRACE_RING_SIZE;
        atomic_store_explicit(&g_trace_queue.tail, next, memory_order_release);
    }

    fflush(fp);
    fclose(fp);
    printf("[Trace] 落盘完成, 共写入 %d 条记录\n", written_count);
    return NULL;
}
