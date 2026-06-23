#include "trace_logger.h"
#include "global_def.h"
#include "soem/soem.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>

// ================== 全局队列 ==================
TraceQueue_t          g_trace_queue;
PipelineTraceQueue_t  g_pipeline_queue;

static volatile int trace_shutdown_request = 0;

// ================== 初始化 ==================
// @Context: Non-RealTime Background Thread (初始化阶段)
void TraceLogger_Init(void)
{
    memset(&g_trace_queue, 0, sizeof(TraceQueue_t));
    memset(&g_pipeline_queue, 0, sizeof(PipelineTraceQueue_t));
    pthread_mutex_init(&g_pipeline_queue.lock, NULL);
    atomic_store_explicit(&g_pipeline_queue.seq, 0, memory_order_relaxed);
    trace_shutdown_request = 0;
    printf("[Trace] 无锁探针队列初始化完成 (RT SPSC=%d, Pipeline MPMC=%d)\n",
           TRACE_RING_SIZE, PIPELINE_RING_SIZE);
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

// ================== 生产者 A：RT 线程 (SPSC lock-free) ==================
//
// SPSC: 仅 RT 线程写 head，仅 logger 线程写 tail。
// 使用 acquire/release ordering:
//   - Push: release store on head (数据写入先于 head 更新可见)
//   - 满检查: acquire load on tail (读最新的 tail)
//   - 队列满时丢弃 — 绝不阻塞 RT 线程
//
// @Context: 1ms Hard-RT Thread
// @Danger: 无阻塞, 无 printf, 无 malloc。最坏情况 O(1) 丢弃。
// stage_id 强制写为 STAGE_RT_INTERPOLATOR (调用方无需关心)
void TraceLogger_Push(int32_t cycle, double virtual_time_ms,
                       double x, double y, double z,
                       double b, double c, double v_target)
{
    int head = atomic_load_explicit(&g_trace_queue.head, memory_order_relaxed);
    int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_acquire);
    int next = (head + 1) % TRACE_RING_SIZE;

    if (next == tail) return;   // 队列满 → 丢弃 (不阻塞 RT 线程)

    TraceEntry_t *entry = &g_trace_queue.buffer[head];
    entry->stage_id        = STAGE_RT_INTERPOLATOR;
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

// ================== 生产者 B：后台管线 (MPMC, mutex 保护) ==================
//
// @Context: Non-RealTime Background Thread (parser / cutter_comp / bspline)
// @Thread-Safety: pthread_mutex 保护, 三个后台线程可并发调用。
// @Safe: 队列满时静默丢弃 (mutex 立即释放, 不阻塞调用线程)。
//
// 后台探针的 cycle 字段填全局递增 seq (与 RT 周期计数无语义重叠,
// 通过 stage_id 列区分); virtual_time_ms 填 0.0 (后台无 RT 时钟概念)。
void TraceLogger_PushPipeline(int stage_id,
                               const double pos[AXIS_NUM],
                               double v_target)
{
    // 从动态轴映射提取 X/Y/Z/B/C (未映射轴填 0.0)
    int idx_x = g_axis_map['X' - 'A'];
    int idx_y = g_axis_map['Y' - 'A'];
    int idx_z = g_axis_map['Z' - 'A'];
    int idx_b = g_axis_map['B' - 'A'];
    int idx_c = g_axis_map['C' - 'A'];

    int seq = atomic_fetch_add_explicit(&g_pipeline_queue.seq, 1,
                                         memory_order_relaxed);

    pthread_mutex_lock(&g_pipeline_queue.lock);
    int next = (g_pipeline_queue.head + 1) % PIPELINE_RING_SIZE;
    if (next == g_pipeline_queue.tail) {
        // 队列满 → 丢弃 (mutex 立即释放)
        pthread_mutex_unlock(&g_pipeline_queue.lock);
        return;
    }

    TraceEntry_t *entry = &g_pipeline_queue.buffer[g_pipeline_queue.head];
    entry->stage_id        = stage_id;
    entry->cycle           = seq;
    entry->virtual_time_ms = 0.0;  // 后台无 RT 时钟概念
    entry->x_mm            = (idx_x >= 0) ? pos[idx_x] : 0.0;
    entry->y_mm            = (idx_y >= 0) ? pos[idx_y] : 0.0;
    entry->z_mm            = (idx_z >= 0) ? pos[idx_z] : 0.0;
    entry->b_deg           = (idx_b >= 0) ? pos[idx_b] : 0.0;
    entry->c_deg           = (idx_c >= 0) ? pos[idx_c] : 0.0;
    entry->v_target        = v_target;

    g_pipeline_queue.head = next;
    pthread_mutex_unlock(&g_pipeline_queue.lock);
}

// ================== 单条记录 → CSV 行 (统一格式) ==================
// 输出列序: cycle, stage_id, virtual_time_ms, x_mm, y_mm, z_mm, b_deg, c_deg, v_target
static int trace_entry_to_csv(char *buf, int buf_size, const TraceEntry_t *e)
{
    return snprintf(buf, (size_t)buf_size,
                    "%d,%d,%.3f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                    e->cycle,
                    e->stage_id,
                    e->virtual_time_ms,
                    e->x_mm,
                    e->y_mm,
                    e->z_mm,
                    e->b_deg,
                    e->c_deg,
                    e->v_target);
}

// ================== 消费者：落盘线程 ==================
//
// @Context: Non-RealTime Background Thread
// @Safe: Math functions, blocking, and I/O are allowed.
//
// 每 100ms 醒来，依次排空 RT SPSC 队列 + 后台 MPMC 队列，统一写入 CSV。
// 收到 shutdown 请求后排空残余数据再退出。
OSAL_THREAD_FUNC TraceLogger_ThreadFunc(void *arg)
{
    (void)arg;

    FILE *fp = fopen("cnc_trace_log.csv", "w");
    if (fp == NULL) {
        printf("[Trace] 无法创建 cnc_trace_log.csv！\n");
        return NULL;
    }

    // CSV 表头 (新增 stage_id 列, 位于 cycle 与 time 之间)
    fprintf(fp, "cycle,stage_id,virtual_time_ms,x_mm,y_mm,z_mm,b_deg,c_deg,v_target\n");
    fflush(fp);

    char line[TRACE_CSV_BUF];
    int written_count = 0;
    int flush_counter = 0;

    while (!trace_shutdown_request) {
        osal_usleep(100000); // 100ms 周期

        int flushed_this_cycle = 0;

        // ---- 1. 排空 RT SPSC 队列 (STAGE_RT_INTERPOLATOR) ----
        while (1) {
            int head = atomic_load_explicit(&g_trace_queue.head, memory_order_acquire);
            int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_relaxed);

            if (tail == head) break; // 队列空

            TraceEntry_t *entry = &g_trace_queue.buffer[tail];
            int len = trace_entry_to_csv(line, sizeof(line), entry);
            if (len > 0 && len < (int)sizeof(line)) {
                fputs(line, fp);
                written_count++;
                flushed_this_cycle++;
            }

            int next = (tail + 1) % TRACE_RING_SIZE;
            atomic_store_explicit(&g_trace_queue.tail, next, memory_order_release);
        }

        // ---- 2. 排空后台 MPMC 管线队列 (STAGE_PARSER/CUTTER_COMP/BSPLINE) ----
        pthread_mutex_lock(&g_pipeline_queue.lock);
        while (1) {
            if (g_pipeline_queue.tail == g_pipeline_queue.head) break;

            TraceEntry_t *entry = &g_pipeline_queue.buffer[g_pipeline_queue.tail];
            int len = trace_entry_to_csv(line, sizeof(line), entry);
            if (len > 0 && len < (int)sizeof(line)) {
                fputs(line, fp);
                written_count++;
                flushed_this_cycle++;
            }

            g_pipeline_queue.tail = (g_pipeline_queue.tail + 1) % PIPELINE_RING_SIZE;
        }
        pthread_mutex_unlock(&g_pipeline_queue.lock);

        // 每 10 次循环或有数据时 flush
        flush_counter++;
        if (flushed_this_cycle > 0 && flush_counter >= 10) {
            fflush(fp);
            flush_counter = 0;
        }
    }

    // ---- 关机排空: 把残余数据写完 ----
    // RT SPSC 残余
    while (1) {
        int head = atomic_load_explicit(&g_trace_queue.head, memory_order_acquire);
        int tail = atomic_load_explicit(&g_trace_queue.tail, memory_order_relaxed);
        if (tail == head) break;
        trace_entry_to_csv(line, sizeof(line), &g_trace_queue.buffer[tail]);
        fputs(line, fp);
        atomic_store_explicit(&g_trace_queue.tail,
                              (tail + 1) % TRACE_RING_SIZE,
                              memory_order_release);
    }
    // Pipeline MPMC 残余
    pthread_mutex_lock(&g_pipeline_queue.lock);
    while (g_pipeline_queue.tail != g_pipeline_queue.head) {
        trace_entry_to_csv(line, sizeof(line),
                           &g_pipeline_queue.buffer[g_pipeline_queue.tail]);
        fputs(line, fp);
        g_pipeline_queue.tail = (g_pipeline_queue.tail + 1) % PIPELINE_RING_SIZE;
    }
    pthread_mutex_unlock(&g_pipeline_queue.lock);

    fflush(fp);
    fclose(fp);
    printf("[Trace] 落盘完成, 共写入 %d 条记录 (含管线 stage 标签)\n", written_count);
    return NULL;
}
