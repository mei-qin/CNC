#ifndef SIM_ENGINE_H
#define SIM_ENGINE_H

#include <stdio.h>
#include "axis_cfg.h"
#include "trace_logger.h"   // 引入 STAGE_RT_INTERPOLATOR 宏
#include <stdatomic.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

// ================== 高频无锁双缓冲仿真轨迹采集器 ==================
//
// 设计目标: g_sim_mode == 1 (极速仿真) 时, 以每周期 1 条的频率
// 无阻塞采集插补器物理坐标, 双缓冲交替落盘实现海量数据高速写入。
//
// 生产者: ecat_thread_rt (1ms 虚拟周期, 无锁写入活跃缓冲)
// 消费者: sim_flush_thread (后台非实时线程, 批量 fwrite 落盘)
//
// 架构: Double-Buffer Ping-Pong
//   RT 线程填满 Buffer[active_idx] → 原子交换 → sem_post 唤醒落盘线程
//   落盘线程检测 flush_pending[idx] → fwrite 整块写入 → 重置缓冲

// 每条轨迹采样记录 (AXIS_NUM=5 时 64 bytes 起步, 已含 stage_id 列)
// stage_id 恒为 STAGE_RT_INTERPOLATOR —— sim_engine 仅采集 1ms RT 插补物理波形
typedef struct {
    int      stage_id;         // 管线阶段标签 (固定 STAGE_RT_INTERPOLATOR)
    uint64_t cycle;            // RT 周期计数
    double   virtual_time_ms;  // 插补器虚拟时间 (ms)
    double   pos[AXIS_NUM];    // 各轴物理坐标 (mm / deg)
    double   v_target;         // 目标速度 (mm/ms)
} sim_trace_record_t;

// 二进制文件头 (固定 512 bytes, record_count 在关闭时回填)
#define SIM_FILE_HEADER_SIZE  512

#pragma pack(push, 1)
typedef struct {
    char     magic[4];           // "SIM1"
    uint32_t version;            // 1
    uint32_t axis_count;         // AXIS_NUM
    char     axis_names[8][16];  // 轴名表 (最多 8 轴)
    uint32_t record_size;        // sizeof(sim_trace_record_t)
    uint64_t record_count;       // 总记录数 (关闭时回填)
    uint8_t  reserved[SIM_FILE_HEADER_SIZE - 4 - 4 - 4 - 128 - 4 - 8];
} sim_file_header_t;
#pragma pack(pop)

// 双缓冲容量: 每个缓冲 1M 条 ≈ 64 MB (AXIS_NUM=5)
#define SIM_BUF_CAPACITY  (1 << 20)

// 双缓冲控制器
// RT 线程写端与落盘线程读端按 cache-line 隔离, 杜绝 false sharing
typedef struct {
    // --- 缓冲区 ---
    sim_trace_record_t *bufs[2];
    uint32_t            capacity;

    // --- RT 线程写端 (cache-line 独占) ---
    _Alignas(64) _Atomic uint32_t counts[2];
    _Alignas(64) _Atomic int      active_idx;

    // --- 落盘线程读端 (cache-line 独占) ---
    _Alignas(64) _Atomic int      flush_pending[2];
    _Alignas(64) _Atomic uint64_t total_records;
    _Alignas(64) _Atomic uint64_t dropped_records;

    // --- 线程控制 ---
    sem_t          flush_sem;
    pthread_t      flush_thread;
    volatile int   running;

    // --- 文件输出 ---
    char           output_path[260];
    FILE          *fp;
    int            use_binary;
    uint64_t       file_record_count;
    long           header_offset;
} sim_logger_t;

extern sim_logger_t g_sim_logger;

// ================== API ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
// 分配双缓冲内存, 创建输出文件, 写入文件头
int  sim_engine_init(const char *output_path, int use_binary);

// @Context: Non-RealTime Background Thread
// 启动后台落盘线程
int  sim_engine_start(void);

// @Context: Non-RealTime Background Thread (仿真结束后调用)
// 刷新残余数据, 等待落盘线程退出, 回填文件头, 关闭文件
void sim_engine_finish(void);

// @Context: 1ms Hard-RT Thread (ecat_thread_rt 内调用)
// @Danger: 无锁, 无阻塞, 无 printf, 无 malloc。
// RT 线程每周期调用: 写入插补器坐标到活跃缓冲;
// 缓冲满时原子交换 active_idx 并 sem_post 唤醒落盘线程。
static inline void sim_engine_push(uint64_t cycle, double virtual_time_ms,
                                     const double pos[AXIS_NUM], double v_target);

// 落盘线程入口
void *sim_flush_thread_func(void *arg);

// ================== inline 实现 ==================

// @Context: 1ms Hard-RT Thread (EtherCAT / 仿真)
// @Danger: NO BLOCKING, NO PRINTF, NO MALLOC.
// 双缓冲写入: 活跃缓冲满 → 检查另一缓冲可用性 → 交换或静默丢弃。
// 绝不自旋等待: 若另一缓冲仍在落盘, 直接丢弃当前记录 (不阻塞 RT 线程)。
static inline void sim_engine_push(uint64_t cycle, double virtual_time_ms,
                                     const double pos[AXIS_NUM], double v_target)
{
    sim_logger_t *L = &g_sim_logger;
    int idx = atomic_load_explicit(&L->active_idx, memory_order_relaxed);
    uint32_t count = atomic_load_explicit(&L->counts[idx], memory_order_relaxed);

    if (count >= L->capacity) {
        int next = 1 - idx;

        // 先检查另一缓冲是否可用 (已在 flush_pending == 0 表示落盘完成)
        if (atomic_load_explicit(&L->flush_pending[next], memory_order_acquire)) {
            // 另一缓冲仍在落盘 → 静默丢弃 (绝不自旋等待)
            atomic_fetch_add_explicit(&L->dropped_records, 1, memory_order_relaxed);
            return;
        }

        // 另一缓冲可用: 标记当前缓冲待落盘, 交换
        atomic_store_explicit(&L->flush_pending[idx], 1, memory_order_release);
        atomic_store_explicit(&L->counts[next], 0, memory_order_relaxed);
        atomic_store_explicit(&L->active_idx, next, memory_order_release);
        sem_post(&L->flush_sem);

        idx = next;
        count = 0;
    }

    // 写入记录 (纯内存操作, 零系统调用)
    sim_trace_record_t *r = &L->bufs[idx][count];
    r->stage_id        = STAGE_RT_INTERPOLATOR;  // 标记 1ms RT 插补器物理执行阶段
    r->cycle           = cycle;
    r->virtual_time_ms = virtual_time_ms;
    for (int i = 0; i < AXIS_NUM; i++) r->pos[i] = pos[i];
    r->v_target        = v_target;

    // Release store: 确保记录数据对落盘线程可见后再更新 count
    atomic_store_explicit(&L->counts[idx], count + 1, memory_order_release);
    atomic_fetch_add_explicit(&L->total_records, 1, memory_order_relaxed);
}

#endif // SIM_ENGINE_H
