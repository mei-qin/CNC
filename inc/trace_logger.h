#ifndef TRACE_LOGGER_H
#define TRACE_LOGGER_H

#include "axis_cfg.h"
#include "soem/soem.h"
#include "osal_compat.h"
#include <stdatomic.h>
#include <stdint.h>
#include <pthread.h>

// ================== 无锁异步数据日志探针 ==================
// 生产者: ecat_thread_rt (1ms 硬实时线程, cycle%5 采样)
// 消费者: trace_logger_thread (100ms 落盘线程)
// 队列策略: SPSC 无锁环形队列, 满时丢弃 (绝不阻塞 RT 线程)

#define TRACE_RING_SIZE  8192    // 环形队列容量 (约 40s @ 5ms 采样)
#define TRACE_CSV_BUF    2048    // CSV 行缓冲区

// ================== 多级管线探针 (Pipeline Trace) ==================
//
// 一条 G 指令从解析到 1ms 物理执行的全生命周期形态变化,
// 按管线阶段分别埋点,统一写入同一份 CSV (新增 stage_id 列)。
//
//   STAGE_PARSER         —— G 代码原始意图 (machine_target_pos)
//   STAGE_CUTTER_COMP    —— 刀补引擎输出 (偏置后坐标)
//   STAGE_BSPLINE        —— B 样条平滑引擎输出 (等距重采样点)
//   STAGE_PLANNER        —— 速度规划后 (预留, 当前管线无独立埋点)
//   STAGE_RT_INTERPOLATOR—— 1ms 硬实时插补器物理执行坐标
#define STAGE_PARSER          1
#define STAGE_CUTTER_COMP     2
#define STAGE_BSPLINE         3
#define STAGE_PLANNER         4
#define STAGE_RT_INTERPOLATOR 5

// 单条轨迹采样记录 (RT SPSC 队列 + 后台 MPMC 队列共用同一记录格式)
typedef struct {
    int      stage_id;          // 管线阶段标签 (STAGE_*)
    int32_t  cycle;             // RT 周期计数 (后台 stage 填全局 seq)
    double   virtual_time_ms;   // 插补器虚拟时间 (ms) — 后台 stage 填 0.0
    double   x_mm;              // X 轴物理坐标 (mm)
    double   y_mm;              // Y 轴物理坐标 (mm)
    double   z_mm;              // Z 轴物理坐标 (mm)
    double   b_deg;             // B 轴角度 (deg)
    double   c_deg;             // C 轴角度 (deg)
    double   v_target;          // 目标速度 (mm/ms)
} TraceEntry_t;

// 无锁 SPSC 环形队列
// 生产者: RT 线程 (唯一写入 head)
// 消费者: 落盘线程 (唯一写入 tail)
typedef struct {
    TraceEntry_t buffer[TRACE_RING_SIZE];
    _Atomic int  head;           // 生产者偏移
    _Atomic int  tail;           // 消费者偏移
} TraceQueue_t;

// 后台管线 MPMC 环形队列
// 生产者: parser / cutter_comp / bspline 三个 Non-RT 线程 (共享互斥锁)
// 消费者: 落盘线程 (单独加锁消费)
// 策略: 满时静默丢弃 (不阻塞后台线程, 也不污染 RT 队列的 SPSC 不变量)
#define PIPELINE_RING_SIZE  4096
typedef struct {
    TraceEntry_t   buffer[PIPELINE_RING_SIZE];
    pthread_mutex_t lock;
    int            head;         // 下一个写入位置 (生产者推进)
    int            tail;         // 下一个读取位置 (消费者推进)
    _Atomic int    seq;          // 后台探针全局递增序列号 (写入 entry->cycle)
} PipelineTraceQueue_t;

extern TraceQueue_t          g_trace_queue;
extern PipelineTraceQueue_t g_pipeline_queue;

// @Context: Non-RealTime Background Thread (初始化阶段)
// 初始化队列 (head=tail=0), 重置关闭标志
void TraceLogger_Init(void);

// @Context: Non-RealTime Background Thread (SMC_InitAndStart 内调用)
// 启动落盘线程。返回 0=成功, -1=失败
int  TraceLogger_StartThread(void);

// @Context: Non-RealTime Background Thread (SMC_Close 内调用)
// 通知落盘线程排空队列并退出
void TraceLogger_StopThread(void);

// @Context: 1ms Hard-RT Thread (ecat_thread_rt 内调用)
// @Danger: 无锁, 无阻塞, 无 printf。队列满时静默丢弃。
// 生产者接口: RT 线程每 5ms 且 is_moving==1 时调用
// 内部自动打 STAGE_RT_INTERPOLATOR 标签
void TraceLogger_Push(int32_t cycle, double virtual_time_ms,
                       double x, double y, double z,
                       double b, double c, double v_target);

// @Context: Non-RealTime Background Thread (parser / cutter_comp / bspline)
// @Thread-Safety: 内部 pthread_mutex 保护, 多后台线程可并发调用。
// @Safe: 队列满时静默丢弃 (绝阻塞后台线程)。
// 管线探针通用接口: 自动从 g_axis_map 提取 X/Y/Z/B/C,
// 自动递增 seq 写入 cycle, virtual_time_ms 填 0.0。
// stage_id: STAGE_PARSER / STAGE_CUTTER_COMP / STAGE_BSPLINE / STAGE_PLANNER
// pos[AXIS_NUM]: 该阶段的机械绝对坐标
// v_target: 该阶段的合成速度 (mm/ms); 无意义时传 0.0
void TraceLogger_PushPipeline(int stage_id,
                               const double pos[AXIS_NUM],
                               double v_target);

// 落盘线程入口 (由 osal_thread_create 调用)
OSAL_THREAD_FUNC TraceLogger_ThreadFunc(void *arg);

#endif // TRACE_LOGGER_H
