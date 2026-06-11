#ifndef TRACE_LOGGER_H
#define TRACE_LOGGER_H

#include "axis_cfg.h"
#include <stdatomic.h>
#include <stdint.h>

// ================== 无锁异步数据日志探针 ==================
// 生产者: ecat_thread_rt (1ms 硬实时线程, cycle%5 采样)
// 消费者: trace_logger_thread (100ms 落盘线程)
// 队列策略: SPSC 无锁环形队列, 满时丢弃 (绝不阻塞 RT 线程)

#define TRACE_RING_SIZE  8192    // 环形队列容量 (约 40s @ 5ms 采样)
#define TRACE_CSV_BUF    2048    // CSV 行缓冲区

// 单条轨迹采样记录
typedef struct {
    int32_t  cycle;              // RT 周期计数
    double   virtual_time_ms;    // 插补器虚拟时间 (ms)
    double   x_mm;               // X 轴物理坐标 (mm)
    double   y_mm;               // Y 轴物理坐标 (mm)
    double   z_mm;               // Z 轴物理坐标 (mm)
    double   b_deg;              // B 轴角度 (deg)
    double   c_deg;              // C 轴角度 (deg)
    double   v_target;           // 目标速度 (mm/ms)
} TraceEntry_t;

// 无锁 SPSC 环形队列
// 生产者: RT 线程 (唯一写入 head)
// 消费者: 落盘线程 (唯一写入 tail)
typedef struct {
    TraceEntry_t buffer[TRACE_RING_SIZE];
    _Atomic int  head;           // 生产者偏移
    _Atomic int  tail;           // 消费者偏移
} TraceQueue_t;

extern TraceQueue_t g_trace_queue;

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
void TraceLogger_Push(int32_t cycle, double virtual_time_ms,
                       double x, double y, double z,
                       double b, double c, double v_target);

// 落盘线程入口 (由 osal_thread_create 调用)
OSAL_THREAD_FUNC TraceLogger_ThreadFunc(void *arg);

#endif // TRACE_LOGGER_H
