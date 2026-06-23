#ifndef BSPLINE_ENGINE_H
#define BSPLINE_ENGINE_H

#include "axis_cfg.h"
#include "soem/soem.h"
#include "osal_compat.h"
#include <pthread.h>

// ================== B-Spline 压缩平滑引擎 ==================
// 独立后台线程，从解析器接收 G01 微段，通过 3 阶 B 样条全局拟合后
// 以极致均匀步长重新离散，推入底层 g_cmd_queue。
// 仅处理 G01 非 RTCP 直线段，G00/G02/G03/RTCP 均直通。

#define BSPLINE_DIRTY_QUEUE_SIZE   1024  // 脏点队列容量

#define BSPLINE_DEGREE             3     // 3 阶 B 样条
#define BSPLINE_ORDER              (BSPLINE_DEGREE + 1)  // order = degree + 1 = 4

#define BSPLINE_WAKE_THRESHOLD     30    // 脏点累积数达到此值唤醒计算线程
#define BSPLINE_SHARP_ANGLE_DEG    30.0  // 尖角判定阈值 (度)
#define BSPLINE_STEP_SIZE_MM       0.2   // 重离散化均匀步长 (mm)
#define BSPLINE_MAX_CTRL_POINTS    128   // 单批次最大控制点数
#define BSPLINE_KNOT_VEC_MAX       (BSPLINE_MAX_CTRL_POINTS + BSPLINE_ORDER + 1)

// 弧长反查 LUT 采样密度
#define BSPLINE_ARC_LUT_SAMPLES    256

// ================== 数据结构 ==================

// 脏点：解析器推入的原始运动点
typedef struct {
    double pos[AXIS_NUM];    // 机械绝对坐标 (mm / deg)
    double speed;            // 合成速度 (mm/s)，已经过 G94/G93 换算
    double g93_time;         // G93 时间预算 (秒)，0.0 = G94 模式
} DirtyPoint_t;

// 脏点环形队列 (parser 生产, B-Spline 线程消费)
typedef struct {
    DirtyPoint_t buffer[BSPLINE_DIRTY_QUEUE_SIZE];
    volatile int head;       // 下一个写入位置 (producer 推进)
    volatile int tail;       // 下一个读取位置 (consumer 推进)
} SplineDirtyQueue_t;

// 引擎配置参数
typedef struct {
    int    enabled;           // 总开关: 1=启用, 0=直通 (parser 直接调 api_push_trajectory)
    double sharp_angle_rad;   // 尖角阈值 (弧度)，预计算 = 30 * DEG_TO_RAD
    double step_size_mm;      // 重离散化步长 (mm)
    int    wake_threshold;    // 唤醒计算阈值 (脏点数)
    int    cpu_core;          // 线程绑定的 CPU 核心号
} BSplineConfig_t;

// ================== 公共 API ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
// 初始化 B-Spline 引擎状态、互斥锁、条件变量。
// 必须在 axis_sys_init() 之后、线程创建之前调用。
void BSpline_Init(void);

// @Context: Non-RealTime Background Thread (SMC_InitAndStart 内调用)
// 启动 B-Spline 后台线程，绑定到配置的 CPU 核心。
// 返回 0=成功, -1=失败。
int  BSpline_StartThread(void);

// @Context: Non-RealTime Background Thread (SMC_Close 内调用)
// 停止 B-Spline 后台线程。先排空脏点队列中的剩余点，再退出。
void BSpline_StopThread(void);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: bspline_mutex 保护队列写入
// 向脏点队列推入一个原始运动点 (替代 api_push_trajectory)。
// pos:   机械绝对坐标
// speed: 合成速度 mm/s
// g93_time: G93 时间预算 (秒)，0.0=G94
// 返回 0=成功, -1=队列满且系统报警/关闭
int  BSpline_PushDirtyPoint(double pos[AXIS_NUM], double speed, double g93_time);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: 内部加锁 + 信号通知
// 强制触发 B-Spline 计算线程处理当前队列中的所有脏点。
// 用于 M 代码前 (保证指令顺序) 或文件结束 (排空尾部)。
void BSpline_Flush(void);

// @Context: Non-RealTime Background Thread (初始化阶段)
// 配置 B-Spline 引擎参数。仅在停机状态下调用。
void BSpline_Configure(const BSplineConfig_t *config);

// 获取当前配置 (只读)
const BSplineConfig_t* BSpline_GetConfig(void);

// B-Spline 后台线程入口 (由 osal_thread_create 调用)
OSAL_THREAD_FUNC bspline_thread_func(void *arg);

#endif // BSPLINE_ENGINE_H
