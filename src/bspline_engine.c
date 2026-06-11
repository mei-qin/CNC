#include "bspline_engine.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "planner.h"
#include "gcode_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ================== 静态全局状态 ==================

static SplineDirtyQueue_t g_dirty_queue = {{{0}}, 0, 0};
static BSplineConfig_t g_bspline_config = {
    .enabled          = 0,
    .sharp_angle_rad  = 30.0 * DEG_TO_RAD,
    .step_size_mm     = BSPLINE_STEP_SIZE_MM,
    .wake_threshold   = BSPLINE_WAKE_THRESHOLD,
    .cpu_core         = 2
};

static pthread_mutex_t bspline_mutex;
static pthread_cond_t  bspline_cond;
static volatile int bspline_flush_request    = 0;
static volatile int bspline_shutdown_request = 0;

// ================== 静态工作缓冲区 (无 malloc) ==================

static double batch_ctrl[BSPLINE_MAX_CTRL_POINTS][AXIS_NUM];
static double batch_knots[BSPLINE_KNOT_VEC_MAX];
static double batch_speeds[BSPLINE_MAX_CTRL_POINTS];
static double batch_g93[BSPLINE_MAX_CTRL_POINTS];

// 弧长反查 LUT
typedef struct {
    double t;            // 参数
    double cum_arc;      // 累计弧长 (mm)
} ArcLutEntry_t;

static ArcLutEntry_t arc_lut[BSPLINE_ARC_LUT_SAMPLES + 1];
static int arc_lut_count = 0;

// ================== 前向声明 ==================

static void bspline_drain_queue(int count);
static void bspline_process_batch_from(int start, int count);
static void build_clamped_knot_vector(int n, double *knots);
static void de_boor_evaluate(int n, int p, const double *knots,
                              double ctrl[][AXIS_NUM], double t, double out[AXIS_NUM]);
static double compute_spline_arc_length(int n, int p, const double *knots,
                                         double ctrl[][AXIS_NUM],
                                         ArcLutEntry_t *lut, int *lut_count);
static double arc_length_to_t(double target_arc, const ArcLutEntry_t *lut, int lut_count);
static int  is_sharp_corner(double prev[AXIS_NUM], double corner[AXIS_NUM],
                             double next[AXIS_NUM], double threshold_rad);
static double compute_equivalent_dist(const double a[AXIS_NUM], const double b[AXIS_NUM]);

// ================== 公共 API 实现 ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
void BSpline_Init(void)
{
    memset(&g_dirty_queue, 0, sizeof(SplineDirtyQueue_t));
    pthread_mutex_init(&bspline_mutex, NULL);
    pthread_cond_init(&bspline_cond, NULL);
    bspline_flush_request    = 0;
    bspline_shutdown_request = 0;
    printf("[BSpline] 引擎初始化完成 (step=%.2fmm, wake=%d, core=%d)\n",
           g_bspline_config.step_size_mm,
           g_bspline_config.wake_threshold,
           g_bspline_config.cpu_core);
}

// @Context: Non-RealTime Background Thread
int BSpline_StartThread(void)
{
    if (!osal_thread_create(&thread_bspline, 128000, &bspline_thread_func, NULL)) {
        printf("[BSpline] 平滑线程创建失败！\n");
        return -1;
    }
    printf("[BSpline] 平滑线程已启动\n");
    return 0;
}

// @Context: Non-RealTime Background Thread
void BSpline_StopThread(void)
{
    pthread_mutex_lock(&bspline_mutex);
    bspline_shutdown_request = 1;
    pthread_cond_signal(&bspline_cond);
    pthread_mutex_unlock(&bspline_mutex);

    // 等待线程排空队列并退出 (最多 3 秒)
    int wait = 30;
    while (bspline_shutdown_request != 0 && wait > 0) {
        osal_usleep(100000);
        wait--;
    }
    if (bspline_shutdown_request != 0) {
        printf("[BSpline] 线程停止超时，强制退出\n");
        bspline_shutdown_request = 0;
    } else {
        printf("[BSpline] 线程已安全停止\n");
    }
}

// @Context: Non-RealTime Background Thread (parser)
// @Thread-Safety: bspline_mutex 保护队列写入
int BSpline_PushDirtyPoint(double pos[AXIS_NUM], double speed, double g93_time)
{
    if (atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) == 1) {
        return -1;
    }

    pthread_mutex_lock(&bspline_mutex);

    int next_head = (g_dirty_queue.head + 1) % BSPLINE_DIRTY_QUEUE_SIZE;
    while (next_head == g_dirty_queue.tail) {
        pthread_mutex_unlock(&bspline_mutex);
        if (bspline_shutdown_request) return -1;
        if (atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) == 1) return -1;
        osal_usleep(1000);
        pthread_mutex_lock(&bspline_mutex);
        next_head = (g_dirty_queue.head + 1) % BSPLINE_DIRTY_QUEUE_SIZE;
    }

    DirtyPoint_t *dp = &g_dirty_queue.buffer[g_dirty_queue.head];
    memcpy(dp->pos, pos, sizeof(double) * AXIS_NUM);
    dp->speed    = speed;
    dp->g93_time = g93_time;

    g_dirty_queue.head = next_head;

    int dirty_count = (g_dirty_queue.head - g_dirty_queue.tail
                       + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
    if (dirty_count >= g_bspline_config.wake_threshold) {
        pthread_cond_signal(&bspline_cond);
    }

    pthread_mutex_unlock(&bspline_mutex);
    return 0;
}

// @Context: Non-RealTime Background Thread (parser)
// @Thread-Safety: 内部加锁 + 信号通知
void BSpline_Flush(void)
{
    int dirty_count;
    pthread_mutex_lock(&bspline_mutex);
    dirty_count = (g_dirty_queue.head - g_dirty_queue.tail
                   + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
    if (dirty_count > 0) {
        bspline_flush_request = 1;
        pthread_cond_signal(&bspline_cond);
    }
    pthread_mutex_unlock(&bspline_mutex);

    // 等待 flush 完成 (排空队列)
    int wait = 500; // 最多 5 秒
    while (wait > 0) {
        pthread_mutex_lock(&bspline_mutex);
        dirty_count = (g_dirty_queue.head - g_dirty_queue.tail
                       + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
        int flush_done = (dirty_count == 0 && !bspline_flush_request);
        pthread_mutex_unlock(&bspline_mutex);
        if (flush_done) break;
        osal_usleep(10000);
        wait--;
    }
}

// @Context: Non-RealTime Background Thread (初始化阶段)
void BSpline_Configure(const BSplineConfig_t *config)
{
    if (config == NULL) return;
    memcpy(&g_bspline_config, config, sizeof(BSplineConfig_t));
    // 确保角度阈值合法
    if (g_bspline_config.sharp_angle_rad <= 0.0)
        g_bspline_config.sharp_angle_rad = 30.0 * DEG_TO_RAD;
    if (g_bspline_config.step_size_mm <= 0.01)
        g_bspline_config.step_size_mm = BSPLINE_STEP_SIZE_MM;
    if (g_bspline_config.wake_threshold < 4)
        g_bspline_config.wake_threshold = 4;
    printf("[BSpline] 配置更新: enabled=%d, step=%.2f, wake=%d, core=%d\n",
           g_bspline_config.enabled,
           g_bspline_config.step_size_mm,
           g_bspline_config.wake_threshold,
           g_bspline_config.cpu_core);
}

const BSplineConfig_t* BSpline_GetConfig(void)
{
    return &g_bspline_config;
}

// ================== 后台线程主函数 ==================

// @Context: Non-RealTime Background Thread
// @Safe: Math functions, blocking, and I/O are allowed.
OSAL_THREAD_FUNC bspline_thread_func(void *arg)
{
    (void)arg;

    // ---- CPU 亲和性绑定 ----
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(g_bspline_config.cpu_core, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
        printf("[BSpline] CPU 核心绑定失败 (core=%d)，继续运行\n",
               g_bspline_config.cpu_core);
    } else {
        printf("[BSpline] 已绑定到 CPU Core %d\n", g_bspline_config.cpu_core);
    }

    // ---- 主循环 ----
    while (!bspline_shutdown_request) {

        pthread_mutex_lock(&bspline_mutex);

        int dirty_count = (g_dirty_queue.head - g_dirty_queue.tail
                           + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;

        // 阻塞等待: 队列达到唤醒阈值 / flush 请求 / 关闭请求
        while (dirty_count < g_bspline_config.wake_threshold
               && !bspline_flush_request
               && !bspline_shutdown_request) {
            pthread_cond_wait(&bspline_cond, &bspline_mutex);
            dirty_count = (g_dirty_queue.head - g_dirty_queue.tail
                           + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
        }

        int snapshot_count = dirty_count;
        bspline_flush_request = 0;

        pthread_mutex_unlock(&bspline_mutex);

        if (snapshot_count > 0) {
            bspline_drain_queue(snapshot_count);
        }

        // 处理后再次检查队列: 若仍有残余点则立即继续 (防止滞留)
        pthread_mutex_lock(&bspline_mutex);
        int remaining = (g_dirty_queue.head - g_dirty_queue.tail
                         + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
        pthread_mutex_unlock(&bspline_mutex);
        if (remaining > 0) continue; // 不睡眠，立即处理残余
    }

    printf("[BSpline] 线程退出\n");
    bspline_shutdown_request = 0;
    return NULL;
}

// ================== 队列排空与批次处理 ==================

// @Context: Non-RealTime Background Thread (bspline_thread_func 内)
// 从脏点队列中取出 count 个点到 batch_ctrl 等工作缓冲区，
// 然后执行尖角分割 + B 样条拟合 + 重离散化 + 入队。
static void bspline_drain_queue(int count)
{
    if (count > BSPLINE_MAX_CTRL_POINTS) {
        count = BSPLINE_MAX_CTRL_POINTS;
    }

    // ---- 1. 加锁拷贝脏点到工作缓冲区 ----
    pthread_mutex_lock(&bspline_mutex);

    int actual = 0;
    while (actual < count && g_dirty_queue.tail != g_dirty_queue.head) {
        DirtyPoint_t *dp = &g_dirty_queue.buffer[g_dirty_queue.tail];
        memcpy(batch_ctrl[actual], dp->pos, sizeof(double) * AXIS_NUM);
        batch_speeds[actual] = dp->speed;
        batch_g93[actual]    = dp->g93_time;
        actual++;
        g_dirty_queue.tail = (g_dirty_queue.tail + 1) % BSPLINE_DIRTY_QUEUE_SIZE;
    }

    pthread_mutex_unlock(&bspline_mutex);

    if (actual == 0) return;

    // ---- 2. 尖角分割 + 逐子批处理 ----
    int sub_start = 0;
    for (int i = 1; i < actual - 1; i++) {
        if (is_sharp_corner(batch_ctrl[i - 1], batch_ctrl[i], batch_ctrl[i + 1],
                            g_bspline_config.sharp_angle_rad)) {
            // 尖角点: 将 [sub_start, i] (含) 作为一个子批处理
            int sub_count = i - sub_start + 1;
            if (sub_count > 0) {
                bspline_process_batch_from(sub_start, sub_count);
            }
            sub_start = i; // 新子批从尖角点开始
        }
    }
    // 处理尾部剩余
    {
        int sub_count = actual - sub_start;
        if (sub_count > 0) {
            bspline_process_batch_from(sub_start, sub_count);
        }
    }
}

// ================== 子批处理入口 ==================

// @Context: Non-RealTime Background Thread (bspline_thread_func 内)
// 对 batch_ctrl[start..start+count-1] 执行 B 样条拟合 + 重离散化 + 入队。
static void bspline_process_batch_from(int start, int count)
{
    if (count < 1) return;

    // ---- 少于 4 个点: 直接透传 ----
    if (count < BSPLINE_ORDER) {
        for (int i = 0; i < count; i++) {
            int idx = start + i;
            double speed = batch_speeds[idx];
            if (speed < 1e-6) speed = 1e-6;
            api_push_trajectory(batch_ctrl[idx], speed, DEFAULT_ACC, DEFAULT_DEC);
        }
        return;
    }

    // ---- 构建 clamped 节点向量 ----
    int n = count; // 控制点数
    build_clamped_knot_vector(n, batch_knots);

    // ---- 计算样条弧长 + 构建 LUT ----
    double total_arc = compute_spline_arc_length(
        n, BSPLINE_DEGREE, batch_knots,
        &batch_ctrl[start], arc_lut, &arc_lut_count);

    if (total_arc < 1e-6) {
        // 退化: 所有点几乎重合, 只推最后一个点
        api_push_trajectory(batch_ctrl[start + count - 1],
                           batch_speeds[start + count - 1] > 1e-6
                               ? batch_speeds[start + count - 1] : 1e-6,
                           DEFAULT_ACC, DEFAULT_DEC);
        return;
    }

    // ---- 重离散化 ----
    int num_seg = (int)ceil(total_arc / g_bspline_config.step_size_mm);
    if (num_seg < 1) num_seg = 1;

    // 计算整批的总 G93 时间预算
    double total_g93_time = 0.0;
    double avg_speed = 0.0;
    for (int i = 0; i < count; i++) {
        total_g93_time += batch_g93[start + i];
        avg_speed += batch_speeds[start + i];
    }
    avg_speed /= (double)count;

    double prev_pos[AXIS_NUM];
    memcpy(prev_pos, batch_ctrl[start], sizeof(double) * AXIS_NUM);

    for (int i = 1; i <= num_seg; i++) {
        double target_arc = (double)i / (double)num_seg * total_arc;
        double t = arc_length_to_t(target_arc, arc_lut, arc_lut_count);

        double curr_pos[AXIS_NUM];
        // 末段强制对齐终点
        if (i == num_seg) {
            memcpy(curr_pos, batch_ctrl[start + count - 1], sizeof(double) * AXIS_NUM);
        } else {
            de_boor_evaluate(n, BSPLINE_DEGREE, batch_knots,
                            &batch_ctrl[start], t, curr_pos);
        }

        // NaN 防护
        int nan_detected = 0;
        for (int j = 0; j < AXIS_NUM; j++) {
            if (isnan(curr_pos[j]) || isinf(curr_pos[j])) {
                nan_detected = 1;
                break;
            }
        }
        if (nan_detected) {
            memcpy(curr_pos, prev_pos, sizeof(double) * AXIS_NUM);
            continue;
        }

        // 计算本段物理距离
        double seg_dist = compute_equivalent_dist(prev_pos, curr_pos);
        if (seg_dist < 1e-9) continue;

        // 速度分配
        double seg_speed;
        if (total_g93_time > 1e-9) {
            // G93 模式: 保持总时间预算
            seg_speed = seg_dist / total_g93_time * (double)num_seg;
        } else {
            // G94 模式: 使用批次平均速度
            seg_speed = avg_speed;
        }
        if (seg_speed < 1e-6) seg_speed = 1e-6;

        if (api_push_trajectory(curr_pos, seg_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) {
            printf("[BSpline] 入队失败 (报警)，中止本批次\n");
            return;
        }

        memcpy(prev_pos, curr_pos, sizeof(double) * AXIS_NUM);
    }

    printf("[BSpline] 批次完成: %d 点 -> %d 段 (弧长 %.2f mm)\n",
           count, num_seg, total_arc);
}

// ================== 3 阶 B 样条数学引擎 ==================

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学计算
// 构建 clamped 节点向量: 首尾各重复 (degree+1) 次，内部均匀分布。
// n: 控制点数, knots[]: 输出节点向量 (长度 n + degree + 1)
//
// 对于 3 阶 B 样条 (degree=3, order=4):
//   首尾各 clamped 4 次，强制曲线穿过首尾控制点，
//   且切线方向与首尾段折线一致。
static void build_clamped_knot_vector(int n, double *knots)
{
    int p = BSPLINE_DEGREE; // 3
    int m = n + p + 1;      // 节点向量总长

    // 首 p+1 个节点 = 0.0 (clamped start)
    for (int i = 0; i <= p; i++) {
        knots[i] = 0.0;
    }
    // 末 p+1 个节点 = 1.0 (clamped end)
    for (int i = m - p - 1; i < m; i++) {
        knots[i] = 1.0;
    }
    // 内部节点: 均匀分布 (n - p - 1 个内部节点)
    int interior = n - p - 1; // n - 4
    for (int i = 1; i <= interior; i++) {
        knots[p + i] = (double)i / (double)(interior + 1);
    }
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学计算
// De Boor 递推算法: 在参数 t 处求值 B 样条曲线。
// n: 控制点数
// p: 阶数 (degree, 本实现固定为 3)
// knots: 节点向量 (长度 n+p+1)
// ctrl: 控制点数组 [n][AXIS_NUM] (注意: ctrl 指向 batch_ctrl[start])
// t: 参数 ∈ [0, 1]
// out: 输出位置 [AXIS_NUM]
static void de_boor_evaluate(int n, int p, const double *knots,
                              double ctrl[][AXIS_NUM], double t, double out[AXIS_NUM])
{
    // 边界钳制: t=0 返回首点, t=1 返回末点
    if (t <= 0.0) { memcpy(out, ctrl[0], sizeof(double) * AXIS_NUM); return; }
    if (t >= 1.0) { memcpy(out, ctrl[n - 1], sizeof(double) * AXIS_NUM); return; }

    // 找到 t 所在的节点区间: knots[k] <= t < knots[k+1]
    int k = p; // 从第一个非零节点区间开始搜索
    while (k < n && knots[k + 1] <= t) {
        k++;
    }
    if (k >= n) k = n - 1;

    // De Boor 递推:
    // d[j]^{r} = (1 - alpha) * d[j-1]^{r-1} + alpha * d[j]^{r-1}
    // 初始: d[j]^0 = ctrl[k - p + j], j = 0..p
    double d[4][AXIS_NUM]; // p+1 = 4
    for (int j = 0; j <= p; j++) {
        memcpy(d[j], ctrl[k - p + j], sizeof(double) * AXIS_NUM);
    }

    for (int r = 1; r <= p; r++) {
        for (int j = p; j >= r; j--) {
            int idx = k - p + j;
            double t_left  = knots[idx];
            double t_right = knots[idx + p + 1 - r];
            double alpha = 0.0;
            double denom = t_right - t_left;
            if (denom > 1e-15) {
                alpha = (t - t_left) / denom;
            }
            if (alpha < 0.0) alpha = 0.0;
            if (alpha > 1.0) alpha = 1.0;

            for (int ax = 0; ax < AXIS_NUM; ax++) {
                d[j][ax] = (1.0 - alpha) * d[j - 1][ax] + alpha * d[j][ax];
            }
        }
    }

    memcpy(out, d[p], sizeof(double) * AXIS_NUM);
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学计算
// 通过密集采样计算样条曲线的总弧长，同时构建弧长反查 LUT。
// 返回总弧长 (mm)。lut[] 和 lut_count 由调用者提供。
static double compute_spline_arc_length(int n, int p, const double *knots,
                                         double ctrl[][AXIS_NUM],
                                         ArcLutEntry_t *lut, int *lut_count)
{
    int samples = BSPLINE_ARC_LUT_SAMPLES;
    double total = 0.0;

    // LUT 第一个条目: t=0, arc=0
    lut[0].t       = 0.0;
    lut[0].cum_arc = 0.0;
    *lut_count = 1;

    double prev[AXIS_NUM];
    de_boor_evaluate(n, p, knots, ctrl, 0.0, prev);

    for (int i = 1; i <= samples; i++) {
        double t = (double)i / (double)samples;
        double curr[AXIS_NUM];
        de_boor_evaluate(n, p, knots, ctrl, t, curr);

        double seg = compute_equivalent_dist(prev, curr);
        total += seg;

        lut[*lut_count].t       = t;
        lut[*lut_count].cum_arc = total;
        (*lut_count)++;

        memcpy(prev, curr, sizeof(double) * AXIS_NUM);
    }

    return total;
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学计算
// 给定目标弧长，在 LUT 中通过二分查找反查参数 t。
static double arc_length_to_t(double target_arc, const ArcLutEntry_t *lut, int lut_count)
{
    if (target_arc <= 0.0) return 0.0;
    if (lut_count < 2) return 0.0;
    if (target_arc >= lut[lut_count - 1].cum_arc) return 1.0;

    // 二分查找: 找到 lut[lo].cum_arc <= target_arc < lut[hi].cum_arc
    int lo = 0, hi = lut_count - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (lut[mid].cum_arc <= target_arc) {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    double arc_lo = lut[lo].cum_arc;
    double arc_hi = lut[hi].cum_arc;
    double denom  = arc_hi - arc_lo;
    if (denom < 1e-15) return lut[lo].t;

    double ratio = (target_arc - arc_lo) / denom;
    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    return lut[lo].t + ratio * (lut[hi].t - lut[lo].t);
}

// ================== 尖角检测 ==================

// @Context: Non-RealTime Background Thread
// @Safe: 纯几何计算
// 判断三个连续点构成的拐角是否为尖角 (偏转角 > threshold_rad)。
// prev, corner, next: 三个连续点的机械绝对坐标。
// 返回 1=尖角, 0=平滑过渡。
static int is_sharp_corner(double prev[AXIS_NUM], double corner[AXIS_NUM],
                             double next[AXIS_NUM], double threshold_rad)
{
    double d1[AXIS_NUM], d2[AXIS_NUM];
    double len1_sq = 0.0, len2_sq = 0.0;

    for (int i = 0; i < AXIS_NUM; i++) {
        d1[i] = corner[i] - prev[i];
        d2[i] = next[i] - corner[i];
        // 旋转轴弧长折算
        if (g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0) {
            d1[i] *= DEG_TO_RAD * g_axis[i].equivalent_radius;
            d2[i] *= DEG_TO_RAD * g_axis[i].equivalent_radius;
        }
        len1_sq += d1[i] * d1[i];
        len2_sq += d2[i] * d2[i];
    }

    double len1 = sqrt(len1_sq);
    double len2 = sqrt(len2_sq);
    if (len1 < 1e-9 || len2 < 1e-9) return 0;

    double cos_angle = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        cos_angle += (d1[i] / len1) * (d2[i] / len2);
    }
    if (cos_angle > 1.0)  cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;

    // 方向向量夹角即为偏转角
    double angle = acos(cos_angle);
    return (angle > threshold_rad) ? 1 : 0;
}

// ================== 工具函数 ==================

// @Context: Non-RealTime Background Thread
// @Safe: 纯几何计算
// 计算两点间的等效空间距离 (mm)，旋转轴弧长折算。
static double compute_equivalent_dist(const double a[AXIS_NUM], const double b[AXIS_NUM])
{
    double dist_sq = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        double d = b[i] - a[i];
        if (g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0) {
            d = d * DEG_TO_RAD * g_axis[i].equivalent_radius;
        }
        dist_sq += d * d;
    }
    return sqrt(dist_sq);
}
