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
static CoordSystem_t batch_wcs[BSPLINE_MAX_CTRL_POINTS];               // 与脏点 wcs 一一对应, push 时透传
static double batch_offsets[BSPLINE_MAX_CTRL_POINTS][AXIS_NUM];        // 与脏点 wcs_offset_snap 一一对应 (H-1)

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
//
// 前置尖角拦截 (Pre-emptive Sharp Corner Blocking):
//   入队前用队尾最后两点 (P_n-2, P_n-1) 与当前 pos 构成三元组,调 is_sharp_corner。
//   若判定为尖角,说明 pos 属于另一条折线 —— 立即同步 BSpline_Flush() 把当前队列
//   (截至 P_n-1 的平滑段) 全部排空下发,然后 pos 进入空队列成为新批次起点。
//   这样尖角后的点不再被混入当前拟合批次,且降低批处理延迟。
//   注意: BSpline_Flush 自带 bspline_mutex 内部加锁,故此处必须先 unlock 再调 Flush,
//   否则会自死锁。
int BSpline_PushDirtyPoint(double pos[AXIS_NUM], double speed, double g93_time)
{
    if (atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) == 1) {
        return -1;
    }

    // ---- 前置尖角拦截: 检查 (P_n-2, P_n-1, pos) ----
    pthread_mutex_lock(&bspline_mutex);
    int dirty_count_pre = (g_dirty_queue.head - g_dirty_queue.tail
                           + BSPLINE_DIRTY_QUEUE_SIZE) % BSPLINE_DIRTY_QUEUE_SIZE;
    int corner_detected = 0;
    if (dirty_count_pre >= 2) {
        int idx_n1 = (g_dirty_queue.head - 1 + BSPLINE_DIRTY_QUEUE_SIZE)
                     % BSPLINE_DIRTY_QUEUE_SIZE;
        int idx_n2 = (g_dirty_queue.head - 2 + BSPLINE_DIRTY_QUEUE_SIZE)
                     % BSPLINE_DIRTY_QUEUE_SIZE;
        DirtyPoint_t *P_n1 = &g_dirty_queue.buffer[idx_n1];
        DirtyPoint_t *P_n2 = &g_dirty_queue.buffer[idx_n2];
        corner_detected = is_sharp_corner(P_n2->pos, P_n1->pos, pos,
                                          g_bspline_config.sharp_angle_rad);
    }
    pthread_mutex_unlock(&bspline_mutex);

    if (corner_detected) {
        // 同步排空当前平滑段队列 (P_n-1 是其终点)。
        // Flush 返回后队列必为空 (5 秒超时保护),后续 push 的 pos 即为新批次起点。
        BSpline_Flush();
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
    dp->wcs      = g_state.modal_wcs;  // parser 线程捕获,避免 bspline 线程跨线程读 g_state
    // H-1 + P2' + P5': 冻结入队瞬间的偏置向量 (work_offsets 或 ext + G52 local_offset)。
    // 与 axis_ctrl.c snapshot_wcs_offset 同语义 (各编译单元独立 static, 避免跨单元耦合)。
    // 优先级: G54.1 Pn ext > G54-G59 work_offsets > G53 (零基准)
    // G52 叠加仅 G54-G59 / G54.1 Pn 路径; G53 走 else 分支输出零, 不叠加。
    {
        int wcs_idx = (g_state.modal_wcs >= COORD_G54 && g_state.modal_wcs <= COORD_G59)
                      ? (g_state.modal_wcs - 1) : -1;
        if(wcs_idx >= 0){
            if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                int ext_idx = g_state.modal_ext_wcs_p - 1;
                for(int i = 0; i < AXIS_NUM; i++)
                    dp->wcs_offset_snap[i] = g_coord_mgr.work_offsets_ext[ext_idx][i];
            } else {
                for(int i = 0; i < AXIS_NUM; i++)
                    dp->wcs_offset_snap[i] = g_coord_mgr.work_offsets[wcs_idx][i];
            }
            if(g_state.local_offset_active){
                for(int i = 0; i < AXIS_NUM; i++)
                    dp->wcs_offset_snap[i] += g_state.local_offset[i];
            }
        }else{
            for(int i = 0; i < AXIS_NUM; i++) dp->wcs_offset_snap[i] = 0.0;
        }
    }

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
        batch_wcs[actual]    = dp->wcs;
        memcpy(batch_offsets[actual], dp->wcs_offset_snap, sizeof(double) * AXIS_NUM);
        actual++;
        g_dirty_queue.tail = (g_dirty_queue.tail + 1) % BSPLINE_DIRTY_QUEUE_SIZE;
    }

    pthread_mutex_unlock(&bspline_mutex);

    if (actual == 0) return;

    // ---- 2. 尖角分割 + 逐子批处理 (C4 2026-07-27 加固) ----
    // 旧版: 锐角点 i 作为子批边界共享点, 被 B-Spline 拟合平滑掉, 实际几何变圆角.
    // C4: 锐角点单独走 api_push_trajectory_passthrough_wcs 透传 (is_fillet=1 防 planner
    //      二次抹圆), 子批范围 [sub_start, i-1] 不含锐角点, B-Spline 仅拟合非锐角段.
    //      几何精度: 锐角严格保留; B-Spline 拟合质量: 仅平滑段, 无锐角点干扰.
    int sub_start = 0;
    for (int i = 1; i < actual - 1; i++) {
        if (is_sharp_corner(batch_ctrl[i - 1], batch_ctrl[i], batch_ctrl[i + 1],
                            g_bspline_config.sharp_angle_rad)) {
            // (a) 子批 [sub_start, i-1] 做 B-Spline (不含锐角点)
            int sub_count = i - sub_start;
            if (sub_count > 0) {
                bspline_process_batch_from(sub_start, sub_count);
            }
            // (b) 锐角点 i 单独透传, 保留锐角几何
            //     passthrough_wcs 内部设 is_fillet=1, planner 反/正扫描加屏障,
            //     与 B-Spline 段衔接处速度自动归零过渡, 不撕裂.
            {
                double v = batch_speeds[i] > 1e-6 ? batch_speeds[i] : 1e-6;
                api_push_trajectory_passthrough_wcs(batch_ctrl[i], v,
                                                     DEFAULT_ACC, DEFAULT_DEC,
                                                     batch_wcs[i], batch_offsets[i]);
            }
            sub_start = i + 1;  // 新批从锐角点之后开始 (不再共享锐角点)
        }
    }
    // (c) 处理尾部剩余 [sub_start, actual-1] 做 B-Spline
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

    // ---- 少于 4 个点: 直接透传,标记 is_fillet=1 防止底层二次抹圆 ----
    // 这些点是锐角切批后的残余,需保留其锐角几何,planner_fillet_preprocess 必须跳过。
    if (count < BSPLINE_ORDER) {
        for (int i = 0; i < count; i++) {
            int idx = start + i;
            double speed = batch_speeds[idx];
            if (speed < 1e-6) speed = 1e-6;
            api_push_trajectory_passthrough_wcs(batch_ctrl[idx], speed,
                                                 DEFAULT_ACC, DEFAULT_DEC,
                                                 batch_wcs[idx], batch_offsets[idx]);
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
        // 用 passthrough_wcs 透传 (bspline 线程禁止读 g_state.modal_wcs)
        double v = batch_speeds[start + count - 1] > 1e-6
                       ? batch_speeds[start + count - 1] : 1e-6;
        api_push_trajectory_passthrough_wcs(batch_ctrl[start + count - 1],
                                             v, DEFAULT_ACC, DEFAULT_DEC,
                                             batch_wcs[start + count - 1],
                                             batch_offsets[start + count - 1]);
        return;
    }

    // ---- 绝对比例等距采样 (Absolute Equidistant Resampling) ----
    // 废除 t += dt 试探法 (overshoot discard 引起微米级步长方差)。
    // 改为对弧长做严格 N 等分,每个采样点的累计弧长 = i/N × total_arc。
    // 相邻输出点之间的弧长必然精确等于 total_arc/num_seg,
    // 仅受 arc_length_to_t 的 LUT 插值精度约束 (O(1/SAMPLES²) 量级)。
    //
    // 计算整批的总 G93 时间预算 + 平均速度
    double total_g93_time = 0.0;
    double avg_speed = 0.0;
    for (int i = 0; i < count; i++) {
        total_g93_time += batch_g93[start + i];
        avg_speed += batch_speeds[start + i];
    }
    avg_speed /= (double)count;

    // G93 模式下,整批匀速 = total_arc / total_g93_time (恒定)
    double g93_avg_speed = (total_g93_time > 1e-9)
                            ? total_arc / total_g93_time : 0.0;

    // ---- 等分数计算 + 严格段弧长 ----
    double step = g_bspline_config.step_size_mm;
    if (step < 1e-6) step = 1e-6;  // 防除零
    // 固定步长采样 (Fixed-Step Resampling): seg_arc = step 对大多数段,
    // 末段吸收余数 (total_arc - full_steps*step)。
    // 替代原 ceil() 方案: ceil 对小弧长子批 (尖角切分残余) 会产生 num_seg=1 + 极小 seg_arc,
    // 跨批次 CV 高达 0.58。改 floor 后,绝大多数段严格等于 step,跨批次 CV 显著降低。
    int full_steps = (int)floor(total_arc / step);
    if (full_steps < 0) full_steps = 0;
    int num_seg = full_steps + 1;  // 末段总是存在 (吸收余数或等于 step)
    double last_seg_arc = total_arc - (double)full_steps * step;

    int seg_out = 0;
    for (int i = 1; i <= num_seg; i++) {
        double target_arc;
        double current_seg_arc;
        if (i < num_seg) {
            // 整数倍步长段: target_arc = i * step,seg_arc = step
            target_arc = (double)i * step;
            current_seg_arc = step;
        } else {
            // 末段: 吸收余数,对齐 total_arc (消除浮点累积误差)
            target_arc = total_arc;
            current_seg_arc = last_seg_arc;
        }
        int is_end_point = (i == num_seg);

        // LUT 反查: target_arc -> t
        double t = arc_length_to_t(target_arc, arc_lut, arc_lut_count);
        double curr_pos[AXIS_NUM];
        de_boor_evaluate(n, BSPLINE_DEGREE, batch_knots,
                        &batch_ctrl[start], t, curr_pos);

        // NaN/Inf 防护: 退化点跳过 (极罕见,LUT 边界处可能)
        int nan_detected = 0;
        for (int j = 0; j < AXIS_NUM; j++) {
            if (isnan(curr_pos[j]) || isinf(curr_pos[j])) {
                nan_detected = 1; break;
            }
        }
        if (nan_detected) continue;

        // 终点对齐: 最后一段强制 curr_pos = 最后控制点,消除 LUT 插值残留
        if (is_end_point) {
            memcpy(curr_pos, batch_ctrl[start + count - 1],
                   sizeof(double) * AXIS_NUM);
        }

        // @Context: Non-RealTime Background Thread (bspline_thread_func)
        // @Stage: STAGE_BSPLINE —— 3 阶 B 样条等距重采样输出点
        // curr_pos 已通过 NaN 防护 + 终点对齐,是最终的等距重采样形态。
        // v_target 取本批次恒定速度 (G93=整批匀速, G94=avg_speed),
        // 与下方 seg_speed 同源, /1000.0 转换为 mm/ms 与 RT 探针对齐。
        {
            double bspline_v = (total_g93_time > 1e-9) ? g93_avg_speed : avg_speed;
            if (bspline_v < 1e-6) bspline_v = 1e-6;
            TraceLogger_PushPipeline(STAGE_BSPLINE, curr_pos, bspline_v / 1000.0);
        }

        // 入队 (整段步长 = step,末段 = last_seg_arc,速度恒定)
        // 全部用 _wcs 变体: bspline 线程不得读 parser 的 g_state.modal_wcs 与 work_offsets。
        // WCS/offset 取 batch_wcs[start] / batch_offsets[start]:
        //   parser 切 WCS 或写 #5221 时已 BSpline_Flush,同批次 WCS+偏置恒定。
        double seg_speed;
        int push_ret;
        if (total_g93_time > 1e-9) {
            // G93: 段速度 = 整批匀速,段时长按本段弧长比例分配
            // (current_seg_arc: 整段=step,末段=last_seg_arc)
            seg_speed = g93_avg_speed;
            if (seg_speed < 1e-6) seg_speed = 1e-6;
            double dt_this = current_seg_arc / total_arc * total_g93_time;
            push_ret = api_push_trajectory_g93_wcs(curr_pos, seg_speed,
                                                    DEFAULT_ACC, DEFAULT_DEC,
                                                    dt_this, batch_wcs[start],
                                                    batch_offsets[start]);
        } else {
            // G94 模式: B-样条已输出严格等距平滑段,必须透传(is_fillet=1),
            // 否则 Planner 的 G64 拐角抹圆会对 B-样条段做二次篡改,破坏等距性。
            seg_speed = avg_speed;
            if (seg_speed < 1e-6) seg_speed = 1e-6;
            push_ret = api_push_trajectory_passthrough_wcs(curr_pos, seg_speed,
                                                             DEFAULT_ACC, DEFAULT_DEC,
                                                             batch_wcs[start],
                                                             batch_offsets[start]);
        }
        if (push_ret < 0) {
            printf("[BSpline] 入队失败 (报警)，中止本批次\n");
            return;
        }
        seg_out++;
    }

    printf("[BSpline] 批次完成: %d 点 -> %d 段 (弧长 %.2f mm, 步长 %.4f mm, 末段 %.4f mm)\n",
           count, seg_out, total_arc, step, last_seg_arc);
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
//
// 数学等价优化: 抛弃 acos (在 cos_angle → ±1 处数值不稳定)。
// 由于 cos 在 [0, π] 单调递减:
//   angle > threshold_rad  <=>  cos(angle) < cos(threshold_rad)
// 因此直接比较 cos_angle < cos_thresh 即可,一次乘法比 acos 更快更稳。
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
    // 动态信噪比 (SNR) 屏蔽: 以平滑步长的 15% 作为噪声底噪阈值。
    // 仅当【两个向量同时】短于底噪时才视为 CAM 量化噪声 (低通吸收)。
    // 若只有一个短 (如 flush 残余批次的边缘短向量),仍尝试检测尖角 ——
    // 因为真实尖角通常至少有一侧是正常长度的向量。
    // 早期 OR 逻辑会漏判 45° 拐角 (一侧短向量触发整角屏蔽),改 AND 修复。
    double noise_floor = g_bspline_config.step_size_mm * 0.15;
    if (len1 < noise_floor && len2 < noise_floor) return 0;

    double cos_angle = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        cos_angle += (d1[i] / len1) * (d2[i] / len2);
    }
    if (cos_angle > 1.0)  cos_angle = 1.0;
    if (cos_angle < -1.0) cos_angle = -1.0;

    // 暴力截断: cos_thresh 预计算,直接比较,绕过 acos
    double cos_thresh = cos(threshold_rad);
    return (cos_angle < cos_thresh) ? 1 : 0;
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
