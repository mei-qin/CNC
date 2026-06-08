#include "planner.h"
#include "gcode_parser.h"

// ====================================================================
// 7 段式 S 曲线 (Jerk Control) — 绝对解析式预计算
// @Context: Non-RealTime Background Thread (caller holds planner_mutex)
// 所有内部物理量单位：mm/ms, mm/ms^2, mm/ms^3 → 时间自然为 ms
// ====================================================================

// 默认 Jerk：5000 mm/s^3 → mm/ms^3
#define DEFAULT_JERK_MS3  (5000.0 / 1.0e9)
// 二分查找迭代次数
#define SCURVE_BISECT_ITERS  20
// 前瞻精确求解迭代次数
#define LOOKAHEAD_BISECT_ITERS  24
// 前瞻深度上限：防止单次规划段数过多阻塞入队
#define LOOKAHEAD_MAX_DEPTH  200
// 拐角圆角平滑引擎参数
#define FILLET_SUB_SEGMENTS  3
#define FILLET_SAFETY_RATIO  0.4

// --------------------------------------------------------------------
// 加速段 v_s → v_m 的时序与距离
// 输出: tj = jerk段时长(T1=T3), ta = 匀加速时长(T2), *S_out = 距离
// --------------------------------------------------------------------
static void compute_acc_profile(double v_s, double v_m, double a_max, double jerk,
                                double *tj, double *ta, double *S_out)
{
    double dv = v_m - v_s;
    if (dv <= 1e-12 || jerk <= 1e-15 || a_max <= 1e-15) {
        *tj = 0.0; *ta = 0.0; *S_out = 0.0; return;
    }
    double tj_max = a_max / jerk;
    double dv_j   = 0.5 * jerk * tj_max * tj_max;

    if (dv < 2.0 * dv_j) {
        // 退化：加速度峰值 < a_max
        *tj = sqrt(dv / jerk);
        *ta = 0.0;
        *S_out = v_s * 2.0 * (*tj) + jerk * (*tj) * (*tj) * (*tj);
    } else {
        *tj = tj_max;
        *ta = (dv - 2.0 * dv_j) / a_max;
        double t = *tj, a = *ta;
        *S_out = v_s * (2.0*t + a) + a_max*t*t + 1.5*a_max*t*a + 0.5*a_max*a*a;
    }
}

// --------------------------------------------------------------------
// 减速段 v_m → v_e 的时序与距离
// 输出: tj = jerk段时长(T5=T7), td = 匀减速时长(T6), *S_out = 距离
// --------------------------------------------------------------------
static void compute_dec_profile(double v_m, double v_e, double d_max, double jerk,
                                double *tj, double *td, double *S_out)
{
    double dv = v_m - v_e;
    if (dv <= 1e-12 || jerk <= 1e-15 || d_max <= 1e-15) {
        *tj = 0.0; *td = 0.0; *S_out = 0.0; return;
    }
    double tj_max = d_max / jerk;
    double dv_j   = 0.5 * jerk * tj_max * tj_max;

    if (dv < 2.0 * dv_j) {
        *tj = sqrt(dv / jerk);
        *td = 0.0;
        *S_out = v_m * 2.0 * (*tj) - jerk * (*tj) * (*tj) * (*tj);
    } else {
        *tj = tj_max;
        *td = (dv - 2.0 * dv_j) / d_max;
        double t = *tj, d = *td;
        *S_out = v_m * (2.0*t + d) - d_max*t*t - 1.5*d_max*t*d - 0.5*d_max*d*d;
    }
}

// --------------------------------------------------------------------
// 精确求解：给定 v_end 和可用距离 S，求最大 v_start
//   使得 S 曲线减速 (v_start → v_end) 恰好 fits in S
// @Context: Non-RealTime Background Thread
// --------------------------------------------------------------------
static double solve_max_vstart_scurve(double v_end, double S,
                                       double d_max, double jerk, double v_ceil)
{
    double lo = v_end, hi = fmax(v_end, v_ceil);
    for (int i = 0; i < LOOKAHEAD_BISECT_ITERS; i++) {
        double mid = 0.5 * (lo + hi);
        double tj, td, S_test;
        compute_dec_profile(mid, v_end, d_max, jerk, &tj, &td, &S_test);
        if (S_test > S) hi = mid; else lo = mid;
    }
    return lo;
}

// --------------------------------------------------------------------
// 精确求解：给定 v_start 和可用距离 S，求最大 v_end
//   使得 S 曲线加速 (v_start → v_end) 恰好 fits in S
// @Context: Non-RealTime Background Thread
// --------------------------------------------------------------------
static double solve_max_vend_scurve(double v_start, double S,
                                     double a_max, double jerk, double v_ceil)
{
    double lo = v_start, hi = fmax(v_start, v_ceil);
    for (int i = 0; i < LOOKAHEAD_BISECT_ITERS; i++) {
        double mid = 0.5 * (lo + hi);
        double tj, ta, S_test;
        compute_acc_profile(v_start, mid, a_max, jerk, &tj, &ta, &S_test);
        if (S_test > S) hi = mid; else lo = mid;
    }
    return lo;
}

// --------------------------------------------------------------------
// 获取段的等效 jerk（段 jerk > 0 则取段值，否则取默认）
// --------------------------------------------------------------------
static inline double seg_effective_jerk(const TrajectorySegment_t *seg)
{
    return (seg->jerk > 1e-15) ? seg->jerk : DEFAULT_JERK_MS3;
}

// --------------------------------------------------------------------
// 精确 S 曲线制动距离 (v → 0)
// @Context: Non-RealTime Background Thread
// --------------------------------------------------------------------
static double compute_stop_distance(double v, double d_max, double jerk)
{
    if (v <= 1e-12 || d_max <= 1e-15 || jerk <= 1e-15) return 0.0;
    double tj, td, S;
    compute_dec_profile(v, 0.0, d_max, jerk, &tj, &td, &S);
    return S;
}

// --------------------------------------------------------------------
// 重算单段的 7 段式 S 曲线预计算（含内部距离熔断）
// @Context: Non-RealTime Background Thread (caller holds planner_mutex)
// 注意：此函数不执行反向传播，仅保证本段 T1~T7 与 v_start/v_end 自洽
// --------------------------------------------------------------------
static void recompute_scurve_profile(TrajectorySegment_t *seg)
{
    if (seg->cmd_type == CMD_TYPE_MCODE || seg->total_distance <= 1e-6) {
        seg->T1=0; seg->T2=0; seg->T3=0; seg->T4=0;
        seg->T5=0; seg->T6=0; seg->T7=0; seg->T_total=0;
        seg->v0=0; seg->v1=0; seg->v2=0; seg->v3=0;
        seg->v4=0; seg->v5=0; seg->v6=0;
        seg->s0=0; seg->s1=0; seg->s2=0; seg->s3=0;
        seg->s4=0; seg->s5=0; seg->s6=0;
        seg->j1=0; seg->a2=0; seg->j3=0;
        seg->j5=0; seg->a6=0; seg->j7=0;
        seg->T_total = 1.0;
        seg->total_distance = 0.0;
        return;
    }

    double v_s   = seg->v_start;
    double v_e   = seg->v_end;
    double v_m   = seg->v_target;
    double a_max = fmax(seg->acc, 1e-9);
    double d_max = fmax(seg->dec, 1e-9);
    double S     = seg->total_distance;
    double J     = (seg->jerk > 1e-15) ? seg->jerk : DEFAULT_JERK_MS3;

    if (v_m < v_s) v_m = v_s;
    if (v_m < v_e) v_m = v_e;
    seg->v_target = v_m;

    double tj_a, ta, S_acc;
    double tj_d, td, S_dec;

    compute_acc_profile(v_s, v_m, a_max, J, &tj_a, &ta, &S_acc);
    compute_dec_profile(v_m, v_e, d_max, J, &tj_d, &td, &S_dec);

    double t_cru;

    if (S_acc + S_dec <= S + 1e-9) {
        t_cru = (v_m > 1e-12) ? (S - S_acc - S_dec) / v_m : 0.0;
    } else {
        double lo = fmax(v_s, v_e);
        double hi = v_m;

        /* 内部距离熔断：若最低可行速度仍超距，钳制到 0 */
        {
            double sa_test, sd_test, d1, d2;
            compute_acc_profile(v_s, lo, a_max, J, &d1, &d2, &sa_test);
            compute_dec_profile(lo, v_e, d_max, J, &d1, &d2, &sd_test);
            if (sa_test + sd_test > S) {
                v_s = 0.0; v_e = 0.0; lo = 0.0;
                seg->v_start = 0.0; seg->v_end = 0.0;
            }
        }

        for (int iter = 0; iter < SCURVE_BISECT_ITERS; iter++) {
            double mid = 0.5 * (lo + hi);
            double sa, sd, d1, d2;
            compute_acc_profile(v_s, mid, a_max, J, &d1, &d2, &sa);
            compute_dec_profile(mid, v_e, d_max, J, &d1, &d2, &sd);
            if (sa + sd > S) hi = mid; else lo = mid;
        }

        v_m = lo;
        seg->v_target = v_m;
        compute_acc_profile(v_s, v_m, a_max, J, &tj_a, &ta, &S_acc);
        compute_dec_profile(v_m, v_e, d_max, J, &tj_d, &td, &S_dec);
        t_cru = 0.0;
    }

    if (t_cru < 0.0) t_cru = 0.0;

    double dt1 = tj_a;
    double dt2 = ta;
    double dt3 = tj_a;
    double dt4 = t_cru;
    double dt5 = tj_d;
    double dt6 = td;
    double dt7 = tj_d;

    seg->T1 = dt1;
    seg->T2 = seg->T1 + dt2;
    seg->T3 = seg->T2 + dt3;
    seg->T4 = seg->T3 + dt4;
    seg->T5 = seg->T4 + dt5;
    seg->T6 = seg->T5 + dt6;
    seg->T7 = seg->T6 + dt7;
    seg->T_total = seg->T7;

    double jrk_a = (dt1 > 1e-12) ? J : 0.0;
    double jrk_d = (dt5 > 1e-12) ? J : 0.0;
    double a_peak = jrk_a * dt1;
    double d_peak = jrk_d * dt5;

    seg->j1 =  jrk_a;
    seg->a2 =  a_peak;
    seg->j3 = -jrk_a;
    seg->j5 = -jrk_d;
    seg->a6 = -d_peak;
    seg->j7 =  jrk_d;

    seg->v0 = v_s;
    seg->v1 = v_s + 0.5 * jrk_a * dt1 * dt1;
    seg->v2 = seg->v1 + a_peak * dt2;
    seg->v3 = v_m;
    seg->v4 = v_m;
    seg->v5 = v_m - 0.5 * jrk_d * dt5 * dt5;
    seg->v6 = seg->v5 - d_peak * dt6;

    seg->s0 = 0.0;
    seg->s1 = seg->s0 + seg->v0*dt1 + seg->j1*dt1*dt1*dt1/6.0;
    seg->s2 = seg->s1 + seg->v1*dt2 + 0.5*seg->a2*dt2*dt2;
    seg->s3 = seg->s2 + seg->v2*dt3 + 0.5*seg->a2*dt3*dt3 + seg->j3*dt3*dt3*dt3/6.0;
    seg->s4 = seg->s3 + seg->v3*dt4;
    seg->s5 = seg->s4 + seg->v4*dt5 + seg->j5*dt5*dt5*dt5/6.0;
    seg->s6 = seg->s5 + seg->v5*dt6 + 0.5*seg->a6*dt6*dt6;

    /* NaN / Inf 熔断（总时长上限 30s） */
    if (isnan(seg->T_total) || isinf(seg->T_total)
        || seg->T_total <= 0.0 || seg->T_total > 30000.0) {
        seg->T1=0; seg->T2=0; seg->T3=0; seg->T4=0;
        seg->T5=0; seg->T6=0; seg->T7=0;
        seg->T_total = 1.0;
        seg->v0=0; seg->v1=0; seg->v2=0; seg->v3=0;
        seg->v4=0; seg->v5=0; seg->v6=0;
        seg->s0=0; seg->s1=0; seg->s2=0; seg->s3=0;
        seg->s4=0; seg->s5=0; seg->s6=0;
        seg->j1=0; seg->a2=0; seg->j3=0;
        seg->j5=0; seg->a6=0; seg->j7=0;
        seg->total_distance = 0.0;
        seg->v_target = 0.0;
    }
}

// ====================================================================
// G64 向心加速度过弯模型
// @Context: Non-RealTime Background Thread (caller holds planner_mutex)
// ====================================================================
double calculate_junction_speed(TrajectorySegment_t *prev, TrajectorySegment_t *curr)
{
    if (prev->cmd_type == CMD_TYPE_MCODE || curr->cmd_type == CMD_TYPE_MCODE) {
        return 0.0;
    }

    double cos_theta = 0.0;
    for (int i = 0; i < AXIS_NUM; i++) {
        cos_theta += prev->dir_vec[i] * curr->dir_vec[i];
    }
    if (cos_theta > 1.0)  cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;

    if (cos_theta >= 0.999) return fmin(prev->v_max, curr->v_max);
    if (cos_theta <= -0.999) return 0.0;

    double alpha = acos(cos_theta);
    double denom = 1.0 - cos(alpha * 0.5);
    if (denom <= 1e-6) denom = 1e-6;

    double v_allow = sqrt(g_planner_config.max_centripetal_acc
                        * (g_planner_config.corner_tolerance / denom));
    if (isnan(v_allow) || isinf(v_allow) || v_allow < 0.0) v_allow = 0.0;

    return fmin(v_allow, fmin(prev->v_max, curr->v_max));
}

// ====================================================================
// Corner Rounding 预处理引擎 (Fillet Arc Insertion)
// @Context: Non-RealTime Background Thread (caller holds planner_mutex)
// @Math: 给定连续两段直线 L1(方向D1)和 L2(方向D2)，夹角 theta = acos(D1·D2)
//   半外角 alpha = (PI-theta)/2
//   容差 delta = R*(1/sin_alpha - 1) → R = delta*sin_alpha/(1-sin_alpha)
//   切点距 d = R*cot_alpha，限制 d <= 0.4*min(L1,L2)
//   圆弧用 SLERP 离散为 FILLET_SUB_SEGMENTS 个微小直线段插入队列
// ====================================================================
static int planner_fillet_preprocess(int plan_tail, int old_head)
{
    double delta = g_planner_config.corner_tolerance;
    if (delta <= 1e-6) return old_head; // 容差禁用，跳过

    int head = old_head;
    int count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count < 2) return head;

    // 反向扫描：从队尾向队首逐对处理，避免插入后索引漂移
    int i = (plan_tail + count - 1) % QUEUE_SIZE;
    while (i != plan_tail) {
        int prev = (i - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
        TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[i];

        // ---- 门控：跳过已释放 / M代码 / 零距离 / 已平滑段 ----
        if (atomic_load_explicit(&s_prev->is_ready, memory_order_acquire) == 1
         || atomic_load_explicit(&s_curr->is_ready, memory_order_acquire) == 1
         || s_prev->cmd_type != CMD_TYPE_MOTION
         || s_curr->cmd_type != CMD_TYPE_MOTION
         || s_prev->is_fillet == 1
         || s_curr->is_fillet == 1
         || s_prev->total_distance <= 1e-6
         || s_curr->total_distance <= 1e-6) {
            i = prev; continue;
        }

        // ---- 1. 夹角计算 ----
        double cos_theta = 0.0;
        for (int j = 0; j < AXIS_NUM; j++)
            cos_theta += s_prev->dir_vec[j] * s_curr->dir_vec[j];
        if (cos_theta >  1.0) cos_theta =  1.0;
        if (cos_theta < -1.0) cos_theta = -1.0;
        if (cos_theta >= 0.999 || cos_theta <= -0.999) { i = prev; continue; }

        double theta   = acos(cos_theta);
        double half_ex = (M_PI - theta) * 0.5;   // 半外角
        double sin_ha  = sin(half_ex);
        double cos_ha  = cos(half_ex);
        if (sin_ha <= 1e-6) { i = prev; continue; }

        // ---- 2. 圆弧半径 R 与切点距 d ----
        double R = delta * sin_ha / (1.0 - sin_ha);
        double d = R * cos_ha / sin_ha;
        double d_limit = FILLET_SAFETY_RATIO * fmin(s_prev->total_distance,
                                                     s_curr->total_distance);
        if (d > d_limit) { d = d_limit; R = d * sin_ha / cos_ha; }
        if (d < 1e-6 || R < 1e-6) { i = prev; continue; }

        // ---- 3. 队列空间检查 ----
        int queue_used = (head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
        if (QUEUE_SIZE - 1 - queue_used < FILLET_SUB_SEGMENTS) break;

        // ---- 4. 几何计算：顶点 / 切点 / 圆心 ----
        //   V = s_prev 终点 (拐角顶点)
        //   T1 = V - d*D1   T2 = V + d*D2
        //   bisector = (-D1+D2)/|...|   C = V + (R/sin_ha)*bisector
        double V[AXIS_NUM], T1[AXIS_NUM], T2[AXIS_NUM], C[AXIS_NUM];
        double bisec[AXIS_NUM];
        double bisec_sq = 0.0;
        for (int j = 0; j < AXIS_NUM; j++) {
            V[j] = s_prev->target_pos[j];
            bisec[j] = -s_prev->dir_vec[j] + s_curr->dir_vec[j];
            bisec_sq += bisec[j] * bisec[j];
        }
        double bisec_len = sqrt(bisec_sq);
        if (bisec_len < 1e-12) { i = prev; continue; }

        double bisec_d = R / sin_ha;
        for (int j = 0; j < AXIS_NUM; j++) {
            bisec[j] /= bisec_len;
            T1[j] = V[j] - d * s_prev->dir_vec[j];
            T2[j] = V[j] + d * s_curr->dir_vec[j];
            C[j] = V[j] + bisec_d * bisec[j];
        }

        // ---- 5. SLERP 几何预校验（在队列操作之前，失败可直接 continue）----
        double r1[AXIS_NUM], r2[AXIS_NUM];
        double r1sq = 0.0, r2sq = 0.0;
        for (int j = 0; j < AXIS_NUM; j++) {
            r1[j] = T1[j] - C[j]; r1sq += r1[j]*r1[j];
            r2[j] = T2[j] - C[j]; r2sq += r2[j]*r2[j];
        }
        double r1m = sqrt(r1sq), r2m = sqrt(r2sq);
        if (r1m < 1e-12 || r2m < 1e-12) { i = prev; continue; }

        double dot_rr = 0.0;
        for (int j = 0; j < AXIS_NUM; j++)
            dot_rr += (r1[j]/r1m) * (r2[j]/r2m);
        if (dot_rr >  1.0) dot_rr =  1.0;
        if (dot_rr < -1.0) dot_rr = -1.0;
        double arc_angle = acos(dot_rr);
        double sin_arc   = sin(arc_angle);
        if (sin_arc < 1e-12) sin_arc = 1e-12;

        // ---- 6. 环形队列段平移（从 head-1 倒序搬至 i，前移 K 位）----
        {
            int src = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            int stop = (prev + QUEUE_SIZE) % QUEUE_SIZE;
            while (src != stop) {
                int dst = (src + FILLET_SUB_SEGMENTS) % QUEUE_SIZE;
                g_cmd_queue.buffer[dst] = g_cmd_queue.buffer[src];
                src = (src - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            }
        }
        head = (head + FILLET_SUB_SEGMENTS) % QUEUE_SIZE;

        // ---- 7. 缩短段 prev：终点退到切点 T1 ----
        s_prev->total_distance -= d;
        for (int j = 0; j < AXIS_NUM; j++)
            s_prev->target_pos[j] = T1[j];

        // ---- 8. 生成圆弧子段 (SLERP 离散) ----
        //   写入位置 i..i+K-1 (平移留下的空位)，s_curr 在 i+K
        {
            TrajectorySegment_t tmpl = *s_curr;
            tmpl.cmd_type  = CMD_TYPE_MOTION;
            tmpl.is_fillet = 1;  // 标记为圆弧子段，防止重复平滑
            tmpl.m_code    = 0;
            atomic_store_explicit(&tmpl.is_ready, 0, memory_order_relaxed);
            tmpl.v_start = 0.0; tmpl.v_end = 0.0;
            tmpl.T1=0; tmpl.T2=0; tmpl.T3=0; tmpl.T4=0;
            tmpl.T5=0; tmpl.T6=0; tmpl.T7=0; tmpl.T_total=0;

            double prev_pt[AXIS_NUM];
            memcpy(prev_pt, T1, sizeof(double)*AXIS_NUM);

            for (int k = 0; k < FILLET_SUB_SEGMENTS; k++) {
                int si = (i + k) % QUEUE_SIZE;
                TrajectorySegment_t *fs = &g_cmd_queue.buffer[si];
                *fs = tmpl;

                double pt[AXIS_NUM];
                if (k == FILLET_SUB_SEGMENTS - 1) {
                    memcpy(pt, T2, sizeof(double)*AXIS_NUM);
                } else {
                    double t  = (double)(k + 1) / (double)FILLET_SUB_SEGMENTS;
                    double s1 = sin((1.0-t)*arc_angle) / sin_arc;
                    double s2 = sin(t*arc_angle) / sin_arc;
                    for (int j = 0; j < AXIS_NUM; j++)
                        pt[j] = C[j] + s1*r1[j] + s2*r2[j];
                }

                double seg_d = 0.0;
                for (int j = 0; j < AXIS_NUM; j++) {
                    fs->target_pos[j] = pt[j];
                    double dx = pt[j] - prev_pt[j];
                    seg_d += dx*dx;
                }
                seg_d = sqrt(seg_d);
                fs->total_distance = seg_d;
                if (seg_d > 1e-9) {
                    for (int j = 0; j < AXIS_NUM; j++)
                        fs->dir_vec[j] = (pt[j]-prev_pt[j]) / seg_d;
                }
                memcpy(prev_pt, pt, sizeof(double)*AXIS_NUM);
            }
        }

        // ---- 9. 缩短段 curr（已平移到 i+K）：起点退到切点 T2 ----
        {
            int ni = (i + FILLET_SUB_SEGMENTS) % QUEUE_SIZE;
            g_cmd_queue.buffer[ni].total_distance -= d;
        }

        i = prev; // 继续向前扫描
    }

    g_cmd_queue.head = head;
    return head;
}

// ====================================================================
// Deep Look-Ahead 前瞻核心引擎
// @Context: Non-RealTime Background Thread (parser 或 看门狗)
// @Thread-Safety: 由 planner_mutex 保护，禁止 RT 线程调用！
//
// 核心重构要点：
//   1. 废弃 plan_count==1/2/3 硬编码分支，统一反向+正向扫描
//   2. LOOKAHEAD_MAX_DEPTH 上限防阻塞，超出时分批处理
//   3. 动态安全释放窗口：基于真实 S 曲线制动距离判定
//   4. M 代码屏障：停稳释放，不受制动距离限制
//   5. force_flush：无视安全窗口，强制清空队列
// ====================================================================
void planner_recalculate(int force_flush)
{
    pthread_mutex_lock(&planner_mutex);

    // ---- 0. 队列快照 ----
    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;
    int count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) { pthread_mutex_unlock(&planner_mutex); return; }

    // ---- 1. 找到 plan_tail：第一个未释放段 ----
    int plan_tail = tail;
    while (plan_tail != head &&
           atomic_load_explicit(&g_cmd_queue.buffer[plan_tail].is_ready,
                                memory_order_acquire) == 1) {
        plan_tail = (plan_tail + 1) % QUEUE_SIZE;
    }
    int plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (plan_count == 0) { pthread_mutex_unlock(&planner_mutex); return; }

    // ---- 1.5 拐角圆角平滑预处理 ----
    head = planner_fillet_preprocess(plan_tail, head);
    count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;

    // ---- 2. 分批上限：截断有效规划窗口 ----
    int eff_head = head;
    if (plan_count > LOOKAHEAD_MAX_DEPTH) {
        eff_head = (plan_tail + LOOKAHEAD_MAX_DEPTH) % QUEUE_SIZE;
        plan_count = LOOKAHEAD_MAX_DEPTH;
    }
    int eff_last = (eff_head - 1 + QUEUE_SIZE) % QUEUE_SIZE;

    // ---- 3. 初始化未释放段速度为 0 ----
    {
        int c = plan_tail;
        while (c != eff_head) {
            if (atomic_load_explicit(&g_cmd_queue.buffer[c].is_ready,
                                     memory_order_acquire) == 0) {
                g_cmd_queue.buffer[c].v_start = 0.0;
                g_cmd_queue.buffer[c].v_end   = 0.0;
            }
            c = (c + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 4. 统一速度规划：反向扫描 + 正向扫描
    //    废弃硬编码 plan_count==1/2/3 分支，所有段数统一处理
    // ================================================================

    // 4a. 末端段必须刹停
    g_cmd_queue.buffer[eff_last].v_end = 0.0;

    // 4b. 反向扫描：从 eff_last 向 plan_tail 传播减速约束
    if (plan_count >= 2) {
        int curr = eff_last;
        while (curr != plan_tail) {
            int prev = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev];
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];

            double v_junc = calculate_junction_speed(s_prev, s_curr);
            double d_max  = fmax(s_curr->dec, 1e-9);
            double J_c    = seg_effective_jerk(s_curr);

            double max_v_start = solve_max_vstart_scurve(
                s_curr->v_end, s_curr->total_distance,
                d_max, J_c, s_curr->v_target);

            s_curr->v_start = fmin(v_junc, fmin(max_v_start, s_curr->v_target));
            s_prev->v_end = s_curr->v_start;
            curr = prev;
        }
    }

    // 4c. plan_tail 与已释放段的衔接约束
    if (plan_tail != tail) {
        int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        TrajectorySegment_t *sp = &g_cmd_queue.buffer[plan_tail];
        double v_junc = calculate_junction_speed(
            &g_cmd_queue.buffer[prev_idx], sp);
        double d_max = fmax(sp->dec, 1e-9);
        double J_p   = seg_effective_jerk(sp);

        double max_vs = solve_max_vstart_scurve(
            sp->v_end, sp->total_distance,
            d_max, J_p, sp->v_target);

        sp->v_start = fmin(v_junc, fmin(max_vs, sp->v_target));
    }

    // 4d. 正向扫描：从 plan_tail 向 eff_last 传播加速约束
    if (plan_count >= 2) {
        int curr = plan_tail;
        while (curr != eff_last) {
            TrajectorySegment_t *s_curr = &g_cmd_queue.buffer[curr];
            int next = (curr + 1) % QUEUE_SIZE;

            double a_max = fmax(s_curr->acc, 1e-9);
            double J_c   = seg_effective_jerk(s_curr);

            double max_v_end = solve_max_vend_scurve(
                s_curr->v_start, s_curr->total_distance,
                a_max, J_c, s_curr->v_target);

            s_curr->v_end = fmin(s_curr->v_end, max_v_end);
            s_curr->v_end = fmin(s_curr->v_end, s_curr->v_target);
            g_cmd_queue.buffer[next].v_start = s_curr->v_end;
            curr = next;
        }
    }

    // ================================================================
    // 5. 动态安全释放窗口 — 精确 S 曲线制动距离判定
    //    从 eff_last 向 plan_tail 累加距离，找到满足刹停要求
    //    的最远安全边界。M 代码屏障不受距离限制直接释放。
    // ================================================================
    int safe_release_head;
    {
        // 默认不释放任何段
        safe_release_head = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;

        if (force_flush || !g_parser_ctrl.is_running) {
            // 强制清空或解析器已停：全部释放
            safe_release_head = eff_last;
        } else {
            double accum = 0.0;
            int scan = eff_last;
            int seg_count = 0;

            while (scan != (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
                TrajectorySegment_t *seg = &g_cmd_queue.buffer[scan];

                // M 代码屏障：M 代码之前的段可安全释放
                if (seg->cmd_type == CMD_TYPE_MCODE) {
                    safe_release_head = scan;
                    break;
                }

                accum += seg->total_distance;
                seg_count++;

                double J = seg_effective_jerk(seg);
                double d_max = fmax(seg->dec, 1e-9);
                double S_stop = compute_stop_distance(seg->v_target, d_max, J);

                if (accum >= S_stop || seg_count >= LOOKAHEAD_MAX_DEPTH) {
                    safe_release_head = scan;
                    break;
                }

                scan = (scan - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            }
        }
    }

    // ================================================================
    // 6. 7段式 S 曲线预计算 + 距离熔断反向传播
    //    仅对安全窗口内的段执行，复用 recompute_scurve_profile
    // ================================================================
    {
        int curr = plan_tail;
        int safe_end = (safe_release_head + 1) % QUEUE_SIZE;

        while (curr != safe_end) {
            TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];

            // M 代码 / 零距离：直接归零
            if (seg->cmd_type == CMD_TYPE_MCODE || seg->total_distance <= 1e-6) {
                recompute_scurve_profile(seg);
                curr = (curr + 1) % QUEUE_SIZE;
                continue;
            }

            // 距离熔断预检：若最低速度仍超距，强制归零并反向传播
            {
                double lo = fmax(seg->v_start, seg->v_end);
                if (lo > 1e-12) {
                    double J   = seg_effective_jerk(seg);
                    double a_m = fmax(seg->acc, 1e-9);
                    double d_m = fmax(seg->dec, 1e-9);
                    double sa, sd, d1, d2;
                    compute_acc_profile(seg->v_start, lo, a_m, J, &d1, &d2, &sa);
                    compute_dec_profile(lo, seg->v_end, d_m, J, &d1, &d2, &sd);
                    if (sa + sd > seg->total_distance) {
                        seg->v_start = 0.0;
                        seg->v_end   = 0.0;
                        // 反向传播至前一未释放段
                        if (curr != plan_tail) {
                            int prev_idx = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
                            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev_idx];
                            if (atomic_load_explicit(&s_prev->is_ready,
                                                     memory_order_acquire) == 0) {
                                s_prev->v_end = 0.0;
                                recompute_scurve_profile(s_prev);
                            }
                        }
                    }
                }
            }

            recompute_scurve_profile(seg);
            curr = (curr + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 7. 原子释放安全窗口内的段
    // ================================================================
    {
        int mark_curr = plan_tail;
        int release_end = (safe_release_head + 1) % QUEUE_SIZE;
        while (mark_curr != release_end) {
            atomic_store_explicit(&g_cmd_queue.buffer[mark_curr].is_ready,
                                  1, memory_order_release);
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    }

    pthread_mutex_unlock(&planner_mutex);
}
