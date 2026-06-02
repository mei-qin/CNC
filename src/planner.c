#include "planner.h"

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
// Look-Ahead 前瞻核心引擎（精确 S 曲线距离求解版）
// @Context: Non-RealTime Background Thread (parser 或 看门狗)
// @Thread-Safety: 由 planner_mutex 保护，禁止 RT 线程调用！
// ====================================================================
void planner_recalculate(int force_flush)
{
    pthread_mutex_lock(&planner_mutex);

    int tail = g_cmd_queue.tail;
    int head = g_cmd_queue.head;
    int count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) { pthread_mutex_unlock(&planner_mutex); return; }

    // ---- plan_tail 结界 ----
    int plan_tail = tail;
    while (plan_tail != head &&
           atomic_load_explicit(&g_cmd_queue.buffer[plan_tail].is_ready,
                                memory_order_acquire) == 1) {
        plan_tail = (plan_tail + 1) % QUEUE_SIZE;
    }
    int plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (plan_count == 0) { pthread_mutex_unlock(&planner_mutex); return; }

    // ---- 0. 初始化未释放段 ----
    {
        int c = plan_tail;
        while (c != head) {
            if (atomic_load_explicit(&g_cmd_queue.buffer[c].is_ready,
                                     memory_order_acquire) == 0) {
                g_cmd_queue.buffer[c].v_start = 0.0;
                g_cmd_queue.buffer[c].v_end   = 0.0;
            }
            c = (c + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 前瞻速度平滑（精确 S 曲线距离求解，废弃 K 系数）
    // 直接调用 compute_dec_profile / compute_acc_profile + 二分搜索
    // 精确计算 S 曲线真实刹车/加速距离。
    // ================================================================
    if (plan_count >= 3) {
        int safe_tail = (plan_tail + 1) % QUEUE_SIZE;
        int curr = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        g_cmd_queue.buffer[curr].v_end = 0.0;

        // ---- 1. 反向扫描 ----
        while (curr != safe_tail) {
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

        // plan_tail 衔接
        if (plan_tail != tail) {
            int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            double v_junc = calculate_junction_speed(
                &g_cmd_queue.buffer[prev_idx], &g_cmd_queue.buffer[plan_tail]);
            TrajectorySegment_t *sp = &g_cmd_queue.buffer[plan_tail];
            double d_max = fmax(sp->dec, 1e-9);
            double J_p   = seg_effective_jerk(sp);

            double max_vs = solve_max_vstart_scurve(
                sp->v_end, sp->total_distance,
                d_max, J_p, sp->v_target);

            sp->v_start = fmin(v_junc, fmin(max_vs, sp->v_target));
        }

        // ---- 2. 正向扫描 ----
        curr = plan_tail;

        while (curr != (head - 1 + QUEUE_SIZE) % QUEUE_SIZE) {
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

    // ---- plan_count == 2 ----
    if (plan_count == 2) {
        int seg0 = plan_tail;
        int seg1 = (plan_tail + 1) % QUEUE_SIZE;
        TrajectorySegment_t *s0 = &g_cmd_queue.buffer[seg0];
        TrajectorySegment_t *s1 = &g_cmd_queue.buffer[seg1];

        double v_junc = calculate_junction_speed(s0, s1);
        v_junc = fmin(v_junc, fmin(s0->v_target, s1->v_target));

        double a0 = fmax(s0->acc, 1e-9);
        double J0 = seg_effective_jerk(s0);
        double max_ve0 = solve_max_vend_scurve(
            s0->v_start, s0->total_distance, a0, J0, s0->v_target);
        s0->v_end = fmin(v_junc, fmin(max_ve0, s0->v_target));

        double d1 = fmax(s1->dec, 1e-9);
        double J1 = seg_effective_jerk(s1);
        double max_vs1 = solve_max_vstart_scurve(
            s1->v_end, s1->total_distance, d1, J1, s1->v_target);
        s1->v_start = fmin(s0->v_end, fmin(max_vs1, s1->v_target));
        s0->v_end = s1->v_start;

        if (plan_tail != tail) {
            int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
            double vj = calculate_junction_speed(&g_cmd_queue.buffer[prev_idx], s0);
            double d0 = fmax(s0->dec, 1e-9);
            double J0b = seg_effective_jerk(s0);
            double max_vs0 = solve_max_vstart_scurve(
                s0->v_end, s0->total_distance, d0, J0b, s0->v_target);
            s0->v_start = fmin(vj, fmin(max_vs0, s0->v_target));
        }
    }

    // ---- plan_count == 1 ----
    if (plan_count == 1 && plan_tail != tail) {
        int prev_idx = (plan_tail - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        TrajectorySegment_t *sp = &g_cmd_queue.buffer[plan_tail];
        double v_junc = calculate_junction_speed(&g_cmd_queue.buffer[prev_idx], sp);
        double d_max = fmax(sp->dec, 1e-9);
        double J_sp  = seg_effective_jerk(sp);
        double max_vs = solve_max_vstart_scurve(
            sp->v_end, sp->total_distance, d_max, J_sp, sp->v_target);
        sp->v_start = fmin(v_junc, fmin(max_vs, sp->v_target));
    }

    // ================================================================
    // 3. 7段式 S 曲线绝对解析式预计算
    //    严禁 ceil()！所有时间为精确 double ms。
    //    planner 直接把 7 段入口状态全部算好，RT 线程只做查表插值。
    // ================================================================
    {
        double J_default = DEFAULT_JERK_MS3;
        int curr = plan_tail;

        while (curr != head) {
            TrajectorySegment_t *seg = &g_cmd_queue.buffer[curr];

            // ---- M 代码 / 零距离：直接归零 ----
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
                curr = (curr + 1) % QUEUE_SIZE;
                continue;
            }

            double v_s   = seg->v_start;
            double v_e   = seg->v_end;
            double v_m   = seg->v_target;
            double a_max = fmax(seg->acc, 1e-9);
            double d_max = fmax(seg->dec, 1e-9);
            double S     = seg->total_distance;
            double J     = (seg->jerk > 1e-15) ? seg->jerk : J_default;

            // 安全钳制
            if (v_m < v_s) v_m = v_s;
            if (v_m < v_e) v_m = v_e;
            seg->v_target = v_m;

            // ---- 步骤 1：计算加速段和减速段的 (tj, ta/td, S) ----
            double tj_a, ta, S_acc;
            double tj_d, td, S_dec;

            compute_acc_profile(v_s, v_m, a_max, J, &tj_a, &ta, &S_acc);
            compute_dec_profile(v_m, v_e, d_max, J, &tj_d, &td, &S_dec);

            double t_cru;

            // ---- 步骤 2：距离校验 ----
            if (S_acc + S_dec <= S + 1e-9) {
                t_cru = (v_m > 1e-12) ? (S - S_acc - S_dec) / v_m : 0.0;
            } else {
                // 二分查找 v_m_real
                double lo = fmax(v_s, v_e);
                double hi = v_m;

                // ---- 防爆熔断：若最低速度仍超距，强制归零并反向传播 ----
                {
                    double sa_test, sd_test, dummy1, dummy2;
                    compute_acc_profile(v_s, lo, a_max, J,
                                        &dummy1, &dummy2, &sa_test);
                    compute_dec_profile(lo, v_e, d_max, J,
                                        &dummy1, &dummy2, &sd_test);
                    if (sa_test + sd_test > S) {
                        // 前瞻给死局，触发熔断
                        v_s = 0.0; v_e = 0.0; lo = 0.0;
                        seg->v_start = 0.0; seg->v_end = 0.0;

                        // 反向传播：修改前驱段 v_end 以保持速度连续性
                        // 仅修改尚未释放的前驱段
                        if (curr != plan_tail) {
                            int prev_idx = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
                            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev_idx];
                            if (atomic_load_explicit(&s_prev->is_ready,
                                                     memory_order_acquire) == 0) {
                                s_prev->v_end = 0.0;
                            }
                        }
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

            // ---- 步骤 3：预计算 7 段入口状态 ----
            // 各段持续时间
            double dt1 = tj_a;       // T1: 加加速
            double dt2 = ta;         // T2: 匀加速
            double dt3 = tj_a;       // T3: 减加速 (=T1, 对称)
            double dt4 = t_cru;      // T4: 匀速
            double dt5 = tj_d;       // T5: 加减速
            double dt6 = td;         // T6: 匀减速
            double dt7 = tj_d;       // T7: 减减速 (=T5, 对称)

            // 累计时间节点
            seg->T1 = dt1;
            seg->T2 = seg->T1 + dt2;
            seg->T3 = seg->T2 + dt3;
            seg->T4 = seg->T3 + dt4;
            seg->T5 = seg->T4 + dt5;
            seg->T6 = seg->T5 + dt6;
            seg->T7 = seg->T6 + dt7;
            seg->T_total = seg->T7;

            // 各段控制量
            double jrk_a = (dt1 > 1e-12) ? J : 0.0;
            double jrk_d = (dt5 > 1e-12) ? J : 0.0;
            double a_peak = jrk_a * dt1;   // T1 结束时的加速度峰值
            double d_peak = jrk_d * dt5;   // T5 结束时的减速度峰值
            seg->j1 =  jrk_a;              // T1: jerk > 0
            seg->a2 =  a_peak;             // T2: 恒定加速度 = a_peak
            seg->j3 = -jrk_a;              // T3: jerk < 0（加速度下降）
            seg->j5 = -jrk_d;              // T5: jerk < 0（减速度上升）
            seg->a6 = -d_peak;             // T6: 恒定减速度 = -d_peak
            seg->j7 =  jrk_d;              // T7: jerk > 0（减速度下降）

            // 各段入口速度
            seg->v0 = v_s;
            seg->v1 = v_s + 0.5 * jrk_a * dt1 * dt1;
            seg->v2 = seg->v1 + a_peak * dt2;
            seg->v3 = v_m;                              // = seg->v2 + 0.5*jrk_a*dt3^2
            seg->v4 = v_m;                              // 匀速不变
            seg->v5 = v_m - 0.5 * jrk_d * dt5 * dt5;
            seg->v6 = seg->v5 - d_peak * dt6;
            // v7 = v_e（不存，由 v6 + 0.5*j7*dt7^2 推出）

            // 各段入口位移（解析积分）
            // T1: s = v0*t + j1*t^3/6     T2: s = v1*t + a2*t^2/2
            // T3: s = v2*t + j3*t^3/6      T4: s = v3*t (匀速)
            // T5: s = v4*t + j5*t^3/6      T6: s = v5*t + a6*t^2/2
            // T7: s = v6*t + j7*t^3/6
            seg->s0 = 0.0;
            seg->s1 = seg->s0 + seg->v0*dt1 + seg->j1*dt1*dt1*dt1/6.0;
            seg->s2 = seg->s1 + seg->v1*dt2 + 0.5*seg->a2*dt2*dt2;
            seg->s3 = seg->s2 + seg->v2*dt3 + 0.5*seg->a2*dt3*dt3 + seg->j3*dt3*dt3*dt3/6.0;
            seg->s4 = seg->s3 + seg->v3*dt4;
            seg->s5 = seg->s4 + seg->v4*dt5 + seg->j5*dt5*dt5*dt5/6.0;
            seg->s6 = seg->s5 + seg->v5*dt6 + 0.5*seg->a6*dt6*dt6;

            // NaN / Inf 熔断（总时长上限 30s）
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

            curr = (curr + 1) % QUEUE_SIZE;
        }
    }

    // ================================================================
    // 4. 释放标志
    // ================================================================
    if (force_flush || plan_count < 3) {
        int mark_curr = plan_tail;
        while (mark_curr != head) {
            atomic_store_explicit(&g_cmd_queue.buffer[mark_curr].is_ready,
                                  1, memory_order_release);
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    } else {
        int safe_release_head = (head - 2 + QUEUE_SIZE) % QUEUE_SIZE;
        if (g_cmd_queue.buffer[(head - 1 + QUEUE_SIZE) % QUEUE_SIZE].cmd_type
            == CMD_TYPE_MCODE) {
            safe_release_head = (head - 1 + QUEUE_SIZE) % QUEUE_SIZE;
        }
        int mark_curr = plan_tail;
        while (mark_curr != safe_release_head) {
            atomic_store_explicit(&g_cmd_queue.buffer[mark_curr].is_ready,
                                  1, memory_order_release);
            mark_curr = (mark_curr + 1) % QUEUE_SIZE;
        }
    }

    pthread_mutex_unlock(&planner_mutex);
}
