#include "planner.h"
#include "gcode_parser.h"

// ====================================================================
// 7 段式 S 曲线 (Jerk Control) — 绝对解析式预计算
// @Context: Non-RealTime Background Thread (caller holds queue_spinlock)
// 所有内部物理量单位：mm/ms, mm/ms^2, mm/ms^3 → 时间自然为 ms
// ====================================================================

// 默认 Jerk：5000 mm/s^3 → mm/ms^3
#define DEFAULT_JERK_MS3  (5000.0 / 1.0e9)
// 二分查找迭代次数
#define SCURVE_BISECT_ITERS  20
// 前瞻精确求解迭代次数
#define LOOKAHEAD_BISECT_ITERS  24
// 前瞻硬上限 (防御性): 真实有效窗口由动态刹车距离决定 (见 step 2),
// 此值仅在段长全部极小且 v_target 极大的极端情况下兜底,避免无限扫描。
// 取 QUEUE_SIZE 一半,留出 push 端空槽缓冲
#define LOOKAHEAD_HARD_CAP  (QUEUE_SIZE / 2)
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
//
// 退化防御 (Audit CONDITIONAL PASS → PASS):
//   当 v_end == v_ceil 时,lo = hi = v_end,二分区间坍塌为单点,
//   24 次迭代 compute_dec_profile(v_end, v_end, ...) 始终 dv=0 → S_test=0,
//   距离 S 完全不参与决策,函数盲目返回 v_end。
//   触发场景: 反向扫描中前段 v_end 耦合自后段 v_start,
//             若后段共线 + 距离充足 → 后段 v_start = v_target → 前段 v_end = v_target,
//             当 v_target == v_ceil 时即退化。
//   当前依赖 Step 6 距离熔断兜底;此防御改为初级预防,不再依赖次级纠正。
// --------------------------------------------------------------------
static double solve_max_vstart_scurve(double v_end, double S,
                                       double d_max, double jerk, double v_ceil)
{
    double lo = v_end, hi = fmax(v_end, v_ceil);

    // 退化防御: lo==hi 时二分无意义,直接物理检验 v_end 能否在 S 内完成减速。
    // 注意: 此处 == 比较安全——lo 和 hi 都来自同一 v_end 字面赋值,
    //       无浮点运算误差累积。
    if (lo == hi) {
        double tj, td, S_test;
        compute_dec_profile(lo, v_end, d_max, jerk, &tj, &td, &S_test);
        // S_test ≤ S: 恒速通过,无需减速距离,v_start = v_end 合法
        // S_test > S: 物理不可能 (但此分支 dv=0 必然 S_test=0,理论上不会触发)
        return (S_test <= S) ? lo : 0.0;
    }

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
//
// 退化防御: 同 solve_max_vstart_scurve,处理 v_start == v_ceil 的二分坍塌。
// --------------------------------------------------------------------
static double solve_max_vend_scurve(double v_start, double S,
                                     double a_max, double jerk, double v_ceil)
{
    double lo = v_start, hi = fmax(v_start, v_ceil);

    // 退化防御: lo==hi 时直接物理检验
    if (lo == hi) {
        double tj, ta, S_test;
        compute_acc_profile(v_start, lo, a_max, jerk, &tj, &ta, &S_test);
        return (S_test <= S) ? lo : v_start;
    }

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
// @Context: Non-RealTime Background Thread (caller holds queue_spinlock)
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

    // G93 强一致性段: api_push_trajectory_impl 已预计算纯匀速参数
    // (T4=T_total, v0..v6=v_target, s4..s6=total_distance)。
    // 反向/正向扫描可能改写了 v_start/v_end,这里强制恢复,确保 RT 线程
    // 读到的是恒速绝对解析参数,绝对遵守 G93 时间预算。
    if (seg->is_g93_strict) {
        double v_const = seg->v_target;
        seg->v_start = v_const;
        seg->v_end   = v_const;
        seg->v_max   = v_const;
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

        /* Hazard 1 修复: 距离熔断不再粗暴 v_s=v_e=0 (会导致密集微段 stop-go 卡顿)。
         * 改为二分搜索 scale ∈ [0,1], 保持 v_s/v_e 比例不变, 找到最大可使
         * sa(v_s*scale → lo*scale) + sd(lo*scale → v_e*scale) ≤ S 的比例。
         * 段从而能以非零连续速度通过, 避免 0.01mm 球头刀 CAM 路径的哒哒卡顿。
         * 注意: 仅当本段 v_s 或 v_e 非零时才需要 scale; 二者本就为 0 时 sa=sd=0, 不会进入此分支。
         */
        {
            double sa_test, sd_test, d1, d2;
            compute_acc_profile(v_s, lo, a_max, J, &d1, &d2, &sa_test);
            compute_dec_profile(lo, v_e, d_max, J, &d1, &d2, &sd_test);
            if (sa_test + sd_test > S) {
                double sc_lo = 0.0, sc_hi = 1.0;
                for (int it = 0; it < SCURVE_BISECT_ITERS; it++) {
                    double sc_mid = 0.5 * (sc_lo + sc_hi);
                    double vs_t = v_s * sc_mid, ve_t = v_e * sc_mid;
                    double lo_t = fmax(vs_t, ve_t);
                    double sa_t, sd_t, d3, d4;
                    compute_acc_profile(vs_t, lo_t, a_max, J, &d3, &d4, &sa_t);
                    compute_dec_profile(lo_t, ve_t, d_max, J, &d3, &d4, &sd_t);
                    if (sa_t + sd_t > S) sc_hi = sc_mid; else sc_lo = sc_mid;
                }
                v_s *= sc_lo;
                v_e *= sc_lo;
                lo = fmax(v_s, v_e);
                seg->v_start = v_s;
                seg->v_end   = v_e;
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
// G64 + Jerk-Limited 双限制过弯模型
// @Context: Non-RealTime Background Thread (caller holds queue_spinlock)
//
// 物理模型 (Beudaert-Lavernhe-Lartigue 双 S 曲线入口/出口):
//   拐角处等效为半径 R_eff 的圆弧, 速度矢量在入口/出口经历从直线 (κ=0)
//   到圆弧 (κ=1/R_eff) 的过渡。
//
//   等效半径: R_eff = δ / (1 - cos(θ/2))   其中 δ 为拐角容差, θ 为夹角
//
//   ① 向心加速度限制 (经典 G64):
//        v_acc = sqrt(A_max · R_eff)
//      推导: a_n = v²/R_eff ≤ A_max → v ≤ sqrt(A_max · R_eff)
//
//   ② Jerk 限制 (新增, 解决高速微段冲击):
//      入口处 a_n 从 0 渐变到 v²/R_eff, 假设 jerk = J_max (双 S 线性渐变),
//      渐变时间 t = 2v²/(R_eff · J_max), 通过距离 ≈ v·t = 2v³/(R_eff · J_max)。
//      要求该距离 ≤ δ, 解得:
//        v_jerk = (J_max · R_eff · δ / 2)^(1/3)
//      量纲检验: [mm/s³ · mm · mm]^(1/3) = [mm³/s³]^(1/3) = mm/s ✓
//
//   最终: v_junc = min(v_acc, v_jerk, prev->v_max, curr->v_max)
//
// 工程意义:
//   - 低速/小拐角场景: v_acc < v_jerk, 由向心加速度主导 (与旧版一致)
//   - 高速/大拐角场景: v_jerk < v_acc, 由 jerk 主导, 防止驱动器电流冲击
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

    // 共线/反相特判: 跳过数学计算
    if (cos_theta >= 0.999) return fmin(prev->v_max, curr->v_max);
    if (cos_theta <= -0.999) return 0.0;

    double alpha = acos(cos_theta);
    double denom = 1.0 - cos(alpha * 0.5);
    if (denom <= 1e-6) denom = 1e-6;

    double delta = g_planner_config.corner_tolerance;
    double A_max = g_planner_config.max_centripetal_acc;

    // 等效圆弧半径
    double R_eff = delta / denom;

    // ① 向心加速度限制
    double v_acc_limit = sqrt(A_max * R_eff);

    // ② Jerk 限制: 取两段中更严格的 jerk (短板效应)
    //   seg_effective_jerk 已在 api_push_trajectory_impl 经过短板限幅,
    //   反映了"通过此段时各轴允许的最大 jerk"。
    //   拐角过渡的 jerk 由两段共享, 取 min 保证不超任一段的物理极限。
    double J_eff = fmin(seg_effective_jerk(prev), seg_effective_jerk(curr));
    double v_jerk_limit = cbrt(0.5 * J_eff * R_eff * delta);

    // 取最严格限制 (NaN/Inf 防御)
    double v_allow = fmin(v_acc_limit, v_jerk_limit);
    if (isnan(v_allow) || isinf(v_allow) || v_allow < 0.0) v_allow = 0.0;

    return fmin(v_allow, fmin(prev->v_max, curr->v_max));
}

// ====================================================================
// Corner Rounding 预处理引擎 (Fillet Arc Insertion)
// @Context: Non-RealTime Background Thread (caller holds queue_spinlock)
// @Math: 给定连续两段直线 L1(方向D1)和 L2(方向D2)，夹角 theta = acos(D1·D2)
//   半外角 alpha = (PI-theta)/2
//   容差 delta = R*(1/sin_alpha - 1) → R = delta*sin_alpha/(1-sin_alpha)
//   切点距 d = R*cot_alpha，限制 d <= 0.4*min(L1,L2)
//   圆弧用 SLERP 离散为 FILLET_SUB_SEGMENTS 个微小直线段插入队列
// @Thread-Safety: 调用方持 queue_spinlock → fillet 的内存平移与生产者 push
//                 互斥,绝对安全。write_head 推进用 relaxed 即可,
//                 可见性由调用方的 spinlock release 统一建立。
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
         || s_prev->is_exact_stop == 1     // P2-A-4: 精准停段跳过圆角化
         || s_curr->is_exact_stop == 1
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
        // 持锁期间 RT 消费者可能继续推进 read_tail (RT 不取锁),
        // 用 relaxed 读 read_tail 即可——可见性由 spinlock acquire 保证
        // (本线程刚获取 spinlock,看到的是上一任持锁者修改后的最新状态)。
        // RT 1ms 周期的推进最多让本检查保守 1ms,不会引发数据覆写。
        int cur_tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_relaxed);
        int queue_used = (head - cur_tail + QUEUE_SIZE) % QUEUE_SIZE;
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

    // relaxed 推进 write_head: fillet 已完成子段插入 + 缩短 prev/curr 操作,
    // 可见性由 planner_recalculate 出口的 spinlock release 统一建立。
    // (fillet 的所有写入都在持锁期间,不需要 release 写)
    atomic_store_explicit(&g_cmd_queue.write_head, head, memory_order_relaxed);
    return head;
}

// ====================================================================
// Deep Look-Ahead 前瞻核心引擎
// @Context: Non-RealTime Background Thread (parser / bspline / 看门狗)
// @Thread-Safety: queue_spinlock Try-Lock 避让模式:
//                 - 入口 try-lock,失败立即返回 (生产者绝不阻塞)
//                 - 持锁期间: 生产者/planner/watchdog 全部互斥,
//                   fillet 内存平移绝对安全
//                 - 单次规划: 持锁期间 write_head 不变,无需 while(1) 兜底
//                 - RT 线程不取此锁,优先级反转免疫
//
// 核心重构要点：
//   1. 废弃 plan_count==1/2/3 硬编码分支，统一反向+正向扫描
//   2. 动态 Look-ahead Depth: 基于 worst-case 刹车距离扩展 eff_head,
//      LOOKAHEAD_HARD_CAP 仅作硬上限防御 (替代旧版写死的 200 段短视)
//   3. 动态安全释放窗口：基于真实 S 曲线制动距离判定
//   4. M 代码屏障：停稳释放，不受制动距离限制
//   5. force_flush：无视安全窗口，强制清空队列
//   6. Try-Lock 避让 + 单次规划: 替代 pthread_mutex 阻塞语义,
//      RT 线程 1ms 周期永不被阻塞。
//   7. api_flush_planner 通过 planner_recalculate_locked 实现 Spin-Wait,
//      消除文件末尾"最后一点饥饿"。
// ====================================================================

// @Context: callee MUST hold queue_spinlock (either via try-lock or spin-wait).
//           Does NOT acquire or release the lock — that is the caller's responsibility.
// @Thread-Safety: All buffer access is serialized by the caller's lock.
void planner_recalculate_locked(int force_flush)
{

    // ---- 0. 队列快照 (持锁期间 relaxed 读即可,可见性由 spinlock 保证) ----
    int tail = atomic_load_explicit(&g_cmd_queue.read_tail,  memory_order_relaxed);
    int head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_relaxed);
    int count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (count == 0) goto release_and_exit;

    // ---- 1. 找到 plan_tail：第一个未释放段 ----
    int plan_tail = tail;
    while (plan_tail != head &&
           atomic_load_explicit(&g_cmd_queue.buffer[plan_tail].is_ready,
                                memory_order_acquire) == 1) {
        plan_tail = (plan_tail + 1) % QUEUE_SIZE;
    }
    int plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
    if (plan_count == 0) goto release_and_exit;

    // ---- 1.5 拐角圆角平滑预处理 (持锁期间内存平移绝对安全) ----
    head = planner_fillet_preprocess(plan_tail, head);
    count = (head - tail + QUEUE_SIZE) % QUEUE_SIZE;
    plan_count = (head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;

    // ---- 2. 动态扩展有效规划窗口 (基于物理刹车距离) ----
    // 旧版: 写死 LOOKAHEAD_MAX_DEPTH=200,在 F10000 + 0.01mm 微段场景下
    //       (刹车距离可达 20mm → 需要 ~2000 段),规划器严重短视,
    //       末端 v_end=0 约束无法反向传播到 plan_tail, 导致超速冲过拐角。
    // 新版: 用所有未释放段中的 (max_v_target, min_dec, min_jerk) 计算
    //       worst-case 刹车距离,然后从 plan_tail 向前累加段距离直到 ≥ S_stop。
    //
    // 不变量:
    //   ① eff_last 段以 v_target 进入时, eff_head 前累积距离足够其刹停
    //   ② 硬上限 LOOKAHEAD_HARD_CAP 防御段长全部极小的病态输入
    int eff_head = head;
    {
        // (a) 第一遍扫描: 找未释放段中的 worst-case 减速参数
        double max_v_target = 0.0;
        double min_dec      = 1e9;
        double min_jerk     = 1e9;
        int cap_n           = 0;
        int c               = plan_tail;
        while (c != head && cap_n < LOOKAHEAD_HARD_CAP) {
            TrajectorySegment_t *seg = &g_cmd_queue.buffer[c];
            if (atomic_load_explicit(&seg->is_ready, memory_order_acquire) == 0) {
                if (seg->cmd_type == CMD_TYPE_MOTION && seg->v_target > max_v_target) {
                    max_v_target = seg->v_target;
                }
                if (seg->dec > 1e-9 && seg->dec < min_dec)   min_dec  = seg->dec;
                if (seg->jerk > 1e-15 && seg->jerk < min_jerk) min_jerk = seg->jerk;
            }
            c = (c + 1) % QUEUE_SIZE;
            cap_n++;
        }

        // (b) 计算 worst-case 刹车距离
        //   S_stop = compute_stop_distance(max_v_target, min_dec, min_jerk)
        //   若所有段都是 M 代码或零距离, max_v_target=0 → S_stop=0 → eff_head=head (全部规划)
        double S_worst = 0.0;
        if (max_v_target > 1e-12) {
            double J_worst = (min_jerk < 1e9) ? min_jerk : DEFAULT_JERK_MS3;
            double d_worst = (min_dec  < 1e9) ? min_dec  : (100.0 / 1e6); // 兜底 100 mm/s²
            S_worst = compute_stop_distance(max_v_target, d_worst, J_worst);
        }

        // (c) 第二遍扫描: 从 plan_tail 累加距离,直到 ≥ S_worst 或耗尽段数/硬上限
        if (S_worst > 1e-9) {
            double accum = 0.0;
            int n = 0;
            int c2 = plan_tail;
            while (c2 != head && accum < S_worst && n < LOOKAHEAD_HARD_CAP) {
                accum += g_cmd_queue.buffer[c2].total_distance;
                c2 = (c2 + 1) % QUEUE_SIZE;
                n++;
            }
            eff_head = c2;
            plan_count = (eff_head - plan_tail + QUEUE_SIZE) % QUEUE_SIZE;
        }
        // else: S_worst=0, 保持 eff_head=head (规划全部未释放段)
    }
    int eff_last = (eff_head - 1 + QUEUE_SIZE) % QUEUE_SIZE;

    // ---- 3. 初始化未释放段速度为 0 + 短段物理速度钳制 ----
    // 短段物理钳制 (Short-Segment Bottleneck):
    //   极短微段 (L < 0.1mm) 没有足够距离完成"加速-减速"轮廓,
    //   若 v_target 过大,S 曲线预计算 (recompute_scurve_profile) 内部
    //   二分求解器会返回不合理的 v_target (或触发距离熔断)。
    //   物理上限推导 (v_start=v_end=0, 无匀速段):
    //     s_acc + s_dec = v²/a_max + v²/a_max = 2v²/a_max ... 不对
    //     正确: 单段加速到 v 再减到 0, 距离 = v²/a_max (前半加速 + 后半减速)
    //     → v_max_seg = sqrt(a_max · L)
    //   S 曲线额外损耗约 30% (jerk ramp 占用时间), 用 0.7 系数保守估计。
    //   G93 强一致性段豁免: 用户已强制纯匀速, 时间预算刚性。
    {
        int c = plan_tail;
        while (c != eff_head) {
            TrajectorySegment_t *seg = &g_cmd_queue.buffer[c];
            if (atomic_load_explicit(&seg->is_ready, memory_order_acquire) == 0) {
                seg->v_start = 0.0;
                seg->v_end   = 0.0;

                // 短段物理钳制 (仅对常规运动段)
                if (seg->cmd_type == CMD_TYPE_MOTION
                    && !seg->is_g93_strict
                    && seg->total_distance > 1e-9) {
                    double a_max_seg = fmax(seg->acc, 1e-9);
                    double v_phys_max = sqrt(a_max_seg * seg->total_distance) * 0.7;
                    if (seg->v_target > v_phys_max) {
                        seg->v_target = v_phys_max;
                        seg->v_max    = v_phys_max;
                    }
                }
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

            // ---- M 代码速度屏障 (显式强制) ----
            // 若本段在队列中的下一段 (curr+1) 为 M 代码,必须强制 v_end=0,
            // 严禁运动段以非零过渡速度滑入 M 代码屏障 (主轴换向/换刀前必须停稳)。
            // 现有 calculate_junction_speed 在 prev/curr 任一为 MCODE 时也返回 0
            // 已能间接保证,此处的显式预扫是对该不变量的防御性加固,
            // 避免未来重构 (例如改写 junction 公式) 时悄然破坏该约束。
            // eff_last 段已在 4a 强制 v_end=0,无需重复检查。
            if (curr != eff_last) {
                int next_idx = (curr + 1) % QUEUE_SIZE;
                if (g_cmd_queue.buffer[next_idx].cmd_type == CMD_TYPE_MCODE) {
                    s_curr->v_end = 0.0;
                }
                // P2-A-4: G09/G61 精准停屏障
                // 若下一段标记为 is_exact_stop, 强制本段 v_end=0 (与 MCODE 屏障同机制).
                // 工业语义: 精准停拐角必须完全静止才能开始下段, 防过切/清根.
                if (g_cmd_queue.buffer[next_idx].is_exact_stop) {
                    s_curr->v_end = 0.0;
                }
            }
            // P2-A-4: 本段标记为 is_exact_stop 时, 也强制 v_end=0
            // (G09/G61 段本身出口必须停稳, 不依赖下段标记)
            if (s_curr->is_exact_stop) {
                s_curr->v_end = 0.0;
            }

            // ---- G93 强一致性: v_start 锁定 v_target,不走 junction 限幅 ----
            // 保证 api_push_trajectory_impl 预计算的纯匀速参数不被破坏。
            if (s_curr->is_g93_strict) {
                s_curr->v_start = s_curr->v_target;
            } else {
                double v_junc = calculate_junction_speed(s_prev, s_curr);
                double d_max  = fmax(s_curr->dec, 1e-9);
                double J_c    = seg_effective_jerk(s_curr);

                double max_v_start = solve_max_vstart_scurve(
                    s_curr->v_end, s_curr->total_distance,
                    d_max, J_c, s_curr->v_target);

                s_curr->v_start = fmin(v_junc, fmin(max_v_start, s_curr->v_target));
            }

            // ---- G93 强一致性 prev: v_end 必须锁定 v_target,邻居据此规划减速 ----
            if (s_prev->is_g93_strict) {
                s_prev->v_end = s_prev->v_target;
            } else {
                s_prev->v_end = s_curr->v_start;
            }
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

            // ---- G93 强一致性: v_end 锁定 v_target,不走加速限幅 ----
            if (s_curr->is_g93_strict) {
                s_curr->v_end = s_curr->v_target;
                g_cmd_queue.buffer[next].v_start = s_curr->v_target;
                curr = next;
                continue;
            }

            // ---- P2-A-4: G09/G61 精准停正向屏障 ----
            // 若本段标记为 is_exact_stop, 强制 v_end=0, 下段 v_start=0 (精准停拐角).
            // 必须在 G93 strict 之后, 正常加速限幅之前 (优先级: G93 > 精准停 > G64 圆角).
            if (s_curr->is_exact_stop) {
                s_curr->v_end = 0.0;
                g_cmd_queue.buffer[next].v_start = 0.0;
                curr = next;
                continue;
            }

            double a_max = fmax(s_curr->acc, 1e-9);
            double J_c   = seg_effective_jerk(s_curr);

            double max_v_end = solve_max_vend_scurve(
                s_curr->v_start, s_curr->total_distance,
                a_max, J_c, s_curr->v_target);

            s_curr->v_end = fmin(s_curr->v_end, max_v_end);
            s_curr->v_end = fmin(s_curr->v_end, s_curr->v_target);

            // 下一段若为 G93 强一致性,其 v_start 必须锁定为 v_target,
            // 而非由本段 v_end 决定 (避免被本段的加速限幅压低)。
            if (g_cmd_queue.buffer[next].is_g93_strict) {
                g_cmd_queue.buffer[next].v_start = g_cmd_queue.buffer[next].v_target;
            } else {
                g_cmd_queue.buffer[next].v_start = s_curr->v_end;
            }
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

                if (accum >= S_stop || seg_count >= LOOKAHEAD_HARD_CAP) {
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

            // Hazard 1 修复: 距离熔断改 scale-down (与 recompute_scurve_profile 同语义),
            // 不再 v_start=v_end=0。反向传播把前一未释放段的 v_end 同步到本段 scaled v_start
            // (速度连续), 不再硬性置零。
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
                        double sc_lo = 0.0, sc_hi = 1.0;
                        for (int it = 0; it < SCURVE_BISECT_ITERS; it++) {
                            double sc_mid = 0.5 * (sc_lo + sc_hi);
                            double vs_t = seg->v_start * sc_mid, ve_t = seg->v_end * sc_mid;
                            double lo_t = fmax(vs_t, ve_t);
                            double sa_t, sd_t, d3, d4;
                            compute_acc_profile(vs_t, lo_t, a_m, J, &d3, &d4, &sa_t);
                            compute_dec_profile(lo_t, ve_t, d_m, J, &d3, &d4, &sd_t);
                            if (sa_t + sd_t > seg->total_distance) sc_hi = sc_mid; else sc_lo = sc_mid;
                        }
                        seg->v_start *= sc_lo;
                        seg->v_end   *= sc_lo;
                        // 反向传播至前一未释放段: v_end 必须等于本段 scaled v_start (速度连续)
                        if (curr != plan_tail) {
                            int prev_idx = (curr - 1 + QUEUE_SIZE) % QUEUE_SIZE;
                            TrajectorySegment_t *s_prev = &g_cmd_queue.buffer[prev_idx];
                            if (atomic_load_explicit(&s_prev->is_ready,
                                                     memory_order_acquire) == 0) {
                                s_prev->v_end = seg->v_start;
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
    // 7. 原子释放安全窗口内的段 (release 写 is_ready)
    //    RT 消费者【不取 spinlock】,通过 is_ready 的 release/acquire 配对
    //    建立与 planner 的 happens-before,看到所有 S 曲线参数。
    //    注意: release 写 is_ready 不能省略——即使 spinlock release 也会
    //    建立 happens-before,但 RT 不取锁,只能通过 is_ready 获得可见性。
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

    return;  // 锁由调用者管理

release_and_exit:
    return;  // 空队列,锁由调用者管理
}

// ====================================================================
// planner_recalculate — Try-Lock 入口 (非阻塞, 供生产者在 push 后使用)
//
// @Context: 生产者在 api_push_trajectory_impl / api_push_mcode 内部,
//           完成入队后调用此函数触发前瞻规划。
// @Thread-Safety: Try-Lock 避让,竞争失败直接返回。
//   - 生产者正在 push: 退让, 生产者 push 完成后会再次触发
//   - 其他 planner 持锁: 当前调用 redundant, 直接返回
//   - watchdog 持锁: 同上, watchdog 兜底规划已覆盖
//   注意: api_flush_planner 不走此路径, 而是直接 spin-wait +
//         planner_recalculate_locked, 保证文件末尾强制 flush。
// ====================================================================
void planner_recalculate(int force_flush)
{
    if (atomic_flag_test_and_set_explicit(&g_cmd_queue.queue_spinlock,
                                          memory_order_acquire)) {
        return;  // Try-Lock 失败: 不阻塞, 依赖调用者下次触发
    }
    planner_recalculate_locked(force_flush);
    atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);
}
