#include "cutter_comp.h"
#include "gcode_parser.h"
#include "global_def.h"
#include "trace_logger.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ================== 全局引擎状态 ==================
static CutterCompState_t g_comp;

// ================== 2D 计算几何基础数学库 ==================
// 以下函数均在当前激活平面的两个主轴上操作。
// 非平面轴（第三线性轴、旋转轴）不受偏置影响，线性跟随。

// @Context: Non-RealTime Background Thread
// @Safe: 纯查表，无副作用。
// 根据 G17/G18/G19 获取平面两个主轴的物理索引。
// ax1=第一轴, ax2=第二轴 (偏置在此两轴构成的 2D 平面内计算)
static void get_plane_axes(int plane, int *ax1, int *ax2)
{
    switch(plane) {
        case 18: // ZX 平面
            *ax1 = g_axis_map['Z' - 'A'];
            *ax2 = g_axis_map['X' - 'A'];
            break;
        case 19: // YZ 平面
            *ax1 = g_axis_map['Y' - 'A'];
            *ax2 = g_axis_map['Z' - 'A'];
            break;
        default: // G17 XY 平面 (默认)
            *ax1 = g_axis_map['X' - 'A'];
            *ax2 = g_axis_map['Y' - 'A'];
            break;
    }
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学。
// 计算方向向量 (dx, dy) 的左法向量 (逆时针旋转 90° 的单位向量)。
// 左法向量 = (-dy, dx) / |(dx, dy)|
static void compute_left_normal(double dx, double dy, double *nx, double *ny)
{
    double len = hypot(dx, dy);
    if(len < COMP_ZERO_LEN_TOL) {
        *nx = 0.0;
        *ny = 0.0;
        return;
    }
    *nx = -dy / len;
    *ny =  dx / len;
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学。
// 2D 叉乘: (ax, ay) x (bx, by) = ax*by - ay*bx
// 正值表示从 a 到 b 为逆时针 (CCW)，负值为顺时针 (CW)。
static double cross2d(double ax, double ay, double bx, double by)
{
    return ax * by - ay * bx;
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学。
// 两条 2D 直线的交点求解。
// Line 1: 过点 (x1,y1)，方向 (dx1,dy1)
// Line 2: 过点 (x2,y2)，方向 (dx2,dy2)
// 交点写入 (ox, oy)。
// 返回 0=成功, -1=平行/近共线 (denom < COMP_PARALLEL_TOL)
static int line_intersect(double x1, double y1, double dx1, double dy1,
                          double x2, double y2, double dx2, double dy2,
                          double *ox, double *oy, double *t_out, double *s_out)
{
    double denom = dx1 * dy2 - dy1 * dx2;
    if(fabs(denom) < COMP_PARALLEL_TOL) return -1;

    double t = ((x2 - x1) * dy2 - (y2 - y1) * dx2) / denom;
    double s = ((x2 - x1) * dy1 - (y2 - y1) * dx1) / denom;
    if(t_out) *t_out = t;
    if(s_out) *s_out = s;
    *ox = x1 + t * dx1;
    *oy = y1 + t * dy1;
    return 0;
}

// =====================================================================
// 阶段 1: 圆弧偏置几何工具 (纯数学, 阶段 2-4 起被 process_corner 调用)
// =====================================================================
// 这三个 helper 不被任何现有代码路径引用, 仅服务于未来 G02/G03 偏置。
// 提前定义为后续阶段铺路, 同时可独立单测验证几何正确性。

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学。
// 圆弧偏置: 给定原始圆心/半径/CW方向/补偿模式/刀具半径, 算偏置后圆心 + 偏置半径
//
// Fanuc 标准偏置规则 (G41=COMP_LEFT, G42=COMP_RIGHT):
//   - G41 + CCW(G03): 偏置朝内, R' = R - r  (刀心轨迹在工件圆弧内侧)
//   - G41 + CW  (G02): 偏置朝外, R' = R + r  (刀心轨迹在工件圆弧外侧)
//   - G42 + CCW(G03): 偏置朝外, R' = R + r
//   - G42 + CW  (G02): 偏置朝内, R' = R - r
//
// 物理推导 (圆心在原点, 起点 (R,0)):
//   - G03 CCW 在 (R,0) 切向 (0,+1), 左法向 (-1,0) 朝心 → G41 刀心 (R-r,0)
//   - G02 CW  在 (R,0) 切向 (0,-1), 左法向 (+1,0) 朝外 → G41 刀心 (R+r,0)
//   - G42 与 G41 镜像 (右法向)
//
// 规则总结: (mode==LEFT) XOR (is_CW==1) → 内偏 R-r; 否则外偏 R+r
//   内偏 R < r 时返回 -1 (过切, 刀具半径大于工件圆弧半径)
//
// 圆心不变: 偏置圆弧与原始圆弧同心 (刀具中心轨迹在同一圆心上)
//
// 参数:
//   center[2]:  原始圆心 (ax1, ax2)
//   R:          原始半径
//   is_CW:      1=CW(G02), 0=CCW(G03)
//   mode:       COMP_LEFT / COMP_RIGHT
//   radius:     刀具半径 r
//   out_center: 偏置后圆心 (= center, 同心)
//   out_R:      偏置后半径 R'
// 返回 0=成功, -1=过切
static int offset_arc(const double center[2], double R, int is_CW,
                       int mode, double radius,
                       double out_center[2], double *out_R)
{
    // (mode==LEFT) XOR (is_CW==1) → 内偏
    int inner = ((mode == COMP_LEFT) ? 1 : 0) ^ (is_CW ? 1 : 0);
    double R_off = inner ? (R - radius) : (R + radius);
    if(R_off < COMP_ZERO_LEN_TOL) {
        // 内偏后半径 <= 0: 刀具半径大于工件圆弧半径, 无法加工
        return -1;
    }
    out_center[0] = center[0];
    out_center[1] = center[1];
    *out_R = R_off;
    return 0;
}

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学。
// 直线与圆弧求切点 (用于 LINE→ARC / ARC→LINE 拐角处理)
//
// 几何关系:
//   偏置后的直线 (offset_line) 与偏置后的圆弧 (offset_arc) 在切点处相切
//   切点条件: 直线到圆心距离 = 偏置圆弧半径 R'
//
// 求解 (line_dir 为单位向量时):
//   1. 直线方程: P = line_p0 + t * line_dir (t 单位为 mm)
//   2. 圆心到直线距离: d = |cross(line_dir, arc_center - line_p0)| (单位向量省略 /|dir|)
//   3. 切点为直线到圆心垂线的垂足: t = dot(arc_center - line_p0, line_dir)
//
// 越界检测 (函数内部完成, 越界返回 -1):
//   - t ∈ [0, line_len]:        切点必须落在直线段有效范围内
//   - |s| ∈ [0, |arc_sweep|]:   切点必须落在圆弧扫角范围内 (s 与 arc_sweep 同号)
//
// 参数:
//   line_p0[2]:       直线起点 (ax1, ax2)
//   line_dir[2]:      直线单位方向向量 (调用方必须归一化, 否则返回 -1)
//   line_len:         直线段长度 (mm), 用于 t 越界检测
//   arc_center[2]:    圆心
//   arc_R:            圆弧半径 (偏置后)
//   arc_start_angle:  圆弧起点相对圆心的极角 (rad, atan2 取值范围 (-π, π])
//   arc_sweep:        圆弧扫角 (rad, 正=CCW G03, 负=CW G02)
//   out_tan[2]:       切点坐标
//   out_t:            切点在直线上的参数 (mm, 范围 [0, line_len])
//   out_s:            切点在圆弧上的扫角位置 (rad, 与 arc_sweep 同符号; |s| 范围 [0, |arc_sweep|])
// 返回 0=成功, -1=不相切 / line_dir 非单位 / 切点越界 / 几何退化
static int line_arc_tangent(const double line_p0[2], const double line_dir[2],
                             double line_len,
                             const double arc_center[2], double arc_R,
                             double arc_start_angle, double arc_sweep,
                             double out_tan[2], double *out_t, double *out_s)
{
    // ---- 参数契约: line_dir 必须是单位向量 ----
    double dir_sq = line_dir[0]*line_dir[0] + line_dir[1]*line_dir[1];
    if(fabs(dir_sq - 1.0) > 1e-6) return -1;
    if(line_len < COMP_ZERO_LEN_TOL) return -1;

    double dx = arc_center[0] - line_p0[0];
    double dy = arc_center[1] - line_p0[1];

    // ---- 直线到圆心的距离 (line_dir 已归一化, 省略 /|dir|) ----
    double cross = line_dir[0]*dy - line_dir[1]*dx;
    double dist = fabs(cross);

    // ---- 切点条件: dist == arc_R (相对容差, R≈0 时退化为绝对容差) ----
    double tol = COMP_TANGENT_TOL * (arc_R + 1.0);
    if(fabs(dist - arc_R) > tol) return -1;

    // ---- 切点 = 圆心到直线的垂足 (line_dir 单位向量 so t 直接是 mm) ----
    double t = line_dir[0]*dx + line_dir[1]*dy;

    // ---- 越界检测: t ∈ [0, line_len] ----
    if(t < -COMP_ZERO_LEN_TOL || t > line_len + COMP_ZERO_LEN_TOL) return -1;

    out_tan[0] = line_p0[0] + t * line_dir[0];
    out_tan[1] = line_p0[1] + t * line_dir[1];
    if(out_t) *out_t = t;

    // ---- 切点在圆弧上的扫角位置 s (相对 start_angle) ----
    if(out_s) {
        double abs_angle = atan2(out_tan[1] - arc_center[1],
                                  out_tan[0] - arc_center[0]);
        double delta = abs_angle - arc_start_angle;
        // 归一化 delta 到 [-2π, 2π]
        while(delta >  2.0 * M_PI) delta -= 2.0 * M_PI;
        while(delta < -2.0 * M_PI) delta += 2.0 * M_PI;

        double s;
        if(arc_sweep >= 0.0) {
            // CCW: s 应 ∈ [0, arc_sweep]
            while(delta < -COMP_PARALLEL_TOL) delta += 2.0 * M_PI;
            s = delta;
            if(s > arc_sweep + COMP_PARALLEL_TOL) return -1;
        } else {
            // CW: s 应 ∈ [arc_sweep, 0]
            while(delta > COMP_PARALLEL_TOL) delta -= 2.0 * M_PI;
            s = delta;
            if(s < arc_sweep - COMP_PARALLEL_TOL) return -1;
        }
        *out_s = s;
    }
    return 0;
}

// (已删除 arc_arc_tangent: 无调用方 + 死代码 + 实际 ARC-ARC 拐角用圆-圆相交直接求解)
// 旧版死代码 bug: r1_eff = external ? R1 : R1 (三元两分支相同, 死代码) +
//                 sign 翻转逻辑漏 cw2 + 缺 s1/s2 输出 + 多种退化漏掉
// 替代方案: process_corner_arc_arc 内部直接用圆-圆相交公式, 不需此 helper

// ================== 刀补输出 helper (探针埋点 + 回调转发) ==================
//
// @Context: Non-RealTime Background Thread (parser)
// @Stage: STAGE_CUTTER_COMP —— 刀补引擎输出的偏置后坐标
//
// 所有"生成了新的偏置坐标并准备调用 g_comp.output_fn"的代码路径统一走本函数:
//   1. 先记录 STAGE_CUTTER_COMP 探针 (pos 即偏置后机械绝对坐标)
//   2. 再转发到 output_fn (下发到 B-Spline 或 Planner)
// 这样 process_corner / flush_last_segment 中的 10+ 个 output_fn 调用点
// 自动获得统一的管线探针埋点,不会漏掉任何偏置输出。
// speed 单位: 调用方传 mm/s,trace 字段 v_target 期望 mm/ms,做 /1000.0 换算。
//
// C3 (2026-07-27): cutter_emit 签名不变 (caller 仍传 4 参数),
// 内部从 g_comp 读取 G93 上下文 (g93_strict_pending/g93_dt_sec_pending),
// 透传到 output_fn. PushPoint/PushArc 入口负责缓存 G93 上下文.
static int cutter_emit(const double pos[AXIS_NUM], double speed,
                       double acc, double dec)
{
    TraceLogger_PushPipeline(STAGE_CUTTER_COMP, pos, speed / 1000.0);
    // C3-fix2: 记录实际刀位 (圆弧离散化入口对齐用)
    memcpy(g_comp.last_emit_pos, pos, sizeof(double) * AXIS_NUM);
    g_comp.last_emit_valid = 1;
    return g_comp.output_fn(pos, speed, acc, dec,
                             g_comp.g93_strict_pending,
                             g_comp.g93_dt_sec_pending);
}

// C3-fix (2026-07-28): 显式 G93 上下文版本 —— 圆弧离散化微段时间均分专用.
//
// 为什么不能用 cutter_emit (全局 pending):
//   ARC 本体离散化被推迟到"下一段 push 时"才发生 (滑动窗口延迟输出),
//   彼时 PushPoint/PushArc 入口已把 g_comp.g93_dt_sec_pending 覆盖为
//   新段的整段预算 (如 6s), 且从未按微段均分 →
//   每个微段各拿整弧 6s, 整弧耗时膨胀 num_seg 倍 (实测 N×6000ms)。
// 修复: 圆弧微段一律显式传"段自身 g93_dt_sec 均分后的 seg_T"。
static int cutter_emit_dt(const double pos[AXIS_NUM], double speed,
                          double acc, double dec,
                          int is_g93_strict, double g93_dt_sec)
{
    TraceLogger_PushPipeline(STAGE_CUTTER_COMP, pos, speed / 1000.0);
    // C3-fix2: 记录实际刀位 (圆弧离散化入口对齐用)
    memcpy(g_comp.last_emit_pos, pos, sizeof(double) * AXIS_NUM);
    g_comp.last_emit_valid = 1;
    return g_comp.output_fn(pos, speed, acc, dec,
                             is_g93_strict, g93_dt_sec);
}

// ================== 核心：拐角分类与轨迹重构 ==================
//
// 阶段 2 重构: 拐角处理按 (segA.type, segB.type) 分发
//   LINE-LINE: 现有逻辑 (process_corner_line_line, 已稳定, 几何零改动)
//   LINE-ARC / ARC-LINE / ARC-ARC: 阶段 3 起填充 (process_corner_mixed stub)

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学计算 + 调用回调函数。
//
// LINE-LINE 拐角处理 (从原 process_corner 完整迁移, 几何逻辑零改动):
//   segA: prev→curr (LINE)
//   segB: curr→next (LINE)
//
// 拓扑判定表 (2D 叉乘 cross = uA × uB):
//   cross * mode_sign > 0 → 内拐角 (Inner): 偏置线收敛 → 求交点 + 燕尾检测
//   cross * mode_sign < 0 → 外拐角 (Outer): 偏置线断口 → 插过渡圆弧
//
// 返回 0=成功, -1=燕尾过切
static int process_corner_line_line(const CompSegment_t *segA,
                                     const CompSegment_t *segB)
{
    const double *prev = segA->start_pos;
    const double *curr = segA->end_pos;   // == segB->start_pos
    const double *next = segB->end_pos;
    double speed = segB->speed;
    double acc   = segB->acc;
    double dec   = segB->dec;

    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    // 段 A 方向: prev → curr
    double dAx = curr[ax1] - prev[ax1];
    double dAy = curr[ax2] - prev[ax2];
    // 段 B 方向: curr → next
    double dBx = next[ax1] - curr[ax1];
    double dBy = next[ax2] - curr[ax2];

    double lenA = hypot(dAx, dAy);
    double lenB = hypot(dBx, dBy);
    if(lenA < COMP_ZERO_LEN_TOL || lenB < COMP_ZERO_LEN_TOL) return 0;

    // 单位方向向量
    double uAx = dAx / lenA, uAy = dAy / lenA;
    double uBx = dBx / lenB, uBy = dBy / lenB;

    // 2D 叉乘 → 转向判定: >0 左转(CCW), <0 右转(CW)
    double cross = cross2d(uAx, uAy, uBx, uBy);

    // 偏置方向符号: G41=+1(左), G42=-1(右)
    int mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;

    // 段 A 的偏置法向量 (指向刀具中心侧)
    double nAx, nAy;
    compute_left_normal(uAx, uAy, &nAx, &nAy);
    nAx *= mode_sign;
    nAy *= mode_sign;

    // 段 B 的偏置法向量
    double nBx, nBy;
    compute_left_normal(uBx, uBy, &nBx, &nBy);
    nBx *= mode_sign;
    nBy *= mode_sign;

    double R = g_comp.radius;

    // ---- 首段偏置起始点 ----
    // 先输出编程轨迹点 prev (从上一个未补偿位置平滑过渡),
    // 再输出偏置起点, 形成垂直切入偏置线。
    if(g_comp.first_seg_pending) {
        cutter_emit(prev, speed, acc, dec);
        double start[AXIS_NUM];
        memcpy(start, prev, sizeof(double) * AXIS_NUM);
        start[ax1] += R * nAx;
        start[ax2] += R * nAy;
        cutter_emit(start, speed, acc, dec);
        g_comp.first_seg_pending = 0;
    }

    // ---- 共线检测: 叉乘接近零，无拐角 ----
    if(fabs(cross) < COMP_PARALLEL_TOL) {
        double pt[AXIS_NUM];
        memcpy(pt, curr, sizeof(double) * AXIS_NUM);
        pt[ax1] += R * nAx;
        pt[ax2] += R * nAy;
        cutter_emit(pt, speed, acc, dec);
        return 0;
    }

    // ---- 内外拐角分类 ----
    if(cross * mode_sign > 0) {
        // ============================================================
        // 内拐角 (Inner Corner): 偏置线收敛
        //
        // 刀具被挤向工件侧，两条偏置线自然相交，交点替代原始拐角。
        // 必须校验交点参数 t1, s2 的合法性:
        //   t1 ∈ [0, lenA]: 交点在偏置线 A 的有效范围内
        //   s2 ∈ [-lenB, 0]: 交点在偏置线 B 的有效范围内
        // 越界 = 燕尾过切 (刀具半径 > 最小曲率半径), 报错拦截。
        // ============================================================
        double oAx = prev[ax1] + R * nAx;
        double oAy = prev[ax2] + R * nAy;
        double oBx = next[ax1] + R * nBx;
        double oBy = next[ax2] + R * nBy;

        double ix, iy, tA, sB;
        if(line_intersect(oAx, oAy, uAx, uAy,
                          oBx, oBy, uBx, uBy,
                          &ix, &iy, &tA, &sB) == 0) {
            // 燕尾拦截: 交点必须落在两段偏置线的正向范围内
            // t1 > lenA → 交点反向延伸, 刀具倒退切穿工件
            // s2 < -lenB → 交点超出 B 段, 同理
            // t1 < 0 或 s2 > 0 → 交点在反方向
            if(tA < -1e-6 || tA > lenA + 1e-6 ||
               sB < -lenB - 1e-6 || sB > 1e-6) {
                printf("[CutterComp报警] 燕尾过切! 内拐角干涉: "
                       "R=%.2f, tA=%.2f/%.2f, sB=%.2f/-%.2f\n",
                       R, tA, lenA, sB, lenB);
                return -1;
            }

            double pt[AXIS_NUM];
            memcpy(pt, curr, sizeof(double) * AXIS_NUM);
            pt[ax1] = ix;
            pt[ax2] = iy;
            cutter_emit(pt, speed, acc, dec);
        }

    } else {
        // ============================================================
        // 外拐角 (Outer Corner): 偏置线出现断口
        //
        // 刀具被甩向外侧，两条偏置线之间存在间隙。
        // 几何处理:
        //   1. 前段延伸至垂足 footA = curr + R * nA
        //   2. 插入过渡圆弧: 圆心=curr, 半径=R, 从 footA 到 footB
        //      G41 外角圆弧为 CW (sweep < 0), G42 外角圆弧为 CCW (sweep > 0)
        //   3. 后段从垂足 footB = curr + R * nB 开始
        // ============================================================
        double footAx = curr[ax1] + R * nAx;
        double footAy = curr[ax2] + R * nAy;
        double footBx = curr[ax1] + R * nBx;
        double footBy = curr[ax2] + R * nBy;

        // ① 输出前段延伸终点 (垂足 A)
        double ptA[AXIS_NUM];
        memcpy(ptA, curr, sizeof(double) * AXIS_NUM);
        ptA[ax1] = footAx;
        ptA[ax2] = footAy;
        cutter_emit(ptA, speed, acc, dec);

        // ② 生成过渡圆弧微段
        double start_angle = atan2(footAy - curr[ax2], footAx - curr[ax1]);
        double end_angle   = atan2(footBy - curr[ax2], footBx - curr[ax1]);
        double sweep = end_angle - start_angle;

        // 归一化到 [-PI, PI]: 取短弧方向
        // 经拓扑验证, 短弧方向与正确的圆弧方向天然一致:
        //   G41 外拐角 → sweep < 0 (CW)
        //   G42 外拐角 → sweep > 0 (CCW)
        while(sweep >  M_PI) sweep -= 2.0 * M_PI;
        while(sweep < -M_PI) sweep += 2.0 * M_PI;

        double arc_len = fabs(sweep) * R;
        int num_seg = (int)ceil(arc_len / COMP_ARC_STEP_MM);
        if(num_seg < 1) num_seg = 1;
        double angle_step = sweep / (double)num_seg;

        // ③ 生成圆弧微段: 最后一微段强制对齐理论终点 footB，消除浮点累积
        // C3-fix (2026-07-28): 过渡弧是刀补插入的非编程几何, 不占 G93 时间预算
        // → 显式走非 strict 路径 (dt=0), 防止微段各拿整块 6s 导致 N×6s 膨胀
        for(int i = 1; i <= num_seg; i++) {
            double pt[AXIS_NUM];
            memcpy(pt, curr, sizeof(double) * AXIS_NUM);
            if(i == num_seg) {
                pt[ax1] = footBx;
                pt[ax2] = footBy;
            } else {
                double theta = start_angle + (double)i * angle_step;
                pt[ax1] = curr[ax1] + R * cos(theta);
                pt[ax2] = curr[ax2] + R * sin(theta);
            }
            cutter_emit_dt(pt, speed, acc, dec, 0, 0.0);
        }
    }

    return 0;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 + 调用回调。
//
// LINE-ARC 拐角处理 (方案 B 第 1 个里程碑):
//   segA: prev → curr (LINE)
//   segB : curr → next (ARC), 圆心/半径/CW 已在 CompSegment_t 中
//
// 输出时序 (继承 process_segment_pair 语义): 仅输出 segA 的偏置轨迹 + 拐角过渡,
//   ARC segB 等下一段处理时再离散化输出。
//
// 拓扑判定 (2D 叉乘 cross = uA × uB_tangent_at_start):
//   cross * mode_sign > 0 → 内拐角: 偏置线与偏置弧相交 → 二次方程求交点
//   cross * mode_sign < 0 → 外拐角: 偏置线与偏置弧断口 → 插过渡弧 (圆心=curr, 半径=R)
//   |cross| < tol         → 光滑过渡 (共切线): 直接输出 segA 偏置终点
//
// 过切检测:
//   - segB 内偏后 R' ≤ 0      → offset_arc 返回 -1 (已有)
//   - 内角二次方程 disc < 0   → 偏置线与偏置弧无交点 (几何退化)
//   - 交点 t ∉ [0, lenA]      → segA 段内无交点 (内角干涉)
//   - 交点 s ∉ [0, sweep]     → segB 弧段内无交点 (sweep 不足)
//
// 返回 0=成功, -1=过切/燕尾
static int process_corner_line_arc(const CompSegment_t *segA,
                                    const CompSegment_t *segB)
{
    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    const double *prev = segA->start_pos;
    const double *curr = segA->end_pos;   // == segB->start_pos
    double speed = segB->speed, acc = segB->acc, dec = segB->dec;
    double R = g_comp.radius;
    int    mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;

    // ---- segA 方向 + 单位化 ----
    double dAx = curr[ax1] - prev[ax1];
    double dAy = curr[ax2] - prev[ax2];
    double lenA = hypot(dAx, dAy);
    if(lenA < COMP_ZERO_LEN_TOL) return 0;
    double uAx = dAx / lenA, uAy = dAy / lenA;

    // ---- segA 偏置法向 ----
    double nAx, nAy;
    compute_left_normal(uAx, uAy, &nAx, &nAy);
    nAx *= mode_sign; nAy *= mode_sign;

    // ---- segB 偏置弧 (offset_arc: 同心 + R±r) ----
    if(fabs(segB->radius) < COMP_ZERO_LEN_TOL) return -1;
    double center_off[2], R_off;
    if(offset_arc(segB->center, segB->radius, segB->is_CW,
                   g_comp.mode, R, center_off, &R_off) < 0) {
        printf("[CutterComp报警] LINE→ARC 圆弧过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
               segB->radius, R);
        return -1;
    }

    // ---- segB 起点切向 (用于内外角分类) ----
    // CCW 切向 = 径向左法向; CW 切向 = 径向右法向 (取反)
    double rad_x = segB->start_pos[ax1] - segB->center[0];
    double rad_y = segB->start_pos[ax2] - segB->center[1];
    double tx, ty;
    compute_left_normal(rad_x, rad_y, &tx, &ty);
    if(segB->is_CW) { tx = -tx; ty = -ty; }

    // ---- 内/外角分类 ----
    double cross = cross2d(uAx, uAy, tx, ty);

    // ---- 首段处理 (与 LINE-LINE 一致: 输出 prev + 偏置起点) ----
    if(g_comp.first_seg_pending) {
        cutter_emit(prev, speed, acc, dec);
        double start[AXIS_NUM];
        memcpy(start, prev, sizeof(double) * AXIS_NUM);
        start[ax1] += R * nAx;
        start[ax2] += R * nAy;
        cutter_emit(start, speed, acc, dec);
        g_comp.first_seg_pending = 0;
    }

    // ---- 光滑过渡 (共切线): 直接输出 segA 偏置终点 ----
    if(fabs(cross) < COMP_PARALLEL_TOL) {
        double pt[AXIS_NUM];
        memcpy(pt, curr, sizeof(double) * AXIS_NUM);
        pt[ax1] += R * nAx;
        pt[ax2] += R * nAy;
        cutter_emit(pt, speed, acc, dec);
        return 0;
    }

    // segA 偏置线起点: prev + R*nA (沿 uA 方向延伸)
    double offA0_x = prev[ax1] + R * nAx;
    double offA0_y = prev[ax2] + R * nAy;

    if(cross * mode_sign > 0) {
        // ============================================================
        // 内拐角: 偏置线与偏置弧相交
        // 直线 P = offA0 + t * uA  (uA 单位向量, t 单位 mm)
        // 圆   (P - center_off)^2 = R_off^2
        // 代入展开 (uA 已归一化): t^2 + 2 b t + c = 0
        //   b = (offA0 - center_off) · uA
        //   c = |offA0 - center_off|^2 - R_off^2
        // ============================================================
        double dx0 = offA0_x - center_off[0];
        double dy0 = offA0_y - center_off[1];
        double b = dx0 * uAx + dy0 * uAy;
        double c = dx0 * dx0 + dy0 * dy0 - R_off * R_off;
        double disc = b * b - c;   // 等价 4(b²-c), 直接判 disc

        if(disc < 0.0) {
            printf("[CutterComp报警] LINE→ARC 内角偏置线与偏置弧无交点 "
                   "(disc=%.4f, 几何退化)\n", disc);
            return -1;
        }
        double sqd = sqrt(disc);
        double t1 = -b - sqd;   // 入口根 (较小)
        double t2 = -b + sqd;   // 出口根

        // 选 t ∈ [0, lenA] 中较小者 (偏置线最先碰到的弧点)
        double t;
        int t1_ok = (t1 >= -COMP_ZERO_LEN_TOL && t1 <= lenA + COMP_ZERO_LEN_TOL);
        int t2_ok = (t2 >= -COMP_ZERO_LEN_TOL && t2 <= lenA + COMP_ZERO_LEN_TOL);
        if(t1_ok && t2_ok)      t = (t1 < t2) ? t1 : t2;
        else if(t1_ok)          t = t1;
        else if(t2_ok)          t = t2;
        else {
            printf("[CutterComp报警] LINE→ARC 内角交点越界: "
                   "t1=%.3f, t2=%.3f, lenA=%.3f\n", t1, t2, lenA);
            return -1;
        }

        // 验证交点在 segB 偏置弧扫角范围内
        double ix = offA0_x + t * uAx;
        double iy = offA0_y + t * uAy;
        double start_angle = atan2(rad_y, rad_x);
        double abs_angle   = atan2(iy - center_off[1], ix - center_off[0]);
        double delta = abs_angle - start_angle;
        while(delta >  M_PI) delta -= 2.0 * M_PI;
        while(delta < -M_PI) delta += 2.0 * M_PI;
        double sweep = segB->sweep;
        if(sweep >= 0.0) {
            while(delta < -COMP_PARALLEL_TOL) delta += 2.0 * M_PI;
            if(delta > sweep + COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] LINE→ARC 内角交点越弧段: "
                       "s=%.4f rad > sweep=%.4f rad\n", delta, sweep);
                return -1;
            }
        } else {
            while(delta > COMP_PARALLEL_TOL) delta -= 2.0 * M_PI;
            if(delta < sweep - COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] LINE→ARC 内角交点越弧段: "
                       "s=%.4f rad < sweep=%.4f rad\n", delta, sweep);
                return -1;
            }
        }

        // 输出交点 (segA 偏置终点 = segB 偏置起点)
        double pt[AXIS_NUM];
        memcpy(pt, curr, sizeof(double) * AXIS_NUM);
        pt[ax1] = ix;
        pt[ax2] = iy;
        cutter_emit(pt, speed, acc, dec);

    } else {
        // ============================================================
        // 外拐角: 偏置线与偏置弧断口, 插过渡弧
        // 过渡弧: 圆心 = curr, 半径 = R
        //   起点 footA = curr + R * nA       (segA 偏置线垂足)
        //   终点 footB = curr + R * nB_start (segB 偏置弧上, nB = 径向朝偏置侧)
        // nB_start 推导:
        //   CCW+LEFT  内偏 → -径向, CW+LEFT  外偏 → +径向
        //   CCW+RIGHT 外偏 → +径向, CW+RIGHT 内偏 → -径向
        //   规律: inner = (mode==LEFT) XOR is_CW; inner→-径向, outer→+径向
        // ============================================================
        double inv_R = 1.0 / segB->radius;
        double radial_ux = rad_x * inv_R;
        double radial_uy = rad_y * inv_R;
        int inner_segB = ((g_comp.mode == COMP_LEFT) ? 1 : 0) ^ (segB->is_CW ? 1 : 0);
        double rsign = inner_segB ? -1.0 : 1.0;
        double nBx = rsign * radial_ux;
        double nBy = rsign * radial_uy;

        double footAx = curr[ax1] + R * nAx;
        double footAy = curr[ax2] + R * nAy;
        double footBx = curr[ax1] + R * nBx;
        double footBy = curr[ax2] + R * nBy;

        // ① 输出 segA 偏置线延伸终点 (footA)
        double ptA[AXIS_NUM];
        memcpy(ptA, curr, sizeof(double) * AXIS_NUM);
        ptA[ax1] = footAx;
        ptA[ax2] = footAy;
        cutter_emit(ptA, speed, acc, dec);

        // ② 生成过渡圆弧微段 (与 LINE-LINE 外角逻辑同源)
        double start_angle = atan2(footAy - curr[ax2], footAx - curr[ax1]);
        double end_angle   = atan2(footBy - curr[ax2], footBx - curr[ax1]);
        double sweep = end_angle - start_angle;
        while(sweep >  M_PI) sweep -= 2.0 * M_PI;
        while(sweep < -M_PI) sweep += 2.0 * M_PI;

        double arc_len = fabs(sweep) * R;
        int num_seg = (int)ceil(arc_len / COMP_ARC_STEP_MM);
        if(num_seg < 1) num_seg = 1;
        double angle_step = sweep / (double)num_seg;

        // C3-fix (2026-07-28): 过渡弧非编程几何, 不占 G93 预算 (dt=0)
        for(int i = 1; i <= num_seg; i++) {
            double pt[AXIS_NUM];
            memcpy(pt, curr, sizeof(double) * AXIS_NUM);
            if(i == num_seg) {
                pt[ax1] = footBx;
                pt[ax2] = footBy;
            } else {
                double theta = start_angle + (double)i * angle_step;
                pt[ax1] = curr[ax1] + R * cos(theta);
                pt[ax2] = curr[ax2] + R * sin(theta);
            }
            cutter_emit_dt(pt, speed, acc, dec, 0, 0.0);
        }
    }

    return 0;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 + 调用回调。
//
// segA 偏置弧离散化 helper (方案 B 第 5 个里程碑抽出, 供 ARC-LINE / ARC-ARC 复用):
//   - 离散化范围: 从理论起点 start_off 沿 s_end 扫角到 end_off
//   - s_end 单位 rad, 与 arc->sweep 同符号; |s_end| ≤ |arc->sweep|
//   - 平面两轴: center + R_off·(cos θ, sin θ)
//   - 非平面轴: 按 (i/num_seg)·(s_end/sweep) 比例线性跟随 (支持螺旋 + 部分离散化)
//   - 末点强制对齐 (end_off_x, end_off_y), 消除浮点累积
//
// 注: 当 s_end = arc->sweep 时等价全 sweep 离散化 (与 flush_last_segment ARC 末段同源)
static void discretize_offset_arc(const CompSegment_t *arc,
                                   double R_off, double start_angle, double s_end,
                                   double end_off_x, double end_off_y,
                                   int ax1, int ax2,
                                   double speed, double acc, double dec)
{
    // C3-fix2 (2026-07-28): 入口对齐 —— 消除"回程过切"。
    // 场景: segA(上段) 与本弧为内拐角时, 实际入刀点已被剪裁到偏置弧内
    // 某角度 (如 165.5°), 但调用方仍传理论起点 (180°)。若不对齐, 第一个
    // 微段会回退重切已剪裁的楔形区 (几何过切), 且该微段长度异常
    // (实测 4.55mm vs 正常 0.5mm) → G93 均分时间下 v 尖峰 9 倍。
    // 对齐条件: 最后 emit 刀位落在偏置圆上 (|dist-R_off|<10µm) 且沿扫向
    // 领先理论起点不超过 |s_end| → 前移 start_angle, 缩短 s_end
    // (时间随 sweep_ratio 自动按占比缩短, 保持匀速)。
    if(g_comp.last_emit_valid) {
        double ldx = g_comp.last_emit_pos[ax1] - arc->center[0];
        double ldy = g_comp.last_emit_pos[ax2] - arc->center[1];
        double lr  = hypot(ldx, ldy);
        if(fabs(lr - R_off) < 0.01) {
            double la = atan2(ldy, ldx);
            double delta = la - start_angle;
            if(s_end >= 0.0) {
                while(delta < 0.0)          delta += 2.0 * M_PI;
                while(delta >= 2.0 * M_PI)  delta -= 2.0 * M_PI;
                if(delta > COMP_PARALLEL_TOL && delta <= s_end + COMP_PARALLEL_TOL) {
                    start_angle += delta;
                    s_end -= delta;
                    if(s_end < 0.0) s_end = 0.0;
                }
            } else {
                while(delta > 0.0)           delta -= 2.0 * M_PI;
                while(delta <= -2.0 * M_PI)  delta += 2.0 * M_PI;
                if(delta < -COMP_PARALLEL_TOL && delta >= s_end - COMP_PARALLEL_TOL) {
                    start_angle += delta;
                    s_end -= delta;
                    if(s_end > 0.0) s_end = 0.0;
                }
            }
        }
    }

    double arc_len = fabs(s_end) * R_off;
    int num_seg = (int)ceil(arc_len / COMP_ARC_STEP_MM);
    if(num_seg < 1) num_seg = 1;
    double angle_step = s_end / (double)num_seg;
    double sweep_ratio = (fabs(arc->sweep) > COMP_PARALLEL_TOL)
                         ? (s_end / arc->sweep) : 1.0;

    // C3-fix (2026-07-28): G93 时间均分 —— 用"段自身" arc->g93_dt_sec
    // (不能用 g_comp.g93_dt_sec_pending: ARC 延迟输出时已被下一段覆盖)。
    // 部分离散化 (内角剪裁 s_end < sweep) 时, 时间按扫角占比缩放:
    //   T_portion = T_total × |s_end / sweep|
    // 再按 strategy-A 同款累计差分法均分到 num_seg 个微段 (浮点稳定):
    //   seg_T(i) = T_portion×i/N − T_portion×(i−1)/N
    double T_portion = (arc->g93_dt_sec > 1e-9)
                       ? arc->g93_dt_sec * fabs(sweep_ratio) : 0.0;
    int is_g93_arc = (T_portion > 1e-9) ? 1 : 0;

    for(int i = 1; i <= num_seg; i++) {
        double pt[AXIS_NUM];
        memcpy(pt, arc->start_pos, sizeof(double) * AXIS_NUM);
        double ratio = (double)i / (double)num_seg * sweep_ratio;

        // 非平面轴线性跟随 (按 sweep_ratio 缩放, 支持部分离散化)
        for(int j = 0; j < AXIS_NUM; j++) {
            if(j != ax1 && j != ax2) {
                pt[j] = arc->start_pos[j]
                      + (arc->end_pos[j] - arc->start_pos[j]) * ratio;
            }
        }

        if(i == num_seg) {
            // 末点强制对齐 (消除浮点累积)
            pt[ax1] = end_off_x;
            pt[ax2] = end_off_y;
        } else {
            double theta = start_angle + (double)i * angle_step;
            pt[ax1] = arc->center[0] + R_off * cos(theta);
            pt[ax2] = arc->center[1] + R_off * sin(theta);
        }
        double seg_T = is_g93_arc
                       ? (T_portion * (double)i / (double)num_seg
                          - T_portion * (double)(i - 1) / (double)num_seg)
                       : 0.0;
        cutter_emit_dt(pt, speed, acc, dec, is_g93_arc, seg_T);
    }
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 + 调用回调。
//
// ARC-LINE 拐角处理 (方案 B 第 5 个里程碑, 与 LINE-ARC 对称):
//   segA: ARC (prev_arc → curr), 圆心/半径/CW/sweep 已在 CompSegment_t 中
//   segB : LINE (curr → next)
//
// 输出时序: 离散化 segA 偏置弧 + 拐角过渡, segB 等下一段处理时输出。
//
// 拓扑判定 (segA 终点切向 vs segB 方向):
//   segA 终点切向:
//     CCW (G03): 左法向 of (end - center)
//     CW  (G02): -左法向 of (end - center)
//   cross = uA_end_tangent × uB
//   cross * mode_sign > 0 → 内拐角: 偏置弧与偏置线相交 → 二次方程求交点
//   cross * mode_sign < 0 → 外拐角: 偏置弧与偏置线断口 → 离散化全 sweep + 过渡弧
//   |cross| < tol         → 光滑过渡: 直接离散化全 sweep 到 end_off
//
// 离散化范围:
//   光滑/外角 → 全 sweep (s_end = arc->sweep)
//   内角      → 部分 sweep (s_end = 交点扫角, segA 偏置弧在交点处被剪裁)
//
// 过切检测:
//   - offset_arc R' ≤ 0      → 已有 (PushArc 早失败, 此处二次保护)
//   - 内角 disc < 0          → 偏置弧与偏置线无交点
//   - 交点 t ∉ [0, lenB]      → segB 段内无交点
//   - 交点 s ∉ [0, |sweep|]   → segA 弧段内无交点 (内角剪裁过度)
//
// 返回 0=成功, -1=过切/燕尾
static int process_corner_arc_line(const CompSegment_t *segA,
                                    const CompSegment_t *segB)
{
    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    const double *curr = segA->end_pos;   // == segB->start_pos
    const double *next = segB->end_pos;
    double speed = segB->speed, acc = segB->acc, dec = segB->dec;
    double R = g_comp.radius;
    int    mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;

    // ---- segA 偏置弧 (offset_arc: 同心 + R±r) ----
    if(fabs(segA->radius) < COMP_ZERO_LEN_TOL) return -1;
    double center_off[2], R_off;
    if(offset_arc(segA->center, segA->radius, segA->is_CW,
                   g_comp.mode, R, center_off, &R_off) < 0) {
        printf("[CutterComp报警] ARC→LINE 圆弧过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
               segA->radius, R);
        return -1;
    }

    // ---- segB 方向 + 偏置法向 ----
    double dBx = next[ax1] - curr[ax1];
    double dBy = next[ax2] - curr[ax2];
    double lenB = hypot(dBx, dBy);
    if(lenB < COMP_ZERO_LEN_TOL) return 0;
    double uBx = dBx / lenB, uBy = dBy / lenB;
    double nBx, nBy;
    compute_left_normal(uBx, uBy, &nBx, &nBy);
    nBx *= mode_sign; nBy *= mode_sign;

    // ---- segA 终点切向 (用于内外角分类) ----
    double rad_ex = segA->end_pos[ax1] - segA->center[0];
    double rad_ey = segA->end_pos[ax2] - segA->center[1];
    double tx, ty;
    compute_left_normal(rad_ex, rad_ey, &tx, &ty);
    if(segA->is_CW) { tx = -tx; ty = -ty; }

    double cross = cross2d(tx, ty, uBx, uBy);

    // ---- segA 偏置弧理论起点/终点 (径向缩放到 R_off) ----
    double rad_sx = segA->start_pos[ax1] - segA->center[0];
    double rad_sy = segA->start_pos[ax2] - segA->center[1];
    double inv_R_orig = 1.0 / segA->radius;
    double start_off_x = segA->center[0] + R_off * rad_sx * inv_R_orig;
    double start_off_y = segA->center[1] + R_off * rad_sy * inv_R_orig;
    double end_off_x   = segA->center[0] + R_off * rad_ex * inv_R_orig;
    double end_off_y   = segA->center[1] + R_off * rad_ey * inv_R_orig;
    double start_angle = atan2(rad_sy, rad_sx);
    double sweep_total = segA->sweep;

    // ---- 首段防御 (PushArc 入口已硬报警拦截 G41+G02 起刀, 这里双保险) ----
    if(g_comp.first_seg_pending) {
        cutter_emit(segA->start_pos, speed, acc, dec);
        double start_off[AXIS_NUM];
        memcpy(start_off, segA->start_pos, sizeof(double) * AXIS_NUM);
        start_off[ax1] = start_off_x;
        start_off[ax2] = start_off_y;
        cutter_emit(start_off, speed, acc, dec);
        g_comp.first_seg_pending = 0;
    }

    // ============================================================
    // 光滑过渡 (共切线): cross ≈ 0
    // 离散化 segA 偏置弧全 sweep 到 end_off (与 segB 偏置线起点自然衔接)
    // ============================================================
    if(fabs(cross) < COMP_PARALLEL_TOL) {
        discretize_offset_arc(segA, R_off, start_angle, sweep_total,
                              end_off_x, end_off_y, ax1, ax2,
                              speed, acc, dec);
        return 0;
    }

    if(cross * mode_sign > 0) {
        // ============================================================
        // 内拐角: segA 偏置弧与 segB 偏置线相交
        // 二次方程求交点: t² + 2b t + c = 0 (uB 单位向量)
        //   offB0 = curr + R·nB (segB 偏置线起点)
        //   d0 = offB0 - center_off
        //   b = d0·uB,  c = |d0|² - R_off²
        // 离散化 segA 偏置弧从理论起点到交点 (部分 sweep)
        // ============================================================
        double offB0_x = curr[ax1] + R * nBx;
        double offB0_y = curr[ax2] + R * nBy;
        double dx0 = offB0_x - center_off[0];
        double dy0 = offB0_y - center_off[1];
        double b = dx0 * uBx + dy0 * uBy;
        double c = dx0 * dx0 + dy0 * dy0 - R_off * R_off;
        double disc = b * b - c;

        if(disc < 0.0) {
            printf("[CutterComp报警] ARC→LINE 内角偏置弧与偏置线无交点 "
                   "(disc=%.4f)\n", disc);
            return -1;
        }
        double sqd = sqrt(disc);
        double t1 = -b - sqd;
        double t2 = -b + sqd;

        double t;
        int t1_ok = (t1 >= -COMP_ZERO_LEN_TOL && t1 <= lenB + COMP_ZERO_LEN_TOL);
        int t2_ok = (t2 >= -COMP_ZERO_LEN_TOL && t2 <= lenB + COMP_ZERO_LEN_TOL);
        if(t1_ok && t2_ok)      t = (t1 < t2) ? t1 : t2;
        else if(t1_ok)          t = t1;
        else if(t2_ok)          t = t2;
        else {
            printf("[CutterComp报警] ARC→LINE 内角交点越界 segB: "
                   "t1=%.3f, t2=%.3f, lenB=%.3f\n", t1, t2, lenB);
            return -1;
        }

        double ix = offB0_x + t * uBx;
        double iy = offB0_y + t * uBy;

        // 验证交点在 segA 偏置弧扫角范围内
        double abs_angle = atan2(iy - center_off[1], ix - center_off[0]);
        double delta = abs_angle - start_angle;
        while(delta >  M_PI) delta -= 2.0 * M_PI;
        while(delta < -M_PI) delta += 2.0 * M_PI;
        double s_end;
        if(sweep_total >= 0.0) {
            while(delta < -COMP_PARALLEL_TOL) delta += 2.0 * M_PI;
            s_end = delta;
            if(s_end > sweep_total + COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→LINE 内角交点越弧段: "
                       "s=%.4f > sweep=%.4f\n", s_end, sweep_total);
                return -1;
            }
        } else {
            while(delta > COMP_PARALLEL_TOL) delta -= 2.0 * M_PI;
            s_end = delta;
            if(s_end < sweep_total - COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→LINE 内角交点越弧段: "
                       "s=%.4f < sweep=%.4f\n", s_end, sweep_total);
                return -1;
            }
        }

        // 离散化 segA 偏置弧从理论起点到交点 (s_end, 不是 sweep_total)
        discretize_offset_arc(segA, R_off, start_angle, s_end,
                              ix, iy, ax1, ax2, speed, acc, dec);

    } else {
        // ============================================================
        // 外拐角: segA 偏置弧与 segB 偏置线断口, 插过渡弧
        // 1) 离散化 segA 偏置弧全 sweep 到 end_off (= footA, 推导见下)
        // 2) 过渡弧: 圆心 = curr, 半径 = R, footA → footB
        //
        // footA 推导:
        //   segA 偏置弧在终点处的"朝偏置侧法向":
        //     inner_segA = (mode==LEFT) XOR is_CW → 内偏 → -径向, 外偏 → +径向
        //   nA_end = ±径向 (rad_ex, rad_ey)/R
        //   footA = curr + R·nA_end = end + (±R)·(end-center)/R = center + (R±r)·径向
        //         = center + R_off·径向 = end_off  ✓ (footA 与 end_off 重合)
        // ============================================================
        // 步骤 1: 离散化全 sweep 到 end_off
        discretize_offset_arc(segA, R_off, start_angle, sweep_total,
                              end_off_x, end_off_y, ax1, ax2,
                              speed, acc, dec);

        // 步骤 2: 过渡弧 footA → footB
        double footBx = curr[ax1] + R * nBx;
        double footBy = curr[ax2] + R * nBy;

        // footA == end_off (理论相等, 取 end_off 避免浮点误差)
        double start_angle_t = atan2(end_off_y - curr[ax2], end_off_x - curr[ax1]);
        double end_angle_t   = atan2(footBy - curr[ax2], footBx - curr[ax1]);
        double sweep_t = end_angle_t - start_angle_t;
        while(sweep_t >  M_PI) sweep_t -= 2.0 * M_PI;
        while(sweep_t < -M_PI) sweep_t += 2.0 * M_PI;

        double arc_len_t = fabs(sweep_t) * R;
        int num_seg = (int)ceil(arc_len_t / COMP_ARC_STEP_MM);
        if(num_seg < 1) num_seg = 1;
        double angle_step = sweep_t / (double)num_seg;

        // C3-fix (2026-07-28): 过渡弧非编程几何, 不占 G93 预算 (dt=0)
        for(int i = 1; i <= num_seg; i++) {
            double pt[AXIS_NUM];
            memcpy(pt, curr, sizeof(double) * AXIS_NUM);
            if(i == num_seg) {
                pt[ax1] = footBx;
                pt[ax2] = footBy;
            } else {
                double theta = start_angle_t + (double)i * angle_step;
                pt[ax1] = curr[ax1] + R * cos(theta);
                pt[ax2] = curr[ax2] + R * sin(theta);
            }
            cutter_emit_dt(pt, speed, acc, dec, 0, 0.0);
        }
    }

    return 0;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 + 调用回调。
//
// ARC-ARC 拐角处理 (方案 B 第 6 个里程碑, 最终):
//   segA: ARC (prev_arc → curr)
//   segB : ARC (curr → next_arc)
//
// 输出时序: 离散化 segA 偏置弧 + 拐角过渡, segB 等下一段处理时输出。
//
// 拓扑判定 (segA 终点切向 vs segB 起点切向):
//   segA 终点切向: CCW=左法向(end-center_A), CW=取反
//   segB 起点切向: CCW=左法向(start-center_B), CW=取反
//   cross = uA_end_tangent × uB_start_tangent
//   cross * mode_sign > 0 → 内拐角: 两偏置弧相交 (圆-圆相交公式)
//   cross * mode_sign < 0 → 外拐角: 两偏置弧断口 → 离散化 segA 全 sweep + 过渡弧
//   |cross| < tol         → 光滑过渡: 直接离散化 segA 全 sweep 到 end_off_A
//
// 内角圆-圆相交:
//   d = |center_off_B - center_off_A|
//   条件: |R_A - R_B| < d < R_A + R_B
//   交点公式: a = (R1² - R2² + d²) / (2d), h = √(R1² - a²)
//             中点 M = c1 + a*(c2-c1)/d
//             交点 = M ± h*perp(c2-c1)/d (取靠近 curr 的)
//   离散化 segA 偏置弧从理论起点到交点 (部分 sweep)
//
// 外角过渡弧:
//   圆心 = curr, 半径 = R_tool
//   footA = end_off_A (segA 终点偏置法向缩放, 等价 segA 偏置弧终点)
//   footB = start_off_B (segB 偏置弧理论起点)
//
// 过切检测:
//   - segA 或 segB 内偏 R' ≤ 0       → offset_arc 报警
//   - 内角两偏置弧圆心重合 / 不相交  → 报警
//   - 交点越 segA sweep 范围          → 报警
//   - 交点越 segB sweep 范围          → 报警
//
// 返回 0=成功, -1=过切/燕尾
static int process_corner_arc_arc(const CompSegment_t *segA,
                                   const CompSegment_t *segB)
{
    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    const double *curr = segA->end_pos;   // == segB->start_pos
    double speed = segB->speed, acc = segB->acc, dec = segB->dec;
    double R = g_comp.radius;
    int    mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;

    // ---- segA 偏置弧 ----
    if(fabs(segA->radius) < COMP_ZERO_LEN_TOL) return -1;
    double center_off_A[2], R_off_A;
    if(offset_arc(segA->center, segA->radius, segA->is_CW,
                   g_comp.mode, R, center_off_A, &R_off_A) < 0) {
        printf("[CutterComp报警] ARC→ARC segA 圆弧过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
               segA->radius, R);
        return -1;
    }

    // ---- segB 偏置弧 ----
    if(fabs(segB->radius) < COMP_ZERO_LEN_TOL) return -1;
    double center_off_B[2], R_off_B;
    if(offset_arc(segB->center, segB->radius, segB->is_CW,
                   g_comp.mode, R, center_off_B, &R_off_B) < 0) {
        printf("[CutterComp报警] ARC→ARC segB 圆弧过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
               segB->radius, R);
        return -1;
    }

    // ---- segA 终点切向 ----
    double rad_A_ex = segA->end_pos[ax1] - segA->center[0];
    double rad_A_ey = segA->end_pos[ax2] - segA->center[1];
    double tx_A, ty_A;
    compute_left_normal(rad_A_ex, rad_A_ey, &tx_A, &ty_A);
    if(segA->is_CW) { tx_A = -tx_A; ty_A = -ty_A; }

    // ---- segB 起点切向 ----
    double rad_B_sx = segB->start_pos[ax1] - segB->center[0];
    double rad_B_sy = segB->start_pos[ax2] - segB->center[1];
    double tx_B, ty_B;
    compute_left_normal(rad_B_sx, rad_B_sy, &tx_B, &ty_B);
    if(segB->is_CW) { tx_B = -tx_B; ty_B = -ty_B; }

    double cross = cross2d(tx_A, ty_A, tx_B, ty_B);

    // ---- segA 偏置弧理论起点/终点/扫角 ----
    double inv_R_A = 1.0 / segA->radius;
    double rad_A_sx = segA->start_pos[ax1] - segA->center[0];
    double rad_A_sy = segA->start_pos[ax2] - segA->center[1];
    double start_off_A_x = segA->center[0] + R_off_A * rad_A_sx * inv_R_A;
    double start_off_A_y = segA->center[1] + R_off_A * rad_A_sy * inv_R_A;
    double end_off_A_x   = segA->center[0] + R_off_A * rad_A_ex * inv_R_A;
    double end_off_A_y   = segA->center[1] + R_off_A * rad_A_ey * inv_R_A;
    double start_angle_A = atan2(rad_A_sy, rad_A_sx);
    double sweep_A = segA->sweep;

    // ---- 首段防御 ----
    if(g_comp.first_seg_pending) {
        cutter_emit(segA->start_pos, speed, acc, dec);
        double start_off[AXIS_NUM];
        memcpy(start_off, segA->start_pos, sizeof(double) * AXIS_NUM);
        start_off[ax1] = start_off_A_x;
        start_off[ax2] = start_off_A_y;
        cutter_emit(start_off, speed, acc, dec);
        g_comp.first_seg_pending = 0;
    }

    // ============================================================
    // 光滑过渡 (共切线): cross ≈ 0
    // 离散化 segA 偏置弧全 sweep 到 end_off_A (= segB 偏置弧理论起点)
    // ============================================================
    if(fabs(cross) < COMP_PARALLEL_TOL) {
        discretize_offset_arc(segA, R_off_A, start_angle_A, sweep_A,
                              end_off_A_x, end_off_A_y, ax1, ax2,
                              speed, acc, dec);
        return 0;
    }

    if(cross * mode_sign > 0) {
        // ============================================================
        // 内拐角: 两偏置弧相交 (圆-圆相交公式)
        // ============================================================
        double dx = center_off_B[0] - center_off_A[0];
        double dy = center_off_B[1] - center_off_A[1];
        double d = hypot(dx, dy);
        if(d < COMP_ZERO_LEN_TOL) {
            printf("[CutterComp报警] ARC→ARC 内角: 两偏置弧圆心重合\n");
            return -1;
        }

        // 相交条件: |R_A - R_B| < d < R_A + R_B
        double sum_R = R_off_A + R_off_B;
        double diff_R = fabs(R_off_A - R_off_B);
        if(d > sum_R + COMP_PARALLEL_TOL || d < diff_R - COMP_PARALLEL_TOL) {
            printf("[CutterComp报警] ARC→ARC 内角: 两偏置弧无交点 "
                   "(d=%.3f, R1+R2=%.3f, |R1-R2|=%.3f)\n", d, sum_R, diff_R);
            return -1;
        }

        // 圆-圆相交公式
        double a_coef = (R_off_A * R_off_A - R_off_B * R_off_B + d * d) / (2.0 * d);
        double h_sq = R_off_A * R_off_A - a_coef * a_coef;
        if(h_sq < 0.0) h_sq = 0.0;
        double h = sqrt(h_sq);
        double mx = center_off_A[0] + a_coef * dx / d;
        double my = center_off_A[1] + a_coef * dy / d;
        // 两个交点, 取靠近 curr 的
        double ix1 = mx - h * dy / d;
        double iy1 = my + h * dx / d;
        double ix2 = mx + h * dy / d;
        double iy2 = my - h * dx / d;
        double d1 = hypot(ix1 - curr[ax1], iy1 - curr[ax2]);
        double d2 = hypot(ix2 - curr[ax1], iy2 - curr[ax2]);
        double ix = (d1 < d2) ? ix1 : ix2;
        double iy = (d1 < d2) ? iy1 : iy2;

        // 验证交点在 segA 偏置弧 sweep 范围内
        double abs_angle_A = atan2(iy - center_off_A[1], ix - center_off_A[0]);
        double delta_A = abs_angle_A - start_angle_A;
        while(delta_A >  M_PI) delta_A -= 2.0 * M_PI;
        while(delta_A < -M_PI) delta_A += 2.0 * M_PI;
        double s_end_A;
        if(sweep_A >= 0.0) {
            while(delta_A < -COMP_PARALLEL_TOL) delta_A += 2.0 * M_PI;
            s_end_A = delta_A;
            if(s_end_A > sweep_A + COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→ARC 内角交点越 segA 弧段: "
                       "s=%.4f > sweep=%.4f\n", s_end_A, sweep_A);
                return -1;
            }
        } else {
            while(delta_A > COMP_PARALLEL_TOL) delta_A -= 2.0 * M_PI;
            s_end_A = delta_A;
            if(s_end_A < sweep_A - COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→ARC 内角交点越 segA 弧段: "
                       "s=%.4f < sweep=%.4f\n", s_end_A, sweep_A);
                return -1;
            }
        }

        // 验证交点在 segB 偏置弧 sweep 范围内
        double abs_angle_B = atan2(iy - center_off_B[1], ix - center_off_B[0]);
        double start_angle_B = atan2(rad_B_sy, rad_B_sx);
        double delta_B = abs_angle_B - start_angle_B;
        while(delta_B >  M_PI) delta_B -= 2.0 * M_PI;
        while(delta_B < -M_PI) delta_B += 2.0 * M_PI;
        double sweep_B = segB->sweep;
        if(sweep_B >= 0.0) {
            while(delta_B < -COMP_PARALLEL_TOL) delta_B += 2.0 * M_PI;
            if(delta_B > sweep_B + COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→ARC 内角交点越 segB 弧段: "
                       "s=%.4f > sweep=%.4f\n", delta_B, sweep_B);
                return -1;
            }
        } else {
            while(delta_B > COMP_PARALLEL_TOL) delta_B -= 2.0 * M_PI;
            if(delta_B < sweep_B - COMP_PARALLEL_TOL) {
                printf("[CutterComp报警] ARC→ARC 内角交点越 segB 弧段: "
                       "s=%.4f < sweep=%.4f\n", delta_B, sweep_B);
                return -1;
            }
        }

        // 离散化 segA 偏置弧从理论起点到交点 (部分 sweep)
        discretize_offset_arc(segA, R_off_A, start_angle_A, s_end_A,
                              ix, iy, ax1, ax2, speed, acc, dec);

    } else {
        // ============================================================
        // 外拐角: 两偏置弧断口, 离散化 segA 全 sweep + 过渡弧
        // 1) 离散化 segA 偏置弧全 sweep 到 end_off_A (= footA)
        // 2) 过渡弧: 圆心 = curr, 半径 = R, footA → footB (= segB 偏置弧理论起点)
        // ============================================================
        discretize_offset_arc(segA, R_off_A, start_angle_A, sweep_A,
                              end_off_A_x, end_off_A_y, ax1, ax2,
                              speed, acc, dec);

        // segB 偏置弧理论起点 (= footB)
        double inv_R_B = 1.0 / segB->radius;
        double start_off_B_x = segB->center[0] + R_off_B * rad_B_sx * inv_R_B;
        double start_off_B_y = segB->center[1] + R_off_B * rad_B_sy * inv_R_B;

        // 过渡弧 footA (= end_off_A) → footB (= start_off_B)
        double start_angle_t = atan2(end_off_A_y - curr[ax2], end_off_A_x - curr[ax1]);
        double end_angle_t   = atan2(start_off_B_y - curr[ax2], start_off_B_x - curr[ax1]);
        double sweep_t = end_angle_t - start_angle_t;
        while(sweep_t >  M_PI) sweep_t -= 2.0 * M_PI;
        while(sweep_t < -M_PI) sweep_t += 2.0 * M_PI;

        double arc_len_t = fabs(sweep_t) * R;
        int num_seg = (int)ceil(arc_len_t / COMP_ARC_STEP_MM);
        if(num_seg < 1) num_seg = 1;
        double angle_step = sweep_t / (double)num_seg;

        // C3-fix (2026-07-28): 过渡弧非编程几何, 不占 G93 预算 (dt=0)
        for(int i = 1; i <= num_seg; i++) {
            double pt[AXIS_NUM];
            memcpy(pt, curr, sizeof(double) * AXIS_NUM);
            if(i == num_seg) {
                pt[ax1] = start_off_B_x;
                pt[ax2] = start_off_B_y;
            } else {
                double theta = start_angle_t + (double)i * angle_step;
                pt[ax1] = curr[ax1] + R * cos(theta);
                pt[ax2] = curr[ax2] + R * sin(theta);
            }
            cutter_emit_dt(pt, speed, acc, dec, 0, 0.0);
        }
    }

    return 0;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// 统一混合拐角分发器 (方案 B 完整覆盖 LINE/ARC 全组合)
static int process_corner_mixed(const CompSegment_t *segA,
                                 const CompSegment_t *segB)
{
    // LINE-ARC 已实现 (方案 B 第 1 个里程碑)
    if(segA->type == COMP_SEG_LINE && segB->type == COMP_SEG_ARC) {
        return process_corner_line_arc(segA, segB);
    }
    // ARC-LINE 已实现 (方案 B 第 5 个里程碑)
    if(segA->type == COMP_SEG_ARC && segB->type == COMP_SEG_LINE) {
        return process_corner_arc_line(segA, segB);
    }
    // ARC-ARC 已实现 (方案 B 第 6 个里程碑, 最终)
    if(segA->type == COMP_SEG_ARC && segB->type == COMP_SEG_ARC) {
        return process_corner_arc_arc(segA, segB);
    }

    // 不应到达 (LINE-LINE 走 process_corner_line_line, 不进 mixed)
    printf("[CutterComp报警] 未知拐角组合: %d→%d\n", segA->type, segB->type);
    return -1;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// 统一拐角分发器: 按 (segA.type, segB.type) 选择处理函数
// 返回 0=成功, -1=过切/燕尾
static int process_segment_pair(const CompSegment_t *segA,
                                 const CompSegment_t *segB)
{
    if(segA->type == COMP_SEG_LINE && segB->type == COMP_SEG_LINE) {
        return process_corner_line_line(segA, segB);
    }
    // LINE-ARC / ARC-LINE / ARC-ARC: 阶段 3 填充
    return process_corner_mixed(segA, segB);
}

// ================== 末段刷新辅助 ==================

// @Context: Non-RealTime Background Thread
// @Safe: 纯数学 + 调用回调。
// 刷出窗口中最后一段的偏置终点，并输出回退点 (返回编程轨迹)。
// 由 CutterComp_Disable 内部调用。
static void flush_last_segment(void)
{
    if(g_comp.window_count < 2) return;

    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    // 阶段 2 重构: 从 CompSegment_t 提取 prev/curr/speed/acc/dec
    // ARC 末段刷出 (方案 B): 全 sweep 离散化偏置弧 + 退刀点
    // LINE 末段刷出: 原有逻辑保持不变
    CompSegment_t *seg_prev = &g_comp.window_seg[0];
    CompSegment_t *seg_curr = &g_comp.window_seg[1];
    double speed = seg_curr->speed, acc = seg_curr->acc, dec = seg_curr->dec;
    double R = g_comp.radius;
    int mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;

    if(seg_curr->type == COMP_SEG_ARC) {
        // ============================================================
        // ARC 末段刷出 (方案 B 第 2 个里程碑)
        //
        // 触发场景: G40 时 window[1] 仍是 ARC (上段 process_segment_pair
        //   已输出 seg_prev 偏置 + 拐角过渡, ARC 偏置弧的"起点"已对齐)
        //   flush 从 ARC 偏置弧理论起点离散化全 sweep 到末点 + 退刀点。
        //
        // 离散化:
        //   - 平面两轴: center + R_off * (cos θ, sin θ)
        //   - 非平面轴: 线性跟随 (支持螺旋 G02/G03 X Y Z I J K)
        //   - 末点强制对齐理论偏置终点 (消除浮点累积)
        //
        // 过切检测:
        //   - radius < tol       → 报警 (无效圆弧)
        //   - offset_arc R' ≤ 0  → 报警 (内偏过切, 与 PushArc 一致)
        // ============================================================
        if(fabs(seg_curr->radius) < COMP_ZERO_LEN_TOL) {
            printf("[CutterComp报警] flush ARC 末段: 无效半径 %.6f\n", seg_curr->radius);
            return;
        }
        double center_off[2], R_off;
        if(offset_arc(seg_curr->center, seg_curr->radius, seg_curr->is_CW,
                       g_comp.mode, R, center_off, &R_off) < 0) {
            printf("[CutterComp报警] flush ARC 末段过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
                   seg_curr->radius, R);
            return;
        }

        // 偏置弧理论起点 (径向缩放到 R_off, 同心性质保证)
        double rad_sx = seg_curr->start_pos[ax1] - seg_curr->center[0];
        double rad_sy = seg_curr->start_pos[ax2] - seg_curr->center[1];
        double inv_R_orig = 1.0 / seg_curr->radius;
        double rad_ex = seg_curr->end_pos[ax1] - seg_curr->center[0];
        double rad_ey = seg_curr->end_pos[ax2] - seg_curr->center[1];

        // 首段防御 (PushArc 入口应已硬报警拦截 G41+G02 起刀, 这里双保险)
        if(g_comp.first_seg_pending) {
            cutter_emit(seg_curr->start_pos, speed, acc, dec);
            double start_off[AXIS_NUM];
            memcpy(start_off, seg_curr->start_pos, sizeof(double) * AXIS_NUM);
            start_off[ax1] = seg_curr->center[0] + R_off * rad_sx * inv_R_orig;
            start_off[ax2] = seg_curr->center[1] + R_off * rad_sy * inv_R_orig;
            cutter_emit(start_off, speed, acc, dec);
            g_comp.first_seg_pending = 0;
        }

        // 离散化偏置弧 (全 sweep)
        // C3-fix / C3-fix2 (2026-07-28): 复用 discretize_offset_arc ——
        //   1) G93 时间均分用段自身 g93_dt_sec (G40 flush 时全局 pending 已不可信)
        //   2) 入口对齐消除回程过切 (LINE→ARC 内角剪裁后 G40 直接 flush 场景)
        double start_angle = atan2(rad_sy, rad_sx);
        double end_off_ax1 = seg_curr->center[0] + R_off * rad_ex * inv_R_orig;
        double end_off_ax2 = seg_curr->center[1] + R_off * rad_ey * inv_R_orig;
        discretize_offset_arc(seg_curr, R_off, start_angle, seg_curr->sweep,
                              end_off_ax1, end_off_ax2, ax1, ax2,
                              speed, acc, dec);

        // 退刀点: 从偏置终点垂直回到编程 end_pos, 距离恰好为 R 或 (R-r 内偏时)
        // 确保后续无补偿指令从编程位置开始
        double end_off[AXIS_NUM];
        memcpy(end_off, seg_curr->end_pos, sizeof(double) * AXIS_NUM);
        end_off[ax1] = seg_curr->center[0] + R_off * rad_ex * inv_R_orig;
        end_off[ax2] = seg_curr->center[1] + R_off * rad_ey * inv_R_orig;
        double ret_dx = seg_curr->end_pos[ax1] - end_off[ax1];
        double ret_dy = seg_curr->end_pos[ax2] - end_off[ax2];
        if(hypot(ret_dx, ret_dy) > COMP_ZERO_LEN_TOL) {
            cutter_emit(seg_curr->end_pos, speed, acc, dec);
        }
        return;
    }

    const double *prev = seg_prev->end_pos;   // 上段的终点 = 末段的起点
    const double *curr = seg_curr->end_pos;

    // 末段方向向量
    double dx = curr[ax1] - prev[ax1];
    double dy = curr[ax2] - prev[ax2];
    double len = hypot(dx, dy);
    if(len < COMP_ZERO_LEN_TOL) return;

    // 末段偏置法向量
    double nx, ny;
    compute_left_normal(dx, dy, &nx, &ny);
    nx *= mode_sign;
    ny *= mode_sign;

    // 如果首段尚未输出 (只推了 2 个点就 Disable)，输出过渡 + 偏置起点
    if(g_comp.first_seg_pending) {
        // 先输出编程轨迹点 (确保从上一个未补偿位置平滑过渡)
        cutter_emit(prev, speed, acc, dec);
        // 垂直切入偏置线
        double start[AXIS_NUM];
        memcpy(start, prev, sizeof(double) * AXIS_NUM);
        start[ax1] += R * nx;
        start[ax2] += R * ny;
        cutter_emit(start, speed, acc, dec);
        g_comp.first_seg_pending = 0;
    }

    // 输出末段偏置终点
    double end[AXIS_NUM];
    memcpy(end, curr, sizeof(double) * AXIS_NUM);
    end[ax1] += R * nx;
    end[ax2] += R * ny;
    cutter_emit(end, speed, acc, dec);

    // 回退点: 从偏置终点垂直返回到编程轨迹
    // 距离恰好为 R，确保后续无补偿指令从正确的编程位置开始。
    double ret[AXIS_NUM];
    memcpy(ret, curr, sizeof(double) * AXIS_NUM);
    // 检查偏置终点与编程位置的距离是否值得输出
    double ret_dx = ret[ax1] - end[ax1];
    double ret_dy = ret[ax2] - end[ax2];
    if(hypot(ret_dx, ret_dy) > COMP_ZERO_LEN_TOL) {
        cutter_emit(ret, speed, acc, dec);
    }
}

// ================== 公共 API 实现 ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
// @Safe: 纯内存清零。
void CutterComp_Init(void)
{
    memset(&g_comp, 0, sizeof(g_comp));
    g_comp.mode = COMP_OFF;
    g_comp.radius = 0.0;
    g_comp.active_plane = 17;
    g_comp.window_count = 0;
    g_comp.first_seg_pending = 0;
    g_comp.output_fn = NULL;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 仅修改内部状态，无 I/O。
void CutterComp_Enable(int mode, double radius)
{
    if(mode != COMP_LEFT && mode != COMP_RIGHT) {
        printf("[CutterComp] 无效的补偿模式: %d\n", mode);
        return;
    }
    if(radius < COMP_ZERO_LEN_TOL) {
        printf("[CutterComp] 刀具半径无效: %.6f mm\n", radius);
        return;
    }

    g_comp.mode = mode;
    g_comp.radius = radius;
    g_comp.active_plane = g_state.active_plane;

    // 记录当前机器坐标作为 Start-up Block 起点 P0
    // 将 g_state.current_pos (逻辑坐标) + 工件偏置 → 机械绝对坐标
    // 读 parser 模态 g_state.modal_wcs (与 gcode_parser.c 偏置查询同源),
    // 不读 g_coord_mgr.current_coord (RT 线程滞后字段,会撕裂刀补起点)。
    //
    // 阶段 2: P0 作为零长 LINE 段填入 window_seg[0] (start_pos=end_pos=P0)
    //         后续 PushPoint 推入 P1 时, 自动构造 (P0→P1) LINE 段进窗口
    {
        int wcs_idx = (g_state.modal_wcs >= COORD_G54 &&
                       g_state.modal_wcs <= COORD_G59)
                      ? (g_state.modal_wcs - 1) : -1;
        CompSegment_t *p0 = &g_comp.window_seg[0];
        memset(p0, 0, sizeof(*p0));
        p0->type = COMP_SEG_LINE;
        p0->speed = 0.0;
        p0->acc = DEFAULT_ACC;
        p0->dec = DEFAULT_DEC;
        for(int i = 0; i < AXIS_NUM; i++){
            // P2': 与 snapshot_wcs_offset 同步, G54-G59 路径叠加 G52 local_offset
            // P5': 优先用 ext WCS (G54.1 Pn) 偏置, 否则 regular WCS
            double w;
            if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][i];
            } else {
                w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
            }
            if(g_state.local_offset_active && wcs_idx >= 0) w += g_state.local_offset[i];
            double p = g_state.current_pos[i] + w;
            p0->start_pos[i] = p;
            p0->end_pos[i]   = p;   // 零长 LINE: start == end
        }
    }
    g_comp.window_count = 1;  // P0 已就位，下一条 PushPoint(P1) 将填入 window_seg[1]
    g_comp.first_seg_pending = 1;
    g_comp.last_emit_valid = 0;  // C3-fix2: 清陈旧刀位, 防误触发弧入口对齐

    printf("[CutterComp] %s 已启用, 半径=%.3f mm, 平面=G%d\n",
           mode == COMP_LEFT ? "左补偿(G41)" : "右补偿(G42)",
           radius, g_comp.active_plane);
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 调用回调函数。
void CutterComp_Disable(void)
{
    if(g_comp.mode == COMP_OFF) return;

    // 刷出缓冲区中最后一段
    if(g_comp.output_fn) {
        flush_last_segment();
    }

    printf("[CutterComp] 补偿已关闭 (G40), 刷出 %d 个缓冲点\n",
           g_comp.window_count);

    g_comp.mode = COMP_OFF;
    g_comp.radius = 0.0;
    g_comp.window_count = 0;
    g_comp.first_seg_pending = 0;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 仅存储函数指针。
void CutterComp_SetOutput(CutterOutputCB fn)
{
    g_comp.output_fn = fn;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: 由 parser 线程独占调用。
//
// 阶段 2 重构: PushPoint 把单点包装为 LINE segment 后, 走统一的"段入队"路径
// 与阶段 3 的 PushArc 共用滑动窗口 + process_segment_pair 流程
//
// C3 (2026-07-27): 签名扩展 4→6 参数, 透传 G93 强一致性上下文.
//   入口缓存 G93 到 g_comp, cutter_emit / 直通路径 读取后透传到 output_fn.
int CutterComp_PushPoint(double pos[AXIS_NUM], double speed,
                         double acc, double dec,
                         int is_g93_strict, double g93_dt_sec)
{
    // C3: 缓存 G93 上下文 (cutter_emit / 后续 cutter_emit 调用读取)
    g_comp.g93_strict_pending = is_g93_strict;
    g_comp.g93_dt_sec_pending = g93_dt_sec;

    // 补偿关闭: 直通到输出回调
    if(g_comp.mode == COMP_OFF) {
        if(g_comp.output_fn) {
            return g_comp.output_fn(pos, speed, acc, dec,
                                     is_g93_strict, g93_dt_sec);
        }
        return -1;
    }

    if(!g_comp.output_fn) return -1;

    // 同步激活平面 (可能中途被 G17/G18/G19 切换)
    g_comp.active_plane = g_state.active_plane;

    // 把单点 pos 包装为 LINE segment (start = window_seg[0].end, end = pos)
    // window_seg[0].end 在 Enable 时设为 P0, 第一次 PushPoint 后会自动接续
    CompSegment_t new_seg;
    memset(&new_seg, 0, sizeof(new_seg));
    new_seg.type = COMP_SEG_LINE;
    memcpy(new_seg.start_pos, g_comp.window_seg[0].end_pos,
           sizeof(double) * AXIS_NUM);
    memcpy(new_seg.end_pos, pos, sizeof(double) * AXIS_NUM);
    new_seg.speed = speed;
    new_seg.acc = acc;
    new_seg.dec = dec;

    // 第一次推入 (window_count==1): 仅填窗口, 不触发拐角处理
    // (因为没有 segA 可配对)
    if(g_comp.window_count < 2) {
        g_comp.window_seg[1] = new_seg;
        g_comp.window_count = 2;
        return 0;
    }

    // window_count == 2: window_seg[1] 是 segA (上段), new_seg 是 segB (本段)
    // 触发拐角判定与偏置轨迹重构
    if(process_segment_pair(&g_comp.window_seg[1], &new_seg) < 0) {
        printf("[CutterComp] 燕尾过切，中止当前文件！\n");
        return -1;
    }

    // 滑动窗口: window_seg[0] = window_seg[1], window_seg[1] = new_seg
    g_comp.window_seg[0] = g_comp.window_seg[1];
    g_comp.window_seg[1] = new_seg;

    return 0;
}

// @Context: Non-RealTime Background Thread
int CutterComp_GetMode(void)
{
    return g_comp.mode;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: 由 parser 线程独占调用。
//
// 圆弧刀补主入口 (PushArc):
//   - COMP_ARC_STRATEGY=A (回滚路径): 圆弧离散化为微段, 复用 LINE-LINE 拐角
//   - COMP_ARC_STRATEGY=B (默认):     整弧入窗口, process_segment_pair 触发精确拐角
//
// 共同前置 (无论 A/B):
//   1. 起刀段硬报警: G41/G42 后第一段不能为圆弧 (Fanuc 标准要求 G00/G01 法向切入)
//   2. 早失败: offset_arc 检查偏置后半径, 内偏 R<r 立即报警, 不浪费窗口推进
//
// 方案 B 路径 (默认):
//   - 整弧 CompSegment_t 入 window_seg[1]
//   - 与 window_seg[1] 旧值(作为 segA) 触发 process_segment_pair → process_corner_line_arc
//   - 滑动窗口: window[0] = 旧 window[1], window[1] = arc
//   - ARC 本体离散化推迟到 (a) 下一段 ARC→X 拐角处理 或 (b) G40 时 flush_last_segment
#if COMP_ARC_STRATEGY == COMP_ARC_STRATEGY_A
// --------------------------------------------------------------------
// 方案 A 路径: 圆弧离散化为 LINE 微段, 每个微段调 PushPoint
// (保留作紧急回滚, 默认编译时不参与构建)
// --------------------------------------------------------------------
static int push_arc_strategy_a_discretize(const CompSegment_t *arc,
                                           double R_off, int ax1, int ax2)
{
    double arc_len = fabs(arc->sweep) * R_off;
    int num_seg = (int)ceil(arc_len / COMP_ARC_STEP_MM);
    if(num_seg < 1) num_seg = 1;
    double angle_step = arc->sweep / (double)num_seg;

    double start_angle = atan2(arc->start_pos[ax2] - arc->center[1],
                                arc->start_pos[ax1] - arc->center[0]);

    for(int i = 1; i <= num_seg; i++) {
        double pt[AXIS_NUM];
        double ratio = (double)i / (double)num_seg;

        for(int j = 0; j < AXIS_NUM; j++) {
            if(j != ax1 && j != ax2) {
                pt[j] = arc->start_pos[j]
                      + (arc->end_pos[j] - arc->start_pos[j]) * ratio;
            }
        }

        double theta;
        if(i == num_seg) theta = start_angle + arc->sweep;
        else             theta = start_angle + (double)i * angle_step;
        pt[ax1] = arc->center[0] + R_off * cos(theta);
        pt[ax2] = arc->center[1] + R_off * sin(theta);

        // C3 (2026-07-27): 圆弧 G93 时间均分到每个微段 (seg_T = T_total / num_seg)
        // 浮点累积风险: 用 i/num_seg 比例算累计时间, 避免均除累积误差
        // seg_T_at_i = T_total × i / num_seg - T_total × (i-1) / num_seg
        //            = T_total / num_seg (理论上), 但浮点累减法更稳定
        double total_T = arc->g93_dt_sec;
        double seg_T = (total_T > 1e-9)
                       ? (total_T * (double)i / (double)num_seg
                          - total_T * (double)(i - 1) / (double)num_seg)
                       : 0.0;
        int is_g93_seg = (seg_T > 1e-9) ? 1 : 0;
        if(CutterComp_PushPoint(pt, arc->speed, arc->acc, arc->dec,
                                  is_g93_seg, seg_T) < 0) return -1;
    }
    return 0;
}
#endif

int CutterComp_PushArc(const CompSegment_t *arc)
{
    // C3 (2026-07-27): 缓存 G93 上下文 (cutter_emit / 直通路径读取)
    int is_g93_arc = (arc->g93_dt_sec > 1e-9) ? 1 : 0;
    g_comp.g93_strict_pending = is_g93_arc;
    g_comp.g93_dt_sec_pending = arc->g93_dt_sec;

    // 补偿关闭: 直通终点 (与 PushPoint 同语义)
    if(g_comp.mode == COMP_OFF) {
        if(g_comp.output_fn) {
            return g_comp.output_fn(arc->end_pos, arc->speed, arc->acc, arc->dec,
                                     is_g93_arc, arc->g93_dt_sec);
        }
        return -1;
    }
    if(!g_comp.output_fn) return -1;

    // 同步激活平面 (G17/G18/G19) + 算偏置半径 (早失败)
    g_comp.active_plane = g_state.active_plane;
    int ax1, ax2;
    get_plane_axes(g_comp.active_plane, &ax1, &ax2);

    double center_off[2];
    double R_off;
    if(offset_arc(arc->center, arc->radius, arc->is_CW,
                   g_comp.mode, g_comp.radius,
                   center_off, &R_off) < 0) {
        printf("[CutterComp报警] 圆弧过切: R=%.3f 内偏后 ≤ 0 (r=%.3f)\n",
               arc->radius, g_comp.radius);
        return -1;
    }

#if COMP_ARC_STRATEGY == COMP_ARC_STRATEGY_A
    // 方案 A: 离散化为微段 (回滚路径, 通过 PushPoint 复用 LINE-LINE 拐角)
    // 方案 A 不需要起刀段硬报警: 第一个微段作为 LINE 进入窗口, 走正常 LINE 起刀流程
    return push_arc_strategy_a_discretize(arc, R_off, ax1, ax2);
#else
    // ============================================================
    // 方案 B: 整弧入窗口 (默认路径)
    // ============================================================

    // ---- 起刀段硬报警 (Fanuc 标准: 起刀必须 LINE 法向切入) ----
    // 判定: window_count < 2 表示 Enable 后尚未推入任何运动段, 本 ARC 是首段
    // (注: 不能用 first_seg_pending 判定, 它在第一次 PushPoint 后仍为 1,
    //  直到第二次 PushPoint 触发 process_corner 才清除)
    if(g_comp.window_count < 2) {
        printf("[CutterComp报警] G41/G42 起刀段不能为圆弧 (需 G00/G01 法向切入)\n");
        return -1;
    }

    // window_count == 2: window_seg[1] 是 segA (上段), arc 是 segB (本段)
    // 触发拐角判定 (LINE-ARC 已实现; ARC-LINE/ARC-ARC 仍为 STUB)
    if(process_segment_pair(&g_comp.window_seg[1], arc) < 0) {
        printf("[CutterComp报警] ARC 入队时拐角处理失败 (过切或几何退化)\n");
        return -1;
    }

    // 滑动窗口: window[0] = 旧 window[1], window[1] = arc
    g_comp.window_seg[0] = g_comp.window_seg[1];
    g_comp.window_seg[1] = *arc;
    return 0;
#endif
}

// @Context: Non-RealTime Background Thread
double CutterComp_GetRadius(void)
{
    return g_comp.radius;
}
