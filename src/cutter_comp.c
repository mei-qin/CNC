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
static int cutter_emit(const double pos[AXIS_NUM], double speed,
                       double acc, double dec)
{
    TraceLogger_PushPipeline(STAGE_CUTTER_COMP, pos, speed / 1000.0);
    return g_comp.output_fn(pos, speed, acc, dec);
}

// ================== 核心：拐角分类与轨迹重构 ==================

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学计算 + 调用回调函数。
//
// 处理一个拐角 (curr 是拐点):
//   prev → curr (段 A, 长度 lenA),  curr → next (段 B, 长度 lenB)
//
// 拓扑判定表 (2D 叉乘 cross = uA × uB):
//   cross > 0 → 路径左转 (CCW)
//   cross < 0 → 路径右转 (CW)
//
//   G41 (左补偿) + 左转 (cross > 0) → 内拐角: 刀具被挤向工件，偏置线收敛 → 求交点
//   G41 (左补偿) + 右转 (cross < 0) → 外拐角: 刀具被甩向外侧，偏置线断口 → 插圆弧
//   G42 (右补偿) + 左转 (cross > 0) → 外拐角: 刀具被甩向外侧，偏置线断口 → 插圆弧
//   G42 (右补偿) + 右转 (cross < 0) → 内拐角: 刀具被挤向工件，偏置线收敛 → 求交点
//
//   即: cross * mode_sign > 0 → 内拐角 (Inner), < 0 → 外拐角 (Outer)
//
// 返回 0=成功, -1=燕尾过切 (不可加工干涉，调用方应中止文件)

static int process_corner(double prev[AXIS_NUM], double curr[AXIS_NUM],
                          double next[AXIS_NUM],
                          double speed, double acc, double dec)
{
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
            cutter_emit(pt, speed, acc, dec);
        }
    }

    return 0;
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

    double *prev = g_comp.window[0];
    double *curr = g_comp.window[1];
    double speed = g_comp.window_speed[1];
    double acc   = g_comp.window_acc[1];
    double dec   = g_comp.window_dec[1];

    // 末段方向向量
    double dx = curr[ax1] - prev[ax1];
    double dy = curr[ax2] - prev[ax2];
    double len = hypot(dx, dy);
    if(len < COMP_ZERO_LEN_TOL) return;

    // 末段偏置法向量
    double nx, ny;
    compute_left_normal(dx, dy, &nx, &ny);
    int mode_sign = (g_comp.mode == COMP_LEFT) ? 1 : -1;
    nx *= mode_sign;
    ny *= mode_sign;

    double R = g_comp.radius;

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
    // 将 g_state.current_pos (逻辑坐标) + 工件偏移 → 机械绝对坐标
    {
        int wcs_idx = (g_coord_mgr.current_coord >= COORD_G54 &&
                       g_coord_mgr.current_coord <= COORD_G59)
                      ? (g_coord_mgr.current_coord - 1) : -1;
        for(int i = 0; i < AXIS_NUM; i++){
            double w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
            g_comp.window[0][i] = g_state.current_pos[i] + w;
        }
    }
    g_comp.window_speed[0] = 0.0;
    g_comp.window_acc[0]   = DEFAULT_ACC;
    g_comp.window_dec[0]   = DEFAULT_DEC;
    g_comp.window_count = 1;  // P0 已就位，下一条 PushPoint(P1) 将填入 window[1]
    g_comp.first_seg_pending = 1;

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
int CutterComp_PushPoint(double pos[AXIS_NUM], double speed,
                         double acc, double dec)
{
    // 补偿关闭: 直通到输出回调
    if(g_comp.mode == COMP_OFF) {
        if(g_comp.output_fn) {
            return g_comp.output_fn(pos, speed, acc, dec);
        }
        return -1;
    }

    if(!g_comp.output_fn) return -1;

    // 同步激活平面 (可能中途被 G17/G18/G19 切换)
    g_comp.active_plane = g_state.active_plane;

    // 填充滑动窗口
    if(g_comp.window_count < 2) {
        memcpy(g_comp.window[g_comp.window_count], pos,
               sizeof(double) * AXIS_NUM);
        g_comp.window_speed[g_comp.window_count] = speed;
        g_comp.window_acc[g_comp.window_count]   = acc;
        g_comp.window_dec[g_comp.window_count]   = dec;
        g_comp.window_count++;
        return 0;  // 需要至少 2 个点才能形成段，暂不输出
    }

    // window_count == 2: window[0]=prev, window[1]=curr, pos=next
    // 触发拐角判定与偏置轨迹重构
    if(process_corner(g_comp.window[0], g_comp.window[1], pos,
                      speed, acc, dec) < 0) {
        printf("[CutterComp] 燕尾过切，中止当前文件！\n");
        return -1;
    }

    // 滑动窗口: 丢弃 prev，prev=curr, curr=next
    memcpy(g_comp.window[0], g_comp.window[1],
           sizeof(double) * AXIS_NUM);
    g_comp.window_speed[0] = g_comp.window_speed[1];
    g_comp.window_acc[0]   = g_comp.window_acc[1];
    g_comp.window_dec[0]   = g_comp.window_dec[1];

    memcpy(g_comp.window[1], pos, sizeof(double) * AXIS_NUM);
    g_comp.window_speed[1] = speed;
    g_comp.window_acc[1]   = acc;
    g_comp.window_dec[1]   = dec;

    return 0;
}

// @Context: Non-RealTime Background Thread
int CutterComp_GetMode(void)
{
    return g_comp.mode;
}

// @Context: Non-RealTime Background Thread
double CutterComp_GetRadius(void)
{
    return g_comp.radius;
}
