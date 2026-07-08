#ifndef CUTTER_COMP_H
#define CUTTER_COMP_H

#include "axis_cfg.h"

// ================== 2D 刀具半径补偿引擎 ==================
// 插入于 gcode_parser 与 B-Spline/Planner 之间的计算几何层。
// 支持 G41(左补偿) / G42(右补偿) / G40(取消补偿)。
//
// 当前覆盖 (阶段 1-3 完成):
//   - G01/G00 直线偏置: LINE-LINE 拐角 (内角求交/外角插弧), 已稳定
//   - G02/G03 圆弧偏置: 方案 A "圆弧离散为偏置后 LINE 微段, 复用 LINE-LINE 拐角处理"
//     * offset_arc 算偏置半径 (外偏 R+r / 内偏 R-r, 过切检测 R<r 报警)
//     * 按 COMP_ARC_STEP_MM 离散化偏置后圆弧
//     * 每个微段调 CutterComp_PushPoint 走 LINE-LINE 拐角
//     * 支持螺旋 G02/G03 X Y Z I J K (非平面轴线性插值)
//   - 已知限制 (P1+ 优化项):
//     * ARC↔LINE 拐角的"反向相交"可能触发 LINE-LINE 燕尾检测 (微段级)
//     * 圆弧 + 直线锐角的精细过渡可能有小抖动 (0.5mm 步长导致)
//     * 方案 B (独立 LINE-ARC/ARC-LINE/ARC-ARC 几何) 留作未来优化
//
// 引擎在激活平面的两个轴上做 2D 偏置，非平面轴线性跟随。
// 仅支持 G17 (XY) 平面; G18 (ZX) / G19 (YZ) 理论上可工作但未实测。

// ---- 补偿模式枚举 ----
typedef enum {
    COMP_OFF   = 0,  // G40: 无补偿
    COMP_LEFT  = 1,  // G41: 左补偿 (刀具偏置在运动方向左侧)
    COMP_RIGHT = 2   // G42: 右补偿 (刀具偏置在运动方向右侧)
} CompMode;

// ---- 段类型枚举 (阶段 1: 数据结构定义, 阶段 2 起被引擎使用) ----
typedef enum {
    COMP_SEG_LINE = 0,   // G00/G01 直线
    COMP_SEG_ARC  = 1,   // G02/G03 圆弧
} CompSegType_t;

// ---- 单段几何 (统一表达直线和圆弧, 阶段 2 起替换 window[2][AXIS_NUM]) ----
// ARC 专属字段仅在 type == COMP_SEG_ARC 时有效
// center/radius/sweep/is_CW 均在激活平面 (G17=XY/G18=ZX/G19=YZ) 的两主轴上
typedef struct {
    CompSegType_t type;
    double start_pos[AXIS_NUM];   // 起点 (机械绝对坐标)
    double end_pos[AXIS_NUM];     // 终点 (机械绝对坐标)
    double speed, acc, dec;       // 运动参数 (与 api_push_trajectory 同单位)
    // ARC 专属
    double center[2];             // 圆心在激活平面两主轴上的坐标 (ax1, ax2)
    double radius;                // 圆弧半径 (mm)
    double sweep;                 // 扫角 (rad, 正=CCW G03, 负=CW G02)
    int    is_CW;                 // 1=G02 CW, 0=G03 CCW
} CompSegment_t;

// ---- 几何容差与圆弧离散化参数 ----
#define COMP_PARALLEL_TOL    1e-9   // 共线/平行判定容差
#define COMP_TANGENT_TOL     1e-6   // 相切判定相对容差 (|dist - R| / (R+1))
#define COMP_ARC_STEP_MM     0.5    // 外拐角过渡圆弧微段步长 (mm)
#define COMP_ZERO_LEN_TOL    1e-6   // 零长段判定容差 (mm)

// ---- 圆弧刀补策略选择 (编译开关) ----
// COMP_ARC_STRATEGY_A: 方案 A (圆弧离散化为微段, 复用 LINE-LINE 拐角) - 稳定路径, 留作回滚
// COMP_ARC_STRATEGY_B: 方案 B (独立 LINE/ARC 几何, 精确拐角) - 默认启用, 当前实现
// 紧急回滚: 把下面的默认值改为 COMP_ARC_STRATEGY_A
#define COMP_ARC_STRATEGY_A  0
#define COMP_ARC_STRATEGY_B  1
#ifndef COMP_ARC_STRATEGY
#define COMP_ARC_STRATEGY    COMP_ARC_STRATEGY_B
#endif

// ---- 输出回调函数类型 ----
// 补偿引擎将偏置后的点通过此回调下发到 B-Spline 或 Planner。
// 参数与 api_push_trajectory 保持一致，便于直接替换。
typedef int (*CutterOutputCB)(double pos[AXIS_NUM], double speed,
                              double acc, double dec);

// ---- 补偿引擎状态结构体 ----
typedef struct {
    CompMode mode;               // 当前补偿模式
    double   radius;             // 刀具半径 (mm)
    int      active_plane;       // 当前工作平面: 17=XY, 18=ZX, 19=YZ

    // 两段滑动窗口: window_seg[0]=prev_seg, window_seg[1]=curr_seg
    // 新推入的段作为 next_seg, 与 window_seg[1] 构成"curr→next"拐角判定
    // (阶段 2 重构: 从 double[2][AXIS_NUM] 改为 CompSegment_t[2], 支持线段/圆弧统一表达)
    CompSegment_t window_seg[2];
    int      window_count;       // 当前窗口中的段数 (0~2)

    int      first_seg_pending;  // 首段标志: 尚未输出偏置起始点
    CutterOutputCB output_fn;    // 输出回调函数
} CutterCompState_t;

// ================== 公共 API ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
// @Safe: 纯内存清零。
// 初始化补偿引擎状态。必须在 parser 线程启动前调用。
void CutterComp_Init(void);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 仅修改内部状态，无 I/O。
// 启用刀具半径补偿 (处理 G41/G42)。
// mode:   COMP_LEFT (G41) 或 COMP_RIGHT (G42)
// radius: 刀具半径 (mm)，必须 > 0
void CutterComp_Enable(int mode, double radius);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 调用回调函数 (api_push_trajectory)。
// 取消刀具半径补偿 (处理 G40)。
// 刷出缓冲区中的残余段，输出末段偏置终点和回退点，然后关闭补偿。
void CutterComp_Disable(void);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 仅存储函数指针。
// 设置输出回调函数。补偿引擎通过此回调下发偏置后的运动点。
// 典型用法: CutterComp_SetOutput(api_push_trajectory);
void CutterComp_SetOutput(CutterOutputCB fn);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: 由 parser 线程独占调用，天然线程安全。
// 推入一个运动点，引擎内部缓存并进行 2D 偏置计算。
// 当窗口满 (3点) 时自动触发拐角判定和偏置轨迹重构，
// 偏置后的点通过注册的回调函数下发到 B-Spline 或 Planner。
// pos:   机械绝对坐标 [AXIS_NUM]
// speed: 合成速度 (mm/s)
// acc:   加速度 (mm/s^2)
// dec:   减速度 (mm/s^2)
// 返回 0=成功, -1=输出回调失败
int  CutterComp_PushPoint(double pos[AXIS_NUM], double speed,
                          double acc, double dec);

// @Context: Non-RealTime Background Thread (parser 调用)
// @Thread-Safety: 由 parser 线程独占调用。
// 推入一个圆弧段 (G02/G03), 引擎内部按偏置规则重构后通过回调下发。
// arc->type 必须为 COMP_SEG_ARC; start_pos/end_pos/center/radius/sweep/is_CW 必须有效。
// 返回 0=成功, -1=过切或输出回调失败
// 注: 阶段 1 仅声明, 实现见阶段 3 (CutterComp_PushArc_internal)
int  CutterComp_PushArc(const CompSegment_t *arc);

// @Context: Non-RealTime Background Thread
// 查询当前补偿模式 (COMP_OFF / COMP_LEFT / COMP_RIGHT)
int  CutterComp_GetMode(void);

// @Context: Non-RealTime Background Thread
// 查询当前刀具半径 (mm)
double CutterComp_GetRadius(void);

#endif // CUTTER_COMP_H
