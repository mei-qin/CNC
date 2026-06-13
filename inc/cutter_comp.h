#ifndef CUTTER_COMP_H
#define CUTTER_COMP_H

#include "axis_cfg.h"

// ================== 2D 刀具半径补偿引擎 ==================
// 插入于 gcode_parser 与 B-Spline/Planner 之间的计算几何层。
// 支持 G41(左补偿) / G42(右补偿) / G40(取消补偿)。
// 当前版本仅处理纯直线段 (G01/G00) 偏置，不含 G02/G03 圆弧偏置与过切检测。
// 引擎在激活平面的两个轴上做 2D 偏置，非平面轴线性跟随。

// ---- 补偿模式枚举 ----
typedef enum {
    COMP_OFF   = 0,  // G40: 无补偿
    COMP_LEFT  = 1,  // G41: 左补偿 (刀具偏置在运动方向左侧)
    COMP_RIGHT = 2   // G42: 右补偿 (刀具偏置在运动方向右侧)
} CompMode;

// ---- 几何容差与圆弧离散化参数 ----
#define COMP_PARALLEL_TOL    1e-9   // 共线/平行判定容差
#define COMP_ARC_STEP_MM     0.5    // 外拐角过渡圆弧微段步长 (mm)
#define COMP_ZERO_LEN_TOL    1e-6   // 零长段判定容差 (mm)

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

    // 三点滑动窗口: window[0]=prev, window[1]=curr
    // 新推入的点作为 next，与前两点构成拐角判定窗口
    double   window[2][AXIS_NUM];
    double   window_speed[2];    // 每个窗口点对应的运动速度
    double   window_acc[2];      // 加速度
    double   window_dec[2];      // 减速度
    int      window_count;       // 当前窗口中的点数 (0~2)

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

// @Context: Non-RealTime Background Thread
// 查询当前补偿模式 (COMP_OFF / COMP_LEFT / COMP_RIGHT)
int  CutterComp_GetMode(void);

// @Context: Non-RealTime Background Thread
// 查询当前刀具半径 (mm)
double CutterComp_GetRadius(void);

#endif // CUTTER_COMP_H
