#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "soem/soem.h"
#include "osal_compat.h"
#include "axis_cfg.h"
#include "cutter_comp.h"

#define RAPID_SPEED_MM_MIN 3000.0 // G00 快速移动速度（mm/min），可根据需要调整
#define DEFAULT_ACC  50.0   // 默认加速度（mm/s^2）
#define DEFAULT_DEC  50.0   // 默认减速度（mm/s^2）
#define DEFAULT_JERK 5000.0  // 默认加加速度（mm/s^3）

#define FEED_MODE_G93 93  // 倒数时间进给：F = 1/T_min
#define FEED_MODE_G94 94  // 每分钟进给：F = mm/min

typedef struct {
    double current_pos[AXIS_NUM];

    double feedrate_mm_min; // 当前进给速度（mm/min）
    int is_absolute;       // 是否为绝对坐标模式（G90）1=G90，0=G91
    int active_plane;      // 当前工作平面: 17=XY(默认), 18=ZX, 19=YZ
    int motion_mode;       // 运动组模态状态: 0=G00, 1=G01, 2=G02, 3=G03
    int feed_mode;         // 进给模式: FEED_MODE_G93 或 FEED_MODE_G94
    int rtcp_enabled;      // G43.4 RTCP刀尖跟随: 0=关闭, 1=开启
    int bspline_enabled;   // B-Spline平滑模式: 0=关闭(M50 P0), 1=开启(M50 P1)
    int pending_comp_mode; // 待激活的刀补模式: COMP_OFF/G41/G42, 配合 D 代码使用

    // ---- Phase 2A.2: 固定循环模态字段 ----
    int    active_cycle;        // 0=无/G80, 81=钻孔, 82=带暂停, 83=深孔啄钻
    double cycle_R_plane;       // R 平面 (modal, 通常为安全高度)
    double cycle_Z_bottom;      // 孔底 Z (modal)
    double cycle_dwell_ms;      // G82 孔底暂停 (P, modal) [注: Phase 2A.2 简化版暂未实现]
    double cycle_peck_depth;    // G83 啄钻步进 (Q, modal, 必须 > 0)
    int    cycle_retract_mode;  // 98=退到初始 Z (G98), 99=退到 R 平面 (G99); 0=未指定默认 G98
    double cycle_initial_Z;     // 循环激活时捕获的 Z (G98 退回点)
} GCodeState_t;

typedef struct{
    char filepath[256];
    int is_running;
    int is_paused;
    int abort_request;
}ParserControl_t;

// Function declarations for G-code parsing
int parse_gcode_line(const char *gcode_line);
const char* skip_spaces(const char* str);
OSAL_THREAD_FUNC parser_thread_func(void *arg);
int  generate_arc_trajectory(double start_pos[AXIS_NUM],double end_pos[AXIS_NUM],
                             double offset_1st, double offset_2nd,
                             int is_CW,double feedrate_mm_min,double g93_T_sec);
int  generate_linear_rtcp_trajectory(double start_pos[AXIS_NUM],double end_pos[AXIS_NUM],
                                     double run_speed_mm,double g93_T_sec);
// Phase 2A.2: 固定循环展开 (G81/G82/G83) — 在 parse_gcode_line 中调用
int  generate_fixed_cycle(double target_pos[AXIS_NUM],
                          double start_pos[AXIS_NUM],
                          double feedrate_mm_min);
#endif // GCODE_PARSER_H