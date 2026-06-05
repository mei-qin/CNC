#ifndef GCODE_PARSER_H
#define GCODE_PARSER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "soem/soem.h"
#include "axis_cfg.h"

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
#endif // GCODE_PARSER_H