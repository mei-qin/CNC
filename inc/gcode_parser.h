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
    CoordSystem_t modal_wcs; // 模态工件坐标系 (G53..G59)，仅 parser 线程读/写。
                            // 解析 G54~G59 时更新此字段; push 段时盖章到 seg.active_wcs;
                            // 计算 work_offsets 偏置查询时读此字段。
                            // 注意: 不再直接写 g_coord_mgr.current_coord — 那是 RT 线程独占字段,
                            // 由 RT 消费段时根据 seg.active_wcs 更新,保证 UI/宏与物理运动同步。
    int feed_mode;         // 进给模式: FEED_MODE_G93 或 FEED_MODE_G94
    int rtcp_enabled;      // G43.4 RTCP刀尖跟随: 0=关闭, 1=开启
    int bspline_enabled;   // B-Spline平滑模式: 0=关闭(M50 P0), 1=开启(M50 P1)
    int pending_comp_mode; // 待激活的刀补模式: COMP_OFF/G41/G42, 配合 D 代码使用

    // ---- Phase 2A.2: 固定循环模态字段 ----
    int    active_cycle;        // 0=无/G80, 73=高速啄钻(G73), 81=钻孔, 82=带暂停, 83=深孔啄钻
    double cycle_R_plane;       // R 平面 (modal, 通常为安全高度)
    double cycle_Z_bottom;      // 孔底 Z (modal)
    double cycle_dwell_ms;      // G82 孔底暂停 (P, modal) [注: Phase 2A.2 简化版暂未实现]
    double cycle_peck_depth;    // G83 啄钻步进 (Q, modal, 必须 > 0)
    int    cycle_retract_mode;  // 98=退到初始 Z (G98), 99=退到 R 平面 (G99); 0=未指定默认 G98
    double cycle_initial_Z;     // 循环激活时捕获的 Z (G98 退回点)

    // ---- P1': M 代码辅助状态机 modal 字段 ----
    // parser 端即时镜像; api_push_mcode 时快照到 TrajectorySegment_t,
    // RT 消费时拷到 g_interpolator._rt 字段。
    int    spindle_mode;        // 0=off(M5), 1=CW(M3), 2=CCW(M4)
    double spindle_rpm;         // S 代码最近值 (rpm, >0)
    int    coolant_state;       // bit flags: bit0=flood(M8), bit1=mist(M7);
                                // 0=off, 1=flood, 2=mist, 3=flood+mist; M9 清零
    int    current_tool_id;     // T 代码当前刀号 (M6 切换到此刀号)

    // ---- P0-Laser: 激光器模态字段 (parser 端) ----
    // M62/M63/M67/M68/M10/M11/M12 写入; M3/M5 联动 laser_shutter_pending;
    // 入队时快照到 seg.aux_laser_*; RT 消费时同步到 g_laser_rt.
    // 注意: 这些字段在 M30 应当与 spindle/coolant 一同复位 (Phase A 顺便修 P1 BUG).
    int    laser_shutter_pending;   // M62/M63 同步激光闸 (0/1)
    double laser_power_pending;     // M67 E<n> 激光功率 (W)
    double laser_freq_pending;      // M68 E<n> 激光频率 (Hz)
    int    gas_select;              // 0=off, 1=N2, 2=O2, 3=Air (M10/M11/M12)

    // ---- P2': G52 局部坐标系 ----
    // local_offset[i] 叠加在 work_offsets[modal_wcs] 之上 (Fanuc 标准):
    //   effective_offset[i] = work_offsets[modal_wcs][i] + local_offset[i]
    // local_offset_active=0 时 (G52 X0 Y0 Z0 或上电默认), 不参与叠加。
    // G53 块无视此字段 (机械坐标天然 bypass WCS+G52)。
    double local_offset[AXIS_NUM];
    int    local_offset_active;

    // ---- P4' Phase 2: G66/G67 模态宏调用 ----
    // G66 P<Onum> A-...Z- 激活: 此后每个运动段 (G00-G03/G73/G81-G83) 执行后,
    //   自动以存储的 args 调用一次 O<Onum> 宏 (类似 G65 单次调用)。
    // G67 取消模态宏调用。M30 / 程序结束自动复位 (parser_thread_func 重置 g_state)。
    // 注: G66 与 G65 共用 CallFrame 栈与 M99 返回路径, 每次触发都是独立单次调用 (L=1)。
    int    modal_macro_active;     // 0=未激活(G67/默认), 1=G66 已激活
    int    modal_macro_O_num;      // G66 P<Onum> 目标
    double modal_macro_args[27];   // 字母参数 (索引 1-26)
    int    modal_macro_args_set[27];

    // ---- P5': G54.1 P1-P48 扩展工件坐标系 ----
    // Fanuc 标准: G54.1 Pn 选择 48 组扩展 WCS 之一, 优先级高于 modal_wcs (G54-G59)
    // 0=未激活 (走 modal_wcs 路径), 1-48=对应 P1-P48
    // G54-G59 任意一切换会清零此字段; 反之 G54.1 Pn 激活时 modal_wcs 字段被忽略
    // 数据存储: g_coord_mgr.work_offsets_ext[48][AXIS_NUM], 由 #7001-#7948 写入
    int    modal_ext_wcs_p;

    // ---- Laser Phase B4: 段级工艺标记 modal (SEG_FLAG_* 位图) ----
    // M72/M73 包裹期间 OR SEG_FLAG_LEAD_IN; M74/M75 包裹期间 OR SEG_FLAG_MICRO_JOINT.
    // 入队时快照到 TrajectorySegment_t.seg_flags, UI 据此区分引线/微连接段.
    // M30/M2 程序结束自动清零 (跟其他 modal 字段一致), 防止跨程序泄漏.
    uint8_t laser_seg_flags;

    // ---- P2-A-4: G09/G61/G64 精准停模态 ----
    // modal_exact_stop: 0=G64 连续切削 (默认), 1=G61 精准停 (模态, 后续段都 v_end=0)
    // exact_stop_this_block: G09 一次性标志, 入队后清零.
    //   G-code 语义: G09 单次精准停本段, G61 模态精准停后续段, G64 取消模态精准停.
    //   Fanuc 行为: G09 单独一行无运动代码时, 标志延续到下一运动段 (符合工业惯例).
    // 入队时 (api_push_trajectory_impl): seg.is_exact_stop = modal_exact_stop || exact_stop_this_block;
    //                                    exact_stop_this_block = 0;  // 消费一次
    int modal_exact_stop;
    int exact_stop_this_block;
} GCodeState_t;

typedef struct{
    char filepath[256];
    int is_running;
    int is_paused;
    int abort_request;
    // ---- P0-b v2: LoadProgram / RunLoadedProgram 模式 ----
    // program_mode = PROGRAM_MODE_RUN (0): 正常执行, parser 入队 motion queue, RT 消费
    // program_mode = PROGRAM_MODE_PREVIEW (1): LoadProgram 模式, parser 解析填 PreviewStreamer,
    //                                          但 api_push_trajectory_impl 早返回不入 motion queue。
    //                                          RunLoadedProgram 时切回 RUN, 同 filepath 再跑一遍。
    // 跨线程: SMC_API 写, parser_thread 读。x86 int 写原子, 用 atomic_load(acquire) 防缓存。
    int program_mode;
}ParserControl_t;

#define PROGRAM_MODE_RUN     0   // SMC_RunGCodeFile / SMC_RunLoadedProgram 路径
#define PROGRAM_MODE_PREVIEW 1   // SMC_LoadProgram 路径

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

// M1 可选停开关: 默认 0=禁用 (M1 no-op), 1=M1 等价 M0 (HMI 通过 SMC_SetOptionalStopEnable 切换)
// 线程安全: 单写者 (SMC_SetOptionalStopEnable) + 单读者 (parser M1 分支), int 写天然原子
extern int g_optional_stop_enabled;

#endif // GCODE_PARSER_H