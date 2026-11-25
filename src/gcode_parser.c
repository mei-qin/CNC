#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "kinematics.h"
#include "bspline_engine.h"
#include "macro_eval.h"   // 宏变量与表达式引擎
#include "program_loader.h"  // Phase 2B M1: 文件加载器 + N 标签 + GOTO
#include "event_logger.h"    // P1-b: 参数校验报警 + 程序生命周期
#include "smc_protocol.h"    // P2-A: SMC_MODE_* 位定义 (M30 override 重置用)
#include "block_timing.h"    // 论文1 S10: block 级计时插桩 (bt_block_open/close/reset)
#include <math.h>
#include <stdlib.h>       // getenv/strtod/atoi (pose-inject 实验开关, Non-RT)
#include <stdatomic.h>    // atomic_store_explicit (M3 S 值保护报警)
#define PI 3.14159265358979323846
#define ARC_SEGMENT_LENGTH_MM 0.5 // 圆弧插补时的分段长度，单位mm
#define RTCP_LINEAR_SEGMENT_MM 0.5 // RTCP直线微段打碎步长，单位mm

// C99 designated initializer: Phase 2A.2 固定循环字段自动归零 (active_cycle=0, retract_mode=0)
// modal_wcs 默认 COORD_G54 (与 g_coord_mgr 启动初值一致, 典型 CNC 上电默认)
GCodeState_t g_state = {
    .feedrate_mm_min   = 1000.0,
    .is_absolute       = 1,
    .active_plane      = 17,
    .motion_mode       = 1,
    .modal_wcs         = COORD_G54,
    .feed_mode         = FEED_MODE_G94,
};

// M1 可选停全局开关 (HMI 通过 SMC_SetOptionalStopEnable 写, parser M1 分支读)
// @Thread-Safety: 单写者 + 单读者, int 写天然原子, 无需锁
int g_optional_stop_enabled = 0;

// ---- Phase 2B M1: PC 寄存器与跳转信号 ----
// 仅 parser_thread_func 修改 PC; parse_gcode_line 通过 g_pc_jump_pending 信号触发跳转
// 全部为 parser 后台线程独占访问, 无需锁
static int g_pc = 0;                    // 当前程序计数器 (lines[] 索引)
static int g_pc_jump_pending = 0;       // parse_gcode_line 设置, parser_thread_func 消费
static int g_pc_jump_target = 0;        // 跳转目标 lines[] 索引
static GCodeProgram_t *g_current_program = NULL;  // 当前运行程序 (NULL 表示走旧路径)
static int g_pc_step_counter = 0;       // 步进计数器, 防 GOTO 死循环

// ---- Phase 2B M5: 子程序调用栈 ----
// M98/G65 压栈保存调用者上下文, M99 弹栈恢复
// 现代语义: #1-#33 局部变量 + modal G-code 状态都完全隔离
#define MAX_CALL_DEPTH 8  // 子程序嵌套深度上限 (Fanuc 0i=4, 30i=10, 取中)
typedef struct {
    int          return_pc;          // 调用者 PC (M98/G65 行的下一行)
    int          entry_line;         // 子程序入口 (O-label 行索引), 用于 L<重复> 再次跳入
    int          repeat_remaining;   // L-1 后剩余次数, 每次 M99 减一
    double       saved_locals[34];   // 调用者 #1-#33 快照 (索引 1..33)
    GCodeState_t saved_state;        // 调用者 modal G-code 状态快照
    // P4' G65: 字母参数 + 重复调用标志
    // is_g65_frame=1 时, M99 repeat 路径需重新应用 g65_args 到 #1-#26 (Fanuc 语义)
    int          is_g65_frame;
    double       g65_args[27];       // 字母参数 (索引 1-26 对应 #1-#26)
    int          g65_args_set[27];   // 哪些字母被指定 (1=指定)
} CallFrame_t;
static CallFrame_t g_call_stack[MAX_CALL_DEPTH];
static int         g_call_stack_top = 0;
ParserControl_t g_parser_ctrl = {"", 0, 0, 0}; // 全局G-code解析控制变量，初始值为未运行、未暂停、未请求中止
extern int api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_trajectory_g93(double target_pos[AXIS_NUM],double speed,double acc,double dec,double g93_dt_sec);
extern int api_push_trajectory_passthrough(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_trajectory_rtcp(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_mcode(int m_code, double s_value, double p_value, double q_value, double r_value);
// 论文1 B0′ 消融 (axis_ctrl.c 定义): 1=每微段继承整块进给字 (τ_b=n_b·T_b)
extern int g_g93_f_inherit;

// P4' Phase 2: G65/G66 共用的宏调用分发 helper (前向声明)
// 参数: o_num=目标O号, l_repeat=重复次数, args/args_set=字母参数(#1-#26),
//       is_g65_frame=1 则 M99 repeat 路径重新应用 args, tag=日志标识("G65"/"G66-modal")
// 返回: 0=成功(g_pc_jump_pending 已设), -1=错误, 1=L=0 no-op (调用方应 return 0)
static int dispatch_macro_call(int o_num, int l_repeat,
                                const double args[27], const int args_set[27],
                                int is_g65_frame, const char *tag);

const char* skip_spaces(const char* str)
{
    while(*str==' '||*str=='\t') str++;
    return str;
}

// ====================================================================
// Cross-Semantics Pose 实验 (2026-09-30) — 注入配置 static 区
// (上移至首个使用点 apply_rtcp_to_pos 之前; getenv 解析在 helper 内)
// ====================================================================
static double g_pose_inject_deg     = 0.0;
static int    g_pose_inject_beg     = 0;
static int    g_pose_inject_end     = 0;
static int    g_pose_inject_mode    = 0;
static int    g_pose_inject_init    = 0;
static int    g_pose_inject_prev_on = 0;

// @Context: Non-RealTime Background Thread
// @Safe: 纯坐标变换，调用 Kinematics_Inverse 纯函数。
// 动态适配 AC / BC / ABC 构型：
//   - X/Y/Z 必须全部映射，否则无法逆解
//   - A/B/C 未映射时角度取 0.0，不影响已配置的旋转轴
static double pose_inject_delta_deg(int src_line);   // C2 注入点引用 (定义在下方)

static void apply_rtcp_to_pos(double pos[AXIS_NUM])
{
    int idx_x = g_axis_map['X' - 'A'];
    int idx_y = g_axis_map['Y' - 'A'];
    int idx_z = g_axis_map['Z' - 'A'];
    int idx_a = g_axis_map['A' - 'A'];
    int idx_b = g_axis_map['B' - 'A'];
    int idx_c = g_axis_map['C' - 'A'];

    // 线性轴是逆解的前提，旋转轴缺失不影响
    if(idx_x < 0 || idx_y < 0 || idx_z < 0) return;

    double tip[3] = { pos[idx_x], pos[idx_y], pos[idx_z] };
    double rot_a = (idx_a >= 0) ? pos[idx_a] : 0.0;
    double rot_b = (idx_b >= 0) ? pos[idx_b] : 0.0;
    double rot_c = (idx_c >= 0) ? pos[idx_c] : 0.0;
    double joint[3];
    Kinematics_Inverse(tip, rot_a, rot_b, rot_c, joint);
    pos[idx_x] = joint[0];
    pos[idx_y] = joint[1];
    pos[idx_z] = joint[2];

    // Cross-Semantics Pose 实验 C2 (uncompensated, MODE=1): RTCP 逆解之后仅对
    // B 命令加 δ, X/Z 保持 clean 补偿 → 姿态偏差传播到 TCP (positive control)。
    // 注入点在软限位 0x0024 检查之后 (prereg §1 已注明); 窗口行号判定限制
    // 注入范围, MODE=0/缺省时零影响。
    if (g_pose_inject_mode == 1 && idx_b >= 0) {
        pos[idx_b] += pose_inject_delta_deg((int)g_current_line_no);
    }
}

// ====================================================================
// P0: 旋转轴 G90 最短路径解缠绕 (wrap-to-nearest)
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯数学 (fmod/fabs), 无副作用。
// 语义: 仅 G90 逻辑坐标调用; G91 增量精确执行; G53 机械绝对直通 (逃生通道)。
// 边界约定: delta 折到 (-180, +180], ±180 归正方向 (与 test_rtcp_math.nc
//           C180 边界用例兼容: +180 保持不动)。
// 与软限位交互: wrap 后目标超限而 ±360° 候选在限内 → 选合法候选;
//               全非法 → 保持最短值, 交 check_soft_limits 0x0024 报警。
// ====================================================================
static double wrap_rotary_shortest(int axis_idx, double machine_cur,
                                   double machine_tgt, int *wrapped)
{
    *wrapped = 0;
    double d = fmod(machine_tgt - machine_cur, 360.0);
    if (d <= -180.0)    d += 360.0;   // (-360,-180] → (0,180]; -180 归正方向
    else if (d > 180.0) d -= 360.0;   // (180,360) → (-180,0); +180 保持
    double best = machine_cur + d;

    // 本就是最短表示 (含 ±360 整数倍的 fmod 残差): 原值返回, 不发事件
    if (fabs(machine_tgt - best) < 1e-9) return machine_tgt;

    // 软限位候选选择 (机械域, 与 api 层 check_soft_limits 同一坐标系)
    if (g_axis[axis_idx].enable_soft_limit) {
        double lo = g_axis[axis_idx].soft_limit_neg;
        double hi = g_axis[axis_idx].soft_limit_pos;
        double cand_p = best + 360.0, cand_m = best - 360.0;
        int ok_best = (best  >= lo - 1e-9 && best  <= hi + 1e-9);
        int ok_p    = (cand_p >= lo - 1e-9 && cand_p <= hi + 1e-9);
        int ok_m    = (cand_m >= lo - 1e-9 && cand_m <= hi + 1e-9);
        if (!ok_best) {
            if (ok_p && ok_m) {
                // 行程 >720° 特例: 双候选合法, 取离原始目标近者
                best = (fabs(cand_p - machine_tgt) <= fabs(cand_m - machine_tgt))
                       ? cand_p : cand_m;
            } else if (ok_p) {
                best = cand_p;
            } else if (ok_m) {
                best = cand_m;
            }
            // 全非法: 保持 best (最短), 交软限位 0x0024 报警
        }
    }

    *wrapped = 1;
    return best;
}

// ====================================================================
// P1 (2026-09-04): G43.5 矢量编程报警 — 事件 0x0027
// value: 0=矢量不可达/构型不支持 1=IJK 与旋转轴字冲突 2=零矢量 3=圆弧禁止
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: printf + EventLogger_Push 允许。调用方负责 return -1 中止文件。
// ====================================================================
static void g435_vector_alarm(int reason, int src_line)
{
    static const char *msg[4] = {
        "G43.5 orientation unreachable/config unsupported",
        "G43.5 I/J/K conflicts with rotary axis words",
        "G43.5 zero tool-axis vector",
        "G43.5 arc (G02/G03) banned in vector mode",
    };
    printf("[Parser][ALARM] %s, 源行 %d\n", msg[reason & 3], src_line);
    atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
    EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0027, reason, msg[reason & 3]);
}

// ====================================================================
// Cross-Semantics Pose 实验 (2026-09-30): C1 受控姿态偏差注入 — sim 实验装置
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: getenv 仅首次调用解析并缓存; 此后纯标量比较。
//
// 环境变量 (WSL2 跑批前设置; 全部缺省 = 注入关闭, 行为与主线路径完全一致):
//   CNC_POSE_INJECT_DEG  δ 峰值 (度, 叠加到 rot_1 解算输出 s1; 0/缺省 = OFF)
//   CNC_POSE_INJECT_BEG  窗口起 (G 代码 line_no, 含端点; 缺省 0)
//   CNC_POSE_INJECT_END  窗口止 (含端点; BEG=END=0 → 全程注入)
//   CNC_POSE_INJECT_MODE 0=C1 masked (缺省): solve 后 s1+=δ, 经完整正常管线;
//                        1=C2 uncompensated: RTCP 逆解后仅 B 命令 += δ,
//                        X/Z 保持 clean 补偿 → TCP 不再恢复 (positive control,
//                        prereg POSE-CROSS-SEMANTICS-1 冻结后追加 2026-09-30)
//
// C1 注入点在 SolveOrientation 之后: s1 += δ 再走正常 wrap + 软限位 0x0024 +
// RTCP 逆解管线 (Inverse(tip, θ+δ) 数学保证 TCP 严格不变, u_tool 精确偏差 δ
// —— "TCP-preserved / pose-deviated" 的受控化, 即 declared≠executed 姿态)。
// C2 注入点在 apply_rtcp_to_pos 尾部 (逆解之后): B 命令偏 δ 而 X/Z 不再
// 补偿 → 偏差传播到 TCP (预期 ≈ 2L·sin(δ/2), 正式批 TCP oracle 应 FAIL)。
// 安全网交互: 奇异 0x0026 在解算器内部, 不在注入路径; 软限位 0x0024 会检查
// C1 注入后目标 → δ 必须选在旋转轴软限位余量内 (实验设计约束, 见实验方案 §7);
// C2 注入点在 0x0024 检查之后 (prereg §1 已注明该差异)。
// 窗口激活/退出各发一次 0x003B 事件 (provenance 标记, 防 run 条件混淆;
// MODE=1 下事件由 apply_rtcp_to_pos 注入点发出, OFF 沿需窗口后存在下一段
// 运动段才触发, 程序尾缺失可接受)。
// ====================================================================
static double pose_inject_delta_deg(int src_line)
{
    if (!g_pose_inject_init) {
        const char *s;
        if ((s = getenv("CNC_POSE_INJECT_DEG")) != NULL) g_pose_inject_deg = strtod(s, NULL);
        if ((s = getenv("CNC_POSE_INJECT_BEG")) != NULL) g_pose_inject_beg = atoi(s);
        if ((s = getenv("CNC_POSE_INJECT_END")) != NULL) g_pose_inject_end = atoi(s);
        if ((s = getenv("CNC_POSE_INJECT_MODE")) != NULL) g_pose_inject_mode = atoi(s);
        g_pose_inject_init = 1;
        if (g_pose_inject_deg != 0.0)
            printf("[Parser] pose-inject armed: d=%+.3fdeg win=[%d,%d] mode=%d\n",
                   g_pose_inject_deg, g_pose_inject_beg, g_pose_inject_end,
                   g_pose_inject_mode);
    }
    int on = (g_pose_inject_deg != 0.0)
          && ((g_pose_inject_beg == 0 && g_pose_inject_end == 0)
                ? 1
                : (src_line >= g_pose_inject_beg && src_line <= g_pose_inject_end));
    if (on != g_pose_inject_prev_on) {
        char msg[64];
        snprintf(msg, sizeof(msg), "pose inject %s d=%+.3fdeg",
                 on ? "ON" : "OFF", g_pose_inject_deg);
        EventLogger_Push(on ? SEVERITY_WARN : SEVERITY_INFO,
                         SOURCE_PARSER, 0x003B, src_line, msg);
        g_pose_inject_prev_on = on;
    }
    return on ? g_pose_inject_deg : 0.0;
}

// ====================================================================
// P1 (2026-09-07): G68.2 倾斜面报警 — 事件 0x0028
// value: 0=姿态不可达/构型不支持 1=法向非法(零矢量) 2=圆弧禁止 3=固定循环禁止 4=定位段被拒
// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: printf + EventLogger_Push 允许。调用方负责 return -1 中止文件。
// ====================================================================
static void g682_alarm(int reason, int src_line)
{
    static const char *msg[5] = {
        "G68.2 orientation unreachable/config unsupported",
        "G68.2 zero/invalid tilt normal vector",
        "G68.2 arc (G02/G03) banned on tilted plane",
        "G68.2 fixed cycle (G73/G81-G83) banned on tilted plane",
        "G68.2 auto-positioning move rejected",
    };
    // 注: 索引必须用 reason 本身。早期写作 msg[reason & 4] 会把 reason=1/2/3
    //     全部折叠成 msg[0] ("orientation unreachable"), 掩盖真实拒绝原因
    //     (P1 G682 规范 §4 TC-4 圆弧禁止时实测打印串即错, value 却是对的)。
    printf("[Parser][ALARM] %s, 源行 %d\n", msg[reason], src_line);
    atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
    EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0028, reason, msg[reason]);
}

// ====================================================================
// P1 (2026-09-07): 倾斜面 XYZ 旋转 helper (行主序 R, 列=[u v n])
// @Context: Non-RealTime Background Thread (parser 成形点/捕获块调用)
// @Safe: 纯乘加, 无 math.h libcall
// ====================================================================
static void tilt_apply_rot(const double p_in[3], double p_out[3])
{
    const double *R = g_coord_mgr.tilt_rot;
    p_out[0] = R[0]*p_in[0] + R[1]*p_in[1] + R[2]*p_in[2];
    p_out[1] = R[3]*p_in[0] + R[4]*p_in[1] + R[5]*p_in[2];
    p_out[2] = R[6]*p_in[0] + R[7]*p_in[1] + R[8]*p_in[2];
}
static void tilt_apply_rotT(const double p_in[3], double p_out[3])
{
    const double *R = g_coord_mgr.tilt_rot;
    p_out[0] = R[0]*p_in[0] + R[3]*p_in[1] + R[6]*p_in[2];
    p_out[1] = R[1]*p_in[0] + R[4]*p_in[1] + R[7]*p_in[2];
    p_out[2] = R[2]*p_in[0] + R[5]*p_in[1] + R[8]*p_in[2];
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯坐标变换,可使用 math.h。采样预估离线进行,与 RT 线程无关。
//
// 估算 RTCP 段的 Jacobian Ratio 峰值: max(物理步长 / 逻辑步长)。
// 用途: ① 自适应打碎步长 (R_max 越大,段越细)
//       ② 全局逻辑进给速度压制 (R_max > 1 时, run_speed_mm /= R_max)
//
// 采样策略 (动态密度,修复 Nyquist 盲区审计 🟠 HIGH):
//   - 采样间距上限 5mm (工业经验值,覆盖典型 RTCP 旋转高曲率区)
//   - N = max(21, ceil(logical_total_mm / 5.0))
//   - 上限 200 点 (性能 trade-off, 超长段接受精度损失)
//   - 短段 (< 105mm) 用 21 点,与原方案一致
//   - 长段 (≥ 105mm) 按密度扩展,1000mm 段采 200 点 (5mm 间距)
//
// 退化路径防御 (修复量纲混乱审计 🟡 MEDIUM):
//   - logical_total < 1e-6 mm 时,返回保守值 10.0 触发自适应收紧
//     (旧版返回 1.0 会退化为纯平动假设,失去保护)
//   - 旋转轴 equivalent_radius=0 时,警告但继续 (角度与 mm 混算,
//     R 值失去物理意义,但保守的 10.0 兜底仍能保护)
//
// 返回 R_max ≥ 1.0 (保守路径返回 ≥ 10.0)
//
// tip_only 选择【分母】定义 —— 两种度量的物理语义不同,不可混用:
//
//   tip_only=0 (运动学度量 R_kin, 既有行为, 保持不变):
//     分母 = 全轴编程位移,旋转轴 delta 按 equivalent_radius 折算成弧长。
//     语义: "物理轴比编程进给快多少倍" → 用于自适应打碎步长与进给压制,
//           保证物理轴速度不超过编程进给。改动它会改变既有轨迹 → 禁止改。
//
//   tip_only=1 (奇异点度量 R_sing, P0-RTCP):
//     分母 = 纯刀尖线位移 (XYZ),旋转轴变化【不】计入。
//     语义: "刀尖微小位移是否要求巨大物理补偿" —— 旋转轴摆动是【因】
//           (姿态变化),若把它折算进分母,分母会被放大到主导量级,
//           把 R 稀释到 <2,奇异点永远检测不到 (TC-5 实测 1.179 vs 阈值 30)。
//     与圆弧侧 r_seg = step_dist / arc_logical_step 的定义保持一致 ——
//     圆弧侧 arc_logical_step 本就是纯刀尖弧长,不含旋转轴折算。
//
// 退化语义: 刀尖静止 (纯姿态重定位) 时 tip_only=1 的分母 ≈ 0 → 走退化分支
//           返回 RTCP_R_MAX_DEGENERATE (10.0),不报警。这与 spec §5 对
//           L3_kinematics_rtcp / test_rtcp_math 的期望一致。
#define RTCP_JACOBIAN_SAMPLES_MIN  21
#define RTCP_JACOBIAN_SAMPLES_MAX  200
#define RTCP_JACOBIAN_SPACING_MM   5.0
#define RTCP_R_MAX_DEGENERATE      10.0  // 退化路径保守值
static double estimate_max_jacobian_ratio(double start_pos[AXIS_NUM],
                                          double end_pos[AXIS_NUM],
                                          int tip_only)
{
    // ---- 入口防御: 旋转轴半径配置检查 (审计攻击面 3) ----
    // 若旋转轴 equivalent_radius=0,其 delta 以原始角度值参与距离计算,
    // 与物理 mm 量纲不一致,R 值失去物理意义。警告并继续。
    for(int j = 0; j < AXIS_NUM; j++){
        if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius <= 0.0){
            double delta = fabs(end_pos[j] - start_pos[j]);
            if(delta > 1e-6){
                printf("[RTCP] 警告: 旋转轴 %s equivalent_radius 未配置,"
                       "Jacobian 预估可能失准 (本段 %s 变化 %.3f°)\n",
                       g_axis[j].axis_name, g_axis[j].axis_name, delta);
            }
        }
    }

    // ---- 计算逻辑总距离 ----
    // tip_only=1: 跳过旋转轴,只累计刀尖线位移 (见函数头注释)。
    // tip_only=0: 含旋转轴弧长折算,与打碎逻辑一致 (既有行为)。
    double logical_total_sq = 0.0;
    for(int j = 0; j < AXIS_NUM; j++){
        if(tip_only && g_axis[j].axis_type == 1) continue;
        double delta = end_pos[j] - start_pos[j];
        if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
            delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
        }
        logical_total_sq += delta * delta;
    }
    double logical_total = sqrt(logical_total_sq);

    // ---- 退化路径防御: logical_total 过小时返回保守值 ----
    // 旧版阈值 1e-9 + 返回 1.0 有两个问题:
    //   (a) [1e-9, 2.1e-8] 区间 logical_step ≤ 1e-9 跳过比率计算,R_max 保持 1.0
    //   (b) 纯姿态变化段 (刀尖不动) 若 radius 未配置, logical_total 仍可能非零
    // 提高阈值到 1e-6 + 返回 10.0,触发自适应收紧兜底
    if(logical_total < 1e-6) return RTCP_R_MAX_DEGENERATE;

    // ---- 动态采样密度 (审计攻击面 2) ----
    int N = RTCP_JACOBIAN_SAMPLES_MIN;
    double spacing_threshold = RTCP_JACOBIAN_SPACING_MM * (double)N;  // 105mm
    if(logical_total > spacing_threshold){
        N = (int)ceil(logical_total / RTCP_JACOBIAN_SPACING_MM);
        if(N > RTCP_JACOBIAN_SAMPLES_MAX) N = RTCP_JACOBIAN_SAMPLES_MAX;
    }

    // ---- 逐点采样: 物理步长 / 逻辑步长 ----
    double R_max = 1.0;
    double prev_phys[AXIS_NUM];
    memcpy(prev_phys, start_pos, sizeof(double) * AXIS_NUM);
    apply_rtcp_to_pos(prev_phys);

    for(int i = 1; i <= N; i++){
        double ratio = (double)i / (double)N;
        double pos[AXIS_NUM];
        for(int j = 0; j < AXIS_NUM; j++){
            pos[j] = start_pos[j] + (end_pos[j] - start_pos[j]) * ratio;
        }
        // 末点强制对齐终点,消除浮点累积
        if(i == N){
            memcpy(pos, end_pos, sizeof(double) * AXIS_NUM);
        }
        apply_rtcp_to_pos(pos);

        double phys_step_sq = 0.0;
        for(int j = 0; j < AXIS_NUM; j++){
            double d = pos[j] - prev_phys[j];
            if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                d = d * (PI / 180.0) * g_axis[j].equivalent_radius;
            }
            phys_step_sq += d * d;
        }
        double phys_step = sqrt(phys_step_sq);
        double logical_step = logical_total / (double)N;
        if(logical_step > 1e-9){
            double R = phys_step / logical_step;
            if(R > R_max) R_max = R;
        }
        memcpy(prev_phys, pos, sizeof(double) * AXIS_NUM);
    }
    return R_max;
}

// @Context: Non-RealTime Background Thread (parser 调用)
// @Safe: 纯坐标变换 + 调用 api_push_trajectory_rtcp（队列写入，线程安全）。
// RTCP 直线微段打碎 (Adaptive Jacobian Step):
//   1. 21 点 Jacobian 预估 R_max (物理步长 / 逻辑步长 的峰值)
//   2. 自适应步长: adaptive_segment_mm = 0.5 / max(1.0, R_max), 下限 0.01mm
//      - 纯直线 (R_max=1): 保持 0.5mm
//      - 强非线性 (R_max=10): 收紧到 0.05mm
//   3. 全局逻辑进给压制: 若 R_max > 1.0, run_speed_mm /= R_max
//      保证物理轴速度不超过原始进给,防止微段内物理超速
//   4. 逐点逆解 + 入队 (标记 is_rtcp_active=1)
//   5. 无旋转变化时退化为单次逆解入队
int generate_linear_rtcp_trajectory(double start_pos[AXIS_NUM], double end_pos[AXIS_NUM],
                                    double run_speed_mm, double g93_T_sec)
{
    int idx_a = g_axis_map['A' - 'A'];
    int idx_b = g_axis_map['B' - 'A'];
    int idx_c = g_axis_map['C' - 'A'];

    // 判定旋转轴是否有变化
    int has_rotation = 0;
    if(idx_a >= 0 && fabs(end_pos[idx_a] - start_pos[idx_a]) > 1e-6) has_rotation = 1;
    if(idx_b >= 0 && fabs(end_pos[idx_b] - start_pos[idx_b]) > 1e-6) has_rotation = 1;
    if(idx_c >= 0 && fabs(end_pos[idx_c] - start_pos[idx_c]) > 1e-6) has_rotation = 1;

    // 无旋转变化：单次逆解入队 (R_max 必为 1.0)
    if(!has_rotation){
        double phys_end[AXIS_NUM];
        memcpy(phys_end, end_pos, sizeof(double) * AXIS_NUM);
        apply_rtcp_to_pos(phys_end);
        double speed_mm_sec = run_speed_mm / 60.0;
        // G93 必须走强一致性 wrapper (论文1 V2 战役 发现 F1, 2026-09-18):
        // 此前本分支恒调 api_push_trajectory_rtcp(), 使 is_g93_strict=0 /
        // g93_dt_sec=0, 严格时间剖面被跳过而退化成梯形 —— 实测
        //   G43.4 G93 G01 X10 B0 C0 F60  ->  tau = 1.159 s (T=1 s, +15.9%)
        // 且 tau ≈ T + v_cmd/a_max (v=5/10/20 mm/s -> 超时 110/159/409 ms),
        // v_cmd 在给定长度内达不到时饱和到 2*sqrt(L/a)。
        // 无旋转时 RTCP 变换是常量平移, 物理位移 = 逻辑位移, 故
        // speed_mm_sec = dist_total/T 已是正确的时间守恒速度, 直接沿用。
        if(g93_T_sec > 1e-9){
            // 论文1 S10: 无旋转分支是**单段**承载整块预算, gen=1, sum==cmd。
            bt_block_open(g93_T_sec * 1000.0, 1, g93_T_sec * 1000.0,
                          g_current_line_no);
            int rc = api_push_trajectory_g93(phys_end, speed_mm_sec,
                                             DEFAULT_ACC, DEFAULT_DEC, g93_T_sec);
            bt_block_close();
            return rc;
        }
        return api_push_trajectory_rtcp(phys_end, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC);
    }

    // ---- 1. Jacobian 预估 (21 点采样) ----
    // R_kin (tip_only=0): 运动学度量, 供下方自适应步长 + 进给压制使用。
    //   必须保持既有定义 —— 改动它会改变所有既有 RTCP 轨迹的步长与进给,
    //   直接破坏 §5 回归的 byte-identical 判据。
    double R_max = estimate_max_jacobian_ratio(start_pos, end_pos, 0);

    // ---- 1.5 P0 奇异点硬报警 (专用度量 R_sing) ----
    // 物理含义: 旋转轴接近奇异位形时, 微小刀尖位移要求巨大的物理轴补偿运动,
    //          R_sing (物理步长 / 刀尖线位移 峰值) 急剧上升。硬报警防止奇异段执行。
    // 必须用 tip_only=1 的独立度量: 若沿用 R_kin, 其分母含旋转轴 eq_radius
    //          折算 (B 摆 45° 即 62.8mm), 会把 R 稀释到 ~1.18, 奇异点永远
    //          检测不到 —— 这是 TC-5 失败的根因 (实测 R=1.18 vs 阈值 30)。
    // 刀尖静止 (纯姿态重定位) 时 R_sing 走退化分支 =10.0, 不报警,
    //          与 spec §5 对 L3_kinematics_rtcp / test_rtcp_math 的期望一致。
    double R_sing = estimate_max_jacobian_ratio(start_pos, end_pos, 1);
    if(g_rtcp_singularity_rmax > 0.0 && R_sing > g_rtcp_singularity_rmax){
        char ev_buf[64];
        snprintf(ev_buf, sizeof(ev_buf), "RTCP singular R_max=%.1f>%.1f",
                 R_sing, g_rtcp_singularity_rmax);
        printf("[RTCP] 奇异点报警: %s (中止文件)\n", ev_buf);
        EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0026,
                         (int32_t)(R_sing * 10.0), ev_buf);
        atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
        return -1;   // 打碎前拦截, 零段入队
    }

    // ---- 2. 全局逻辑进给压制 (R_max > 1.0 时) ----
    // 物理含义: 若物理轴运动比逻辑刀尖快 R_max 倍,则把刀尖进给等比降低,
    //          保证物理轴速度不超过原始 run_speed_mm 对应的物理速度。
    // G93 模式时间预算刚性,豁免此压制 (用户已强制 T_sec,不能改),
    //          改由下方自适应步长保证微段内物理速度合理。
    if(g93_T_sec <= 1e-9 && R_max > 1.0){
        run_speed_mm /= R_max;
        if(run_speed_mm < 1e-6) run_speed_mm = 1e-6;
    }

    // ---- 3. 等效空间距离 (含旋转轴弧长折算) ----
    double dist_eq = 0.0;
    for(int i = 0; i < AXIS_NUM; i++){
        double delta = end_pos[i] - start_pos[i];
        if(g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0){
            delta = delta * (PI / 180.0) * g_axis[i].equivalent_radius;
        }
        dist_eq += delta * delta;
    }
    dist_eq = sqrt(dist_eq);

    // ---- 4. 段数与时间预算: 自适应步长 + G93/G94 双模式 ----
    // G93: 1ms 时间切分 (T_sec * 1000),与 RT 线程 1ms 周期对齐,绝对守恒总时间。
    //      封顶 20000 段 (20 秒),防止超长 G93 段塞爆 QUEUE_SIZE。
    // G94: 自适应空间切分。
    //      adaptive_segment_mm = 0.5 / max(1.0, R_max),下限 0.01mm。
    //      线性段保持 0.5mm; 非线性段收紧至最低 0.01mm,
    //      保证 S 曲线 7 段预计算在每个微段内的物理一致性。
    int num_segments;
    double dt_per_seg;
    if(g93_T_sec > 1e-9){
        num_segments = (int)ceil(g93_T_sec * 1000.0);
        if(num_segments > 20000) num_segments = 20000;
        if(num_segments < 1) num_segments = 1;
        dt_per_seg = g93_T_sec / (double)num_segments;
        // B0′ 消融 (论文1, 2026-09-17): 禁用 IV-B 再分配, 微段直接继承整块
        // 进给字 → τ_b = n_b·T_b。下方 phys_speed = phys_dist/dt_per_seg 随之
        // 自动一致 (段速度 = 段长/继承预算), 无需二次改动。
        if(g_g93_f_inherit) dt_per_seg = g93_T_sec;
    } else {
        double adaptive_segment_mm = RTCP_LINEAR_SEGMENT_MM / fmax(1.0, R_max);
        if(adaptive_segment_mm < 0.01) adaptive_segment_mm = 0.01;
        if(adaptive_segment_mm > RTCP_LINEAR_SEGMENT_MM) adaptive_segment_mm = RTCP_LINEAR_SEGMENT_MM;
        num_segments = (int)ceil(dist_eq / adaptive_segment_mm);
        if(num_segments < 1) num_segments = 1;
        // ---- 段数硬上限 (审计攻击面 1, CRITICAL) ----
        // 旧版无上限: R_max=500 + dist_eq=2000mm → 200,000 段 → Parser 阻塞 3 分钟。
        // 红方原方案 return -1 会触发 parse_gcode_line 中止整个文件,等同 DoS 升级。
        // 改进: clamp + SEVERE 警告 + 继续执行。物理一致性由 Layer 2 (Planner 短板限幅) 兜底。
        // 用户通过警告主动修正 G-code,而非被动中止生产。
        if(num_segments > 20000) {
            printf("[RTCP] SEVERE: 单段打碎 %d 段超过上限 20000 (dist=%.1fmm, R_max=%.1f),"
                   "强制 clamp, 物理一致性可能损失 - 请检查 G-code 奇异点!\n",
                   num_segments, dist_eq, R_max);
            num_segments = 20000;
        }
        double seg_logical_dist = dist_eq / (double)num_segments;
        double feed_mm_sec = run_speed_mm / 60.0;
        if(feed_mm_sec < 1e-6) feed_mm_sec = 1e-6;
        dt_per_seg = seg_logical_dist / feed_mm_sec;
    }

    // ---- 4.5 论文1 S10: block 计时插桩 (仅 G93) ----
    // 必须在任何 push 之前声明: block_seg_count 要在**第一个**微段入队时就已知
    // (段按值入队, 无法事后回填)。G94 自适应空间切分不参与本实验, 保持未插桩。
    //   block_budget_cmd_ms = T*1000                  (指令时长)
    //   block_budget_sum_ms = Σ dt_per_seg*1000       (实际下发预算之和)
    //   Full : sum == cmd  (= T*1000)
    //   -H2  : sum == n*T*1000  (B0′ 每微段继承整块 T)
    if(g93_T_sec > 1e-9){
        bt_block_open(g93_T_sec * 1000.0, num_segments,
                      dt_per_seg * (double)num_segments * 1000.0,
                      g_current_line_no);
    }

    // ---- 5. 逐点插值 + 逆解 + 物理速度计算 + 入队 ----
    for(int i = 1; i <= num_segments; i++){
        double ratio = (double)i / (double)num_segments;
        double interp_pos[AXIS_NUM];

        // 逻辑空间 N 维线性插值
        for(int j = 0; j < AXIS_NUM; j++){
            interp_pos[j] = start_pos[j] + (end_pos[j] - start_pos[j]) * ratio;
        }

        // 末段强制对齐终点，消除浮点累积误差
        if(i == num_segments){
            memcpy(interp_pos, end_pos, sizeof(double) * AXIS_NUM);
        }

        // 逐点 RTCP 逆解
        apply_rtcp_to_pos(interp_pos);

        // 计算本微段物理距离（等效半径折算旋转轴弧长）
        double phys_dist_sq = 0.0;
        for(int j = 0; j < AXIS_NUM; j++){
            double delta = interp_pos[j] - api_get_cursor(j);
            if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
            }
            phys_dist_sq += delta * delta;
        }
        double phys_dist = sqrt(phys_dist_sq);

        // 物理下发速度 = 物理距离 / 时间预算
        // H1 消融 (S10, 2026-09-20): tip_basis=1 时改为信任"声明进给"
        // (run_speed_mm 由刀尖欧氏长度 L^P 反推) 而非逐微段实测物理量
        // phys_dist/dt_per_seg —— 朴素控制器不做微段重测。执行侧 strict 段
        // 仍按 dist/T 重算, 故轨迹不变; 只有可行性门被系统性低估 m 倍。
        double phys_speed = (dt_per_seg > 1e-9) ? phys_dist / dt_per_seg : 1e-6;
        if(g_g93_tip_basis && g93_T_sec > 1e-9) phys_speed = run_speed_mm / 60.0;
        if(phys_speed < 1e-6) phys_speed = 1e-6;

        // G93 模式走强一致性路径,豁免底层短板限幅,刚性守恒时间预算。
        // G94 模式走标准路径 + RTCP 标记。
        // 注意: G93 路径暂未提供 _rtcp 变体,因为 G93 已豁免短板限幅,
        //       is_rtcp_active 元数据对 G93 段意义不大 (RT 线程不区分对待)。
        //       后续若需 G93 RTCP 元数据,可扩展 api_push_trajectory_g93_rtcp。
        // F4 (2026-09-19): 本处是**按时间切分**出来的微段 (dt_per_seg = T/n ≈ 1ms),
        //   改用 _slice 变体显式告知 axis_ctrl, 使其在可行性回退时走恒速通过,
        //   避免逐微段 0→v→0 的爬行 (S0 B4 实测 τ/T = 46.69, 应 5.33)。
        int push_ret;
        if(g93_T_sec > 1e-9){
            push_ret = api_push_trajectory_g93_slice(interp_pos, phys_speed,
                                                     DEFAULT_ACC, DEFAULT_DEC,
                                                     dt_per_seg);
        } else {
            push_ret = api_push_trajectory_rtcp(interp_pos, phys_speed,
                                                 DEFAULT_ACC, DEFAULT_DEC);
        }
        if(push_ret < 0){
            bt_block_close();
            return -1;
        }
    }
    bt_block_close();

    printf("[Parser] RTCP直线打碎: %d 段 (等效距离 %.2f mm, R_max=%.2f, 步长=%.3fmm)\n",
           num_segments, dist_eq, R_max,
           (g93_T_sec > 1e-9) ? 0.0 : dist_eq / fmax(1, num_segments));
    return 0;
}

// ============================================================
// P4' Phase 2: G65/G66 共用的宏调用分发 helper
// ============================================================
// @Context: Non-RealTime Background Thread (parser)
// @Safe: 仅修改 g_call_stack / g_pc_jump_*, 无阻塞 I/O
//
// 功能:
//   1. 校验 P<Onum> / O-label 存在 / 调用栈深度
//   2. 压 CallFrame (含 caller #1-#33 + modal 状态 + G65 args)
//   3. ClearLocals + 应用 args 到 #1-#26 (G65/G66 共用语义)
//   4. 设 g_pc_jump_target/pending 跳到子程序入口
//
// 返回: 0=成功跳转 / 1=L=0 no-op (调用方 return 0) / -1=错误 (调用方 return -1)
// ============================================================
static int dispatch_macro_call(int o_num, int l_repeat,
                                const double args[27], const int args_set[27],
                                int is_g65_frame, const char *tag)
{
    if(!g_current_program){
        printf("[Parser] %s 在无 program 上下文\n", tag);
        return -1;
    }
    if(o_num < 1){
        printf("[Parser] %s 缺 P<Onum> (源行 %d)\n", tag,
               g_current_program->lines[g_pc].line_no);
        return -1;
    }
    if(l_repeat == 0){
        printf("[Parser] %s P%d L0 no-op (源行 %d)\n", tag, o_num,
               g_current_program->lines[g_pc].line_no);
        return 1;  // no-op, 调用方 return 0
    }
    int o_idx = Program_FindOLabel(g_current_program, o_num);
    if(o_idx < 0){
        printf("[Parser] %s P%d 未找到子程序 (源行 %d)\n", tag, o_num,
               g_current_program->lines[g_pc].line_no);
        return -1;
    }
    if(g_call_stack_top >= MAX_CALL_DEPTH){
        printf("[Parser] %s 子程序嵌套超 %d (源行 %d)\n", tag, MAX_CALL_DEPTH,
               g_current_program->lines[g_pc].line_no);
        return -1;
    }
    int entry = g_current_program->o_labels[o_idx].line_idx;

    int argc = 0;
    for(int n = 1; n <= 26; n++) if(args_set[n]) argc++;

    CallFrame_t *f = &g_call_stack[g_call_stack_top];
    f->return_pc        = g_pc + 1;
    f->entry_line       = entry;
    f->repeat_remaining = l_repeat - 1;
    Macro_GetLocals(f->saved_locals);
    f->saved_state      = g_state;
    f->is_g65_frame     = is_g65_frame;
    for(int n = 1; n <= 26; n++){
        f->g65_args[n]     = args[n];
        f->g65_args_set[n] = args_set[n];
    }
    g_call_stack_top++;

    // P4' Phase 2: 进入子程序时清掉 modal_macro_active, 防止宏体内运动递归触发自身
    // (M99 真返回时由 saved_state 自动恢复, M99 repeat 路径不恢复因 frame 仍占用)
    g_state.modal_macro_active = 0;

    Macro_ClearLocals();
    for(int n = 1; n <= 26; n++){
        if(args_set[n]) Macro_SetValue(n, args[n]);
    }

    g_pc_jump_target  = entry;
    g_pc_jump_pending = 1;
    printf("[Parser] %s P%d L%d 调用宏 (entry=%d, depth=%d, %d 个参数, 源行 %d)\n",
           tag, o_num, l_repeat, entry, g_call_stack_top, argc,
           g_current_program->lines[g_pc].line_no);
    return 0;
}

// 程序级模态复位 — M30/M2 (Step 3) 与程序中止收尾共用 (2026-11-25 抽出)。
// 作用: 把"程序内建立、不应跨程序存活"的 G/M 模态回到安全默认值 (纯赋值, 无 I/O)。
// 背景: 此前只有 M30 路径复位; 程序被中止 (Stop/急停/报警联动/RPC abort) 时直接退出,
//   M3 出光、冷却液、刀补、G68.2 倾斜等模态残留 — 运动段的 aux 快照取自这些字段,
//   下个程序的定位段会带着出光快照入队, 属实际安全隐患。
// 注意: 不复位运动模态 (G90/G91、G94/G93、G17/G18/G19、坐标系), 与 M30 既有行为一致。
static void gcode_reset_program_modals(void)
{
    // 历史 BUG: M30 仅清 spindle/coolant, 导致 modal_macro_active /
    //   active_cycle / bspline_enabled 等模态字段泄漏到下个程序.
    //   Phase A 顺便修复, 严格遵循 Fanuc M30=程序复位语义.
    g_state.spindle_mode   = 0;
    g_state.spindle_rpm    = 0.0;
    g_state.coolant_state  = 0;
    g_state.modal_macro_active = 0;          // G66 模态宏泄漏
    g_state.modal_macro_O_num = 0;
    g_state.active_cycle       = 0;          // G81/G82/G83 固定循环泄漏
    g_state.cycle_retract_mode = 0;          // G98/G99
    g_state.bspline_enabled    = 0;          // M50 P1
    g_state.rtcp_enabled       = 0;          // G43.4
    g_state.vector_mode        = 0;          // G43.5 (P1)
    g_coord_mgr.tilt_active    = 0;          // G68.2/G68.3 (P1)
    g_state.pending_comp_mode  = COMP_OFF;   // G41/G42
    g_state.local_offset_active= 0;          // G52
    g_state.modal_ext_wcs_p    = 0;          // G54.1 Pn
    // P0-Laser: M30 强制关激光 (与 spindle 一同复位)
    g_state.laser_shutter_pending = 0;
    g_state.laser_power_pending   = 0.0;
    g_state.laser_freq_pending    = 0.0;
    g_state.gas_select            = 0;
    g_state.laser_gas_pressure_pending = 0.0;   // P0-2
    // Laser B4: 清段级工艺标记 modal (防跨程序泄漏)
    g_state.laser_seg_flags       = 0;
    // P1-5: 微连接系数回默认 1.0 (程序级 modal, 不跨程序泄漏)
    atomic_store_explicit(&g_laser_cfg.mj_ratio, 1.0,
                          memory_order_release);
}

int parse_gcode_line(const char *gcode_line)
{
    // 宏赋值行首拦截：#N = <expr>
    // 标准 CNC 中赋值必须独占一行（不会出现 G01 X10 #100 = 5 这种语法）
    if(Macro_TryParseAssignment(gcode_line)){
        return 0;
    }

    char buffer[256];   // 从 128 提升：宏表达式可较长，与上游 line_buffer[256] 对齐
    strncpy(buffer, gcode_line, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    int has_move=0;
    int has_axis[AXIS_NUM]={0};
    double val_axis[AXIS_NUM]={0};
    double offset_i=0.0,offset_j=0.0,offset_k=0.0; // 圆弧偏移：非模态，逐行清零
                                                        // P1: G43.5 模态下兼作刀轴矢量分量
    int has_ijk_word=0;   // P1: 本行出现 I/J/K 字 (矢量解算触发条件; 缺分量=0)
    int is_g682_block=0;  // P1 (2026-09-07): G68.2 倾斜面 (字母循环后捕获+自动定位)
    int is_g683_block=0;  // P1: G68.3 欧拉角倾斜面
    int is_g69_block=0;   // P1: G69 取消倾斜面
    int m_code=-1;
    double s_value=0.0;
    double t_value=0.0;  // P1': T 代码 (刀号)
    double p_value=0.0, q_value=0.0, r_value=0.0; // M代码扩展参数
    double e_value=0.0;  // P0-Laser: E 代码 (M67 激光功率 / M68 频率)
    int l_value=1;       // Phase 2B M5: M98 L<重复次数>, 默认 1
    int is_non_motion_g=0; // 非运动组拦截锁：G04/G10/G28/G92 等
    int has_f=0;           // F 值存在标志（G93 非模态校验）
    int has_r=0;           // R 值存在标志（G91 固定循环 R/Z 顺序处理）
    int is_G53_this_block=0;  // G53 非模态机械坐标：仅影响本行
    int is_g52_block=0;       // P2': G52 局部坐标系设定块, 字母循环后捕获
    int is_g04_block=0;       // Phase B2: G04 dwell 块, 字母循环后捕获并 push M64 段
    double g04_dwell_ms=0.0;  // Phase B2: G04 P<ms> 捕获值
    int is_g28_block=0;       // P0-1: G28 返回参考点, 字母循环后触发 axis_homing_multi
    int is_g65_block=0;       // P4': G65 用户宏调用, 字母循环后处理
    int is_g66_block=0;       // P4' Phase 2: G66 模态宏调用激活
    int is_g541_block=0;      // P5': G54.1 Pn 扩展 WCS, 字母循环后处理

    // P4': G65 字母参数捕获 (Fanuc Format I, 简化无 I/J/K 重复)
    // 索引 1-26 对应 #1-#26; 索引 0 不用
    double g65_args[27]     = {0};
    int    g65_args_set[27] = {0};

    // P0-b v1: 设置当前源行号 (axis_ctrl api_push_trajectory_impl 读此填 seg->line_no)
    // parser_thread 单写者, 无竞争。g_pc 可能因跳转指令变化, 每行入口刷新。
    g_current_line_no = g_current_program ?
                        g_current_program->lines[g_pc].line_no : -1;

    char *p=buffer;
    while(*p!='\0'){
        p=(char*)skip_spaces(p);
        if(*p=='\0') break;

        // ---- Phase 2B M5: O<num> 子程序标签 ----
        // 两种语义:
        //   (a) 栈顶帧 entry_line == g_pc: M98 跳入此行, O 行视为 no-op 标签, 继续执行子程序体
        //   (b) 主流程 fall-through: 跳到 o_labels[].skip_to (子程序 M99 之后)
        if(toupper((unsigned char)p[0])=='O' && isdigit((unsigned char)p[1])){
            char *end;
            long o_num = strtol(p + 1, &end, 10);
            if(end == p + 1){
                printf("[Parser] O 后缺编号 (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            // 推进主游标到行尾
            while(*end) end++;
            p = end;

            if(!g_current_program){
                printf("[Parser] O 标签在无 program 上下文\n");
                return -1;
            }
            // 判断是 M98 跳入 (栈顶 entry_line == g_pc) 还是主流程穿越
            int via_m98 = (g_call_stack_top > 0 &&
                          g_call_stack[g_call_stack_top - 1].entry_line == g_pc);
            if(via_m98){
                printf("[Parser] O%d 子程序入口 (M98 跳入, 源行 %d)\n",
                       (int)o_num, g_current_program->lines[g_pc].line_no);
                // PC++ 由主循环处理, 继续子程序体第一行
            } else {
                int idx = Program_FindOLabel(g_current_program, (int)o_num);
                if(idx < 0 || g_current_program->o_labels[idx].skip_to < 0){
                    printf("[Parser] O%d 跳转目标无效 (源行 %d)\n",
                           (int)o_num, g_current_program->lines[g_pc].line_no);
                    return -1;
                }
                g_pc_jump_target = g_current_program->o_labels[idx].skip_to;
                g_pc_jump_pending = 1;
                printf("[Parser] 主流程穿越 O%d, 跳到行索引 %d (源行 %d)\n",
                       (int)o_num, g_pc_jump_target,
                       g_current_program->lines[g_pc].line_no);
            }
            continue;
        }

        // ---- Phase 2B M3: WHILE [<cond>] DO n (块循环头部) ----
        // 求值条件: 真则进入循环体 (PC++), 假则跳到匹配 END 之后
        // WHILE 后必须非字母 (防 WHILEX 误匹配)
        if(toupper((unsigned char)p[0])=='W' && toupper((unsigned char)p[1])=='H' &&
           toupper((unsigned char)p[2])=='I' && toupper((unsigned char)p[3])=='L' &&
           toupper((unsigned char)p[4])=='E' && !isalpha((unsigned char)p[5])){
            char *w_p = p + 5;
            while(*w_p == ' ' || *w_p == '\t') w_p++;
            if(*w_p != '['){
                printf("[Parser] WHILE 后缺 '[' (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            w_p++;
            const char *cond_p = w_p;
            double cond = Evaluate_Expression(&cond_p);
            while(*cond_p == ' ' || *cond_p == '\t') cond_p++;
            if(*cond_p != ']'){
                printf("[Parser] WHILE 条件缺 ']' (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            cond_p++;
            // cond_p 此后应是 "DO n", 运行时不需解析编号 (预扫描已配对)
            // 推进主游标到行尾
            while(*cond_p) cond_p++;
            p = (char *)cond_p;

            if(!g_current_program){
                printf("[Parser] WHILE 在无 program 上下文\n");
                return -1;
            }
            int end_idx = g_current_program->do_to_end[g_pc];
            if(end_idx < 0){
                printf("[Parser] WHILE 行未配对 END (预扫描遗漏? 源行 %d)\n",
                       g_current_program->lines[g_pc].line_no);
                return -1;
            }

            if(cond != 0.0){
                printf("[Parser] WHILE [...] (条件真) 进入循环 (源行 %d)\n",
                       g_current_program->lines[g_pc].line_no);
                // PC++ 由主循环处理
            } else {
                g_pc_jump_target = end_idx + 1;
                g_pc_jump_pending = 1;
                printf("[Parser] WHILE [...] (条件假) 跳到行索引 %d (源行 %d, 跳过循环体)\n",
                       end_idx + 1, g_current_program->lines[end_idx].line_no);
            }
            continue;
        }

        // ---- Phase 2B M3: END n (块循环尾部, 无条件跳回 WHILE) ----
        if(toupper((unsigned char)p[0])=='E' && toupper((unsigned char)p[1])=='N' &&
           toupper((unsigned char)p[2])=='D' && !isalpha((unsigned char)p[3])){
            // 推进主游标到行尾 (END n 的编号运行时不需校验, 预扫描已验证)
            while(*p) p++;

            if(!g_current_program){
                printf("[Parser] END 在无 program 上下文\n");
                return -1;
            }
            int do_idx = g_current_program->end_to_do[g_pc];
            if(do_idx < 0){
                printf("[Parser] END 行未配对 DO (预扫描遗漏? 源行 %d)\n",
                       g_current_program->lines[g_pc].line_no);
                return -1;
            }
            g_pc_jump_target = do_idx;
            g_pc_jump_pending = 1;
            printf("[Parser] END 跳回 WHILE 行索引 %d (源行 %d, 重新求值条件)\n",
                   do_idx, g_current_program->lines[do_idx].line_no);
            continue;
        }

        // ---- Phase 2B M2: IF [<cond>] GOTO n (条件跳转) ----
        // 单行格式: IF [条件] GOTO 标签号
        // 条件真则触发 g_pc_jump_pending (复用 M1 的跳转信号), 假则继续下一行
        // IF 后必须非字母 (排除 IFX 等误匹配)
        if(toupper((unsigned char)p[0])=='I' && toupper((unsigned char)p[1])=='F' &&
           !(isalpha((unsigned char)p[2]))){
            char *if_p = p + 2;
            while(*if_p == ' ' || *if_p == '\t') if_p++;
            if(*if_p != '['){
                printf("[Parser] IF 后缺 '[' (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            if_p++;
            const char *cond_p = if_p;
            double cond = Evaluate_Expression(&cond_p);
            while(*cond_p == ' ' || *cond_p == '\t') cond_p++;
            if(*cond_p != ']'){
                printf("[Parser] IF 条件缺 ']' (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            cond_p++;
            while(*cond_p == ' ' || *cond_p == '\t') cond_p++;

            // 期望 "GOTO n"
            if(!(toupper((unsigned char)cond_p[0])=='G' &&
                 toupper((unsigned char)cond_p[1])=='O' &&
                 toupper((unsigned char)cond_p[2])=='T' &&
                 toupper((unsigned char)cond_p[3])=='O')){
                printf("[Parser] IF [...] 后必须跟 GOTO n (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            cond_p += 4;
            while(*cond_p == ' ' || *cond_p == '\t') cond_p++;
            char *end;
            long label = strtol(cond_p, &end, 10);
            if(end == cond_p){
                printf("[Parser] IF GOTO 后缺标签号 (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            p = end;  // 推进主游标 (IF 行通常到此结束)

            if(cond != 0.0){
                if(!g_current_program){
                    printf("[Parser] IF GOTO 在无 program 上下文\n");
                    return -1;
                }
                int target = Program_FindLabel(g_current_program, (int)label);
                if(target < 0){
                    printf("[Parser] IF GOTO %d: 未定义的 N 标签 (源行 %d)\n",
                           (int)label,
                           g_current_program->lines[g_pc].line_no);
                    return -1;
                }
                g_pc_jump_target = target;
                g_pc_jump_pending = 1;
                printf("[Parser] IF [...] GOTO %d (条件真) -> 行索引 %d (源行 %d)\n",
                       (int)label, target,
                       g_current_program->lines[target].line_no);
            } else {
                printf("[Parser] IF [...] GOTO %d (条件假, 不跳)\n", (int)label);
            }
            continue;
        }

        // ---- Phase 2B M1: GOTO 特殊词法 (4 字母关键字, 不走 letter+value 单字符解析) ----
        // 大小写不敏感匹配 "GOTO", 后跟空格 + N 标签号
        // 触发 g_pc_jump_pending 信号, parser_thread_func 主循环消费并跳转
        if(toupper((unsigned char)p[0])=='G' && toupper((unsigned char)p[1])=='O' &&
           toupper((unsigned char)p[2])=='T' && toupper((unsigned char)p[3])=='O'){
            char *gp = p + 4;
            while(*gp == ' ' || *gp == '\t') gp++;
            char *end;
            long label = strtol(gp, &end, 10);
            if(end == gp){
                printf("[Parser] GOTO 后缺标签号 (源行 %d)\n",
                       g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            p = end;  // 推进主游标 (本行 GOTO 后通常无内容, 推进到行尾)

            if(!g_current_program){
                printf("[Parser] GOTO 在无 program 上下文 (旧调用路径?), 拒绝\n");
                return -1;
            }
            int target = Program_FindLabel(g_current_program, (int)label);
            if(target < 0){
                printf("[Parser] GOTO %d: 未定义的 N 标签 (源行 %d)\n",
                       (int)label,
                       g_current_program->lines[g_pc].line_no);
                return -1;
            }
            g_pc_jump_target = target;
            g_pc_jump_pending = 1;
            printf("[Parser] GOTO %d -> 跳转到行索引 %d (源行 %d)\n",
                   (int)label, target,
                   g_current_program->lines[target].line_no);
            continue;
        }

        // 括号注释 (...) — 深度计数匹配, 支持嵌套括号与 M03(LSON) 等紧贴写法
        // @Fix 旧实现扫描到第一个 ')' 即闭合, 注释内出现嵌套 '(' 时会提前结束,
        //      注释剩余正文 (可能含 M30 / G53 / 轴地址字) 被当作真指令解析。
        //      实测: "(4. 无 0x0024 (软限位未拒绝), 无报警, 程序正常 M30)"
        //      → 泄漏 M30 提前触发安全停, 后续真实运动段全部被吞 (零段入队)。
        // @Thread-Safety 仅访问栈上指针 p 与局部 depth, 无共享状态
        if(*p=='('){
            // @Fix (2026-09-04) 真正实现深度计数 —— 旧代码仍是"扫描到第一个 ')'",
            //   注释里写了深度计数但实现没跟上 (修复曾在基线比对时被回退丢失)。
            //   真实故障: tests/gcode/test_rtcp_singular_arc.nc 第 5 行
            //     "( 本机有效 RTCP 臂 = pivot_z(200) - tool_len(150) = 50mm, B100 ..."
            //   → 在 "pivot_z(200)" 的 ')' 处提前闭合, 残留正文里的 "B100" / "X0.03"
            //     被当作真指令字解析, 污染 modal。TC-6 因 B 被预设而全程未触发
            //     奇异报警 (clean 版同参数实测 r=104.3>30 正常报警)。
            // @Safe 注释不跨行: 遇 '\n'/'\r' 强制结束 (未闭合的 '(' 不会吞掉后续真指令)。
            //   Fanuc 语义一致。TC-6 第 5 行本身就是未闭合注释 (行尾无 ')')。
            // @Thread-Safety 仅访问栈上指针 p 与局部 depth, 无共享状态。
            int depth = 0;
            while(*p != '\0' && *p != '\n' && *p != '\r'){
                if(*p == '('){
                    depth++;
                    p++;
                } else if(*p == ')'){
                    depth--;
                    p++;
                    if(depth <= 0) break;   // 配对闭合, 已消耗 ')'
                } else {
                    p++;
                }
            }
            continue;
        }
        // 分号 / 百分号：后续全部忽略
        if(*p==';' || *p=='%') break;
        // 换行符：行尾自然结束
        if(*p=='\n' || *p=='\r') break;

        // ---- Fix C: 非 ASCII 字节 (UTF-8 中文等多字节字符) 静默跳过 ----
        // 防止中文夹在词法单元中间污染 letter 语义 (如 "R中文-18" 被解析成 R=0)
        // UTF-8 首字节 (0xC0-0xFD) 和续字节 (0x80-0xBF) 均高位为 1, 此检测覆盖所有非 ASCII
        if(((unsigned char)*p) & 0x80){
            p++;
            continue;
        }

        char letter=toupper(*p);
        p++;

        // 宏表达式/变量分支：遇到 '[' 或 '#' 走递归下降求值，否则继续 strtod
        // 例：X[10*SIN[30]]、X#100、F[#101+5]
        double value;
        if(*p == '[' || *p == '#'){
            const char *cp = p;                 // const 中转，避免 char** ↔ const char** 类型冲突
            value = Evaluate_Expression(&cp);
            p = (char *)cp;
        } else {
            // ---- Fix B: strtod 失败检测 ----
            // 字母后无有效数值时 strtod 返回 0.0 且不推进 p, 旧逻辑会静默用 0 更新
            // 模态字段 (如 R=0), 在 G91 下导致 cycle_R_plane 严重错误。
            // 现改为: 检测 p 未推进 → 警告并跳过本词法单元, 不进入 switch
            const char *before = p;
            value = strtod(p, &p);
            if(p == before){
                printf("[Parser] 警告: '%c' 后无有效数值, 跳过该词法单元\n", letter);
                continue;
            }
        }

        // P4': G65/G66 用户宏参数捕获 (在 switch 之前, 与 axis 映射并行)
        // Fanuc Format I 字母→#N 映射 (跳过 G/L/N/O/P, 它们有特殊语义):
        //   A=#1, B=#2, C=#3, I=#4, J=#5, K=#6, D=#7, E=#8, F=#9,
        //   H=#11, M=#13, Q=#17, R=#18, S=#19, T=#20, U=#21, V=#22, W=#23,
        //   X=#24, Y=#25, Z=#26
        if((is_g65_block || is_g66_block) && letter >= 'A' && letter <= 'Z'){
            static const int letter_to_macro[26] = {
                1,  // A
                2,  // B
                3,  // C
                7,  // D
                8,  // E
                9,  // F
                -1, // G (skip, G 代码组)
                11, // H
                4,  // I
                5,  // J
                6,  // K
                -1, // L (skip, 重复次数)
                13, // M
                -1, // N (skip, 行号)
                -1, // O (skip, 子程序号)
                -1, // P (skip, G65 目标 O 号)
                17, // Q
                18, // R
                19, // S
                20, // T
                21, // U
                22, // V
                23, // W
                24, // X
                25, // Y
                26  // Z
            };
            int macro_n = letter_to_macro[letter - 'A'];
            if(macro_n > 0){
                g65_args[macro_n]     = value;
                g65_args_set[macro_n] = 1;
            }
        }

        switch(letter){
            case 'G':
                if(value==0.0)      { g_state.motion_mode=0; g_current_motion_type = MOTION_TYPE_RAPID; }   // G00 快速
                else if(value==1.0) { g_state.motion_mode=1; g_current_motion_type = MOTION_TYPE_LINEAR; } // G01 直线
                else if(value==2.0) { g_state.motion_mode=2; g_current_motion_type = MOTION_TYPE_ARC_CW; }  // G02 顺弧
                else if(value==3.0) { g_state.motion_mode=3; g_current_motion_type = MOTION_TYPE_ARC_CCW; } // G03 逆弧
                else if(value==4.0) { is_non_motion_g=1; is_g04_block=1; }    // G04 暂停
                else if(value==10.0) is_non_motion_g=1;    // G10 数据设定
                else if(value==17.0) g_state.active_plane=17;
                else if(value==18.0) g_state.active_plane=18;
                else if(value==19.0) g_state.active_plane=19;
                else if(value==28.0) { is_non_motion_g=1; is_g28_block=1; }    // G28 返回参考点 (P0-1)
                else if(value==40.0){
                    // G40: 取消刀具半径补偿
                    if(CutterComp_GetMode() != COMP_OFF){
                        // 刷出 B-Spline 蓄水池，保证补偿引擎的输出在 M 代码/新指令前到达
                        if(g_state.bspline_enabled) BSpline_Flush();
                        CutterComp_Disable();
                    }
                }
                else if(value==41.0) g_state.pending_comp_mode = COMP_LEFT;  // G41: 左补偿 (等待 D 值)
                else if(value==42.0) g_state.pending_comp_mode = COMP_RIGHT; // G42: 右补偿 (等待 D 值)
                else if(value==90.0) g_state.is_absolute=1;
                else if(value==91.0) g_state.is_absolute=0;
                else if(value==92.0) is_non_motion_g=1;    // G92 坐标偏移
                else if(value==53.0) is_G53_this_block=1;  // G53 非模态机械坐标
                else if(value==52.0){
                    // P2': G52 局部坐标系。具体 X/Y/Z 值在字母循环后捕获
                    is_g52_block = 1;
                    is_non_motion_g = 1;  // G52 本身不触发运动
                }
                else if(value == 65.0){
                    // P4': G65 用户宏调用 — 字母循环后处理
                    // P<Onum> 指定目标, L<重复>, A-Z 映射到子程序 #1-#26
                    is_g65_block    = 1;
                    is_non_motion_g = 1;
                }
                else if(value == 66.0){
                    // P4' Phase 2: G66 模态宏调用激活 — 字母循环后处理
                    is_g66_block    = 1;
                    is_non_motion_g = 1;
                }
                else if(value == 67.0){
                    // P4' Phase 2: G67 取消模态宏调用
                    if(g_state.modal_macro_active){
                        printf("[Parser] G67 取消模态宏调用 (源行 %d)\n",
                               g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                        g_state.modal_macro_active = 0;
                    }
                    is_non_motion_g = 1;
                }
                else if(value==93.0) g_state.feed_mode=FEED_MODE_G93; // G93 倒数时间
                else if(value==94.0) g_state.feed_mode=FEED_MODE_G94; // G94 每分钟
                // ---- P2-A-4: G09/G61/G64 精准停 ----
                // G09: 单次精准停 (仅本运动段 v_end=0, 不影响后续段模态)
                // G61: 模态精准停开启 (后续所有段 v_end=0, 直到 G64 取消)
                // G64: 模态连续切削 (默认, 拐角允许 G64 容差内过弯不归零)
                // 工业用途: G09/G61 用于拐角清根/防过切/精密定位; G64 用于高速连续加工.
                // P2-A-4: G09/G61/G64 是"运动修饰词", 必须与同行的 G00/G01 共存,
                //   不能设 is_non_motion_g=1 (否则会拦截本行 G01 的段生成, 漏掉该段运动,
                //   且 exact_stop_this_block 永不被消费而泄漏到下一行). G04/G10/G28/G92 等
                //   才是真正的非运动指令, 保留 is_non_motion_g 锁.
                else if(value==9.0)  { g_state.exact_stop_this_block = 1; }
                else if(value==61.0) { g_state.modal_exact_stop = 1; }
                else if(value==64.0) { g_state.modal_exact_stop = 0; }
                // P1 (2026-09-04): G43.5 刀轴矢量编程 — I/J/K 为刀轴单位矢量,
                // 字母循环后解算旋转角 (与 43.4 判据无重叠: 差 0.1 > 0.05)
                else if(fabs(value - 43.5) < 0.05){
                    g_state.rtcp_enabled = 1;
                    g_state.vector_mode  = 1;
                }
                else if(fabs(value - 43.4) < 0.05){
                    g_state.rtcp_enabled = 1;
                    g_state.vector_mode  = 0;   // 显式降级回角度编程
                }
                else if(value >= 49.0 && value < 50.0){
                    g_state.rtcp_enabled = 0; // G49 关闭RTCP
                    g_state.vector_mode  = 0;
                }
                // ---- P1 (2026-09-07): G68.2/G68.3 倾斜面 / G69 取消 ----
                // 参数捕获延迟到字母循环后 (G68.2 需 I/J/K/Q; G68.3 需 X/Y/Z 欧拉角)
                else if(fabs(value - 68.2) < 0.05){
                    is_g682_block   = 1;
                    is_non_motion_g = 1;
                }
                else if(fabs(value - 68.3) < 0.05){
                    is_g683_block   = 1;
                    is_non_motion_g = 1;
                }
                else if(value >= 69.0 && value < 70.0){
                    is_g69_block    = 1;
                    is_non_motion_g = 1;
                }
                else if(fabs(value - 54.1) < 0.05){
                    // P5': G54.1 Pn 扩展 WCS — 字母循环后处理 (需读 P)
                    // 必须在 G54-G59 范围检查之前匹配, 否则会被误判为 G54
                    is_g541_block    = 1;
                    is_non_motion_g = 1;
                }
                else if(value>=54.0 && value<=59.0){
                    // 切换模态 WCS — 仅写 parser 本地 g_state.modal_wcs, 不再直接污染
                    // g_coord_mgr.current_coord (那是 RT 线程消费段时的独占写者)。
                    // 先同步 Flush BSpline 脏点队列,保证本批平滑段全在旧 WCS 下捕获,
                    // 后续脏点用新 WCS — 杜绝 bspline 批次跨 WCS 边界。
                    if(g_state.bspline_enabled) BSpline_Flush();
                    g_state.modal_wcs = (CoordSystem_t)((int)value - 53); // 54->G54, 55->G55...
                    g_state.modal_ext_wcs_p = 0;  // P5': G54-G59 切换时清空 ext WCS
                }
                // ---- Phase 2A.2: 固定循环 G80/G81/G82/G83/G98/G99 ----
                else if(value == 80.0){
                    // G80: 取消固定循环
                    if(g_state.bspline_enabled) BSpline_Flush();
                    g_state.active_cycle = 0;
                }
                else if(value == 73.0 || value == 81.0 || value == 82.0 || value == 83.0){
                    // G73/G81/G82/G83: 激活固定循环
                    // 首次激活 (或 G80 后重新激活) 时捕获 cycle_initial_Z (G98 退回点)
                    int z_idx_g81 = g_axis_map['Z' - 'A'];
                    if(g_state.active_cycle == 0 && z_idx_g81 >= 0){
                        g_state.cycle_initial_Z = g_state.current_pos[z_idx_g81];
                    }
                    g_state.active_cycle = (int)value;
                }
                else if(value == 98.0) g_state.cycle_retract_mode = 98;
                else if(value == 99.0) g_state.cycle_retract_mode = 99;
                break;
            case 'F':g_state.feedrate_mm_min=value;has_f=1;break;
            case 'I':offset_i=value;has_move=1;has_ijk_word=1;break;
            case 'J':offset_j=value;has_move=1;has_ijk_word=1;break;
            case 'K':offset_k=value;has_move=1;has_ijk_word=1;break;
            case 'L':
                // Phase 2B M5: M98 L<重复次数>; 其他语境下 L 暂未使用
                if(value < 0 || value > MAX_M98_REPEAT){
                    printf("[Parser] L 值越界 (%g, 允许 0..%d, 源行 %d)\n",
                           value, MAX_M98_REPEAT,
                           g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                    return -1;
                }
                l_value = (int)value;
                break;
            case 'M':
                m_code=(int)value;
                // Phase 2B M1: 主程序中 M99 = 回到首行 (子程序 M99 行为留给 M5)
                // Phase 2B M5: 栈非空时 M99 = 子程序返回 / 重复调用
                if(m_code == 99 && g_current_program){
                    if(g_call_stack_top > 0){
                        CallFrame_t *f = &g_call_stack[g_call_stack_top - 1];
                        if(f->repeat_remaining > 0){
                            // 重复调用: 不弹栈, 不恢复 (modal/locals 保持隔离)
                            // 每轮独立: 再次清零 #1-#33
                            f->repeat_remaining--;
                            Macro_ClearLocals();
                            // P4' G65: 字母参数需在每次重复时重新应用 (Fanuc 语义:
                            // G65 L<n> 表示用相同参数调用 n 次)
                            if(f->is_g65_frame){
                                for(int n = 1; n <= 26; n++){
                                    if(f->g65_args_set[n])
                                        Macro_SetValue(n, f->g65_args[n]);
                                }
                            }
                            g_pc_jump_target  = f->entry_line;
                            g_pc_jump_pending = 1;
                            printf("[Parser] M99 重复调用 (剩 %d 次, 跳回入口 %d, 源行 %d)%s\n",
                                   f->repeat_remaining, f->entry_line,
                                   g_current_program->lines[g_pc].line_no,
                                   f->is_g65_frame ? " [G65 参数重应用]" : "");
                        } else {
                            // 真正返回: 恢复调用者 #1-#33 + modal 状态, 弹栈
                            int ret = f->return_pc;
                            Macro_SetLocals(f->saved_locals);
                            g_state = f->saved_state;
                            g_call_stack_top--;
                            g_pc_jump_target  = ret;
                            g_pc_jump_pending = 1;
                            printf("[Parser] M99 返回调用者 (PC=%d, depth=%d, 源行 %d)\n",
                                   ret, g_call_stack_top,
                                   g_current_program->lines[g_pc].line_no);
                        }
                        m_code = -1;  // 不入队 M99
                    } else {
                        // 主程序 M99: 回到首行
                        g_pc_jump_target = 0;
                        g_pc_jump_pending = 1;
                        printf("[Parser] M99 主程序循环 -> 回到首行\n");
                        m_code = -1;
                    }
                }
                // Phase 2B M5: M98 子程序调用
                // 处理推迟到字母循环结束 (需要检查严格字母 + 已收齐 P/L)
                // 这里仅标记, 不在 case 内立即处理
                // ---- P1': M 代码辅助状态机分发 ----
                // M0/M1/M2/M30 = parser 级拦截 (不入队, 不入 RT 等待)
                // M3/M4/M5/M6/M7/M8/M9/M19 = 物理 M 代码, 仅更新 g_state modal,
                //   后续 api_push_mcode 快照到 seg, RT 消费时同步 g_interpolator._rt 镜像
                // P0-2: M0 真暂停
                // 旧行为: 只设 is_paused=1, RT 队列里残留运动继续走完
                // 新行为: 加 api_motion_pause() 让 RT 用 time_scale 平滑减速到 0
                // 红线 #4: Feedhold 用 time_scale, 禁止直接置 is_moving=0 或清队列
                // 恢复路径: HMI 发 SMC_ResumeProcessing → api_motion_resume() + is_paused=0
                else if(m_code == 0){
                    // 断点续切 (2026-09-08): SEARCH 重放 / RUN 跳段区间内的 M0 属
                    // "已执行区间", 不暂停 (目标边界在其后)
                    if (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH ||
                        (g_parser_ctrl.program_mode == PROGRAM_MODE_RUN &&
                         g_parser_ctrl.resume_active &&
                         g_current_line_no < g_parser_ctrl.resume_target_line)) {
                        m_code = -1;
                    } else {
                        int src_line = g_current_program ?
                                       g_current_program->lines[g_pc].line_no : -1;
                        g_parser_ctrl.is_paused = 1;
                        api_motion_pause();
                        printf("[Parser] M0 程序暂停 (源行 %d, 已发 motion_pause)\n", src_line);
                        m_code = -1;  // 不入队
                    }
                }
                // P0-2: M1 可选停 — 受 g_optional_stop_enabled 开关控制
                // 开关关 (默认): M1 视为 no-op (兼容现有 NC 程序)
                // 开关开 (HMI 显式启用): M1 等价 M0, 走真暂停路径
                else if(m_code == 1){
                    // 断点续切: 同 M0 门控
                    if (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH ||
                        (g_parser_ctrl.program_mode == PROGRAM_MODE_RUN &&
                         g_parser_ctrl.resume_active &&
                         g_current_line_no < g_parser_ctrl.resume_target_line)) {
                        m_code = -1;
                    } else {
                        int src_line = g_current_program ?
                                       g_current_program->lines[g_pc].line_no : -1;
                        if (g_optional_stop_enabled) {
                            g_parser_ctrl.is_paused = 1;
                            api_motion_pause();
                            printf("[Parser] M1 可选停生效 (源行 %d, 已发 motion_pause)\n", src_line);
                        } else {
                            printf("[Parser] M1 optional stop 跳过 (开关关, 源行 %d)\n", src_line);
                        }
                        m_code = -1;
                    }
                }
                // P0-1: M2/M30 安全停 5 步流程
                // 旧行为: 直接 is_running=0, 后果 RT 队列里残留运动继续走、spindle/coolant 不关
                // 新行为: 平滑减速 -> 等队列空 -> 模态复位 -> 抢写 RT 镜像 -> 关 parser
                // 抢写 RT 镜像是 RT 单写者例外: 此时队列已空, RT 不会再消费任何 M 段,
                // 写者已 quiescent, parser 安全抢写一次 (与 api_motion_resume 同级别操作)
                else if(m_code == 2 || m_code == 30){
                    // 断点续切 (2026-09-08):
                    //   SEARCH 中途遇 M2/M30 → 目标行不可达 (原程序到此为止),
                    //     result=3 终止搜索 (ResumeSearch 返回 -5)
                    //   RUN 跳段区间 (line<target) 的 M2/M30 不触发安全停 —
                    //     SEARCH 已保证该情形不可达, 此处防御性跳过
                    if (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH) {
                        atomic_store_explicit(&g_program_resume_result, 3,
                                              memory_order_release);
                        m_code = -1;
                    } else if (g_parser_ctrl.program_mode == PROGRAM_MODE_RUN &&
                               g_parser_ctrl.resume_active &&
                               g_current_line_no < g_parser_ctrl.resume_target_line) {
                        m_code = -1;
                    }
                    else {
                    int src_line = g_current_program ?
                                   g_current_program->lines[g_pc].line_no : -1;
                    printf("[Parser] M%d 程序结束 → 安全停流程启动 (源行 %d)\n",
                           m_code, src_line);

                    // Step 1 (修订): 不调 api_motion_pause
                    //
                    // 原设计: api_motion_pause() → time_scale 减到 0 → RT 平滑减速
                    // 致命 bug: time_scale=0 → ms_budget=0 → RT 不消费队列
                    //   → is_trajectory_finished() 永远 false
                    //   → Step 2 等 Q 空永远等不到, 30s 超时强制收尾
                    //   → CSV 轨迹严重截断, 后续段全部丢失
                    //
                    // 新设计: 信任 planner 的 S 曲线, 每段 v_end 已含减速到 0
                    //   (除非连接段, 但连接段也被下一段的 S 曲线消费掉)
                    //   RT 自然消费完所有段, is_moving=0, is_trajectory_finished=真
                    //
                    // 适用场景: M30/M2 是程序末尾, 所有段已 push 完
                    //   不需要"平滑减速"(planner 已经做了)

                    // Step 2: 等队列排空 + RT 自然停下 (100ms 轮询, 30s 超时)
                    int wait_ms = 0;
                    while (!is_trajectory_finished() && wait_ms < 30000) {
                        if (g_parser_ctrl.abort_request) break;
                        osal_usleep(100000);
                        wait_ms += 100;
                    }
                    if (wait_ms >= 30000) {
                        printf("[Parser][WARN] M%d 安全停超时 (队列 30s 未空), 强制收尾\n",
                               m_code);
                    }

                    // Step 3: parser 模态复位 (spindle/coolant 全关 + 修复 P1 跨程序泄漏)
                    // 2026-11-25: 复位体抽为 gcode_reset_program_modals(), 与程序中止收尾共用。
                    gcode_reset_program_modals();

                    // P2-A: M30/M2 重置 override (默认行为, 除非 OVERRIDE_PERSIST 置位)
                    // 工业惯例: 每次新程序从 100% 开始, 防上次调机残留倍率影响下一件.
                    // UI 想跨程序保留 (调试场景) 可设 SMC_MODE_OVERRIDE_PERSIST (bit4).
                    // 此处直接写 g_interpolator (parser 已等队列空, RT 不会同时写 override).
                    // @BugFix: 仅 RUN 模式 (真实程序结束) 重置. PREVIEW 仅是结构解析,
                    //   不应对"尚未运行的程序"动手, 否则会抹掉操作员 Load 前设的倍率
                    //   (典型工作流: SetOverride → LoadProgram → RunLoadedProgram).
                    if (g_parser_ctrl.program_mode == PROGRAM_MODE_RUN) {
                        if (!(g_interpolator.mode_flags & SMC_MODE_OVERRIDE_PERSIST)) {
                            g_interpolator.feed_override_ratio    = 1.0;
                            g_interpolator.rapid_override_ratio   = 1.0;
                            g_interpolator.spindle_override_ratio = 1.0;
                        }
                        // 运行模式标志 (single block / dry run) 随 M30 程序复位一并清零,
                        // 保证"程序结束 = 干净起点" (也满足 T11 期望 mode_flags=0).
                        g_interpolator.mode_flags = 0;
                    }
                    // 模态精准停也要重置 (G61 不应跨程序)
                    g_state.modal_exact_stop       = 0;
                    g_state.exact_stop_this_block  = 0;

                    // Step 4: 抢写 RT 镜像, HMI 立即可见 spindle/coolant/laser 已停
                    g_interpolator.spindle_mode_rt  = 0;
                    g_interpolator.spindle_rpm_rt   = 0.0;
                    g_interpolator.coolant_state_rt = 0;
                    // P0-Laser RT 镜像抢停 (RT 段消费链路自然刷新, 此处只做"立即生效")
                    g_interpolator.laser_enable_rt  = 0;
                    g_interpolator.laser_shutter_rt = 0;
                    g_interpolator.laser_power_w_rt = 0.0;
                    g_interpolator.laser_freq_hz_rt = 0.0;
                    g_interpolator.gas_select_rt    = 0;
                    // P0-Laser: 同步抢写 g_laser_rt (sim_engine_push 直接读 g_laser_rt,
                    // 不读 g_interpolator.laser_*_rt — 仅抢写后者会导致 CSV 末尾保留旧值).
                    // 安全性: M30 Step 2 已等队列空, RT 不会再消费任何段调 apply_aux,
                    // 此时 g_laser_rt 写者已 quiescent, parser 抢写无竞争 (与抢写
                    // g_interpolator 同级别操作).
                    g_laser_rt.enable     = 0;
                    g_laser_rt.shutter    = 0;
                    g_laser_rt.power_w    = 0.0;
                    g_laser_rt.freq_hz    = 0.0;
                    g_laser_rt.gas_select = 0;
                    g_laser_rt.gas_pressure_bar = 0.0;   // P0-2: 气压同步归零
                    // P0-Laser: 触发 RT 下个 cycle 强制记录一条 sim_engine 采样,
                    // 让 CSV 末尾出现归零状态 (否则 should_log=0 → sim_engine_push 不调).
                    atomic_store_explicit(&g_sim_force_log, 1, memory_order_release);

                    // Step 5: 关 parser 主循环
                    g_parser_ctrl.is_running = 0;
                    printf("[Parser] M%d 安全停完成 (耗时 %.1fs)\n",
                           m_code, wait_ms / 1000.0);
                    m_code = -1;   // 不入队
                    }  // 断点续切门控 else 块结束
                }
                // P0-3: M3/M4 S 值保护 — 拒绝 spindle_rpm<=0 的主轴启动
                // 后果场景: 操作员忘写 S, M3 入队后 RT 等 2 秒主轴"启动"(实际没转),
                //           后续 G1 加工撞刀。
                // ⚠ 本分支原在此处立即检查 rpm, 但 "M3 S1000" 同行时 case 'S' 在
                //   case 'M' 之后解析, g_state.spindle_rpm 还是旧值 (0), 误报 ALARM.
                //   修复: 实际 rpm 检查 + modal 更新延迟到字母循环结束后统一处理
                //   (见本函数末尾 "M3/M4 deferred" 块). 此处仅保留 else-if 占位
                //   保证链完整, m_code 保持 3/4 不变.
                else if(m_code == 3 || m_code == 4){
                    // deferred — 见 post-letter-loop 处理
                }
                else if(m_code == 5){
                    g_state.spindle_mode = 0;
                    g_state.spindle_rpm  = 0.0;
                    // P0-Laser: M5 联动关激光闸 (主轴/激光模式都关, 安全兜底)
                    if (g_laser_cfg.do_slave_id >= 0) {
                        g_state.laser_shutter_pending = 0;
                    }
                }
                else if(m_code == 7){ g_state.coolant_state |= 0x2; }  // 置 mist 位 (bit1)
                else if(m_code == 8){ g_state.coolant_state |= 0x1; }  // 置 flood 位 (bit0)
                else if(m_code == 9){ g_state.coolant_state  = 0x0; }  // M9 全清
                else if(m_code == 6){
                    // M6 换刀: T 字母已通过 case 'T' 写入 g_state.current_tool_id
                    // 实际 ATC 物理动作留硬件阶段; sim 中仅入队触发 RT 等待 + 状态同步
                    printf("[Parser] M6 换刀 -> T%d (源行 %d)\n",
                           g_state.current_tool_id,
                           g_current_program ? g_current_program->lines[g_pc].line_no : -1);
                }
                else if(m_code == 19){
                    // M19 主轴定向: sim 中视为 M5 等价 (无实际角度控制硬件)
                    g_state.spindle_mode = 0;
                }
                // ============ P0-Laser: 激光器同步 M 代码 ============
                // M62 P0:  激光闸同步 ON  (与下一段运动段边界 1ms 对齐)
                // M63 P0:  激光闸同步 OFF
                // M67 E<W>: 同步激光功率模拟量 (0..g_laser_cfg.power_max_w)
                //           — 注: E 值检查在 case 'E' 内处理 (字母循环顺序)
                // M68 E<Hz>: 同步激光频率模拟量 — 同上, case 'E' 内处理
                // M10/M11/M12: 切换 N2/O2/Air 气体阀 (互斥)
                // 设计: 这些 M 代码在激光模式 (do_slave_id >= 0) 下生效, 否则静默忽略
                //       (CAM 常输出 M62/M63 作通用同步输出, 无硬件时安全 no-op)
                // 入队: 默认走 M 段路径 (CMD_TYPE_MCODE), RT 消费时 freeze 插补,
                //       保证 M62/M63 与后续运动段边界严格对齐.
                //
                // ============ Phase B1: 功率-速度耦合 M 代码 ============
                // M70 P<0/1>: 切换耦合模式 (0=off, 1=查表耦合)
                // M71 P<v_thresh>: 设置低速阈值 (mm/s)
                // 注: P 值处理在字母循环之后 (line 1406+ if(m_code>=0) 块内) —
                //     字母循环顺序保证 P 在 M 之后解析, 此处只识别 m_code 不读 p_value
                //     (避免字母顺序 BUG: case 'M' 内读 p_value 时 P 字母尚未解析)
                else if(m_code == 70 || m_code == 71 || m_code == 77){
                    // 模态 M 代码, m_code 保持原值, 处理延迟到字母循环后 (与 M50 同模式)
                }
                else if(m_code == 62 || m_code == 63 || m_code == 67 ||
                        m_code == 68 || m_code == 10 || m_code == 11 ||
                        m_code == 12 || m_code == 13){
                    if(g_laser_cfg.do_slave_id < 0){
                        // 未配置激光: 静默忽略 (不入队, 不报警)
                        m_code = -1;
                    } else if(m_code == 62){
                        g_state.laser_shutter_pending = 1;
                    } else if(m_code == 63){
                        g_state.laser_shutter_pending = 0;
                    } else if(m_code == 67 || m_code == 68){
                        // M67/M68 的 e_value 检查延迟到 case 'E' (字母循环内 E 在 M 后解析).
                        // 这里不做事, m_code 保持原值, 等待字母循环到 E 时设置 pending.
                        // 若本行无 E 字母 (单独 M67), pending 保持上一次值 (Fanuc 模态语义).
                    }
                    else if(m_code == 10){ g_state.gas_select = 1; }  // N2
                    else if(m_code == 11){ g_state.gas_select = 2; }    // O2
                    else if(m_code == 12){ g_state.gas_select = 3; }    // Air
                    // P0-2: M13 P<bar> 气压值在 case 'P' 校验+写 pending
                    // (字母循环 M→P 顺序, 与 M67/M68 的 E 字段同模式; 无 P 保持模态)
                }
                // ---- Laser Phase B4: 引线/微连接段标记 (modal, M72-M75) ----
                // @Context: M72-M75 是 CAM 段级工艺标记, 不依赖激光硬件配置 (do_slave_id).
                //   仅切换 g_state.laser_seg_flags 位, 入队时快照到 seg_flags.
                //   M72/M73 = lead-in start/end; M74/M75 = micro-joint start/end.
                //   M30/M2 程序结束统一清零 (见 M30 modal 重置块).
                // @Danger: 原实现将 M72-M75 误嵌套在 M62-12 条件块内部, 导致永不执行 (已修复).
                // @Thread-Safety: parser 单线程写, axis_ctrl 入队时原子读快照.
                else if(m_code == 72){ g_state.laser_seg_flags |= SEG_FLAG_LEAD_IN;   m_code = -1; }
                else if(m_code == 73){ g_state.laser_seg_flags &= ~SEG_FLAG_LEAD_IN;  m_code = -1; }
                else if(m_code == 74){ g_state.laser_seg_flags |= SEG_FLAG_MICRO_JOINT; m_code = -1; }
                else if(m_code == 75){ g_state.laser_seg_flags &= ~SEG_FLAG_MICRO_JOINT; m_code = -1; }
                // ============ P0-Laser 结束 ============
                break;
            case 'D':
                // D 代码: 刀具半径补偿值 (mm)
                // 当 G41/G42 已在本行声明时，配合 D 值激活补偿
                if(g_state.pending_comp_mode != COMP_OFF){
                    // 刷出 B-Spline 蓄水池，保证补偿引擎输出顺序
                    if(g_state.bspline_enabled) BSpline_Flush();
                    CutterComp_Enable(g_state.pending_comp_mode, value);
                    g_state.pending_comp_mode = COMP_OFF;
                }
                break;
            case 'P':
                p_value=value;
                if(g_state.active_cycle == 82) g_state.cycle_dwell_ms = value;
                // Phase B2: G04 P<ms> 捕获 (与 G82 dwell 字段独立, 避免模态污染)
                if(is_g04_block) g04_dwell_ms = value;
                // P0-2 (2026-09-03): M13 P<bar> 辅助气压 (与 M67 E 同校验模式)
                // 量程默认 0 → 未 RPC 配置 (0x0067) 时任何 P>0 都报警 (防呆)
                if(m_code == 13){
                    if(value < 0.0 || value > g_laser_cfg.pressure_max_bar){
                        int src_line = g_current_program ?
                                       g_current_program->lines[g_pc].line_no : -1;
                        printf("[Parser][ALARM] M13 P=%.2f 越界 (0-%.2f bar), 源行 %d\n",
                               value, g_laser_cfg.pressure_max_bar, src_line);
                        atomic_store_explicit(&g_sys_alarm_state, 1,
                                              memory_order_release);
                        EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0021,
                                         (int32_t)(value * 1000),
                                         "M13 gas pressure out of range");
                        m_code = -1;
                    } else {
                        g_state.laser_gas_pressure_pending = value;
                    }
                }
                break;
            case 'Q':
                q_value=value;
                // P3': G73/G83 共用 cycle_peck_depth (Q, 啄钻步进)
                if(g_state.active_cycle == 83 || g_state.active_cycle == 73)
                    g_state.cycle_peck_depth = value;
                break;
            case 'R':
                r_value=value;
                has_r=1;
                // 注: cycle_R_plane 的捕获延迟到运动门控分支
                // (G91 模式下 R 相对 cycle_initial_Z, 需先确定 G90/G91 再转换)
                break;
            case 'S':
                s_value=value;
                g_state.spindle_rpm = value;  // P1': 立即更新 modal, M3/M4 时 RT 镜像同步生效
                break;
            case 'E':
                // P0-Laser: E 字母值 (M67 功率 W / M68 频率 Hz)
                // 字母循环顺序保证 E 在 M 之后解析 (例如 "M67 E1500"), 此处 e_value
                // 已被设置, 可以安全检查越界并更新 pending 字段. case 'M' 内的 M67/M68
                // 分支已删除 (那里访问 e_value 会拿到 0.0 — 字母顺序 BUG).
                e_value = value;
                if (m_code == 67) {
                    if (e_value < 0.0 || e_value > g_laser_cfg.power_max_w) {
                        int src_line = g_current_program ?
                                       g_current_program->lines[g_pc].line_no : -1;
                        printf("[Parser][ALARM] M67 E=%.2f 越界 (0-%.0f W), 源行 %d\n",
                               e_value, g_laser_cfg.power_max_w, src_line);
                        atomic_store_explicit(&g_sys_alarm_state, 1,
                                              memory_order_release);
                        EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0021,
                                         (int32_t)(e_value * 1000),
                                         "M67 laser power out of range");
                        m_code = -1;
                    } else {
                        g_state.laser_power_pending = e_value;
                    }
                } else if (m_code == 68) {
                    if (e_value < 0.0 || e_value > g_laser_cfg.freq_max_hz) {
                        int src_line = g_current_program ?
                                       g_current_program->lines[g_pc].line_no : -1;
                        printf("[Parser][ALARM] M68 E=%.2f 越界 (0-%.0f Hz), 源行 %d\n",
                               e_value, g_laser_cfg.freq_max_hz, src_line);
                        atomic_store_explicit(&g_sys_alarm_state, 1,
                                              memory_order_release);
                        EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0021,
                                         (int32_t)(e_value),
                                         "M68 laser freq out of range");
                        m_code = -1;
                    } else {
                        g_state.laser_freq_pending = e_value;
                    }
                }
                break;
            case 'T':
                // P1': T 代码 — 立即更新 modal 刀号; M6 时 RT 镜像同步
                t_value = value;
                g_state.current_tool_id = (int)value;
                break;
            default:
                // 动态轴映射：任何 A-Z 字母若在 g_axis_map 中有映射则视为运动轴
                if(letter >= 'A' && letter <= 'Z'){
                    int idx = g_axis_map[letter - 'A'];
                    if(idx >= 0 && idx < AXIS_NUM){
                        val_axis[idx] = value;
                        has_move = 1;
                        has_axis[idx] = 1;
                        // 注: Z 值到 cycle_Z_bottom 的捕获延迟到运动门控分支
                        // (G91 模式下 Z 相对 R 平面, 需 G90/G91 转换 + R 先于 Z 处理)
                    }
                    // 未映射字母静默忽略（可能是注释残留或非标指令）
                }
                break;
        }
    }

    // ---- P1 (2026-09-04): G43.5 刀轴矢量 — 字母循环后姿态解算 ----
    // 仲裁顺序固定: ①圆弧禁止(先于冲突, 防弧 IJK 误触冲突) ②IJK×旋转轴字冲突
    // ③零矢量 ④闭式解算 ⑤写入 val_axis/has_axis。
    // 解算角写入后天然流经 wrap 最短路径 + 软限位 0x0024 + 奇异点 0x0026 管线
    // (P0 三件套即解算器的安全网); has_move 已由 case 'I' 置 1, 走标准运动门控。
    // 纯 IJK 行 (无 XYZ) = 姿态重定位段 (逻辑零位移走退化分支, 不触奇异报警)。
    if (g_state.vector_mode && !is_g65_block && !is_g66_block
        && !is_g682_block && !is_g683_block && !is_g69_block) {
        int src_line = g_current_program ? g_current_program->lines[g_pc].line_no : -1;
        if (g_state.motion_mode == 2 || g_state.motion_mode == 3) {
            g435_vector_alarm(3, src_line);          // 圆弧在矢量模态下禁止
            return -1;
        }
        if (has_ijk_word) {
            int r1 = g_kin_config.rot_1_idx;
            int r2 = g_kin_config.rot_2_idx;
            if (r1 < 0 || r2 < 0) {                  // 运动学未配置 (KIN_NONE)
                g435_vector_alarm(0, src_line);
                return -1;
            }
            if (has_axis[r1] || has_axis[r2]) {      // IJK 与 B/C 轴字互斥
                g435_vector_alarm(1, src_line);
                return -1;
            }
            double mag = sqrt(offset_i * offset_i + offset_j * offset_j
                              + offset_k * offset_k);
            if (mag < 1e-9) {
                g435_vector_alarm(2, src_line);
                return -1;
            }
            double u[3]     = { offset_i, offset_j, offset_k };
            // P1 G68.2 联动: 倾斜面激活时矢量按倾斜面系解释 → 变换到机床系再解算
            // (法向 I0 J0 K1 = 垂直斜面, 符合 3+2 直觉)
            if (g_coord_mgr.tilt_active) {
                double u_m[3];
                tilt_apply_rot(u, u_m);
                u[0] = u_m[0]; u[1] = u_m[1]; u[2] = u_m[2];
            }
            double cur1     = g_state.current_pos[r1];
            double cur2     = g_state.current_pos[r2];
            double s1 = 0.0, s2 = 0.0;
            int n_solved = 0;
            if (Kinematics_SolveOrientation(u, cur1, cur2, &s1, &s2, &n_solved)
                != KIN_ORIENT_OK) {
                g435_vector_alarm(0, src_line);      // 不可达 / 构型不支持
                return -1;
            }
            // 矢量是绝对姿态语义; G91 下转增量 (等效 target = s)
            // Cross-Semantics Pose 实验 C1 (MODE=0): 解算输出受控偏差
            // (declared≠executed)。δ 之后流经完整正常管线 (wrap/软限位/RTCP
            // 逆解) — TCP 数学保持。MODE=1 (C2) 时此处不注入 (0x003B 事件与
            // 注入移至 apply_rtcp_to_pos 尾部)。
            if (g_pose_inject_mode == 0)
                s1 += pose_inject_delta_deg(src_line);
            val_axis[r1] = g_state.is_absolute ? s1 : (s1 - cur1);
            has_axis[r1] = 1;
            if (n_solved >= 2) {
                val_axis[r2] = g_state.is_absolute ? s2 : (s2 - cur2);
                has_axis[r2] = 1;
            }
            printf("[Parser] G43.5 矢量解算: I%.4f J%.4f K%.4f → %s=%.3f%s (chain=%d)\n",
                   offset_i, offset_j, offset_k,
                   g_axis[r1].axis_name, s1,
                   (n_solved >= 2) ? "" : " [rot2 保持]", g_kin_config.chain_order);
            if (n_solved >= 2) {
                printf("[Parser] G43.5            %s=%.3f (双解取近)\n",
                       g_axis[r2].axis_name, s2);
            }
            // Cross-Semantics Pose 实验: declared u_tool witness — 事件 0x002B
            // message 记 G 代码声明的原始 I/J/K (declared) + 解算输出角 (含注入,
            // 即 parser 域的 executed 代理; RT 域 native executed witness = CSV
            // B/C 列与 u_tool_* 列)。value = rot1 毫度。
            {
                char ev[64];
                if (n_solved >= 2)
                    snprintf(ev, sizeof(ev), "G43.5 u=(%.3f,%.3f,%.3f) %s=%.3f %s=%.3f",
                             offset_i, offset_j, offset_k,
                             g_axis[r1].axis_name, s1,
                             g_axis[r2].axis_name, s2);
                else
                    snprintf(ev, sizeof(ev), "G43.5 u=(%.3f,%.3f,%.3f) %s=%.3f [r2h]",
                             offset_i, offset_j, offset_k,
                             g_axis[r1].axis_name, s1);
                EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x002B,
                                 (int32_t)(s1 * 1000.0), ev);
            }
            offset_i = offset_j = offset_k = 0.0;    // 防御性清残留
        }
    }

    // ---- P2': G52 局部坐标系捕获 (字母循环结束后) ----
    // Fanuc 语义: G52 X_ Y_ Z_ 设定局部偏置绝对值, 未指定轴归零
    // 全零 (X0 Y0 Z0) 视为取消 local_offset_active
    if(is_g52_block){
        for(int i = 0; i < AXIS_NUM; i++){
            g_state.local_offset[i] = has_axis[i] ? val_axis[i] : 0.0;
        }
        int all_zero = 1;
        for(int i = 0; i < AXIS_NUM; i++){
            if(fabs(g_state.local_offset[i]) > 1e-9){ all_zero = 0; break; }
        }
        g_state.local_offset_active = !all_zero;
        printf("[Parser] G52 局部坐标系 %s\n",
               g_state.local_offset_active ? "激活" : "取消");
    }

    // ---- P1 (2026-09-07): G68.2/G68.3 倾斜面捕获 + 旋转轴自动定位 ----
    // 顺序在 G52 之后: 倾斜面原点在捕获后的总平移 (w+G52+origin) 中生效。
    // 自动定位复用 G43.5 姿态解算器 (法向 = R 第 3 列), 定位完成后再激活 —
    // 定位段本身不套 R (激活前), 语义对齐 CYCLE800。
    // @Context: Non-RealTime Background Thread @Safe: 阻塞等待 + 数学允许
    if(is_g682_block || is_g683_block || is_g69_block){
        int src_line = g_current_program ? g_current_program->lines[g_pc].line_no : -1;
        int wcs_idx = (g_state.modal_wcs >= COORD_G54 &&
                       g_state.modal_wcs <= COORD_G59) ? (g_state.modal_wcs - 1) : -1;

        if(is_g69_block){
            if(g_coord_mgr.tilt_active){
                g_coord_mgr.tilt_active = 0;
                printf("[Parser] G69: 倾斜面已取消\n");
                EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x002A, 0,
                                 "tilt plane cancelled");
            }
            return 0;   // 本行结束, 不走运动门控
        }

        double R[9];
        int build_ok = 1;
        if(is_g682_block){
            double n_in[3] = { offset_i, offset_j, offset_k };
            build_ok = (Kinematics_BuildTiltFrame(n_in, q_value, R) == 0);
        } else {
            // G68.3: X/Y/Z 为固定角 (extrinsic X→Y→Z, 度), 直接读 val_axis
            int xi = g_axis_map['X' - 'A'];
            int yi = g_axis_map['Y' - 'A'];
            int zi = g_axis_map['Z' - 'A'];
            if(xi < 0 || yi < 0 || zi < 0){ g682_alarm(1, src_line); return -1; }
            Kinematics_BuildEulerXYZ(has_axis[xi] ? val_axis[xi] : 0.0,
                                     has_axis[yi] ? val_axis[yi] : 0.0,
                                     has_axis[zi] ? val_axis[zi] : 0.0, R);
        }
        if(!build_ok){ g682_alarm(1, src_line); return -1; }

        // 倾斜面原点 (仅 G68.2 有; G68.3 的 XYZ 已用作欧拉角)
        for(int i = 0; i < AXIS_NUM; i++){
            g_coord_mgr.tilt_origin[i] =
                (is_g682_block && has_axis[i] && g_axis[i].axis_type != 1)
                ? val_axis[i] : 0.0;
        }

        // ---- 自动定位: 解算法向姿态并生成 G0 级定位段 ----
        int r1 = g_kin_config.rot_1_idx;
        int r2 = g_kin_config.rot_2_idx;
        if(r1 < 0 || r2 < 0){ g682_alarm(0, src_line); return -1; }
        double n_machine[3] = { R[2], R[5], R[8] };   // R 第 3 列 = 法向在机床系的像
        double cur1 = g_state.current_pos[r1];
        double cur2 = g_state.current_pos[r2];
        double s1 = 0.0, s2 = 0.0;
        int n_solved = 0;
        if(Kinematics_SolveOrientation(n_machine, cur1, cur2, &s1, &s2, &n_solved)
           != KIN_ORIENT_OK){
            g682_alarm(0, src_line); return -1;
        }
        // 定位段目标 (机械域): 线性轴取实际机械现位 (api_get_cursor = planner 游标,
        // 倾斜面已激活时换面也不丢旧旋转/原点量), 旋转轴取解算绝对角
        {
            double pos_target[AXIS_NUM];
            for(int i = 0; i < AXIS_NUM; i++){
                pos_target[i] = api_get_cursor(i);
            }
            pos_target[r1] = s1;
            if(n_solved >= 2) pos_target[r2] = s2;
            if(api_push_trajectory(pos_target, 6000.0, DEFAULT_ACC, DEFAULT_DEC) < 0){
                g682_alarm(4, src_line); return -1;   // 软限位/队列拒绝
            }
        }
        // 同步等待定位完成 (G28 先例: parser 阻塞, RT 持续消费)
        // P1: 改用 wait_motion_settle — 先等 is_moving 置 1 再等归 0。原写法在 RT
        //     尚未取段时立即返回, 下一行 (如被禁的 G02) 会抢先报警锁存, RT 放弃
        //     已入队的定位段 → TC-4/TC-5 的 CSV 零数据行 (spec 要求定位段已执行)。
        wait_motion_settle();
        api_sync_planner_cursor();
        // 旋转轴逻辑位同步 (s 为机械绝对角, logical = s − w)
        for(int k = 0; k < 2; k++){
            int r = (k == 0) ? r1 : r2;
            if(r < 0) continue;
            if(k == 1 && n_solved < 2) continue;
            double w;
            if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][r];
            } else {
                w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][r] : 0.0;
            }
            if(g_state.local_offset_active) w += g_state.local_offset[r];
            g_state.current_pos[r] = ((k == 0) ? s1 : s2) - w;
        }

        // ---- 激活 ----
        memcpy(g_coord_mgr.tilt_rot, R, sizeof(double) * 9);
        g_coord_mgr.tilt_active = 1;
        printf("[Parser] %s 倾斜面激活: 法向=(%.3f,%.3f,%.3f) Q=%.1f%s\n",
               is_g682_block ? "G68.2" : "G68.3",
               n_machine[0], n_machine[1], n_machine[2], q_value,
               (n_solved >= 2) ? "" : " [rot2 保持]");
        EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x0029,
                         (int32_t)((s1 + s2) * 10.0), "tilt plane activated");
        return 0;   // 不走运动门控
    }

    // ---- Phase 2B M5: M98 子程序调用 (字母循环结束后统一处理) ----
    // 严格字母检查: M98 行只允许 P (子程序号) 和 L (重复次数)
    // 任何轴字母 (X/Y/Z/A/B/C/I/J/K/R/D/F/S...) 出现 → 报错
    // L=0 视为 no-op (Fanuc 0i/30i 标准)
    if(m_code == 98){
        if(has_move){
            printf("[Parser] M98 行不允许运动指令字母 (源行 %d)\n",
                   g_current_program ? g_current_program->lines[g_pc].line_no : -1);
            return -1;
        }
        if(!g_current_program){
            printf("[Parser] M98 在无 program 上下文\n");
            return -1;
        }
        if(l_value == 0){
            // L=0 no-op: 不调用, M98 行视为空操作
            printf("[Parser] M98 P%d L0 no-op (源行 %d)\n",
                   (int)p_value, g_current_program->lines[g_pc].line_no);
            return 0;
        }
        if(p_value < 1.0){
            printf("[Parser] M98 缺 P<Onum> 或 P<1 (源行 %d)\n",
                   g_current_program->lines[g_pc].line_no);
            return -1;
        }
        int o_idx = Program_FindOLabel(g_current_program, (int)p_value);
        if(o_idx < 0){
            printf("[Parser] M98 P%d 未找到子程序 (源行 %d)\n",
                   (int)p_value, g_current_program->lines[g_pc].line_no);
            return -1;
        }
        if(g_call_stack_top >= MAX_CALL_DEPTH){
            printf("[Parser] M98 子程序嵌套超 %d (源行 %d)\n",
                   MAX_CALL_DEPTH, g_current_program->lines[g_pc].line_no);
            return -1;
        }
        int entry = g_current_program->o_labels[o_idx].line_idx;
        // 压栈: 保存调用者 PC, 入口, 剩余次数, 局部变量, modal 状态
        CallFrame_t *f = &g_call_stack[g_call_stack_top];
        f->return_pc        = g_pc + 1;
        f->entry_line       = entry;
        f->repeat_remaining = l_value - 1;
        Macro_GetLocals(f->saved_locals);
        f->saved_state      = g_state;
        f->is_g65_frame     = 0;  // P4': M98 帧不走 G65 字母重应用路径
        g_call_stack_top++;
        // 现代语义: 子程序看到的 #1-#33 是清零的副本 (隔离)
        Macro_ClearLocals();
        // 跳到子程序入口 (O 行)
        g_pc_jump_target  = entry;
        g_pc_jump_pending = 1;
        printf("[Parser] M98 P%d L%d 调用子程序 (entry=%d, depth=%d, 源行 %d)\n",
               (int)p_value, l_value, entry, g_call_stack_top,
               g_current_program->lines[g_pc].line_no);
        return 0;  // 不入 mcode 队列
    }

    // ---- P0-1: G28 返回参考点 (字母循环结束后处理) ----
    // Fanuc 风格 v2 (B4, 2026-07-23): 若 G28 带 X_Y_Z_ 中间点, 先入队运动段到中间点,
    //   wait_motion_done 后再 axis_homing_multi. 中间点用于避免回零路径撞夹具/压板.
    //   无中间点时退化为 LinuxCNC 风格 v1 (直接 axis_homing_multi).
    //
    // 中间点坐标系: 当前模态 WCS (G54-G59) 或 G53 (本行带 G53), 与普通运动段一致
    // 中间点速度: 用 g_state.feedrate_mm_min (last_F), 与 Fanuc 行为一致
    // 调 axis_homing_multi 阻塞 parser_thread, 期间 RT 冻结 motion queue
    // G28 后续段在 homing 完成后继续解析 (v2: homing_shift 已重新锚定, home_offset 常量不动)
    if(is_g28_block){
        if(!g_homing_cfg.enabled || g_homing_cfg.order_count == 0){
            printf("[Parser] G28: homing 未配置, 调 SMC_ConfigHomingAll 先\n");
            atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
            return -1;
        }

        // B4 (2026-07-23): 检查是否带中间点 (has_axis[] 任一字母)
        int has_intermediate = 0;
        for(int i = 0; i < AXIS_NUM; i++){
            if(has_axis[i]) { has_intermediate = 1; break; }
        }

        if(has_intermediate){
            // WCS 偏置查询 (复用 line 1636-1683 逻辑)
            int wcs_idx = (g_state.modal_wcs >= COORD_G54 &&
                           g_state.modal_wcs <= COORD_G59)
                          ? (g_state.modal_wcs - 1) : -1;

            double target_pos[AXIS_NUM];
            double machine_target_pos[AXIS_NUM];
            double w_arr[AXIS_NUM];
            for(int i = 0; i < AXIS_NUM; i++){
                double w;
                if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                    w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][i];
                } else {
                    w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
                }
                if(g_state.local_offset_active && wcs_idx >= 0) w += g_state.local_offset[i];
                // P1 G68.2: 倾斜面原点并入总平移 (与 snapshot_wcs_offset 同步)
                if(g_coord_mgr.tilt_active && wcs_idx >= 0)
                    w += g_coord_mgr.tilt_origin[i];
                w_arr[i] = w;

                double start_pos = g_state.current_pos[i];
                double machine_start = start_pos + w;
                double machine_target;

                if(is_G53_this_block){
                    // G53 非模态: val_axis 是机械绝对坐标, 强制 G90
                    machine_target = has_axis[i] ? val_axis[i] : machine_start;
                    target_pos[i] = machine_target - w;
                } else if(g_state.is_absolute){
                    // G90: val_axis 是 WCS 逻辑坐标
                    target_pos[i] = has_axis[i] ? val_axis[i] : start_pos;
                    machine_target = target_pos[i] + w;
                    // P0: 旋转轴最短路径 (G28 中间点也是运动目标, 与主路径同语义)
                    if(has_axis[i] && g_axis[i].axis_type == 1 && g_axis[i].rot_wrap_enable){
                        int wrapped = 0;
                        double m_tgt = wrap_rotary_shortest(i, machine_start, machine_target, &wrapped);
                        if(wrapped){
                            char ev_buf[64];
                            snprintf(ev_buf, sizeof(ev_buf), "G28 mid %s wrap %.2f -> %.2f",
                                     g_axis[i].axis_name, machine_target, m_tgt);
                            machine_target = m_tgt;
                            target_pos[i] = m_tgt - w;
                            EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x0025, i, ev_buf);
                        }
                    }
                } else {
                    // G91: val_axis 是增量
                    target_pos[i] = start_pos + (has_axis[i] ? val_axis[i] : 0);
                    machine_target = target_pos[i] + w;
                }
                machine_target_pos[i] = machine_target;
            }
            // P1 G68.2: 倾斜面激活时中间点 XYZ 套旋转 (Fanuc 语义: 中间点经 WCS)
            // G53 子分支 (机械绝对) 正向直通不套 R, 仅反推已在上方分支逐轴处理 —
            // 反推的 Rᵀ 修正此处统一覆写 XYZ 的 target_pos。
            if(g_coord_mgr.tilt_active && !is_G53_this_block){
                int xi = g_axis_map['X' - 'A'];
                int yi = g_axis_map['Y' - 'A'];
                int zi = g_axis_map['Z' - 'A'];
                if(xi >= 0 && yi >= 0 && zi >= 0){
                    const double t3[3] = { target_pos[xi], target_pos[yi], target_pos[zi] };
                    double mt[3];
                    tilt_apply_rot(t3, mt);
                    machine_target_pos[xi] = mt[0] + w_arr[xi];
                    machine_target_pos[yi] = mt[1] + w_arr[yi];
                    machine_target_pos[zi] = mt[2] + w_arr[zi];
                }
            }
            if(g_coord_mgr.tilt_active && is_G53_this_block){
                int xi = g_axis_map['X' - 'A'];
                int yi = g_axis_map['Y' - 'A'];
                int zi = g_axis_map['Z' - 'A'];
                if(xi >= 0 && yi >= 0 && zi >= 0){
                    const double d3[3] = {
                        machine_target_pos[xi] - w_arr[xi],
                        machine_target_pos[yi] - w_arr[yi],
                        machine_target_pos[zi] - w_arr[zi]
                    };
                    double tp[3];
                    tilt_apply_rotT(d3, tp);
                    target_pos[xi] = tp[0];
                    target_pos[yi] = tp[1];
                    target_pos[zi] = tp[2];
                }
            }

            // 软限位检查 (与 axis_ctrl.c:check_soft_limits 同语义, 但该函数 static 不可直接调)
            for(int i = 0; i < AXIS_NUM; i++){
                if(g_axis[i].enable_soft_limit){
                    if(machine_target_pos[i] > g_axis[i].soft_limit_pos ||
                       machine_target_pos[i] < g_axis[i].soft_limit_neg){
                        char ev_buf[64];
                        printf("[Parser] G28 中间点 %s=%.3f 超软限位 [%.2f, %.2f]\n",
                               g_axis[i].axis_name, machine_target_pos[i],
                               g_axis[i].soft_limit_neg, g_axis[i].soft_limit_pos);
                        snprintf(ev_buf, sizeof(ev_buf), "G28 mid %s=%.2f SL [%.1f,%.1f]",
                                 g_axis[i].axis_name, machine_target_pos[i],
                                 g_axis[i].soft_limit_neg, g_axis[i].soft_limit_pos);
                        EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0024, i, ev_buf);
                        atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                        return -1;
                    }
                }
            }

            printf("[Parser] G28 中间点:");
            for(int i = 0; i < AXIS_NUM; i++){
                if(has_axis[i]) printf(" %s=%.3f", g_axis[i].axis_name, target_pos[i]);
            }
            double speed = (g_state.feedrate_mm_min > 0.0)
                           ? g_state.feedrate_mm_min : 1000.0;
            printf(" (speed=%.1f mm/min)\n", speed);

            // 入队中间点运动段 — 推机械域目标 (与主运动分发 :api_push_trajectory(machine_target_pos)
            // 同语义; 原实现误推逻辑域 target_pos, w≠0 时中间点错位 — P1 (2026-09-07) 修正,
            // w=0 (默认 SetZero) 时两者相等, 既有 g28 用例行为不变)
            int rc = api_push_trajectory(machine_target_pos, speed, DEFAULT_ACC, DEFAULT_DEC);
            if(rc < 0){
                printf("[Parser] G28 中间点入队失败 (rc=%d)\n", rc);
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                return -1;
            }

            // 同步等待运动段完成 (parser_thread 阻塞, RT 持续消费 queue)
            // 与 axis_homing_multi 同模式, Non-RT 安全
            // P1: 改用 wait_motion_settle — 原写法在 RT 尚未取段时立即返回,
            //     导致 axis_homing_multi 与仍在飞的中间点运动抢 RT,
            //     kickoff 握手 2s 超时 → FAULT 回滚, 中间点被截断 (TC-8)。
            wait_motion_settle();
            // 同步 plan_cursor (避免后续段基于过期位置, 与 JOG/SafeLift 同模式)
            api_sync_planner_cursor();
            // 更新 parser current_pos (中间点已到达, 后续 G28 homing 不影响 current_pos)
            for(int i = 0; i < AXIS_NUM; i++){
                g_state.current_pos[i] = target_pos[i];
            }
            printf("[Parser] G28 中间点运动完成, 触发回零\n");
        }

        printf("[Parser] G28: 触发 axis_homing_multi (%d 轴顺序回零)\n",
               g_homing_cfg.order_count);
        int rc = axis_homing_multi(g_homing_cfg.order, g_homing_cfg.order_count,
                                    SOURCE_PARSER);
        if(rc < 0){
            printf("[Parser] G28: axis_homing_multi FAULT, 已回滚\n");
            // alarm_state 已由 axis_homing_multi 内部设置
            return -1;
        }
        printf("[Parser] G28: homing 完成, 继续解析\n");
        return 0;  // G28 本行结束, 不走 motion/mcode 路径
    }

    // ---- P4': G65 用户宏调用 (字母循环结束后处理) ----
    // 与 M98 区别: M98 子程序看到清零的 #1-#33; G65 子程序看到清零 + 字母映射的 #1-#26
    // 完全复用 M5 CallFrame 栈与 M99 返回路径; Phase 2 重构为 dispatch_macro_call helper
    if(is_g65_block){
        int rc = dispatch_macro_call((int)p_value, l_value,
                                      g65_args, g65_args_set,
                                      1, "G65");
        if(rc < 0) return -1;
        return 0;  // 0=已调用或 L=0 no-op, 都不再走后续 motion/mcode 路径
    }

    // ---- P4' Phase 2: G66 模态宏调用激活 (字母循环结束后处理) ----
    // G66 P- A-...Z- : 激活 modal macro, 此后每个运动段后自动调用一次
    // G66 行本身无运动 (is_non_motion_g=1), 仅记录参数到 g_state
    if(is_g66_block){
        if(!g_current_program){
            printf("[Parser] G66 在无 program 上下文\n");
            return -1;
        }
        if(p_value < 1.0){
            printf("[Parser] G66 缺 P<Onum> (源行 %d)\n",
                   g_current_program->lines[g_pc].line_no);
            return -1;
        }
        int o_idx = Program_FindOLabel(g_current_program, (int)p_value);
        if(o_idx < 0){
            printf("[Parser] G66 P%d 未找到子程序 (源行 %d)\n",
                   (int)p_value, g_current_program->lines[g_pc].line_no);
            return -1;
        }
        // 激活 modal macro: 存参数到 g_state
        g_state.modal_macro_active = 1;
        g_state.modal_macro_O_num  = (int)p_value;
        for(int n = 1; n <= 26; n++){
            g_state.modal_macro_args[n]     = g65_args[n];
            g_state.modal_macro_args_set[n] = g65_args_set[n];
        }
        int argc = 0;
        for(int n = 1; n <= 26; n++) if(g65_args_set[n]) argc++;
        printf("[Parser] G66 P%d 激活模态宏调用 (%d 个参数, 源行 %d)\n",
               (int)p_value, argc, g_current_program->lines[g_pc].line_no);
        return 0;
    }

    // ---- P5': G54.1 Pn 扩展 WCS (字母循环结束后处理) ----
    // Fanuc 标准: G54.1 P1-P48 选择 48 组扩展 WCS 之一
    // 优先级: G54.1 激活时覆盖 modal_wcs (G54-G59); G54-G59 任意一切换会清零 modal_ext_wcs_p
    if(is_g541_block){
        int p = (int)p_value;
        if(p < 1 || p > 48){
            printf("[Parser] G54.1 P%d 越界 (允许 1-48, 源行 %d)\n", p,
                   g_current_program ? g_current_program->lines[g_pc].line_no : -1);
            return -1;
        }
        if(g_state.bspline_enabled) BSpline_Flush();
        g_state.modal_ext_wcs_p = p;
        printf("[Parser] G54.1 P%d 激活扩展 WCS (源行 %d)\n", p,
               g_current_program ? g_current_program->lines[g_pc].line_no : -1);
    }

    // ---- Phase B2: G04 P<ms> dwell 处理 (push M64 段, RT 消费时 freeze N ms) ----
    // 设计: G04 是非模态非运动指令, 字母循环后处理. push 一个 M64 段入队 (p_value=dwell_ms),
    //       RT 消费 M64 时进 is_waiting_mcode=1, ecat_core.c switch case 64 等待 dwell_ms.
    // 用 M64: 不与 M3/M4/M5/M6/M7/M8/M9/M19 冲突, M60-M77 是 RS274 用户定义区.
    // dwell 期间 is_moving=0, coupling_update 直接 return → 保持 P_base (穿孔需要全功率).
    if(is_g04_block){
        if(g04_dwell_ms < 0.0){
            int src_line = g_current_program ?
                           g_current_program->lines[g_pc].line_no : -1;
            printf("[Parser][ALARM] G04 P=%.2f 不能为负, 源行 %d\n",
                   g04_dwell_ms, src_line);
            atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
            EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0022,
                             (int32_t)(g04_dwell_ms * 1000), "G04 dwell P value negative");
            return -1;
        }
        // 用 p_value 字段传 dwell_ms 给 api_push_mcode (无需新参数)
        if(api_push_mcode(64, 0.0, g04_dwell_ms, 0.0, 0.0) < 0){
            printf("[Parser] G04 dwell 入队失败\n");
            return -1;
        }
        printf("[Parser] G04 dwell: %.0f ms (M64 段)\n", g04_dwell_ms);
        // G04 本行不入运动段, 后续 has_move 门控会跳过
    }

    // ---- M3/M4 spindle 启动延迟处理 (修复 "M3 S1000" 同行时序 bug) ----
    // 原 case 'M' 内的 rpm 检查在此刻执行: 字母循环已结束, S 字母 (如有) 必然
    // 已写入 g_state.spindle_rpm. 修复"M3 S1000"同行场景 (M 在前 S 在后).
    // 行为对齐原 case 'M' 逻辑:
    //   rpm<=0 → ALARM + m_code=-1 (阻止下方默认 api_push_mcode 入队)
    //   rpm>0  → 更新 spindle_mode + P0-Laser 联动, m_code 保持 3/4 由下方默认 else 入队
    if (m_code == 3 || m_code == 4) {
        if (g_state.spindle_rpm <= 0.0) {
            int src_line = g_current_program ?
                           g_current_program->lines[g_pc].line_no : -1;
            printf("[Parser][ALARM] M%d 拒绝入队: spindle_rpm=%.2f <= 0 "
                   "(源行 %d, 请先 S<rpm>)\n",
                   m_code, g_state.spindle_rpm, src_line);
            atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
            EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0020, m_code,
                             "M3/M4 spindle_rpm<=0 (no S specified)");
            m_code = -1;   // 阻止下方默认 api_push_mcode 入队
        } else {
            g_state.spindle_mode = (m_code == 3) ? 1 : 2;   // CW / CCW
            // P0-Laser: 激光模式 (do_slave_id >= 0) 下 M3 联动激光闸 ON
            // 复用 spindle_mode 作为激光使能镜像 (aux_laser_enable 派生自此)
            if (g_laser_cfg.do_slave_id >= 0) {
                g_state.laser_shutter_pending = 1;
                // M67 未显式设置功率时, 用 S 值做默认功率 (激光器常见写法)
                if (g_state.laser_power_pending <= 0.0) {
                    g_state.laser_power_pending = g_state.spindle_rpm;
                }
            }
        }
    }

    // 处理M代码：压入队列作为同步屏障
    if (m_code >= 0) {
        // B-Spline 缓冲区排空：M 代码前必须强制刷新，保证指令顺序
        if (g_state.bspline_enabled) {
            BSpline_Flush();
        }

        // M50: B-Spline 平滑模式开关 (P1=开启, P0=关闭)
        if (m_code == 50) {
            g_state.bspline_enabled = (p_value > 0.5) ? 1 : 0;
            printf("[Parser] B-Spline平滑模式: %s\n", g_state.bspline_enabled ? "开启" : "关闭");
            // M50 不入队，仅切换模式
        }
        // Phase B1: M70 P<0/1> 切换耦合模式 (字母循环后 p_value 已就绪, 避免字母顺序 BUG)
        else if (m_code == 70) {
            if (g_laser_cfg.do_slave_id < 0) {
                // 未配置激光: 静默忽略
            } else if (p_value != 0.0 && p_value != 1.0) {
                int src_line = g_current_program ?
                               g_current_program->lines[g_pc].line_no : -1;
                printf("[Parser][ALARM] M70 P=%.2f 越界 (0 或 1), 源行 %d\n",
                       p_value, src_line);
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0023,
                                 (int32_t)(p_value * 1000), "M70 coupling mode P oob");
            } else {
                atomic_store_explicit(&g_laser_cfg.coupling_mode, (int)p_value,
                                      memory_order_release);
                // B1 诊断: 写入后立即 verify 读回 (排查跨线程可见性)
                int verify = atomic_load_explicit(&g_laser_cfg.coupling_mode,
                                                  memory_order_acquire);
                printf("[Parser] M70 写入: p_value=%.2f → coupling_mode=%d (verify=%d, addr=%p)\n",
                       p_value, verify, verify,
                       (void*)&g_laser_cfg.coupling_mode);
                printf("[Parser] 激光 P-v 耦合: %s\n",
                       verify ? "启用 (查表)" : "关闭 (直接 P_base)");
            }
            // M70 不入队 (模态切换, 与 M50 同模式)
        }
        // Phase B1: M71 P<v_thresh> 设置低速阈值
        else if (m_code == 71) {
            if (g_laser_cfg.do_slave_id < 0) {
                // 未配置激光: 静默忽略
            } else if (p_value < 0.0) {
                int src_line = g_current_program ?
                               g_current_program->lines[g_pc].line_no : -1;
                printf("[Parser][ALARM] M71 P=%.2f 不能为负, 源行 %d\n",
                       p_value, src_line);
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0023,
                                 (int32_t)(p_value * 1000), "M71 v_thresh P negative");
            } else {
                atomic_store_explicit(&g_laser_cfg.v_thresh_mm_s, p_value,
                                      memory_order_release);
                printf("[Parser] 激光 v_thresh = %.2f mm/s\n", p_value);
            }
            // M71 不入队
        }
        // P1-5 (2026-09-03): M77 P<ratio> 微连接段功率系数 (modal, 不入队)
        // SEG_FLAG_MICRO_JOINT (M74/M75 包裹) 段 coupling ratio × ratio → 降功率不切穿。
        // 入队时快照到 seg.aux_laser_mj_ratio (与 M70/M71 同 B1 段级快照模式)。
        else if (m_code == 77) {
            if (g_laser_cfg.do_slave_id < 0) {
                // 未配置激光: 静默忽略
            } else if (p_value <= 0.0 || p_value > 1.0) {
                int src_line = g_current_program ?
                               g_current_program->lines[g_pc].line_no : -1;
                printf("[Parser][ALARM] M77 P=%.3f 越界 (0-1], 源行 %d\n",
                       p_value, src_line);
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0023,
                                 (int32_t)(p_value * 1000), "M77 mj ratio P oob");
            } else {
                atomic_store_explicit(&g_laser_cfg.mj_ratio, p_value,
                                      memory_order_release);
                printf("[Parser] 微连接功率系数 = %.3f\n", p_value);
            }
            // M77 不入队
        }
        // P1-4 (2026-09-03): M76 P<级数> 渐进穿孔 (queued M 段, 台阶表由 RPC 0x0069 配)
        // 逐级 {功率/频率/停留} 驱动, 整体完成 pierce_count++ 并恢复 M67/M68 切割参数。
        // hold 冻结: P0-1 gate 拦 timer++ → 暂停时台阶不推进 (闸关不计时)。
        else if (m_code == 76) {
            int stages = (int)(p_value + 0.5);
            if (g_laser_cfg.do_slave_id < 0) {
                // 未配置激光: 静默忽略
            } else if (fabs(p_value - (double)stages) > 1e-6 || stages < 1
                       || stages > g_laser_cfg.pierce_table_len) {
                int src_line = g_current_program ?
                               g_current_program->lines[g_pc].line_no : -1;
                printf("[Parser][ALARM] M76 P=%.2f 级数非法 (1-%d 整数), 源行 %d\n",
                       p_value, g_laser_cfg.pierce_table_len, src_line);
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0023,
                                 (int32_t)(p_value * 1000), "M76 pierce stages oob");
                return -1;
            } else {
                // p_value 复用 dwell_ms 通道传级数 (RT: mcode_p_value_ms)
                if(api_push_mcode(76, 0.0, (double)stages, 0.0, 0.0) < 0){
                    printf("[Parser] M76 入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
                printf("[Parser] M76 渐进穿孔: %d 级\n", stages);
            }
            m_code = -1;   // 已手动入队, 不走默认路径
        } else {
            if(api_push_mcode(m_code, s_value, p_value, q_value, r_value) < 0){
                printf("[Parser] M代码入队失败(报警)，中止当前文件！\n");
                return -1;
            }
            printf("[Parser] 解析M代码: M%02d S%.1f P%.1f Q%.1f R%.1f\n", m_code, s_value, p_value, q_value, r_value);
        }
    }

    // ---- Phase 2A.2: 固定循环优先路径 (在常规运动门控前拦截) ----
    // 固定循环激活时, XY 运动展开为钻孔循环; 跳过常规 G00/G01/G02/G03 分发
    if(has_move && !is_non_motion_g && g_state.active_cycle != 0){
        // P1 G68.2: 倾斜面激活时固定循环禁止 (Z 轴硬编码假设, 二期专项支持)
        if(g_coord_mgr.tilt_active){
            g682_alarm(3, g_current_program ?
                             g_current_program->lines[g_pc].line_no : -1);
            return -1;
        }
        int z_idx_fc = g_axis_map['Z' - 'A'];
        if(z_idx_fc < 0){
            printf("[Parser] 固定循环要求 Z 轴已映射！拒绝执行\n");
            return -1;
        }

        // ---- Phase 2A++: G90/G91 双模式 R/Z 捕获 (顺序: 先 R 后 Z) ----
        // G90 (绝对): R 和 Z 直接使用本行原始值
        // G91 (相对, Fanuc 标准):
        //   R 相对 cycle_initial_Z (本行激活时已捕获)
        //   Z 相对 (本行新 R 或上一次模态 R) 平面
        if(has_r){
            if(g_state.is_absolute){
                g_state.cycle_R_plane = r_value;
            } else {
                g_state.cycle_R_plane = g_state.cycle_initial_Z + r_value;
            }
        }
        if(has_axis[z_idx_fc]){
            if(g_state.is_absolute){
                g_state.cycle_Z_bottom = val_axis[z_idx_fc];
            } else {
                // G91: Z 相对 R 平面 (本行已更新的 R 优先, 否则用模态 R)
                g_state.cycle_Z_bottom = g_state.cycle_R_plane + val_axis[z_idx_fc];
            }
        }

        // 构造 cycle 目标位置 (统一转为绝对坐标, G91 在此转换)
        double cycle_target[AXIS_NUM];
        for(int i = 0; i < AXIS_NUM; i++){
            if(has_axis[i]){
                cycle_target[i] = g_state.is_absolute
                                  ? val_axis[i]
                                  : g_state.current_pos[i] + val_axis[i];
            } else {
                cycle_target[i] = g_state.current_pos[i];
            }
        }

        // 钻孔尖角不应平滑: 进入循环前强制 Flush
        if(g_state.bspline_enabled) BSpline_Flush();

        if(generate_fixed_cycle(cycle_target, g_state.current_pos,
                                 g_state.feedrate_mm_min) < 0){
            printf("[Parser] 固定循环入队失败(报警)，中止当前文件！\n");
            return -1;
        }

        // 更新 g_state.current_pos 到退回点 (G98=initial_Z, G99=R_plane)
        double retract_Z = (g_state.cycle_retract_mode == 99)
                           ? g_state.cycle_R_plane : g_state.cycle_initial_Z;
        for(int i = 0; i < AXIS_NUM; i++){
            if(i == z_idx_fc){
                g_state.current_pos[i] = retract_Z;
            } else {
                g_state.current_pos[i] = cycle_target[i];
            }
        }

        printf("[Parser] 固定循环 G%d 完成: 孔深 Z=%.3f, R平面=%.3f, 退回=%s\n",
               g_state.active_cycle, g_state.cycle_Z_bottom, g_state.cycle_R_plane,
               (g_state.cycle_retract_mode == 99) ? "R平面(G99)" : "初始Z(G98)");
        return 0;
    }

    // 运动门控：仅当本行包含显式轴运动且非运动参数指令时才触发轨迹下发
    if(has_move && !is_non_motion_g){
        double target_pos[AXIS_NUM];   // 逻辑坐标（工件坐标系）
        double start_pos[AXIS_NUM];
        double machine_target_pos[AXIS_NUM];
        double machine_start_pos[AXIS_NUM];

        // 工件坐标偏置查询（G53 行内标记不影响模态 WCS）
        // 读 parser 模态 g_state.modal_wcs (本线程独占写), 不读 g_coord_mgr.current_coord
        // (那是 RT 线程消费段时才更新的"物理当前 WCS",滞后于解析)。
        int wcs_idx = (g_state.modal_wcs >= COORD_G54 &&
                       g_state.modal_wcs <= COORD_G59)
                      ? (g_state.modal_wcs - 1) : -1;

        if(is_G53_this_block){
            // G53 非模态：val_axis 是机械绝对坐标，强制 G90，忽略 G91
            // P2': back-calc 用的 w 必须用 effective offset (WCS+G52),
            //       否则下一行正常模式下 start_pos 与 machine_start 会对不上 G52 量
            // P5': w 优先取 ext WCS (G54.1 Pn), 否则 regular WCS (G54-G59)
            // P1 G68.2: w 含 tilt_origin; 正向直通不套 R (机械坐标天然 bypass),
            //           反推 target = Rᵀ·(machine − w) 保证逻辑域连续
            double w_arr[AXIS_NUM];
            for(int i=0;i<AXIS_NUM;i++){
                double w;
                if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                    w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][i];
                } else {
                    w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
                }
                if(g_state.local_offset_active && wcs_idx >= 0) w += g_state.local_offset[i];
                if(g_coord_mgr.tilt_active && wcs_idx >= 0) w += g_coord_mgr.tilt_origin[i];
                w_arr[i] = w;
                start_pos[i] = g_state.current_pos[i];
                machine_start_pos[i] = start_pos[i] + w;
                machine_target_pos[i] = has_axis[i] ? val_axis[i] : machine_start_pos[i];
                // 反推逻辑坐标 (在 WCS+G52 空间), 保证下一行回到正常模式时起点不撕裂
                target_pos[i] = machine_target_pos[i] - w;
            }
            // P1 G68.2: 倾斜面激活时 XYZ 三轴覆写
            //   machine_start = R·start + w (逻辑 start 的机械像, 与正常分支一致)
            //   target = Rᵀ·(machine_target − w)
            if(g_coord_mgr.tilt_active){
                int xi = g_axis_map['X' - 'A'];
                int yi = g_axis_map['Y' - 'A'];
                int zi = g_axis_map['Z' - 'A'];
                if(xi >= 0 && yi >= 0 && zi >= 0){
                    const double s3[3] = { start_pos[xi], start_pos[yi], start_pos[zi] };
                    const double d3[3] = {
                        machine_target_pos[xi] - w_arr[xi],
                        machine_target_pos[yi] - w_arr[yi],
                        machine_target_pos[zi] - w_arr[zi]
                    };
                    double ms[3], tp[3];
                    tilt_apply_rot(s3, ms);
                    tilt_apply_rotT(d3, tp);
                    machine_start_pos[xi] = ms[0] + w_arr[xi];
                    machine_start_pos[yi] = ms[1] + w_arr[yi];
                    machine_start_pos[zi] = ms[2] + w_arr[zi];
                    // 未指定轴的 machine_target = machine_start (保持现位语义)
                    if(!has_axis[xi]) machine_target_pos[xi] = machine_start_pos[xi];
                    if(!has_axis[yi]) machine_target_pos[yi] = machine_start_pos[yi];
                    if(!has_axis[zi]) machine_target_pos[zi] = machine_start_pos[zi];
                    target_pos[xi] = tp[0];
                    target_pos[yi] = tp[1];
                    target_pos[zi] = tp[2];
                }
            }
        }else{
            // 正常模式：val_axis 是逻辑坐标，按 G90/G91 计算
            for(int i=0;i<AXIS_NUM;i++){
                start_pos[i] = g_state.current_pos[i];
                if(g_state.is_absolute){
                    target_pos[i] = has_axis[i] ? val_axis[i] : g_state.current_pos[i];
                }else{
                    target_pos[i] = g_state.current_pos[i] + (has_axis[i] ? val_axis[i] : 0);
                }
            }
            double w_arr[AXIS_NUM];
            for(int i=0;i<AXIS_NUM;i++){
                // P2': push 时也需包含 G52 stacking, 与 snapshot_wcs_offset 保持一致,
                // 否则 RT 用 snap 反推 logical 会撕裂 G52 量
                // P5': 优先用 ext WCS (G54.1 Pn) 偏置, 否则 regular WCS
                double w;
                if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                    w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][i];
                } else {
                    w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
                }
                if(g_state.local_offset_active) w += g_state.local_offset[i];
                // P1 G68.2: 倾斜面原点并入总平移 — 与 axis_ctrl.c snapshot_wcs_offset 同步!
                if(g_coord_mgr.tilt_active) w += g_coord_mgr.tilt_origin[i];
                w_arr[i] = w;
                machine_target_pos[i] = target_pos[i] + w;
                machine_start_pos[i] = start_pos[i] + w;

                // P0: 旋转轴 G90 最短路径解缠绕 (G91/G53 不进此分支:
                // G53 在上方分支直通, G91 由 is_absolute 条件排除)
                if(g_state.is_absolute && has_axis[i] &&
                   g_axis[i].axis_type == 1 && g_axis[i].rot_wrap_enable){
                    int wrapped = 0;
                    double m_tgt = wrap_rotary_shortest(i, machine_start_pos[i],
                                                        machine_target_pos[i], &wrapped);
                    if(wrapped){
                        char ev_buf[64];
                        snprintf(ev_buf, sizeof(ev_buf), "%s wrap %.2f -> %.2f",
                                 g_axis[i].axis_name, machine_target_pos[i], m_tgt);
                        machine_target_pos[i] = m_tgt;
                        target_pos[i] = m_tgt - w;   // 保持逻辑/机械反推一致 (旋转轴, R 不作用)
                        printf("[Parser] %s\n", ev_buf);
                        EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x0025, i, ev_buf);
                    }
                }
            }
            // P1 G68.2: 倾斜面激活时 XYZ 线性轴覆写 machine = R·p + w
            // (R 只作用线性轴, B/C 角度不旋转; w 已含 tilt_origin; G91 逻辑域计算
            //  在上方 target 成形完成, 此处统一变换, 无需分支处理)
            if(g_coord_mgr.tilt_active){
                int xi = g_axis_map['X' - 'A'];
                int yi = g_axis_map['Y' - 'A'];
                int zi = g_axis_map['Z' - 'A'];
                if(xi >= 0 && yi >= 0 && zi >= 0){
                    const double t3[3] = { target_pos[xi], target_pos[yi], target_pos[zi] };
                    const double s3[3] = { start_pos[xi], start_pos[yi], start_pos[zi] };
                    double mt[3], ms[3];
                    tilt_apply_rot(t3, mt);
                    tilt_apply_rot(s3, ms);
                    machine_target_pos[xi] = mt[0] + w_arr[xi];
                    machine_target_pos[yi] = mt[1] + w_arr[yi];
                    machine_target_pos[zi] = mt[2] + w_arr[zi];
                    machine_start_pos[xi] = ms[0] + w_arr[xi];
                    machine_start_pos[yi] = ms[1] + w_arr[yi];
                    machine_start_pos[zi] = ms[2] + w_arr[zi];
                }
            }
        }

        // RTCP 逆解：圆弧由 arc generator 逐点逆解，直线由 generate_linear_rtcp_trajectory 处理
        // 不再在此处对起终点做硬算，避免旋转运动时中间轨迹挖坑

        // 多轴等效距离计算（用于 G93 速度反推）
        // H1 消融 (S10, 2026-09-20): tip_basis=1 时改用**刀尖欧氏长度 L^P**
        // (只累加直线轴, 旋转轴折算行程不计入) ⇒ 下发速率 = L^P/T 而非 L^M/T。
        // 执行侧 G93 strict 段是时间驱动 (axis_ctrl v_const = dist/T), 故轨迹不变;
        // 变的只有可行性门看到的 req_v (低估 m = L^M/L^P 倍)。
        double dist_total = 0.0;
        for(int i=0;i<AXIS_NUM;i++){
            if(g_g93_tip_basis && g_axis[i].axis_type == 1) continue;
            double delta = machine_target_pos[i] - machine_start_pos[i];
            if(g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0){
                // 旋转轴：角度 → 弧长 (mm)
                delta = delta * (PI / 180.0) * g_axis[i].equivalent_radius;
            }
            dist_total += delta * delta;
        }
        dist_total = sqrt(dist_total);

        // G53，屏蔽一切姿态补偿和平滑
        int local_rtcp_enabled = g_state.rtcp_enabled;
        int local_bspline_enabled = g_state.bspline_enabled;
        if(is_G53_this_block) {
            local_rtcp_enabled = 0;
            local_bspline_enabled = 0;
        }

        // ---- 异步时序同步屏障 ----
        // 判断当前指令是否会绕过 B-Spline 蓄水池直接下发
        int will_bypass_bspline = 1;
        if(g_state.bspline_enabled && g_state.motion_mode == 1 && !g_state.rtcp_enabled){
            will_bypass_bspline = 0; // 纯 G01 将进入蓄水池
        }

        // 如果开启了 B-Spline，且当前指令准备“插队直通底层”，必须先排空蓄水池！
        // 这保证了时间先后的绝对顺序，并让底层的 plan_cursor 更新到最新位置
        if (g_state.bspline_enabled && will_bypass_bspline) {
            BSpline_Flush();
        }
        // ---------------------------------------------

        // 进给速度计算：G00 始终快速，G93 按时间反推，G94 直取 F 值
        double run_speed_mm;   // mm/min
        double g93_T_sec = 0.0;

        if(g_state.motion_mode == 0){
            run_speed_mm = RAPID_SPEED_MM_MIN;
        } else if(g_state.feed_mode == FEED_MODE_G93){
            if(!has_f){
                printf("[Parser] G93 模式下每行运动指令必须显式给出 F 值！\n");
                return -1;
            }
            if(g_state.feedrate_mm_min <= 1e-6){
                printf("[Parser] G93 F值过小(%.6f)，除零风险，拒绝下发！\n", g_state.feedrate_mm_min);
                return -1;
            }
            g93_T_sec = 60.0 / g_state.feedrate_mm_min;
            double v_req = dist_total / g93_T_sec;
            if(v_req < 1e-6) v_req = 1e-6;
            run_speed_mm = v_req * 60.0;
        } else {
            run_speed_mm = g_state.feedrate_mm_min;
        }

        if(g_state.motion_mode==2 || g_state.motion_mode==3){
            // P1 G68.2: 倾斜面激活时圆弧禁止 (平面重映射列二期; 0x0028 reason=2)
            if(g_coord_mgr.tilt_active){
                g682_alarm(2, g_current_program ?
                             g_current_program->lines[g_pc].line_no : -1);
                return -1;
            }
            double off_1st, off_2nd;
            switch(g_state.active_plane){
                case 18: off_1st=offset_k; off_2nd=offset_i; break;
                case 19: off_1st=offset_j; off_2nd=offset_k; break;
                default: off_1st=offset_i; off_2nd=offset_j; break;
            }

            // ---- 阶段 3: 刀补激活时 G02/G03 走 CutterComp_PushArc ----
            // 圆弧偏置由 cutter_comp.c 内部完成 (offset_arc + 离散化)
            // 与 G01 直线一样, 引擎内部计算偏置后通过回调下发
            // 注: 圆心 (ax1, ax2) 由 I/J (G17) / K/I (G18) / J/K (G19) 解析得到
            if(CutterComp_GetMode() != COMP_OFF){
                int ax1_idx, ax2_idx;
                double center[2];
                // 复用 cutter_comp.c 的 get_plane_axes 逻辑 (此处直接重算避免 extern)
                switch(g_state.active_plane){
                    case 18: ax1_idx = g_axis_map['Z'-'A']; ax2_idx = g_axis_map['X'-'A']; break;
                    case 19: ax1_idx = g_axis_map['Y'-'A']; ax2_idx = g_axis_map['Z'-'A']; break;
                    default: ax1_idx = g_axis_map['X'-'A']; ax2_idx = g_axis_map['Y'-'A']; break;
                }
                center[0] = machine_start_pos[ax1_idx] + off_1st;
                center[1] = machine_start_pos[ax2_idx] + off_2nd;
                double radius = hypot(off_1st, off_2nd);
                if(radius < 1e-6){
                    printf("[Parser] G02/G03 圆弧半径为 0 (I/J/K 未指定?), 中止\n");
                    return -1;
                }
                // sweep: 从 start 到 end 的扫角 (atan2 跨象限)
                double start_ang = atan2(machine_start_pos[ax2_idx] - center[1],
                                          machine_start_pos[ax1_idx] - center[0]);
                double end_ang   = atan2(machine_target_pos[ax2_idx] - center[1],
                                          machine_target_pos[ax1_idx] - center[0]);
                double sweep = end_ang - start_ang;
                // G02 (CW): sweep 归一化到 [-2π, 0]; G03 (CCW): 归一化到 [0, 2π]
                int is_CW = (g_state.motion_mode == 2);
                if(is_CW){
                    while(sweep > 0)  sweep -= 2.0 * 3.14159265358979323846;
                    while(sweep < -2.0 * 3.14159265358979323846) sweep += 2.0 * 3.14159265358979323846;
                } else {
                    while(sweep < 0)  sweep += 2.0 * 3.14159265358979323846;
                    while(sweep >  2.0 * 3.14159265358979323846) sweep -= 2.0 * 3.14159265358979323846;
                }

                CompSegment_t arc_seg;
                memset(&arc_seg, 0, sizeof(arc_seg));
                arc_seg.type = COMP_SEG_ARC;
                memcpy(arc_seg.start_pos, machine_start_pos, sizeof(double) * AXIS_NUM);
                memcpy(arc_seg.end_pos,   machine_target_pos, sizeof(double) * AXIS_NUM);
                arc_seg.center[0] = center[0];
                arc_seg.center[1] = center[1];
                arc_seg.radius = radius;
                arc_seg.sweep = sweep;
                arc_seg.is_CW = is_CW;
                arc_seg.speed = run_speed_mm / 60.0;
                arc_seg.acc = DEFAULT_ACC;
                arc_seg.dec = DEFAULT_DEC;
                // C3 (2026-07-27): G93 强一致性时间预算 (PushArc 内部均分到离散微段)
                arc_seg.g93_dt_sec = (g_state.feed_mode == FEED_MODE_G93) ? g93_T_sec : 0.0;
                // V13.1 Phase-0: semantic owner —— 本行是 arc_seg.g93_dt_sec 的来源
                arc_seg.source_line = g_current_line_no;

                if(CutterComp_PushArc(&arc_seg) < 0){
                    printf("[Parser] 刀补圆弧入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            } else if(generate_arc_trajectory(machine_start_pos,
                                    machine_target_pos,
                                    off_1st, off_2nd,
                                    g_state.motion_mode==2, run_speed_mm, g93_T_sec) < 0){
                printf("[Parser] 圆弧入队失败(报警)，中止当前文件！\n");
                return -1;
            }

        } else if(g_state.rtcp_enabled){
            // RTCP 直线：微段打碎 + 逐点逆解
            // 刀补引擎暂不支持 RTCP 路径 (旋转轴参与偏置复杂度超出 2D 范围)
            if(generate_linear_rtcp_trajectory(machine_start_pos,
                                               machine_target_pos,
                                               run_speed_mm, g93_T_sec) < 0){
                printf("[Parser] RTCP直线入队失败(报警)，中止当前文件！\n");
                return -1;
            }
        } else {
            double speed_mm_sec=run_speed_mm/60.0;

            // ---- 刀具半径补偿路由 (直线段) ----
            // 补偿激活时，G01/G00 直线段通过 CutterComp_PushPoint 走偏置引擎，
            // 引擎内部计算偏置后通过回调下发到 B-Spline 或 Planner。
            // 阶段 3: G02/G03 圆弧在上方 motion_mode==2||3 分支内独立走 CutterComp_PushArc;
            //         RTCP 路径仍直通 (旋转轴参与偏置复杂度超出 2D 范围)
            if(CutterComp_GetMode() != COMP_OFF && g_state.motion_mode <= 1){
                // 刀补模式: 通过补偿引擎入队 (引擎内部已设置输出回调)
                // C3 (2026-07-27): 透传 G93 强一致性上下文, 保时间绝对守恒
                int is_g93_here = (g_state.feed_mode == FEED_MODE_G93) ? 1 : 0;
                if(CutterComp_PushPoint(machine_target_pos, speed_mm_sec,
                                        DEFAULT_ACC, DEFAULT_DEC,
                                        is_g93_here, g93_T_sec,
                                        g_current_line_no) < 0){
                    printf("[Parser] 刀补引擎入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            } else if(!will_bypass_bspline &&
                      // 断点续切 (2026-09-08): 跳段区间不喂蓄水池 — bspline 延迟推送
                      // 携带的是"推送时刻"的行号, 会以当前 line_no 混入已执行区间;
                      // 改走直通路径由 impl 跳段分支统一丢弃
                      g_parser_ctrl.program_mode != PROGRAM_MODE_SEARCH &&
                      !(g_parser_ctrl.program_mode == PROGRAM_MODE_RUN &&
                        g_parser_ctrl.resume_active)){
                // G01 + B-Spline 模式: 通过平滑引擎入队
                if(BSpline_PushDirtyPoint(machine_target_pos, speed_mm_sec, g93_T_sec) < 0){
                    printf("[Parser] B-Spline入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            } else {
                // 直通: G00 或 B-Spline 未启用
                // 2026-09-16 修复 (L2_g93_strict 实测 5343ms vs T=2500ms, +113.6%):
                // G93 普通直通路径 (无 RTCP/无刀补/无样条) 此前走 api_push_trajectory
                // (is_g93_strict=0, g93_dt_sec=0) — parser 按物理等效距离反推的 v_req
                // 被 planner 短板限幅压低 (C 轴大回转需求超限) → 块时长膨胀。
                // 这是 C3 "四路径全覆盖" 遗漏的第五条路径, E3 机制 (前瞻动力学覆盖
                // 时间预算) 的实证案例。G01+G93 改走 g93 wrapper 保严格剖面;
                // G00 不携带时间预算, 维持原路径。
                int push_rc;
                if (g_state.feed_mode == FEED_MODE_G93 && g_state.motion_mode == 1) {
                    // 论文1 S10: 直通 G93 路径是**单段**承载整块预算 (不打碎),
                    // 故 segments_generated = 1, sum == cmd == T*1000。
                    bt_block_open(g93_T_sec * 1000.0, 1, g93_T_sec * 1000.0,
                                  g_current_line_no);
                    push_rc = api_push_trajectory_g93(machine_target_pos, speed_mm_sec,
                                                      DEFAULT_ACC, DEFAULT_DEC, g93_T_sec);
                    bt_block_close();
                } else {
                    push_rc = api_push_trajectory(machine_target_pos, speed_mm_sec,
                                                  DEFAULT_ACC, DEFAULT_DEC);
                }
                if(push_rc < 0){
                    printf("[Parser] 运动指令入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            }
        }

        // @Context: Non-RealTime Background Thread (parser)
        // @Stage: STAGE_PARSER —— G 代码原始意图坐标 (machine_target_pos)
        // 无论本行走圆弧 / RTCP / 刀补 / B-Spline / 直通中的哪条路径,
        // machine_target_pos 都是 Parser 解析出的唯一意图点,
        // 在更新 g_state.current_pos 前一次性记录,作为管线起点的基准。
        TraceLogger_PushPipeline(STAGE_PARSER, machine_target_pos, 0.0);

        for(int i=0;i<AXIS_NUM;i++){
            g_state.current_pos[i]=target_pos[i];
        }

        // 动态打印：按 g_axis_map 映射输出已配置轴的标签与数值
        printf("[Parser] %s -> ", buffer);
        const char axis_labels[] = "XYZABCUVW";
        for(int k=0;k<9;k++){
            int idx = g_axis_map[axis_labels[k] - 'A'];
            if(idx >= 0 && idx < AXIS_NUM){
                printf("%c=%.3f ", axis_labels[k], target_pos[idx]);
            }
        }
        printf("F=%.1f\n", run_speed_mm);
    }

    // ---- P4' Phase 2: G66 模态宏调用触发 ----
    // 仅在"运动已下发"时触发 (与运动门控同条件: has_move && !is_non_motion_g)
    // G66 行本身 is_non_motion_g=1, 不触发; G67 同理
    // 触发 = 单次宏调用 (L=1), 复用 dispatch_macro_call, is_g65_frame=1 让 M99 重应用参数
    // (每次触发独立, repeat_remaining=0, M99 真返回)
    if(g_state.modal_macro_active && has_move && !is_non_motion_g){
        int rc = dispatch_macro_call(g_state.modal_macro_O_num, 1,
                                      g_state.modal_macro_args,
                                      g_state.modal_macro_args_set,
                                      1, "G66-modal");
        if(rc < 0) return -1;
        // rc=0 (已调用) 或 rc=1 (L=0 no-op, 不会发生因 L=1 固定)
        return 0;
    }

    return 0;
}

OSAL_THREAD_FUNC parser_thread_func(void *arg){
    while(1){
        if(g_parser_ctrl.is_running==1){
            while(!g_all_axis_op_ready){
                osal_usleep(100000); // 等待所有轴准备就绪
            }

            printf("[Parser] Processing file: %s (mode=%s)\n", g_parser_ctrl.filepath,
                   g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ? "PREVIEW" :
                   (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH ? "SEARCH" : "RUN"));

            /* P1-b: 程序生命周期事件 - 解析启动
             * 断点续切 (2026-09-08): SEARCH 启动走 0x0039 (重放校验, 区分于 preview/run) */
            EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER,
                             g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ? 0x0030 :
                             (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH ? 0x0039 : 0x0032),
                             0, g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ?
                                "LoadProgram start (preview)" :
                                (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH ?
                                 "ResumeSearch start (replay)" : "RunLoadedProgram start"));

            // ---- P0-b v2: 程序级统计清零 (每次启动 parser 都清, 不论 RUN/PREVIEW) ----
            // bbox 用 ±1e18 哨兵, 首段必然更新; 无运动段的程序保留哨兵, UI 自查。
            // first_seg_id 用 (uint64_t)-1 哨兵, axis_ctrl preview 早返回时设实际值。
            g_program_total_time_ms = 0.0;
            for (int i = 0; i < AXIS_NUM; i++) {
                g_program_bbox_min[i] = 1e18;
                g_program_bbox_max[i] = -1e18;
            }
            g_program_first_seg_id = (uint64_t)-1;
            g_program_last_seg_id = 0;
            g_program_total_lines = 0;
            g_program_num_o_labels = 0;
            g_program_num_n_labels = 0;
            // 断点续切: SEARCH 不清 load_done — 它是 UI 的 LoadProgram 状态信号,
            // 搜索只是校验, 不改变"已加载待运行/续切"语义
            if (g_parser_ctrl.program_mode != PROGRAM_MODE_SEARCH) {
                atomic_store_explicit(&g_program_load_done, 0, memory_order_release);
            }

            // ---- Phase 2B M1: 全文加载替代 fgets 流 ----
            // 一次性把文件读入内存, 建立 N 标签表, 供 PC 游标遍历 + GOTO 跳转
            GCodeProgram_t *prog = Program_Load(g_parser_ctrl.filepath);
            if(!prog){
                printf("[Parser错误] 加载失败: %s\n", g_parser_ctrl.filepath);
                // 2026-09-09 keep-last-good (opcua_probe §11-2/3): 异步加载失败
                // 恢复上一成功加载的程序身份 (Fanuc 选程失败保留旧程序惯例),
                // LineCount / RunLoadedProgram 语义不因失败加载归零。
                // fail-fast (API 层 stat) 已挡掉不可达路径类; 此处兜其余
                // (读取中断/行数超限/解析期文件被删等)。
                if (api_program_identity_restore() == 0) {
                    EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x003A, 0,
                                     "program load failed, previous program retained");
                } else {
                    EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x003A, 0,
                                     "program load failed (no previous program)");
                }
                // Stop 落在加载失败分支: 不经退出路径, 就地取走待办 (无残余段可清, 仅撤刹停请求)
                if (atomic_exchange_explicit(&g_parser_ctrl.stop_flush_pending, 0,
                                             memory_order_acq_rel))
                    g_interpolator.pause_request = 0;
                g_parser_ctrl.is_running=0;
                continue;
            }
            g_current_program = prog;

            // P0-b v2: cache 程序元数据 (parser 结束时 Program_Free, 这里读完就丢)
            g_program_total_lines  = (int32_t)prog->num_lines;
            g_program_num_o_labels = (int32_t)prog->num_o_labels;
            g_program_num_n_labels = (int32_t)prog->num_n_labels;

            while(!is_trajectory_finished()){
                osal_usleep(100000); // 等待当前轨迹执行完成，检查频率为100ms
            }

            api_sync_planner_cursor(); // 同步规划器光标，确保新轨迹从当前状态开始
            api_g93_jerk_reset();      // 论文1 IV-E: 严格块链状态清零 (每程序独立)
            api_g93_posthoc_reset();   // 论文1 M1: 事后修正速度赤字清零 (每程序独立)

            for(int i=0;i<AXIS_NUM;i++){
                g_state.current_pos[i]=api_get_cursor(i);
            }

            // ---- Phase 2B M1: PC 游标主循环 (替代原 fgets while) ----
            g_pc = 0;
            g_pc_jump_pending = 0;
            g_pc_step_counter = 0;
            g_call_stack_top = 0;   // Phase 2B M5: 子程序调用栈重置 (防上次运行残留)
            int abort_file = 0;

            while(g_pc < prog->num_lines){
                // 步进计数器防死循环 (无条件 GOTO 后向跳转的兜底保护)
                if(g_pc_step_counter >= MAX_PC_STEPS){
                    printf("[Parser] PC 步进超 %d (疑似死循环), 中止文件\n",
                           MAX_PC_STEPS);
                    abort_file = 1;
                    break;
                }
                g_pc_step_counter++;

                if(g_parser_ctrl.abort_request){
                    printf("[Parser] 中止请求已收到，停止解析文件: %s\n", g_parser_ctrl.filepath);
                    abort_file = 1;
                    break;
                }
                // 暂停检查 (2026-08-27 死锁修复: 暂停等待必须同时消费 abort_request —
                // 原循环只等 is_paused, 暂停期间 Stop 的 abort_request 永不被消费,
                // parser 卡死在此 → RunLoadedProgram 拒绝(is_running=1) → UI Start/Stop 全无响应)
                while(g_parser_ctrl.is_paused && !g_parser_ctrl.abort_request){
                    osal_usleep(100000); // 暂停时每100ms检查一次状态
                }
                if(g_parser_ctrl.abort_request){
                    printf("[Parser] 暂停中收到中止请求, 停止解析文件: %s\n", g_parser_ctrl.filepath);
                    abort_file = 1;
                    break;
                }

                // 解析当前行 G-code 命令 (PC 指向), 入队失败或 GOTO 错则中止文件
                if(parse_gcode_line(prog->lines[g_pc].text) < 0){
                    printf("[Parser] 行 %d 解析失败, 中止文件: %s\n",
                           prog->lines[g_pc].line_no, g_parser_ctrl.filepath);
                    abort_file = 1;
                    break;
                }

                // ---- 断点续切 (2026-09-08): SEARCH 重放的退出条件 ----
                // result!=0: 目标行已完整解析 (1=边界冻结 / 2=非G00边界)
                //            或 SEARCH 中途遇 M2/M30 (3=目标不可达) — 均止于整行解析后
                // 当前行 == 目标行且 result 仍为 0: 该行无任何运动推送 (纯 modal 行)
                //   → 非 G00 边界, finalize 时统一映射 result=2
                if (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH) {
                    if (atomic_load_explicit(&g_program_resume_result,
                                             memory_order_acquire) != 0) {
                        break;
                    }
                    if (prog->lines[g_pc].line_no == g_parser_ctrl.resume_target_line) {
                        break;
                    }
                }

                // 跳转信号消费: parse_gcode_line 内部设置 g_pc_jump_pending
                if(g_pc_jump_pending){
                    g_pc = g_pc_jump_target;
                    g_pc_jump_pending = 0;
                } else {
                    g_pc++;
                }
            }

            // ---- 断点续切: SEARCH 模式收尾 (2026-09-08) ----
            // 不走下方 M30 级清理 (SEARCH 未入队任何段: 无需 flush/无需模态复位 —
            // g_state 保留重放建立的边界模态, 供 UI 参考; 正式续切由 ResumeStart
            // 从头重放重建, 不依赖此处状态)。
            if (g_parser_ctrl.program_mode == PROGRAM_MODE_SEARCH) {
                int r = atomic_load_explicit(&g_program_resume_result,
                                             memory_order_acquire);
                if (g_parser_ctrl.abort_request || abort_file) {
                    // 搜索被中止: 映射 3 (行不可达), 让阻塞的 ResumeSearch 拿到确定返回
                    atomic_store_explicit(&g_program_resume_result, 3,
                                          memory_order_release);
                    EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x0038,
                                     g_parser_ctrl.resume_target_line,
                                     "ResumeSearch aborted");
                } else if (r == 1) {
                    // 边界行完整解析后 result=1: 冻结完成, 可 ResumeStart
                    // 注: 若刀补引擎在边界处仍激活, Phase 1 拒绝 (补偿引擎缓冲的
                    // 跨行输出与跳段行号语义冲突; 激光程序不走刀补, 不受影响)
                    if (CutterComp_GetMode() != COMP_OFF) {
                        atomic_store_explicit(&g_program_resume_result, 2,
                                              memory_order_release);
                        EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x0038,
                                         g_parser_ctrl.resume_target_line,
                                         "ResumeSearch rejected (cutter comp active at boundary)");
                    } else {
                        atomic_store_explicit(&g_program_resume_ready, 1,
                                              memory_order_release);
                        EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER, 0x0036,
                                         g_parser_ctrl.resume_target_line,
                                         "ResumeSearch done (G00 boundary frozen)");
                    }
                } else {
                    // r==0 (行存在但无运动推送) → 2; r==2/3 原样
                    if (r == 0) r = 2;
                    atomic_store_explicit(&g_program_resume_result, r,
                                          memory_order_release);
                    EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x0038,
                                     g_parser_ctrl.resume_target_line,
                                     r == 2 ? "ResumeSearch rejected (not a G00 boundary)"
                                            : "ResumeSearch rejected (line not found)");
                }
                Program_Free(prog);
                g_current_program = NULL;
                // 回落 PREVIEW 语义: GetProgramStructure 的 is_loaded 状态机不受
                // 未知模式影响; UI 用 snapshot RESUME_READY 位区分"待续切"
                g_parser_ctrl.program_mode = PROGRAM_MODE_PREVIEW;
                g_parser_ctrl.is_running = 0;
                g_parser_ctrl.abort_request = 0;
                // Stop 落在 SEARCH 模式: 不经退出路径, 就地取走待办 (SEARCH 不入队运动段)
                if (atomic_exchange_explicit(&g_parser_ctrl.stop_flush_pending, 0,
                                             memory_order_acq_rel))
                    g_interpolator.pause_request = 0;
                continue;   // 回 while(1) 主循环 (is_running=0 → 休眠)
            }

            Program_Free(prog);
            g_current_program = NULL;

            if(g_state.bspline_enabled){
                BSpline_Flush();
            }
            api_flush_planner();

            // ---- 2026-11-25 程序 Stop 收尾: 此后 parser 不再入队 (运动段唯一生产者已停) ----
            // 向 RT 提交 stop_flush_request: 运动中先经 HOLD_BRAKING 刹停 (约 200ms),
            // 到 HOLD_PAUSED 或静止后由 RT 丢弃全部残余段并关工艺输出 (不进报警态)。
            // 必须由 parser 在这里提交, 不能由 API 线程提前提交: 否则 RT 清完队列后
            // parser 仍可能推入残余段 (Hazard 2 同类撕裂)。
            const int stop_pending =
                atomic_exchange_explicit(&g_parser_ctrl.stop_flush_pending, 0,
                                         memory_order_acq_rel);
            if (stop_pending) {
                g_parser_ctrl.is_paused = 0;        // 解除 parser 侧暂停 (Stop 不再可"继续")
                g_interpolator.pause_request = 1;   // 未停稳则刹停; RT 清队列时一并解除
                atomic_store_explicit(&g_interpolator.stop_flush_request, 1,
                                      memory_order_release);
                printf("[Parser] 程序 Stop: 已提交 RT 清队列, 残余段将被丢弃\n");
            }

            // ---- 程序被中止 (Stop / 急停 / 报警联动 / RPC abort / 解析失败): 模态回安全默认 ----
            // 此前中止路径跳过 M30, M3 出光/冷却液/刀补等模态残留到下个程序 (见
            // gcode_reset_program_modals 注释)。队列里已入队的段带的是入队时快照, 不受影响。
            if (abort_file) {
                gcode_reset_program_modals();
            }

            // 2026-08-27: Abort-while-paused 收尾 — 暂停中中止时 feedhold 仍挂着
            // (pause_request=1 / HOLD_PAUSED / time_scale=0), 无人恢复则 flush 过的
            // 残余队列冻结永不消费 (软停止语义 = 清暂停让 RT 走完残余段)。
            // 正常完成/非暂停中止路径 is_paused 必为 0, 此清理只命中暂停中中止。
            if (g_parser_ctrl.is_paused) {
                g_parser_ctrl.is_paused = 0;
                api_motion_resume();   // pause_request=0 → RT HOLD_RESUMING
            }

            // Phase 2B M5: 子程序未返回检测 (非致命, 便于调试)
            if(!abort_file && g_call_stack_top != 0){
                printf("[Parser] 警告: 文件结束时调用栈非空 (depth=%d, 子程序缺 M99?)\n",
                       g_call_stack_top);
                g_call_stack_top = 0;  // 强制清零, 防污染下次运行
            }

            if(!abort_file){
                printf("[Parser] 文件处理完成: %s (PC 步进 %d)\n",
                       g_parser_ctrl.filepath, g_pc_step_counter);
                /* P1-b: 程序生命周期事件 - 解析完成 (M30 自然结束)
                 * P2-A-0 (2026-07-16): 区分 PREVIEW vs RUN 完成事件码
                 *   - PREVIEW 走 0x0031 (LoadProgram done), UI 据此知道可 RunLoadedProgram
                 *   - RUN 走 0x0033 (program done), UI 据此知道加工结束
                 * 修复 P1-b 已知限制 #2 (0x0031 之前未独立 instrument). */
                EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER,
                                 g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ? 0x0031 : 0x0033,
                                 g_pc_step_counter,
                                 g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ?
                                    "LoadProgram done (preview)" : "program done (M30 or EOF reached)");
            }

            // ---- P0-b v2: LoadProgram (PREVIEW 模式) 完成信号 ----
            // 设 g_program_load_done=1, UI 据此知道可调 RunLoadedProgram。
            // program_mode 不清, 由 SMC_RunLoadedProgram 切回 RUN。
            // RUN 模式完成时 load_done 保持 0 (它只跟踪 LoadProgram 状态)。
            if (g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW) {
                atomic_store_explicit(&g_program_load_done, 1, memory_order_release);
                printf("[Parser] LoadProgram 完成: first_seg=%llu last_seg=%llu bbox_min=(%.2f,%.2f,%.2f) bbox_max=(%.2f,%.2f,%.2f)\n",
                       (unsigned long long)g_program_first_seg_id,
                       (unsigned long long)g_program_last_seg_id,
                       g_program_bbox_min[0], g_program_bbox_min[1], g_program_bbox_min[2],
                       g_program_bbox_max[0], g_program_bbox_max[1], g_program_bbox_max[2]);
            }

            g_parser_ctrl.is_running=0; // 处理完成后重置状态
            g_parser_ctrl.abort_request=0; // 重置中止请求
        }

        osal_usleep(50000); // 主循环每50ms检查一次状态
    }
}

// @Context: Non-RealTime Background Thread (parser)
// @Safe: 纯坐标计算 + api_push_trajectory 入队, 与 generate_arc_trajectory 同级
//
// 固定循环展开 (G81/G82/G83):
//   1. 快速 XY 到孔位 (Z 保持当前)
//   2. 快速 Z 降到 R 平面 (若当前 Z 高于 R)
//   3. 工进 Z 到孔底:
//      - G81/G82: 单次工进
//      - G83: 啄钻循环 (每 cycle_peck_depth mm 退回 R 排屑, 安全上限 10000 次)
//   4. 快速 Z 退回: G98=cycle_initial_Z, G99=R 平面
//
// 限制 (Phase 2A.2 简化版):
//   - G82 孔底暂停 (cycle_dwell_ms) 暂未实现, 行为同 G81
//   - G83 退回后无 "快速降至距上次切削 0.5mm" 优化
//
// 安全检查:
//   - Z 轴必须已映射
//   - Z_bottom 必须 < R_plane
//   - G83 cycle_peck_depth 必须 > 0
//   - G83 啄钻次数 <= 10000 (防 Q 极小死循环)
//
// 返回值: 0=成功, -1=安全检查失败或入队被拒
int generate_fixed_cycle(double target_pos[AXIS_NUM],
                          double start_pos[AXIS_NUM],
                          double feedrate_mm_min)
{
    int z_idx = g_axis_map['Z' - 'A'];
    if(z_idx < 0){
        printf("[Parser] 固定循环要求 Z 轴已映射！\n");
        return -1;
    }

    double R_plane   = g_state.cycle_R_plane;
    double Z_bottom  = g_state.cycle_Z_bottom;
    double retract_Z = (g_state.cycle_retract_mode == 99)
                        ? R_plane : g_state.cycle_initial_Z;

    // ---- 安全检查 ----
    if(Z_bottom >= R_plane){
        printf("[Parser] 固定循环 G%d: 孔底 Z(%.3f) 必须 < R 平面(%.3f)！拒绝\n",
               g_state.active_cycle, Z_bottom, R_plane);
        return -1;
    }
    if((g_state.active_cycle == 83 || g_state.active_cycle == 73)
       && g_state.cycle_peck_depth <= 0.0){
        printf("[Parser] G%d 啄钻步进 Q 必须 > 0！当前 Q=%.3f\n",
               g_state.active_cycle, g_state.cycle_peck_depth);
        return -1;
    }

    double pos[AXIS_NUM];
    memcpy(pos, start_pos, sizeof(double) * AXIS_NUM);

    double rapid_speed = RAPID_SPEED_MM_MIN / 60.0;   // mm/s
    double feed_speed  = feedrate_mm_min / 60.0;
    if(feed_speed < 1e-6) feed_speed = 1e-6;

    // ---- 1. 快速 XY 到孔位 (Z 保持) ----
    for(int i = 0; i < AXIS_NUM; i++){
        if(i == z_idx) continue;
        pos[i] = target_pos[i];
    }
    if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

    // ---- 2. 快速 Z 降到 R 平面 (若当前 Z 高于 R) ----
    if(pos[z_idx] > R_plane){
        pos[z_idx] = R_plane;
        if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;
    }

    // ---- 3. 工进 / 啄钻 ----
    if(g_state.active_cycle == 73){
        // P3': G73 高速啄钻 (断屑, 不排屑)
        // 与 G83 区别: 每次工进后仅局部退回 RETRACT_MM (默认 0.5mm), 不退到 R
        // 工业价值: 少 50-70% 空行程时间 (浅-中深孔节拍优先场景)
        const int peck_limit = 10000;
        const double RETRACT_MM = 0.5;  // Fanuc 默认 d=0.5mm (参数 5101)
        double cur_Z = pos[z_idx];      // 当前 Z (已降到 R 或更低)
        int peck_count = 0;

        while(cur_Z > Z_bottom + 1e-9 && peck_count < peck_limit){
            // 本次工进终点: cur_Z - peck_depth, 但不超 Z_bottom
            double next_Z = cur_Z - g_state.cycle_peck_depth;
            if(next_Z < Z_bottom) next_Z = Z_bottom;

            // 工进 (切削)
            pos[z_idx] = next_Z;
            if(api_push_trajectory(pos, feed_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

            // 局部断屑退刀 (仅未到孔底时; 已到孔底由第 4 步统一退刀)
            if(next_Z > Z_bottom + 1e-9){
                double retract_to = next_Z + RETRACT_MM;
                if(retract_to > R_plane) retract_to = R_plane;  // 钳制: 不超过 R
                pos[z_idx] = retract_to;
                if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;
            }

            cur_Z = next_Z;
            peck_count++;
        }
        if(peck_count >= peck_limit){
            printf("[Parser] G73 啄钻次数超限 %d, 中止 (Q=%.3f 可能过小)\n",
                   peck_limit, g_state.cycle_peck_depth);
            return -1;
        }
    }
    else if(g_state.active_cycle == 83){
        // G83: 啄钻循环 (Phase 2A++ 优化版)
        // 工业标准: 退回 R 排屑后, 快速降至 last_cut_Z + CLEARANCE_MM,
        //           再工进 peck_depth (减少空切削时间)
        // 切削量保证: 每次工进 feed_end_Z - feed_start_Z = peck_depth (最后一段可能更短)
        const int peck_limit = 10000;
        const double CLEARANCE_MM = 0.5;  // 快速降至距上次孔底 0.5mm
        double last_cut_Z = R_plane;     // 上次工进终点 (首次从 R 开始)
        int peck_count = 0;

        while(last_cut_Z > Z_bottom + 1e-9 && peck_count < peck_limit){
            // 本次工进起点: 首次为 R, 后续为 last_cut_Z + CLEARANCE_MM
            double feed_start_Z = (peck_count == 0)
                                  ? R_plane : last_cut_Z + CLEARANCE_MM;
            if(feed_start_Z > R_plane) feed_start_Z = R_plane;
            // 本次工进终点: peck_depth 之下, 但不超过 Z_bottom
            double feed_end_Z = feed_start_Z - g_state.cycle_peck_depth;
            if(feed_end_Z < Z_bottom) feed_end_Z = Z_bottom;

            // (非首次) 快速降至 feed_start_Z (排屑后回切)
            if(peck_count > 0){
                pos[z_idx] = feed_start_Z;
                if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;
            }

            // 工进 feed_end_Z (切削)
            pos[z_idx] = feed_end_Z;
            if(api_push_trajectory(pos, feed_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

            // 退回 R 平面 (排屑)
            pos[z_idx] = R_plane;
            if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

            last_cut_Z = feed_end_Z;
            peck_count++;
        }
        if(peck_count >= peck_limit){
            printf("[Parser] G83 啄钻次数超限 %d, 中止 (Q=%.3f 可能过小)\n",
                   peck_limit, g_state.cycle_peck_depth);
            return -1;
        }
    } else {
        // G81/G82: 单次工进
        pos[z_idx] = Z_bottom;
        if(api_push_trajectory(pos, feed_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

        // ---- Phase 2A++: G82 孔底暂停 (parser 级阻塞方案) ----
        // 实现: 等待队列排空 (确保工进到位) → 轮询 sleep dwell_ms
        // 已知限制:
        //   - 暂停期间 parser 阻塞, 后续 G-code 行延迟处理 (短 dwell 可接受)
        //   - dwell 期间通过 10ms 轮询响应 abort_request
        if(g_state.active_cycle == 82 && g_state.cycle_dwell_ms > 0.0){
            while(!is_trajectory_finished()){
                if(g_parser_ctrl.abort_request) return -1;
                osal_usleep(1000);  // 1ms 轮询队列状态
            }
            int dwell_us = (int)(g_state.cycle_dwell_ms * 1000.0);
            const int poll_step_us = 10000;  // 10ms 轮询步长
            int elapsed_us = 0;
            while(elapsed_us < dwell_us){
                if(g_parser_ctrl.abort_request) return -1;
                int step = (dwell_us - elapsed_us < poll_step_us)
                           ? (dwell_us - elapsed_us) : poll_step_us;
                osal_usleep(step);
                elapsed_us += step;
            }
        }
    }

    // ---- 4. 快速 Z 退回 ----
    pos[z_idx] = retract_Z;
    if(api_push_trajectory(pos, rapid_speed, DEFAULT_ACC, DEFAULT_DEC) < 0) return -1;

    return 0;
}

// @Context: Non-RealTime Background Thread (parser)
// offset_1st / offset_2nd: 圆心相对于起点在平面第一轴/第二轴方向的偏移
// feedrate_mm_min: G94 进给速度 (mm/min)，G93 模式下仅零半径退化时使用
// g93_T_sec: G93 模式下整段圆弧的可用时间(秒)，<=0 表示 G94 模式
// 返回值: 0=成功, -1=入队被拒(报警)
int generate_arc_trajectory(double start_pos[AXIS_NUM],double end_pos[AXIS_NUM],
                             double offset_1st, double offset_2nd,
                             int is_CW,double feedrate_mm_min,double g93_T_sec)
{
    // ---- 动态平面轴映射：查表获取 X/Y/Z 的真实轴索引 ----
    int idx_x = g_axis_map['X' - 'A'];
    int idx_y = g_axis_map['Y' - 'A'];
    int idx_z = g_axis_map['Z' - 'A'];
    int ax1 = -1, ax2 = -1;

    switch(g_state.active_plane){
        case 18: // ZX平面
            if(idx_z < 0 || idx_z >= AXIS_NUM || idx_x < 0 || idx_x >= AXIS_NUM){
                printf("[Parser] G18 ZX平面要求 X/Z 轴均已映射！\n"); return -1;
            }
            ax1=idx_z; ax2=idx_x; break;
        case 19: // YZ平面
            if(idx_y < 0 || idx_y >= AXIS_NUM || idx_z < 0 || idx_z >= AXIS_NUM){
                printf("[Parser] G19 YZ平面要求 Y/Z 轴均已映射！\n"); return -1;
            }
            ax1=idx_y; ax2=idx_z; break;
        default: // XY平面
            if(idx_x < 0 || idx_x >= AXIS_NUM || idx_y < 0 || idx_y >= AXIS_NUM){
                printf("[Parser] G17 XY平面要求 X/Y 轴均已映射！\n"); return -1;
            }
            ax1=idx_x; ax2=idx_y; break;
    }

    // ---- 1. 圆心坐标 ----
    double cx = start_pos[ax1] + offset_1st;
    double cy = start_pos[ax2] + offset_2nd;

    // ---- 2. 半径 ----
    double radius = hypot(start_pos[ax1] - cx, start_pos[ax2] - cy);
    if(radius < 0.001) {
        // 零半径退化：起终点重合则真无操作，否则退化为直线以防位置漂移
        double dist_sq = 0.0;
        for(int j = 0; j < AXIS_NUM; j++){
            double d = end_pos[j] - start_pos[j];
            dist_sq += d * d;
        }
        if(dist_sq < 1e-12) return 0;

        if(g_state.rtcp_enabled){
            // 零半径退化 + RTCP：按逻辑时间预算 ÷ 物理距离计算下发速度
            double logical_dist_sq = 0.0;
            for(int j = 0; j < AXIS_NUM; j++){
                double delta = end_pos[j] - start_pos[j];
                if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                    delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
                }
                logical_dist_sq += delta * delta;
            }
            double logical_dist = sqrt(logical_dist_sq);

            apply_rtcp_to_pos(end_pos);

            double phys_dist_sq = 0.0;
            for(int j = 0; j < AXIS_NUM; j++){
                double delta = end_pos[j] - api_get_cursor(j);
                if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                    delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
                }
                phys_dist_sq += delta * delta;
            }
            double phys_dist = sqrt(phys_dist_sq);

            double dt;
            if(g93_T_sec > 1e-9){
                dt = g93_T_sec;
            } else {
                double feed_mm_sec = feedrate_mm_min / 60.0;
                if(feed_mm_sec < 1e-6) feed_mm_sec = 1e-6;
                dt = (logical_dist > 1e-9) ? logical_dist / feed_mm_sec : 1.0;
            }
            double phys_speed = (dt > 1e-9) ? phys_dist / dt : 1e-6;
            if(phys_speed < 1e-6) phys_speed = 1e-6;
            // G93 强一致性: 豁免短板限幅
            if(g93_T_sec > 1e-9){
                return api_push_trajectory_g93(end_pos, phys_speed,
                                                DEFAULT_ACC, DEFAULT_DEC, g93_T_sec);
            }
            return api_push_trajectory(end_pos, phys_speed, DEFAULT_ACC, DEFAULT_DEC);
        } else {
            double speed_mm_sec;
            if(g93_T_sec > 1e-9){
                speed_mm_sec = sqrt(dist_sq) / g93_T_sec;
                if(speed_mm_sec < 1e-6) speed_mm_sec = 1e-6;
            }else{
                speed_mm_sec = feedrate_mm_min / 60.0;
            }
            return api_push_trajectory(end_pos, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC);
        }
    }

    // ---- 3. 起始/结束角度 ----
    double theta_start = atan2(start_pos[ax2] - cy, start_pos[ax1] - cx);
    double theta_end   = atan2(end_pos[ax2]   - cy, end_pos[ax1]   - cx);

    // ---- 4. 顺逆时针角度调整 ----
    if(is_CW){
        if(theta_end >= theta_start) theta_end -= 2.0 * PI;
    } else {
        if(theta_end <= theta_start) theta_end += 2.0 * PI;
    }

    double total_angle = theta_end - theta_start;

    // ---- 5. 分段数: G93 时间切分, G94 空间切分 ----
    double arc_length = fabs(total_angle) * radius;
    int num_segments;
    if(g93_T_sec > 1e-9){
        // G93 时间切分: 1ms 一段,与 RT 周期对齐,封顶 20000
        num_segments = (int)ceil(g93_T_sec * 1000.0);
        if(num_segments > 20000) num_segments = 20000;
    } else {
        num_segments = (int)ceil(arc_length / ARC_SEGMENT_LENGTH_MM);
    }
    if(num_segments < 1) num_segments = 1;

    // 螺旋真实空间长度 = sqrt(弧长² + 非平面轴位移²)
    double non_plane_dist_sq = 0.0;
    for(int j = 0; j < AXIS_NUM; j++){
        if(j != ax1 && j != ax2){
            double delta = end_pos[j] - start_pos[j];
            if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
            }
            non_plane_dist_sq += delta * delta;
        }
    }
    double helical_length = sqrt(arc_length * arc_length + non_plane_dist_sq);

    // G93: 用螺旋真实长度 / T_sec 反推速度，保证绝对时间到达
    double speed_mm_sec;
    if(g93_T_sec > 1e-9){
        speed_mm_sec = helical_length / g93_T_sec;
        if(speed_mm_sec < 1e-6) speed_mm_sec = 1e-6;
    }else{
        speed_mm_sec = feedrate_mm_min / 60.0;
    }

    // 每微段逻辑时间预算 (G93 强一致性必填, G94 RTCP 也用)
    double dt_per_seg = 0.0;
    if(g93_T_sec > 1e-9){
        // G93: 整段时间预算均分到每微段,RTCP/非 RTCP 一致
        dt_per_seg = g93_T_sec / (double)num_segments;
        // B0′ 消融 (论文1, 2026-09-17): 圆弧侧同样回退为 F-inheritance;
        // 发射循环 seg_speed = phys_dist/dt_per_seg 自动保持一致。
        if(g_g93_f_inherit) dt_per_seg = g93_T_sec;
    } else if(g_state.rtcp_enabled){
        double seg_logical_dist = helical_length / (double)num_segments;
        double feed_mm_sec = feedrate_mm_min / 60.0;
        if(feed_mm_sec < 1e-6) feed_mm_sec = 1e-6;
        dt_per_seg = seg_logical_dist / feed_mm_sec;
    }

    double angle_step = total_angle / num_segments;
    // P0: 圆弧微段逻辑步长 (循环不变量) — 奇异点逐微段检查用
    // arc_logical_step     : 含旋转轴折算 (helical_length 原定义), 供速度/时间预算。
    // arc_logical_step_tip : 纯刀尖弧长 (跳过旋转轴折算), 仅供奇异点检查 ——
    //   P0-Fix (2026-09-04): 旧实现 r_seg 分母用 helical_length, 其中旋转轴折算
    //   (TC-6: B60°×80mm=83.78mm) 占主导, 把纯弧长 6.28mm 稀释 ~13 倍,
    //   r_seg≈1.x 永远够不到阈值 —— 与直线侧 R_sing 的 root cause (分母含旋转
    //   折算稀释) 完全同构。修复后分母 = 刀尖真实弧长, 分子仍为物理全轴步长
    //   (含折算), 语义 = "单位刀尖位移要求的物理轴运动倍数"。
    // @Context: parser 线程 (非 RT), 循环不变量, 无共享状态访问
    // @Safe: 纯算术, 无 malloc/阻塞
    double non_plane_tip_sq = 0.0;
    for(int j = 0; j < AXIS_NUM; j++){
        if(j != ax1 && j != ax2 && g_axis[j].axis_type != 1){
            double delta = end_pos[j] - start_pos[j];
            non_plane_tip_sq += delta * delta;
        }
    }
    double helical_length_tip = sqrt(arc_length * arc_length + non_plane_tip_sq);
    double arc_logical_step     = helical_length / (double)num_segments;
    double arc_logical_step_tip = helical_length_tip / (double)num_segments;
    double next_pos[AXIS_NUM];
    /* P0-Fix (2026-09-03): 上一微段的物理坐标 (RTCP 逆解后), 用于计算【微段自身】
     * 物理步长。不可用 api_get_cursor() 代替 —— 那是规划游标, parser 领先 RT
     * (队列积压) 时会滞后数百个微段, 使 delta 变成"累计位移"而非单段步长,
     * 令 r_seg 被放大约队列长度倍 → 末段 (强制对齐 end_pos) 必然误报奇异点。
     * 与直线侧 estimate_max_jacobian_ratio() 的纯几何定义保持一致。 */
    double prev_phys[AXIS_NUM];
    int    have_prev_phys = 0;

    // ---- 5.5 论文1 S10: block 计时插桩 (仅 G93) ----
    if(g93_T_sec > 1e-9){
        bt_block_open(g93_T_sec * 1000.0, num_segments,
                      dt_per_seg * (double)num_segments * 1000.0,
                      g_current_line_no);
    }

    // ---- 6. 逐点插补 ----
    for(int i = 1; i <= num_segments; i++){
        double theta = theta_start + i * angle_step;

        // 圆弧平面轴：精确三角函数投影
        next_pos[ax1] = cx + radius * cos(theta);
        next_pos[ax2] = cy + radius * sin(theta);

        // 非平面轴（第三线性轴 + 所有旋转轴）：线性跟随
        double progress_ratio = (double)i / (double)num_segments;
        for(int j = 0; j < AXIS_NUM; j++){
            if(j != ax1 && j != ax2){
                next_pos[j] = start_pos[j] + (end_pos[j] - start_pos[j]) * progress_ratio;
            }
        }

        // 末段强制对齐终点，消除浮点累积误差
        if(i == num_segments){
            for(int j = 0; j < AXIS_NUM; j++){
                next_pos[j] = end_pos[j];
            }
        }

        double seg_speed;
        if(g_state.rtcp_enabled){
            // RTCP 逐点逆解 + 物理速度修正
            apply_rtcp_to_pos(next_pos);

            double phys_dist_sq = 0.0;
            for(int j = 0; j < AXIS_NUM; j++){
                double delta = next_pos[j] - api_get_cursor(j);
                if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                    delta = delta * (PI / 180.0) * g_axis[j].equivalent_radius;
                }
                phys_dist_sq += delta * delta;
            }
            double phys_dist = sqrt(phys_dist_sq);

            // P0 奇异点硬报警: 逐微段 phys/log 比, 与直线侧 R_max 同定义同阈值。
            // 圆弧无预估采样, 微段级检查密度更高; 中途报警时前段已入队,
            // alarm 置位后 RT 走 HOLD_BRAKING 安全停车 (既有语义)。
            //
            // P0-Fix (2026-09-03): r_seg 必须用【本微段自身】物理步长 (相邻采样点
            // 之差), 不能用 phys_dist (相对滞后规划游标)。否则队列积压时
            // r_seg 被放大约队列长度倍。
            //
            // 末段 (i == num_segments) 豁免: 该段被强制对齐到 end_pos 以消除浮点
            // 累积误差。当 end_pos 与圆弧几何不自洽 (例: G02 X10 Y10 I5 J0 ——
            // 圆心(5,0) 半径 5, 而终点(10,10) 到圆心距离 11.18 ≠ 5, 不在圆上) 时,
            // 末段会出现毫米级位置瞬跳 (实测 6.18mm), 这是几何修正而非 Jacobian
            // 奇异, 用它做比值必然误报 (实测 r=970.8>30 触发报警急停, 运动腰斩)。
            // 注: 该瞬跳本身是既有缺陷 (G93 路径豁免短板限幅 → 超速), 独立跟踪。
            if(g_rtcp_singularity_rmax > 0.0 && arc_logical_step > 1e-9
               && i < num_segments){
                double step_pos[AXIS_NUM];
                if(!have_prev_phys){
                    memcpy(step_pos, start_pos, sizeof(step_pos));
                    apply_rtcp_to_pos(step_pos);   // 起点物理坐标 (首微段基准)
                    have_prev_phys = 1;
                } else {
                    memcpy(step_pos, prev_phys, sizeof(step_pos));
                }
                double step_sq = 0.0;
                for(int j = 0; j < AXIS_NUM; j++){
                    double d = next_pos[j] - step_pos[j];
                    if(g_axis[j].axis_type == 1 && g_axis[j].equivalent_radius > 0.0){
                        d = d * (PI / 180.0) * g_axis[j].equivalent_radius;
                    }
                    step_sq += d * d;
                }
                double step_dist = sqrt(step_sq);
                memcpy(prev_phys, next_pos, sizeof(prev_phys));

                // P0-Fix (2026-09-04): 分母改用 arc_logical_step_tip (纯刀尖弧长),
                // 旧分母 arc_logical_step 含旋转轴折算会把 r 稀释 ~13 倍 (见上)。
                double r_seg = step_dist / arc_logical_step_tip;
                if(r_seg > g_rtcp_singularity_rmax){
                    char ev_buf[64];
                    snprintf(ev_buf, sizeof(ev_buf), "RTCP arc singular r=%.1f>%.1f seg %d/%d",
                             r_seg, g_rtcp_singularity_rmax, i, num_segments);
                    printf("[RTCP] 圆弧奇异点报警: %s (中止文件)\n", ev_buf);
                    EventLogger_Push(SEVERITY_ALARM, SOURCE_PARSER, 0x0026,
                                     (int32_t)(r_seg * 10.0), ev_buf);
                    atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                    return -1;
                }
            }

            seg_speed = (dt_per_seg > 1e-9) ? phys_dist / dt_per_seg : 1e-6;
            if(seg_speed < 1e-6) seg_speed = 1e-6;
        } else {
            seg_speed = speed_mm_sec;
        }

        // G93 模式下 seg_speed 已按 phys_dist/dt_per_seg 或 helical_length/T_sec 精确推算,
        // 必须走强一致性路径豁免短板限幅,否则时间预算被静默破坏。
        int push_ret;
        if(g93_T_sec > 1e-9){
            push_ret = api_push_trajectory_g93(next_pos, seg_speed,
                                                DEFAULT_ACC, DEFAULT_DEC, dt_per_seg);
        } else {
            push_ret = api_push_trajectory(next_pos, seg_speed,
                                            DEFAULT_ACC, DEFAULT_DEC);
        }
        if(push_ret < 0){
            bt_block_close();
            return -1;
        }
    }
    bt_block_close();

    printf("[Parser] 生成了 %d 个圆弧插补点 (平面 G%d, 速度 %.2f mm/s)\n",
           num_segments, g_state.active_plane, speed_mm_sec);
    return 0;
}