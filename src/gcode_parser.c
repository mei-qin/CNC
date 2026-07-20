#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "kinematics.h"
#include "bspline_engine.h"
#include "macro_eval.h"   // 宏变量与表达式引擎
#include "program_loader.h"  // Phase 2B M1: 文件加载器 + N 标签 + GOTO
#include "event_logger.h"    // P1-b: 参数校验报警 + 程序生命周期
#include "smc_protocol.h"    // P2-A: SMC_MODE_* 位定义 (M30 override 重置用)
#include <math.h>
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

// @Context: Non-RealTime Background Thread
// @Safe: 纯坐标变换，调用 Kinematics_Inverse 纯函数。
// 动态适配 AC / BC / ABC 构型：
//   - X/Y/Z 必须全部映射，否则无法逆解
//   - A/B/C 未映射时角度取 0.0，不影响已配置的旋转轴
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
#define RTCP_JACOBIAN_SAMPLES_MIN  21
#define RTCP_JACOBIAN_SAMPLES_MAX  200
#define RTCP_JACOBIAN_SPACING_MM   5.0
#define RTCP_R_MAX_DEGENERATE      10.0  // 退化路径保守值
static double estimate_max_jacobian_ratio(double start_pos[AXIS_NUM],
                                          double end_pos[AXIS_NUM])
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

    // ---- 计算逻辑总距离 (含旋转轴弧长折算,与打碎逻辑一致) ----
    double logical_total_sq = 0.0;
    for(int j = 0; j < AXIS_NUM; j++){
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
        return api_push_trajectory_rtcp(phys_end, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC);
    }

    // ---- 1. Jacobian 预估 (21 点采样) ----
    double R_max = estimate_max_jacobian_ratio(start_pos, end_pos);

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
        double phys_speed = (dt_per_seg > 1e-9) ? phys_dist / dt_per_seg : 1e-6;
        if(phys_speed < 1e-6) phys_speed = 1e-6;

        // G93 模式走强一致性路径,豁免底层短板限幅,刚性守恒时间预算。
        // G94 模式走标准路径 + RTCP 标记。
        // 注意: G93 路径暂未提供 _rtcp 变体,因为 G93 已豁免短板限幅,
        //       is_rtcp_active 元数据对 G93 段意义不大 (RT 线程不区分对待)。
        //       后续若需 G93 RTCP 元数据,可扩展 api_push_trajectory_g93_rtcp。
        int push_ret;
        if(g93_T_sec > 1e-9){
            push_ret = api_push_trajectory_g93(interp_pos, phys_speed,
                                                DEFAULT_ACC, DEFAULT_DEC, dt_per_seg);
        } else {
            push_ret = api_push_trajectory_rtcp(interp_pos, phys_speed,
                                                 DEFAULT_ACC, DEFAULT_DEC);
        }
        if(push_ret < 0){
            return -1;
        }
    }

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

        // 括号注释 (...) — 支持 M03(LSON) 等紧贴写法
        if(*p=='('){
            while(*p!=')' && *p!='\0') p++;
            if(*p==')') p++;
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
                else if(fabs(value - 43.4) < 0.05) g_state.rtcp_enabled = 1; // G43.4 开启RTCP
                else if(value >= 49.0 && value < 50.0) g_state.rtcp_enabled = 0; // G49 关闭RTCP
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
            case 'I':offset_i=value;has_move=1;break;
            case 'J':offset_j=value;has_move=1;break;
            case 'K':offset_k=value;has_move=1;break;
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
                    int src_line = g_current_program ?
                                   g_current_program->lines[g_pc].line_no : -1;
                    g_parser_ctrl.is_paused = 1;
                    api_motion_pause();
                    printf("[Parser] M0 程序暂停 (源行 %d, 已发 motion_pause)\n", src_line);
                    m_code = -1;  // 不入队
                }
                // P0-2: M1 可选停 — 受 g_optional_stop_enabled 开关控制
                // 开关关 (默认): M1 视为 no-op (兼容现有 NC 程序)
                // 开关开 (HMI 显式启用): M1 等价 M0, 走真暂停路径
                else if(m_code == 1){
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
                // P0-1: M2/M30 安全停 5 步流程
                // 旧行为: 直接 is_running=0, 后果 RT 队列里残留运动继续走、spindle/coolant 不关
                // 新行为: 平滑减速 -> 等队列空 -> 模态复位 -> 抢写 RT 镜像 -> 关 parser
                // 抢写 RT 镜像是 RT 单写者例外: 此时队列已空, RT 不会再消费任何 M 段,
                // 写者已 quiescent, parser 安全抢写一次 (与 api_motion_resume 同级别操作)
                else if(m_code == 2 || m_code == 30){
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
                    g_state.pending_comp_mode  = COMP_OFF;   // G41/G42
                    g_state.local_offset_active= 0;          // G52
                    g_state.modal_ext_wcs_p    = 0;          // G54.1 Pn
                    // P0-Laser: M30 强制关激光 (与 spindle 一同复位)
                    g_state.laser_shutter_pending = 0;
                    g_state.laser_power_pending   = 0.0;
                    g_state.laser_freq_pending    = 0.0;
                    g_state.gas_select            = 0;
                    // Laser B4: 清段级工艺标记 modal (防跨程序泄漏)
                    g_state.laser_seg_flags       = 0;

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
                    // P0-Laser: 触发 RT 下个 cycle 强制记录一条 sim_engine 采样,
                    // 让 CSV 末尾出现归零状态 (否则 should_log=0 → sim_engine_push 不调).
                    atomic_store_explicit(&g_sim_force_log, 1, memory_order_release);

                    // Step 5: 关 parser 主循环
                    g_parser_ctrl.is_running = 0;
                    printf("[Parser] M%d 安全停完成 (耗时 %.1fs)\n",
                           m_code, wait_ms / 1000.0);
                    m_code = -1;   // 不入队
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
                else if(m_code == 70 || m_code == 71){
                    // 模态 M 代码, m_code 保持原值, 处理延迟到字母循环后 (与 M50 同模式)
                }
                else if(m_code == 62 || m_code == 63 || m_code == 67 ||
                        m_code == 68 || m_code == 10 || m_code == 11 || m_code == 12){
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
    // LinuxCNC 风格 v1: 直接 homing, 不解析 X_Y_Z_ 中间点 (Fanuc 风格留 v2)
    // 调 axis_homing_multi 阻塞 parser_thread, 期间 RT 冻结 motion queue
    // G28 后续段在 homing 完成后继续解析 (v2: homing_shift 已重新锚定, home_offset 常量不动)
    if(is_g28_block){
        if(!g_homing_cfg.enabled || g_homing_cfg.order_count == 0){
            printf("[Parser] G28: homing 未配置, 调 SMC_ConfigHomingAll 先\n");
            atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
            return -1;
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
            for(int i=0;i<AXIS_NUM;i++){
                double w;
                if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
                    w = g_coord_mgr.work_offsets_ext[g_state.modal_ext_wcs_p - 1][i];
                } else {
                    w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
                }
                if(g_state.local_offset_active && wcs_idx >= 0) w += g_state.local_offset[i];
                start_pos[i] = g_state.current_pos[i];
                machine_start_pos[i] = start_pos[i] + w;
                machine_target_pos[i] = has_axis[i] ? val_axis[i] : machine_start_pos[i];
                // 反推逻辑坐标 (在 WCS+G52 空间), 保证下一行回到正常模式时起点不撕裂
                target_pos[i] = machine_target_pos[i] - w;
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
                machine_target_pos[i] = target_pos[i] + w;
                machine_start_pos[i] = start_pos[i] + w;
            }
        }

        // RTCP 逆解：圆弧由 arc generator 逐点逆解，直线由 generate_linear_rtcp_trajectory 处理
        // 不再在此处对起终点做硬算，避免旋转运动时中间轨迹挖坑

        // 多轴等效距离计算（用于 G93 速度反推）
        double dist_total = 0.0;
        for(int i=0;i<AXIS_NUM;i++){
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
                if(CutterComp_PushPoint(machine_target_pos, speed_mm_sec,
                                        DEFAULT_ACC, DEFAULT_DEC) < 0){
                    printf("[Parser] 刀补引擎入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            } else if(!will_bypass_bspline){
                // G01 + B-Spline 模式: 通过平滑引擎入队
                if(BSpline_PushDirtyPoint(machine_target_pos, speed_mm_sec, g93_T_sec) < 0){
                    printf("[Parser] B-Spline入队失败(报警)，中止当前文件！\n");
                    return -1;
                }
            } else {
                // 直通: G00 或 B-Spline 未启用
                if(api_push_trajectory(machine_target_pos,speed_mm_sec,DEFAULT_ACC,DEFAULT_DEC) < 0){
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
                   g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ? "PREVIEW" : "RUN");

            /* P1-b: 程序生命周期事件 - 解析启动 */
            EventLogger_Push(SEVERITY_INFO, SOURCE_PARSER,
                             g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ? 0x0030 : 0x0032,
                             0, g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW ?
                                "LoadProgram start (preview)" : "RunLoadedProgram start");

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
            atomic_store_explicit(&g_program_load_done, 0, memory_order_release);

            // ---- Phase 2B M1: 全文加载替代 fgets 流 ----
            // 一次性把文件读入内存, 建立 N 标签表, 供 PC 游标遍历 + GOTO 跳转
            GCodeProgram_t *prog = Program_Load(g_parser_ctrl.filepath);
            if(!prog){
                printf("[Parser错误] 加载失败: %s\n", g_parser_ctrl.filepath);
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
                // 暂停检查
                while(g_parser_ctrl.is_paused){
                    osal_usleep(100000); // 暂停时每100ms检查一次状态
                }

                // 解析当前行 G-code 命令 (PC 指向), 入队失败或 GOTO 错则中止文件
                if(parse_gcode_line(prog->lines[g_pc].text) < 0){
                    printf("[Parser] 行 %d 解析失败, 中止文件: %s\n",
                           prog->lines[g_pc].line_no, g_parser_ctrl.filepath);
                    abort_file = 1;
                    break;
                }

                // 跳转信号消费: parse_gcode_line 内部设置 g_pc_jump_pending
                if(g_pc_jump_pending){
                    g_pc = g_pc_jump_target;
                    g_pc_jump_pending = 0;
                } else {
                    g_pc++;
                }
            }

            Program_Free(prog);
            g_current_program = NULL;

            if(g_state.bspline_enabled){
                BSpline_Flush();
            }
            api_flush_planner();

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
    } else if(g_state.rtcp_enabled){
        double seg_logical_dist = helical_length / (double)num_segments;
        double feed_mm_sec = feedrate_mm_min / 60.0;
        if(feed_mm_sec < 1e-6) feed_mm_sec = 1e-6;
        dt_per_seg = seg_logical_dist / feed_mm_sec;
    }

    double angle_step = total_angle / num_segments;
    double next_pos[AXIS_NUM];

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
            return -1;
        }
    }

    printf("[Parser] 生成了 %d 个圆弧插补点 (平面 G%d, 速度 %.2f mm/s)\n",
           num_segments, g_state.active_plane, speed_mm_sec);
    return 0;
}