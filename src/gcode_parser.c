#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "kinematics.h"
#include "bspline_engine.h"
#include <math.h>
#define PI 3.14159265358979323846
#define ARC_SEGMENT_LENGTH_MM 0.5 // 圆弧插补时的分段长度，单位mm
#define RTCP_LINEAR_SEGMENT_MM 0.5 // RTCP直线微段打碎步长，单位mm

GCodeState_t g_state = {{0}, 1000.0, 1, 17, 1, FEED_MODE_G94, 0, 0, 0}; // rtcp=0, bspline=0, comp=OFF
ParserControl_t g_parser_ctrl = {"", 0, 0, 0}; // 全局G-code解析控制变量，初始值为未运行、未暂停、未请求中止
extern int api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_trajectory_g93(double target_pos[AXIS_NUM],double speed,double acc,double dec,double g93_dt_sec);
extern int api_push_trajectory_passthrough(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_trajectory_rtcp(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_mcode(int m_code, double s_value, double p_value, double q_value, double r_value);

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

int parse_gcode_line(const char *gcode_line)
{
    char buffer[128];
    strncpy(buffer, gcode_line, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    int has_move=0;
    int has_axis[AXIS_NUM]={0};
    double val_axis[AXIS_NUM]={0};
    double offset_i=0.0,offset_j=0.0,offset_k=0.0; // 圆弧偏移：非模态，逐行清零
    int m_code=-1;
    double s_value=0.0;
    double p_value=0.0, q_value=0.0, r_value=0.0; // M代码扩展参数
    int is_non_motion_g=0; // 非运动组拦截锁：G04/G10/G28/G92 等
    int has_f=0;           // F 值存在标志（G93 非模态校验）
    int is_G53_this_block=0;  // G53 非模态机械坐标：仅影响本行

    char *p=buffer;
    while(*p!='\0'){
        p=(char*)skip_spaces(p);
        if(*p=='\0') break;

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

        char letter=toupper(*p);
        p++;
        double value=strtod(p, &p);

        switch(letter){
            case 'G':
                if(value==0.0)      g_state.motion_mode=0; // G00 快速
                else if(value==1.0) g_state.motion_mode=1; // G01 直线
                else if(value==2.0) g_state.motion_mode=2; // G02 顺弧
                else if(value==3.0) g_state.motion_mode=3; // G03 逆弧
                else if(value==4.0)  is_non_motion_g=1;    // G04 暂停
                else if(value==10.0) is_non_motion_g=1;    // G10 数据设定
                else if(value==17.0) g_state.active_plane=17;
                else if(value==18.0) g_state.active_plane=18;
                else if(value==19.0) g_state.active_plane=19;
                else if(value==28.0) is_non_motion_g=1;    // G28 返回参考点
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
                else if(value==93.0) g_state.feed_mode=FEED_MODE_G93; // G93 倒数时间
                else if(value==94.0) g_state.feed_mode=FEED_MODE_G94; // G94 每分钟
                else if(fabs(value - 43.4) < 0.05) g_state.rtcp_enabled = 1; // G43.4 开启RTCP
                else if(value >= 49.0 && value < 50.0) g_state.rtcp_enabled = 0; // G49 关闭RTCP
                else if(value>=54.0 && value<=59.0) g_coord_mgr.current_coord = (int)value - 53; // 54->1(G54), 55->2(G55)...
                break;
            case 'F':g_state.feedrate_mm_min=value;has_f=1;break;
            case 'I':offset_i=value;has_move=1;break;
            case 'J':offset_j=value;has_move=1;break;
            case 'K':offset_k=value;has_move=1;break;
            case 'M':m_code=(int)value;break;
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
            case 'P':p_value=value;break;
            case 'Q':q_value=value;break;
            case 'R':r_value=value;break;
            case 'S':s_value=value;break;
            default:
                // 动态轴映射：任何 A-Z 字母若在 g_axis_map 中有映射则视为运动轴
                if(letter >= 'A' && letter <= 'Z'){
                    int idx = g_axis_map[letter - 'A'];
                    if(idx >= 0 && idx < AXIS_NUM){
                        val_axis[idx] = value;
                        has_move = 1;
                        has_axis[idx] = 1;
                    }
                    // 未映射字母静默忽略（可能是注释残留或非标指令）
                }
                break;
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
        } else {
            if(api_push_mcode(m_code, s_value, p_value, q_value, r_value) < 0){
                printf("[Parser] M代码入队失败(报警)，中止当前文件！\n");
                return -1;
            }
            printf("[Parser] 解析M代码: M%02d S%.1f P%.1f Q%.1f R%.1f\n", m_code, s_value, p_value, q_value, r_value);
        }
    }

    // 运动门控：仅当本行包含显式轴运动且非运动参数指令时才触发轨迹下发
    if(has_move && !is_non_motion_g){
        double target_pos[AXIS_NUM];   // 逻辑坐标（工件坐标系）
        double start_pos[AXIS_NUM];
        double machine_target_pos[AXIS_NUM];
        double machine_start_pos[AXIS_NUM];

        // 工件坐标偏置查询（G53 行内标记不影响模态 WCS）
        int wcs_idx = (g_coord_mgr.current_coord >= COORD_G54 &&
                       g_coord_mgr.current_coord <= COORD_G59)
                      ? (g_coord_mgr.current_coord - 1) : -1;

        if(is_G53_this_block){
            // G53 非模态：val_axis 是机械绝对坐标，强制 G90，忽略 G91
            for(int i=0;i<AXIS_NUM;i++){
                double w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
                start_pos[i] = g_state.current_pos[i];
                machine_start_pos[i] = start_pos[i] + w;
                machine_target_pos[i] = has_axis[i] ? val_axis[i] : machine_start_pos[i];
                // 反推逻辑坐标，保证下一行回到正常模式时起点不撕裂
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
                double w = (wcs_idx >= 0) ? g_coord_mgr.work_offsets[wcs_idx][i] : 0.0;
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
            if(generate_arc_trajectory(machine_start_pos,
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

            // ---- 刀具半径补偿路由 ----
            // 补偿激活时，G01/G00 直线段通过 CutterComp_PushPoint 走偏置引擎，
            // 引擎内部计算偏置后通过回调下发到 B-Spline 或 Planner。
            // G02/G03 圆弧和 RTCP 直通，不经过刀补引擎。
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

    return 0;
}

OSAL_THREAD_FUNC parser_thread_func(void *arg){
    char line_buffer[256];
    
    while(1){

        //1.
        if(g_parser_ctrl.is_running==1){


            while(!g_all_axis_op_ready){
                osal_usleep(100000); // 等待所有轴准备就   
            }
            
            printf("[Parser] Processing file: %s\n", g_parser_ctrl.filepath);
            FILE *fp=fopen(g_parser_ctrl.filepath,"r");
            if(fp==NULL){
                printf("[Parser错误] 无法打开文件: %s\n", g_parser_ctrl.filepath);
                g_parser_ctrl.is_running=0;
                continue;
            }

            while(!is_trajectory_finished()){
                osal_usleep(100000); // 等待当前轨迹执行完成，检查频率为100ms
            }

            api_sync_planner_cursor(); // 同步规划器光标，确保新轨迹从当前状态开始

            for(int i=0;i<AXIS_NUM;i++){
                g_state.current_pos[i]=api_get_cursor(i);
            }
            //g_state.current_x_mm=api_get_cursor_x();
            //g_state.current_y_mm=api_get_cursor_y();
            //g_state.current_z_mm=api_get_cursor_z();

            while(fgets(line_buffer,sizeof(line_buffer),fp)!=NULL){
                if(g_parser_ctrl.abort_request){
                    printf("[Parser] 中止请求已收到，停止解析文件: %s\n", g_parser_ctrl.filepath);
                    break;
                }
                // 暂停检查
                while(g_parser_ctrl.is_paused){
                    osal_usleep(100000); // 暂停时每100ms检查一次状态
                }
                // 解析当前行G-code命令，入队失败(报警)则中止文件
                if(parse_gcode_line(line_buffer) < 0){
                    printf("[Parser] 入队失败，中止文件解析: %s\n", g_parser_ctrl.filepath);
                    break;
                }
            }
            fclose(fp);
            if(g_state.bspline_enabled){
                BSpline_Flush();
            }
            api_flush_planner();
            printf("[Parser] 文件处理完成: %s\n", g_parser_ctrl.filepath);
            g_parser_ctrl.is_running=0; // 处理完成后重置状态
            g_parser_ctrl.abort_request=0; // 重置中止请求
        }

        osal_usleep(50000); // 主循环每50ms检查一次状态
    }
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