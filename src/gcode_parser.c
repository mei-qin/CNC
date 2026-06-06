#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "kinematics.h"
#include <math.h>
#define PI 3.14159265358979323846
#define ARC_SEGMENT_LENGTH_MM 0.5 // 圆弧插补时的分段长度，单位mm
#define RTCP_LINEAR_SEGMENT_MM 0.5 // RTCP直线微段打碎步长，单位mm

GCodeState_t g_state = {{0}, 1000.0, 1, 17, 1, FEED_MODE_G94, 0}; // rtcp_enabled=0
ParserControl_t g_parser_ctrl = {"", 0, 0, 0}; // 全局G-code解析控制变量，初始值为未运行、未暂停、未请求中止
extern int api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
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
// @Safe: 纯坐标变换 + 调用 api_push_trajectory（队列写入，线程安全）。
// RTCP 直线微段打碎：当直线段包含旋转轴运动时，按 0.5mm 等效步长
// 将逻辑空间直线打碎成微小段，逐点做逆解后入队，消除刀尖弧形挖坑效应。
// 无旋转变化时退化为单次逆解入队。
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

    // 无旋转变化：单次逆解入队
    if(!has_rotation){
        double phys_end[AXIS_NUM];
        memcpy(phys_end, end_pos, sizeof(double) * AXIS_NUM);
        apply_rtcp_to_pos(phys_end);
        double speed_mm_sec = run_speed_mm / 60.0;
        return api_push_trajectory(phys_end, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC);
    }

    // 有旋转变化：计算等效空间距离（含旋转轴弧长折算）
    double dist_eq = 0.0;
    for(int i = 0; i < AXIS_NUM; i++){
        double delta = end_pos[i] - start_pos[i];
        if(g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0){
            delta = delta * (PI / 180.0) * g_axis[i].equivalent_radius;
        }
        dist_eq += delta * delta;
    }
    dist_eq = sqrt(dist_eq);

    int num_segments = (int)ceil(dist_eq / RTCP_LINEAR_SEGMENT_MM);
    if(num_segments < 1) num_segments = 1;

    // 速度计算：G93 按等效距离反推，G94 直取
    double speed_mm_sec;
    if(g93_T_sec > 1e-9){
        speed_mm_sec = dist_eq / g93_T_sec;
        if(speed_mm_sec < 1e-6) speed_mm_sec = 1e-6;
    } else {
        speed_mm_sec = run_speed_mm / 60.0;
    }

    // 逐点插值 + 逆解 + 入队（跳过起点 i=0，从 i=1 开始）
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

        if(api_push_trajectory(interp_pos, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC) < 0){
            return -1;
        }
    }

    printf("[Parser] RTCP直线打碎: %d 段 (等效距离 %.2f mm, 速度 %.2f mm/s)\n",
           num_segments, dist_eq, speed_mm_sec);
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
                else if(value==90.0) g_state.is_absolute=1;
                else if(value==91.0) g_state.is_absolute=0;
                else if(value==92.0) is_non_motion_g=1;    // G92 坐标偏移
                else if(value==53.0) is_G53_this_block=1;  // G53 非模态机械坐标
                else if(value==93.0) g_state.feed_mode=FEED_MODE_G93; // G93 倒数时间
                else if(value==94.0) g_state.feed_mode=FEED_MODE_G94; // G94 每分钟
                else if(fabs(value - 43.4) < 0.05) g_state.rtcp_enabled = 1; // G43.4 开启RTCP
                else if(value >= 49.0 && value < 50.0) g_state.rtcp_enabled = 0; // G49 关闭RTCP
                break;
            case 'F':g_state.feedrate_mm_min=value;has_f=1;break;
            case 'I':offset_i=value;has_move=1;break;
            case 'J':offset_j=value;has_move=1;break;
            case 'K':offset_k=value;has_move=1;break;
            case 'M':m_code=(int)value;break;
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
        if(api_push_mcode(m_code, s_value, p_value, q_value, r_value) < 0){
            printf("[Parser] M代码入队失败(报警)，中止当前文件！\n");
            return -1;
        }
        printf("[Parser] 解析M代码: M%02d S%.1f P%.1f Q%.1f R%.1f\n", m_code, s_value, p_value, q_value, r_value);
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
            if(generate_linear_rtcp_trajectory(machine_start_pos,
                                               machine_target_pos,
                                               run_speed_mm, g93_T_sec) < 0){
                printf("[Parser] RTCP直线入队失败(报警)，中止当前文件！\n");
                return -1;
            }
        } else {
            double speed_mm_sec=run_speed_mm/60.0;
            if(api_push_trajectory(machine_target_pos,speed_mm_sec,DEFAULT_ACC,DEFAULT_DEC) < 0){
                printf("[Parser] 运动指令入队失败(报警)，中止当前文件！\n");
                return -1;
            }
        }

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
        double speed_mm_sec;
        if(g93_T_sec > 1e-9){
            speed_mm_sec = sqrt(dist_sq) / g93_T_sec;
            if(speed_mm_sec < 1e-6) speed_mm_sec = 1e-6;
        }else{
            speed_mm_sec = feedrate_mm_min / 60.0;
        }
        if(g_state.rtcp_enabled){
            apply_rtcp_to_pos(end_pos);
        }
        return api_push_trajectory(end_pos, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC);
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

    // ---- 5. 分段数与切向速度 ----
    double arc_length = fabs(total_angle) * radius;
    int num_segments = (int)ceil(arc_length / ARC_SEGMENT_LENGTH_MM);
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

        // RTCP 逐点逆解：每个插补点用自身的旋转轴角度补偿 XYZ
        if(g_state.rtcp_enabled){
            apply_rtcp_to_pos(next_pos);
        }

        if(api_push_trajectory(next_pos, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC) < 0){
            return -1;
        }
    }

    printf("[Parser] 生成了 %d 个圆弧插补点 (平面 G%d, 速度 %.2f mm/s)\n",
           num_segments, g_state.active_plane, speed_mm_sec);
    return 0;
}