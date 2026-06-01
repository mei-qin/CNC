#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include <math.h>
#define PI 3.14159265358979323846
#define ARC_SEGMENT_LENGTH_MM 0.5 // 圆弧插补时的分段长度，单位mm

GCodeState_t g_state = {{0}, 1000.0, 1, 17}; // 全局G-code状态变量，默认XY平面
ParserControl_t g_parser_ctrl = {"", 0, 0, 0}; // 全局G-code解析控制变量，初始值为未运行、未暂停、未请求中止
extern int api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern int api_push_mcode(int m_code, double s_value);

const char* skip_spaces(const char* str)
{
    while(*str==' '||*str=='\t') str++;
    return str;
}

int parse_gcode_line(const char *gcode_line)
{
    char buffer[128];
    strncpy(buffer, gcode_line, sizeof(buffer));
    
    int is_G00=0,is_G01=0,is_G02=0,is_G03=0;
    int has_move=0;
    //int has_x=0,has_y=0,has_z=0;
    int has_axis[AXIS_NUM]={0};

    double val_axis[AXIS_NUM]={0};
    //double val_x=0,val_y=0,val_z=0;
    double offset_i=0.0,offset_j=0.0,offset_k=0.0;// 圆弧中心相对起点的偏移量，G02/G03专用
    int m_code=-1;        // M代码编号（-1表示无M代码）
    double s_value=0.0;   // S值（主轴转速等）


    char *p=buffer;
    while(*p!='\0'){
        p=(char*)skip_spaces(p);
        if(*p=='\0') break;

        char letter=toupper(*p);
        p++;

        double value=strtod(p, &p); // 解析数字，p会更新到数字后面的位置

        switch(letter){
            case 'G':
                if(value==0.0) is_G00=1;
                else if(value==1.0) is_G01=1;
                else if(value==90.0) g_state.is_absolute=1;
                else if(value==91.0) g_state.is_absolute=0;
                else if(value==2.0) is_G02=1;
                else if(value==3.0) is_G03=1;
                else if(value==17.0) g_state.active_plane=17;
                else if(value==18.0) g_state.active_plane=18;
                else if(value==19.0) g_state.active_plane=19;
                  
                break;
            case 'X':val_axis[0]=value;has_move=1;has_axis[0]=1; break;
            case 'Y':val_axis[1]=value;has_move=1;has_axis[1]=1; break;
            case 'Z':val_axis[2]=value;has_move=1;has_axis[2]=1; break;
            case 'A':if(AXIS_NUM>3){val_axis[3]=value;has_move=1;has_axis[3]=1;} break;
            case 'B':if(AXIS_NUM>4){val_axis[4]=value;has_move=1;has_axis[4]=1;} break;
            case 'F':g_state.feedrate_mm_min=value;break;
            case 'I':offset_i=value;break;
            case 'J':offset_j=value;break;
            case 'K':offset_k=value;break;
            case 'M':m_code=(int)value;break;
            case 'S':s_value=value;break;
            default:
                // 其他命令暂不处理
                break;
        }

    }

    // 处理M代码：压入队列作为同步屏障
    if (m_code >= 0) {
        if(api_push_mcode(m_code, s_value) < 0){
            printf("[Parser] M代码入队失败(报警)，中止当前文件！\n");
            return -1;
        }
        printf("[Parser] 解析M代码: M%02d S%.1f\n", m_code, s_value);
    }

    if(is_G00||is_G01||is_G02||is_G03||has_move){

        double target_pos[AXIS_NUM];
        double start_pos[AXIS_NUM];

        for(int i=0;i<AXIS_NUM;i++){
            start_pos[i]=g_state.current_pos[i];
            if(g_state.is_absolute){
                target_pos[i]=has_axis[i]?val_axis[i]:g_state.current_pos[i];
            }else{
                target_pos[i]=g_state.current_pos[i]+(has_axis[i]?val_axis[i]:0);
            }

        }

        double run_speed_mm=is_G00?RAPID_SPEED_MM_MIN:g_state.feedrate_mm_min;

        double machine_target_pos[AXIS_NUM];
        double machine_start_pos[AXIS_NUM];

        for(int i=0;i<AXIS_NUM;i++){
            if(g_coord_mgr.current_coord==COORD_G53){
                machine_target_pos[i]=target_pos[i];
                machine_start_pos[i]=start_pos[i];
            }else{
                int idx=g_coord_mgr.current_coord-1;
                machine_target_pos[i]=target_pos[i]+g_coord_mgr.work_offsets[idx][i];
                machine_start_pos[i]=start_pos[i]+g_coord_mgr.work_offsets[idx][i];
            }
        }


        if(is_G02||is_G03){
            double off_1st, off_2nd;
            switch(g_state.active_plane){
                case 18: off_1st=offset_k; off_2nd=offset_i; break; // ZX: K→Z, I→X
                case 19: off_1st=offset_j; off_2nd=offset_k; break; // YZ: J→Y, K→Z
                default: off_1st=offset_i; off_2nd=offset_j; break; // XY: I→X, J→Y
            }
            if(generate_arc_trajectory(machine_start_pos,
                                    machine_target_pos,
                                    off_1st, off_2nd,
                                    is_G02, run_speed_mm) < 0){
                printf("[Parser] 圆弧入队失败(报警)，中止当前文件！\n");
                return -1;
            }

        } else{

            double speed_mm_sec=run_speed_mm/60.0;

            if(api_push_trajectory(machine_target_pos,speed_mm_sec,DEFAULT_ACC,DEFAULT_DEC) < 0){
                printf("[Parser] 运动指令入队失败(报警)，中止当前文件！\n");
                return -1;
            }

        }

        for(int i=0;i<AXIS_NUM;i++){
            g_state.current_pos[i]=target_pos[i];
        }

        printf("[Parser] 解析命令: %s -> 目标 (%.3f, %.3f, %.3f) mm, 速度 %.1f mm/min\n",
                buffer, target_pos[0], target_pos[1], target_pos[2], run_speed_mm);



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
// 返回值: 0=成功, -1=入队被拒(报警)
int generate_arc_trajectory(double start_pos[AXIS_NUM],double end_pos[AXIS_NUM],
                             double offset_1st, double offset_2nd,
                             int is_CW,double feedrate_mm_min)
{
    // ---- 动态轴映射 ----
    int ax1, ax2;
    if((g_state.active_plane == 18 || g_state.active_plane == 19) && AXIS_NUM < 3){
        printf("[Parser] G%d 需要3轴以上，当前 AXIS_NUM=%d，拒绝执行！\n",
               g_state.active_plane, AXIS_NUM);
        return -1;
    }
    switch(g_state.active_plane){
        case 18: ax1=2; ax2=0; break; // ZX: 第一轴Z, 第二轴X
        case 19: ax1=1; ax2=2; break; // YZ: 第一轴Y, 第二轴Z
        default: ax1=0; ax2=1; break; // XY: 第一轴X, 第二轴Y
    }

    // ---- 1. 圆心坐标 ----
    double cx = start_pos[ax1] + offset_1st;
    double cy = start_pos[ax2] + offset_2nd;

    // ---- 2. 半径 ----
    double radius = hypot(start_pos[ax1] - cx, start_pos[ax2] - cy);
    if(radius < 0.001) return 0;

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

    // ---- 5. 分段数 ----
    double arc_length = fabs(total_angle) * radius;
    int num_segments = (int)ceil(arc_length / ARC_SEGMENT_LENGTH_MM);
    if(num_segments < 1) num_segments = 1;

    double angle_step = total_angle / num_segments;
    double speed_mm_sec = feedrate_mm_min / 60.0;

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

        if(api_push_trajectory(next_pos, speed_mm_sec, DEFAULT_ACC, DEFAULT_DEC) < 0){
            return -1;
        }
    }

    printf("[Parser] 生成了 %d 个圆弧插补点 (平面 G%d)\n", num_segments, g_state.active_plane);
    return 0;
}