#include "axis_ctrl.h"
#include "global_def.h"
#include "soem/soem.h"
#include "sim_drive.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "planner.h"
#include "gcode_parser.h"
#include "preview_streamer.h"   /* P0-b v1: PreviewStreamer_Push */
/************************ 全局变量定义（仅轴相关，其余在ecat_core.c） ************************/
AxisCtrl_t g_axis[AXIS_NUM];            // 五轴核心数组，全局唯一定义
int g_axis_map[26];                     // 动态轴映射表：'A'-'Z' → 轴索引，-1=未映射
int g_all_axis_op_ready = 0;            // 五轴均使能就绪标志
int g_all_axis_reach = 0;               // 五轴均目标到达标志
Interpolator_t g_interpolator={0};
CommandQueue_t g_cmd_queue={0};
static double plan_cursor[AXIS_NUM]={0};
CoordManager_t g_coord_mgr={COORD_G54,{0},{0},{0}};
PlannerConfig_t g_planner_config={0.05, 500.0};

/* ---- P0-b v1: parser → axis_ctrl 段元数据传递 (定义在此, extern 在 global_def.h) ----
 * g_seg_id_counter: 段 ID 单调递增, atomic_fetch_add 在 api_push_trajectory_impl
 *                  内调用 (跨 parser/bspline 多线程入队, 必须 atomic)
 * g_current_line_no: parser 入口设 (= g_current_program->lines[g_pc].line_no)
 * g_current_motion_type: parser case 'G' 设 (G00/G01/G02/G03)
 *                       默认 MOTION_TYPE_OTHER (0xFF), parser 线程单写者
 */
_Atomic uint64_t g_seg_id_counter = 0;
int32_t          g_current_line_no = 0;
uint8_t          g_current_motion_type = MOTION_TYPE_OTHER;

/* ---- P0-b v2: 程序级统计 (定义在此, extern 在 global_def.h) ----
 * parser_thread_func 启动时清零 (filepath 设置后), 入队时累加/比较,
 * SMC_GetProgramStructure 读出。
 * g_program_bbox 初值用 ±1e18 哨兵, 首段必然更新; 程序无运动段时哨兵保留 (UI 自查)。
 */
double           g_program_total_time_ms = 0.0;
double           g_program_bbox_min[AXIS_NUM] = {0};
double           g_program_bbox_max[AXIS_NUM] = {0};
_Atomic int      g_program_load_done = 0;
uint64_t         g_program_first_seg_id = 0;
uint64_t         g_program_last_seg_id = 0;
int32_t          g_program_total_lines = 0;
int32_t          g_program_num_o_labels = 0;
int32_t          g_program_num_n_labels = 0;
/************************ 五轴系统初始化（核心配置，修改此函数即可调整轴参数） ************************/
static int check_soft_limits(double target_pos[AXIS_NUM]){
    for(int i=0;i<AXIS_NUM;i++){
        if(g_axis[i].enable_soft_limit){
            if(target_pos[i]>g_axis[i].soft_limit_pos||target_pos[i]<g_axis[i].soft_limit_neg){
                printf("[软限位] %s 目标位置%.2f 超出范围 [%.2f, %.2f]\n",
                       g_axis[i].axis_name, target_pos[i],
                       g_axis[i].soft_limit_neg, g_axis[i].soft_limit_pos);
                return 0;
            }
        }
    }
    return 1;
}

void api_sync_planner_cursor(){
    for(int i=0;i<AXIS_NUM;i++){
        plan_cursor[i]=g_axis[i].current_cmd_pos;
    }
}

double api_get_cursor(int axis_idx){
    if(axis_idx < 0 || axis_idx >= AXIS_NUM) return 0.0;
    return plan_cursor[axis_idx];
}


void wait_motion_done(){
    while(g_interpolator.is_moving) osal_usleep(10000);
}

void api_set_zero(int axis_idx){

    if(g_coord_mgr.current_coord==COORD_G53){
        printf("[API] G53坐标系不允许设置零点！\n");
        return;
    }

    int coord_idx=g_coord_mgr.current_coord-1; // G54对应0，G55对应1，以此类推
    for(int i=0;i<AXIS_NUM;i++){
        if(axis_idx==AXIS_ALL||axis_idx==i){
            g_coord_mgr.work_offsets[coord_idx][i]=g_coord_mgr.current_g53_pos[i];

            g_coord_mgr.current_logical_pos[i]=0.0; // 设置当前逻辑坐标为0
            plan_cursor[i]=0.0; // 同步规划器光标
        }
    }
    printf("[API] 已设置 %s 坐标系零点，当前G53位置 (%.3f, %.3f, %.3f)\n", 
            (g_coord_mgr.current_coord==COORD_G54)?"G54":"G55",
            g_coord_mgr.current_g53_pos[0], g_coord_mgr.current_g53_pos[1], g_coord_mgr.current_g53_pos[2]);
}


void api_go_zero(int axis_idx,double speed){
    double t_pos[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        t_pos[i]=plan_cursor[i];
    }
    //double tx=plan_cursor_x;
    //double ty=plan_cursor_y;
    //double tz=plan_cursor_z;

    if(axis_idx==AXIS_ALL){
        for(int i=0;i<AXIS_NUM;i++){
            t_pos[i]=0.0;
        }
    }else if(axis_idx>=0.0&&axis_idx<AXIS_NUM){
        t_pos[axis_idx]=0.0;
    }

    api_push_trajectory(t_pos,speed,DEFAULT_ACC,DEFAULT_DEC);
}



void api_move_relative(int axis_idx,double distance,double speed){
    double t_pos[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        t_pos[i]=plan_cursor[i];
    }

    if(axis_idx==AXIS_ALL){
        for(int i=0;i<AXIS_NUM;i++){
            t_pos[i]+=distance;
        }
    }else if(axis_idx>=0&&axis_idx<AXIS_NUM){
        t_pos[axis_idx]+=distance;
    }

    api_push_trajectory(t_pos,speed,DEFAULT_ACC,DEFAULT_DEC);
}


// 前向声明: 静态实现函数,所有公开包装器均调用此函数。
static int api_push_trajectory_impl(double target_pos[AXIS_NUM],
                                     double speed_sec_mm, double acc_sec_mm,
                                     double dec_sec_mm, int is_g93_strict,
                                     double g93_dt_sec, int is_fillet,
                                     int is_rtcp_active, CoordSystem_t wcs,
                                     const double wcs_offset_snap[AXIS_NUM]);

// @Context: Non-RealTime Background Thread (parser / bspline 入队前调用)
// @Thread-Safety: 读 g_coord_mgr.work_offsets + g_state.local_offset — 仅在 parser 线程或 setup API 上下文调用。
// 把当前模态 WCS 对应的偏置向量从 work_offsets 拷出。G53 (idx<0) 输出全零。
// 设计目的: 让段在入队瞬间"冻结"此刻的偏置, RT 后续用此快照推导 UI 逻辑坐标,
// 不再受 parser 中途 `#5221=..` / G10 L2 改 work_offsets 的污染 (H-1)。
// P2': G54-G59 路径上叠加 g_state.local_offset (Fanuc G52 局部坐标系)。
//      G53 路径不叠加 (机械坐标天然 bypass WCS+G52)。
static inline void snapshot_wcs_offset(CoordSystem_t wcs, double out[AXIS_NUM])
{
    int idx = (wcs >= COORD_G54 && wcs <= COORD_G59) ? (wcs - 1) : -1;
    if(idx >= 0){
        // P5': G54.1 Pn 扩展 WCS 优先 (modal_ext_wcs_p 1-48)
        // 激活时覆盖 modal_wcs 路径, 否则走原 G54-G59 表
        if(g_state.modal_ext_wcs_p >= 1 && g_state.modal_ext_wcs_p <= 48){
            int ext_idx = g_state.modal_ext_wcs_p - 1;
            for(int i = 0; i < AXIS_NUM; i++)
                out[i] = g_coord_mgr.work_offsets_ext[ext_idx][i];
        } else {
            for(int i = 0; i < AXIS_NUM; i++) out[i] = g_coord_mgr.work_offsets[idx][i];
        }
        // P2': G52 局部坐标系叠加 (仅 G54-G59 路径)
        if(g_state.local_offset_active){
            for(int i = 0; i < AXIS_NUM; i++) out[i] += g_state.local_offset[i];
        }
    }else{
        // G53 或未定义 WCS: 零基准, 不叠加 G52
        for(int i = 0; i < AXIS_NUM; i++) out[i] = 0.0;
    }
}

// @Context: Non-RealTime Background Thread (parser / 上层管理线程)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// 常规入队包装: 走完整的短板效应限幅路径。WCS 盖章取 parser 模态 g_state.modal_wcs,
// 偏置快照在入队瞬间从 work_offsets 冻结。
int api_push_trajectory(double target_pos[AXIS_NUM],
                         double speed_sec_mm, double acc_sec_mm, double dec_sec_mm)
{
    double snap[AXIS_NUM];
    snapshot_wcs_offset(g_state.modal_wcs, snap);
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 0, 0.0, 0, 0,
                                     g_state.modal_wcs, snap);
}

// @Context: Non-RealTime Background Thread (parser / bspline)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// G93 强一致性入队包装: 豁免短板限幅,预计算纯匀速,刚性守恒时间。
// 同时锁死几何拓扑 (is_fillet=1),禁止 Planner 对 G93 微段做 G64 拐角抹圆篡改。
int api_push_trajectory_g93(double target_pos[AXIS_NUM],
                             double speed_sec_mm, double acc_sec_mm,
                             double dec_sec_mm, double g93_dt_sec)
{
    double snap[AXIS_NUM];
    snapshot_wcs_offset(g_state.modal_wcs, snap);
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 1, g93_dt_sec, 1, 0,
                                     g_state.modal_wcs, snap);
}

// @Context: Non-RealTime Background Thread (bspline)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// 免抹圆透传包装: is_fillet=1,planner_fillet_preprocess 跳过本段。
// bspline 线程须改用 api_push_trajectory_passthrough_wcs 显式传 WCS+快照,避免读 g_state。
int api_push_trajectory_passthrough(double target_pos[AXIS_NUM],
                                     double speed_sec_mm, double acc_sec_mm,
                                     double dec_sec_mm)
{
    double snap[AXIS_NUM];
    snapshot_wcs_offset(g_state.modal_wcs, snap);
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 0, 0.0, 1, 0,
                                     g_state.modal_wcs, snap);
}

// @Context: Non-RealTime Background Thread (bspline 专用)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// 显式 WCS+偏置快照透传: wcs/snap 均来自 DirtyPoint_t (脏点捕获时已冻结),
// 避免 bspline 线程跨线程读 parser 的 g_state.modal_wcs 与 g_coord_mgr.work_offsets。
int api_push_trajectory_passthrough_wcs(double target_pos[AXIS_NUM],
                                         double speed_sec_mm, double acc_sec_mm,
                                         double dec_sec_mm, CoordSystem_t wcs,
                                         const double wcs_offset_snap[AXIS_NUM])
{
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 0, 0.0, 1, 0,
                                     wcs, wcs_offset_snap);
}

// @Context: Non-RealTime Background Thread (bspline 专用)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// 显式 WCS+偏置快照的 G93 强一致性入队 (G93 + BSpline 复合路径)。
int api_push_trajectory_g93_wcs(double target_pos[AXIS_NUM],
                                   double speed_sec_mm, double acc_sec_mm,
                                   double dec_sec_mm,
                                   double g93_dt_sec, CoordSystem_t wcs,
                                   const double wcs_offset_snap[AXIS_NUM])
{
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 1, g93_dt_sec, 1, 0,
                                     wcs, wcs_offset_snap);
}

// @Context: Non-RealTime Background Thread (RTCP 路径专用)
// @Thread-Safety: queue_spinlock 互斥,与其他 push 函数共享。
// RTCP 包装器: 入队时设置 is_rtcp_active=1 元数据。
// 调用契约: target_pos 必须已是物理关节坐标 (apply_rtcp_to_pos 处理过)。
//   RT 线程消费时不做逆解,继续物理关节空间 S 曲线插补,保持 1ms 硬实时纯粹性。
int api_push_trajectory_rtcp(double target_pos[AXIS_NUM],
                              double speed_sec_mm, double acc_sec_mm,
                              double dec_sec_mm)
{
    double snap[AXIS_NUM];
    snapshot_wcs_offset(g_state.modal_wcs, snap);
    return api_push_trajectory_impl(target_pos, speed_sec_mm,
                                     acc_sec_mm, dec_sec_mm, 0, 0.0, 0, 1,
                                     g_state.modal_wcs, snap);
}

// @Context: Non-RealTime Background Thread (parser / 上层管理线程)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
// is_g93_strict=0: 常规路径,执行完整的短板效应限幅。
// is_g93_strict=1: G93 强一致性路径,豁免 max_speed/max_acc/max_dec/max_jerk
//                  短板限幅,并在 mutex 内预计算纯匀速 (T4=T_total),
//                  保证 1ms 线程解析时绝对遵守 g93_dt_sec 时间预算。
//                  调用者必须已按 phys_dist / g93_dt_sec 精确推算 speed_sec_mm。
// g93_dt_sec: G93 微段时间预算(秒),仅 is_g93_strict=1 时生效,否则忽略。
// is_fillet:    1=标记为免抹圆段,planner_fillet_preprocess 跳过本段。
// is_rtcp_active: 1=RTCP 路径产生的段 (target_pos 已是物理关节坐标),
//                 仅元数据,不参与插补决策。
// wcs: 本段所属工件坐标系索引 (G53..G59)。RT 线程消费段时据此更新
//      g_coord_mgr.current_coord,使 UI 显示/宏系统变量与物理运动严格同步。
//      parser 调用方传 g_state.modal_wcs; bspline 调用方传 DirtyPoint_t.wcs。
// wcs_offset_snap: 段入队瞬间 work_offsets[wcs-1] 的快照。RT 拷到 active_offset,
//                  用于 current_logical_pos 推导,隔离 parser 中途改 work_offsets 的污染。
static int api_push_trajectory_impl(double target_pos[AXIS_NUM],
                                     double speed_sec_mm, double acc_sec_mm,
                                     double dec_sec_mm,
                                     int is_g93_strict, double g93_dt_sec,
                                     int is_fillet, int is_rtcp_active,
                                     CoordSystem_t wcs,
                                     const double wcs_offset_snap[AXIS_NUM])
{
    if(atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==1) {
        printf("[SAFETY] 系统报警中，拒绝入队运动指令！\n");
        return -1;
    }
    if(!g_all_axis_op_ready) return -1;
    if(!check_soft_limits(target_pos)) return -1;

    if(speed_sec_mm <= 0.0) {
        printf("[SAFETY] 进给速度 %.3f mm/s 非正数，拒绝执行！\n", speed_sec_mm);
        return -1;
    }

    // ---- P0-b v2: LoadProgram preview 模式 ----
    // 不入 motion queue (RT 不消费, current_seg_id 保持), 仅推 PreviewStreamer + 更新统计。
    // 安全检查 (alarm/soft_limit/speed) 仍走完, 防止 preview 时把错误数据塞给 UI。
    // 跨线程读 program_mode: x86 int 读原子, 此处 plain read 与 is_running/is_paused 同模式。
    if (g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW) {
        // 1. 计算粗略总距离 (UI 显示用, 不做 S 曲线完整规划)
        double pv_dist = 0.0;
        for (int i = 0; i < AXIS_NUM; i++) {
            double d = target_pos[i] - plan_cursor[i];
            pv_dist += d * d;
        }
        pv_dist = sqrt(pv_dist);   // parser 线程非 RT, sqrt OK

        // 2. 构造最小 preview_seg (仅 UI 可见字段, S 曲线参数全 0)
        TrajectorySegment_t pv_seg;
        memset(&pv_seg, 0, sizeof(pv_seg));
        pv_seg.cmd_type = CMD_TYPE_MOTION;
        for (int i = 0; i < AXIS_NUM; i++) {
            pv_seg.target_pos[i] = target_pos[i];
            pv_seg.dir_vec[i] = (pv_dist > 1e-9) ?
                                (target_pos[i] - plan_cursor[i]) / pv_dist : 0.0;
        }
        pv_seg.total_distance = pv_dist;
        pv_seg.v_max = speed_sec_mm;
        pv_seg.v_target = speed_sec_mm / 1000.0;   // mm/s → mm/ms (与正常路径一致)
        pv_seg.is_rtcp_active = is_rtcp_active ? 1 : 0;
        pv_seg.is_fillet = is_fillet ? 1 : 0;
        pv_seg.active_wcs = wcs;
        for (int i = 0; i < AXIS_NUM; i++) {
            pv_seg.wcs_offset_snap[i] = wcs_offset_snap[i];
        }
        pv_seg.seg_id = atomic_fetch_add_explicit(&g_seg_id_counter, 1,
                                                   memory_order_relaxed);
        pv_seg.line_no = g_current_line_no;
        pv_seg.motion_type = g_current_motion_type;

        // 3. 推 PreviewStreamer (9529 client 收)
        PreviewStreamer_Push(&pv_seg);

        // 4. 更新 bbox (parser_thread_func 启动时 init ±1e18 哨兵, 首段必更新)
        for (int i = 0; i < AXIS_NUM; i++) {
            if (target_pos[i] < g_program_bbox_min[i]) g_program_bbox_min[i] = target_pos[i];
            if (target_pos[i] > g_program_bbox_max[i]) g_program_bbox_max[i] = target_pos[i];
        }

        // 5. 记录首/末段 seg_id (parser_thread_func 启动时 init UINT64_MAX 哨兵)
        if (g_program_first_seg_id == (uint64_t)-1) {
            g_program_first_seg_id = pv_seg.seg_id;
        }
        g_program_last_seg_id = pv_seg.seg_id;

        // 6. 更新 plan_cursor (parser 下一段 dist 计算依赖, 必须更新)
        for (int i = 0; i < AXIS_NUM; i++) {
            plan_cursor[i] = target_pos[i];
        }

        return 0;   // 不入 motion queue
    }

    // ---- 第一步：计算原始偏差（mm 或 deg）----
    double delta_raw[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        delta_raw[i]=target_pos[i]-plan_cursor[i];
    }

    for(int i=0;i<AXIS_NUM;i++){
        if(fabs(delta_raw[i]) > 0.0001 && g_axis[i].max_speed <= 0.0) {
            printf("[SAFETY] %s 轴动力学未配置(max_speed=0)，拒绝执行运动指令！\n",
                   g_axis[i].axis_name);
            return -1;
        }
    }

    // ---- 第二步：统一量纲 → 等效毫米位移 ----
    double delta_mm[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        if(g_axis[i].axis_type == 1){
            if(g_axis[i].equivalent_radius <= 0.0) {
                printf("[SAFETY] %s 旋转轴等效半径未配置(%.2f)，拒绝执行！\n",
                       g_axis[i].axis_name, g_axis[i].equivalent_radius);
                return -1;
            }
            delta_mm[i] = delta_raw[i] * DEG_TO_RAD * g_axis[i].equivalent_radius;
        } else {
            delta_mm[i] = delta_raw[i];
        }
    }

    // ---- 第三步：空间合成距离（纯 mm）----
    double dist_sq = 0.0;
    for(int i=0;i<AXIS_NUM;i++){
        dist_sq += delta_mm[i] * delta_mm[i];
    }
    double dist = sqrt(dist_sq);
    if(dist < 1e-6) dist = 0.0;

    // 方向向量（预计算，mutex 内直接拷贝）
    double dir_vec[AXIS_NUM];
    for(int i=0;i<AXIS_NUM;i++){
        dir_vec[i] = (dist > 1e-6) ? delta_mm[i] / dist : 0.0;
    }

    // ---- 第四步：短板效应限幅（纯配置量运算，无需 mutex）----
    // G93 强一致性模式: 完全豁免,保证刚性时间。
    // 常规模式: 按各轴 max_* 配置等比例压低,避免单轴超载。
    double final_speed_ratio = 1.0;
    double final_acc_ratio   = 1.0;
    double final_dec_ratio   = 1.0;
    double final_jerk_ratio  = 1.0;

    if(!is_g93_strict){
        for(int i = 0; i < AXIS_NUM; i++){
            if(fabs(delta_mm[i]) < 0.0001 || dist < 1e-6) continue;

            double axis_ratio_mm = fabs(delta_mm[i]) / dist;

            double req_v_mm = speed_sec_mm * axis_ratio_mm;
            double req_a_mm = acc_sec_mm   * axis_ratio_mm;
            double req_d_mm = dec_sec_mm   * axis_ratio_mm;
            double req_j_mm = DEFAULT_JERK * axis_ratio_mm;

            double req_v, req_a, req_d, req_j;
            if(g_axis[i].axis_type == 1 && g_axis[i].equivalent_radius > 0.0){
                double safe_r = fmax(g_axis[i].equivalent_radius, 1e-4);
                double conv = DEG_TO_RAD * safe_r;
                req_v = req_v_mm / conv;
                req_a = req_a_mm / conv;
                req_d = req_d_mm / conv;
                req_j = req_j_mm / conv;
            } else {
                req_v = req_v_mm;
                req_a = req_a_mm;
                req_d = req_d_mm;
                req_j = req_j_mm;
            }

            if(g_axis[i].max_speed > 0.0 && req_v > g_axis[i].max_speed){
                double r = g_axis[i].max_speed / req_v;
                if(r < final_speed_ratio) final_speed_ratio = r;
            }
            if(g_axis[i].max_acc > 0.0 && req_a > g_axis[i].max_acc){
                double r = g_axis[i].max_acc / req_a;
                if(r < final_acc_ratio) final_acc_ratio = r;
            }
            if(g_axis[i].max_dec > 0.0 && req_d > g_axis[i].max_dec){
                double r = g_axis[i].max_dec / req_d;
                if(r < final_dec_ratio) final_dec_ratio = r;
            }
            if(g_axis[i].max_jerk > 0.0 && req_j > g_axis[i].max_jerk){
                double r = g_axis[i].max_jerk / req_j;
                if(r < final_jerk_ratio) final_jerk_ratio = r;
            }
        }
    }

    speed_sec_mm *= final_speed_ratio;
    acc_sec_mm   *= final_acc_ratio;
    dec_sec_mm   *= final_dec_ratio;
    double jerk_sec_mm = DEFAULT_JERK * final_jerk_ratio;

    if(speed_sec_mm < 1e-6) speed_sec_mm = 1e-6;
    if(acc_sec_mm < 1e-6)   acc_sec_mm   = 1e-6;
    if(dec_sec_mm < 1e-6)   dec_sec_mm   = 1e-6;
    if(jerk_sec_mm < 1e-6)  jerk_sec_mm  = 1e-6;

    // === queue_spinlock 保护的入队临界区 ===
    // 后台线程互斥: parser/bspline/planner/watchdog 通过 queue_spinlock 串行化。
    // RT 线程【不取此锁】,仍保持 lock-free 消费 (优先级反转免疫)。
    //
    // 持锁/释放节奏:
    //   - 队列有空槽: 一次持锁完成 buffer 写入 + write_head 推进 (µs 级)
    //   - 队列满: 必须释放锁后再 sleep,否则会阻塞其他后台线程 (含 watchdog 兜底)
    int head, next_head;
    while (1) {
        // 自旋获取 spinlock (acquire): 与上一任持锁者 release 配对,看到所有 buffer 修改
        while (atomic_flag_test_and_set_explicit(&g_cmd_queue.queue_spinlock,
                                                 memory_order_acquire)) {
#if defined(__x86_64__) || defined(__i386__)
            __asm__ volatile("pause" ::: "memory");
#elif defined(__aarch64__)
            __asm__ volatile("yield" ::: "memory");
#endif
        }
        // 持锁后: read_tail 可见性由 spinlock acquire 保证,用 relaxed 读即可
        int tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_relaxed);
        head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_relaxed);
        next_head = (head + 1) % QUEUE_SIZE;
        if (next_head != tail) break;  // 有空槽,持锁继续

        // 队列满: 释放锁,sleep 1ms 让 RT 消费推进 read_tail
        atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);
        if (!dorun || atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==1) return -1;
        osal_usleep(1000);
    }

    // 写入 buffer[head]: relaxed 即可,可见性由 spinlock release + is_ready release 双重保证。
    TrajectorySegment_t *seg = &g_cmd_queue.buffer[head];
    atomic_store_explicit(&seg->is_ready, 0, memory_order_relaxed);
    seg->cmd_type = CMD_TYPE_MOTION;
    seg->is_fillet = is_fillet ? 1 : 0;
    seg->is_g93_strict = (is_g93_strict && g93_dt_sec > 1e-9) ? 1 : 0;
    seg->is_rtcp_active = is_rtcp_active ? 1 : 0;
    seg->active_wcs = wcs;   // 段内带内 WCS, RT 消费时同步 g_coord_mgr.current_coord
    for(int i = 0; i < AXIS_NUM; i++){
        seg->wcs_offset_snap[i] = wcs_offset_snap[i];  // H-1: 冻结入队瞬间的偏置向量
    }
    seg->m_code = 0;
    seg->s_value = 0.0;
    seg->total_distance = dist;

    // Phase B1: 段级耦合配置快照 (运动段也需快照, RT sync_config 每段都调)
    // 注: 运动段的 aux_laser_enable/shutter/power_w/freq/gas 不填 (Phase A 设计, apply_aux
    //     只在 M 段调, 保持 g_laser_rt 模态). 但 coupling_mode 是配置同步, 必须填.
    seg->aux_laser_coupling_mode = atomic_load_explicit(&g_laser_cfg.coupling_mode,
                                                        memory_order_acquire);
    seg->aux_laser_v_thresh      = atomic_load_explicit(&g_laser_cfg.v_thresh_mm_s,
                                                        memory_order_acquire);

    for(int i=0;i<AXIS_NUM;i++){
        seg->target_pos[i]=target_pos[i];
        seg->dir_vec[i] = dir_vec[i];
    }

    seg->v_target = speed_sec_mm / 1000.0;  // mm/s → mm/ms
    seg->acc = acc_sec_mm / 1000000.0;       // mm/s^2 → mm/ms^2
    seg->dec = dec_sec_mm / 1000000.0;
    seg->jerk = jerk_sec_mm / 1.0e9;          // mm/s^3 → mm/ms^3
    seg->v_start = 0.0;
    seg->v_end = 0.0;
    seg->v_max = seg->v_target;

    // 7段式 S 曲线绝对解析参数初始化（由 planner_recalculate 填充）
    seg->T1=0; seg->T2=0; seg->T3=0; seg->T4=0;
    seg->T5=0; seg->T6=0; seg->T7=0; seg->T_total=0;
    seg->v0=0; seg->v1=0; seg->v2=0; seg->v3=0;
    seg->v4=0; seg->v5=0; seg->v6=0;
    seg->s0=0; seg->s1=0; seg->s2=0; seg->s3=0;
    seg->s4=0; seg->s5=0; seg->s6=0;
    seg->j1=0; seg->a2=0; seg->j3=0;

    // ---- G93 强一致性预计算: 纯匀速 (T4=T_total, 加减速阶段全部为 0) ----
    // 让 1ms 线程的绝对解析方程直接以恒定 v_target 走完整个 T_total,
    // 不再受 planner 的 S 曲线限幅影响,绝对遵守 g93_dt_sec 时间预算。
    // recompute_scurve_profile 检测到 is_g93_strict 后会跳过重算。
    if(seg->is_g93_strict){
        double T_total_ms = g93_dt_sec * 1000.0;
        double v_const = (T_total_ms > 1e-9) ? dist / T_total_ms : seg->v_target;
        if(v_const < 1e-9) v_const = 1e-9;

        seg->T_total = T_total_ms;
        seg->T1 = 0.0; seg->T2 = 0.0; seg->T3 = 0.0;
        seg->T4 = T_total_ms;
        seg->T5 = T_total_ms; seg->T6 = T_total_ms; seg->T7 = T_total_ms;

        seg->v_target = v_const;
        seg->v_start  = v_const;
        seg->v_end    = v_const;
        seg->v_max    = v_const;
        seg->v0 = v_const; seg->v1 = v_const; seg->v2 = v_const; seg->v3 = v_const;
        seg->v4 = v_const; seg->v5 = v_const; seg->v6 = v_const;

        seg->s0 = 0.0; seg->s1 = 0.0; seg->s2 = 0.0; seg->s3 = 0.0;
        seg->s4 = dist; seg->s5 = dist; seg->s6 = dist;

        seg->j1 = 0.0; seg->a2 = 0.0; seg->j3 = 0.0;
        seg->j5 = 0.0; seg->a6 = 0.0; seg->j7 = 0.0;
    }
    seg->j5=0; seg->a6=0; seg->j7=0;

    for(int i=0;i<AXIS_NUM;i++){
        plan_cursor[i]=target_pos[i];
    }

    // ---- P0-b v1: 段元数据填充 + fork 副本到 PreviewStreamer ----
    // 必须在 write_head 推进之前完成, 保证 RT 消费者看到的 seg 字段完整
    // (RT 通过 is_ready + read_tail acquire 看到 seg_id/line_no/motion_type).
    // PreviewStreamer_Push 内部仅 memcpy + atomic store, 无锁, 不显著延长 spinlock 持有时间.
    // 设计上 PreviewStreamer 是只读 fork 副本, 不影响 motion queue 行为.
    seg->seg_id = atomic_fetch_add_explicit(&g_seg_id_counter, 1, memory_order_relaxed);
    seg->line_no = g_current_line_no;
    seg->motion_type = g_current_motion_type;
    PreviewStreamer_Push(seg);

    // relaxed 推进 write_head: 由 spinlock release 统一建立可见性。
    atomic_store_explicit(&g_cmd_queue.write_head, next_head, memory_order_relaxed);

    // 释放 spinlock (release): 建立 buffer 修改的 happens-before,
    // 下一任 acquire 持锁者 (planner/producer) 必定看到本段完整数据。
    atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);

    // 触发规划: planner 内部 try-lock 同一把 spinlock,
    // 失败立即返回 (生产者 push 不阻塞),由 watchdog 兜底。
    planner_recalculate(0);
    return 0;
}


// @Context: Non-RealTime Background Thread (parser / 上层管理线程)
// @Thread-Safety: Lock-Free SPSC 队列 (假设生产者串行调用)
int api_push_mcode(int m_code, double s_value, double p_value, double q_value, double r_value)
{
    if(atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==1) {
        printf("[SAFETY] 系统报警中，拒绝入队M代码指令！\n");
        return -1;
    }
    if(!g_all_axis_op_ready) return -1;

    // === queue_spinlock 保护的入队临界区 (与 api_push_trajectory_impl 同模式) ===
    int head, next_head;
    while (1) {
        while (atomic_flag_test_and_set_explicit(&g_cmd_queue.queue_spinlock,
                                                 memory_order_acquire)) {
#if defined(__x86_64__) || defined(__i386__)
            __asm__ volatile("pause" ::: "memory");
#elif defined(__aarch64__)
            __asm__ volatile("yield" ::: "memory");
#endif
        }
        int tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_relaxed);
        head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_relaxed);
        next_head = (head + 1) % QUEUE_SIZE;
        if (next_head != tail) break;

        atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);
        if (!dorun || atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==1) return -1;
        osal_usleep(1000);
    }

    TrajectorySegment_t *seg = &g_cmd_queue.buffer[head];
    memset(seg, 0, sizeof(TrajectorySegment_t));

    seg->cmd_type      = CMD_TYPE_MCODE;
    seg->active_wcs    = g_state.modal_wcs;  // M 代码段也需携带 WCS: RT 消费 M 代码时同步翻 UI 坐标系
    {
        // H-1: M 代码段同样冻结偏置快照, RT 在 M 代码等待屏障期间推导 UI 逻辑坐标要用
        double snap[AXIS_NUM];
        snapshot_wcs_offset(g_state.modal_wcs, snap);
        for(int i = 0; i < AXIS_NUM; i++) seg->wcs_offset_snap[i] = snap[i];
    }
    seg->m_code        = m_code;
    seg->s_value       = s_value;
    seg->p_value       = p_value;
    seg->q_value       = q_value;
    seg->r_value       = r_value;
    atomic_store_explicit(&seg->is_ready, 0, memory_order_relaxed);
    seg->is_rtcp_active = 0;  // M 代码不属于 RTCP 运动路径
    seg->total_distance = 0.0;
    seg->v_target      = 0.0;
    seg->v_start       = 0.0;
    seg->v_end         = 0.0;
    seg->acc           = 0.0;
    seg->dec           = 0.0;
    seg->T1=0; seg->T2=0; seg->T3=0; seg->T4=0;
    seg->T5=0; seg->T6=0; seg->T7=0; seg->T_total=0;
    seg->v0=0; seg->v1=0; seg->v2=0; seg->v3=0;
    seg->v4=0; seg->v5=0; seg->v6=0;
    seg->s0=0; seg->s1=0; seg->s2=0; seg->s3=0;
    seg->s4=0; seg->s5=0; seg->s6=0;
    seg->j1=0; seg->a2=0; seg->j3=0;
    seg->j5=0; seg->a6=0; seg->j7=0;

    // ---- P1': 辅助状态机快照 (parser 端 g_state → seg, 供 RT 消费时同步镜像) ----
    seg->aux_spindle_mode = g_state.spindle_mode;
    seg->aux_spindle_rpm  = g_state.spindle_rpm;
    seg->aux_coolant      = g_state.coolant_state;
    seg->aux_tool_id      = g_state.current_tool_id;

    // ---- P0-Laser: 激光辅助状态机快照 (与 P1' 同链路) ----
    // M3/M5 联动 laser_enable (复用 spindle_mode); M62/M63 同步激光闸;
    // M67/M68 设置功率/频率; M10/M11/M12 选气体. 每段都拷 (含运动段),
    // 保证激光开/关与运动段 1ms 边界严格对齐.
    seg->aux_laser_enable  = (g_state.spindle_mode != 0) ? 1 : 0;
    seg->aux_laser_shutter = g_state.laser_shutter_pending;
    seg->aux_laser_power_w = g_state.laser_power_pending;
    seg->aux_laser_freq_hz = g_state.laser_freq_pending;
    seg->aux_gas_select    = g_state.gas_select;
    // Phase B1: 段级耦合配置快照 (修复架构 BUG — 不再让 RT 读全局 g_laser_cfg)
    seg->aux_laser_coupling_mode = atomic_load_explicit(&g_laser_cfg.coupling_mode,
                                                        memory_order_acquire);
    seg->aux_laser_v_thresh      = atomic_load_explicit(&g_laser_cfg.v_thresh_mm_s,
                                                        memory_order_acquire);

    // relaxed 推进 write_head: 由 spinlock release 建立可见性。
    atomic_store_explicit(&g_cmd_queue.write_head, next_head, memory_order_relaxed);
    atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);

    planner_recalculate(0);
    return 0;
}


int is_trajectory_finished(){
    // Acquire 读两侧游标: 与生产者 release 写、消费者 release 写配对,
    // 确保看到最新的队列状态 (避免 stale read 导致提前返回 1)。
    int head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_acquire);
    int tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_acquire);
    if(head==tail
       && g_interpolator.is_moving==0
       && g_interpolator.is_waiting_mcode==0){
        return 1;
    }
    return 0;
}

// @Context: Non-RealTime Background Thread (parser 文件解析结束调用)
// @Thread-Safety: Spin-Wait 获取 queue_spinlock 后直接调用内部核心逻辑。
//   区别于 planner_recalculate 的非阻塞 try-lock: 这里是 parser 生命周期
//   的最后一道防线，必须保证 flush 成功，否则最后一段永远得不到 is_ready=1。
// @Danger: 此处阻塞最坏约 µs 级 (持锁者的临界区为纯计算)，且 parser 线程
//          已无后续实时性要求，允许短暂阻塞。
void api_flush_planner(){
    int retries = 0;
    // Spin-Wait: 自旋直到成功获取 queue_spinlock。
    // 竞争者 (watchdog/BSpline) 持锁时间均在 µs 级，正常 1~3 次 PAUSE 即可拿到。
    while (atomic_flag_test_and_set_explicit(&g_cmd_queue.queue_spinlock,
                                             memory_order_acquire)) {
        retries++;
#if defined(__x86_64__) || defined(__i386__)
        __asm__ volatile("pause" ::: "memory");
#elif defined(__aarch64__)
        __asm__ volatile("yield" ::: "memory");
#endif
        // 防御: 10^6 次 PAUSE ≈ 50ms @ x86 (Skylake ~140 cycles/PAUSE @ 3GHz)。
        // 抢不到说明持锁者死锁/内核卡死/调度异常,放弃 flush 避免永久阻塞 parser。
        // 必须留 printf 痕迹: 这是"不该发生"的异常路径,静默 return 会掩盖真实故障。
        if (retries > 1000000) {
            printf("[FLUSH] WARN: queue_spinlock spin-wait 超时 (~50ms),"
                   "flush 放弃 - 持锁者可能死锁或调度异常\n");
            return;
        }
    }
    // 持锁成功，直接内联 planner_recalculate 的核心逻辑，
    // force_flush=1 保证全部残余段 is_ready 置 1。
    planner_recalculate_locked(1);
    atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);
}

void api_motion_pause(){
    g_interpolator.pause_request=1;
}

void api_motion_resume(){
    g_interpolator.pause_request=0;
}

// @Context: Non-RealTime Background Thread (上层管理线程调用)
// @Thread-Safety: 仅设置 alarm_reset_request 标志，由 RT 线程在 HOLD_PAUSED 安全点消化
// 安全条件：全部轴使能就绪 + 全部轴跟随误差归零
// 注意：不再检查队列和插补器状态（这两个检查曾导致死锁和竞态），
//       实际复位动作由 RT 线程在确认电机刹停(HOLD_PAUSED)后执行
//
// Hazard 2 修复 (飞车防护): 复位必须先联动中止 Parser, 并同步等待其退出,
// 再提交 RT 清队列请求。否则 parser 会继续基于报警前 cursor 推 target_pos 段,
// RT 清队列后新段继续入队 → dir_vec 撕裂物理位置 → 飞车。
int api_alarm_reset(void){
    if(atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==0){
        printf("[Alarm] 当前无报警，无需复位\n");
        return 0;
    }

    // 防重入：上一次请求尚未被 RT 线程消化，拒绝重复提交
    if(atomic_load_explicit(&g_interpolator.alarm_reset_request, memory_order_acquire)==1){
        printf("[Alarm] 上一次复位请求尚未被消化，请勿重复提交\n");
        return -2;
    }

    // ---- Hazard 2: 先联动中止 Parser ----
    // 必须在提交 RT 复位之前确保 parser 已停止 push 段, 否则清队列后仍有新段入队。
    if(g_parser_ctrl.is_running){
        g_parser_ctrl.abort_request = 1;
        printf("[Alarm] 联动中止 Parser, 等待解析线程退出...\n");
        // Spin-wait 上限 2s: parser 主循环每行检查 abort_request (gcode_parser.c:1161),
        // dwell 期间 10ms 轮询 (gcode_parser.c:1337/1344), 正常 <100ms 退出。
        for(int i = 0; i < 2000; i++){
            if(!g_parser_ctrl.is_running) break;
            osal_usleep(1000);
        }
        if(g_parser_ctrl.is_running){
            // 兜底: 超时仍未退出 (极端: parser 卡在阻塞系统调用), 强制继续复位。
            // RT 清队列仍会丢弃未消费段, 但若 parser 此后才推新段, 仍可能撕裂 —
            // 此处仅告警, 由上层决定是否进一步处置 (如 kill 线程)。
            printf("[Alarm] 警告: Parser 2s 内未退出, 强制继续复位 (注意残留段风险)\n");
        }
        // abort_request 复位由下次 SMC_RunGCodeFile 统一处理 (smc_api.c:403),
        // 不在此处清零 — 防 parser 在 exit 路径上误判。
    }

    // 提交复位请求，由 RT 线程在安全点(HOLD_PAUSED && op_ready)执行实际状态清理
    atomic_store_explicit(&g_interpolator.alarm_reset_request, 1, memory_order_release);

    printf("[Alarm] 复位请求已提交，等待 RT 线程在安全点确认...\n");
    return 0;
}

void axis_sys_init(void)
{
    // 0. 轴数据结构初始化
    memset(g_axis,0,sizeof(g_axis));
    for(int i=0;i<26;i++) g_axis_map[i]=-1; // 轴映射表全部置为未映射

    // 1. 插补器初始化 (零填充足够,无 atomic 字段需要单独初始化)
    memset(&g_interpolator,0,sizeof(Interpolator_t));

    // 2. Hybrid CommandQueue 初始化:
    //    - buffer 由生产者在使用槽位时自行初始化,无需清零
    //    - write_head / read_tail / queue_spinlock 必须显式原子初始化
    atomic_store_explicit(&g_cmd_queue.write_head, 0, memory_order_relaxed);
    atomic_store_explicit(&g_cmd_queue.read_tail,  0, memory_order_relaxed);
    atomic_flag_clear_explicit(&g_cmd_queue.queue_spinlock, memory_order_release);


    // 3. 规划器光标初始化
    for(int i=0;i<AXIS_NUM;i++){
        plan_cursor[i]=0.0;
    }

    // 4. 全局状态标志初始化
    g_all_axis_op_ready=0;
    g_all_axis_reach=0;

    for(int i=0;i<AXIS_NUM;i++){
        g_axis[i].pulse_per_unit=10000.0; // 默认10000脉冲/单位
        // 动力学安全默认值：防止未配置时零速死锁或无限速飞车
        g_axis[i].max_speed = 200.0;  // mm/s
        g_axis[i].max_acc   = 200.0;  // mm/s^2
        g_axis[i].max_dec   = 200.0;  // mm/s^2
        g_axis[i].max_jerk  = 5000.0; // mm/s^3
        // 旋转轴等效半径默认值：用户须通过 SMC_ConfigAxisDynamics 覆盖
        g_axis[i].equivalent_radius = 0.0; // 0.0 表示未配置，退化回原始行为
    }

    // 5.
    g_interpolator.virtual_time_ms=0.0;
    g_interpolator.time_scale=1.0;
    g_interpolator.hold_state=HOLD_NORMAL;
    g_interpolator.pause_request=0;

    printf("[系统初始化] 五轴控制系统已初始化，等待SOEM主站配置...\n");




}

/************************ 单轴SDO配置PP模式 ************************/
void axis_sdo_config_pp(int axis_idx)
{
    
    axis_sdo_config_mode(axis_idx, CSP_MODE);
    
    
    /*if (axis_idx < 0 || axis_idx >= AXIS_NUM)
    {
        printf("[SDO错误] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    if (g_axis[axis_idx].is_error) return; // 故障轴跳过

    uint8 pp_mode = 0x01; // CiA402 PP模式=1
    int sz = sizeof(pp_mode);
    ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM);
    osal_usleep(200000);
    if (ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, sz, &pp_mode, EC_TIMEOUTRXM) == 0)
    {
        printf("[SDO配置] %s PP模式配置成功(0x6060=1)\n", g_axis[axis_idx].axis_name);
    }
    else
    {
        printf("[SDO错误] %s PP模式配置失败！请检查从站ID和SAFE_OP状态\n", g_axis[axis_idx].axis_name);
        g_axis[axis_idx].is_error = 1;
        exit(-1); // 配置失败直接退出，
    }*/


}

void axis_sdo_config_mode(int axis_idx, uint8_t mode)
{
    if (g_sim_mode) return; // 仿真模式: 跳过真实 SDO 写入
    if (axis_idx < 0 || axis_idx >= AXIS_NUM)
    {
        printf("[SDO错误] 轴索引越界！输入索引：%d，最大索引：%d\n", axis_idx, AXIS_NUM-1);
        return;
    }
    if (g_axis[axis_idx].is_error) return;

    int sz = sizeof(mode);
    // 1.写入CiA402模式值（0x6060）
    for(int s=0;s<g_axis[axis_idx].slave_count;s++){
        int wkc = ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_ids[s], 0x6060, 0x00, FALSE, sz, &mode, EC_TIMEOUTRXM);
        osal_usleep(200000); 

        //sz=sizeof(g_axis[axis_idx].pp_target_pos);
        //ecx_SDOwrite(&ctx, g_axis[axis_idx].slave_id, 0x607A, 0x00, FALSE, sz, &g_axis[axis_idx].pp_target_pos, EC_TIMEOUTRXM);
        //osal_usleep(200000);
           // 2. 读回当前模式，验证是否配置成功
    }
}

// SDO读取函数（读取指定索引/子索引的值）
uint8_t axis_sdo_read_mode(int axis_idx)
{
    if (g_sim_mode) return CSP_MODE; // 仿真模式: 返回 CSP 模式
    uint8_t mode = 0;
    int sz = sizeof(mode);
    // 调用SOEM的SDO读函数（ecx_SDOread，与write逻辑对称）
    int wkc = ecx_SDOread(&ctx, g_axis[axis_idx].slave_id, 0x6060, 0x00, FALSE, &sz, &mode, EC_TIMEOUTRXM);
    
    // 即使wkc=0，也返回读取到的mode（可能有效）
    return mode;
}

/************************ PDO写（控制字+目标位置） ************************/
void axis_pdo_write(int slave_id, uint16 cw, int32 pos)
{
    // ---- 仿真模式: 转发到 sim_drive 推进状态机 + 一阶伺服 ----
    if (g_sim_mode) {
        int axis_idx, subidx;
        if (sim_drive_lookup_slave(slave_id, &axis_idx, &subidx) == 0) {
            uint64_t cyc = atomic_load_explicit(&g_sim_rt_cycle, memory_order_relaxed);
            sim_drive_step_axis(axis_idx, subidx, cw, pos, cyc);
            g_axis[axis_idx].sim_cmd_cw    = cw;
            g_axis[axis_idx].sim_target_pos = pos;
        }
        return;
    }

    // 越界检查
    if (slave_id <= 0 || slave_id > ctx.slavecount) return;

    // 空指针检查+故障置位
    uint8 *out = ctx.slavelist[slave_id].outputs;
    if (!out)
    {
        printf("[PDO错误] %s PDO输出缓冲区为空！\n");
        return;
    }

    // 写入控制字（2字节，小端）
    out[PDO_CW_BYTE0] = cw & 0xFF;
    out[PDO_CW_BYTE1] = (cw >> 8) & 0xFF;

    // 写入目标位置（4字节，小端，台达B3-E标准）
    out[PDO_POS_BYTE0] = pos & 0xFF;
    out[PDO_POS_BYTE1] = (pos >> 8) & 0xFF;
    out[PDO_POS_BYTE2] = (pos >> 16) & 0xFF;
    out[PDO_POS_BYTE3] = (pos >> 24) & 0xFF;
}

/************************ 单轴PDO读（状态字） ************************/
uint16 axis_pdo_read_sw(int slave_id)
{
    // ---- 仿真模式: 查 sim_drive 内部 CiA402 状态 (含 SW_ERROR / SW_TARGET_REACH) ----
    if (g_sim_mode) {
        int axis_idx, subidx;
        if (sim_drive_lookup_slave(slave_id, &axis_idx, &subidx) == 0) {
            return sim_drive_get_sw(axis_idx, subidx);
        }
        return 0x0000;
    }

    // 1. 基础校验：索引越界/轴故障/从站ID无效
    if (slave_id <= 0 || slave_id > ctx.slavecount) return 0xFFFF;

    // 2. 重试获取PDO输入缓冲区（防偶发空指针）
    uint8 *in = NULL;
    for (int retry = 0; retry < 2; retry++) {
        in = ctx.slavelist[slave_id].inputs;
        if (in) break;
        osal_usleep(1000); // 1ms重试间隔
    }

    // 3. 解析状态字（小端序：低字节在前，高字节在后）
    uint16 sw = (uint16)(in[PDO_SW_BYTE1] << 8 | in[PDO_SW_BYTE0]);
    return sw;
}

/************************ 状态位解析函数（上层调用） ************************/
// 判断某状态位是否为1
int axis_check_sw_bit(int axis_idx, uint16 bit_mask)
{
    uint16 sw = axis_pdo_read_sw(axis_idx);
    if (sw == 0xFFFF) return -1; // 读取失败
    return (sw & bit_mask) ? 1 : 0; // 1=位有效，0=位无效
}

// 打印状态字详细解析（调试用）
void axis_print_sw_detail(int axis_idx)
{
    uint16 sw = axis_pdo_read_sw(axis_idx);
    if (sw == 0xFFFF) {
        printf("[状态字解析] %s 读取失败！\n", g_axis[axis_idx].axis_name);
        return;
    }

    printf("[状态字解析] %s 完整值：0x%04X\n", g_axis[axis_idx].axis_name, sw);
    printf("  - 准备功能启动：%s\n", (sw & SW_READY_FUNC_START) ? "是" : "否");
    printf("  - 伺服准备完成：%s\n", (sw & SW_SERVO_READY) ? "是" : "否");
    printf("  - 伺服使能：%s\n", (sw & SW_SERVO_ENABLE) ? "是" : "否");
    printf("  - 异常信号：%s\n", (sw & SW_ERROR) ? "是" : "否");
    printf("  - 入力侧供电：%s\n", (sw & SW_MAIN_POWER_ON) ? "是" : "否");
    printf("  - 紧急停止：%s\n", (sw & SW_EMERGENCY_STOP) ? "是" : "否");
    printf("  - 警告信号：%s\n", (sw & SW_WARNING) ? "是" : "否");
    printf("  - 远程控制：%s\n", (sw & SW_REMOTE_CTRL) ? "是" : "否");
    printf("  - 目标到达：%s\n", (sw & SW_TARGET_REACH) ? "是" : "否");
}


/************************ 配置CSP模式所有参数 ************************/
void axis_config_csp_params(int axis_idx)
{
    if (g_sim_mode) return; // 仿真模式: 跳过真实 SDO 配置
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    
    int slave_id = g_axis[axis_idx].slave_id;
    printf("\n========== 配置 %s CSP参数（修正版） ==========\n", g_axis[axis_idx].axis_name);
    
    // 1. 最大轮廓速度 (0x607F) - 单位: pulse/s
    // 使用驱动器实际支持的值（从读回值看，60000是有效的）
    uint32_t profile_velocity = 60000;  // 60k pulse/s = 60 pulse/ms
    int wkc = ecx_SDOwrite(&ctx, slave_id, 0x607F, 0x00, FALSE, 4, &profile_velocity, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    // 读回验证
    uint32_t read_vel = 0;
    int sz = sizeof(read_vel);
    ecx_SDOread(&ctx, slave_id, 0x607F, 0x00, FALSE, &sz, &read_vel, EC_TIMEOUTRXM);
    printf("0x607F 最大轮廓速度: %d pulse/s\n", read_vel);
    
    for(int i=0;i<AXIS_NUM;i++){
        int32_t actual_pos = 0;
        sz = sizeof(actual_pos);
        ecx_SDOread(&ctx, g_axis[i].slave_id, 0x6064, 0x00, FALSE, &sz, &actual_pos, EC_TIMEOUTRXM);
        g_axis[i].target_pos=actual_pos;
    }
    /*// 2. 加速度 (0x6083) - 使用驱动器支持的值
    uint32_t acceleration = 10000;  // 从读回值看，19264附近是有效的
    ecx_SDOwrite(&ctx, slave_id, 0x6083, 0x00, FALSE, 4, &acceleration, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    uint32_t read_acc = 0;
    ecx_SDOread(&ctx, slave_id, 0x6083, 0x00, FALSE, &sz, &read_acc, EC_TIMEOUTRXM);
    printf("0x6083 加速度: %d pulse/s²\n", read_acc);
    
    // 3. 减速度 (0x6084) - 与加速度相同
    ecx_SDOwrite(&ctx, slave_id, 0x6084, 0x00, FALSE, 4, &acceleration, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    uint32_t read_dec = 0;
    ecx_SDOread(&ctx, slave_id, 0x6084, 0x00, FALSE, &sz, &read_dec, EC_TIMEOUTRXM);
    printf("0x6084 减速度: %d pulse/s²\n", read_dec);
     */ 
    // 4. 软件限位保持不变
    int32_t min_pos = -1000000;
    int32_t max_pos = 1000000;
    ecx_SDOwrite(&ctx, slave_id, 0x607D, 0x01, FALSE, 4, &min_pos, EC_TIMEOUTRXM);
    osal_usleep(30000);
    ecx_SDOwrite(&ctx, slave_id, 0x607D, 0x02, FALSE, 4, &max_pos, EC_TIMEOUTRXM);
    osal_usleep(30000);
    
    printf("========== %s CSP参数配置完成 ==========\n\n", g_axis[axis_idx].axis_name);
}


/************************ 读取CSP运行状态 ************************/
void axis_read_csp_status(int axis_idx)
{
    if (g_sim_mode) return; // 仿真模式: 无真实驱动器状态可读
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    
    int slave_id = g_axis[axis_idx].slave_id;
    
    // 1. 状态字
    uint16_t sw = axis_pdo_read_sw(axis_idx);
    
    // 2. 当前模式 (0x6061)
    uint8_t current_mode = 0;
    int sz = sizeof(current_mode);
    ecx_SDOread(&ctx, slave_id, 0x6061, 0x00, FALSE, &sz, &current_mode, EC_TIMEOUTRXM);
    
    // 3. 实际位置 (0x6064)
    int32_t actual_pos = 0;
    sz = sizeof(actual_pos);
    ecx_SDOread(&ctx, slave_id, 0x6064, 0x00, FALSE, &sz, &actual_pos, EC_TIMEOUTRXM);
    
    // 4. 跟随误差 (0x60F4)
    int32_t following_error = 0;
    sz = sizeof(following_error);
    ecx_SDOread(&ctx, slave_id, 0x60F4, 0x00, FALSE, &sz, &following_error, EC_TIMEOUTRXM);
    
    // 5. 速度实际值 (0x606C)
    int32_t actual_vel = 0;
    sz = sizeof(actual_vel);
    ecx_SDOread(&ctx, slave_id, 0x606C, 0x00, FALSE, &sz, &actual_vel, EC_TIMEOUTRXM);
    
    printf("\n========== %s 运行状态 ==========\n", g_axis[axis_idx].axis_name);
    printf("状态字: 0x%04X (bit3=%d, bit12=%d)\n", sw, (sw>>3)&1, (sw>>12)&1);
    printf("当前模式 (0x6061): %d %s\n", current_mode, 
           (current_mode == 8) ? "(CSP)" : "(其他)");
    printf("目标位置: %d\n", g_axis[axis_idx].target_pos);
    printf("实际位置 (0x6064): %d\n", actual_pos);
    printf("跟随误差 (0x60F4): %d\n", following_error);
    printf("实际速度 (0x606C): %d pulse/s\n", actual_vel);
    
    if(actual_pos != g_axis[axis_idx].target_pos) {
        printf("⚠️ 实际位置与目标位置不符！差值: %d\n", 
               g_axis[axis_idx].target_pos - actual_pos);
    }
    
    if(following_error > 1000) {
        printf("⚠️ 跟随误差过大！\n");
    }
    
    printf("=====================================\n");
}

void check_pdo_mapping(int axis_idx)
{
    if (g_sim_mode) { printf("[SIM] check_pdo_mapping: 仿真模式下跳过\n"); return; }
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s PDO映射检查 ==========\n", g_axis[axis_idx].axis_name);
    
    // 检查RxPDO分配 (0x1C12)
    uint8_t sub_count = 0;
    int sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1C12, 0x00, FALSE, &sz, &sub_count, EC_TIMEOUTRXM);
    printf("RxPDO分配 (0x1C12) 子索引数: %d\n", sub_count);
    
    for(int i = 1; i <= sub_count && i <= 10; i++) {
        uint16_t pdo_num = 0;
        sz = 2;
        ecx_SDOread(&ctx, slave_id, 0x1C12, i, FALSE, &sz, &pdo_num, EC_TIMEOUTRXM);
        printf("  RxPDO[%d] = 0x%04X\n", i, pdo_num);
        
        // 打印RxPDO详细映射
        if(pdo_num >= 0x1600 && pdo_num <= 0x1603) {
            uint8_t map_sub = 0;
            sz = 1;
            ecx_SDOread(&ctx, slave_id, pdo_num, 0x00, FALSE, &sz, &map_sub, EC_TIMEOUTRXM);
            printf("    RxPDO 0x%04X 映射对象数: %d\n", pdo_num, map_sub);
            
            for(int j = 1; j <= map_sub; j++) {
                uint32_t mapping = 0;
                sz = 4;
                ecx_SDOread(&ctx, slave_id, pdo_num, j, FALSE, &sz, &mapping, EC_TIMEOUTRXM);
                uint16_t index = (mapping >> 16) & 0xFFFF;
                uint8_t sub = (mapping >> 8) & 0xFF;
                uint8_t bits = mapping & 0xFF;
                printf("      0x%04X:%02X (%d bits)\n", index, sub, bits);
            }
        }
    }
    
    // 检查TxPDO分配 (0x1C13) - 关键修改：打印详细映射
    sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1C13, 0x00, FALSE, &sz, &sub_count, EC_TIMEOUTRXM);
    printf("\nTxPDO分配 (0x1C13) 子索引数: %d\n", sub_count);
    
    for(int i = 1; i <= sub_count && i <= 10; i++) {
        uint16_t pdo_num = 0;
        sz = 2;
        ecx_SDOread(&ctx, slave_id, 0x1C13, i, FALSE, &sz, &pdo_num, EC_TIMEOUTRXM);
        printf("  TxPDO[%d] = 0x%04X\n", i, pdo_num);
        
        // ========== 关键修改：打印TxPDO详细映射 ==========
        if(pdo_num >= 0x1A00 && pdo_num <= 0x1A03) {
            uint8_t map_sub = 0;
            sz = 1;
            ecx_SDOread(&ctx, slave_id, pdo_num, 0x00, FALSE, &sz, &map_sub, EC_TIMEOUTRXM);
            printf("    TxPDO 0x%04X 映射对象数: %d\n", pdo_num, map_sub);
            
            for(int j = 1; j <= map_sub; j++) {
                uint32_t mapping = 0;
                sz = 4;
                ecx_SDOread(&ctx, slave_id, pdo_num, j, FALSE, &sz, &mapping, EC_TIMEOUTRXM);
                uint16_t index = (mapping >> 16) & 0xFFFF;
                uint8_t sub = (mapping >> 8) & 0xFF;
                uint8_t bits = mapping & 0xFF;
                printf("      0x%04X:%02X (%d bits)\n", index, sub, bits);
                
                // 检查是否包含实际位置
                if(index == 0x6064) {
                    printf("      ✅ 包含实际位置 (0x6064)\n");
                }
            }
        }
    }
    
    printf("=====================================\n");
}

void read_error_history(int axis_idx)
{
    if (g_sim_mode) { printf("[SIM] read_error_history: 仿真模式下跳过\n"); return; }
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s 故障历史 ==========\n", g_axis[axis_idx].axis_name);
    
    // 读取故障码数量
    uint8_t err_count = 0;
    int sz = sizeof(err_count);
    ecx_SDOread(&ctx, slave_id, 0x1003, 0x00, FALSE, &sz, &err_count, EC_TIMEOUTRXM);
    printf("故障记录数: %d\n", err_count);
    
    // 读取每个故障码
    for(int i = 1; i <= err_count && i <= 8; i++) {
        uint32_t err_code = 0;
        sz = sizeof(err_code);
        ecx_SDOread(&ctx, slave_id, 0x1003, i, FALSE, &sz, &err_code, EC_TIMEOUTRXM);
        printf("故障[%d]: 0x%08X\n", i, err_code);
        
        // 常见故障码解释
        switch(err_code & 0xFFFF) {
            case 0x2310:
                printf("  位置跟随误差过大\n");
                break;
            case 0x2320:
                printf("  速度过快\n");
                break;
            case 0x5110:
                printf("  参数设置错误\n");
                break;
            case 0x7380:
                printf("  紧急停止\n");
                break;
            default:
                printf("  请查阅台达B3手册\n");
        }
    }
    
    // 清除故障历史（谨慎使用）
    // uint8_t clear = 0;
    // ecx_SDOwrite(&ctx, slave_id, 0x1003, 0x00, FALSE, 1, &clear, EC_TIMEOUTRXM);
    
    printf("====================================\n");
}

/*********************** 写控制字（使能过程用） ************************/
void axis_pdo_write_cw_only(int axis_idx, uint16 cw)
{
    // 越界检查
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    if (g_axis[axis_idx].is_error) return;

    uint8 *out = ctx.slavelist[g_axis[axis_idx].slave_id].outputs;
    if (!out)
    {
        g_axis[axis_idx].is_error = 1;
        printf("[PDO错误] %s PDO输出缓冲区为空！\n", g_axis[axis_idx].axis_name);
        return;
    }

    // 写入控制字
    out[PDO_CW_BYTE0] = cw & 0xFF;
    out[PDO_CW_BYTE1] = (cw >> 8) & 0xFF;
    
  
}

void diagnose_sync_failure(int axis_idx)
{
    if (g_sim_mode) { printf("[SIM] diagnose_sync_failure: 仿真模式下跳过\n"); return; }
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("\n========== %s 同步诊断 ==========\n", g_axis[axis_idx].axis_name);
    
    // 1. 检查DC同步状态 (0x1C32:08)
    uint16_t sync_error = 0;
    int sz = 2;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x08, FALSE, &sz, &sync_error, EC_TIMEOUTRXM);
    printf("0x1C32:08 (同步错误) = 0x%04X\n", sync_error);
    
    // 2. 检查周期过小/过大计数
    uint32_t cycle_small = 0, cycle_large = 0;
    sz = 4;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x0A, FALSE, &sz, &cycle_small, EC_TIMEOUTRXM);
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x0B, FALSE, &sz, &cycle_large, EC_TIMEOUTRXM);
    printf("周期过小计数: %d\n", cycle_small);
    printf("周期过大计数: %d\n", cycle_large);
    
    // 3. 检查当前同步模式 (0x1C32:01)
    uint16_t sync_mode = 0;
    sz = 2;
    ecx_SDOread(&ctx, slave_id, 0x1C32, 0x01, FALSE, &sz, &sync_mode, EC_TIMEOUTRXM);
    printf("当前同步模式: %d (2=DC模式)\n", sync_mode);
    
    // 4. 检查DC激活状态
    printf("DCactive: %d\n", ctx.slavelist[slave_id].DCactive);
    printf("DCcycle: %d ns\n", ctx.slavelist[slave_id].DCcycle);
    
    // 5. 读取警告码 (0x1003可能记录警告)
    uint8_t err_count = 0;
    sz = 1;
    ecx_SDOread(&ctx, slave_id, 0x1003, 0x00, FALSE, &sz, &err_count, EC_TIMEOUTRXM);
    printf("故障记录数: %d\n", err_count);
    for(int i = 1; i <= err_count; i++) {
        uint32_t err_code = 0;
        sz = 4;
        ecx_SDOread(&ctx, slave_id, 0x1003, i, FALSE, &sz, &err_code, EC_TIMEOUTRXM);
        printf("  故障[%d]: 0x%08X\n", i, err_code);
    }
    
    printf("====================================\n");
}



/************************ 从PDO读取实际位置 ************************/
int32 axis_pdo_read_pos(int slave_id)
{
    // ---- 仿真模式: 返回 sim_drive 一阶低通推算的实际位置 ----
    if (g_sim_mode) {
        int axis_idx, subidx;
        if (sim_drive_lookup_slave(slave_id, &axis_idx, &subidx) == 0) {
            return sim_drive_get_pos(axis_idx, subidx);
        }
        return 0;
    }

    if (slave_id <=0 || slave_id > ctx.slavecount) return 0;
    
    uint8 *in = ctx.slavelist[slave_id].inputs;
    if (!in) return 0;
    
    // TxPDO 0x1A01 映射：偏移0-1状态字，偏移2-5实际位置
    int32_t actual_pos = (int32_t)(
        in[2] | 
        (in[3] << 8) | 
        (in[4] << 16) | 
        (in[5] << 24)
    );
    
    return actual_pos;
}

/************************ 从PDO读取跟随误差 (0x60F4) ************************/
int32_t axis_pdo_read_follow_err(int slave_id)
{
    // ---- 仿真模式: 返回 sim_drive 的 (target - actual) ----
    if (g_sim_mode) {
        int axis_idx, subidx;
        if (sim_drive_lookup_slave(slave_id, &axis_idx, &subidx) == 0) {
            return sim_drive_get_follow_err(axis_idx, subidx);
        }
        return 0;
    }

    if (slave_id <=0 || slave_id > ctx.slavecount) return 0;

    uint8 *in = ctx.slavelist[slave_id].inputs;
    if (!in) return 0;

    int32_t follow_err = (int32_t)(
        in[PDO_FOLLOW_BYTE0] |
        (in[PDO_FOLLOW_BYTE1] << 8) |
        (in[PDO_FOLLOW_BYTE2] << 16) |
        (in[PDO_FOLLOW_BYTE3] << 24)
    );

    return follow_err;
}

/************************ 执行原点复归 (Homing) ************************/
void axis_homing(int axis_idx)
{
    if (g_sim_mode) return; // 仿真模式: 跳过真实 SDO 归零
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    int slave_id = g_axis[axis_idx].slave_id;
    
    printf("[Homing] %s 开始原点复归...\n", g_axis[axis_idx].axis_name);
    
    // 1. 切换到原点复归模式 (0x6060 = 6)
    uint8_t homing_mode = 6;
    ecx_SDOwrite(&ctx, slave_id, 0x6060, 0x00, FALSE, 1, &homing_mode, EC_TIMEOUTRXM);
    osal_usleep(100000);
    
    // 2. 设置原点复归方法 (0x6098 = 35) —— 将当前位置设为原点
    int8_t homing_method = 35;
    ecx_SDOwrite(&ctx, slave_id, 0x6098, 0x00, FALSE, 1, &homing_method, EC_TIMEOUTRXM);
    osal_usleep(50000);
    
    // 3. 触发原点复归（控制字 bit4 = 1）
    // 注意：需要先使能，然后发送带触发位的控制字
    // 假设此时伺服已在使能状态（状态字0x0237）
    axis_pdo_write_cw_only(axis_idx, 0x001F);  // 0x001F = 使能 + 触发位
    ecx_send_processdata(&ctx);
    
    // 4. 等待复归完成（状态字 bit12 可能变为1，或等待一段时间）
    int timeout = 100; // 100ms * 100 = 10s
    uint16_t sw = 0;
    do {
        osal_usleep(100000);
        sw = axis_pdo_read_sw(axis_idx);
        timeout--;
        if (timeout == 0) {
            printf("[Homing] %s 超时！\n", g_axis[axis_idx].axis_name);
            break;
        }
    } while (!(sw & 0x1000)); // 假设 bit12 表示复归完成（需查手册）
    
    printf("[Homing] %s 完成，状态字=0x%04X\n", g_axis[axis_idx].axis_name, sw);
    
    // 5. 切换回CSP模式
    uint8_t csp_mode = 8;
    ecx_SDOwrite(&ctx, slave_id, 0x6060, 0x00, FALSE, 1, &csp_mode, EC_TIMEOUTRXM);
    osal_usleep(100000);
}
