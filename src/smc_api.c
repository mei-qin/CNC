#include "smc_api.h"
#include "global_def.h"
#include "ecat_core.h"
#include "axis_ctrl.h"
#include "gcode_parser.h"
#include "kinematics.h"
#include "bspline_engine.h"
#include "trace_logger.h"
#include "sim_drive.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>

// 底层轴房间号自动分配计数器
static int g_allocated_axis_count = 0;

// 字母 → 底层索引查表（内部工具函数）
// 返回 -1 表示轴未配置
static int axis_lookup(char axis_letter) {
    char c = toupper((unsigned char)axis_letter);
    if(c < 'A' || c > 'Z') return -1;
    return g_axis_map[c - 'A'];
}

// ================== 系统管理 ==================
int SMC_InitAndStart(const char *netif_name)
{
    printf("[SMC_API] 正在初始化运动控制内核...\n");

    // sim 模式: sim_drive 必须在 RT 线程启动前初始化好
    // 否则 RT 主循环进入 dorun==1 后立即调 sim_drive_get_sw 会读到 cia_state=0,
    // 而 sim_cia_advance 对未初始化状态返回原值, 状态机永久卡死。
    if (g_sim_mode) {
        sim_drive_init_all();
    }

    if (!osal_thread_create_rt(&thread_rt, 128000, &ecat_thread_rt, NULL)) {
        printf("[SMC_API] 实时控制线程创建失败！\n"); return -1;
    }
    if (!osal_thread_create(&thread_chk, 128000, &ecat_thread_chk, NULL)) {
        printf("[SMC_API] 故障检查线程创建失败！\n"); return -1;
    }
    if(!osal_thread_create(&thread_parser, 128000, &parser_thread_func, NULL)){
        printf("[SMC_API] G-code解析线程创建失败！\n"); return -1;
    }

    BSpline_Init();
    if(BSpline_StartThread() < 0){
        printf("[SMC_API] B-Spline平滑线程启动失败！\n"); return -1;
    }

    // 刀补引擎初始化: 设置输出回调为 api_push_trajectory
    // 当刀补激活时，偏置后的点直接发往规划器 (不经过 B-Spline)
    CutterComp_Init();
    CutterComp_SetOutput(api_push_trajectory);

    TraceLogger_Init();
    if (!g_sim_mode) {
        if(TraceLogger_StartThread() < 0){
            printf("[SMC_API] 轨迹探针线程启动失败！\n"); return -1;
        }
    }

    // 仿真模式: 启动高频双缓冲轨迹采集器
    if (g_sim_mode) {
        char ts_buf[300];
        time_t now = time(NULL);
        struct tm *tm_info = localtime(&now);
        strftime(ts_buf, sizeof(ts_buf),
                 "cnc_trace_log_%Y%m%d_%H%M%S.csv", tm_info);
        if (sim_engine_init(ts_buf, 0) != 0) {
            printf("[SMC_API] 仿真轨迹采集器初始化失败！\n"); return -1;
        }
        if (sim_engine_start() != 0) {
            printf("[SMC_API] 仿真轨迹落盘线程启动失败！\n"); return -1;
        }
    }

    if (g_sim_mode) {
        // ---- 仿真模式: 跳过 EtherCAT 硬件初始化 ----
        printf("[SMC_API] 仿真模式: 跳过 ecat_bringup\n");
        // sim_drive 已在函数入口初始化好, 此处只设 mappingdone/dorun 让 RT 进入主循环
        // 不设 g_all_axis_op_ready=1: 让下面的等待循环真正等 RT 状态机跑到 case 3
        // (sim_drive 状态机已稳定, 150 周期约 1-10ms 即可达 case 3, 远小于 100ms 等待粒度)
        // 旧实现设 1 会让等待循环首次读到 1 立即退出, 但 RT 此时还在 case 0/1/2,
        // 期间 client 查询 spindle/coolant 会拿到 -1
        mappingdone = 1;
        dorun = 1;
    } else {
        ecat_bringup((char*)netif_name);
    }

    printf("[SMC_API] 等待伺服全轴使能并进入 CSP 同步...\n");
    int timeout = 50; // 最多等5秒
    while (!g_all_axis_op_ready && timeout > 0) {
        osal_usleep(100000);
        timeout--;
    }
    if (timeout == 0) {
        printf("[SMC_API] 伺服使能超时！请检查硬件。\n"); return -1;
    }

    // 给系统时钟收敛留时间
    osal_usleep(1000000);
    printf("[SMC_API] 内核启动完毕，伺服就绪！\n");
    return 0;
}

void SMC_Close(void)
{
    printf("\n[SMC_API] 收到关闭请求，触发优雅下电时序...\n");

    // 先停止 B-Spline 平滑线程 (排空队列后退出)
    BSpline_StopThread();

    // 仿真模式: 停止双缓冲轨迹采集器 (排空残余数据后关闭文件)
    if (g_sim_mode) {
        sim_engine_finish();
    }

    // 停止轨迹探针落盘线程 (排空残余数据后退出)
    TraceLogger_StopThread();

    // 请求 RT 线程进入优雅下电状态机（抱闸闭合 + CiA402 降级）
    dorun = 2;

    // 阻塞等待 RT 线程完成下电状态机（最多 3 秒）
    int wait_cycles = 30;
    while (dorun != 0 && wait_cycles > 0) {
        osal_usleep(100000);
        wait_cycles--;
    }
    if (dorun != 0) {
        printf("[SMC_API] 优雅下电超时（3s），强制降级！\n");
        dorun = 0;
        osal_usleep(200000);
    } else {
        printf("[SMC_API] RT 线程优雅下电完成。\n");
    }

    // 降级 EtherCAT 状态机：OP → SAFE_OP → INIT (仅真实硬件模式)
    if (!g_sim_mode) {
        ctx.slavelist[0].state = EC_STATE_SAFE_OP;
        ecx_writestate(&ctx, 0);
        if (ecx_statecheck(&ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP) {
            printf("[ECAT警告] 切换SAFE_OP失败，但继续执行降级流程\n");
        }
        ctx.slavelist[0].state = EC_STATE_INIT;
        ecx_writestate(&ctx, 0);
        if (ecx_statecheck(&ctx, 0, EC_STATE_INIT, EC_TIMEOUTSTATE) != EC_STATE_INIT) {
            printf("[ECAT警告] 切换INIT失败，但继续关闭主站\n");
        }
        ecx_close(&ctx);
    }
    printf("[SMC_API] 系统已安全关闭！\n");
}

// ================== 轴配置 ==================

// @Context: Non-RealTime Background Thread（仅初始化阶段或停机重配置时调用）
// @Thread-Safety: 写入 g_axis[] 和 g_axis_map[]；加工运行中（队列未空 / 插补器未静止）
//                 禁止调用，否则会撕裂 RT 线程读取的 slave_ids[]，引发 PDO 映射错乱。
//
// 语义说明：
//   - 字母尚未映射 → 分配新房间号（g_allocated_axis_count++）
//   - 字母已映射   → 走重新配置路径，覆盖现有槽位的拓扑参数，不消耗新房间号
//                    （生产场景：rpc_server 启动时硬编码 5 轴 fallback，CAM 启动后
//                     通过 RPC 覆盖同一批轴的 master/slave 配置）
int SMC_ConfigAxisTopology(const char* axis_name, int is_dual_drive, int master_id, int slave_id){
    if(axis_name == NULL || axis_name[0] == '\0') {
        printf("[SMC_API] 轴名不能为空！\n");
        return -1;
    }

    // 首字母合法性校验前置（避免非法字母时被"房间号已满"误报掩盖真实错误）
    char letter = toupper((unsigned char)axis_name[0]);
    if(letter < 'A' || letter > 'Z') {
        printf("[SMC_API] 轴名首字母 '%c' 不在 A-Z 范围内！\n", axis_name[0]);
        return -1;
    }

    // ===== 重新配置分支：字母已映射，覆盖现有槽位的拓扑参数 =====
    int existing_idx = g_axis_map[letter - 'A'];
    if(existing_idx >= 0) {
        // 安全闸门：加工运行中禁止修改拓扑（与 SMC_ConfigAxisDynamics 一致）
        if(g_parser_ctrl.is_running || !is_trajectory_finished()) {
            printf("[SMC_API ERROR] 系统运行中（队列未空或插补器未静止），"
                   "禁止重新配置 '%s' 拓扑！\n", axis_name);
            return -1;
        }
        if(is_dual_drive){
            g_axis[existing_idx].slave_ids[0] = master_id;
            g_axis[existing_idx].slave_ids[1] = slave_id;
            g_axis[existing_idx].slave_count  = 2;
            printf("[SMC_API] 重新配置 '%s' → 房间[%d], 字母 '%c', 双驱 主ID:%d 从ID:%d\n",
                   axis_name, existing_idx, letter, master_id, slave_id);
        } else {
            g_axis[existing_idx].slave_ids[0] = master_id;
            g_axis[existing_idx].slave_count  = 1;
            printf("[SMC_API] 重新配置 '%s' → 房间[%d], 字母 '%c', 单驱 ID:%d\n",
                   axis_name, existing_idx, letter, master_id);
        }
        // 同步刷新轴名（用户可能把 "X轴" 重命名为 "X1"）
        strncpy(g_axis[existing_idx].axis_name, axis_name,
                sizeof(g_axis[existing_idx].axis_name) - 1);
        g_axis[existing_idx].axis_name[sizeof(g_axis[existing_idx].axis_name) - 1] = '\0';
        return 0;
    }

    // ===== 新分配分支：检查房间号是否用尽 =====
    if(g_allocated_axis_count >= AXIS_NUM) {
        printf("[SMC_API] 轴房间号已满(%d/%d)，无法继续分配 '%s'！\n",
               g_allocated_axis_count, AXIS_NUM, axis_name);
        return -1;
    }

    // 自动分配房间号
    int axis_idx = g_allocated_axis_count;

    // 写入轴名
    strncpy(g_axis[axis_idx].axis_name, axis_name, sizeof(g_axis[axis_idx].axis_name) - 1);
    g_axis[axis_idx].axis_name[sizeof(g_axis[axis_idx].axis_name) - 1] = '\0';

    // 注册字母 → 底层索引映射
    g_axis_map[letter - 'A'] = axis_idx;

    // 拓扑赋值
    if(is_dual_drive){
        g_axis[axis_idx].slave_ids[0] = master_id;
        g_axis[axis_idx].slave_ids[1] = slave_id;
        g_axis[axis_idx].slave_count  = 2;
        printf("[SMC_API] 配置 '%s' → 房间[%d], 字母 '%c', 双驱 主ID:%d 从ID:%d\n",
               axis_name, axis_idx, letter, master_id, slave_id);
    } else {
        g_axis[axis_idx].slave_ids[0] = master_id;
        g_axis[axis_idx].slave_count  = 1;
        printf("[SMC_API] 配置 '%s' → 房间[%d], 字母 '%c', 单驱 ID:%d\n",
               axis_name, axis_idx, letter, master_id);
    }

    g_allocated_axis_count++;
    return 0;
}

int SMC_ConfigSoftLimit(char axis_letter, int enable, double neg_limit_mm, double pos_limit_mm){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置软限位！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    g_axis[idx].enable_soft_limit = enable;
    g_axis[idx].soft_limit_neg    = neg_limit_mm;
    g_axis[idx].soft_limit_pos    = pos_limit_mm;
    return 0;
}

int SMC_ConfigGantrySyncAlarm(char axis_letter, int enable, int32_t tolerance_pulse, int32_t max_error_pulse, int time_ms){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置龙门同步报警！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    g_axis[idx].enable_sync_alarm    = enable;
    g_axis[idx].sync_tolerance_pulse = tolerance_pulse;
    g_axis[idx].sync_max_err_pulse   = max_error_pulse;
    g_axis[idx].sync_err_time_ms     = time_ms;
    g_axis[idx]._current_sync_timer  = 0;
    return 0;
}

void SMC_ConfigPulsePerUnit(char axis_letter, double pulse_per_unit){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设置脉冲当量！\n", toupper((unsigned char)axis_letter));
        return;
    }
    g_axis[idx].pulse_per_unit = pulse_per_unit;
}

// @Thread-Safety: Requires atomic operations or lock-free design.
// 写入 g_axis[] 全局数组，仅允许在初始化阶段（加工停止时）调用。
int SMC_ConfigAxisDynamics(char axis_letter, int type, double max_v, double max_a, double max_d, double equivalent_radius){
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API ERROR] 轴 '%c' 未配置！\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    if(g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API ERROR] 系统运行中（队列未空或插补器未静止），禁止修改轴动力学参数！\n");
        return -1;
    }
    if(max_v <= 0.0 || max_a <= 0.0 || max_d <= 0.0) {
        printf("[SMC_API ERROR] %s 动力学参数必须大于0！Vmax=%.1f, Amax=%.1f, Dmax=%.1f\n",
               g_axis[idx].axis_name, max_v, max_a, max_d);
        return -1;
    }
    if(type == 1 && equivalent_radius <= 0.0) {
        printf("[SMC_API ERROR] %s 为旋转轴，equivalent_radius 必须大于0！\n",
               g_axis[idx].axis_name);
        return -1;
    }
    g_axis[idx].axis_type          = type;
    g_axis[idx].equivalent_radius  = equivalent_radius;
    __sync_synchronize();
    g_axis[idx].max_speed = max_v;
    g_axis[idx].max_acc   = max_a;
    g_axis[idx].max_dec   = max_d;
    __sync_synchronize();
    printf("[SMC_API] %s 动力学: type=%d, Vmax=%.1f, Amax=%.1f, Dmax=%.1f, eq_radius=%.2f\n",
           g_axis[idx].axis_name, type, max_v, max_a, max_d, equivalent_radius);
    return 0;
}

// @Thread-Safety: Requires atomic operations or lock-free design.
int SMC_ConfigPlannerParams(double tolerance, double max_centripetal_acc){
    if(g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API ERROR] 系统运行中（队列未空或插补器未静止），禁止修改规划器参数！\n");
        return -1;
    }
    if(tolerance <= 0.0 || max_centripetal_acc <= 0.0) {
        printf("[SMC_API ERROR] 规划器参数必须大于0！tolerance=%.4f, centripetal_acc=%.1f\n",
               tolerance, max_centripetal_acc);
        return -1;
    }
    __sync_synchronize();
    g_planner_config.corner_tolerance    = tolerance;
    g_planner_config.max_centripetal_acc = max_centripetal_acc;
    __sync_synchronize();
    printf("[SMC_API] 规划器参数: tolerance=%.4f, centripetal_acc=%.1f\n",
           tolerance, max_centripetal_acc);
    return 0;
}

// @Context: Non-RealTime Background Thread (初始化阶段调用)
// @Thread-Safety: 写入运动学全局偏置，仅允许停机时配置
void SMC_ConfigKinematicsOffset(double tool_len, double pivot_x, double pivot_y, double pivot_z) {
    g_kin_config.tool_offset[0] = 0.0;
    g_kin_config.tool_offset[1] = 0.0;
    g_kin_config.tool_offset[2] = tool_len; // 沿 -Z 方向
    g_kin_config.pivot_offset[0] = pivot_x;
    g_kin_config.pivot_offset[1] = pivot_y;
    g_kin_config.pivot_offset[2] = pivot_z;
    printf("[SMC_API] 运动学偏置: tool_len=%.2f mm, pivot=(%.2f, %.2f, %.2f) mm\n",
           tool_len, pivot_x, pivot_y, pivot_z);
}

// @Context: Non-RealTime Background Thread (初始化阶段调用)
// @Thread-Safety: 写入运动学全局配置，仅允许停机时配置
void SMC_ConfigKinematics(int type,
                          int r1_idx, int r1_axis,
                          int r2_idx, int r2_axis,
                          double tool_off[3], double pivot_off[3]) {
    g_kin_config.type      = type;
    g_kin_config.rot_1_idx = r1_idx;
    g_kin_config.rot_1_axis = r1_axis;
    g_kin_config.rot_2_idx = r2_idx;
    g_kin_config.rot_2_axis = r2_axis;
    memcpy(g_kin_config.tool_offset, tool_off, sizeof(double) * 3);
    memcpy(g_kin_config.pivot_offset, pivot_off, sizeof(double) * 3);
    printf("[SMC_API] 运动学构型: type=%d, R1=[idx=%d,axis=%d], R2=[idx=%d,axis=%d]\n",
           type, r1_idx, r1_axis, r2_idx, r2_axis);
}

// ================== 坐标与状态 ==================
double SMC_GetLogicalPos(char axis_letter) {
    if(axis_letter == SMC_AXIS_ALL) return 0.0;
    int idx = axis_lookup(axis_letter);
    if(idx < 0) return 0.0;
    return g_coord_mgr.current_logical_pos[idx];
}

int SMC_IsParserRunning() { return g_parser_ctrl.is_running; }
int SMC_IsMotionDone() { return is_trajectory_finished(); }
int SMC_GetQueueCount() {
    // 非实时调用方 (UI / 上层管理线程): acquire 读两侧游标,
    // 与生产者 release 写 write_head、RT 消费者 release 写 read_tail 配对,
    // 返回一致性的队列长度快照。
    int h = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_acquire);
    int t = atomic_load_explicit(&g_cmd_queue.read_tail,  memory_order_acquire);
    return (h - t + QUEUE_SIZE) % QUEUE_SIZE;
}

// @Thread-Safety: 只读 g_axis_map[]，初始化后不变
int SMC_IsAxisConfigured(char axis_letter) {
    char c = toupper((unsigned char)axis_letter);
    if(c < 'A' || c > 'Z') return 0;
    return (g_axis_map[c - 'A'] >= 0) ? 1 : 0;
}

// @Thread-Safety: 使用 atomic_load_explicit 读取原子变量，
// 普通 int 变量 (is_paused/is_running) 为 UI 显示容许短暂撕裂，不会触发总线错误
void SMC_GetSystemStatusStr(char* out_str, int max_len) {
    if(out_str == NULL || max_len <= 0) return;
    if(atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) == 1) {
        snprintf(out_str, max_len, "ALARM");
    } else if(g_parser_ctrl.is_paused == 1) {
        snprintf(out_str, max_len, "HOLD");
    } else if(g_parser_ctrl.is_running == 1 ||
              atomic_load_explicit(&g_interpolator.is_moving, memory_order_acquire) == 1) {
        snprintf(out_str, max_len, "RUN");
    } else {
        snprintf(out_str, max_len, "IDLE");
    }
}

// ================== 运动控制 ==================
void SMC_SetZero(char axis_letter) {
    if(axis_letter == SMC_AXIS_ALL) { api_set_zero(AXIS_ALL); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法设零！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_set_zero(idx);
}

void SMC_MoveRelative(char axis_letter, double distance, double speed) {
    if(axis_letter == SMC_AXIS_ALL) { api_move_relative(AXIS_ALL, distance, speed); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法点动！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_move_relative(idx, distance, speed);
}

void SMC_GoZero(char axis_letter, double speed) {
    if(axis_letter == SMC_AXIS_ALL) { api_go_zero(AXIS_ALL, speed); return; }
    int idx = axis_lookup(axis_letter);
    if(idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法回零！\n", toupper((unsigned char)axis_letter));
        return;
    }
    api_go_zero(idx, speed);
}

// ================== G代码加工 ==================
int SMC_RunGCodeFile(const char *filepath) {
    if (g_parser_ctrl.is_running) return -1;
    strncpy(g_parser_ctrl.filepath, filepath, sizeof(g_parser_ctrl.filepath)-1);
    g_parser_ctrl.abort_request = 0;
    g_parser_ctrl.is_paused = 0;
    g_parser_ctrl.is_running = 1;
    return 0;
}

void SMC_AbortProcessing() { g_parser_ctrl.abort_request = 1; }

void SMC_PauseProcessing(){
    g_parser_ctrl.is_paused = 1;
    api_motion_pause();
}

void SMC_ResumeProcessing(){
    g_parser_ctrl.is_paused = 0;
    api_motion_resume();
}

// ================== 仿真驱动器 API (仅 sim 模式) ==================

int SMC_InjectAxisFault(char axis_letter, int slave_subidx)
{
    if (!g_sim_mode) {
        printf("[SMC_API] InjectAxisFault 仅 sim 模式有效\n");
        return -2;
    }
    int idx = axis_lookup(axis_letter);
    if (idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置，无法注入故障\n",
               toupper((unsigned char)axis_letter));
        return -1;
    }
    printf("[SMC_API] 注入故障: 轴 %c motor %d\n",
           toupper((unsigned char)axis_letter), slave_subidx);
    return sim_drive_inject_fault(idx, slave_subidx);
}

int SMC_ConfigSimDynamics(char axis_letter, double alpha)
{
    if (!g_sim_mode) {
        printf("[SMC_API] ConfigSimDynamics 仅 sim 模式有效\n");
        return -2;
    }
    int idx = axis_lookup(axis_letter);
    if (idx < 0) {
        printf("[SMC_API] 轴 '%c' 未配置\n", toupper((unsigned char)axis_letter));
        return -1;
    }
    if (!(alpha > 0.0) || !(alpha < 1.0)) {
        printf("[SMC_API] alpha=%.3f 越界 (0,1)\n", alpha);
        return -1;
    }
    return sim_drive_config_alpha(idx, alpha);
}

// ================== 辅助状态查询 API ==================
// 数据源: g_interpolator.*_rt 镜像字段 (RT 单写者, 此处单读者)
// 反映"RT 实际执行到的"模态, 而非 parser 解析到的"将来"模态
// M 段等待期间 mcode_wait_timer 走完前, 镜像保持上一次的值, 这是正确语义

// @Context: Non-RealTime (HMI/CAM 通过 RPC 调用)
// @Thread-Safety: int/double 对齐天然原子; 加 acquire 显式 happens-before 标注
int SMC_GetSpindleState(int *mode, double *rpm)
{
    if (!g_all_axis_op_ready) return -1;
    if (mode) *mode = g_interpolator.spindle_mode_rt;
    if (rpm)  *rpm  = g_interpolator.spindle_rpm_rt;
    return 0;
}

int SMC_GetCoolantState(int *state)
{
    if (!g_all_axis_op_ready) return -1;
    if (state) *state = g_interpolator.coolant_state_rt;
    return 0;
}

int SMC_GetCurrentTool(int *tool_id)
{
    if (!g_all_axis_op_ready) return -1;
    if (tool_id) *tool_id = g_interpolator.current_tool_id_rt;
    return 0;
}

// @Thread-Safety: 单写者 (此 API), 单读者 (parser M1 分支)
int SMC_SetOptionalStopEnable(int enable)
{
    g_optional_stop_enabled = enable ? 1 : 0;
    printf("[SMC_API] M1 可选停开关 -> %d\n", g_optional_stop_enabled);
    return 0;
}
