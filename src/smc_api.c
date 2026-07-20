#include "smc_api.h"
#include "global_def.h"
#include "ecat_core.h"
#include "axis_ctrl.h"
#include "gcode_parser.h"
#include "kinematics.h"
#include "bspline_engine.h"
#include "trace_logger.h"
#include "sim_drive.h"
#include "preview_streamer.h"   /* P0-b v2: PreviewStreamer_GetWriteSeq */
#include "event_logger.h"       /* P1-b: EventLogger_Push */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <string.h>
#include <pthread.h>            /* P0-1 fix: RPC 触发 homing 的 Non-RT worker 线程 */

// 底层轴房间号自动分配计数器
static int g_allocated_axis_count = 0;

// P0-3: Safe Z Lift 配置 (非RT init 阶段单写者)
// 默认 enabled=0, z_axis_idx=-1 (未配置), 调 SMC_ConfigSafeLiftZ 后激活
SafeLiftConfig_t g_safe_lift_cfg = { 0, -1, 50.0, 20.0, 0 };

// P0-1: Homing 配置 (非RT init 阶段单写者)
// 默认 enabled=0, order_count=0; 默认 timeout_ms=10000 / home_switch=-1 在 SMC_ConfigHoming 设
HomingGlobalConfig_t g_homing_cfg = {0};

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

    // P2-A: 实时倍率字段初始化 (必须在 RT 线程启动前完成)
    // g_interpolator 是 BSS 零初始化全局, override_ratio 必须 1.0 (非零) 才能
    //   保证未调 SMC_SetOverride 时正常加工 (feed=100% 默认).
    // mode_flags 默认 0: single_block=0 / dry_run=0 / override_persist=0 (M30 重置生效).
    g_interpolator.feed_override_ratio    = 1.0;
    g_interpolator.rapid_override_ratio   = 1.0;
    g_interpolator.spindle_override_ratio = 1.0;
    g_interpolator.mode_flags             = 0;
    g_interpolator.current_motion_type_rt = 0;

    // P0-3 SafeLift: RT 字段显式清零 (memset 已零初始化, 此处注释+可读性)
    // enabled 默认 0: 必须调 SMC_ConfigSafeLiftZ 才激活
    atomic_store_explicit(&g_interpolator.safe_lift_pending_req, 0,
                          memory_order_release);
    atomic_store_explicit(&g_interpolator.safe_lift_cancel_req, 0,
                          memory_order_release);
    g_interpolator.safe_lift_state       = 0;  /* IDLE */
    g_interpolator.safe_lift_source      = 0;
    g_interpolator.safe_lift_start_z     = 0.0;
    g_interpolator.safe_lift_pulse_step  = 0.0;

    // P0-1 Homing/JOG: RT 字段显式清零 (memset 已零初始化, 此处可读性)
    atomic_store_explicit(&g_interpolator.homing_pending_req, 0, memory_order_release);
    atomic_store_explicit(&g_interpolator.homing_cancel_req, 0, memory_order_release);
    g_interpolator.homing_state          = 0;  /* IDLE */
    g_interpolator.homing_axis_idx       = -1;
    g_interpolator.homing_source         = 0;
    g_interpolator.homing_method_in_use  = 0;
    atomic_store_explicit(&g_interpolator.jog_active_req, 0, memory_order_release);
    g_interpolator.jog_axis_idx          = -1;
    g_interpolator.jog_direction         = 0;
    g_interpolator.jog_speed_mm_s        = 0.0;
    g_interpolator.jog_step_mm           = 0.0;

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

    // === 关键修复: 必须先让 RT 线程完全停止, 再释放 SimEngine / TraceLogger 资源 ===
    // 原顺序: 先 BSpline_StopThread + sim_engine_finish()(free bufs) + TraceLogger_StopThread,
    //         再 dorun=2 等 RT 停。但 RT 线程在 dorun=2 期间仍每周期调用
    //         sim_engine_push() 写 bufs[idx][count]; 此时 bufs 已被 free ->
    //         use-after-free -> SIGSEGV (崩溃点: ecat_thread_rt -> sim_engine.h:214)。
    // 故调整为: 先请求 RT 优雅下电并等待其真正退出(dorun==0), 再 tear down 采集器。

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

    // 现在 RT 已停止, 不会再 push SimEngine / TraceLogger -> 安全 tear down
    // 先停止 B-Spline 平滑线程 (排空队列后退出)
    BSpline_StopThread();

    // 仿真模式: 停止双缓冲轨迹采集器 (排空残余数据后关闭文件 + free)
    if (g_sim_mode) {
        sim_engine_finish();
    }

    // 停止轨迹探针落盘线程 (排空残余数据后退出)
    TraceLogger_StopThread();

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

void SMC_AbortProcessing() {
    g_parser_ctrl.abort_request = 1;
    EventLogger_Push(SEVERITY_WARN, SOURCE_PARSER, 0x0034, 0,
                     "program abort requested by UI");
}

void SMC_PauseProcessing(){
    g_parser_ctrl.is_paused = 1;
    api_motion_pause();
}

void SMC_ResumeProcessing(){
    g_parser_ctrl.is_paused = 0;
    api_motion_resume();
}

// ================== P0-b v2: LoadProgram / RunLoadedProgram ==================
int SMC_LoadProgram(const char *filepath) {
    if (filepath == NULL || filepath[0] == '\0') return -2;
    if (g_parser_ctrl.is_running) return -1;

    strncpy(g_parser_ctrl.filepath, filepath, sizeof(g_parser_ctrl.filepath) - 1);
    g_parser_ctrl.filepath[sizeof(g_parser_ctrl.filepath) - 1] = '\0';
    g_parser_ctrl.abort_request = 0;
    g_parser_ctrl.is_paused = 0;
    g_parser_ctrl.program_mode = PROGRAM_MODE_PREVIEW;   // P0-b v2: preview 模式
    g_parser_ctrl.is_running = 1;                         // 触发 parser_thread
    return 0;
}

int SMC_RunLoadedProgram(void) {
    if (g_parser_ctrl.is_running) return -2;
    if (!atomic_load_explicit(&g_program_load_done, memory_order_acquire)) return -1;
    if (g_parser_ctrl.filepath[0] == '\0') return -2;

    // 切回 RUN 模式, 同 filepath 再跑一遍
    g_parser_ctrl.program_mode = PROGRAM_MODE_RUN;
    atomic_store_explicit(&g_program_load_done, 0, memory_order_release);
    g_parser_ctrl.abort_request = 0;
    g_parser_ctrl.is_paused = 0;
    g_parser_ctrl.is_running = 1;
    return 0;
}

int SMC_GetProgramStructure(SmcGetProgramStructureRes *out) {
    if (out == NULL) return -1;
    memset(out, 0, sizeof(*out));

    int is_running = g_parser_ctrl.is_running;
    int load_done = atomic_load_explicit(&g_program_load_done, memory_order_acquire);
    uint64_t total_segs = PreviewStreamer_GetWriteSeq();

    // 状态判定 (UI 据此显示 "未加载/loading/loaded/running/done")
    if (is_running && g_parser_ctrl.program_mode == PROGRAM_MODE_PREVIEW) {
        out->is_loaded = 4;   // loading preview
    } else if (is_running && g_parser_ctrl.program_mode == PROGRAM_MODE_RUN) {
        out->is_loaded = 2;   // running
    } else if (load_done) {
        out->is_loaded = 1;   // loaded (待 run)
    } else if (total_segs > 0) {
        out->is_loaded = 3;   // done (run 完成)
    } else {
        out->is_loaded = 0;   // 未加载
    }

    // 文件路径
    strncpy(out->filepath, g_parser_ctrl.filepath, SMC_FILEPATH_MAX_LEN - 1);

    // 元数据 (parser 完成时 g_current_program 已 free, 从 cache 全局读)
    out->total_lines   = g_program_total_lines;
    out->num_o_labels   = g_program_num_o_labels;
    out->num_n_labels   = g_program_num_n_labels;
    out->total_segments = (int32_t)total_segs;
    out->first_seg_id   = g_program_first_seg_id;
    out->last_seg_id    = g_program_last_seg_id;
    out->estimated_time_ms = g_program_total_time_ms;
    for (int i = 0; i < AXIS_NUM; i++) {
        out->bbox_min[i] = g_program_bbox_min[i];
        out->bbox_max[i] = g_program_bbox_max[i];
    }

    out->ret_code = (total_segs > 0 || load_done) ? 0 : -1;
    return 0;
}

// ================== P1-b: ClearAlarm ==================
int SMC_ClearAlarm(void) {
    /* 安全闸: parser 正在跑时拒绝清 alarm, 防 RT 清队列时撞刀。
     * 用户应先 AbortProcessing 停程序, 再 ClearAlarm。 */
    if (g_parser_ctrl.is_running) {
        EventLogger_Push(SEVERITY_WARN, SOURCE_MANUAL, 0x0040, -1,
                         "ClearAlarm rejected: parser running, AbortProcessing first");
        return -1;
    }
    if (!g_all_axis_op_ready) {
        EventLogger_Push(SEVERITY_WARN, SOURCE_MANUAL, 0x0040, -2,
                         "ClearAlarm rejected: axes not op-ready");
        return -2;
    }
    EventLogger_Push(SEVERITY_INFO, SOURCE_MANUAL, 0x0040, 0,
                     "ClearAlarm requested by UI");
    api_alarm_reset();   /* axis_ctrl.c:756, 设 alarm_reset_request 让 RT 异步清 */
    return 0;
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

// ================== 激光切割子系统 API (Phase A) ==================
// @Thread-Safety: 主线程 init 阶段单写者; 必须在 SMC_InitAndStart 之前调
// RT 线程只读 g_laser_cfg, 所以 init 后再修改无效 (但不报错)
// sim 模式与硬件模式都允许配置 — sim 下 RT 路径会安全跳过 PDO 输出

int SMC_ConfigLaserIO(int do_slave_id, int ao_slave_id, int di_slave_id)
{
    if (do_slave_id < -1 || ao_slave_id < -1 || di_slave_id < -1) return -1;
    g_laser_cfg.do_slave_id = do_slave_id;
    g_laser_cfg.ao_slave_id = ao_slave_id;
    g_laser_cfg.di_slave_id = di_slave_id;
    return 0;
}

static uint8_t laser_validate_bit(uint8_t b)
{
    return (b > 15) ? 0 : b;  // 越界钳到 0
}

int SMC_ConfigLaserDOBits(uint8_t b_enable, uint8_t b_shutter,
                          uint8_t b_gas_n2, uint8_t b_gas_o2, uint8_t b_gas_air,
                          uint8_t b_alarm_lamp)
{
    g_laser_cfg.bit_laser_enable = laser_validate_bit(b_enable);
    g_laser_cfg.bit_laser_shutter= laser_validate_bit(b_shutter);
    g_laser_cfg.bit_gas_n2       = laser_validate_bit(b_gas_n2);
    g_laser_cfg.bit_gas_o2       = laser_validate_bit(b_gas_o2);
    g_laser_cfg.bit_gas_air      = laser_validate_bit(b_gas_air);
    g_laser_cfg.bit_alarm_lamp   = laser_validate_bit(b_alarm_lamp);
    return 0;
}

int SMC_ConfigLaserDIBits(uint8_t b_door, uint8_t b_estop, uint8_t b_laser_alm,
                          uint8_t b_water_t, uint8_t b_water_f, uint8_t b_gas_p)
{
    g_laser_cfg.bit_di_door       = laser_validate_bit(b_door);
    g_laser_cfg.bit_di_estop_soft = laser_validate_bit(b_estop);
    g_laser_cfg.bit_di_laser_alm  = laser_validate_bit(b_laser_alm);
    g_laser_cfg.bit_di_water_temp = laser_validate_bit(b_water_t);
    g_laser_cfg.bit_di_water_flow = laser_validate_bit(b_water_f);
    g_laser_cfg.bit_di_gas_press  = laser_validate_bit(b_gas_p);
    return 0;
}

int SMC_ConfigLaserAOChannels(uint8_t ch_power, uint8_t ch_freq)
{
    g_laser_cfg.ch_ao_power = ch_power;
    g_laser_cfg.ch_ao_freq  = ch_freq;
    return 0;
}

int SMC_ConfigLaserRange(double power_max_w, double freq_max_hz, double power_min_w)
{
    if (power_max_w <= 0.0 || freq_max_hz <= 0.0 || power_min_w < 0.0) {
        printf("[SMC_API] ConfigLaserRange 量程越界 (power_max=%.1f freq_max=%.1f min=%.1f)\n",
               power_max_w, freq_max_hz, power_min_w);
        return -1;
    }
    g_laser_cfg.power_max_w = power_max_w;
    g_laser_cfg.freq_max_hz = freq_max_hz;
    g_laser_cfg.power_min_w = power_min_w;
    return 0;
}

// Phase B1: 功率-速度耦合配置
// @Thread-Safety: 主线程 init 阶段单写者; RT 线程只读 g_laser_cfg.coupling_mode/v_thresh
int SMC_ConfigLaserCoupling(int mode, double v_thresh_mm_s)
{
    if (mode != 0 && mode != 1) {
        printf("[SMC_API] ConfigLaserCoupling mode=%d 越界 (0 或 1)\n", mode);
        return -1;
    }
    if (v_thresh_mm_s < 0.0) {
        printf("[SMC_API] ConfigLaserCoupling v_thresh=%.2f 不能为负\n", v_thresh_mm_s);
        return -1;
    }
    atomic_store_explicit(&g_laser_cfg.coupling_mode, mode, memory_order_release);
    atomic_store_explicit(&g_laser_cfg.v_thresh_mm_s, v_thresh_mm_s, memory_order_release);
    return 0;
}

// Phase B1: 配置功率-速度耦合表
// 约束: count ∈ [1, LASER_COUPLE_TABLE_MAX], v_mm_s 单调不减, ratio ∈ [0,1]
int SMC_ConfigLaserCoupleTable(const LaserCouplePoint_t *points, int count)
{
    if (points == NULL || count < 1 || count > LASER_COUPLE_TABLE_MAX) {
        printf("[SMC_API] ConfigLaserCoupleTable 参数越界 (count=%d, max=%d)\n",
               count, LASER_COUPLE_TABLE_MAX);
        return -1;
    }
    // 校验单调不减 + ratio 范围
    for (int i = 0; i < count; i++) {
        if (points[i].ratio < 0.0 || points[i].ratio > 1.0) {
            printf("[SMC_API] ConfigLaserCoupleTable 点 %d ratio=%.3f 越界 [0,1]\n",
                   i, points[i].ratio);
            return -1;
        }
        if (i > 0 && points[i].v_mm_s < points[i-1].v_mm_s - 1e-6) {
            printf("[SMC_API] ConfigLaserCoupleTable v_mm_s 必须单调不减 (点 %d %.2f < 点 %d %.2f)\n",
                   i, points[i].v_mm_s, i-1, points[i-1].v_mm_s);
            return -1;
        }
    }
    for (int i = 0; i < count; i++) {
        g_laser_cfg.couple_table[i] = points[i];
    }
    g_laser_cfg.couple_table_len = count;
    return 0;
}

// ================== P0-3: Safe Z Lift ==================
// @Context: Non-RealTime (init 阶段, SMC_InitAndStart 之前调)
// @Thread-Safety: g_safe_lift_cfg 是 init 阶段单写者, RT 线程启动后只读
// 设计: 配置时一次性解析 z_letter → z_axis_idx, RT 路径零字符串操作
int SMC_ConfigSafeLiftZ(char z_letter, double safe_z_mm,
                        double lift_speed_mm_s, int auto_on_alarm)
{
    int idx = axis_lookup(z_letter);
    if (idx < 0) {
        printf("[SMC_API] ConfigSafeLiftZ 轴 '%c' 未配置\n",
               toupper((unsigned char)z_letter));
        return -1;
    }
    if (g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API] ConfigSafeLiftZ 系统运行中, 禁止修改\n");
        return -1;
    }
    // 旋转轴 (axis_type=1) 抬升语义无效, 拒绝
    if (g_axis[idx].axis_type != 0) {
        printf("[SMC_API] ConfigSafeLiftZ %s 是旋转轴, 仅线性 Z 轴合法\n",
               g_axis[idx].axis_name);
        return -1;
    }
    if (lift_speed_mm_s <= 0.0) {
        printf("[SMC_API] ConfigSafeLiftZ lift_speed=%.2f 必须 > 0\n",
               lift_speed_mm_s);
        return -1;
    }
    // safe_z 超软限位正限: 拒绝 + 报警事件 (event 0x0044)
    if (g_axis[idx].enable_soft_limit && safe_z_mm > g_axis[idx].soft_limit_pos) {
        printf("[SMC_API] ConfigSafeLiftZ safe_z=%.2f 超过 Z 软限位正限 %.2f\n",
               safe_z_mm, g_axis[idx].soft_limit_pos);
        EventLogger_Push(SEVERITY_ALARM, SOURCE_MANUAL, 0x0044,
                         (int32_t)(safe_z_mm * 100),
                         "safe_lift config rejected (over soft limit)");
        return -2;
    }
    if (auto_on_alarm != 0 && auto_on_alarm != 1) {
        printf("[SMC_API] ConfigSafeLiftZ auto_on_alarm=%d 必须 0 或 1\n",
               auto_on_alarm);
        return -1;
    }

    g_safe_lift_cfg.z_axis_idx      = idx;
    g_safe_lift_cfg.safe_z_target_mm = safe_z_mm;
    g_safe_lift_cfg.lift_speed_mm_s  = lift_speed_mm_s;
    g_safe_lift_cfg.auto_on_alarm    = auto_on_alarm;
    __sync_synchronize();
    g_safe_lift_cfg.enabled          = 1;  // 最后置 1, RT 看到时其他字段已就绪

    printf("[SMC_API] SafeLift 配置: z=%s, target=%.2f mm, speed=%.1f mm/s, auto_on_alarm=%d\n",
           g_axis[idx].axis_name, safe_z_mm, lift_speed_mm_s, auto_on_alarm);
    return 0;
}

// @Context: Non-RealTime (HMI/CAM 通过 RPC 调用)
// @Thread-Safety: 仅 atomic_store 1 个标志位, RT 在 cycle 头 atomic_load 后清
// Idempotent: PENDING/RUNNING/DONE 中再调为 no-op, 返回 0
int SMC_SafeLiftZ(void)
{
    if (!g_safe_lift_cfg.enabled) {
        printf("[SMC_API] SafeLiftZ 未配置, 调 SMC_ConfigSafeLiftZ 先\n");
        return -1;
    }
    atomic_store_explicit(&g_interpolator.safe_lift_pending_req, 1,
                          memory_order_release);
    return 0;
}

// @Context: Non-RealTime
// 仅 PENDING/DONE 可取消, RUNNING 拒绝 (避免 Z 卡在工件与安全高度之间)
// 返回: 0=已提交取消, -1=未配置或正在抬升拒绝
int SMC_CancelSafeLiftZ(void)
{
    if (!g_safe_lift_cfg.enabled) return -1;
    int st = g_interpolator.safe_lift_state;
    if (st == 2) {
        // RUNNING: 拒绝, 避免撞刀
        return -1;
    }
    atomic_store_explicit(&g_interpolator.safe_lift_cancel_req, 1,
                          memory_order_release);
    return 0;
}

// @Context: Non-RealTime (HMI 用, 60Hz 安全)
// @Thread-Safety: int/double 对齐天然原子; acquire 显式 happens-before 标注
int SMC_GetSafeLiftState(int *out_state, double *out_progress_mm)
{
    if (!g_safe_lift_cfg.enabled) {
        if (out_state)       *out_state       = 0;
        if (out_progress_mm) *out_progress_mm = 0.0;
        return -1;
    }
    int st = g_interpolator.safe_lift_state;
    if (out_state) *out_state = st;
    if (out_progress_mm) {
        int z = g_safe_lift_cfg.z_axis_idx;
        if (st == 2 || st == 3) {
            *out_progress_mm = g_axis[z].current_cmd_pos
                               - g_interpolator.safe_lift_start_z;
        } else {
            *out_progress_mm = 0.0;
        }
    }
    return 0;
}

// ================== P0-1 Homing: 工业级回零 ==================
// @Context: Non-RealTime (init 阶段, SMC_InitAndStart 之前调)
int SMC_ConfigHoming(char axis_letter, int method, double search_speed,
                     double creep_speed, int direction, int timeout_ms)
{
    int idx = axis_lookup(axis_letter);
    if (idx < 0) {
        printf("[SMC_API] ConfigHoming 轴 '%c' 未配置\n",
               toupper((unsigned char)axis_letter));
        return -1;
    }
    if (g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API] ConfigHoming 系统运行中, 禁止修改\n");
        return -1;
    }
    // v1 仅支持 method 35; method 1-19 需硬件 home switch (未接入)
    if (method != 35) {
        printf("[SMC_API] ConfigHoming method=%d v1 不支持 (仅 35; 1-19 需 home switch)\n",
               method);
        EventLogger_Push(SEVERITY_WARN, SOURCE_DRIVE, 0x0009, method,
                         "homing method not supported in v1");
        return -3;
    }
    if (direction != 1 && direction != -1) {
        printf("[SMC_API] ConfigHoming direction=%d 必须 +1 或 -1\n", direction);
        return -1;
    }
    if (timeout_ms < 1000 || timeout_ms > 60000) {
        printf("[SMC_API] ConfigHoming timeout_ms=%d 越界 [1000, 60000]\n", timeout_ms);
        return -1;
    }

    HomingAxisCfg_t *cfg = &g_homing_cfg.axis[idx];
    cfg->enabled            = 1;
    cfg->method             = method;
    cfg->search_speed_mm_s  = (search_speed > 0.0) ? search_speed : 10.0;
    cfg->creep_speed_mm_s   = (creep_speed > 0.0) ? creep_speed : 1.0;
    cfg->direction          = direction;
    cfg->timeout_ms         = timeout_ms;
    cfg->home_switch_pdo_bit = -1;  // v2 用

    printf("[SMC_API] ConfigHoming %s: method=%d, timeout=%d ms\n",
           g_axis[idx].axis_name, method, timeout_ms);
    return 0;
}

int SMC_ConfigHomingAll(const char *order_letters)
{
    if (order_letters == NULL || order_letters[0] == '\0') {
        printf("[SMC_API] ConfigHomingAll order_letters 为空\n");
        return -1;
    }
    if (g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API] ConfigHomingAll 系统运行中, 禁止修改\n");
        return -1;
    }

    g_homing_cfg.order_count = 0;
    for (int i = 0; order_letters[i] != '\0' && g_homing_cfg.order_count < AXIS_NUM; i++) {
        int idx = axis_lookup(order_letters[i]);
        if (idx < 0) {
            printf("[SMC_API] ConfigHomingAll 轴 '%c' 未配置\n",
                   toupper((unsigned char)order_letters[i]));
            return -2;
        }
        // 默认每轴 method 35, timeout 10000, direction +1
        if (!g_homing_cfg.axis[idx].enabled) {
            HomingAxisCfg_t *cfg = &g_homing_cfg.axis[idx];
            cfg->enabled            = 1;
            cfg->method             = 35;
            cfg->search_speed_mm_s  = 10.0;
            cfg->creep_speed_mm_s   = 1.0;
            cfg->direction          = 1;
            cfg->timeout_ms         = 10000;
            cfg->home_switch_pdo_bit = -1;
        }
        g_homing_cfg.order[g_homing_cfg.order_count++] = idx;
    }

    __sync_synchronize();
    g_homing_cfg.enabled = 1;
    printf("[SMC_API] ConfigHomingAll: %d 轴顺序回零\n", g_homing_cfg.order_count);
    return 0;
}

// 内部 helper: 互斥检查 (SafeLift / Homing / JOG 三者)
static int homing_mutex_ok(void)
{
    if (g_interpolator.safe_lift_state != 0) return 0;
    if (atomic_load_explicit(&g_interpolator.jog_active_req, memory_order_acquire) != 0) return 0;
    return 1;
}

// v2 (2026-07-20): 首周期锚定检查. 防止 Non-RT (SMC_HomeAxis/HomeAll) 在 RT 首周期
// 锚定完成前抢跑 - 否则会读到 home_offset=0 + homing_shift=0 的旧值, homing 重新锚定时
// homing_shift = cur_pulse - 0 = cur_pulse (而非 cur_pulse - home_offset), 错位.
// acquire 读 home_offset_anchored, RT 端 release 写保证 home_offset/homing_shift 已就绪.
static int axis_anchored_ok(void)
{
    for (int i = 0; i < AXIS_NUM; i++) {
        if (g_axis[i].slave_count > 0
            && atomic_load_explicit(&g_axis[i].home_offset_anchored,
                                    memory_order_acquire) == 0) {
            return 0;
        }
    }
    return 1;
}

// ================== P0-1 fix: RPC homing worker ==================
// @Context: Non-RealTime worker thread
// 背景: SMC_HomeAxis/HomeAll 仅能设 pending_req (RT 消费 -> PENDING -> RUNNING),
//       但 RUNNING(2) 的实际 SDO/轮询/DONE 收尾必须在 Non-RT 由 axis_homing 完成.
//       G28 parser 路径由 parser_thread 充当此 worker; RPC 路径原本【缺】worker,
//       导致 homing 永久卡在 RUNNING(2) + time_scale=0 -> 机床 DoS.
// 方案: RPC 触发时 spawn 一个 detached worker 调 axis_homing_multi (单轴 count=1),
//       统一走 "RT 协同 PENDING->RUNNING + fresh 延迟 + axis_homing 收尾" 路径.
static pthread_t g_homing_worker_tid;
static _Atomic int g_homing_worker_busy = 0;   // 1=worker 运行中 (防重入)
static int         g_homing_worker_order[AXIS_NUM];
static int         g_homing_worker_count = 0;

static void *homing_worker_fn(void *arg)
{
    (void)arg;
    // axis_homing_multi 内部逐轴驱动 pending_req + 等 RT RUNNING + axis_homing 收尾,
    // 全部成功后恢复 time_scale; 任一轴 FAULT 则 all-or-nothing 回滚.
    axis_homing_multi(g_homing_worker_order, g_homing_worker_count, SOURCE_MANUAL);
    atomic_store_explicit(&g_homing_worker_busy, 0, memory_order_release);
    return NULL;
}

// 启动 homing worker. order/count 拷入静态缓冲 (worker 生命周期内有效).
// 返回 0=已 spawn, -1=上一个 worker 未结束 (拒绝重入).
static int spawn_homing_worker(const int *order, int count)
{
    if (count <= 0 || count > AXIS_NUM) return -1;
    int expected = 0;
    if (!atomic_compare_exchange_strong_explicit(&g_homing_worker_busy, &expected, 1,
                                                 memory_order_acq_rel,
                                                 memory_order_acquire)) {
        printf("[SMC_API] homing worker 忙, 拒绝重入\n");
        return -1;
    }
    for (int i = 0; i < count; i++) g_homing_worker_order[i] = order[i];
    g_homing_worker_count = count;
    if (pthread_create(&g_homing_worker_tid, NULL, homing_worker_fn, NULL) != 0) {
        atomic_store_explicit(&g_homing_worker_busy, 0, memory_order_release);
        printf("[SMC_API] homing worker pthread_create 失败\n");
        return -1;
    }
    pthread_detach(g_homing_worker_tid);
    return 0;
}

int SMC_HomeAxis(char axis_letter)
{
    if (!g_homing_cfg.enabled) {
        printf("[SMC_API] HomeAxis 未配置, 调 SMC_ConfigHomingAll 先\n");
        return -1;
    }
    int idx = axis_lookup(axis_letter);
    if (idx < 0 || !g_homing_cfg.axis[idx].enabled) {
        printf("[SMC_API] HomeAxis 轴 '%c' 未配置回零\n",
               toupper((unsigned char)axis_letter));
        return -1;
    }
    if (g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API] HomeAxis 系统运行中, 先 Abort\n");
        return -1;
    }
    if (!homing_mutex_ok()) {
        printf("[SMC_API] HomeAxis 与 SafeLift/JOG 冲突\n");
        return -1;
    }
    // v2 (2026-07-20): 防 Non-RT 抢跑 - 等首周期锚定完成, 否则拒绝.
    if (!axis_anchored_ok()) {
        printf("[SMC_API] HomeAxis 等待 RT 首周期锚定完成\n");
        return -1;
    }

    g_interpolator.homing_source        = SOURCE_MANUAL;
    g_interpolator.homing_method_in_use = g_homing_cfg.axis[idx].method;
    // P0-1 fix: spawn Non-RT worker 执行实际回零 (axis_homing_multi 内部会设
    //   pending_req 让 RT 协同 + axis_homing 收尾). 不再仅设 pending_req 后返回,
    //   否则无 worker -> 永久卡 RUNNING(2).
    int one[1] = { idx };
    if (spawn_homing_worker(one, 1) != 0) {
        printf("[SMC_API] HomeAxis worker 启动失败\n");
        return -1;
    }
    return 0;
}

int SMC_HomeAll(void)
{
    if (!g_homing_cfg.enabled || g_homing_cfg.order_count == 0) {
        printf("[SMC_API] HomeAll 未配置, 调 SMC_ConfigHomingAll 先\n");
        return -1;
    }
    if (g_parser_ctrl.is_running || !is_trajectory_finished()) {
        printf("[SMC_API] HomeAll 系统运行中, 先 Abort\n");
        return -1;
    }
    if (!homing_mutex_ok()) {
        printf("[SMC_API] HomeAll 与 SafeLift/JOG 冲突\n");
        return -1;
    }
    // v2 (2026-07-20): 防 Non-RT 抢跑 - 等首周期锚定完成, 否则拒绝.
    if (!axis_anchored_ok()) {
        printf("[SMC_API] HomeAll 等待 RT 首周期锚定完成\n");
        return -1;
    }

    g_interpolator.homing_source        = SOURCE_MANUAL;
    g_interpolator.homing_method_in_use = 35;
    // P0-1 fix: spawn Non-RT worker 按配置 order 顺序回零 (all-or-nothing 回滚在
    //   axis_homing_multi 内). 原仅设 pending_req -> 无 worker -> 永久卡 RUNNING(2).
    if (spawn_homing_worker(g_homing_cfg.order, g_homing_cfg.order_count) != 0) {
        printf("[SMC_API] HomeAll worker 启动失败\n");
        return -1;
    }
    return 0;
}

int SMC_CancelHoming(void)
{
    if (!g_homing_cfg.enabled) return -1;
    int st = g_interpolator.homing_state;
    if (st == 2) {
        // RUNNING: 拒绝 (axis_homing 内部轮询 cancel_req 才能安全退出)
        // v1 简化: 仅 PENDING/DONE 可 cancel; RUNNING 让 axis_homing 自己跑完
        return -1;
    }
    atomic_store_explicit(&g_interpolator.homing_cancel_req, 1, memory_order_release);
    return 0;
}

int SMC_GetHomingState(int *out_state, int *out_axis_idx, double *out_progress_pct)
{
    if (!g_homing_cfg.enabled) {
        if (out_state)        *out_state        = 0;
        if (out_axis_idx)     *out_axis_idx     = -1;
        if (out_progress_pct) *out_progress_pct = 0.0;
        return -1;
    }
    if (out_state)    *out_state    = g_interpolator.homing_state;
    if (out_axis_idx) *out_axis_idx = g_interpolator.homing_axis_idx;
    if (out_progress_pct) {
        if (g_interpolator.homing_axis_idx >= 0) {
            *out_progress_pct = (g_interpolator.homing_state == 3) ? 1.0 : 0.0;
        } else {
            // HomeAll 模式: 按 order_count 估算 (v1 简化)
            *out_progress_pct = (g_interpolator.homing_state == 3) ? 1.0 : 0.0;
        }
    }
    return 0;
}

// ================== P0-1 JOG: 手动定位 (method 35 前置) ==================
// @Context: Non-RealTime (HMI/CAM 通过 RPC 调用)
int SMC_JogStart(char axis_letter, int direction, double speed_mm_s)
{
    int idx = axis_lookup(axis_letter);
    if (idx < 0) {
        printf("[SMC_API] JogStart 轴 '%c' 未配置\n",
               toupper((unsigned char)axis_letter));
        return -1;
    }
    if (direction != 1 && direction != -1) return -2;
    if (speed_mm_s <= 0.0) {
        printf("[SMC_API] JogStart speed=%.2f 必须 > 0\n", speed_mm_s);
        return -1;
    }
    // 三功能互斥
    if (g_interpolator.homing_state != 0
        || g_interpolator.safe_lift_state != 0
        || g_parser_ctrl.is_running) {
        printf("[SMC_API] JogStart 与 Homing/SafeLift/parser 冲突\n");
        return -1;
    }
    // 检查当前是否已 JOG (单轴 JOG 模式 v1)
    if (atomic_load_explicit(&g_interpolator.jog_active_req, memory_order_acquire) != 0) {
        printf("[SMC_API] JogStart 已有 JOG 进行中 (axis=%d)\n",
               g_interpolator.jog_axis_idx);
        return -1;
    }

    g_interpolator.jog_axis_idx   = idx;
    g_interpolator.jog_direction  = direction;
    g_interpolator.jog_speed_mm_s = speed_mm_s;
    // step_mm 每 cycle 1ms 推进, 含方向
    g_interpolator.jog_step_mm    = speed_mm_s * (double)direction / 1000.0;
    __sync_synchronize();
    atomic_store_explicit(&g_interpolator.jog_active_req, 1, memory_order_release);

    EventLogger_Push(SEVERITY_INFO, SOURCE_MANUAL, 0x000A, idx, "jog start");
    return 0;
}

int SMC_JogStop(char axis_letter)
{
    if (axis_letter == SMC_AXIS_ALL) {
        atomic_store_explicit(&g_interpolator.jog_active_req, 0, memory_order_release);
        g_interpolator.jog_axis_idx = -1;
        // P0-1 hotfix (2026-07-20): 恢复 time_scale=1.0
        // 背景: RT JOG 子状态机 (ecat_core.c L778) ACTIVE 期间每 cycle 强制 time_scale=0
        //   冻结段消费 (while gate 屏蔽 motion queue). jog_active_req=0 后 RT 不再写
        //   time_scale=0, 但**也没人恢复 time_scale=1.0**, 导致后续段消费永久屏蔽
        //   (ms_budget=0). 现象: 首次增量 MoveRelative 工作, 切连续 JogStart 后切回增量
        //   坐标不变 (增量段入队但 RT 不消费).
        // 修复: 与 axis_homing_multi (axis_ctrl.c L1614) / SafeLift (ecat_core.c L487)
        //   同 pattern — Non-RT 显式恢复 time_scale. x86_64 8B 对齐 double 天然原子,
        //   下一 cycle RT 读到新值, 无 race.
        g_interpolator.time_scale = 1.0;
        return 0;
    }
    if (atomic_load_explicit(&g_interpolator.jog_active_req, memory_order_acquire) == 0) {
        return -1;
    }
    if (g_interpolator.jog_axis_idx != axis_lookup(axis_letter)) {
        return -1;  // 不是这个轴在 JOG
    }
    atomic_store_explicit(&g_interpolator.jog_active_req, 0, memory_order_release);
    g_interpolator.jog_axis_idx = -1;
    g_interpolator.time_scale = 1.0;  // P0-1 hotfix: 同 ALL 路径, 恢复段消费
    return 0;
}

double SMC_JogGetPos(char axis_letter)
{
    int idx = axis_lookup(axis_letter);
    if (idx < 0) return 0.0;
    return g_axis[idx].current_cmd_pos;
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

// 激光器完整状态查询 (镜像 RT 单写者字段 g_laser_rt + g_interpolator 派生 + 加工统计)
// @Thread-Safety: g_laser_rt / g_interpolator 均为 RT 线程单写者, 此处 acquire 读.
//   int/double 字段对齐天然原子; pierce_count (int32) / laser_on_time_ms (int64)
//   在 32-bit 平台可能撕裂, HMI 容忍 1ms 级滞后 (统计字段非安全关键).
//   emergency_kill 与 enable 可能瞬间撕裂, HMI 容忍 1ms 级滞后.
// 返回: 0=成功 (out 已 fill), -1=out 为空或激光未配置 (do_slave_id<0)
int SMC_GetLaserState(SmcGetLaserStateRes *out)
{
    if (!out) return -1;
    if (g_laser_cfg.do_slave_id < 0) { out->ret_code = -1; return -1; }

    // ---- 状态字段 (镜像 g_laser_rt) ----
    out->enable           = g_laser_rt.enable;
    out->shutter          = g_laser_rt.shutter;
    out->power_w          = g_laser_rt.power_w;
    out->freq_hz          = g_laser_rt.freq_hz;
    out->gas_select       = g_laser_rt.gas_select;
    out->interlock        = g_laser_rt.interlock_status;
    out->emergency_kill   = g_laser_rt.emergency_kill;
    out->P_base_w         = g_laser_rt.P_base_w;
    out->v_actual_mm_s    = g_laser_rt.v_actual_mm_s;
    out->coupling_mode_rt = g_laser_rt.coupling_mode_rt;

    // ---- 派生字段 (从 g_interpolator 计算) ----
    // is_piercing: G04 dwell (M64 段) 等待期间为 1, 供 HMI 显示 "穿孔中" 状态
    out->is_piercing = (g_interpolator.is_waiting_mcode &&
                        g_interpolator.current_mcode == 64) ? 1 : 0;
    // current_seg_flags: 当前段工艺标记 (lead_in / micro_joint), 段消费环同步
    out->current_seg_flags = g_interpolator.current_seg_flags_rt;

    // ---- 加工统计 (RT 累计, 跨程序不清零) ----
    out->pierce_count     = g_laser_rt.pierce_count;
    out->laser_on_time_ms = g_laser_rt.laser_on_time_ms;

    out->ret_code = 0;
    return 0;
}

// @Thread-Safety: 单写者 (此 API), 单读者 (parser M1 分支)
int SMC_SetOptionalStopEnable(int enable)
{
    g_optional_stop_enabled = enable ? 1 : 0;
    printf("[SMC_API] M1 可选停开关 -> %d\n", g_optional_stop_enabled);
    return 0;
}

// ================== P2-A: 实时倍率系统 ==================
// @Context: Non-RealTime (HMI/CAM 通过 RPC 调用)
// @Thread-Safety: 单写者 (此 API), RT 单读者 (每 cycle ms_budget 计算).
//   int/double 对齐天然原子, 单字段不会撕裂. 多字段非一致快照容忍
//   (操作员旋钮转一刻度 → 1-2 个 cycle 内全部生效).
//
// clamp 规则 (v1):
//   feed_pct   ∈ [0, 100]  (> 100 clamp 100, 不报错)
//   rapid_pct  ∈ [0, 100]
//   spindle_pct ∈ [0, 120]
// clamp 不视为错误, ret_code=0, 实际值通过 out_* 回读.
int SMC_SetOverride(int feed_pct, int rapid_pct, int spindle_pct,
                    uint16_t mode_mask, uint16_t mode_value,
                    int *out_feed, int *out_rapid, int *out_spindle,
                    uint16_t *out_mode)
{
    // 全 no-op 检测 (UI 误调用保护)
    if (feed_pct < 0 && rapid_pct < 0 && spindle_pct < 0 && mode_mask == 0) {
        if (out_feed)    *out_feed    = (int)(g_interpolator.feed_override_ratio   * 100.0 + 0.5);
        if (out_rapid)   *out_rapid   = (int)(g_interpolator.rapid_override_ratio  * 100.0 + 0.5);
        if (out_spindle) *out_spindle = (int)(g_interpolator.spindle_override_ratio * 100.0 + 0.5);
        if (out_mode)    *out_mode    = g_interpolator.mode_flags;
        return -1;
    }

    // feed_override_ratio
    if (feed_pct >= 0) {
        int v = feed_pct;
        if (v > 100) v = 100;       // v1 锁 100
        g_interpolator.feed_override_ratio = v / 100.0;
    }
    // rapid_override_ratio
    if (rapid_pct >= 0) {
        int v = rapid_pct;
        if (v > 100) v = 100;
        g_interpolator.rapid_override_ratio = v / 100.0;
    }
    // spindle_override_ratio
    if (spindle_pct >= 0) {
        int v = spindle_pct;
        if (v > 120) v = 120;
        g_interpolator.spindle_override_ratio = v / 100.0;
    }
    // mode_flags: mask/value 模式 (与位寄存器同)
    if (mode_mask != 0) {
        uint16_t cur = g_interpolator.mode_flags;
        uint16_t new_flags = (cur & ~mode_mask) | (mode_value & mode_mask);
        g_interpolator.mode_flags = new_flags;
    }

    // 回读 clamp 后实际生效值 (UI 旋钮位置同步用)
    if (out_feed)    *out_feed    = (int)(g_interpolator.feed_override_ratio   * 100.0 + 0.5);
    if (out_rapid)   *out_rapid   = (int)(g_interpolator.rapid_override_ratio  * 100.0 + 0.5);
    if (out_spindle) *out_spindle = (int)(g_interpolator.spindle_override_ratio * 100.0 + 0.5);
    if (out_mode)    *out_mode    = g_interpolator.mode_flags;

    printf("[SMC_API] Override: feed=%d%% rapid=%d%% spindle=%d%% mode=0x%04X\n",
           (int)(g_interpolator.feed_override_ratio   * 100.0 + 0.5),
           (int)(g_interpolator.rapid_override_ratio  * 100.0 + 0.5),
           (int)(g_interpolator.spindle_override_ratio * 100.0 + 0.5),
           g_interpolator.mode_flags);
    return 0;
}

int SMC_GetOverride(int *feed_pct, int *rapid_pct, int *spindle_pct,
                    uint16_t *mode_flags)
{
    if (!feed_pct && !rapid_pct && !spindle_pct && !mode_flags) return -1;
    if (feed_pct)    *feed_pct    = (int)(g_interpolator.feed_override_ratio   * 100.0 + 0.5);
    if (rapid_pct)   *rapid_pct   = (int)(g_interpolator.rapid_override_ratio  * 100.0 + 0.5);
    if (spindle_pct) *spindle_pct = (int)(g_interpolator.spindle_override_ratio * 100.0 + 0.5);
    if (mode_flags)  *mode_flags  = g_interpolator.mode_flags;
    return 0;
}
