#include "ecat_core.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "planner.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>

/************************ 全局变量定义（SOEM+实时控制相关） ************************/
// SOEM核心变量
ecx_contextt ctx;
uint8 IOmap[4096];
OSAL_THREAD_HANDLE thread_rt;
OSAL_THREAD_HANDLE thread_chk;
OSAL_THREAD_HANDLE thread_parser;
int expectedWKC;
int wkc;
int mappingdone = 0;
volatile int dorun = 0;
int inOP = 0;
_Atomic int g_sys_alarm_state = 0;
int dowkccheck = 0, currentgroup = 0;
int g_all_axis_enabled = 0;

// 实时控制变量
int cycle = 0;
int g_csp_ready = 0;
int64 cycletime = CYCLE_TIME_NS;
int64 syncoffset = 500000;
int64 timeerror;
float pgain = 0.01f;
float igain = 0.00002f;
extern struct timespec ts;

/************************ 实时线程环形日志 ************************/
RtLog_t g_rt_log = {{0}, 0, 0};

static void rt_log(const char *fmt, ...)
{
    int next = (g_rt_log.head + 1) % RT_LOG_BUF_SIZE;
    if (next == g_rt_log.tail) return; // 缓冲满，丢弃
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(g_rt_log.buffer[g_rt_log.head], RT_LOG_MSG_LEN, fmt, ap);
    va_end(ap);
    __sync_synchronize(); // 写内存屏障：确保 buffer 内容对 drain 线程可见后再更新 head
    g_rt_log.head = next;
}

static void rt_log_drain(void)
{
    while (1) {
        __sync_synchronize(); // 读屏障：确保看到最新的 head 和 buffer 内容
        if (g_rt_log.tail == g_rt_log.head) break;
        printf("%s\n", g_rt_log.buffer[g_rt_log.tail]);
        g_rt_log.tail = (g_rt_log.tail + 1) % RT_LOG_BUF_SIZE;
        __sync_synchronize(); // 写屏障：确保 tail 更新对实时线程可见
    }
}




/************************ 时间工具：纳秒级时间累加 ************************/
void add_time_ns(ec_timet *ts, int64 addtime)
{
    ec_timet addts;
    addts.tv_nsec = addtime % NSEC_PER_SEC;
    addts.tv_sec  = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
    osal_timespecadd(ts, &addts, ts);
}


/**
 * 同步函数：根据从站DC时间调整主站下一个周期的唤醒时刻。
 */
void ec_sync(int64 reftime,int64 cycletime,int64 *offsettime){
    static int64 integral=0;
    int64 delta;
    delta=(reftime-syncoffset)%cycletime;
    if(delta>(cycletime/2)){
        delta=delta-cycletime;
    }
    timeerror=-delta;
    integral+=timeerror;
    *offsettime=(int64)((timeerror*pgain)+(integral*igain));
}

/************************ 实时控制线程（核心！1ms周期，五轴同周期控制） ************************/
OSAL_THREAD_FUNC_RT ecat_thread_rt(void *arg)
{
    ec_timet ts;
    int ht;
    static int64_t toff = 0;
    uint16 sw = 0;

    dorun=0;
    while (!mappingdone) osal_usleep(100);
    osal_get_monotonic_time(&ts);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    struct sched_param sp = {.sched_priority = 99};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);

    struct sched_param param;
    param.sched_priority = 99;
    sched_setscheduler(0, SCHED_FIFO, &param);
    rt_log("[RT] 已设置FIFO调度，优先级99");

    osal_get_monotonic_time(&ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    ecx_send_processdata(&ctx);
    rt_log("[RT] 初始化完成，等待dorun启动");

    while (1)
    {
        add_time_ns(&ts, cycletime + toff);
        osal_monotonic_sleep(&ts);

        if(dorun>0){
            cycle++;
            wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

            static int is_first_run=1;
            if(is_first_run&&wkc>0){
                for(int i=0;i<AXIS_NUM;i++){
                    for(int s=0;s<g_axis[i].slave_count;s++){
                        int slave_id=g_axis[i].slave_ids[s];
                        g_axis[i].home_offset[s]=axis_pdo_read_pos(slave_id);
                    }
                }
            }
            is_first_run=0;

            if (ctx.slavelist[0].hasdc && (wkc > 0))
                ec_sync(ctx.DCtime, cycletime, &toff);

            // === 报警复位安全点：必须在 g_all_axis_op_ready 门控之外 ===
            // 若复位逻辑在门控内，op_ready=0 时请求永远无法被消费 → 死锁。
            // 但消费条件必须同时满足：插补器已停稳 + 驱动器全部就绪。
            // 若驱动器仍在 case 4 故障恢复中，此条件不满足，RT 线程安全空转等待。
            if(g_interpolator.alarm_reset_request){
                if(g_all_axis_op_ready && (!g_interpolator.is_moving || g_interpolator.hold_state == HOLD_PAUSED)){
                    g_interpolator.is_moving = 0;
                    g_interpolator.is_waiting_mcode = 0;
                    g_interpolator.time_scale = 1.0;
                    g_interpolator.hold_state = HOLD_NORMAL;
                    g_interpolator.alarm_reset_request = 0;
                    g_cmd_queue.tail = g_cmd_queue.head;
                    // 同步插补器位置到驱动器实际位置，防止故障恢复后位置偏差
                    for(int j=0;j<AXIS_NUM;j++){
                        if(g_axis[j].slave_count < 1 || g_axis[j].slave_ids[0] <= 0) continue;
                        int64_t raw_pulse_j = (int64_t)axis_pdo_read_pos(g_axis[j].slave_ids[0])
                                            - (int64_t)g_axis[j].home_offset[0];
                        double ppu = g_axis[j].pulse_per_unit;
                        g_axis[j].current_cmd_pos = (ppu > 1e-6) ? (double)raw_pulse_j / ppu : 0.0;
                        g_interpolator.current_pos[j] = g_axis[j].current_cmd_pos;
                    }
                    api_sync_planner_cursor();
                    atomic_store_explicit(&g_sys_alarm_state, 0, memory_order_release);
                    rt_log("[RT] 报警复位已执行，队列已清空，位置已同步");
                }
            }

            // === 插补器运动控制：必须在 g_all_axis_op_ready 门控之外 ===
            // 单轴故障时，非故障轴仍需跟踪平滑刹车轨迹。
            // 若插补器被门控冻结，time_scale 无法递减，HOLD_BRAKING 永远不会完成。
            if(g_interpolator.is_moving){

                int alarm_active = atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire);
                if(alarm_active){
                    if(g_interpolator.hold_state != HOLD_PAUSED && g_interpolator.hold_state != HOLD_BRAKING){
                        g_interpolator.hold_state = HOLD_BRAKING;
                    }
                } else {
                    if (g_interpolator.pause_request && g_interpolator.hold_state == HOLD_NORMAL) {
                        g_interpolator.hold_state = HOLD_BRAKING;
                    } else if (!g_interpolator.pause_request && g_interpolator.hold_state == HOLD_PAUSED) {
                        g_interpolator.hold_state = HOLD_RESUMING;
                    }
                }

                const double TIME_DEC_STEP = 0.005;

                if (g_interpolator.hold_state == HOLD_BRAKING) {
                    g_interpolator.time_scale -= TIME_DEC_STEP;
                    if (g_interpolator.time_scale <= 0.0) {
                        g_interpolator.time_scale = 0.0;
                        g_interpolator.hold_state = HOLD_PAUSED;
                    }
                } else if (g_interpolator.hold_state == HOLD_RESUMING) {
                    g_interpolator.time_scale += TIME_DEC_STEP;
                    if (g_interpolator.time_scale >= 1.0) {
                        g_interpolator.time_scale = 1.0;
                        g_interpolator.hold_state = HOLD_NORMAL;
                    }
                }

                g_interpolator.virtual_time_ms+=g_interpolator.time_scale;

                // ==== 7段式 S 曲线绝对解析插补 ====
                // @Context: 1ms Hard-RT Thread (EtherCAT)
                // @Danger: NO BLOCKING, NO MATH.H (sqrt/acos), NO PRINTF, NO MALLOC.
                // 无状态：仅凭 virtual_time_ms 与预计算参数直接解析 s。
                {
                    double t = g_interpolator.virtual_time_ms;
                    double s = 0.0;
                    // 局部缓存，减少对结构体的重复访问
                    double _T1=g_interpolator.T1, _T2=g_interpolator.T2;
                    double _T3=g_interpolator.T3, _T4=g_interpolator.T4;
                    double _T5=g_interpolator.T5, _T6=g_interpolator.T6;
                    double _T7=g_interpolator.T7;

                    if (t >= _T7) {
                        s = g_interpolator.total_distance;
                        g_interpolator.is_moving = 0;
                    }
                    else if (t <= _T1) {
                        // T1: 加加速段  s = s0 + v0*dt + j1*dt^3/6
                        double dt = t;
                        s = g_interpolator.s0 + g_interpolator.v0*dt
                          + g_interpolator.j1*dt*dt*dt/6.0;
                    }
                    else if (t <= _T2) {
                        // T2: 匀加速段  s = s1 + v1*dt + 0.5*a2*dt^2
                        double dt = t - _T1;
                        s = g_interpolator.s1 + g_interpolator.v1*dt
                          + 0.5*g_interpolator.a2*dt*dt;
                    }
                    else if (t <= _T3) {
                        // T3: 减加速段  s = s2 + v2*dt + 0.5*a2*dt^2 + j3*dt^3/6
                        double dt = t - _T2;
                        s = g_interpolator.s2 + g_interpolator.v2*dt
                          + 0.5*g_interpolator.a2*dt*dt
                          + g_interpolator.j3*dt*dt*dt/6.0;
                    }
                    else if (t <= _T4) {
                        // T4: 匀速段  s = s3 + v3*dt
                        double dt = t - _T3;
                        s = g_interpolator.s3 + g_interpolator.v3*dt;
                    }
                    else if (t <= _T5) {
                        // T5: 加减速段  s = s4 + v4*dt + j5*dt^3/6
                        double dt = t - _T4;
                        s = g_interpolator.s4 + g_interpolator.v4*dt
                          + g_interpolator.j5*dt*dt*dt/6.0;
                    }
                    else if (t <= _T6) {
                        // T6: 匀减速段  s = s5 + v5*dt + 0.5*a6*dt^2
                        double dt = t - _T5;
                        s = g_interpolator.s5 + g_interpolator.v5*dt
                          + 0.5*g_interpolator.a6*dt*dt;
                    }
                    else {
                        // T7: 减减速段  s = s6 + v6*dt + 0.5*a6*dt^2 + j7*dt^3/6
                        double dt = t - _T6;
                        s = g_interpolator.s6 + g_interpolator.v6*dt
                          + 0.5*g_interpolator.a6*dt*dt
                          + g_interpolator.j7*dt*dt*dt/6.0;
                    }

                    double ratio = 0.0;
                    if (g_interpolator.total_distance > 1e-6) {
                        ratio = s / g_interpolator.total_distance;
                    }
                    if (ratio > 1.0) ratio = 1.0;
                    if (ratio < 0.0) ratio = 0.0;

                    for (int j = 0; j < AXIS_NUM; j++) {
                        g_interpolator.current_pos[j] = g_interpolator.start_pos[j]
                            + (g_interpolator.target_pos[j] - g_interpolator.start_pos[j]) * ratio;
                    }
                }

            }

            if(g_all_axis_op_ready){
                for(int i=0;i<AXIS_NUM;i++){
                    if(g_axis[i].slave_count>=2&&g_axis[i].enable_sync_alarm){
                        int slave_1=g_axis[i].slave_ids[0];
                        int slave_2=g_axis[i].slave_ids[1];

                        int32_t act_1=axis_pdo_read_pos(slave_1)-g_axis[i].home_offset[0];
                        int32_t act_2=axis_pdo_read_pos(slave_2)-g_axis[i].home_offset[1];

                        int32_t diff=abs(act_1-act_2);
                        if(diff>g_axis[i].sync_max_err_pulse){
                           rt_log("同步异常！轴%d 从站%d与%d位置差%d超过最大误差%d", i, slave_1, slave_2, diff, g_axis[i].sync_max_err_pulse);
                           atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                           continue;
                        }

                        if(diff>g_axis[i].sync_tolerance_pulse){
                            g_axis[i]._current_sync_timer++;
                            if(g_axis[i]._current_sync_timer>g_axis[i].sync_err_time_ms){
                                rt_log("同步报警！轴%d 从站%d与%d差%d持续%dms超限%d", i, slave_1, slave_2, diff, g_axis[i]._current_sync_timer, g_axis[i].sync_err_time_ms);
                                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                            }
                        }else{
                            g_axis[i]._current_sync_timer=0;
                        }
                    }
                }
            }

            if(g_all_axis_op_ready){
                if(g_interpolator.is_moving==0&&g_interpolator.is_waiting_mcode==0&&g_cmd_queue.head!=g_cmd_queue.tail&&atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire)==0){

                    if(atomic_load_explicit(&g_cmd_queue.buffer[g_cmd_queue.tail].is_ready, memory_order_acquire)==1){
                        TrajectorySegment_t seg=g_cmd_queue.buffer[g_cmd_queue.tail];
                        g_cmd_queue.tail=(g_cmd_queue.tail+1)%QUEUE_SIZE;

                        if(seg.cmd_type==CMD_TYPE_MCODE){
                            g_interpolator.is_waiting_mcode=1;
                            g_interpolator.mcode_wait_timer=0;
                            g_interpolator.current_mcode=seg.m_code;
                            continue;
                        }

                        for(int j=0;j<AXIS_NUM;j++){
                            g_interpolator.start_pos[j]=g_interpolator.current_pos[j];
                            g_interpolator.target_pos[j]=seg.target_pos[j];
                            g_interpolator.dir_vec[j]=seg.dir_vec[j];
                        }

                        g_interpolator.total_distance=seg.total_distance;
                        // 7段式 S 曲线绝对解析参数（planner 已预计算）
                        g_interpolator.T1=seg.T1; g_interpolator.T2=seg.T2;
                        g_interpolator.T3=seg.T3; g_interpolator.T4=seg.T4;
                        g_interpolator.T5=seg.T5; g_interpolator.T6=seg.T6;
                        g_interpolator.T7=seg.T7;
                        g_interpolator.v0=seg.v0; g_interpolator.v1=seg.v1;
                        g_interpolator.v2=seg.v2; g_interpolator.v3=seg.v3;
                        g_interpolator.v4=seg.v4; g_interpolator.v5=seg.v5;
                        g_interpolator.v6=seg.v6;
                        g_interpolator.s0=seg.s0; g_interpolator.s1=seg.s1;
                        g_interpolator.s2=seg.s2; g_interpolator.s3=seg.s3;
                        g_interpolator.s4=seg.s4; g_interpolator.s5=seg.s5;
                        g_interpolator.s6=seg.s6;
                        g_interpolator.j1=seg.j1; g_interpolator.a2=seg.a2;
                        g_interpolator.j3=seg.j3;
                        g_interpolator.j5=seg.j5; g_interpolator.a6=seg.a6;
                        g_interpolator.j7=seg.j7;
                        g_interpolator.v_target=seg.v_target;
                        g_interpolator.acc=seg.acc;
                        g_interpolator.dec=seg.dec;
                        g_interpolator.v_start=seg.v_start;
                        g_interpolator.v_end=seg.v_end;
                        g_interpolator.virtual_time_ms=0.0;

                        if(seg.T_total > 0.5){
                            g_interpolator.is_moving=1;
                        }else{
                            rt_log("invalid trajectory segment: T_total=%.3f",seg.T_total);
                        }
                    }
                }

                // 非阻塞M代码等待逻辑
                if(g_interpolator.is_waiting_mcode){
                    if(g_interpolator.mcode_wait_timer < INT32_MAX) {
                        g_interpolator.mcode_wait_timer++;
                    }
                    int wait_target_ms;
                    switch(g_interpolator.current_mcode){
                        case 3:  wait_target_ms=2000; break;
                        case 5:  wait_target_ms=1000; break;
                        default: wait_target_ms=1000; break;
                    }
                    if(g_interpolator.mcode_wait_timer >= wait_target_ms ||
                       g_interpolator.mcode_wait_timer >= MCODE_WAIT_TIMEOUT_MS){
                        g_interpolator.is_waiting_mcode=0;
                        g_interpolator.mcode_wait_timer=0;
                    }
                }

                for(int j=0;j<AXIS_NUM;j++){
                    g_coord_mgr.current_g53_pos[j]=g_interpolator.current_pos[j];
                    if(g_coord_mgr.current_coord==COORD_G53){
                        g_coord_mgr.current_logical_pos[j]=g_coord_mgr.current_g53_pos[j];
                    }else{
                        int idx=g_coord_mgr.current_coord-1;
                        g_coord_mgr.current_logical_pos[j]=g_coord_mgr.current_g53_pos[j]-g_coord_mgr.work_offsets[idx][j];
                    }
                }
            }

            int all_ready_flag=1;

            for(int i=0;i<AXIS_NUM;i++){
                int primary_slave=g_axis[i].slave_ids[0];
                uint16_t sw=axis_pdo_read_sw(primary_slave);
                g_axis[i].cia_step_delay++;

                int32_t output_cw=CW_ENABLE_OP;

                switch(g_axis[i].cia_step){
                    case 0:
                    output_cw=CW_SHUTDOWN;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if(g_axis[i].cia_step_delay>50&&((sw&SW_MASK)==SW_SHUTDOWN_RDY)){
                        g_axis[i].cia_step++;
                        g_axis[i].cia_step_delay=0;
                    }
                    all_ready_flag=0;
                    break;

                    case 1:
                    output_cw=CW_SWITCH_ON;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if(g_axis[i].cia_step_delay>50&&((sw&SW_MASK)==SW_SWITCHED_ON)){
                        if(llabs(timeerror)<80000){
                            g_axis[i].cia_step++;
                            g_axis[i].cia_step_delay=0;
                        }
                    }
                    all_ready_flag=0;
                    break;

                    case 2:
                    output_cw=CW_ENABLE_OP;
                    if(g_axis[i].cia_step_delay==1){
                        int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                        g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                    }
                    if((sw&SW_MASK)==SW_OP_ENABLED){
                        g_axis[i].cia_step_delay++;
                        if(g_axis[i].cia_step_delay>50){
                            g_axis[i].cia_step_delay=0;
                            g_axis[i].cia_step++;

                            int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                            g_axis[i].current_cmd_pos=(double)raw_pulse/g_axis[i].pulse_per_unit;
                            api_sync_planner_cursor();
                        }
                    }
                    all_ready_flag=0;
                    break;

                    case 3:
                    output_cw=CW_ENABLE_OP;

                    // 驱动器故障检测：必须在跟随误差监控之前
                    // 当驱动器内部触发故障(如 AL009)时，状态字 bit3(SW_ERROR) 置位，
                    // 需进入 case 4 执行 CW_FAULT_RESET 并重新走使能流程。
                    // 绝对禁止在此处置 is_moving=0！仅触发报警，让虚拟时间轴平滑刹车。
                    if(sw & SW_ERROR){
                        g_axis[i].cia_step = 4;
                        g_axis[i].cia_step_delay = 0;
                        g_axis[i]._follow_err_timer = 0;
                        all_ready_flag = 0;
                        output_cw = CW_FAULT_RESET;
                        atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                        break;
                    }

                    if(g_axis[i].cia_step_delay==1){
                        int64_t primary_pulse = (int64_t)axis_pdo_read_pos(g_axis[i].slave_ids[0])
                                              - (int64_t)g_axis[i].home_offset[0];

                        double ppu = g_axis[i].pulse_per_unit;
                        g_axis[i].current_cmd_pos = (ppu > 1e-6) ? (double)primary_pulse / ppu : 0.0;
                        // 仅在插补器完全静止时才同步位置，运动/刹车中禁止覆写 current_pos！
                        // 否则插补器下一周期会用 start_pos + dir_vec*ratio 覆盖回来，产生位置阶跃。
                        if(!g_interpolator.is_moving) {
                            g_interpolator.current_pos[i] = g_axis[i].current_cmd_pos;
                        }
                    }else{
                        // 无论是否报警，只要轴在 case 3（健康运行），就必须追随插补器！
                        // 插补器已在报警时启动 HOLD_BRAKING 平滑减速，健康轴必须跟随减速，
                        // 否则等于硬停，违背平滑急停设计。
                        g_axis[i].current_cmd_pos=g_interpolator.current_pos[i];
                    }

                    // 跟随误差监控：直接从驱动器 TxPDO 0x60F4 读取（驱动器自身计算，无坐标系偏移风险）
                    {
                        int32_t follow_err=axis_pdo_read_follow_err(g_axis[i].slave_ids[0]);
                        int32_t abs_err=follow_err<0?-follow_err:follow_err;
                        if(abs_err>FOLLOW_ERR_MAX_PULSE){
                            rt_log("[跟随误差] %s 硬限超差 %d 脉冲",g_axis[i].axis_name,abs_err);
                            g_interpolator.is_moving = 0;
                            g_interpolator.is_waiting_mcode = 0;
                            atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                        }else if(abs_err>FOLLOW_ERR_WARN_PULSE){
                            g_axis[i]._follow_err_timer++;
                            if(g_axis[i]._follow_err_timer>=FOLLOW_ERR_WARN_TIME_MS){
                                rt_log("[跟随误差] %s 警告持续 %dms",g_axis[i].axis_name,g_axis[i]._follow_err_timer);
                                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                            }
                        }else{
                            g_axis[i]._follow_err_timer=0;
                        }
                    }

                    break;

                    case 4:
                    // 驱动器故障恢复：发送 CW_FAULT_RESET，等待故障位清除后重新使能
                    all_ready_flag = 0;
                    if(sw & SW_ERROR){
                        output_cw = CW_FAULT_RESET;
                        // 故障期间：命令位置死咬实际位置，防止恢复瞬间飞车追位
                        if(g_axis[i].slave_count >= 1 && g_axis[i].slave_ids[0] > 0){
                            int32_t raw_pulse = axis_pdo_read_pos(g_axis[i].slave_ids[0]) - g_axis[i].home_offset[0];
                            double ppu = g_axis[i].pulse_per_unit;
                            g_axis[i].current_cmd_pos = (ppu > 1e-6) ? (double)raw_pulse / ppu : 0.0;
                        }
                    } else {
                        // 故障已清除，回到 case 0 重新走完整使能流程
                        g_axis[i].cia_step = 0;
                        g_axis[i].cia_step_delay = 0;
                        rt_log("[CiA402] %s 故障已清除，重新使能", g_axis[i].axis_name);
                    }
                    break;
                }

                for(int s=0;s<g_axis[i].slave_count;s++){
                    int slave_id=g_axis[i].slave_ids[s];
                    int32_t logical_pulse=(int32_t)round(g_axis[i].current_cmd_pos*g_axis[i].pulse_per_unit);
                    int32_t phys_pos_to_send=logical_pulse+g_axis[i].home_offset[s];
                    axis_pdo_write(slave_id,output_cw,phys_pos_to_send);
                }
            }

            g_all_axis_op_ready=all_ready_flag;

            ecx_mbxhandler(&ctx, 0, 4);
            ecx_send_processdata(&ctx);

        }else{
            g_interpolator.is_waiting_mcode=0;
            g_interpolator.mcode_wait_timer=0;
            for(int i=0;i<AXIS_NUM;i++){
               g_axis[i]._follow_err_timer=0;
            }
            for(int i=0;i<AXIS_NUM;i++){
               for(int s=0;s<g_axis[i].slave_count;s++){
                int slave_id=g_axis[i].slave_ids[s];
                int32_t act=axis_pdo_read_pos(slave_id);
                axis_pdo_write(slave_id,CW_SHUTDOWN,act);
               }
            }
            ecx_send_processdata(&ctx);
        }

    }
    return ;
}

/************************ 故障检查线程 ************************/
OSAL_THREAD_FUNC ecat_thread_chk(void *arg)
{
    int slaveix;
    // 饥饿看门狗状态
    static volatile int last_queue_head = -1;
    static int starvation_counter = 0;
    #define STARVATION_THRESHOLD 5  // 5 × 10ms = 50ms 无新指令视为饥饿

    while (1)
    {
        if (inOP && ((dowkccheck > 2) || ctx.grouplist[currentgroup].docheckstate))
        {
            ctx.grouplist[currentgroup].docheckstate = FALSE;
            ecx_readstate(&ctx);

            for (int i = 0; i < AXIS_NUM; i++)
            {
                for(int s=0;s<g_axis[i].slave_count;s++){
                    slaveix = g_axis[i].slave_ids[s];
                    ec_slavet *slave = &ctx.slavelist[slaveix];
                    if (slave->state != EC_STATE_OPERATIONAL)
                    {
                       ctx.grouplist[currentgroup].docheckstate = TRUE;
                       g_axis[i].is_error = 1;
                       printf("[故障检测] %s 非OP状态！当前状态码：%d\n", g_axis[i].axis_name, slave->state);
                    }
                }
            }
            dowkccheck = 0;
        }

        // ---- 队列饥饿看门狗 ----
        // 仅在解析器已关闭（无新文件读入）时才允许触发强制释放
        // 防止文件 I/O 阻塞或网络抖动导致的误强行结段
        {
            int count = (g_cmd_queue.head - g_cmd_queue.tail + QUEUE_SIZE) % QUEUE_SIZE;
            if(count > 0 && count < 3 && !g_parser_ctrl.is_running) {
                if(g_cmd_queue.head == last_queue_head) {
                    starvation_counter++;
                    if(starvation_counter >= STARVATION_THRESHOLD) {
                        planner_recalculate(1);
                        starvation_counter = 0;
                    }
                } else {
                    starvation_counter = 0;
                    last_queue_head = g_cmd_queue.head;
                }
            } else {
                starvation_counter = 0;
                last_queue_head = g_cmd_queue.head;
            }
        }

        rt_log_drain();
        osal_usleep(10000);
    }
    return ;
}

/************************ ECAT主站启动封装 ************************/
void ecat_bringup(char *ifname)
{
    if (!ecx_init(&ctx, ifname))
    {
        printf("[ECAT错误] 主站初始化失败！网卡：%s\n", ifname);
        exit(-1);
    }
    printf("[ECAT] 主站初始化成功，网卡：%s\n", ifname);

    if (ecx_config_init(&ctx) <= 0)
    {
        printf("[ECAT错误] 未扫描到任何EtherCAT从站！\n");
        ecx_close(&ctx);
        exit(-1);
    }
    printf("[ECAT] 扫描到%d个EtherCAT从站\n", ctx.slavecount);

    // TxPDO 重映射：添加 0x60F4 跟随误差到 PDO 输入（必须在 config_map_group 之前）
    for(int i=0;i<AXIS_NUM;i++){
        for(int s=0;s<g_axis[i].slave_count;s++){
            uint16_t slave=g_axis[i].slave_ids[s];
            if(slave==0) continue;

            // 切到 PRE-OP 才能改 PDO 映射
            ctx.slavelist[slave].state=EC_STATE_PRE_OP;
            ecx_writestate(&ctx,slave);
            ecx_statecheck(&ctx,slave,EC_STATE_PRE_OP,EC_TIMEOUTSTATE);

            // 禁用 TxPDO 分配
            uint8_t zero=0;
            ecx_SDOwrite(&ctx,slave,0x1C13,0x00,FALSE,1,&zero,EC_TIMEOUTRXM);

            // 清空映射对象 0x1A00
            ecx_SDOwrite(&ctx,slave,0x1A00,0x00,FALSE,1,&zero,EC_TIMEOUTRXM);

            // 写入三条映射：状态字(16bit) + 实际位置(32bit) + 跟随误差(32bit)
            uint32_t map_sw   =0x60410010;
            uint32_t map_pos  =0x60640020;
            uint32_t map_ferr =0x60F40020;
            ecx_SDOwrite(&ctx,slave,0x1A00,0x01,FALSE,4,&map_sw,EC_TIMEOUTRXM);
            ecx_SDOwrite(&ctx,slave,0x1A00,0x02,FALSE,4,&map_pos,EC_TIMEOUTRXM);
            ecx_SDOwrite(&ctx,slave,0x1A00,0x03,FALSE,4,&map_ferr,EC_TIMEOUTRXM);

            // 设置映射条目数
            uint8_t map_cnt=3;
            ecx_SDOwrite(&ctx,slave,0x1A00,0x00,FALSE,1,&map_cnt,EC_TIMEOUTRXM);

            // 绑定到 TxPDO 分配
            uint16_t pdo_idx=0x1A00;
            ecx_SDOwrite(&ctx,slave,0x1C13,0x01,FALSE,2,&pdo_idx,EC_TIMEOUTRXM);

            // 启用 TxPDO 分配
            uint8_t one=1;
            ecx_SDOwrite(&ctx,slave,0x1C13,0x00,FALSE,1,&one,EC_TIMEOUTRXM);

            printf("[PDO] 从站%d TxPDO: 0x6041+0x6064+0x60F4\n", slave);
        }
    }

    // SOEM 读取更新后的 PDO 映射，自动分配 IOmap 偏移
    ecx_config_map_group(&ctx, IOmap, 0);
    ecx_configdc(&ctx);

    for(int i=1;i<=ctx.slavecount;i++){
        ecx_dcsync0(&ctx,i,TRUE,cycletime,0);
    }

    printf("[ECAT] trun to PRE_OP!\n");
    for(int i=0;i<AXIS_NUM;i++){
        for(int s=0;s<g_axis[i].slave_count;s++){
          uint16_t slave=g_axis[i].slave_ids[s];
          if(slave==0) continue;

          ctx.slavelist[slave].state=EC_STATE_PRE_OP;
          ecx_writestate(&ctx,slave);
          ecx_statecheck(&ctx,slave,EC_STATE_PRE_OP,EC_TIMEOUTSTATE);

          uint16_t sync_mode=2;
          int sz=sizeof(sync_mode);
          int r1=ecx_SDOwrite(&ctx,slave,0x1C32,0x01,FALSE,sz,&sync_mode,EC_TIMEOUTRXM);
          int r2=ecx_SDOwrite(&ctx,slave,0x1C33,0x01,FALSE,sz,&sync_mode,EC_TIMEOUTRXM);
          if(r1>0&&r2>0){
              printf("success to 0x1C32=2\n",g_axis[i].axis_name);
          }else{
              printf("fail to 0x1C32=2\n",g_axis[i].axis_name);
          }
          ctx.slavelist[slave].state=EC_STATE_SAFE_OP;
          ecx_writestate(&ctx,slave);
          ecx_statecheck(&ctx,slave,EC_STATE_SAFE_OP,EC_TIMEOUTSTATE);
        }
    }

    for(int i=0;i<AXIS_NUM;i++){
        for(int s=0;s<g_axis[i].slave_count;s++){
          uint16_t slave=g_axis[i].slave_ids[s];
          if(slave==0) continue;

          uint8_t time_value=1;
          int8_t time_index=-3;
          int r1=ecx_SDOwrite(&ctx,slave,0x60C2,0x01,FALSE,1,&time_value,EC_TIMEOUTRXM);
          int r2=ecx_SDOwrite(&ctx,slave,0x60C2,0x02,FALSE,1,&time_index,EC_TIMEOUTRXM);

          if(r1>0&&r2>0){
              printf("success 0x60C2\n",g_axis[i].axis_name);
          }else{
              printf("fail to 0x60C2\n",g_axis[i].axis_name);
          }
        }
    }

    expectedWKC = ctx.grouplist[0].outputsWKC * 2 + ctx.grouplist[0].inputsWKC;
    printf("[ECAT] PDO映射+DC配置完成，期望WKC值：%d\n", expectedWKC);

    for (int i = 0; i < AXIS_NUM; i++)
    {
        for(int s=0;s<g_axis[i].slave_count;s++){
            uint16_t slave=g_axis[i].slave_ids[s];
            ctx.slavelist[slave].state = EC_STATE_SAFE_OP;
            ecx_writestate(&ctx, slave);
            if (ecx_statecheck(&ctx, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE) != EC_STATE_SAFE_OP)
            {
               printf("[ECAT错误] %s 无法进入SAFE_OP状态！\n", g_axis[i].axis_name);
               ecx_close(&ctx);
               exit(-1);
            }
        }
    }
    printf("[ECAT] 所有轴已进入SAFE_OP状态，开始SDO配置\n");

    for (int i = 0; i < AXIS_NUM; i++)
    {
        axis_sdo_config_pp(i);
    }

    mappingdone = 1;
    dorun = 1;
    osal_usleep(1000000);

    ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(&ctx, 0);
    if (ecx_statecheck(&ctx, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE) != EC_STATE_OPERATIONAL)
    {
        printf("[ECAT错误] 无法进入全局OP状态！\n");
        ecx_close(&ctx);
        exit(-1);
    }

    inOP = 1;
    printf("[ECAT] 全局OP状态进入成功，五轴伺服系统启动完成！\n");
}
