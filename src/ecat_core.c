#include "ecat_core.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include "planner.h"
#include "trace_logger.h"
#include "sim_engine.h"
#include "sim_drive.h"
#include "snapshot_hub.h"
#include "event_logger.h"   /* P1-b: EventLogger_Push (RT 安全, 无锁无 malloc) */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdarg.h>
#include <time.h>
#include <sched.h>      /* sched_setscheduler / sched_yield */

/************************ 全局变量定义（SOEM+实时控制相关） ************************/
// SOEM核心变量
ecx_contextt ctx;
uint8 IOmap[4096];
OSAL_THREAD_HANDLE thread_rt;
OSAL_THREAD_HANDLE thread_chk;
OSAL_THREAD_HANDLE thread_parser;
OSAL_THREAD_HANDLE thread_bspline;
int expectedWKC;
int wkc;
int mappingdone = 0;
volatile int dorun = 0;
int inOP = 0;
_Atomic int g_sys_alarm_state = 0;
int dowkccheck = 0, currentgroup = 0;
int g_all_axis_enabled = 0;
int g_sim_mode = 0;

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

// ====================================================================
// rt_resolve_scurve_step — 7段式 S 曲线绝对解析 (RT 内联函数)
// @Context: 1ms Hard-RT Thread (ecat_thread_rt 内部)
// @Danger: NO BLOCKING, NO MATH.H (sqrt/acos), NO PRINTF, NO MALLOC.
//
// 输入: g_interpolator.virtual_time_ms 已被调用方更新到目标时刻
// 输出: 更新 g_interpolator.current_pos[AXIS_NUM], v_current, is_moving,
//            current_phase, phase_T_curr, phase_T_next
//
// 双重优化 (与旧版逐位兼容):
//   (a) Phase-Cached Switch: 跨周期 phase 缓存替代 7 层 if-else 链
//   (b) 2-Double Bound Cache: 仅缓存当前 phase 下/上边界 (16B),
//       全量 T1..T7 (56B) 仅在 phase 转换时懒构建
//
// 关键修复 (阶段 1-A): case 7 末态 v_now = v_end (而非硬编码 0.0),
//   保证跨段瞬间速度连续 (v_end_A = v_start_B by planner 反向扫描)。
// ====================================================================
static inline void rt_resolve_scurve_step(double eff_scale)
{
    double t = g_interpolator.virtual_time_ms;
    double s = 0.0;
    double v_now = 0.0;

    int phase = g_interpolator.current_phase;
    if (phase < 1) phase = 1;
    if (phase > 7) phase = 7;

    double _T_curr = g_interpolator.phase_T_curr;
    double _T_next = g_interpolator.phase_T_next;

    // Phase 状态机前向推进: t 单调递增 → phase 仅递增
    if (t > _T_next && phase < 7) {
        const double T_arr[8] = {0.0, g_interpolator.T1, g_interpolator.T2,
                                 g_interpolator.T3, g_interpolator.T4,
                                 g_interpolator.T5, g_interpolator.T6,
                                 g_interpolator.T7};
        do {
            _T_curr = _T_next;
            phase++;
            _T_next = T_arr[phase];
        } while (phase < 7 && t > _T_next);
    }

    g_interpolator.current_phase = phase;
    g_interpolator.phase_T_curr = _T_curr;
    g_interpolator.phase_T_next = _T_next;

    double dt = t - _T_curr;

    switch (phase) {
        case 1:
            s = g_interpolator.s0 + g_interpolator.v0*dt
              + g_interpolator.j1*dt*dt*dt/6.0;
            v_now = g_interpolator.v0 + 0.5*g_interpolator.j1*dt*dt;
            break;
        case 2:
            s = g_interpolator.s1 + g_interpolator.v1*dt
              + 0.5*g_interpolator.a2*dt*dt;
            v_now = g_interpolator.v1 + g_interpolator.a2*dt;
            break;
        case 3:
            s = g_interpolator.s2 + g_interpolator.v2*dt
              + 0.5*g_interpolator.a2*dt*dt
              + g_interpolator.j3*dt*dt*dt/6.0;
            v_now = g_interpolator.v2 + g_interpolator.a2*dt
                  + 0.5*g_interpolator.j3*dt*dt;
            break;
        case 4:
            // T4 匀速段: v_current 由段加载时预置的 seg.v3,跳过 v_now 写入
            s = g_interpolator.s3 + g_interpolator.v3*dt;
            break;
        case 5:
            s = g_interpolator.s4 + g_interpolator.v4*dt
              + g_interpolator.j5*dt*dt*dt/6.0;
            v_now = g_interpolator.v4 + 0.5*g_interpolator.j5*dt*dt;
            break;
        case 6:
            s = g_interpolator.s5 + g_interpolator.v5*dt
              + 0.5*g_interpolator.a6*dt*dt;
            v_now = g_interpolator.v5 + g_interpolator.a6*dt;
            break;
        case 7:
        default:
            if (t >= _T_next) {
                // 段末态: s = total_distance, v_now = v_end (阶段 1-A 修复)
                s = g_interpolator.total_distance;
                v_now = g_interpolator.v_end;
                g_interpolator.is_moving = 0;
            } else {
                s = g_interpolator.s6 + g_interpolator.v6*dt
                  + 0.5*g_interpolator.a6*dt*dt
                  + g_interpolator.j7*dt*dt*dt/6.0;
                v_now = g_interpolator.v6 + g_interpolator.a6*dt
                      + 0.5*g_interpolator.j7*dt*dt;
            }
            break;
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

    // T4 phase 跳过 v_current 写入 (已预置); 其他 phase 写回 v_now
    // P2-A: v_current 必须是"物理瞬时速度" = 段内 S 曲线速度(ds/dτ) × 实际时间缩放(eff_scale).
    //   eff_scale = time_scale × override_ratio, 即 virtual_time 相对 wall-clock 的真实推进速率.
    //   不乘 eff_scale 则 override/feedhold 期间 v_current 仍显示满速, 与 operator HMI/探针语义不符
    //   (代码注释 line 740 明确标注 v_current = "物理瞬时速度"). 正常加工 eff_scale=1.0, 行为不变.
    if (phase != 4) {
        if (v_now < 0.0) v_now = 0.0;
        g_interpolator.v_current = v_now * eff_scale;
    }
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

    // ============================================================
    // 调度策略: 硬件模式 = SCHED_FIFO 99 + 锁 Core 3 (硬实时)
    //           仿真模式 = SCHED_OTHER + 不锁核 (避免饿死同核线程)
    // 仿真模式若保留 SCHED_FIFO 99, 主循环空转 (ecat_core.c:250)
    // 会以最高优先级独占 Core 3, 把同核的 parser/chk/BSpline 线程
    // 饿死, 在 WSL2 等核心数受限的环境下进一步触发 OS 卡死。
    // ============================================================
    if (g_sim_mode) {
        struct sched_param sp_sim = {.sched_priority = 0};
        sched_setscheduler(0, SCHED_OTHER, &sp_sim);
        rt_log("[RT] 仿真模式: 已降级 SCHED_OTHER, 不锁 CPU 亲和性");
    } else {
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
    }

    osal_get_monotonic_time(&ts);
    ht = (ts.tv_nsec / 1000000) + 1;
    ts.tv_nsec = ht * 1000000;
    if (!g_sim_mode) ecx_send_processdata(&ctx);
    rt_log("[RT] 初始化完成，等待dorun启动");

    while (1)
    {
        // ================================================================
        // 仿真模式节流 (避免 tight loop 饿死 I/O 线程):
        //   默认 500us = 节流到 wall-clock ≈ 0.5× 实时 (5000 cycles ≈ 2.5s/5mm G01)
        //   SIM_RT_SLEEP_US=0    = 真正 tight loop (超光速, 验证场景慎用)
        //   SIM_RT_SLEEP_US=1000 = 严格匹配 1ms cycletime (真实节拍)
        //
        // 问题背景: 此前 sim 模式 cycle 末有 osal_usleep(10000) (10ms 周期),
        // RT 线程实际 wall-clock = 10ms/cycle, 38 段队列 × 5000 cycles × 10ms
        // = 31 分钟, 远超 30s 安全停超时, 造成 "Q:N 始终清不完"。
        //
        // 修复: cycle 内由 SIM_RT_SLEEP_US 节流 (默认 500us), cycle 末 sim 模式
        //       不再额外 sleep, wall-clock ≈ 0.5ms/cycle, 38 段 × 2.5s = 95s
        //       (仍可能超 30s 超时, 但比 31 分钟好; 验证推荐 SIM_RT_SLEEP_US=100)
        // ================================================================
        if (g_sim_mode) {
            static int sim_sleep_us = -1;
            if (sim_sleep_us < 0) {
                const char *env = getenv("SIM_RT_SLEEP_US");
                sim_sleep_us = env ? atoi(env) : 500;
                rt_log("[RT] 仿真节流: SIM_RT_SLEEP_US=%d us (来源: %s)",
                       sim_sleep_us, env ? "环境变量" : "默认值");
            }
            if (sim_sleep_us > 0) osal_usleep(sim_sleep_us);
            // sim_sleep_us == 0: 真正 tight loop (用户显式要求超光速)
        } else {
            add_time_ns(&ts, cycletime + toff);
            osal_monotonic_sleep(&ts);
        }

        if(dorun == 1){
            cycle++;

            // ---- 仿真模式: 全局周期计数 (sim_drive_step_axis 用此去重) ----
            // CLAUDE.md 红线 #3 的最小例外: 仅一行原子递增, 不引入分支逻辑
            if (g_sim_mode) {
                atomic_fetch_add_explicit(&g_sim_rt_cycle, 1, memory_order_relaxed);
            }

            // ---- 仿真模式: 跳过真实 EtherCAT 收发 ----
            if (g_sim_mode) {
                wkc = expectedWKC > 0 ? expectedWKC : 1;
                // 跳过 ec_sync (无物理 DC 时钟)
            } else {
                wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);
            }

            static int is_first_run=1;
            if(is_first_run&&wkc>0){
                for(int i=0;i<AXIS_NUM;i++){
                    for(int s=0;s<g_axis[i].slave_count;s++){
                        int slave_id=g_axis[i].slave_ids[s];
                        g_axis[i].home_offset[s]=axis_pdo_read_pos(slave_id);
                    }
                }
                // P0 修复: is_first_run=0 必须在 if 块内, 只有 wkc>0 锚定成功才置 0.
                // 原 BUG: is_first_run=0 在 if 块外, 首周期 wkc=0 (EtherCAT 刚启动 PDO 未交换)
                //         时 home_offset[] 保持 0 但 is_first_run 已置 0, 下次永不锚定.
                //         后果: 下游 axis_pdo_read_pos(slave_id) - home_offset[0] 持续错位,
                //               机械坐标系整体偏移, 撞机风险.
                // 修复后: 首周期 wkc=0 时 is_first_run 保持 1, 下个周期重试直到锚定成功.
                // sim 模式: wkc stub 始终>0, 首周期立即锚定, 行为不变.
                is_first_run=0;
                rt_log("[RT] home_offset 锚定完成 (cycle=%d wkc=%d)", cycle, wkc);
            }

            if (!g_sim_mode && ctx.slavelist[0].hasdc && (wkc > 0))
                ec_sync(ctx.DCtime, cycletime, &toff);

            // ---- P0-Laser: cycle 头安全门 (alarm_reset 之前, 段消费之前) ----
            // 检查 DI 互锁 + g_sys_alarm_state, 异常时立即关激光 + 锁存 emergency_kill.
            // 必须早于段消费环: 段消费时会调 laser_rt_apply_aux, 如果 emergency_kill=1
            // 则 apply_aux 屏蔽 seg 推进, 保证段消费不会重新打开激光.
            if (laser_rt_safety_gate()) {
                // DI 互锁异常触发系统软停机 (与跟随误差超差同等级别)
                // safety_gate 内部已 set g_laser_rt, 这里只需保证运动也停车
                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                /* P1-b: laser safety 互锁报警事件 (rt_log 风格, 常量 message) */
                EventLogger_Push(SEVERITY_ALARM, SOURCE_LASER, 0x0010,
                                 (int32_t)g_laser_rt.interlock_status,
                             "laser safety interlock triggered (door/estop/alm/water/gas)");
            }

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
                    // P0-Laser: 报警复位时清激光急停锁存 (不自动重开激光, 需 M3 重发)
                    laser_rt_reset();
                    // Lock-Free 队列报警复位: 把 read_tail 推到 write_head,
                    // 丢弃所有未消费段。RT 线程是 read_tail 的唯一写者,这里
                    // release 写保证后续 ecx_send_processdata 看到的 read_tail
                    // 已永久推进,生产者下次 acquire 读 read_tail 也会立即观察到。
                    {
                        int cur_head = atomic_load_explicit(&g_cmd_queue.write_head,
                                                            memory_order_acquire);
                        atomic_store_explicit(&g_cmd_queue.read_tail, cur_head,
                                              memory_order_release);
                    }
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
                    /* P1-b: 通知 UI alarm 已清 (异步 ClearAlarm 的最终确认) */
                    EventLogger_Push(SEVERITY_INFO, SOURCE_MANUAL, 0x0041, 0,
                                     "alarm cleared by RT (queue flushed, pos synced)");
                }
            }

            // ==== 无缝连续插补消费环 (Seamless Continuous Interpolation) ====
            // @Context: 1ms Hard-RT Thread (EtherCAT)
            // @Danger: NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
            //
            // 架构原则 (阶段 2 重构):
            //   旧版每个 1ms 周期只走一段的一部分,跨段在两个周期边界发生,
            //   量化伪影掩盖了 S 曲线的微小不连续。
            //   新版用 while 循环消费 ms_budget = time_scale 时间预算,
            //   允许同一周期内跨过多段 (消除量化伪影),
            //   实现真正的无缝连续插补环。
            //
            // 关键不变量:
            //   ① ms_budget 总量 = time_scale (feedhold 控制流速)
            //   ② 跨段瞬间: current_pos 连续 (start_pos_B = current_pos_A),
            //                v_current 连续 (v_end_A = v_start_B by planner 反向扫描)
            //   ③ 微段 Snap 必须扣除 ms_budget (防死循环)
            //   ④ 迭代上限 RT_ITER_CAP = 64 (防御病态输入)

            // ---- feedhold 状态机 (前移到 while 之前) ----
            // 必须在 ms_budget 取值前递减 time_scale,否则 HOLD_BRAKING 失效。
            // is_moving=0 时也允许执行 (resume 状态可能从静止恢复),
            // 但 hold_state 转换只在 is_moving=1 时触发 (静止时无需 刹车/恢复)。
            {
                int alarm_active = atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire);
                if (g_interpolator.is_moving) {
                    if (alarm_active) {
                        if (g_interpolator.hold_state != HOLD_PAUSED
                            && g_interpolator.hold_state != HOLD_BRAKING) {
                            g_interpolator.hold_state = HOLD_BRAKING;
                        }
                    } else {
                        if (g_interpolator.pause_request && g_interpolator.hold_state == HOLD_NORMAL) {
                            g_interpolator.hold_state = HOLD_BRAKING;
                        } else if (!g_interpolator.pause_request
                                   && g_interpolator.hold_state == HOLD_PAUSED) {
                            g_interpolator.hold_state = HOLD_RESUMING;
                        }
                    }
                } else {
                    // P2-A-3: 单段死锁修复 (Plan agent 发现的关键 BUG)
                    // 现象: G93 strict 段完成时 v_current=v_target (非 0), 段完成 is_moving 1→0,
                    //       原状态机因 if(is_moving) gate 跳过 → pause_request 永不消费 → 死锁.
                    //       同样问题在普通段 + 单段模式时也发生 (段完成瞬间 is_moving=0).
                    // 修复: !is_moving && pause_request && hold_state==NORMAL → 直接 HOLD_PAUSED.
                    //       无需刹车 (本就静止), 直接进入 PAUSED 等待 ResumeProcessing.
                    //       time_scale 置 0 防止 ms_budget 误判本周期仍要插补.
                    if (g_interpolator.pause_request
                        && g_interpolator.hold_state == HOLD_NORMAL) {
                        g_interpolator.hold_state = HOLD_PAUSED;
                        g_interpolator.time_scale = 0.0;
                    } else if (!g_interpolator.pause_request
                               && g_interpolator.hold_state == HOLD_PAUSED) {
                        // P2-A-3 单段模式 resume 修复 (补 is_moving=0 分支):
                        //   段完成瞬间 is_moving=0 且 hold_state=HOLD_PAUSED, 若只在
                        //   is_moving=1 分支处理 !pause_request→HOLD_RESUMING, 则 resume
                        //   清掉 pause_request 后本分支无出口 → 永久 HOLD_PAUSED 死锁
                        //   (D4 单段只停一次且 done=False 的根因). 此处补上静止态 resume.
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
            }

            // ---- 无缝插补消费环 ----
            // just_loaded_seg 跨周期锁存: while 内置 1, trace 块消费清 0
            // 必须在 while 之前声明 (C 语法: 先声明后使用)
            static int just_loaded_seg = 0;
            #define RT_ITER_CAP 64  // 防御: 64 段/周期 已足够覆盖最密集微段场景
            // ---- P2-A: 实时倍率数学 ----
            // @Context: 1ms Hard-RT Thread (EtherCAT)
            // @Danger: NO BLOCKING / NO MATH.H / NO PRINTF / NO MALLOC. 仅乘法 + clamp.
            //
            // 设计原则: 完全复用 time_scale feedhold 机制. eff_scale 是 time_scale × override_ratio.
            //   - time_scale 由 feedhold 状态机维护 (HOLD_BRAKING/RESUMING 平滑过渡)
            //   - override_ratio 由 SMC_SetOverride (非 RT) 写, RT 每 cycle 读
            //   - G00 段用 rapid_override_ratio, G01/G02/G03 用 feed_override_ratio
            //   - dry_run 模式下所有段都走 rapid_override (industry std prove-out)
            //   - G93 strict 段也受影响 (与 LinuxCNC/Fanuc 一致: virtual time 流速变慢 → wall-clock 翻倍)
            //
            // 红线 #3 (无缝插补无 static 状态继承): ms_budget 仅缩放 virtual_time_ms 推进步长,
            //   7 段 S 曲线解析方程 s(τ) = s_n + v_n·τ + ½·a_n·τ² + ⅙·j_n·τ³ 不变.
            // v1 clamp: eff_scale ≤ 1.0 (与 feedhold envelope 测试一致, v2 开放到 1.5).
            double eff_scale = g_interpolator.time_scale;
            if (g_interpolator.is_moving) {
                int is_rapid = (g_interpolator.current_motion_type_rt == 0);  // G00 = motion_type 0
                if (g_interpolator.mode_flags & SMC_MODE_DRY_RUN) is_rapid = 1;  // dry_run 强制 rapid 通道
                double ratio = is_rapid ? g_interpolator.rapid_override_ratio
                                        : g_interpolator.feed_override_ratio;
                eff_scale *= ratio;
                if (eff_scale > 1.0) eff_scale = 1.0;  // v1 不允许超 100% (待 v2 envelope 验证)
                if (eff_scale < 0.0) eff_scale = 0.0;  // 防御 (override 不应为负, 但 clamp 兜底)
            }
            double ms_budget = eff_scale;
            int rt_iter = 0;

            while (ms_budget > 1e-6 && rt_iter < RT_ITER_CAP) {
                rt_iter++;

                // (1) M 代码屏障: 冻结插补,等待 mcode_wait_timer 计时
                if (g_interpolator.is_waiting_mcode) break;

                // (2) 报警冻结: 不消费预算,等下一周期处理
                if (atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) != 0) break;

                // (3) 加载新段 (当 is_moving=0 时尝试加载)
                if (!g_interpolator.is_moving) {
                    int rt_head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_acquire);
                    int rt_tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_acquire);

                    if (rt_head == rt_tail) {
                        // 队列真实枯竭 (Starvation):
                        //   解析器仍运行 → 等待 planner 释放段,break 不归零 v_current
                        //   解析器已停   → 彻底没指令,v_current 强制归零 (防止残留速度误导 trace)
                        if (!g_parser_ctrl.is_running) {
                            g_interpolator.v_current = 0.0;
                        }
                        break;
                    }

                    if (atomic_load_explicit(&g_cmd_queue.buffer[rt_tail].is_ready,
                                              memory_order_acquire) != 1) {
                        // 队列有数据但 Planner 还没算完 (Starvation)
                        break;
                    }

                    // 拷出整段 + 推进 read_tail (release)
                    TrajectorySegment_t seg = g_cmd_queue.buffer[rt_tail];
                    atomic_store_explicit(&g_cmd_queue.read_tail,
                                          (rt_tail + 1) % QUEUE_SIZE,
                                          memory_order_release);

                    // 段内带内 WCS + 偏置快照同步:
                    //   current_coord = seg.active_wcs (WCS 切换不超前)
                    //   active_offset  = seg.wcs_offset_snap (隔离 #5221/G10 L2 中途改 work_offsets 的污染)
                    // 两者都是 g_coord_mgr 内 RT 单写者字段。普通 store 即可 — UI/宏/TCP 都是读者,
                    // 通过下个 PDO 周期的 happens-before 自然可见,无需原子或锁。
                    g_coord_mgr.current_coord = seg.active_wcs;
                    for(int j = 0; j < AXIS_NUM; j++){
                        g_coord_mgr.active_offset[j] = seg.wcs_offset_snap[j];
                    }

                    // ---- Phase B1: 段级耦合配置同步 (cmd_type 分支前, M 段 + 运动段共用) ----
                    // 把入队时快照的 coupling_mode/v_thresh 同步到 g_laser_rt.*_rt 镜像.
                    // 段执行期间 coupling_update 读 *_rt 镜像, 不再读全局 g_laser_cfg.
                    // 修复架构 BUG: parser 单线程解析快, M70 P1 → M70 P0 顺序覆盖会在 RT 第一次读
                    //   之前完成, RT 全程看到 mode=0. 段级快照让 mode 在段执行期间稳定.
                    laser_rt_sync_config(&seg);

                    // M 代码段: 进入等待屏障
                    if (seg.cmd_type == CMD_TYPE_MCODE) {
                        g_interpolator.is_waiting_mcode = 1;
                        g_interpolator.mcode_wait_timer = 0;
                        g_interpolator.current_mcode = seg.m_code;
                        // ---- P1': 同步 aux 状态镜像 (RT 单写者, CSV 读) ----
                        // parser 入队时已把 g_state aux 字段快照到 seg;
                        // 这里消费 seg 时一次性拷到 g_interpolator._rt, 供 sim CSV trace。
                        g_interpolator.spindle_mode_rt    = seg.aux_spindle_mode;
                        g_interpolator.spindle_rpm_rt     = seg.aux_spindle_rpm;
                        g_interpolator.coolant_state_rt   = seg.aux_coolant;
                        g_interpolator.current_tool_id_rt = seg.aux_tool_id;
                        // Phase B2: G04 dwell P 值传递 (M64 段, switch case 64 读)
                        g_interpolator.mcode_p_value_ms = seg.p_value;
                        // ---- P0-Laser: 同步激光镜像 (仅 M 段, 运动段保持模态) ----
                        // 设计原因: 运动段的 aux_laser_* 未填 (api_push_trajectory_impl
                        // 不写这些字段, 与 P1' spindle/coolant 同模式). 若运动段也同步,
                        // memset=0 的 aux_laser_* 会清零 g_laser_rt, 破坏激光模态.
                        // 正确语义: 激光开/关由 M62/M63/M67/M68/M10/M3/M5 等 M 段触发,
                        //           运动段沿用上次模态状态 (Fanuc 标准).
                        g_interpolator.laser_enable_rt  = seg.aux_laser_enable;
                        g_interpolator.laser_shutter_rt = seg.aux_laser_shutter;
                        g_interpolator.laser_power_w_rt = seg.aux_laser_power_w;
                        g_interpolator.laser_freq_hz_rt = seg.aux_laser_freq_hz;
                        g_interpolator.gas_select_rt    = seg.aux_gas_select;
                        // P0-Laser-Q: 段级工艺标记镜像 (M 段消费时同步)
                        // M 段 (含 M64 dwell / M72-M75 段标记) 也要同步 seg_flags,
                        // 让 HMI 在穿孔 dwell 期间能读到正确的 lead_in 上下文.
                        g_interpolator.current_seg_flags_rt = seg.seg_flags;
                        laser_rt_apply_aux(&seg);
                        break;
                    }

                    // 运动段: 装载 S 曲线参数
                    for (int j = 0; j < AXIS_NUM; j++) {
                        g_interpolator.start_pos[j] = g_interpolator.current_pos[j];
                        g_interpolator.target_pos[j] = seg.target_pos[j];
                        g_interpolator.dir_vec[j] = seg.dir_vec[j];
                    }
                    g_interpolator.total_distance = seg.total_distance;
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
                    g_interpolator.virtual_time_ms = 0.0;
                    g_interpolator.current_seg_id_rt = seg.seg_id;  // P0-c: 记录当前段 ID
                    // P2-A: 同步当前段 motion_type 给 RT 倍率数学 (区分 feed/rapid 通道)
                    // 必须在 ms_budget 计算前完成 (本块在 while 主循环内, 下次 cycle 立即生效).
                    // 注意: 第一次进入本块时 is_moving=0, ms_budget 不读 current_motion_type_rt
                    //       (eff_scale 计算有 is_moving 门控), 无 stale read 风险.
                    g_interpolator.current_motion_type_rt = seg.motion_type;
                    // P2-A-4: 镜像精准停标记 (E2E 可观测, 客户端/测试据此识别精准停拐角)
                    g_interpolator.current_seg_is_exact_stop_rt = seg.is_exact_stop;
                    // P0-Laser-Q: 段级工艺标记镜像 (运动段消费时同步)
                    // 运动段 (G00-G03) 的 seg_flags 由 parser M72-M75 modal 快照而来,
                    // HMI 据此判断 "当前切割段是不是引线/微连接".
                    g_interpolator.current_seg_flags_rt = seg.seg_flags;
                    g_interpolator.current_phase = 1;
                    g_interpolator.phase_T_curr = 0.0;
                    g_interpolator.phase_T_next = seg.T1;

                    if (seg.T_total > 0.5) {
                        // 正常段: 进入插补消费
                        g_interpolator.is_moving = 1;
                        g_interpolator.v_current = seg.v3 * eff_scale;  // T4 预置 (case 1 会立即覆盖)
                    } else {
                        // 微段 Snap (阶段 1-B 修复死循环):
                        //   T_total ≤ 0.5ms 时无法稳定插补,直接快进到 target_pos。
                        //   必须扣除 ms_budget = seg.T_total (按段时长),
                        //   否则连续 Snap 微段会触发 while 死循环。
                        //   下限 0.001ms 防御 T_total=0 病态值。
                        for (int j = 0; j < AXIS_NUM; j++) {
                            g_interpolator.current_pos[j] = g_interpolator.target_pos[j];
                        }
                        double snap_time = (seg.T_total > 0.001) ? seg.T_total : 0.001;
                        ms_budget -= snap_time;
                        just_loaded_seg = 1;  // 触发 trace 记录 Snap 落点
                        // 不置 is_moving,继续 while 加载下一段
                        continue;
                    }

                    just_loaded_seg = 1;  // 触发 trace 记录新段落点
                }

                // (4) 执行 S 曲线插补消费 (此时 is_moving=1)
                //   两条路径:
                //     A. ms_budget < remaining_time → 停留在本段内部,预算耗尽
                //     B. ms_budget >= remaining_time → 本段结束,扣除预算,继续 while
                {
                    double remaining_time = g_interpolator.T7 - g_interpolator.virtual_time_ms;
                    if (remaining_time < 1e-6) remaining_time = 1e-6;  // 防御 T7≈0

                    if (ms_budget < remaining_time) {
                        // 路径 A: 预算不够走完本段,停留
                        g_interpolator.virtual_time_ms += ms_budget;
                        rt_resolve_scurve_step(eff_scale);  // 解析当前 phase,更新 current_pos/v_current
                        ms_budget = 0.0;  // 预算耗尽,while 退出
                    } else {
                        // 路径 B: 跨段! 本段在本周期内结束
                        g_interpolator.virtual_time_ms = g_interpolator.T7;
                        rt_resolve_scurve_step(eff_scale);  // 末态: s=total_distance, v_current=v_end
                        ms_budget -= remaining_time;
                        g_interpolator.is_moving = 0;  // 标记本段结束,下次 while 加载新段
                        // P2-A-3: 单段模式 re-arm (运动段完成时)
                        // 读 mode_flags 当前值, 允许 UI 中途 toggle off (下次完成时不再 re-arm).
                        // 设 ms_budget=0 强制 while 退出, 防止本 cycle 内连续吃多段.
                        // 下个 cycle 顶部状态机会检测到 pause_request 并进 HOLD_PAUSED.
                        if (g_interpolator.mode_flags & SMC_MODE_SINGLE_BLOCK) {
                            g_interpolator.pause_request = 1;
                            ms_budget = 0.0;
                        }
                        // while 继续 (若 ms_budget 还有,加载下一段)
                    }
                }
            }  // end while

            // ---- P2-A-2: Override + Dry-Run "有效输出"计算 (RT 单写者) ----
            // @Context: 1ms Hard-RT Thread (EtherCAT)
            // @Danger: 仅乘法 + 条件赋值, NO BLOCKING / NO MATH.H / NO PRINTF.
            //
            // 设计原因: 主轴倍率必须连续生效 (操作员旋钮实时性要求), 不能只在 seg-load 时乘.
            //          Dry-Run 是工业 prove-out 安全机制: 强制 spindle/coolant/laser 不出力,
            //          让程序空跑验证轨迹不碰撞, 但主轴/激光不实际启动.
            //
            // 计算时机: 在 seg-load while 之后, sim_engine_push / SnapshotHub_Publish 之前.
            //          每 cycle 1 次 (不论段边界), 保证操作员旋钮变化 1 cycle 内可见.
            //
            // 红线合规:
            //   - 仅修改输出镜像, 不改 *_rt 原始模态 (dry_run 关闭后立即恢复正常)
            //   - 不影响 ms_budget (override 时间缩放在前面已完成)
            //   - laser_enable_rt 强制 0 后, laser_rt_apply_aux 不会重开 (emergency_kill 同机制)
            if (g_interpolator.mode_flags & SMC_MODE_DRY_RUN) {
                // Dry-Run: 强制 0
                g_interpolator.eff_spindle_mode_rt  = 0;
                g_interpolator.eff_spindle_rpm_rt   = 0.0;
                g_interpolator.eff_coolant_state_rt = 0;
                g_interpolator.eff_laser_enable_rt  = 0;
            } else {
                // 正常: 应用主轴倍率 (连续生效, 旋钮变化 1 cycle 内可见)
                g_interpolator.eff_spindle_mode_rt  = g_interpolator.spindle_mode_rt;
                g_interpolator.eff_spindle_rpm_rt   = (g_interpolator.spindle_mode_rt != 0)
                    ? g_interpolator.spindle_rpm_rt * g_interpolator.spindle_override_ratio
                    : 0.0;
                g_interpolator.eff_coolant_state_rt = g_interpolator.coolant_state_rt;
                g_interpolator.eff_laser_enable_rt  = g_interpolator.laser_enable_rt;
            }

            // ---- 无锁轨迹探针 + 仿真采集 (阶段 3 精简) ----
            // @Context: 1ms Hard-RT Thread (EtherCAT)
            // @Danger: TraceLogger_Push / sim_engine_push 均无锁非阻塞, 满时静默丢弃。
            //
            // 新架构 (while 消费环) 下的触发语义:
            //   旧版 is_falling_edge 在新架构中失去意义 —— 跨段在 while 内部消化,
            //   退出 while 后的 is_moving 状态可能是 "刚跨完段正在等下一段"
            //   (is_moving=0 但运动未真正结束)。检测 1→0 下降沿会产生大量误触发。
            //
            //   新版触发条件 (二选一):
            //   (a) is_moving == 1            —— 正常连续插补中
            //   (b) just_loaded_seg == 1      —— while 内刚加载过段 (含 Snap 路径)
            //
            //   just_loaded_seg 在 while 内置 1, 此处消费后清 0。
            //   Snap 微段 (T_total ≤ 0.5) 的 v_current 无物理意义,log_velocity 置零。
            int current_is_moving = atomic_load_explicit(&g_interpolator.is_moving,
                                                          memory_order_acquire);
            // P0-Laser: 检查强制日志标志 (parser M30 抢写 g_laser_rt 后置 1)
            // 用途: M30 Step 4 抢写后 RT 静止 (is_moving=0 ∧ just_loaded_seg=0),
            //       should_log=0 → sim_engine_push 不调 → CSV 末尾保留旧值.
            //       force_log=1 时下个 cycle 强制记录一次新状态, 然后自动清 0.
            int force_log = atomic_load_explicit(&g_sim_force_log, memory_order_acquire);
            // Phase B2: M 段等待期间也记录 (dwell/M3/M5 等 freeze 期 is_moving=0 但状态机在变)
            // 不加此项: G04 P1000 dwell 期间 CSV 0 行, 无法验证 dwell power_w=P_base
            int in_mcode_wait = (g_interpolator.is_waiting_mcode != 0);
            int should_log_this_cycle = current_is_moving || just_loaded_seg
                                     || force_log || in_mcode_wait;
            if (force_log) {
                atomic_store_explicit(&g_sim_force_log, 0, memory_order_release);
            }

            // 探针速度基准: 物理瞬时速度 v_current
            int is_snap_segment = just_loaded_seg && (g_interpolator.T7 <= 0.5);
            double log_velocity = g_interpolator.v_current;
            if (is_snap_segment || (!current_is_moving && !just_loaded_seg)) {
                // Snap 微段或完全静止时, log_velocity 强制 0.0
                log_velocity = 0.0;
            }

            // 消费跨周期锁存标志
            int was_loaded = just_loaded_seg;
            just_loaded_seg = 0;

            // ---- 真实硬件探针: 运动中每 5ms 降采样 + 加载立即记录 ----
            if (!g_sim_mode && should_log_this_cycle
                && (cycle % 5 == 0 || was_loaded)) {
                int idx_x = g_axis_map['X' - 'A'];
                int idx_y = g_axis_map['Y' - 'A'];
                int idx_z = g_axis_map['Z' - 'A'];
                int idx_b = g_axis_map['B' - 'A'];
                int idx_c = g_axis_map['C' - 'A'];
                TraceLogger_Push(
                    cycle,
                    g_interpolator.virtual_time_ms,
                    (idx_x >= 0) ? g_interpolator.current_pos[idx_x] : 0.0,
                    (idx_y >= 0) ? g_interpolator.current_pos[idx_y] : 0.0,
                    (idx_z >= 0) ? g_interpolator.current_pos[idx_z] : 0.0,
                    (idx_b >= 0) ? g_interpolator.current_pos[idx_b] : 0.0,
                    (idx_c >= 0) ? g_interpolator.current_pos[idx_c] : 0.0,
                    log_velocity
                );
            }

            // ---- 仿真模式: 每周期全量轨迹采集 (1ms 精度, 无降采样, 绝不漏 Snap) ----
            // @Context: 1ms Hard-RT Thread
            // @Danger: sim_engine_push 无锁无阻塞, 双缓冲原子交换+sem_post
            // 生产 1:1 完整物理轨迹供上位机 3D 渲染比对
            if (g_sim_mode && should_log_this_cycle) {
                double local_off[3];
                for (int i = 0; i < 3; i++) local_off[i] = g_coord_mgr.active_offset[i];
                int x_idx = g_axis_map['X' - 'A'];
                double off_g54_x = (x_idx >= 0) ? g_coord_mgr.work_offsets[0][x_idx] : 0.0;
                sim_engine_push((uint64_t)cycle,
                                g_interpolator.virtual_time_ms,
                                g_interpolator.current_pos,
                                log_velocity,
                                (int)g_coord_mgr.current_coord,
                                g_coord_mgr.current_logical_pos,
                                local_off,
                                off_g54_x,
                                g_interpolator.eff_spindle_mode_rt,
                                g_interpolator.eff_spindle_rpm_rt,
                                g_interpolator.eff_coolant_state_rt,
                                g_interpolator.current_tool_id_rt,
                                // P0-Laser: 7 个激光状态镜像 (从 g_laser_rt 读)
                                g_laser_rt.enable,
                                g_laser_rt.shutter,
                                g_laser_rt.power_w,
                                g_laser_rt.freq_hz,
                                g_laser_rt.gas_select,
                                g_laser_rt.emergency_kill,
                                g_laser_rt.interlock_status,
                                // Phase B1: 当前周期瞬时速度 (供验证 P-v 耦合曲线)
                                g_laser_rt.v_actual_mm_s,
                                // P0-Laser-Q: 状态查询闭环 4 字段
                                g_laser_rt.pierce_count,
                                g_laser_rt.laser_on_time_ms,
                                g_interpolator.current_seg_flags_rt,
                                // is_piercing 派生: G04 dwell (M64) 等待期间为 1
                                (g_interpolator.is_waiting_mcode &&
                                 g_interpolator.current_mcode == 64) ? 1 : 0);
            }

            // ---- P0-a: 状态快照推送 (所有模式, 无条件) ----
            // @Context: 1ms Hard-RT Thread
            // @Danger: SnapshotHub_Publish 仅 atomic store + memcpy, 无锁无阻塞.
            //         不受 should_log_this_cycle / g_sim_mode 约束:
            //         静止时也要推, 让 HMI 看到 alarm/feedhold 状态变化.
            //         g_state / g_parser_ctrl 字段 best-effort 读, 撕裂容忍.
            SnapshotHub_Publish((uint32_t)cycle);

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
                           EventLogger_Push(SEVERITY_ALARM, SOURCE_DRIVE, 0x0001, diff,
                                            "gantry sync err: position diff exceeds max");
                           continue;
                        }

                        if(diff>g_axis[i].sync_tolerance_pulse){
                            g_axis[i]._current_sync_timer++;
                            if(g_axis[i]._current_sync_timer>g_axis[i].sync_err_time_ms){
                                rt_log("同步报警！轴%d 从站%d与%d差%d持续%dms超限%d", i, slave_1, slave_2, diff, g_axis[i]._current_sync_timer, g_axis[i].sync_err_time_ms);
                                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                                EventLogger_Push(SEVERITY_ALARM, SOURCE_DRIVE, 0x0001, diff,
                                                 "gantry sync err: persistent tolerance breach");
                            }
                        }else{
                            g_axis[i]._current_sync_timer=0;
                        }
                    }
                }
            }

            // ---- 非阻塞 M 代码等待计时 (阶段 2 保留) ----
            // 段加载逻辑已移到 while 消费环,此处仅保留 mcode_wait_timer 递增。
            if(g_all_axis_op_ready && g_interpolator.is_waiting_mcode){
                if(g_interpolator.mcode_wait_timer < INT32_MAX) {
                    g_interpolator.mcode_wait_timer++;
                }
                int wait_target_ms;
                switch(g_interpolator.current_mcode){
                    // ---- P1': 按语义分类的等待时间 ----
                    case 3:  wait_target_ms=2000; break;  // M3 spindle CW 加速
                    case 4:  wait_target_ms=2000; break;  // M4 spindle CCW
                    case 5:  wait_target_ms=1500; break;  // M5 spindle stop (含减速)
                    case 6:  wait_target_ms=3000; break;  // M6 换刀 (sim 中占位)
                    case 7:  wait_target_ms=500;  break;  // M7 mist coolant
                    case 8:  wait_target_ms=500;  break;  // M8 flood coolant
                    case 9:  wait_target_ms=500;  break;  // M9 coolant off
                    case 19: wait_target_ms=1000; break;  // M19 spindle orient
                    // ---- Phase B2: G04 dwell (M64 段携带 dwell_ms 在 p_value) ----
                    case 64: wait_target_ms = (int)(g_interpolator.mcode_p_value_ms + 0.5);
                             if(wait_target_ms < 1) wait_target_ms = 1;
                             if(wait_target_ms > MCODE_WAIT_TIMEOUT_MS)
                                 wait_target_ms = MCODE_WAIT_TIMEOUT_MS;
                             break;
                    default: wait_target_ms=500;  break;
                }
                if(g_interpolator.mcode_wait_timer >= wait_target_ms ||
                   g_interpolator.mcode_wait_timer >= MCODE_WAIT_TIMEOUT_MS){
                    g_interpolator.is_waiting_mcode=0;
                    g_interpolator.mcode_wait_timer=0;
                    // P0-Laser-Q: M64 (G04 dwell) 完成时累计穿孔次数
                    // @Context: 1ms Hard-RT Thread
                    // @Danger: 仅 int32 ++, 无 malloc/阻塞/math.h
                    // 必须判 current_mcode==64: 该完成分支处理所有 M 段 (M0/M1/M60-M75),
                    // 只有 M64 (G04 dwell 转换来的穿孔段) 才算穿孔.
                    if (g_interpolator.current_mcode == 64) {
                        g_laser_rt.pierce_count++;
                    }
                    // P2-A-3: 单段模式 re-arm (M 段完成时)
                    // M 段 (M3/M5/M8/M64 dwell 等) 完成后, 若 single_block 模式,
                    // 设 pause_request=1, 下个 cycle 进 HOLD_PAUSED 等 Cycle Start.
                    if (g_interpolator.mode_flags & SMC_MODE_SINGLE_BLOCK) {
                        g_interpolator.pause_request = 1;
                    }
                }
            }

            // ---- 坐标系管理器更新 (阶段 2 保留) ----
            // H-1 修复: 不再读 g_coord_mgr.work_offsets[idx] (那张表可被 parser 中途写污染),
            // 改读 RT 单写者的 active_offset (段消费时已 = seg.wcs_offset_snap)。
            // G53 段的 active_offset 全零,等价于"逻辑坐标 = 机械坐标",不再需要分支。
            if(g_all_axis_op_ready){
                for(int j=0;j<AXIS_NUM;j++){
                    g_coord_mgr.current_g53_pos[j]=g_interpolator.current_pos[j];
                    g_coord_mgr.current_logical_pos[j]=g_coord_mgr.current_g53_pos[j]
                                                       - g_coord_mgr.active_offset[j];
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
                        double ppu=g_axis[i].pulse_per_unit;
                        g_axis[i].current_cmd_pos=(ppu>1e-6)?((double)raw_pulse/ppu):0.0;
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
                        double ppu=g_axis[i].pulse_per_unit;
                        g_axis[i].current_cmd_pos=(ppu>1e-6)?((double)raw_pulse/ppu):0.0;
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
                        double ppu=g_axis[i].pulse_per_unit;
                        g_axis[i].current_cmd_pos=(ppu>1e-6)?((double)raw_pulse/ppu):0.0;
                    }
                    if((sw&SW_MASK)==SW_OP_ENABLED){
                        g_axis[i].cia_step_delay++;
                        if(g_axis[i].cia_step_delay>50){
                            g_axis[i].cia_step_delay=0;
                            g_axis[i].cia_step++;

                            int32_t raw_pulse=axis_pdo_read_pos(primary_slave)-g_axis[i].home_offset[0];
                            double ppu=g_axis[i].pulse_per_unit;
                            g_axis[i].current_cmd_pos=(ppu>1e-6)?((double)raw_pulse/ppu):0.0;
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
                        EventLogger_Push(SEVERITY_ALARM, SOURCE_DRIVE, 0x0002, i,
                                         "drive SW_ERROR detected, FAULT_RESET sent");
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
                            EventLogger_Push(SEVERITY_ALARM, SOURCE_DRIVE, 0x0003, abs_err,
                                             "follow err hard limit exceeded");
                        }else if(abs_err>FOLLOW_ERR_WARN_PULSE){
                            g_axis[i]._follow_err_timer++;
                            if(g_axis[i]._follow_err_timer>=FOLLOW_ERR_WARN_TIME_MS){
                                rt_log("[跟随误差] %s 警告持续 %dms",g_axis[i].axis_name,g_axis[i]._follow_err_timer);
                                atomic_store_explicit(&g_sys_alarm_state, 1, memory_order_release);
                                EventLogger_Push(SEVERITY_ALARM, SOURCE_DRIVE, 0x0004, abs_err,
                                                 "follow err warn persistent");
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
                    /* round-half-away-from-zero 不走 libm: 硬 RT 线程禁止 round()/math.h (libcall → 上下文切换) */
                    double _pos_pulse=g_axis[i].current_cmd_pos*g_axis[i].pulse_per_unit;
                    int32_t logical_pulse=(int32_t)(_pos_pulse>=0.0?_pos_pulse+0.5:_pos_pulse-0.5);
                    int32_t phys_pos_to_send=logical_pulse+g_axis[i].home_offset[s];
                    axis_pdo_write(slave_id,output_cw,phys_pos_to_send);
                }
            }

            g_all_axis_op_ready=all_ready_flag;

            // ---- P0-Laser Phase B1: 功率-速度耦合 (在 flush_pdos 前重算 power_w) ----
            // 读 g_interpolator.v_current + 耦合表, 重算 g_laser_rt.power_w.
            // coupling_mode=0 时内部直接 return, 不影响性能 (单次 int 比较).
            laser_rt_coupling_update();

            // ---- P0-Laser-Q: 加工统计累计 (cycle 尾, coupling 后, flush 前) ----
            // laser_on_time_ms: enable && shutter && !emergency_kill 时每 cycle += 1 (1ms)
            //   enable=主使能 (M3), shutter=激光闸 (M62), 两者都 1 才真出光 (Phase A 设计)
            //   emergency_kill 时禁累加 (急停后不计入加工时长)
            // @Context: 1ms Hard-RT Thread
            // @Danger: 仅 int64 += 1, 无 malloc/阻塞/math.h
            if (g_laser_rt.enable && g_laser_rt.shutter && !g_laser_rt.emergency_kill) {
                g_laser_rt.laser_on_time_ms += 1;
            }

            // ---- P0-Laser: cycle 末刷新激光 PDO ----
            // 必须在 ecx_send_processdata 之前, 保证 DO/AO 在本周期 PDO 帧中生效.
            // sim 模式或 do_slave_id<0 时内部安全跳过, 仅保留 g_laser_rt 供 trace.
            laser_rt_flush_pdos();

            if (!g_sim_mode) ecx_mbxhandler(&ctx, 0, 4);
            if (!g_sim_mode) ecx_send_processdata(&ctx);

        }else if(dorun == 2){
            // ================================================================
            // 优雅下电状态机（1ms 实时心跳不断！）
            // @Context: 1ms Hard-RT Thread (EtherCAT)
            // @Danger: NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
            // 抱闸闭合期间维持 EtherCAT 心跳，防止通信丢失。
            // ================================================================
            if (g_sim_mode) {
                // ============================================================
                // 仿真模式: 简化两步下电 (5ms Step0 + ≤10ms Step1)
                // 不真等 500ms 抱闸, 但走完 CiA402 降级时序, 让
                // ecat_thread_chk / 优雅下电状态机在 sim 中可观察。
                // 计数器用 cycle++ 的等价物 (sim_rt_cycle) 推进。
                // ============================================================
                static int sim_shutdown_step    = 0;
                static int sim_shutdown_counter = 0;

                if (sim_shutdown_step == 0) {
                    // Step 0: 维持 CW_SWITCH_ON + 锁定位置 5ms
                    for (int i = 0; i < AXIS_NUM; i++) {
                        for (int s = 0; s < g_axis[i].slave_count; s++) {
                            axis_pdo_write(g_axis[i].slave_ids[s], CW_SWITCH_ON,
                                           g_axis[i].sim_target_pos);
                        }
                    }
                    if (++sim_shutdown_counter >= 5) {
                        sim_shutdown_step    = 1;
                        sim_shutdown_counter = 0;
                    }
                } else if (sim_shutdown_step == 1) {
                    // Step 1: 发 CW_SHUTDOWN 降级, 等所有 slave 回 SHUTDOWN_RDY
                    int all_back = 1;
                    for (int i = 0; i < AXIS_NUM; i++) {
                        for (int s = 0; s < g_axis[i].slave_count; s++) {
                            int slave_id = g_axis[i].slave_ids[s];
                            axis_pdo_write(slave_id, CW_SHUTDOWN,
                                           g_axis[i].sim_target_pos);
                            uint16_t sw = axis_pdo_read_sw(slave_id);
                            if ((sw & SW_MASK) != SW_SHUTDOWN_RDY) all_back = 0;
                        }
                    }
                    if (all_back || ++sim_shutdown_counter >= 10) {
                        sim_shutdown_step    = 0;
                        sim_shutdown_counter = 0;
                        dorun = 0;
                    }
                }
            } else {
                wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRET);

            static int shutdown_step = 0;
            static int shutdown_counter = 0;

            if(shutdown_step == 0){
                // Step 0: 抱闸闭合等待期 — 维持位置锁定 500ms
                for(int i=0;i<AXIS_NUM;i++){
                    for(int s=0;s<g_axis[i].slave_count;s++){
                        int slave_id=g_axis[i].slave_ids[s];
                        int32_t act=axis_pdo_read_pos(slave_id);
                        axis_pdo_write(slave_id, CW_SWITCH_ON, act);
                    }
                }
                ecx_send_processdata(&ctx);
                shutdown_counter++;
                if(shutdown_counter >= 500){
                    shutdown_step = 1;
                    shutdown_counter = 0;
                }
            }
            else if(shutdown_step == 1){
                // Step 1: CiA402 降级 — 切断动力，等待驱动器确认
                int all_ready = 1;
                for(int i=0;i<AXIS_NUM;i++){
                    for(int s=0;s<g_axis[i].slave_count;s++){
                        int slave_id=g_axis[i].slave_ids[s];
                        int32_t act=axis_pdo_read_pos(slave_id);
                        axis_pdo_write(slave_id, CW_SHUTDOWN, act);
                        uint16 sw = axis_pdo_read_sw(slave_id);
                        if((sw & SW_MASK) != SW_SHUTDOWN_RDY) all_ready = 0;
                    }
                }
                ecx_send_processdata(&ctx);
                shutdown_counter++;
                if(all_ready || shutdown_counter >= 1000){
                    // 所有轴已降级或超时，交还控制权给 SMC_Close
                    dorun = 0;
                    shutdown_step = 0;
                    shutdown_counter = 0;
                }
            }
            } // end else (real hardware shutdown)

        }else{
            // dorun == 0: 心跳兜底（SMC_Close 执行 EtherCAT 降级期间的过渡态）
            if (!g_sim_mode) {
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

        // ============================================================
        // 仿真模式 cycle 末尾处理 (修订):
        //
        // 历史问题: 此前 sim 模式 cycle 末调 sched_yield() "让出 CPU 防止火力空转",
        // 但在 WSL2 + SCHED_OTHER + 多线程 (parser/chk/BSpline/trace_logger/SimEngine)
        // 竞争环境下, sched_yield 让出后 RT 线程要等数十 ms 才能再调度,
        // 导致 cycle wall-clock 飙到 6.58s (而非预期的 500us), 38 段队列 30s 内
        // 一段都消费不完。
        //
        // 修复: cycle 末不再 sched_yield。CPU 让出已由 cycle 内的 osal_usleep
        // (SIM_RT_SLEEP_US) 承担 —— sleep 期间 OS 自然调度其他线程。
        // 硬实时模式绝对不能加 sched_yield 或任何 sleep 调用。
        // ============================================================
        // (sim 模式无操作; real 模式不能在此 yield)
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
    // 解析器卡死二级检测
    static volatile int last_stuck_head = -1;
    static volatile int last_stuck_tail = -1;
    static int stuck_counter = 0;
    #define STUCK_THRESHOLD 200  // 200 × 10ms = 2s 队列无任何进展视为卡死

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
        // 路径 A：解析器已停止 → 队列中任意数量的未释放段均需强制释放
        // 路径 B：解析器标记为运行但实际卡死 → 二级超时兜底
        // 看门狗线程 (非 RT) acquire 读两侧游标,与生产者/RT 消费者 release 写配对。
        {
            int wd_head = atomic_load_explicit(&g_cmd_queue.write_head, memory_order_acquire);
            int wd_tail = atomic_load_explicit(&g_cmd_queue.read_tail, memory_order_acquire);
            int count = (wd_head - wd_tail + QUEUE_SIZE) % QUEUE_SIZE;
            if(count > 0 && !g_parser_ctrl.is_running) {
                // 路径 A：解析器已停，不限段数
                if(wd_head == last_queue_head) {
                    starvation_counter++;
                    if(starvation_counter >= STARVATION_THRESHOLD) {
                        planner_recalculate(1);
                        starvation_counter = 0;
                    }
                } else {
                    starvation_counter = 0;
                    last_queue_head = wd_head;
                }
            } else if(count > 0) {
                // 路径 B：解析器仍标记运行，但队列首尾均无进展 → 可能卡死
                if(wd_head == last_stuck_head &&
                   wd_tail == last_stuck_tail) {
                    stuck_counter++;
                    if(stuck_counter >= STUCK_THRESHOLD) {
                        planner_recalculate(1);
                        stuck_counter = 0;
                    }
                } else {
                    stuck_counter = 0;
                    last_stuck_head = wd_head;
                    last_stuck_tail = wd_tail;
                }
            } else {
                starvation_counter = 0;
                stuck_counter = 0;
                last_queue_head = wd_head;
                last_stuck_head = wd_head;
                last_stuck_tail = wd_tail;
            }
        }

        rt_log_drain();
        // ============================================================
        // cycle 末尾 sleep:
        //   real 模式: 10ms EtherCAT 周期 (与 DC 时钟同步)
        //   sim 模式:  不额外 sleep, cycle 内已由 SIM_RT_SLEEP_US 节流
        //              (此前 sim 模式也走 10ms sleep, 导致 wall-clock = 10ms/cycle
        //               远超虚拟时间 1ms, 38 段队列需 31 分钟才能清空)
        // ============================================================
        if (!g_sim_mode) {
            osal_usleep(10000);
        }
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
