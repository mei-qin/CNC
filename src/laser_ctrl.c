// =====================================================================
// laser_ctrl.c — 激光切割辅助子系统实现 (Phase A 安全地基)
// =====================================================================
//
// 线程安全模型 (与 queue_spinlock / RT 单写者架构一致):
//   - g_laser_cfg:  主线程 init 阶段单写者, RT 线程只读
//   - g_laser_rt:   RT 线程 (ecat_thread_rt) 唯一写者, 后台 HMI/trace 只读
//   - 状态推进链路:
//       parser → g_state.laser_*_pending
//             → seg.aux_laser_*           (axis_ctrl.c:563 段入队快照)
//             → g_interpolator.laser_*_rt (ecat_core.c 段消费环, RT 单写者)
//             → g_laser_rt                (laser_rt_apply_aux, RT 单写者)
//             → PDO DO/AO                 (laser_rt_flush_pdos, RT 单写者)
//
// sim 模式行为:
//   - safety_gate 仅检查 g_sys_alarm_state (DI 检查跳过)
//   - flush_pdos 不输出 PDO (g_laser_rt 状态保留供 trace_logger 读)
// =====================================================================

#include "laser_ctrl.h"
#include "global_def.h"
#include <stdatomic.h>

// 全局实例定义 (单一定义点, global_def.h extern 引用)
LaserConfig_t  g_laser_cfg;
LaserRTState_t g_laser_rt;

// =====================================================================
// 硬件抽象层 (HAL) — Phase A 简化 stub
// =====================================================================
// 真实硬件阶段需根据具体 EtherCAT I/O 模块 (如 Beckhoff EL2809/EL4134
// 或国产等同模块) 的 PDO 布局重写这 3 个函数。
//
// sim 模式 + slave_id<0 时这 3 个函数都是 no-op, 但 g_laser_rt 状态
// 仍正确推进, 供 trace_logger / 仿真 CSV 输出验证。
//
// 硬件重写时的红线:
//   - 仅在 ecat_thread_rt 上下文调用, 必须满足 Hard-RT 约束
//   - 读 DI: 直接从 ctx.slavelist[id].inputs + bit 偏移读 1 字节
//   - 写 DO: 直接写 ctx.slavelist[id].outputs + bit 偏移
//   - 写 AO: 直接写 16-bit 到 ctx.slavelist[id].outputs + 通道偏移
// =====================================================================

// @Context: 1ms Hard-RT Thread
// 读 16-bit DI 状态 (1=该位异常). sim/未配置返回 0 (无异常).
static uint16_t laser_hw_read_di(int slave_id)
{
    if (slave_id < 0 || g_sim_mode) return 0;

    // === 硬件阶段在此插入 PDO 读 ===
    // uint8_t *inputs = ctx.slavelist[slave_id].inputs;
    // return (uint16_t)(inputs[0] | (inputs[1] << 8));
    return 0;
}

// @Context: 1ms Hard-RT Thread
// 写 16-bit DO 状态. sim/未配置为 no-op.
static void laser_hw_write_do(int slave_id, uint16_t val)
{
    if (slave_id < 0 || g_sim_mode) return;

    // === 硬件阶段在此插入 PDO 写 ===
    // uint8_t *outputs = ctx.slavelist[slave_id].outputs;
    // outputs[0] = (uint8_t)(val & 0xFF);
    // outputs[1] = (uint8_t)(val >> 8);
    (void)val;
}

// @Context: 1ms Hard-RT Thread
// 写 16-bit AO 通道. sim/未配置为 no-op.
static void laser_hw_write_ao(int slave_id, uint8_t ch, uint16_t raw)
{
    if (slave_id < 0 || g_sim_mode) return;

    // === 硬件阶段在此插入 PDO 写 ===
    // uint8_t *outputs = ctx.slavelist[slave_id].outputs;
    // outputs[ch * 2]     = (uint8_t)(raw & 0xFF);
    // outputs[ch * 2 + 1] = (uint8_t)(raw >> 8);
    (void)ch; (void)raw;
}

// =====================================================================
// 公共接口实现
// =====================================================================

// @Context: Non-RealTime (SMC_InitAndStart 前, 由 axis_sys_init 调用)
void laser_ctrl_init(void)
{
    g_laser_cfg.do_slave_id = -1;
    g_laser_cfg.ao_slave_id = -1;
    g_laser_cfg.di_slave_id = -1;

    // DO bit 偏移默认 (用户可经 SMC_ConfigLaserDOBits 覆盖)
    g_laser_cfg.bit_laser_enable  = 0;
    g_laser_cfg.bit_laser_shutter = 1;
    g_laser_cfg.bit_gas_n2        = 2;
    g_laser_cfg.bit_gas_o2        = 3;
    g_laser_cfg.bit_gas_air       = 4;
    g_laser_cfg.bit_alarm_lamp    = 5;

    // DI bit 偏移默认
    g_laser_cfg.bit_di_door       = 0;
    g_laser_cfg.bit_di_estop_soft = 1;
    g_laser_cfg.bit_di_laser_alm  = 2;
    g_laser_cfg.bit_di_water_temp = 3;
    g_laser_cfg.bit_di_water_flow = 4;
    g_laser_cfg.bit_di_gas_press  = 5;

    // AO 通道偏移默认
    g_laser_cfg.ch_ao_power = 0;
    g_laser_cfg.ch_ao_freq  = 1;

    // 物理量程默认
    g_laser_cfg.power_max_w = 3000.0;
    g_laser_cfg.freq_max_hz = 5000.0;
    g_laser_cfg.power_min_w = 50.0;

    // Phase B1: 功率-速度耦合默认配置 (_Atomic 字段必须用 atomic_store)
    atomic_store_explicit(&g_laser_cfg.coupling_mode, 0, memory_order_relaxed);
    atomic_store_explicit(&g_laser_cfg.v_thresh_mm_s, 5.0, memory_order_relaxed);
    // 默认表: 线性 v=0→0, v=50 mm/s→1.0 (用户通过 SMC_ConfigLaserCoupleTable 自定义)
    g_laser_cfg.couple_table[0].v_mm_s = 0.0;
    g_laser_cfg.couple_table[0].ratio  = 0.0;
    g_laser_cfg.couple_table[1].v_mm_s = 50.0;
    g_laser_cfg.couple_table[1].ratio  = 1.0;
    g_laser_cfg.couple_table_len = 2;

    // RT 状态归零
    g_laser_rt.enable           = 0;
    g_laser_rt.shutter          = 0;
    g_laser_rt.power_w          = 0.0;
    g_laser_rt.freq_hz          = 0.0;
    g_laser_rt.gas_select       = 0;
    g_laser_rt.emergency_kill   = 0;
    g_laser_rt.interlock_status = 0;
    // Phase B1: P_base / v_actual 归零
    g_laser_rt.P_base_w         = 0.0;
    g_laser_rt.v_actual_mm_s    = 0.0;
    // P0-Laser-Q: 加工统计归零 (跨程序累计, 仅 init 阶段清零一次)
    g_laser_rt.pierce_count     = 0;
    g_laser_rt.laser_on_time_ms = 0;
}

// =====================================================================
// Phase B1: 功率-速度耦合查表 (内联, RT 调用)
// =====================================================================
// @Context: 1ms Hard-RT Thread (laser_rt_coupling_update 内部调)
// @Danger: 顺序查找 O(n), n ≤ 16, 约 100 条指令; 线性插值单次乘除.
// 表语义: 表通常很短, 顺序查找比二分更 cache-friendly.
// 不变量: couple_table_len >= 1 (laser_ctrl_init 保证); v_mm_s 单调不减.
static inline double laser_lookup_ratio(double v_mm_s)
{
    const LaserCouplePoint_t *t = g_laser_cfg.couple_table;
    int n = g_laser_cfg.couple_table_len;
    if (n <= 0) return 1.0;                 // 空表: 保守输出 P_base
    if (n == 1) return t[0].ratio;          // 单点表

    // v 低于表头: 返回表头 ratio (典型为 0, 静止关断)
    if (v_mm_s <= t[0].v_mm_s) return t[0].ratio;
    // v 高于表尾: 返回表尾 ratio (典型为 1.0, 饱和)
    if (v_mm_s >= t[n-1].v_mm_s) return t[n-1].ratio;

    // 顺序查找 + 线性插值
    for (int i = 0; i < n - 1; i++) {
        if (v_mm_s >= t[i].v_mm_s && v_mm_s <= t[i+1].v_mm_s) {
            double dv = t[i+1].v_mm_s - t[i].v_mm_s;
            if (dv < 1e-6) return t[i].ratio;   // 防除零 (重复采样点)
            double alpha = (v_mm_s - t[i].v_mm_s) / dv;
            return t[i].ratio + alpha * (t[i+1].ratio - t[i].ratio);
        }
    }
    return t[n-1].ratio;                    // 不应到达 (逻辑兜底)
}

// @Context: 1ms Hard-RT Thread (ecat_thread_rt cycle 末, flush_pdos 之前)
// @Danger: NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
// 根据 g_interpolator.v_current / v_target + 耦合表, 重算 g_laser_rt.power_w.
// 调用时机: apply_aux 之后 (P_base 已设), flush_pdos 之前 (power_w 输出到 PDO).
// 设计: emergency_kill/耦合关闭/静止时保持 g_laser_rt.power_w = P_base (apply_aux 设的)
void laser_rt_coupling_update(void)
{
    // Phase B1: 改读段级镜像 (coupling_mode_rt / v_thresh_rt), 不再读全局 g_laser_cfg
    // 历史架构 BUG: g_laser_cfg.coupling_mode 在 RT 第一次读前就被 parser 顺序覆盖
    //   (M70 P1 → M70 P0), RT 全程看到 mode=0.
    // 修复: parser 入队时快照 mode 到 seg.aux_laser_coupling_mode, RT 消费段时
    //   laser_rt_sync_config() 同步到 g_laser_rt.coupling_mode_rt. 段执行期间 mode 稳定.
    int    cur_mode    = g_laser_rt.coupling_mode_rt;
    double cur_v_thresh = g_laser_rt.v_thresh_rt;

    // === B1 入口诊断 (验证调用次数 + early return 状态, 验证后删除) ===
    if (g_sim_mode) {
        static int entry_counter = 0;
        if (++entry_counter >= 5000) {
            entry_counter = 0;
            printf("[B1-ENTRY] ekill=%d mode_rt=%d is_moving=%d tbl_len=%d v_thresh_rt=%.2f\n",
                   g_laser_rt.emergency_kill,
                   cur_mode,
                   g_interpolator.is_moving,
                   g_laser_cfg.couple_table_len,
                   cur_v_thresh);
            fflush(stdout);
        }
    }

    if (g_laser_rt.emergency_kill) return;
    if (cur_mode == 0) return;                       // off: 保持 P_base
    if (!g_interpolator.is_moving) return;           // 静止: 保持 P_base (避免 Snap 微段误归零)

    double v_curr_ms = g_interpolator.v_current;     // mm/ms
    double v_curr_mm_s = v_curr_ms * 1000.0;         // 转 mm/s
    g_laser_rt.v_actual_mm_s = v_curr_mm_s;

    // 低速阈值: 避免起弧/穿孔点火前过烧
    if (v_curr_mm_s < cur_v_thresh) {
        g_laser_rt.power_w = 0.0;
        return;
    }

    double ratio = laser_lookup_ratio(v_curr_mm_s);
    g_laser_rt.power_w = g_laser_rt.P_base_w * ratio;

    // === B1 末端诊断 (验证 v_actual 链路, 验证后删除) ===
    if (g_sim_mode) {
        static int diag_counter = 0;
        if (++diag_counter >= 5000) {
            diag_counter = 0;
            printf("[B1-DIAG] mode_rt=%d v_cur=%.6f mm/ms=%.3f mm/s vt=%.6f P_base=%.1f ratio=%.3f P_out=%.1f\n",
                   cur_mode,
                   v_curr_ms, v_curr_mm_s,
                   g_interpolator.v_target,
                   g_laser_rt.P_base_w, ratio, g_laser_rt.power_w);
            fflush(stdout);
        }
    }
}

// @Context: 1ms Hard-RT Thread (EtherCAT, ecat_thread_rt cycle 头)
// @Danger:   NO BLOCKING, NO MATH.H (单次乘除), NO PRINTF, NO MALLOC.
int laser_rt_safety_gate(void)
{
    // 1. 系统 ALM (跟随误差/轴 ALM/EtherCAT wkc 丢失等) → 必关
    //    g_sys_alarm_state 是 _Atomic int, 任何后台线程 release 写都能被 acquire 读到
    if (atomic_load_explicit(&g_sys_alarm_state, memory_order_acquire) != 0) {
        if (!g_laser_rt.emergency_kill) {
            laser_emergency_kill(0x8000);  // bit15 = system_alarm
        }
        return 1;
    }

    // 2. DI 互锁检查 (仅硬件模式且 di_slave_id 配置时)
    if (g_laser_cfg.di_slave_id >= 0 && !g_sim_mode) {
        uint16_t di = laser_hw_read_di(g_laser_cfg.di_slave_id);
        uint16_t mask = 0;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_door))       mask |= 0x0001;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_estop_soft)) mask |= 0x0002;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_laser_alm))  mask |= 0x0004;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_water_temp)) mask |= 0x0008;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_water_flow)) mask |= 0x0010;
        if (di & (uint16_t)(1u << g_laser_cfg.bit_di_gas_press))  mask |= 0x0020;
        if (mask) {
            laser_emergency_kill(mask);
            return 1;
        }
    }
    return 0;
}

// @Context: 1ms Hard-RT Thread (段消费环 seg 拷出后立即调)
// @Danger:   NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
void laser_rt_apply_aux(const TrajectorySegment_t *seg)
{
    // emergency_kill 锁存时, 任何 seg 推进都被屏蔽
    // 必须显式 laser_rt_reset() (alarm_reset 路径) 才能再开激光
    if (g_laser_rt.emergency_kill) return;

    g_laser_rt.enable     = seg->aux_laser_enable;
    g_laser_rt.shutter    = seg->aux_laser_shutter;
    g_laser_rt.power_w    = seg->aux_laser_power_w;   // 当前输出功率 (耦合 update 会覆盖)
    g_laser_rt.P_base_w   = seg->aux_laser_power_w;   // 基准功率 (耦合 update 的参考)
    g_laser_rt.freq_hz    = seg->aux_laser_freq_hz;
    g_laser_rt.gas_select = seg->aux_gas_select;
}

// @Context: 1ms Hard-RT Thread (ecat_thread_rt cycle 尾, ecx_send_processdata 前)
// @Danger:   NO BLOCKING, NO PRINTF. 浮点仅单次乘除 (不走 math.h libcall).
void laser_rt_flush_pdos(void)
{
    // sim 模式或未配置 DO 从站: 不触 PDO, g_laser_rt 状态保留供 trace 读
    if (g_sim_mode) return;
    if (g_laser_cfg.do_slave_id < 0) return;

    // 1. DO 输出: 16-bit 组合
    // can_emit: 激光使能且未急停 — 控制 enable DO + AO 输出
    int can_emit = (g_laser_rt.enable && !g_laser_rt.emergency_kill);

    uint16_t do_val = 0;
    if (can_emit) {
        do_val |= (uint16_t)(1u << g_laser_cfg.bit_laser_enable);
    }
    if (g_laser_rt.shutter && !g_laser_rt.emergency_kill) {
        do_val |= (uint16_t)(1u << g_laser_cfg.bit_laser_shutter);
    }
    // 气阀: gas_select 决定哪一路接通 (互斥, M10/M11/M12 切换)
    if (can_emit) {
        if (g_laser_rt.gas_select == 1) do_val |= (uint16_t)(1u << g_laser_cfg.bit_gas_n2);
        else if (g_laser_rt.gas_select == 2) do_val |= (uint16_t)(1u << g_laser_cfg.bit_gas_o2);
        else if (g_laser_rt.gas_select == 3) do_val |= (uint16_t)(1u << g_laser_cfg.bit_gas_air);
    }
    if (g_laser_rt.emergency_kill) {
        do_val |= (uint16_t)(1u << g_laser_cfg.bit_alarm_lamp);
    }
    laser_hw_write_do(g_laser_cfg.do_slave_id, do_val);

    // 2. AO 输出: 功率 / 频率 (0-65535 = 0-10V)
    if (g_laser_cfg.ao_slave_id >= 0) {
        uint16_t power_raw = 0;
        uint16_t freq_raw  = 0;
        if (can_emit) {
            if (g_laser_cfg.power_max_w > 0.0) {
                // 整数化前用 double 计算精度足够 (power_w * 65535 / 3000 ≤ 65535 * 1)
                double scaled = g_laser_rt.power_w * 65535.0 / g_laser_cfg.power_max_w;
                if (scaled <= 0.0) power_raw = 0;
                else if (scaled >= 65535.0) power_raw = 65535;
                else power_raw = (uint16_t)(scaled + 0.5);
            }
            if (g_laser_cfg.freq_max_hz > 0.0) {
                double scaled = g_laser_rt.freq_hz * 65535.0 / g_laser_cfg.freq_max_hz;
                if (scaled <= 0.0) freq_raw = 0;
                else if (scaled >= 65535.0) freq_raw = 65535;
                else freq_raw = (uint16_t)(scaled + 0.5);
            }
        }
        laser_hw_write_ao(g_laser_cfg.ao_slave_id, g_laser_cfg.ch_ao_power, power_raw);
        laser_hw_write_ao(g_laser_cfg.ao_slave_id, g_laser_cfg.ch_ao_freq,  freq_raw);
    }
}

// @Context: 1ms Hard-RT Thread (safety_gate 内部 / 报警路径直调)
void laser_emergency_kill(uint16_t interlock_mask)
{
    g_laser_rt.emergency_kill   = 1;
    g_laser_rt.enable           = 0;
    g_laser_rt.shutter          = 0;
    g_laser_rt.power_w          = 0.0;
    g_laser_rt.freq_hz          = 0.0;
    // 注: gas_select 不清, flush_pdos 中根据 enable=0 自动关闭气阀 DO
    g_laser_rt.interlock_status = interlock_mask;
}

// Phase B1: 段级耦合配置同步
// 每段消费时调 (cmd_type 分支前), 把入队时快照的 mode/v_thresh 同步到 RT 镜像
void laser_rt_sync_config(const TrajectorySegment_t *seg)
{
    g_laser_rt.coupling_mode_rt = seg->aux_laser_coupling_mode;
    g_laser_rt.v_thresh_rt      = seg->aux_laser_v_thresh;
}

// @Context: 1ms Hard-RT Thread (alarm_reset 路径调)
void laser_rt_reset(void)
{
    g_laser_rt.emergency_kill   = 0;
    g_laser_rt.interlock_status = 0;
    // 不自动重开激光 — 用户需显式重发 M3 (避免报警复位后突然出激光)
}
