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
#include "event_logger.h"   // P0-3c: EventLogger_Push (RT 安全, 无锁无 malloc)
#include <stdatomic.h>

// 全局实例定义 (单一定义点, global_def.h extern 引用)
LaserConfig_t  g_laser_cfg;
LaserRTState_t g_laser_rt;

// ---- P0-3a (2026-09-09): sim DI 注入通道 ----
// 写者: RPC 0x0073 handler (Non-RT, laser_sim_inject_di) — release
// 读者: RT safety_gate → laser_hw_read_di — acquire
static _Atomic uint16_t g_laser_sim_di_pattern = 0;
static _Atomic int      g_laser_sim_di_en      = 0;

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
// 读 DI 状态 (1=该位异常), 宽度 di_bytes (1/2 字节).
// P0-3a (2026-09-09): sim 模式下若 RPC 0x0073 注入使能, 返回注入图案 —
//   互锁链在 WSL2 sim 下可测 (原设计 sim 恒 0 = 互锁零测试覆盖).
// P1.5-2 (2026-09-15): 0x0073 去 sim-only — 实机即 DI 强制 (调机隔离故障点),
//   强制优先级最高 (覆盖 PDO 物理读数); 实机开强制推事件 0x0015 审计。
// P0-4a (2026-09-10): 硬件路径实装, 参数化 {slave_id, di_byte_off, di_bytes};
//   Ibytes 越界钳制 — SOEM IOmap 连续布局, 读越本从站 = 读到邻站数据。
static uint16_t laser_hw_read_di(int slave_id)
{
    /* 强制/注入优先 (sim = 传感器模拟; 实机 = DI force 调机工具) */
    if (atomic_load_explicit(&g_laser_sim_di_en, memory_order_acquire))
        return atomic_load_explicit(&g_laser_sim_di_pattern, memory_order_acquire);

    if (g_sim_mode) return 0;   /* sim 未注入 = 恒 0 */
    if (slave_id < 0 || slave_id > ctx.slavecount) return 0;

    uint8_t *inputs = ctx.slavelist[slave_id].inputs;
    if (inputs == NULL) return 0;
    if (g_laser_cfg.di_byte_off + g_laser_cfg.di_bytes
        > (int)ctx.slavelist[slave_id].Ibytes) return 0;   // 越界 = 配置错误, 视无异常

    uint16_t v = inputs[g_laser_cfg.di_byte_off];
    if (g_laser_cfg.di_bytes >= 2)
        v |= (uint16_t)(inputs[g_laser_cfg.di_byte_off + 1] << 8);
    return v;
}

// @Context: 1ms Hard-RT Thread
// 写 DO 状态, 宽度 do_bytes (1/2 字节). sim/未配置为 no-op.
// P0-4a: 参数化 {do_byte_off, do_bytes}; Obytes 越界钳制 (8 点端子只写 1 字节,
//   多写的字节会落入 IOmap 中下一从站的输出区)。
static void laser_hw_write_do(int slave_id, uint16_t val)
{
    if (slave_id < 0 || slave_id > ctx.slavecount) return;
    if (g_sim_mode) return;

    uint8_t *outputs = ctx.slavelist[slave_id].outputs;
    if (outputs == NULL) return;
    if (g_laser_cfg.do_byte_off + g_laser_cfg.do_bytes
        > (int)ctx.slavelist[slave_id].Obytes) return;     // 越界 = 配置错误, 整体不写

    outputs[g_laser_cfg.do_byte_off] = (uint8_t)(val & 0xFF);
    if (g_laser_cfg.do_bytes >= 2)
        outputs[g_laser_cfg.do_byte_off + 1] = (uint8_t)(val >> 8);
}

// @Context: 1ms Hard-RT Thread
// 写 16-bit AO 通道. sim/未配置为 no-op.
// P0-4a: 参数化 {ao_byte_off, ao_stride}; 通道地址 = ao_byte_off + ch × ao_stride。
static void laser_hw_write_ao(int slave_id, uint8_t ch, uint16_t raw)
{
    if (slave_id < 0 || slave_id > ctx.slavecount) return;
    if (g_sim_mode) return;

    uint8_t *outputs = ctx.slavelist[slave_id].outputs;
    if (outputs == NULL) return;
    int off = g_laser_cfg.ao_byte_off + (int)ch * g_laser_cfg.ao_stride;
    if (off + 2 > (int)ctx.slavelist[slave_id].Obytes) return;   // 越界钳制

    outputs[off]     = (uint8_t)(raw & 0xFF);
    outputs[off + 1] = (uint8_t)(raw >> 8);
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
    // P0-4a (2026-09-10): PDO 布局默认 = EL 级 16-bit 端子 (EL1809/EL2809/EL4132 类)
    // 换端子不改代码, 只改 hw 档案 [laser_io] 或 RPC 0x0075
    g_laser_cfg.di_byte_off = 0;
    g_laser_cfg.di_bytes    = 2;
    g_laser_cfg.do_byte_off = 0;
    g_laser_cfg.do_bytes    = 2;
    g_laser_cfg.ao_byte_off = 0;
    g_laser_cfg.ao_stride   = 2;
    // P0-2: 气压通道默认 2 (功率 0 / 频率 1 之后); 量程 0 = 未配置不输出
    g_laser_cfg.ch_ao_pressure  = 2;
    g_laser_cfg.pressure_max_bar = 0.0;

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

    // P1-5: 微连接功率系数默认 1.0 (不衰减, 兼容既有程序)
    atomic_store_explicit(&g_laser_cfg.mj_ratio, 1.0, memory_order_release);

    // P1-4: 渐进穿孔默认表 1 级 (全功率 500ms — 与 G04+M67 组合的既有语义等价)
    g_laser_cfg.pierce_table[0].power_w  = 3000.0;
    g_laser_cfg.pierce_table[0].freq_hz  = 2000.0;
    g_laser_cfg.pierce_table[0].dwell_ms = 500;
    g_laser_cfg.pierce_table_len = 1;

    // P0-2 (2026-09-09): 互锁规则表默认空 = 默认 6 路链 (行为兼容);
    // hw 参数表 [laser_interlock] 或 RPC 0x0074 配置覆盖
    g_laser_cfg.il_count = 0;

    // RT 状态归零
    g_laser_rt.enable           = 0;
    g_laser_rt.shutter          = 0;
    g_laser_rt.power_w          = 0.0;
    g_laser_rt.freq_hz          = 0.0;
    g_laser_rt.gas_select       = 0;
    g_laser_rt.gas_pressure_bar = 0.0;   // P0-2: 气压设定归零
    g_laser_rt.emergency_kill   = 0;
    g_laser_rt.interlock_status = 0;
    // Phase B1: P_base / v_actual 归零
    g_laser_rt.P_base_w         = 0.0;
    g_laser_rt.v_actual_mm_s    = 0.0;
    // P0-Laser-Q: 加工统计归零 (跨程序累计, 仅 init 阶段清零一次)
    g_laser_rt.pierce_count     = 0;
    g_laser_rt.laser_on_time_ms = 0;
    // P0-1: feedhold 光闸 gate 归零 (运行期由 HOLD 状态机每 cycle 重写, 此处仅初始化语义)
    g_laser_rt.hold_gate_off    = 0;
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

    // P1-5 (2026-09-03): 微连接段功率系数
    // SEG_FLAG_MICRO_JOINT 置位段 (M74/M75 包裹): ratio × mj_ratio → 降功率不切穿。
    // 引线段 (bit0) 不调 — 引线在废料区, 保持全功率。
    if (g_laser_rt.current_seg_flags & SEG_FLAG_MICRO_JOINT) {
        ratio *= g_laser_rt.mj_ratio_rt;
        if (ratio > 1.0) ratio = 1.0;
        if (ratio < 0.0) ratio = 0.0;
    }
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

    // 2. DI 互锁检查
    //    硬件模式: di_slave_id >= 0 且已配置时读 PDO DI
    //    sim 模式: RPC 0x0073 注入使能时读注入图案 (P0-3a, 补互锁测试盲区)
    int di_check_active;
    if (!g_sim_mode) {
        di_check_active = (g_laser_cfg.di_slave_id >= 0);
    } else {
        di_check_active = (atomic_load_explicit(&g_laser_sim_di_en,
                                                memory_order_acquire) != 0);
    }
    if (di_check_active) {
        uint16_t di = laser_hw_read_di(g_laser_cfg.di_slave_id);
        uint16_t mask = 0;

        // P0-2 (2026-09-09): 互锁链规则表驱动 (hw 参数表 [laser_interlock] / RPC 0x0074)
        // il_count == 0 回退默认 6 路链 (bit_di_* 固定映射, 兼容既有行为)
        if (g_laser_cfg.il_count > 0) {
            int n = g_laser_cfg.il_count;
            if (n > LASER_IL_MAX) n = LASER_IL_MAX;   // 越界钳制 (规则表源已校验, 双保险)
            for (int i = 0; i < n; i++) {
                if (di & (uint16_t)(1u << g_laser_cfg.il_bit[i]))
                    mask |= g_laser_cfg.il_mask[i];
            }
        } else {
            // 默认链 (硬编码兼容路径): 门/软急停/激光ALM/水温/水流/气压
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_door))       mask |= 0x0001;
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_estop_soft)) mask |= 0x0002;
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_laser_alm))  mask |= 0x0004;
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_water_temp)) mask |= 0x0008;
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_water_flow)) mask |= 0x0010;
            if (di & (uint16_t)(1u << g_laser_cfg.bit_di_gas_press))  mask |= 0x0020;
        }

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
    // P0-2: M13 辅助气压 (仅 M 段同步, 运动段模态保持)
    g_laser_rt.gas_pressure_bar = seg->aux_laser_gas_pressure;
}

// @Context: 1ms Hard-RT Thread (M76 段加载 + 等待块台阶推进)
// @Danger:   仅数组读 + 字段赋值 (无 math.h/阻塞/malloc/printf)
// P1-4 (2026-09-03): 写入第 idx 级穿孔台阶 (功率/频率)。
// 穿孔期 is_moving=0 → coupling_update 直接 return, 台阶值稳定输出 (与 M64 同机理);
// 整体完成时由等待块恢复 g_interpolator.laser_*_w_rt 段快照 (切割参数)。
void laser_rt_apply_pierce_stage(int idx)
{
    if (idx < 0 || idx >= g_laser_cfg.pierce_table_len) return;
    g_laser_rt.P_base_w = g_laser_cfg.pierce_table[idx].power_w;
    g_laser_rt.power_w  = g_laser_cfg.pierce_table[idx].power_w;
    g_laser_rt.freq_hz  = g_laser_cfg.pierce_table[idx].freq_hz;
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
    if (g_laser_rt.shutter && !g_laser_rt.emergency_kill && !g_laser_rt.hold_gate_off) {
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

    // 3. P0-2: 气压 AO (第 3 路, 比例阀 0-10V)
    // 刻意不受 can_emit 门 — 切割工艺要求关光后吹气继续 (冷却/防渣堵嘴)。
    // 仅量程门 (pressure_max_bar<=0 = 未配置不输出) + 上下 clamp, 单次乘除无 libcall。
    if (g_laser_cfg.ao_slave_id >= 0 && g_laser_cfg.pressure_max_bar > 0.0) {
        double scaled_p = g_laser_rt.gas_pressure_bar * 65535.0
                          / g_laser_cfg.pressure_max_bar;
        uint16_t pressure_raw;
        if (scaled_p <= 0.0)        pressure_raw = 0;
        else if (scaled_p >= 65535.0) pressure_raw = 65535;
        else                        pressure_raw = (uint16_t)(scaled_p + 0.5);
        laser_hw_write_ao(g_laser_cfg.ao_slave_id, g_laser_cfg.ch_ao_pressure,
                          pressure_raw);
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
    // P1-5: seg_flags + 微连接系数段级镜像 (M 段/运动段统一, 单点覆盖 ecat_core.c:977)
    g_laser_rt.current_seg_flags = seg->seg_flags;
    g_laser_rt.mj_ratio_rt       = seg->aux_laser_mj_ratio;
}

// @Context: 1ms Hard-RT Thread (alarm_reset 路径调)
void laser_rt_reset(void)
{
    // P0-3c (2026-09-09): 复位审计事件 — 仅实际解除锁存时推 (防重复复位噪音);
    // value 携带复位前的 interlock 位图, 与触发事件 0x0010 的 value 闭环对照
    if (g_laser_rt.emergency_kill) {
        EventLogger_Push(SEVERITY_INFO, SOURCE_LASER, 0x0014,
                         (int32_t)g_laser_rt.interlock_status,
                         "laser interlock reset (re-armed; M3 required)");
    }
    g_laser_rt.emergency_kill   = 0;
    g_laser_rt.interlock_status = 0;
    // 不自动重开激光 — 用户需显式重发 M3 (避免报警复位后突然出激光)
}

// @Context: Non-RealTime (RPC 0x0073 handler 线程)
// @Thread-Safety: 字段级 atomic, RT safety_gate acquire 读。
void laser_sim_inject_di(uint16_t pattern, int enable)
{
    atomic_store_explicit(&g_laser_sim_di_pattern, pattern, memory_order_release);
    atomic_store_explicit(&g_laser_sim_di_en, enable ? 1 : 0, memory_order_release);
}
