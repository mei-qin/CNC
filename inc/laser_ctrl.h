#ifndef LASER_CTRL_H
#define LASER_CTRL_H

// =====================================================================
// laser_ctrl — 激光切割辅助子系统 (Phase A 安全地基)
// =====================================================================
//
// 设计目标 (Phase A 范围):
//   1. 急停硬关断: 任何互锁 DI / 系统报警 → 1 cycle 内拉低激光 DO + AO
//   2. 同步 M 代码: M62/M63 激光闸 / M67 功率 / M68 频率 / M10/M11/M12 气体三选
//   3. 段边界对齐: 激光开/关与运动段 1ms 边界同步, 杜绝段间隙过烧
//
// 推迟到 Phase B (本头文件不涉及):
//   - 激光功率-速度耦合 (P = base × v/v_max)
//   - G04 dwell 穿孔
//   - Z 轴电容跟随 PID
//
// 线程模型 (与项目 queue_spinlock / RT 单写者架构一致):
//   ✦ 主线程 (main / SMC_API): 配置 g_laser_cfg 字段 (init 阶段, 单写者)
//   ✦ Parser 线程: 间接经 g_state.laser_*_pending → seg.aux_laser_* 链路
//   ✦ RT 线程 (ecat_thread_rt): g_laser_rt 唯一写者
//     - cycle 头调 laser_rt_safety_gate() — 检查互锁
//     - 段消费调 laser_rt_apply_aux(seg) — 推进激光状态
//     - cycle 尾调 laser_rt_flush_pdos() — 写物理 DO/AO
//
// 关键不变量:
//   - emergency_kill 一旦置 1, 必须显式 laser_rt_reset() 才能再开激光
//     (HMI 报警复位路径会调)
//   - g_laser_rt.enable/shutter/power_w 在 emergency_kill=1 时强制为 0/0/0
//     (无论 parser 推来什么 seg, 都会被 safety_gate 在 cycle 头覆盖)
// =====================================================================

#include <stdint.h>
#include "axis_cfg.h"   // TrajectorySegment_t / _Atomic
#include "smc_protocol.h"  // LaserCouplePoint_t + LASER_COUPLE_TABLE_MAX (P0-Laser-ConfigRPC 移入协议头供 SDK 共享)

// =====================================================================
// P0-Laser Phase B1: 功率-速度耦合 (分段查表) 表语义说明
// =====================================================================
// 表语义: 绝对速度表, 输入 v_current (mm/s), 输出 ratio (0-1)
//   ratio = laser_lookup_ratio(v_current)
//   g_laser_rt.power_w = g_laser_rt.P_base_w * ratio
// 表约束: v_mm_s 单调不减, ratio 在 [0, 1] 内, 长度 1..LASER_COUPLE_TABLE_MAX
// 默认表 (laser_ctrl_init 设置): 线性, v=0→0, v=50 mm/s→1.0
// 用户通过 SMC_ConfigLaserCoupleTable 自定义 (典型: 起弧/切割/饱和 三段)
// 注: LaserCouplePoint_t / LASER_COUPLE_TABLE_MAX 定义已移到 smc_protocol.h (P0-Laser-ConfigRPC)
// =====================================================================

typedef struct {
    // ---- EtherCAT 从站索引 (-1 = 该通道未配置, RT 跳过) ----
    int do_slave_id;            // 数字输出从站 (16-bit)
    int ao_slave_id;            // 模拟输出从站 (2-ch 16-bit)
    int di_slave_id;            // 安全互锁输入从站 (16-bit)

    // ---- DO bit 偏移 (0-15) ----
    uint8_t bit_laser_enable;   // 激光器主使能 (软控 + 硬线 INTERLOCK 双重)
    uint8_t bit_laser_shutter;  // 激光闸 (M62/M63 同步)
    uint8_t bit_gas_n2;         // M10 N2 阀
    uint8_t bit_gas_o2;         // M11 O2 阀
    uint8_t bit_gas_air;        // M12 Air 阀
    uint8_t bit_alarm_lamp;     // 报警灯 (emergency_kill=1 时亮)

    // ---- DI bit 偏移 (0-15) — 任一异常立即关激光 + 软停机 ----
    // 注: 急停按钮/门禁另走激光器外部硬线 INTERLOCK, 这里 DI 仅作软件感知
    uint8_t bit_di_door;        // 门禁互锁 (开 = 异常)
    uint8_t bit_di_estop_soft;  // 急停软线
    uint8_t bit_di_laser_alm;   // 激光器 ALM 触点
    uint8_t bit_di_water_temp;  // 水温异常
    uint8_t bit_di_water_flow;  // 水流不足
    uint8_t bit_di_gas_press;   // 气压异常

    // ---- AO 通道偏移 ----
    uint8_t ch_ao_power;        // 功率通道 (0-10V, 0-65535 = 0-power_max_w)
    uint8_t ch_ao_freq;         // 频率通道 (0-10V, 0-65535 = 0-freq_max_hz)

    // ---- 物理量程 ----
    double  power_max_w;        // 默认 3000.0
    double  freq_max_hz;        // 默认 5000.0
    double  power_min_w;        // 起辉功率下限 (默认 50.0, 暂未在 RT 强制)

    // ---- Phase B1: 功率-速度耦合 (分段查表) ----
    // coupling_mode 跨线程: parser (M70) / SMC_API 写, RT (coupling_update) 读
    // _Atomic volatile 双重保护:
    //   _Atomic   保证原子性 + C11 内存模型 (release/acquire happens-before)
    //   volatile  强制编译器每次访问都从内存读取 (禁止 register 缓存, 防 -O2 提升)
    // (历史 BUG: 仅 _Atomic 时 RT 仍读到旧值 0, addr 相同 — 怀疑 gcc -O2 跨循环提升 load)
    _Atomic volatile int coupling_mode;        // 0=off (默认), 1=查表耦合
    _Atomic volatile double v_thresh_mm_s;     // 低速阈值 (mm/s)
    LaserCouplePoint_t couple_table[LASER_COUPLE_TABLE_MAX];  // 查表采样点 (init 阶段配置, RT 只读)
    int    couple_table_len;    // 实际采样点数 (1..LASER_COUPLE_TABLE_MAX, init 阶段配置)
} LaserConfig_t;

typedef struct {
    // ---- RT 单写者镜像状态 (RT 线程消费 seg 时同步) ----
    int    enable;              // 激光器主使能 (M3=1, M5=0)
    int    shutter;             // 激光闸 (M62=1, M63=0)
    double power_w;             // 当前功率 (W)
    double freq_hz;             // 当前频率 (Hz)
    int    gas_select;          // 0=off, 1=N2, 2=O2, 3=Air

    // ---- 加工统计 (RT 单写者累计, HMI 只读) ----
    // pierce_count: M64 段 (G04 dwell) 完成时 ++, 跨程序不清零 (累计量)
    // laser_on_time_ms: enable && shutter && !emergency_kill 时每 cycle += 1 (1ms 周期)
    //   64-bit 字段在 32-bit 平台可能撕裂, HMI 容忍 1ms 级滞后 (统计字段非安全关键)
    int32_t  pierce_count;
    int64_t  laser_on_time_ms;

    // ---- 急停锁存 (一旦置 1, 需显式 laser_rt_reset() 才能再开) ----
    int    emergency_kill;
    // ---- 互锁状态位图 (供 HMI 显示是哪个 DI 触发) ----
    //   bit0=door bit1=estop_soft bit2=laser_alm
    //   bit3=water_temp bit4=water_flow bit5=gas_press
    //   bit15=system_alarm (跟随误差/轴 ALM 等经 g_sys_alarm_state 触发)
    uint16_t interlock_status;

    // ---- Phase B1: 功率-速度耦合镜像 ----
    double P_base_w;            // 基准功率 (M67 设置, 不被耦合覆盖, apply_aux 同步)
    double v_actual_mm_s;       // 当前周期瞬时速度 (RT 写, trace/HMI 读)
    // 段级配置镜像 (RT 消费段时从 seg.aux_laser_* 同步, 不再每 cycle 读 g_laser_cfg)
    int    coupling_mode_rt;    // 当前段生效的 coupling mode (0/1)
    double v_thresh_rt;         // 当前段生效的 v_thresh (mm/s)
} LaserRTState_t;

extern LaserConfig_t  g_laser_cfg;
extern LaserRTState_t g_laser_rt;

// =====================================================================
// 接口 (实现于 laser_ctrl.c)
// =====================================================================

// @Context: Non-RealTime (SMC_InitAndStart 前)
// 清零 g_laser_rt, 配置默认量程, do/ao/di slave_id = -1 (未配置)
void laser_ctrl_init(void);

// @Context: 1ms Hard-RT Thread (EtherCAT, ecat_thread_rt cycle 头)
// @Danger:   NO BLOCKING, NO MATH.H (单次乘除), NO PRINTF, NO MALLOC.
// 检查 g_sys_alarm_state + DI 互锁位, 异常时调 laser_emergency_kill + 锁存.
// 返回: 0=本周期正常, 1=本周期已触发急停
int  laser_rt_safety_gate(void);

// @Context: 1ms Hard-RT Thread (段消费环, seg 拷出后立即调)
// @Danger:   NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
// 把 seg.aux_laser_* 同步到 g_laser_rt (仅当 emergency_kill == 0 时生效).
// 设计: 即使是运动段 (cmd_type==MOTION) 也调, 让段边界激光开关严格 1ms 对齐.
void laser_rt_apply_aux(const TrajectorySegment_t *seg);

// @Context: 1ms Hard-RT Thread (段消费环, seg 拷出后立即调, cmd_type 分支前)
// @Danger:   仅写 g_laser_rt 字段, 极快.
// Phase B1: 同步段级耦合配置 (coupling_mode_rt + v_thresh_rt).
// 每段都调 (M 段 + 运动段), 保证段执行期间用入队时的 mode, 不受后续 M70 写入影响.
// 设计原因: parser 单线程解析远快于 RT 消费, 若 RT 读全局 g_laser_cfg.coupling_mode,
//   M70 P1 → M70 P0 顺序覆盖会在 RT 第一次读之前完成, RT 全程看到 mode=0.
void laser_rt_sync_config(const TrajectorySegment_t *seg);

// @Context: 1ms Hard-RT Thread (ecat_thread_rt cycle 尾, ecx_send_processdata 前)
// @Danger:   NO BLOCKING, NO PRINTF.
// 把 g_laser_rt 状态写到 DO/AO PDO (sim 模式下仅写 trace, 不触 PDO).
void laser_rt_flush_pdos(void);

// @Context: 1ms Hard-RT Thread (safety_gate 内部调, 或报警路径直调)
// @Danger:   仅写 g_laser_rt 字段, 极快.
// 锁存 emergency_kill=1, 拉低 enable/shutter/power_w, 记录 interlock 位图.
void laser_emergency_kill(uint16_t interlock_mask);

// @Context: 1ms Hard-RT Thread (alarm_reset 路径调)
// 清 emergency_kill 锁存 (但不自动重开激光, 需 M3 重发).
void laser_rt_reset(void);

#endif // LASER_CTRL_H
