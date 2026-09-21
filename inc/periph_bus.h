#ifndef PERIPH_BUS_H
#define PERIPH_BUS_H

/* =====================================================================
 *  periph_bus.h —— P0-1 CNC↔外围 G/F 信号表 (软 PLC 地基)
 *
 *  定位 (2026-09-09 内置软 PLC 规划 P0):
 *    对标 Fanuc G/F 信号接口, 在 RT 内核与 (未来的) 软 PLC / 外围管理
 *    进程之间建立一张双向信号表。P0 阶段只落"表 + G 区发布"——
 *    互锁链仍由 laser_rt_safety_gate 在 RT 内数据驱动评估;
 *    P1 软 PLC 线程上线后, F 区由 PLC 写入, G 区成为 PLC 的唯一输入。
 *
 *  方向约定 (与 Fanuc 一致):
 *    G 区 (CNC → 外围):  RT 线程每 1ms 发布 — 循环状态/模态命令镜像/
 *                        M 代码选通 (strobe)。外围侧只读。
 *    F 区 (外围 → CNC):  外围管理侧 (P1 = 软 PLC 线程) 单写者,
 *                        RT / 后台 acquire 读。P0 阶段无写者, 全 0。
 *
 *  并发模型:
 *    G 区: Seqlock 单缓冲 (与 snapshot_hub 同模式)
 *          单写者 = 1ms Hard-RT Thread; 多读者 = Non-RT。
 *          seq 奇数=写中, 偶数=稳定; release/acquire 序 (ARM-safe,
 *          已按 cnc-arm-port-plan 审计结论选型, 不用 seq_cst)。
 *    F 区: 字段级 _Atomic (单写者 release store, 读者 acquire load)。
 *          P1 的 PLC 线程是唯一写者; 不做双缓冲 (字段独立, 无需
 *          跨字段一致性快照 — 慢速外围状态天然最终一致)。
 *
 *  红线 (继承软 PLC 规划 R2):
 *    F 区的任何位都【不直接】驱动 PDO — 外设 DO 必须过 RT 内核
 *    veto 层 (安全链 AND) 才输出。G 区只镜像状态, 不承载安全判定。
 * ===================================================================== */

#include <stdint.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 帧标识 ---- */
#define PERIPH_BUS_MAGIC     0x50455247u   /* "PERG" little-endian */
#define PERIPH_BUS_VERSION   2u            /* v2 (2026-09-14, P1.5-1): +ai[4] 模拟输入工程量
                                            * v1: 初版 */

/* ---- G 区 flags 位定义 ---- */
#define PERIPH_G_FLAG_RUNNING     0x00000001u  /* g_interpolator.is_moving */
#define PERIPH_G_FLAG_HOLD        0x00000002u  /* hold_state != HOLD_NORMAL */
#define PERIPH_G_FLAG_ALARM       0x00000004u  /* g_sys_alarm_state != 0 */
#define PERIPH_G_FLAG_PAUSED      0x00000008u  /* parser is_paused */
#define PERIPH_G_FLAG_WAIT_MCODE  0x00000010u  /* is_waiting_mcode */
#define PERIPH_G_FLAG_LASER_KILL  0x00000020u  /* g_laser_rt.emergency_kill */

#pragma pack(push, 1)

/* ---- G 区 (CNC → 外围, RT 单写者, ~56B) ----
 * 字段全部镜像自 RT 单写者状态 (g_interpolator / g_laser_rt),
 * 与 SMC_Snapshot_t 同源但面向外围管理: 只保留外围关心的聚合量,
 * 不含位置/坐标系 (那是 UI 的事, 走 9528 snapshot)。 */
typedef struct {
    uint32_t magic;             /* PERIPH_BUS_MAGIC */
    uint32_t version;           /* PERIPH_BUS_VERSION */
    uint64_t g_seq;             /* 发布序号 (每次 +2, 偶数=稳定) */
    uint32_t cycle;             /* RT 周期计数 (硬时间锚点) */
    uint32_t flags;             /* PERIPH_G_FLAG_* */

    /* M 代码选通: PLC 域 M 码 (P1 预留 M80-M89) 经此镜像;
     * v1 仅镜像现有等待态 (current_mcode / is_waiting_mcode),
     * P1 握手语义 (strobe/FIN) 在此字段上叠加, 不改布局。 */
    int32_t  mcode_value;       /* g_interpolator.current_mcode */
    int32_t  mcode_strobe;      /* g_interpolator.is_waiting_mcode */

    /* 模态命令镜像 (外围侧据此驱动非运动设备的状态灯/联动, 不驱动功率) */
    int32_t  spindle_mode;      /* eff_spindle_mode_rt (0=M5, 1=M3, 2=M4) */
    int32_t  coolant_state;     /* eff_coolant_state_rt (bit0=flood bit1=mist) */
    int32_t  laser_gate;        /* eff_laser_enable_rt && shutter 聚合 (1=出光中) */

    /* 激光安全态镜像 (外围侧报警灯/蜂鸣器联动用) */
    uint16_t laser_interlock;   /* g_laser_rt.interlock_status 位图 */
    uint16_t _pad16;

    /* 时间 (外围侧慢速逻辑的统一时基) */
    double   uptime_ms;         /* cycle * 1ms */

    /* P1.5-1 (2026-09-14): AI 模拟输入工程量 (RT 周期采样, PLC 经 G_AI0-3 读)。
     * 数据链: periph_ai_rt_poll (RT) → g_periph_ai_rt → 本字段 → PLC G_AIx。
     * 未配置通道恒 0.0; 量程换算在 RT 侧完成 (raw 0-65535 → 0..fs)。 */
    double   ai[4];
} PeriphG_t;

#pragma pack(pop)

/* ---- F 区 (外围 → CNC, P1 软 PLC 单写者) ----
 * P0 无写者: 全 0 语义 = "无外围管理侧在线", RT 侧不得据此做安全判定。
 * 字段独立 atomic, 无需整体快照。 */
typedef struct {
    _Atomic uint32_t heartbeat;       /* PLC 扫描周期计数 (P1 看门狗数据源) */
    _Atomic uint16_t peripheral_ok;   /* bit0=chiller bit1=dust bit2=lube bit3=door_ind
                                       * bit4=gas_supply bit5..15 预留 (1=正常) */
    _Atomic uint16_t mcode_fin;       /* PLC 域 M 码完成应答 (P1: 码值回打) */
    _Atomic uint16_t f_status;        /* P1-2: F_STATUS 系统变量 (用户自由状态位, 仅 watch) */
    _Atomic uint16_t f_version;       /* 写侧协议版本 (0=无人写) */
} PeriphF_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime (main 启动早期, RT 线程启动前)
 * @Safe: 静态清零 + atomic init。幂等。 */
int  PeriphBus_Init(void);

/* @Context: 1ms Hard-RT Thread (ecat_thread_rt cycle 尾, SnapshotHub_Publish 之后)
 * @Danger: 仅 atomic store + 字段赋值。NO BLOCKING / NO PRINTF / NO MALLOC / NO Math.h。
 *          源字段全部 RT 单写者, 无撕裂。 */
void PeriphBus_PublishG(uint32_t cycle);

/* @Context: Non-RealTime Background Thread (P1 PLC 线程 / 诊断工具)
 * @Safe: memcpy + atomic load, 可阻塞。
 * @return 0=稳定读; -1=写者持续忙 (1000 次重试失败, *out 为 best-effort 旧值)。 */
int  PeriphBus_ReadG(PeriphG_t *out);

/* @Context: Non-RealTime (P1 软 PLC 扫描周期尾; P0 无调用方)
 * @Thread-Safety: F 区设计单写者 (PLC 线程); 多写者属契约违反。 */
static inline void PeriphBus_WriteF_Heartbeat(PeriphF_t *f, uint32_t v)
{
    atomic_store_explicit(&f->heartbeat, v, memory_order_release);
}
static inline void PeriphBus_WriteF_PeripheralOk(PeriphF_t *f, uint16_t v)
{
    atomic_store_explicit(&f->peripheral_ok, v, memory_order_release);
}

/* @Context: 任意线程 (RT best-effort / 后台)
 * @return F 区单例指针 (字段级 acquire 读, 无快照语义)。 */
const PeriphF_t *PeriphBus_GetF(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PERIPH_BUS_H */
