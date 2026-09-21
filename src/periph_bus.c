/* =====================================================================
 *  periph_bus.c —— P0-1 CNC↔外围 G/F 信号表实现
 *
 *  设计见 inc/periph_bus.h 头注释。
 *
 *  关键不变量 (与 snapshot_hub 一致):
 *    1. g_pg_seq 奇数 = RT 写者正在写, reader 必须重试
 *    2. g_pg_seq 偶数 = 稳定, reader 可读
 *    3. 每次 publish seq +2, 偶数 seq 回写 payload 的 g_seq 字段
 *    4. Publish 不感知 reader 数量, 不阻塞, 不分配内存
 *
 *  字段来源 (全部 RT 单写者, 无撕裂):
 *    g_interpolator.eff_* / is_waiting_mcode / current_mcode / hold_state
 *    g_laser_rt.interlock_status / emergency_kill
 *    g_sys_alarm_state (_Atomic)
 *    g_parser_ctrl.is_paused (parser 写, best-effort 撕裂容忍 — 同 snapshot)
 * ===================================================================== */

#include "global_def.h"   /* g_interpolator / g_laser_rt / g_sys_alarm_state / g_parser_ctrl
                           * 必须在 periph_bus.h 之前, 与 snapshot_hub.c 同约束 */
#include "periph_bus.h"
#include "periph_ai.h"    /* P1.5-1: g_periph_ai_rt (AI 工程量入 G 区) */
#include <string.h>

/* ---- 静态分配 (RT 禁 malloc) ---- */
static PeriphG_t    g_pg_buf;
static _Atomic uint64_t g_pg_seq = 0;
static PeriphF_t    g_pf;        /* P0 无写者, 全 0 */

/* @Context: Non-RealTime Background Thread (main 启动早期)
 * @Safe: 静态清零 + atomic init。幂等。 */
int PeriphBus_Init(void)
{
    memset(&g_pg_buf, 0, sizeof(g_pg_buf));
    memset(&g_pf, 0, sizeof(g_pf));   /* _Atomic 全 0 = C11 合法初始化 */
    atomic_store_explicit(&g_pg_seq, 0, memory_order_release);
    g_pg_buf.magic   = PERIPH_BUS_MAGIC;
    g_pg_buf.version = PERIPH_BUS_VERSION;
    return 0;
}

/* @Context: 1ms Hard-RT Thread (ecat_thread_rt cycle 尾)
 * @Danger: 仅 atomic store + 字段赋值。NO BLOCKING / NO PRINTF / NO MALLOC / NO Math.h。 */
void PeriphBus_PublishG(uint32_t cycle)
{
    /* ---- Step 1: 标记写中 (seq 奇数) ---- */
    uint64_t s = atomic_load_explicit(&g_pg_seq, memory_order_relaxed);
    atomic_store_explicit(&g_pg_seq, s + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);

    /* ---- Step 2: 写所有字段 ---- */
    PeriphG_t *p = &g_pg_buf;

    p->magic   = PERIPH_BUS_MAGIC;
    p->version = PERIPH_BUS_VERSION;
    p->cycle   = cycle;

    /* flags 位图聚合 (源与 SnapshotHub_Publish 同) */
    uint32_t fl = 0;
    if (atomic_load_explicit(&g_interpolator.is_moving, memory_order_relaxed))
        fl |= PERIPH_G_FLAG_RUNNING;
    if (g_interpolator.hold_state != (int)HOLD_NORMAL)     fl |= PERIPH_G_FLAG_HOLD;
    if (atomic_load_explicit(&g_sys_alarm_state, memory_order_relaxed))
        fl |= PERIPH_G_FLAG_ALARM;
    if (g_parser_ctrl.is_paused)                    fl |= PERIPH_G_FLAG_PAUSED;
    if (g_interpolator.is_waiting_mcode)            fl |= PERIPH_G_FLAG_WAIT_MCODE;
    if (g_laser_rt.emergency_kill)                  fl |= PERIPH_G_FLAG_LASER_KILL;
    p->flags = fl;

    /* M 代码选通镜像 (P1: PLC 域 M80-M89 握手在此叠加) */
    p->mcode_value  = g_interpolator.current_mcode;
    p->mcode_strobe = g_interpolator.is_waiting_mcode;

    /* 模态命令镜像 (eff_* = override/dry_run 之后的有效输出) */
    p->spindle_mode  = g_interpolator.eff_spindle_mode_rt;
    p->coolant_state = g_interpolator.eff_coolant_state_rt;
    p->laser_gate    = (g_interpolator.eff_laser_enable_rt && g_laser_rt.shutter) ? 1 : 0;

    /* 激光安全态镜像 */
    p->laser_interlock = g_laser_rt.interlock_status;

    /* 时基 */
    p->uptime_ms = (double)cycle * 1.0;

    /* P1.5-1: AI 模拟输入工程量 (periph_ai_rt_poll 先行填充) */
    for (int i = 0; i < 4; i++) p->ai[i] = g_periph_ai_rt[i];

    p->g_seq = s + 2;

    /* ---- Step 3: 标记写完成 (seq 偶数) ---- */
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&g_pg_seq, s + 2, memory_order_relaxed);
}

/* @Context: Non-RealTime Background Thread
 * @Safe: memcpy + atomic load, 可阻塞。 */
int PeriphBus_ReadG(PeriphG_t *out)
{
    if (out == NULL) return -1;

    for (int retry = 0; retry < 1000; retry++) {
        uint64_t s1 = atomic_load_explicit(&g_pg_seq, memory_order_acquire);
        if (s1 & 1) continue;          /* 写者正在写, 重试 */
        *out = g_pg_buf;
        atomic_thread_fence(memory_order_acquire);
        uint64_t s2 = atomic_load_explicit(&g_pg_seq, memory_order_acquire);
        if (s1 == s2) return 0;        /* 稳定读 */
    }
    return -1;
}

/* @Context: 任意线程
 * @return F 区单例指针 (字段级 acquire 读, 无快照语义)。 */
const PeriphF_t *PeriphBus_GetF(void)
{
    return &g_pf;
}
