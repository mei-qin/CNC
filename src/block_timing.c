/* ============================================================================
 * block_timing.c — 论文1 S10 (2026-09-20): 控制器内 block 级计时插桩
 *
 * 详见 inc/block_timing.h 顶部设计说明。本文件只放计数器读写,
 * 不含任何阻塞/IO/数学库调用; RT 侧三个函数由 1ms Hard-RT 线程调用。
 * ==========================================================================*/

#include "block_timing.h"
#include <string.h>

BlockTiming_t g_bt;

/* ------------------------------------------------------------------ Non-RT */

void bt_reset_all(void)
{
    memset(&g_bt, 0, sizeof(g_bt));
}

/* 声明"下一个 block 开始生成"。必须在任何 push 之前调用,
 * 因为 block_seg_count 需要在**第一个**微段入队时就已知。 */
void bt_block_open(double cmd_ms, int count, double sum_ms, int line_no)
{
    g_bt.pend_uid++;
    if (g_bt.pend_uid == 0) g_bt.pend_uid = 1;   /* 0 保留 = 未插桩 */
    g_bt.pend_count   = (count > 0) ? (uint32_t)count : 0;
    g_bt.pend_idx     = 0;
    g_bt.pend_line_no = line_no;
    g_bt.pend_cmd_ms  = cmd_ms;
    g_bt.pend_sum_ms  = sum_ms;
    g_bt.pend_active  = 1;
}

void bt_block_close(void)
{
    g_bt.pend_active = 0;
}

/* 由 api_push_trajectory_impl 在段元数据填充处调用: 把当前 block 描述符
 * 逐个盖章到微段上 (段结构按值入队, 无法事后回填)。 */
void bt_stamp_segment(uint32_t *uid, uint32_t *idx, uint32_t *count,
                      double *cmd_ms, double *sum_ms)
{
    if (g_bt.pend_active) {
        *uid    = g_bt.pend_uid;
        *idx    = g_bt.pend_idx;
        *count  = g_bt.pend_count;
        *cmd_ms = g_bt.pend_cmd_ms;
        *sum_ms = g_bt.pend_sum_ms;
        g_bt.pend_idx++;
    } else {
        /* 未插桩路径 (G00 直通 / B-Spline / 刀补引擎): uid=0, RT 不做 block 记账 */
        *uid    = 0;
        *idx    = 0;
        *count  = 0;
        *cmd_ms = 0.0;
        *sum_ms = 0.0;
    }
}

/* --------------------------------------------------------------------- RT */

void bt_rt_block_start(uint32_t uid, int32_t line_no, uint32_t gen,
                       double cmd_ms, double sum_ms, int cycle)
{
    if (g_bt.cur_active && g_bt.cur_uid != uid) {
        g_bt.blocks_closed++;                /* 上一个 block 被顶掉 = 已闭合 */
    }
    g_bt.cur_uid    = uid;
    g_bt.cur_active = 1;

    g_bt.block_uid                 = uid;
    g_bt.block_line_no             = line_no;
    g_bt.block_start_cycle         = (uint64_t)cycle;
    /* 预置: 若本 block 在同一周期内跑完, elapsed 应为 1 而非 0 */
    g_bt.block_end_cycle_exclusive = (uint64_t)cycle + 1;
    g_bt.block_elapsed_cycles      = 1;
    g_bt.segments_generated        = gen;
    g_bt.segments_completed        = 0;
    g_bt.block_budget_cmd_ms       = cmd_ms;
    g_bt.block_budget_sum_ms       = sum_ms;
    g_bt.block_virtual_ms          = 0.0;
    g_bt.block_finish_cycle        = 0;

    if (g_bt.ctrl_start_cycle == 0) {
        g_bt.ctrl_start_cycle = (uint64_t)cycle;   /* 程序首个运动段 */
    }
}

void bt_rt_segment_done(int cycle)
{
    if (!g_bt.cur_active) return;

    g_bt.segments_completed++;
    g_bt.segments_completed_total++;

    /* exclusive-end 约定: 完成所在周期 + 1 */
    g_bt.block_end_cycle_exclusive = (uint64_t)cycle + 1;
    g_bt.block_elapsed_cycles      = g_bt.block_end_cycle_exclusive
                                     - g_bt.block_start_cycle;

    g_bt.ctrl_end_cycle_exclusive  = (uint64_t)cycle + 1;
    g_bt.controller_elapsed_cycles = g_bt.ctrl_end_cycle_exclusive
                                     - g_bt.ctrl_start_cycle;
    /* 末微段完成 ⇒ 记下完成周期, 供 bt_rt_cycle 的"在飞"门控使用 */
    if (g_bt.block_finish_cycle == 0
        && g_bt.segments_completed >= g_bt.segments_generated) {
        g_bt.block_finish_cycle = (uint64_t)cycle;
    }
}

void bt_rt_cycle(int cycle, double virtual_inc_ms, int hold_active,
                 double cycle_time_ms, double eff_scale, int moving)
{
    g_bt.rt_cycle_counter     = (uint64_t)cycle;
    g_bt.virtual_elapsed_ms  += virtual_inc_ms;
    /* ⚠ "block 在飞"门控 (2026-09-20 修): cur_active 在 block 完成后**不会**自动
     * 清 (只有被下一个 block 顶掉才清)。若不额外门控, 程序尾部的空转周期
     * (G94/M30 等无运动周期, virt_inc = 0) 会被全部计进 block_stall_cycles /
     * block_virtual_ms, 使"连续运动 block 的 stall 应为 0"这条预注册判据
     * 必然失败 —— H2 首轮据此每次都判无效并重试 6 次 (实测教训)。
     * 门控: cycle <= block_finish_cycle (末微段完成所在周期仍计入, 之后不再计)。
     * 注意顺序: 段完成回调在 bt_rt_cycle **之前**触发, 故不能用
     * `segments_completed < segments_generated` 直接门控 —— 那会漏掉完成周期
     * 本身的虚拟时间, 使 block_virtual_ms 少最后一个周期。 */
    int inflight = (g_bt.cur_active
                    && (g_bt.block_finish_cycle == 0
                        || (uint64_t)cycle <= g_bt.block_finish_cycle));
    if (inflight) {
        g_bt.block_virtual_ms += virtual_inc_ms;
        if (virtual_inc_ms <= 0.0) g_bt.block_stall_cycles++;
    }
    g_bt.controller_elapsed_ms = (double)g_bt.controller_elapsed_cycles
                                 * cycle_time_ms;
    if (inflight && hold_active) {
        g_bt.hold_active_cycles++;
        g_bt.hold_active_ms += cycle_time_ms;
        if (virtual_inc_ms <= 0.0) g_bt.hold_frozen_cycles++;
    }
    /* ---- 实测倍率 (只统计 is_moving 周期; 静止周期 eff_scale 不含 override) ---- */
    if (inflight && moving) {
        g_bt.override_eff_sum    += eff_scale;
        g_bt.override_eff_cycles += 1;
    }
    /* ---- 声明式模型: feedhold 生效期虚拟时间恒不推进 ----
     * 只在"确实在消费时间预算"的周期累加 (is_moving 或本周期真有推进),
     * 否则空转周期会被 model 计入而 measured 不计 ⇒ 两者不可比。
     * ⚠ 必须共用上面的 inflight 门控: 否则块结束后多 1 个周期被计入,
     * virtual_model_ms (4000.5) 与 block_virtual_ms (4000.0) 差半个周期,
     * 模型/实测失去可比性 (2026-09-20 修)。 */
    if (inflight && !hold_active && (moving || virtual_inc_ms > 0.0)) {
        g_bt.virtual_model_ms += cycle_time_ms * eff_scale;
    }
}
