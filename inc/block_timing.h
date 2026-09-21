#ifndef BLOCK_TIMING_H
#define BLOCK_TIMING_H

/* ============================================================================
 * block_timing.h — 论文1 S10 (2026-09-20): 控制器内 block 级计时插桩
 * ----------------------------------------------------------------------------
 * 背景 (用户红线): "不要再从 trace 行数推时间。直接在控制器里记录。"
 *   CSV trace 受 sim_engine 闸门 (ecat_core.c should_log_this_cycle) 影响,
 *   窗口两端都会丢行 (首周期无基准行 / 微段终端行在 ms_budget 恰耗尽时不落),
 *   因此 cy[i_end] - cy[i0] 恒少 1, 任何基于行数的时长口径都带系统性偏差。
 *
 * 方案: 在 RT 线程内直接用 RT 周期计数器 `cycle` (ecat_core.c:80, 每 1ms 递增,
 *   独立于 logging 闸门) 给每个 G-code block 打上起止刻度。
 *
 * 约定 (固定, 全项目统一):
 *   block_start_cycle          = 该 block 第一个微段开始执行的 RT 周期
 *   block_end_cycle_exclusive  = 该 block 最后一个微段完成所在周期 + 1
 *   block_elapsed_cycles       = block_end_cycle_exclusive - block_start_cycle
 *   => 单周期完成的 block 得 elapsed = 1 (而非 0)。这是"exclusive-end"约定。
 *
 * 三种时间 (H5 明确要求区分):
 *   T_virtual    = virtual_elapsed_ms        —— 语义/虚拟时间 (被倍率 + feedhold 缩放)
 *   T_ctrl       = controller_elapsed_cycles × Ts —— 控制器真实流逝时间
 *   T_host       = 宿主墙钟 (由 Python harness 测, 不在本模块)
 *
 * 数据流:
 *   parser (Non-RT): bt_block_open()/bt_block_close() 声明"正在生成的 block"
 *                    -> api_push_trajectory_impl 用 bt_stamp_segment() 给每个
 *                       微段盖章 (block_uid / index / count / budget)
 *   RT (ecat_core):  bt_rt_block_start()  段装载 (新 uid) 时打开始刻度
 *                    bt_rt_segment_done() 段完成时打结束刻度 (exclusive)
 *                    bt_rt_cycle()        每周期累加虚拟时间 / hold 计数
 *   RPC 0x0089:      一次性读出全部字段
 * ==========================================================================*/

#include <stdint.h>

typedef struct {
    /* ---- parser 侧 (Non-RT): 当前正在生成的 block 描述符 ---- */
    uint32_t pend_uid;       /* 当前 block 实例号 (1-based; 0 = "未插桩", RT 忽略) */
    uint32_t pend_count;     /* 本 block 声明生成的微段数 */
    uint32_t pend_idx;       /* 已盖章的微段序号 */
    int32_t  pend_line_no;   /* 源 G 代码行号 (1-based) */
    double   pend_cmd_ms;    /* 指令 block 时长 (ms) = T * 1000 */
    double   pend_sum_ms;    /* 实际下发微段时长之和 (ms) = Σ dt_per_seg * 1000 */
    int      pend_active;    /* 1 = 处于 open/close 窗口内 */

    /* ---- RT 侧: 当前 (或最后一个) block ---- */
    uint32_t block_uid;
    int32_t  block_line_no;
    uint64_t block_start_cycle;
    uint64_t block_end_cycle_exclusive;
    uint64_t block_elapsed_cycles;
    /* block 在飞期间"虚拟时间没有推进"的周期数 (= 停滞周期)。
     * 取代旧口径的 CSV dropped_cycles: 前者是控制器内事实, 后者是落盘伪影。
     * 连续运动的 G93 block 期望恒为 0; feedhold 期间会大量累加。 */
    uint32_t block_stall_cycles;
    uint32_t segments_generated;    /* parser 声明生成 */
    uint32_t segments_completed;    /* RT 实际跑完 */
    double   block_budget_cmd_ms;
    double   block_budget_sum_ms;
    /* 本 block 实际推进的虚拟时间 (ms)。
     * ⚠ 与 block_elapsed_cycles 不同: 后者含一个"残差清理周期" —— 每个微段的
     *   预算在装载周期被吃掉后, 还剩 ~1e-16 ms (double 表示误差), 被
     *   ecat_core 的 `if(remaining_time < 1e-6) remaining_time = 1e-6` 下限
     *   放大成 1e-6 ms 并在**下一周期**清理; 末微段没有后继段, 于是多占 1 个
     *   RT 周期。故 Full 的 elapsed_cycles 恒为 n+1, 而 block_virtual_ms 恒为 T。 */
    double   block_virtual_ms;
    /* 末微段完成所在周期 (0 = 未完成)。用于 bt_rt_cycle 的"block 在飞"门控:
     * cur_active 在 block 结束后不会自动清 (只有被下一 block 顶掉才清), 若直接
     * 用 cur_active 门控, 程序尾部空转周期会被误计入 stall/virtual。 */
    uint64_t block_finish_cycle;

    /* ---- RT 侧: block 切换检测 ---- */
    uint32_t cur_uid;               /* 当前 in-flight block 的 uid */
    int      cur_active;            /* 1 = 已有 in-flight block */
    uint64_t blocks_closed;         /* 已完成 (被下一个 block 顶掉) 的 block 数 */

    /* ---- RT 侧: 程序级 ---- */
    uint64_t ctrl_start_cycle;              /* 首个运动段装载周期 */
    uint64_t ctrl_end_cycle_exclusive;      /* 末个运动段完成周期 + 1 */
    uint64_t controller_elapsed_cycles;     /* T_ctrl 的周期数 */
    double   controller_elapsed_ms;         /* = cycles * Ts */
    double   virtual_elapsed_ms;            /* T_virtual (被倍率/feedhold 缩放) */
    uint64_t hold_active_cycles;            /* feedhold 生效周期数 (BRAKING|PAUSED) */
    double   hold_active_ms;                /* = hold_active_cycles * Ts (T_hold,actual) */
    /* ---- H5 细分 (2026-09-20) ----
     * hold_frozen_cycles: 虚拟时间**完全**不推进的周期数 (advance == 0)。
     *   feedhold 包络 (BRAKING/RESUMING, TIME_DEC_STEP=0.005 ⇒ 200 cyc 斜坡,
     *   平均推进 100 ms) 期间虚拟时间仍在部分推进, 故 frozen < active。
     * override_eff_*: 实测 eff_scale (RT 每周期真实缩放系数) 的累计/计数,
     *   只统计 is_moving 周期 —— 这是"倍率是否真的生效"的控制器内证据,
     *   也是 clamp (`if(eff_scale > 1.0) eff_scale = 1.0`) 的唯一硬证据。
     * virtual_model_ms: 声明式模型累加 Σ(hold ? 0 : Ts*eff_scale)。
     *   与 virtual_elapsed_ms 之差 = feedhold 包络期间仍推进的虚拟时间。 */
    uint64_t hold_frozen_cycles;
    double   override_eff_sum;
    uint64_t override_eff_cycles;
    double   virtual_model_ms;
    uint64_t segments_completed_total;      /* 程序级运动段完成计数 */
    uint64_t rt_cycle_counter;              /* RT 全局 cycle 镜像 (读数时刻) */
} BlockTiming_t;

extern BlockTiming_t g_bt;

/* ---- Non-RT (parser) ---- */
void bt_reset_all(void);
void bt_block_open(double cmd_ms, int count, double sum_ms, int line_no);
void bt_block_close(void);
void bt_stamp_segment(uint32_t *uid, uint32_t *idx, uint32_t *count,
                      double *cmd_ms, double *sum_ms);

/* ---- RT (ecat_core, 1ms Hard-RT Thread) ----
 * @Danger: 仅整数/浮点赋值与加法, NO BLOCKING / NO MATH.H / NO PRINTF / NO MALLOC. */
void bt_rt_block_start(uint32_t uid, int32_t line_no, uint32_t gen,
                       double cmd_ms, double sum_ms, int cycle);
void bt_rt_segment_done(int cycle);
void bt_rt_cycle(int cycle, double virtual_inc_ms, int hold_active,
                 double cycle_time_ms, double eff_scale, int moving);

#endif /* BLOCK_TIMING_H */
