/* =====================================================================
 *  snapshot_hub.c  ——  P0-a 状态快照中心实现
 *
 *  设计见 inc/snapshot_hub.h 头注释。
 *
 *  关键不变量:
 *    1. g_snap_seq 奇数 = RT 写者正在写, reader 必须重试
 *    2. g_snap_seq 偶数 = 稳定, reader 可读
 *    3. 每次 publish 把 seq 加 2 (奇 → 偶), 偶数 seq 同时回写到 payload 的
 *       snapshot_seq 字段, 供 client 端做"丢帧检测" (相邻帧 seq 应 +2)
 *    4. Publish 不感知 reader 数量, 不阻塞, 不分配内存
 *
 *  字段来源审计 (见 inc/snapshot_hub.h 头注释):
 *    - g_interpolator.*     : RT 单写者 (除 is_moving 用 _Atomic)
 *    - g_coord_mgr.*        : RT 单写者
 *    - g_laser_rt.*         : RT 单写者
 *    - g_sys_alarm_state    : _Atomic int, RT + parser 都可能写
 *    - g_state.*            : parser 单写者, RT best-effort 读
 *    - g_parser_ctrl.*      : parser 单写者, RT best-effort 读
 * ===================================================================== */

#include "global_def.h"   /* g_interpolator, g_coord_mgr, g_laser_rt, g_state, g_parser_ctrl, g_sys_alarm_state
                           * 必须在 snapshot_hub.h 之前, 确保 AXIS_NUM 用 axis_cfg.h 的权威值,
                           * 不触发 snapshot_hub.h 的 fallback (#define AXIS_NUM 5) */
#include "snapshot_hub.h"
#include <string.h>

/* ---- 静态分配 (RT 禁 malloc) ---- */
static SMC_Snapshot_t g_snap_buf;
static _Atomic uint64_t g_snap_seq = 0;

/* @Context: Non-RealTime Background Thread (main 启动早期)
 * @Safe: 静态清零 + atomic init。幂等。 */
int SnapshotHub_Init(void)
{
    memset(&g_snap_buf, 0, sizeof(g_snap_buf));
    atomic_store_explicit(&g_snap_seq, 0, memory_order_release);
    g_snap_buf.magic   = SMC_SNAPSHOT_MAGIC;
    g_snap_buf.version = SMC_SNAPSHOT_VERSION;
    return 0;
}

/* @Context: 1ms Hard-RT Thread (EtherCAT, ecat_thread_rt cycle 尾)
 * @Danger: 仅 atomic store + memcpy + 直接读全局字段。
 *         NO BLOCKING / NO PRINTF / NO MALLOC / NO Math.h。
 *         浮点 OK (位置/速度都是 double)。
 *         g_state / g_parser_ctrl 字段 best-effort 读, 撕裂容忍。
 */
void SnapshotHub_Publish(uint32_t cycle)
{
    /* ---- Step 1: 标记写中 (seq 奇数) ---- */
    uint64_t s = atomic_load_explicit(&g_snap_seq, memory_order_relaxed);
    atomic_store_explicit(&g_snap_seq, s + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);

    /* ---- Step 2: 写所有字段 ---- */
    SMC_Snapshot_t *p = &g_snap_buf;

    /* 包头 */
    p->magic   = SMC_SNAPSHOT_MAGIC;
    p->version = SMC_SNAPSHOT_VERSION;
    p->cycle   = cycle;

    /* 时间 (g_interpolator, RT 单写者) */
    p->virtual_time_ms = g_interpolator.virtual_time_ms;
    p->uptime_ms       = (double)cycle * 1.0;   /* 硬启动时间 ms, cycle 是 RT 周期计数 (1ms/周期) */
    p->time_scale      = g_interpolator.time_scale;

    /* RT 运动状态 (g_interpolator) */
    p->is_moving        = atomic_load_explicit(&g_interpolator.is_moving, memory_order_relaxed);
    p->hold_state       = (int32_t)g_interpolator.hold_state;
    p->is_waiting_mcode = g_interpolator.is_waiting_mcode;
    p->current_mcode    = g_interpolator.current_mcode;
    p->mcode_p_value_ms = g_interpolator.mcode_p_value_ms;

    /* P0-c: 实时光标 (从 g_interpolator 镜像, RT 单写者) */
    p->current_seg_id = g_interpolator.current_seg_id_rt;
    {
        double T7 = g_interpolator.T7;
        if (T7 > 1e-9) {
            double prog = g_interpolator.virtual_time_ms / T7;
            if (prog < 0.0) prog = 0.0;
            if (prog > 1.0) prog = 1.0;
            p->segment_progress = prog;
        } else {
            /* T7=0 (段未加载或微段): 进度 0 */
            p->segment_progress = 0.0;
        }
    }

    /* 位置 (N 轴向量, g_interpolator, RT 单写者) */
    for (int i = 0; i < AXIS_NUM; i++) {
        p->machine_pos[i] = g_interpolator.current_pos[i];
        p->target_pos[i]  = g_interpolator.target_pos[i];
        p->start_pos[i]   = g_interpolator.start_pos[i];
    }

    /* 速度 (g_interpolator.v_current/v_target 是 mm/ms, 转 mm/s 给 UI) */
    p->v_current_mm_s = g_interpolator.v_current * 1000.0;
    p->v_target_mm_s  = g_interpolator.v_target  * 1000.0;
    /* g_state.feedrate_mm_min: best-effort 读 (parser 写) */
    p->feedrate_mm_min = g_state.feedrate_mm_min;

    /* Modal (g_state, best-effort 读, parser 写频率 ~10Hz)
     * 注意 motion_mode 反映"当前活动运动类型", IDLE (is_moving=0) 时强制 0.
     * 原始 modal 残留 (G01 等) 在 IDLE 时显示会让 UI 用户误判正在切削.
     * modal 状态 UI 可从 cycle start 后首段 G 代码识别, 不需 snapshot 提供. */
    p->motion_mode   = p->is_moving ? g_state.motion_mode : 0;
    p->active_plane  = g_state.active_plane;
    p->is_absolute   = g_state.is_absolute;
    p->feed_mode     = g_state.feed_mode;
    p->rtcp_enabled  = g_state.rtcp_enabled;
    p->active_cycle  = g_state.active_cycle;

    /* 坐标系 (g_coord_mgr, RT 单写者: current_coord/active_offset/current_logical_pos) */
    p->current_coord = (int32_t)g_coord_mgr.current_coord;
    for (int i = 0; i < AXIS_NUM; i++) {
        p->active_offset[i] = g_coord_mgr.active_offset[i];
        p->logical_pos[i]   = g_coord_mgr.current_logical_pos[i];
    }

    /* 主轴/冷却/刀 (_rt 镜像, RT 单写者: 段消费时从 seg 同步) */
    p->spindle_mode  = g_interpolator.spindle_mode_rt;
    p->spindle_rpm   = g_interpolator.spindle_rpm_rt;
    p->coolant_state = g_interpolator.coolant_state_rt;
    p->tool_id       = g_interpolator.current_tool_id_rt;

    /* 激光 (g_laser_rt, RT 单写者) */
    p->laser_enable         = g_laser_rt.enable;
    p->laser_shutter        = g_laser_rt.shutter;
    p->laser_power_w        = g_laser_rt.power_w;
    p->laser_freq_hz        = g_laser_rt.freq_hz;
    p->gas_select           = g_laser_rt.gas_select;
    p->laser_emergency_kill = g_laser_rt.emergency_kill;
    p->laser_interlock      = g_laser_rt.interlock_status;
    p->laser_v_actual_mm_s  = g_laser_rt.v_actual_mm_s;

    /* 系统状态 */
    p->sys_alarm_state   = atomic_load_explicit(&g_sys_alarm_state, memory_order_relaxed);
    p->parser_is_running = g_parser_ctrl.is_running;   /* best-effort */
    p->parser_is_paused  = g_parser_ctrl.is_paused;    /* best-effort */

    /* flags 位图聚合 (SDK 端做 RUN/HOLD/ALARM 语义聚合更方便) */
    uint32_t fl = 0;
    if (p->sys_alarm_state)                  fl |= SMC_SNAP_FLAG_ALARM;
    if (p->parser_is_paused)                 fl |= SMC_SNAP_FLAG_PAUSED;
    if (g_parser_ctrl.abort_request)         fl |= SMC_SNAP_FLAG_ABORT_REQ;
    if (p->hold_state != (int32_t)HOLD_NORMAL) fl |= SMC_SNAP_FLAG_HOLD;
    if (p->is_waiting_mcode)                 fl |= SMC_SNAP_FLAG_WAIT_MCODE;
    p->flags = fl;

    /* payload 内 snapshot_seq = 写完成后的稳定 seq (s+2) */
    p->snapshot_seq = s + 2;

    /* ---- Step 3: 标记写完成 (seq 偶数) ---- */
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&g_snap_seq, s + 2, memory_order_relaxed);
}

/* @Context: Non-RealTime Background Thread (push server client_thread)
 * @Safe: memcpy + atomic load, 可阻塞。
 *        1000 次重试均失败时返回 -1, *out 为最后一次重试的拷贝 (best-effort)。 */
int SnapshotHub_ReadLatest(SMC_Snapshot_t *out)
{
    if (out == NULL) return -1;

    for (int retry = 0; retry < 1000; retry++) {
        uint64_t s1 = atomic_load_explicit(&g_snap_seq, memory_order_acquire);
        if (s1 & 1) {
            /* 写者正在写, 重试 */
            continue;
        }
        /* 结构体赋值 = memcpy (packed 结构, 编译器生成 memcpy) */
        *out = g_snap_buf;
        atomic_thread_fence(memory_order_acquire);
        uint64_t s2 = atomic_load_explicit(&g_snap_seq, memory_order_acquire);
        if (s1 == s2) {
            return 0;   /* 稳定读 */
        }
        /* 写者在此期间又写了, 重试 */
    }
    return -1;   /* 写者持续忙, 放弃。*out 是最后一次重试的拷贝。 */
}
