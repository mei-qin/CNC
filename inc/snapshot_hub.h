#ifndef SNAPSHOT_HUB_H
#define SNAPSHOT_HUB_H

/* =====================================================================
 *  snapshot_hub.h  ——  P0-a 状态快照中心 (RT → 多 UI client 推送地基)
 *
 *  定位:
 *    RT 线程每 1ms 把整体状态镜像写入单缓冲 seqlock, 外部 UI client
 *    (经 rpc_push_server 端口 9528) 按各自 freq (10/60/200Hz) 拉取最新帧。
 *    解决现有 50+ SMC_GetXxx polling 模型的状态撕裂 + 60fps 不流畅问题。
 *
 *  并发模型 (Seqlock Single-Buffer):
 *    单写者: 1ms Hard-RT Thread (ecat_thread_rt cycle 尾)
 *      seq++; atomic_thread_fence(release); WRITE all fields; fence; seq++
 *      (奇数 = 写中, 偶数 = 稳定)
 *    多读者: Non-RT Background Thread (push server client_thread)
 *      do { s1 = seq; if (s1&1) continue; memcpy; s2 = seq; } while (s1 != s2);
 *    RT 端 O(1) 代价 (memcpy ~200B + 2 atomic store), reader 互不阻塞。
 *
 *  为什么不用 sim_logger 双缓冲 active_idx swap:
 *    swap + seq++ 是两个独立原子操作, reader 可能在 swap 后 seq++ 前
 *    读到撕裂数据。Seqlock 单缓冲靠 seq 重读检测撕裂, 更简单且省一半内存。
 *
 *  为什么不用 sem_post 唤醒:
 *    sim_logger 是"生产者→消费者"模型 (1Hz sem_post), push 通道是
 *    "多 reader 各自拉取"模型 (60/10/200Hz 并存), sem 唤醒谁?
 *    每 client 一个 sem 违反"RT 不感知 client"原则。
 *
 *  内存序:
 *    用 memory_order_release (写) + memory_order_acquire (读),
 *    禁用 memory_order_seq_cst (MFENCE 全屏障影响 RT 抖动)。
 *
 *  字段来源 (混合):
 *    g_interpolator.*       — RT 单写者, 安全 (除 is_moving 用 atomic)
 *    g_coord_mgr.*          — RT 单写者 (current_coord/active_offset/current_logical_pos)
 *    g_laser_rt.*           — RT 单写者
 *    g_sys_alarm_state      — _Atomic int
 *    g_parser_ctrl.*        — parser 线程写, RT best-effort 读 (撕裂容忍)
 *    g_state.*              — parser 线程写, RT best-effort 读 (撕裂容忍)
 *    x86 上 int/double 单字段读写天然原子, parser 写频率 ~10Hz (每行 G 代码一次),
 *    撕裂概率极低。若实测 UI 出现 modal 跳变, 回退到选项 B (TrajectorySegment_t
 *    加 modal_*_seg, RT 消费时同步到 _rt 镜像)。
 * ===================================================================== */

#include <stdint.h>

/* AXIS_NUM 由 axis_cfg.h 定义 (CNC Core 端), 此处允许外部预定义。
 * Windows SDK 包含本头文件解析帧时不依赖 axis_cfg.h / SOEM,
 * 默认按 5 轴布局 (与 CNC Core 一致, 改 AXIS_NUM 必须 bump SMC_SNAPSHOT_VERSION)。 */
#ifndef AXIS_NUM
#define AXIS_NUM 5
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 帧标识 (与 SmcPushFrameHeader.magic 一致) ---- */
#define SMC_SNAPSHOT_MAGIC    0x534E4150u   /* "SNAP" little-endian */
#define SMC_SNAPSHOT_VERSION  4u   /* v4: P2-A 实时倍率字段 (2026-07-16).
                                    *      v3: 加 current_seg_id + segment_progress 字段 (2026-07-13, P0-c) */

#pragma pack(push, 1)

/* ---- flags 位定义 (SMC_Snapshot_t.flags) ---- */
#define SMC_SNAP_FLAG_ALARM       0x00000001u   /* g_sys_alarm_state == 1 */
#define SMC_SNAP_FLAG_PAUSED      0x00000002u   /* g_parser_ctrl.is_paused */
#define SMC_SNAP_FLAG_ABORT_REQ   0x00000004u   /* g_parser_ctrl.abort_request */
#define SMC_SNAP_FLAG_HOLD        0x00000008u   /* g_interpolator.hold_state != HOLD_NORMAL */
#define SMC_SNAP_FLAG_WAIT_MCODE  0x00000010u   /* g_interpolator.is_waiting_mcode */

/* P0-a 状态快照结构 (~200B, AXIS_NUM=5)
 *
 * 字段顺序按 "时间 → 运动状态 → 位置 → 速度 → 模态 → 坐标系 → 主轴冷却刀 →
 *            激光 → 系统状态" 分组, 便于 UI 端按段解析。
 * 整体打包后 ~200 字节, 60Hz × 多 client 带宽 < 100KB/s, 远低于 TCP 限额。
 */
typedef struct {
    /* === 包头 (20B) === */
    uint32_t magic;            /* SMC_SNAPSHOT_MAGIC, 客户端校验帧同步 */
    uint32_t version;          /* SMC_SNAPSHOT_VERSION, 字段表变更必须 bump */
    uint64_t snapshot_seq;     /* hub 内部递增序号 (每次 publish +2, 偶数=稳定) */
    uint32_t cycle;            /* RT cycle 计数 (extern int cycle, 截断到 uint32) */
    uint32_t flags;            /* SMC_SNAP_FLAG_* 位图, SDK 端做 RUN/HOLD/ALARM 聚合 */

    /* === 时间 (24B) === */
    double virtual_time_ms;    /* g_interpolator.virtual_time_ms, 段内插补时间 (每段从 0), 与 sim CSV 对齐用 */
    double uptime_ms;          /* 系统启动以来硬时间 = cycle * 1ms (RT 周期计数). UI 显示 "已运行时长" */
    double time_scale;         /* g_interpolator.time_scale (0-1, feedhold 真实状态) */

    /* === RT 运动状态 (32B) === */
    int32_t is_moving;         /* g_interpolator.is_moving (_Atomic, RT 用 atomic_load) */
    int32_t hold_state;        /* g_interpolator.hold_state (FeedHoldState_t 枚举) */
    int32_t is_waiting_mcode;  /* g_interpolator.is_waiting_mcode */
    int32_t current_mcode;     /* g_interpolator.current_mcode (等待中的 M 代码编号) */
    double  mcode_p_value_ms;  /* g_interpolator.mcode_p_value_ms (G04 dwell 等) */

    /* === P0-c: 实时光标 (16B) ===
     * UI 据此高亮 G 代码当前行 + 在轨迹上标记当前段 + 进度条。
     * current_seg_id: 段加载时从 seg.seg_id 拷贝 (RT 段消费环 ecat_core.c)。
     *                 静止时保留上一段 id (UI 显示"刚执行完")。
     * segment_progress: virtual_time_ms / T7, 钳到 [0, 1]。
     *                    M 代码段 T7 可能无意义, UI 看 is_waiting_mcode 显示等待态。 */
    uint64_t current_seg_id;
    double  segment_progress;

    int32_t _pad32;            /* 8B 对齐填充, 后续字段都是 double */

    /* === 位置 (N 轴向量, 机械绝对坐标 G53) === */
    double machine_pos[AXIS_NUM];   /* g_interpolator.current_pos */
    double target_pos[AXIS_NUM];    /* g_interpolator.target_pos */
    double start_pos[AXIS_NUM];     /* g_interpolator.start_pos */

    /* === 速度 (24B) === */
    double v_current_mm_s;     /* g_interpolator.v_current * 1000 (mm/ms → mm/s) */
    double v_target_mm_s;      /* g_interpolator.v_target * 1000 */
    double feedrate_mm_min;    /* best-effort 读 g_state.feedrate_mm_min */

    /* === Modal (best-effort 读 g_state) (24B) === */
    int32_t motion_mode;       /* 0=G00, 1=G01, 2=G02, 3=G03 */
    int32_t active_plane;      /* 17=XY, 18=ZX, 19=YZ */
    int32_t is_absolute;       /* 1=G90, 0=G91 */
    int32_t feed_mode;         /* 93=G93, 94=G94 */
    int32_t rtcp_enabled;      /* G43.4 开关 */
    int32_t active_cycle;      /* 0/G80=无, 73/81/82/83 */

    /* === 坐标系 (RT 单写者, 安全) === */
    int32_t current_coord;     /* g_coord_mgr.current_coord (CoordSystem_t 0..6) */
    double  active_offset[AXIS_NUM];   /* g_coord_mgr.active_offset */
    double  logical_pos[AXIS_NUM];     /* g_coord_mgr.current_logical_pos */

    /* === 主轴/冷却/刀 (从 _rt 镜像, RT 单写者) (24B) === */
    int32_t spindle_mode;      /* 0=off(M5), 1=CW(M3), 2=CCW(M4) */
    double  spindle_rpm;       /* rpm */
    int32_t coolant_state;     /* bit0=flood(M8), bit1=mist(M7); 0/1/2/3 */
    int32_t tool_id;           /* 当前刀号 (M6 切换后) */

    /* === 激光 (从 g_laser_rt, RT 单写者) === */
    int32_t laser_enable;
    int32_t laser_shutter;
    double  laser_power_w;
    double  laser_freq_hz;
    int32_t gas_select;            /* 0=off, 1=N2, 2=O2, 3=Air */
    int32_t laser_emergency_kill;  /* 急停锁存 (1=激光已关, 需复位) */
    uint16_t laser_interlock;      /* 互锁位图 bit0=door...bit15=system_alarm */
    uint16_t _pad16;               /* 4B 对齐 */
    double  laser_v_actual_mm_s;   /* P-v 耦合瞬时速度 (mm/s) */

    /* === 系统状态 === */
    int32_t sys_alarm_state;     /* g_sys_alarm_state (0=正常, 1=软停机) */
    int32_t parser_is_running;   /* g_parser_ctrl.is_running (best-effort) */
    int32_t parser_is_paused;    /* g_parser_ctrl.is_paused (best-effort) */

    /* === P2-A: 实时倍率 (16B, 2026-07-16) ===
     * 反映 RT 当前生效的 override + mode_flags (与 SMC_GetOverride RPC 同源).
     * UI 60Hz refresh 时直接读 snapshot, 不必再发 RPC, 减 9527 流量.
     * 字段语义:
     *   feed_override_pct    ∈ [0..100] (v1 锁 100, 超 100 clamp)
     *   rapid_override_pct   ∈ [0..100]
     *   spindle_override_pct ∈ [0..120]
     *   mode_flags           见 smc_protocol.h SMC_MODE_* 位定义:
     *                          bit0=SINGLE_BLOCK, bit1=DRY_RUN, bit4=OVERRIDE_PERSIST */
    int32_t feed_override_pct;
    int32_t rapid_override_pct;
    int32_t spindle_override_pct;
    int32_t mode_flags;

    int32_t current_seg_is_exact_stop;  /* P2-A-4: 当前段 is_exact_stop 镜像 (RT 段消费环同步),
                                           供客户端/测试识别精准停拐角 (G09/G61=1, G64=0).
                                           复用原 _tail_pad 预留位, 保持 SNAP_SIZE=440 不变. */
} SMC_Snapshot_t;

#pragma pack(pop)

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime Background Thread (SMC_InitAndStart 之前)
 * @Safe: 静态分配的 g_snap_buf 清零, seq=0。可在 main 启动早期调用一次。
 *        幂等, 多次调用无副作用。 */
int  SnapshotHub_Init(void);

/* @Context: 1ms Hard-RT Thread (EtherCAT, ecat_thread_rt cycle 尾)
 * @Danger: 仅 atomic store + memcpy + 直接读 g_interpolator/g_coord_mgr/g_laser_rt 字段。
 *         NO BLOCKING / NO PRINTF / NO MALLOC / NO Math.h。
 *         浮点 OK (位置/速度都是 double)。
 *         g_state / g_parser_ctrl 字段 best-effort 读, 撕裂容忍 (parser 写频率 ~10Hz)。
 *         不受 should_log_this_cycle / g_sim_mode 约束 — 静止时也要推, 让 HMI 看到
 *         alarm/feedhold 状态变化。
 * @param cycle RT 周期计数 (extern int cycle, 调用方做 (uint32_t)cycle 转换) */
void SnapshotHub_Publish(uint32_t cycle);

/* @Context: Non-RealTime Background Thread (push server client_thread)
 * @Safe: memcpy + atomic load, 可阻塞。
 * @return 0=成功拷贝最新帧; -1=写者持续忙 (1000 次重试均读到 seq 奇数或变化),
 *         此时 *out 内容是最后一次重试的旧值, 调用方可继续使用 (best-effort)。 */
int  SnapshotHub_ReadLatest(SMC_Snapshot_t *out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SNAPSHOT_HUB_H */
