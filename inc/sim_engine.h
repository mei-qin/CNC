#ifndef SIM_ENGINE_H
#define SIM_ENGINE_H

#include <stdio.h>
#include "axis_cfg.h"
#include "trace_logger.h"   // 引入 STAGE_RT_INTERPOLATOR 宏
#include <stdatomic.h>
#include <stdint.h>
#include <semaphore.h>
#include <pthread.h>

// ================== 高频无锁双缓冲仿真轨迹采集器 ==================
//
// 设计目标: g_sim_mode == 1 (极速仿真) 时, 以每周期 1 条的频率
// 无阻塞采集插补器物理坐标, 双缓冲交替落盘实现海量数据高速写入。
//
// 生产者: ecat_thread_rt (1ms 虚拟周期, 无锁写入活跃缓冲)
// 消费者: sim_flush_thread (后台非实时线程, 批量 fwrite 落盘)
//
// 架构: Double-Buffer Ping-Pong
//   RT 线程填满 Buffer[active_idx] → 原子交换 → sem_post 唤醒落盘线程
//   落盘线程检测 flush_pending[idx] → fwrite 整块写入 → 重置缓冲

// 每条轨迹采样记录 (AXIS_NUM=5 时 64 bytes 起步, 已含 stage_id 列)
// stage_id 恒为 STAGE_RT_INTERPOLATOR —— sim_engine 仅采集 1ms RT 插补物理波形
// ---- WCS 同步验证扩展字段 (Phase 2C) ----
// current_coord:         RT 当前工件坐标系 (0=G53..6=G59), 断言 A/B/C 关键列
// current_logical_pos[]: RT 推导的 UI 逻辑坐标 (物理坐标 - 生效偏置)
// active_offset[]:       当前生效的 X/Y/Z 三轴偏置 (即 work_offsets[current_coord])
// 这些字段在 sim_engine_finish 后与物理坐标一起回填 CSV，验证完可回退。
typedef struct {
    int      stage_id;         // 管线阶段标签 (固定 STAGE_RT_INTERPOLATOR)
    uint64_t cycle;            // RT 周期计数
    double   virtual_time_ms;  // 插补器虚拟时间 (ms)
    double   pos[AXIS_NUM];    // 各轴物理坐标 (mm / deg)
    double   v_target;         // 目标速度 (mm/ms)
    // ---- WCS 同步验证 (临时扩展队列, 验证完移除) ----
    int      current_coord;                // RT 当前 WCS 索引 (0=G53..6=G59)
    double   current_logical_pos[AXIS_NUM]; // RT 逻辑坐标
    double   active_offset[3];             // 生效偏置 X/Y/Z (= work_offsets[current_coord][0..2])
    double   work_offsets_g54_x;           // H-1: parser 对 work_offsets[G54][X] 的当前写入值
    // ---- P1': 辅助状态机 CSV 追踪 (从 g_interpolator._rt 镜像读) ----
    int      spindle_mode;                // 0=off, 1=CW(M3), 2=CCW(M4)
    double   spindle_rpm;                 // rpm
    int      coolant_state;               // bit0=flood(M8), bit1=mist(M7); 0/1/2/3
    int      tool_id;                     // 当前刀号 (M6 切换后)
    // ---- P0-Laser: 激光器状态 CSV 追踪 (从 g_laser_rt 读) ----
    // 用于验证 M62/M63 与运动段边界 1ms 对齐, 以及 alarm 触发时 emergency_kill 锁存路径
    int      laser_enable;                // 0/1 (M3/M5 联动)
    int      laser_shutter;               // 0/1 (M62/M63 同步)
    double   laser_power_w;               // M67 设置 (W) - 耦合 update 后的实际输出
    double   laser_freq_hz;               // M68 设置 (Hz)
    int      gas_select;                  // 0=off, 1=N2, 2=O2, 3=Air
    int      laser_emergency_kill;        // 急停锁存 (1=激光已关, 需复位)
    uint16_t laser_interlock;             // 互锁位图 (bit0=door...bit15=system_alarm)
    // ---- Phase B1: 功率-速度耦合 CSV 追踪 ----
    // laser_v_actual_mm_s: 当前周期瞬时速度 (mm/s), 验证 P-v 耦合曲线
    //                       对比 v_target 列可看 S 曲线 phase + 耦合比
    double   laser_v_actual_mm_s;         // RT 写, g_laser_rt.v_actual_mm_s
    // ---- P0-Laser-Q: 状态查询闭环 CSV 追踪 (验证 pierce_count/laser_on_time/seg_flags/is_piercing) ----
    int32_t  pierce_count;                // 累计穿孔次数 (M64 段完成 ++, 跨程序不清零)
    int64_t  laser_on_time_ms;            // 累计激光开启时间 ms (enable&&shutter&&!ekill 时累加)
    uint8_t  current_seg_flags;           // 当前段工艺标记 (bit0=lead_in, bit1=micro_joint)
    int      is_piercing;                 // 是否在 G04 穿孔 dwell 中 (派生: is_waiting_mcode && current_mcode==64)
    // ---- P0-3 SafeLift: 抬升状态机 CSV 追踪 (验证 alarm 自动 / 手动 / DONE 路径) ----
    int      safe_lift_state;             // 0=IDLE, 1=PENDING, 2=RUNNING, 3=DONE
    double   safe_lift_z_cmd;             // Z 轴当前命令位置 (mm) = g_axis[z_idx].current_cmd_pos
    // ---- P0-1 Homing + JOG: 状态机 CSV 追踪 ----
    int      homing_state;                // 0/1/2/3/4
    int      homing_axis_idx;             // 当前回零轴 (-1=HomeAll)
    int      jog_active;                  // 0/1
    int      jog_axis_idx;                // JOG 中的轴 (-1=未激活)
} sim_trace_record_t;

// 二进制文件头 (固定 512 bytes, record_count 在关闭时回填)
#define SIM_FILE_HEADER_SIZE  512

#pragma pack(push, 1)
typedef struct {
    char     magic[4];           // "SIM1"
    uint32_t version;            // 1
    uint32_t axis_count;         // AXIS_NUM
    char     axis_names[8][16];  // 轴名表 (最多 8 轴)
    uint32_t record_size;        // sizeof(sim_trace_record_t)
    uint64_t record_count;       // 总记录数 (关闭时回填)
    uint8_t  reserved[SIM_FILE_HEADER_SIZE - 4 - 4 - 4 - 128 - 4 - 8];
} sim_file_header_t;
#pragma pack(pop)

// 双缓冲容量: 每个缓冲 1M 条 ≈ 64 MB (AXIS_NUM=5)
#define SIM_BUF_CAPACITY  (1 << 18)  /* 256K records ≈ 38 MB/buf @ extended struct size; temp reduce for WSL2 */

// 双缓冲控制器
// RT 线程写端与落盘线程读端按 cache-line 隔离, 杜绝 false sharing
typedef struct {
    // --- 缓冲区 ---
    sim_trace_record_t *bufs[2];
    uint32_t            capacity;

    // --- RT 线程写端 (cache-line 独占) ---
    _Alignas(64) _Atomic uint32_t counts[2];
    _Alignas(64) _Atomic int      active_idx;

    // --- 落盘线程读端 (cache-line 独占) ---
    _Alignas(64) _Atomic int      flush_pending[2];
    _Alignas(64) _Atomic uint64_t total_records;
    _Alignas(64) _Atomic uint64_t dropped_records;

    // --- 线程控制 ---
    sem_t          flush_sem;
    pthread_t      flush_thread;
    volatile int   running;

    // --- 文件输出 ---
    char           output_path[260];
    FILE          *fp;
    int            use_binary;
    uint64_t       file_record_count;
    long           header_offset;
} sim_logger_t;

extern sim_logger_t g_sim_logger;

// P0-Laser: 强制下次 cycle 记录一条 sim_engine 采样
// 用途: M30 安全停 Step 4 抢写 g_laser_rt 后, RT 已静止 (is_moving=0 ∧ just_loaded_seg=0),
//       should_log_this_cycle=0 → sim_engine_push 不调 → CSV 末尾保留旧值.
//       parser 抢写后 set 此标志, RT 下个 cycle 强制记录一次新状态, 然后自动清 0.
// 线程安全: 单写者 (parser M30 路径), 单读者 (RT 线程 sim_engine_push 路径), atomic int.
extern _Atomic int g_sim_force_log;

// ================== API ==================

// @Context: Non-RealTime Background Thread (初始化阶段)
// 分配双缓冲内存, 创建输出文件, 写入文件头
int  sim_engine_init(const char *output_path, int use_binary);

// @Context: Non-RealTime Background Thread
// 启动后台落盘线程
int  sim_engine_start(void);

// @Context: Non-RealTime Background Thread (仿真结束后调用)
// 刷新残余数据, 等待落盘线程退出, 回填文件头, 关闭文件
void sim_engine_finish(void);

// 落盘线程入口 (由 sim_engine_start 创建)
void *sim_flush_thread_func(void *arg);

// @Context: 1ms Hard-RT Thread (ecat_thread_rt 内调用)
// @Danger: 无锁, 无阻塞, 无 printf, 无 malloc。
// RT 线程每周期调用: 写入插补器坐标到活跃缓冲;
// 缓冲满时原子交换 active_idx 并 sem_post 唤醒落盘线程。
static inline void sim_engine_push(uint64_t cycle, double virtual_time_ms,
                                     const double pos[AXIS_NUM], double v_target,
                                     int current_coord,
                                     const double current_logical_pos[AXIS_NUM],
                                     const double active_offset[3],
                                     double work_offsets_g54_x,
                                     int spindle_mode, double spindle_rpm,
                                     int coolant_state, int tool_id,
                                     int laser_enable, int laser_shutter,
                                     double laser_power_w, double laser_freq_hz,
                                     int gas_select,
                                     int laser_emergency_kill,
                                     uint16_t laser_interlock,
                                     double laser_v_actual_mm_s,
                                     int32_t pierce_count,
                                     int64_t laser_on_time_ms,
                                     uint8_t current_seg_flags,
                                     int is_piercing,
                                     int safe_lift_state,
                                     double safe_lift_z_cmd,
                                     int homing_state,
                                     int homing_axis_idx,
                                     int jog_active,
                                     int jog_axis_idx);

// ================== inline 实现 ==================

// @Context: 1ms Hard-RT Thread (EtherCAT / 仿真)
// @Danger: NO BLOCKING, NO PRINTF, NO MALLOC.
// 双缓冲写入: 活跃缓冲满 → 检查另一缓冲可用性 → 交换或静默丢弃。
// 绝不自旋等待: 若另一缓冲仍在落盘, 直接丢弃当前记录 (不阻塞 RT 线程)。
static inline void sim_engine_push(uint64_t cycle, double virtual_time_ms,
                                     const double pos[AXIS_NUM], double v_target,
                                     int current_coord,
                                     const double current_logical_pos[AXIS_NUM],
                                     const double active_offset[3],
                                     double work_offsets_g54_x,
                                     int spindle_mode, double spindle_rpm,
                                     int coolant_state, int tool_id,
                                     int laser_enable, int laser_shutter,
                                     double laser_power_w, double laser_freq_hz,
                                     int gas_select,
                                     int laser_emergency_kill,
                                     uint16_t laser_interlock,
                                     double laser_v_actual_mm_s,
                                     int32_t pierce_count,
                                     int64_t laser_on_time_ms,
                                     uint8_t current_seg_flags,
                                     int is_piercing,
                                     int safe_lift_state,
                                     double safe_lift_z_cmd,
                                     int homing_state,
                                     int homing_axis_idx,
                                     int jog_active,
                                     int jog_axis_idx)
{
    sim_logger_t *L = &g_sim_logger;
    int idx = atomic_load_explicit(&L->active_idx, memory_order_relaxed);
    uint32_t count = atomic_load_explicit(&L->counts[idx], memory_order_relaxed);

    if (count >= L->capacity) {
        int next = 1 - idx;
        if (atomic_load_explicit(&L->flush_pending[next], memory_order_acquire)) {
            atomic_fetch_add_explicit(&L->dropped_records, 1, memory_order_relaxed);
            return;
        }
        atomic_store_explicit(&L->flush_pending[idx], 1, memory_order_release);
        atomic_store_explicit(&L->counts[next], 0, memory_order_relaxed);
        atomic_store_explicit(&L->active_idx, next, memory_order_release);
        sem_post(&L->flush_sem);
        idx = next;
        count = 0;
    }

    sim_trace_record_t *r = &L->bufs[idx][count];
    r->stage_id        = STAGE_RT_INTERPOLATOR;
    r->cycle           = cycle;
    r->virtual_time_ms = virtual_time_ms;
    for (int i = 0; i < AXIS_NUM; i++) r->pos[i] = pos[i];
    r->v_target        = v_target;
    r->current_coord   = current_coord;
    for (int i = 0; i < AXIS_NUM; i++) r->current_logical_pos[i] = current_logical_pos[i];
    for (int i = 0; i < 3; i++)           r->active_offset[i]       = active_offset[i];
    r->work_offsets_g54_x = work_offsets_g54_x;
    r->spindle_mode   = spindle_mode;
    r->spindle_rpm    = spindle_rpm;
    r->coolant_state  = coolant_state;
    r->tool_id        = tool_id;
    // P0-Laser: 激光状态镜像 (sim CSV 用于验证段边界 1ms 对齐 + 急停锁存)
    r->laser_enable         = laser_enable;
    r->laser_shutter        = laser_shutter;
    r->laser_power_w        = laser_power_w;
    r->laser_freq_hz        = laser_freq_hz;
    r->gas_select           = gas_select;
    r->laser_emergency_kill = laser_emergency_kill;
    r->laser_interlock      = laser_interlock;
    r->laser_v_actual_mm_s  = laser_v_actual_mm_s;
    // P0-Laser-Q: 状态查询闭环 4 字段 (从 g_laser_rt / g_interpolator 读, 调用方传入)
    r->pierce_count         = pierce_count;
    r->laser_on_time_ms     = laser_on_time_ms;
    r->current_seg_flags    = current_seg_flags;
    r->is_piercing          = is_piercing;
    // P0-3 SafeLift: 抬升状态机 CSV 追踪
    r->safe_lift_state      = safe_lift_state;
    r->safe_lift_z_cmd      = safe_lift_z_cmd;
    // P0-1 Homing + JOG: 状态机 CSV 追踪
    r->homing_state         = homing_state;
    r->homing_axis_idx      = homing_axis_idx;
    r->jog_active           = jog_active;
    r->jog_axis_idx         = jog_axis_idx;

    atomic_store_explicit(&L->counts[idx], count + 1, memory_order_release);
    atomic_fetch_add_explicit(&L->total_records, 1, memory_order_relaxed);
}

#endif // SIM_ENGINE_H
