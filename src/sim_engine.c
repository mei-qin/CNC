#include "sim_engine.h"
#include "global_def.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

// ================== 全局实例 ==================
sim_logger_t g_sim_logger;
_Atomic int  g_sim_force_log = 0;  // P0-Laser: parser M30 抢写 g_laser_rt 后置 1, RT 下个 cycle 强制记录一次

// ================== 初始化 ==================
// @Context: Non-RealTime Background Thread (初始化阶段)
// @Safe: Math functions, blocking, and I/O are allowed.
int sim_engine_init(const char *output_path, int use_binary)
{
    memset(&g_sim_logger, 0, sizeof(sim_logger_t));

    g_sim_logger.capacity = SIM_BUF_CAPACITY;
    size_t buf_size = (size_t)SIM_BUF_CAPACITY * sizeof(sim_trace_record_t);

    // 分配双缓冲 (每个 64 MB, 共 128 MB)
    g_sim_logger.bufs[0] = (sim_trace_record_t *)malloc(buf_size);
    g_sim_logger.bufs[1] = (sim_trace_record_t *)malloc(buf_size);
    if (!g_sim_logger.bufs[0] || !g_sim_logger.bufs[1]) {
        printf("[SimEngine] 内存分配失败! 需要 %zu MB x 2\n", buf_size / (1024 * 1024));
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }
    memset(g_sim_logger.bufs[0], 0, buf_size);
    memset(g_sim_logger.bufs[1], 0, buf_size);

    // 原子变量初始化
    atomic_store_explicit(&g_sim_logger.counts[0], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.counts[1], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.active_idx, 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.flush_pending[0], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.flush_pending[1], 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.total_records, 0, memory_order_relaxed);
    atomic_store_explicit(&g_sim_logger.dropped_records, 0, memory_order_relaxed);

    g_sim_logger.use_binary = use_binary;
    g_sim_logger.file_record_count = 0;
    g_sim_logger.running = 1;
    strncpy(g_sim_logger.output_path, output_path,
            sizeof(g_sim_logger.output_path) - 1);

    // 创建输出文件
    g_sim_logger.fp = fopen(output_path, use_binary ? "wb" : "w");
    if (!g_sim_logger.fp) {
        printf("[SimEngine] 无法创建输出文件: %s\n", output_path);
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }

    // 写入文件头
    if (use_binary) {
        sim_file_header_t header;
        memset(&header, 0, SIM_FILE_HEADER_SIZE);
        memcpy(header.magic, "SIM1", 4);
        header.version     = 1;
        header.axis_count  = AXIS_NUM;
        header.record_size = (uint32_t)sizeof(sim_trace_record_t);
        header.record_count = 0;

        for (int i = 0; i < AXIS_NUM && i < 8; i++) {
            strncpy(header.axis_names[i], g_axis[i].axis_name, 15);
            header.axis_names[i][15] = '\0';
        }

        fwrite(&header, SIM_FILE_HEADER_SIZE, 1, g_sim_logger.fp);
        fflush(g_sim_logger.fp);

        // 记录 record_count 字段在文件中的偏移 (关闭时回填)
        g_sim_logger.header_offset = (long)offsetof(sim_file_header_t, record_count);
    } else {
        // CSV 表头 (含 WCS + H-1 + P1' aux 状态机扩展列)
        fprintf(g_sim_logger.fp, "cycle,stage_id,virtual_time_ms");
        for (int i = 0; i < AXIS_NUM; i++)
            fprintf(g_sim_logger.fp, ",%s", g_axis[i].axis_name);
        fprintf(g_sim_logger.fp, ",v_target,coord_rt");
        for (int i = 0; i < AXIS_NUM; i++)
            fprintf(g_sim_logger.fp, ",log_%s", g_axis[i].axis_name);
        for (int i = 0; i < 3; i++)
            fprintf(g_sim_logger.fp, ",off_%s", g_axis[i].axis_name);
        fprintf(g_sim_logger.fp, ",off_g54_X");
        // ---- P1': 辅助状态机 CSV 列 ----
        fprintf(g_sim_logger.fp, ",spindle_mode,spindle_rpm,coolant,tool_id");
        // ---- P0-Laser: 激光器状态 CSV 列 ----
        fprintf(g_sim_logger.fp,
                ",laser_enable,laser_shutter,laser_power_w,laser_freq_hz,"
                "gas_select,laser_emergency_kill,laser_interlock");
        // Phase B1: 功率-速度耦合速度列
        fprintf(g_sim_logger.fp, ",laser_v_actual_mm_s");
        // P0-Laser-Q: 状态查询闭环 4 列
        fprintf(g_sim_logger.fp,
                ",pierce_count,laser_on_time_ms,current_seg_flags,is_piercing");
        // P0-3 SafeLift: 抬升状态机 2 列
        fprintf(g_sim_logger.fp, ",safe_lift_state,safe_lift_z_cmd");
        // P0-1 Homing + JOG: 状态机 4 列
        fprintf(g_sim_logger.fp, ",homing_state,homing_axis_idx,jog_active,jog_axis_idx");
        // v2 (2026-07-20): home_offset 常量化 CSV 暴露 10 列
        //   5 轴 home_offset_dump (安装时常量, T1 验证首周期后不变)
        //   5 轴 homing_shift_dump (homing 后变量, T2 验证变化量 = cur_pulse - home_offset)
        fprintf(g_sim_logger.fp,
                ",home_offset_dump_X,home_offset_dump_Y,home_offset_dump_Z,"
                "home_offset_dump_B,home_offset_dump_C"
                ",homing_shift_dump_X,homing_shift_dump_Y,homing_shift_dump_Z,"
                "homing_shift_dump_B,homing_shift_dump_C");
        fprintf(g_sim_logger.fp, "\n");
        fflush(g_sim_logger.fp);
    }

    // 初始化信号量 (初始值 0, RT 线程 post 时唤醒)
    if (sem_init(&g_sim_logger.flush_sem, 0, 0) != 0) {
        printf("[SimEngine] 信号量初始化失败!\n");
        fclose(g_sim_logger.fp);
        free(g_sim_logger.bufs[0]);
        free(g_sim_logger.bufs[1]);
        return -1;
    }

    printf("[SimEngine] 双缓冲采集器就绪 (容量=%d x2, %zu MB/buf, 格式=%s, 文件=%s)\n",
           SIM_BUF_CAPACITY, buf_size / (1024 * 1024),
           use_binary ? "BINARY" : "CSV", output_path);
    return 0;
}

// ================== 启动落盘线程 ==================
// @Context: Non-RealTime Background Thread
int sim_engine_start(void)
{
    if (pthread_create(&g_sim_logger.flush_thread, NULL,
                        sim_flush_thread_func, &g_sim_logger) != 0) {
        printf("[SimEngine] 落盘线程创建失败!\n");
        return -1;
    }
    printf("[SimEngine] 落盘线程已启动\n");
    return 0;
}

// ================== 落盘线程 ==================
// @Context: Non-RealTime Background Thread
// @Safe: Math functions, blocking, and I/O are allowed.
//
// 阻塞在 sem_wait, 被 RT 线程的 sem_post 唤醒后:
// 1. 扫描两个缓冲的 flush_pending 标志
// 2. 对标记为 1 的缓冲: 读取 counts[i] 条记录, fwrite 整块落盘
// 3. 重置 flush_pending[i] = 0 (允许 RT 线程再次使用该缓冲)
//
// 关闭时: RT 线程设置 running=0 并 sem_post, 本线程排空残余后退出。
void *sim_flush_thread_func(void *arg)
{
    sim_logger_t *L = (sim_logger_t *)arg;

    while (1) {
        sem_wait(&L->flush_sem);

        for (int i = 0; i < 2; i++) {
            if (!atomic_load_explicit(&L->flush_pending[i], memory_order_acquire))
                continue;

            uint32_t cnt = atomic_load_explicit(&L->counts[i], memory_order_acquire);
            if (cnt == 0) {
                atomic_store_explicit(&L->flush_pending[i], 0, memory_order_release);
                continue;
            }

            if (L->use_binary) {
                // 二进制落盘: 单次 fwrite 写入整块 (极高速)
                size_t written = fwrite(L->bufs[i], sizeof(sim_trace_record_t),
                                        cnt, L->fp);
                L->file_record_count += written;
            } else {
                // 输出列序: cycle, stage_id, virtual_time_ms, 5×pos, v_target,
                //            coord_rt, 5×logical_pos, 3×active_offset, off_g54_X,
                //            spindle_mode, spindle_rpm, coolant, tool_id
                for (uint32_t j = 0; j < cnt; j++) {
                    sim_trace_record_t *r = &L->bufs[i][j];
                    fprintf(L->fp, "%" PRIu64 ",%d,%.3f",
                            r->cycle, r->stage_id, r->virtual_time_ms);
                    for (int a = 0; a < AXIS_NUM; a++)
                        fprintf(L->fp, ",%.6f", r->pos[a]);
                    fprintf(L->fp, ",%.6f,%d", r->v_target, r->current_coord);
                    for (int a = 0; a < AXIS_NUM; a++)
                        fprintf(L->fp, ",%.6f", r->current_logical_pos[a]);
                    for (int a = 0; a < 3; a++)
                        fprintf(L->fp, ",%.6f", r->active_offset[a]);
                    fprintf(L->fp, ",%.6f", r->work_offsets_g54_x);
                    // ---- P1': aux 状态机 4 列 ----
                    fprintf(L->fp, ",%d,%.3f,%d,%d",
                            r->spindle_mode, r->spindle_rpm,
                            r->coolant_state, r->tool_id);
                    // ---- P0-Laser: 激光状态 7 列 ----
                    fprintf(L->fp, ",%d,%d,%.3f,%.3f,%d,%d,%u",
                            r->laser_enable, r->laser_shutter,
                            r->laser_power_w, r->laser_freq_hz,
                            r->gas_select, r->laser_emergency_kill,
                            (unsigned)r->laser_interlock);
                    // Phase B1: 速度列 (1 列)
                    fprintf(L->fp, ",%.3f", r->laser_v_actual_mm_s);
                    // P0-Laser-Q: 状态查询闭环 4 列
                    fprintf(L->fp, ",%d,%lld,%u,%d",
                            (int)r->pierce_count,
                            (long long)r->laser_on_time_ms,
                            (unsigned)r->current_seg_flags,
                            r->is_piercing);
                    // P0-3 SafeLift: 抬升状态机 2 列
                    fprintf(L->fp, ",%d,%.3f",
                            r->safe_lift_state, r->safe_lift_z_cmd);
                    // P0-1 Homing + JOG: 状态机 4 列
                    fprintf(L->fp, ",%d,%d,%d,%d",
                            r->homing_state, r->homing_axis_idx,
                            r->jog_active, r->jog_axis_idx);
                    // v2 (2026-07-20): home_offset 常量化 10 列
                    // AXIS_NUM=5 顺序: X(0), Y(1), Z(2), B(3), C(4)
                    fprintf(L->fp, ",%d,%d,%d,%d,%d",
                            r->home_offset_dump[0], r->home_offset_dump[1],
                            r->home_offset_dump[2], r->home_offset_dump[3],
                            r->home_offset_dump[4]);
                    fprintf(L->fp, ",%d,%d,%d,%d,%d",
                            r->homing_shift_dump[0], r->homing_shift_dump[1],
                            r->homing_shift_dump[2], r->homing_shift_dump[3],
                            r->homing_shift_dump[4]);
                    fprintf(L->fp, "\n");
                }
                L->file_record_count += cnt;
            }

            fflush(L->fp);
            atomic_store_explicit(&L->flush_pending[i], 0, memory_order_release);
        }

        // 关闭条件: running=0 即退出 (本轮已排空所有 pending 缓冲, 含 sim_engine_finish
        //           标记的残余 active 缓冲)。注意: 必须仅判 running==0 即可退出,
        //           不能在末次 flush 后绕回 sem_wait —— 否则落盘线程永久阻塞 -> SMC_Close
        //           卡死 (pthread_join 永远等不到落盘线程退出)。
        if (!L->running) break;
    }

    return NULL;
}

// ================== 停止并回填 ==================
// @Context: Non-RealTime Background Thread (仿真结束后调用)
// @Safe: Math functions, blocking, and I/O are allowed.
void sim_engine_finish(void)
{
    if (!g_sim_logger.running) return;

    // 1. 将活跃缓冲的残余数据标记为待落盘
    int idx = atomic_load_explicit(&g_sim_logger.active_idx, memory_order_relaxed);
    uint32_t cnt = atomic_load_explicit(&g_sim_logger.counts[idx], memory_order_acquire);
    if (cnt > 0) {
        atomic_store_explicit(&g_sim_logger.flush_pending[idx], 1, memory_order_release);
    }

    // 2. 通知落盘线程退出 (先标记, 再唤醒)
    g_sim_logger.running = 0;
    sem_post(&g_sim_logger.flush_sem);

    // 3. 等待落盘线程完成
    pthread_join(g_sim_logger.flush_thread, NULL);

    // 4. 二进制模式: 回填文件头的 record_count
    if (g_sim_logger.use_binary && g_sim_logger.fp) {
        fseek(g_sim_logger.fp, g_sim_logger.header_offset, SEEK_SET);
        fwrite(&g_sim_logger.file_record_count, sizeof(uint64_t), 1, g_sim_logger.fp);
    }

    // 5. 关闭文件
    if (g_sim_logger.fp) {
        fflush(g_sim_logger.fp);
        fclose(g_sim_logger.fp);
        g_sim_logger.fp = NULL;
    }

    // 6. 释放内存
    free(g_sim_logger.bufs[0]);
    free(g_sim_logger.bufs[1]);
    g_sim_logger.bufs[0] = NULL;
    g_sim_logger.bufs[1] = NULL;

    sem_destroy(&g_sim_logger.flush_sem);

    printf("[SimEngine] 轨迹落盘完成, 共 %" PRIu64 " 条记录, 文件: %s\n",
           g_sim_logger.file_record_count, g_sim_logger.output_path);

    uint64_t dropped = atomic_load_explicit(&g_sim_logger.dropped_records, memory_order_relaxed);
    if (dropped > 0) {
        printf("[SimEngine] 警告: 丢弃 %" PRIu64 " 条记录 (落盘速度不足)\n", dropped);
    }
}
