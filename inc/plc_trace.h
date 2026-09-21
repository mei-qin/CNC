#ifndef PLC_TRACE_H
#define PLC_TRACE_H

/* =====================================================================
 *  plc_trace.h —— P1.5-3 PLC 变量示波器 (Trace)
 *
 *  定位: 排互锁/握手时序问题的工具 — 周期采样 PLC 变量 (含 G_/F_ 系统
 *  变量, 如 G_MCODE_STROBE 的 M80 握手边沿) 进环形缓冲, 触发后定格,
 *  RPC 分页导出供上位画时序曲线。
 *
 *  状态机:
 *    IDLE --arm--> ARMED (预触发滚动录, 满环覆盖)
 *              --触发条件满足--> TRIGGED --录满 post 500--> DONE (定格)
 *    ARMED/TRIGGED --手动 stop--> DONE (立即定格)
 *    任意 --abort--> IDLE (清数据)
 *
 *  线程模型: config/arm/stop/abort/read 由 RPC 线程调 (g_tr_mu);
 *    plc_trace_tick 由 PLC 扫描线程每周期调 (采样 + 触发评估, 在
 *    PlcVm_ApplyForce 之后 → 强制值亦被 trace)。
 *    锁序: plc_core g_plc_mu → g_tr_mu (单向, 无反序)。
 *
 *  时间轴: t_ms = heartbeat × 10 (PLC 扫描周期时基); div 分频控制
 *  采样密度 (1=10ms/点, 100=1s/点, 1000 点 = 10s~1000s 窗口)。
 * ===================================================================== */

#include <stdint.h>
#include "plc_lang.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PLC_TRACE_CH_MAX    8
#define PLC_TRACE_DEPTH     1000    /* 环深 (样本数) */
#define PLC_TRACE_POST      (PLC_TRACE_DEPTH / 2)   /* 触发后样本数 = 500 */
#define PLC_TRACE_NAME_LEN  32

typedef enum {
    PLC_TRIG_MANUAL = 0,    /* 手动 stop 定格 (一直滚动录) */
    PLC_TRIG_LEVEL_GE,      /* 通道值 >= threshold */
    PLC_TRIG_LEVEL_LE,      /* 通道值 <= threshold */
    PLC_TRIG_RISE,          /* 上穿: prev < th && cur >= th */
    PLC_TRIG_FALL           /* 下穿: prev > th && cur <= th */
} PlcTrigKind;

typedef struct {
    int32_t  n_ch;                            /* 1-8 */
    int32_t  div;                             /* 采样分频 1-100 */
    int32_t  trig_kind;                       /* PlcTrigKind */
    int32_t  trig_ch;                         /* 0..n_ch-1 */
    double   trig_threshold;
    char     names[PLC_TRACE_CH_MAX][PLC_TRACE_NAME_LEN];
} PlcTraceCfg_t;

typedef struct {
    uint32_t seq;                             /* arm 起全局递增 */
    uint32_t t_ms;                            /* heartbeat × 10 */
    double   v[PLC_TRACE_CH_MAX];
} PlcTraceSample_t;                           /* 72B */

/* @Context: Non-RealTime (RPC 0x007F)
 * @return 0=ok; -1=参数越界; -2=ARMED/TRIGGED 录制中拒改 */
int  plc_trace_config(const PlcTraceCfg_t *cfg);

/* @Context: Non-RealTime (plc_core_trace_arm, 持 g_plc_mu 时调)
 * 名字已在 plc_core 解析为变量表序号 (系统+用户全表可 trace)。
 * @return 0=ok; -1=未配置 (须先 config) */
int  plc_trace_arm(const int16_t *var_idx);

/* @Context: Non-RealTime (RPC 0x0080)
 * stop: ARMED/TRIGGED → DONE 定格; abort: 任意态 → IDLE 清数据 */
int  plc_trace_stop(void);
int  plc_trace_abort(void);

/* @Context: Non-RealTime (RPC 0x0080 查询)
 * @param out_state  0=IDLE 1=ARMED 2=TRIGGED 3=DONE
 * @param out_n      可导出样本数 = min(总采样, PLC_TRACE_DEPTH) */
void plc_trace_get_state(int *out_state, int *out_n);

/* @Context: Non-RealTime (RPC 0x0081)
 * 仅 DONE 态可读 (录制中读 -3); 按时间升序导出逻辑序号 [offset, offset+max)
 * @return 实际拷贝条数; *n_total 输出总数; offset 越界 *n_total 仍有效, 返回 -2 */
int  plc_trace_read(int offset, PlcTraceSample_t *out, int max, int *n_total);

/* @Context: Non-RealTime (plc_core 扫描线程, 每周期, ApplyForce 之后)
 * 采样 + 触发评估 + 状态推进。采样值按变量类型转换 (REAL 原样, 其余 double)。 */
void plc_trace_tick(const PlcProgram_t *prog, const PlcVal *cells,
                    uint32_t heartbeat);

/* 配置快照 (plc_core_trace_arm 解析名字用) */
void plc_trace_get_cfg(PlcTraceCfg_t *out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PLC_TRACE_H */
