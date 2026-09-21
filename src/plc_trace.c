/* =====================================================================
 *  plc_trace.c —— P1.5-3 PLC 变量示波器实现
 *
 *  设计见 inc/plc_trace.h 头注释。
 *  环读出: 逻辑序号 i ∈ [0, n_total) → ring[(wcount - n_total + i) % DEPTH]
 *    (wcount 为 arm 起累计采样数; 未满环时 n_total = wcount, 恰好从 0 起)
 * ===================================================================== */

#include "plc_trace.h"
#include <stdio.h>
#include <string.h>
#include <pthread.h>

enum { TR_IDLE = 0, TR_ARMED, TR_TRIGGED, TR_DONE };

static pthread_mutex_t g_tr_mu = PTHREAD_MUTEX_INITIALIZER;
static PlcTraceCfg_t   g_cfg;
static int             g_cfg_valid = 0;
static int             g_state = TR_IDLE;
static PlcTraceSample_t g_ring[PLC_TRACE_DEPTH];
static uint32_t        g_wcount;          /* arm 起累计采样数 */
static uint32_t        g_trig_at;         /* 触发样本序号+1 (post 计数起点) */
static double          g_prev_trig;       /* 触发通道上一次采样值 (边沿用) */
static int             g_have_prev = 0;
static int32_t         g_div_cnt = 0;
static int16_t         g_var_idx[PLC_TRACE_CH_MAX];

int plc_trace_config(const PlcTraceCfg_t *cfg)
{
    if (cfg == NULL) return -1;
    if (cfg->n_ch < 1 || cfg->n_ch > PLC_TRACE_CH_MAX
        || cfg->div < 1 || cfg->div > 100
        || cfg->trig_kind < PLC_TRIG_MANUAL || cfg->trig_kind > PLC_TRIG_FALL
        || cfg->trig_ch < 0 || cfg->trig_ch >= cfg->n_ch)
        return -1;
    for (int i = 0; i < cfg->n_ch; i++) {
        if (cfg->names[i][0] == '\0') return -1;
    }

    pthread_mutex_lock(&g_tr_mu);
    if (g_state == TR_ARMED || g_state == TR_TRIGGED) {
        pthread_mutex_unlock(&g_tr_mu);
        return -2;
    }
    g_cfg = *cfg;
    for (int i = cfg->n_ch; i < PLC_TRACE_CH_MAX; i++) g_cfg.names[i][0] = '\0';
    g_cfg_valid = 1;
    pthread_mutex_unlock(&g_tr_mu);
    printf("[TRACE] 配置: %d 通道 div=%d 触发=%d(ch%d, th=%.6g)\n",
           cfg->n_ch, cfg->div, cfg->trig_kind, cfg->trig_ch, cfg->trig_threshold);
    return 0;
}

void plc_trace_get_cfg(PlcTraceCfg_t *out)
{
    if (out == NULL) return;
    pthread_mutex_lock(&g_tr_mu);
    *out = g_cfg;
    pthread_mutex_unlock(&g_tr_mu);
}

int plc_trace_arm(const int16_t *var_idx)
{
    pthread_mutex_lock(&g_tr_mu);
    if (!g_cfg_valid || var_idx == NULL) {
        pthread_mutex_unlock(&g_tr_mu);
        return -1;
    }
    for (int i = 0; i < PLC_TRACE_CH_MAX; i++)
        g_var_idx[i] = var_idx[i];
    g_wcount = 0;
    g_trig_at = 0;
    g_have_prev = 0;
    g_div_cnt = 0;
    memset(g_ring, 0, sizeof(g_ring));
    g_state = TR_ARMED;
    pthread_mutex_unlock(&g_tr_mu);
    printf("[TRACE] arm: 预触发滚动录 (pre %d / post %d 样本)\n",
           PLC_TRACE_DEPTH - PLC_TRACE_POST, PLC_TRACE_POST);
    return 0;
}

int plc_trace_stop(void)
{
    pthread_mutex_lock(&g_tr_mu);
    if (g_state != TR_ARMED && g_state != TR_TRIGGED) {
        pthread_mutex_unlock(&g_tr_mu);
        return -1;
    }
    g_state = TR_DONE;
    pthread_mutex_unlock(&g_tr_mu);
    printf("[TRACE] 手动定格: %u 样本\n", g_wcount);
    return 0;
}

int plc_trace_abort(void)
{
    pthread_mutex_lock(&g_tr_mu);
    g_state = TR_IDLE;
    g_wcount = 0;
    pthread_mutex_unlock(&g_tr_mu);
    printf("[TRACE] abort: 数据清零回 IDLE\n");
    return 0;
}

void plc_trace_get_state(int *out_state, int *out_n)
{
    pthread_mutex_lock(&g_tr_mu);
    if (out_state) *out_state = g_state;
    if (out_n) {
        *out_n = (g_wcount > PLC_TRACE_DEPTH) ? PLC_TRACE_DEPTH : (int)g_wcount;
    }
    pthread_mutex_unlock(&g_tr_mu);
}

int plc_trace_read(int offset, PlcTraceSample_t *out, int max, int *n_total)
{
    pthread_mutex_lock(&g_tr_mu);
    int n = (g_wcount > PLC_TRACE_DEPTH) ? PLC_TRACE_DEPTH : (int)g_wcount;
    if (n_total != NULL) *n_total = n;
    if (g_state != TR_DONE) {
        pthread_mutex_unlock(&g_tr_mu);
        return -3;
    }
    if (offset < 0 || offset >= n || out == NULL || max <= 0) {
        pthread_mutex_unlock(&g_tr_mu);
        return -2;
    }
    int navail = n - offset;
    if (navail > max) navail = max;
    uint32_t start_seq = g_wcount - (uint32_t)n;
    for (int i = 0; i < navail; i++) {
        uint32_t logical = (uint32_t)offset + (uint32_t)i;
        out[i] = g_ring[(start_seq + logical) % PLC_TRACE_DEPTH];
    }
    pthread_mutex_unlock(&g_tr_mu);
    return navail;
}

/* @Context: PLC 扫描线程 (每周期; 10ms 周期, 锁开销可忽略) */
void plc_trace_tick(const PlcProgram_t *prog, const PlcVal *cells,
                    uint32_t heartbeat)
{
    pthread_mutex_lock(&g_tr_mu);
    if (g_state != TR_ARMED && g_state != TR_TRIGGED) {
        pthread_mutex_unlock(&g_tr_mu);
        return;
    }
    /* 分频 */
    if (++g_div_cnt < g_cfg.div) {
        pthread_mutex_unlock(&g_tr_mu);
        return;
    }
    g_div_cnt = 0;

    /* 采样 */
    PlcTraceSample_t *s = &g_ring[g_wcount % PLC_TRACE_DEPTH];
    s->seq = g_wcount;
    s->t_ms = heartbeat * 10u;
    for (int i = 0; i < g_cfg.n_ch && i < PLC_TRACE_CH_MAX; i++) {
        int vi = g_var_idx[i];
        if (prog == NULL || cells == NULL || vi < 0 || vi >= prog->n_vars) {
            s->v[i] = 0.0;
            continue;
        }
        s->v[i] = (prog->vars[vi].type == PLC_T_REAL)
                  ? cells[vi].v.d : (double)cells[vi].v.i;
    }
    g_wcount++;

    /* 触发评估 (仅 ARMED; 边沿用上一采样值) */
    if (g_state == TR_ARMED) {
        double v = s->v[g_cfg.trig_ch];
        int fired = 0;
        switch (g_cfg.trig_kind) {
        case PLC_TRIG_LEVEL_GE: fired = (v >= g_cfg.trig_threshold); break;
        case PLC_TRIG_LEVEL_LE: fired = (v <= g_cfg.trig_threshold); break;
        case PLC_TRIG_RISE:
            fired = g_have_prev && g_prev_trig < g_cfg.trig_threshold
                    && v >= g_cfg.trig_threshold;
            break;
        case PLC_TRIG_FALL:
            fired = g_have_prev && g_prev_trig > g_cfg.trig_threshold
                    && v <= g_cfg.trig_threshold;
            break;
        default: break;    /* MANUAL: 仅 stop 定格 */
        }
        if (fired) {
            g_state = TR_TRIGGED;
            g_trig_at = g_wcount;    /* post 从触发样本之后计 */
            printf("[TRACE] 触发: seq=%u v=%.6g (post %d 样本后定格)\n",
                   s->seq, v, PLC_TRACE_POST);
        }
        g_prev_trig = v;
        g_have_prev = 1;
    }

    /* post 满 → DONE */
    if (g_state == TR_TRIGGED && g_wcount - g_trig_at >= PLC_TRACE_POST) {
        g_state = TR_DONE;
        printf("[TRACE] DONE: %u 样本可导出\n", g_wcount);
    }
    pthread_mutex_unlock(&g_tr_mu);
}
