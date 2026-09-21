/* =====================================================================
 *  periph_ai.c —— P1.5-1 外围模拟输入通道实现
 *
 *  设计见 inc/periph_ai.h 头注释。硬件 PDO 访问与 laser HAL 同纪律:
 * Ibytes 越界钳制 (SOEM IOmap 连续, 越界读 = 读邻站)。
 * ===================================================================== */

#include "periph_ai.h"
#include "global_def.h"     /* ctx / g_sim_mode */
#include "soem/soem.h"
#include <stdio.h>

PeriphAiConfig_t g_periph_ai_cfg;
double           g_periph_ai_rt[PERIPH_AI_CH_MAX];

/* sim 传感器注入 (RPC 0x007D 写 release / RT poll acquire) */
static _Atomic double  g_ai_sim_val[PERIPH_AI_CH_MAX];
static _Atomic uint32_t g_ai_sim_en = 0;     /* bit ch = 通道注入使能 */

void periph_ai_init(void)
{
    g_periph_ai_cfg.slave_id = -1;
    g_periph_ai_cfg.byte_off = 0;
    g_periph_ai_cfg.stride = 2;
    g_periph_ai_cfg.count = 0;
    for (int i = 0; i < PERIPH_AI_CH_MAX; i++) g_periph_ai_cfg.fs[i] = 0.0;
    for (int i = 0; i < PERIPH_AI_CH_MAX; i++) g_periph_ai_rt[i] = 0.0;
    atomic_store_explicit(&g_ai_sim_en, 0, memory_order_relaxed);
}

int periph_ai_config(int slave_id, int byte_off, int stride, int count,
                     const double *fs)
{
    if (slave_id < -1 || byte_off < 0 || byte_off > 63
        || (stride != 2 && stride != 4)
        || count < 0 || count > PERIPH_AI_CH_MAX)
        return -1;
    g_periph_ai_cfg.slave_id = slave_id;
    g_periph_ai_cfg.byte_off = (uint8_t)byte_off;
    g_periph_ai_cfg.stride = (uint8_t)stride;
    g_periph_ai_cfg.count = (uint8_t)count;
    for (int i = 0; i < PERIPH_AI_CH_MAX; i++)
        g_periph_ai_cfg.fs[i] = (fs != NULL) ? fs[i] : 0.0;
    printf("[AI] 模拟输入配置: slave=%d off=%d stride=%d count=%d "
           "fs=[%.3g %.3g %.3g %.3g]\n",
           slave_id, byte_off, stride, count,
           g_periph_ai_cfg.fs[0], g_periph_ai_cfg.fs[1],
           g_periph_ai_cfg.fs[2], g_periph_ai_cfg.fs[3]);
    return 0;
}

int periph_ai_inject(int ch, double val)
{
    if (!g_sim_mode) return -2;
    if (ch < 0 || ch >= PERIPH_AI_CH_MAX) return -1;
    atomic_store_explicit(&g_ai_sim_val[ch], val, memory_order_release);
    uint32_t m = atomic_load_explicit(&g_ai_sim_en, memory_order_relaxed);
    atomic_store_explicit(&g_ai_sim_en, m | (1u << ch), memory_order_release);
    printf("[AI] sim 注入: ch%d = %.4f\n", ch, val);
    return 0;
}

/* @Context: 1ms Hard-RT Thread
 * @Danger: 仅字节读 + 单次乘除 + atomic load。NO LOCK/PRINTF/MALLOC。 */
void periph_ai_rt_poll(void)
{
    if (g_sim_mode) {
        uint32_t m = atomic_load_explicit(&g_ai_sim_en, memory_order_acquire);
        for (int i = 0; i < PERIPH_AI_CH_MAX; i++) {
            g_periph_ai_rt[i] = (m & (1u << i))
                ? atomic_load_explicit(&g_ai_sim_val[i], memory_order_acquire)
                : 0.0;
        }
        return;
    }

    int sid = g_periph_ai_cfg.slave_id;
    if (sid < 0 || sid > ctx.slavecount || g_periph_ai_cfg.count == 0) {
        for (int i = 0; i < PERIPH_AI_CH_MAX; i++) g_periph_ai_rt[i] = 0.0;
        return;
    }
    uint8_t *inputs = ctx.slavelist[sid].inputs;
    if (inputs == NULL) {
        for (int i = 0; i < PERIPH_AI_CH_MAX; i++) g_periph_ai_rt[i] = 0.0;
        return;
    }
    for (int i = 0; i < PERIPH_AI_CH_MAX; i++) {
        if (i >= g_periph_ai_cfg.count || g_periph_ai_cfg.fs[i] <= 0.0) {
            g_periph_ai_rt[i] = 0.0;
            continue;
        }
        int off = g_periph_ai_cfg.byte_off + i * g_periph_ai_cfg.stride;
        if (off + 2 > (int)ctx.slavelist[sid].Ibytes) {
            g_periph_ai_rt[i] = 0.0;   /* 越界 = 配置错误, 输出安全 0 */
            continue;
        }
        uint16_t raw = (uint16_t)(inputs[off] | (inputs[off + 1] << 8));
        g_periph_ai_rt[i] = (double)raw * g_periph_ai_cfg.fs[i] / 65535.0;
    }
}
