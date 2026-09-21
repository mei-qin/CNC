#ifndef PERIPH_AI_H
#define PERIPH_AI_H

/* =====================================================================
 *  periph_ai.h —— P1.5-1 外围模拟输入 (AI) 通道 (2026-09-14)
 *
 *  定位: 水温/气压/流量等模拟量传感器的采集通道, 供软 PLC 经系统变量
 *  G_AI0..G_AI3 (REAL, 工程量) 读取。
 *
 *  数据链 (红线合规: Non-RT 的 PLC 永不直接读 IOmap):
 *    RT 周期 (ecat_core, PublishG 之前): periph_ai_rt_poll()
 *      硬件: 读 slave PDO (参数化 byte_off/stride, Obytes 钳制)
 *            raw 16-bit (0-65535) → 工程量 0..fs[ch]
 *      sim:  返回 RPC 0x007D 注入值 (传感器模拟)
 *      → 写 g_periph_ai_rt[4] (RT 单写者)
 *    PeriphBus_PublishG: ai[i] = g_periph_ai_rt[i] (G 区 seqlock)
 *    PLC 扫描线程: G_AIx ← G 区 (plc_vm refresh)
 *
 *  线程模型: g_periph_ai_cfg init 阶段单写者 (档案/RPC), RT 只读;
 *    sim 注入 atomic (RPC 写 / RT 读)。与 laser_ctrl 的 couple_table 同模式。
 *
 *  原始值符号性: v1 按 unsigned 0-65535 换算; 端子 ESI 若为有符号
 *  (EL3xxx 常见 ±10V 左对齐) 在 P0-4b 实装时加 sign 标志位。
 * ===================================================================== */

#include <stdint.h>
#include <stdatomic.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PERIPH_AI_CH_MAX  4

typedef struct {
    int      slave_id;                    /* AI 从站 (-1 = 未配置) */
    uint8_t  byte_off;                    /* 通道 0 起始字节 (0-63) */
    uint8_t  stride;                      /* 每通道字节 (2/4) */
    uint8_t  count;                       /* 通道数 0-4 */
    double   fs[PERIPH_AI_CH_MAX];        /* 每通道满量程工程值 (raw 65535 → fs;
                                             <=0 = 该通道未配, 输出恒 0) */
} PeriphAiConfig_t;

extern PeriphAiConfig_t g_periph_ai_cfg;
extern double           g_periph_ai_rt[PERIPH_AI_CH_MAX];   /* RT 单写者工程量 */

/* @Context: Non-RealTime (启动早期; rpc_server 两入口)
 * @Safe: 清零配置 (slave=-1, count=0)。幂等。 */
void periph_ai_init(void);

/* @Context: Non-RealTime (RPC 0x007C handler / 档案 apply)
 * @return 0=ok; -1=参数越界 (off>63 / stride∉{2,4} / count>4 / slave<-1) */
int  periph_ai_config(int slave_id, int byte_off, int stride, int count,
                      const double *fs);

/* @Context: Non-RealTime (RPC 0x007D, sim-only 传感器值注入)
 * @return 0=ok; -1=通道越界; -2=非 sim 模式 */
int  periph_ai_inject(int ch, double val);

/* @Context: 1ms Hard-RT Thread (ecat_core, PeriphBus_PublishG 之前)
 * @Danger: 仅 PDO 字节读 + 乘除 + atomic load, 无锁无阻塞。 */
void periph_ai_rt_poll(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PERIPH_AI_H */
