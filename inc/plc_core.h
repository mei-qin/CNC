#ifndef PLC_CORE_H
#define PLC_CORE_H

/* =====================================================================
 *  plc_core.h —— P1 软 PLC 核心 (程序槽 / 扫描线程 / G-F 绑定)
 *
 *  P1-1 (2026-09-11) 范围: 程序槽 (下载-编译-持有) + CRC + 状态查询。
 *  P1-2 扩展: 10ms 扫描线程 (SCHED_OTHER) + G 读/F 写 + boot autoload。
 *
 *  并发模型:
 *    - RPC handler 线程 (0x0076-0x007A) 与扫描线程 (P1-2) 经 g_plc_mu 互斥;
 *    - 程序对象换装仅在 STOPPED/EMPTY 态 (运行中拒绝下载);
 *    - RT 内核不访问本模块 — G/F 交换走 periph_bus (seqlock/atomic)。
 *
 *  红线 (R2): 本模块永远不直接触 EtherCAT PDO; F 区信号仅是"状态汇聚",
 *  任何安全判定都在 RT 内核侧完成。
 * ===================================================================== */

#include <stdint.h>
#include "plc_lang.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 程序文本上限: 8B RPC 头 + 4080B ≤ SMC_MAX_PAYLOAD_BYTES(4096),
 * 与 smc_protocol.h SMC_PLC_TEXT_MAX 保持一致 (改任一必须同步另一处) */
#define PLC_PROG_TEXT_MAX   4080

typedef enum {
    PLC_STATE_EMPTY = 0,    /* 无程序 */
    PLC_STATE_STOPPED,      /* 已编译, 未运行 */
    PLC_STATE_RUNNING,      /* 扫描线程执行中 (P1-2 起) */
    PLC_STATE_ERROR         /* 运行期错误停机 (P1-2 起) */
} PlcRunState;

/* @Context: Non-RealTime (main/rpc_server 启动早期)
 * @Safe: 清空程序槽。幂等。 */
int  plc_core_init(void);

/* @Context: Non-RealTime (RPC 0x0076 handler)
 * @Safe: mutex + malloc (编译) 。失败保持原槽不变。
 * @param text/text_len  程序文本 (不要求 NUL 终结)
 * @param crc_expect     文本 CRC-32 (IEEE, zlib.crc32 兼容), 不符返回 -3
 * @param diags/diag_max 编译诊断输出 (容纳编译错误的行号+消息)
 * @param n_diag         诊断总数 (可能 > diag_max, 只拷贝前 diag_max 条)
 * @return 0=编译通过并持有 (状态→STOPPED); -1=编译错误; -2=超长; -3=CRC 不符;
 *         -4=运行中拒绝换装 (P1-2 起) */
int  plc_core_download(const char *text, int len, uint32_t crc_expect,
                       PlcDiag_t *diags, int diag_max, int *n_diag);

/* @Context: Non-RealTime (RPC 0x0077)
 * @return 0=成功; -1=槽内无文本。文本指针指向内部静态槽 (调用方勿释放)。 */
int  plc_core_get_text(const char **out_text, int *out_len);

/* ---- P1-2 (2026-09-11): 扫描线程 / 状态 / 变量快照 ---- */

typedef struct {
    int      state;        /* PlcRunState */
    int      n_vars;       /* 用户变量数 */
    int      n_stmts;      /* 顶层语句数 */
    int      text_len;
    uint32_t heartbeat;    /* 已完成扫描周期数 */
    uint32_t cycle_us;     /* 最近周期执行耗时 (µs, 不含睡眠) */
    int32_t  err_line;     /* 运行期错误行 (0=无) */
    char     err_msg[64];
} PlcCoreStatus_t;

/* 变量快照条目 (0x007A): 用户变量 + 3 个 F 系统变量 (值统一 double) */
typedef struct {
    char     name[32];
    uint8_t  type;         /* PlcType (FB 实例时无意义) */
    uint8_t  cls;          /* PlcVarClass */
    uint8_t  fb;           /* PlcFbKind */
    uint8_t  forced;       /* P1.5-2: 1 = 该变量当前被强制 (原 reserved 位) */
    double   value;        /* BOOL 0/1, 数值原样 (强制时呈现强制值) */
} PlcVarSnap_t;

/* @Context: Non-RealTime (RPC 0x0078 handler)
 * @param action 0=stop 1=start 2=reset
 * @return 0=成功; -1=状态不允许 (无程序/已运行/未运行) */
int  plc_core_ctrl(int action);

/* @Context: Non-RealTime (RPC 0x0079) */
int  plc_core_get_status(PlcCoreStatus_t *out);

/* @Context: Non-RealTime (RPC 0x007A)
 * @param out      输出缓冲 (≤ max 条)
 * @param n_total  输出快照总数 (可能 > max, 调用方分页)
 * @return 拷贝条数 */
int  plc_core_snapshot_vars(PlcVarSnap_t *out, int max, int *n_total);

/* @Context: Non-RealTime (RPC 0x0079) — 兼容旧 4 字段查询 */
int  plc_core_get_state(int *out_state, int *out_n_vars, int *out_n_stmts,
                        int *out_text_len);

/* CRC-32 (IEEE 802.3, poly 0xEDB88320, init/xor 0xFFFFFFFF) — Python 端
 * 用 zlib.crc32 校验一致。RT 无关, 非 RT 上下文使用。 */
uint32_t plc_crc32(const uint8_t *data, int len);

/* ---- P1-3 (2026-09-12): RT 侧看门狗数据源 + 测试钩子 ---- */

/* @Context: 1ms Hard-RT Thread (ecat_core 看门狗检查)
 * @Danger: 仅 atomic relaxed load, 无锁无阻塞。
 * 语义: RUNNING 且 heartbeat 30ms 未变化 → RT 侧报警 (事件 0x0004)。
 * 非 RUNNING 时 heartbeat 恒 0 语义 (看门狗不武装)。 */
int      plc_rt_is_running(void);
uint32_t plc_rt_heartbeat(void);

/* @Context: Non-RealTime (RPC 0x007B, sim-only 测试钩子)
 * 冻结 heartbeat 发布 (扫描线程继续跑) — 模拟调度饥饿, 验证 RT 看门狗。
 * @return 0=ok, -2=非 sim 模式 */
int  plc_core_test_freeze(int on);

/* @Context: Non-RealTime (RPC 0x007E / OPC UA PLC.ForceVar)
 * 用户变量强制 (输出强制语义, 周期末覆写; 见 plc_vm.h)。
 * @param name  变量名 (大小写不敏感); NULL 或空串且 on==0 = 清除全部强制
 * @param val   强制值 (按变量声明类型转换/钳制)
 * @param on    1=强制 0=解除
 * @return 0=ok; -1=变量不存在或非用户变量; -3=非 RUNNING 态
 *         (强制仅在 RUNNING 有效, start/stop 重启后自动清零) */
int  plc_core_force_var(const char *name, double val, int on);

/* @Context: Non-RealTime (RPC 0x0080 action=1)
 * Trace arm: 需 PLC RUNNING; 把 trace 配置里的通道名解析为变量表序号
 * (系统 G_/F_ + 用户全表可 trace) 后启动录制。
 * @return 0=ok; -1=通道名不可解析或未配置; -3=非 RUNNING */
int  plc_core_trace_arm(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PLC_CORE_H */
