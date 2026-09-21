#ifndef PLC_VM_H
#define PLC_VM_H

/* =====================================================================
 *  plc_vm.h —— P1 软 PLC 树遍历解释器
 *
 *  执行模型 (docs/P1_SOFTPLC_DESIGN_SPEC.md §1/§3.4):
 *    每扫描周期: G cells 从 PeriphG 刷新 → 顺序执行语句 → F cells 由
 *    调用方 (plc_core) 发布到 periph_bus F 区。变量/FB 状态跨周期保持。
 *
 *  上下文: Non-RealTime (plc_core 10ms SCHED_OTHER 线程)。
 *  @Thread-Safety: 单线程执行 — plc_core 保证同一 PlcVm_t 仅扫描线程
 *    访问; RPC 侧读数值走 plc_core 的 mutex 快照, 不直接触 vm。
 *
 *  运行期错误 (div/mod 零, WHILE 超时): 置 runtime_err + err_line,
 *    本周期中止。调用方据此转 ERROR 态停机 (事件 P1-4)。
 * ===================================================================== */

#include "plc_lang.h"
#include "periph_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PLC_VM_ERR_MSG_LEN 64

/* FB 实例运行态 (IEC 语义: 未赋值参数沿用上次, 首周期 IN=FALSE/PT=0) */
typedef struct {
    uint8_t  kind;        /* PlcFbKind */
    uint8_t  prev_in;     /* 上一周期 IN/CLK (CTU 上升沿/触发器记忆) */
    uint8_t  q;           /* Q 输出锁存 */
    int32_t  pt;          /* TON/TOF: 最近一次 PT */
    int32_t  pv;          /* CTU: 最近一次 PV */
    int64_t  elapsed;     /* TON/TOF: 累计计时 ms */
    int32_t  cv;          /* CTU: 计数值 */
} PlcFbState_t;

typedef struct {
    PlcProgram_t *prog;               /* 借用引用 (plc_core 持有所有权) */
    PlcVal        cells[PLC_SYS_VAR_MAX + PLC_USER_VAR_MAX];   /* 变量值 */
    int16_t       fb_of[PLC_SYS_VAR_MAX + PLC_USER_VAR_MAX];   /* var→FB 槽 (-1=非 FB) */
    PlcFbState_t  fb[PLC_USER_VAR_MAX];
    int           n_fb;
    /* P1.5-2 (2026-09-15): 用户变量强制 (输出强制语义 — 程序写被覆写,
     * watch/F 发布/下周期读到的都是强制值)。
     * 写者: RPC/OPC UA (plc_core_force_var, release); 读者: 扫描线程
     * 每周期末 PlcVm_ApplyForce (acquire)。索引 = 用户变量序 (var_idx - 系统数)。
     * PlcVm_Init 清零 → 强制不跨 start/stop 重启 (文档化语义)。 */
    _Atomic uint8_t  force_on[PLC_USER_VAR_MAX];
    _Atomic double   force_val[PLC_USER_VAR_MAX];
    /* P1-3: M 码握手电平语义 — G_MCODE_STROBE=0 期间持续清零 F_MCODE_FIN
     * (stale FIN 不得完成下一次握手; VM 层实现, 用户程序无感) */
    uint8_t       prev_strobe;   /* 保留 (诊断), 电平语义下不再必需 */
    /* 运行期错误 */
    int      runtime_err;
    int32_t  err_line;
    char     err_msg[PLC_VM_ERR_MSG_LEN];
} PlcVm_t;

/* @Context: Non-RealTime (plc_core, start 时一次)
 * @return 0=成功; -1=参数空 */
int  PlcVm_Init(PlcVm_t *vm, PlcProgram_t *prog);

/* @Context: Non-RealTime (plc_core 扫描线程, 每周期)
 * @param g          本周期 G 区快照 (PeriphBus_ReadG 的产物)
 * @param dt_ms      距上周期实际毫秒 (计时器用; 首周期传 10)
 * @param deadline_ns WHILE 超时界限 (CLOCK_MONOTONIC ns, 超时判运行错误)
 * @return 0=本周期完成; -1=运行期错误 (vm->err_line/err_msg 填充) */
int  PlcVm_Cycle(PlcVm_t *vm, const PeriphG_t *g, int64_t dt_ms,
                 uint64_t deadline_ns);

/* @Context: Non-RealTime (plc_core 扫描线程, PlcVm_Cycle 成功后)
 * 周期末应用强制表: 强制变量的 cell 覆写为 force_val (按声明类型转换)。
 * 必须在快照更新/F 发布之前调用 (watch 与 F 区呈现强制值)。 */
void PlcVm_ApplyForce(PlcVm_t *vm);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PLC_VM_H */
