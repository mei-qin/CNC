#ifndef LIMIT_MAP_H
#define LIMIT_MAP_H

/* =====================================================================
 *  limit_map.h —— 机械轴限位 → 输入来源 映射中心 (Phase 1, 2026-11-23)
 *
 *  定位 (第一阶段任务书: 跨从站限位映射离线实现与验证):
 *    机器档案按 [机械轴 × 正/负方向] 配置限位输入来源 (从站 + 输入选择 + 极性),
 *    本模块提供唯一的共用映射逻辑: 输入 = 配置表 + 按从站区分的样本数组,
 *    输出 = 每个 轴×方向 的诊断结果。纯逻辑、无 sim/内核依赖,
 *    独立测试程序 (tests/test_limit_map.c) 与 RPC 诊断查询共用同一份代码。
 *
 *  输入字约定 (ASDA-B3-E 0x60FD 数字输入, 本阶段边界止于已解码 uint32_t):
 *    bit 0    = NL (负限位功能状态, 驱动器已解释)
 *    bit 1    = PL (正限位功能状态, 驱动器已解释)
 *    bit 16-19= DI1-DI4 (物理端子原始电平)
 *  极性规则:
 *    选 DI1-DI4 时按配置的有效电平解释 (active_high 反转语义);
 *    选 NL/PL 时直接使用驱动器报告的功能状态, 不做任何反相
 *    (驱动器内部 P2-xx 已把端子电平折算成功能位, 再反相 = 双重取反)。
 *
 *  结果四态 (任务书一.2, 严禁把"未配置"解释为"确认现场没有开关"):
 *    UNCONFIGURED  : 档案中该 轴×方向 无配置 (旧档案兼容路径)
 *    UNKNOWN       : 已配置但来源站不存在 / 样本缺失 / 样本无效
 *                    (即使缓冲区保留上次有效输入字, 也返回未知)
 *    NOT_TRIGGERED : 样本有效, 按极性解释后电平未激活
 *    TRIGGERED     : 样本有效, 按极性解释后电平激活
 * ===================================================================== */

#include <stdint.h>
#include "axis_cfg.h"      /* AXIS_NUM */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 输入选择 (档案 [limit_src] input 字段; 值即档案解析后的存储值) ---- */
typedef enum {
    LMIN_DI1 = 1,   /* 0x60FD bit16 */
    LMIN_DI2,       /* 0x60FD bit17 */
    LMIN_DI3,       /* 0x60FD bit18 */
    LMIN_DI4,       /* 0x60FD bit19 */
    LMIN_NL,        /* 0x60FD bit0  (功能位, 不反相) */
    LMIN_PL,        /* 0x60FD bit1  (功能位, 不反相) */
} LimitInput_t;

/* ---- 诊断结果四态 ---- */
typedef enum {
    LM_RES_UNCONFIGURED   = 0,
    LM_RES_UNKNOWN        = 1,
    LM_RES_NOT_TRIGGERED  = 2,
    LM_RES_TRIGGERED      = 3,
} LimitResult_t;

/* ---- 单从站样本 (word 与 valid 必须作为同一样本成对消费, 见 sim_drive 打包原子) ---- */
typedef struct {
    uint32_t word;    /* 完整 32 位 0x60FD 输入字 (解码后) */
    int      valid;   /* 0 = 本周期样本无效/缺失 → 结果未知, word 仅作证据回显 */
} LimitSample_t;

/* ---- 单 轴×方向 配置 (dir: 0 = 正端 '+', 1 = 负端 '−') ---- */
typedef struct {
    int slave;        /* 来源从站站号 (1-based; 运行期不存在 → UNKNOWN, 非配置错误) */
    int input;        /* LimitInput_t */
    int active_high;  /* DI1-4 有效电平: 1=高有效 0=低有效; NL/PL 忽略本字段 */
    int configured;   /* 0 = 档案未配置该方向 */
} LimitSrcCfg_t;

/* ---- 单 轴×方向 诊断输出 ---- */
typedef struct {
    LimitResult_t result;
    int      slave;         /* 来源站 (未配置=0) */
    int      input;         /* LimitInput_t (未配置=0) */
    uint32_t raw_word;      /* 来源站当前输入字原值 (证据回显) */
    int      sample_valid;  /* 来源站样本有效标志 (证据回显) */
} LimitDiag_t;

/* @Context: 纯函数 (任意线程)
 * @Safe: 无副作用。输入选择 → 0x60FD 掩码位。非法枚举返回 0。 */
uint32_t limit_map_bit(LimitInput_t in);

/* @Context: 纯函数 (RPC 诊断线程 / 独立测试程序共用)
 * @Safe: 无副作用, 无 I/O。
 * samples 按 slave-1 索引 (站号 1..n_slaves), cfg 为 [AXIS_NUM][2] 配置表。
 * @return 0=成功; -1=参数非法 (axis/dir 越界, 或 configured 但 input 枚举非法) */
int limit_map_eval(const LimitSrcCfg_t cfg[AXIS_NUM][2], const LimitSample_t *samples,
                   int n_slaves, int axis, int dir, LimitDiag_t *out);

/* ---- 运行期配置表 (kernel_init_hw apply 一次写入, 之后只读) ----
 * 零初始化 = 全部 UNCONFIGURED (kernel_init 非 hw 路径 / 旧档案的默认态)。 */

/* @Context: Non-RealTime (kernel_init_hw, SMC_InitAndStart 之前调用一次) */
void limit_map_set_runtime(const LimitSrcCfg_t (*tbl)[AXIS_NUM][2]);

/* @Context: Non-RealTime (RPC 诊断查询线程)
 * @return 运行期配置表只读引用 */
const LimitSrcCfg_t (*limit_map_runtime(void))[AXIS_NUM][2];

#ifdef __cplusplus
}
#endif

#endif /* LIMIT_MAP_H */
