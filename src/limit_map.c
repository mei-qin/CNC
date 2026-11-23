/* =====================================================================
 *  limit_map.c —— 机械轴限位 → 输入来源 映射 (纯逻辑, Phase 1)
 *  契约与极性规则见 inc/limit_map.h 头部注释。
 * ===================================================================== */

#include "limit_map.h"

#include <string.h>

/* 运行期配置表: 零初始化 = 全部 UNCONFIGURED。
 * 写者: kernel_init_hw (apply 11.8, 单线程初始化阶段, SMC_InitAndStart 前);
 * 读者: RPC 诊断查询线程 (SMC_QueryLimitDiag)。写完成后只读, 无锁。 */
static LimitSrcCfg_t g_limit_rt[AXIS_NUM][2];

uint32_t limit_map_bit(LimitInput_t in)
{
    switch (in) {
    case LMIN_NL:  return 1u << 0;   /* bit0  负限位功能位 */
    case LMIN_PL:  return 1u << 1;   /* bit1  正限位功能位 */
    case LMIN_DI1: return 1u << 16;  /* bit16 DI1 */
    case LMIN_DI2: return 1u << 17;  /* bit17 DI2 */
    case LMIN_DI3: return 1u << 18;  /* bit18 DI3 */
    case LMIN_DI4: return 1u << 19;  /* bit19 DI4 */
    default:       return 0;
    }
}

int limit_map_eval(const LimitSrcCfg_t cfg[AXIS_NUM][2], const LimitSample_t *samples,
                   int n_slaves, int axis, int dir, LimitDiag_t *out)
{
    if (!cfg || !out || axis < 0 || axis >= AXIS_NUM || (dir != 0 && dir != 1))
        return -1;

    const LimitSrcCfg_t *c = &cfg[axis][dir];
    if (c->configured && (c->input < LMIN_DI1 || c->input > LMIN_PL))
        return -1;   /* 档案解析层已挡; 直调调用者的表构造 bug */

    LimitDiag_t d;
    memset(&d, 0, sizeof(d));
    d.slave  = c->configured ? c->slave  : 0;
    d.input  = c->configured ? c->input  : 0;

    if (!c->configured) {
        /* 未配置 ≠ 现场没有开关, 只是档案没说 (旧档案兼容路径) */
        d.result = LM_RES_UNCONFIGURED;
    } else if (c->slave < 1 || c->slave > n_slaves || !samples) {
        /* 已配置但来源站不在当前拓扑 → 未知 (非配置错误: 解析期只验 1..32) */
        d.result = LM_RES_UNKNOWN;
    } else {
        const LimitSample_t *s = &samples[c->slave - 1];
        d.raw_word     = s->word;      /* 证据回显 (含无效样本的残留值) */
        d.sample_valid = s->valid;
        if (!s->valid) {
            d.result = LM_RES_UNKNOWN; /* 样本无效 → 未知, 不消费残留 word */
        } else {
            uint32_t bit = limit_map_bit((LimitInput_t)c->input);
            int active;
            if (c->input == LMIN_NL || c->input == LMIN_PL)
                active = (s->word & bit) != 0;              /* 功能位直取, 不反相 */
            else if (c->active_high)
                active = (s->word & bit) != 0;              /* DI 高有效 */
            else
                active = (s->word & bit) == 0;              /* DI 低有效 */
            d.result = active ? LM_RES_TRIGGERED : LM_RES_NOT_TRIGGERED;
        }
    }

    *out = d;
    return 0;
}

void limit_map_set_runtime(const LimitSrcCfg_t (*tbl)[AXIS_NUM][2])
{
    if (!tbl)
        memset(g_limit_rt, 0, sizeof(g_limit_rt));   /* 显式清空 = 全 UNCONFIGURED */
    else
        memcpy(g_limit_rt, tbl, sizeof(g_limit_rt));
}

const LimitSrcCfg_t (*limit_map_runtime(void))[AXIS_NUM][2]
{
    return (const LimitSrcCfg_t (*)[AXIS_NUM][2])g_limit_rt;
}
