/* =====================================================================
 *  sim_drive.c  ——  Sim 模式数字孪生驱动器实现
 *
 *  L1 CiA402 状态机镜像: 按 CW 低 4 位边沿推进 sim_cia_state
 *  L2 一阶伺服模型:     pos += (target - pos) * alpha
 *
 *  设计原则:
 *    1. RT 路径零分配零阻塞零 libm
 *    2. 真硬件路径零侵入 (本文件只被 axis_pdo_* 的 sim 分支调用)
 *    3. motor 级 last_advance_cycle 去重 (双驱轴每周期 write 两次)
 *    4. fault_injected 标志单向通信: RPC 写 1, RT 读后清 0
 * ===================================================================== */

#include "sim_drive.h"
#include "global_def.h"   /* g_axis[] */
#include <string.h>

SimAxisState_t   g_sim_axis[AXIS_NUM];
_Atomic uint64_t g_sim_rt_cycle = 0;

/* =====================================================================
 *  内部工具: CiA402 状态推进
 *
 *  转移规则 (仅看 CW 低 4 位, 满足现有 case 0/1/2/3 使能流程):
 *    SHUTDOWN (0x0006) → SWITCH_ON (0x0007) → ENABLE_OP (0x000F)
 *  降级同理可逆。
 *  fault_injected 标志优先一切, 强制置 FAULT。
 *  FAULT 态下只有 CW_FAULT_RESET (0x0080) 上升沿能退出 → 回 SWITCH_ON_DISABLED。
 * ===================================================================== */
static SimCia402State_t sim_cia_advance(SimCia402State_t cur,
                                          uint16_t last_cw, uint16_t cw,
                                          int fault_injected)
{
    /* (1) 外部注入故障: 优先一切, 强制进 FAULT */
    if (fault_injected) {
        return SIM_CIA_FAULT;
    }

    /* (2) FAULT 态: 只看 CW_FAULT_RESET (bit7) 上升沿 */
    if (cur == SIM_CIA_FAULT) {
        if ((cw & CW_FAULT_RESET) && !(last_cw & CW_FAULT_RESET)) {
            return SIM_CIA_SWITCH_ON_DISABLED;
        }
        return SIM_CIA_FAULT;
    }

    /* (3) 防御: 未初始化或异常状态值 (含 cur=0 race window) 视为 SWITCH_ON_DISABLED
     *       避免状态机永久卡死在 default 分支 */
    if (cur != SIM_CIA_SWITCH_ON_DISABLED &&
        cur != SIM_CIA_READY_TO_SWITCH_ON &&
        cur != SIM_CIA_SWITCHED_ON &&
        cur != SIM_CIA_OP_ENABLED) {
        return SIM_CIA_SWITCH_ON_DISABLED;
    }

    /* (4) 健康路径: 按 CW 低 4 位边沿转移 */
    uint16_t cw_low = cw & 0x000F;
    switch (cur) {
        case SIM_CIA_SWITCH_ON_DISABLED:
            if (cw_low == 0x0006) return SIM_CIA_READY_TO_SWITCH_ON;
            break;
        case SIM_CIA_READY_TO_SWITCH_ON:
            if (cw_low == 0x0007) return SIM_CIA_SWITCHED_ON;
            if (cw_low == 0x000F) return SIM_CIA_OP_ENABLED;   /* 跳过 SWITCHED_ON */
            if (cw_low == 0x0000) return SIM_CIA_SWITCH_ON_DISABLED;
            break;
        case SIM_CIA_SWITCHED_ON:
            if (cw_low == 0x000F) return SIM_CIA_OP_ENABLED;
            if (cw_low == 0x0006) return SIM_CIA_READY_TO_SWITCH_ON;
            break;
        case SIM_CIA_OP_ENABLED:
            if (cw_low == 0x0007) return SIM_CIA_SWITCHED_ON;
            if (cw_low == 0x0006) return SIM_CIA_READY_TO_SWITCH_ON;
            break;
        default:
            break;
    }
    return cur;
}

/* 状态 → SW 字映射 (符合 CiA402 规范 + 项目 SW_* 宏) */
static uint16_t sim_cia_to_sw(SimCia402State_t st, int target_reached)
{
    uint16_t sw;
    switch (st) {
        case SIM_CIA_SWITCH_ON_DISABLED:   sw = 0x0040; break;  /* bit6 only */
        case SIM_CIA_READY_TO_SWITCH_ON:   sw = SW_SHUTDOWN_RDY;  break;  /* 0x0021 */
        case SIM_CIA_SWITCHED_ON:          sw = SW_SWITCHED_ON;   break;  /* 0x0023 */
        case SIM_CIA_OP_ENABLED:           sw = SW_OP_ENABLED;    break;  /* 0x0027 */
        case SIM_CIA_FAULT:                sw = SW_ERROR;         break;  /* 0x0008 */
        default:                           sw = 0x0000; break;
    }
    if (target_reached && st == SIM_CIA_OP_ENABLED) {
        sw |= SW_TARGET_REACH;   /* bit10 */
    }
    return sw;
}

/* =====================================================================
 *  公开 API
 * ===================================================================== */

void sim_drive_init_all(void)
{
    memset(g_sim_axis, 0, sizeof(g_sim_axis));
    for (int i = 0; i < AXIS_NUM; i++) {
        g_sim_axis[i].alpha = 0.2;
        for (int s = 0; s < MAX_SLAVES_PER_AXIS; s++) {
            g_sim_axis[i].motor[s].cia_state = SIM_CIA_SWITCH_ON_DISABLED;
            g_sim_axis[i].motor[s].last_cw   = 0x0000;
        }
    }
}

void sim_drive_step_axis(int axis_idx, int slave_subidx,
                          uint16_t cw, int32_t target, uint64_t cycle)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return;
    if (slave_subidx < 0 || slave_subidx >= MAX_SLAVES_PER_AXIS) return;

    SimAxisState_t *a = &g_sim_axis[axis_idx];
    SimMotor_t     *m = &a->motor[slave_subidx];

    /* (a) CiA402 状态机推进 (含 fault_injected 优先置 FAULT) */
    SimCia402State_t prev = m->cia_state;
    m->cia_state = sim_cia_advance(m->cia_state, m->last_cw, cw, m->fault_injected);
    m->last_cw   = cw;

    /* (d) 退出 FAULT 态时清零 fault_injected (允许下次重新注入) */
    if (prev == SIM_CIA_FAULT && m->cia_state != SIM_CIA_FAULT) {
        m->fault_injected = 0;
    }

    /* motor 级去重: 同一 motor 同一 cycle 只推进一次物理 */
    if (m->last_advance_cycle == cycle) {
        return;
    }
    m->last_advance_cycle = cycle;

    /* (b) 一阶低通推进 + (c) target_reach 锁存: 仅 OP_ENABLED 态 */
    if (m->cia_state == SIM_CIA_OP_ENABLED) {
        m->target = target;
        /* pos += (target - pos) * alpha
         * 用整数运算避免浮点累积误差; alpha∈(0,1) 物理稳定
         * 为了不损失精度, 先把 (target-pos)*alpha 放大到 alpha*65536 计算
         * 然后右移 16 位得到整数增量 */
        int64_t err = (int64_t)target - (int64_t)m->pos;
        int64_t scaled_alpha = (int64_t)(a->alpha * 65536.0);
        int32_t delta = (int32_t)((err * scaled_alpha) >> 16);
        m->pos += delta;

        int32_t abs_err = (err >= 0) ? (int32_t)err : (int32_t)(-err);
        m->target_reach_latch = (abs_err < 2) ? 1 : 0;
    } else {
        /* 非 OP 态: 位置冻结, 不锁 target_reach */
        m->target_reach_latch = 0;
    }
}

uint16_t sim_drive_get_sw(int axis_idx, int slave_subidx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return 0x0000;
    if (slave_subidx < 0 || slave_subidx >= MAX_SLAVES_PER_AXIS) return 0x0000;
    SimMotor_t *m = &g_sim_axis[axis_idx].motor[slave_subidx];
    return sim_cia_to_sw(m->cia_state, m->target_reach_latch);
}

int32_t sim_drive_get_pos(int axis_idx, int slave_subidx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return 0;
    if (slave_subidx < 0 || slave_subidx >= MAX_SLAVES_PER_AXIS) return 0;
    return g_sim_axis[axis_idx].motor[slave_subidx].pos;
}

int32_t sim_drive_get_follow_err(int axis_idx, int slave_subidx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return 0;
    if (slave_subidx < 0 || slave_subidx >= MAX_SLAVES_PER_AXIS) return 0;
    SimMotor_t *m = &g_sim_axis[axis_idx].motor[slave_subidx];
    return m->target - m->pos;
}

int sim_drive_inject_fault(int axis_idx, int slave_subidx)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return -1;
    if (slave_subidx < 0 || slave_subidx >= MAX_SLAVES_PER_AXIS) return -1;
    g_sim_axis[axis_idx].motor[slave_subidx].fault_injected = 1;
    return 0;
}

int sim_drive_config_alpha(int axis_idx, double alpha)
{
    if (axis_idx < 0 || axis_idx >= AXIS_NUM) return -1;
    if (!(alpha > 0.0) || !(alpha < 1.0)) return -1;
    g_sim_axis[axis_idx].alpha = alpha;
    return 0;
}

int sim_drive_lookup_slave(int slave_id, int *axis_idx_out, int *subidx_out)
{
    for (int i = 0; i < AXIS_NUM; i++) {
        for (int s = 0; s < g_axis[i].slave_count; s++) {
            if (g_axis[i].slave_ids[s] == slave_id) {
                if (axis_idx_out) *axis_idx_out = i;
                if (subidx_out)   *subidx_out   = s;
                return 0;
            }
        }
    }
    return -1;
}
