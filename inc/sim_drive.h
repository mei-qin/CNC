#ifndef SIM_DRIVE_H
#define SIM_DRIVE_H

/* =====================================================================
 *  sim_drive.h  ——  Sim 模式数字孪生驱动器（L1 CiA402 镜像 + L2 一阶伺服）
 *
 *  定位: 在 g_sim_mode==1 时接管 axis_pdo_write/read_sw/read_pos/
 *        read_follow_err 的 sim 分支, 让 sim 中能演练真硬件才会出现的
 *        CiA402 状态机响应、跟随误差监控、故障注入与恢复、优雅下电时序。
 *
 *  与 RT 主循环 g_axis[i].cia_step 的关系:
 *    - cia_step   : RT 线程视角, 描述"上层使能进度" (case 0..4)
 *    - sim_cia_state: sim 驱动器内部状态, 描述"驱动器对 CW 的响应结果"
 *    二者通过 SW 反馈耦合: RT 写 CW → sim_cia_state 推进 → SW 暴露给 RT 判断
 *
 *  线程安全:
 *    - 生产者: ecat_thread_rt (1ms 周期, 通过 axis_pdo_write 调 step_axis)
 *    - 消费者: 同一线程内的 axis_pdo_read_* (查 sim 内部状态)
 *    - 故障注入: 非实时 RPC 线程写 fault_injected 标志, RT 读后置 FAULT
 *    所有字段访问都在 RT 线程内闭环, 无需锁; fault_injected 跨线程用 atomic
 *    语义保证 (单 bit bool 标志, 写者置 1, RT 读者清 0, 丢失一次注入无害)。
 * ===================================================================== */

#include "axis_cfg.h"
#include <stdint.h>
#include <stdatomic.h>

/* ===== L1: sim 驱动器内部 CiA402 状态 ===== */
/* 与 axis_cfg.h 的 SW_* 宏一一对应 */
typedef enum {
    SIM_CIA_SWITCH_ON_DISABLED = 1,  /* SW=0x0040, 等 CW_SHUTDOWN (0x0006)        */
    SIM_CIA_READY_TO_SWITCH_ON = 2,  /* SW=0x0021 (SHUTDOWN_RDY), 等 CW_SWITCH_ON */
    SIM_CIA_SWITCHED_ON        = 3,  /* SW=0x0023, 等 CW_ENABLE_OP (0x000F)       */
    SIM_CIA_OP_ENABLED         = 4,  /* SW=0x0027, 运行态                         */
    SIM_CIA_FAULT              = 7,  /* SW=0x0008 (SW_ERROR), 等 CW_FAULT_RESET 上升沿 */
} SimCia402State_t;

/* 单 motor (从站) 的 sim 状态 */
typedef struct {
    int32_t           pos;                 /* 当前实际位置 (脉冲)              */
    int32_t           target;              /* 当前目标位置 (脉冲)              */
    SimCia402State_t  cia_state;           /* CiA402 内部状态                  */
    uint16_t          last_cw;             /* 上周期 CW (边沿检测)             */
    int               fault_injected;      /* 外部 RPC 注入故障标志            */
    int               target_reach_latch;  /* 到达目标锁存 (驱动 SW bit10)     */
    uint64_t          last_advance_cycle;  /* motor 级去重 (防双驱一周期双推)  */
} SimMotor_t;

/* 单轴的 sim 状态 (双驱轴含 2 个独立 motor) */
typedef struct {
    SimMotor_t motor[MAX_SLAVES_PER_AXIS];
    double     alpha;   /* 一阶低通系数, 默认 0.2, 范围 (0, 1)
                         * pos += (target - pos) * alpha
                         * 匀速运动时 err ≈ v_ms * (1-alpha)/alpha 脉冲
                         * alpha=0.2 → err ≈ v_ms * 4 (高速时易触发跟随误差报警) */
} SimAxisState_t;

extern SimAxisState_t  g_sim_axis[AXIS_NUM];
extern _Atomic uint64_t g_sim_rt_cycle;

/* ================== API ================== */

/* @Context: Non-RealTime Background Thread (SMC_InitAndStart sim 分支调用)
 * @Safe: 仅 memset + 字段初始化, 无 I/O, 无锁 (单线程初始化阶段)。
 * 把所有轴所有 motor 初始化为 SIM_CIA_SWITCH_ON_DISABLED, alpha=0.2, pos=0。
 * 必须在 g_axis[] 拓扑配置完成后调用。 */
void sim_drive_init_all(void);

/* @Context: 1ms Hard-RT Thread (从 axis_pdo_write sim 分支调用)
 * @Danger: NO BLOCKING, NO MATH.H, NO PRINTF, NO MALLOC.
 *
 * 每周期被 RT 调用一次 (每 slave 一次, 内部用 motor 级 last_advance_cycle
 * 防止双驱轴一周期被双推)。
 *
 * 推进顺序:
 *   (a) CiA402 状态机: 按 last_cw → cw 边沿推进, fault_injected 优先置 FAULT
 *   (b) 一阶低通: 仅在 OP_ENABLED 态推进 pos += (target - pos) * alpha
 *   (c) target_reach 锁存: abs(target - pos) < 2 时置 1 (驱动 SW bit10)
 *   (d) 退出 FAULT 态时由本函数清零 fault_injected
 */
void sim_drive_step_axis(int axis_idx, int slave_subidx,
                          uint16_t cw, int32_t target, uint64_t cycle);

/* @Context: 1ms Hard-RT Thread
 * @Danger: 只读, 无副作用。
 * 返回 (axis_idx, subidx) 对应 motor 当前 SW 字 (含 SW_ERROR/SW_TARGET_REACH) */
uint16_t sim_drive_get_sw(int axis_idx, int slave_subidx);

/* @Context: 1ms Hard-RT Thread
 * @Danger: 只读。返回 sim_actual_pos (一阶低通推算结果) */
int32_t sim_drive_get_pos(int axis_idx, int slave_subidx);

/* @Context: 1ms Hard-RT Thread
 * @Danger: 只读。返回 target - pos (脉冲) */
int32_t sim_drive_get_follow_err(int axis_idx, int slave_subidx);

/* @Context: Non-RealTime Background Thread (RPC 线程)
 * @Thread-Safety: 写 fault_injected 标志, RT 线程下一周期 step_axis 读后置 FAULT。
 * 返回 0=成功, -1=参数越界 */
int sim_drive_inject_fault(int axis_idx, int slave_subidx);

/* @Context: Non-RealTime Background Thread (RPC 线程)
 * @Thread-Safety: 写 alpha 字段, RT 线程下一周期生效。
 * alpha 必须在 (0, 1) 范围内, 否则返回 -1 */
int sim_drive_config_alpha(int axis_idx, double alpha);

/* @Context: Non-RealTime Background Thread (RPC 线程)
 * @Thread-Safety: 直接给 motor[1].pos 加 offset (一次性写入, RT 下一周期消费).
 *   B2 (2026-07-23) 模拟双驱龙门轴主从机械静态差. 用于 pre-align 算法 sim 验证.
 *   axis_idx 必须是 slave_count>=2 的双驱轴; 否则返回 -1.
 *   offset_pulse: 直接加到 slave motor pos 的偏移量 (模拟机械梁倾斜或安装错位).
 *   注入后 master/slave pos 形成静态差, axis_homing pre-align 阶段以对称到中点算法
 *   (p_center=(p_m+p_s)/2, 设 homing_shift 让主从命令都=p_center) 消除静态差.
 *   注: 实机不需要此 mock (机械差天然存在), 仅 sim 验证用. */
int sim_drive_inject_gantry_offset(int axis_idx, int32_t offset_pulse);

/* @Context: Non-RealTime Background Thread (pre-align worker, axis_ctrl.c)
 * @Thread-Safety: 直接把 motor[subidx].pos 按一阶低通推向 target (与 sim_drive_step_axis
 *   同款整数算法, 但绕过 cia_state 门限 + PDO 写跳过).
 *   B2 (2026-07-23) 用途: sim 模式下 homing 处于 RUNNING 时 RT 冻结该轴 PDO 写,
 *   实际电机位置不更新, pre-align 收敛轮询读不到真实收敛. 此函数让 pre-align worker
 *   在 sim 下手动把实际位置推向 p_center, 模拟实机伺服在 homing 期间跟随命令的收敛.
 *   alpha 为固定健康速率(与 ConfigSimDynamics 的 gap 衰减解耦), 必须在 (0,1).
 *   返回 0=成功, -1=参数越界 */
int sim_drive_step_toward(int axis_idx, int slave_subidx, int32_t target, double alpha);

/* @Context: 1ms Hard-RT Thread
 * @Danger: 只读, O(AXIS_NUM * MAX_SLAVES_PER_AXIS) ≤ 10 次比较。
 * slave_id → (axis_idx, subidx) 线性查找。
 * 找到返回 0 并写 *axis_idx_out / *subidx_out; 未找到返回 -1 */
int sim_drive_lookup_slave(int slave_id, int *axis_idx_out, int *subidx_out);

#endif /* SIM_DRIVE_H */
