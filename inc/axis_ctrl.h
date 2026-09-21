#ifndef AXIS_CTRL_H
#define AXIS_CTRL_H

#include "axis_cfg.h"
#include <stdint.h>
/************************ 五轴核心控制函数声明 ************************/
// 1. 五轴系统初始化：配置每个轴的从站ID、轴名、目标位置（核心配置）
void axis_sys_init(void);

// 2. 单轴SDO配置：给指定轴配置CiA402 PP模式（0x6060=1）
void axis_sdo_config_pp(int axis_idx);
void axis_sdo_config_mode(int axis_idx, uint8_t mode);

// 3. 单轴PDO写：写入控制字+目标位置
void axis_pdo_write(int slave_id, uint16 cw, int32 pos);

// 4. 单轴PDO读：读取状态字
uint16 axis_pdo_read_sw(int slave_id);

void axis_clear_target_reach(int axis_idx);

void axis_print_sw_detail(int axis_idx);

uint8_t axis_sdo_read_mode(int axis_idx);



void axis_config_csp_params(int axis_idx);
void axis_read_csp_status(int axis_idx);
void check_pdo_mapping(int axis_idx);

void read_error_history(int axis_idx);
void axis_pdo_write_cw_only(int axis_idx, uint16 cw);
void diagnose_sync_failure(int axis_idx);
void axis_read_txpdo(int axis_idx);

int32 axis_pdo_read_pos(int slave_id);
int32_t axis_pdo_read_follow_err(int slave_id);
void axis_homing(int axis_idx);
// P0-1: 多轴串行回零 + all-or-nothing 回滚 (HomeAll/G28 用)
int axis_homing_multi(const int *axis_indices, int count, int source);

void api_set_zero(int axis_idx);
void api_go_zero(int axis_idx,double speed);
void api_move_relative(int axis_idx,double distance,double speed);
void api_move_line_3d(double target_x,double target_y,double target_z,double speed_pulse_per_sec);
void api_move_3d_relative(double dx,double dy,double dz,double speed);
void wait_motion_done();
// P1 G68.2: 先等运动开始 (上限 500ms) 再等结束 — 消除 push 后的 is_moving 竞态
void wait_motion_settle(void);
void api_push_abs(double tx,double ty,double tz,double speed);
void api_push_rel(double tx,double ty,double tz,double speed);
void api_sync_planner_cursor();
double api_get_cursor(int axis_idx);
int is_trajectory_finished();
// G93 jerk-bounded 转移剖面 (论文1 IV-E, 2026-09-16): 配置 + 链状态清零
// (程序启动时调 reset; enable=0 时严格块完全退化为 IV-C 现行为)
extern int    g_g93_jerk_enable;
extern double g_g93_jerk_eps_v;
extern double g_g93_jerk_eps_t;
void api_g93_jerk_reset(void);
// 论文1 B0′ 消融 (2026-09-17, 复审 P06): 1=禁用 IV-B 时间再分配,
// 每微段继承整块进给字 (τ_b = n_b·T_b, E2 陷阱可执行复现)。
// 实验专用, 默认 0; 运行中禁止切换 (由 SMC_ConfigG93Ablation 把关)。
extern int g_g93_f_inherit;

/* 论文1 M1 替代设计 (2026-09-18): post-hoc 速度赤字 carry-over */
extern int    g_g93_posthoc;
extern double g_g93_posthoc_debt;
void api_g93_posthoc_reset(void);

/* 论文1 V2 仿真战役 (2026-09-18): IV-D 长度比重标定 关闭开关 (Naive 基线) */
extern int g_g93_ratio_off;

/* 论文1 B2 消融 (2026-09-19, S3 planner-rate-limiting): 1=关闭 G93 强一致豁免,
 * 下游 planner 正常短板限幅 (基线 B2 可执行复现)。实验专用, 默认 0;
 * 运行中禁止切换 (由 SMC_ConfigG93StrictOff 把关)。 */
extern int g_g93_strict_off;

/* 论文1 H1 消融 (S10, 2026-09-20): IV-A 长度基开关。
 * g_g93_tip_basis=1: 需求速率改按**刀尖欧氏长度 L^P** 反推 (旋转轴等效行程不
 *   计入), 而非机械轴等效长度 L^M。G93 strict 段是时间驱动
 *   (axis_ctrl.c: v_const = dist/T 覆盖入队速度), 故执行本身不变;
 *   变的只有第三步.5 可行性门看到的 req_v = v_declared·|Δ_i|/L^M
 *   —— 被系统性低估 m = L^M/L^P 倍 ⇒ 门放行, 绑定轴实际跑到 m·v_ref
 *   (硬限速被突破)。即 S0 归档基线 C=96°/s=5.33× 硬限的机理复现。
 * 仅实验用, 默认 0 (生产 = IV-A 物理基 L^M)。
 * 运行中禁止切换 (由 SMC_ConfigG93TipBasis 把关)。 */
extern int g_g93_tip_basis;

// 论文1 N22 (2026-09-17): enqueue 临界区耗时统计 (parser+bspline 写, RPC 读)
void api_enqueue_stats_reset(void);
void api_enqueue_stats_get(uint64_t *count_all, uint64_t *count_strict,
                           double *mean_us_all, double *mean_us_strict,
                           double *max_us_strict);
// 加载 keep-last-good (2026-09-09, opcua_probe §11-2/3): Load/Run 前暂存上一成功
// 加载身份, parser Program_Load 失败时恢复 (Fanuc 选程失败保留旧程序惯例)。
void api_program_identity_save(void);
int  api_program_identity_restore(void);
int  api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
// @Context: Non-RealTime Background Thread (parser / bspline)
// @Thread-Safety: Lock-Free SPSC 队列,write_head 由生产者串行独占写入
// G93 强一致性入队: 豁免 max_speed/max_jerk 短板限幅,并强制纯匀速 (T4=T_total),
// 保证 1ms 线程解析时绝对遵守 g93_dt_sec 时间预算。
// g93_dt_sec: 本微段时间预算(秒),必须 > 0;speed 应为 phys_dist / g93_dt_sec。
int  api_push_trajectory_g93(double target_pos[AXIS_NUM],
                              double speed,double acc,double dec,
                              double g93_dt_sec);
// F4 (2026-09-19): RTCP G93 **时间切分微段**专用入队。
//   @Why: RTCP 的 G93 直线在 parser 端按 n = ceil(T*1000) 做**时间切分**,
//     而切分发生在 axis_ctrl 第三步.5 可行性判定**之前**。一旦该块被判不可行
//     (eff_strict=0), 回退到常规规划时, 每个 ~1ms 的微段都会各自做一次
//     0 → v → 0 的 S 曲线规划; 1ms 内全程受加速度限幅 (S_acc+S_dec >> S),
//     宏观表现为爬行 —— S0 B4 实测 τ/T = 46.69 (正常限速只应 5.33)。
//   @Fix: 该类微段改为**恒速通过** (v_start = v_end = v_target, 速度已在
//     第四步按绑定轴 max_speed 封顶)。整块累积时长 = dist/v_capped = T·r,
//     峰值轴速恰为 max_speed, 绝不越限; 同时保留时间切分 ⇒ RTCP 路径保真。
//   @Scope: 仅 RTCP 打碎路径使用。非 RTCP 的 G93 整块段 (S3/S4 用的
//     g93_nc_plain) 仍是单段, 回退即整块梯形, 行为不变。
int  api_push_trajectory_g93_slice(double target_pos[AXIS_NUM],
                                    double speed,double acc,double dec,
                                    double g93_dt_sec);
// C3 (2026-07-27): 刀补输出统一入队 (作为 CutterOutputCB 注册).
//   透传 G93 强一致性 + 强制 is_fillet=1 + parser_thread 读 g_state 冻结 WCS 快照
int  api_push_trajectory_cutter(double target_pos[AXIS_NUM],
                                 double speed,double acc,double dec,
                                 int is_g93_strict, double g93_dt_sec);
// 免抹圆透传入队: 写入 is_fillet=1 标记,planner_fillet_preprocess 跳过本段。
// 用于 B-Spline 引擎在锐角切批后直接透传的段,防止底层二次抹圆。
int  api_push_trajectory_passthrough(double target_pos[AXIS_NUM],
                                      double speed,double acc,double dec);
// 免抹圆透传 + 显式 WCS: B-Spline 线程专用, wcs 来自脏点捕获时的模态 WCS,
// 避免读 parser 线程的 g_state.modal_wcs 造成数据竞争。
// wcs_offset_snap: 段生效时的偏置向量快照 (脏点捕获时已固定), 透传到段内, RT 用它推导 UI 逻辑坐标。
int  api_push_trajectory_passthrough_wcs(double target_pos[AXIS_NUM],
                                          double speed,double acc,double dec,
                                          CoordSystem_t wcs,
                                          const double wcs_offset_snap[AXIS_NUM]);
// G93 强一致性入队 + 显式 WCS: B-Spline 线程专用 (G93 + BSpline 复合路径)。
int  api_push_trajectory_g93_wcs(double target_pos[AXIS_NUM],
                                   double speed,double acc,double dec,
                                   double g93_dt_sec, CoordSystem_t wcs,
                                   const double wcs_offset_snap[AXIS_NUM]);
// @Context: Non-RealTime Background Thread (RTCP 路径专用)
// @Thread-Safety: queue_spinlock 互斥,与其他 push 函数共享。
// RTCP 路径包装: 入队时设置 is_rtcp_active=1 元数据。
//   target_pos 必须已是物理关节坐标 (Parser 已调用 apply_rtcp_to_pos)。
//   RT 线程读取时不做逆解,直接插补物理关节空间,保持 1ms 硬实时纯粹性。
int  api_push_trajectory_rtcp(double target_pos[AXIS_NUM],
                               double speed,double acc,double dec);
int  api_push_mcode(int m_code, double s_value, double p_value, double q_value, double r_value);
void api_push_continuous_segment(double val_x,double val_y,double val_z,double speed_sec);
void api_flush_planner();
void api_motion_pause();
void api_motion_resume();
int  api_alarm_reset();
#endif // AXIS_CTRL_H
