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

void api_set_zero(int axis_idx);
void api_go_zero(int axis_idx,double speed);
void api_move_relative(int axis_idx,double distance,double speed);
void api_move_line_3d(double target_x,double target_y,double target_z,double speed_pulse_per_sec);
void api_move_3d_relative(double dx,double dy,double dz,double speed);
void wait_motion_done();
void api_push_abs(double tx,double ty,double tz,double speed);
void api_push_rel(double tx,double ty,double tz,double speed);
void api_sync_planner_cursor();
double api_get_cursor(int axis_idx);
int is_trajectory_finished();
int  api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
// @Context: Non-RealTime Background Thread (parser / bspline)
// @Thread-Safety: Lock-Free SPSC 队列,write_head 由生产者串行独占写入
// G93 强一致性入队: 豁免 max_speed/max_jerk 短板限幅,并强制纯匀速 (T4=T_total),
// 保证 1ms 线程解析时绝对遵守 g93_dt_sec 时间预算。
// g93_dt_sec: 本微段时间预算(秒),必须 > 0;speed 应为 phys_dist / g93_dt_sec。
int  api_push_trajectory_g93(double target_pos[AXIS_NUM],
                              double speed,double acc,double dec,
                              double g93_dt_sec);
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
