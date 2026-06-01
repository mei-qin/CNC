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
int  api_push_mcode(int m_code, double s_value);
void api_push_continuous_segment(double val_x,double val_y,double val_z,double speed_sec);
void api_flush_planner();
void api_motion_pause();
void api_motion_resume();
int  api_alarm_reset();
#endif // AXIS_CTRL_H
