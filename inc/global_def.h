#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H

#include "soem/soem.h"
#include "axis_cfg.h"
#include "gcode_parser.h"
#include <pthread.h>
/************************ SOEM全局核心变量（b3_drive原有） ************************/
extern ecx_contextt ctx;                // SOEM主站上下文
extern uint8 IOmap[4096];               // PDO映射缓冲区（所有轴共用）
extern OSAL_THREAD_HANDLE thread_rt;    // 实时线程句柄
extern OSAL_THREAD_HANDLE thread_chk;   // 故障检查线程句柄
extern OSAL_THREAD_HANDLE thread_parser; // G-code解析线程句柄

extern int expectedWKC;                 // SOEM期望WKC值
extern int wkc;                         // 实际WKC值
extern int mappingdone;
extern volatile int dorun;
extern int inOP;
extern int dowkccheck, currentgroup;    // SOEM故障检查标志

/************************ 实时控制全局变量 ************************/
extern int cycle;                       // 实时周期计数
extern int64 cycletime;                 // 实时周期（1ms）
extern int64 syncoffset, timeerror;     // DC同步参数
extern float pgain, igain;              // DC同步PI参数
extern int64_t g_cycle;
extern int g_all_axis_enabled; 
extern int64 ec_DCtime;                 // SOEM 提供的全局变量，单位 ns
/************************ 五轴全局核心变量 ************************/
extern AxisCtrl_t g_axis[AXIS_NUM];     // 五轴控制数组（核心！）
extern int g_all_axis_op_ready;         // 全局标志：五轴均使能就绪
extern int g_all_axis_reach;            // 全局标志：五轴均目标到达
extern int g_csp_ready; 
extern Interpolator_t g_interpolator;   // 插补器状态结构体
extern CommandQueue_t g_cmd_queue;      // 运动命令队列
extern GCodeState_t g_state;             // 全局G-code状态变量
extern ParserControl_t g_parser_ctrl;     // 全局G-code解析控制变量
extern double g_pulse_per_unit[AXIS_NUM]; // 每轴脉冲/单位（如mm）转换系数
extern CoordManager_t g_coord_mgr; // 坐标系管理器
extern RtLog_t g_rt_log;           // 实时线程环形日志缓冲
extern PlannerConfig_t g_planner_config; // 规划器全局参数
extern pthread_mutex_t planner_mutex;    // 规划器互斥锁（parser线程 vs 看门狗线程）
extern _Atomic int g_sys_alarm_state;    // 系统报警状态: 0=正常, 1=软停机报警中

#endif // GLOBAL_DEF_H
