#ifndef ECAT_CORE_H

#define ECAT_CORE_H

#include "soem/soem.h"
#include "axis_cfg.h"


/************************ SOEM核心函数声明 ************************/
// 1. 时间工具：纳秒级时间累加
void add_time_ns(ec_timet *ts, int64 addtime);

// 2. DC同步PI函数：适配SOEM旧版本（DCcycle），五轴共用时钟同步
void ec_sync(int64 reftime,int64 cycletime,int64 *offsettime);

// 3. ECAT主站启动：包含PDO映射/DC配置/SAFE_OP/SDO/OP切换（核心封装）
void ecat_bringup(char *ifname);

/************************ SOEM线程函数声明（适配OSAL） ************************/
// 1. 实时控制线程：1ms周期，五轴核心控制（RT线程，优先级高）
OSAL_THREAD_FUNC_RT ecat_thread_rt(void *arg);

// 2. 故障检查线程：10ms周期，非实时，检测轴状态/故障（不影响实时性）
OSAL_THREAD_FUNC ecat_thread_chk(void *arg);

#endif // ECAT_CORE_H
