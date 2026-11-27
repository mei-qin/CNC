#ifndef ECAT_CORE_H

#define ECAT_CORE_H

#include "soem/soem.h"
#include "osal_compat.h"
#include "axis_cfg.h"


/************************ SOEM核心函数声明 ************************/
// 1. 时间工具：纳秒级时间累加
void add_time_ns(ec_timet *ts, int64 addtime);

// 2. DC同步PI函数：适配SOEM旧版本（DCcycle），五轴共用时钟同步
void ec_sync(int64 reftime,int64 cycletime,int64 *offsettime);

// 3. ECAT主站启动：包含PDO映射/DC配置/SAFE_OP/SDO/OP切换（核心封装）
void ecat_bringup(char *ifname);

/* ================= P0-4a (2026-09-10): 期望拓扑校验 =================
 * 防slave索引漂移: IOmap/PDO地址按拓扑位置分配, 增减/换序从站会让
 * 所有 slave_id 引用整体漂移 (读邻站数据 = 安全事故)。bring-up 枚举后
 * 按SII vendor/product对照期望表, 不符拒绝进OP。
 * 表空(count=0) = 不校验, 兼容现状; 由 kernel_init_hw 从 hw 档案
 * [ecat_topo] 段在 SMC_InitAndStart 之前注入。 */
#define ECAT_TOPO_MAX 32
typedef struct {
    uint32_t vendor;    /* SII manufacturer code, 0 = 该位通配 */
    uint32_t product;   /* SII product code,     0 = 该位通配 */
} EcatTopoExpect_t;

/* @Context: Non-RealTime (kernel_init_hw, SMC_InitAndStart 之前)
 * @Thread-Safety: bring-up 前单写者; RT 线程启动后严禁调用。 */
void ecat_set_expected_topo(const EcatTopoExpect_t *list, int count);

/************************ 伺服诊断 / 总线恢复 (Non-RT, 2026-11-27) ************************/
/* 从站角色名 ("X" / "Y sub" / "IO" / "?" = 未知), bring-up 枚举后建表 */
const char *ecat_slave_role_name(int slave);

/* 诊断线程缓存的 0x603F 故障码: 0=未检出故障, >0=故障码, -1=SDO 读取失败, -2=使能超时 */
int ecat_servo_err_code(int axis_idx, int sub);

/* 总线故障恢复 (System.Reset 调用): wkc 已恢复且各站回到 OP 才解除锁存。
 * @return 0=已解除, -1=wkc 仍不符 (sim: 注入仍在), -2=有从站回不到 OP */
int ecat_bus_recover(void);

/************************ SOEM线程函数声明（适配OSAL） ************************/
// 1. 实时控制线程：1ms周期，五轴核心控制（RT线程，优先级高）
OSAL_THREAD_FUNC_RT ecat_thread_rt(void *arg);

// 2. 故障检查线程：10ms周期，非实时，检测轴状态/故障（不影响实时性）
OSAL_THREAD_FUNC ecat_thread_chk(void *arg);

#endif // ECAT_CORE_H
