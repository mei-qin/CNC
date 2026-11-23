#ifndef PARAM_STORE_H
#define PARAM_STORE_H

/* =====================================================================
 *  param_store.h —— 机床参数档案 (Machine Profile) 持久化中心
 *
 *  定位 (2026-08-28 市场化改造 P1+P2):
 *    把 rpc_server.c 的编译期 g_hw_* IBN 参数表外置为 INI 文件,
 *    并为运行期参数修改 (OPC UA CNC.Config / 9527 RPC) 提供
 *    "写入即存 + 审计" 的统一落盘通道。
 *
 *  分层原则 (不可打破):
 *    本模块只做 [数据存储 + 文件 I/O + 审计事件], 不 include smc_api.h,
 *    不调用任何 SMC_* — 参数应用一律由 rpc_server (kernel_init_hw) 驱动。
 *    依赖仅 event_logger.h / global_def.h (g_sim_mode), 无环形 include。
 *
 *  文件约定 (对齐 SMC_PROGRAM_DIR 模式):
 *    路径 = 环境变量 SMC_MACHINE_PROFILE, 默认 "./machine_profile.ini"。
 *    首次运行无文件 → 用编译期默认档案 (前期实机开发测试值);
 *    第一次运行期参数变更时自动生成完整档案文件 (self-bootstrap)。
 *    备份/恢复 = 复制该文件 (等价 Fanuc 参数输出/输入)。
 *
 *  生效级语义 (与 OPC UA 契约 §10 对齐):
 *    立即级  : 动力学/软限位/规划器/运动学 (SMC_Config* 静止闸门保证)
 *    重启级  : 拓扑 (EtherCAT 组态期), ppu (位置映射连续性, 运行期只入档案)
 *
 *  线程模型:
 *    SMC_Config* 钩子 (9527 handler 线程 / OPC UA server 线程) 与
 *    显式 save 并发 → 内部 pthread mutex 串行化; g_profile 一致性由
 *    mutex 保证, 文件写入为 tmp+rename+fsync 原子操作。
 *
 *  sim 保护:
 *    g_sim_mode==1 且未显式设 SMC_MACHINE_PROFILE 时, save() 拒绝写默认路径
 *    (防 sim 会话污染实机同目录档案); 显式指定档案路径 (测试/联调) 允许落盘。
 * ===================================================================== */

#include <stdint.h>
#include "axis_cfg.h"      /* AXIS_NUM */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 未填哨兵 (沿用 kernel_init_hw 的 IBN 填空约定) ---- */
#define PS_TODO_D   (-999999.0)
#define PS_TODO_I   (-999999)

/* ---- 档案文件版本 ---- */
#define PS_PROFILE_VERSION   1

/* ---- P0-2 (2026-09-09): 激光互锁规则表条数上限 (与 smc_protocol.h LASER_IL_MAX 对齐,
 *      kernel apply 侧 min() 双保险, 改任一必须同步另一处) ---- */
#define PS_LASER_IL_MAX      8

/* ---- P0-4a (2026-09-10): EtherCAT 期望拓扑表条数上限 (与 ecat_core.h ECAT_TOPO_MAX 对齐) ---- */
#define PS_ECAT_TOPO_MAX     32

/* ---- P1.5-1 (2026-09-14): 外围 AI 通道数上限 (与 periph_ai.h PERIPH_AI_CH_MAX 对齐) ---- */
#define PS_PERIPH_AI_MAX     4

/* ---- 审计事件 code (source = SOURCE_CONFIG, 见 event_logger.h 分段表) ---- */
#define PS_EV_LOAD_OK        0x0001   /* profile 文件加载成功 */
#define PS_EV_LOAD_FALLBACK  0x0002   /* 加载失败回退编译期默认 */
#define PS_EV_CHANGE         0x0003   /* 参数变更审计 (old→new) */
#define PS_EV_REJECT         0x0004   /* 参数被拒 (校验/闸门) */
#define PS_EV_SAVE           0x0005   /* 档案落盘 */
#define PS_EV_WRITE_PROTECT  0x0006   /* 写保护拒绝 */

/* ---- 单轴档案 (拓扑/当量/动力学/软限位/龙门/回零 全量) ---- */
typedef struct {
    /* 拓扑 (重启级, 仅档案) */
    int    is_dual;
    int    master_id;
    int    slave_id;
    /* 脉冲当量 (重启级) */
    double ppu;
    /* 动力学 (立即级) */
    int    dyn_type;         /* 0=线性 1=旋转 */
    double max_speed;
    double max_acc;
    double max_dec;
    double eq_radius;
    /* 软限位 (立即级) */
    int    sl_enable;
    double sl_neg;
    double sl_pos;
    /* P0 旋转轴最短路径解缠绕 (立即级): 1=G90 绝对指令 wrap-to-nearest
     * 仅 G90 逻辑坐标生效; G91 增量精确执行; G53 机械绝对直通 (逃生通道) */
    int    rot_wrap_enable;
    /* 龙门同步报警 (双驱轴生效) */
    int    gs_tol_pulse;
    int    gs_max_err_pulse;
    int    gs_err_time_ms;
    /* 回零 (v1.5: m35 = 回 WCS 零点运动) */
    int    hm_enable;
    int    hm_method;
    int    hm_direction;
    int    hm_timeout_ms;
    double hm_speed;            /* 回零速度 mm/s; >0 固定值, 0 = 轴 max_speed */
} ProfileAxis_t;

/* ---- 全局档案 ---- */
typedef struct {
    int    loaded;                        /* 1=load() 已完成, 钩子生效 */
    int    dirty;                         /* 1=影子与磁盘不一致 */
    int    axis_count;
    char   axis_order[AXIS_NUM][8];       /* 房间号分配顺序 (轴名=单字母) */
    ProfileAxis_t ax[AXIS_NUM];
    /* 五轴运动学 (立即级) */
    double tool_len;
    double pivot_x, pivot_y, pivot_z;
    /* P0 RTCP 奇异点阈值: 直线 R_max 预估 / 圆弧逐微段 phys/log 比 超过则硬报警
     * 默认 30.0; <=0 显式禁用检测 */
    double singularity_rmax;
    /* P1 (2026-09-04): 双摆头链序 0=R1 外层 (已交付默认) / 1=R2 外层 (C外B内腕)。
     * 同时参数化逆解装配与 G43.5 姿态解算; 实机观测确认后切换 */
    int    chain_order;
    /* 规划器 (立即级) */
    double corner_tolerance;
    double max_centripetal_acc;
    /* Z 安全抬升 (立即级) */
    double safe_z_mm;
    double lift_speed_mm_s;
    int    auto_on_alarm;
    /* 回零全局 */
    char   homing_order[16];              /* "ZXYBC" */
    int    auto_on_init;
    /* 激光互锁规则表 (立即级, P0-2 2026-09-09): count=0 = 内核默认 6 路链 */
    int    laser_il_count;                /* 0..PS_LASER_IL_MAX */
    int    laser_il_bit[PS_LASER_IL_MAX]; /* DI bit 0-15 */
    int    laser_il_mask[PS_LASER_IL_MAX];/* interlock 置位码 (bit0-14) */
    /* 激光 I/O 端子 PDO 布局 (立即级, P0-4a 2026-09-10): 默认 EL 级 16-bit */
    int    laser_io_di_off, laser_io_di_bytes;   /* 0-63 / 1-2 */
    int    laser_io_do_off, laser_io_do_bytes;
    int    laser_io_ao_off, laser_io_ao_stride;  /* 0-63 / 2-4 */
    /* 外围 AI 模拟输入 (立即级, P1.5-1 2026-09-14): 默认未配置 (count=0) */
    int    periph_ai_slave;                      /* -1 / 0-63 */
    int    periph_ai_off;                        /* byte 0-63 */
    int    periph_ai_stride;                     /* 2/4 */
    int    periph_ai_count;                      /* 0-4 */
    double periph_ai_fs[PS_PERIPH_AI_MAX];       /* 每通道满量程 (<=0 未配) */
    /* EtherCAT 期望拓扑 (重启级, P0-4a): bring-up 枚举后对照 SII vendor/product,
     * 不符拒绝进 OP (防 slave 索引漂移: 增减/换序从站)。count=0 = 不校验 */
    int    ecat_topo_count;                          /* 0..PS_ECAT_TOPO_MAX */
    uint32_t ecat_topo_vendor[PS_ECAT_TOPO_MAX];     /* SII manufacturer, 0=通配 */
    uint32_t ecat_topo_product[PS_ECAT_TOPO_MAX];    /* SII product code, 0=通配 */
    /* 机械轴限位输入来源 (Phase 1, 2026-11-23): [轴][dir], dir 0=正端 1=负端。
     * 与轴自身驱动无绑定 — 允许跨从站取信号 (如 Y+ 取 X 驱动 5 号站 DI3)。
     * cfg = 字段掩码 (bit0 slave / bit1 input / bit2 level): 解析期累见,
     * 收尾强制 ∈ {0, 0x7} (半配置致命), 故对外语义 0=未配置 0x7=已配置。
     * 未配置方向 = UNCONFIGURED (诊断口径, 非确认无开关)。值域:
     *   slave 1..PS_ECAT_TOPO_MAX (运行期站不存在 → 诊断 UNKNOWN, 非配置错误)
     *   input 1..6 = LimitInput_t (di1..di4/nl/pl)
     *   level 1=高有效 0=低有效 (仅 DI 生效; NL/PL 功能位不反相) */
    int    limit_src_slave[AXIS_NUM][2];
    int    limit_src_input[AXIS_NUM][2];
    int    limit_src_level[AXIS_NUM][2];
    int    limit_src_cfg[AXIS_NUM][2];
} MachineProfile_t;

/* =====================================================================
 *  API
 * ===================================================================== */

/* @Context: Non-RealTime (kernel_init_hw, EventLogger_Init 之后调用)
 * @Safe: 文件 I/O + 解析。失败/损坏/版本不识别 → 影子=编译期默认 + WARN 0x0002。
 * 成功 → 影子=默认值被文件覆盖后的合并结果 + INFO 0x0001。
 * 幂等: 重复调用重新加载。 */
void param_store_load(void);

/* @Context: Non-RealTime (kernel_init_hw apply 循环)
 * @return 档案只读引用 (load 之前返回编译期默认) */
const MachineProfile_t *param_store_get(void);

/* @Context: Non-RealTime (kernel_init_hw, SMC_Config* 应用之前)
 * @return 0=校验通过; -1=A 级存在未填项 (调用方必须拒绝启动)。
 *         B 级 (动力学/运动学未填) 仅打印警告, apply 侧跳过。 */
int param_store_check(void);

/* @Context: Non-RealTime (OPC UA SaveProfile / notify 写入即存路径)
 * @Safe: mutex + tmp+rename+fsync 原子写; g_sim_mode 时拒绝。
 * @return 0=成功, -1=sim 模式拒绝, -2=文件 I/O 失败 */
int param_store_save(void);

/* @Context: 任意 Non-RT 线程
 * @return 1=影子与磁盘不一致 (UI 的 ProfileDirty 节点数据源) */
int  param_store_dirty(void);
void param_store_mark_saved(void);

/* 运行期写档案不走 SMC (ppu 重启级语义): OPC UA handler 直接调。
 * @return 0=已更新+落盘; -1=轴不存在; -2=sim 拒绝; -3=I/O 失败 */
int param_store_set_axis_ppu(char axis_letter, double ppu);

/* =====================================================================
 *  SMC_Config* 成功路径钩子 (P2)
 *  语义: 更新影子对应字段; 与影子相同 (无变化) → 完全 no-op
 *        (boot 应用期天然静默, 运行期 no-op 写入不产生噪音);
 *        有变化 → 审计事件 0x0003 + 写入即存 (非 sim)。
 *        load() 之前调用一律忽略。
 * ===================================================================== */
void param_store_notify_topo(const char *axis_name, int is_dual, int master, int slave);
void param_store_notify_ppu(char axis_letter, double new_ppu);
void param_store_notify_dyn(char axis_letter, int type,
                            double v, double a, double d, double r);
void param_store_notify_softlimit(char axis_letter, int en, double neg, double pos);
void param_store_notify_gantry_sync(char axis_letter, int tol, int max, int time_ms);
void param_store_notify_gantry_align(char axis_letter, int tol, int timeout_ms);
void param_store_notify_planner(double tol, double acc);
void param_store_notify_kinematics(double tool, double px, double py, double pz);
void param_store_notify_safelift(double safe_z, double speed, int auto_on);
void param_store_notify_homing(char axis_letter, int method, int timeout, int dir);
void param_store_notify_homing_all(const char *order, int auto_on_init);

/* 档案文件路径 (env SMC_MACHINE_PROFILE → ./machine_profile.ini); 测试/诊断用 */
const char *param_store_path(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* PARAM_STORE_H */
