#ifndef SMC_CONTROLLER_SDK_H
#define SMC_CONTROLLER_SDK_H

/* =====================================================================
 *  SmcControllerSdk.h  ——  CAM / HMI 侧 RPC 客户端 SDK (C++)
 *
 *  使用方式: CAM 团队 #include 本头文件, 实例化 SmcController,
 *            Connect() 后即可像本地方法一样操作 CNC Core 的全部 API。
 *
 *  覆盖范围: 与 smc_api.h 全量 23 个 API 一一对应 (Drop "SMC_" 前缀)。
 *
 *  运行环境: 跨平台, 同时支持 Windows (Winsock2) 与 Linux (POSIX)。
 *            不涉及任何实时上下文, 是普通用户态阻塞式 I/O 客户端。
 *
 *  线程安全: 单实例非线程安全; 多线程请每线程一个实例, 或外加锁。
 *
 *  返回值约定:
 *    - 所有方法返回 bool 表示协议层成败 (true=成功, false=见 LastError())
 *    - 原本返回 int 的 SMC_* 函数, 通过 out_ret_code 输出业务返回值
 *    - 原本返回 double 的函数, 通过 out_position/out_xxx 输出
 *    - 原本 void 返回的函数, 仅以 bool 表达成败
 * ===================================================================== */

#include <string>
#include <cstdint>
#include <mutex>

#include "smc_protocol.h"   /* 共享协议定义 + SMC_AXIS_ALL 等常量 */

/* 跨平台 socket 句柄类型抽象 */
#ifdef _WIN32
    #ifndef _WIN32_LEAN_AND_MEAN
        #define _WIN32_LEAN_AND_MEAN
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>
    using smc_socket_t   = SOCKET;
    #define SMC_INVALID_SOCKET  INVALID_SOCKET
#else
    using smc_socket_t   = int;
    #define SMC_INVALID_SOCKET  (-1)
#endif

class SmcController
{
public:
    SmcController();
    ~SmcController();

    /* 禁止拷贝 (内含 socket fd, 拷贝会引发双重 close) */
    SmcController(const SmcController &)            = delete;
    SmcController &operator=(const SmcController &) = delete;

    /* ---------- 连接管理 ---------- */
    bool Connect(const std::string &ip, int port);
    void Disconnect();
    bool IsConnected() const;
    int  LastError() const;     /* SmcErrCode, 协议层最近一次错误 */

    /* =============================================================
     * 系统生命周期  (CAM 通常不直接调用, 由 CNC 服务自身管理)
     * ============================================================= */
    /* 映射 SMC_InitAndStart: 阻塞到全轴就绪, 0=成功 -1=失败 */
    bool InitAndStart(const std::string &netif_name, int &out_ret_code);
    /* 映射 SMC_Close: 关闭 CNC 内核 (危险操作, CAM 慎用) */
    bool Close();

    /* =============================================================
     * 轴配置
     * ============================================================= */
    bool ConfigAxisTopology(const std::string &axis_name,
                            int is_dual_drive, int master_id, int slave_id,
                            int &out_ret_code);
    bool ConfigSoftLimit(char axis_letter, int enable,
                         double neg_limit_mm, double pos_limit_mm,
                         int &out_ret_code);
    bool ConfigGantrySyncAlarm(char axis_letter, int enable,
                               int tolerance_pulse, int max_error_pulse, int time_ms,
                               int &out_ret_code);
    bool ConfigPulsePerUnit(char axis_letter, double pulse_per_unit);
    bool ConfigAxisDynamics(char axis_letter, int type,
                            double max_v, double max_a, double max_d,
                            double equivalent_radius, int &out_ret_code);
    bool ConfigPlannerParams(double tolerance, double max_centripetal_acc,
                             int &out_ret_code);
    bool ConfigKinematicsOffset(double tool_len,
                                double pivot_x, double pivot_y, double pivot_z);
    bool ConfigKinematics(int type,
                          int r1_idx, int r1_axis,
                          int r2_idx, int r2_axis,
                          double tool_off_x, double tool_off_y, double tool_off_z,
                          double pivot_off_x, double pivot_off_y, double pivot_off_z);

    /* =============================================================
     * 坐标与状态查询
     * ============================================================= */
    bool GetLogicalPos(char axis_letter, double &out_position);   /* 单位: 脉冲 */
    bool IsParserRunning(int &out_running);                       /* 1/0 */
    bool IsMotionDone(int &out_done);                             /* 1/0 */
    bool GetQueueCount(int &out_count);
    bool IsAxisConfigured(char axis_letter, int &out_configured); /* 1/0 */
    bool GetSystemStatus(std::string &out_status);                /* "RUN"/"IDLE"/... */

    /* =============================================================
     * 运动控制
     * ============================================================= */
    bool SetZero(char axis_letter);            /* '*' = 全轴归零 */
    bool MoveRelative(char axis_letter, double distance, double speed);
    bool GoZero(char axis_letter, double speed);

    /* =============================================================
     * G 代码加工
     * ============================================================= */
    bool RunGCodeFile(const std::string &filepath, int &out_ret_code);
    bool PauseProcessing();
    bool ResumeProcessing();
    bool AbortProcessing();

    /* =============================================================
     * P0-b v2: LoadProgram / RunLoadedProgram / GetProgramStructure
     *
     * 典型流程: LoadProgram (preview) → GetProgramStructure → 操作员检查 → RunLoadedProgram
     * ============================================================= */
    /* 加载程序到 preview cache (parser 跑但不执行, 段流推送给 UI 画轨迹)
     * 返回码: 0=ok, -1=parser 忙, -2=filepath 无效 */
    bool LoadProgram(const std::string &filepath, int &out_ret_code);
    /* 执行已加载程序 (LoadProgram 必须先完成, 内部重新解析同一 filepath)
     * 返回码: 0=ok, -1=LoadProgram 未完成, -2=parser 忙 */
    bool RunLoadedProgram(int &out_ret_code);
    /* 查询程序结构元数据 (~390B 响应, 含 filepath/段数/bbox/估算工时) */
    bool GetProgramStructure(SmcGetProgramStructureRes &out);

    /* =============================================================
     * P1-b: ClearAlarm
     * ============================================================= */
    /* 清除系统报警 (异步: 调用立即返回, RT 清完通过 event stream 通知)。
     * 返回码: 0=请求已提交, -1=parser 正在跑 (先 AbortProcessing), -2=轴未就绪 */
    bool ClearAlarm(int &out_ret_code);

    /* =============================================================
     * P0-Laser-Q: 激光切割子系统状态查询
     * ============================================================= */
    /* 查询激光器完整状态 (镜像 RT 单写者字段 + 段级派生 + 加工统计, ~75B 响应)
     * 返回码 (out.ret_code): 0=ok, -1=激光未配置 (do_slave_id<0)
     * 字段详见 SmcGetLaserStateRes 定义 (smc_protocol.h):
     *   状态: enable/shutter/power_w/freq_hz/gas_select/interlock/emergency_kill/
     *         P_base_w/v_actual_mm_s/coupling_mode_rt/is_piercing/current_seg_flags
     *   统计: pierce_count/laser_on_time_ms */
    bool GetLaserState(SmcGetLaserStateRes &out);

    /* =============================================================
     * P0-Laser-ConfigRPC: 激光配置 (0x0050-0x0056)
     * 时序: 必须在 InitAndStart 前调用 (init-time 单写者语义, 与 ConfigAxisTopology 同)
     * 返回码: 0=成功, -1=参数越界 (由 SMC_ConfigLaser* 内部校验)
     * ============================================================= */
    bool ConfigLaserIO(int do_slave_id, int ao_slave_id, int di_slave_id, int &out_ret_code);
    bool ConfigLaserDOBits(uint8_t b_enable, uint8_t b_shutter,
                           uint8_t b_gas_n2, uint8_t b_gas_o2, uint8_t b_gas_air,
                           uint8_t b_alarm_lamp, int &out_ret_code);
    bool ConfigLaserDIBits(uint8_t b_door, uint8_t b_estop, uint8_t b_laser_alm,
                           uint8_t b_water_t, uint8_t b_water_f, uint8_t b_gas_p,
                           int &out_ret_code);
    bool ConfigLaserAOChannels(uint8_t ch_power, uint8_t ch_freq, int &out_ret_code);
    bool ConfigLaserRange(double power_max_w, double freq_max_hz, double power_min_w,
                          int &out_ret_code);
    bool ConfigLaserCoupling(int mode, double v_thresh_mm_s, int &out_ret_code);
    /* P-v 耦合查表 (count: 1..16, 越界 SDK 层拦截返回 ret_code=-1 不发 RPC) */
    bool ConfigLaserCoupleTable(const LaserCouplePoint_t *points, int count, int &out_ret_code);

    /* =============================================================
     * P0-3: Safe Z Lift (紧急抬升避让)
     *
     * 典型流程:
     *   init 阶段: ConfigSafeLiftZ('Z', 50.0, 20.0, auto_on_alarm=true)
     *   报警时:   RT 自动触发 (auto_on_alarm=1) 或操作员按 UI 按钮 TriggerSafeLiftZ
     *   处理后:   GetSafeLiftState 看 state (0/1/2/3) + progress_mm
     *   异常取消: CancelSafeLiftZ (仅 PENDING/DONE 可取消, RUNNING 拒绝)
     *
     * 返回: true=协议层成功 (业务结果在 out_ret_code)
     * ============================================================= */
    bool ConfigSafeLiftZ(char z_letter, double safe_z_mm,
                         double lift_speed_mm_s, bool auto_on_alarm,
                         int &out_ret_code);
    bool TriggerSafeLiftZ(int &out_ret_code);
    bool CancelSafeLiftZ(int &out_ret_code);
    /* out_state: 0=IDLE, 1=PENDING, 2=RUNNING, 3=DONE
     * out_progress_mm: 已抬升距离 (current_z - start_z)
     * out_z_target_mm / out_z_current_mm: HMI 显示用 */
    bool GetSafeLiftState(int &out_state, double &out_progress_mm,
                          double &out_z_target_mm, double &out_z_current_mm,
                          int &out_enabled, int &out_ret_code);

    /* =============================================================
     * P0-1: 工业级回零 (G28 / SMC_HomeAxis / SMC_HomeAll)
     *
     * 典型流程:
     *   init 阶段: ConfigHomingAll("ZXYBC") 配置顺序
     *   手动定位: JogStart/JogStop 把机器拖到参考位 (method 35 前置)
     *   触发回零: TriggerHoming (axis='\0'=HomeAll 或单轴)
     *   查询: GetHomingState
     *   异常: CancelHoming (仅 PENDING/DONE)
     *
     * 返回: true=协议层成功 (业务结果在 out_ret_code)
     * ============================================================= */
    bool ConfigHomingAxis(char axis_letter, int method, double search_speed,
                          double creep_speed, int direction, int timeout_ms,
                          int &out_ret_code);
    bool ConfigHomingAll(const std::string &order_letters, int &out_ret_code);
    bool TriggerHoming(char axis_letter, int &out_ret_code);  /* '\0'=HomeAll */
    bool CancelHoming(int &out_ret_code);
    bool GetHomingState(int &out_state, int &out_axis_idx, double &out_progress_pct,
                        int &out_enabled, int &out_ret_code);

    /* =============================================================
     * P0-1: JOG 模式 (method 35 前置依赖 — 手动定位参考位)
     * ============================================================= */
    bool JogStart(char axis_letter, int direction, double speed_mm_s,
                  int &out_ret_code);
    bool JogStop(char axis_letter, int &out_ret_code);  /* '*' 停所有 */

    /* =============================================================
     * P2-A: 实时倍率系统 (Feed/Rapid/Spindle Override + Mode Flags)
     *
     * 典型流程: HMI 滑条/旋钮 onChange → SetOverride (mask/value 模式, 允许部分修改)
     *           HMI 60Hz refresh → GetOverride (同步旋钮位置, 防止多 client 竞争)
     *
     * 参数语义 (与 SMC_SetOverride 一致):
     *   feed_pct/rapid_pct/spindle_pct: -1=不改, 0..120=设置值 (clamp 后通过 out 回读)
     *   mode_mask: 要修改的 mode_flags 位 (0=不改任何位)
     *   mode_value: mask 标识的位写入 0 或 1
     *   out.*: clamp 后的实际生效值
     * 返回: true=协议层成功 (clamp 不视为错误)
     * ============================================================= */
    bool SetOverride(int feed_pct, int rapid_pct, int spindle_pct,
                     uint16_t mode_mask, uint16_t mode_value,
                     SmcSetOverrideRes &out);
    bool GetOverride(SmcGetOverrideRes &out);

private:
    /* ---------- 内部收发原语 ---------- */
    int  send_all(const void *buf, size_t n);
    int  recv_all(void *buf, size_t n);
    bool sendRequest(uint16_t cmd_type, const void *payload, uint16_t payload_len);
    bool recvResponse(int32_t &err_code, void *payload_buf,
                      uint32_t payload_cap, uint32_t &payload_len);

    /* ---------- 业务调用 helper (按返回值类型分类, DRY) ----------
     * void   返回的 SMC_* -> invokeNoRet
     * int    返回的 SMC_* -> invokeIntRet
     * double 返回的 SMC_* -> invokeDoubleRet
     * 字符串 返回的 SMC_* -> invokeStrRet
     */
    bool invokeNoRet(uint16_t cmd,
                     const void *req = nullptr, uint16_t req_len = 0);
    bool invokeIntRet(uint16_t cmd, int &out,
                      const void *req = nullptr, uint16_t req_len = 0);
    bool invokeDoubleRet(uint16_t cmd, double &out,
                         const void *req = nullptr, uint16_t req_len = 0);
    bool invokeStrRet(uint16_t cmd, char *buf, uint32_t buf_cap);

    /* WSL2 路径自动转换: Windows 绝对路径 -> /mnt/<drive>/... 挂载路径。
     * 仅在 RunGCodeFile 内部调用, 对 CAM 完全透明。*/
    static std::string TranslatePathForWSL(const std::string &win_path);

private:
    smc_socket_t sock_fd_;
    int          last_err_;
    std::mutex   comm_mutex_;  /* TCP 收发原子锁 */
};

#endif /* SMC_CONTROLLER_SDK_H */
