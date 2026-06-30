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
};

#endif /* SMC_CONTROLLER_SDK_H */
