/* =====================================================================
 *  SmcControllerSdk.cpp  ——  CAM / HMI 侧 RPC 客户端 SDK 实现
 *
 *  @Context: Non-RealTime Client Application (CAM/HMI)
 *  @Safe:    阻塞 connect/recv/send, 抛异常/打印均允许。这是普通
 *            用户态进程, 与 CNC Core 1ms 硬实时线程无任何共享内存。
 *
 *  Windows 编译说明:
 *    头文件已通过 #ifdef _WIN32 自动切换到 winsock2 + ws2tcpip,
 *    并依赖 ws2_32.lib。
 *      - MSVC : 链接选项加 ws2_32.lib (或本文件末尾的 #pragma comment)
 *      - MinGW: 加 -lws2_32
 *    Winsock 生命周期: 构造函数 WSAStartup / 析构函数 WSACleanup,
 *    Windows 内部引用计数自动配对, 多 SmcController 实例安全共存。
 *
 *  WSL2 联调路径转换:
 *    部署拓扑 = Windows 宿主机 (CAM) <-> WSL2/Linux (CNC Core)。
 *    CAM 传入的 Windows 路径 (C:\\..., D:/..., \\wsl.localhost\...) 由
 *    TranslatePathForWSL 自动转成 WSL2 视角的 /mnt/<drive>/... 再入 wire。
 * ===================================================================== */

#include "SmcControllerSdk.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdio>

#ifdef _WIN32
    #pragma comment(lib, "ws2_32.lib")
#endif

/* =====================================================================
 * 构造 / 析构
 * ================================================================== */
SmcController::SmcController()
    : sock_fd_(SMC_INVALID_SOCKET), last_err_(SMC_OK)
{
#ifdef _WIN32
    /* Windows 必须在使用任何 socket API 前 WSAStartup。
     * 放在构造里保证实例化即可 Connect, 无需调用方关心初始化顺序。
     * 多实例: Windows 引用计数, 每次 WSAStartup 都需配对 WSACleanup。*/
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
        last_err_ = SMC_ERR_SOCKET;
        std::fprintf(stderr, "[sdk] WSAStartup 失败, 实例不可用\n");
    }
#endif
}

SmcController::~SmcController()
{
    Disconnect();
#ifdef _WIN32
    /* 与构造里的 WSAStartup 配对释放, 即使 socket 仍在用也安全 (引用计数)。*/
    WSACleanup();
#endif
}

bool SmcController::IsConnected() const
{
    return sock_fd_ != SMC_INVALID_SOCKET;
}

int  SmcController::LastError() const { return last_err_; }

/* =====================================================================
 * Connect / Disconnect
 * ================================================================== */
bool SmcController::Connect(const std::string &ip, int port)
{
    /* Winsock 已在构造函数 WSAStartup, 这里无需重复初始化 */
    if (last_err_ == SMC_ERR_SOCKET
#ifdef _WIN32
        && sock_fd_ == SMC_INVALID_SOCKET
#endif
       ) {
        /* 构造时 WSAStartup 失败, 实例整体不可用 */
        return false;
    }

    Disconnect();   /* 清理上次残留 */

    smc_socket_t fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == SMC_INVALID_SOCKET) {
        last_err_ = SMC_ERR_SOCKET;
        return false;
    }

    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(port));

    if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
        last_err_ = SMC_ERR_PARAM;
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
        return false;
    }

    if (connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        last_err_ = SMC_ERR_SOCKET;
#ifdef _WIN32
        closesocket(fd);
#else
        close(fd);
#endif
        return false;
    }

    sock_fd_  = fd;
    last_err_ = SMC_OK;
    return true;
}

void SmcController::Disconnect()
{
    if (sock_fd_ != SMC_INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(sock_fd_);
#else
        close(sock_fd_);
#endif
        sock_fd_ = SMC_INVALID_SOCKET;
    }
}

/* =====================================================================
 * send_all / recv_all  —— 防 TCP 半包/粘包
 * ================================================================== */
int SmcController::send_all(const void *buf, size_t n)
{
    const uint8_t *p = static_cast<const uint8_t *>(buf);
    size_t sent = 0;

    while (sent < n) {
#ifdef _WIN32
        int s = ::send(sock_fd_, reinterpret_cast<const char *>(p + sent),
                       static_cast<int>(n - sent), 0);
        if (s == SOCKET_ERROR) return -1;
#else
        ssize_t s = ::send(sock_fd_, p + sent, n - sent, 0);
        if (s <= 0) {
            if (s < 0 && errno == EINTR) continue;
            return -1;
        }
#endif
        sent += static_cast<size_t>(s);
    }
    return 0;
}

int SmcController::recv_all(void *buf, size_t n)
{
    uint8_t *p = static_cast<uint8_t *>(buf);
    size_t  got = 0;

    while (got < n) {
#ifdef _WIN32
        int r = ::recv(sock_fd_, reinterpret_cast<char *>(p + got),
                       static_cast<int>(n - got), 0);
        if (r == 0 || r == SOCKET_ERROR) return -1;
        got += static_cast<size_t>(r);
#else
        ssize_t r = ::recv(sock_fd_, p + got, n - got, 0);
        if (r == 0) return -1;
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        got += static_cast<size_t>(r);
#endif
    }
    return 0;
}

/* =====================================================================
 * sendRequest / recvResponse
 * ================================================================== */
bool SmcController::sendRequest(uint16_t cmd_type, const void *payload,
                                uint16_t payload_len)
{
    SmcReqHeader hdr;
    hdr.cmd_type = cmd_type;
    hdr.data_len = payload_len;

    if (send_all(&hdr, sizeof(hdr)) != 0) {
        last_err_ = SMC_ERR_SOCKET;
        return false;
    }
    if (payload_len > 0 && payload != nullptr) {
        if (send_all(payload, payload_len) != 0) {
            last_err_ = SMC_ERR_SOCKET;
            return false;
        }
    }
    return true;
}

bool SmcController::recvResponse(int32_t &err_code, void *payload_buf,
                                 uint32_t payload_cap, uint32_t &payload_len)
{
    SmcResHeader hdr;
    if (recv_all(&hdr, sizeof(hdr)) != 0) {
        last_err_ = SMC_ERR_SOCKET;
        return false;
    }
    err_code    = hdr.err_code;
    payload_len = hdr.data_len;

    if (payload_len <= payload_cap) {
        if (payload_len > 0) {
            if (recv_all(payload_buf, payload_len) != 0) {
                last_err_ = SMC_ERR_SOCKET;
                return false;
            }
        }
        return true;
    }

    /* 异常路径: 实际 payload 超过预期, 逐块丢弃以维持流对齐 */
    uint8_t  discard[256];
    uint32_t remaining = payload_len;
    while (remaining > 0) {
        uint32_t chunk = remaining < sizeof(discard)
                       ? remaining
                       : static_cast<uint32_t>(sizeof(discard));
        if (recv_all(discard, chunk) != 0) {
            last_err_ = SMC_ERR_SOCKET;
            return false;
        }
        remaining -= chunk;
    }
    last_err_ = SMC_ERR_INTERNAL;
    return false;
}

/* =====================================================================
 * TranslatePathForWSL  ——  Windows 路径 -> WSL2 挂载路径 (仅 RunGCodeFile 用)
 *
 *   部署拓扑: CAM 在 Windows 宿主机, CNC Core 在 WSL2 (Linux)。
 *   CAM 团队传 Windows 原生路径, 但 CNC Core 只能识别 Linux 视角下的
 *   /mnt/<drive>/... 路径。本函数在 SDK 内部透明转换, CAM 无需感知。
 *
 *   覆盖三种输入:
 *     1) 已是 Linux 绝对路径   /home/cnc/x.nc
 *        -> 原样返回 (Linux 端 CAM 也走这条分支, 行为对称)
 *     2) 盘符绝对路径          C:\CNC\x.nc  /  d:/y.nc  (大小写不限)
 *        -> /mnt/<lower_drive>/CNC/x.nc
 *     3) WSL UNC 路径          \\wsl.localhost\Ubuntu\home\z.nc  /  \\wsl$\..
 *        -> /home/z.nc   (剥掉 //<authority>/<distro>/ 前缀)
 *     其他 (相对路径 / 未知格式) -> 原样返回, 交给服务端处理或拒绝。
 * ================================================================== */
std::string SmcController::TranslatePathForWSL(const std::string &win_path)
{
    if (win_path.empty()) return win_path;

    /* 1) 已是 Linux 绝对路径, 直接返回 */
    if (win_path[0] == '/') return win_path;

    /* 统一反斜杠 -> 正斜杠, 简化后续切分 */
    std::string p = win_path;
    std::replace(p.begin(), p.end(), '\\', '/');

    /* 2) UNC 路径: //wsl$/<distro>/... 或 //wsl.localhost/<distro>/...
     *    跳过 '//' + authority + '/' + distro + '/', 剩余即 Linux 路径。*/
    if (p.size() >= 2 && p[0] == '/' && p[1] == '/') {
        size_t pos = 2;
        while (pos < p.size() && p[pos] != '/') pos++;   /* 跳过 authority */
        if (pos < p.size()) pos++;                       /* 跳过 / */
        while (pos < p.size() && p[pos] != '/') pos++;   /* 跳过 distro */
        if (pos < p.size()) pos++;                       /* 跳过 / */
        if (pos > 0 && pos <= p.size()) {
            return p.substr(pos - 1);                    /* 含前导 / */
        }
        return win_path;   /* 解析失败, 原样返回让服务端判定 */
    }

    /* 3) 盘符绝对路径: <Letter>:/... (大小写不限) */
    if (p.size() >= 3
        && std::isalpha(static_cast<unsigned char>(p[0]))
        && p[1] == ':'
        && p[2] == '/') {
        char drive = static_cast<char>(
            std::tolower(static_cast<unsigned char>(p[0])));
        return "/mnt/" + std::string(1, drive) + "/" + p.substr(3);
    }

    /* 4) 其他 (相对路径 / 未知格式) -> 原样返回 */
    return win_path;
}

/* =====================================================================
 * 业务调用 helper —— 把 23 个方法压缩到 4 类共性
 * ================================================================== */
bool SmcController::invokeNoRet(uint16_t cmd, const void *req, uint16_t req_len)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!IsConnected()) { last_err_ = SMC_ERR_SOCKET; return false; }
    if (!sendRequest(cmd, req, req_len)) return false;

    int32_t  err = 0;
    uint32_t len = 0;
    uint8_t  dummy;
    if (!recvResponse(err, &dummy, 0, len)) return false;
    if (err != SMC_OK) { last_err_ = err; return false; }
    return true;
}

bool SmcController::invokeIntRet(uint16_t cmd, int &out,
                                 const void *req, uint16_t req_len)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!IsConnected()) { last_err_ = SMC_ERR_SOCKET; return false; }
    if (!sendRequest(cmd, req, req_len)) return false;

    int32_t  wire = 0;
    int32_t  err  = 0;
    uint32_t len  = 0;
    if (!recvResponse(err, &wire, sizeof(wire), len)) return false;
    if (err != SMC_OK) { last_err_ = err; return false; }
    if (len < sizeof(wire)) { last_err_ = SMC_ERR_INTERNAL; return false; }
    out = static_cast<int>(wire);
    return true;
}

bool SmcController::invokeDoubleRet(uint16_t cmd, double &out,
                                    const void *req, uint16_t req_len)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!IsConnected()) { last_err_ = SMC_ERR_SOCKET; return false; }
    if (!sendRequest(cmd, req, req_len)) return false;

    double   wire = 0.0;
    int32_t  err  = 0;
    uint32_t len  = 0;
    if (!recvResponse(err, &wire, sizeof(wire), len)) return false;
    if (err != SMC_OK) { last_err_ = err; return false; }
    if (len < sizeof(wire)) { last_err_ = SMC_ERR_INTERNAL; return false; }
    out = wire;
    return true;
}

bool SmcController::invokeStrRet(uint16_t cmd, char *buf, uint32_t buf_cap)
{
    std::lock_guard<std::mutex> lock(comm_mutex_);
    if (!IsConnected()) { last_err_ = SMC_ERR_SOCKET; return false; }
    if (buf == nullptr || buf_cap == 0) { last_err_ = SMC_ERR_PARAM; return false; }
    if (!sendRequest(cmd, nullptr, 0)) return false;

    int32_t  err = 0;
    uint32_t len = 0;
    if (!recvResponse(err, buf, buf_cap, len)) return false;
    if (err != SMC_OK) { last_err_ = err; return false; }

    /* 强制字符串终止: 优先在收到的数据末尾, 否则缓冲最后一字节 */
    if (len < buf_cap) buf[len] = '\0';
    else               buf[buf_cap - 1] = '\0';
    return true;
}

/* =====================================================================
 * 业务 API 实现 —— 系统生命周期
 * ================================================================== */
bool SmcController::InitAndStart(const std::string &netif_name, int &out_ret_code)
{
    if (netif_name.size() >= SMC_NETIF_NAME_MAX_LEN) {
        last_err_ = SMC_ERR_PARAM;
        return false;
    }
    SmcInitAndStartReq req;
    std::memset(&req, 0, sizeof(req));
    std::strncpy(req.netif_name, netif_name.c_str(), SMC_NETIF_NAME_MAX_LEN - 1);
    return invokeIntRet(SMC_CMD_INIT_AND_START, out_ret_code, &req, sizeof(req));
}

bool SmcController::Close()
{
    return invokeNoRet(SMC_CMD_CLOSE);
}

/* =====================================================================
 * 业务 API 实现 —— 轴配置
 * ================================================================== */
bool SmcController::ConfigAxisTopology(const std::string &axis_name,
                                       int is_dual_drive, int master_id, int slave_id,
                                       int &out_ret_code)
{
    if (axis_name.size() >= SMC_AXIS_NAME_MAX_LEN) {
        last_err_ = SMC_ERR_PARAM;
        return false;
    }
    SmcConfigAxisTopologyReq req;
    std::memset(&req, 0, sizeof(req));
    std::strncpy(req.axis_name, axis_name.c_str(), SMC_AXIS_NAME_MAX_LEN - 1);
    req.is_dual_drive = is_dual_drive;
    req.master_id     = master_id;
    req.slave_id      = slave_id;
    return invokeIntRet(SMC_CMD_CONFIG_AXIS_TOPOLOGY, out_ret_code, &req, sizeof(req));
}

bool SmcController::ConfigSoftLimit(char axis_letter, int enable,
                                    double neg_limit_mm, double pos_limit_mm,
                                    int &out_ret_code)
{
    SmcConfigSoftLimitReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter  = axis_letter;
    req.enable       = enable;
    req.neg_limit_mm = neg_limit_mm;
    req.pos_limit_mm = pos_limit_mm;
    return invokeIntRet(SMC_CMD_CONFIG_SOFT_LIMIT, out_ret_code, &req, sizeof(req));
}

bool SmcController::ConfigGantrySyncAlarm(char axis_letter, int enable,
                                          int tolerance_pulse, int max_error_pulse, int time_ms,
                                          int &out_ret_code)
{
    SmcConfigGantrySyncAlarmReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter      = axis_letter;
    req.enable           = enable;
    req.tolerance_pulse  = tolerance_pulse;
    req.max_error_pulse  = max_error_pulse;
    req.time_ms          = time_ms;
    return invokeIntRet(SMC_CMD_CONFIG_GANTRY_SYNC_ALARM, out_ret_code, &req, sizeof(req));
}

bool SmcController::ConfigPulsePerUnit(char axis_letter, double pulse_per_unit)
{
    SmcConfigPulsePerUnitReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter   = axis_letter;
    req.pulse_per_unit = pulse_per_unit;
    return invokeNoRet(SMC_CMD_CONFIG_PULSE_PER_UNIT, &req, sizeof(req));
}

bool SmcController::ConfigAxisDynamics(char axis_letter, int type,
                                       double max_v, double max_a, double max_d,
                                       double equivalent_radius, int &out_ret_code)
{
    SmcConfigAxisDynamicsReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter      = axis_letter;
    req.type             = type;
    req.max_v            = max_v;
    req.max_a            = max_a;
    req.max_d            = max_d;
    req.equivalent_radius = equivalent_radius;
    return invokeIntRet(SMC_CMD_CONFIG_AXIS_DYNAMICS, out_ret_code, &req, sizeof(req));
}

bool SmcController::ConfigPlannerParams(double tolerance, double max_centripetal_acc,
                                        int &out_ret_code)
{
    SmcConfigPlannerParamsReq req;
    std::memset(&req, 0, sizeof(req));
    req.tolerance         = tolerance;
    req.max_centripetal_acc = max_centripetal_acc;
    return invokeIntRet(SMC_CMD_CONFIG_PLANNER_PARAMS, out_ret_code, &req, sizeof(req));
}

bool SmcController::ConfigKinematicsOffset(double tool_len,
                                           double pivot_x, double pivot_y, double pivot_z)
{
    SmcConfigKinematicsOffsetReq req;
    std::memset(&req, 0, sizeof(req));
    req.tool_len = tool_len;
    req.pivot_x  = pivot_x;
    req.pivot_y  = pivot_y;
    req.pivot_z  = pivot_z;
    return invokeNoRet(SMC_CMD_CONFIG_KINEMATICS_OFFSET, &req, sizeof(req));
}

bool SmcController::ConfigKinematics(int type,
                                     int r1_idx, int r1_axis,
                                     int r2_idx, int r2_axis,
                                     double tool_off_x, double tool_off_y, double tool_off_z,
                                     double pivot_off_x, double pivot_off_y, double pivot_off_z)
{
    SmcConfigKinematicsReq req;
    std::memset(&req, 0, sizeof(req));
    req.type         = type;
    req.r1_idx       = r1_idx;
    req.r1_axis      = r1_axis;
    req.r2_idx       = r2_idx;
    req.r2_axis      = r2_axis;
    req.tool_off[0]  = tool_off_x;
    req.tool_off[1]  = tool_off_y;
    req.tool_off[2]  = tool_off_z;
    req.pivot_off[0] = pivot_off_x;
    req.pivot_off[1] = pivot_off_y;
    req.pivot_off[2] = pivot_off_z;
    return invokeNoRet(SMC_CMD_CONFIG_KINEMATICS, &req, sizeof(req));
}

/* =====================================================================
 * 业务 API 实现 —— 坐标与状态查询
 * ================================================================== */
bool SmcController::GetLogicalPos(char axis_letter, double &out_position)
{
    SmcGetLogicalPosReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter = axis_letter;
    return invokeDoubleRet(SMC_CMD_GET_LOGICAL_POS, out_position, &req, sizeof(req));
}

bool SmcController::IsParserRunning(int &out_running)
{
    return invokeIntRet(SMC_CMD_IS_PARSER_RUNNING, out_running);
}

bool SmcController::IsMotionDone(int &out_done)
{
    return invokeIntRet(SMC_CMD_IS_MOTION_DONE, out_done);
}

bool SmcController::GetQueueCount(int &out_count)
{
    return invokeIntRet(SMC_CMD_GET_QUEUE_COUNT, out_count);
}

bool SmcController::IsAxisConfigured(char axis_letter, int &out_configured)
{
    SmcIsAxisConfiguredReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter = axis_letter;
    return invokeIntRet(SMC_CMD_IS_AXIS_CONFIGURED, out_configured, &req, sizeof(req));
}

bool SmcController::GetSystemStatus(std::string &out_status)
{
    char buf[SMC_STATUS_STR_MAX_LEN];
    if (!invokeStrRet(SMC_CMD_GET_SYSTEM_STATUS, buf, sizeof(buf))) return false;
    out_status = buf;
    return true;
}

/* =====================================================================
 * 业务 API 实现 —— 运动控制
 * ================================================================== */
bool SmcController::SetZero(char axis_letter)
{
    SmcSetZeroReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter = axis_letter;
    return invokeNoRet(SMC_CMD_SET_ZERO, &req, sizeof(req));
}

bool SmcController::MoveRelative(char axis_letter, double distance, double speed)
{
    SmcMoveRelativeReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter = axis_letter;
    req.distance    = distance;
    req.speed       = speed;
    return invokeNoRet(SMC_CMD_MOVE_RELATIVE, &req, sizeof(req));
}

bool SmcController::GoZero(char axis_letter, double speed)
{
    SmcGoZeroReq req;
    std::memset(&req, 0, sizeof(req));
    req.axis_letter = axis_letter;
    req.speed       = speed;
    return invokeNoRet(SMC_CMD_GO_ZERO, &req, sizeof(req));
}

/* =====================================================================
 * 业务 API 实现 —— G 代码加工
 * ================================================================== */
bool SmcController::RunGCodeFile(const std::string &filepath, int &out_ret_code)
{
    /* WSL2 跨 OS 联调: CAM 端传 Windows 路径, 透明转换为 Linux 视角再入 wire。
     * - Windows CAM 传 "C:\\CNC\\x.nc"   -> /mnt/c/CNC/x.nc
     * - Linux   CAM 传 "/home/cnc/x.nc" -> 原样 (Case 1 直通)
     * 转换后路径仍受定长 SMC_FILEPATH_MAX_LEN (256) 限制。*/
    std::string linux_path = TranslatePathForWSL(filepath);
    if (linux_path.size() >= SMC_FILEPATH_MAX_LEN) {
        last_err_ = SMC_ERR_PARAM;
        return false;
    }
    SmcRunGCodeFileReq req;
    std::memset(&req, 0, sizeof(req));
    std::strncpy(req.filepath, linux_path.c_str(), SMC_FILEPATH_MAX_LEN - 1);
    return invokeIntRet(SMC_CMD_RUN_GCODE_FILE, out_ret_code, &req, sizeof(req));
}

bool SmcController::PauseProcessing()  { return invokeNoRet(SMC_CMD_PAUSE_PROCESSING); }
bool SmcController::ResumeProcessing() { return invokeNoRet(SMC_CMD_RESUME_PROCESSING); }
bool SmcController::AbortProcessing()  { return invokeNoRet(SMC_CMD_ABORT_PROCESSING); }
