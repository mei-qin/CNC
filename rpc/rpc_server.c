/* =====================================================================
 *  rpc_server.c  ——  CNC Core 端 TCP RPC 服务端 (纯 C / Linux, 端口 9527)
 *
 *  运行位置: Ubuntu 实时主机的"非实时"用户态进程, 与 1ms 硬实时
 *            EtherCAT 线程 (ecat_thread_rt) 同机但不同线程, 互不阻塞。
 *
 *  @Context: Non-RealTime Network Service Thread (TCP RPC)
 *  @Safe:    Blocking I/O (recv/send/accept), malloc/free, printf 均允许。
 *            绝对禁止在本文件中调用任何 ecat_thread_rt 路径或持有实时锁;
 *            与底层 SMC_* API 的线程安全由 smc_api 内部保证。
 *
 *  编译提示:
 *    假设本文件位于 CNC/rpc/, smc_api.h 位于 CNC/inc/, smc_protocol.h 同目录:
 *      gcc -O2 -Wall -I../inc -I. rpc_server.c -o rpc_server -lpthread
 *    生产环境通常由 rpc_server 自己在 main() 起点调用 SMC_InitAndStart,
 *    本示例为聚焦 RPC 逻辑暂不包含该步骤, 改由 CAM 通过 RPC 触发。
 * ===================================================================== */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "smc_protocol.h"
#include "smc_api.h"

#define SMC_RPC_PORT      9527
#define SMC_RPC_BACKLOG   8
#define SMC_RESP_BUF_LEN  256    /* 单包响应 payload 缓冲, 所有 Res 都 < 此值 */

/* ---------------------------------------------------------------------
 * recv_all / send_all  —— TCP 粘包/半包/截断的根本治理
 *
 *   TCP 是字节流而非消息边界, 一次 recv/send 完全可能:
 *     - 只返回 1 字节              (慢启动 / Nagle / 半包)
 *     - 跨多条应用层消息的字节       (粘包, 由上层定长 Header 切分)
 *     - 被信号打断返回 -1, EINTR   (本函数自动重试, 不向上层暴露)
 *     - 对端关闭返回 0             (本函数返回 -1, 让上层关闭 fd)
 *
 *   本函数以"应用层消息长度 N"为唯一终止条件, 循环直至收满 N 字节;
 *   配合定长 SmcReqHeader (4B) / SmcResHeader (8B), 在协议帧层面天然
 *   切分消息流, 彻底消除粘包与半包。
 *
 *   返回: 0 = 成功收/发满 N 字节; -1 = 对端关闭或不可恢复错误。
 * ------------------------------------------------------------------ */
static int recv_all(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    size_t  got = 0;

    while (got < n) {
        ssize_t r = recv(fd, p + got, n - got, 0);
        if (r == 0) return -1;             /* 对端正常关闭 */
        if (r < 0) {
            if (errno == EINTR) continue;  /* 信号打断可重试 */
            return -1;
        }
        got += (size_t)r;
    }
    return 0;
}

static int send_all(int fd, const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    size_t  sent = 0;

    while (sent < n) {
        ssize_t s = send(fd, p + sent, n - sent, 0);
        if (s < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (s == 0) return -1;             /* send 极少返回 0, 视作异常 */
        sent += (size_t)s;
    }
    return 0;
}

/* ---------------------------------------------------------------------
 * 处理一个完整请求-响应回合
 *   流程: 读 SmcReqHeader -> 读 payload -> switch 调 SMC_* -> 写响应
 *
 *   返回:  0 = 成功处理完一轮 (可继续下一轮, 复用连接)
 *          -1 = 对端断开或链路错误 (调用方应关闭 fd)
 * ------------------------------------------------------------------ */
static int handle_client_request(int client_fd)
{
    SmcReqHeader req_hdr;
    SmcResHeader res_hdr;
    uint8_t     *payload = NULL;
    uint8_t      resp_buf[SMC_RESP_BUF_LEN];
    size_t       resp_payload_len = 0;

    memset(&res_hdr, 0, sizeof(res_hdr));

    /* ---- Step 1: 读请求头 ---- */
    if (recv_all(client_fd, &req_hdr, sizeof(req_hdr)) != 0) {
        return -1;
    }

    /* 防御: 拒绝异常大 payload, 避免 malloc 被打满 */
    if (req_hdr.data_len > SMC_MAX_PAYLOAD_BYTES) {
        fprintf(stderr, "[rpc] data_len=%u 超过上限, 断开\n", req_hdr.data_len);
        return -1;
    }

    /* ---- Step 2: 读 payload ---- */
    if (req_hdr.data_len > 0) {
        payload = (uint8_t *)malloc(req_hdr.data_len);
        if (payload == NULL) return -1;
        if (recv_all(client_fd, payload, req_hdr.data_len) != 0) {
            free(payload);
            return -1;
        }
    }

    /* ---- Step 3: 派发到具体业务 ---- */
    switch ((SmcCmdType)req_hdr.cmd_type) {

    /* ===== 系统生命周期 ===== */
    case SMC_CMD_INIT_AND_START: {
        if (req_hdr.data_len < sizeof(SmcInitAndStartReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInitAndStartReq *req = (SmcInitAndStartReq *)payload;
        req->netif_name[SMC_NETIF_NAME_MAX_LEN - 1] = '\0';
        SmcInitAndStartRes *res = (SmcInitAndStartRes *)resp_buf;
        res->ret_code = SMC_InitAndStart(req->netif_name);
        res_hdr.err_code      = SMC_OK;
        resp_payload_len      = sizeof(SmcInitAndStartRes);
        break;
    }
    case SMC_CMD_CLOSE: {
        /* 危险操作: 关闭整个 CNC 内核 (伺服下电、释放网卡)。
         * 通常由 CNC 服务自身在退出时调用, 暴露给 CAM 仅供监督工具使用。*/
        SMC_Close();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }

    /* ===== 轴配置 ===== */
    case SMC_CMD_CONFIG_AXIS_TOPOLOGY: {
        if (req_hdr.data_len < sizeof(SmcConfigAxisTopologyReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigAxisTopologyReq *req = (SmcConfigAxisTopologyReq *)payload;
        req->axis_name[SMC_AXIS_NAME_MAX_LEN - 1] = '\0';
        SmcConfigAxisTopologyRes *res = (SmcConfigAxisTopologyRes *)resp_buf;
        res->ret_code = SMC_ConfigAxisTopology(
            req->axis_name, req->is_dual_drive, req->master_id, req->slave_id);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigAxisTopologyRes);
        break;
    }
    case SMC_CMD_CONFIG_SOFT_LIMIT: {
        if (req_hdr.data_len < sizeof(SmcConfigSoftLimitReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigSoftLimitReq *req = (SmcConfigSoftLimitReq *)payload;
        SmcConfigSoftLimitRes *res = (SmcConfigSoftLimitRes *)resp_buf;
        res->ret_code = SMC_ConfigSoftLimit(
            req->axis_letter, req->enable, req->neg_limit_mm, req->pos_limit_mm);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigSoftLimitRes);
        break;
    }
    case SMC_CMD_CONFIG_GANTRY_SYNC_ALARM: {
        if (req_hdr.data_len < sizeof(SmcConfigGantrySyncAlarmReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigGantrySyncAlarmReq *req = (SmcConfigGantrySyncAlarmReq *)payload;
        SmcConfigGantrySyncAlarmRes *res = (SmcConfigGantrySyncAlarmRes *)resp_buf;
        res->ret_code = SMC_ConfigGantrySyncAlarm(
            req->axis_letter, req->enable,
            req->tolerance_pulse, req->max_error_pulse, req->time_ms);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigGantrySyncAlarmRes);
        break;
    }
    case SMC_CMD_CONFIG_PULSE_PER_UNIT: {
        if (req_hdr.data_len < sizeof(SmcConfigPulsePerUnitReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigPulsePerUnitReq *req = (SmcConfigPulsePerUnitReq *)payload;
        SMC_ConfigPulsePerUnit(req->axis_letter, req->pulse_per_unit);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_CONFIG_AXIS_DYNAMICS: {
        if (req_hdr.data_len < sizeof(SmcConfigAxisDynamicsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigAxisDynamicsReq *req = (SmcConfigAxisDynamicsReq *)payload;
        SmcConfigAxisDynamicsRes *res = (SmcConfigAxisDynamicsRes *)resp_buf;
        res->ret_code = SMC_ConfigAxisDynamics(
            req->axis_letter, req->type,
            req->max_v, req->max_a, req->max_d, req->equivalent_radius);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigAxisDynamicsRes);
        break;
    }
    case SMC_CMD_CONFIG_PLANNER_PARAMS: {
        if (req_hdr.data_len < sizeof(SmcConfigPlannerParamsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigPlannerParamsReq *req = (SmcConfigPlannerParamsReq *)payload;
        SmcConfigPlannerParamsRes *res = (SmcConfigPlannerParamsRes *)resp_buf;
        res->ret_code = SMC_ConfigPlannerParams(req->tolerance, req->max_centripetal_acc);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigPlannerParamsRes);
        break;
    }
    case SMC_CMD_CONFIG_KINEMATICS_OFFSET: {
        if (req_hdr.data_len < sizeof(SmcConfigKinematicsOffsetReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigKinematicsOffsetReq *req = (SmcConfigKinematicsOffsetReq *)payload;
        SMC_ConfigKinematicsOffset(
            req->tool_len, req->pivot_x, req->pivot_y, req->pivot_z);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_CONFIG_KINEMATICS: {
        if (req_hdr.data_len < sizeof(SmcConfigKinematicsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigKinematicsReq *req = (SmcConfigKinematicsReq *)payload;
        /* C 数组参数按地址传入; wire 端 tool_off/pivot_off 已是 double[3] */
        SMC_ConfigKinematics(req->type,
                             req->r1_idx, req->r1_axis,
                             req->r2_idx, req->r2_axis,
                             req->tool_off, req->pivot_off);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }

    /* ===== 坐标与状态查询 ===== */
    case SMC_CMD_GET_LOGICAL_POS: {
        if (req_hdr.data_len < sizeof(SmcGetLogicalPosReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcGetLogicalPosReq *req = (SmcGetLogicalPosReq *)payload;
        SmcGetLogicalPosRes *res = (SmcGetLogicalPosRes *)resp_buf;
        res->position = SMC_GetLogicalPos(req->axis_letter);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetLogicalPosRes);
        break;
    }
    case SMC_CMD_IS_PARSER_RUNNING: {
        SmcIsParserRunningRes *res = (SmcIsParserRunningRes *)resp_buf;
        res->running = SMC_IsParserRunning();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcIsParserRunningRes);
        break;
    }
    case SMC_CMD_IS_MOTION_DONE: {
        SmcIsMotionDoneRes *res = (SmcIsMotionDoneRes *)resp_buf;
        res->done = SMC_IsMotionDone();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcIsMotionDoneRes);
        break;
    }
    case SMC_CMD_GET_QUEUE_COUNT: {
        SmcGetQueueCountRes *res = (SmcGetQueueCountRes *)resp_buf;
        res->count = SMC_GetQueueCount();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetQueueCountRes);
        break;
    }
    case SMC_CMD_IS_AXIS_CONFIGURED: {
        if (req_hdr.data_len < sizeof(SmcIsAxisConfiguredReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcIsAxisConfiguredReq *req = (SmcIsAxisConfiguredReq *)payload;
        SmcIsAxisConfiguredRes *res = (SmcIsAxisConfiguredRes *)resp_buf;
        res->configured = SMC_IsAxisConfigured(req->axis_letter);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcIsAxisConfiguredRes);
        break;
    }
    case SMC_CMD_GET_SYSTEM_STATUS: {
        SmcGetSystemStatusRes *res = (SmcGetSystemStatusRes *)resp_buf;
        /* 把 res->status_str 直接当 out_str 缓冲喂给 SMC_GetSystemStatusStr */
        memset(res->status_str, 0, sizeof(res->status_str));
        SMC_GetSystemStatusStr(res->status_str, (int)sizeof(res->status_str));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetSystemStatusRes);
        break;
    }

    /* ===== 运动控制 ===== */
    case SMC_CMD_SET_ZERO: {
        if (req_hdr.data_len < sizeof(SmcSetZeroReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcSetZeroReq *req = (SmcSetZeroReq *)payload;
        SMC_SetZero(req->axis_letter);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_MOVE_RELATIVE: {
        if (req_hdr.data_len < sizeof(SmcMoveRelativeReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcMoveRelativeReq *req = (SmcMoveRelativeReq *)payload;
        SMC_MoveRelative(req->axis_letter, req->distance, req->speed);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_GO_ZERO: {
        if (req_hdr.data_len < sizeof(SmcGoZeroReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcGoZeroReq *req = (SmcGoZeroReq *)payload;
        SMC_GoZero(req->axis_letter, req->speed);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }

    /* ===== G 代码加工 ===== */
    case SMC_CMD_RUN_GCODE_FILE: {
        if (req_hdr.data_len < sizeof(SmcRunGCodeFileReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcRunGCodeFileReq *req = (SmcRunGCodeFileReq *)payload;
        req->filepath[SMC_FILEPATH_MAX_LEN - 1] = '\0';
        SmcRunGCodeFileRes *res = (SmcRunGCodeFileRes *)resp_buf;
        res->ret_code = SMC_RunGCodeFile(req->filepath);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcRunGCodeFileRes);
        break;
    }
    case SMC_CMD_PAUSE_PROCESSING: {
        SMC_PauseProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_RESUME_PROCESSING: {
        SMC_ResumeProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_ABORT_PROCESSING: {
        SMC_AbortProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }

    default:
        /* 未知命令: 仍回响应头让客户端可继续下一轮 */
        res_hdr.err_code = SMC_ERR_UNKNOWN_CMD;
        resp_payload_len = 0;
        break;
    }

    free(payload);

    /* ---- Step 4: 写响应 ---- */
    res_hdr.data_len = (uint32_t)resp_payload_len;
    if (send_all(client_fd, &res_hdr, sizeof(res_hdr)) != 0) return -1;
    if (resp_payload_len > 0) {
        if (send_all(client_fd, resp_buf, resp_payload_len) != 0) return -1;
    }
    return 0;
}

/* ---------------------------------------------------------------------
 * main: listen / accept / 同连接多请求循环
 *   说明: 这是最简同步模型, 单连接串行处理。
 *        生产环境多 CAM 并发时, 改为 epoll 或每连接一线程; 但需保证
 *        多线程对 SMC_* 的并发由 smc_api 内部串行化, 或在 RPC 层加锁。
 * ------------------------------------------------------------------ */
int main(void)
{
    /* 客户端异常断开时忽略 SIGPIPE, 让 recv/send 返回错误而非崩进程 */
    signal(SIGPIPE, SIG_IGN);

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { perror("socket"); return 1; }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    /* 绑定 INADDR_ANY (0.0.0.0) —— WSL2 跨 OS 联调关键点:
     *   WSL2 的 localhostForwarding (默认开启) 会把 Windows 宿主机对
     *   127.0.0.1:9527 的访问自动转发到 WSL2 内的同端口监听服务。
     *   若只绑 127.0.0.1, 可能因 WSL2 网络命名空间隔离导致宿主机无法连通。
     *   生产部署若启用了 mirrored 网络模式, 同样建议保持 0.0.0.0。*/
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(SMC_RPC_PORT);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(listen_fd); return 1;
    }
    if (listen(listen_fd, SMC_RPC_BACKLOG) < 0) {
        perror("listen"); close(listen_fd); return 1;
    }
    printf("[rpc] listening on 0.0.0.0:%d ...\n", SMC_RPC_PORT);

    for (;;) {
        int client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            perror("accept");
            continue;
        }
        printf("[rpc] client connected, fd=%d\n", client_fd);

        /* 同一连接上循环处理多轮请求, 直到对端断开 */
        while (handle_client_request(client_fd) == 0) {
            /* nothing */;
        }
        close(client_fd);
        printf("[rpc] client disconnected, fd=%d\n", client_fd);
    }

    close(listen_fd);
    return 0;
}
