/* =====================================================================
 *  rpc_server.c  ——  CNC Core 生产入口 (纯 C / Linux, 端口 9527)
 *
 *  定位: CNC 内核的唯一生产运行入口。取代 main.c 的 scanf 测试菜单,
 *        启动时自主完成硬件初始化 + 轴配置 + 运动学配置,
 *        随后进入 TCP RPC 监听循环, 接受 MoveControl 的远程控制。
 *
 *  运行: sudo ./rpc_server <eth_iface|sim>
 *        - eth_iface: 真实 EtherCAT 网卡 (如 enp7s0)
 *        - sim:       纯软件仿真 (无硬件)
 *
 *  线程安全: rpc_server 自身是非实时线程, 通过 smc_api.h 与
 *            ecat_thread_rt (1ms SCHED_FIFO) 及 parser 线程交互,
 *            smc_api 内部保证线程安全。
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

/* kernel_init 所需的头文件 */
#include "global_def.h"
#include "kinematics.h"
#include "macro_eval.h"
#include "snapshot_hub.h"      /* P0-a: SnapshotHub_Init */
#include "rpc_push_server.h"   /* P0-a: rpc_push_server_start */
#include "preview_streamer.h"  /* P0-b v1: PreviewStreamer_Init */
#include "rpc_preview_server.h" /* P0-b v1: rpc_preview_server_start */
#include "event_logger.h"      /* P1-b: EventLogger_Init */
#include "rpc_event_server.h"  /* P1-b: rpc_event_server_start */
#include "opcua_server.h"      /* 契约 P4: opcua_server_start (OPC UA 4840 对外通道) */
#include "param_store.h"       /* P1+P2: 机床参数档案 (kernel_init_hw 数据源) */

#define SMC_RPC_PORT      9527
#define SMC_RPC_BACKLOG   8
/* P0-b v2: 扩到 512 容纳 SmcGetProgramStructureRes (~390B, filepath[256] + bbox×2) */
#define SMC_RESP_BUF_LEN  512    /* 单包响应 payload 缓冲, 所有 Res 都 < 此值 */

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
        printf("[rpc] 轴拓扑: %s dual=%d master=%d slave=%d ret=%d\n",
               req->axis_name, req->is_dual_drive, req->master_id, req->slave_id, res->ret_code);
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
        printf("[rpc] 软限位: %c enable=%d neg=%.1f pos=%.1f ret=%d\n",
               req->axis_letter, req->enable, req->neg_limit_mm, req->pos_limit_mm, res->ret_code);
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
        printf("[rpc] 龙门报警: %c enable=%d tol=%d max_err=%d ms=%d ret=%d\n",
               req->axis_letter, req->enable,
               req->tolerance_pulse, req->max_error_pulse, req->time_ms, res->ret_code);
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
        printf("[rpc] 脉冲当量: %c ppu=%.1f\n", req->axis_letter, req->pulse_per_unit);
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
        printf("[rpc] 轴动力学: %c type=%d maxV=%.1f maxA=%.1f radius=%.1f ret=%d\n",
               req->axis_letter, req->type,
               req->max_v, req->max_a, req->equivalent_radius, res->ret_code);
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
        printf("[rpc] 规划器: tol=%.3f centrip=%.1f ret=%d\n",
               req->tolerance, req->max_centripetal_acc, res->ret_code);
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
        printf("[rpc] 运动学: type=%d r1=(%d,%d) r2=(%d,%d)\n",
               req->type, req->r1_idx, req->r1_axis, req->r2_idx, req->r2_axis);
        break;
    }
    case SMC_CMD_INJECT_AXIS_FAULT: {
        if (req_hdr.data_len < sizeof(SmcInjectAxisFaultReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInjectAxisFaultReq *req = (SmcInjectAxisFaultReq *)payload;
        SmcInjectAxisFaultRes *res = (SmcInjectAxisFaultRes *)resp_buf;
        res->ret_code = SMC_InjectAxisFault(req->axis_letter, req->slave_subidx);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcInjectAxisFaultRes);
        printf("[rpc] 注入故障: %c motor=%d ret=%d\n",
               req->axis_letter, req->slave_subidx, res->ret_code);
        break;
    }
    case SMC_CMD_CONFIG_SIM_DYNAMICS: {
        if (req_hdr.data_len < sizeof(SmcConfigSimDynamicsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigSimDynamicsReq *req = (SmcConfigSimDynamicsReq *)payload;
        SmcConfigSimDynamicsRes *res = (SmcConfigSimDynamicsRes *)resp_buf;
        res->ret_code = SMC_ConfigSimDynamics(req->axis_letter, req->alpha);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigSimDynamicsRes);
        printf("[rpc] sim 动力学: %c alpha=%.3f ret=%d\n",
               req->axis_letter, req->alpha, res->ret_code);
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
    case SMC_CMD_GET_SPINDLE_STATE: {
        SmcGetSpindleStateRes *res = (SmcGetSpindleStateRes *)resp_buf;
        res->mode = 0; res->rpm = 0.0; res->ret_code = 0;
        res->ret_code = SMC_GetSpindleState(&res->mode, &res->rpm);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetSpindleStateRes);
        break;
    }
    case SMC_CMD_GET_COOLANT_STATE: {
        SmcGetCoolantStateRes *res = (SmcGetCoolantStateRes *)resp_buf;
        res->state = 0; res->ret_code = 0;
        res->ret_code = SMC_GetCoolantState(&res->state);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetCoolantStateRes);
        break;
    }
    case SMC_CMD_GET_CURRENT_TOOL: {
        SmcGetCurrentToolRes *res = (SmcGetCurrentToolRes *)resp_buf;
        res->tool_id = 0; res->ret_code = 0;
        res->ret_code = SMC_GetCurrentTool(&res->tool_id);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetCurrentToolRes);
        break;
    }
    case SMC_CMD_GET_LASER_STATE: {
        /* P0-Laser-Q: 激光器完整状态查询 (14 字段 + ret_code, ~75B)
         * SMC_GetLaserState struct-based out param, 直接 fill res.
         * memset 兜底: 未配置激光时 SMC_GetLaserState 返回 -1 早返回,
         * pierce_count / laser_on_time_ms 等字段需为 0 而非未初始化垃圾值. */
        SmcGetLaserStateRes *res = (SmcGetLaserStateRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = SMC_GetLaserState(res);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetLaserStateRes);
        break;
    }

    case SMC_CMD_SET_OVERRIDE: {
        /* P2-A: 实时倍率 + 模式标志设置
         * mask/value 模式: -1=不改 (允许部分修改), 实际生效值通过 Res 回读.
         * EventLogger_Push 0x0042 INFO: 操作员旋钮变更审计 (便于事后追溯). */
        if (req_hdr.data_len < sizeof(SmcSetOverrideReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcSetOverrideReq *req = (SmcSetOverrideReq *)payload;
        SmcSetOverrideRes *res = (SmcSetOverrideRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = SMC_SetOverride(req->feed_pct, req->rapid_pct, req->spindle_pct,
                                        req->mode_mask, req->mode_value,
                                        &res->actual_feed_pct, &res->actual_rapid_pct,
                                        &res->actual_spindle_pct, &res->actual_mode_flags);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcSetOverrideRes);

        /* P2-A: MANUAL 0x0042 override changed (INFO, 便于操作员面板审计) */
        EventLogger_Push(SEVERITY_INFO, SOURCE_MANUAL, 0x0042,
                         (int32_t)res->actual_mode_flags,
                         "override changed by operator");
        break;
    }
    case SMC_CMD_GET_OVERRIDE: {
        /* P2-A: 实时倍率 + 模式标志查询 (无 Req, ~16B Res) */
        SmcGetOverrideRes *res = (SmcGetOverrideRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = SMC_GetOverride(&res->feed_pct, &res->rapid_pct,
                                        &res->spindle_pct, &res->mode_flags);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetOverrideRes);
        break;
    }

    /* ===== P0-Laser-ConfigRPC: 激光配置 (0x0050-0x0056) =====
     * 7 个 case 块同模式: data_len 守卫 → cast req → 调 SMC_ConfigLaser* → 返回 Res(ret_code)
     * ConfigLaser* 必须在 SMC_InitAndStart 前调 (init-time 单写者), rpc 层不做时序校验. */
    case SMC_CMD_CONFIG_LASER_IO: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserIOReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserIOReq *req = (SmcConfigLaserIOReq *)payload;
        SmcConfigLaserIORes *res = (SmcConfigLaserIORes *)resp_buf;
        res->ret_code = SMC_ConfigLaserIO(req->do_slave_id, req->ao_slave_id, req->di_slave_id);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserIORes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_DO_BITS: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserDOBitsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserDOBitsReq *req = (SmcConfigLaserDOBitsReq *)payload;
        SmcConfigLaserDOBitsRes *res = (SmcConfigLaserDOBitsRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserDOBits(req->b_enable, req->b_shutter,
                                              req->b_gas_n2, req->b_gas_o2,
                                              req->b_gas_air, req->b_alarm_lamp);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserDOBitsRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_DI_BITS: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserDIBitsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserDIBitsReq *req = (SmcConfigLaserDIBitsReq *)payload;
        SmcConfigLaserDIBitsRes *res = (SmcConfigLaserDIBitsRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserDIBits(req->b_door, req->b_estop, req->b_laser_alm,
                                              req->b_water_t, req->b_water_f, req->b_gas_p);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserDIBitsRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_AO_CHANNELS: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserAOChannelsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserAOChannelsReq *req = (SmcConfigLaserAOChannelsReq *)payload;
        SmcConfigLaserAOChannelsRes *res = (SmcConfigLaserAOChannelsRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserAOChannels(req->ch_power, req->ch_freq);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserAOChannelsRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_RANGE: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserRangeReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserRangeReq *req = (SmcConfigLaserRangeReq *)payload;
        SmcConfigLaserRangeRes *res = (SmcConfigLaserRangeRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserRange(req->power_max_w, req->freq_max_hz, req->power_min_w);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserRangeRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_COUPLING: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserCouplingReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserCouplingReq *req = (SmcConfigLaserCouplingReq *)payload;
        SmcConfigLaserCouplingRes *res = (SmcConfigLaserCouplingRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserCoupling(req->mode, req->v_thresh_mm_s);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserCouplingRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_COUPLE_TABLE: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserCoupleTableReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserCoupleTableReq *req = (SmcConfigLaserCoupleTableReq *)payload;
        SmcConfigLaserCoupleTableRes *res = (SmcConfigLaserCoupleTableRes *)resp_buf;
        /* count 越界 (0 或 >16) 由 SMC_ConfigLaserCoupleTable 内部校验, 返回 -1 */
        res->ret_code = SMC_ConfigLaserCoupleTable(req->points, req->count);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserCoupleTableRes);
        break;
    }
    case SMC_CMD_SET_OPTIONAL_STOP_ENABLE: {
        if (req_hdr.data_len < sizeof(SmcSetOptionalStopEnableReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcSetOptionalStopEnableReq *req = (SmcSetOptionalStopEnableReq *)payload;
        SmcSetOptionalStopEnableRes *res = (SmcSetOptionalStopEnableRes *)resp_buf;
        res->ret_code = SMC_SetOptionalStopEnable(req->enable);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcSetOptionalStopEnableRes);
        printf("[rpc] M1 可选停: enable=%d ret=%d\n", req->enable, res->ret_code);
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
        if (req->axis_letter == '*')
            printf("[rpc] 运动: 全轴归零\n");
        else
            printf("[rpc] 运动: %c轴 归零\n", req->axis_letter);
        break;
    }
    case SMC_CMD_MOVE_RELATIVE: {
        if (req_hdr.data_len < sizeof(SmcMoveRelativeReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcMoveRelativeReq *req = (SmcMoveRelativeReq *)payload;
        printf("[rpc] 运动: %c轴 相对移动 %.3f | 速度 %.1f\n",
               req->axis_letter, req->distance, req->speed);
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
        printf("[rpc] 运动: %c轴 回零 | 速度 %.1f\n",
               req->axis_letter, req->speed);
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
        printf("[rpc] 加工: 运行 %s\n", req->filepath);
        SmcRunGCodeFileRes *res = (SmcRunGCodeFileRes *)resp_buf;
        res->ret_code = SMC_RunGCodeFile(req->filepath);
        printf("[rpc] 加工: 返回码=%d\n", res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcRunGCodeFileRes);
        break;
    }
    case SMC_CMD_PAUSE_PROCESSING: {
        printf("[rpc] 加工: 暂停\n");
        SMC_PauseProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_RESUME_PROCESSING: {
        printf("[rpc] 加工: 继续\n");
        SMC_ResumeProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }
    case SMC_CMD_ABORT_PROCESSING: {
        printf("[rpc] 加工: 中止\n");
        SMC_AbortProcessing();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 0;
        break;
    }

    /* ===== P0-b v2: LoadProgram / RunLoadedProgram / GetProgramStructure ===== */
    case SMC_CMD_LOAD_PROGRAM: {
        if (req_hdr.data_len < sizeof(SmcLoadProgramReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcLoadProgramReq *req = (SmcLoadProgramReq *)payload;
        req->filepath[SMC_FILEPATH_MAX_LEN - 1] = '\0';
        printf("[rpc] LoadProgram (preview): %s\n", req->filepath);
        SmcLoadProgramRes *res = (SmcLoadProgramRes *)resp_buf;
        res->ret_code = SMC_LoadProgram(req->filepath);
        printf("[rpc] LoadProgram: 返回码=%d\n", res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcLoadProgramRes);
        break;
    }
    case SMC_CMD_RUN_LOADED_PROGRAM: {
        printf("[rpc] RunLoadedProgram\n");
        SmcRunLoadedProgramRes *res = (SmcRunLoadedProgramRes *)resp_buf;
        res->ret_code = SMC_RunLoadedProgram();
        printf("[rpc] RunLoadedProgram: 返回码=%d\n", res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcRunLoadedProgramRes);
        break;
    }
    case SMC_CMD_GET_PROGRAM_STRUCTURE: {
        /* 无 Req, Res = SmcGetProgramStructureRes (~390B < SMC_RESP_BUF_LEN=512) */
        SmcGetProgramStructureRes *res = (SmcGetProgramStructureRes *)resp_buf;
        SMC_GetProgramStructure(res);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetProgramStructureRes);
        break;
    }
    case SMC_CMD_CLEAR_ALARM: {
        /* P1-b: 无 Req, Res = SmcClearAlarmRes */
        printf("[rpc] ClearAlarm\n");
        SmcClearAlarmRes *res = (SmcClearAlarmRes *)resp_buf;
        res->ret_code = SMC_ClearAlarm();
        printf("[rpc] ClearAlarm: 返回码=%d (0=ok, -1=parser busy, -2=axes not ready)\n",
               res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcClearAlarmRes);
        break;
    }

    /* ===== P0-A: Emergency Stop (2026-07-21, 软急停一站式触发) ===== */
    case SMC_CMD_EMERGENCY_STOP: {
        if (req_hdr.data_len < sizeof(SmcEmergencyStopReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcEmergencyStopReq *req = (SmcEmergencyStopReq *)payload;
        printf("[rpc] 急停: reason=%d msg='%s'\n", req->reason_code, req->message);
        SmcEmergencyStopRes *res = (SmcEmergencyStopRes *)resp_buf;
        res->ret_code = SMC_EmergencyStop(req->reason_code, req->message);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcEmergencyStopRes);
        break;
    }

    /* ===== B2 (2026-07-23): Gantry Pre-Align 配置 ===== */
    case SMC_CMD_CONFIG_GANTRY_ALIGN: {
        if (req_hdr.data_len < sizeof(SmcConfigGantryAlignReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigGantryAlignReq *req = (SmcConfigGantryAlignReq *)payload;
        SmcConfigGantryAlignRes *res = (SmcConfigGantryAlignRes *)resp_buf;
        res->ret_code = SMC_ConfigGantryAlign((char)req->z_letter,
                                              req->tol_pulse,
                                              req->timeout_ms);
        printf("[rpc] ConfigGantryAlign: z=%c tol=%d timeout=%d ret=%d\n",
               (char)req->z_letter, req->tol_pulse, req->timeout_ms, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigGantryAlignRes);
        break;
    }

    /* ===== B2 (2026-07-23): Gantry Mock 注入 (sim-only) ===== */
    case SMC_CMD_INJECT_GANTRY_OFFSET: {
        if (req_hdr.data_len < sizeof(SmcInjectGantryOffsetReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInjectGantryOffsetReq *req = (SmcInjectGantryOffsetReq *)payload;
        SmcInjectGantryOffsetRes *res = (SmcInjectGantryOffsetRes *)resp_buf;
        res->ret_code = SMC_InjectGantryOffset((char)req->z_letter,
                                                req->offset_pulse);
        printf("[rpc] InjectGantryOffset: z=%c offset=%d ret=%d\n",
               (char)req->z_letter, req->offset_pulse, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcInjectGantryOffsetRes);
        break;
    }

    /* ===== P0-3: Safe Z Lift ===== */
    case SMC_CMD_CONFIG_SAFE_LIFT: {
        if (req_hdr.data_len < sizeof(SmcConfigSafeLiftReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigSafeLiftReq *req = (SmcConfigSafeLiftReq *)payload;
        SmcConfigSafeLiftRes *res = (SmcConfigSafeLiftRes *)resp_buf;
        res->ret_code = SMC_ConfigSafeLiftZ((char)req->z_letter,
                                             req->safe_z_mm,
                                             req->lift_speed_mm_s,
                                             req->auto_on_alarm);
        printf("[rpc] ConfigSafeLiftZ: z=%c target=%.2f ret=%d\n",
               (char)req->z_letter, req->safe_z_mm, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigSafeLiftRes);
        break;
    }
    case SMC_CMD_SAFE_LIFT_TRIGGER: {
        /* 无 Req */
        SmcSafeLiftTriggerRes *res = (SmcSafeLiftTriggerRes *)resp_buf;
        res->ret_code = SMC_SafeLiftZ();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcSafeLiftTriggerRes);
        break;
    }
    case SMC_CMD_SAFE_LIFT_CANCEL: {
        /* 无 Req */
        SmcSafeLiftCancelRes *res = (SmcSafeLiftCancelRes *)resp_buf;
        res->ret_code = SMC_CancelSafeLiftZ();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcSafeLiftCancelRes);
        break;
    }
    case SMC_CMD_GET_SAFE_LIFT: {
        /* 无 Req, Res = SmcGetSafeLiftRes (~36B) */
        SmcGetSafeLiftRes *res = (SmcGetSafeLiftRes *)resp_buf;
        int state = 0;
        double progress = 0.0;
        int rc = SMC_GetSafeLiftState(&state, &progress);
        res->ret_code     = rc;
        res->state        = state;
        res->enabled      = g_safe_lift_cfg.enabled;
        res->_pad         = 0;
        res->progress_mm  = progress;
        res->z_target_mm  = g_safe_lift_cfg.safe_z_target_mm;
        res->z_current_mm = (g_safe_lift_cfg.enabled && g_safe_lift_cfg.z_axis_idx >= 0)
                            ? g_axis[g_safe_lift_cfg.z_axis_idx].current_cmd_pos
                            : 0.0;
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetSafeLiftRes);
        break;
    }

    /* ===== P0-1 Homing ===== */
    case SMC_CMD_CONFIG_HOMING_AXIS: {
        if (req_hdr.data_len < sizeof(SmcConfigHomingAxisReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigHomingAxisReq *req = (SmcConfigHomingAxisReq *)payload;
        SmcConfigHomingAxisRes *res = (SmcConfigHomingAxisRes *)resp_buf;
        res->ret_code = SMC_ConfigHoming((char)req->z_letter, req->method,
                                          req->search_speed_mm_s,
                                          req->creep_speed_mm_s,
                                          req->direction, req->timeout_ms);
        printf("[rpc] ConfigHoming %c method=%d ret=%d\n",
               (char)req->z_letter, req->method, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigHomingAxisRes);
        break;
    }
    case SMC_CMD_CONFIG_HOMING_ORDER: {
        if (req_hdr.data_len < sizeof(SmcConfigHomingOrderReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigHomingOrderReq *req = (SmcConfigHomingOrderReq *)payload;
        req->order_letters[SMC_HOMING_ORDER_MAX_LEN - 1] = '\0';
        SmcConfigHomingOrderRes *res = (SmcConfigHomingOrderRes *)resp_buf;
        res->ret_code = SMC_ConfigHomingAll(req->order_letters);
        printf("[rpc] ConfigHomingAll order=%s ret=%d\n",
               req->order_letters, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigHomingOrderRes);
        break;
    }
    /* ===== B4 (2026-07-23): Homing Order v2 (order + auto_on_init) ===== */
    case SMC_CMD_CONFIG_HOMING_ORDER_EX: {
        if (req_hdr.data_len < sizeof(SmcConfigHomingOrderExReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigHomingOrderExReq *req = (SmcConfigHomingOrderExReq *)payload;
        req->order_letters[SMC_HOMING_ORDER_MAX_LEN - 1] = '\0';
        SmcConfigHomingOrderExRes *res = (SmcConfigHomingOrderExRes *)resp_buf;
        res->ret_code = SMC_ConfigHomingAllEx(req->order_letters, req->auto_on_init);
        printf("[rpc] ConfigHomingAllEx order=%s auto_on_init=%d ret=%d\n",
               req->order_letters, req->auto_on_init, res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigHomingOrderExRes);
        break;
    }
    case SMC_CMD_HOMING_TRIGGER: {
        if (req_hdr.data_len < sizeof(SmcHomingTriggerReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcHomingTriggerReq *req = (SmcHomingTriggerReq *)payload;
        SmcHomingTriggerRes *res = (SmcHomingTriggerRes *)resp_buf;
        if (req->z_letter == 0) {
            res->ret_code = SMC_HomeAll();
            printf("[rpc] HomeAll ret=%d\n", res->ret_code);
        } else {
            res->ret_code = SMC_HomeAxis((char)req->z_letter);
            printf("[rpc] HomeAxis %c ret=%d\n", (char)req->z_letter, res->ret_code);
        }
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcHomingTriggerRes);
        break;
    }
    case SMC_CMD_HOMING_CANCEL: {
        SmcHomingCancelRes *res = (SmcHomingCancelRes *)resp_buf;
        res->ret_code = SMC_CancelHoming();
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcHomingCancelRes);
        break;
    }
    case SMC_CMD_GET_HOMING: {
        SmcGetHomingRes *res = (SmcGetHomingRes *)resp_buf;
        int state = 0, axis_idx = -1;
        double progress = 0.0;
        int rc = SMC_GetHomingState(&state, &axis_idx, &progress);
        res->ret_code     = rc;
        res->state        = state;
        res->axis_idx     = axis_idx;
        res->enabled      = g_homing_cfg.enabled;
        res->_pad         = 0;
        res->progress_pct = progress;
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetHomingRes);
        break;
    }

    /* ===== P0-1 JOG ===== */
    case SMC_CMD_JOG_START: {
        if (req_hdr.data_len < sizeof(SmcJogStartReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcJogStartReq *req = (SmcJogStartReq *)payload;
        SmcJogStartRes *res = (SmcJogStartRes *)resp_buf;
        res->ret_code = SMC_JogStart((char)req->z_letter, req->direction,
                                      req->speed_mm_s);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcJogStartRes);
        break;
    }
    case SMC_CMD_JOG_STOP: {
        if (req_hdr.data_len < sizeof(SmcJogStopReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcJogStopReq *req = (SmcJogStopReq *)payload;
        SmcJogStopRes *res = (SmcJogStopRes *)resp_buf;
        res->ret_code = SMC_JogStop((char)req->z_letter);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcJogStopRes);
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

/* =====================================================================
 * 内核初始化 — 与 main.c 硬编码配置等价 (生产环境用 RPC 覆盖)
 * ================================================================== */

extern int g_sim_mode;
extern void axis_sys_init(void);

static int kernel_init(const char *iface)
{
    /* 仿真模式检测 */
    if (strcmp(iface, "sim") == 0) {
        g_sim_mode = 1;
        printf("[rpc] ### 纯软件仿真模式已激活 ###\n");
    }

    printf("\n==============================================\n");
    printf("     SMC 五轴高端数控系统内核 (V2.0) \n");
    if (g_sim_mode) printf("     [SIMULATION MODE - 无真实硬件]\n");
    printf("==============================================\n");

    /* 1. 系统底层内存与互斥锁初始化 */
    axis_sys_init();
    Macro_Init();

    /* P0-a: 状态快照中心初始化 (RT 线程启动前必须就绪, 否则首次 Publish 读未初始化字段) */
    SnapshotHub_Init();

    /* P0-b v1: 段流推送中心初始化 (parser 启动前必须就绪, 否则首次 Push 写未初始化 ring) */
    PreviewStreamer_Init();

    /* P1-b: 事件/报警流推送中心初始化 */
    EventLogger_Init();

    /* 2. 默认轴拓扑 — MoveControl 启动后通过 RPC 覆盖 */
    SMC_ConfigAxisTopology("X", 0, 5, 0);
    SMC_ConfigAxisTopology("Y", 1, 3, 4);
    SMC_ConfigAxisTopology("Z", 0, 6, 0);
    SMC_ConfigAxisTopology("C", 0, 1, 0);
    SMC_ConfigAxisTopology("B", 0, 2, 0);

    /* 3. 脉冲当量 */
    SMC_ConfigPulsePerUnit('X', 10000.0);
    SMC_ConfigPulsePerUnit('Y', 10000.0);
    SMC_ConfigPulsePerUnit('Z', 1000.0);
    SMC_ConfigPulsePerUnit('C', 2777.7778);
    SMC_ConfigPulsePerUnit('B', 2777.7778);

    /* 4. 动力学 */
    SMC_ConfigAxisDynamics('X', 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics('Y', 0, 50.0, 200.0, 200.0, 0.0);
    SMC_ConfigAxisDynamics('Z', 0, 30.0, 100.0, 100.0, 0.0);
    SMC_ConfigAxisDynamics('C', 1, 18.0,  72.0,  72.0, 50.0);
    SMC_ConfigAxisDynamics('B', 1, 18.0,  72.0,  72.0, 80.0);

    /* 5. 五轴运动学 (Head-Head 构型) */
    double tool_off[3]  = {0.0, 0.0, 150.0};
    double pivot_off[3] = {0.0, 0.0, 200.0};
    SMC_ConfigKinematicsOffset(150.0, 0.0, 0.0, 200.0);
    SMC_ConfigKinematics(KIN_HEAD_HEAD,
                         g_axis_map['B'-'A'], 1,
                         g_axis_map['C'-'A'], 2,
                         tool_off, pivot_off);

    /* 6. 规划器与安全 */
    SMC_ConfigPlannerParams(0.05, 500.0);
    SMC_ConfigGantrySyncAlarm('Y', 1, 1000, 8000, 100);
    SMC_ConfigSoftLimit('Z', 1, -500.0, 200.0);

    /* 7. 启动 EtherCAT / 仿真内核 */
    if (SMC_InitAndStart(iface) != 0) {
        fprintf(stderr, "[rpc] 内核启动失败\n");
        return -1;
    }

    /* 等待全轴就绪 */
    while (!g_all_axis_op_ready) {
        usleep(100000);
    }

    printf("[rpc] 物理原点锚定, 内核就绪\n");

    SMC_SetZero(SMC_AXIS_ALL);
    sleep(1);
    printf("[rpc] G54 坐标系初始化完毕, 准备接受 MoveControl 连接\n");

    return 0;
}

/* =====================================================================
 *  kernel_init_hw —— 实机参数 profile (P1 档案化, 2026-08-28)
 *
 *  用法: sudo ./rpc_server <EtherCAT网卡名|sim> hw
 *    · 与 kernel_init (sim 默认值) 并存, 互不影响
 *    · "sim hw" 组合 = sim 运动 + hw 档案参数 (先在仿真里验证档案本身)
 *
 *  参数来源 (原编译期 g_hw_* 表已由 param_store 取代, 默认值单一维护点
 *  在 param_store.c ps_default_profile — 与旧表逐值等价):
 *    环境变量 SMC_MACHINE_PROFILE → 默认 ./machine_profile.ini
 *    · 文件缺失/损坏 → 编译期默认 (WARN 事件 0x0002, 行为同改造前)
 *    · 文件含 PS_TODO_* 哨兵 → param_store_check A/B/C 分级
 *      (A 级未填拒绝启动, B 级跳过该项警告, 与原 IBN 填空语义一致)
 *    · 运行期经 OPC UA CNC.Config / 9527 RPC 修改的参数写入即存回此文件
 * =====================================================================
 */

/* @Context: Non-RealTime (main 启动早期, RT 线程创建前)
 * 与 kernel_init 同序列, 数据源换为 param_store 机床参数档案 */
static int kernel_init_hw(const char *iface)
{
    /* 仿真模式检测 (允许 "sim hw": sim 运动 + hw 档案参数联调;
     * sim 下 param_store 拒绝落盘, 不会污染实机档案) */
    if (strcmp(iface, "sim") == 0) {
        g_sim_mode = 1;
        printf("[rpc] ### 仿真模式 + 实机参数档案 ###\n");
    }

    printf("\n==============================================\n");
    printf("     SMC 五轴高端数控系统内核 (V2.0-HW)\n");
    if (g_sim_mode) printf("     [SIMULATION MOTION + HW PROFILE]\n");
    printf("==============================================\n");

    /* 1. 底层初始化 (EventLogger 必须先于 param_store_load,
     *    否则档案回退 WARN 进不了事件环) */
    axis_sys_init();
    Macro_Init();
    SnapshotHub_Init();
    PreviewStreamer_Init();
    EventLogger_Init();

    /* 2. 档案加载 + A/B/C 哨兵校验 (A 级未填 → 拒绝启动, 防 sim 值上实机) */
    param_store_load();
    if (param_store_check() != 0)
        return -1;
    const MachineProfile_t *hw = param_store_get();

    /* 3. A1 轴拓扑 (axis_order 顺序 = 房间号分配顺序) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        SMC_ConfigAxisTopology(hw->axis_order[i], hw->ax[i].is_dual,
                               hw->ax[i].master_id, hw->ax[i].slave_id);

    /* 4. A2 脉冲当量 (静止守卫: 此刻队列空/未运行, 必然通过) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        SMC_ConfigPulsePerUnit(hw->axis_order[i][0], hw->ax[i].ppu);

    /* 5. B1 轴动力学 (哨兵未填跳过 → 系统 BSS 默认保守值) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].max_speed != PS_TODO_D)
            SMC_ConfigAxisDynamics(hw->axis_order[i][0], hw->ax[i].dyn_type,
                                   hw->ax[i].max_speed, hw->ax[i].max_acc,
                                   hw->ax[i].max_dec, hw->ax[i].eq_radius);

    /* 6. B2 五轴运动学 (Head-Head B绕Y/C绕Z 固定构型, 偏置来自档案;
     *    哨兵未填跳过 → RTCP 不可用) */
    if (hw->tool_len != PS_TODO_D && hw->pivot_z != PS_TODO_D) {
        double tool_off[3]  = {0.0, 0.0, hw->tool_len};
        double pivot_off[3] = {hw->pivot_x, hw->pivot_y, hw->pivot_z};
        SMC_ConfigKinematicsOffset(hw->tool_len, hw->pivot_x,
                                   hw->pivot_y, hw->pivot_z);
        SMC_ConfigKinematics(KIN_HEAD_HEAD,
                             g_axis_map['B'-'A'], 1,
                             g_axis_map['C'-'A'], 2,
                             tool_off, pivot_off);
    }

    /* 7. C 级规划器 (档案值; 回退默认 0.05/500) */
    if (hw->corner_tolerance > 0.0 && hw->max_centripetal_acc > 0.0)
        SMC_ConfigPlannerParams(hw->corner_tolerance, hw->max_centripetal_acc);

    /* 8. A3 软限位 (enable=1 的轴) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].sl_enable)
            SMC_ConfigSoftLimit(hw->axis_order[i][0], 1,
                                hw->ax[i].sl_neg, hw->ax[i].sl_pos);

    /* 9. 双驱龙门 (按拓扑自动发现): sync 三值来自档案;
     *    align tol 由 ppu 折算 0.05mm (派生值, 不入档案) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++) {
        if (hw->ax[i].is_dual != 1) continue;
        char letter = hw->axis_order[i][0];
        if (hw->ax[i].gs_tol_pulse > 0)
            SMC_ConfigGantrySyncAlarm(letter, 1,
                                      hw->ax[i].gs_tol_pulse,
                                      hw->ax[i].gs_max_err_pulse,
                                      hw->ax[i].gs_err_time_ms);
        SMC_ConfigGantryAlign(letter, (int32_t)(hw->ax[i].ppu * 0.05), 3000);
        printf("[hw] 双驱轴 %c: 龙门同步 (档案) + 预对中 (ppu 折算)\n", letter);
    }

    /* 10. A4 Z 安全抬升 (auto_on_alarm 来自档案, 默认 0 — 2026-08-28 实机禁用
     *     自动抬升: 寸动调机期误触发 + DONE 态锁死全轴寸动; 档案改 1 启用,
     *     手动抬升 API 0x0057 不受影响) */
    if (hw->safe_z_mm != PS_TODO_D)
        SMC_ConfigSafeLiftZ('Z', hw->safe_z_mm, hw->lift_speed_mm_s,
                            hw->auto_on_alarm);

    /* 11. B 级回零 (v1 method 35 软件法; 顺序/auto_on_init 来自档案,
     *     auto_on_init=0 手动触发, 失败要人看见。硬件开关接入后切 1-19) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].hm_enable)
            SMC_ConfigHoming(hw->axis_order[i][0], hw->ax[i].hm_method,
                             0, 0, hw->ax[i].hm_direction, hw->ax[i].hm_timeout_ms);
    if (hw->homing_order[0] != '\0')
        SMC_ConfigHomingAllEx(hw->homing_order, hw->auto_on_init);

    /* 12. 启动内核 (实机 = EtherCAT 组态 + CiA402 使能; sim = 软件驱动) */
    if (SMC_InitAndStart(iface) != 0) {
        fprintf(stderr, "[rpc] 内核启动失败\n");
        return -1;
    }
    while (!g_all_axis_op_ready) {
        usleep(100000);
    }
    printf("[rpc][hw] 内核就绪 (实机参数档案)\n");

    /* 注意: 与 sim 不同, 这里【不】自动 SetZero —
     * 实机坐标基准由回零 (Home.All) 建立, 而非上电位置 */
    printf("[rpc][hw] 等待回零建立 G53 基准 (movecontrol: CNC.Home.All)\n");

    return 0;
}
/* =====================================================================
 * main — rpc_server 的唯一入口
 *   $ sudo ./rpc_server enp7s0            (实机, sim 默认参数)
 *   $ sudo ./rpc_server enp7s0 hw         (实机, 实机参数 profile)
 *   $ sudo ./rpc_server sim               (纯软件仿真)
 *   $ sudo ./rpc_server sim hw            (仿真运动 + 实机参数联调)
 * ================================================================== */
int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("用法: sudo %s <EtherCAT网卡名|sim> [hw]\n", argv[0]);
        printf("      sudo %s sim                 (纯软件仿真, 无硬件)\n", argv[0]);
        printf("      sudo %s enp7s0              (实机, sim 默认参数, 调试用)\n", argv[0]);
        printf("      sudo %s enp7s0 hw           (实机, 实机参数 profile, 上机用)\n", argv[0]);
        return 1;
    }

    /* argv[2]=="hw": 选实机参数 profile (kernel_init_hw) */
    const int use_hw_profile = (argc >= 3 && strcmp(argv[2], "hw") == 0);

    /* 客户端断开时忽略 SIGPIPE */
    signal(SIGPIPE, SIG_IGN);

    /* 初始化内核 */
    if (use_hw_profile) {
        if (kernel_init_hw(argv[1]) != 0) {
            return 1;
        }
    } else if (kernel_init(argv[1]) != 0) {
        return 1;
    }

    /* 启动 RPC 监听 */
    printf("[rpc] 启动 TCP 服务...\n");

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { perror("[rpc] socket"); return 1; }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(SMC_RPC_PORT);

    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[rpc] bind"); close(listen_fd); return 1;
    }
    if (listen(listen_fd, SMC_RPC_BACKLOG) < 0) {
        perror("[rpc] listen"); close(listen_fd); return 1;
    }
    printf("[rpc] listening on 0.0.0.0:%d ...\n", SMC_RPC_PORT);

    /* P0-a: 启动状态推送通道 (9528, 独立线程, 多客户端)。失败不 fatal, 仅 9527 RPC 可用 */
    if (rpc_push_server_start() < 0) {
        fprintf(stderr, "[push] 推送通道启动失败, 仅 RPC 9527 可用\n");
    }

    /* P0-b v1: 启动段流推送通道 (9529, 独立线程, 多客户端)。失败不 fatal */
    if (rpc_preview_server_start() < 0) {
        fprintf(stderr, "[preview] 段流通道启动失败, 仅 RPC 9527 + push 9528 可用\n");
    }

    /* P1-b: 启动事件流推送通道 (9530, 独立线程, 多客户端)。失败不 fatal */
    if (rpc_event_server_start() < 0) {
        fprintf(stderr, "[event] 事件流通道启动失败, 仅 RPC + push + preview 可用\n");
    }

    /* 契约 P4: OPC UA Server (4840, 独立线程, movecontrol 标准对外通道)。
     * 失败不 fatal — 四端口 RPC 通道继续可用 */
    if (opcua_server_start() < 0) {
        fprintf(stderr, "[opcua] OPC UA 服务启动失败, 仅 RPC/push/preview/event 可用\n");
    }

    /* 接受连接主循环 */
    for (;;) {
        int client_fd = accept(listen_fd, NULL, NULL);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            perror("[rpc] accept");
            continue;
        }
        printf("[rpc] client connected, fd=%d\n", client_fd);

        while (handle_client_request(client_fd) == 0) {
            /* 处理请求直到对端断开 */;
        }
        close(client_fd);
        printf("[rpc] client disconnected, fd=%d\n", client_fd);
    }

    close(listen_fd);
    return 0;
}

/* ---- 旧版线程 API (已废弃, 保留兼容) ---- */

void rpc_server_start(void) {}
void rpc_server_stop(void) {}
