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
#include "periph_bus.h"        /* P0-1: PeriphBus_Init (软 PLC 地基) */
#include "rpc_push_server.h"   /* P0-a: rpc_push_server_start */
#include "preview_streamer.h"  /* P0-b v1: PreviewStreamer_Init */
#include "rpc_preview_server.h" /* P0-b v1: rpc_preview_server_start */
#include "event_logger.h"      /* P1-b: EventLogger_Init */
#include "block_timing.h"      /* 论文1 S10: g_bt / bt_reset_all (RPC 0x0089) */
#include "cutter_comp.h"       /* V13.1: CutterComp_G93Sem* (RPC 0x008A) */
#include "rpc_event_server.h"  /* P1-b: rpc_event_server_start */
#include "opcua_server.h"      /* 契约 P4: opcua_server_start (OPC UA 4840 对外通道) */
#include "param_store.h"       /* P1+P2: 机床参数档案 (kernel_init_hw 数据源) */
#include "ecat_core.h"         /* P0-4a: ecat_set_expected_topo (拓扑校验) */
#include "limit_map.h"         /* Phase 1: 限位映射 runtime 表 (apply 11.8) */
#include "plc_core.h"          /* P1 软 PLC: plc_core_download/get_state (0x0076/0x0079) */
#include "periph_ai.h"         /* P1.5-1: AI 通道 (0x007C/0x007D + init) */
#include "plc_trace.h"         /* P1.5-3: Trace (0x007F-0x0081) */

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
    case SMC_CMD_CONFIG_LASER_GAS: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserGasReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserGasReq *req = (SmcConfigLaserGasReq *)payload;
        SmcConfigLaserGasRes *res = (SmcConfigLaserGasRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserGas(req->ch_ao_pressure, req->pressure_max_bar);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserGasRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_MICRO_JOINT: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserMicroJointReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserMicroJointReq *req = (SmcConfigLaserMicroJointReq *)payload;
        SmcConfigLaserMicroJointRes *res = (SmcConfigLaserMicroJointRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserMicroJoint(req->mj_ratio);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserMicroJointRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_PIERCE_TABLE: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserPierceTableReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserPierceTableReq *req = (SmcConfigLaserPierceTableReq *)payload;
        SmcConfigLaserPierceTableRes *res = (SmcConfigLaserPierceTableRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserPierceTable(req->steps, req->count);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserPierceTableRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_INTERLOCK: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserInterlockReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserInterlockReq *req = (SmcConfigLaserInterlockReq *)payload;
        SmcConfigLaserInterlockRes *res = (SmcConfigLaserInterlockRes *)resp_buf;
        /* count/bit/mask 越界由 SMC_ConfigLaserInterlock 内部校验, 返回 -1 */
        res->ret_code = SMC_ConfigLaserInterlock(req->rules, req->count);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserInterlockRes);
        break;
    }
    case SMC_CMD_INJECT_LASER_DI: {
        if (req_hdr.data_len < sizeof(SmcInjectLaserDiReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInjectLaserDiReq *req = (SmcInjectLaserDiReq *)payload;
        SmcInjectLaserDiRes *res = (SmcInjectLaserDiRes *)resp_buf;
        res->ret_code = SMC_InjectLaserDi(req->pattern, req->enable);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcInjectLaserDiRes);
        break;
    }
    case SMC_CMD_INJECT_SIM_LIMIT_INPUT: {
        /* Phase 1 (2026-11-23): sim-only 按站注入 0x60FD 输入字+有效标志 */
        if (req_hdr.data_len < sizeof(SmcInjectSimLimitInputReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInjectSimLimitInputReq *req = (SmcInjectSimLimitInputReq *)payload;
        SmcInjectSimLimitInputRes *res = (SmcInjectSimLimitInputRes *)resp_buf;
        res->ret_code = SMC_InjectSimLimitInput(req->slave_id, req->valid, req->word);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcInjectSimLimitInputRes);
        break;
    }
    case SMC_CMD_QUERY_LIMIT_DIAG: {
        /* Phase 1 (2026-11-23): 限位映射诊断查询 (轴×方向 定长 10 条目) */
        SmcQueryLimitDiagRes *res = (SmcQueryLimitDiagRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = SMC_QueryLimitDiag(res->entry);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcQueryLimitDiagRes);
        break;
    }
    case SMC_CMD_CONFIG_LASER_PDO_LAYOUT: {
        if (req_hdr.data_len < sizeof(SmcConfigLaserPdoLayoutReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigLaserPdoLayoutReq *req = (SmcConfigLaserPdoLayoutReq *)payload;
        SmcConfigLaserPdoLayoutRes *res = (SmcConfigLaserPdoLayoutRes *)resp_buf;
        res->ret_code = SMC_ConfigLaserPdoLayout(req->di_byte_off, req->di_bytes,
                                                 req->do_byte_off, req->do_bytes,
                                                 req->ao_byte_off, req->ao_stride,
                                                 req->reserved);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigLaserPdoLayoutRes);
        break;
    }
    case SMC_CMD_PLC_DOWNLOAD: {
        /* Req 变长: 8B 头 + text_len 字节; 协议层 err 仅在头不完整时用 */
        SmcPlcDownloadRes *res = (SmcPlcDownloadRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcDownloadRes);
        if (req_hdr.data_len < sizeof(SmcPlcDownloadReqHead)) {
            res->ret_code = -2;
            break;
        }
        SmcPlcDownloadReqHead *h = (SmcPlcDownloadReqHead *)payload;
        if (h->text_len < 1 || h->text_len > SMC_PLC_TEXT_MAX
            || (size_t)h->text_len + sizeof(*h) > req_hdr.data_len) {
            res->ret_code = -2;
            break;
        }
        PlcDiag_t diags[SMC_PLC_DIAG_IN_RESP];
        int nd = 0;
        res->ret_code = plc_core_download((const char *)payload + sizeof(*h),
                                          h->text_len, h->crc32,
                                          diags, SMC_PLC_DIAG_IN_RESP, &nd);
        if (res->ret_code == -1) {
            int ncopy = (nd < SMC_PLC_DIAG_IN_RESP) ? nd : SMC_PLC_DIAG_IN_RESP;
            res->n_diag_total = nd;
            res->n_diag_included = ncopy;
            memcpy(res->diags, diags, (size_t)ncopy * sizeof(SmcPlcDiag_t));
            printf("[rpc] PLC 下载: 编译失败 ret=-1 诊断 %d 条 (首条: 行 %d %s)\n",
                   nd, ncopy > 0 ? diags[0].line : -1,
                   ncopy > 0 ? diags[0].msg : "");
        } else {
            printf("[rpc] PLC 下载: ret=%d\n", res->ret_code);
        }
        break;
    }
    case SMC_CMD_PLC_GET_STATE: {
        SmcPlcGetStateRes *res = (SmcPlcGetStateRes *)resp_buf;
        PlcCoreStatus_t st;
        plc_core_get_status(&st);
        res->state = st.state;
        res->n_vars = st.n_vars;
        res->n_stmts = st.n_stmts;
        res->text_len = st.text_len;
        res->heartbeat = st.heartbeat;
        res->cycle_us = st.cycle_us;
        res->err_line = st.err_line;
        memset(res->err_msg, 0, sizeof(res->err_msg));
        strncpy(res->err_msg, st.err_msg, sizeof(res->err_msg) - 1);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcGetStateRes);
        break;
    }
    case SMC_CMD_PLC_CTRL: {
        if (req_hdr.data_len < sizeof(SmcPlcCtrlReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcCtrlReq *req = (SmcPlcCtrlReq *)payload;
        SmcPlcCtrlRes *res = (SmcPlcCtrlRes *)resp_buf;
        res->ret_code = plc_core_ctrl(req->action);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcCtrlRes);
        printf("[rpc] PLC ctrl: action=%d ret=%d\n", req->action, res->ret_code);
        break;
    }
    case SMC_CMD_PLC_TEST_HOOK: {
        if (req_hdr.data_len < sizeof(SmcPlcTestHookReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcTestHookReq *req = (SmcPlcTestHookReq *)payload;
        SmcPlcTestHookRes *res = (SmcPlcTestHookRes *)resp_buf;
        res->ret_code = plc_core_test_freeze(req->freeze_hb);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcTestHookRes);
        break;
    }
    case SMC_CMD_CONFIG_PERIPH_AI: {
        if (req_hdr.data_len < sizeof(SmcConfigPeriphAiReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigPeriphAiReq *req = (SmcConfigPeriphAiReq *)payload;
        SmcConfigPeriphAiRes *res = (SmcConfigPeriphAiRes *)resp_buf;
        if (req->reserved != 0) {
            res->ret_code = -1;
        } else {
            res->ret_code = SMC_ConfigPeriphAi(req->slave_id, req->byte_off,
                                               req->stride, req->count, req->fs);
        }
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigPeriphAiRes);
        break;
    }
    case SMC_CMD_INJECT_PERIPH_AI: {
        if (req_hdr.data_len < sizeof(SmcInjectPeriphAiReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcInjectPeriphAiReq *req = (SmcInjectPeriphAiReq *)payload;
        SmcInjectPeriphAiRes *res = (SmcInjectPeriphAiRes *)resp_buf;
        res->ret_code = SMC_InjectPeriphAi(req->ch, req->val);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcInjectPeriphAiRes);
        break;
    }
    case SMC_CMD_PLC_FORCE_VAR: {
        if (req_hdr.data_len < sizeof(SmcPlcForceVarReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcForceVarReq *req = (SmcPlcForceVarReq *)payload;
        SmcPlcForceVarRes *res = (SmcPlcForceVarRes *)resp_buf;
        /* name NUL 终结防御: 结构体尾是 reserved, name 可能未终结 */
        char name[33];
        memcpy(name, req->name, 32);
        name[32] = '\0';
        if (req->reserved != 0) {
            res->ret_code = -1;
        } else {
            res->ret_code = SMC_PlcForceVar(name, req->val, req->on);
        }
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcForceVarRes);
        break;
    }
    case SMC_CMD_PLC_TRACE_CONFIG: {
        if (req_hdr.data_len < sizeof(SmcPlcTraceConfigReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcTraceConfigReq *req = (SmcPlcTraceConfigReq *)payload;
        SmcPlcTraceConfigRes *res = (SmcPlcTraceConfigRes *)resp_buf;
        PlcTraceCfg_t cfg;
        memset(&cfg, 0, sizeof(cfg));
        cfg.n_ch = req->n_ch;
        cfg.div = req->div;
        cfg.trig_kind = req->trig_kind;
        cfg.trig_ch = req->trig_ch;
        cfg.trig_threshold = req->trig_threshold;
        for (int i = 0; i < SMC_PLC_TRACE_CH_MAX; i++) {
            memcpy(cfg.names[i], req->names[i], 32);
            cfg.names[i][31] = '\0';   /* NUL 终结防御 */
        }
        res->ret_code = plc_trace_config(&cfg);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcTraceConfigRes);
        break;
    }
    case SMC_CMD_PLC_TRACE_CTRL: {
        if (req_hdr.data_len < sizeof(SmcPlcTraceCtrlReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcTraceCtrlReq *req = (SmcPlcTraceCtrlReq *)payload;
        SmcPlcTraceCtrlRes *res = (SmcPlcTraceCtrlRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcTraceCtrlRes);
        switch (req->action) {
        case 1:
            res->ret_code = plc_core_trace_arm();
            break;
        case 0:
            res->ret_code = plc_trace_stop();
            break;
        case 2:
            res->ret_code = plc_trace_abort();
            break;
        case 3:
            res->ret_code = 0;
            break;
        default:
            res->ret_code = -1;
            break;
        }
        int st = 0, n = 0;
        plc_trace_get_state(&st, &n);
        res->state = st;
        res->n_samples = n;
        break;
    }
    case SMC_CMD_PLC_TRACE_READ: {
        if (req_hdr.data_len < sizeof(SmcPlcTraceReadReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcTraceReadReq *req = (SmcPlcTraceReadReq *)payload;
        SmcPlcTraceReadRes *res = (SmcPlcTraceReadRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 16;      /* 无样本时仅头 */
        PlcTraceSample_t s[SMC_PLC_TRACE_PER_PAGE];
        int n_total = 0;
        int n = plc_trace_read(req->offset, s, SMC_PLC_TRACE_PER_PAGE, &n_total);
        res->n_total = n_total;
        if (n == -3 || n == -2) {
            res->ret_code = n;
            res->next_offset = -1;
            break;
        }
        for (int i = 0; i < n; i++) {
            res->samples[i].seq = s[i].seq;
            res->samples[i].t_ms = s[i].t_ms;
            for (int c = 0; c < SMC_PLC_TRACE_CH_MAX; c++)
                res->samples[i].v[c] = s[i].v[c];
        }
        res->ret_code = 0;
        res->n_included = n;
        res->next_offset = (req->offset + n < n_total) ? (req->offset + n) : -1;
        resp_payload_len = 16 + (uint32_t)n * sizeof(SmcPlcTraceSample_t);
        break;
    }
    case SMC_CMD_PLC_UPLOAD: {
        if (req_hdr.data_len < sizeof(SmcPlcUploadReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcUploadReq *req = (SmcPlcUploadReq *)payload;
        SmcPlcUploadRes *res = (SmcPlcUploadRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = 16;          /* 无 chunk 时仅头 */
        const char *text = NULL;
        int tlen = 0;
        if (plc_core_get_text(&text, &tlen) != 0) {
            res->ret_code = -1;
            res->next_offset = -1;
            break;
        }
        if (req->offset < 0 || req->offset >= tlen) {
            res->ret_code = -2;
            res->total_len = tlen;
            res->next_offset = -1;
            break;
        }
        int n = tlen - req->offset;
        if (n > SMC_PLC_UPLOAD_CHUNK) n = SMC_PLC_UPLOAD_CHUNK;
        memcpy(res->chunk, text + req->offset, (size_t)n);
        res->ret_code = 0;
        res->total_len = tlen;
        res->n_included = n;
        res->next_offset = (req->offset + n < tlen) ? (req->offset + n) : -1;
        resp_payload_len = 16 + (uint32_t)n;
        break;
    }
    case SMC_CMD_PLC_READ_VARS: {
        if (req_hdr.data_len < sizeof(SmcPlcReadVarsReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcPlcReadVarsReq *req = (SmcPlcReadVarsReq *)payload;
        SmcPlcReadVarsRes *res = (SmcPlcReadVarsRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcPlcReadVarsRes);
        int n_total = 0;
        PlcVarSnap_t all[PLC_USER_VAR_MAX + 4];
        int n = plc_core_snapshot_vars(all, PLC_USER_VAR_MAX + 4, &n_total);
        res->n_total = n_total;
        if (req->offset < 0 || req->offset >= n_total) {
            res->ret_code = -1;      /* offset 越界 */
            res->next_offset = -1;
            break;
        }
        int avail = n - req->offset;  /* snapshot 拷贝上限内 */
        if (avail > SMC_PLC_VARS_PER_PAGE) avail = SMC_PLC_VARS_PER_PAGE;
        if (avail < 0) avail = 0;
        for (int i = 0; i < avail; i++) {
            PlcVarSnap_t *s = &all[req->offset + i];
            strncpy(res->vars[i].name, s->name, 31);
            res->vars[i].name[31] = '\0';
            res->vars[i].type = s->type;
            res->vars[i].cls = s->cls;
            res->vars[i].fb = s->fb;
            res->vars[i].forced = s->forced;   /* P1.5-2: 暴露强制标志, watch 可见 */
            res->vars[i].value = s->value;
        }
        res->ret_code = 0;
        res->n_included = avail;
        res->next_offset = (req->offset + avail < n_total) ? (req->offset + avail) : -1;
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

    /* ===== 断点续切 Phase 1 (2026-09-08): ResumeSearch / ResumeStart / GetResume ===== */
    case SMC_CMD_RESUME_SEARCH: {
        if (req_hdr.data_len < sizeof(SmcResumeSearchReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcResumeSearchReq *req = (SmcResumeSearchReq *)payload;
        printf("[rpc] ResumeSearch: target_line=%d\n", (int)req->target_line);
        SmcResumeSearchRes *res = (SmcResumeSearchRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = SMC_ResumeSearch(req->target_line);
        res->target_line = g_parser_ctrl.resume_target_line;
        memcpy(res->target_pos, g_parser_ctrl.resume_target_pos,
               sizeof(double) * AXIS_NUM);
        printf("[rpc] ResumeSearch: 返回码=%d (0=冻结, -3=非G00边界, -5=行不存在)\n",
               res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcResumeSearchRes);
        break;
    }
    case SMC_CMD_RESUME_START: {
        printf("[rpc] ResumeStart\n");
        SmcResumeStartRes *res = (SmcResumeStartRes *)resp_buf;
        res->ret_code = SMC_ResumeStart();
        printf("[rpc] ResumeStart: 返回码=%d\n", res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcResumeStartRes);
        break;
    }
    case SMC_CMD_GET_RESUME: {
        /* 无 Req, Res = SmcGetResumeRes */
        SmcGetResumeRes *res = (SmcGetResumeRes *)resp_buf;
        SMC_GetResumeState(res);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcGetResumeRes);
        break;
    }

    /* ===== 论文1 IV-E (2026-09-16): G93 jerk-bounded 转移剖面 0x0082 ===== */
    case SMC_CMD_CONFIG_G93_JERK: {
        if (req_hdr.data_len < sizeof(SmcConfigG93JerkReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93JerkReq *req = (SmcConfigG93JerkReq *)payload;
        printf("[rpc] ConfigG93Jerk: enable=%d eps_v=%.1f%% eps_t=%.1f%%\n",
               req->enable, req->eps_v_pct, req->eps_t_pct);
        SmcConfigG93JerkRes *res = (SmcConfigG93JerkRes *)resp_buf;
        res->ret_code = SMC_ConfigG93Jerk(req->enable, req->eps_v_pct, req->eps_t_pct);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93JerkRes);
        break;
    }

    /* ===== 论文1 B0′ 消融 (2026-09-17): F-inheritance 回退开关 0x0083 ===== */
    case SMC_CMD_CONFIG_G93_ABLATION: {
        if (req_hdr.data_len < sizeof(SmcConfigG93AblationReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93AblationReq *req = (SmcConfigG93AblationReq *)payload;
        printf("[rpc] ConfigG93Ablation: f_inherit=%d\n", req->f_inherit);
        SmcConfigG93AblationRes *res = (SmcConfigG93AblationRes *)resp_buf;
        res->ret_code = SMC_ConfigG93Ablation(req->f_inherit);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93AblationRes);
        break;
    }

    /* ===== 论文1 M1 替代设计: 事后修正开关 0x0085 (2026-09-18) ===== */
    case SMC_CMD_CONFIG_G93_POSTHOC: {
        if (req_hdr.data_len < sizeof(SmcConfigG93PosthocReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93PosthocReq *req = (SmcConfigG93PosthocReq *)payload;
        printf("[rpc] ConfigG93Posthoc: enable=%d\n", req->enable);
        SmcConfigG93PosthocRes *res = (SmcConfigG93PosthocRes *)resp_buf;
        res->ret_code = SMC_ConfigG93Posthoc(req->enable);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93PosthocRes);
        break;
    }

    case SMC_CMD_CONFIG_G93_RATIO: {
        if (req_hdr.data_len < sizeof(SmcConfigG93RatioReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93RatioReq *req = (SmcConfigG93RatioReq *)payload;
        printf("[rpc] ConfigG93Ratio: ratio_off=%d\n", req->ratio_off);
        SmcConfigG93RatioRes *res = (SmcConfigG93RatioRes *)resp_buf;
        res->ret_code = SMC_ConfigG93Ratio(req->ratio_off);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93RatioRes);
        break;
    }

    case SMC_CMD_CONFIG_G93_STRICT_OFF: {
        if (req_hdr.data_len < sizeof(SmcConfigG93StrictOffReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93StrictOffReq *req = (SmcConfigG93StrictOffReq *)payload;
        printf("[rpc] ConfigG93StrictOff: strict_off=%d\n", req->strict_off);
        SmcConfigG93StrictOffRes *res = (SmcConfigG93StrictOffRes *)resp_buf;
        res->ret_code = SMC_ConfigG93StrictOff(req->strict_off);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93StrictOffRes);
        break;
    }

    /* ===== 论文1 S10-H1 长度基消融开关 0x0088 (2026-09-20) ===== */
    case SMC_CMD_CONFIG_G93_TIP_BASIS: {
        if (req_hdr.data_len < sizeof(SmcConfigG93TipBasisReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcConfigG93TipBasisReq *req = (SmcConfigG93TipBasisReq *)payload;
        printf("[rpc] ConfigG93TipBasis: tip_basis=%d\n", req->tip_basis);
        SmcConfigG93TipBasisRes *res = (SmcConfigG93TipBasisRes *)resp_buf;
        res->ret_code = SMC_ConfigG93TipBasis(req->tip_basis);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcConfigG93TipBasisRes);
        break;
    }

    /* ===== 论文1 S10 (2026-09-20): 控制器内 block 级计时查询 0x0089 ===== */
    case SMC_CMD_QUERY_BLOCK_TIMING: {
        uint8_t bt_op = 0;
        if (req_hdr.data_len >= sizeof(SmcBlockTimingReq)) {
            bt_op = ((SmcBlockTimingReq *)payload)->op;
        }
        if (bt_op == 1) {
            bt_reset_all();       /* 清零 (宿主在每次 run 前显式调用) */
        }
        SmcBlockTimingRes *res = (SmcBlockTimingRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = 0;
        res->block_uid                 = g_bt.block_uid;
        res->block_line_no             = g_bt.block_line_no;
        res->block_start_cycle         = g_bt.block_start_cycle;
        res->block_end_cycle_exclusive = g_bt.block_end_cycle_exclusive;
        res->block_elapsed_cycles      = g_bt.block_elapsed_cycles;
        res->block_stall_cycles        = g_bt.block_stall_cycles;
        res->segments_generated        = g_bt.segments_generated;
        res->segments_completed        = g_bt.segments_completed;
        res->block_budget_cmd_ms       = g_bt.block_budget_cmd_ms;
        res->block_budget_sum_ms       = g_bt.block_budget_sum_ms;
        res->block_virtual_ms          = g_bt.block_virtual_ms;
        res->ctrl_start_cycle          = g_bt.ctrl_start_cycle;
        res->ctrl_end_cycle_exclusive  = g_bt.ctrl_end_cycle_exclusive;
        res->controller_elapsed_cycles = g_bt.controller_elapsed_cycles;
        res->controller_elapsed_ms     = g_bt.controller_elapsed_ms;
        res->virtual_elapsed_ms        = g_bt.virtual_elapsed_ms;
        res->hold_active_cycles        = g_bt.hold_active_cycles;
        res->hold_active_ms            = g_bt.hold_active_ms;
        res->hold_frozen_cycles        = g_bt.hold_frozen_cycles;
        res->override_eff_sum          = g_bt.override_eff_sum;
        res->override_eff_cycles       = g_bt.override_eff_cycles;
        res->virtual_model_ms          = g_bt.virtual_model_ms;
        res->segments_completed_total  = g_bt.segments_completed_total;
        res->blocks_closed             = g_bt.blocks_closed;
        res->rt_cycle_counter          = g_bt.rt_cycle_counter;
        /* V13.1 H3 (2026-09-22): planner 最终目标 (T_plan), 独立于 T_sem */
        res->block_plan_sum_ms         = g_bt.block_plan_sum_ms;
        res->plan_segments_loaded      = g_bt.plan_segments_loaded;
        /* V13.1.1 (2026-09-24) 修复 A: 块级时钟配对量 (与 block_virtual_ms 同窗) */
        res->block_override_eff_sum    = g_bt.block_override_eff_sum;
        res->block_override_eff_cycles = g_bt.block_override_eff_cycles;
        res->block_virtual_model_ms    = g_bt.block_virtual_model_ms;
        printf("[rpc] BlockTiming(op=%u): uid=%u line=%d elapsed=%llu cyc "
               "(gen=%u done=%u, cmd=%.3fms sum=%.3fms) | ctrl=%llu cyc, "
               "virt=%.3fms, hold=%llu cyc, plan=%.3fms/%u | blk_eff=%.1f/%llu "
               "blk_vmodel=%.3fms\n",
               (unsigned)bt_op, res->block_uid, res->block_line_no,
               (unsigned long long)res->block_elapsed_cycles,
               res->segments_generated, res->segments_completed,
               res->block_budget_cmd_ms, res->block_budget_sum_ms,
               (unsigned long long)res->controller_elapsed_cycles,
               res->virtual_elapsed_ms,
               (unsigned long long)res->hold_active_cycles,
               res->block_plan_sum_ms, res->plan_segments_loaded,
               res->block_override_eff_sum,
               (unsigned long long)res->block_override_eff_cycles,
               res->block_virtual_model_ms);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcBlockTimingRes);
        break;
    }

    /* ===== V13.1 (2026-09-22): G93 语义变换事件查询 0x008A ===== */
    case SMC_CMD_QUERY_G93_SEMANTIC_EVENTS: {
        uint8_t  sem_op = 0;
        uint32_t start_seq = 1;
        uint32_t max_count = SMC_G93_SEM_EVENT_MAX_PER_RSP;
        if (req_hdr.data_len >= sizeof(SmcG93SemEventsReq)) {
            SmcG93SemEventsReq *sem_req = (SmcG93SemEventsReq *)payload;
            sem_op     = sem_req->op;
            start_seq  = sem_req->start_seq;
            max_count  = sem_req->max_count;
        }
        if (max_count == 0 || max_count > SMC_G93_SEM_EVENT_MAX_PER_RSP) {
            max_count = SMC_G93_SEM_EVENT_MAX_PER_RSP;
        }
        if (sem_op == 1) {
            CutterComp_G93SemReset();   /* 宿主在每次 run 前显式调用 */
        }
        SmcG93SemEventsRes *res = (SmcG93SemEventsRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code       = 0;
        res->total_count    = CutterComp_G93SemTotal();
        res->dropped_events = CutterComp_G93SemDropped();
        res->returned_count = (uint32_t)CutterComp_G93SemCopy(start_seq, max_count,
                                                              res->events);
        printf("[rpc] G93SemEvents(op=%u): start=%lu total=%lu dropped=%lu "
               "returned=%lu\n",
               (unsigned)sem_op, (unsigned long)start_seq,
               (unsigned long)res->total_count,
               (unsigned long)res->dropped_events,
               (unsigned long)res->returned_count);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcG93SemEventsRes);
        break;
    }

    /* ===== 论文1 N22 (2026-09-17): enqueue 临界区耗时统计 0x0084 ===== */
    case SMC_CMD_QUERY_ENQUEUE_STATS: {
        uint8_t stats_reset = 0;
        if (req_hdr.data_len >= sizeof(SmcEnqueueStatsReq)) {
            stats_reset = ((SmcEnqueueStatsReq *)payload)->reset;
        }
        SmcEnqueueStatsRes *res = (SmcEnqueueStatsRes *)resp_buf;
        memset(res, 0, sizeof(*res));
        res->ret_code = 0;
        SMC_GetEnqueueStats(stats_reset,
                            &res->count_all, &res->count_strict,
                            &res->mean_us_all, &res->mean_us_strict,
                            &res->max_us_strict);
        printf("[rpc] EnqueueStats: all=%lu (%.2f μs) strict=%lu (%.2f μs, max %.2f)\n",
               (unsigned long)res->count_all, res->mean_us_all,
               (unsigned long)res->count_strict, res->mean_us_strict,
               res->max_us_strict);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcEnqueueStatsRes);
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
    /* ===== v1.5 (2026-08-28): 当前位设为激活 WCS 零点 ===== */
    case SMC_CMD_SET_ORIGIN_HERE: {
        if (req_hdr.data_len < sizeof(SmcSetOriginHereReq)) {
            res_hdr.err_code = SMC_ERR_PARAM; break;
        }
        SmcSetOriginHereReq *req = (SmcSetOriginHereReq *)payload;
        SmcSetOriginHereRes *res = (SmcSetOriginHereRes *)resp_buf;
        char axis = (req->z_letter != 0) ? (char)req->z_letter : (char)SMC_AXIS_ALL;
        res->ret_code = SMC_SetOriginHere(axis);
        printf("[rpc] SetOriginHere %s ret=%d\n",
               (req->z_letter != 0) ? "单轴" : "全轴", res->ret_code);
        res_hdr.err_code = SMC_OK;
        resp_payload_len = sizeof(SmcSetOriginHereRes);
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

    /* P0-1: 外围 G/F 信号表初始化 (RT 线程启动前, 同上约束) */
    PeriphBus_Init();

    /* P1-b: 事件/报警流推送中心初始化 (须早于 plc_core_init — autoload 的
     * 编译失败事件 0x0001 否则会在 EventLogger_Init 清环时丢失) */
    EventLogger_Init();

    /* P1: 软 PLC 程序槽初始化 (boot autoload ./plc_prog.st; 须晚于
     * EventLogger_Init — autoload 编译失败事件 0x0001 否则被清环丢失) */
    periph_ai_init();       /* P1.5-1: 外围 AI 通道 (RT 启动前就绪) */
    plc_core_init();

    /* P0-b v1: 段流推送中心初始化 (parser 启动前必须就绪, 否则首次 Push 写未初始化 ring) */
    PreviewStreamer_Init();

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
    /* P0 (2026-09-02): 旋转轴软限位 — 与 main.c / param_store.c ps_default_profile
     * 三处同步, 值以 param_store.c 为单一权威 (C±730°/B±110°) */
    SMC_ConfigSoftLimit('C', 1, -730.0, 730.0);
    SMC_ConfigSoftLimit('B', 1, -110.0, 110.0);

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
    PeriphBus_Init();       /* P0-1: 外围 G/F 信号表 (RT 启动前就绪) */
    EventLogger_Init();     /* P1-b: 事件中心 (须早于 plc_core_init, 同 kernel_init) */
    periph_ai_init();       /* P1.5-1: 外围 AI 通道 */
    plc_core_init();        /* P1: 软 PLC 程序槽 + autoload */
    PreviewStreamer_Init();

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

    /* 6.5 P0: RTCP 奇异点阈值 (0 = 显式禁用; 哨兵未填跳过 → 编译默认 30.0) */
    if (hw->singularity_rmax != PS_TODO_D)
        SMC_ConfigSingularityRmax(hw->singularity_rmax);

    /* 6.6 P1 (2026-09-04): 双摆头链序 (置于 SMC_ConfigKinematics 之后 —
     *    后者不写 chain_order, 但调用顺序即规范)。0=已交付默认 no-op。 */
    SMC_ConfigKinematicsChain(hw->chain_order);

    /* 7. C 级规划器 (档案值; 回退默认 0.05/500) */
    if (hw->corner_tolerance > 0.0 && hw->max_centripetal_acc > 0.0)
        SMC_ConfigPlannerParams(hw->corner_tolerance, hw->max_centripetal_acc);

    /* 8. A3 软限位 (enable=1 的轴) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].sl_enable)
            SMC_ConfigSoftLimit(hw->axis_order[i][0], 1,
                                hw->ax[i].sl_neg, hw->ax[i].sl_pos);

    /* 8.5 P0: 旋转轴 G90 最短路径解缠绕 (ConfigAxisDynamics 已默认开,
     *        此处应用档案显式值 — ini rot_wrap_enable=0 关闭) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].dyn_type == 1)
            SMC_ConfigRotWrapEnable(hw->axis_order[i][0], hw->ax[i].rot_wrap_enable);

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

    /* 11. B 级回零 (v1.5 m35 = 回 WCS 零点运动; 顺序/速度/超时/auto_on_init 来自档案,
     *     hm_speed 0 = 轴 max_speed; auto_on_init=0 手动触发, 失败要人看见。
     *     硬件 home switch 接入后切 1-19) */
    for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++)
        if (hw->ax[i].hm_enable)
            SMC_ConfigHoming(hw->axis_order[i][0], hw->ax[i].hm_method,
                             hw->ax[i].hm_speed, 0, hw->ax[i].hm_direction,
                             hw->ax[i].hm_timeout_ms);
    if (hw->homing_order[0] != '\0')
        SMC_ConfigHomingAllEx(hw->homing_order, hw->auto_on_init);

    /* 11.5 P0-2 (2026-09-09): 激光互锁规则表 (软 PLC 地基, 数据驱动互锁)
     *     count=0 跳过 = 内核默认 6 路链 (laser_ctrl.c bit_di_* 固定映射);
     *     越界已被 param_store 解析层拒绝, 此处 min() 双保险 */
    if (hw->laser_il_count > 0) {
        LaserIlRule_t rules[LASER_IL_MAX];
        int n = (hw->laser_il_count < LASER_IL_MAX) ? hw->laser_il_count : LASER_IL_MAX;
        for (int i = 0; i < n; i++) {
            rules[i].bit      = (uint8_t)hw->laser_il_bit[i];
            rules[i].reserved = 0;
            rules[i].mask     = (uint16_t)hw->laser_il_mask[i];
        }
        SMC_ConfigLaserInterlock(rules, n);
    }

    /* 11.6 P0-4a (2026-09-10): 激光 I/O 端子 PDO 布局 (选型适配骨架)
     *     默认 EL 级 16-bit; 换端子 = 改档案 [laser_io], 内核代码不动 */
    SMC_ConfigLaserPdoLayout((uint8_t)hw->laser_io_di_off, (uint8_t)hw->laser_io_di_bytes,
                             (uint8_t)hw->laser_io_do_off, (uint8_t)hw->laser_io_do_bytes,
                             (uint8_t)hw->laser_io_ao_off, (uint8_t)hw->laser_io_ao_stride,
                             NULL);

    /* 11.6b P1.5-1 (2026-09-14): 外围 AI 通道 (默认未配置, count=0 跳过) */
    if (hw->periph_ai_count > 0 && hw->periph_ai_slave >= 0) {
        SMC_ConfigPeriphAi(hw->periph_ai_slave, hw->periph_ai_off,
                           hw->periph_ai_stride, hw->periph_ai_count,
                           hw->periph_ai_fs);
    }

    /* 11.7 P0-4a: 期望拓扑表 (空表 = 不校验; 配置后 bring-up 失配拒绝进 OP,
     *     防 slave_id 按位置漂移)。必须在 SMC_InitAndStart (ecat_bringup) 之前 */
    if (hw->ecat_topo_count > 0) {
        EcatTopoExpect_t topo[ECAT_TOPO_MAX];
        int n = (hw->ecat_topo_count < ECAT_TOPO_MAX) ? hw->ecat_topo_count : ECAT_TOPO_MAX;
        for (int i = 0; i < n; i++) {
            topo[i].vendor  = hw->ecat_topo_vendor[i];
            topo[i].product = hw->ecat_topo_product[i];
        }
        ecat_set_expected_topo(topo, n);
    }

    /* 11.8 Phase 1 (2026-11-23): 机械轴限位输入来源 ([limit_src] → runtime 表)
     *     供 RPC 0x008C 诊断查询; 换站 = 改档案重启进程, 代码零改动。
     *     本阶段诊断-only: 不接 RT/停机/报警, 真实 PDO 采样属后续阶段。
     *     kernel_init (非 hw 路径) 不 apply → 表保持零初始化 = 全 UNCONFIGURED */
    {
        LimitSrcCfg_t lmt[AXIS_NUM][2];
        memset(lmt, 0, sizeof(lmt));
        for (int i = 0; i < hw->axis_count && i < AXIS_NUM; i++) {
            for (int d = 0; d < 2; d++) {
                if (!hw->limit_src_cfg[i][d]) continue;   /* 解析后 ∈ {0, 0x7} */
                lmt[i][d].configured  = 1;
                lmt[i][d].slave       = hw->limit_src_slave[i][d];
                lmt[i][d].input       = hw->limit_src_input[i][d];
                lmt[i][d].active_high = hw->limit_src_level[i][d];
            }
        }
        limit_map_set_runtime((const LimitSrcCfg_t (*)[AXIS_NUM][2])lmt);
    }

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
