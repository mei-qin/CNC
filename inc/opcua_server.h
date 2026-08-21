#ifndef OPCUA_SERVER_H
#define OPCUA_SERVER_H

/* =====================================================================
 *  opcua_server.h  ——  OPC UA Server (对外标准通道, 契约 P4)
 *
 *  定位:
 *    把 CNC 状态/控制以 OPC UA 信息模型暴露给上位软件 movecontrol
 *    (open62541 client, opc.tcp://<host>:4840)。地址空间严格遵循
 *    movecontrol/doc/CNC_OPCUA_地址空间契约.md (两端 single source of
 *    truth), 方法回调直接调既有 SMC_* API, 不重写运动逻辑。
 *
 *  与既有四通道的关系 (混合架构):
 *    9527 RPC / 9528 snapshot / 9529 preview / 9530 event 保留为板内
 *    调试/降级通道; OPC UA 4840 作为对外标准通道。
 *
 *  线程模型:
 *    opcua_server_start() 创建单个 detached 后台线程 (模仿
 *    rpc_event_server_start 先例)。线程内 UA_Server_run_startup +
 *    循环 UA_Server_run_iterate(block) + 退出时 run_shutdown。
 *
 *  并发模型:
 *    变量节点全部用 DataSource 回调 — 读回调现拉 SnapshotHub (seqlock
 *    快照) / g_axis / g_current_line_no / EventLogger, 无缓存同步问题;
 *    monitored item 采样天然拿到最新值。写回调仅 3 个 override 节点,
 *    钳制后调 SMC_SetOverride (-1=不改 语义)。
 *
 *  RT 安全:
 *    本模块全 Non-RT, 不触碰 ecat_core / 插补 / PDO。
 * ===================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 OPC UA Server 后台线程 (幂等: 已在跑直接返回 0)
 * 返回: 0=已启动 (端口 4840, SecurityPolicy None, 匿名), -1=线程创建失败
 * 失败不 fatal — 调用方 (rpc_server main) 仅打印降级警告 */
int  opcua_server_start(void);

/* 停止后台线程 (进程退出路径用; run_iterate 阻塞中最多延迟一个周期) */
void opcua_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* OPCUA_SERVER_H */
