/* =====================================================================
 *  main.cpp  ——  CAM 团队使用 SmcController SDK 的示例
 *
 *  部署拓扑: CAM/HMI 在 Windows 宿主机, CNC Core 在 WSL2 (Linux)。
 *    - 网络层: Connect("127.0.0.1", 9527) 借助 WSL2 默认的
 *              localhostForwarding 直达 WSL2 内的 rpc_server。
 *    - 文件层: NC 文件可使用 Windows 原生路径 (C:\\... / D:/...),
 *              SDK 的 TranslatePathForWSL 自动转为 /mnt/<drive>/...。
 *
 *  本示例覆盖五大功能分类的典型调用, 让 CAM 开发人员快速上手:
 *    1) 连接管理     —— Connect / Disconnect
 *    2) 状态查询     —— GetLogicalPos / IsParserRunning / GetSystemStatus ...
 *    3) 轴配置       —— ConfigAxisTopology / ConfigSoftLimit / ConfigKinematics ...
 *    4) 运动控制     —— SetZero / MoveRelative / GoZero
 *    5) G 代码加工   —— RunGCodeFile / Pause / Resume / Abort
 *
 *  编译 (Windows, 推荐):
 *      cl /EHsc /std:c++14 /I. main.cpp SmcControllerSdk.cpp ws2_32.lib
 *  编译 (Linux, 同机调试):
 *      g++ -std=c++14 -O2 -I. main.cpp SmcControllerSdk.cpp -o cam_demo
 *
 *  运行前请确认 WSL2 内已启动 rpc_server 监听 9527。
 * ===================================================================== */

#include "SmcControllerSdk.h"

#include <cstdio>
#include <string>

/* 把"调用 + 错误打印"压缩成一句, 让示例主体保持干净 */
#define CHECK(call, label)                                              \
    do {                                                                \
        if (!(call)) {                                                  \
            std::printf("[cam] %s 失败, err=%d\n",                      \
                        label, cnc.LastError());                        \
        } else {                                                        \
            std::printf("[cam] %-22s OK\n", label);                     \
        }                                                               \
    } while (0)

int main()
{
    SmcController cnc;

    /* ===== 1. 建立连接 ===== */
    if (!cnc.Connect("127.0.0.1", 9527)) {
        std::printf("[cam] 连接 CNC Core 失败, err=%d\n", cnc.LastError());
        return 1;
    }
    std::printf("[cam] 已连接 CNC Core RPC 服务\n\n");

    /* ===== 2. 状态查询 (CAM 最常用) ===== */
    int         running = 0, done = 0, queue = 0, configured = 0;
    double      pos_x   = 0.0;
    std::string status;

    CHECK(cnc.IsParserRunning(running),    "IsParserRunning");
    CHECK(cnc.IsMotionDone(done),          "IsMotionDone");
    CHECK(cnc.GetQueueCount(queue),        "GetQueueCount");
    CHECK(cnc.IsAxisConfigured('X', configured), "IsAxisConfigured(X)");
    CHECK(cnc.GetLogicalPos('X', pos_x),   "GetLogicalPos(X)");
    CHECK(cnc.GetSystemStatus(status),     "GetSystemStatus");

    std::printf("\n[cam]  >> parser=%d  done=%d  queue=%d  X配置=%d\n",
                running, done, queue, configured);
    std::printf("[cam]  >> X 脉冲=%.0f  系统状态=%s\n\n",
                pos_x, status.c_str());

    /* ===== 3. 轴配置 (生产中通常只在 CNC 启动期调用一次) =====
     * 这里以 X 轴为例, 演示完整链路; 实际值需根据机床本体调整。*/
    int ret = -1;
    CHECK(cnc.ConfigAxisTopology("X轴", /*is_dual_drive=*/0, 0, 0, ret),
          "ConfigAxisTopology(X)");
    CHECK(cnc.ConfigSoftLimit('X', /*enable=*/1, -100.0, 100.0, ret),
          "ConfigSoftLimit(X,±100mm)");
    CHECK(cnc.ConfigPulsePerUnit('X', 10000.0),
          "ConfigPulsePerUnit(X,10000)");
    CHECK(cnc.ConfigAxisDynamics('X', /*type=*/0, 500.0, 1000.0, 1000.0, 0.0, ret),
          "ConfigAxisDynamics(X)");
    /* 五轴运动学: BC 双摆头, 刀长 150mm, 无 pivot 偏置 */
    CHECK(cnc.ConfigKinematicsOffset(/*tool_len=*/150.0, 0.0, 0.0, 0.0),
          "ConfigKinematicsOffset");
    CHECK(cnc.ConfigKinematics(/*type=*/0,
                               /*r1_idx=*/4, /*r1_axis=*/1,    /* B 轴绕 Y */
                               /*r2_idx=*/5, /*r2_axis=*/2,    /* C 轴绕 Z */
                               0.0, 0.0, 150.0,                /* tool_off */
                               0.0, 0.0, 0.0),                 /* pivot_off */
          "ConfigKinematics");
    std::printf("\n");

    /* ===== 4. 运动控制 ===== */
    CHECK(cnc.SetZero('X'),                       "SetZero(X)");
    CHECK(cnc.MoveRelative('X', 10.0, 500.0),     "MoveRelative(X,+10)");
    CHECK(cnc.GoZero('X', 500.0),                 "GoZero(X)");
    std::printf("\n");

    /* ===== 5. G 代码加工 =====
     * WSL2 联调典型用法: NC 文件存在 Windows 端 C 盘, SDK 内部
     * TranslatePathForWSL 自动转换为 /mnt/c/CNC_Programs/part1.ngc。
     * 同样支持 "D:/test/part.ngc" 或 UNC "\\wsl.localhost\Ubuntu\home\..."。*/
    std::string win_path = R"(C:\CNC_Programs\part1.ngc)";
    CHECK(cnc.RunGCodeFile(win_path, ret),        "RunGCodeFile");
    std::printf("[cam]  >> RunGCodeFile 返回 %d\n", ret);

    CHECK(cnc.PauseProcessing(),                  "PauseProcessing");
    CHECK(cnc.ResumeProcessing(),                 "ResumeProcessing");
    CHECK(cnc.AbortProcessing(),                  "AbortProcessing");
    std::printf("\n");

    /* ===== 收尾 =====
     * 注意: Close() 会通过 RPC 调用 SMC_Close 关闭整个 CNC 内核
     * (伺服下电、释放网卡), 通常由 CNC 服务自身在退出时调用,
     * 这里仅注释演示, 不实际触发:
     *
     *   int dummy = 0;
     *   cnc.InitAndStart("eth0", dummy);   // 反向重启 CNC
     *   cnc.Close();                       // 关闭 CNC (危险)
     */
    cnc.Disconnect();
    std::printf("[cam] 已断开 RPC 连接 (CNC 内核继续运行)\n");
    return 0;
}
