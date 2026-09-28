# L1.5 环境引导 — stock LinuxCNC 最小 adapter 演示

目标: 在未修改源码的 LinuxCNC (stock) 上运行 `g93_probe.ngc`,采集 10 通道
HAL 见证,用 `adapter_l15.py` 产出 M_PRED↔M_obs 对照 (预测已冻结:
`docs/paper1_evidence_interface/M_PRED_linuxcnc_v1.md`,commit `2bfa94e`)。

**时间盒 ≤ 2 周;超时降级为 L1 静态审计投稿。成功标准 = monitorability 按预测
迁移,不是所有契约可判;发现真实 LinuxCNC 偏离属彩票。**

## 1. 环境 (主导风险,先做)

当前 WSL2 是 Ubuntu 24.04,LinuxCNC 官方 uspace deb 只发 Debian:

- **推荐**: 另装 Debian 发行版
  `wsl --install -d Debian` (bookworm),然后按 linuxcnc.org wiki
  "LinuxCNC Debian Packages" 配置 apt 仓库,安装 `linuxcnc-uspace`。
  WSLg (Win11 自带) 提供 axis GUI 所需的 X 环境。
- **备选**: 用 `/mnt/d/code/linuxcnc` (2.9 源码) 编译 rip 环境
  (仓库 docs/src/code/ 下有构建文档;依赖多、耗时长,仅推荐路径失败时用)。
- 安装后记录进 manifest: `linuxcnc --version` 输出 + 与审计基线
  `2.9@fe72abb` 的差异说明。

## 2. 文件清单

| 文件 | 作用 |
|---|---|
| `g93_probe.ngc` | 6 块 G93 探针 (G61 禁融合;F60~120;总运动 ~7.5s;M53 P1) |
| `g93_probe.ini` | mm 单位 sim 配置 (基于官方 configs/sim/axis/axis.ini 改制) |
| `g93_probe.hal` | sampler 10 通道 @ servo-thread 1kHz (`halsampler -t` 导出) |
| `run_l15.sh` | 双终端 runbook: timing (override/feedhold 干预) / capture (导出+分析) |
| `adapter_l15.py` | 证据 adapter: P1-P6 + H1 + M_obs (容差已冻结在文件头) |

INI 依赖发行版自带 HALFILE (`core_sim.hal` / `simulated_home.hal` /
`axis_manualtoolchange.hal`),从 configs/sim/axis/ 目录运行最稳:
`cp /usr/share/linuxcnc/configs/sim/axis/{core_sim.hal,simulated_home.hal,axis_manualtoolchange.hal} .`

## 3. 首接触 checklist (逐项确认后才算环境就绪)

1. `linuxcnc g93_probe.ini` 能启动 axis GUI (WSLg);
2. Home All 成功;Auto 载入 g93_probe.ngc 无解释器错误 (G93 每块 F ✓);
3. `halcmd show pin | grep -E "feed-upm|requested-vel|program-line"` 三引脚存在;
4. `halcmd show pin sampler.0` 10 pin 就绪;`sampler.0.enable` 可 setp;
5. halsampler 导出格式 = 首列 sample 序号 + 10 数值列 (与 adapter COLS 对齐,
   若 -t 行为不符→改 adapter 前先记录,属 obs 侧文档化差异);
6. halui direct-value 语义复核: setp counts=50 后
   `halcmd show pin halui.feed-override.value` 应≈0.5 (若语义为千分数则 scale 改 0.001);
7. FIFO 不溢出: 采集期 <16s,`sampler.0.overruns`==0;
8. 若 motion.feed-hold 被 core_sim.hal 已连接 (setp 报错) → 改用 halui 的
   feed-hold 等价引脚或 setp 前先 unlink (首接触时按实际 HAL 拓扑调整并记录)。

## 4. 冻结纪律

- **首跑前**: `git add scripts/linuxcnc_l15 && git commit` —— adapter/程序/配置
  作为预测侧组件冻结 (M_PRED 本体已冻结);
- **首跑后**: 每个 run 的 capture CSV + l15_report_* 归档;
  pred↔obs 差异如实报告,不回改冻结件 (勘误追加制);
- 重复 3 次有效 run (r1-r3) 即可,不需要更多。

## 5. 已知设计边界 (审计结论,不属环境问题)

- H2/H3 在 stock 接口层无证据 → UNOBSERVABLE 是**预期正确结果**;
- feed-hold 引脚 (IN) 不能直接采进 sampler → P6 用 fu==0 特征窗口判定;
- 零长度 G93 块未测试 (P7 回避;rate=0.1 corner 见 issue #2410)。
