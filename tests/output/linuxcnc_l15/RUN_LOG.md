# L1.5 运行日志 — 首接触 2026-09-29 (Debian-L15 / linuxcnc 2.9.4-2+deb13u1)

环境: WSL 发行版 `Debian-L15`(trixie rootfs, images.linuxcontainers.org 20260928),
`linuxcnc-uspace 1:2.9.4-2+deb13u1`(Debian 官方仓库;buildbot 不可达已记录)。
驱动: linuxcncrsh 无头(DISPLAY=linuxcncrsh), 干预 = halui setp (override) + motion.feed-hold (M53 P1),
采集 = sampler 10ch @ servo 1kHz depth 65536, 导出 = halsampler stdout 重定向。
版本口径: 审计基线 2.9@fe72abb(≈2.9.7+), 实测 2.9.4 — 同系列差异记录在案。

## r1 (有效运行, 2026-09-29 ~01:00)

capture_r1.csv: 11214 行, SHA256 前缀 ce7a8b461d402523。程序 10.0s 完成 IDLE。

| 判定 | 结果 |
|---|---|
| P1 feed_upm==L×F | **FAIL — 仅 G2 圆弧块(line 16)**: fu_med=600.00 vs 预测 1884.96 (ratio 0.3183);5 个直线块全部精确 PASS(600.00/300.00/600.00/424.26/600.00) |
| P2 requested_vel==L×F/60 且不随 override | PASS(圆弧 31.4159 精确命中) |
| P3 override 缩放一致 | PASS |
| P4 program-line 1:1/单调/连续 | PASS |
| P5 H2/H3 UNOBSERVABLE | PASS(构造保证) |
| P6 hold 特征窗口 | PASS(检出 1.001s, 与干预时刻表 1.0s 吻合) |
| H1 参考硬限 | PASS(v_ref_max 31.36 < 200) |
| **M_obs** | **[H1:OBS, H2:UNO, H3:UNO, H4:PARTIAL, H5:PARTIAL] = M_PRED 完全一致** |

### 发现 F-1 (待源码调查, 不定性为 bug)

同一圆弧块上两个原生见证分叉: `requested-vel`(=tc->reqvel, task 换算)与 L×F/60 精确一致,
而 `feed-upm`(=tag.F×net_feed_scale)读数 600.00 —— 数值恰为直线块 B1/B3/B6 的 L×F(600)。
候选解释(未验证): 弧路径的 tag.F 序列化与 enqueue_SET_FEED_RATE 次序/来源存在路径差异
(interp_convert.cc 的 convert_arc* vs convert_straight 分支),或 tp 对 arc tc 的 tag 传播不同。
**处置**: 记为 L2 源码调查项(interp_inverse.cc 已读, convert_arc 分支未逐行核);
在定性前仅描述为"stock 2.9.4 上 arc 块两见证分歧的确定性观测(r1 单样本)"。

## r2/r3 (无效, 2026-09-29)

iocontrol 间歇性楔死(已知 WSL 弱点): task 卡 RUNNING、capture 全 idle、65536 行陈旧。
r2/r3 归档仅为完整性;不进入任何对照统计。

## 遗留与对策

- iocontrol 楔死为环境级问题(多见于 pkill 清场后的非首次 boot;r1 为清场后首 boot 成功)。
  对策: 每次运行前完整清场(root pkill 括号技巧 + /dev/shm/* + /tmp/linuxcnc.lock + ~/.rtapi_fifo),
  失败即整会话重来,不连续复用;r2/r3 重跑 = 一条命令(见 scripts/linuxcnc_l15/oneshot.sh)。
- halsampler 语法澄清(踩坑记录): FILENAME 为位置参数且选项须在前;`-t FILE` 会被当作
  "写文件 FILE",跨进程 stdout 重定向为可靠用法。
- git autocrlf 会给 /mntd 拷贝引入 CRLF,部署时统一 sed 去 CR(oneshot.sh 已含)。
