# M_PRED — LinuxCNC stock 监测性预注册 profile (v1)

| 项 | 内容 |
|---|---|
| 状态 | **待冻结** — 由用户以 git commit 时间戳冻结;冻结后本文件不得修改,勘误只许追加并注明日期 |
| 冻结依据 | `L1b_linuxcnc_audit.md` v0.2 (2026-09-28,文档级+源码级审计) |
| 预测对象 | LinuxCNC stock (未修改源码) + 文档化外部接口,2.9 @ `fe72abb207f388eabaaea211fe28cc6b8d2a80bd` |
| 验证方式 | L1.5:linuxcnc-uspace sim 配置 + G93 测试程序 + `sampler` 采集 + 最小 adapter;观测 profile $\mathbf M^{obs}$ 与本文件逐项对照 |
| 对照口径 | pred/obs 一致 = 判定类别相同 (OBSERVABLE/UNOBSERVABLE/PARTIAL/N-A);PARTIAL 的内部细分差异记为一致但列注 |

## 1. 预注册 profile

$$
\mathbf M^{pred}_{LinuxCNC,stock} = [\text{H1: OBSERVABLE},\;\text{H2: UNOBSERVABLE*},\;\text{H3: UNOBSERVABLE},\;\text{H4: PARTIAL},\;\text{H5: PARTIAL}]
$$

\* H2 备注:简单路径 (无重离散边界) 上预期责任前提不成立,运行时判词应为 **N/A**;"UNOBSERVABLE" 预测的是"文档化接口不提供预算转移见证"这一能力边界。L1.5 报告时两者分开记录。

## 2. 可检验运行时预测

以下预测在冻结时刻做出,L1.5 实测逐条对照 (PASS/FAIL 各条独立计):

| # | 预测 | 依据 |
|---|---|---|
| P1 | G93 单块内 `feed-upm` ≈ (L×F) × net_feed_scale,块内基本恒定;换块随 L、F 变化 | interp_inverse.cc 换算 + control.c:1876 |
| P2 | `requested-vel` ≈ L×F (units/s 口径换算),**不随 feed override 变化** | tc_types.h:127 + tp.c:2629 + tc.c:39 |
| P3 | 50% override 下:`feed-upm` 与 `current-vel` 减半,`requested-vel` 不变 | 同上 + net_feed_scale 乘法链 |
| P4 | `motion.program-line` 与目标块一一对应 (不超前于执行) | tc->id 链 (control.c:1942 / tp.c:2628) |
| P5 | H2/H3 无原生证据可用,adapter 输出 UNOBSERVABLE (不借用输出轨迹重建) | 审计 §3 |
| P6 | feedhold (M53 P1 使能后) 期间 `feed-upm` = 0,恢复后回到 P1 关系 | man page + control.c 门控 |
| P7 (边界,可选) | 零长度 G93 块:rate = 0.1 的既定 corner (issue #2410);测试程序缺省避开 | interp_inverse.cc:66,127 |

## 3. 判定规则 (预注册)

1. H1 判 OBSERVABLE 的条件:requested-vel + motor-pos-cmd + INI 限值三方齐备且采样有效。
2. H4/H5 判 PARTIAL 的条件:速度形态/调节语义见证可用,但时间形态目标语义 (T_policy/T_sem) 无 native witness,需形态转换或重建的步骤在 adapter 中显式标注。
3. L1.5 成功标准 = **monitorability 类别按预测迁移**,不是"全部契约可判";发现真实 LinuxCNC 偏离属额外结果,不改变成功判定。
4. 若 P1-P6 中任何一条 FAIL,如实报告并分析 (预测错误本身是审计信息,不回改本文件)。
