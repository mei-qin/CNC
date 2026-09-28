# L1-b — LinuxCNC stock 可观测性静态审计

| 项 | 内容 |
|---|---|
| 版本 | v0.2 (2026-09-28) — motion(9) 文档级 + **源码级审计完成** |
| 目的 | 支撑论文 §8.5/§9 "第二控制器 monitorability profile"——产出预注册 $\mathbf M^{pred}_{LinuxCNC}$,供 L1.5 stock 实测对照 (prediction-then-verify) |
| 审计对象 | LinuxCNC **stock** = 未修改源码 + 文档化外部接口 (HAL pins / NML status);不含任何 instrumentation patch |
| 源码基线 | `2.9` @ `fe72abb207f388eabaaea211fe28cc6b8d2a80bd` (2026-09-13, shallow clone `D:\code\linuxcnc`,GPLv2,只读参考) |
| 文档证据 | motion(9): `https://linuxcnc.org/docs/2.9/html/man/man9/motion.9.html` (2026-09-28 抓取);halui(1) + `halui.cc` 源码 |
| 证据等级 | **D** = 官方文档核验;**V** = 源码核验 (file:line);**—** = 文档化接口层不存在 |

## 1. 文档化外部接口清单 (与 G93 证据相关的子集)

### 1.1 运动侧 (MOTION/JOINT PINS)

| HAL pin | 语义 | 证据 (file:line @ 2.9) | 稳定性 |
|---|---|---|---|
| `joint.N.motor-pos-cmd` | 关节指令位置,servo 周期更新 | V: motion.c 导出;man D | 非 DEBUG,稳定 |
| `joint.N.pos-cmd` | 关节指令位置 (含螺距/反向隙补偿口径) | D | 非 DEBUG |
| `joint.N.vel-cmd` | 关节指令速度 | D | **(DEBUG)**,man 自述 "dubious",不作依赖 |
| `motion.current-vel` | 当前笛卡尔速度 (tp currentvel) | V: control.c:1947 | 非 DEBUG |
| `motion.requested-vel` | **任务侧按 F 算出的请求速度,前置 override** | V: tp.c:2629 `= tc->reqvel`;tc_types.h:127 注释 "vel requested by F word, calc'd by task";tc.c:39 显示 override 乘法 (`reqvel * max_scale`) 发生在 tp 内部 | 非 DEBUG |
| `motion.feed-upm` | **tag.F × net_feed_scale**;feedhold/inhibit 时 0 | V: control.c:1876-1877 `= tag.fields_float[FEED] * net_feed_scale` | 非 DEBUG |
| `motion.feed-hold` | feedhold 命令位 (**需 M53 P1 使能**) | D (man) | 非 DEBUG |
| `motion.adaptive-feed` | M52 自适应倍率,与 override 乘法叠加 | D (man) | 非 DEBUG |
| `motion.motion-type` | 0/1/2/3… (Idle/Traverse/Linear feed/Arc feed) | V: control.c:1944 | 非 DEBUG |
| `motion.program-line` | **= emcmotStatus->id = tp->execId = tc->id = 产生当前运动的命令行号** | V: control.c:1942;tp.c:2627-2628 (注释 "report our line number to the guis") | 非 DEBUG |
| `motion.distance-to-go` | 当前 move 剩余距离 | V: tp.c:2625 | 非 DEBUG |
| `motion.servo.last-period` / `motion-controller.time` | servo 周期 / 函数耗时 (CPU clocks) | D | 非 DEBUG |
| `halui.feed-override.value` | 当前 feed override 值 | V: halui.cc:728 | 用户态组件 |

采样通道:`sampler`+`halsampler` 或 rtapi ring——标准发行组件,非源码修改,归入 stock。

## 2. G93 语义链 (源码级核验,L1.5 的核心预测依据)

$$
F_{word}\ \xrightarrow[\text{interp\_inverse.cc:68,129}]{rate = L\times F}\ \texttt{settings->feed\_rate}
\ \xrightarrow[\text{interp\_write.cc:338}]{}\ \texttt{tag.F}
\ \xrightarrow[\text{control.c:1876}]{}\ \texttt{feed-upm} = \texttt{tag.F}\times\texttt{net\_feed\_scale}
$$

1. **解释器换算** (interp_inverse.cc:62-72 弧 / 107-135 直线):G93 (INVERSE_TIME) 模式下,每个 G1/G2/G3 块在 canon 运动命令发出**之前**调用 `inverse_time_rate_{straight,arc}`,计算 `rate = L × block->f_number` 并 `enqueue_SET_FEED_RATE(rate)` + **覆写** `settings->feed_rate`。L=0 时硬编码 `rate=0.1`(源码注释引用 GitHub issue #2410)。
2. **tag 序列化** (interp_write.cc:338):运动命令携带的 tag 其 F 字段 = `settings->feed_rate`——**G93 下即换算后每块速度 (units/min),不是原始逆时间 F 字**。man page 的 "F value" 措辞是 G94 视角,两模式在操作语义上反而一致:"本命令将施加的进给速度"。
3. **执行侧导出** (control.c):`feed-upm` = tag.F × net_feed_scale(后置 override);`requested-vel` = tc->reqvel(前置 override);override 乘法点在 tp 内部 (tc.c:39 `reqvel * max_scale`)。
4. **块身份**:`tc->id` ← 命令 id ← 解释器行号;`motion.program-line` 为**运动侧执行见证**。§5.3 所有权陷阱对块身份**不成立**(解释器超前不污染此 pin);仍留 L1.5 实测确认。

## 3. 逐契约审计:Req(H_i) vs Obs(stock LinuxCNC)

| 契约 | 判定 | 依据 |
|---|---|---|
| **H1** | **OBSERVABLE** | 请求侧:`requested-vel`(V:任务侧 F 换算速度)+ INI 轴限值;参考侧:`motor-pos-cmd` 轨迹 @ servo 周期 (非 DEBUG)。man 自述 requested-vel "possibly reduced to accommodate limits"——恰好是 admission 侧痕迹 |
| **H2** | **UNOBSERVABLE (documented interface)** | tp 的 tc 结构 (target/progress/reqvel) 全部内部,无预算转移见证。**补充**:interpreter→canon→tp 为 1 命令 1 tc 映射,G93 换算在 task 侧完成后下发——LinuxCNC 无本文管线的 n 元重离散边界,故简单路径上 H2 责任前提可能不成立 (**N/A 而非 UNOBSERVABLE**);"是否发生重离散"本身无运行时见证,由 L1.5 运行行为 + 源码结构联合判定 |
| **H3** | **UNOBSERVABLE (documented interface)** | 无 T_adm/T_ref/RevisionChain 任何对应物;不能用 tag.F + tc 内部量重建 (反自证) |
| **H4** | **PARTIAL** | 原生策略输出**存在于速度形态**:requested-vel (前置) / tag.F (命令上下文);G93 语义源码级确认 (§2)。缺口:契约实例化需速度→时间形态转换的 adapter 步骤,期望侧 L 必须来自程序几何 (声明侧) 而非输出轨迹,防自证 |
| **H5** | **PARTIAL** | 运行时调节语义完全文档化 (net_feed_scale 乘法链、feedhold 门控、M53 P1 前提);**无 T_sem native witness**(无语义时间概念)。块完成语义只能 output-derived 重建 (distance-to-go/current-vel 轨迹),仅作辅助,主判词缺省 UNOBSERVABLE |

## 4. 预注册 profile

见独立冻结文件 **`M_PRED_linuxcnc_v1.md`**(git commit 时间戳冻结;冻结后只许追加勘误)。

## 5. 关键发现 (论文素材)

1. `requested-vel` / `feed-upm` 构成天然的 pre/post-override 见证对,且 G93 语义源码级明确——"部分绑定"有实质内容。
2. man page 自述 DEBUG 类 pin "dubious"——审计只把非 DEBUG 引脚当稳定依赖,是 evidence interface 稳定性治理的直接素材。
3. `motion.program-line` 是运动侧执行见证 (tc->id),块身份所有权绑定合法。
4. L=0 → rate=0.1 硬编码 corner (issue #2410):LinuxCNC 对零长度 G93 块的既定处理,L1.5 测试程序应避开或显式预期。
5. H2 在第二控制器上可能落在 **N/A 而非 UNOBSERVABLE**——判词分类学在跨控制器场景的分辨力直接体现 (N/A=责任不发生 vs UNOBSERVABLE=责任发生但无见证)。
6. **F-1 (L1.5 实测+源码定性, 2026-09-29)**: G93 弧路径缺 `write_canon_state_tag`
   (convert_straight:4851 有, convert_arc2:602-606 无) → `motion.feed-upm` 在 G93 弧上
   报告上一块的陈旧换算速率 (3 次运行逐位复现: 600.00 vs 1884.96),文档语义失效;
   运动本体不受影响 (requested-vel/轨迹/时序全对)。**注:此发现修正 §1.1 中
   `feed-upm` 在弧上的语义可用性——作为 H4 见证时弧块不可信,直线块可信。**
   这是"输出验收不覆盖见证级语义"论点在第三方控制器上的实证。详见
   tests/output/linuxcnc_l15/RUN_LOG.md。

## 6. 状态

- [x] motion(9) 文档级审计
- [x] halui 引脚源码定位 (halui.cc:728)
- [x] 源码审计:G93 feed 语义 (interp_inverse.cc 全文 + tag 链)
- [x] 源码审计:program-line 归属 (control.c:1942 / tp.c:2628)
- [x] 源码审计:tp 无预算外显确认 (tc 结构内部,无导出)
- [x] 版本钉死:2.9 @ fe72abb
- [x] M_PRED 冻结: commit `2bfa94e` (2026-09-28;docs/ 整体 gitignore 为既有政策,仅此 3 文件 force-add 作时间戳锚)
- [ ] L1.5 实测对照 (Task #2)
