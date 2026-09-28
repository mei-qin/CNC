# L1-a — 证据接口观测插桩成本表 (V13.1)

| 项 | 内容 |
|---|---|
| 创建 | 2026-09-28 |
| 目的 | 支撑论文 §5.2/§8.2 "instrumentation burden is explicitly measurable when instrumentation access exists" — 把母稿两处"未实测插桩成本"的免责声明转为核账数字 |
| 核账基线 | 当前工作树 (`D:\code\CNC`);B3 正式 build = `0db73ab6`。git 提交混杂 (paper 与 OPC UA co-mingled),故用文件级核账而非 git diff |
| 范围声明 | **只计观测插桩**。0x0083-0x0088 为实验干预开关 (故障注入装置)、0x0084 enqueue-stats 为辅助观测,**均不计入** G93 证据接口成本 |

## 1. 证据角色 → 生产位点 → 成本

`Cost = (ΔLOC, N_probe, B_trace, ΔCPU, ΔT, ControlPathChange)`

| 证据角色 (对应 Req) | 生产位点 (当前工作树) | 线程 | ΔLOC (含注释) | N_probe | B_trace | ControlPathChange |
|---|---|---|---|---|---|---|
| T_in 父预算 + 块所有权声明 | `bt_block_open` @ `gcode_parser.c:388,494,2657,3346`;实现 `block_timing.c:22-32` | Non-RT | ~10 (调用点) | 4 | 经 0x0089 快照导出 | **No** — 只写 `g_bt` |
| 块元数据传播 (owner/scope) | `bt_stamp_segment` @ `axis_ctrl.c:1303`;段结构 +5 字段 (`block_uid/seg_index/seg_count/budget_cmd_ms/budget_sum_ms`, 28 B/段) | Non-RT | ~6 | 1 | 28 B/段 队列载荷 | **No** — 新字段仅被 ecat 观测钩子读取,控制数学不消费 |
| T_ref (planner 最终参考) | `bt_rt_record_plan_segment` @ `ecat_core.c:97`;实现 `block_timing.c:101-107` | RT | ~8 | 1 | 累加器 (随 0x0089 导出) | **No** |
| 块开始/完成 + T_sem + elapsed + override | `bt_rt_block_start` @ `ecat_core.c:92`、`bt_rt_segment_done` @ `ecat_core.c:101`、`bt_rt_cycle` @ `ecat_core.c:1271`;实现 `block_timing.c:63-189` | RT | 模块主体 | 3 | 随 0x0089 导出 | **No** — 纯算术/比较,无 IO、无 malloc、无 math.h (已通读全文核实) |
| H4 策略/几何原生事件 (0x008A) | `g93_sem_record` @ `cutter_comp.c:430,450,457,683,1078`;环缓冲 `cutter_comp.c:23-38`、查询 API `cutter_comp.c:2107-2140` | Non-RT (轨迹生成) | ~70 | 5 | **100 B/提交段**;环形 256×100 B = 25.6 KB 静态 | **No** — 只写环 |
| 读出通道 | `rpc_server.c:1101-1163` (0x0089) / `1165-1197` (0x008A);协议结构 `smc_protocol.h:720-759` | RPC 线程 | ~96 + ~45 | 2 | 196 B/快照 (0x0089);416 B/页 (0x008A, 4 事件/页) | **No** — 只读查询 + 显式 reset op (宿主发起,非控制路径) |

## 2. 汇总

| 维度 | 数值 |
|---|---|
| ΔLOC | **~580 行 (含设计注释;纯代码约 ~380)** = block_timing 模块 341 (189c+152h) + 调用点插入 ~25 + 0x008A 系统 ~70 + 协议结构 ~45 + RPC handler ~96 |
| N_probe | **14 个生产位点** (RT 4 + Non-RT 10) |
| B_trace | 每块 196 B (0x0089 快照) + 每提交段 100 B (0x008A 事件);静态内存 +25.6 KB (环) + ~196 B (`g_bt`) + 28 B/段 (队列载荷)。例:B2 单块 20 子段 → 事件 2 KB/块 |
| ΔCPU / ΔT | **已实测** (2026-09-28, WSL2 x86-64, gcc 13.3 -O2, 函数指针调用保守口径): `bt_rt_cycle` 3.5 ns/call → RT 周期净增 **0.0003%** of 1 ms 预算;含 20 段/块摊销最坏周期 **0.0009%**;`bt_rt_block_start` 3.2 / `bt_rt_record_plan_segment` 3.3 / `bt_rt_segment_done` 2.1 ns。口径注:sim 验证宿主测量,非实机 RT 硬件代表值;结论取"开销量级 <0.001% of 周期预算",不外推具体 ns |
| ControlPathChange | **全部 = No** (逐位点核实,见表 1) |

## 3. 核实备注

1. `block_timing.c` 已全文通读:所有函数只写 `g_bt` 结构体;`bt_stamp_segment` 只写段上新増元数据字段;`g93_sem_record` 只写事件环。无任何对规划器/插补器/速度剖面状态的回写。
2. 插桩语义曾经历多次观测正确性修复 (inflight 门控 2026-09-20、`virt_inc>0` 门控 Phase-0、块级配对 V13.1.1 修复 A)——这些修复均只改观测窗口,**不影响控制输出**;注释链完整保留在源码中,可作为"instrumentation discipline"的代码级证据引用。
3. `V13_SOURCE_PACK/INDEX.md` 为 S10 期快照 (覆盖 0x0083-0x0089,不含 0x008A);本表已按当前工作树补全 0x008A。
4. Non-RT 侧成本 (`bt_block_open` 每块 1 次、`g93_sem_record` 每段 1 次) 不占 RT 预算且为 O(1) 内存写,不单独 bench;如需可仿脚本 [A] 段扩展。
5. bench 实测于 2026-09-28 执行 (`scripts/measure_evidence_overhead_wsl2.sh`,WSL2 Ubuntu 24.04 / gcc 13.3.0);函数指针调用阻断内联,为偏保守上界。

## 4. 论文接入点

- §5.2 末段 "本文没有实测跨控制器插桩工时或运行期开销" → 改为本表核账结果 + ΔCPU 实测值
- §8.2 "本文没有实测这一改造的工时、实时开销或完整覆盖收益" → 引用本表口径 (仅观测侧;P4/P5 H3 补齐属新增所有权事件,不在本表内)
- 对外主张措辞按既定约束:成本**显式可核账**仅当 instrumentation access 存在;对闭源控制器不作有界性主张
