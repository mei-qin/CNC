# P1-c 激光段标志 (L/J Flags) 2 终端联调验证报告

> **日期**: 2026-07-14
> **环境**: WSL2 (Ubuntu) / sim 模式
> **被测组件**: `gcode_parser.c` (M72-M75 modal) + `axis_ctrl.c` (seg_flags 快照) + `preview_streamer.h` (v2 协议) + `preview_subscriber.py`
> **测试文件**: `tests/gcode/test_laser_lead_in_micro_joint.nc`

---

## 一、Bug 修复

### 1. `src/gcode_parser.c:1138-1145` — M72-M75 永不执行 (严重)

**违反规则**: 控制流嵌套错误 — M72-M75 的 `else if` 分支被嵌套在 `m_code == 62 || 63 || 67 || 68 || 10 || 11 || 12` 的条件块内部。由于 72-75 不匹配该条件，外层 `else if` 为 false，内层 M72-M75 永远不会执行。

**触发场景**:
```
LoadProgram test_laser_lead_in_micro_joint.nc
→ parser 解析 M72/M73/M74/M75
→ 进入 case 'M' 分支
→ m_code 不匹配 62/63/67/68/10/11/12
→ 跳过整个 else if 块
→ M72-M75 的 laser_seg_flags 设置永远不执行
→ 所有段 seg_flags=0 (无 L/J 标志)
```

**后果**: 激光引线段 (lead-in) 和微连接段 (micro-joint) 无法在 UI 中区分，CAM 工艺标记完全丢失。

**修复**:
```diff
-                    else if(m_code == 12){ g_state.gas_select = 3; }    // Air
-                    // ---- Laser Phase B4: 引线/微连接段标记 (modal, M72-M75) ----
-                    else if(m_code == 72){ g_state.laser_seg_flags |= SEG_FLAG_LEAD_IN; }
-                    else if(m_code == 73){ g_state.laser_seg_flags &= ~SEG_FLAG_LEAD_IN; }
-                    else if(m_code == 74){ g_state.laser_seg_flags |= SEG_FLAG_MICRO_JOINT; }
-                    else if(m_code == 75){ g_state.laser_seg_flags &= ~SEG_FLAG_MICRO_JOINT; }
-                }
+                    else if(m_code == 12){ g_state.gas_select = 3; }    // Air
+                }
+                // ---- Laser Phase B4: 引线/微连接段标记 (modal, M72-M75) ----
+                // @Danger: 原实现将 M72-M75 误嵌套在 M62-12 条件块内部, 导致永不执行 (已修复).
+                else if(m_code == 72){ g_state.laser_seg_flags |= SEG_FLAG_LEAD_IN;     m_code = -1; }
+                else if(m_code == 73){ g_state.laser_seg_flags &= ~SEG_FLAG_LEAD_IN;    m_code = -1; }
+                else if(m_code == 74){ g_state.laser_seg_flags |= SEG_FLAG_MICRO_JOINT; m_code = -1; }
+                else if(m_code == 75){ g_state.laser_seg_flags &= ~SEG_FLAG_MICRO_JOINT; m_code = -1; }
```

**关键改动**:
1. M72-M75 移出 M62-12 块，成为独立的 `else if` 分支
2. 添加 `m_code = -1` — M72-M75 是 modal 标记，不应作为 M-code 段入队
3. 添加 AI-Tags: `@Context`/`@Danger`/`@Thread-Safety`

---

## 二、7 检查点验证结果

### 检查点 1: M72 后续段 flags=L ✅

**T1 preview subscriber 输出**:
```
seg_id=1 line=6  G01  flags=L  target=(+10.00, +0.00, ...)  ← M72 后 lead-in
seg_id=2 line=7  G01  flags=L  target=(+20.00, +5.00, ...)  ← 仍 lead-in
```

### 检查点 2: M73 后段 flags= 空 ✅

```
seg_id=3 line=10 G01  flags=   target=(+50.00, +5.00, ...)  ← M73 后正常
```

### 检查点 3: M74 后段 flags=J ✅

```
seg_id=4 line=13 G01  flags=J  target=(+70.00, +5.00, ...)  ← M74 后 micro-joint
```

### 检查点 4: M75 后段 flags= 空 ✅

```
seg_id=5 line=16 G01  flags=   target=(+100.00, +5.00, ...)  ← M75 后正常
```

### 检查点 5: M30 后 laser_seg_flags 清零 ✅

**第二次 LoadProgram** (同一文件) 输出 seg_id 6-11，flags 模式与 seg_id 0-5 完全一致：

```
seg_id=6  line=2  G00  flags=    ← 清零, 不残留
seg_id=7  line=6  G01  flags=L   ← M72 重新生效
seg_id=8  line=7  G01  flags=L
seg_id=9  line=10 G01  flags=
seg_id=10 line=13 G01  flags=J
seg_id=11 line=16 G01  flags=
```

**结论**: M30 `g_state.laser_seg_flags = 0` 重置正确，跨程序不泄漏。

### 检查点 6: ack version=2 ✅

```
[prev] ack OK: max_per_tick=16 seg_size=560B
```

`preview_subscriber.py` 内部检查 `PREV_VERSION=2`，不匹配会 `FATAL` 退出。ack OK 未 FATAL → version=2 确认。

### 检查点 7: snapshot + event 通道不受影响 ✅

**9528 snapshot**:
```
[snap] subscribed, actual_freq=1Hz
cycle=142082 state=IDLE  pos=(0.00, 0.00, 0.00)
```

**9530 event**:
```
[event] ack OK: max_per_tick=32 event_size=88B
seq=0 INFO PARSER 0x0030 LoadProgram start (preview)
seq=1 INFO PARSER 0x0033 program done (M30) val=18
```

---

## 三、回归测试结果

| # | 测试项 | 结果 |
|---|--------|------|
| R1 | Legacy `test_laser_basic.nc` | ret=0 ✅ |
| R2 | Legacy `test_laser_g04_pierce.nc` | ret=0 ✅ |
| R3 | 9528 snapshot 仍工作 | ✅ (首帧延迟 0.5ms) |
| R4 | 9530 event 仍工作 | ✅ (ack OK + 事件流) |
| R5 | L1_sharp_corner.nc (非激光程序) | flags 全空 ✅ |

> **注**: R3 中 state=ALARM 来自 legacy 激光测试触发的激光安全互锁 (LASER 0x0010)，与 M72-M75 修改无关。

---

## 四、测试 G 代码文件

`tests/gcode/test_laser_lead_in_micro_joint.nc`:
```
G17 G21 G90 G94 G54     (line 1)
G00 X0 Y0               (line 2)  → seg_id=0  flags=
S1000 M3                (line 3)
                        (line 4)
M72                     (line 5)  → lead-in start
G01 X10 Y0 F300         (line 6)  → seg_id=1  flags=L
G01 X20 Y5 F300         (line 7)  → seg_id=2  flags=L
M73                     (line 8)  → lead-in end
                        (line 9)
G01 X50 Y5 F600         (line 10) → seg_id=3  flags=
                        (line 11)
M74                     (line 12) → micro-joint start
G01 X70 Y5 F300         (line 13) → seg_id=4  flags=J
M75                     (line 14) → micro-joint end
                        (line 15)
G01 X100 Y5 F600        (line 16) → seg_id=5  flags=
M5                      (line 17)
M30                     (line 18) → laser_seg_flags=0 清零
```

---

## 五、架构说明

### 数据流

```
M72-M75 (parser)
    ↓
g_state.laser_seg_flags (modal, uint8_t)
    ↓
axis_ctrl.c 入队时快照 → TrajectorySegment_t.seg_flags
    ↓
PreviewStreamer_Push → 9529 preview subscriber
    ↓
Python: seg_flags & 0x01 → "L" (lead-in)
        seg_flags & 0x02 → "J" (micro-joint)
```

### seg_flags 位定义

| Bit | 名称 | 值 | M 代码 |
|-----|------|----|--------|
| 0 | SEG_FLAG_LEAD_IN | 0x01 | M72 (set) / M73 (clear) |
| 1 | SEG_FLAG_MICRO_JOINT | 0x02 | M74 (set) / M75 (clear) |
| 2-7 | 保留 | — | 未来 pierce / over-burn / tab |

### RT 影响: 0

`seg_flags` 是段元数据 (UI 预览用)，不参与 RT 插补/激光控制。B-spline 合并段时继承首段 flags。

---

## 六、验证结论

### 总体评定: ✅ PASS

**7/7 检查点全部通过** + **5/5 回归测试全部通过**

**修复的 Bug**: M72-M75 控制流嵌套错误 (严重 — 永不执行)

**核心验证项**:
- ✅ M72/M73 lead-in 标记正确 (flags=L)
- ✅ M74/M75 micro-joint 标记正确 (flags=J)
- ✅ M30 清零无残留
- ✅ PREV_VERSION=2 协议版本正确
- ✅ 4 端口独立运行互不干扰
- ✅ Legacy 激光测试不破坏
- ✅ 非激光程序 flags 全空
