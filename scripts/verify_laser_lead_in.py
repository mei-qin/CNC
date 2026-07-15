#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
引线 / 微连接 (P1-c) CSV 验证
依据 test_laser_lead_in_micro_joint.nc 程序流程:
  G00 X0 Y0
  S1000 M3
  M72            -> lead-in flag ON  (seg_flags = 0x01 = 1)
  G01 X10 Y0     -> X: 0->10   flags=1
  G01 X20 Y5     -> X: 10->20  flags=1
  M73            -> lead-in flag OFF (seg_flags = 0)
  G01 X50 Y5     -> X: 20->50  flags=0
  M74            -> micro-joint flag ON (seg_flags = 0x02 = 2)
  G01 X70 Y5     -> X: 50->70  flags=2
  M75            -> micro-joint flag OFF (seg_flags = 0)
  G01 X100 Y5    -> X: 70->100 flags=0
  M5 / M30

判据:
  L1 current_seg_flags 出现 {0,1,2} 三类, 且不含 3(0x03 modal 叠加未清) 及其它
  L2 flags=1 的 X 窗口应落在 [0,20]  (lead-in 段, M72-M73 间)
  L3 flags=2 的 X 窗口应落在 [50,70] (micro-joint 段, M74-M75 间)
  L4 pierce_count 全程 = 0 (Hook B 在引线测试不应触发)
  L5 is_piercing 全程 = 0
  L6 (顺序) 标志切换顺序应为 0 -> 1 -> 0 -> 2 -> 0
"""
import csv, sys

PATH = sys.argv[1] if len(sys.argv) > 1 else "tests/output/laser_log/log_laser_lead_in.csv"
EPS = 0.6  # 段末插值过冲容差

rows = []
with open(PATH, newline="") as f:
    r = csv.DictReader(f)
    for row in r:
        rows.append(row)

n = len(rows)
print(f"读取 {n} 行, 列数={len(rows[0]) if rows else 0}")
print(f"源文件: {PATH}\n")

def num(row, k):
    return float(row[k])

# 收集 flags 分布与 X 窗口
from collections import defaultdict
flag_x = defaultdict(list)
flag_vt = defaultdict(list)
pierce_vals = set()
piercing_vals = set()
for row in rows:
    fl = int(num(row, "current_seg_flags"))
    flag_x[fl].append(num(row, "X"))
    flag_vt[fl].append(num(row, "virtual_time_ms"))
    pierce_vals.add(int(num(row, "pierce_count")))
    piercing_vals.add(int(num(row, "is_piercing")))

distinct_flags = sorted(flag_x.keys())
print("=== L1  current_seg_flags 取值分布 ===")
for fl in distinct_flags:
    print(f"  flags={fl:#04x}: {len(flag_x[fl]):6d} 行  X∈[{min(flag_x[fl]):.3f},{max(flag_x[fl]):.3f}]  vt∈[{min(flag_vt[fl]):.0f},{max(flag_vt[fl]):.0f}]")
expected = {0, 1, 2}
L1 = (set(distinct_flags) == expected)
print(f"  -> L1 {'PASS' if L1 else 'FAIL'}: 取值集合={set(distinct_flags)} 期望={expected} (出现3=modal叠加未清, FAIL)\n")

# L2: flags=1 X 窗口
print("=== L2  flags=1 (lead-in) X 窗口 ===")
if 1 in flag_x:
    xmin, xmax = min(flag_x[1]), max(flag_x[1])
    L2 = (-EPS <= xmin <= 20 + EPS) and (xmin >= -EPS) and (xmax <= 20 + EPS)
    print(f"  X∈[{xmin:.3f},{xmax:.3f}]  期望⊆[0,20](容差{EPS})  -> L2 {'PASS' if L2 else 'FAIL'}")
    # 更细: 是否覆盖到 ~10-20 区间 (切割段)
    in_cut = [x for x in flag_x[1] if 9.5 <= x <= 20.5]
    print(f"  其中 X∈[10,20] 的行数={len(in_cut)} (应 >0, 验证 M72-M73 切割段被标记)")
else:
    L2 = False
    print("  flags=1 未出现 -> L2 FAIL (Hook A1 未触发)")

# L3: flags=2 X 窗口
print("=== L3  flags=2 (micro-joint) X 窗口 ===")
if 2 in flag_x:
    xmin, xmax = min(flag_x[2]), max(flag_x[2])
    L3 = (50 - EPS <= xmin) and (xmax <= 70 + EPS)
    print(f"  X∈[{xmin:.3f},{xmax:.3f}]  期望⊆[50,70](容差{EPS})  -> L3 {'PASS' if L3 else 'FAIL'}")
    near70 = [x for x in flag_x[2] if 69.0 <= x <= 70.5]
    print(f"  其中 X≈70 的行数={len(near70)} (应 >0, 验证 M74-M75 段被标记)")
else:
    L3 = False
    print("  flags=2 未出现 -> L3 FAIL (Hook A2 未触发)")

# L4: pierce_count
print("=== L4  pierce_count 全程 ===")
L4 = (pierce_vals == {0})
print(f"  取值集合={pierce_vals}  -> L4 {'PASS' if L4 else 'FAIL'} (出现>0 表示 Hook B 误触发)")

# L5: is_piercing
print("=== L5  is_piercing 全程 ===")
L5 = (piercing_vals == {0})
print(f"  取值集合={piercing_vals}  -> L5 {'PASS' if L5 else 'FAIL'}")

# L6: 顺序
print("=== L6  标志切换顺序 ===")
seq = []
prev = None
for row in rows:
    fl = int(num(row, "current_seg_flags"))
    if fl != prev:
        seq.append(fl)
        prev = fl
print(f"  切换序列: {seq}")
expected_seq = [0, 1, 0, 2, 0]
# 允许前后有额外 0，核心子序列应为 0,1,0,2,0
def contains_subseq(full, sub):
    it = iter(full)
    return all(any(f == s for f in it) for s in sub)
L6 = contains_subseq(seq, expected_seq)
print(f"  核心顺序 {expected_seq} 是否出现 -> L6 {'PASS' if L6 else 'FAIL'}")

print("\n========== 引线/微连接 验证汇总 ==========")
results = {"L1": L1, "L2": L2, "L3": L3, "L4": L4, "L5": L5, "L6": L6}
allpass = all(results.values())
for k, v in results.items():
    print(f"  {k}: {'PASS' if v else 'FAIL'}")
print(f"\n总体: {'✅ 全部 PASS' if allpass else '❌ 存在 FAIL'}")
sys.exit(0 if allpass else 1)
