#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G41 cutter comp geometry analysis — data-driven corner validation
用法: python analyze_crc.py [csv_path]   (默认找最新的 cnc_trace_log_*.csv)
"""
import sys, os, re, glob, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import pandas as pd
import numpy as np


def find_latest_csv():
    candidates = glob.glob("cnc_trace_log_*.csv")
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def extract_timestamp(csv_path):
    m = re.search(r'(\d{8}_\d{6})', os.path.basename(csv_path))
    return m.group(1) if m else None


csv_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_csv()
if csv_path is None or not os.path.exists(csv_path):
    print(f"[ERROR] CSV not found: {csv_path or 'none'}")
    sys.exit(1)

ts = extract_timestamp(csv_path)
print(f"CSV: {csv_path}  (ts={ts or 'N/A'})\n")

df = pd.read_csv(csv_path)
x = df['X'].values; y = df['Y'].values
mask = (np.abs(x) > 0.0001) | (np.abs(y) > 0.0001)
x = x[mask]; y = y[mask]

R = 5.0

# ============================================================
# Assertion A: Outer corner @ (40, 30)
# ============================================================
print("=" * 60)
print("Assertion A: Outer corner @ (40, 30)")
print("=" * 60)
mask_a = (x > 35.0) & (x < 45.0) & (y > 20.0) & (y < 35.0)
xa, ya = x[mask_a], y[mask_a]
if len(xa) > 0:
    cx, cy = 40.0, 30.0
    dists = np.sqrt((xa - cx)**2 + (ya - cy)**2)
    arc_m = (dists > 3.0) & (dists < 7.0)
    arc_x, arc_y, arc_d = xa[arc_m], ya[arc_m], dists[arc_m]
    print(f"  Region pts: {len(xa)}, arc candidates: {len(arc_x)}")
    if len(arc_x) > 0:
        print(f"  Mean dist to (40,30): {np.mean(arc_d):.3f} mm (target 5.0)")
        angles = np.arctan2(arc_y - cy, arc_x - cx)
        amin, amax = np.min(angles), np.max(angles)
        span = np.degrees(amax - amin)
        print(f"  Angle coverage: {np.degrees(amin):.1f} to {np.degrees(amax):.1f} (span {span:.1f})")
        if span > 70:   print("  [OK] Arc detected (>70deg)")
        elif span > 5:  print("  [WARN] Partial arc")
        else:           print("  [FAIL] No arc - inner corner treatment applied")
    print(f"  First 3: ({xa[0]:.2f},{ya[0]:.2f}) ({xa[1]:.2f},{ya[1]:.2f}) ({xa[2]:.2f},{ya[2]:.2f})")
    print(f"  Last 3:  ({xa[-3]:.2f},{ya[-3]:.2f}) ({xa[-2]:.2f},{ya[-2]:.2f}) ({xa[-1]:.2f},{ya[-1]:.2f})")
else:
    print("  [FAIL] No points near (40,30)")
print()

# ============================================================
# Assertion B: Inner corner @ (40, 10)
# ============================================================
print("=" * 60)
print("Assertion B: Inner corner @ (40, 10)")
print("=" * 60)
mask_b = (x > 30.0) & (x < 42.0) & (y > 8.0) & (y < 20.0)
xb, yb = x[mask_b], y[mask_b]
if len(xb) > 0:
    max_y_idx = np.argmax(yb)
    px, py = xb[max_y_idx], yb[max_y_idx]
    dx, dy = px - 40.0, py - 10.0
    print(f"  Corner peak: ({px:.3f}, {py:.3f}), offset: dx={dx:.2f} dy={dy:.2f}")
    print(f"  Dist from orig: {np.sqrt(dx**2+dy**2):.3f} mm")
    if dx < -0.5:  print(f"  [OK] X inward {abs(dx):.1f}mm (inner corner)")
    else:          print(f"  [WARN] No X inward shift")
else:
    print("  [FAIL] No points near (40,10)")
print()

# ============================================================
# Assertion C: Acute corner @ (30, 15) — Swallowtail
# ============================================================
print("=" * 60)
print("Assertion C: Swallowtail @ (30, 15)")
print("=" * 60)
mask_c = (x > 15.0) & (x < 38.0) & (y > 5.0) & (y < 25.0)
xc, yc = x[mask_c], y[mask_c]
if len(xc) > 0:
    xmax, ymin = np.max(xc), np.min(yc)
    x_excess = xmax - 30.0
    y_deficit = 15.0 - ymin
    print(f"  Region: {len(xc)} pts, X [{np.min(xc):.2f},{xmax:.2f}] Y [{ymin:.2f},{np.max(yc):.2f}]")
    print(f"  X excess: {x_excess:.2f}mm, Y deficit: {y_deficit:.2f}mm")
    if x_excess > 2 or y_deficit > 2:
        print(f"  [SEVERE] Swallowtail protrusion {max(x_excess,y_deficit):.1f}mm")
    elif x_excess > 0.5 or y_deficit > 0.5:
        print(f"  [MODERATE] Swallowtail")
    else:
        print(f"  [MILD] Minimal")
    for i in range(1, len(xc)):
        if xc[i] - xc[i-1] < -0.1:
            print(f"  [BACKTRACK] X {xc[i-1]:.3f} -> {xc[i]:.3f} at idx {i}")
            break
    else:
        print("  No backtrack detected")
    print(f"  First 5: ", end="")
    for i in range(min(5, len(xc))):
        print(f"({xc[i]:.2f},{yc[i]:.2f})", end=" ")
    print()
else:
    print("  [FAIL] No points near (30,15)")
print()
print("=" * 60)
print("Analysis complete." if not ts else f"Analysis complete. [ts={ts}]")
print("=" * 60)
