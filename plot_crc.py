#!/usr/bin/env python3
"""
G41/G42 刀具半径补偿 (Cutter Radius Compensation) 2D 可视化分析脚本
======================================================================
读取仿真输出的 cnc_trace_log_*.csv，绘制：
  - 红色实线: 实际刀补插补轨迹 (Compensated Tool Path)
  - 黑色虚线: 原始理论 G 代码轨迹 (Original Contour)
  - 小黑点:   原始拐点

输出文件自动带时间戳，多次运行不会互相覆盖。

依赖: pandas, matplotlib
用法: python plot_crc.py [csv_path]   (默认找最新的 cnc_trace_log_*.csv)
"""

import sys
import os
import re
import glob
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 无头模式，仅输出图片
import matplotlib.pyplot as plt


def extract_timestamp(csv_path):
    """从文件名中提取时间戳: cnc_trace_log_20260613_001200.csv → 20260613_001200"""
    m = re.search(r'(\d{8}_\d{6})', os.path.basename(csv_path))
    return m.group(1) if m else None


def find_latest_csv():
    """查找工作目录下最新的 cnc_trace_log_*.csv"""
    candidates = glob.glob("cnc_trace_log_*.csv")
    if not candidates:
        return None
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def find_xy_columns(df):
    """查找 X 和 Y 列 (支持 'X'/'Y' 或 'x_mm'/'y_mm' 等变体)"""
    x_col = y_col = None
    for col in df.columns:
        c = col.strip()
        if c == 'X':
            x_col = col
        elif c == 'Y':
            y_col = col
    if x_col is None or y_col is None:
        for col in df.columns:
            cu = col.strip().upper()
            if cu.startswith('X') and x_col is None:
                x_col = col
            if cu.startswith('Y') and y_col is None:
                y_col = col
    return x_col, y_col


def trim_standstill(x_arr, y_arr):
    """裁剪掉首尾的零点静止段 (tool不在移动时的采样点)"""
    n = len(x_arr)
    first, last = 0, n - 1
    eps = 0.0001
    for i in range(n):
        if abs(x_arr[i]) > eps or abs(y_arr[i]) > eps:
            first = max(0, i - 3)
            break
    for i in range(n - 1, -1, -1):
        if abs(x_arr[i]) > eps or abs(y_arr[i]) > eps:
            last = min(n - 1, i + 3)
            break
    return x_arr[first:last + 1], y_arr[first:last + 1]


def main():
    # 1. 确定输入 CSV
    csv_path = sys.argv[1] if len(sys.argv) > 1 else find_latest_csv()
    if csv_path is None:
        print("[ERROR] 未指定 CSV 且找不到 cnc_trace_log_*.csv")
        sys.exit(1)
    if not os.path.exists(csv_path):
        print(f"[ERROR] 找不到轨迹文件: {csv_path}")
        sys.exit(1)

    ts = extract_timestamp(csv_path)
    print(f"[INFO] 输入: {csv_path}  (时间戳: {ts or '无'})")

    df = pd.read_csv(csv_path)
    print(f"[INFO] 读取 {len(df)} 条轨迹记录, 列: {df.columns.tolist()}")

    x_col, y_col = find_xy_columns(df)
    if x_col is None or y_col is None:
        print(f"[ERROR] 找不到 X/Y 列。可用列: {df.columns.tolist()}")
        sys.exit(1)
    print(f"[INFO] X 列: '{x_col}', Y 列: '{y_col}'")

    # 2. 裁剪静默段
    x_vals = df[x_col].values.copy()
    y_vals = df[y_col].values.copy()
    x_vals, y_vals = trim_standstill(x_vals, y_vals)
    print(f"[INFO] 有效轨迹点: {len(x_vals)} (裁剪后)")

    # 3. 原始理论 G 代码轨迹 (硬编码)
    original_x = np.array([0, 20, 40, 40, 20, 30, 10, 0], dtype=float)
    original_y = np.array([0, 0,  10, 30, 30, 15, 15, 0], dtype=float)

    # 4. 绘图
    fig, ax = plt.subplots(figsize=(14, 14))

    ax.plot(x_vals, y_vals, 'r-', linewidth=1.0, alpha=0.85,
            label='Compensated Tool Path (G41, R=5mm)')
    ax.plot(original_x, original_y, 'k--', linewidth=1.2,
            label='Original Contour (G-Code)')
    ax.scatter(original_x, original_y, c='black', s=40, zorder=5,
               marker='o', label='Contour Vertices')

    labels = [
        '(0,0) Start/G40', '(20,0)', '(40,10) Inner',
        '(40,30) Outer', '(20,30)',
        '(30,15) Swallowtail', '(10,15)', '(0,0) End'
    ]
    offsets = [
        (-30, -15), (-30, -15), (10, -15), (10, 10),
        (-40, -15), (10, -15), (-30, -15), (10, 10)
    ]
    for i, (ox, oy) in enumerate(zip(original_x, original_y)):
        ax.annotate(labels[i], (ox, oy),
                    textcoords="offset points",
                    xytext=offsets[i], fontsize=8,
                    color='#333333', alpha=0.85,
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                              alpha=0.7, edgecolor='gray', linewidth=0.5))

    ts_label = f"  [{ts}]" if ts else ""
    ax.set_xlabel('X (mm)', fontsize=13)
    ax.set_ylabel('Y (mm)', fontsize=13)
    ax.set_title(
        f'G41 Cutter Radius Compensation — 2D Tool Path Analysis{ts_label}\n'
        '(Left Compensation, R=5.0mm, G17 XY Plane)',
        fontsize=14, fontweight='bold')
    ax.axis('equal')
    ax.grid(True, alpha=0.25)
    ax.legend(loc='upper left', fontsize=10)
    ax.set_xlim(-10, 55)
    ax.set_ylim(-5, 40)

    plt.tight_layout()

    # 5. 输出文件 (带时间戳)
    if ts:
        out_path = f"crc_analysis_{ts}.png"
    else:
        out_path = "crc_analysis.png"
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    print(f"[INFO] 图表已保存至: {out_path}")
    plt.close()


if __name__ == "__main__":
    main()
