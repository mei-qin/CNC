#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_trace.py - CNC 轨迹数据自动化验证脚本

用法:
    python verify_trace.py [--test {rtcp|bspline|g93}] [csv_file]

默认读取 cnc_trace_log.csv。
对 RTCP 补偿、B-Spline 尖角断裂、G93 时间守恒执行断言验证。
使用 --test 指定测试类型; 若不指定则自动检测。
"""

import sys
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ===================================================================
# 配置
# ===================================================================
OUTPUT_DIR = "trace_report"
os.makedirs(OUTPUT_DIR, exist_ok=True)

PASS_COUNT = 0
FAIL_COUNT = 0


def assert_test(name: str, condition: bool, detail: str = ""):
    global PASS_COUNT, FAIL_COUNT
    if condition:
        PASS_COUNT += 1
        print(f"  [PASS] {name}")
    else:
        FAIL_COUNT += 1
        print(f"  [FAIL] {name} -- {detail}")


def load_csv(csv_file: str) -> pd.DataFrame:
    print(f"加载 CSV: {csv_file}")
    df = pd.read_csv(csv_file)
    df = df.sort_values("cycle").reset_index(drop=True)
    return df


def detect_test_type(df: pd.DataFrame) -> str:
    """根据数据特征自动检测测试类型"""
    x_range = df["x_mm"].max() - df["x_mm"].min()
    y_range = df["y_mm"].max() - df["y_mm"].min()
    b_range = df["b_deg"].max() - df["b_deg"].min()

    if b_range > 30.0 and x_range < 1.0 and y_range < 1.0:
        return "rtcp"
    if b_range > 30.0:
        return "g93"
    return "bspline"


# ===================================================================
# Test 1: 连续性检查 (飞车检测) - 通用
# ===================================================================
def test_continuity(df: pd.DataFrame):
    print("\n=== Test 1: 连续性检查 (飞车检测) ===")

    df_m = df[df["v_target"] > 1e-12].copy()
    if len(df_m) == 0:
        print("  警告: CSV 中没有运动数据, 使用全量数据")
        df_m = df.copy()

    df_m = df_m.sort_values("cycle").reset_index(drop=True)
    if len(df_m) > 1:
        df_m["dt_ms"] = df_m["virtual_time_ms"].diff().fillna(5.0)
    else:
        df_m["dt_ms"] = 5.0

    for col in ["x_mm", "y_mm", "z_mm", "b_deg", "c_deg"]:
        diff = df_m[col].diff().fillna(0.0)
        dt = df_m["dt_ms"].clip(lower=0.001)
        vel = diff / dt

        has_nan = vel.isna().any()
        has_inf = np.isinf(vel).any()
        assert_test(f"{col} 无 NaN 速度", not has_nan,
                     f"NaN 速度索引: {vel[vel.isna()].index.tolist()}")
        assert_test(f"{col} 无 Inf 速度", not has_inf,
                     f"Inf 速度索引: {vel[np.isinf(vel)].index.tolist()}")

        # 飞车阈值: 直线轴 > 500 mm/ms, 旋转轴 > 50 deg/ms
        threshold = 50.0 if col in ("b_deg", "c_deg") else 500.0
        over_limit = (vel.abs() > threshold).sum()
        assert_test(f"{col} 速度 < {threshold}", over_limit == 0,
                     f"max={vel.abs().max():.2f}, 超限 {over_limit} 次")


# ===================================================================
# Test 2: RTCP 补偿验证
# ===================================================================
def test_rtcp(df: pd.DataFrame):
    print("\n=== Test 2: RTCP 补偿验证 ===")

    df_m = df[df["v_target"] > 1e-12].copy()
    if len(df_m) == 0:
        df_m = df.copy()
    df_m = df_m.sort_values("cycle").reset_index(drop=True)

    z_range = df_m["z_mm"].max() - df_m["z_mm"].min()
    b_range = df_m["b_deg"].max() - df_m["b_deg"].min()

    assert_test("RTCP: B 轴有显著运动 (>= 30 deg)", b_range >= 30.0,
                f"B 轴范围 = {b_range:.2f} deg")

    # 核心: B 轴倾倒时物理 Z 必须有补偿位移
    b_diff = df_m["b_deg"].diff().abs()
    b_change_mask = b_diff > 0.5
    if b_change_mask.any():
        b_change_start = b_change_mask.idxmax()
        idx_start = max(0, b_change_start - 10)
        idx_end = min(len(df_m) - 1, b_change_start + 200)
        seg = df_m.iloc[idx_start:idx_end]
        z_delta = seg["z_mm"].max() - seg["z_mm"].min()
        assert_test(
            "RTCP: B 轴倾倒时 Z 轴有非零补偿位移",
            abs(z_delta) > 0.01,
            f"B 变化区间 Z 位移 = {z_delta:.4f} mm (< 0.01, RTCP 可能未生效)"
        )
    else:
        assert_test("RTCP: B 轴倾倒时 Z 轴有非零补偿位移", False,
                     "未检测到 B 轴显著变化")

    # X/Y 逻辑坐标不变 → 物理坐标不变? (RTCP 逆解后 X/Y 物理应不变或极小)
    x_range = df_m["x_mm"].max() - df_m["x_mm"].min()
    y_range = df_m["y_mm"].max() - df_m["y_mm"].min()
    assert_test(
        "RTCP: 逻辑 X 不变时物理 X 变化极小 (< 1 mm)",
        x_range < 1.0,
        f"物理 X 范围 = {x_range:.4f} mm"
    )

    # 3D 轨迹图
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    scatter = ax.scatter(
        df_m["x_mm"], df_m["y_mm"], df_m["z_mm"],
        c=df_m["b_deg"], cmap='coolwarm', s=1, alpha=0.7
    )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("RTCP 3D Trajectory (color = B axis deg)")
    fig.colorbar(scatter, ax=ax, label="B (deg)")
    fig.savefig(os.path.join(OUTPUT_DIR, "rtcp_3d_trace.png"), dpi=150)
    plt.close(fig)
    print("  3D 轨迹图已保存: trace_report/rtcp_3d_trace.png")


# ===================================================================
# Test 3: B-Spline 尖角断裂 + M05 速度降零验证
# ===================================================================
def test_bspline(df: pd.DataFrame):
    print("\n=== Test 3: B-Spline 尖角断裂 + M05 速度降零验证 ===")

    df_m = df[df["v_target"] > 1e-12].copy()
    if len(df_m) == 0:
        df_m = df.copy()
    df_m = df_m.sort_values("cycle").reset_index(drop=True)

    # B-Spline 尖角: 在 Y 方向应检测到急剧变化 (X0.3 处 Y 从 0.3→1.3→2.3)
    y_diff = df_m["y_mm"].diff().abs()
    if len(y_diff) > 1:
        max_y_jump = y_diff.max()
        # 尖角断裂意味着 Y 方向的速度不应该被样条抹平
        # 即存在显著的 Y 方向速度跳变
        assert_test(
            "B-Spline: 检测到 Y 方向尖角断裂 (最大 Y 步进 > 0.05 mm)",
            max_y_jump > 0.05,
            f"最大 Y 步进 = {max_y_jump:.4f} mm"
        )
    else:
        assert_test("B-Spline: 检测到 Y 方向尖角断裂", False, "数据量不足")

    # M05 速度降零: 运动段末尾速度必须趋近 0
    # 查找 v_target 从非零降至接近零的拐点
    v = df_m["v_target"].values
    if len(v) > 5:
        # 从后向前找到 v_target 最后一个显著值
        nonzero_idx = np.where(v > 1e-6)[0]
        if len(nonzero_idx) > 0:
            last_moving = nonzero_idx[-1]
            # 最后运动点到队列末尾, v_target 应降至 0
            tail_seg = df_m.iloc[last_moving:min(last_moving + 20, len(df_m))]
            if len(tail_seg) > 0:
                final_v = tail_seg["v_target"].iloc[-1]
                assert_test(
                    f"M05: 速度降至接近零 (v_target={final_v:.6f} < 1e-4)",
                    final_v < 1e-4,
                    f"末尾 v_target = {final_v:.6f}"
                )

    # 2D 轨迹图
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(df_m["x_mm"], df_m["y_mm"], 'b-', linewidth=0.5, alpha=0.8)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("B-Spline XY Trajectory (sharp corner at X=0.3)")
    ax.grid(True, alpha=0.3)
    fig.savefig(os.path.join(OUTPUT_DIR, "bspline_xy_trace.png"), dpi=150)
    plt.close(fig)
    print("  XY 轨迹图已保存: trace_report/bspline_xy_trace.png")


# ===================================================================
# Test 4: G93 时间守恒验证
# ===================================================================
def test_g93(df: pd.DataFrame):
    print("\n=== Test 4: G93 时间守恒验证 ===")

    df_m = df[df["v_target"] > 1e-12].copy()
    if len(df_m) == 0:
        df_m = df.copy()
    df_m = df_m.sort_values("cycle").reset_index(drop=True)

    t_first = df_m["virtual_time_ms"].iloc[0]
    t_last = df_m["virtual_time_ms"].iloc[-1]
    elapsed = t_last - t_first

    print(f"  virtual_time 跨度: {t_first:.1f} -> {t_last:.1f} ms = {elapsed:.1f} ms")

    # G93 F6.0 -> T = 60/6.0 = 10s = 10000ms
    expected_t_ms = 10000.0
    tolerance_ms = 50.0  # +/- 50ms

    assert_test(
        f"G93 时间守恒: {elapsed:.1f} ms ~ {expected_t_ms} ms (+/-{tolerance_ms} ms)",
        abs(elapsed - expected_t_ms) <= tolerance_ms,
        f"偏差 {abs(elapsed - expected_t_ms):.1f} ms 超出容差"
    )

    # 速度均匀性 (G93 应保持恒定切向速度)
    if len(df_m) > 1:
        dt = df_m["virtual_time_ms"].diff().clip(lower=0.001).fillna(5.0)
        vx = df_m["x_mm"].diff().fillna(0) / dt
        vy = df_m["y_mm"].diff().fillna(0) / dt
        vz = df_m["z_mm"].diff().fillna(0) / dt
        vel_mag = np.sqrt(vx**2 + vy**2 + vz**2)
        vel_mag = vel_mag.replace([np.inf, -np.inf], np.nan).dropna()
        if len(vel_mag) > 0 and vel_mag.mean() > 1e-9:
            vel_cv = vel_mag.std() / vel_mag.mean()
            assert_test(
                f"G93 速度均匀性: CV = {vel_cv:.4f} (期望 < 0.5)",
                vel_cv < 0.5,
                f"速度变异系数 {vel_cv:.4f} 偏大"
            )

    # B 轴位移验证 (含旋转轴的圆弧)
    b_range = df_m["b_deg"].max() - df_m["b_deg"].min()
    assert_test(
        f"G93: B 轴有位移 ({b_range:.2f} deg, 期望 >= 40 deg)",
        b_range >= 40.0,
        f"B 轴范围 = {b_range:.2f} deg"
    )

    # 2D XY 轨迹图 (应为圆弧)
    fig, ax = plt.subplots(figsize=(8, 6))
    scatter = ax.scatter(
        df_m["x_mm"], df_m["y_mm"],
        c=df_m["virtual_time_ms"], cmap='viridis', s=1, alpha=0.7
    )
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("G93 XY Trajectory (color = virtual_time_ms)")
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    fig.colorbar(scatter, ax=ax, label="virtual_time (ms)")
    fig.savefig(os.path.join(OUTPUT_DIR, "g93_xy_trace.png"), dpi=150)
    plt.close(fig)
    print("  XY 轨迹图已保存: trace_report/g93_xy_trace.png")


# ===================================================================
# Main
# ===================================================================
def main():
    parser = argparse.ArgumentParser(description="CNC 轨迹数据自动化验证")
    parser.add_argument("csv_file", nargs="?", default="cnc_trace_log.csv",
                        help="CSV 文件路径")
    parser.add_argument("--test", choices=["rtcp", "bspline", "g93"],
                        default=None, help="指定测试类型 (不指定则自动检测)")
    args = parser.parse_args()

    if not os.path.exists(args.csv_file):
        print(f"错误: 文件不存在: {args.csv_file}")
        sys.exit(2)

    df = load_csv(args.csv_file)
    if len(df) == 0:
        print("错误: CSV 文件为空")
        sys.exit(2)

    print(f"数据行数: {len(df)}, 周期范围: {df['cycle'].min()} - {df['cycle'].max()}")

    # 通用连续性检查
    test_continuity(df)

    # 检测或指定测试类型
    test_type = args.test if args.test else detect_test_type(df)
    print(f"\n测试类型: {test_type}")

    if test_type == "rtcp":
        test_rtcp(df)
    elif test_type == "bspline":
        test_bspline(df)
    elif test_type == "g93":
        test_g93(df)

    # 汇总
    print(f"\n{'='*50}")
    print(f"  验证完成: {PASS_COUNT} PASS, {FAIL_COUNT} FAIL")
    print(f"{'='*50}")

    if FAIL_COUNT > 0:
        print("警告: 存在失败断言, 请检查轨迹数据!")
        sys.exit(1)
    else:
        print("所有断言通过 -- 数据流水线健康。")
        sys.exit(0)


if __name__ == "__main__":
    main()
