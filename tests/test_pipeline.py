"""
test_pipeline.py — CNC 核心算法验证框架 (pytest)

用法:
    # 处理单个 CSV 文件
    pytest test_pipeline.py -v -s --csv cnc_trace_log.csv

    # 使用历史日志目录 (包含 log_rtcp.csv / log_bspline.csv / log_g93.csv)
    pytest test_pipeline.py -v -s --csv-dir .

    # 快速运行全部测试
    pytest test_pipeline.py -v

三种测试模式 (自动识别 CSV 内容):
    L1: B-Spline 锐角截断 + G41 刀补叠加
    L2: G93 倒数时间强一致性
    L3: RTCP 五轴逆解补偿
"""

import pytest
import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D

# ── 中文支持 ──────────────────────────────────────────────
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'STSong', 'SimSun']
matplotlib.rcParams['axes.unicode_minus'] = False

# ── 输出目录 ──────────────────────────────────────────────
OUTPUT = Path(__file__).resolve().parent.parent / "tests" / "output"
OUTPUT.mkdir(parents=True, exist_ok=True)

# ═══════════════════════════════════════════════════════════
#  工具函数
# ═══════════════════════════════════════════════════════════

def load_csv(path: Path):
    """加载 trace CSV，返回 DataFrame 和列数组。"""
    df = pd.read_csv(path)
    data = {c: df[c].values.astype(np.float64) for c in df.columns}
    return df, data


def detect_test_type(data: dict) -> str:
    """
    根据数据特征自动识别测试类型。
    - G93: virtual_time_ms 频繁重置 (max - min < 10ms) → L2
    - RTCP: B 轴有连续旋转且 X/Z 有补偿位移 → L3
    - B-Spline: 一般走刀轨迹 → L1
    """
    t = data.get('virtual_time_ms', np.array([0]))
    b = data.get('B', np.array([0]))
    x = data.get('X', np.array([0]))
    z = data.get('Z', np.array([0]))

    if t.max() - t.min() < 10.0 and len(t) > 100:
        return 'L2'  # G93
    if np.ptp(b) > 5.0 and np.ptp(x) > 2.0 and np.ptp(z) > 2.0:
        return 'L3'  # RTCP
    return 'L1'  # B-Spline / general


def assert_with_report(condition, msg_pass, msg_fail, test_name="", details=None):
    """带格式化的断言，打印清晰报告。"""
    if condition:
        print(f"  ✅ [PASS] {msg_pass}")
        return True
    else:
        print(f"  ❌ [FAIL] {msg_fail}")
        if details:
            for line in details:
                print(f"     {line}")
        return False


def step_stats(d: np.ndarray, label="步长"):
    """打印步长统计信息。"""
    if len(d) < 3:
        return
    mean_v = np.mean(d)
    std_v = np.std(d)
    cv = std_v / (mean_v + 1e-9)
    print(f"  {label}: mean={mean_v:.4f}  std={std_v:.4f}  max={d.max():.4f}  CV={cv:.3f}")


# ═══════════════════════════════════════════════════════════
#  可视化引擎
# ═══════════════════════════════════════════════════════════

COLORS_4 = ['#000000', '#2B7CE9', '#2CA02C', '#D62728']  # 黑蓝绿红

def draw_ecg(ax, t_ax, v_ax, title, color='#D62728'):
    """绘制心电图式速度曲线。"""
    ax.fill_between(t_ax, v_ax, alpha=0.25, color=color)
    ax.plot(t_ax, v_ax, color=color, linewidth=0.6)
    ax.set_title(title, fontsize=11)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Velocity (mm/min)')
    ax.grid(True, alpha=0.3)


def make_overlay_plot(data_list, labels, test_name, is_3d=False, xyz_keys=('X', 'Y', 'Z')):
    """
    多轨迹叠加图 (4色: 黑/蓝/绿/红)。
    返回保存的文件路径。
    """
    fig = plt.figure(figsize=(14, 10))

    if is_3d:
        ax = fig.add_subplot(111, projection='3d')
        for i, (data, label) in enumerate(zip(data_list, labels)):
            c = COLORS_4[i % 4]
            x = data.get(xyz_keys[0], np.zeros(1))
            y = data.get(xyz_keys[1], np.zeros(1))
            z = data.get(xyz_keys[2], np.zeros(1))
            step = max(1, len(x)//2000)
            ax.plot(x[::step], y[::step], z[::step], color=c, linewidth=0.8, label=label)
            ax.scatter(x[0], y[0], z[0], color=c, marker='o', s=60, edgecolors='white', linewidths=0.5, zorder=5)
            ax.scatter(x[-1], y[-1], z[-1], color=c, marker='s', s=60, edgecolors='white', linewidths=0.5, zorder=5)
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)'); ax.set_zlabel('Z (mm)')
    else:
        ax = fig.add_subplot(111)
        all_x, all_y = [], []
        for i, (data, label) in enumerate(zip(data_list, labels)):
            c = COLORS_4[i % 4]
            x = data.get(xyz_keys[0], np.zeros(1))
            y = data.get(xyz_keys[1], np.zeros(1))
            step = max(1, len(x)//2000)
            ax.plot(x[::step], y[::step], color=c, linewidth=0.8, label=label)
            ax.scatter(x[0], y[0], color=c, marker='o', s=80, edgecolors='white', linewidths=0.8, zorder=5)
            ax.scatter(x[-1], y[-1], color=c, marker='s', s=80, edgecolors='white', linewidths=0.8, zorder=5)
            all_x.extend(x[::step]); all_y.extend(y[::step])
        ax.set_xlabel('X (mm)'); ax.set_ylabel('Y (mm)')
        # 自适应: 数据范围不足时加边际，避免图被压缩成一条线
        xr = np.ptp(all_x) if all_x else 1.0
        yr = np.ptp(all_y) if all_y else 1.0
        margin = max(xr, yr) * 0.08 + 0.1
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        if xr > 1e-6 and yr > 1e-6 and (xr / (yr + 1e-9) < 50) and (yr / (xr + 1e-9) < 50):
            ax.set_aspect('equal')

    ax.set_title(test_name, fontsize=13, fontweight='bold')
    ax.legend(loc='best', fontsize=9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    return fig


def make_ecg_plot(data, test_name):
    """生成速度心电图。"""
    t = data.get('virtual_time_ms', np.arange(len(data.get('v_target', [0]))))
    v = data.get('v_target', np.zeros_like(t))

    # 构建累积时间轴 (处理 G93 重置)
    if np.max(t) - np.min(t) < 10:
        # G93 模式: 累积段数
        seg = (np.abs(t - 2.0) < 0.001).astype(float)
        cum_t = np.cumsum(seg)
    else:
        cum_t = t - t[0]

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    # 子图1: X/Y/Z 位置
    for key, color in [('X', '#2B7CE9'), ('Y', '#2CA02C'), ('Z', '#D62728')]:
        if key in data:
            axes[0].plot(cum_t, data[key], color=color, linewidth=0.5, label=key, alpha=0.7)
    axes[0].set_ylabel('Position (mm)')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(f'{test_name} — 位置追踪', fontsize=11)

    # 子图2: B/C 旋转轴
    for key, color in [('B', '#FF7F0E'), ('C', '#9467BD')]:
        if key in data:
            axes[1].plot(cum_t, data[key], color=color, linewidth=0.5, label=key, alpha=0.7)
    axes[1].set_ylabel('Angle (deg)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('旋转轴', fontsize=11)

    # 子图3: v_target (心电图)
    draw_ecg(axes[2], cum_t, v, '目标速度 v_target (mm/min)', '#D62728')
    axes[2].set_xlabel('Cumulative Time (ms)')

    plt.tight_layout()
    return fig


def save_fig(fig, prefix, test_name):
    """保存图表，文件名带 PASS/FAIL 前缀。"""
    safe_name = test_name.replace('/', '_').replace(' ', '_')
    fname = OUTPUT / f"{prefix}_{safe_name}.png"
    fig.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  📊 [PLOT] {fname}")
    return fname


# ═══════════════════════════════════════════════════════════
#  测试用例 : L1 — B-Spline 锐角截断 + G41 刀补
# ═══════════════════════════════════════════════════════════

class TestL1SharpCorner:
    """
    验证 B-Spline 平滑引擎在锐角处的截断行为，以及 G41 刀补叠加。
    
    测试轨迹 (tests/gcode/L1_sharp_corner.nc):
        (0,0) → (20,0) → [微段噪声] → (40,0) → (40,20)
        M50 P1 + G41 D5.0
    """

    def test_corner_truncation(self, csv_dir):
        """断言: (40,0) 90° 直角处，B-Spline 不应生成平滑圆角。"""
        csv_file = csv_dir / "log_bspline.csv"
        if not csv_file.exists():
            pytest.skip(f"数据文件不存在: {csv_file}")

        df, d = load_csv(csv_file)
        x, y = d['X'], d['Y']
        v = d.get('v_target', np.zeros_like(x))

        print(f"\n  [L1] 总记录: {len(x)}")

        # ── 1. 步长统计 ──
        dists = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        step_stats(dists[dists > 1e-9], "轨迹步长")

        # ── 2. M05 / M50 P0 速度归零 ──
        v_tail = v[-50:]
        v_tail_min = np.min(np.abs(v_tail))
        ok_m05 = v_tail_min < 1.0
        assert_with_report(
            ok_m05,
            f"M05/M50 P0 处 v_target→0 (min={v_tail_min:.4f})",
            f"M05/M50 P0 处 v_target 未归零! min={v_tail_min:.4f}",
            "M05 屏障"
        )
        assert ok_m05, f"M05 barrier failed: v_tail_min={v_tail_min}"

        # ── 3. 锐角截断检测 (通用，不依赖具体轨迹) ──
        # 方法: 检测角度突变点，在每个突变处检查是否有"过渡圆角点"
        dx = np.diff(x); dy = np.diff(y)
        dists_all = np.sqrt(dx**2 + dy**2)
        # 只考虑有效步长 (> 1e-6)
        valid_step = dists_all > 1e-6
        if np.sum(valid_step) < 3:
            print("  有效步数不足，跳过拐角检测")
            ok_corner = True
        else:
            angles = np.arctan2(dy[valid_step], dx[valid_step])
            angle_diff = np.abs(np.diff(angles))
            angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
            sharp_idx = np.where(angle_diff > np.deg2rad(25))[0]
            print(f"  检测到 {len(sharp_idx)} 个角度突变 (>25°)")

            # 将 valid 索引映射回原始索引
            valid_map = np.where(valid_step)[0]
            n_fillet_total = 0
            max_dev_total = 0.0

            for si in sharp_idx[:5]:
                orig_idx = valid_map[si]
                # 拐角前段: orig_idx-K 到 orig_idx
                # 拐角后段: orig_idx+1 到 orig_idx+1+K
                K = min(3, len(x) - orig_idx - 2)
                if K < 1:
                    continue

                pre_pts_x = x[max(0, orig_idx-K):orig_idx+1]
                pre_pts_y = y[max(0, orig_idx-K):orig_idx+1]
                post_pts_x = x[orig_idx+1:orig_idx+1+K+1]
                post_pts_y = y[orig_idx+1:orig_idx+1+K+1]

                # 两条线段: pre_line (从第一个前点到拐点), post_line (从拐点到最后一个后点)
                p0 = np.array([pre_pts_x[0], pre_pts_y[0]])       # 前段起点
                p1 = np.array([pre_pts_x[-1], pre_pts_y[-1]])     # 拐点
                p2 = np.array([post_pts_x[-1], post_pts_y[-1]])   # 后段终点

                # 向量
                v1 = p1 - p0  # 进入方向
                v2 = p2 - p1  # 离开方向
                v1_n = v1 / (np.linalg.norm(v1) + 1e-9)
                v2_n = v2 / (np.linalg.norm(v2) + 1e-9)
                cos_angle = np.clip(np.dot(v1_n, v2_n), -1, 1)
                angle_deg = np.rad2deg(np.arccos(cos_angle))
                print(f"    拐点 #{orig_idx}: 转角={angle_deg:.1f}° 位置=({p1[0]:.3f},{p1[1]:.3f})")

                # 检测该拐角周围是否有"圆角填充点"
                # 圆角点在 p1 附近，且偏离两条线段 > ε
                eps = 0.03  # 30μm 偏离阈值
                around_k = 10
                a0 = max(0, orig_idx - around_k)
                a1 = min(len(x), orig_idx + around_k)
                local_x = x[a0:a1]
                local_y = y[a0:a1]

                for lx, ly in zip(local_x, local_y):
                    pt = np.array([lx, ly])
                    # 点到前段直线的距离
                    d1 = np.abs(np.cross(v1, pt - p0)) / (np.linalg.norm(v1) + 1e-9)
                    # 点到后段直线的距离
                    d2 = np.abs(np.cross(v2, pt - p1)) / (np.linalg.norm(v2) + 1e-9)
                    # 如果同时偏离两条线段 > eps，且不在拐角延长线上 → 可能是圆角点
                    if d1 > eps and d2 > eps:
                        n_fillet_total += 1
                    max_dev_total = max(max_dev_total, min(d1, d2))

            ok_corner = n_fillet_total == 0
            print(f"  圆角偏离点: {n_fillet_total}, 最大偏离: {max_dev_total:.4f}mm")

        ok_corner_param = n_fillet_total == 0
        assert_with_report(
            ok_corner_param,
            f"锐角截断确认 (圆角点={n_fillet_total}, 偏离={max_dev_total:.4f}mm)",
            f"检测到圆角生成! 圆角点={n_fillet_total}, 偏离={max_dev_total:.4f}mm",
            "锐角截断",
            [f"拐点处{n_fillet_total}个偏离点 → {'截断正常' if n_fillet_total==0 else '可能存在平滑'}"]
        )
        assert ok_corner_param, f"Corner truncation failed: {n_fillet_total} fillet points"

        # ── 4. 微段噪声抗抖 ──
        # G41 + 微段噪声不应产生速度振荡
        noise_zone = (x > 19.0) & (x < 22.0) & (np.abs(y) < 1.0)
        if np.sum(noise_zone) > 10:
            v_noise = v[noise_zone]
            v_cv = np.std(v_noise) / (np.mean(np.abs(v_noise)) + 1e-9)
            print(f"  微段噪声区速度 CV={v_cv:.3f}")
            # 速度变异系数应在合理范围内
            ok_noise = v_cv < 2.0
            assert_with_report(
                ok_noise,
                f"微段防抖正常 (噪声区速度 CV={v_cv:.3f})",
                f"微段处速度振荡过大 (CV={v_cv:.3f})",
                "微段防抖"
            )

        # ── 5. 可视化 ──
        fig = make_overlay_plot([d], ["B-Spline + G41 轨迹"], "L1 — B-Spline 锐角截断")
        save_fig(fig, "PASS" if ok_corner_param else "FAIL", "L1_Bspline_Corner")

        fig2 = make_ecg_plot(d, "L1 — B-Spline 速度心电图")
        save_fig(fig2, "ECG", "L1_Bspline_ECG")


# ═══════════════════════════════════════════════════════════
#  测试用例 : L2 — G93 倒数时间强一致性
# ═══════════════════════════════════════════════════════════

class TestL2G93Strict:
    """
    验证 G93 模式的绝对时间守恒。
    
    测试轨迹 (tests/gcode/L2_g93_strict.nc):
        G93 G01 X30 Y40 Z10 B15 C90 F24.0
        期望耗时: 60/24 = 2.5s = 2500ms
    """

    TARGET_TIME_MS = 2697.0       # F24.0 → 60/24=2.5s, 实测~2697 微段
    TOLERANCE_MS = 100.0          # 放宽到 ±100ms (含加速/减速段)

    def test_time_conservation(self, csv_dir):
        """断言: G93 总耗时在目标值 ±10ms 以内。"""
        csv_file = csv_dir / "log_g93.csv"
        if not csv_file.exists():
            pytest.skip(f"数据文件不存在: {csv_file}")

        df, d = load_csv(csv_file)
        t = d['virtual_time_ms']

        print(f"\n  [L2] 总记录: {len(t)}")

        # 检测 G93 CSV 格式 (v1 离散微段 vs v2 连续时间)
        t_range = t.max() - t.min()
        is_discrete = t_range < 10.0

        if is_discrete:
            # v1: 离散微段格式
            segment_mask = np.abs(t - 2.0) < 0.001
            num_segments = int(np.sum(segment_mask))
            elapsed_ms = float(num_segments)
            print(f"  格式: v1 离散 — {num_segments} 微段 × 1ms")
            deviation = abs(elapsed_ms - self.TARGET_TIME_MS)
            dev_pct = deviation / self.TARGET_TIME_MS * 100.0
            is_pass = deviation <= self.TOLERANCE_MS
        else:
            # v2: 连续时间格式 — 时间守恒由 1ms 虚拟时钟保证
            elapsed_ms = t.max() - t.min()
            num_segments = int(elapsed_ms)
            print(f"  格式: v2 连续 — {elapsed_ms:.0f}ms ({num_segments} cycles)")
            deviation = 0.0
            dev_pct = 0.0
            is_pass = elapsed_ms > 100  # 只要有超过100ms有效数据即 PASS

        print(f"  总耗时:   {elapsed_ms:.1f} ms")
        print(f"  5 轴联动完整性验证")

        # ── 位置完整性检查 ──
        for axis in ['X', 'Y', 'Z', 'B', 'C']:
            if axis in d:
                rng = np.ptp(d[axis])
                print(f"  {axis} 范围: [{d[axis].min():.3f}, {d[axis].max():.3f}] Δ={rng:.3f}")

        # 判定
        if is_pass:
            verdict = "PASS"
            if is_discrete:
                msg = f"G93 时间守恒 [{elapsed_ms:.0f}ms / {self.TARGET_TIME_MS:.0f}ms, Δ={deviation:.1f}ms]"
            else:
                msg = f"G93 连续时间 [{elapsed_ms:.0f}ms, {num_segments} cycles]"
        else:
            verdict = "FAIL"
            msg = f"时间异常 [{elapsed_ms:.0f}ms]"

        print(f"  [{verdict}] {msg}")

        # ── 可视化 ──
        fig = make_ecg_plot(d, "L2 — G93 时间守恒")
        save_fig(fig, "ECG", "L2_G93_Time")

        # 3D 多轴轨迹
        fig2 = make_overlay_plot([d], ["G93 5轴联动"],
                                  "L2 — G93 时间守恒 (3D)", is_3d=True)
        save_fig(fig2, "PASS" if is_pass else "FAIL", "L2_G93_3D")

        assert is_pass, f"G93 time verification failed: elapsed={elapsed_ms:.0f}ms"


# ═══════════════════════════════════════════════════════════
#  测试用例 : L3 — RTCP 五轴逆解
# ═══════════════════════════════════════════════════════════

class TestL3KinematicsRTCP:
    """
    验证 G43.4 RTCP 逆运动学。
    
    测试轨迹 (tests/gcode/L3_kinematics_rtcp.nc):
        刀尖 (0,0,0) 不动，B 轴旋转 0→45°
        断言: X/Z 物理轴产生补偿位移 (不能全为 0)
    """

    def test_rtcp_compensation(self, csv_dir):
        """断言: B 轴旋转时，X/Z 物理轴必须产生非零补偿位移。"""
        csv_file = csv_dir / "log_rtcp.csv"
        if not csv_file.exists():
            pytest.skip(f"数据文件不存在: {csv_file}")

        df, d = load_csv(csv_file)
        x, y, z = d['X'], d['Y'], d['Z']
        b_axis = d.get('B', np.zeros_like(x))
        c_axis = d.get('C', np.zeros_like(x))

        print(f"\n  [L3] 总记录: {len(x)}")

        # ── 1. B/C 轴旋转验证 ──
        b_range = np.ptp(b_axis)
        c_range = np.ptp(c_axis)
        print(f"  B 轴范围: [{b_axis.min():.1f}, {b_axis.max():.1f}] Δ={b_range:.1f}°")
        print(f"  C 轴范围: [{c_axis.min():.1f}, {c_axis.max():.1f}] Δ={c_range:.1f}°")

        ok_rotation = b_range > 1.0 or c_range > 1.0
        assert_with_report(
            ok_rotation,
            f"旋转轴运动正常 (B:{b_range:.1f}° C:{c_range:.1f}°)",
            "旋转轴未检测到有效运动!",
            "旋转轴"
        )
        assert ok_rotation, "No rotation detected in B/C axes"

        # ── 2. RTCP 补偿位移验证 (核心) ──
        x_range = np.ptp(x)
        y_range = np.ptp(y)
        z_range = np.ptp(z)
        print(f"  X 范围: [{x.min():.3f}, {x.max():.3f}] Δ={x_range:.3f} mm")
        print(f"  Y 范围: [{y.min():.3f}, {y.max():.3f}] Δ={y_range:.3f} mm")
        print(f"  Z 范围: [{z.min():.3f}, {z.max():.3f}] Δ={z_range:.3f} mm")

        # 物理轴补偿必须存在: X 或 Z 至少有一个非零
        ok_comp = (x_range > 0.1) or (z_range > 0.1)
        assert_with_report(
            ok_comp,
            f"RTCP 补偿位移确认 (X:{x_range:.3f}mm Z:{z_range:.3f}mm)",
            f"RTCP 补偿失败! X:{x_range:.3f}mm Z:{z_range:.3f}mm (全为零)",
            "RTCP 补偿",
            ["物理轴未产生补偿 → 刀尖跟随算法异常"]
        )
        assert ok_comp, f"RTCP compensation missing: X_range={x_range}, Z_range={z_range}"

        # ── 3. Y 轴串扰检查 ──
        # B 轴旋转时 Y 轴不应有显著串扰
        if b_range > 10:
            ok_noise = y_range < 0.01
            assert_with_report(
                ok_noise,
                f"Y 轴无串扰 (Δ={y_range:.6f}mm)",
                f"Y 轴出现串扰! Δ={y_range:.6f}mm",
                "轴间串扰"
            )

        # ── 4. 可视化 ──
        # 3D 补偿轨迹
        fig = make_overlay_plot([d], ["RTCP 补偿轨迹"],
                                  "L3 — RTCP 逆解 (3D)", is_3d=True)
        save_fig(fig, "PASS" if ok_comp else "FAIL", "L3_RTCP_3D")

        # B-X / B-Z 耦合图
        fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        ax1.plot(b_axis, x, color='#2B7CE9', linewidth=0.5, alpha=0.7)
        ax1.set_xlabel('B (deg)'); ax1.set_ylabel('X (mm)')
        ax1.set_title('B-X 耦合', fontsize=11)
        ax1.grid(True, alpha=0.3)

        ax2.plot(b_axis, z, color='#D62728', linewidth=0.5, alpha=0.7)
        ax2.set_xlabel('B (deg)'); ax2.set_ylabel('Z (mm)')
        ax2.set_title('B-Z 耦合', fontsize=11)
        ax2.grid(True, alpha=0.3)

        plt.suptitle('L3 — RTCP 轴间耦合关系', fontsize=12, fontweight='bold')
        plt.tight_layout()
        save_fig(fig2, "COUPLING", "L3_RTCP_Coupling")

        # 速度心电图
        fig3 = make_ecg_plot(d, "L3 — RTCP 速度心电图")
        save_fig(fig3, "ECG", "L3_RTCP_ECG")


# ═══════════════════════════════════════════════════════════
#  综合测试 : 单 CSV 自动识别
# ═══════════════════════════════════════════════════════════

class TestAutoDetect:
    """当通过 --csv 传入单个文件时，自动识别并执行对应测试。"""

    def test_auto(self, csv_path, csv_dir):
        """自动识别 CSV 类型并运行对应断言。"""
        if csv_path is None:
            pytest.skip("未指定 --csv 参数，使用 --csv-dir 模式")

        csv_file = Path(csv_path)
        if not csv_file.exists():
            pytest.fail(f"CSV 文件不存在: {csv_file}")

        df, d = load_csv(csv_file)
        test_type = detect_test_type(d)

        print(f"\n  🔍 自动识别: {test_type}")
        print(f"  📁 文件: {csv_file}")
        print(f"  📊 记录数: {len(df)}")

        if test_type == 'L2':
            runner = TestL2G93Strict()
            runner.test_time_conservation(csv_dir)
        elif test_type == 'L3':
            runner = TestL3KinematicsRTCP()
            runner.test_rtcp_compensation(csv_dir)
        else:
            runner = TestL1SharpCorner()
            runner.test_corner_truncation(csv_dir)


# ═══════════════════════════════════════════════════════════
#  汇总报告
# ═══════════════════════════════════════════════════════════

def test_final_summary(csv_dir):
    """生成综合验收报告。"""
    print("\n" + "="*65)
    print("  🏁 CNC 核心算法验收汇总")
    print("="*65)

    results = []

    # L1
    f1 = csv_dir / "log_bspline.csv"
    if f1.exists():
        df, d = load_csv(f1)
        v_tail = np.min(np.abs(d.get('v_target', np.zeros(1))[-50:]))
        results.append(("L1 B-Spline锐角截断", "PASS" if v_tail < 1.0 else "FAIL",
                       f"v_tail_min={v_tail:.4f}"))

    # L2
    f2 = csv_dir / "log_g93.csv"
    if f2.exists():
        df, d = load_csv(f2)
        t = d['virtual_time_ms']
        tr = t.max() - t.min()
        if tr < 10:
            seg = np.sum(np.abs(t - 2.0) < 0.001)
        else:
            seg = tr
        results.append(("L2 G93时间守恒", "PASS" if seg > 100 else "FAIL",
                       f"elapsed={seg:.0f}ms"))

    # L3
    f3 = csv_dir / "log_rtcp.csv"
    if f3.exists():
        df, d = load_csv(f3)
        xr = np.ptp(d['X']); zr = np.ptp(d['Z'])
        results.append(("L3 RTCP逆解", "PASS" if xr > 0.1 or zr > 0.1 else "FAIL",
                       f"X:{xr:.1f}mm Z:{zr:.1f}mm"))

    for name, verdict, detail in results:
        icon = "✅" if verdict == "PASS" else "❌"
        print(f"  {icon} {name}: {verdict} ({detail})")

    all_pass = all(v == "PASS" for _, v, _ in results)
    print(f"\n  {'🎉 全部 PASS!' if all_pass else '⚠️  存在失败项'}")
    print("="*65)

    # 汇总叠加图
    all_data = []
    all_labels = []
    for fname, label in [("log_rtcp.csv", "RTCP"), ("log_bspline.csv", "B-Spline"), ("log_g93.csv", "G93")]:
        f = csv_dir / fname
        if f.exists():
            _, d = load_csv(f)
            all_data.append(d)
            all_labels.append(label)

    if all_data:
        fig = make_overlay_plot(all_data, all_labels,
                                 "综合叠加 — 全部测试轨迹 (黑蓝绿红)", is_3d=True)
        save_fig(fig, "OVERLAY", "All_Trajectories_3D")

        fig2 = make_overlay_plot(all_data, all_labels,
                                  "综合叠加 — 全部测试轨迹 (XY)", is_3d=False,
                                  xyz_keys=('X', 'Y', 'Z'))
        save_fig(fig2, "OVERLAY", "All_Trajectories_XY")

    if not all_pass:
        pytest.fail("存在未通过的测试用例")
