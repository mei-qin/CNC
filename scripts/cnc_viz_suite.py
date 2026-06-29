#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CNC 多级管线可视化仪表盘 (cnc_viz_suite.py)
================================================
读取 C 端 trace_logger / sim_engine 输出的多级 CSV 数据,
按 stage_id 拆分为 parser / cutter_comp / bspline / rt 四条数据流,
生成三大可视化仪表盘:

  View 1  Spatial Overlay   全管线空间叠加图
  View 2  Dynamics EKG      七段式心电图 (v / a / jerk 三连图)
  View 3  Error Heatmap     误差热力图 (散点 + Colormap)

CSV schema (与 trace_logger.c 一致):
    cycle, stage_id, virtual_time_ms, x_mm, y_mm, z_mm, b_deg, c_deg, v_target

sim_engine CSV schema (与 sim_engine.c 一致, 列名按轴动态):
    cycle, stage_id, virtual_time_ms, <axis_name...>, v_target
本模块统一归一化为 (x, y, z, b, c) 命名, 兼容两种 schema。
"""

import sys, io, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (register 3d projection)
from scipy.signal import savgol_filter

# 脚本路径常量 (脚本位于 scripts/, 输出落盘到 tests/output/viz_dashboard/)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if not hasattr(sys, 'frozen') else os.path.dirname(sys.executable)
VIZ_OUT_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "tests", "output", "viz_dashboard"))

# 中文字体支持
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'STSong']
matplotlib.rcParams['axes.unicode_minus'] = False

# ==================== Stage 常量 (与 trace_logger.h 一致) ====================
STAGE_PARSER          = 1
STAGE_CUTTER_COMP     = 2
STAGE_BSPLINE         = 3
STAGE_PLANNER         = 4
STAGE_RT_INTERPOLATOR = 5

STAGE_NAMES = {
    STAGE_PARSER:          'Parser',
    STAGE_CUTTER_COMP:     'CutterComp',
    STAGE_BSPLINE:         'BSpline',
    STAGE_PLANNER:         'Planner',
    STAGE_RT_INTERPOLATOR: 'RT_Interp',
}

# ==================== 数据加载与清洗 ====================

# CSV 中各轴的候选列名 (兼容 trace_logger 与 sim_engine 两种 schema)
AXIS_COLUMN_CANDIDATES = {
    'x': ['x_mm', 'X'],
    'y': ['y_mm', 'Y'],
    'z': ['z_mm', 'Z'],
    'b': ['b_deg', 'B'],
    'c': ['c_deg', 'C'],
}


def _pick_column(df, candidates, default=0.0):
    """从候选列名中挑选第一个存在的列, 找不到则填充默认值。"""
    for col in candidates:
        if col in df.columns:
            return df[col].to_numpy(dtype=float)
    return np.full(len(df), default, dtype=float)


def load_pipeline_csv(csv_path):
    """
    加载多级管线 CSV, 归一化为统一列名, 并按 stage_id 拆分。

    返回 dict:
        {
          'parser':  DataFrame  (stage_id == 1)
          'comp':    DataFrame  (stage_id == 2)
          'bspline': DataFrame  (stage_id == 3)
          'planner': DataFrame  (stage_id == 4)
          'rt':      DataFrame  (stage_id == 5)
          'all':     DataFrame  (全部)
        }
    每个 DataFrame 至少含列: cycle, stage_id, virtual_time_ms,
                            x, y, z, b, c, v_target
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV 文件不存在: {csv_path}")

    df = pd.read_csv(csv_path)

    # 兼容老 CSV (无 stage_id 列): 全部归为 STAGE_RT_INTERPOLATOR
    if 'stage_id' not in df.columns:
        print(f"[Viz] 警告: {csv_path} 缺 stage_id 列, 默认全部归为 RT_Interp (5)")
        df['stage_id'] = STAGE_RT_INTERPOLATOR

    # 归一化轴列
    normalized = pd.DataFrame()
    normalized['cycle'] = df['cycle'] if 'cycle' in df.columns else np.arange(len(df))
    normalized['stage_id'] = df['stage_id'].astype(int)
    normalized['virtual_time_ms'] = df['virtual_time_ms'] if 'virtual_time_ms' in df.columns else 0.0
    for axis_key, candidates in AXIS_COLUMN_CANDIDATES.items():
        normalized[axis_key] = _pick_column(df, candidates)
    # v_target 兼容 v_current 等命名
    v_col = None
    for cand in ['v_target', 'v_current', 'v_mm_per_ms']:
        if cand in df.columns:
            v_col = cand
            break
    normalized['v_target'] = df[v_col].to_numpy(dtype=float) if v_col else 0.0

    # 审计修复 #3: 落盘线程以 drain 顺序串行 fwrite RT SPSC + Pipeline MPMC,
    # CSV 中 cycle 列在不同 stage 间必然乱序 (parser seq 0..N 后才轮到 RT cycle 1e6..)。
    # 必须按 stage_id + cycle 排序, 否则 EKG 时间轴会因 stage 切换而横跳。
    normalized = normalized.sort_values(
        ['stage_id', 'cycle'], kind='mergesort'
    ).reset_index(drop=True)

    # 按 stage 拆分
    result = {'all': normalized}
    for sid, name in STAGE_NAMES.items():
        sub = normalized[normalized['stage_id'] == sid].reset_index(drop=True)
        result[name.lower()] = sub
    # 友好别名
    result['parser']  = result['parser']  if 'parser'  in result else pd.DataFrame()
    result['comp']    = result['cuttercomp'] if 'cuttercomp' in result else pd.DataFrame()
    result['bspline'] = result['bspline'] if 'bspline' in result else pd.DataFrame()
    result['planner'] = result['planner'] if 'planner' in result else pd.DataFrame()
    result['rt']      = result['rt_interp']  if 'rt_interp'  in result else pd.DataFrame()

    return result


# ==================== RTCP 检测与正运动学 (Forward Kinematics) ====================
#
# 关键架构事实 (核实 gcode_parser.c:322 + apply_rtcp_to_pos + ecat_core.c):
#   - STAGE_PARSER 记录 machine_target_pos = 逻辑刀尖坐标 (machine frame)
#       例: G43.4 模式下, X/Y/Z 是 WORK PIECE 上刀尖应当到达的位置
#   - STAGE_RT_INTERPOLATOR 记录 g_interpolator.current_pos = 物理关节坐标
#       经过 Kinematics_Inverse 变换后的 X/Y/Z 关节位移 (用于补偿 B/C 旋转)
#
# 当 RTCP 开启 (G43.4) 时, df_parser 与 df_rt 处于不同的坐标空间:
#   - 逻辑刀尖可能近似静止 (X/Y/Z 微动)
#   - 物理关节剧烈运动以维持刀尖位置
# 直接对二者计算欧氏距离会得到荒诞的大误差 (数百 mm 量级)。
#
# RTCP 检测启发式:
#   1. 显式: CLI flag --assume-rtcp / env CNC_ASSUME_RTCP
#   2. 隐式: df_parser 的 B 或 C 轴范围 > RTCP_B_RANGE_THRESH (默认 0.5 deg)
#   3. 验证: 同时检测到 df_rt 的 X/Y/Z 与 df_parser 偏离 > RTCP_COMP_PROOF
#       (物理补偿确实存在)

RTCP_B_RANGE_THRESH_DEG = 0.5    # B/C 轴变化阈值
RTCP_COMP_PROOF_MM      = 1.0    # parser vs rt XYZ 偏离证据阈值


def detect_rtcp(data, assume_rtcp=False):
    """
    检测当前轨迹是否使用了 RTCP (G43.4 刀尖跟随)。

    返回 dict: {
      'rtcp_on': bool,
      'reason': str,        # 检测路径说明
      'b_range_deg': float, # df_parser 的 B 轴变化幅度
      'c_range_deg': float, # df_parser 的 C 轴变化幅度
    }
    """
    df_parser = data.get('parser', pd.DataFrame())

    b_range = float(df_parser['b'].max() - df_parser['b'].min()) if len(df_parser) else 0.0
    c_range = float(df_parser['c'].max() - df_parser['c'].min()) if len(df_parser) else 0.0

    if assume_rtcp:
        return {'rtcp_on': True,
                'reason': '强制 --assume-rtcp',
                'b_range_deg': b_range,
                'c_range_deg': c_range}

    if b_range > RTCP_B_RANGE_THRESH_DEG or c_range > RTCP_B_RANGE_THRESH_DEG:
        return {'rtcp_on': True,
                'reason': f'parser B/C 轴有显著变化 (B={b_range:.2f}deg, '
                          f'C={c_range:.2f}deg > {RTCP_B_RANGE_THRESH_DEG}deg)',
                'b_range_deg': b_range,
                'c_range_deg': c_range}

    return {'rtcp_on': False,
            'reason': 'parser B/C 无显著变化',
            'b_range_deg': b_range,
            'c_range_deg': c_range}


def forward_kinematics_tip(rt_pts, kin_config=None):
    """
    将物理关节坐标 (X_joint, Y_joint, Z_joint, B, C) 还原为逻辑刀尖坐标。

    参数:
      rt_pts:     (N, 5) 数组, 列序 = [x, y, z, b, c]
      kin_config: dict, 运动学配置。None = 恒等变换 (不还原, 仅作占位)。
                  必须包含: type ('HEAD_HEAD'/'TABLE_TABLE'/'MIXED'),
                            rot_1_axis, rot_2_axis, tool_offset[3], pivot_offset[3]

    返回:
      (N, 3) 刀尖 XYZ 坐标

    ⚠️ 当前为占位实现: 仅当 kin_config=None 时返回原始 XYZ (假设无 RTCP)。
    完整 FK 需依据 kinematics.c 的 T_head/T_table 装配实现, 与机床构型耦合,
    建议通过独立 YAML/JSON 配置文件提供 kin_config, 避免硬编码。
    """
    if kin_config is None:
        # 恒等变换: 不做 IK 反演, 直接返回 X/Y/Z
        # 调用方应仅在 RTCP OFF 时使用本函数 (此时 joint == tip)
        return rt_pts[:, :3].copy()

    # TODO: 完整 FK 实现, 依据 kinematics.c 的 T_head/T_table 模型
    raise NotImplementedError(
        "forward_kinematics_tip 完整 FK 实现需 kin_config (机床构型 + tool/pivot offset)。"
        "当前仅支持 kin_config=None (恒等变换, RTCP OFF 场景)。"
    )


# ==================== 几何工具: 点到折线最短距离 ====================

def point_to_segment_dist(p, a, b):
    """
    批量计算点 p 到线段 (a -> b) 的最短欧氏距离。
    p: (N, D), a: (M, D), b: (M, D)
    返回: (N,) 每个点到所有 M 条线段的最小距离。

    采用广播避免双重循环, 适合 N, M ~ 1e4 量级。
    """
    # ab: (M, D)
    ab = b - a
    ab_sq = np.sum(ab * ab, axis=1)            # (M,)
    ab_sq_safe = np.where(ab_sq < 1e-18, 1e-18, ab_sq)

    # 对每个点 p_i: 计算其在每条线段上的投影参数 t
    # ap: (N, M, D) = p_i - a_j
    ap = p[:, None, :] - a[None, :, :]
    t = np.sum(ap * ab[None, :, :], axis=2) / ab_sq_safe[None, :]  # (N, M)
    t = np.clip(t, 0.0, 1.0)

    # 投影点 projection: (N, M, D)
    proj = a[None, :, :] + t[..., None] * ab[None, :, :]
    diff = p[:, None, :] - proj                 # (N, M, D)
    dist = np.sqrt(np.sum(diff * diff, axis=2)) # (N, M)
    return dist.min(axis=1)                     # (N,)


def path_following_error(rt_points, parser_points):
    """
    计算每个 RT 实际执行点到 Parser 原始意图折线的最短距离。
    rt_points:     (N, D) 实际物理执行点
    parser_points: (M, D) 原始意图点
    返回: (N,) 每点的路径跟随误差 (mm)
    """
    if len(rt_points) == 0:
        return np.array([])
    if len(parser_points) < 2:
        # 仅一个意图点: 直接到该点的距离
        return np.linalg.norm(rt_points - parser_points[0:1], axis=1)
    a = parser_points[:-1]
    b = parser_points[1:]
    return point_to_segment_dist(rt_points, a, b)


# ==================== 动力学后处理 (savgol 平滑 + 数值求导) ====================

def compute_dynamics(time_ms, v_mm_per_ms,
                     savgol_window=11, savgol_poly=3):
    """
    从 v_target (mm/ms) 数组计算平滑速度 / 加速度 / jerk。

    步骤:
      1. 将 time_ms 转为秒
      2. savgol_filter 平滑 v (避免量化噪声在求导时放大)
      3. np.gradient 一阶求导 → a (mm/s^2)
      4. np.gradient 二阶求导 → jerk (mm/s^3)

    返回 dict: {t_s, v_smooth, a, jerk}
    """
    n = len(time_ms)
    if n < 5:
        return {'t_s': time_ms / 1000.0,
                'v_smooth': v_mm_per_ms * 1000.0,
                'a': np.zeros(n),
                'jerk': np.zeros(n)}

    t_s = np.asarray(time_ms, dtype=float) / 1000.0
    v_mm_per_s = np.asarray(v_mm_per_ms, dtype=float) * 1000.0

    # savgol 窗口必须为奇数且 <= 数据长度
    w = min(savgol_window, n if n % 2 == 1 else n - 1)
    if w < savgol_poly + 2:
        w = max(savgol_poly + 2, 5)
        if w % 2 == 0:
            w += 1
        w = min(w, n if n % 2 == 1 else n - 1)
    p = min(savgol_poly, w - 2)

    v_smooth = savgol_filter(v_mm_per_s, w, p)
    a = np.gradient(v_smooth, t_s)
    jerk = np.gradient(a, t_s)

    return {'t_s': t_s, 'v_smooth': v_smooth, 'a': a, 'jerk': jerk}


# ==================== View 1: 全管线空间叠加图 ====================

def render_spatial_overlay(data, output_path='viz_spatial_overlay.png',
                           use_3d=True, title_suffix=''):
    """
    View 1: 全管线空间叠加图
      - df_parser  : 黑色虚线 (原始意图)
      - df_comp    : 蓝色实线 (刀补后)
      - df_bspline : 绿色实线 (B 样条平滑后)
      - df_rt      : 红色散点 (1ms 物理执行)
    """
    df_parser  = data.get('parser',  pd.DataFrame())
    df_comp    = data.get('comp',    pd.DataFrame())
    df_bspline = data.get('bspline', pd.DataFrame())
    df_rt      = data.get('rt',      pd.DataFrame())

    fig = plt.figure(figsize=(13, 10))
    if use_3d:
        ax = fig.add_subplot(111, projection='3d')
        plot_fn = lambda x, y, z, *a, **k: ax.plot(x, y, z, *a, **k)
        scatter_fn = lambda x, y, z, *a, **k: ax.scatter(x, y, z, *a, **k)
        ax.set_zlabel('Z (mm)')
    else:
        ax = fig.add_subplot(111)
        plot_fn = lambda x, y, z, *a, **k: ax.plot(x, y, *a, **k)
        scatter_fn = lambda x, y, z, *a, **k: ax.scatter(x, y, *a, **k)
        ax.set_ylabel('Y (mm)')
    ax.set_xlabel('X (mm)')

    if len(df_parser) >= 2:
        plot_fn(df_parser['x'], df_parser['y'], df_parser['z'],
                'k--', linewidth=1.6, label=f'Parser (intent, N={len(df_parser)})')
    if len(df_comp) >= 2:
        plot_fn(df_comp['x'], df_comp['y'], df_comp['z'],
                'b-', linewidth=1.2, alpha=0.85,
                label=f'CutterComp (N={len(df_comp)})')
    if len(df_bspline) >= 2:
        plot_fn(df_bspline['x'], df_bspline['y'], df_bspline['z'],
                'g-', linewidth=1.0, alpha=0.7,
                label=f'BSpline (N={len(df_bspline)})')
    if len(df_rt) > 0:
        # RT 数据通常很大, 散点降采样到 2000 点
        step = max(1, len(df_rt) // 2000)
        scatter_fn(df_rt['x'][::step], df_rt['y'][::step], df_rt['z'][::step],
                   c='red', s=4, alpha=0.6,
                   label=f'RT_Interp (N={len(df_rt)}, step={step})')

    ax.set_title(f'CNC Pipeline Spatial Overlay{title_suffix}')
    ax.legend(loc='best', fontsize=9)
    if not use_3d:
        ax.axis('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close()
    print(f"  [PLOT] {output_path}")
    return output_path


# ==================== View 2: 七段式心电图 ====================

def render_dynamics_ekg(df_rt, a_max=200.0, jerk_max=5000.0,
                        savgol_window=5, savgol_poly=2,
                        output_path='viz_dynamics_ekg.png',
                        title_suffix=''):
    """
    View 2: 动力学心电图
      上图: v_current (mm/s)            阈值 = max(v_target) * 1.05
      中图: a_actual   (mm/s^2)         阈值 = a_max * 1.05 (红线)
      下图: jerk_actual (mm/s^3)        阈值 = jerk_max * 1.05 (红线)

    savgol 平滑参数 (审计修复 #2):
      window=5 (严格, 默认): 捕获 ≥3ms 尖刺, 适合动力学断言
      window=11 (宽松):     抹平 ≤5ms 尖刺, 仅适合趋势观察

    时间轴使用 cycle (1ms/cycle) 转为秒。
    """
    min_rows = max(savgol_window, 5)
    if len(df_rt) < min_rows:
        print(f"  [SKIP] RT 数据不足 ({len(df_rt)} 行 < {min_rows}), 跳过 EKG")
        return None

    # 用 cycle (1ms/cycle) 作为时间基, 累计成秒
    # 注意: virtual_time_ms 是段内时间, 会随段切换重置, 不能直接用作全局时间轴
    time_ms = df_rt['cycle'].to_numpy(dtype=float)
    v_mm_per_ms = df_rt['v_target'].to_numpy(dtype=float)

    dyn = compute_dynamics(time_ms, v_mm_per_ms,
                           savgol_window=savgol_window, savgol_poly=savgol_poly)
    t_s = dyn['t_s']
    v_smooth = dyn['v_smooth']
    a = dyn['a']
    jerk = dyn['jerk']

    v_thresh = np.nanmax(v_smooth) * 1.05 if len(v_smooth) else 0.0
    a_thresh = a_max * 1.05
    jerk_thresh = jerk_max * 1.05

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)

    axes[0].plot(t_s, v_smooth, 'b-', linewidth=0.9, label='v_current')
    axes[0].axhline(v_thresh, color='r', linestyle='--', linewidth=1.0,
                    label=f'thresh={v_thresh:.1f} mm/s')
    axes[0].set_ylabel('v (mm/s)')
    axes[0].set_title(f'Dynamics EKG — v / a / jerk{title_suffix}')
    axes[0].legend(loc='upper right', fontsize=8)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t_s, a, 'g-', linewidth=0.9, label='a_actual')
    axes[1].axhline(a_thresh, color='r', linestyle='--', linewidth=1.0,
                    label=f'+{a_thresh:.1f}')
    axes[1].axhline(-a_thresh, color='r', linestyle='--', linewidth=1.0,
                    label=f'-{a_thresh:.1f}')
    axes[1].set_ylabel('a (mm/s²)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t_s, jerk, 'm-', linewidth=0.9, label='jerk_actual')
    axes[2].axhline(jerk_thresh, color='r', linestyle='--', linewidth=1.0,
                    label=f'+{jerk_thresh:.0f}')
    axes[2].axhline(-jerk_thresh, color='r', linestyle='--', linewidth=1.0,
                    label=f'-{jerk_thresh:.0f}')
    axes[2].set_ylabel('jerk (mm/s³)')
    axes[2].set_xlabel('Time (s)')
    axes[2].legend(loc='upper right', fontsize=8)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close()
    print(f"  [PLOT] {output_path}")
    return output_path


# ==================== View 3: 误差热力图 ====================

def render_error_heatmap(data, output_path='viz_error_heatmap.png',
                         use_3d=False, title_suffix=''):
    """
    View 3: 误差热力图
      散点图绘制 RT 物理轨迹, 点颜色 = 该点距 Parser 意图折线的最短误差。
      Colormap: jet (蓝=小误差 → 红=大误差)
    """
    df_parser = data.get('parser', pd.DataFrame())
    df_rt     = data.get('rt',     pd.DataFrame())

    if len(df_rt) == 0:
        print(f"  [SKIP] RT 数据为空, 跳过误差热力图")
        return None

    rt_pts = df_rt[['x', 'y', 'z']].to_numpy()
    if len(df_parser) >= 2:
        parser_pts = df_parser[['x', 'y', 'z']].to_numpy()
    elif len(df_parser) == 1:
        parser_pts = df_parser[['x', 'y', 'z']].to_numpy()
    else:
        # 无 Parser 数据: 退化到 RT 自身的相邻段距离 (可视化轨迹均匀性)
        print(f"  [WARN] 无 Parser 数据, 误差热力图退化为 RT 步长可视化")
        if len(df_rt) >= 2:
            diffs = np.linalg.norm(np.diff(rt_pts, axis=0), axis=1)
            errs = np.append(diffs, diffs[-1])
        else:
            errs = np.zeros(len(df_rt))
        parser_pts = None
    if len(df_parser) >= 2:
        errs = path_following_error(rt_pts, parser_pts)

    fig = plt.figure(figsize=(12, 9))
    if use_3d:
        ax = fig.add_subplot(111, projection='3d')
        sc = ax.scatter(rt_pts[:, 0], rt_pts[:, 1], rt_pts[:, 2],
                        c=errs, cmap='jet', s=6, alpha=0.8)
        ax.set_zlabel('Z (mm)')
    else:
        ax = fig.add_subplot(111)
        sc = ax.scatter(rt_pts[:, 0], rt_pts[:, 1],
                        c=errs, cmap='jet', s=6, alpha=0.8)
        ax.set_ylabel('Y (mm)')
    ax.set_xlabel('X (mm)')
    ax.set_title(f'Path Following Error Heatmap (max={np.nanmax(errs):.4f} mm)'
                 f'{title_suffix}')
    if not use_3d:
        ax.axis('equal')
    ax.grid(True, alpha=0.3)

    cbar = fig.colorbar(sc, ax=ax, shrink=0.8)
    cbar.set_label('Error (mm)')

    plt.tight_layout()
    plt.savefig(output_path, dpi=120)
    plt.close()
    print(f"  [PLOT] {output_path}  (max_err={np.nanmax(errs):.4f} mm)")
    return output_path, float(np.nanmax(errs))


# ==================== 一键全管线仪表盘 ====================

def render_dashboard(csv_path, out_dir=VIZ_OUT_DIR, a_max=200.0, jerk_max=5000.0,
                     savgol_window=5, savgol_poly=2, use_3d=True):
    """
    一键加载 CSV 并生成全部三大视图, 返回 {view_name: png_path}。
    """
    print(f"\n[Viz] 加载管线 CSV: {csv_path}")
    os.makedirs(out_dir, exist_ok=True)
    data = load_pipeline_csv(csv_path)
    for k in ['parser', 'comp', 'bspline', 'planner', 'rt']:
        print(f"  stage {k:8s}: {len(data.get(k, pd.DataFrame())):6d} 条")

    outputs = {}
    outputs['spatial']  = render_spatial_overlay(
        data, os.path.join(out_dir, 'viz_spatial_overlay.png'),
        use_3d=use_3d)
    outputs['ekg']      = render_dynamics_ekg(
        data.get('rt', pd.DataFrame()),
        a_max=a_max, jerk_max=jerk_max,
        savgol_window=savgol_window, savgol_poly=savgol_poly,
        output_path=os.path.join(out_dir, 'viz_dynamics_ekg.png'))
    err_ret = render_error_heatmap(
        data, os.path.join(out_dir, 'viz_error_heatmap.png'), use_3d=use_3d)
    if err_ret:
        outputs['heatmap'] = err_ret[0]
        outputs['max_err'] = err_ret[1]

    return data, outputs


# ==================== 主入口 ====================

if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='CNC 多级管线可视化仪表盘')
    ap.add_argument('csv', help='输入 CSV 路径 (cnc_trace_log.csv 或 sim 输出)')
    ap.add_argument('--out-dir', default=VIZ_OUT_DIR, help='图表输出目录 (默认 tests/output/viz_dashboard/)')
    ap.add_argument('--a-max', type=float, default=200.0,
                    help='最大加速度阈值 mm/s^2 (默认 200, 与 axis_ctrl.c 一致)')
    ap.add_argument('--jerk-max', type=float, default=5000.0,
                    help='最大 jerk 阈值 mm/s^3 (默认 5000)')
    ap.add_argument('--savgol-window', type=int, default=5,
                    help='savgol 窗口 (默认 5 严格模式, 11 抹平尖刺仅供趋势)')
    ap.add_argument('--savgol-poly', type=int, default=2,
                    help='savgol 多项式阶数 (默认 2)')
    ap.add_argument('--no-3d', action='store_true', help='使用 2D 模式 (默认 3D)')
    args = ap.parse_args()

    render_dashboard(args.csv, out_dir=args.out_dir,
                     a_max=args.a_max, jerk_max=args.jerk_max,
                     savgol_window=args.savgol_window, savgol_poly=args.savgol_poly,
                     use_3d=not args.no_3d)
