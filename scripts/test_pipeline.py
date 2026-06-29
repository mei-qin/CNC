#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CNC 多级管线动力学断言 (test_pipeline.py)
================================================
pytest 套件, 读取 C 端 trace_logger / sim_engine 输出的多级 CSV 数据,
执行两类硬性物理断言:

  test_path_following   —— 路径跟随断言
      df_rt 实际执行点到 df_parser 原始意图折线的最短距离
      未开启刀补时, 最大误差必须 < 0.05 mm

  test_dynamics         —— 动力学物理断言
      df_rt 的 v_current 经 savgol_filter 平滑 + np.gradient 求导后
      max(|a_actual|) <= a_max * 1.05

断言失败时:
  - 自动将相关图表保存为 FAIL_<test>_<view>.png
  - 通过 print 输出错误日志 (pytest -s 可见)

运行:
  pytest test_pipeline.py -v -s
  pytest test_pipeline.py -v -s --csv path/to/cnc_trace_log.csv
  pytest test_pipeline.py -v -s --a-max 250
"""

import sys, io, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pytest

# 同目录导入
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cnc_viz_suite import (
    load_pipeline_csv, path_following_error, compute_dynamics,
    render_spatial_overlay, render_dynamics_ekg, render_error_heatmap,
    detect_rtcp, STAGE_CUTTER_COMP,
)

# 从 conftest.py 获取共享 stash (fixture 写入数据, 失败钩子消费)
from conftest import get_fail_stash
_FAIL_STASH = get_fail_stash()

# ==================== 全局参数 (可通过命令行 / 环境变量覆盖) ====================

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_TRACE_LOG_DIR = os.path.normpath(os.path.join(_SCRIPT_DIR, "..", "tests", "output", "trace_log"))


def _latest_trace_csv():
    """返回 trace_log 目录下最新的 cnc_trace_log_*.csv，没有则返回占位路径。"""
    import glob
    candidates = glob.glob(os.path.join(_TRACE_LOG_DIR, "cnc_trace_log_*.csv"))
    if not candidates:
        return os.path.join(_TRACE_LOG_DIR, 'cnc_trace_log.csv')
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


DEFAULT_CSV_PATH = os.environ.get('CNC_TRACE_CSV', _latest_trace_csv())
DEFAULT_A_MAX    = float(os.environ.get('CNC_A_MAX', '200.0'))    # mm/s^2
DEFAULT_JERK_MAX = float(os.environ.get('CNC_JERK_MAX', '5000.0')) # mm/s^3
PATH_ERR_TOL_MM  = 0.05  # 路径跟随误差容忍 (mm)

# 注: pytest_addoption 在 conftest.py 中注册 (此处注册会因加载时机晚于命令行解析而失败)


# ==================== Session 级数据 fixture (避免重复加载) ====================

@pytest.fixture(scope='session')
def csv_path(request):
    p = request.config.getoption('--csv')
    if not os.path.exists(p):
        pytest.skip(f"CSV 文件不存在: {p} (用 --csv 指定)")
    return p


@pytest.fixture(scope='session')
def pipeline_data(csv_path):
    """加载并按 stage_id 拆分 CSV。"""
    print(f"\n[Test] 加载 CSV: {csv_path}")
    data = load_pipeline_csv(csv_path)
    for k in ['parser', 'comp', 'bspline', 'planner', 'rt']:
        n = len(data.get(k, pd.DataFrame()))
        print(f"       {k:8s}: {n:6d} 条")
    # 暴露给失败钩子用
    _FAIL_STASH['data'] = data
    return data


# ==================== Session 级数据 fixture (避免重复加载) ====================
# (失败钩子 pytest_runtest_makereport 与命令行选项已迁至 conftest.py)


# ==================== 基础健全性测试 ====================

def test_csv_loaded(pipeline_data):
    """CSV 能正确加载, 且至少有 1 个 stage 含数据。"""
    total = sum(len(pipeline_data.get(k, pd.DataFrame()))
                for k in ['parser', 'comp', 'bspline', 'planner', 'rt'])
    assert total > 0, "CSV 中无任何管线数据 (所有 stage 均为空)"


# ==================== 测试 1: 路径跟随断言 ====================

def test_path_following(pipeline_data, request):
    """
    路径跟随断言:
      计算 df_rt 中每个点到 df_parser 折线的最短欧氏距离,
      若未开启刀补 (df_comp 为空) 且未开启 RTCP, 最大误差必须 < 0.05 mm。

    设计依据:
      - 刀补开启时, 刀具中心轨迹被故意偏置 R 距离, 误差 > 0.05 是预期行为 → SKIP
      - RTCP 开启时 (G43.4), df_parser 是逻辑刀尖坐标, df_rt 是物理关节坐标,
        二者处于不同坐标空间, 直接比对会得到荒诞的大误差 → SKIP
        (修复方案: Python 端做正运动学 FK 还原刀尖坐标, 需机床构型配置)
    """
    df_parser = pipeline_data.get('parser', pd.DataFrame())
    df_rt     = pipeline_data.get('rt',     pd.DataFrame())
    df_comp   = pipeline_data.get('comp',   pd.DataFrame())

    # 数据存在性检查
    if len(df_parser) < 2:
        pytest.skip(f"Parser 数据不足 ({len(df_parser)} 行), 无法构建意图折线")
    if len(df_rt) < 2:
        pytest.skip(f"RT 数据不足 ({len(df_rt)} 行), 无法评估路径跟随")

    cutter_comp_on = len(df_comp) > 0
    if cutter_comp_on:
        pytest.skip(f"刀补已开启 (df_comp 含 {len(df_comp)} 行), "
                    f"路径偏离由 R 半径决定, 不适用 0.05mm 阈值")

    # RTCP 检测: parser 端 B/C 轴有变化 → 几乎必然 RTCP 已开
    # (非 RTCP 的 5 轴 G 代码在 parser 与 rt 间是同一空间, 距离不会爆)
    assume_rtcp = request.config.getoption('--assume-rtcp') \
                  if request.config.getoption('--assume-rtcp', default=False) else False
    rtcp_info = detect_rtcp(pipeline_data, assume_rtcp=assume_rtcp)
    print(f"\n[PathFollow][RTCP-Check] {rtcp_info['reason']}")
    if rtcp_info['rtcp_on']:
        pytest.skip(
            f"RTCP 已开启 ({rtcp_info['reason']}); "
            f"df_parser=逻辑刀尖坐标, df_rt=物理关节坐标, 二者空间不同, "
            f"直接比对会得到荒诞误差。需 Python 端实现 FK 还原刀尖坐标。"
        )

    # 计算每点路径误差
    rt_pts     = df_rt[['x', 'y', 'z']].to_numpy()
    parser_pts = df_parser[['x', 'y', 'z']].to_numpy()
    errs       = path_following_error(rt_pts, parser_pts)

    max_err = float(np.nanmax(errs))
    p95_err = float(np.nanpercentile(errs, 95))
    mean_err = float(np.nanmean(errs))

    print(f"[PathFollow] N_rt={len(rt_pts)}  N_parser={len(parser_pts)}")
    print(f"             max_err  = {max_err:.6f} mm")
    print(f"             p95_err  = {p95_err:.6f} mm")
    print(f"             mean_err = {mean_err:.6f} mm")
    print(f"             tol      = {PATH_ERR_TOL_MM} mm")

    assert max_err < PATH_ERR_TOL_MM, (
        f"路径跟随误差超限: max_err={max_err:.4f} mm >= {PATH_ERR_TOL_MM} mm; "
        f"p95={p95_err:.4f}, mean={mean_err:.4f}"
    )


# ==================== 测试 2: 动力学物理断言 ====================

def test_dynamics(pipeline_data, request):
    """
    动力学断言:
      1. 取 df_rt 的 v_target 列 (C 端 log_velocity 实际写入的是物理瞬时速度 v_current)
      2. savgol_filter 平滑 (默认 window=5, poly=2 严格模式) 避免量化噪声在求导时放大
         ⚠️ window 选择极其重要:
            - window=5  (5ms 跨度): 能捕获 ≥3ms 的速度尖刺 (短线段限速 Bug 典型征兆)
            - window=11 (11ms 跨度): 会抹平 ≤5ms 的尖刺, 仅适合趋势观察
         默认严格 5, 趋势观察请显式 --savgol-window 11
      3. np.gradient(v_smooth, dt) → a_actual (mm/s^2)
      4. np.gradient(a, dt)         → jerk_actual (mm/s^3)
      5. 断言: max(|a_actual|) <= a_max * 1.05

    a_max 默认 200 mm/s^2 (与 axis_ctrl.c 默认配置一致)。
    """
    df_rt = pipeline_data.get('rt', pd.DataFrame())
    savgol_window = request.config.getoption('--savgol-window')
    savgol_poly   = request.config.getoption('--savgol-poly')
    min_rows = max(savgol_window, 5)
    if len(df_rt) < min_rows:
        pytest.skip(f"RT 数据不足 ({len(df_rt)} 行), 至少需 {min_rows} 行做 savgol "
                    f"(window={savgol_window})")

    a_max    = request.config.getoption('--a-max')
    jerk_max = request.config.getoption('--jerk-max')

    time_ms     = df_rt['cycle'].to_numpy(dtype=float)
    v_mm_per_ms = df_rt['v_target'].to_numpy(dtype=float)

    dyn = compute_dynamics(time_ms, v_mm_per_ms,
                           savgol_window=savgol_window, savgol_poly=savgol_poly)
    a    = dyn['a']
    jerk = dyn['jerk']

    max_abs_a    = float(np.nanmax(np.abs(a)))
    max_abs_jerk = float(np.nanmax(np.abs(jerk)))
    a_thresh     = a_max * 1.05
    jerk_thresh  = jerk_max * 1.05

    print(f"\n[Dynamics]  N_rt={len(df_rt)}  savgol(window={savgol_window}, poly={savgol_poly})")
    print(f"            a_max(配置)={a_max}  jerk_max(配置)={jerk_max}")
    print(f"            max|a_actual|    = {max_abs_a:.4f} mm/s²   "
          f"(thresh {a_thresh:.2f}, utilization {max_abs_a/a_thresh*100:.1f}%)")
    print(f"            max|jerk_actual| = {max_abs_jerk:.2f} mm/s³  "
          f"(thresh {jerk_thresh:.2f}, utilization {max_abs_jerk/jerk_thresh*100:.1f}%)")

    assert max_abs_a <= a_thresh, (
        f"加速度超限: max|a_actual|={max_abs_a:.4f} mm/s² "
        f"> thresh={a_thresh:.2f} (a_max*1.05={a_max}*1.05); "
        f"max|jerk|={max_abs_jerk:.1f}; "
        f"savgol window={savgol_window} (尖刺可能被抹平, 试试 --savgol-window 5)"
    )


# ==================== 主入口 (允许直接 python 运行做冒烟测试) ====================

if __name__ == '__main__':
    # 直接 python 运行: 加载 CSV + 跑两个断言, 失败时也产出 FAIL 图表
    import getopt
    opts, _ = getopt.getopt(sys.argv[1:], '', ['csv=', 'a-max=', 'jerk-max='])
    kw = {}
    for o, v in opts:
        if o == '--csv':       kw['csv'] = v
        elif o == '--a-max':   kw['a_max'] = float(v)
        elif o == '--jerk-max': kw['jerk_max'] = float(v)

    csv = kw.get('csv', DEFAULT_CSV_PATH)
    if not os.path.exists(csv):
        print(f"[ERROR] CSV 不存在: {csv}")
        sys.exit(2)

    print(f"[Smoke] 直接运行模式, csv={csv}")
    data = load_pipeline_csv(csv)
    _FAIL_STASH['data'] = data

    a_max    = kw.get('a_max', DEFAULT_A_MAX)
    jerk_max = kw.get('jerk_max', DEFAULT_JERK_MAX)
    fail_dir = os.path.dirname(csv) or '.'

    def _render_fail(test_name):
        """失败时产出 FAIL_<test>_<view>.png 三件套"""
        try:
            render_spatial_overlay(
                data,
                output_path=os.path.join(fail_dir, f'FAIL_{test_name}_spatial.png'))
        except Exception as e:
            print(f"  [WARN] spatial 渲染失败: {e}")
        if 'path_following' in test_name:
            try:
                render_error_heatmap(
                    data,
                    output_path=os.path.join(fail_dir, f'FAIL_{test_name}_heatmap.png'))
            except Exception as e:
                print(f"  [WARN] heatmap 渲染失败: {e}")
        if 'dynamics' in test_name:
            try:
                render_dynamics_ekg(
                    data.get('rt', pd.DataFrame()),
                    a_max=a_max, jerk_max=jerk_max,
                    output_path=os.path.join(fail_dir, f'FAIL_{test_name}_ekg.png'))
            except Exception as e:
                print(f"  [WARN] EKG 渲染失败: {e}")

    failures = 0
    for fn in (test_path_following, test_dynamics):
        try:
            if fn is test_dynamics:
                # 直接构造最小 request mock, 走与 pytest 相同的入口
                class _Req:
                    pass
                class _Cfg:
                    def getoption(self, n):
                        if n == '--a-max':    return a_max
                        if n == '--jerk-max': return jerk_max
                        return None
                req = _Req(); req.config = _Cfg()
                fn(data, req)
            else:
                fn(data)
            print(f"  [PASS] {fn.__name__}")
        except AssertionError as e:
            failures += 1
            print(f"  [FAIL] {fn.__name__}: {e}")
            _render_fail(fn.__name__)
        except pytest.skip.Exception as e:
            print(f"  [SKIP] {fn.__name__}: {e}")

    sys.exit(1 if failures else 0)
