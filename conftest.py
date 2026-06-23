#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
conftest.py: pytest 共享 fixture / 命令行选项 / 失败钩子
================================================
1. 注册 --csv / --a-max / --jerk-max 命令行选项 (供 test_pipeline.py 使用)
2. 暴露 session 级 _FAIL_STASH (test 模块填充数据, 钩子消费)
3. 实现 pytest_runtest_makereport 钩子: 断言失败时自动生成 FAIL_*.png

必须在 conftest.py 中 (test_pipeline.py 加载时机晚于命令行解析与钩子注册)。
"""

import os
import sys
import io
import pytest

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

DEFAULT_CSV_PATH = os.environ.get(
    'CNC_TRACE_CSV',
    os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cnc_trace_log.csv')
)


def pytest_addoption(parser):
    parser.addoption('--csv',      default=DEFAULT_CSV_PATH,
                     help='CNC 管线 CSV 文件路径 (默认 ./cnc_trace_log.csv 或 $CNC_TRACE_CSV)')
    parser.addoption('--a-max',    type=float,
                     default=float(os.environ.get('CNC_A_MAX', '200.0')),
                     help='最大加速度阈值 mm/s^2 (默认 200, 与 axis_ctrl.c 一致)')
    parser.addoption('--jerk-max', type=float,
                     default=float(os.environ.get('CNC_JERK_MAX', '5000.0')),
                     help='最大 jerk 阈值 mm/s^3 (默认 5000)')
    parser.addoption('--savgol-window', type=int,
                     default=int(os.environ.get('CNC_SAVGOL_WINDOW', '5')),
                     help='savgol 滤波窗口 (奇数, 默认 5 = 严格 5ms 尖刺检测; '
                          '11 = 抹平 3ms 尖刺, 仅用于趋势观察)')
    parser.addoption('--savgol-poly', type=int,
                     default=int(os.environ.get('CNC_SAVGOL_POLY', '2')),
                     help='savgol 多项式阶数 (默认 2, 必须 < window)')
    parser.addoption('--assume-rtcp', action='store_true',
                     default=os.environ.get('CNC_ASSUME_RTCP', '') == '1',
                     help='强制假设 RTCP 已开启 (跳过 path_following 测试)')


# ==================== Session 级共享 stash ====================
# test_pipeline.py 的 pipeline_data fixture 写入 'data',
# 下面的失败钩子读取 'data' 来渲染 FAIL_*.png 诊断图。
_FAIL_STASH = {}


def get_fail_stash():
    """供 test_pipeline.py 的 fixture 写入。"""
    return _FAIL_STASH


@pytest.hookimpl(tryfirst=True, hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """
    任意测试在 call 阶段断言失败时, 自动产出 FAIL_<test>_<view>.png 三件套:
      FAIL_<test>_spatial.png   (全管线空间叠加图, 通用上下文)
      FAIL_<test>_heatmap.png   (路径误差热力图, path_following 专属)
      FAIL_<test>_ekg.png       (动力学心电图, dynamics 专属)
    """
    outcome = yield
    report = outcome.get_result()
    if report.when != 'call' or report.outcome != 'failed':
        return
    if report.failed and (report.longrepr is None or
                          'AssertionError' not in str(report.longrepr)):
        # 仅对断言失败产出 (跳过 fixture / 导入异常等)
        # 注意: pytest 9.x 中 failed 状态判断已涵盖, 这里二次保险
        pass

    data = _FAIL_STASH.get('data')
    if data is None:
        return

    test_name = item.name
    a_max     = item.config.getoption('--a-max')
    jerk_max  = item.config.getoption('--jerk-max')
    csv_path  = item.config.getoption('--csv')
    fail_dir  = os.path.dirname(os.path.abspath(csv_path)) or '.'

    # 延迟导入避免无 matplotlib 环境下 conftest 加载失败
    try:
        from cnc_viz_suite import (
            render_spatial_overlay, render_dynamics_ekg, render_error_heatmap
        )
    except ImportError as e:
        print(f"[FAIL-ARTIFACT] 无法导入 cnc_viz_suite: {e}", file=sys.stderr)
        return

    print(f"\n[FAIL-ARTIFACT] 测试 {test_name} 断言失败, 生成诊断图表...",
          file=sys.stderr)

    # 1) 总是产出空间叠加图作为整体上下文
    try:
        p = os.path.join(fail_dir, f'FAIL_{test_name}_spatial.png')
        render_spatial_overlay(data, output_path=p,
                               title_suffix=f' [FAIL: {test_name}]')
        print(f"  -> {p}", file=sys.stderr)
    except Exception as e:
        print(f"  [WARN] spatial 渲染失败: {e}", file=sys.stderr)

    # 2) 按测试类型补充专属图表
    if 'path_following' in test_name:
        try:
            ret = render_error_heatmap(
                data,
                output_path=os.path.join(fail_dir, f'FAIL_{test_name}_heatmap.png'),
                title_suffix=f' [FAIL: {test_name}]')
            if ret:
                print(f"  -> {ret[0]}  (max_err={ret[1]:.4f} mm)", file=sys.stderr)
        except Exception as e:
            print(f"  [WARN] heatmap 渲染失败: {e}", file=sys.stderr)

    if 'dynamics' in test_name:
        try:
            df_rt = data.get('rt')
            if df_rt is None or len(df_rt) == 0:
                df_rt = None
            if df_rt is not None:
                p = os.path.join(fail_dir, f'FAIL_{test_name}_ekg.png')
                render_dynamics_ekg(df_rt, a_max=a_max, jerk_max=jerk_max,
                                    output_path=p,
                                    title_suffix=f' [FAIL: {test_name}]')
                print(f"  -> {p}", file=sys.stderr)
        except Exception as e:
            print(f"  [WARN] EKG 渲染失败: {e}", file=sys.stderr)
