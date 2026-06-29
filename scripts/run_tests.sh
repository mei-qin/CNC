#!/bin/bash
# CNC 核心算法验证 — 全自动测试流水线
# 运行环境: WSL2 / Ubuntu
# 用法: bash scripts/run_tests.sh

set -e
PROJ="/mnt/d/code/CNC"
cd "$PROJ"
GCODE_DIR="$PROJ/tests/gcode"
SCENARIO_LOG_DIR="$PROJ/tests/output/scenario_log"
mkdir -p "$SCENARIO_LOG_DIR"

echo "================================================"
echo "  CNC 全自动测试流水线"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "================================================"

# ── 确保编译是最新的 ──
echo ""
echo "[0/6] 编译最新 cnc_core..."
make clean > /dev/null 2>&1
make all 2>&1 | tail -1
echo "      编译完成 ($(stat -c%s cnc_core) bytes)"

# ── 测试1: L1 B-Spline 锐角截断 ──
echo ""
echo "[1/6] 运行 L1_sharp_corner.nc (B-Spline + G41)..."
printf "4\n%s\n0\n" "$GCODE_DIR/L1_sharp_corner.nc" | timeout 300 ./cnc_core sim > /dev/null 2>&1
CSV1=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV1" ]; then
    cp "$CSV1" "$SCENARIO_LOG_DIR/log_bspline.csv"
    echo "      生成: $SCENARIO_LOG_DIR/log_bspline.csv ($(wc -l < "$SCENARIO_LOG_DIR/log_bspline.csv") 行)"
else
    echo "      ERROR: L1 未生成 CSV!"
fi

# ── 测试2: L2 G93 时间守恒 ──
echo ""
echo "[2/6] 运行 L2_g93_strict.nc (G93 倒数时间)..."
printf "4\n%s\n0\n" "$GCODE_DIR/L2_g93_strict.nc" | timeout 300 ./cnc_core sim > /dev/null 2>&1
CSV2=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV2" ]; then
    cp "$CSV2" "$SCENARIO_LOG_DIR/log_g93.csv"
    echo "      生成: $SCENARIO_LOG_DIR/log_g93.csv ($(wc -l < "$SCENARIO_LOG_DIR/log_g93.csv") 行)"
else
    echo "      ERROR: L2 未生成 CSV!"
fi

# ── 测试3: L3 RTCP 逆解 ──
echo ""
echo "[3/6] 运行 L3_kinematics_rtcp.nc (RTCP 逆解)..."
printf "4\n%s\n0\n" "$GCODE_DIR/L3_kinematics_rtcp.nc" | timeout 300 ./cnc_core sim > /dev/null 2>&1
CSV3=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV3" ]; then
    cp "$CSV3" "$SCENARIO_LOG_DIR/log_rtcp.csv"
    echo "      生成: $SCENARIO_LOG_DIR/log_rtcp.csv ($(wc -l < "$SCENARIO_LOG_DIR/log_rtcp.csv") 行)"
else
    echo "      ERROR: L3 未生成 CSV!"
fi

# ── 清理时间戳 CSV ──
rm -f cnc_trace_log_*.csv

echo ""
echo "================================================"
echo "  CSV 数据采集完成!"
echo "  log_bspline.csv : $(wc -l < "$SCENARIO_LOG_DIR/log_bspline.csv" 2>/dev/null || echo 0) 行"
echo "  log_g93.csv     : $(wc -l < "$SCENARIO_LOG_DIR/log_g93.csv" 2>/dev/null || echo 0) 行"
echo "  log_rtcp.csv    : $(wc -l < "$SCENARIO_LOG_DIR/log_rtcp.csv" 2>/dev/null || echo 0) 行"
echo "================================================"
