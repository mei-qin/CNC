#!/bin/bash
# CNC 全自动测试流水线 v2 — WSL 原生文件系统版
# 解决: 9P 文件系统性能瓶颈 + stdin pipe EOF 问题
set -e
WS="/home/meiqin/cnc_ws"
cd "$WS"

echo "================================================"
echo "  CNC 全自动测试流水线 v2"
echo "  工作目录: $WS (WSL 原生 ext4)"
echo "  时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "================================================"

# ── 编译 ──
echo ""
echo "[0/6] 编译 cnc_core..."
make clean > /dev/null 2>&1
make all 2>&1 | grep -E "编译成功|error" || true
echo "      编译完成 ($(stat -c%s cnc_core) bytes)"

GCODE_DIR="$WS/tests/gcode"
WIN_SCENARIO_LOG_DIR="/mnt/d/code/CNC/tests/output/scenario_log"
mkdir -p "$WIN_SCENARIO_LOG_DIR"

# ── 辅助函数: 运行单个测试 ──
run_test() {
    local name="$1"
    local nc_file="$2"
    local out_file="$3"
    local timeout_sec="$4"
    
    echo ""
    echo "[*] 运行 $name..."
    
    # 使用 (sleep; echo) 保持 stdin 打开足够久
    # 先 echo 命令和文件名，然后 sleep 保持管道不关
    (echo "4"; echo "$nc_file"; sleep "${timeout_sec}"; echo "0") | timeout $((timeout_sec + 10)) ./cnc_core sim > /dev/null 2>&1
    
    CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
    if [ -n "$CSV" ]; then
        cp "$CSV" "$out_file"
        LINES=$(wc -l < "$out_file")
        echo "      ✓ $out_file ($LINES 行, $(stat -c%s "$out_file") bytes)"
        rm -f "$CSV"
    else
        echo "      ✗ $name 未生成 CSV!"
    fi
}

# ── 测试1: L1 B-Spline (短) ──
run_test "L1_sharp_corner" "$GCODE_DIR/L1_sharp_corner.nc" "$WIN_SCENARIO_LOG_DIR/log_bspline.csv" 10

# ── 测试2: L2 G93 (中等) ──
run_test "L2_g93_strict" "$GCODE_DIR/L2_g93_strict.nc" "$WIN_SCENARIO_LOG_DIR/log_g93.csv" 30

# ── 测试3: L3 RTCP (中等) ──
run_test "L3_kinematics_rtcp" "$GCODE_DIR/L3_kinematics_rtcp.nc" "$WIN_SCENARIO_LOG_DIR/log_rtcp.csv" 15

# ── 清理时间戳 CSV ──
rm -f cnc_trace_log_*.csv

# ── 复制回 Windows 文件系统 ──
echo ""
echo "[*] 复制结果到 $WIN_SCENARIO_LOG_DIR/ ..."
for f in log_bspline.csv log_g93.csv log_rtcp.csv; do
    if [ -f "$f" ]; then
        cp "$f" "$WIN_SCENARIO_LOG_DIR/$f"
        echo "      $f → $WIN_SCENARIO_LOG_DIR/$f"
    fi
done
# cnc_core 二进制仍归项目根
if [ -f "cnc_core" ]; then
    cp "cnc_core" /mnt/d/code/CNC/"cnc_core"
    echo "      cnc_core → /mnt/d/code/CNC/cnc_core"
fi

echo ""
echo "================================================"
echo "  测试完成!"
echo "  log_bspline.csv : $(wc -l < "$WIN_SCENARIO_LOG_DIR/log_bspline.csv" 2>/dev/null || echo 0) 行"
echo "  log_g93.csv     : $(wc -l < "$WIN_SCENARIO_LOG_DIR/log_g93.csv" 2>/dev/null || echo 0) 行"
echo "  log_rtcp.csv    : $(wc -l < "$WIN_SCENARIO_LOG_DIR/log_rtcp.csv" 2>/dev/null || echo 0) 行"
echo "================================================"
