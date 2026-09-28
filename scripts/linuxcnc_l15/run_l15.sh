#!/usr/bin/env bash
# L1.5 运行手册 (草案 — 首接触后定稿; 手动时刻, 证据以 capture CSV 为准)
# 前提: README_BOOTSTRAP.md 环境已就绪, 本目录可运行 `linuxcnc g93_probe.ini`
set -euo pipefail
D="$(cd "$(dirname "$0")" && pwd)"
cd "$D"

# ---------- 终端 1: 主会话 ----------
#   linuxcnc g93_probe.ini
#   → axis GUI: Home All → 等回零完成 → 关主轴(无需) →
#   → halcmd setp sampler.0.enable 0     # 平时空转不采
#   → File/Open g93_probe.ngc → 切 Auto
#   → 启动前: halcmd setp sampler.0.enable 1
#   → Cycle Start, 同时在终端 2 触发时序脚本 (bash run_l15.sh timing)
#
# ---------- 终端 2: 外部干预时序 (在 Cycle Start 瞬间执行) ----------
# 程序运动段 ~7.5s; 干预窗口设计:
#   t≈1s  override→50%   t≈3s 恢复 100%   (P3)
#   t≈4.5s feedhold 按下 t≈5.5s 释放      (P6; M53 P1 已在程序内)
timing() {
  read -p "按回车开始时序 (与 Cycle Start 同步)..."
  sleep 1
  halcmd setp halui.feed-override.direct-value 1
  halcmd setp halui.feed-override.scale 0.01
  halcmd setp halui.feed-override.counts 50   # → 50%  (语义首接触时用
  echo "[t+1s] override 50%"                  # halcmd show pin halui.feed-override.value 复核)
  sleep 2
  halcmd setp halui.feed-override.counts 100  # → 100%
  echo "[t+3s] override 100%"
  sleep 1.5
  halcmd setp motion.feed-hold 1
  echo "[t+4.5s] feedhold ON"
  sleep 1
  halcmd setp motion.feed-hold 0
  echo "[t+5.5s] feedhold OFF"
}

# ---------- 程序结束后 (终端 2) ----------
capture() {
  N="${1:-r1}"
  halcmd show param sampler.0.overruns || true   # 溢出检查 (期望 0)
  halsampler -t sampler.0 > "capture_${N}.csv"
  wc -l "capture_${N}.csv"
  python3 adapter_l15.py --capture "capture_${N}.csv" \
      --nc g93_probe.ngc --ini g93_probe.ini --out "l15_report_${N}"
}

case "${1:-}" in
  timing) timing ;;
  capture) capture "${2:-r1}" ;;
  *) echo "用法: bash run_l15.sh timing | capture [run_id]"
     echo "先冻结: git add scripts/linuxcnc_l15 && git commit (首跑前)" ;;
esac
