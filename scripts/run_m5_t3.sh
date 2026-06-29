#!/bin/bash
# M5 T3 CSV collection (binary already compiled)
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

rm -f cnc_trace_log_*.csv
printf "4\ntests/gcode/test_m98_nested.nc\n0\n" | timeout 20 ./cnc_core sim >/dev/null 2>&1
CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
[ -n "$CSV" ] && cp "$CSV" "$OUT/log_m98_nested.csv" && echo "T3: $(wc -l < "$OUT/log_m98_nested.csv") rows"
