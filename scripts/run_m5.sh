#!/bin/bash
# M5 T1/T2 CSV collection
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

# T1
rm -f cnc_trace_log_*.csv
printf "4\ntests/gcode/test_m98_basic.nc\n0\n" | timeout 20 ./cnc_core sim >/dev/null 2>&1
CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
[ -n "$CSV" ] && cp "$CSV" "$OUT/log_m98_basic.csv" && echo "T1: $(wc -l < "$OUT/log_m98_basic.csv") rows"

# T2
rm -f cnc_trace_log_*.csv
printf "4\ntests/gcode/test_m98_repeat.nc\n0\n" | timeout 20 ./cnc_core sim >/dev/null 2>&1
CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
[ -n "$CSV" ] && cp "$CSV" "$OUT/log_m98_repeat.csv" && echo "T2: $(wc -l < "$OUT/log_m98_repeat.csv") rows"
