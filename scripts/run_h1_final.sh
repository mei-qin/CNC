#!/bin/bash
# H-1 final run script
cd /home/meiqin/cnc_ws
rm -f cnc_trace_log_*.csv /tmp/h1_out.txt

# Use (echo; sleep; echo) to keep stdin open for trajectory to complete
(
  echo "4"
  echo "tests/gcode/h1_min_repro.nc"
  sleep 60
  echo "0"
) | timeout 90 ./cnc_core sim > /tmp/h1_out.txt 2>&1

CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" /mnt/d/code/CNC/tests/output/scenario_log/log_h1.csv
  echo "CSV: $(wc -l < "$CSV") rows"
  grep -E "Macro|G01|X=|文件|步进|Loader.*加载|error" /tmp/h1_out.txt | head -15
else
  echo "NO_CSV"
fi
