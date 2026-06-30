#!/bin/bash
cd /home/meiqin/cnc_ws
rm -f cnc_trace_log_*.csv

timeout 60 bash -c '
  printf "4\ntests/gcode/h1_min_repro.nc\n"
  sleep 45
  printf "0\n"
' | ./cnc_core sim > /tmp/h1_out.txt 2>&1 &
PID=$!

for i in $(seq 1 60); do
  sleep 1
  if ls cnc_trace_log_*.csv >/dev/null 2>&1; then
    CSV=$(ls -t cnc_trace_log_*.csv | head -1)
    sz=$(wc -c < "$CSV")
    if [ "$sz" -gt 500 ]; then
      cp -f "$CSV" /mnt/d/code/CNC/tests/output/scenario_log/log_h1.csv
      echo "$(wc -l < "$CSV") rows, copied"
      grep -E "Macro|G01|G0.*0|文件处理|步进|Loader|X=" /tmp/h1_out.txt | head -20
      kill $PID 2>/dev/null
      exit 0
    fi
  fi
done
echo "FAILED"
kill $PID 2>/dev/null
