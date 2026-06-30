#!/bin/bash
cd /home/meiqin/cnc_ws
rm -f cnc_trace_log_*.csv

# Run G54.1 test in background
timeout 30 bash -c '
  echo "4"
  echo "tests/gcode/test_g541_ext.nc"
  sleep 20
  echo "0"
' | ./cnc_core sim > /tmp/g541_out.txt 2>&1 &
PID=$!

# Wait for CSV to appear
for i in $(seq 1 25); do
  sleep 1
  if ls cnc_trace_log_*.csv >/dev/null 2>&1; then
    CSV=$(ls -t cnc_trace_log_*.csv | head -1)
    sz=$(wc -c < "$CSV")
    if [ "$sz" -gt 100000 ]; then
      cp -f "$CSV" /mnt/d/code/CNC/tests/output/scenario_log/log_g541.csv
      echo "$(wc -l < "$CSV") rows, csv copied"
      kill $PID 2>/dev/null
      exit 0
    fi
  fi
done
echo "FAILED"
kill $PID 2>/dev/null
