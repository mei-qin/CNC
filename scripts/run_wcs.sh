#!/bin/bash
# Direct run script for WCS sync test
cd /home/meiqin/cnc_ws
rm -f cnc_trace_log_*.csv
rm -f /tmp/wcs_out.txt

# Run sim in background, capture its output
timeout 60 bash -c '
  printf "4\ntests/gcode/wcs_sync_test.nc\n" 
  sleep 40
  printf "0\n"
' | ./cnc_core sim > /tmp/wcs_out.txt 2>&1 &
PID=$!

# Wait for CSV to appear
for i in $(seq 1 50); do
  sleep 1
  if ls cnc_trace_log_*.csv >/dev/null 2>&1; then
    CSV=$(ls -t cnc_trace_log_*.csv | head -1)
    sz=$(wc -c < "$CSV")
    if [ "$sz" -gt 500 ]; then
      cp -f "$CSV" /mnt/d/code/CNC/tests/output/scenario_log/log_wcs_sync.csv
      echo "$(wc -l < "$CSV") rows, copied"
      kill $PID 2>/dev/null
      exit 0
    fi
  fi
done
echo "FAILED to capture CSV"
kill $PID 2>/dev/null
