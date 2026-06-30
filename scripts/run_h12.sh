#!/bin/bash
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

# H1: micro-segment test (sleep 30 for 1000 segments)
echo "=== H1: micro-segment ==="
rm -f cnc_trace_log_*.csv
(
  echo "4"
  echo "tests/gcode/h1_microseg.nc"
  sleep 30
  echo "0"
) | timeout 45 ./cnc_core sim > /tmp/h1_micro_out.txt 2>&1
CSV=$(ls -t cnc_trace_log_*.csv | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" "$OUT/log_h1_microseg.csv"
  echo "H1: $(wc -l < "$CSV") rows"
else
  echo "H1: NO_CSV"
fi

# H2: alarm test (instant - abort happens quickly)
echo "=== H2: alarm ==="
rm -f cnc_trace_log_*.csv
(
  echo "4"
  echo "tests/gcode/h2_alarm.nc"
  sleep 10
  echo "0"
) | timeout 20 ./cnc_core sim > /tmp/h2_alarm_out.txt 2>&1
CSV=$(ls -t cnc_trace_log_*.csv | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" "$OUT/log_h2_alarm.csv"
  echo "H2: $(wc -l < "$CSV") rows"
else
  echo "H2: NO_CSV"
fi
