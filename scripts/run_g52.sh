#!/bin/bash
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

echo "=== MAIN: test_g52_local.nc ==="
rm -f cnc_trace_log_*.csv
(
  echo "4"
  echo "tests/gcode/test_g52_local.nc"
  sleep 15
  echo "0"
) | timeout 25 ./cnc_core sim > /tmp/g52_out.txt 2>&1
CSV=$(ls -t cnc_trace_log_*.csv | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" "$OUT/log_g52.csv"
  echo "MAIN: $(wc -l < "$CSV") rows"
  grep -E "G52|M30|文件处理|error" /tmp/g52_out.txt
else
  echo "MAIN: NO_CSV"
fi

echo ""
echo "=== REGRESSION ==="
for t in tests/gcode/test_g81_drill.nc tests/gcode/test_m98_basic.nc tests/gcode/test_m_codes_basic.nc tests/gcode/wcs_sync_test.nc tests/gcode/test_while.nc; do
  rm -f cnc_trace_log_*.csv
  printf "4\n%s\n0\n" "$t" | timeout 15 ./cnc_core sim > /tmp/reg_out.txt 2>&1
  pc=$(grep -oP "PC .*?[0-9]+" /tmp/reg_out.txt | tail -1 | grep -oP "[0-9]+")
  err=$(grep -c "Loader.*错误\|Alarm" /tmp/reg_out.txt)
  echo "  $t: PC=$pc errors=$err"
done
