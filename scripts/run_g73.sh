#!/bin/bash
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

echo "=== MAIN: test_g73_high_speed.nc ==="
rm -f cnc_trace_log_*.csv
(
  echo "4"
  echo "tests/gcode/test_g73_high_speed.nc"
  sleep 15
  echo "0"
) | timeout 25 ./cnc_core sim > /tmp/g73_out.txt 2>&1
CSV=$(ls -t cnc_trace_log_*.csv | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" "$OUT/log_g73.csv"
  echo "MAIN: $(wc -l < "$CSV") rows"
  grep -E "M30|文件处理|超限|必须|拒绝|error" /tmp/g73_out.txt
else
  echo "MAIN: NO_CSV"
fi

echo ""
echo "=== REGRESSION (8 tests) ==="
for t in tests/gcode/test_g81_drill.nc tests/gcode/test_g82_dwell.nc tests/gcode/test_g83_peck.nc tests/gcode/test_g83_optimized.nc tests/gcode/test_m_codes_basic.nc tests/gcode/test_m98_basic.nc tests/gcode/test_sysvar.nc tests/gcode/wcs_sync_test.nc; do
  rm -f cnc_trace_log_*.csv
  printf "4\n%s\n0\n" "$t" | timeout 15 ./cnc_core sim > /tmp/reg_out.txt 2>&1
  pc=$(grep -oP "PC .*?[0-9]+" /tmp/reg_out.txt | tail -1 | grep -oP "[0-9]+")
  err=$(grep -c "Loader.*错误\|Alarm\|超限\|必须\|拒绝" /tmp/reg_out.txt)
  echo "  $t: PC=$pc err=$err"
done
