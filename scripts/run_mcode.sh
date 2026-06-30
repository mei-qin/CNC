#!/bin/bash
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

echo "=== MAIN: test_m_codes_basic.nc ==="
rm -f cnc_trace_log_*.csv
(
  echo "4"
  echo "tests/gcode/test_m_codes_basic.nc"
  sleep 15
  echo "0"
) | timeout 25 ./cnc_core sim > /tmp/m_code_out.txt 2>&1
CSV=$(ls -t cnc_trace_log_*.csv | head -1)
if [ -n "$CSV" ] && [ "$(wc -c < "$CSV")" -gt 500 ]; then
  cp -f "$CSV" "$OUT/log_m_codes.csv"
  echo "MAIN: $(wc -l < "$CSV") rows"
  grep -E "\[Parser\]" /tmp/m_code_out.txt | head -30
else
  echo "MAIN: NO_CSV"
fi

# Regression tests
run_reg() {
  local name="$1" local nc="$2"
  rm -f cnc_trace_log_*.csv
  printf "4\n%s\n0\n" "$nc" | timeout 15 ./cnc_core sim >/dev/null 2>&1
  local pc=$(grep "PC 步进" /dev/stdin 2>/dev/null || echo "?")
  echo "$name: OK"
}
echo ""
echo "=== REGRESSION ==="
for t in tests/gcode/test_g81_drill.nc tests/gcode/test_m98_basic.nc tests/gcode/test_while.nc tests/gcode/test_goto.nc tests/gcode/test_if_loop.nc; do
  rm -f cnc_trace_log_*.csv
  printf "4\n%s\n0\n" "$t" | timeout 15 ./cnc_core sim > /tmp/reg_out.txt 2>&1
  pc=$(grep -oP "PC \K[0-9]+" /tmp/reg_out.txt)
  echo "  $t: PC=$pc"
done
