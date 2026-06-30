#!/bin/bash
cd /home/meiqin/cnc_ws
mkdir -p /tmp/g65_logs

run_test() {
    local name="$1" local nc="$2" local extra_sleep="$3"
    rm -f cnc_trace_log_*.csv
    (
      echo "4"
      echo "$nc"
      sleep ${extra_sleep:-10}
      echo "0"
    ) | timeout $((extra_sleep + 15)) ./cnc_core sim > "/tmp/g65_logs/${name}.log" 2>&1
    CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
    if [ -n "$CSV" ]; then cp "$CSV" "/tmp/g65_logs/${name}.csv"; fi
    echo "$name: CSV=$(wc -l < "/tmp/g65_logs/${name}.csv" 2>/dev/null || echo 0) rows"
}

# Test 1
echo "=== TEST 1 ==="
run_test "test_g65_macro"   "tests/gcode/test_g65_macro.nc"   15

# Test 2
echo "=== TEST 2 ==="
run_test "test_g65_vs_m98"  "tests/gcode/test_g65_vs_m98.nc"  15

# 11 regressions
echo "=== REGRESSION ==="
for t in test_m98_basic test_m98_locals test_m98_repeat test_m98_nested \
         test_m98_in_while test_m98_overflow test_m98_l0_noop \
         test_m98_modal_isolation test_while test_if_loop test_goto; do
  run_test "$t" "tests/gcode/${t}.nc" 10
done

echo "DONE"
