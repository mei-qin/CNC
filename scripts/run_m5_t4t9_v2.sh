#!/bin/bash
# M5 T4-T9 boundary tests — fixed with sleep approach
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

run_test() {
    local name="$1"  local nc="$2"  local csv="$3"  local sl="$4"
    rm -f cnc_trace_log_*.csv
    (echo "4"; echo "$nc"; sleep "$sl"; echo "0") | timeout $((sl+5)) ./cnc_core sim >/dev/null 2>&1
    CSF=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
    if [ -n "$CSF" ]; then cp "$CSF" "$OUT/$csv"; echo "$name: $(wc -l < "$OUT/$csv") rows"; else echo "$name: NO_CSV"; fi
}

echo "=== M5 Boundary Tests T4-T9 ==="

# T9: L=0 no-op (instant)
run_test "T9" "tests/gcode/test_m98_l0_noop.nc" "log_m98_l0_noop.csv" 5

# T4: depth overflow (instant - error path)
run_test "T4" "tests/gcode/test_m98_overflow.nc" "log_m98_overflow.csv" 5

# T5: locals - G01 X99 needs time for trajectory
run_test "T5" "tests/gcode/test_m98_locals.nc" "log_m98_locals.csv" 20

# T6: repeat independent - 3 x G01 X1
run_test "T6" "tests/gcode/test_m98_repeat_independent.nc" "log_m98_repeat_independent.csv" 15

# T8: modal isolation - G01 X1 + G01 X20
run_test "T8" "tests/gcode/test_m98_modal_isolation.nc" "log_m98_modal_isolation.csv" 15

# T7: WHILE x subprogram - 3 x G91 G01 X1
run_test "T7" "tests/gcode/test_m98_in_while.nc" "log_m98_in_while.csv" 20

echo "ALL DONE"
