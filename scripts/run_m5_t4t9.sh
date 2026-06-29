#!/bin/bash
# M5 T4-T9 boundary tests — batch run + CSV collection
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

run_test() {
    local name="$1"  local nc="$2"  local csv="$3"
    rm -f cnc_trace_log_*.csv
    printf "4\n%s\n0\n" "$nc" | timeout 20 ./cnc_core sim >/dev/null 2>&1
    CSF=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
    if [ -n "$CSF" ]; then cp "$CSF" "$OUT/$csv"; echo "$name: $(wc -l < "$OUT/$csv") rows"; else echo "$name: NO_CSV"; fi
}

# T9: L=0 no-op (quick, simplest first)
run_test "T9" "tests/gcode/test_m98_l0_noop.nc" "log_m98_l0_noop.csv"

# T4: depth overflow
run_test "T4" "tests/gcode/test_m98_overflow.nc" "log_m98_overflow.csv"

# T5: #1-#33 save/restore
run_test "T5" "tests/gcode/test_m98_locals.nc" "log_m98_locals.csv"

# T6: repeat independent locals
run_test "T6" "tests/gcode/test_m98_repeat_independent.nc" "log_m98_repeat_independent.csv"

# T8: modal isolation
run_test "T8" "tests/gcode/test_m98_modal_isolation.nc" "log_m98_modal_isolation.csv"

# T7: WHILE × subprogram
run_test "T7" "tests/gcode/test_m98_in_while.nc" "log_m98_in_while.csv"

echo "ALL DONE"
