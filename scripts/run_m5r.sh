#!/bin/bash
# M5-R regression batch: run 4 tests + collect CSVs
cd /home/meiqin/cnc_ws
OUT=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUT"

run_reg() {
    local name="$1"  local nc="$2"  local sl="$3"
    rm -f cnc_trace_log_*.csv
    printf "4\n%s\n0\n" "$nc" | timeout $((sl+10)) ./cnc_core sim >/dev/null 2>&1
    CSF=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
    if [ -n "$CSF" ]; then
        cp "$CSF" "$OUT/log_${name}_m5.csv"
        echo "${name}: $(wc -l < "$OUT/log_${name}_m5.csv") rows"
    else
        echo "${name}: NO_CSV"
    fi
}

run_reg "goto_m1"     "tests/gcode/test_goto.nc"             10
run_reg "if_loop_m2"  "tests/gcode/test_if_loop.nc"          15
run_reg "while_m3"    "tests/gcode/test_while.nc"            15
run_reg "while_nest"  "tests/gcode/test_while_nested.nc"     25
