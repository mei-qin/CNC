#!/bin/bash
# P0-3 CSV collection script
cd /home/meiqin/cnc_ws
GCODE=tests/gcode
OUTDIR=/mnt/d/code/CNC/tests/output/scenario_log
mkdir -p "$OUTDIR"

# M1: test_goto.nc
rm -f cnc_trace_log_*.csv
printf "4\n%s/test_goto.nc\n0\n" "$GCODE" | timeout 20 ./cnc_core sim >/dev/null 2>&1
CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV" ]; then
    cp "$CSV" "$OUTDIR/log_goto_new.csv"
    echo "M1: $(wc -l < "$OUTDIR/log_goto_new.csv") rows"
else
    echo "M1: FAILED"
fi

# M2: test_if_loop.nc
rm -f cnc_trace_log_*.csv
printf "4\n%s/test_if_loop.nc\n0\n" "$GCODE" | timeout 25 ./cnc_core sim >/dev/null 2>&1
CSV=$(ls -t cnc_trace_log_*.csv 2>/dev/null | head -1)
if [ -n "$CSV" ]; then
    cp "$CSV" "$OUTDIR/log_if_loop_new.csv"
    echo "M2: $(wc -l < "$OUTDIR/log_if_loop_new.csv") rows"
else
    echo "M2: FAILED"
fi
