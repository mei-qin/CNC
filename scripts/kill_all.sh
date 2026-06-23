#!/bin/bash
kill -9 $(ps aux | awk '/cnc_core/ && !/awk/ {print $2}') 2>/dev/null
kill -9 $(ps aux | awk '/run_tests/ && !/awk/ {print $2}') 2>/dev/null
sleep 1
echo "Cleaned. Remaining:"
ps aux | awk '/cnc_core/ && !/awk/ {print}' || echo "(none)"
