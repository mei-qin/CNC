#!/bin/bash
# WP-B pre-run clearing ritual (root) — F-1 RUN_LOG discipline + NML SysV segment purge.
# Usage (as root): bash wpb_clear.sh
# Rationale: pkill -9 prevents the linuxcnc wrapper's graceful cleanup, so NML/HAL
# SysV shm segments and /tmp residue leak across sessions and wedge the next
# iocontrol/milltask handshake. Kill everything (incl. wrapper bash), wait for
# detach, then ipcrm every segment and remove residue.
pkill -9 -x milltask 2>/dev/null
pkill -9 -x linuxcncrsh 2>/dev/null
pkill -9 -x linuxcncsvr 2>/dev/null
pkill -9 -x halui 2>/dev/null
pkill -9 -x io 2>/dev/null
pkill -9 -x rtapi_app 2>/dev/null
pkill -9 -f 'linuxcn[c] -r' 2>/dev/null
pkill -9 -f halcm[d] 2>/dev/null
sleep 3
for id in $(ipcs -m | tail -n +4 | awk '{print $2}' | grep -E '^[0-9]+$'); do
  ipcrm -m "$id" 2>/dev/null
done
rm -f /dev/shm/* /tmp/rtapi* /tmp/linuxcnc.lock /home/l15/.rtapi_fifo 2>/dev/null
sleep 2
# second pass: segments whose owner died between the two passes
for id in $(ipcs -m | tail -n +4 | awk '{print $2}' | grep -E '^[0-9]+$'); do
  ipcrm -m "$id" 2>/dev/null
done
P=$(ps aux | grep -cE 'milltas[k]|rtapi_ap[p]|linuxcncrs[h]|linuxcn[c] -r')
S=$(ipcs -m | grep -c '^0x')
L=$(ls /tmp/linuxcnc.lock 2>/dev/null | wc -l)
echo "CLEAR procs=$P shm_left=$S lock=$L"
if [ "$P" != "0" ] || [ "$S" != "0" ] || [ "$L" != "0" ]; then
  echo "CLEAR_INCOMPLETE"
  exit 1
fi
echo "CLEAR_OK"
