#!/bin/bash
# WP-B (positive transformation-violation) runner — oneshot_rip.sh v2 lineage:
# system apt stack + ~/libctl single-library switch + /proc/maps hard load gate.
# The ONLY variable across LC-T0/T1/T2 is ~/libctl/librs274.so.0 (decided by caller).
# Differences from oneshot_rip.sh: probe program = g93t_probe.ngc (straight-only,
# no arcs), drive = l15t_drive.py (parameterized program, identical timeline),
# lib sha persisted to libsha_<run>.txt, adapter --nc g93t_probe.ngc.
# Usage: oneshot_wpb.sh <run_id>        (libctl content decided by caller)
N="${1:-wpbDry}"
P=/mnt/d/code/CNC/scripts/linuxcnc_l15
cd /home/l15 || exit 1
cp "$P/l15t_drive.py" "$P/adapter_l15.py" . 2>/dev/null
cp "$P/g93_probe.ini" "$P/g93_probe.hal" \
   "$P/g93_probe.var" "$P/g93_probe.tbl" . 2>/dev/null
cp /mnt/d/code/CNC/tests/output/linuxcnc_positive_case/nc/g93t_probe.ngc . 2>/dev/null
sed -i "s/\r$//" g93t_probe.ngc g93_probe.* l15t_drive.py adapter_l15.py
sed "s/^DISPLAY = axis/DISPLAY = linuxcncrsh/" g93_probe.ini > g93_probe_rsh.ini
rm -f .rtapi_fifo .rtapi_* /dev/shm/* /tmp/linuxcnc.lock 2>/dev/null

LIBCTL="$HOME/libctl"
if [ ! -f "$LIBCTL/librs274.so.0" ]; then
  echo "== $LIBCTL/librs274.so.0 missing"; exit 1
fi
LIBSHA=$(sha256sum "$LIBCTL/librs274.so.0" | cut -c1-12)
echo "$LIBSHA" > "libsha_${N}.txt"
export LD_LIBRARY_PATH="$LIBCTL:${LD_LIBRARY_PATH:-}"
echo "== libctl librs274 sha12=$LIBSHA"

bash -x linuxcnc -r -v g93_probe_rsh.ini > bootT.log 2>&1 &
sleep 22
IO_OK=""
for W in $(seq 1 60); do
  if halcmd show pin iocontrol.0.user-enable-out >/dev/null 2>&1; then IO_OK=1; break; fi
  sleep 1
done
C=$(ps aux | grep -cE 'rtapi_ap[p]|milltas[k]|linuxcncrs[h]')
echo "== procs=$C io_ready=${IO_OK:-NO} (waited ${W}s)"
if [ "$C" -lt 3 ]; then
  echo "== bootT trace tail =="
  tr -cd "[:print:]\n" < bootT.log | grep -vE "screen size" | tail -15
  exit 1
fi

# HARD GATE: verify the running milltask actually loaded the libctl library
MPID=$(pgrep -x milltask | head -1)
LOADED=$(grep -o "$LIBCTL/librs274.so.0" /proc/$MPID/maps 2>/dev/null | head -1)
echo "== milltask pid=$MPID loaded=${LOADED:-SYSTEM-LIB!!!}"
if [ -z "$LOADED" ]; then
  echo "== ABORT: libctl library not loaded; run invalid"
  exit 1
fi

python3 l15t_drive.py "$N" g93t_probe.ngc > "${N}_drive.log" 2>&1
DR=$?
echo "== drive_exit=$DR =="
grep -vE "screen size" "${N}_drive.log"
echo "== adapter =="
python3 adapter_l15.py --capture "capture_${N}.csv" --nc g93t_probe.ngc \
    --ini g93_probe.ini --out "l15_report_${N}" 2>&1
