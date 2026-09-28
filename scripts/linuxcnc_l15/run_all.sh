#!/bin/bash
# L1.5 全流程 (以 l15 用户运行): 清场 → 启会话 → 驱动 → adapter
# 用法: wsl -d Debian-L15 -u l15 -e bash /mnt/d/code/CNC/scripts/linuxcnc_l15/run_all.sh <run_id>
N="${1:-r1}"
P=/mnt/d/code/CNC/scripts/linuxcnc_l15
cd /home/l15 || exit 1
cp "$P/l15_drive.py" "$P/adapter_l15.py" . 2>/dev/null

pkill -f "linuxcnc -r" 2>/dev/null; pkill -f rtapi_app 2>/dev/null; sleep 2
rm -f /dev/shm/*rtapi* /dev/shm/*hal* 2>/dev/null || true
rm -f /home/l15/.rtapi_fifo /home/l15/.rtapi_* 2>/dev/null || true

nohup linuxcnc -r g93_probe_rsh.ini > bootS.log 2>&1 &
sleep 22
C=$(ps aux | grep -cE 'rtapi_ap[p]|milltas[k]|linuxcncrs[h]')
echo "== procs=$C"
if [ "$C" -lt 3 ]; then
  echo "SESSION_FAILED; bootS.log:"
  tr -cd "[:print:]\n" < bootS.log | grep -vE "screen size" | tail -10
  exit 1
fi

python3 l15_drive.py "$N" 2>&1
echo "=== adapter ==="
python3 adapter_l15.py --capture "capture_${N}.csv" --nc g93_probe.ngc \
    --ini g93_probe.ini --out "l15_report_${N}"
