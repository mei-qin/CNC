#!/bin/bash
# L1.5 一发式部署: 重试启动会话 → 驱动 r1 → cat 日志 (2026-09-29)
# 在 Debian-L15 内以 root 运行: bash /mnt/d/code/CNC/scripts/linuxcnc_l15/deploy_r1.sh
N="${1:-r1}"
P=/mnt/d/code/CNC/scripts/linuxcnc_l15
cp "$P/l15_drive.py" /home/l15/ 2>/dev/null
chown l15:l15 /home/l15/l15_drive.py

pkill -9 -f linuxcnc 2>/dev/null; sleep 1
rm -f /dev/shm/*rtapi* /dev/shm/*hal* 2>/dev/null

C=0
for A in 1 2 3; do
  su - l15 -c "cd /home/l15 && nohup linuxcnc -r g93_probe_rsh.ini > bootR.log 2>&1 &" >/dev/null 2>&1
  sleep 22
  C=$(ps aux | grep -cE 'rtapi_ap[p]|milltas[k]|linuxcncrs[h]')
  echo "== attempt $A procs=$C"
  [ "$C" -ge 3 ] && break
  pkill -9 -f linuxcnc 2>/dev/null; sleep 1
  rm -f /dev/shm/*rtapi* /dev/shm/*hal* 2>/dev/null
done
[ "$C" -ge 3 ] || { echo "SESSION_FAILED"; exit 1; }

su - l15 -c "cd /home/l15 && python3 l15_drive.py $N > ${N}_drive.log 2>&1; echo drive_exit=\$?"
echo "== drive log =="
cat "/home/l15/${N}_drive.log"
