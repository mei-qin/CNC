#!/bin/bash
# L1.5 一次调用全流程 (l15 用户): 启动(带-x迹) → 验证 → 驱动 → adapter
N="${1:-r1}"
P=/mnt/d/code/CNC/scripts/linuxcnc_l15
cd /home/l15 || exit 1
cp "$P/l15_drive.py" "$P/adapter_l15.py" . 2>/dev/null
cp "$P/g93_probe.ngc" "$P/g93_probe.ini" "$P/g93_probe.hal" \
   "$P/g93_probe.var" "$P/g93_probe.tbl" . 2>/dev/null
sed -i "s/\r$//" g93_probe.* l15_drive.py adapter_l15.py   # git autocrlf 会引入 CRLF, iocontrol 对此致命
sed "s/^DISPLAY = axis/DISPLAY = linuxcncrsh/" g93_probe.ini > g93_probe_rsh.ini
rm -f .rtapi_fifo .rtapi_* /dev/shm/* /tmp/linuxcnc.lock 2>/dev/null

bash -x /usr/bin/linuxcnc -r -v g93_probe_rsh.ini > bootT.log 2>&1 &
sleep 22
# io 就绪门: iocontrol 引脚可见才驱动 (楔死现场 io 均无响应; 2026-09-29)
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

python3 l15_drive.py "$N" > "${N}_drive.log" 2>&1
DR=$?
echo "== drive_exit=$DR =="
grep -vE "screen size" "${N}_drive.log"
echo "== adapter =="
python3 adapter_l15.py --capture "capture_${N}.csv" --nc g93_probe.ngc \
    --ini g93_probe.ini --out "l15_report_${N}" 2>&1
