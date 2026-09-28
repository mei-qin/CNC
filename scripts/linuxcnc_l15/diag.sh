#!/bin/bash
# capture 通道诊断: 会话存活期内现场检查 sampler 状态与 halsampler 导出
cd /home/l15 || exit 1
rm -f .rtapi_fifo .rtapi_* /dev/shm/* /tmp/linuxcnc.lock 2>/dev/null

/usr/bin/linuxcnc -r g93_probe_rsh.ini > bootD.log 2>&1 &
sleep 24
C=$(ps aux | grep -cE 'rtapi_ap[p]|milltas[k]|linuxcncrs[h]')
echo "== procs=$C"; [ "$C" -lt 3 ] && { tail bootD.log; exit 1; }

echo "== setp enable 1 =="
halcmd setp sampler.0.enable 1; echo "rc=$?"
sleep 2
echo "== sample-num / depth =="
halcmd show param sampler.0 2>&1 | grep -E "sample-num|curr-depth|overruns"
echo "== direct halsampler head (3s) =="
timeout 3 halsampler -t sampler.0 2>&1 | head -4
echo "== again after another 2s =="
sleep 2
timeout 3 halsampler -t sampler.0 2>&1 | head -2
echo "== final params =="
halcmd show param sampler.0 2>&1 | grep -E "sample-num|curr-depth|overruns"
