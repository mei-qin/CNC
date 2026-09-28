#!/bin/bash
cd /home/l15 || exit 1
rm -f .rtapi_fifo .rtapi_* /dev/shm/* /tmp/linuxcnc.lock 2>/dev/null
/usr/bin/linuxcnc -r g93_probe_rsh.ini > bootD2.log 2>&1 &
sleep 24
C=$(ps aux | grep -cE 'rtapi_ap[p]|milltas[k]|linuxcncrs[h]')
echo "== procs=$C"; [ "$C" -lt 3 ] && { grep -vE "screen size" bootD2.log | tail; exit 1; }

echo "== bootD2 sampler lines =="
grep -iE "sampler|g93_probe.hal" bootD2.log | tr -cd "[:print:]\n"

echo "== threads =="
halcmd show thread 2>&1

echo "== setp enable + run 2s =="
halcmd setp sampler.0.enable 1 && echo setp_ok
sleep 2

echo "== full param dump grep sampler =="
halcmd show param 2>&1 | grep -i sampler

echo "== halsampler to file, no pipe, 3s =="
timeout 3 halsampler -t sampler.0 > d2cap.csv 2> d2err.txt
echo "rc=$? lines=$(wc -l < d2cap.csv) err=$(cat d2err.txt)"

echo "== halsampler without -t =="
timeout 3 halsampler sampler.0 >> d2cap.csv 2>/dev/null
echo "lines_total=$(wc -l < d2cap.csv)"

echo "== depth param exact =="
halcmd getp sampler.0.overruns 2>&1
halcmd getp sampler.0.sample-num 2>&1
