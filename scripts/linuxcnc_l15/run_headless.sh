#!/bin/bash
# L1.5 无头运行 v2 (2026-09-29 修正: halsampler 必须为会话子进程 loadusr + 行缓冲;
#   程序完成轮询; 首接触增量工具)
# 用法: bash run_headless.sh <run_id>   (前提: linuxcnc -r g93_probe_rsh.ini 会话已在跑)
N="${1:-r1}"
cd /home/l15 || exit 1
RSH() { printf "$1" | timeout 10 nc 127.0.0.1 5007; }

halcmd setp sampler.0.enable 0

# 机器上电 + 瞬时回零 (INI 已去 SEARCH/LATCH/SHARED)
RSH 'hello EMC c 1.0\nset enable EMCTOO\nset echo off\nset verbose on\nset estop off\nset machine on\nquit\n' | grep -E "ACK|NAK" | tail -2
sleep 2
RSH 'hello EMC c 1.0\nset enable EMCTOO\nset echo off\nset home -1\nquit\n' | tail -1
sleep 3
printf "hello EMC c 1.0\nget error\nquit\n" | timeout 6 nc 127.0.0.1 5007 | grep ERROR && { echo "ABORT: homing error"; exit 1; }

# halsampler 作为会话子进程 (loadusr) — 跨进程直接跑会 hal_stream_attach 失败
halcmd setp sampler.0.enable 1
stdbuf -oL halcmd loadusr halsampler -t sampler.0 > "capture_${N}.csv" &
HS=$!

# 外部干预时序: t+1s ovr50, t+3s ovr100, t+4.5s hold ON, t+5.5s hold OFF
( sleep 1.0; RSH 'hello EMC c 1.0\nset enable EMCTOO\nset echo off\nset feed_override 50\nquit\n' > /dev/null;  echo "[t+1.0] ovr 50"
  sleep 2.0; RSH 'hello EMC c 1.0\nset enable EMCTOO\nset echo off\nset feed_override 100\nquit\n' > /dev/null; echo "[t+3.0] ovr 100"
  sleep 1.5; halcmd setp motion.feed-hold 1; echo "[t+4.5] hold ON"
  sleep 1.0; halcmd setp motion.feed-hold 0; echo "[t+5.5] hold OFF" ) &
TIMING=$!

RSH "hello EMC c 1.0\nset enable EMCTOO\nset echo off\nset mode auto\nset open /home/l15/g93_probe.ngc\nset run 0\nquit\n" | tail -2
wait $TIMING

# 轮询程序完成 (最长 40s)
for i in $(seq 1 80); do
  ST=$(printf "hello EMC c 1.0\nget program_status\nquit\n" | timeout 4 nc 127.0.0.1 5007 | grep PROGRAM_STATUS)
  echo "$ST" | grep -q IDLE && { echo "$ST (poll $i)"; break; }
  sleep 0.5
done
sleep 1
halcmd unloadusr halsampler 2>/dev/null; wait $HS 2>/dev/null
halcmd setp sampler.0.enable 0
wc -l "capture_${N}.csv"

echo "=== adapter ==="
python3 adapter_l15.py --capture "capture_${N}.csv" --nc g93_probe.ngc --ini g93_probe.ini --out "l15_report_${N}" | tail -32
