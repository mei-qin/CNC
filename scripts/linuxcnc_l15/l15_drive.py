#!/usr/bin/env python3
"""L1.5 无头驱动 v3 (2026-09-29) — 单一持久 rsh 连接 + halui/halcmd 干预 + 直连 halsampler
用法: python3 l15_drive.py <run_id>
前提: linuxcnc -r g93_probe_rsh.ini 会话已在跑 (milltask/rtapi/linuxcncrsh 存活)
"""
import socket, subprocess, sys, time

N = sys.argv[1] if len(sys.argv) > 1 else "r1"
CAP = f"capture_{N}.csv"


def halcmd(*args):
    return subprocess.run(["halcmd", *args], capture_output=True, text=True).stdout.strip()


class Rsh:
    def __init__(self):
        self.s = socket.create_connection(("127.0.0.1", 5007), timeout=6)
        self.s.settimeout(3)
        assert "ACK" in self.cmd("hello EMC py 1.0"), "hello failed"
        self.cmd("set enable EMCTOO")
        self.cmd("set echo off")
        self.cmd("set verbose on")

    def cmd(self, line):
        self.s.sendall((line + "\r\n").encode())
        buf = b""
        try:
            while b"\n" not in buf:
                chunk = self.s.recv(4096)
                if not chunk:
                    raise ConnectionError(f"rsh closed after: {line}")
                buf += chunk
        except socket.timeout:
            pass
        return buf.decode(errors="replace").strip()

    def get(self, sub):
        return self.cmd(f"get {sub}")


def main():
    halcmd("setp", "sampler.0.enable", "0")
    r = Rsh()
    print(r.cmd("set estop off"))
    print(r.cmd("set machine on"), flush=True)
    time.sleep(1)
    print(r.cmd("set home -1"), flush=True)
    time.sleep(3)
    err = r.get("error")
    if "Cannot home" in err or "NAK" in err:
        print("ABORT homing:", err)
        return 1

    # 清空 boot 期陈旧样本 (enable 默认 TRUE, boot 24s 已积累): 关 → 清 → 开
    halcmd("setp", "sampler.0.enable", "0")
    subprocess.run(["timeout", "40", "halsampler"], stdout=subprocess.DEVNULL)

    def ovr(pct):
        halcmd("setp", "halui.feed-override.direct-value", "1")
        halcmd("setp", "halui.feed-override.scale", "0.01")
        halcmd("setp", "halui.feed-override.counts", str(pct))

    halcmd("setp", "sampler.0.enable", "1")
    print(r.cmd("set mode auto"))
    print(r.cmd("set open /home/l15/g93_probe.ngc"), flush=True)
    t0 = time.time()
    print(r.cmd("set run 0"), flush=True)

    marks = [(1.0, "ovr50"), (3.0, "ovr100"), (4.5, "hold_on"), (5.5, "hold_off")]
    for t, m in marks:
        wait = t0 + t - time.time()
        if wait > 0:
            time.sleep(wait)
        if m == "ovr50":
            ovr(50)
        elif m == "ovr100":
            ovr(100)
        elif m == "hold_on":
            halcmd("setp", "motion.feed-hold", "1")
        else:
            halcmd("setp", "motion.feed-hold", "0")
        print(f"[t+{t:.1f}] {m}", flush=True)

    for i in range(120):
        st = r.get("program_status")
        if "IDLE" in st:
            print(f"done @poll{i} ({time.time()-t0:.1f}s): {st}", flush=True)
            break
        time.sleep(0.5)
    else:
        print("WARN: program not idle after 60s:", st)

    time.sleep(1)
    halcmd("setp", "sampler.0.enable", "0")
    with open(CAP, "w") as f:
        subprocess.run(["timeout", "20", "halsampler"], stdout=f)
    print("ovr pin final:", halcmd("show", "pin", "halui.feed-override.value"))
    r.cmd("quit")
    print("capture lines:", sum(1 for _ in open(CAP)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
