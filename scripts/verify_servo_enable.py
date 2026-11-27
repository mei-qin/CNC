#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐轴伺服使能 端到端验证 (WSL2 内运行; 2026-11-27)

用法 (WSL2, 项目根目录; 需先 make && make probe):
    python3 scripts/verify_servo_enable.py [T1 T2 ...]     # 不带参数 = 全部

用例:
  T1  上电部分使能 (SIM_BOOT_FAULT=Z): 进程不退出, X/Y/C/B 就绪, Z 故障锁存;
      OPC UA 故障场景探针 (Z.Active=false, ServoOk=false, 报警列表有 Z 轴文字)
  T2  自愈 (SIM_BOOT_FAULT=Z + SIM_BOOT_FAULT_MS=600): 故障在重试用尽前撤销
      → Z 自动回使能, 不锁存, 无 0x000F
  T3  重试上限 / 事件有界 (取 T1 会话事件): 0x0002×1, 0x000F×1, 0x0053×2 (WARN+ALARM)
  T5  门控 (T1 会话): JogStart(X)=0, JogStart(Z)=-3, Home.Axis(Z)=-3, Home.All=-3,
      Program.Start=-4, Home.Axis(X)=0
  T4  总线故障: 注入 → 报警 + 掩码清零; 注入未撤销时复位 = -3; 撤销后不复位仍锁存;
      复位 → 全轴回使能, 报警清
  T8  快照 v5 (9528): version=5, 掩码/逐轴状态与 RPC 一致; 总线故障时 bus_fault=1
  T6  SIGTERM (带一个空闲 RPC 连接) 与 RPC Close: 优雅下电 + 进程退出码 0
  T7  3 轴档案 (sim hw): 配置掩码 0x07, 3 轴就绪, 其余轴不参与, 启动不挂起
"""
import glob
import os
import signal
import socket
import struct
import subprocess
import sys
import time
import zlib

HOST = "127.0.0.1"
PORT = 9527
BIN = "/mnt/d/code/CNC/rpc_server"
PROBE = "/mnt/d/code/CNC/opcua_probe"
PROJECT = "/mnt/d/code/CNC"
OUT_DIR = PROJECT + "/tests/output/servo_enable_20261127"
NC_DIR = PROJECT + "/tests/gcode"
EVENT_GLOB = PROJECT + "/event_logs/event_log_*.bin"

CMD_CLOSE = 0x0002
CMD_GET_LOGICAL_POS = 0x0020
CMD_GET_SYSTEM_STATUS = 0x0025
CMD_SUBSCRIBE = 0x002A
CMD_LOAD_PROGRAM = 0x002C
CMD_RUN_LOADED_PROGRAM = 0x002D
CMD_GET_PROGRAM_STRUCTURE = 0x002E
CMD_CLEAR_ALARM = 0x002F
CMD_CONFIG_HOMING_ORDER = 0x005C
CMD_HOMING_TRIGGER = 0x005D
CMD_GET_HOMING = 0x005F
CMD_JOG_START = 0x0060
CMD_JOG_STOP = 0x0061
CMD_GET_AXIS_STATUS = 0x008D
CMD_SERVO_TEST_HOOK = 0x008E

AX_ENTRY = struct.Struct("<BBbBHHi")        # state/stage/fault_slave/_pad/status_word/err_code/retry
AX_RES = struct.Struct("<iIIIB3s")           # ret/config/ready/latched/bus_fault/_pad
EVT = struct.Struct("<QQBBHi64s")
EVHDR = struct.Struct("<IIIIQQ")
SNAP_ACK = struct.Struct("<IIII")
SNAP_HDR = struct.Struct("<IIII")

ST_IDLE, ST_ENABLING, ST_OP, ST_PROBE, ST_LATCHED, ST_BUS = 0, 1, 2, 3, 4, 5
X, Y, Z, C, B = 0, 1, 2, 3, 4
ALL5 = 0x1F

_proc = None
_log_fh = None
_results = []


def report(name, ok, detail=""):
    _results.append((name, bool(ok), detail))
    print("  [%s] %-58s %s" % ("PASS" if ok else "FAIL", name, detail))


# ---------------- RPC ----------------
def recvn(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise ConnectionError("socket closed")
        buf += c
    return buf


def rpc(cmd, payload=b"", timeout=10.0):
    s = socket.create_connection((HOST, PORT), timeout=timeout)
    try:
        s.sendall(struct.pack("<HH", cmd, len(payload)) + payload)
        err, dl = struct.unpack("<iI", recvn(s, 8))
        resp = recvn(s, dl) if dl > 0 else b""
    finally:
        s.close()
    return err, resp


def rpc_int(cmd, payload=b""):
    try:
        err, resp = rpc(cmd, payload)
    except Exception as e:
        return -999, None
    if err != 0:
        return err, None
    if len(resp) < 4:
        return -998, None
    return 0, struct.unpack("<i", resp[:4])[0]


def axis_status():
    try:
        err, resp = rpc(CMD_GET_AXIS_STATUS, b"")
    except Exception:
        return None
    need = AX_RES.size + AX_ENTRY.size * 5
    if err != 0 or len(resp) < need:
        return None
    ret, cfg, rdy, lt, bus, _ = AX_RES.unpack_from(resp, 0)
    keys = ("state", "stage", "fault_slave", "_pad", "sw", "err", "retry")
    axes = [dict(zip(keys, AX_ENTRY.unpack_from(resp, AX_RES.size + AX_ENTRY.size * i)))
            for i in range(5)]
    return {"ret": ret, "cfg": cfg, "rdy": rdy, "lt": lt, "bus": bus, "axes": axes}


def hook(op, axis="X", sub=0):
    return rpc_int(CMD_SERVO_TEST_HOOK, struct.pack("<iBBH", op, ord(axis), sub, 0))


def sys_status():
    try:
        err, resp = rpc(CMD_GET_SYSTEM_STATUS)
    except Exception:
        return "<rpc fail>"
    return resp.decode("ascii", errors="replace").split("\0", 1)[0].strip()


def jog_start(axis, direction, speed):
    return rpc_int(CMD_JOG_START, struct.pack("<c3sid", axis.encode("ascii")[0:1],
                                              b"\0\0\0", direction, speed))


def jog_stop(axis):
    return rpc_int(CMD_JOG_STOP, axis.encode("ascii")[0:1] + b"\0\0\0")


def homing_trigger(axis=""):
    a = axis.encode("ascii")[0:1] if axis else b"\0"
    return rpc_int(CMD_HOMING_TRIGGER, a + b"\0\0\0")


def get_homing():
    try:
        err, resp = rpc(CMD_GET_HOMING, b"")
    except Exception:
        return None
    if err != 0 or len(resp) < 28:
        return None
    return struct.unpack("<iiiiid", resp[:28])[1]


def get_pos(axis):
    try:
        err, resp = rpc(CMD_GET_LOGICAL_POS, axis.encode("ascii")[0:1])
    except Exception:
        return None
    return struct.unpack("<d", resp[:8])[0] if (err == 0 and len(resp) >= 8) else None


def get_structure():
    try:
        err, resp = rpc(CMD_GET_PROGRAM_STRUCTURE, b"")
    except Exception:
        return None
    fmt = "<i256siiiiiQQd5d5d"
    if err != 0 or len(resp) < struct.calcsize(fmt):
        return None
    f = struct.unpack(fmt, resp[:struct.calcsize(fmt)])
    return {"ret": f[0], "is_loaded": f[2]}


def wait_status(pred, timeout):
    last = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        last = axis_status()
        if last and pred(last):
            return last
        time.sleep(0.05)
    return None


# ---------------- server 生命周期 ----------------
def stop_server():
    global _proc, _log_fh
    if _proc and _proc.poll() is None:
        _proc.kill()
        _proc.wait()
    _proc = None
    if _log_fh:
        _log_fh.close()
        _log_fh = None
    subprocess.run(["pkill", "-9", "-x", "rpc_server"], stderr=subprocess.DEVNULL)
    time.sleep(0.4)


def start_server(tag, args=("sim",), env_extra=None, timeout=25):
    """返回 (ok, 本次会话之前已存在的事件日志集合)。"""
    global _proc, _log_fh
    stop_server()
    before = set(glob.glob(EVENT_GLOB))
    env = dict(os.environ)
    nc = os.path.join(OUT_DIR, "nc")
    os.makedirs(nc, exist_ok=True)
    env["SMC_PROGRAM_DIR"] = nc
    env.update(env_extra or {})
    _log_fh = open(os.path.join(OUT_DIR, "server_%s.log" % tag), "w")
    _proc = subprocess.Popen([BIN] + list(args), stdout=_log_fh, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, env=env, cwd=PROJECT,
                             start_new_session=True)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if _proc.poll() is not None:
            return False, before
        try:
            socket.create_connection((HOST, PORT), timeout=1).close()
            time.sleep(0.3)
            return True, before
        except Exception:
            time.sleep(0.2)
    return False, before


def server_log():
    _log_fh.flush()
    with open(_log_fh.name, "rb") as f:
        return f.read().decode("utf-8", errors="replace")


def read_events(before):
    time.sleep(0.5)                      # 落盘线程 100ms 周期
    files = [f for f in glob.glob(EVENT_GLOB) if f not in before]
    if not files:
        return []
    path = max(files, key=os.path.getmtime)
    raw = open(path, "rb").read()
    if raw[:4] != b"EVLG" or len(raw) < EVHDR.size:
        return []
    cap = EVHDR.unpack_from(raw, 0)[2]
    out = []
    for k in range(min(cap, (len(raw) - EVHDR.size) // EVT.size)):
        ts, seq, sev, src, code, val, msg = EVT.unpack_from(raw, EVHDR.size + EVT.size * k)
        if seq == 0:
            continue
        out.append({"seq": seq, "sev": sev, "src": src, "code": code, "val": val,
                    "msg": msg.split(b"\0", 1)[0].decode("utf-8", errors="replace")})
    out.sort(key=lambda e: e["seq"])
    if os.environ.get("SERVO_VERIFY_DEBUG"):
        print("  [events] %s: %d events" % (os.path.basename(path), len(out)))
        for e in out:
            print("    seq=%-5d sev=%d src=%d code=0x%04X val=%-8d %s"
                  % (e["seq"], e["sev"], e["src"], e["code"], e["val"], e["msg"]))
    return out


def count_events(events, src, code, sev=None, contains=None):
    return sum(1 for e in events if e["src"] == src and e["code"] == code
               and (sev is None or e["sev"] == sev)
               and (contains is None or contains in e["msg"]))


# ---------------- 9528 快照 ----------------
def read_snapshot():
    """9528 订阅并读一帧。返回 (ack, frame_hdr, payload, crc_ok)。"""
    s = socket.create_connection((HOST, 9528), timeout=5)
    try:
        s.sendall(struct.pack("<HH", CMD_SUBSCRIBE, 4) + struct.pack("<i", 20))
        ack = SNAP_ACK.unpack(recvn(s, 16))
        hdr_raw = recvn(s, 16)
        hdr = SNAP_HDR.unpack(hdr_raw)
        payload = recvn(s, hdr[2])
    finally:
        s.close()
    crc_ok = (zlib.crc32(hdr_raw[:12] + payload) & 0xFFFFFFFF) == hdr[3]
    return ack, hdr, payload, crc_ok


def run_probe(fault_mode):
    env = dict(os.environ)
    env["SMC_PROGRAM_DIR"] = os.path.join(OUT_DIR, "nc")
    if fault_mode:
        env["OPCUA_PROBE_SERVO_FAULT"] = "1"
    try:
        p = subprocess.run([PROBE], env=env, cwd=OUT_DIR, capture_output=True, timeout=90)
    except Exception as e:
        return None, "probe exception: %s" % e
    text = p.stdout.decode("utf-8", errors="replace")
    open(os.path.join(OUT_DIR, "probe_%s.log" % ("fault" if fault_mode else "normal")), "w").write(text)
    tail = [ln for ln in text.splitlines() if "=== RESULT" in ln]
    return p.returncode, (tail[-1] if tail else "(no RESULT line) rc=%d" % p.returncode)


# ---------------- 用例 ----------------
def t1_t3_t5():
    """T1 部分使能 + 上报, T5 门控, T3 事件有界。"""
    print("\n########## T1/T3/T5  上电部分使能 (SIM_BOOT_FAULT=Z) ##########")
    ok, before = start_server("t1", env_extra={"SIM_BOOT_FAULT": "Z"})
    if not ok:
        report("T1 server starts with a faulted axis", False, "port not ready")
        stop_server()
        return
    st = wait_status(lambda s: s["rdy"] == 0x1B and s["lt"] == 0x04, 6.0)
    report("T1 server alive, axes settled with Z latched", st is not None,
           "" if st else "status never reached rdy=0x1B lt=0x04")
    if not st:
        stop_server()
        return
    report("T1 config_mask == 0x1F (all 5 axes configured)", st["cfg"] == ALL5, hex(st["cfg"]))
    report("T1 ready_mask == 0x1B (X/Y/C/B enabled)", st["rdy"] == 0x1B, hex(st["rdy"]))
    report("T1 latched_mask == 0x04 (Z)", st["lt"] == 0x04, hex(st["lt"]))
    z = st["axes"][Z]
    report("T1 Z state == FAULT_LATCHED, stage == handshake, retries == 3",
           z["state"] == ST_LATCHED and z["stage"] == 0 and z["retry"] == 3,
           "state=%d stage=%d retry=%d" % (z["state"], z["stage"], z["retry"]))
    x = st["axes"][X]
    report("T1 X state == OP_ENABLED, status word 0x27",
           x["state"] == ST_OP and (x["sw"] & 0x6F) == 0x27,
           "state=%d sw=0x%04X" % (x["state"], x["sw"]))
    report("T1 Y/C/B state == OP_ENABLED",
           all(st["axes"][k]["state"] == ST_OP for k in (Y, C, B)),
           str([st["axes"][k]["state"] for k in (Y, C, B)]))
    report("T1 Z status word fault bit set", (z["sw"] & 0x0008) != 0, "sw=0x%04X" % z["sw"])

    rc, line = run_probe(fault_mode=True)
    report("T1 OPC UA servo-fault probe all PASS", rc == 0, line)

    # ---- T5 门控 ----
    e, r = jog_start("X", 1, 10.0)
    report("T5 JogStart(X) accepted (ret 0)", e == 0 and r == 0, "err=%s ret=%s" % (e, r))
    time.sleep(0.8)
    jog_stop("X")
    time.sleep(0.3)
    px = get_pos("X")
    report("T5 X actually moves while Z latched", px is not None and px > 1.0, "X=%s" % px)
    e, r = jog_start("Z", 1, 10.0)
    report("T5 JogStart(Z) rejected (-3)", r == -3, "ret=%s" % r)
    e, r = rpc_int(CMD_CONFIG_HOMING_ORDER, "ZXYBC".ljust(16, "\0").encode("ascii"))
    report("T5 homing order configured", r == 0, "ret=%s" % r)
    e, r = homing_trigger("Z")
    report("T5 Home.Axis(Z) rejected (-3)", r == -3, "ret=%s" % r)
    e, r = homing_trigger("")
    report("T5 Home.All rejected (-3)", r == -3, "ret=%s" % r)
    e, r = homing_trigger("X")
    report("T5 Home.Axis(X) accepted", r == 0, "ret=%s" % r)
    deadline = time.time() + 6
    while time.time() < deadline and get_homing() not in (3, 4):
        time.sleep(0.1)
    report("T5 Home.Axis(X) completes (state DONE)", get_homing() == 3, "state=%s" % get_homing())

    path = os.path.join(NC_DIR, "motion_01_x_axis.nc")
    e, r = rpc_int(CMD_LOAD_PROGRAM, path.encode("utf-8")[:255].ljust(256, b"\0"))
    report("T5 LoadProgram (X only, preview) accepted", r == 0, "ret=%s" % r)
    deadline = time.time() + 10
    stc = None
    while time.time() < deadline:
        stc = get_structure()
        if stc and stc["is_loaded"] in (1, 2, 3):
            break
        time.sleep(0.2)
    e, r = rpc_int(CMD_RUN_LOADED_PROGRAM, b"")
    report("T5 Program.Start rejected (-4: servo not all ready)", r == -4, "ret=%s" % r)

    # ---- T3 事件有界 ----
    ev = read_events(before)
    n_det = count_events(ev, 4, 0x0002)
    n_lat = count_events(ev, 4, 0x000F)
    n_txt_w = count_events(ev, 4, 0x0053, sev=2, contains="Z servo fault")
    n_txt_a = count_events(ev, 4, 0x0053, sev=3, contains="Z servo fault")
    report("T3 fault detected exactly once (0x0002 x1)", n_det == 1, "n=%d" % n_det)
    report("T3 latched event exactly once (0x000F x1)", n_lat == 1, "n=%d" % n_lat)
    report("T3 diagnosis text WARN x1 + ALARM x1 with 'Z servo fault'",
           n_txt_w == 1 and n_txt_a == 1, "warn=%d alarm=%d" % (n_txt_w, n_txt_a))
    report("T3 no re-enable event (0x0054) for a latched axis",
           count_events(ev, 4, 0x0054) == 0, "")
    n0 = len(ev)
    time.sleep(2.0)
    ev2 = read_events(before)
    report("T3 no event flood while latched (quiet for 2s)", len(ev2) == n0,
           "%d -> %d" % (n0, len(ev2)))
    stop_server()


def t2_selfheal():
    print("\n########## T2  故障在重试用尽前撤销 → 自愈 ##########")
    ok, before = start_server("t2", env_extra={"SIM_BOOT_FAULT": "Z", "SIM_BOOT_FAULT_MS": "600"})
    if not ok:
        report("T2 server starts", False, "port not ready")
        stop_server()
        return
    st = wait_status(lambda s: s["rdy"] == ALL5 and s["lt"] == 0, 6.0)
    report("T2 Z re-enabled automatically (ready 0x1F, latched 0)", st is not None,
           "" if st else "not all ready")
    report("T2 no system alarm after self-heal", sys_status() == "IDLE", sys_status())
    ev = read_events(before)
    report("T2 no latch event (0x000F)", count_events(ev, 4, 0x000F) == 0, "")
    report("T2 re-enable event 0x0054 present", count_events(ev, 4, 0x0054) == 1,
           "n=%d" % count_events(ev, 4, 0x0054))
    report("T2 diagnosis only WARN (no ALARM 0x0053)",
           count_events(ev, 4, 0x0053, sev=3) == 0 and count_events(ev, 4, 0x0053, sev=2) >= 1, "")
    if st:
        report("T2 Z status word back to Operation enabled",
               (st["axes"][Z]["sw"] & 0x6F) == 0x27, "sw=0x%04X" % st["axes"][Z]["sw"])
    stop_server()


def t_runtime_fault_reset():
    print("\n########## T4a  运行期故障 → 自动复位失败 → 锁存 → 人工复位恢复 ##########")
    ok, before = start_server("t4a")
    if not ok:
        report("T4a server starts", False, "")
        stop_server()
        return
    e, r = hook(1, "Z")
    report("T4a fault injected into enabled Z", r == 0, "ret=%s" % r)
    time.sleep(0.1)
    report("T4a system alarm set on runtime fault", sys_status() == "ALARM", sys_status())
    st = wait_status(lambda s: s["lt"] == 0x04, 6.0)
    report("T4a Z latched after 3 failed auto resets", st is not None and st["axes"][Z]["retry"] == 3,
           "" if not st else "retry=%d" % st["axes"][Z]["retry"])
    e, r = rpc_int(CMD_CLEAR_ALARM, b"")
    time.sleep(2.2)
    st2 = axis_status()
    # 设计 (已确认的门控方案): 复位时锁存轴重新握手, 故障仍在则再次锁存; 各轴到终态后系统报警即清除,
    # 健康轴保持可手动 (程序启动仍因全轴未就绪被拒)。
    report("T4a Reset while fault persists: Z re-latched, alarm cleared (healthy axes usable)",
           r == 0 and st2["lt"] == 0x04 and st2["rdy"] == 0x1B and sys_status() == "IDLE",
           "reset=%s lt=0x%X rdy=0x%X status=%s" % (r, st2["lt"], st2["rdy"], sys_status()))
    hook(2, "Z")                          # 故障排除
    e, r = rpc_int(CMD_CLEAR_ALARM, b"")
    st3 = wait_status(lambda s: s["rdy"] == ALL5 and s["lt"] == 0, 5.0)
    report("T4a Reset after fault cleared: Z re-enabled", st3 is not None,
           "" if not st3 else hex(st3["rdy"]))
    report("T4a system alarm cleared by RT", sys_status() == "IDLE", sys_status())
    ev = read_events(before)
    report("T4a runtime fault event marks stage 'run'",
           count_events(ev, 4, 0x0053, contains="run") >= 1, "")
    stop_server()


def t4_t8_bus():
    print("\n########## T8/T4  快照 v5 + 总线故障 ##########")
    ok, before = start_server("t4")
    if not ok:
        report("T4 server starts", False, "")
        stop_server()
        return
    # ---- T8 ----
    try:
        ack, hdr, payload, crc_ok = read_snapshot()
        ver_ok = (ack[0] == 0x534E414B and ack[1] == 5 and hdr[1] == 5 and hdr[2] == 472)
        states = struct.unpack_from("<5s", payload, 440)[0]
        bus = payload[445]
        sws = struct.unpack_from("<5H", payload, 448)
        rdy, lt, cfg = struct.unpack_from("<III", payload, 460)
        report("T8 9528 ack + frame version == 5, payload 472B, crc ok", ver_ok and crc_ok,
               "ack=0x%08X v%d frame v%d len %d crc=%s" % (ack[0], ack[1], hdr[1], hdr[2], crc_ok))
        report("T8 v5 fields match RPC (rdy/lt/cfg = 0x1F/0/0x1F, all OP_ENABLED)",
               rdy == ALL5 and lt == 0 and cfg == ALL5 and states == b"\x02" * 5 and bus == 0,
               "rdy=0x%X lt=0x%X cfg=0x%X states=%s bus=%d sw0=0x%04X" %
               (rdy, lt, cfg, states.hex(), bus, sws[0]))
    except Exception as ex:
        report("T8 9528 snapshot v5 readable", False, "exception: %s" % ex)
    # ---- T4 ----
    hook(3)
    st = wait_status(lambda s: s["bus"] == 1 and s["rdy"] == 0, 2.0)
    report("T4 bus fault latched: bus=1, ready_mask=0", st is not None,
           "" if not st else "states=%s" % [a["state"] for a in st["axes"]])
    if st:
        report("T4 all axes BUS_FAULT", all(a["state"] == ST_BUS for a in st["axes"]), "")
    report("T4 system alarm raised on bus fault", sys_status() == "ALARM", sys_status())
    e, r = jog_start("X", 1, 10.0)
    report("T4 JogStart rejected while bus fault", r == -3, "ret=%s" % r)
    e, r = rpc_int(CMD_CLEAR_ALARM, b"")
    report("T4 Reset rejected (-3) while wkc still bad", r == -3, "ret=%s" % r)
    hook(4)
    time.sleep(0.8)
    st = axis_status()
    report("T4 wkc back but NOT self-healing (still latched)", st["bus"] == 1 and st["rdy"] == 0,
           "bus=%d rdy=0x%X" % (st["bus"], st["rdy"]))
    try:
        ack, hdr, payload, _ = read_snapshot()
        report("T8 snapshot v5 shows bus_fault=1, ready_mask=0",
               payload[445] == 1 and struct.unpack_from("<I", payload, 460)[0] == 0,
               "bus=%d rdy=%d" % (payload[445], struct.unpack_from("<I", payload, 460)[0]))
    except Exception as ex:
        report("T8 snapshot v5 during bus fault", False, "exception: %s" % ex)
    e, r = rpc_int(CMD_CLEAR_ALARM, b"")
    st = wait_status(lambda s: s["bus"] == 0 and s["rdy"] == ALL5, 5.0)
    report("T4 Reset after wkc back: bus recovered, all axes re-enabled",
           r == 0 and st is not None, "reset=%s" % r)
    report("T4 system alarm cleared after recovery", sys_status() == "IDLE", sys_status())
    ev = read_events(before)
    report("T4 bus fault event once (0x0050) + recovered once (0x0052)",
           count_events(ev, 4, 0x0050) == 1 and count_events(ev, 4, 0x0052) == 1,
           "0x0050 x%d, 0x0052 x%d" % (count_events(ev, 4, 0x0050), count_events(ev, 4, 0x0052)))
    stop_server()


def t6_shutdown():
    print("\n########## T6  SIGTERM / RPC Close ##########")
    ok, before = start_server("t6a")
    if not ok:
        report("T6 server starts", False, "")
        stop_server()
        return
    idle = socket.create_connection((HOST, PORT), timeout=5)   # 挂一个空闲 RPC 连接
    _proc.send_signal(signal.SIGTERM)
    rc = None
    try:
        rc = _proc.wait(timeout=10)
    except Exception:
        pass
    idle.close()
    log = server_log()
    report("T6 SIGTERM with idle RPC client: process exits code 0", rc == 0, "rc=%s" % rc)
    report("T6 graceful power-down sequence ran",
           ("优雅下电完成" in log or "下电时序" in log) and "系统已安全关闭" in log, "")
    report("T6 no stale CX send after close (RT thread exits)", "已退出" in log, "")
    stop_server()

    ok, before = start_server("t6b")
    if not ok:
        report("T6 server restarts", False, "")
        stop_server()
        return
    try:
        err, _ = rpc(CMD_CLOSE, b"", timeout=30.0)
    except Exception as ex:
        err = "exception: %s" % ex
    rc = None
    try:
        rc = _proc.wait(timeout=10)
    except Exception:
        pass
    report("T6 RPC Close: response ok and process exits code 0", err == 0 and rc == 0,
           "err=%s rc=%s" % (err, rc))
    log = server_log()
    report("T6 RPC Close no second power-down (idempotent SMC_Close)",
           log.count("收到关闭请求") == 1, "count=%d" % log.count("收到关闭请求"))
    stop_server()


def t7_three_axes():
    print("\n########## T7  3 轴档案 (sim hw) ##########")
    prof = os.path.join(OUT_DIR, "profile_3axis.ini")
    with open(prof, "w") as f:
        f.write("[general]\nversion=1\naxis_order=X,Y,Z\n"
                "[kinematics]\ntool_len=-999999.0\npivot_z=-999999.0\n")
    ok, before = start_server("t7", args=("sim", "hw"),
                              env_extra={"SMC_MACHINE_PROFILE": prof})
    if not ok:
        report("T7 server starts with 3-axis profile", False, "port not ready")
        stop_server()
        return
    st = wait_status(lambda s: s["rdy"] == 0x07, 6.0)
    report("T7 config_mask 0x07 / ready_mask 0x07 / latched 0",
           st is not None and st["cfg"] == 0x07 and st["lt"] == 0,
           "" if not st else "cfg=0x%X rdy=0x%X lt=0x%X" % (st["cfg"], st["rdy"], st["lt"]))
    if st:
        report("T7 C/B not participating (IDLE)", st["axes"][C]["state"] == ST_IDLE
               and st["axes"][B]["state"] == ST_IDLE,
               "C=%d B=%d" % (st["axes"][C]["state"], st["axes"][B]["state"]))
    report("T7 all-ready semantics hold for configured axes (status IDLE, jog X ok)",
           sys_status() == "IDLE" and jog_start("X", 1, 10.0)[1] == 0, sys_status())
    jog_stop("X")
    stop_server()


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    want = [a.upper() for a in sys.argv[1:]] or ["T1", "T2", "T4A", "T4", "T6", "T7"]
    table = {"T1": t1_t3_t5, "T3": t1_t3_t5, "T5": t1_t3_t5,
             "T2": t2_selfheal, "T4A": t_runtime_fault_reset,
             "T4": t4_t8_bus, "T8": t4_t8_bus, "T6": t6_shutdown, "T7": t7_three_axes}
    done = set()
    try:
        for t in want:
            if t in table and table[t] not in done:
                done.add(table[t])
                table[t]()
    finally:
        stop_server()
    print("\n========================================")
    print("逐轴伺服使能 验证总结")
    print("========================================")
    n_pass = sum(1 for _, ok, _ in _results if ok)
    n_fail = len(_results) - n_pass
    for name, ok, detail in _results:
        if not ok:
            print("  [FAIL] %s  %s" % (name, detail))
    print("=== RESULT: %d PASS, %d FAIL ===" % (n_pass, n_fail))
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
