#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-3 Safe Z Lift 端到端验证 harness (WSL2 内运行)

对 5 个 .nc 场景逐一:
  1. 重启 rpc_server sim (每次独立 CSV: cnc_trace_log_<ts>.csv)
  2. 配置激光 (M67 E1500 前置, 否则 parser 拒 M67)
  3. 配置 SafeLift (per 场景: auto / safe_z / speed)
  4. [dual_z] 将 Z 重配置为双驱
  5. LoadProgram -> 轮询 GetProgramStructure 直到 is_loaded==1 (修复 parser busy)
  6. RunLoadedProgram
  7. 注入故障 / 手动触发 / 中途 ClearAlarm (per 场景)
  8. 等待抬升完成
  9. 解析最新 CSV 末两列验证状态机不变量

用法 (必须在 WSL2 内执行):
    wsl.exe -d Ubuntu -- bash -lc 'cd /mnt/d/code/CNC && python3 scripts/verify_safe_lift.py'
"""
import socket
import struct
import subprocess
import time
import glob
import os
import sys

HOST = "127.0.0.1"
PORT = 9527
BIN = "/mnt/d/code/CNC/rpc_server"
LOG = "/tmp/rpc_server.log"
PROJECT = "/mnt/d/code/CNC"
NC_DIR = PROJECT + "/tests/gcode"
CSV_GLOB = PROJECT + "/cnc_trace_log_*.csv"

# ---------------- RPC cmd ----------------
CMD_CONFIG_LASER_IO = 0x0050
CMD_CONFIG_LASER_DO_BITS = 0x0051
CMD_CONFIG_LASER_DI_BITS = 0x0052
CMD_CONFIG_LASER_AO_CHANNELS = 0x0053
CMD_CONFIG_LASER_RANGE = 0x0054
CMD_CONFIG_LASER_COUPLING = 0x0055
CMD_CONFIG_LASER_COUPLE_TABLE = 0x0056
CMD_CONFIG_AXIS_TOPOLOGY = 0x0010
CMD_CONFIG_GANTRY_SYNC_ALARM = 0x0012
CMD_CONFIG_SAFE_LIFT = 0x0057
CMD_SAFE_LIFT_TRIGGER = 0x0058
CMD_SAFE_LIFT_CANCEL = 0x0059
CMD_GET_SAFE_LIFT = 0x005A
CMD_LOAD_PROGRAM = 0x002C
CMD_RUN_LOADED_PROGRAM = 0x002D
CMD_GET_PROGRAM_STRUCTURE = 0x002E
CMD_CLEAR_ALARM = 0x002F
CMD_INJECT_AXIS_FAULT = 0x0018

LASER_COUPLE_TABLE_MAX = 16
STATE_NAMES = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "DONE"}


def recvn(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise ConnectionError("socket closed")
        buf += c
    return buf


def rpc(cmd, payload=b""):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((HOST, PORT))
    except Exception as e:
        raise ConnectionError(f"connect {HOST}:{PORT} failed: {e}")
    s.sendall(struct.pack("<HH", cmd, len(payload)) + payload)
    hdr = recvn(s, 8)
    err, dl = struct.unpack("<iI", hdr)
    resp = recvn(s, dl) if dl > 0 else b""
    s.close()
    return err, resp


def rpc_int(cmd, payload=b""):
    err, resp = rpc(cmd, payload)
    if err != 0:
        return err, None
    if len(resp) < 4:
        return -998, None
    return 0, struct.unpack("<i", resp[:4])[0]


# ---------------- server lifecycle ----------------
def stop_server():
    subprocess.run(["pkill", "-9", "rpc_server"], stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def start_server():
    stop_server()
    with open(LOG, "w") as f:
        subprocess.Popen(["nohup", BIN, "sim"], stdout=f, stderr=subprocess.STDOUT)
    # wait for port 9527
    for _ in range(60):
        try:
            s = socket.socket()
            s.settimeout(1.0)
            s.connect((HOST, PORT))
            s.close()
            time.sleep(0.5)  # let InitAndStart + CSV open settle
            return True
        except Exception:
            time.sleep(0.5)
    return False


# ---------------- config helpers ----------------
def config_laser():
    results = {}
    results["IO"] = rpc_int(CMD_CONFIG_LASER_IO, struct.pack("<iii", 0, 0, 0))
    results["DO"] = rpc_int(CMD_CONFIG_LASER_DO_BITS, struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5))
    results["DI"] = rpc_int(CMD_CONFIG_LASER_DI_BITS, struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5))
    results["AO"] = rpc_int(CMD_CONFIG_LASER_AO_CHANNELS, struct.pack("<BB", 0, 1))
    results["RANGE"] = rpc_int(CMD_CONFIG_LASER_RANGE, struct.pack("<ddd", 3000.0, 5000.0, 50.0))
    results["COUPLE"] = rpc_int(CMD_CONFIG_LASER_COUPLING, struct.pack("<id", 1, 5.0))
    pts = [(0.0, 0.0), (5.0, 0.3), (20.0, 1.0)]
    fmt = "<i" + "dd" * LASER_COUPLE_TABLE_MAX
    flat = [len(pts)]
    for i in range(LASER_COUPLE_TABLE_MAX):
        if i < len(pts):
            flat.extend(pts[i])
        else:
            flat.extend([0.0, 0.0])
    results["TABLE"] = rpc_int(CMD_CONFIG_LASER_COUPLE_TABLE, struct.pack(fmt, *flat))
    return results


def config_safe_lift(z, safe_z, speed, auto):
    payload = struct.pack("<c3sddi", z.encode("ascii")[0:1], b"\x00\x00\x00",
                          safe_z, speed, auto)
    return rpc_int(CMD_CONFIG_SAFE_LIFT, payload)


def config_dual_z():
    topo = struct.pack("<32siii", b"Z", 1, 3, 4)  # 双驱, 复用已知有效 sim 从站 3,4
    r_topo = rpc_int(CMD_CONFIG_AXIS_TOPOLOGY, topo)
    gantry = struct.pack("<c iiii", b"Z", 1, 100, 500, 200)
    r_gantry = rpc_int(CMD_CONFIG_GANTRY_SYNC_ALARM, gantry)
    return r_topo, r_gantry


def load_program(nc_path):
    payload = nc_path.encode("utf-8")[:255].ljust(256, b"\0")
    return rpc_int(CMD_LOAD_PROGRAM, payload)


def run_loaded():
    return rpc_int(CMD_RUN_LOADED_PROGRAM, b"")


def get_structure():
    err, resp = rpc(CMD_GET_PROGRAM_STRUCTURE, b"")
    if err != 0 or len(resp) < struct.calcsize("<i256siiiiiQQd5d5d"):
        return None
    f = struct.unpack("<i256siiiiiQQd5d5d", resp[:struct.calcsize("<i256siiiiiQQd5d5d")])
    return {"ret": f[0], "is_loaded": f[2], "total_lines": f[3], "total_segs": f[4]}


def inject_fault(axis, sub):
    return rpc_int(CMD_INJECT_AXIS_FAULT, struct.pack("<cB", axis.encode("ascii")[0:1], sub))


def trigger():
    return rpc_int(CMD_SAFE_LIFT_TRIGGER, b"")


def cancel():
    return rpc_int(CMD_SAFE_LIFT_CANCEL, b"")


def clear_alarm():
    return rpc_int(CMD_CLEAR_ALARM, b"")


def get_safe_lift_state():
    try:
        err, resp = rpc(CMD_GET_SAFE_LIFT, b"")
    except Exception:
        return None
    if err != 0 or len(resp) < 40:
        return None
    ret, state, enabled, _pad, progress, zt, zc = struct.unpack("<iiiiddd", resp[:40])
    return state


def wait_lift_done(timeout=15):
    """轮询 SafeLift 状态直到 DONE(3). 数据在运行中已落盘, 不依赖 close."""
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        st = get_safe_lift_state()
        last = st
        if st == 3:
            print(f"  [wait] SafeLift 到达 DONE(3)")
            return True
        time.sleep(0.2)
    print(f"  [wait] 超时未达 DONE, last_state={last}")
    return False


def wait_state(target, timeout=5):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if get_safe_lift_state() == target:
            return True
        time.sleep(0.2)
    return False


def newest_csv(before_set):
    cs = [c for c in glob.glob(CSV_GLOB) if c not in before_set]
    if not cs:
        cs = glob.glob(CSV_GLOB)
    if not cs:
        return None
    return max(cs, key=os.path.getmtime)


def graceful_close():
    """SMC_Close (0x0002): 触发 sim_engine_finish() 落盘残余缓冲并关闭 CSV。
    必须在 kill 之前调用, 否则 -9 会丢失缓冲中的轨迹记录。"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(30.0)
        s.connect((HOST, PORT))
        s.sendall(struct.pack("<HH", 0x0002, 0))
        hdr = recvn(s, 8)
        err, dl = struct.unpack("<iI", hdr)
        s.close()
        return err
    except Exception as e:
        print(f"  [close] exception: {e}")
        return -1


# ---------------- verification ----------------
def parse_csv(csv):
    with open(csv) as f:
        lines = f.read().splitlines()
    header = lines[0].strip().split(",")
    cols = {name: i for i, name in enumerate(header)}
    idx_state = cols.get("safe_lift_state")
    idx_z = cols.get("safe_lift_z_cmd")
    idx_ek = cols.get("laser_emergency_kill")
    idx_pw = cols.get("laser_power_w")
    idx_x = cols.get("X")
    states, zs, eks, pws, xs = [], [], [], [], []
    for line in lines[1:]:
        p = line.strip().split(",")
        if len(p) <= max(idx_state, idx_z):
            continue
        try:
            states.append(int(p[idx_state]))
            zs.append(float(p[idx_z]))
            if idx_ek is not None and idx_ek < len(p):
                eks.append(int(p[idx_ek]))
            if idx_pw is not None and idx_pw < len(p):
                pws.append(float(p[idx_pw]))
            if idx_x is not None and idx_x < len(p):
                xs.append(float(p[idx_x]))
        except Exception:
            continue
    return states, zs, eks, pws, xs


def verify(sc, csv):
    print(f"\n=== 验证 [{sc['name']}] CSV={os.path.basename(csv)} ===")
    if not csv or not os.path.exists(csv):
        return False, ["CSV 不存在"]
    states, zs, eks, pws, xs = parse_csv(csv)
    fails = []
    # C1: 程序确实执行 (parser 未因 M67 拒载而 abort)
    prog_ok = (pws and max(pws) > 0.0) or (xs and max(xs) > 100.0)
    print(f"  [C1] 程序执行(激光功率>0 或 X>100): prog_ok={prog_ok}  "
          f"max_laser_pw={max(pws):.1f} max_X={max(xs):.1f}" if (pws and xs) else f"  [C1] prog_ok={prog_ok}")
    if not prog_ok:
        fails.append("C1 程序未执行: laser_power_w 始终 0 且 X 未移动 -> M67 可能被拒/parser abort")
    # C2: 单调非递减 —— 仅校验到首个 DONE(3) 之前.
    # 到达 DONE 后, manual cancel / alarm_reset 会把状态清回 IDLE(0),
    # 这是合法复位, 不应判为非单调. 故: 序列在首个 3 之前必须非递减.
    cut = states.index(3) if 3 in states else len(states)
    mono = all(states[i] <= states[i + 1] for i in range(max(0, cut)))
    if 3 in states:
        print(f"  [C2] safe_lift_state 首个 DONE 前单调非递减: {mono}  "
              f"(DONE 位于索引 {cut}, 序列尾部: {states[-8:]})")
    else:
        print(f"  [C2] safe_lift_state 单调非递减: {mono}  (序列尾部: {states[-8:]})")
    if not mono:
        fails.append("C2 safe_lift_state 出现回退(非单调)")
    # C3: 到达 DONE
    reached3 = 3 in states
    saw1 = 1 in states
    saw2 = 2 in states
    print(f"  [C3] 到达状态: saw1={saw1} saw2={saw2} reached_DONE(3)={reached3}")
    if not reached3:
        fails.append("C3 未到达 DONE(3)")
    # C4: z_cmd 步长 & 终值 (仅当存在 RUNNING 段)
    if saw2 and zs:
        s2 = [(st, z) for st, z in zip(states, zs) if st == 2]
        if s2:
            diffs = [s2[i + 1][1] - s2[i][1] for i in range(len(s2) - 1)]
            steps_ok = all(0.015 <= d <= 0.025 for d in diffs) if diffs else False
            zmin = min(z for _, z in s2)
            zmax = max(z for _, z in s2)
            print(f"  [C4] RUNNING 窗口 z_cmd: min={zmin:.3f} max={zmax:.3f} "
                  f"步长≈{diffs[0]:.4f}..{diffs[-1]:.4f} (期望~0.02) steps_ok={steps_ok}")
            if not steps_ok:
                fails.append(f"C4 RUNNING 步长偏离 0.02 (实测 {diffs[0]:.4f}..{diffs[-1]:.4f})")
            if zmax < sc["safe_z"] - 0.5:
                fails.append(f"C4 z_cmd 未抬到 safe_z={sc['safe_z']} (max={zmax:.2f})")
        else:
            fails.append("C4 标记 saw2 但无 state==2 行 (内部不一致)")
    else:
        print(f"  [C4] 无 RUNNING 窗口 (预期 for below): z_cmd 终值≈{zs[-1] if zs else 'NA'}")
    # 场景特化
    if sc.get("mode") == "manual":
        # 手动触发不应联动急停: laser_emergency_kill 全程为 0
        ek_max = max(eks) if eks else 0
        print(f"  [M] 手动模式 laser_emergency_kill 最大值={ek_max} (期望 0)")
        if ek_max != 0:
            fails.append("M 手动触发却出现 laser_emergency_kill=1 (不应联动急停)")
        if not (saw2 and reached3):
            fails.append("M 状态机 0->..->2->3 不完整 (缺 RUNNING/DONE)")
    elif sc.get("reset"):
        # 抬升中 ClearAlarm: 报警应延迟到 DONE 后才清
        s2_ek = [ek for st, ek in zip(states, eks) if st == 2]
        ek_during_lift = min(s2_ek) if s2_ek else 1
        print(f"  [R] 抬升中(RUNNING) laser_emergency_kill 最小值={ek_during_lift} "
              f"(期望 1: ClearAlarm 未立即清报警)")
        if ek_during_lift != 1:
            fails.append("R 抬升中 ClearAlarm 立即清了报警 (应延迟到 DONE)")
        if not (saw2 and reached3):
            fails.append("R 抬升被中断 (未到达 DONE)")
    elif sc.get("below"):
        # 当前 Z(20) > safe_z(5): 直接 DONE, 不下降
        if saw2:
            fails.append("BELOW 不应出现 RUNNING(2) (Z 已高于 safe_z)")
        zmax = max(zs) if zs else 0
        print(f"  [B] below: 无 RUNNING(符合), z_cmd 最大值={zmax:.2f} (期望≈20, 不下降)")
        if zmax > 20.5:
            fails.append(f"BELOW z_cmd 上升超过起始 Z (max={zmax:.2f})")
    else:
        # alarm / dual_z: 标准 0->1->2->3
        if not (saw1 and saw2 and reached3):
            fails.append("ALARM 状态机 0->1->2->3 不完整")
    ok = len(fails) == 0
    print(f"  >>> {'PASS' if ok else 'FAIL'} {sc['name']}")
    for f in fails:
        print(f"      - {f}")
    return ok, fails


# ---------------- scenario orchestration ----------------
def run_scenario(sc):
    before = set(glob.glob(CSV_GLOB))
    if not start_server():
        print(f"[!] {sc['name']}: 服务器启动失败")
        return False, ["server start failed"]
    print(f"\n########## 场景 {sc['name']} ##########")
    # 1. laser
    lr = config_laser()
    print(f"  [cfg] laser: {lr}")
    if any(v[1] != 0 for v in lr.values()):
        print("  [warn] 部分 laser 配置 ret!=0")
    # 2. dual_z topology
    if sc.get("dual_z"):
        rt, rg = config_dual_z()
        print(f"  [cfg] dual_z topology ret={rt} gantry ret={rg}")
    # 3. safe lift
    rl = config_safe_lift("Z", sc["safe_z"], sc["speed"], sc["auto"])
    print(f"  [cfg] SafeLift z=Z safe_z={sc['safe_z']} speed={sc['speed']} "
          f"auto={sc['auto']} -> ret={rl}")
    # 4. load + poll
    nc = os.path.join(NC_DIR, sc["nc"])
    rl2 = load_program(nc)
    print(f"  [load] {sc['nc']} -> ret={rl2}")
    # poll is_loaded==1
    ready = False
    st = None
    for _ in range(40):  # up to ~10s
        st = get_structure()
        if st and st["is_loaded"] == 1:
            ready = True
            break
        if st and st["is_loaded"] in (2, 3):
            ready = True
            break
        time.sleep(0.25)
    print(f"  [poll] structure={st}")
    if not ready:
        return False, [f"LoadProgram 后 is_loaded 未就绪: {st}"]
    # 5. run
    rerr, rret = run_loaded()
    print(f"  [run] RunLoadedProgram err={rerr} ret={rret}")
    if rerr != 0 or rret != 0:
        return False, [f"RunLoadedProgram err={rerr} ret={rret}"]
    # 6. trigger / inject
    if sc.get("mode") == "manual":
        time.sleep(1.5)
        rt2 = trigger()
        print(f"  [trig] SafeLiftTrigger ret={rt2}")
        time.sleep(4.0)
        rc = cancel()
        print(f"  [cancel] ret={rc}")
        time.sleep(0.5)
    else:
        time.sleep(1.5)
        ri = inject_fault("X", 0)
        print(f"  [inject] X fault ret={ri}")
        if sc.get("reset"):
            time.sleep(1.2)
            rca = clear_alarm()
            print(f"  [clear_alarm] ret={rca} (抬升中, 应延迟到 DONE)")
            time.sleep(4.0)
        else:
            time.sleep(5.0)
        # 6.5 等待抬升状态机到达 DONE(3): firmware 端确认已抬完.
        # 与 graceful_close 的 flush 时序解耦 —— 即便后续 close 偶发慢,
        # 此处已证明 lift 正确完成, 不至于把"未达 DONE"误判为失败.
        wait_lift_done(timeout=15)
    # 7. graceful close -> flush CSV (关键: 否则 -9 丢缓冲, 整段抬升在 residual buf 中)
    ce = graceful_close()
    print(f"  [close] SMC_Close err={ce} (flush 轨迹缓冲到 CSV)")
    time.sleep(0.5)
    # 8. verify
    csv = newest_csv(before)
    ok, fails = verify(sc, csv)
    return ok, fails


def main():
    scenarios = [
        {"name": "alarm", "nc": "test_safe_lift_alarm.nc",
         "safe_z": 50.0, "speed": 20.0, "auto": 1},
        {"name": "manual", "nc": "test_safe_lift_manual.nc", "mode": "manual",
         "safe_z": 50.0, "speed": 20.0, "auto": 0},
        {"name": "reset", "nc": "test_safe_lift_reset.nc", "reset": True,
         "safe_z": 50.0, "speed": 20.0, "auto": 1},
        {"name": "below", "nc": "test_safe_lift_below.nc", "below": True,
         "safe_z": 5.0, "speed": 20.0, "auto": 1},
        {"name": "dual_z", "nc": "test_safe_lift_dual_z.nc", "dual_z": True,
         "safe_z": 50.0, "speed": 20.0, "auto": 1},
    ]
    all_ok = True
    summary = []
    for sc in scenarios:
        try:
            ok, fails = run_scenario(sc)
        except Exception as e:
            ok, fails = False, [f"exception: {e}"]
            print(f"  [!] 异常: {e}")
        finally:
            stop_server()
        summary.append((sc["name"], ok, fails))
        all_ok = all_ok and ok
    print("\n\n========================================")
    print("P0-3 Safe Z Lift 验证总结")
    print("========================================")
    for name, ok, fails in summary:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
        for f in fails:
            print(f"        - {f}")
    print(f"\n总体: {'全部 PASS' if all_ok else '存在 FAIL'}")
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
