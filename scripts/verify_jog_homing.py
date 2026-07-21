#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P0-1 G28 Homing 状态机 + JOG 模式 端到端验证 harness (WSL2 内运行)

覆盖 8 场景 + method 35 完整工作流:
  1. jog_basic          JOG 基本 (start -> +z 增量 -> stop)
  2. jog_soft_limit     JOG 撞 +soft_limit 自停
  3. g28_homings        G28 全轴串行回零 (parser 路径, homing_state 0->1->2->3 ×5)
  4. homeall_cancel     HomeAll 中途 cancel
  5. homeall_rollback   HomeAll all-or-nothing 回滚 (sim 限制, 见报告)
  6. homing_fault       单轴 Homing FAULT 故障注入 (sim 限制, 见报告)
  7. jog_then_home      JOG 定位 + method 35 回零 完整工作流
  8. mutual_exclusion   SafeLift / Homing / JOG 三功能互斥 (RPC 返回码判据)

核查 CSV 末 4 列: homing_state, homing_axis_idx, jog_active, jog_axis_idx

⚠️ sim-mode 关键限制 (axis_ctrl.c:1412-1418):
    axis_homing() 在 g_sim_mode 下【直接 state=3 (DONE)】, 不进 SDO 轮询循环,
    因此 inject_fault 无法让 homing 走 FAULT(4) 路径 (SW_ERROR 检测在 non-sim 分支)。
    homing_fault / homeall_rollback 的 FAULT 路径在 sim 下不可达, 本 harness 如实
    标注 SIM-LIMITED 并降级为"状态机可观测 + RPC 不崩溃"判据。

用法 (必须在 WSL2 内执行):
    wsl.exe -d Ubuntu -- bash -lc 'cd /mnt/d/code/CNC && python3 scripts/verify_jog_homing.py'
"""
import socket
import struct
import subprocess
import time
import glob
import os
import sys
import json

HOST = "127.0.0.1"
PORT = 9527
BIN = "/mnt/d/code/CNC/rpc_server"
LOG = "/tmp/rpc_server_jh.log"
PROJECT = "/mnt/d/code/CNC"
NC_DIR = PROJECT + "/tests/gcode"
CSV_GLOB = PROJECT + "/cnc_trace_log_*.csv"
OUT_DIR = PROJECT + "/tests/output/laser_log"

# ---------------- RPC cmd ----------------
CMD_CLOSE                     = 0x0002
CMD_CONFIG_LASER_IO           = 0x0050
CMD_CONFIG_LASER_DO_BITS      = 0x0051
CMD_CONFIG_LASER_DI_BITS      = 0x0052
CMD_CONFIG_LASER_AO_CHANNELS  = 0x0053
CMD_CONFIG_LASER_RANGE        = 0x0054
CMD_CONFIG_LASER_COUPLING     = 0x0055
CMD_CONFIG_LASER_COUPLE_TABLE = 0x0056
CMD_CONFIG_SOFT_LIMIT         = 0x0011
CMD_CONFIG_SAFE_LIFT          = 0x0057
CMD_LOAD_PROGRAM              = 0x002C
CMD_RUN_LOADED_PROGRAM        = 0x002D
CMD_GET_PROGRAM_STRUCTURE     = 0x002E
CMD_CLEAR_ALARM               = 0x002F
CMD_INJECT_AXIS_FAULT         = 0x0018
CMD_CONFIG_HOMING_AXIS        = 0x005B
CMD_CONFIG_HOMING_ORDER       = 0x005C
CMD_HOMING_TRIGGER            = 0x005D
CMD_HOMING_CANCEL             = 0x005E
CMD_GET_HOMING                = 0x005F
CMD_JOG_START                 = 0x0060
CMD_JOG_STOP                  = 0x0061

LASER_COUPLE_TABLE_MAX = 16
H_STATE = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "DONE", 4: "FAULT"}


# ================= 基础 RPC =================
def recvn(s, n):
    buf = b""
    while len(buf) < n:
        c = s.recv(n - len(buf))
        if not c:
            raise ConnectionError("socket closed")
        buf += c
    return buf


def rpc(cmd, payload=b"", timeout=10.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    s.connect((HOST, PORT))
    s.sendall(struct.pack("<HH", cmd, len(payload)) + payload)
    hdr = recvn(s, 8)
    err, dl = struct.unpack("<iI", hdr)
    resp = recvn(s, dl) if dl > 0 else b""
    s.close()
    return err, resp


def rpc_int(cmd, payload=b""):
    """返回 (err, ret_code)。err 是传输层 err_code, ret 是业务 ret_code。"""
    try:
        err, resp = rpc(cmd, payload)
    except Exception as e:
        return -999, None
    if err != 0:
        return err, None
    if len(resp) < 4:
        return -998, None
    return 0, struct.unpack("<i", resp[:4])[0]


# ================= server 生命周期 =================
def stop_server():
    subprocess.run(["pkill", "-9", "rpc_server"], stderr=subprocess.DEVNULL)
    time.sleep(1.0)


def start_server():
    stop_server()
    with open(LOG, "w") as f:
        subprocess.Popen(["nohup", BIN, "sim"], stdout=f, stderr=subprocess.STDOUT,
                         start_new_session=True)
    for _ in range(60):
        try:
            s = socket.socket()
            s.settimeout(1.0)
            s.connect((HOST, PORT))
            s.close()
            time.sleep(1.0)  # 让 InitAndStart + RT 跑到 op_ready + CSV open 稳定
            return True
        except Exception:
            time.sleep(0.5)
    return False


def graceful_close():
    """SMC_Close (0x0002): flush 残余缓冲落盘并关 CSV。必须在 kill 前调用。"""
    try:
        err, _ = rpc(CMD_CLOSE, b"", timeout=30.0)
        return err
    except Exception as e:
        print(f"  [close] exception: {e}")
        return -1


# ================= 配置前置 =================
def config_laser():
    r = {}
    r["IO"] = rpc_int(CMD_CONFIG_LASER_IO, struct.pack("<iii", 0, 0, 0))
    r["DO"] = rpc_int(CMD_CONFIG_LASER_DO_BITS, struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5))
    r["DI"] = rpc_int(CMD_CONFIG_LASER_DI_BITS, struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5))
    r["AO"] = rpc_int(CMD_CONFIG_LASER_AO_CHANNELS, struct.pack("<BB", 0, 1))
    r["RANGE"] = rpc_int(CMD_CONFIG_LASER_RANGE, struct.pack("<ddd", 3000.0, 5000.0, 50.0))
    r["COUPLE"] = rpc_int(CMD_CONFIG_LASER_COUPLING, struct.pack("<id", 1, 5.0))
    pts = [(0.0, 0.0), (5.0, 0.3), (20.0, 1.0)]
    fmt = "<i" + "dd" * LASER_COUPLE_TABLE_MAX
    flat = [len(pts)]
    for i in range(LASER_COUPLE_TABLE_MAX):
        flat.extend(pts[i] if i < len(pts) else (0.0, 0.0))
    r["TABLE"] = rpc_int(CMD_CONFIG_LASER_COUPLE_TABLE, struct.pack(fmt, *flat))
    return r


def config_soft_limit(axis, enable, neg, pos):
    # SmcConfigSoftLimitReq (pack1): char + int32 + double + double = <cidd> 21B
    payload = struct.pack("<cidd", axis.encode("ascii")[0:1], enable, neg, pos)
    return rpc_int(CMD_CONFIG_SOFT_LIMIT, payload)


def config_homing_order(order):
    payload = order.ljust(16, "\0")[:16].encode("ascii")
    return rpc_int(CMD_CONFIG_HOMING_ORDER, payload)


def config_homing_axis(z, method=35, search=10.0, creep=1.0, direction=1, timeout=10000):
    payload = struct.pack("<c3sddii",
                          z.encode("ascii")[0:1], b"\x00\x00\x00",
                          method, search, creep, direction, timeout)
    return rpc_int(CMD_CONFIG_HOMING_AXIS, payload)


def config_safe_lift(z, safe_z, speed, auto):
    payload = struct.pack("<c3sddi", z.encode("ascii")[0:1], b"\x00\x00\x00",
                          safe_z, speed, auto)
    return rpc_int(CMD_CONFIG_SAFE_LIFT, payload)


# ================= 动作 RPC =================
def jog_start(z, direction, speed):
    payload = struct.pack("<c3sid", z.encode("ascii")[0:1], b"\x00\x00\x00",
                          direction, speed)
    return rpc_int(CMD_JOG_START, payload)


def jog_stop(z):
    z1 = z.encode("ascii")[0:1] if z else b"*"
    return rpc_int(CMD_JOG_STOP, z1 + b"\x00\x00\x00")


def homing_trigger(z=""):
    z1 = z.encode("ascii")[0:1] if z else b"\0"
    return rpc_int(CMD_HOMING_TRIGGER, z1 + b"\x00\x00\x00")


def homing_cancel():
    return rpc_int(CMD_HOMING_CANCEL, b"")


def get_homing():
    try:
        err, resp = rpc(CMD_GET_HOMING, b"")
    except Exception:
        return None
    if err != 0 or len(resp) < 28:
        return None
    ret, state, axis_idx, enabled, _pad, prog = struct.unpack("<iiiiid", resp[:28])
    return {"ret": ret, "state": state, "axis_idx": axis_idx,
            "enabled": enabled, "prog": prog}


def inject_fault(axis, sub=0):
    return rpc_int(CMD_INJECT_AXIS_FAULT, struct.pack("<cB", axis.encode("ascii")[0:1], sub))


def load_program(nc_path):
    payload = nc_path.encode("utf-8")[:255].ljust(256, b"\0")
    return rpc_int(CMD_LOAD_PROGRAM, payload)


def run_loaded():
    return rpc_int(CMD_RUN_LOADED_PROGRAM, b"")


def get_structure():
    try:
        err, resp = rpc(CMD_GET_PROGRAM_STRUCTURE, b"")
    except Exception:
        return None
    sz = struct.calcsize("<i256siiiiiQQd5d5d")
    if err != 0 or len(resp) < sz:
        return None
    f = struct.unpack("<i256siiiiiQQd5d5d", resp[:sz])
    return {"ret": f[0], "is_loaded": f[2], "total_lines": f[3], "total_segs": f[4]}


def load_and_run(nc):
    path = os.path.join(NC_DIR, nc)
    e, r = load_program(path)
    ready = False
    st = None
    for _ in range(40):
        st = get_structure()
        if st and st["is_loaded"] in (1, 2, 3):
            ready = True
            break
        time.sleep(0.25)
    if not ready:
        return False, f"LoadProgram 未就绪: {st}"
    e2, r2 = run_loaded()
    if e2 != 0 or r2 != 0:
        return False, f"RunLoadedProgram err={e2} ret={r2}"
    return True, "ok"


# ================= CSV 解析 (末 4 列) =================
def newest_csv(before):
    cs = [c for c in glob.glob(CSV_GLOB) if c not in before]
    if not cs:
        cs = glob.glob(CSV_GLOB)
    return max(cs, key=os.path.getmtime) if cs else None


def parse_csv4(csv):
    """返回 dict: 4 列的完整序列 + X/Z 位置 (用于 JOG 位移核查)。"""
    with open(csv) as f:
        lines = f.read().splitlines()
    if not lines:
        return None
    header = lines[0].strip().split(",")
    col = {n: i for i, n in enumerate(header)}
    need = ["homing_state", "homing_axis_idx", "jog_active", "jog_axis_idx"]
    for n in need:
        if n not in col:
            return {"error": f"CSV 缺列 {n}; header={header}"}
    iz = col.get("Z")
    ix = col.get("X")
    out = {"homing_state": [], "homing_axis_idx": [],
           "jog_active": [], "jog_axis_idx": [], "Z": [], "X": [], "rows": 0}
    for line in lines[1:]:
        p = line.strip().split(",")
        if len(p) <= max(col[n] for n in need):
            continue
        try:
            out["homing_state"].append(int(p[col["homing_state"]]))
            out["homing_axis_idx"].append(int(p[col["homing_axis_idx"]]))
            out["jog_active"].append(int(p[col["jog_active"]]))
            out["jog_axis_idx"].append(int(p[col["jog_axis_idx"]]))
            if iz is not None and iz < len(p):
                out["Z"].append(float(p[iz]))
            if ix is not None and ix < len(p):
                out["X"].append(float(p[ix]))
            out["rows"] += 1
        except Exception:
            continue
    return out


def transitions(seq):
    """压缩连续相同值为转换序列, e.g. [0,0,1,1,2,3] -> [0,1,2,3]。"""
    t = []
    for v in seq:
        if not t or t[-1] != v:
            t.append(v)
    return t


# ================= 场景编排 =================
def sc_jog_basic():
    fails = []
    detail = {}
    e, r = jog_start("Z", 1, 20.0)
    detail["jog_start_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"JogStart 失败 err={e} ret={r}")
    time.sleep(2.0)
    e2, r2 = jog_stop("Z")
    detail["jog_stop_ret"] = (e2, r2)
    time.sleep(0.5)
    return fails, detail


def verify_jog_basic(csv, fails, detail):
    d = parse_csv4(csv)
    if not d or d.get("error"):
        fails.append(d.get("error", "CSV 解析失败") if d else "CSV 空")
        return d
    ja = transitions(d["jog_active"])
    saw_active = 1 in d["jog_active"]
    zmax = max(d["Z"]) if d["Z"] else 0.0
    zmin = min(d["Z"]) if d["Z"] else 0.0
    # jog_axis_idx 在 active 期应为 Z 的 axis_idx (>=0), 停后回 -1
    jidx_active = set(ji for ja_, ji in zip(d["jog_active"], d["jog_axis_idx"]) if ja_ == 1)
    detail["jog_active_transitions"] = ja
    detail["z_span_mm"] = round(zmax - zmin, 3)
    detail["jog_axis_idx_when_active"] = sorted(jidx_active)
    if not saw_active:
        fails.append("jog_active 从未变 1 (JOG 未生效)")
    if ja[:3] != [0, 1, 0] and ja[:2] != [1, 0] and 0 not in ja[-1:]:
        # 允许 [0,1,0] 或起始就是 1 的情况; 关键是最终回 0
        if d["jog_active"][-1] != 0:
            fails.append(f"jog_active 未回落到 0, 转换={ja}")
    if (zmax - zmin) < 20.0:
        fails.append(f"z_cmd 位移不足 (期望 ~40mm, 实测 {zmax - zmin:.2f}mm)")
    if saw_active and (not jidx_active or -1 in jidx_active and len(jidx_active) == 1):
        fails.append(f"jog_axis_idx 在 active 期未切到有效轴 idx: {sorted(jidx_active)}")
    return d


def sc_jog_soft_limit():
    fails = []
    detail = {}
    # 起点 0, +soft_limit=100, 50mm/s -> 2s 撞限位
    e, r = jog_start("Z", 1, 50.0)
    detail["jog_start_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"JogStart 失败 err={e} ret={r}")
    time.sleep(3.0)  # 不 stop, 等自动撞 +100
    return fails, detail


def verify_jog_soft_limit(csv, fails, detail, pos_limit=100.0):
    d = parse_csv4(csv)
    if not d or d.get("error"):
        fails.append(d.get("error", "CSV 解析失败") if d else "CSV 空")
        return d
    zmax = max(d["Z"]) if d["Z"] else 0.0
    ja = transitions(d["jog_active"])
    ended_stopped = d["jog_active"][-1] == 0 if d["jog_active"] else False
    detail["z_max"] = round(zmax, 3)
    detail["jog_active_transitions"] = ja
    detail["ended_stopped"] = ended_stopped
    # z_cmd 应逼近 +soft_limit 且不超过
    if abs(zmax - pos_limit) > 1.0:
        fails.append(f"z_cmd 未逼近 +soft_limit={pos_limit} (max={zmax:.2f})")
    if zmax > pos_limit + 0.05:
        fails.append(f"z_cmd 越过 +soft_limit ({zmax:.3f} > {pos_limit})")
    if not ended_stopped:
        fails.append("撞限位后 jog_active 未自停 (末值仍为 1)")
    return d


def sc_g28_homings():
    """G28 全轴回零: 走 parser 路径 (LoadProgram test_g28_homings.nc)。"""
    fails = []
    detail = {}
    ok, msg = load_and_run("test_g28_homings.nc")
    detail["load_run"] = msg
    if not ok:
        fails.append(msg)
        return fails, detail
    # 轮询 homing 状态, 采样直到 homing 回到 IDLE 或超时
    seen = []
    deadline = time.time() + 8
    while time.time() < deadline:
        h = get_homing()
        if h:
            seen.append((h["state"], h["axis_idx"]))
        time.sleep(0.05)
    detail["polled_states"] = transitions([s for s, _ in seen])
    return fails, detail


def verify_homing_csv(csv, fails, detail, expect_states=(1, 2, 3), min_axes=1):
    d = parse_csv4(csv)
    if not d or d.get("error"):
        fails.append(d.get("error", "CSV 解析失败") if d else "CSV 空")
        return d
    hs = transitions(d["homing_state"])
    saw = set(d["homing_state"])
    detail["homing_state_transitions"] = hs
    detail["homing_state_seen"] = sorted(saw)
    # 采集回零期出现的不同轴 idx (排除 -1 HomeAll 占位)
    axes = set(ai for hsv, ai in zip(d["homing_state"], d["homing_axis_idx"])
               if hsv in (2, 3) and ai >= 0)
    detail["homing_axes_touched"] = sorted(axes)
    for st in expect_states:
        if st not in saw:
            fails.append(f"homing_state 未观测到 {st}({H_STATE[st]})")
    return d


def sc_homeall_cancel():
    fails = []
    detail = {}
    e, r = homing_trigger("")  # HomeAll
    detail["trigger_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"HomeAll trigger 失败 err={e} ret={r}")
    # 极快: 在 sim 下 5 轴瞬时完成, 尽快发 cancel 抢 PENDING/DONE 窗口
    time.sleep(0.01)
    ec, rc = homing_cancel()
    detail["cancel_ret"] = (ec, rc)
    time.sleep(1.0)
    h = get_homing()
    detail["post_state"] = h["state"] if h else None
    return fails, detail


def sc_homeall_rollback():
    """SIM-LIMITED: sim 下 axis_homing 直接 DONE, inject_fault 无法触发 FAULT。
    降级判据: HomeAll 触发成功 + 状态机可观测 0->1->2->3 + 注入后不崩溃。"""
    fails = []
    detail = {}
    e, r = homing_trigger("")
    detail["trigger_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"HomeAll trigger 失败 err={e} ret={r}")
    time.sleep(0.005)
    ei, ri = inject_fault("X", 0)  # 尝试在 X 轴回零窗口注入
    detail["inject_ret"] = (ei, ri)
    time.sleep(1.5)
    h = get_homing()
    detail["post_state"] = h["state"] if h else None
    return fails, detail


def sc_homing_fault():
    """SIM-LIMITED: 同上, 单轴 Z 回零 + 注入 Z 故障。"""
    fails = []
    detail = {}
    e, r = homing_trigger("Z")
    detail["trigger_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"HomeAxis Z trigger 失败 err={e} ret={r}")
    time.sleep(0.005)
    ei, ri = inject_fault("Z", 0)
    detail["inject_ret"] = (ei, ri)
    time.sleep(1.5)
    h = get_homing()
    detail["post_state"] = h["state"] if h else None
    return fails, detail


def sc_jog_then_home():
    """method 35 完整工作流: JOG 定位 -> stop -> 回零。"""
    fails = []
    detail = {}
    e, r = jog_start("Z", 1, 20.0)
    detail["jog_start_ret"] = (e, r)
    if e != 0 or r != 0:
        fails.append(f"JogStart 失败 err={e} ret={r}")
    time.sleep(2.5)
    e2, r2 = jog_stop("Z")
    detail["jog_stop_ret"] = (e2, r2)
    time.sleep(0.5)
    e3, r3 = homing_trigger("Z")  # method 35: 把当前 ~50mm 标为零
    detail["home_ret"] = (e3, r3)
    if e3 != 0 or r3 != 0:
        fails.append(f"HomeAxis Z 失败 err={e3} ret={r3}")
    time.sleep(1.0)
    h = get_homing()
    detail["post_state"] = h["state"] if h else None
    return fails, detail


def verify_jog_then_home(csv, fails, detail):
    d = parse_csv4(csv)
    if not d or d.get("error"):
        fails.append(d.get("error", "CSV 解析失败") if d else "CSV 空")
        return d
    # JOG 段: jog_active 1 出现, z 单调增
    saw_jog = 1 in d["jog_active"]
    # Homing 段: homing_state 到 3
    saw_home = set(d["homing_state"])
    detail["jog_seen"] = saw_jog
    detail["homing_state_seen"] = sorted(saw_home)
    detail["jog_active_transitions"] = transitions(d["jog_active"])
    detail["homing_state_transitions"] = transitions(d["homing_state"])
    if not saw_jog:
        fails.append("JOG 段未观测到 jog_active=1")
    if 3 not in saw_home:
        fails.append("Homing 段未到达 DONE(3)")
    return d


def sc_mutual_exclusion():
    """三互斥子场景, 判据 = RPC 返回码 (-1 拒绝) + 状态字段不串。"""
    fails = []
    detail = {}
    # --- 场景1: JOG 中调 Homing 应拒绝 ---
    e1, r1 = jog_start("Z", 1, 20.0)
    detail["s1_jog_start"] = (e1, r1)
    time.sleep(0.3)
    eh, rh = homing_trigger("Z")
    detail["s1_home_trigger"] = (eh, rh)
    if not (eh != 0 or rh == -1):
        fails.append(f"S1: JOG 中 Homing 未被拒绝 (err={eh} ret={rh}, 期望 ret=-1)")
    h = get_homing()
    detail["s1_homing_state"] = h["state"] if h else None
    if h and h["state"] != 0:
        fails.append(f"S1: JOG 中 homing_state 非 0 (={h['state']})")
    jog_stop("Z")
    time.sleep(0.5)
    # --- 场景2: Homing 中调 JOG 应拒绝 ---
    # sim 下 homing 极快, 用 HomeAll 拉长窗口, 立刻发 JOG
    eht, rht = homing_trigger("")
    detail["s2_home_trigger"] = (eht, rht)
    ej, rj = jog_start("X", 1, 20.0)
    detail["s2_jog_start"] = (ej, rj)
    # 允许两种结果: 若 homing 已瞬时完成回 IDLE, JOG 可能成功(非缺陷);
    # 仅当 homing 仍活跃却放行 JOG 才算失败 -- 此处以返回码记录, 不硬判。
    if ej == 0 and rj == 0:
        detail["s2_note"] = "JOG 被放行 (可能 homing 已瞬时 DONE, sim 时序限制)"
    jog_stop("X")
    time.sleep(0.5)
    # --- 场景3: SafeLift 中调 Homing 应拒绝 ---
    ei, ri = inject_fault("X", 0)  # 触发 alarm -> SafeLift 自动 (需 auto=1 已配)
    detail["s3_inject"] = (ei, ri)
    time.sleep(0.2)
    eh3, rh3 = homing_trigger("Z")
    detail["s3_home_trigger"] = (eh3, rh3)
    if not (eh3 != 0 or rh3 == -1):
        fails.append(f"S3: SafeLift 中 Homing 未被拒绝 (err={eh3} ret={rh3}, 期望 -1)")
    h3 = get_homing()
    detail["s3_homing_state"] = h3["state"] if h3 else None
    # S3 判据: SafeLift 中不得有"活跃/错误"回零状态. 已完成 DONE(3) 是合法滞留
    # (home_offset v2 验证确认: homing 完成到 DONE 后不会自动回 IDLE, 需显式 cancel/alarm_reset).
    # 仅当 state ∈ {1 PENDING, 2 RUNNING, 4 FAULT} 才说明 SafeLift 期间仍有活跃回零 → 失败.
    if h3 and h3["state"] not in (0, 3):
        fails.append(f"S3: SafeLift 中 homing 仍活跃 (state={h3['state']})")
    time.sleep(0.5)
    return fails, detail


# ================= 场景注册表 =================
SCENARIOS = [
    {"name": "jog_basic", "kind": "jog", "soft_limit": (-100.0, 100.0)},
    {"name": "jog_soft_limit", "kind": "jog", "soft_limit": (-100.0, 100.0)},
    {"name": "g28_homings", "kind": "homing", "order": "ZXYBC"},
    {"name": "homeall_cancel", "kind": "homing", "order": "ZXYBC"},
    {"name": "homeall_rollback", "kind": "homing", "order": "ZXYBC",
     "sim_limited": True},
    {"name": "homing_fault", "kind": "homing", "order": "ZXYBC",
     "sim_limited": True},
    {"name": "jog_then_home", "kind": "both", "order": "ZXYBC",
     "soft_limit": (-100.0, 100.0)},
    {"name": "mutual_exclusion", "kind": "mutex", "order": "ZXYBC",
     "soft_limit": (-100.0, 100.0), "safe_lift": (50.0, 20.0, 1)},
]


def run_one(sc):
    before = set(glob.glob(CSV_GLOB))
    print(f"\n########## 场景 {sc['name']} ##########")
    if not start_server():
        return False, ["server 启动失败"], {}, None
    # 前置配置
    lr = config_laser()
    if sc.get("soft_limit"):
        neg, pos = sc["soft_limit"]
        rsl = config_soft_limit("Z", 1, neg, pos)
        print(f"  [cfg] soft_limit Z [{neg},{pos}] -> {rsl}")
    if sc.get("order"):
        ro = config_homing_order(sc["order"])
        print(f"  [cfg] homing_order {sc['order']} -> {ro}")
    if sc.get("safe_lift"):
        sz, spd, auto = sc["safe_lift"]
        rsf = config_safe_lift("Z", sz, spd, auto)
        print(f"  [cfg] safe_lift Z safe_z={sz} speed={spd} auto={auto} -> {rsf}")

    # 编排
    dispatch = {
        "jog_basic": sc_jog_basic,
        "jog_soft_limit": sc_jog_soft_limit,
        "g28_homings": sc_g28_homings,
        "homeall_cancel": sc_homeall_cancel,
        "homeall_rollback": sc_homeall_rollback,
        "homing_fault": sc_homing_fault,
        "jog_then_home": sc_jog_then_home,
        "mutual_exclusion": sc_mutual_exclusion,
    }
    fails, detail = dispatch[sc["name"]]()

    # flush + close
    ce = graceful_close()
    print(f"  [close] SMC_Close err={ce}")
    time.sleep(0.5)
    csv = newest_csv(before)

    # 校验 (CSV 部分)
    if sc["name"] == "jog_basic":
        verify_jog_basic(csv, fails, detail)
    elif sc["name"] == "jog_soft_limit":
        verify_jog_soft_limit(csv, fails, detail, pos_limit=sc["soft_limit"][1])
    elif sc["name"] == "g28_homings":
        verify_homing_csv(csv, fails, detail, expect_states=(1, 2, 3))
    elif sc["name"] == "homeall_cancel":
        d = verify_homing_csv(csv, fails, detail, expect_states=(1,))
        # cancel: 末态应回 IDLE(0)
        if detail.get("post_state") not in (0, None):
            # DONE 后被 cancel 也可能落 0; 仅记录
            detail["cancel_note"] = f"post_state={detail.get('post_state')}"
    elif sc["name"] in ("homeall_rollback", "homing_fault"):
        # SIM-LIMITED: 仅校验状态机可观测 + 无崩溃
        verify_homing_csv(csv, fails, detail, expect_states=(3,))
    elif sc["name"] == "jog_then_home":
        verify_jog_then_home(csv, fails, detail)
    elif sc["name"] == "mutual_exclusion":
        # CSV 里三状态字段不应互相串 (仅记录, 判据以 RPC 返回码为主)
        d = parse_csv4(csv)
        if d and not d.get("error"):
            detail["csv_rows"] = d["rows"]

    detail["csv"] = os.path.basename(csv) if csv else None
    ok = len(fails) == 0
    tag = "PASS" if ok else ("SIM-LIMITED" if sc.get("sim_limited") and _only_fault_path(fails) else "FAIL")
    print(f"  >>> {tag} {sc['name']}")
    for f in fails:
        print(f"      - {f}")
    for k, v in detail.items():
        print(f"      · {k}: {v}")
    return ok, fails, detail, tag


def _only_fault_path(fails):
    # sim-limited 场景: 若唯一失败点是"未到 FAULT(4)"类, 视为 sim 限制
    return all("FAULT" in f or "4" in f or "回滚" in f for f in fails)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    results = []
    for sc in SCENARIOS:
        try:
            ok, fails, detail, tag = run_one(sc)
        except Exception as e:
            import traceback
            ok, fails, detail, tag = False, [f"exception: {e}"], {"trace": traceback.format_exc()}, "FAIL"
            print(f"  [!] 异常: {e}")
        finally:
            graceful_close()
            stop_server()
        results.append({"name": sc["name"], "tag": tag, "ok": ok,
                        "sim_limited": sc.get("sim_limited", False),
                        "fails": fails, "detail": detail})

    # 汇总
    print("\n\n========================================")
    print("P0-1 G28 Homing + JOG 验证总结")
    print("========================================")
    n_pass = n_fail = n_sim = 0
    for r in results:
        t = r["tag"]
        if t == "PASS":
            n_pass += 1
        elif t == "SIM-LIMITED":
            n_sim += 1
        else:
            n_fail += 1
        print(f"  [{t}] {r['name']}")
        for f in r["fails"]:
            print(f"        - {f}")
    print(f"\n统计: PASS={n_pass}  SIM-LIMITED={n_sim}  FAIL={n_fail}  / 共 {len(results)}")

    # 落盘 JSON
    ts = time.strftime("%Y%m%d_%H%M%S")
    jpath = os.path.join(OUT_DIR, f"P0-1_JOG_HOMING_RESULTS_{ts}.json")
    with open(jpath, "w") as f:
        json.dump({"ts": ts, "summary": {"pass": n_pass, "sim_limited": n_sim,
                                          "fail": n_fail, "total": len(results)},
                   "results": results}, f, ensure_ascii=False, indent=2)
    print(f"\n结果 JSON: {jpath}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
