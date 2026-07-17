#!/usr/bin/env python3
"""safe_lift_rpc.py  ——  P0-3 Safe Z Lift RPC 客户端 (config / trigger / cancel / get)

用法:
    # 配置 (init 阶段, 在 SMC_InitAndStart 之前调; sim 模式下需 rpc_server 启动后调)
    python3 safe_lift_rpc.py config --z=Z --safe_z=50.0 --speed=20.0 --auto=1

    # 手动触发 (加工中或 alarm 后)
    python3 safe_lift_rpc.py trigger

    # 取消 (仅 PENDING/DONE, RUNNING 拒绝 -1)
    python3 safe_lift_rpc.py cancel

    # 查询状态
    python3 safe_lift_rpc.py get

    # 可选参数: --host=127.0.0.1 --port=9527

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 scripts/safe_lift_rpc.py config --z=Z --safe_z=50.0 --speed=20.0 --auto=1
          python3 scripts/safe_lift_rpc.py trigger
          python3 scripts/safe_lift_rpc.py get
"""
import socket
import struct
import sys

# RPC cmd (与 smc_protocol.h 一致)
CMD_CONFIG_SAFE_LIFT  = 0x0057
CMD_SAFE_LIFT_TRIGGER = 0x0058
CMD_SAFE_LIFT_CANCEL  = 0x0059
CMD_GET_SAFE_LIFT     = 0x005A

STATE_NAMES = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "DONE"}


def recvn(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n}")
        buf += chunk
    return buf


def parse_args():
    host = "127.0.0.1"
    port = 9527
    cmd = None
    params = {"z": "Z", "safe_z": 50.0, "speed": 20.0, "auto": 1}
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        sys.exit(2)
    cmd = args[0]
    for a in args[1:]:
        if a.startswith("--host="):
            host = a.split("=", 1)[1]
        elif a.startswith("--port="):
            port = int(a.split("=", 1)[1])
        elif a.startswith("--z="):
            params["z"] = a.split("=", 1)[1]
        elif a.startswith("--safe_z="):
            params["safe_z"] = float(a.split("=", 1)[1])
        elif a.startswith("--speed="):
            params["speed"] = float(a.split("=", 1)[1])
        elif a.startswith("--auto="):
            params["auto"] = int(a.split("=", 1)[1])
    return host, port, cmd, params


def rpc_call(host, port, cmd_id, payload=b""):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((host, port))
    except Exception as e:
        print(f"[rpc] connect failed: {e}", file=sys.stderr)
        sys.exit(1)
    req_hdr = struct.pack("<HH", cmd_id, len(payload))
    s.sendall(req_hdr + payload)
    hdr = recvn(s, 8)
    err_code, data_len = struct.unpack("<iI", hdr)
    resp = recvn(s, data_len) if data_len > 0 else b""
    s.close()
    return err_code, resp


def cmd_config(host, port, p):
    # SmcConfigSafeLiftReq (pack(1)): uint8 z_letter, uint8[3] pad, double safe_z, double speed, int32 auto
    payload = struct.pack("<c3sddi",
                          p["z"].encode("ascii")[0:1],
                          b"\x00\x00\x00",
                          p["safe_z"], p["speed"], p["auto"])
    err, resp = rpc_call(host, port, CMD_CONFIG_SAFE_LIFT, payload)
    if err != 0 or len(resp) < 4:
        print(f"[rpc] ConfigSafeLiftZ err={err} resp_len={len(resp)}")
        return -1
    ret = struct.unpack("<i", resp[:4])[0]
    print(f"[rpc] ConfigSafeLiftZ z={p['z']} target={p['safe_z']}mm "
          f"speed={p['speed']}mm/s auto_on_alarm={p['auto']} ret={ret} "
          f"(0=ok, -1=轴未配置/参数非法/旋转轴, -2=超软限位)")
    return ret


def cmd_trigger(host, port):
    err, resp = rpc_call(host, port, CMD_SAFE_LIFT_TRIGGER)
    if err != 0 or len(resp) < 4:
        print(f"[rpc] TriggerSafeLiftZ err={err}")
        return -1
    ret = struct.unpack("<i", resp[:4])[0]
    print(f"[rpc] TriggerSafeLiftZ ret={ret} (0=已提交, -1=未配置)")
    return ret


def cmd_cancel(host, port):
    err, resp = rpc_call(host, port, CMD_SAFE_LIFT_CANCEL)
    if err != 0 or len(resp) < 4:
        print(f"[rpc] CancelSafeLiftZ err={err}")
        return -1
    ret = struct.unpack("<i", resp[:4])[0]
    print(f"[rpc] CancelSafeLiftZ ret={ret} (0=已取消, -1=未配置或RUNNING中拒绝)")
    return ret


def cmd_get(host, port):
    err, resp = rpc_call(host, port, CMD_GET_SAFE_LIFT)
    # SmcGetSafeLiftRes (pack(1)): int32 ret_code, int32 state, int32 enabled, int32 pad,
    #                               double progress_mm, double z_target_mm, double z_current_mm
    if err != 0 or len(resp) < 36:
        print(f"[rpc] GetSafeLiftState err={err} resp_len={len(resp)}")
        return -1
    ret, state, enabled, _pad, progress, z_target, z_current = struct.unpack(
        "<iiiiddd", resp[:36])
    state_str = STATE_NAMES.get(state, f"?({state})")
    print(f"[rpc] GetSafeLiftState:")
    print(f"      ret_code     = {ret}")
    print(f"      enabled      = {enabled}")
    print(f"      state        = {state} ({state_str})")
    print(f"      z_target_mm  = {z_target:.3f}")
    print(f"      z_current_mm = {z_current:.3f}")
    print(f"      progress_mm  = {progress:.3f}")
    return ret


def main():
    host, port, cmd, p = parse_args()
    if cmd == "config":
        return 0 if cmd_config(host, port, p) == 0 else 1
    elif cmd == "trigger":
        return 0 if cmd_trigger(host, port) == 0 else 1
    elif cmd == "cancel":
        return 0 if cmd_cancel(host, port) == 0 else 1
    elif cmd == "get":
        cmd_get(host, port)
        return 0
    else:
        print(f"unknown cmd: {cmd}", file=sys.stderr)
        print(__doc__)
        return 2


if __name__ == "__main__":
    sys.exit(main())
