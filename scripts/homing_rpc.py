#!/usr/bin/env python3
"""homing_rpc.py  ——  P0-1 Homing RPC 客户端 (config / trigger / cancel / get)

用法:
    # 配置回零顺序 (init 阶段)
    python3 homing_rpc.py config_order --order=ZXYBC

    # 配置单轴回零参数 (可选, 默认 method=35 timeout=10000)
    python3 homing_rpc.py config_axis --z=Z --method=35 --timeout=10000

    # 单轴回零
    python3 homing_rpc.py trigger --z=Z

    # 全轴串行回零 (Z → X → Y → B → C)
    python3 homing_rpc.py trigger --z=

    # 取消 (仅 PENDING/DONE)
    python3 homing_rpc.py cancel

    # 查询状态
    python3 homing_rpc.py get

    # 可选: --host=127.0.0.1 --port=9527

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 scripts/homing_rpc.py config_order --order=ZXYBC
          python3 scripts/homing_rpc.py trigger --z=
          python3 scripts/homing_rpc.py get
"""
import socket
import struct
import sys

CMD_CONFIG_HOMING_AXIS  = 0x005B
CMD_CONFIG_HOMING_ORDER = 0x005C
CMD_HOMING_TRIGGER      = 0x005D
CMD_HOMING_CANCEL       = 0x005E
CMD_GET_HOMING          = 0x005F

STATE_NAMES = {0: "IDLE", 1: "PENDING", 2: "RUNNING", 3: "DONE", 4: "FAULT"}


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
    p = {"z": "", "order": "ZXYBC", "method": 35, "timeout": 10000,
         "search": 10.0, "creep": 1.0, "direction": 1}
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
            p["z"] = a.split("=", 1)[1]
        elif a.startswith("--order="):
            p["order"] = a.split("=", 1)[1]
        elif a.startswith("--method="):
            p["method"] = int(a.split("=", 1)[1])
        elif a.startswith("--timeout="):
            p["timeout"] = int(a.split("=", 1)[1])
        elif a.startswith("--search="):
            p["search"] = float(a.split("=", 1)[1])
        elif a.startswith("--creep="):
            p["creep"] = float(a.split("=", 1)[1])
        elif a.startswith("--direction="):
            p["direction"] = int(a.split("=", 1)[1])
    return host, port, cmd, p


def rpc_call(host, port, cmd_id, payload=b""):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(15.0)
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


def cmd_config_axis(host, port, p):
    # SmcConfigHomingAxisReq (pack(1)): u8 z + 3 pad + i32 method + d search + d creep + i32 direction + i32 timeout
    payload = struct.pack("<c3sddi",
                          p["z"].encode("ascii")[0:1] if p["z"] else b"Z",
                          b"\x00\x00\x00",
                          p["method"],
                          p["search"], p["creep"],
                          p["direction"]) + struct.pack("<i", p["timeout"])
    err, resp = rpc_call(host, port, CMD_CONFIG_HOMING_AXIS, payload)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    print(f"[rpc] ConfigHomingAxis z={p['z']} method={p['method']} ret={ret} "
          f"(0=ok, -1=轴未配置/参数非法/运行中, -3=method v1 不支持)")
    return ret


def cmd_config_order(host, port, p):
    # SmcConfigHomingOrderReq: char[16]
    order = p["order"].ljust(16, '\0')[:16].encode("ascii")
    err, resp = rpc_call(host, port, CMD_CONFIG_HOMING_ORDER, order)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    print(f"[rpc] ConfigHomingAll order={p['order']} ret={ret} "
          f"(0=ok, -1=空, -2=未配置的轴)")
    return ret


def cmd_trigger(host, port, p):
    # SmcHomingTriggerReq: u8 z + 3 pad  ('\0' = HomeAll)
    z = p["z"].encode("ascii")[0:1] if p["z"] else b"\0"
    payload = z + b"\x00\x00\x00"
    err, resp = rpc_call(host, port, CMD_HOMING_TRIGGER, payload)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    if p["z"]:
        print(f"[rpc] HomeAxis {p['z']} ret={ret} (0=ok, -1=parser busy/冲突)")
    else:
        print(f"[rpc] HomeAll ret={ret} (0=ok, -1=parser busy/冲突)")
    return ret


def cmd_cancel(host, port):
    err, resp = rpc_call(host, port, CMD_HOMING_CANCEL)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    print(f"[rpc] CancelHoming ret={ret} (0=ok, -1=未配置)")
    return ret


def cmd_get(host, port):
    err, resp = rpc_call(host, port, CMD_GET_HOMING)
    # SmcGetHomingRes (pack(1)): 5×i32(20B) + 1×double(8B) = 28B (非 32B!)
    if err != 0 or len(resp) < 28:
        print(f"[rpc] GetHomingState err={err} resp_len={len(resp)}")
        return -1
    # SmcGetHomingRes (pack(1)): i32 ret + i32 state + i32 axis + i32 enabled + i32 pad + double progress
    ret, state, axis_idx, enabled, _pad, progress = struct.unpack("<iiiiid", resp[:28])
    state_str = STATE_NAMES.get(state, f"?({state})")
    print(f"[rpc] GetHomingState:")
    print(f"      ret_code     = {ret}")
    print(f"      enabled      = {enabled}")
    print(f"      state        = {state} ({state_str})")
    print(f"      axis_idx     = {axis_idx} (-1=HomeAll 顺序模式)")
    print(f"      progress_pct = {progress:.3f}")
    return ret


def main():
    host, port, cmd, p = parse_args()
    if cmd == "config_axis":
        return 0 if cmd_config_axis(host, port, p) == 0 else 1
    elif cmd == "config_order":
        return 0 if cmd_config_order(host, port, p) == 0 else 1
    elif cmd == "trigger":
        return 0 if cmd_trigger(host, port, p) == 0 else 1
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
