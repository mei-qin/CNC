#!/usr/bin/env python3
"""jog_rpc.py  ——  P0-1 JOG RPC 客户端 (start / stop / getpos)

用法:
    # 启动 JOG (持续运动直到 stop 或撞软限位)
    python3 jog_rpc.py start --z=Z --direction=1 --speed=20.0
    # direction: +1 正向 / -1 负向; speed mm/s

    # 停止 JOG
    python3 jog_rpc.py stop --z=Z
    python3 jog_rpc.py stop --z=*      # 全停

    # 查询位置
    python3 jog_rpc.py getpos --z=Z

    # 可选: --host=127.0.0.1 --port=9527

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 scripts/jog_rpc.py start --z=Z --direction=1 --speed=20.0
          sleep 2
          python3 scripts/jog_rpc.py stop --z=Z
          python3 scripts/jog_rpc.py getpos --z=Z   (应显示约 +40mm)

method 35 前置工作流:
    1. JogStart 把机器手动拖到机械参考位
    2. getpos 确认位置准确
    3. SMC_HomeAxis (调 homing_rpc.py trigger) 把当前位置标为零
    4. 后续 G 代码基于此零点加工
"""
import socket
import struct
import sys

CMD_JOG_START = 0x0060
CMD_JOG_STOP  = 0x0061


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
    p = {"z": "Z", "direction": 1, "speed": 20.0}
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
        elif a.startswith("--direction="):
            p["direction"] = int(a.split("=", 1)[1])
        elif a.startswith("--speed="):
            p["speed"] = float(a.split("=", 1)[1])
    return host, port, cmd, p


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


def cmd_start(host, port, p):
    # SmcJogStartReq (pack(1)): u8 z + 3 pad + i32 direction + double speed
    payload = struct.pack("<c3sid",
                          p["z"].encode("ascii")[0:1] if p["z"] else b"Z",
                          b"\x00\x00\x00",
                          p["direction"], p["speed"])
    err, resp = rpc_call(host, port, CMD_JOG_START, payload)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    print(f"[rpc] JogStart {p['z']} dir={p['direction']} speed={p['speed']}mm/s ret={ret} "
          f"(0=ok, -1=轴未配置/冲突, -2=direction 非法)")
    return ret


def cmd_stop(host, port, p):
    # SmcJogStopReq: u8 z + 3 pad ('*' = 全停)
    z = p["z"].encode("ascii")[0:1] if p["z"] else b"*"
    payload = z + b"\x00\x00\x00"
    err, resp = rpc_call(host, port, CMD_JOG_STOP, payload)
    ret = struct.unpack("<i", resp[:4])[0] if len(resp) >= 4 else -999
    print(f"[rpc] JogStop {p['z']} ret={ret} (0=ok, -1=未在 JOG)")
    return ret


def main():
    host, port, cmd, p = parse_args()
    if cmd == "start":
        return 0 if cmd_start(host, port, p) == 0 else 1
    elif cmd == "stop":
        return 0 if cmd_stop(host, port, p) == 0 else 1
    else:
        print(f"unknown cmd: {cmd}", file=sys.stderr)
        print(__doc__)
        return 2


if __name__ == "__main__":
    sys.exit(main())
