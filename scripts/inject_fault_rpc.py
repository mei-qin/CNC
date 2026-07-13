#!/usr/bin/env python3
"""inject_fault_rpc.py  ——  SMC_InjectAxisFault RPC 客户端 (sim 故障注入)

用法:
    python3 inject_fault_rpc.py [axis_letter] [slave_subidx] [--host=127.0.0.1] [--port=9527]

默认: axis='X' slave_subidx=0 (主 motor)

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 scripts/inject_fault_rpc.py X 0
"""
import socket
import struct
import sys

CMD_INJECT_AXIS_FAULT = 0x0018

def recvn(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n}")
        buf += chunk
    return buf

def main():
    host = "127.0.0.1"
    port = 9527
    axis = "X"
    subidx = 0

    args = sys.argv[1:]
    if len(args) >= 1:
        axis = args[0]
    if len(args) >= 2:
        subidx = int(args[1])
    for i, a in enumerate(args):
        if a == "--host" and i + 1 < len(args):
            host = args[i + 1]
        if a == "--port" and i + 1 < len(args):
            port = int(args[i + 1])

    # SmcInjectAxisFaultReq: char axis_letter + uint8_t slave_subidx (packed)
    payload = struct.pack("<cB", axis.encode("ascii")[0:1], subidx)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((host, port))
    except Exception as e:
        print(f"[rpc] connect failed: {e}", file=sys.stderr)
        sys.exit(1)

    req_hdr = struct.pack("<HH", CMD_INJECT_AXIS_FAULT, len(payload))
    s.sendall(req_hdr + payload)

    hdr = recvn(s, 8)
    err_code, data_len = struct.unpack("<iI", hdr)
    if data_len > 0:
        resp = recvn(s, data_len)
        ret_code = struct.unpack("<i", resp)[0]
    else:
        ret_code = -999

    s.close()
    print(f"[rpc] InjectAxisFault axis={axis} motor={subidx} ret={ret_code} "
          f"(0=ok, -1=bad axis, -2=not sim)")
    return ret_code

if __name__ == "__main__":
    sys.exit(main())
