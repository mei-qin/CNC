#!/usr/bin/env python3
"""run_gcode_rpc.py — 通过 RPC (9527) 触发 G 代码加工
用法: python3 run_gcode_rpc.py [nc_filepath]
"""
import socket, struct, sys

def main():
    host = "127.0.0.1"
    port = 9527
    filepath = sys.argv[1] if len(sys.argv) > 1 else "tests/gcode/test_snapshot_e2e.nc"

    # 补全为 WSL2 内绝对路径
    if not filepath.startswith("/"):
        filepath = "/mnt/d/code/CNC/" + filepath

    # 固定 256 字节, null 填充
    path_bytes = filepath.encode("utf-8")[:255].ljust(256, b"\0")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((host, port))
    except Exception as e:
        print(f"connect failed: {e}")
        sys.exit(1)

    # SmcReqHeader: cmd_type=0x0040, data_len=256  (little-endian uint16)
    req_hdr = struct.pack("<HH", 0x0040, 256)
    s.sendall(req_hdr + path_bytes)
    print(f"[rpc] sent RUN_GCODE_FILE: {filepath}")

    # 读响应: SmcResHeader 8B + payload
    hdr = s.recv(8)
    if len(hdr) < 8:
        print(f"[rpc] short response: {len(hdr)} bytes")
        s.close(); return
    err_code, data_len = struct.unpack("<iI", hdr)
    if err_code != 0:
        print(f"[rpc] protocol err: {err_code}")
        s.close(); return

    if data_len >= 4:
        payload = s.recv(data_len)
        ret_code = struct.unpack("<i", payload[:4])[0]
        print(f"[rpc] RunGCodeFile ret={ret_code}")
    else:
        print(f"[rpc] no payload, err_code={err_code}")

    s.close()

if __name__ == "__main__":
    main()
