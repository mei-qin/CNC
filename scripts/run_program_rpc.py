#!/usr/bin/env python3
"""run_program_rpc.py  ——  P0-b v2 程序加载/执行/查询 RPC 测试客户端

用法:
    python3 run_program_rpc.py [filepath] [--mode=legacy|load|run|structure]
                               [--host=127.0.0.1] [--port=9527]

模式:
    legacy     SMC_RunGCodeFile (现有, 直接跑), 默认
    load       SMC_LoadProgram (preview, 解析不执行), filepath 必填
    run        SMC_RunLoadedProgram (执行已 load 的程序), filepath 可省
    structure  SMC_GetProgramStructure (查询元数据), filepath 可省

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 run_program_rpc.py tests/gcode/test_m3_s_same_line.nc --mode=load
          python3 run_program_rpc.py --mode=structure
          python3 run_program_rpc.py --mode=run
"""
import socket
import struct
import sys
import argparse


# ---- RPC cmd ids (与 rpc/smc_protocol.h 一致) ----
CMD_RUN_GCODE_FILE        = 0x0040
CMD_LOAD_PROGRAM          = 0x002C
CMD_RUN_LOADED_PROGRAM    = 0x002D
CMD_GET_PROGRAM_STRUCTURE = 0x002E
CMD_CLEAR_ALARM           = 0x002F

# 路径前缀 (Win → WSL2 路径转换在 client 端做, 与 server 端 TranslatePathForWSL 等价)
PATH_PREFIX = "/mnt/d/code/CNC/"


def recvn(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n}")
        buf += chunk
    return buf


def send_req_recv_resp(host: str, port: int, cmd: int, payload: bytes = b"") -> tuple:
    """发送 RPC 请求, 返回 (err_code, payload_bytes)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((host, port))
    except Exception as e:
        print(f"[rpc] connect failed: {e}", file=sys.stderr)
        sys.exit(1)

    req_hdr = struct.pack("<HH", cmd, len(payload))
    s.sendall(req_hdr + payload)

    hdr = recvn(s, 8)
    err_code, data_len = struct.unpack("<iI", hdr)
    if data_len > 0:
        resp_payload = recvn(s, data_len)
    else:
        resp_payload = b""

    s.close()
    return err_code, resp_payload


def normalize_filepath(filepath: str) -> str:
    """相对路径 → WSL2 绝对路径"""
    if not filepath.startswith("/"):
        filepath = PATH_PREFIX + filepath
    return filepath


def pack_filepath(filepath: str) -> bytes:
    """256 字节定长 filepath (null 填充)"""
    return filepath.encode("utf-8")[:255].ljust(256, b"\0")


def mode_legacy(host: str, port: int, filepath: str):
    """SMC_RunGCodeFile (cmd=0x0040), payload=filepath[256], Res=int32 ret"""
    fp = normalize_filepath(filepath)
    print(f"[rpc] RUN_GCODE_FILE: {fp}")
    err, payload = send_req_recv_resp(host, port, CMD_RUN_GCODE_FILE, pack_filepath(fp))
    if err != 0:
        print(f"[rpc] protocol err: {err}", file=sys.stderr); sys.exit(1)
    ret = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else "???"
    print(f"[rpc] RunGCodeFile ret={ret}")


def mode_load(host: str, port: int, filepath: str):
    """SMC_LoadProgram (cmd=0x002C), payload=filepath[256], Res=int32 ret"""
    fp = normalize_filepath(filepath)
    print(f"[rpc] LOAD_PROGRAM (preview): {fp}")
    err, payload = send_req_recv_resp(host, port, CMD_LOAD_PROGRAM, pack_filepath(fp))
    if err != 0:
        print(f"[rpc] protocol err: {err}", file=sys.stderr); sys.exit(1)
    ret = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else "???"
    print(f"[rpc] LoadProgram ret={ret}  (0=ok, -1=parser busy, -2=filepath invalid)")
    if ret == 0:
        print(f"[rpc] LoadProgram 异步启动, 等 parser 跑完 (g_program_load_done=1)...")


def mode_run(host: str, port: int):
    """SMC_RunLoadedProgram (cmd=0x002D), 无 Req, Res=int32 ret"""
    print(f"[rpc] RUN_LOADED_PROGRAM")
    err, payload = send_req_recv_resp(host, port, CMD_RUN_LOADED_PROGRAM, b"")
    if err != 0:
        print(f"[rpc] protocol err: {err}", file=sys.stderr); sys.exit(1)
    ret = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else "???"
    print(f"[rpc] RunLoadedProgram ret={ret}  (0=ok, -1=LoadProgram 未完成, -2=parser busy)")


def mode_structure(host: str, port: int):
    """SMC_GetProgramStructure (cmd=0x002E), 无 Req, Res=SmcGetProgramStructureRes (~390B)"""
    print(f"[rpc] GET_PROGRAM_STRUCTURE")
    err, payload = send_req_recv_resp(host, port, CMD_GET_PROGRAM_STRUCTURE, b"")
    if err != 0:
        print(f"[rpc] protocol err: {err}", file=sys.stderr); sys.exit(1)

    # 解析 SmcGetProgramStructureRes (#pragma pack(1))
    # 字段顺序与 rpc/smc_protocol.h 一致
    STRUCT_FMT = "<i"                       # ret_code
    STRUCT_FMT += "256s"                    # filepath[256]
    STRUCT_FMT += "iiiii"                   # is_loaded, total_lines, total_segments, num_o_labels, num_n_labels
    STRUCT_FMT += "QQ"                      # first_seg_id, last_seg_id
    STRUCT_FMT += "d"                       # estimated_time_ms
    STRUCT_FMT += "5d5d"                    # bbox_min[5], bbox_max[5]

    if len(payload) < struct.calcsize(STRUCT_FMT):
        print(f"[rpc] payload too short: {len(payload)} < {struct.calcsize(STRUCT_FMT)}",
              file=sys.stderr)
        sys.exit(1)

    fields = struct.unpack(STRUCT_FMT, payload[:struct.calcsize(STRUCT_FMT)])
    (ret_code, filepath_b, is_loaded, total_lines, total_segments,
     num_o, num_n, first_seg, last_seg, est_time,
     bbox_min_x, bbox_min_y, bbox_min_z, bbox_min_b, bbox_min_c,
     bbox_max_x, bbox_max_y, bbox_max_z, bbox_max_b, bbox_max_c) = fields

    filepath = filepath_b.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    is_loaded_names = {0: "未加载", 1: "loaded(待run)", 2: "running", 3: "done", 4: "loading preview"}

    print(f"  ret_code     = {ret_code}")
    print(f"  filepath     = {filepath}")
    print(f"  is_loaded    = {is_loaded} ({is_loaded_names.get(is_loaded, '?')})")
    print(f"  total_lines  = {total_lines}")
    print(f"  total_segs   = {total_segments}")
    print(f"  num_o_labels = {num_o}")
    print(f"  num_n_labels = {num_n}")
    print(f"  first_seg_id = {first_seg}")
    print(f"  last_seg_id  = {last_seg}")
    print(f"  est_time     = {est_time:.1f} ms ({est_time/1000.0:.2f} s)")
    print(f"  bbox_min XYZ = ({bbox_min_x:.2f}, {bbox_min_y:.2f}, {bbox_min_z:.2f})")
    print(f"  bbox_max XYZ = ({bbox_max_x:.2f}, {bbox_max_y:.2f}, {bbox_max_z:.2f})")
    print(f"  bbox size    = ({bbox_max_x-bbox_min_x:.2f}, {bbox_max_y-bbox_min_y:.2f}, {bbox_max_z-bbox_min_z:.2f})")


def mode_clear_alarm(host: str, port: int):
    """SMC_ClearAlarm (cmd=0x002F), 无 Req, Res=int32 ret"""
    print(f"[rpc] CLEAR_ALARM")
    err, payload = send_req_recv_resp(host, port, CMD_CLEAR_ALARM, b"")
    if err != 0:
        print(f"[rpc] protocol err: {err}", file=sys.stderr); sys.exit(1)
    ret = struct.unpack("<i", payload[:4])[0] if len(payload) >= 4 else "???"
    print(f"[rpc] ClearAlarm ret={ret}  (0=ok, -1=parser busy 先 abort, -2=axes not ready)")


def main():
    parser = argparse.ArgumentParser(description="P0-b v2 / P1-b 程序加载/执行/查询/清报警 RPC 客户端")
    parser.add_argument("filepath", nargs="?", default="",
                        help="G 代码文件路径 (load/legacy 模式必填, run/structure/clear_alarm 可省)")
    parser.add_argument("--mode", choices=["legacy", "load", "run", "structure", "clear_alarm"],
                        default="legacy", help="操作模式 (默认 legacy)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9527)
    args = parser.parse_args()

    if args.mode in ("legacy", "load") and not args.filepath:
        print(f"[rpc] mode={args.mode} 需要 filepath 参数", file=sys.stderr)
        sys.exit(1)

    if args.mode == "legacy":
        mode_legacy(args.host, args.port, args.filepath)
    elif args.mode == "load":
        mode_load(args.host, args.port, args.filepath)
    elif args.mode == "run":
        mode_run(args.host, args.port)
    elif args.mode == "structure":
        mode_structure(args.host, args.port)
    elif args.mode == "clear_alarm":
        mode_clear_alarm(args.host, args.port)


if __name__ == "__main__":
    main()
