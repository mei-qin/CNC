#!/usr/bin/env python3
"""event_subscriber.py  ——  P1-b 事件/报警流 Python 验证客户端

用法:
    python3 event_subscriber.py [host] [port] [freq_hz] [from_seq]
    默认: host=127.0.0.1  port=9530  freq=10  from_seq=0

流程:
    1. TCP connect 到事件流端口 (默认 9530)
    2. 发送 subscribe req: SmcReqHeader{cmd=0x0030, data_len=12} + int32 freq + uint64 from_seq
    3. 收 SmcEventAck (16B), 校验 magic + event_size_bytes
    4. 循环收 [SmcEventFrameHeader + N × SmcEvent_t]
    5. 校验 CRC32, 解析 timestamp/severity/source/code/value/message

WSL2 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 event_subscriber.py 127.0.0.1 9530 10 0

    触发 alarm:
    python3 scripts/run_gcode_rpc.py tests/gcode/h2_alarm.nc
    或用 SMC_InjectAxisFault RPC

    清 alarm:
    python3 scripts/run_program_rpc.py --mode=clear_alarm
"""

import socket
import struct
import sys
import zlib
import time
import signal

# ---- 协议常量 ----
EVENT_MAGIC     = 0x45564E54   # "EVNT"
EVENT_ACK_MAGIC = 0x45564E4B   # "EVNK"
EVENT_VERSION   = 1
SMC_CMD_EVENT_SUBSCRIBE = 0x0030

# severity
SEVERITY_NAMES = {1: "INFO", 2: "WARN", 3: "ALARM", 4: "FATAL"}

# source
SOURCE_NAMES = {1: "PARSER", 2: "RT", 3: "LASER", 4: "DRIVE", 5: "PLC", 6: "MANUAL"}

# event code 描述表 (UI 友好显示)
EVENT_CODE_DESC = {
    # DRIVE
    (4, 0x0001): "sync err gantry",
    (4, 0x0002): "drive SW_ERROR",
    (4, 0x0003): "follow err hard",
    (4, 0x0004): "follow err warn",
    (4, 0x0005): "safe_z_lift state",  # P0-3: value=0 done(above) / 1 cancel / 2 running / 3 done / interlock位图(alarm触发)
    (4, 0x0006): "homing state",      # P0-1: value=1 pending / 2 running / 3 done / 4 cancel
    (4, 0x0007): "homing timeout",    # P0-1: value=axis_idx
    (4, 0x0008): "homing FAULT",      # P0-1: value=axis_idx (含 all-or-nothing 回滚)
    (4, 0x0009): "homing method N/A", # P0-1: value=method (v1 仅 35)
    (4, 0x000B): "jog soft limit",    # P0-1: value=axis_idx
    # LASER
    (3, 0x0010): "laser safety door",
    (3, 0x0011): "estop soft",
    (3, 0x0012): "laser ALM",
    (3, 0x0013): "water/gas interlock",
    # PARSER
    (1, 0x0020): "M3/M4 spindle_rpm<=0",
    (1, 0x0021): "M67/M68 oob",
    (1, 0x0022): "G04 P neg",
    (1, 0x0023): "M70/M71 oob",
    (1, 0x0030): "LoadProgram start",
    (1, 0x0031): "LoadProgram done",
    (1, 0x0032): "RunLoadedProgram start",
    (1, 0x0033): "program done (M30)",
    (1, 0x0034): "program abort",
    # MANUAL
    (6, 0x0040): "ClearAlarm req",
    (6, 0x0041): "alarm cleared by RT",
    (6, 0x0042): "override changed",       # P2-A
    (6, 0x0043): "safe_z_lift manual",     # P0-3: value=0 pending / 1 cancelled
    (6, 0x0044): "safe_lift config reject",# P0-3: value=safe_z_mm × 100
    (6, 0x004A): "jog start",              # P0-1: value=axis_idx
}

REQ_HDR_FMT = "<HH"
SUB_FMT = "<iQ"
ACK_FMT = "<IIII"
ACK_SIZE = 16
FRAME_HDR_FMT = "<IIII"
FRAME_HDR_SIZE = 16

# SmcEvent_t (#pragma pack(1), 88B)
EVENT_FMT = "<QQBBHi64s"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
# 预期 88, 实际由 ack.event_size_bytes 校验


def recvn(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n}")
        buf += chunk
    return buf


def parse_event(payload: bytes) -> dict:
    """解析 88B SmcEvent_t"""
    (timestamp_ms, event_seq, severity, source, code, value, message_b) = \
        struct.unpack(EVENT_FMT, payload)
    message = message_b.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return {
        "timestamp_ms": timestamp_ms,
        "event_seq":    event_seq,
        "severity":     severity,
        "source":       source,
        "code":         code,
        "value":        value,
        "message":      message,
    }


def main():
    host     = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port     = int(sys.argv[2]) if len(sys.argv) > 2 else 9530
    freq     = int(sys.argv[3]) if len(sys.argv) > 3 else 10
    from_seq = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    print(f"[event] connecting {host}:{port} freq={freq} from_seq={from_seq}...", flush=True)
    try:
        sock.connect((host, port))
    except (socket.timeout, ConnectionRefusedError) as e:
        print(f"[event] connect failed: {e}", file=sys.stderr)
        sys.exit(1)
    sock.settimeout(None)

    req = struct.pack(REQ_HDR_FMT, SMC_CMD_EVENT_SUBSCRIBE, 12) + \
          struct.pack(SUB_FMT, freq, from_seq)
    sock.sendall(req)

    # 读 SmcEventAck
    try:
        ack_buf = recvn(sock, ACK_SIZE)
    except ConnectionError as e:
        print(f"[event] disconnected waiting for ack: {e}", file=sys.stderr)
        sys.exit(1)
    ack_magic, ack_ver, max_per_tick, ev_size = struct.unpack(ACK_FMT, ack_buf)
    if ack_magic != EVENT_ACK_MAGIC:
        print(f"[event] FATAL: ack magic {ack_magic:#x} != expected {EVENT_ACK_MAGIC:#x}",
              file=sys.stderr)
        sys.exit(1)
    if ack_ver != EVENT_VERSION:
        print(f"[event] FATAL: ack version {ack_ver} != {EVENT_VERSION}", file=sys.stderr)
        sys.exit(1)
    if ev_size != EVENT_SIZE:
        print(f"[event] FATAL: server event_size={ev_size} != client {EVENT_SIZE}",
              file=sys.stderr)
        sys.exit(1)

    print(f"[event] ack OK: max_per_tick={max_per_tick} event_size={ev_size}B", flush=True)
    print(f"[event] waiting for events...", flush=True)

    frame_count = 0
    ev_total = 0
    crc_fail = 0
    last_seq = 0
    t0 = time.time()

    while True:
        try:
            hdr_buf = recvn(sock, FRAME_HDR_SIZE)
        except ConnectionError as e:
            print(f"[event] disconnected: {e}", file=sys.stderr)
            break

        magic, version, ev_count, crc = struct.unpack(FRAME_HDR_FMT, hdr_buf)
        if magic != EVENT_MAGIC:
            print(f"[event] FATAL: frame magic {magic:#x} != expected {EVENT_MAGIC:#x}",
                  file=sys.stderr)
            break
        if version != EVENT_VERSION:
            print(f"[event] FATAL: frame version {version} != {EVENT_VERSION}", file=sys.stderr)
            break
        if ev_count == 0 or ev_count > max_per_tick:
            print(f"[event] WARN: ev_count={ev_count} 越界 (0 < n ≤ {max_per_tick})",
                  file=sys.stderr)
            continue

        payload_bytes = ev_count * EVENT_SIZE
        try:
            payload = recvn(sock, payload_bytes)
        except ConnectionError as e:
            print(f"[event] disconnected mid-events: {e}", file=sys.stderr)
            break

        calc_crc = zlib.crc32(hdr_buf[8:12] + payload) & 0xFFFFFFFF
        if calc_crc != crc:
            crc_fail += 1
            print(f"[event] CRC mismatch: calc={calc_crc:#x} frame={crc:#x}, 丢帧",
                  file=sys.stderr)
            continue

        frame_count += 1
        ev_total += ev_count

        for i in range(ev_count):
            ev_payload = payload[i*EVENT_SIZE : (i+1)*EVENT_SIZE]
            ev = parse_event(ev_payload)

            if last_seq and ev["event_seq"] != last_seq + 1:
                gap = ev["event_seq"] - last_seq - 1
                print(f"[event] 丢事件 {gap} (seq {last_seq} → {ev['event_seq']})",
                      file=sys.stderr)
            last_seq = ev["event_seq"]

            sev_name = SEVERITY_NAMES.get(ev["severity"], f"?{ev['severity']}")
            src_name = SOURCE_NAMES.get(ev["source"], f"?{ev['source']}")
            desc = EVENT_CODE_DESC.get((ev["source"], ev["code"]), "")
            val_str = f" val={ev['value']}" if ev["value"] != 0 else ""
            print(
                f"[{ev['timestamp_ms']/1000.0:7.2f}s] seq={ev['event_seq']:>5} "
                f"{sev_name:<5} {src_name:<7} code=0x{ev['code']:04X} "
                f"{desc:<30} msg=\"{ev['message']}\"{val_str}",
                flush=True,
            )

    elapsed = time.time() - t0
    if elapsed > 0:
        print(f"\n[event] 收到 {frame_count} 帧 / {ev_total} 事件, CRC 失败 {crc_fail}, "
              f"用时 {elapsed:.1f}s, 事件速率 {ev_total/elapsed:.1f} ev/s")
    sock.close()


if __name__ == "__main__":
    main()
