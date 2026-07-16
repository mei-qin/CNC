#!/usr/bin/env python3
"""snap_subscriber.py  ——  P0-a 推送通道 Python 验证客户端

用法:
    python3 snap_subscriber.py [host] [port] [freq_hz]
    默认: host=127.0.0.1  port=9528  freq=60

流程:
    1. TCP connect 到推送端口 (默认 9528)
    2. 发送 subscribe 请求: SmcReqHeader{cmd=0x002A, data_len=4} + int32_t freq
    3. 循环接收帧: SmcPushFrameHeader(16B) + SMC_Snapshot_t(400B)
    4. 校验 magic / version / CRC32, 解析字段, 单行打印关键字段

WSL2 sim 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 snap_subscriber.py 127.0.0.1 9528 60
"""

import socket
import struct
import sys
import zlib
import time
import signal

# ---- 协议常量 (与 inc/snapshot_hub.h / inc/rpc_push_server.h 一致) ----
SNAP_MAGIC = 0x534E4150   # "SNAP" little-endian
SNAP_VERSION = 4          # v4: P2-A 实时倍率字段 (2026-07-16)
                           # v3: 加 current_seg_id + segment_progress (2026-07-13, P0-c)
SMC_CMD_SUBSCRIBE = 0x002A
SMC_ACK_MAGIC = 0x534E414B   # "SNAK" - SubscribeAck 帧, 与 SNAP 同族末字节不同

# SmcReqHeader: uint16 cmd_type + uint16 data_len (4B)
REQ_HDR_FMT = "<HH"
# int32_t freq_hz payload
FREQ_FMT = "<i"

# SmcPushFrameHeader: uint32 magic + uint32 version + uint32 payload_len + uint32 crc32 (16B)
FRAME_HDR_FMT = "<IIII"
FRAME_HDR_SIZE = 16

# SubscribeAck: 与 FRAME_HDR_FMT 同尺寸 (16B), 便于复用读缓冲
ACK_FMT = "<IIII"
ACK_SIZE = 16

# SMC_Snapshot_t (AXIS_NUM=5, packed)
# 字段顺序必须与 inc/snapshot_hub.h SMC_Snapshot_t 完全一致
SNAP_FMT = (
    "<"
    "I"    # magic
    "I"    # version
    "Q"    # snapshot_seq
    "I"    # cycle
    "I"    # flags
    "d"    # virtual_time_ms
    "d"    # uptime_ms        (v2 新增: 系统启动硬时间 = cycle * 1ms)
    "d"    # time_scale
    "i"    # is_moving
    "i"    # hold_state
    "i"    # is_waiting_mcode
    "i"    # current_mcode
    "d"    # mcode_p_value_ms
    "Q"    # current_seg_id       (v3 新增: P0-c 实时光标)
    "d"    # segment_progress     (v3 新增: 0-1 进度)
    "i"    # _pad32
    "5d"   # machine_pos
    "5d"   # target_pos
    "5d"   # start_pos
    "d"    # v_current_mm_s
    "d"    # v_target_mm_s
    "d"    # feedrate_mm_min
    "i"    # motion_mode
    "i"    # active_plane
    "i"    # is_absolute
    "i"    # feed_mode
    "i"    # rtcp_enabled
    "i"    # active_cycle
    "i"    # current_coord
    "5d"   # active_offset
    "5d"   # logical_pos
    "i"    # spindle_mode
    "d"    # spindle_rpm
    "i"    # coolant_state
    "i"    # tool_id
    "i"    # laser_enable
    "i"    # laser_shutter
    "d"    # laser_power_w
    "d"    # laser_freq_hz
    "i"    # gas_select
    "i"    # laser_emergency_kill
    "H"    # laser_interlock
    "H"    # _pad16
    "d"    # laser_v_actual_mm_s
    "i"    # sys_alarm_state
    "i"    # parser_is_running
    "i"    # parser_is_paused
    "i"    # feed_override_pct        (v4 新增: P2-A 实时倍率)
    "i"    # rapid_override_pct       (v4 新增)
    "i"    # spindle_override_pct     (v4 新增)
    "i"    # mode_flags               (v4 新增: SMC_MODE_* 位图)
    "i"    # current_seg_is_exact_stop (P2-A-4: 精准停段镜像, 复用原 _tail_pad 位)
)

SNAP_SIZE = struct.calcsize(SNAP_FMT)
assert SNAP_SIZE == 440, f"SMC_Snapshot_t size mismatch: {SNAP_SIZE} != 440 (v4 含 P2-A 倍率字段)"

# 字段名 (与 SNAP_FMT 顺序一致, 数组字段保留为元组)
SNAP_FIELDS = [
    "magic", "version", "snapshot_seq", "cycle", "flags",
    "virtual_time_ms", "uptime_ms", "time_scale",
    "is_moving", "hold_state", "is_waiting_mcode", "current_mcode", "mcode_p_value_ms",
    "current_seg_id", "segment_progress", "_pad32",
    "machine_pos", "target_pos", "start_pos",
    "v_current_mm_s", "v_target_mm_s", "feedrate_mm_min",
    "motion_mode", "active_plane", "is_absolute", "feed_mode", "rtcp_enabled", "active_cycle",
    "current_coord", "active_offset", "logical_pos",
    "spindle_mode", "spindle_rpm", "coolant_state", "tool_id",
    "laser_enable", "laser_shutter", "laser_power_w", "laser_freq_hz",
    "gas_select", "laser_emergency_kill", "laser_interlock", "_pad16", "laser_v_actual_mm_s",
    "sys_alarm_state", "parser_is_running", "parser_is_paused",
    "feed_override_pct", "rapid_override_pct", "spindle_override_pct", "mode_flags",
    "current_seg_is_exact_stop",
]

# flags 位定义
FLAG_ALARM       = 0x01
FLAG_PAUSED      = 0x02
FLAG_ABORT_REQ   = 0x04
FLAG_HOLD        = 0x08
FLAG_WAIT_MCODE  = 0x10

# FeedHoldState_t 枚举
HOLD_NAMES = {0: "NORMAL", 1: "BRAKING", 2: "PAUSED", 3: "RESUMING"}


def recvn(sock: socket.socket, n: int) -> bytes:
    """循环 recv 直至收满 n 字节, 返回 bytes; 对端关闭抛 ConnectionError"""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n} bytes")
        buf += chunk
    return buf


def parse_snapshot(payload: bytes) -> dict:
    """把 payload 解析为 dict, 数组字段（machine_pos 等 5d）保留为 tuple"""
    values = list(struct.unpack(SNAP_FMT, payload))
    # SNAP_FIELDS 中数组名（如 machine_pos）对应 fmt 中 "5d" 的 5 个值
    # zip 会把前 4 个字段吞掉导致错位——手动收拢
    ARRAY5_NAMES = {"machine_pos", "target_pos", "start_pos", "active_offset", "logical_pos"}
    i = 0
    result = {}
    for name in SNAP_FIELDS:
        if name in ARRAY5_NAMES:
            result[name] = tuple(values[i:i+5])
            i += 5
        else:
            result[name] = values[i]
            i += 1
    return result


def state_str(snap: dict) -> str:
    """聚合 RUN/HOLD/ALARM/IDLE 语义 (SDK 端做, 不依赖底层枚举)"""
    if snap["sys_alarm_state"]:
        return "ALARM"
    if snap["flags"] & FLAG_ABORT_REQ:
        return "ABORTING"
    if snap["parser_is_paused"] or snap["hold_state"] != 0:
        return f"HOLD({HOLD_NAMES.get(snap['hold_state'], '?')})"
    if snap["parser_is_running"] or snap["is_moving"]:
        return "RUN"
    return "IDLE"


def motion_str(mode: int) -> str:
    return {0: "G00", 1: "G01", 2: "G02", 3: "G03"}.get(mode, f"G?{mode}")


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9528
    freq = int(sys.argv[3]) if len(sys.argv) > 3 else 60

    # Ctrl+C 优雅退出
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    print(f"[snap] connecting {host}:{port} freq={freq}...", flush=True)
    t_connect = time.time()
    try:
        sock.connect((host, port))
    except (socket.timeout, ConnectionRefusedError) as e:
        print(f"[snap] connect failed: {e}", file=sys.stderr)
        sys.exit(1)
    sock.settimeout(None)

    # 发送 subscribe req
    req = struct.pack(REQ_HDR_FMT, SMC_CMD_SUBSCRIBE, 4) + struct.pack(FREQ_FMT, freq)
    sock.sendall(req)

    # Bug #7: 读 SubscribeAck, 验证 server 实际接受的 freq
    try:
        ack_buf = recvn(sock, ACK_SIZE)
    except ConnectionError as e:
        print(f"[snap] disconnected waiting for ack: {e}", file=sys.stderr)
        sys.exit(1)
    ack_magic, ack_ver, actual_freq, _reserved = struct.unpack(ACK_FMT, ack_buf)
    if ack_magic != SMC_ACK_MAGIC:
        print(f"[snap] FATAL: ack magic mismatch {ack_magic:#x} (expected {SMC_ACK_MAGIC:#x})",
              file=sys.stderr)
        sys.exit(1)
    if ack_ver != SNAP_VERSION:
        print(f"[snap] FATAL: ack version {ack_ver} != client {SNAP_VERSION}, 协议不兼容",
              file=sys.stderr)
        sys.exit(1)
    if actual_freq != freq:
        print(f"[snap] WARN: server 把 freq={freq} clamp 到 {actual_freq} (越界或边界)",
              file=sys.stderr)
    print(f"[snap] subscribed, actual_freq={actual_freq}Hz (requested {freq}), waiting for frames...",
          flush=True)

    frame_count = 0
    crc_fail = 0
    last_seq = 0
    t_first_frame = None   # Bug #8: 首帧到达时间
    t_second_frame = None  # Bug #8: 第二帧到达时间 (验证推送周期)
    t0 = time.time()

    while True:
        # 读帧头 16B
        try:
            hdr_buf = recvn(sock, FRAME_HDR_SIZE)
        except ConnectionError as e:
            print(f"[snap] disconnected: {e}", file=sys.stderr)
            break

        magic, version, payload_len, crc = struct.unpack(FRAME_HDR_FMT, hdr_buf)

        if magic != SNAP_MAGIC:
            print(f"[snap] FATAL: magic mismatch {magic:#x} (expected {SNAP_MAGIC:#x})", file=sys.stderr)
            # 帧同步丢失, 尝试跳过当前 payload 重新对齐
            if payload_len and payload_len <= SNAP_SIZE * 2:
                try: recvn(sock, payload_len)
                except ConnectionError: break
            continue

        if version != SNAP_VERSION:
            print(f"[snap] FATAL: version mismatch {version} (expected {SNAP_VERSION})", file=sys.stderr)
            break

        if payload_len != SNAP_SIZE:
            print(f"[snap] WARN: payload_len={payload_len} != expected {SNAP_SIZE}, 跳过帧", file=sys.stderr)
            try: recvn(sock, payload_len)
            except ConnectionError: break
            continue

        # 读 payload
        try:
            payload = recvn(sock, payload_len)
        except ConnectionError as e:
            print(f"[snap] disconnected mid-payload: {e}", file=sys.stderr)
            break

        # CRC32 校验 (覆盖 header[0..12] + payload)
        calc_crc = zlib.crc32(hdr_buf[:12] + payload) & 0xFFFFFFFF
        if calc_crc != crc:
            crc_fail += 1
            print(f"[snap] CRC mismatch: calc={calc_crc:#x} frame={crc:#x}, 丢帧", file=sys.stderr)
            continue

        snap = parse_snapshot(payload)
        frame_count += 1

        # Bug #8: 首帧/第二帧时间统计 (验证 force_log + 推送频率)
        now = time.time()
        if t_first_frame is None:
            t_first_frame = now
            first_delay_ms = (now - t_connect) * 1000.0
            print(f"[snap] 首帧延迟 {first_delay_ms:.1f}ms (force_log 验证, 应 <100ms)",
                  file=sys.stderr)
        elif t_second_frame is None:
            t_second_frame = now
            period_ms = (now - t_first_frame) * 1000.0
            expected_ms = 1000.0 / actual_freq
            drift_pct = abs(period_ms - expected_ms) / expected_ms * 100.0
            print(f"[snap] 推送周期 {period_ms:.2f}ms (期望 {expected_ms:.2f}ms, "
                  f"偏漂 {drift_pct:.1f}%, 应 <10%)", file=sys.stderr)

        # 丢帧检测 (相邻 seq 应 +2)
        if last_seq and snap["snapshot_seq"] != last_seq + 2:
            gap = (snap["snapshot_seq"] - last_seq) // 2 - 1
            print(f"[snap] 丢帧 {gap} (seq {last_seq} → {snap['snapshot_seq']})", file=sys.stderr)
        last_seq = snap["snapshot_seq"]

        # 单行打印关键字段
        mp = snap["machine_pos"]
        # P2-A: 倍率显示 (F/R/S%, mode_flags 解码)
        mode_str = (
            f"{'SB' if snap['mode_flags'] & 0x01 else '--'}"
            f"{'DR' if snap['mode_flags'] & 0x02 else '--'}"
        )
        print(
            f"cycle={snap['cycle']:>8} seq={snap['snapshot_seq']:>6} "
            f"state={state_str(snap):<14} "
            f"pos=({mp[0]:+8.2f},{mp[1]:+8.2f},{mp[2]:+8.2f}) "
            f"v={snap['v_current_mm_s']:6.1f}mm/s "
            f"{motion_str(snap['motion_mode'])} "
            f"spindle={snap['spindle_mode']}@{snap['spindle_rpm']:.0f}rpm "
            f"laser={snap['laser_enable']}P={snap['laser_power_w']:.0f}W "
            f"ovr=F{snap['feed_override_pct']}%/R{snap['rapid_override_pct']}%/S{snap['spindle_override_pct']}%[{mode_str}] "
            f"up={snap['uptime_ms']/1000.0:.2f}s "
            f"vt={snap['virtual_time_ms']:.1f}ms "
            f"ts={snap['time_scale']:.2f} "
            f"cursor=seg{snap['current_seg_id']:>5}@{snap['segment_progress']*100:5.1f}%",
            flush=True,
        )

    elapsed = time.time() - t0
    print(f"\n[snap] 收到 {frame_count} 帧, CRC 失败 {crc_fail}, 用时 {elapsed:.1f}s, "
          f"实际频率 {frame_count/elapsed:.1f}Hz" if elapsed > 0 else "")
    sock.close()


if __name__ == "__main__":
    main()
