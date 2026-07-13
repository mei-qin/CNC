#!/usr/bin/env python3
"""preview_subscriber.py  ——  P0-b v1 段流推送 Python 验证客户端

用法:
    python3 preview_subscriber.py [host] [port] [freq_hz] [from_seq]
    默认: host=127.0.0.1  port=9529  freq=60  from_seq=0

流程:
    1. TCP connect 到段流端口 (默认 9529)
    2. 发送 subscribe req: SmcReqHeader{cmd=0x002B, data_len=12} + int32 freq + uint64 from_seq
    3. 收 SmcPreviewAck (16B), 校验 magic + seg_size_bytes
    4. 循环收 [SmcPreviewFrameHeader + N × TrajectorySegment_t]
    5. 校验 CRC32 (覆盖 seg_count 字段 + segments)
    6. 解析关键字段 (seg_id / line_no / motion_type / target_pos), 单段一行打印

WSL2 sim 联调:
    WSL2: sudo ./rpc_server sim
    Win:  python3 preview_subscriber.py 127.0.0.1 9529 60 0

    另开终端跑 G 代码触发段流:
    python3 scripts/run_gcode_for_snap_test.py tests/gcode/test_m3_s_same_line.nc
"""

import socket
import struct
import sys
import zlib
import time
import signal

# ---- 协议常量 ----
PREV_MAGIC       = 0x53524556   # "PREV"
PREV_ACK_MAGIC   = 0x50524146   # "PRAK"
PREV_VERSION     = 1
SMC_CMD_PREVIEW_SUBSCRIBE = 0x002B

# SmcReqHeader: uint16 cmd_type + uint16 data_len
REQ_HDR_FMT = "<HH"
# Subscribe payload: int32 freq_hz + uint64 from_seq
SUB_FMT = "<iQ"

# SmcPreviewAck: uint32 magic + uint32 version + uint32 max_per_tick + uint32 seg_size_bytes
ACK_FMT = "<IIII"
ACK_SIZE = 16

# SmcPreviewFrameHeader: uint32 magic + uint32 version + uint32 seg_count + uint32 crc32
FRAME_HDR_FMT = "<IIII"
FRAME_HDR_SIZE = 16

# TrajectorySegment_t (AXIS_NUM=5, native alignment)
# 顺序必须与 inc/axis_cfg.h TrajectorySegment_t 完全一致。
# 用 "@" native alignment 匹配 C struct 内存布局 (含编译器 padding)。
SEG_FMT = "@" + "".join([
    "5d",     # target_pos[5]                0-39
    "i",      # is_ready                     40-43
    "i",      # speed                        44-47
    "i",      # cmd_type                     48-51
    "Q",      # seg_id (8B align)            56-63
    "i",      # line_no                      64-67
    "B",      # motion_type                  68
              # pad 3B                       69-71
    "i",      # is_fillet                    72-75
    "i",      # is_g93_strict                76-79
    "i",      # is_rtcp_active               80-83
    "i",      # active_wcs                   84-87
    "5d",     # wcs_offset_snap[5]           88-127
    "i",      # m_code                       128-131
              # pad 4B                       132-135
    "d",      # s_value                      136-143
    "d",      # p_value                      144-151
    "d",      # q_value                      152-159
    "d",      # r_value                      160-167
    "i",      # aux_spindle_mode             168-171
              # pad 4B                       172-175
    "d",      # aux_spindle_rpm              176-183
    "i",      # aux_coolant                  184-187
    "i",      # aux_tool_id                  188-191
    "i",      # aux_laser_enable             192-195
    "i",      # aux_laser_shutter            196-199
    "d",      # aux_laser_power_w            200-207
    "d",      # aux_laser_freq_hz            208-215
    "i",      # aux_gas_select               216-219
    "i",      # aux_laser_coupling_mode      220-223
    "d",      # aux_laser_v_thresh           224-231
    "d",      # total_distance               232-239
    "5d",     # dir_vec[5]                   240-279
    "d",      # v_max                        280-287
    "d",      # v_start                      288-295
    "d",      # v_end                        296-303
    "7d",     # T1-T7                        304-359
    "7d",     # v0-v6                        360-415
    "7d",     # s0-s6                        416-471
    "6d",     # j1,a2,j3,j5,a6,j7            472-519
    "d",      # T_total                      520-527
    "d",      # v_target                     528-535
    "d",      # acc                          536-543
    "d",      # dec                          544-551
    "d",      # jerk                         552-559
])
SEG_SIZE = struct.calcsize(SEG_FMT)
# 预期 560, 实际由 ack.seg_size_bytes 校验

# 字段名 (与 SEG_FMT 顺序一致; 数组字段保留为 tuple)
SEG_FIELDS = [
    "target_pos", "is_ready", "speed", "cmd_type",
    "seg_id", "line_no", "motion_type",
    "is_fillet", "is_g93_strict", "is_rtcp_active", "active_wcs",
    "wcs_offset_snap",
    "m_code", "s_value", "p_value", "q_value", "r_value",
    "aux_spindle_mode", "aux_spindle_rpm",
    "aux_coolant", "aux_tool_id",
    "aux_laser_enable", "aux_laser_shutter",
    "aux_laser_power_w", "aux_laser_freq_hz",
    "aux_gas_select", "aux_laser_coupling_mode",
    "aux_laser_v_thresh", "total_distance",
    "dir_vec", "v_max", "v_start", "v_end",
    "T1_T7", "v0_v6", "s0_s6", "j1_j7",
    "T_total", "v_target", "acc", "dec", "jerk",
]

# motion_type 枚举 (与 axis_cfg.h MOTION_TYPE_* 一致)
MOTION_NAMES = {
    0:   "G00",
    1:   "G01",
    2:   "G02",
    3:   "G03",
    4:   "NURBS",
    0xFF: "OTHER",
}

# cmd_type 枚举
CMD_TYPE_MOTION = 0
CMD_TYPE_MCODE  = 1


def recvn(sock: socket.socket, n: int) -> bytes:
    """循环 recv 直至收满 n 字节"""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n} bytes")
        buf += chunk
    return buf


def parse_segment(payload: bytes) -> dict:
    """解析单段 (SEG_SIZE 字节) 为 dict。数组字段聚合为 tuple。"""
    values = list(struct.unpack(SEG_FMT, payload))
    # 手动聚合数组字段 (struct.unpack 把 5d 展开为 5 个独立值)
    array_specs = [
        ("target_pos", 5),
        ("wcs_offset_snap", 5),
        ("dir_vec", 5),
        ("T1_T7", 7),
        ("v0_v6", 7),
        ("s0_s6", 7),
        ("j1_j7", 6),
    ]
    result = {}
    i = 0
    field_idx = 0
    while field_idx < len(SEG_FIELDS):
        name = SEG_FIELDS[field_idx]
        match = next((s for s in array_specs if s[0] == name), None)
        if match:
            arr_name, arr_size = match
            result[arr_name] = tuple(values[i:i+arr_size])
            i += arr_size
        else:
            result[name] = values[i]
            i += 1
        field_idx += 1
    return result


def main():
    host     = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port     = int(sys.argv[2]) if len(sys.argv) > 2 else 9529
    freq     = int(sys.argv[3]) if len(sys.argv) > 3 else 60
    from_seq = int(sys.argv[4]) if len(sys.argv) > 4 else 0

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    print(f"[prev] connecting {host}:{port} freq={freq} from_seq={from_seq}...", flush=True)
    try:
        sock.connect((host, port))
    except (socket.timeout, ConnectionRefusedError) as e:
        print(f"[prev] connect failed: {e}", file=sys.stderr)
        sys.exit(1)
    sock.settimeout(None)

    # 发送 subscribe req (SmcReqHeader + payload)
    req = struct.pack(REQ_HDR_FMT, SMC_CMD_PREVIEW_SUBSCRIBE, 12) + \
          struct.pack(SUB_FMT, freq, from_seq)
    sock.sendall(req)

    # 读 SmcPreviewAck
    try:
        ack_buf = recvn(sock, ACK_SIZE)
    except ConnectionError as e:
        print(f"[prev] disconnected waiting for ack: {e}", file=sys.stderr)
        sys.exit(1)
    ack_magic, ack_ver, max_per_tick, seg_size = struct.unpack(ACK_FMT, ack_buf)
    if ack_magic != PREV_ACK_MAGIC:
        print(f"[prev] FATAL: ack magic {ack_magic:#x} != expected {PREV_ACK_MAGIC:#x}",
              file=sys.stderr)
        sys.exit(1)
    if ack_ver != PREV_VERSION:
        print(f"[prev] FATAL: ack version {ack_ver} != {PREV_VERSION}", file=sys.stderr)
        sys.exit(1)
    if seg_size != SEG_SIZE:
        print(f"[prev] FATAL: server seg_size={seg_size} != client SEG_SIZE={SEG_SIZE}. "
              f"AXIS_NUM 或字段表不一致, 重编译两端", file=sys.stderr)
        sys.exit(1)

    print(f"[prev] ack OK: max_per_tick={max_per_tick} seg_size={seg_size}B", flush=True)
    print(f"[prev] client SEG_SIZE={SEG_SIZE}B, 等待段流...", flush=True)

    frame_count = 0
    seg_total = 0
    crc_fail = 0
    last_seg_id = 0
    t0 = time.time()

    while True:
        # 读帧头 16B
        try:
            hdr_buf = recvn(sock, FRAME_HDR_SIZE)
        except ConnectionError as e:
            print(f"[prev] disconnected: {e}", file=sys.stderr)
            break

        magic, version, seg_count, crc = struct.unpack(FRAME_HDR_FMT, hdr_buf)
        if magic != PREV_MAGIC:
            print(f"[prev] FATAL: frame magic {magic:#x} != expected {PREV_MAGIC:#x}",
                  file=sys.stderr)
            break
        if version != PREV_VERSION:
            print(f"[prev] FATAL: frame version {version} != {PREV_VERSION}", file=sys.stderr)
            break
        if seg_count == 0 or seg_count > max_per_tick:
            print(f"[prev] WARN: seg_count={seg_count} 越界 (0 < n ≤ {max_per_tick})",
                  file=sys.stderr)
            continue

        # 读 segments
        seg_bytes = seg_count * SEG_SIZE
        try:
            payload = recvn(sock, seg_bytes)
        except ConnectionError as e:
            print(f"[prev] disconnected mid-segments: {e}", file=sys.stderr)
            break

        # CRC32 校验: 覆盖 seg_count 字段 (header[8..12]) + segments
        calc_crc = zlib.crc32(hdr_buf[8:12] + payload) & 0xFFFFFFFF
        if calc_crc != crc:
            crc_fail += 1
            print(f"[prev] CRC mismatch: calc={calc_crc:#x} frame={crc:#x}, 丢帧",
                  file=sys.stderr)
            continue

        frame_count += 1
        seg_total += seg_count

        # 解析每段并打印关键字段
        for i in range(seg_count):
            seg_payload = payload[i*SEG_SIZE : (i+1)*SEG_SIZE]
            seg = parse_segment(seg_payload)

            # 丢段检测 (相邻 seg_id 应 +1)
            if last_seg_id and seg["seg_id"] != last_seg_id + 1:
                gap = seg["seg_id"] - last_seg_id - 1
                print(f"[prev] 丢段 {gap} (seg_id {last_seg_id} → {seg['seg_id']})",
                      file=sys.stderr)
            last_seg_id = seg["seg_id"]

            # 单行打印关键字段
            if seg["cmd_type"] == CMD_TYPE_MOTION:
                tp = seg["target_pos"]
                mt = MOTION_NAMES.get(seg["motion_type"], f"?{seg['motion_type']}")
                print(
                    f"seg_id={seg['seg_id']:>6} line={seg['line_no']:>4} "
                    f"{mt:<6} target=({tp[0]:+8.2f},{tp[1]:+8.2f},{tp[2]:+8.2f},"
                    f"{tp[3]:+6.2f},{tp[4]:+6.2f}) "
                    f"v_max={seg['v_max']:.1f}mm/s "
                    f"wcs={seg['active_wcs']} rtcp={seg['is_rtcp_active']}",
                    flush=True,
                )
            else:  # CMD_TYPE_MCODE
                print(
                    f"seg_id={seg['seg_id']:>6} line={seg['line_no']:>4} "
                    f"MCODE  m={seg['m_code']:>3} s={seg['s_value']:.0f} "
                    f"p={seg['p_value']:.0f} q={seg['q_value']:.0f} r={seg['r_value']:.0f} "
                    f"spindle={seg['aux_spindle_mode']}@{seg['aux_spindle_rpm']:.0f}rpm "
                    f"coolant={seg['aux_coolant']} tool={seg['aux_tool_id']}",
                    flush=True,
                )

    elapsed = time.time() - t0
    if elapsed > 0:
        print(f"\n[prev] 收到 {frame_count} 帧 / {seg_total} 段, CRC 失败 {crc_fail}, "
              f"用时 {elapsed:.1f}s, 段速率 {seg_total/elapsed:.1f} seg/s")
    sock.close()


if __name__ == "__main__":
    main()
