#!/usr/bin/env python3
"""test_laser_config_rpc.py — 7 个激光配置 RPC cmd 端到端验证 (P0-Laser-ConfigRPC)

用法:
    WSL2: sudo ./cnc_core sim  (先启动 server, 监听 9527)
    Win:  python3 scripts/test_laser_config_rpc.py [--host 127.0.0.1] [--port 9527]

测试内容:
    1. 7 个 ConfigLaser* 正常调用 → 预期 ret_code=0
    2. ConfigLaserCoupleTable 越界 (count=0, count=17) → 预期 ret_code=-1

注意:
    - ConfigLaser* 设计为 init-time 调用 (SMC_InitAndStart 前), 但 RPC 层不做时序校验,
      server 任何状态都会接受并返回 ret_code=0 (配置是否生效取决于调用时序)
    - 本脚本只验证 RPC 路径 (协议封装 + 字段映射 + ret_code), 不验证配置行为生效
      (行为生效需跑 sim 看 CSV 功率曲线, 见 plan 步骤 3)
"""
import socket
import struct
import sys

# ---- cmd 枚举 (与 smc_protocol.h 0x0050-0x0056 同步) ----
CMD_CONFIG_LASER_IO           = 0x0050
CMD_CONFIG_LASER_DO_BITS      = 0x0051
CMD_CONFIG_LASER_DI_BITS      = 0x0052
CMD_CONFIG_LASER_AO_CHANNELS  = 0x0053
CMD_CONFIG_LASER_RANGE        = 0x0054
CMD_CONFIG_LASER_COUPLING     = 0x0055
CMD_CONFIG_LASER_COUPLE_TABLE = 0x0056

LASER_COUPLE_TABLE_MAX = 16


def recvn(sock, n):
    """确保读到 n 字节 (处理 TCP 分包)"""
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError(f"socket closed, got {len(buf)}/{n}")
        buf += chunk
    return buf


def rpc_call(host, port, cmd, payload):
    """单次 RPC: connect → send req → recv res → close.
    返回 (err_code, ret_code, err_msg); 连接失败 err_code=-999."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(10.0)
    try:
        s.connect((host, port))
    except Exception as e:
        return (-999, -999, f"connect failed: {e}")

    try:
        # SmcReqHeader: uint16 cmd + uint16 data_len (4B)
        req_hdr = struct.pack("<HH", cmd, len(payload))
        s.sendall(req_hdr + payload)

        # SmcResHeader: int32 err_code + uint32 data_len (8B)
        hdr = recvn(s, 8)
        err_code, data_len = struct.unpack("<iI", hdr)

        # Res payload: 至少 4B (int32 ret_code)
        if data_len >= 4:
            resp = recvn(s, data_len)
            ret_code = struct.unpack("<i", resp[:4])[0]
        else:
            ret_code = -998
        return (err_code, ret_code, "")
    except Exception as e:
        return (-997, -997, f"rpc error: {e}")
    finally:
        s.close()


def pack_couple_table(points):
    """打包 CoupleTable Req: count + 16 个 LaserCouplePoint_t (固定槽)
    points: [(v_mm_s, ratio), ...] 长度 1..16
    返回 260B bytes (pack(1) 与 C 端 SmcConfigLaserCoupleTableReq 一致)"""
    count = len(points)
    # 格式: <i + 16×(dd)  →  4 + 256 = 260B
    fmt = "<i" + "dd" * LASER_COUPLE_TABLE_MAX
    flat = [count]
    for i in range(LASER_COUPLE_TABLE_MAX):
        if i < count:
            flat.extend(points[i])
        else:
            flat.extend([0.0, 0.0])  # 未用槽填零
    return struct.pack(fmt, *flat)


def pack_couple_table_raw_count(count):
    """打包 CoupleTable Req 但用任意 count (用于越界测试, points 全零)"""
    fmt = "<i" + "dd" * LASER_COUPLE_TABLE_MAX
    flat = [count]
    for _ in range(LASER_COUPLE_TABLE_MAX):
        flat.extend([0.0, 0.0])
    return struct.pack(fmt, *flat)


def run_test(host, port, name, cmd, payload, expect_ret):
    """跑单个测试用例 + 打印结果. 返回 True=PASS / False=FAIL"""
    err, ret, msg = rpc_call(host, port, cmd, payload)
    ok = (err == 0 and ret == expect_ret)
    status = "PASS" if ok else "FAIL"
    detail = f"err={err} ret={ret} (expect err=0 ret={expect_ret})"
    if msg:
        detail += f" [{msg}]"
    print(f"  [{status}] {name:42s} {detail}")
    return ok


def main():
    host = "127.0.0.1"
    port = 9527

    # 简单参数解析 (与 inject_fault_rpc.py 同风格)
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--host" and i + 1 < len(args):
            host = args[i + 1]; i += 2
        elif args[i] == "--port" and i + 1 < len(args):
            port = int(args[i + 1]); i += 2
        else:
            i += 1

    print(f"=== 激光配置 RPC 端到端测试 ({host}:{port}) ===")
    print(f"=== 正常调用 (期望 ret_code=0) ===")
    print()

    results = []

    # 1. ConfigLaserIO(0, 0, 0) — sim 模式 3 个虚拟从站 id
    payload = struct.pack("<iii", 0, 0, 0)
    results.append(run_test(host, port, "ConfigLaserIO(0,0,0)",
                            CMD_CONFIG_LASER_IO, payload, 0))

    # 2. ConfigLaserDOBits(0,1,2,3,4,5) — 默认 DO bit 偏移
    payload = struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5)
    results.append(run_test(host, port, "ConfigLaserDOBits(0,1,2,3,4,5)",
                            CMD_CONFIG_LASER_DO_BITS, payload, 0))

    # 3. ConfigLaserDIBits(0,1,2,3,4,5) — 默认 DI bit 偏移
    payload = struct.pack("<BBBBBB", 0, 1, 2, 3, 4, 5)
    results.append(run_test(host, port, "ConfigLaserDIBits(0,1,2,3,4,5)",
                            CMD_CONFIG_LASER_DI_BITS, payload, 0))

    # 4. ConfigLaserAOChannels(0,1) — power=ch0, freq=ch1
    payload = struct.pack("<BB", 0, 1)
    results.append(run_test(host, port, "ConfigLaserAOChannels(0,1)",
                            CMD_CONFIG_LASER_AO_CHANNELS, payload, 0))

    # 5. ConfigLaserRange(3000, 5000, 50) — 默认量程
    payload = struct.pack("<ddd", 3000.0, 5000.0, 50.0)
    results.append(run_test(host, port, "ConfigLaserRange(3000,5000,50)",
                            CMD_CONFIG_LASER_RANGE, payload, 0))

    # 6. ConfigLaserCoupling(1, 5.0) — 开耦合 + v_thresh=5 mm/s
    payload = struct.pack("<id", 1, 5.0)
    results.append(run_test(host, port, "ConfigLaserCoupling(mode=1, v_thresh=5.0)",
                            CMD_CONFIG_LASER_COUPLING, payload, 0))

    # 7. ConfigLaserCoupleTable (3 点: 起弧/切割/饱和)
    points = [(0.0, 0.0), (5.0, 0.3), (20.0, 1.0)]
    payload = pack_couple_table(points)
    results.append(run_test(host, port, "ConfigLaserCoupleTable(3 points)",
                            CMD_CONFIG_LASER_COUPLE_TABLE, payload, 0))

    print()
    print(f"=== 越界场景 (期望 ret_code=-1) ===")
    print()

    # 8. ConfigLaserCoupleTable count=0 (越界: 必须 >=1)
    payload = pack_couple_table_raw_count(0)
    results.append(run_test(host, port, "ConfigLaserCoupleTable(count=0)",
                            CMD_CONFIG_LASER_COUPLE_TABLE, payload, -1))

    # 9. ConfigLaserCoupleTable count=17 (越界: 必须 <=16)
    payload = pack_couple_table_raw_count(17)
    results.append(run_test(host, port, "ConfigLaserCoupleTable(count=17)",
                            CMD_CONFIG_LASER_COUPLE_TABLE, payload, -1))

    # 10. ConfigLaserCoupling mode=5 (越界: 必须 0 或 1)
    payload = struct.pack("<id", 5, 5.0)
    results.append(run_test(host, port, "ConfigLaserCoupling(mode=5 invalid)",
                            CMD_CONFIG_LASER_COUPLING, payload, -1))

    # 11. ConfigLaserRange 负值 (越界: power_max 必须 >0)
    payload = struct.pack("<ddd", -100.0, 5000.0, 50.0)
    results.append(run_test(host, port, "ConfigLaserRange(power_max=-100 invalid)",
                            CMD_CONFIG_LASER_RANGE, payload, -1))

    print()
    passed = sum(results)
    total = len(results)
    print(f"=== 总结: {passed}/{total} 通过 ===")
    if passed == total:
        print("全部 PASS — 7 个激光配置 RPC cmd 端到端通路正常")
        return 0
    else:
        print("有 FAIL — 检查上方 [FAIL] 行的 err/ret_code")
        return 1


if __name__ == "__main__":
    sys.exit(main())
