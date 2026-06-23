"""CNC 仿真自动化运行器
通过 Python subprocess 直接控制 cnc_core sim 的交互式菜单。
"""

import subprocess
import sys
import time
import os

WSL_EXE = 'C:/Windows/System32/wsl.exe'
WS_DIR = '/home/meiqin/cnc_ws'
WIN_DIR = '/mnt/d/code/CNC'

TESTS = {
    'L1': ('tests/gcode/L1_sharp_corner.nc', 'log_bspline.csv', 60),
    'L2': ('tests/gcode/L2_g93_strict.nc', 'log_g93.csv', 120),
    'L3': ('tests/gcode/L3_kinematics_rtcp.nc', 'log_rtcp.csv', 60),
}


def run_wsl(cmd, timeout=30):
    """Run a command in WSL and return output."""
    r = subprocess.run(
        [WSL_EXE, '-e', 'bash', '-c', cmd],
        capture_output=True, timeout=timeout
    )
    return r.stdout.decode('utf-8', errors='replace'), r.returncode


def run_single_test(name, nc_path, out_csv, timeout_sec):
    """Run a single NC test file through cnc_core sim."""
    print(f"\n{'='*60}")
    print(f"  [{name}] 运行 {nc_path}")
    print(f"{'='*60}")
    
    full_nc = f"{WS_DIR}/{nc_path}"
    
    # Build the command
    cmd = (
        f"cd {WS_DIR} && "
        f"printf '4\\n{full_nc}\\n0\\n' | "
        f"timeout {timeout_sec} ./cnc_core sim"
    )
    
    t0 = time.time()
    out, rc = run_wsl(cmd, timeout=timeout_sec + 10)
    elapsed = time.time() - t0
    
    print(f"  耗时: {elapsed:.1f}s, returncode={rc}")
    
    # Find and rename CSV
    find_out, _ = run_wsl(
        f"cd {WS_DIR} && ls -t cnc_trace_log_*.csv 2>/dev/null | head -1",
        timeout=5
    )
    csv_file = find_out.strip()
    
    if csv_file:
        lines, _ = run_wsl(f"wc -l < {WS_DIR}/{csv_file}", timeout=5)
        size, _ = run_wsl(f"stat -c%s {WS_DIR}/{csv_file}", timeout=5)
        print(f"  CSV: {csv_file} ({lines.strip()} 行, {size.strip()} bytes)")
        
        # Copy to output
        run_wsl(f"cp {WS_DIR}/{csv_file} {WS_DIR}/{out_csv}", timeout=5)
        run_wsl(f"cp {WS_DIR}/{out_csv} {WIN_DIR}/{out_csv}", timeout=5)
        run_wsl(f"rm -f {WS_DIR}/{csv_file}", timeout=5)
        print(f"  ✓ 保存: {out_csv}")
        return True
    else:
        print(f"  ✗ 未生成 CSV!")
        return False


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    
    print("=" * 60)
    print("  CNC 全自动测试流水线 v3 (Python 控制)")
    print("=" * 60)
    
    # Step 0: Compile
    print("\n[0] 编译 cnc_core...")
    out, _ = run_wsl(f"cd {WS_DIR} && make clean >/dev/null 2>&1 && make all 2>&1 | tail -3", timeout=30)
    print(out)
    
    # Check binary
    out, _ = run_wsl(f"stat -c%s {WS_DIR}/cnc_core", timeout=5)
    print(f"  二进制: {out.strip()} bytes")
    
    # Run tests
    results = {}
    for name, (nc_path, out_csv, timeout_sec) in TESTS.items():
        ok = run_single_test(name, nc_path, out_csv, timeout_sec)
        results[name] = ok
    
    # Summary
    print(f"\n{'='*60}")
    print(f"  结果汇总:")
    for name, ok in results.items():
        status = "✓ PASS" if ok else "✗ FAIL"
        print(f"    {name}: {status}")
    print(f"{'='*60}")
    
    return all(results.values())


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
