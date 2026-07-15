import csv, sys

path = sys.argv[1]
rows = []
with open(path, 'r', newline='') as f:
    reader = csv.DictReader(f)
    cols = reader.fieldnames
    for r in reader:
        rows.append(r)

print(f"总行数 (含header): {len(rows)+1}")
print(f"列数: {len(cols)}")
print(f"列名: {cols}")

def num(r, k):
    try:
        return int(float(r[k]))
    except:
        return r[k]

# C1: pierce_count 末行 = 1
last = rows[-1]
pc_last = num(last, 'pierce_count')
print(f"\n[C1] pierce_count 末行 = {pc_last}  (预期 1)  -> {'PASS' if pc_last==1 else 'FAIL(>=2重复触发 / =0未触发)'}")

# C2: is_piercing 分布
from collections import Counter
ip_dist = Counter(num(r,'is_piercing') for r in rows)
print(f"[C2] is_piercing 分布 = {dict(ip_dist)}  (预期 0 和 1, 1的行数≈1000)")
ip1 = ip_dist.get(1, 0)
print(f"     is_piercing=1 行数 = {ip1}")

# C3: laser_on_time_ms 末行 ≈ 10000 ± 200
lot_last = num(last, 'laser_on_time_ms')
print(f"[C3] laser_on_time_ms 末行 = {lot_last}  (预期 ≈10000±200)  -> {'PASS' if 9800<=lot_last<=10200 else 'FAIL'}")

# C4: current_seg_flags 唯一值 (应仅 0)
sf_dist = Counter(num(r,'current_seg_flags') for r in rows)
print(f"[C4] current_seg_flags 唯一值 = {dict(sf_dist)}  (预期仅 0)  -> {'PASS' if sf_dist=={0:len(rows)} else 'FAIL(出现非0)'}")

# C5: laser_emergency_kill 唯一值 (应仅 0)
ek_dist = Counter(num(r,'laser_emergency_kill') for r in rows)
print(f"[C5] laser_emergency_kill 唯一值 = {dict(ek_dist)}  (预期仅 0)  -> {'PASS' if ek_dist=={0:len(rows)} else 'FAIL(出现1)'}")

# 额外: laser_enable/shutter 末行 (确认激光实际开启)
print(f"\n[附加] 末行 laser_enable={last['laser_enable']} laser_shutter={last['laser_shutter']} laser_power_w={last['laser_power_w']}")
