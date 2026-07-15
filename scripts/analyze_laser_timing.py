import csv

path = '/mnt/d/code/CNC/tests/output/laser_log/log_laser_g04.csv'
rows = list(csv.DictReader(open(path)))
print(f"总行数: {len(rows)}")

def f(r,k): return float(r[k])

# 找 shutter 开启/关闭的 virtual_time 窗口
shutter_on_start = None
shutter_on_end = None
for r in rows:
    sh = int(float(r['laser_shutter']))
    vt = float(r['virtual_time_ms'])
    if sh == 1 and shutter_on_start is None:
        shutter_on_start = vt
    if sh == 1:
        shutter_on_end = vt

print(f"laser_shutter=1 窗口: vt [{shutter_on_start}, {shutter_on_end}] 持续 {shutter_on_end-shutter_on_start:.0f} ms")

# enable 窗口
en_start=en_end=None
for r in rows:
    en = int(float(r['laser_enable']))
    vt = float(r['virtual_time_ms'])
    if en==1 and en_start is None: en_start=vt
    if en==1: en_end=vt
print(f"laser_enable=1 窗口: vt [{en_start}, {en_end}] 持续 {en_end-en_start:.0f} ms")

# laser_on_time_ms 起止
lot_vals = [int(float(r['laser_on_time_ms'])) for r in rows]
print(f"laser_on_time_ms: min={min(lot_vals)} max={max(lot_vals)}")
# 找第一/最后非零
first_nonzero = next(i for i,v in enumerate(lot_vals) if v>0)
last_nonzero = max(i for i,v in enumerate(lot_vals) if v>0)
print(f"  首次>0: row {first_nonzero} (vt={float(rows[first_nonzero]['virtual_time_ms']):.0f})")
print(f"  最后非零: row {last_nonzero} (vt={float(rows[last_nonzero]['virtual_time_ms']):.0f})")

# 切割段 (G01 X100) virtual_time 窗口
# 找 X 从 0 到 100 的运动
x_start=x_end=None
for r in rows:
    x = float(r['X'])
    vt = float(r['virtual_time_ms'])
    if x > 1 and x_start is None: x_start = vt
    if x > 1: x_end = vt
print(f"X>1 (切割) 窗口: vt [{x_start}, {x_end}] 持续 {x_end-x_start:.0f} ms")

# dwell 窗口 (G04 P1000): power_w 或 is_piercing=1 的窗口
ip_rows = [(i,float(r['virtual_time_ms'])) for i,r in enumerate(rows) if int(float(r['is_piercing']))==1]
if ip_rows:
    print(f"is_piercing=1 窗口: vt [{ip_rows[0][1]:.0f}, {ip_rows[-1][1]:.0f}] 持续 {ip_rows[-1][1]-ip_rows[0][1]:.0f} ms (行数 {len(ip_rows)})")

# 检查 shutter=1 但 X 未运动的时段 (多计的激光时间)
extra = 0
for r in rows:
    if int(float(r['laser_shutter']))==1 and float(r['X'])<1 and int(float(r['is_piercing']))==0:
        extra += 1
print(f"shutter=1 但非切割非dwell 的行数(多计激光时间): {extra} (~{extra}ms)")
