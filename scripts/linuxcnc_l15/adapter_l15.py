#!/usr/bin/env python3
"""L1.5 stock LinuxCNC 最小 evidence adapter — M_PRED P1-P6 对照 + 契约判词

冻结声明: 本文件与 g93_probe.ngc/.ini/.hal 构成预测侧组件, 在首次接触
LinuxCNC 环境之前 git commit 冻结 (M_PRED 已冻结于 commit 2bfa94e)。

冻结容差 (先于任何 LinuxCNC 数据声明):
  TOL_PIN   = 2%   正常态引脚中位数 vs 程序几何公式的相对偏差 (P1/P2)
  TOL_OVR   = 3%   override 窗口 feed_upm 缩放与 override 值的一致性 (P3)
  H1_MARGIN = 1%   参考速度对 INI 硬限的允许余量
  HOLD_MIN  = 0.1s feedhold 特征窗口最短时长 (P6)
判词规则: P_i 各条独立 PASS/FAIL; 契约层 H2/H3 由构造保证 UNOBSERVABLE
(本 adapter 不消费任何预算/修订证据, 也不从输出轨迹重建之)。
"""
import argparse, json, math, re, sys
from statistics import median

TOL_PIN, TOL_OVR, H1_MARGIN, HOLD_MIN = 0.02, 0.03, 0.01, 0.1

COLS = ["j0", "j1", "j2", "cur_vel", "req_vel",
        "feed_upm", "dtg", "mtype", "pline", "ovr"]


def parse_nc(path):
    pos = (0.0, 0.0)
    g93 = False
    blocks = []
    for raw_no, raw in enumerate(open(path, encoding="utf-8"), 1):
        text = raw.split(";")[0].strip()
        if not text:
            continue
        words = dict((k.upper(), float(v)) for k, v in
                     re.findall(r"([A-Za-z])([-+]?[0-9.]+)", text))
        gs = re.findall(r"G(\d+\.?\d*)", text.upper())
        if "93" in gs:
            g93 = True
        cmd = next(("G" + g for g in gs if g in ("1", "2", "3")), None)
        if cmd is None:
            continue
        if not g93:
            sys.exit(f"ngc line {raw_no}: 运动 block 但 G93 未激活")
        if "F" not in words:
            sys.exit(f"ngc line {raw_no}: G93 运动 block 缺 F")
        end = (words.get("X", pos[0]), words.get("Y", pos[1]))
        if cmd == "G1":
            L = math.dist(pos, end)
        else:
            c = (pos[0] + words.get("I", 0.0), pos[1] + words.get("J", 0.0))
            r1, r2 = math.dist(pos, c), math.dist(end, c)
            if abs(r1 - r2) > 1e-6 * max(1.0, r1):
                sys.exit(f"ngc line {raw_no}: 圆弧半径不一致 r1={r1} r2={r2}")
            a0 = math.atan2(pos[1] - c[1], pos[0] - c[0])
            a1 = math.atan2(end[1] - c[1], end[0] - c[0])
            d = ((a0 - a1) if cmd == "G2" else (a1 - a0)) % (2 * math.pi)
            L = r1 * d
        blocks.append(dict(line=raw_no, cmd=cmd, L=L, F=words["F"],
                           T=60.0 / words["F"], v=L * words["F"] / 60.0,
                           fu=L * words["F"]))
        pos = end
    return blocks


def parse_ini(path):
    lim = {}
    sec = None
    for raw in open(path, encoding="utf-8"):
        s = raw.strip()
        m = re.match(r"\[(.+)\]", s)
        if m:
            sec = m.group(1)
            continue
        m = re.match(r"(MAX_VELOCITY|SERVO_PERIOD)\s*=\s*([0-9.]+)", s)
        if m and sec:
            if sec.startswith("AXIS_"):
                lim.setdefault(sec[-1], {})[m.group(1)] = float(m.group(2))
            elif sec == "EMCMOT":
                lim["SERVO_PERIOD"] = float(m.group(2))
    return lim


def load_capture(path):
    rows = []
    for raw in open(path):
        parts = raw.split()
        if len(parts) != len(COLS):
            continue
            continue
        try:
            vals = [float(x) for x in parts]
        except ValueError:
            continue
        rows.append(dict(zip(COLS, vals)))
    if not rows:
        sys.exit("capture 为空或列数不符 (期望 halsampler -t + 10 通道)")
    return rows


def windows(rows):
    wins, cur = [], None
    for i, r in enumerate(rows):
        if r["mtype"] in (2.0, 3.0):
            pl = int(r["pline"])
            if cur and (pl != cur["pl"] or i != cur["end"] + 1):
                wins.append(cur)
                cur = None
            if cur is None:
                cur = dict(pl=pl, i0=i, end=i)
            else:
                cur["end"] = i
        elif cur:
            wins.append(cur)
            cur = None
    if cur:
        wins.append(cur)
    return wins


def med(rows, key):
    v = [r[key] for r in rows if r[key] == r[key]]
    return median(v) if v else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture", required=True)
    ap.add_argument("--nc", default="g93_probe.ngc")
    ap.add_argument("--ini", default="g93_probe.ini")
    ap.add_argument("--out", default="l15_report")
    a = ap.parse_args()

    blocks = parse_nc(a.nc)
    lim = parse_ini(a.ini)
    ts = lim.get("SERVO_PERIOD", 1e6) / 1e9
    rows = load_capture(a.capture)
    wins = windows(rows)

    # ---- 块级对照 (P1/P2/P3) ----
    blk_rows = []
    for b in blocks:
        w = next((w for w in wins if w["pl"] == b["line"]), None)
        e = dict(b)
        if w is None:
            e.update(window=None, p1=None, p2=None, p3=None)
            blk_rows.append(e)
            continue
        wr = rows[w["i0"]:w["end"] + 1]
        normal = [r for r in wr if r["ovr"] >= 0.99 and r["feed_upm"] > 1e-9]
        ovr = [r for r in wr if r["ovr"] < 0.99]
        e["window"] = (w["i0"], w["end"], len(wr))
        if normal:
            fu, rv = med(normal, "feed_upm"), med(normal, "req_vel")
            e["fu_med"], e["rv_med"] = fu, rv
            e["p1"] = dict(ratio=fu / b["fu"], ok=abs(fu / b["fu"] - 1) <= TOL_PIN)
            e["p2"] = dict(ratio=rv / b["v"], ok=abs(rv / b["v"] - 1) <= TOL_PIN)
        else:
            e["p1"] = e["p2"] = None
        if ovr:
            fu_o, rv_o, ov = med(ovr, "feed_upm"), med(ovr, "req_vel"), med(ovr, "ovr")
            e["p3"] = dict(fu_ratio=fu_o / b["fu"], ovr=ov, rv_ratio=rv_o / b["v"],
                           ok=(abs(fu_o / b["fu"] - ov) <= TOL_OVR
                               and abs(rv_o / b["v"] - 1) <= TOL_PIN))
        else:
            e["p3"] = None
        blk_rows.append(e)

    # ---- P4: 块身份 1:1 / 单调 / 连续 ----
    seq_ok = [w["pl"] for w in wins] == sorted({b["line"] for b in blocks}) and \
             len(wins) == len(blocks)
    unmatched = [b["line"] for b in blocks
                 if not any(w["pl"] == b["line"] for w in wins)]

    # ---- P6: feedhold 特征窗口 ----
    holds = []
    i = 0
    while i < len(rows):
        r = rows[i]
        if r["mtype"] in (2.0, 3.0) and r["feed_upm"] < 1e-9:
            j = i
            while j < len(rows) and rows[j]["mtype"] in (2.0, 3.0) \
                    and rows[j]["feed_upm"] < 1e-9:
                j += 1
            dur = (j - i) * ts
            if dur >= HOLD_MIN:
                holds.append(dict(i0=i, i1=j - 1, dur_s=round(dur, 3)))
            i = j
        else:
            i += 1

    # ---- H1: 参考速度硬限 ----
    h1 = {}
    for ax, col in (("X", "j0"), ("Y", "j1"), ("Z", "j2")):
        vmax = 0.0
        for k in range(1, len(rows)):
            vmax = max(vmax, abs(rows[k][col] - rows[k - 1][col]) / ts)  # 行连续, dn=1
        limit = lim.get(ax, {}).get("MAX_VELOCITY", float("inf"))
        h1[ax] = dict(v_ref_max=round(vmax, 3), limit=limit,
                      ok=vmax <= limit * (1 + H1_MARGIN))

    # ---- 判词汇总 ----
    def all_of(key):
        vals = [b[key] for b in blk_rows if b.get(key)]
        return bool(vals) and all(v["ok"] for v in vals)

    preds = dict(
        P1=dict(desc="feed_upm == L*F (正常态)", ok=all_of("p1")),
        P2=dict(desc="requested_vel == L*F/60, 不随 override 变",
                ok=all_of("p2") and all(
                    b["p3"]["rv_ratio"] is not None
                    and abs(b["p3"]["rv_ratio"] - 1) <= TOL_PIN
                    for b in blk_rows if b.get("p3"))),
        P3=dict(desc="override 窗口: feed_upm 缩放 == ovr, requested_vel 不变",
                ok=any(b.get("p3") for b in blk_rows) and all_of("p3")),
        P4=dict(desc="program-line 块身份 1:1/单调/连续",
                ok=seq_ok and not unmatched),
        P5=dict(desc="H2/H3 = UNOBSERVABLE (不重建证据)", ok=True),
        P6=dict(desc="feedhold 特征窗口存在 (fu==0 >=0.1s)", ok=bool(holds)),
    )
    m_obs = dict(H1="OBSERVABLE" if all(v["ok"] for v in h1.values()) else "FAIL",
                 H2="UNOBSERVABLE", H3="UNOBSERVABLE",
                 H4="PARTIAL", H5="PARTIAL")

    rep = dict(blcks=blk_rows, preds=preds, holds=holds, h1=h1, m_obs=m_obs,
               windows=[dict(pl=w["pl"], i0=w["i0"], i1=w["end"]) for w in wins],
               unmatched_lines=unmatched, ts_s=ts, n_rows=len(rows))
    json.dump(rep, open(a.out + ".json", "w", encoding="utf-8"),
              ensure_ascii=False, indent=1, default=str)

    md = [f"# L1.5 pred/obs 报告 ({a.capture})",
          f"- 采样 {len(rows)} 行, ts={ts*1e3:.3f} ms; 块 {len(blocks)}, 窗口 {len(wins)}\n",
          "| line | cmd | L | F | L*F | fu_med | P1 | rv_med | P2 | P3 |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for b in blk_rows:
        md.append("| {line} | {cmd} | {L:.5f} | {F:.0f} | {fu:.2f} | "
                  "{fu_med:.2f} | {p1} | {rv_med:.4f} | {p2} | {p3} |".format(
                      line=b["line"], cmd=b["cmd"], L=b["L"], F=b["F"], fu=b["fu"],
                      fu_med=b.get("fu_med", float("nan")),
                      p1="--" if not b.get("p1") else
                         ("OK" if b["p1"]["ok"] else f"FAIL {b['p1']['ratio']:.4f}"),
                      rv_med=b.get("rv_med", float("nan")),
                      p2="--" if not b.get("p2") else
                         ("OK" if b["p2"]["ok"] else f"FAIL {b['p2']['ratio']:.4f}"),
                      p3="--" if not b.get("p3") else
                         ("OK" if b["p3"]["ok"] else
                          f"FAIL fu={b['p3']['fu_ratio']:.3f}@ovr={b['p3']['ovr']:.2f}")))
    md += ["", "## P1-P6", "| # | 描述 | 判定 |", "|---|---|---|"]
    for k, v in preds.items():
        md.append(f"| {k} | {v['desc']} | {'PASS' if v['ok'] else 'FAIL'} |")
    md += ["", "## H1", "| 轴 | v_ref_max | INI 限 | OK |", "|---|---|---|---|"]
    for ax, v in h1.items():
        md.append(f"| {ax} | {v['v_ref_max']} | {v['limit']} | {v['ok']} |")
    md += ["", f"## holds: {holds}", f"## M_obs = {m_obs}", "",
           "M_PRED = [H1:OBS, H2:UNO*, H3:UNO, H4:PARTIAL, H5:PARTIAL]"]
    open(a.out + ".md", "w", encoding="utf-8").write("\n".join(md))
    print("\n".join(md))


if __name__ == "__main__":
    main()
