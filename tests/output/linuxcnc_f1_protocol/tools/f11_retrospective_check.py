#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F1.1 retrospective (NON-SCORED) check of the v2 arbitration rules.

Validates, on all accessible historical captures only:
  R1 kinematic integrity : |sum(cur_vel)*ts / D_path - 1| <= 2%
  R2 command agreement   : |(fu_med/60) / rv_med - 1| <= 2%   (normal rows)
  R3 T-indication        : |rv_med / (L*F/60) - 1| <= 2%
and documents the v1 failure mode (whole-window integration of command-level
channels) plus the position-derived execution-time statistic t_exec98 used by
the (amended) gold standard.

No CNC execution. Historical data only. Output: JSON receipt.
"""
import hashlib
import json
import math
import sys

TS = 0.001
TOL = 0.02
L_ARC = 15.707963267948966   # r=10 quarter circle
F_ARC = 120.0
RV_ORACLE = L_ARC * F_ARC / 60.0

FILES = [
    ("g1r1", "T0W1 stock 2.9.4 (F-1 G1)"), ("g1r2", "T0W1 stock 2.9.4 (F-1 G1)"),
    ("g1r3", "T0W1 stock 2.9.4 (F-1 G1)"), ("g1s1", "T0W1 stock 2.9.4 (G1 rerun)"),
    ("g1s2", "T0W1 stock 2.9.4 (G1 rerun)"), ("g1s3", "T0W1 stock 2.9.4 (G1 rerun)"),
    ("p1s1", "T0W0 patched 2.9.4 (F-1 G3)"), ("p1s2", "T0W0 patched 2.9.4 (F-1 G3)"),
    ("p1s3", "T0W0 patched 2.9.4 (F-1 G3)"), ("g5s1", "T0W1 revert 2.9.4 (F-1 G5)"),
    ("cs1", "T0W1 stock 2.9.10 (CS-PRESENT)"), ("cs3", "T0W1 stock 2.9.10 (CS-PRESENT)"),
    ("cs5", "T0W1 stock 2.9.10 (CS-PRESENT)"),
]


def load(path):
    rows, nondata, parse_fail = [], 0, 0
    for raw in open(path):
        p = raw.split()
        if len(p) != 10:
            nondata += 1
            continue
        try:
            rows.append([float(x) for x in p])
        except ValueError:
            parse_fail += 1
    return rows, nondata, parse_fail


def arc_window(rows):
    spans, cur = [], None
    for i, r in enumerate(rows):
        if r[7] in (2.0, 3.0):
            pl = int(r[8])
            if cur and (pl != cur["pl"] or i != cur["end"] + 1):
                spans.append(cur)
                cur = None
            cur = dict(pl=pl, i0=i, end=i) if cur is None else dict(pl=cur["pl"], i0=cur["i0"], end=i)
        elif cur:
            spans.append(cur)
            cur = None
    if cur:
        spans.append(cur)
    for s in spans:
        if s["pl"] == 16:
            return s
    return None


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "."
    out = []
    for name, desc in FILES:
        path = "%s/capture_%s.csv" % (src, name)
        blob = open(path, "rb").read()
        rows, nondata, parse_fail = load(path)
        w = arc_window(rows)
        assert w is not None, name
        seg = rows[w["i0"]:w["end"] + 1]
        d_path, sum_cv, sum_rv, sum_fu = 0.0, 0.0, 0.0, 0.0
        cum, t_rows = 0.0, None
        prev = None
        for k, r in enumerate(seg):
            if prev is not None:
                step = math.dist(prev[0:3], r[0:3])
                d_path += step
                cum += step
                if t_rows is None and cum >= 0.98 * L_ARC:
                    t_rows = k
            sum_cv += r[3] * TS
            sum_rv += r[4] * TS
            sum_fu += (r[5] / 60.0) * TS
            prev = r
        normal = [r for r in seg if r[9] >= 0.99 and r[5] > 1e-9]
        nf = sorted(r[5] for r in normal)[len(normal) // 2]
        nr = sorted(r[4] for r in normal)[len(normal) // 2]
        r1 = sum_cv / d_path - 1
        r2 = (nf / 60.0) / nr - 1
        r3 = nr / RV_ORACLE - 1
        out.append(dict(
            run=name, desc=desc, sha256=hashlib.sha256(blob).hexdigest(),
            data_rows=len(rows), non_data_lines=nondata, parse_fail=parse_fail,
            arc_rows=len(seg), arc_dur_s=round(len(seg) * TS, 3),
            D_path_mm=round(d_path, 6),
            v1_style_int_req_vel_mm=round(sum_rv, 6),
            v1_style_rel_err_req_vel=round(sum_rv / d_path - 1, 6),
            v1_style_int_feed_upm_mm=round(sum_fu, 6),
            v1_style_rel_err_feed_upm=round(sum_fu / d_path - 1, 6),
            int_current_vel_mm=round(sum_cv, 6),
            R1_kinematic_rel_err=round(r1, 6), R1_pass=abs(r1) <= TOL,
            fu_med=nf, rv_med=nr,
            R2_command_agreement_rel_err=round(r2, 6), R2_pass=abs(r2) <= TOL,
            R3_T_indication_rel_err=round(r3, 6), R3_pass=abs(r3) <= TOL,
            t_exec98_s=round((t_rows * TS) if t_rows is not None else -1, 4),
            v_eff98=round(L_ARC / (t_rows * TS), 4) if t_rows else -1,
        ))
    rep = dict(
        study="F1.1 retrospective rule validation on historical captures; NON-SCORED; no new CNC runs",
        rules={"R1": "|sum(cur_vel)*ts / D_path - 1| <= 2%",
               "R2": "|(fu_med/60) / rv_med - 1| <= 2% (normal rows)",
               "R3": "|rv_med / (L*F/60) - 1| <= 2%",
               "v1_documented_failure": "whole-window integration of command-level channels (req_vel/feed_upm) vs path length"},
        constants=dict(ts_s=TS, L_arc_mm=L_ARC, F=F_ARC, rv_oracle=RV_ORACLE),
        records=out)
    print(json.dumps(rep, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
