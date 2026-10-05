#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WP-B condition-level verdict aggregator (frozen with the prereg).

Consumes ONLY the frozen checker outputs (l15_report_*.json produced by the
byte-identical adapter_l15.py) plus the per-run lib sha sidecars. It does NOT
recompute residuals from captures and does NOT modify any frozen component.

Mapping (prereg §Q, frozen):
  Qualification = block_identity(P4) AND hold_window(P6)
                  AND witness_windows_complete (all NC blocks have a capture
                  window AND an evaluated P1 sample set)
  Semantic      = P1 across all blocks (feed_upm == L*F within frozen
                  TOL_PIN=2%)
H-B pattern checks (expected-outcome verification, not verdict inputs):
  LC-T0/T2: every block p1.ok, all ratios within 2%
  LC-T1   : exactly one failing block, its line == 16, ratio in [1.045,1.055],
            at least one passing block before AND after it

Usage: wpb_verdict.py <condition> <report1.json> [<report2.json> ...]
       (reports may be absolute or /home/l15-relative; libsha_<run>.txt is
        looked up next to each report)
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from adapter_l15 import parse_nc  # frozen independent oracle (L*F from NC geometry)

NC = "g93t_probe.ngc"
SEED_LINE = 16
RATIO_LO, RATIO_HI = 1.045, 1.055
TOL = 0.02


def run_id_of(path):
    base = os.path.basename(path)[len("l15_report_"):-len(".json")]
    return base


def main():
    cond = sys.argv[1]
    reports = sys.argv[2:]
    if not reports:
        print("no reports")
        return 2

    blocks = parse_nc(NC)  # independent oracle, unchanged
    oracle = {b["line"]: b["fu"] for b in blocks}

    runs = []
    per_run = []
    for p in reports:
        rid = run_id_of(p)
        d = json.load(open(p, encoding="utf-8"))
        lib = ""
        sha_path = os.path.join(os.path.dirname(os.path.abspath(p)),
                                "libsha_%s.txt" % rid)
        if os.path.exists(sha_path):
            lib = open(sha_path).read().strip()
        # qualification components
        p4 = d["preds"]["P4"]["ok"]
        p6 = d["preds"]["P6"]["ok"]
        win_lines = {w["pl"] for w in d["windows"]}
        windows_complete = win_lines == set(oracle)
        witness_evaluated = all(
            b.get("p1") is not None for b in d["blcks"])
        qual = bool(p4 and p6 and windows_complete and witness_evaluated)
        # semantic components
        fails = [b for b in d["blcks"] if b.get("p1") and not b["p1"]["ok"]]
        sem = all(b.get("p1") and b["p1"]["ok"] for b in d["blcks"])
        per_run.append(dict(
            run=rid, lib_sha12=lib, rows=d["n_rows"],
            p1_ok=d["preds"]["P1"]["ok"], p2_ok=d["preds"]["P2"]["ok"],
            p3_ok=d["preds"]["P3"]["ok"], p4_ok=p4, p6_ok=p6,
            windows_complete=windows_complete,
            witness_evaluated=witness_evaluated,
            qualification=("PASS" if qual else "FAIL"),
            semantic=("PASS" if sem else "FAIL"),
            failing_lines=[b["line"] for b in fails],
            blocks=[dict(line=b["line"], fu_oracle=round(b["fu"], 4),
                         fu_med=b.get("fu_med"),
                         ratio=(None if not b.get("p1") else round(b["p1"]["ratio"], 9)),
                         p1=("PASS" if (b.get("p1") and b["p1"]["ok"]) else
                             ("FAIL" if b.get("p1") else "NONE")),
                         p2_ratio=(None if not b.get("p2") else round(b["p2"]["ratio"], 9)))
                    for b in d["blcks"]]))
        runs.append(rid)

    # H-B expected-pattern verification (per condition)
    hb_ok, hb_why = True, []
    for r in per_run:
        bl = {b["line"]: b for b in r["blocks"]}
        if cond in ("LC-T0", "LC-T2"):
            bad = [l for l, b in bl.items() if b["p1"] != "PASS"]
            if bad:
                hb_ok = False
                hb_why.append("%s: unexpected FAIL on %s" % (r["run"], bad))
        elif cond == "LC-T1":
            if r["failing_lines"] != [SEED_LINE]:
                hb_ok = False
                hb_why.append("%s: failing lines %s != [%d]" %
                              (r["run"], r["failing_lines"], SEED_LINE))
            ratio = bl[SEED_LINE]["ratio"]
            if ratio is None or not (RATIO_LO <= ratio <= RATIO_HI):
                hb_ok = False
                hb_why.append("%s: target ratio %s outside [%s,%s]" %
                              (r["run"], ratio, RATIO_LO, RATIO_HI))
            before = [b for l, b in bl.items() if l < SEED_LINE and b["p1"] == "PASS"]
            after = [b for l, b in bl.items() if l > SEED_LINE and b["p1"] == "PASS"]
            if not before or not after:
                hb_ok = False
                hb_why.append("%s: missing before/after PASS showcase" % r["run"])
        else:
            hb_ok = False
            hb_why.append("unknown condition " + cond)

    # determinism: per-block fu_med identical across runs of the condition
    sigs = {tuple(b["fu_med"] for b in r["blocks"]) for r in per_run}
    deterministic = len(sigs) == 1

    verdict = dict(
        condition=cond,
        runs=runs,
        per_run=per_run,
        signature_deterministic=deterministic,
        qualification=("PASS" if all(r["qualification"] == "PASS" for r in per_run) else "FAIL"),
        semantic=("PASS" if all(r["semantic"] == "PASS" for r in per_run) else "FAIL"),
        hb_pattern_confirmed=hb_ok,
        hb_pattern_notes=hb_why,
        oracle=dict(file=NC, fu_by_line={str(k): round(v, 4) for k, v in oracle.items()}),
        success_criterion=dict(
            qualification_pass=all(r["qualification"] == "PASS" for r in per_run),
            semantic_fail=all(r["semantic"] == "FAIL" for r in per_run),
            note="taskbook §12.1: seed condition needs qualification=PASS and semantic=FAIL"),
    )
    out = "verdicts/%s.json" % cond
    os.makedirs(os.path.dirname(out), exist_ok=True)
    json.dump(verdict, open(out, "w", encoding="utf-8"), indent=1)
    print(json.dumps({k: verdict[k] for k in
                      ("condition", "runs", "qualification", "semantic",
                       "hb_pattern_confirmed", "signature_deterministic")}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
