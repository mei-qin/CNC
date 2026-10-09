# F1 Prospective Microstudy Protocol — Freeze Proposal (NO EXPERIMENT AUTHORIZED)

**Package:** `linuxcnc_f1_protocol` · **Date:** 2026-10-10 · **Revision:** v2 (F1.1 amended — see `F1_1_AMENDMENT.md`; amendment informed by historical-capture recomputation, disclosed, pre-execution; original `F1_PROTOCOL_PACKAGE_20261010.zip` preserved unmodified)
**Line of authority:** S4-B Reframing Gate (B3 GO FOR F0 ONLY) → F0 (static feasibility, this project) → F0 Independent Review (CONDITIONAL PASS; F1 protocol design GO; execution HOLD) → F1 proposal → F1 Independent Review (HOLD; one F1.1 rework round authorized) → **this v2 = the F1.1 review request. No build, run, scoring, or archive edit is authorized by this package.**
**Taskbooks:** F1_PROTOCOL_PREPARATION_TASKBOOK + F1_1_LIMITED_REWORK_TASKBOOK (Works A–D).

---

## 1. Design (frozen)

- **Cells:** T0W0, T0W1, T1W0, T1W1 — one fixed eligibility unit (g93_probe.ngc line-16 G93 arc) per F0; identical NC/INI/HAL/checker bytes across all cells.
- **Replicates:** 3 technical repeats per cell (run IDs `F1-T0W0-R1…R3`, …). Replicates verify signature determinism only; the cell is the unit; **no N=12 independence claim**.
- **Gold standard:** per F1_GOLD_STANDARD_AND_EXPECTATIONS (§1–2), incl. VOID/NON-ADJUDICABLE rules for provenance↔runtime disagreement.
- **Expected values:** pre-declared prediction table (F1_GOLD_STANDARD §4) — includes the W0×T1 declared cross-term (1979.2034) and the W1 masking signature (600.00 identical in T0W1/T1W1).
- **Comparators & scoring:** per F1_COMPARATOR_FREEZE (six workflows, five-way outcomes, anti-rigging rules, parity pre-declared).

## 2. Variants, build, load (frozen requirements; nothing built yet)

Variant composition and byte-grounded source identities: F1_GOLD_STANDARD §3. In summary:

| Variant | interp_inverse.cc | interp_convert.cc | librs274.so.0 |
|---|---|---|---|
| T0W1 stock | c98db3d4… | f2e7cf41… | **frozen stock reference = tree-built d3e4ecd4…** |
| T0W0 | c98db3d4… | **6ab767c7…** (byte-reproduces historical F-1 patched source) | build + record sha before runs |
| T1W1 | **2096776f…** | f2e7cf41… | build + record sha before runs |
| T1W0 | 2096776f… | 6ab767c7… | build + record sha before runs |

Build/load obligations (all four variants, no exceptions; F1.1 item 2 strengthened):

1. Build inside the frozen rip tree from the variant source pair; **echo and archive the exact compiler/build commands and environment**; record post-build source SHA256 + lib SHA256 in a per-variant `source_identity` receipt before any run.
2. Runtime switch via `~/libctl/librs274.so.0` (single variable component); per run: libsha file + **hard gate abort unless `/proc/<milltask>/maps` shows the libctl library** + maps snapshot archived with **inode and library path**, plus **pre-run and post-run SHA256 of the exact libctl file and a hash of the mapped file read during execution** (F1.1 Work D.2) (`oneshot_wpb.sh` lineage, §3 of that script: read-verified this session).
3. Historical manifest hashes are **precedent only**; they do not certify any new build. The single stock reference lib (d3e4ecd4) anchors stock equivalence and revert checks; deb lib 3a8ce231 is reference-only.
4. Runner: `oneshot_wpb.sh` lineage adapted back to the arc probe (`g93_probe.ngc`, `l15_drive.py` timeline) **plus** the F1 capture-integrity accounting (overruns==0, non-data-line count, parse-fail abort — F1_CAPTURE_AND_WINDOW_SPEC §3).

## 3. Frozen tolerances and tools (unchanged bytes)

`adapter_l15.py` eb6a6690… with TOL_PIN=2%, TOL_OVR=3%, H1_MARGIN=1%, HOLD_MIN=0.1 s; `g93_probe.ngc` fdcc31b2…; `g93_probe.hal` 660550b6…. Any byte change to any frozen tool ⇒ protocol void ⇒ re-review.

## 4. Pre-execution gates (in order)

1. **Independent signed audit of this frozen proposal** (the F1 review): completeness of source receipts, capture schema correction, comparator fairness/parity policy, truth independence, outcome coding. 
2. Only after explicit F1 execution approval: **guard-activation dry run** (VOID-marked, non-scored; WP-B §9 precedent) confirming the line-16 seed fires and the W patch governs the arc tag.
3. Then the 12 formal runs (4 cells × 3), each with full per-run attestation; VOID rules apply per F1_GOLD_STANDARD §2.

## 5. STOP criteria (single round; no successive ad-hoc repairs)

1. FULL-B cannot be adequately specified without new research-size tooling.
2. No trustworthy independent T1/W1 truth (provenance or motion-side chain breaks).
3. Row/block correspondence or loaded-binary attestation unreliable.
4. Planned wins would rely on giving I+Q superior information or denying FULL-B source access.
5. Runtime T/W independence fails (a toggle leaks into the other axis beyond the declared W0×T1 cross-term).
Any STOP ⇒ return to the S4-4 manuscript unchanged; EAV-1 stays stopped.

## 6. Package contents & receipts

```
linuxcnc_f1_protocol/
  F1_PROTOCOL_FREEZE.md                (this file; v2)
  F1_1_AMENDMENT.md                    (F1.1 bounded correction: disclosure + Works A–D)
  F1_CAPTURE_AND_WINDOW_SPEC.md        (Work 1 — capture erratum + selector freeze)
  F1_GOLD_STANDARD_AND_EXPECTATIONS.md (Work 2 — truth + predictions; F1.1 execution-truth separation)
  F1_COMPARATOR_FREEZE.md              (Work 3 — fairness + verdict rules + scoring; F1.1 v2 arbitration + FULL-B tree + precedence)
  pristine/interp_inverse.cc           c98db3d4… (WSL byte copy, hash-verified)
  pristine/interp_convert.cc           f2e7cf41… (WSL byte copy, hash-verified)
  variants/T1W1_interp_inverse.cc      2096776f… (prospective seeded source)
  variants/T0W0_interp_convert.cc      6ab767c7… (= historical patched source, reproduced)
  variants/HASHES.txt
  patches/F1_T_seed_interp_inverse.patch
  patches/F1_W_tag_interp_convert.patch
  patches/F1_TW_combined.patch
  receipts/adapter_l15.py              eb6a6690… (checker byte receipt, WSL copy verified)
  receipts/g93_probe.ngc               fdcc31b2…
  receipts/g93_probe.hal               660550b6…
  receipts/F1_1_RETROSPECTIVE_RECEIPT.json (13 historical captures, NON-SCORED)
  tools/make_variants.py               (text-only reconstruction script)
  tools/f11_retrospective_check.py     (read-only retrospective analysis script)
  SHA256SUMS
```

**Declaration:** no controller build, seeded-code commit to any runtime tree, test run, pilot count, EAV-1 restart, or edit to B3/F-1/WP-B/CS frozen archives has occurred in the preparation of this package. The `variants/` files are text artifacts constructed offline from pristine bytes; the WSL subject tree remains pristine (re-verified by hash this session).
