# F0 Gate Report — Binary Feasibility Decision (Read-only)

**Package:** `linuxcnc_f0_interaction` · **Date:** 2026-10-09
**Taskbook:** MICRO_STUDY_F0_FEASIBILITY_TASKBOOK (via `S4_B_Reframing_Interaction_Test_Package.zip`, sha256 `40546429ecddcd566961de735a7f40a5b398cff4255368afdd1c51a21d42befe`)
**Companion deliverables:** `F0_EVIDENCE_ADMISSIBILITY_MATRIX.csv`, `F0_SOURCE_BASIS.md`, `F0_BASELINE_FAIRNESS.md`

---

## Verdict

# READY_FOR_INDEPENDENT_PROTOCOL_REVIEW

All six must-have static checks pass. No dynamic execution was performed and none is authorized by this report. The four cells are constructible with bounded, single-file source work on proven templates; truth is independently adjudicable in every cell (including T1/W1); same-stage/same-window evidence is structural; build→load linkage has a fully specified per-run protocol with a direct historical precedent; baseline access is fair and the parity risk is explicitly accepted.

## Must-have checklist

| Gate | Must-have (taskbook) | Finding | State |
|---|---|---|---|
| G1 | One G93 arc block; expected conversion, original formula, native diagnostic + independent witness, units, block bounds, run assumptions; no cross-cycle promotion without key | Line 16 arc (L=15.70796327, F=120); fu=L·F=1884.9556 mm/min, rv=/60=31.415927 mm/s; witness=feed-upm tag pin (mm/min); independent=requested-vel pin + offline NC oracle; windows=sampler-row spans keyed on mtype/program-line; single 1 kHz sampler ⇒ row index is the same-cycle key | **PASS** |
| G2 | Exact prospective source locations for T and W toggles; toggling one does not silently change the other; alternatives isolable | T=`interp_inverse.cc:68` (post-`rate=length*block->f_number` guarded ×1.05, mirror of WP-B straight twin, same file, proven guard); W=`interp_convert.cc:602/:689/:847` append `write_canon_state_tag` (pre-registered blueprint, historically executed p1s1-3); disjoint files/functions; T never writes tag inputs, W never writes conversion inputs; W0×T1 product value pre-declared (1979.2034); W1 masking is the designed object, adjudicable via independent channels; build/harness alternatives excluded by libctl+maps+revert protocol | **PASS** (previous BLOCKED item — arc-specific T design — resolved by direct source read) |
| G3 | Independent evidence separating CORRECT_CONVERSION / CONVERSION_VIOLATION / UNTRUSTED_WITNESS even in T1/W1; truth never assigned by the method's own verdict | T truth = source diff + requested-vel vs NC oracle (motion-side, not tag-derived); W truth = source diff + fu/rv dissociation; in T1/W1: fu stays stale 600.00 (untrusted) while rv reads 32.986723 (ratio exactly 1.05) — conversion fact established without the audited witness | **PASS** |
| G4 | Exact commit/version, build flags, diff/hash of both toggles, products, load path, process-map linkage; 2.9.4 ≠ 2.9.10 | Subject pinned: deb `1:2.9.4-2+deb13u1` (e14c9399…), rip tree, stock milltask d90c4b57…; four lib variants specified with composition; per-run libsha + `/proc/<milltask>/maps` hard gate + snapshot archival; **correction to preliminary screen:** F1-CLOSURE-FREEZE-1 manifest *does* record the patched pair (source 6ab767c7 / lib 9972b40d), so the W0 historical identity chain is closed at manifest level; prospective protocol requires per-run receipts for all cells | **PASS** (upgraded from HOLD; per-run receipt discipline carried into F1) |
| G5 | O / I / Q / I+Q / FULL-B with matched access; ABSTAIN counted separately; equivalence reported as equivalence | Five workflows defined on identical raw access (10-channel capture, NC/INI/HAL, provenance receipts); expected-classification table and false-attribution analysis in F0_BASELINE_FAIRNESS.md; I(P1-only) provably cannot separate T0W1 from T1W1 (identical fu signatures) — the measurable mechanism; FULL-B parity explicitly accepted as a reportable outcome | **PASS** |
| G6 | Four planned cells with audit-unit identities, pre-declared truth, identical NC/windows, no observed outcomes; replicates don't multiply units | Matrix CSV: four cells on one fixed unit (g93_probe.ngc line 16, sha fdcc31b2…), truth columns all independent of the audited witness, identical NC/window spec, feasibility states only; expected-value table marked as unscored predictions; historical signatures labeled H; unit=cell, replicates=3 technical | **PASS** |

## Evidence-base corrections recorded during F0 (transparency)

1. Preliminary screen's G4 "incomplete patched-binary provenance" is narrowed: the patched source/lib hash pair exists in `FREEZE_MANIFEST.json` (6ab767c7 / 9972b40d); what is missing at package level is the patched source *file member* and per-run maps snapshots — the latter is made mandatory for all four prospective cells.
2. The preliminary screen's G2 "BLOCKED until arc-specific T design independently checked" is resolved by direct read of `interp_inverse.cc` L47–73: the arc twin is line-parallel to the seeded straight twin; no new mechanism is required.

## Explicit non-claims

- No experiment was run; no number in this package is a new observation.
- No claim that four historical runs already constitute the 2×2 (they do not: T1 cells are prospective only).
- No claim of relative advantage over FULL-B; equivalence will be reported as equivalence.
- No EAV-1 revival; no change to frozen archives (B3, LC-D1, WP-B, CS, pose, A0–A3).

## Conditions bound into F1 (all-or-nothing; no ad-hoc repair rounds)

1. F1 freeze must fix: hypothesis, four-cell design, the single stock reference lib identity, machine-readable case selection, per-cell expected source-level truth (§7 of F0_SOURCE_BASIS), unchanged runtime gates (adapter eb6a6690 tolerances), ABSTAIN/cause counting rules (§5 of F0_BASELINE_FAIRNESS), and the I-workflow tie-breaking convention (F0_BASELINE_FAIRNESS §4.3) — all before any run.
2. Guard-activation dry-run for the arc seed (WP-B §9 precedent) — VOID-able, not scored.
3. Per-run maps snapshot + libsha for every cell; source/lib hashes for the two new T1 variants recorded at build time; W patch must reproduce source sha 6ab767c7.
4. Any must-have failure during F1 execution (mixed commit, missing receipt, non-independent toggles manifesting at runtime, window non-isomorphism) ⇒ STOP in one round; return to S4-4 manuscript positioning. Original EAV-1 remains stopped.

**Bottom line:** the read-only screen found the two intervention axes source-orthogonal, independently adjudicable, and buildable on frozen precedents. The remaining risk is concentrated exactly where the gate places it — independent F1 protocol review — not in tool construction.
