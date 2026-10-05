# Preregistration — LinuxCNC Positive Transformation-Violation Case (WP-B)

Canonical name: **LC-POSX-1** (condition labels LC-T0 / LC-T1 / LC-T2)
Document: `tests/output/linuxcnc_positive_case/prereg/prereg_positive_transform_violation.md`
Design authority: `Work_Package_B_LinuxCNC_Positive_Transformation_Case.md` (§9 freeze list)

**Freeze discipline.** Sections 1–8 below are fixed before any formal run.
Exploratory dry-runs (§9) are permitted before freezing (taskbook §6.1) and are
recorded as such. After the FREEZE record (§10) is written, no change to δ,
target block, tolerance, checker, oracle, expected-value definition, or
repetition plan is allowed; deviations are handled only by dated amendment.

---

## 1. Position and claim boundary

This experiment is a **positive control / construct-validity case** for RQ3:

    correct binding + admissible evidence + true transformation violation
    => qualified semantic FAIL

It complements the existing LinuxCNC Class-C case (wrong witness semantics =>
qualified inconsistency => source adjudication). It is NOT a fault-prevalence
study, NOT a detection-rate estimate, NOT a claim that LinuxCNC contains such a
defect naturally, and NOT a physical-safety argument. The seeded condition is an
experimental intervention, clearly marked in the source patch and never to be
described as a LinuxCNC defect.

Subject: **LinuxCNC 2.9.4-2+deb13u1** (the frozen F-1 subject), WSL distro
Debian-L15, system apt stack + `~/libctl` single-library switch, `/proc/<milltask>/maps`
hard load verification per run — identical runtime discipline to F-1.

## 2. Responsibility under test

G93 inverse-time feed transformation, straight-move path:

    f_u = L · F

- L: straight-block path length (program geometry)
- F: inverse-time feed parameter (block F word)
- f_u: post-conversion rate, produced by
  `Interp::inverse_time_rate_straight` (src/emc/rs274ngc/interp_inverse.cc),
  which computes `rate = length * block->f_number`, then
  (a) `enqueue_SET_FEED_RATE(rate)` — the real commanded feed, and
  (b) `settings->feed_rate = rate` — the value captured by the canonical state
      tag (`GM_FIELD_FLOAT_FEED`) that drives the `motion.feed-upm` diagnostic
      witness (task-planner side).

The witness binding is UNCHANGED in all three conditions: the `motion.feed-upm`
HAL pin remains bound to the post-conversion rate producer via the canonical
state tag. The seed acts inside the transformation itself, so both the witness
value and the commanded motion change together (expected; taskbook §13).

## 3. Seed (frozen design)

- Form: multiplicative bias on the post-conversion rate (taskbook §6.1 first choice)
- Magnitude: **δ = 0.05** (`f_u_seed = 1.05 · L · F`)
- Location: `interp_inverse.cc`, `Interp::inverse_time_rate_straight`, the
  `rate = length * block->f_number;` statement of the straight-path function
  (the arc twin statement is NOT touched; the patch anchor includes the
  whitespace line unique to the straight function)
- Targeting: **only the preregistered block** — `settings->sequence_number == 16`,
  i.e. source line 16 of `g93t_probe.ngc` (B4). `settings->sequence_number` is
  the NC source line counter (`interp_execute.cc:241` assigns it to
  `block->line_number`; the L1.5 P4 1:1 window/line match empirically confirms
  the identity on this exact harness).
- All other G93 blocks and all non-G93 paths are untouched.

Isolation conditions (taskbook §5.1):
- C1 binding isolation: no change to witness source identifier, producer stage,
  evidence owner, observation scope, lineage declaration, or role binding. The
  observation channel (`g93_probe.hal`, 10ch sampler @1kHz) is byte-identical.
- C2 qualification preservation: the seeded run must keep P4 (block identity),
  P6 (feedhold window), complete per-block windows, and evaluated P1 samples.
  If the seeded run becomes INVALID/UNOBSERVABLE, the seed design has failed
  and must be redone (taskbook §12.2).
- C3 transformation-only: the patch touches only the transformation function —
  not the diagnostic capture, not the checker, not the CSV, not the replay
  script, not the tolerance, not the witness location.
- C4 independent oracle: expected values remain `L·F` and `L·F/60` computed by
  the frozen `adapter_l15.py::parse_nc` from NC geometry alone; the seeded
  implementation is never consulted for expected values.
- C5 reversibility: stock → seeded → revert proven by source hashes, library
  SHA256, and per-run `/proc/<milltask>/maps` loaded-image verification.

## 4. Input program (authored, frozen)

`nc/g93t_probe.ngc` — straight-only G93 program, 6 motion blocks (source lines
13–18), **no arcs** (the F-1 arc tag-ordering signature would make stock FAIL
on arcs; a positive case requires stock PASS on every block). G61 exact path,
per-block F word, M53 P1 feedhold-permissive header — same conventions as the
L1.5 probe.

| block | line | cmd | L (mm) | F (1/min) | T (s) | v=L·F/60 (mm/s) | fu=L·F (upm) |
|---|---|---|---|---|---|---|---|
| B1 | 13 | G1 X10   | 10.00000 | 24 | 2.5 | 4.000  | 240.00 |
| B2 | 14 | G1 Y10   | 10.00000 | 30 | 2.0 | 5.000  | 300.00 |
| B3 | 15 | G1 X0    | 10.00000 | 60 | 1.0 | 10.000 | 600.00 |
| B4 | 16 | G1 Y0    | 10.00000 | 60 | 1.0 | 10.000 | 600.00 | **seed target** |
| B5 | 17 | G1 X10Y10| 14.14214 | 30 | 2.0 | 7.071  | 424.26 |
| B6 | 18 | G1 X0Y0  | 14.14214 | 60 | 1.0 | 14.142 | 848.53 |

Seeded expectation (LC-T1 only): B4 fu = 630.00 upm, v = 10.5 mm/s; all other
blocks exactly as above.

Action timeline (frozen, identical to L1.5): ovr50 @t+1.0 and ovr100 @t+3.0
(both mid-B1), feedhold 4.5–5.5 s (mid-B2). B3–B6 are intervention-free by
construction (computed timeline: B1 ends ≈4.0 s, B2 spans ≈4.0–7.0 s incl. the
1 s hold; seeded B4 shortens 1.0 s → ≈0.952 s).

## 5. Frozen checker and verdict mapping

Checker: `scripts/linuxcnc_l15/adapter_l15.py`, byte-identical (LF) to the copy
used in every L1.5/F-1 run — sha256 `eb6a669039403126a3c2b770b9c4e62ea50336c6224cd0fb3dd5cf7827125905`.
Tolerances frozen inside it (TOL_PIN=2%, TOL_OVR=3%); NOT changed for this
experiment. The verdict aggregator `scripts/linuxcnc_l15/wpb_verdict.py`
consumes only adapter outputs (it never recomputes residuals and never reads
the seeded implementation).

Mapping (frozen):

| SEI term | Operational check in this experiment |
|---|---|
| TraceValid / block identity | P4: program-line windows 1:1, monotone, complete vs NC blocks |
| Observation channel live | P6: feedhold feature window ≥0.1 s present; sampler non-empty |
| Witness presence per block | every NC block has a capture window AND an evaluated P1 sample set |
| **Qualification** | P4 ∧ P6 ∧ windows-complete ∧ witness-present (all must hold) |
| Semantic predicate (R = f_u==L·F) | P1 per block and aggregate, frozen 2% tolerance |
| Motion-consistency predicate | P2 (requested_vel == L·F/60); expected to FAIL on the seeded block in LC-T1 (motion changes — taskbook §13) |

Verdict semantics per condition:
- `Verdict = PASS` ⇔ Qualification = PASS ∧ semantic P1 = PASS
- `Verdict = FAIL` ⇔ Qualification = PASS ∧ semantic P1 = FAIL
- Qualification failure ⇒ INVALID (not a FAIL; taskbook §12.2)

## 6. Conditions and expected outcomes (frozen before formal runs)

| Condition | libctl/librs274.so.0 | Reps | Qualification | Semantic P1 | P2 profile | Verdict |
|---|---|---|---|---|---|---|
| LC-T0 stock | tree-rebuilt STOCK lib | 3 | PASS | PASS (all 6 blocks) | PASS | PASS |
| LC-T1 seeded | tree-rebuilt seeded lib | 3 | PASS | **FAIL, exactly block line 16, ratio ∈ [1.045,1.055]** | FAIL on line 16 only | **FAIL** |
| LC-T2 revert | byte-identical LC-T0 lib artifact | 3 | PASS | PASS (all 6 blocks) | PASS | PASS |

Additional frozen expectations for LC-T1 (H-B2 pattern, checked by the
aggregator): at least one passing block before AND after line 16 (isolation
showcase); per-block fu_med bit-stable across the 3 repetitions within each
condition (determinism, as in F-1/L1.5).

Design note (stronger than the F-1 contrast): in LC-T1 both native channels
move TOGETHER to the seeded value (P1 and P2 both FAIL on line 16 at ratio
≈1.05), whereas the F-1 case had P1 FAIL with P2 PASS (witness-level
divergence). This two-channel coherence is the witness-level vs
transformation-level discriminator.

Library plan: LC-T0 and LC-T2 use the SAME tree-rebuilt stock artifact
(byte-identical), so T0 vs T1 differ by the seed patch alone under one build
configuration; the Debian packaged stock library
(`3a8ce231…`, F-1 lineage) remains the external reference and is NOT swapped
in during formal runs.

## 7. Formal run procedure (frozen)

Per run (order LC-T0-R1..R3, LC-T1-R1..R3, LC-T2-R1..R3):
1. root clearing ritual: `pkill` rtapi_app/milltask/linuxcncrsh (bracket
   trick), `rm -f /dev/shm/* /tmp/rtapi* /tmp/linuxcnc.lock`, sleep 3
2. libctl content set to the condition library; sha256 recorded to
   `libsha_<run>.txt`
3. `oneshot_wpb.sh <run>` as l15: boot linuxcncrsh headless, io-ready gate,
   **/proc/<milltask>/maps hard gate** (abort unless libctl lib loaded),
   `l15t_drive.py <run> g93t_probe.ngc` (frozen timeline), then the frozen
   adapter produces `l15_report_<run>.json`
4. artifacts per run: capture CSV, report JSON+MD, drive log, libsha
5. abort/void rule: any boot failure, maps-gate failure, homing abort, or
   non-IDLE completion voids the run; the run is repeated and both records kept

## 8. Run record fields (frozen)

Per run: source file hashes (interp_inverse.cc, interp_convert.cc), library
sha256 (libctl artifact + loaded-image check result), NC hash, INI/HAL hash,
adapter/drive/runner hashes, capture row count, per-block L/F/expected/observed
fu_med/ratio/verdict, P1–P6 profile, holds, H1, run completion status.
Aggregation: `wpb_verdict.py` per condition (qualification, semantic, H-B
pattern confirmation, determinism).

## 9. Dry-run record (pre-freeze, exploratory; all captures/reports kept in dry/)

Pre-freeze corrections (allowed; none touches δ, seed location, target block,
tolerances, checker, or oracle):

1. **NC line-layout correction.** The first NC draft carried too many header
   comment lines, placing the motion blocks at source lines 19–24 (target B4
   at line 22; an intermediate trim reached 14–19/B4 at 17). The compiled seed
   guard (`sequence_number == 16`) would therefore have been inert. The header
   was trimmed to exactly 10 comment lines, making the line layout identical
   to g93_probe.ngc (blocks at 13–18, **B4 at 16**), after which dry-runs were
   repeated. Recorded as a pre-freeze input-authoring correction, NOT an
   amendment.
2. **Clearing-ritual hardening.** Two dry boots (DryT0b/DryT0c) wedged
   (machine-on no ACK, program RUNNING with pline=0, sampler overrun) — the
   documented L1.5 non-first-boot iocontrol wedge, here caused by NML/HAL SysV
   shm segments surviving `pkill -9` (which bypasses the wrapper's graceful
   `ipcrm` cleanup). Remedy: `wpb_clear.sh` (kill set incl. wrapper bash, then
   purge every SysV segment, /dev/shm, /tmp residue; verify procs=0 ∧ shm=0 ∧
   lock=0 before boot). Both wedged runs are VOIDED as environment failures
   and kept in dry/. A cross-distro port conflict was also cleared once
   (a stale 2026-10-01 linuxcncrsh session in Debian-CS held port 5007 across
   the shared WSL2 network namespace).

Valid dry evidence (final NC, sha 5ec0c33d…):

- **Dry-T0d — stock tree lib d3e4ecd4…**: program 12.0 s to IDLE; all six
  blocks fu_med EXACT (240.00 / 300.00 / 600.00 / 600.00 / 424.26 / 848.53);
  P1–P6 all PASS; hold 1.002 s detected; H1 PASS. `/proc/maps` gate OK.
- **Dry-T1 — seeded lib 645579…**: program 11.5 s to IDLE (shortened by
  exactly the seeded block's 0.05 s); line 15 PASS (600.00), **line 16 FAIL —
  fu_med 630.00, P1 ratio 1.0500, rv_med 10.5000, P2 ratio 1.0500 (both
  native channels coherent at the seeded value)**, lines 17–18 PASS;
  P3/P4/P5/P6 all PASS (qualification intact); no abort, trace valid,
  windows complete. δ=0.05 confirmed executable with no side effects.

Dry conclusion: seed design valid; expected pattern of §6 confirmed on both
conditions; freezing for formal runs.

## 10. FREEZE record

**FROZEN at 2026-10-05 04:35 +08:00, before any formal run.**

Frozen identities (sha256):

| component | sha256 |
|---|---|
| NC input g93t_probe.ngc | 5ec0c33d41e5c2a2d3cbb5ef1fffcd511896272a21cf2879a46ee25304091d5e |
| checker adapter_l15.py (unchanged, = L1.5/F-1 byte-identical) | eb6a669039403126a3c2b770b9c4e62ea50336c6224cd0fb3dd5cf7827125905 |
| verdict aggregator wpb_verdict.py | aa5e638956a63e0af382220995707a18bbdf40c2022c235bbd88e36f9d99ac43 |
| drive l15t_drive.py | b805d2965ce3b645c2d6ec2970b2da5a407c07546e0562bfbd24f5336a099740 |
| runner oneshot_wpb.sh | 43266b21b9f10ca0834c705ce59fad7f847c6ca4970548a58f1d06a351e6efe8 |
| clearing ritual wpb_clear.sh | 9b10b8584414cc83ad16610cc9aec832871cc6b4d35b3f401b7127a7278f0451 |
| seed patcher apply_wpb_seed.py | c78e2c8636c1cfac7983fb0c506f9372f53f416695041f4af03e1971bb845658 |
| interp_inverse.cc stock (= orig.tar.gz member) | c98db3d43f28955090528651a0cd7c0b422c4d0e79de03f425ba3ecc72b2f9f0 |
| interp_inverse.cc seeded | b22bdaff0c66c172888e61effb6b646bfa307f956464d9472ed50217c66e9ddc |
| interp_convert.cc (all conditions, pristine stock) | f2e7cf41ec6bc4b1cd86bf6a5620c7d6028314f66a2980491ffab19156a506cb |
| **librs274.so.0 stock-tree (LC-T0 & LC-T2)** | d3e4ecd46061f07859325238bd7a8588e426453c573e606eee9767056111dc52 |
| **librs274.so.0 seeded (LC-T1)** | 645579905c85317af91e50a11a15388d3e5de9789545b0a343495eadffd2ffc2 |
| patches/stock_to_seed.patch | (recorded in hashes/SHA256SUMS with all run artifacts) |

Build reproducibility note: rebuilding the stock library from the restored
stock source reproduced d3e4ecd4… byte-identically, so LC-T0 and LC-T2 share
one artifact and differ from LC-T1 only by the seed patch under one build
configuration.

Expected formal outcome (frozen; taskbook §9.4):

| Condition | Qualification | Semantic P1 | Verdict |
|---|---|---|---|
| LC-T0 ×3 | PASS | PASS | PASS |
| LC-T1 ×3 | PASS | FAIL (line 16 only, ratio≈1.05) | **FAIL** |
| LC-T2 ×3 | PASS | PASS | PASS |

Formal run order: LC-T0-R1, LC-T0-R2, LC-T0-R3, LC-T1-R1, LC-T1-R2, LC-T1-R3,
LC-T2-R1, LC-T2-R2, LC-T2-R3 — each preceded by wpb_clear.sh (must print
CLEAR_OK), libctl set to the condition artifact, oneshot_wpb.sh with /proc/maps
gate, verdict aggregation by the frozen wpb_verdict.py. No deviation allowed;
failures are recorded and handled per §7.5 abort rule.

## 11. AMENDMENT 01 (2026-10-05, post-run, pre-publication)

The frozen verdict aggregator `wpb_verdict.py` (sha256 aa5e6389…) contained a
syntax defect (a stray `]` in the final summary print) discovered at its first
execution, after the formal runs and before it had ever produced output. Fix =
one-character deletion in a print statement; no change to any qualification /
semantic / H-B-pattern computation, inputs consumed, or thresholds. Corrected
sha256: 7b6e6a533c6ecfb04cc76479e056e990e5942f53482700d380540eafd0be3c62.
The per-run checker (adapter_l15.py, eb6a6690…) that produced every report was
never touched. All 9 formal runs and their verdicts stand as produced under the
frozen design; only the aggregation print statement was repaired.
