# F1.1 Amendment — Bounded Protocol Correction (NON-SCORED, PRE-EXECUTION)

**Package:** `linuxcnc_f1_protocol` · **Date:** 2026-10-10
**Authority:** F1_1_LIMITED_REWORK_TASKBOOK (via `F1_Independent_Protocol_Review_20261010.zip`, sha256 `8ff078cfba1989c1561a2a971af4c92e5eb90d3509ccd6ad90fe1bfa6aa2852c`). Preparation only — no build, no dry-run, no scored runs. Original `F1_PROTOCOL_PACKAGE_20261010.zip` preserved unmodified.

## 0. Disclosure (required by taskbook Work A.1)

**This amendment is informed by recomputation of existing historical captures and is therefore NOT a pristine pre-registration.** The v1 comparator premise was prospectively falsified before any F1 run by the independent reviewer (HISTORICAL_ARBITRATION_RECEIPT.json: T0W0/T0W1 535-row baselines show `Σreq_vel·dt / Σ‖ΔXYZ‖ ≈ 1.0685`, far outside the ±2% admission rule) and independently reconfirmed by this project on **13** historical captures (§2.3). All changes below are formal protocol corrections made *before* F1 execution; no F1 outcome exists; no threshold was widened to absorb the discrepancy (the failing test itself was replaced on source-semantic grounds).

## 1. Work A — numerical arbitration failure: diagnosis and v2 rules

### 1.1 Source-semantic grounding (read-verified in the subject tree; no claim from signal names)

| Channel | Producer (source anchor) | Semantics |
|---|---|---|
| `motion.requested-vel` | `control.c:1944` publishes `emcmotStatus->requested_vel`; set at `src/emc/tp/tp.c:2629` `emcmotStatus->requested_vel = tc->reqvel;` | **nominal command velocity of the active trajectory-planner segment** — held at the segment's target feed, independent of accel/decel ramps or completed motion |
| `motion.current-vel` | `control.c:1943` publishes `emcmotStatus->current_vel`; set at `tp.c:2630` `emcmotStatus->current_vel = tc->currentvel;` (blend: `tp.c:3064`) | **realized instantaneous velocity** of the executing segment (after ramps) |
| `joint.N.motor-pos-cmd` | `control.c:2011–2016` writes `joint->motor_pos_cmd` to the HAL pin every cycle | realized per-cycle position command — the legitimate source of executed path distance |
| `motion.feed-upm` | `control.c:1872` = executing motion's state-tag `GM_FIELD_FLOAT_FEED` | the **audited tag witness** (diagnostic, command-level claim) |

**Diagnosis of the v1 failure:** the arc analysis window spans 535 ms while the ideal feed time is 500 ms (exact-stop decel tail ≈ 35 ms inside the window). Command-level channels (`requested-vel`, `feed-upm`) hold their target value across the whole window, so their time-integral exceeds the executed path by ≈ window_overhang/feed_time = 35/510 ≈ 6.9% — exactly the observed 6.8496%. Integrating a command datum against realized path length was a category error, not a tolerance problem.

### 1.2 v2 arbitration rules (replacing F1_COMPARATOR_FREEZE §2.3; frozen)

```
R1 kinematic integrity (capture-level, uses realized channels only):
   | Σ(cur_vel_k·ts) / D_path − 1| ≤ 2%,  D_path = Σ‖Δ(j0,j1,j2)‖ over W₁₆
   → validates that positions and realized velocity are mutually consistent
     (anti-tampering / capture-coherence precondition; not a witness verdict)

R2 command agreement (witness-level, medians over normal rows: ovr ≥ 0.99 ∧ fu > 1e-9):
   | (fu_med/60) / rv_med − 1 | ≤ 2%
   → the tag witness's claimed rate must agree with the planner's commanded rate.
     FAIL = witness impeached (behavioral; no cause claim). Medians are robust
     to window overhang: historically fu_med=1884.955566 (W0) / 600.00 (W1),
     rv_med=31.415927 exactly.

R3 T-indication (command level vs NC oracle):
   | rv_med / (L·F/60) − 1 | ≤ 2%  → T indicated OK; else violated.
```

Arbitration logic of I(P1+P2): R1 must pass (else run integrity VOID); witness impeachment = ¬R2; T-state from R3 (command channel vs NC geometry — no integration involved).

### 1.3 Retrospective receipt (NON-SCORED; all accessible historical captures)

`receipts/F1_1_RETROSPECTIVE_RECEIPT.json`, computed by `tools/f11_retrospective_check.py` (read-only script) over **13** captures: g1r1–3, g1s1–3, g5s1 (2.9.4 stock/revert), p1s1–3 (2.9.4 W0-patched), cs1/cs3/cs5 (2.9.10 stock). Summary:

| Metric | W0 runs (p1s1-3) | W1 runs (g1r/g1s/g5s/cs ×10) |
|---|---|---|
| R1 error | 0.0150% PASS ×3 | 0.0150% PASS ×10 |
| v1-style rv integral error (documented failure) | 6.8496% | 6.8496% |
| R2 command agreement | **PASS ×3** (1884.955566/60 = 31.415926 vs 31.415927) | **FAIL ×10** (600/60 = 10 vs 31.415927) |
| R3 T-indication | PASS ×3 | PASS ×10 |
| t_exec98 (position-derived, §2) | 0.5090 s ×3 | 0.5090 s ×10 |

The v2 rules separate the W axis perfectly on historical data, pass kinematic integrity on both versions (2.9.4/2.9.10), and reproduce the reviewer's failure numbers bit-for-bit. These are **retrospective validations of rule behavior, not F1 outcomes**; T1 behavior remains prospective-only prediction.

## 2. Work B — de-coupled truth labels (amends F1_GOLD_STANDARD)

### 2.1 Fourfold state definition

- **T_config** — seeded code in the loaded library (source diff + lib sha + maps): configured fact.
- **T_executed** — the arc conversion actually ran seeded at runtime: established by **position-derived execution time** `t_exec98` = first window row where cumulative ‖Δpos‖ ≥ 0.98·L, and by **window row count** (segmentation pins only). Predictions: T0 ⇒ t_exec98 ≈ 0.5090 s (historically uniform ×13), 535 rows; T1 ⇒ t_exec98 ≈ 0.4848 s, ≈511 rows (1/1.05 scaling; ramp-shape caveat absorbed by the ±2% band). **Neither statistic uses `requested-vel`, `feed-upm`, or `current-vel`.**
- **W_config** — tag ordering in the loaded library (source diff + provenance): configured fact.
- **W_effective** — which tag actually governed the arc motion. **Declared limitation:** W_effective is observable only through the audited witness itself (the tag's feed field); no independent channel exists. The protocol therefore adjudicates W truth at configuration level and treats runtime fu signatures as *predictions*, never as W truth.

### 2.2 Channel-usage map (circularity prohibition)

| Channel | Gold (truth) | I(P1+P2) | Q | I+Q | FULL-B | O |
|---|---|---|---|---|---|---|
| positions j0–j2 | **yes** (t_exec98) | yes (R1) | — | yes | yes | **only input** |
| current-vel | no | yes (R1) | — | yes | yes | no |
| requested-vel | **no** | yes (R2/R3) | — | yes | yes | no |
| feed-upm | **no** | yes (R2) | — | yes | yes | no |
| segmentation pins (mtype/pline) | yes (row count) | yes (window) | — | yes | yes | no |
| source diffs / provenance | yes (config) | **no** | yes | yes | yes | no |

Gold-side T_executed and every comparator's T verdict use **disjoint channel sets**; no classifier's successful T verdict can be derived from the same sole raw variable that accepted its truth label. (If a future reviewer requires even position-disjointness, STOP applies per taskbook Work B.3 — no new instrumentation is proposed.)

### 2.3 Contradiction and activation-unconfirmed handling (amends §2 of gold doc)

- Provenance↔runtime contradiction (T_config=T1 but t_exec98 in the T0 band, or vice versa): run = **VOID**, label **ACTIVATION-UNCONFIRMED**, counted and reported with anomaly details. Never relabeled, never silently dropped.
- VOID/ACTIVATION-UNCONFIRMED runs reduce the scored denominator only visibly: the report must show total runs, voids by reason, and scored runs.
- W_config=W0 with fu signature ≡ W1 prediction (or converse): VOID + anomaly report (patch not governing), same visibility rule.

## 3. Work C — operational FULL-B, Q, I+Q and scoring precedence (amends F1_COMPARATOR_FREEZE)

### 3.1 FULL-B deterministic decision tree (frozen)

```
FULL-B(capture, NC, oracle, diffs, provenance):
  0. integrity: non-data-lines==1 ∧ parse_fail==0 ∧ overruns==0 ∧ P4 1:1 windows
       else → INTEGRITY_FAIL (not scored as verdict)
  1. R1 kinematic check (current-vel vs path):
       fail → flag CAPTURE_INCONSISTENT; verdict = ABSTAIN(both dims)
  2. provenance check: libsha/maps vs manifest; if mismatch → flag
       LOADED-CODE-DEVIATES (+ may name the variant from supplied diffs);
       verdict = ABSTAIN(both dims)  [does not guess]
  3. W determination (source review, acting on supplied diffs):
       tag-after-converter at 3 arc routes → W=correct-stage
       stock :479 order                    → W=wrong-stage
  4. T determination:
       R2 passes → witness coherent; T from R3 (rv_med vs oracle)
       R2 fails  → witness value UNTRUSTED-as-rate (behaviorally impeached);
                   T from R3 (command channel vs oracle)
  5. verdict fields (required, fixed schema):
       {T_state, W_state, W_evidence: source-cited|behavioral,
        T_evidence: oracle-ratio|source-diff|both, flags[], abstentions[]}
```
Q and I+Q keep their §2.4/§2.5 structures with the same required verdict schema; all three are reproducible from the tree above without author discretion. Input access remains identical across workflows (unchanged §1 of comparator freeze).

### 3.2 Outcome precedence and mixed cases (frozen)

Per dimension d ∈ {T, W}: label_d ∈ {matches gold, contradicts gold, abstain}; anchored_d = decisive evidence channel discriminates the cells it claims to separate (pre-declared equivalence: fu is non-discriminating between T0W1/T1W1).

Priority (first match wins):
1. any dimension label contradicts gold → **FALSE_ATTRIBUTION** (regardless of anchoring)
2. all decided labels match gold but any decisive evidence unanchored → **UNANCHORED**
3. both dimensions match, anchored → **CORRECT_JOINT**
4. no contradiction, ≥1 abstain, ≥1 match → **PARTIAL**
5. all abstain → **ABSTAIN**

Illustrations (pre-declared, not run-scored):
- I(P1-only) @T0W1: labels T=violated (contradicts) → FALSE_ATTRIBUTION.
- I(P1-only) @T1W1: labels T=violated (matches) but fu non-discriminating in W1 → UNANCHORED.
- I(P1+P2) @T1W1: T=violated (R3, anchored), W abstain (no source) → PARTIAL.
- FULL-B @T1W1: T=violated, W=wrong-stage, both anchored → CORRECT_JOINT.
- R1 failure anywhere → integrity VOID (not an outcome code).

"Unsupported correct label" (matches gold, anchored evidence present but weaker, e.g., behavioral-only W impeachment by I(P1+P2) in W0 cells) is recorded as CORRECT_JOINT with `W_evidence: behavioral` — the cause-support axis remains descriptive-secondary, never folded into the primary count.

### 3.3 Honest value statement (taskbook Work C.3)

FULL-B, given identical information and the tree above, is **expected to classify all four cells identically to I+Q** (both reach source-cited W and oracle-anchored T). Q with activation evidence likewise recovers the configured state. The study therefore does **not** measure a detection-accuracy advantage of A+B; its measurable content is (i) the bounded causal demonstration that stage-responsibility and witness-stage factors compose independently on one real controller unit, and (ii) the explicit abstention/attribution accounting that a conventional workflow performs only when configured to (organizational/audit value). Method-specific engineering cost is explicitly **not measured**. If the independent reviewer judges (ii) insufficient as an independently measurable A+B effect, the recommendation is **STOP and return to the S4-4 submission candidate** — accepted here in advance.

## 4. Work D — minor corrections

1. **O demoted to non-scored context workflow** (deterministic but coarse): time base = data-row ordinal × 1 ms; nominal reference = NC block timing with constant-velocity-per-block model; accel ramps uncompensated (covered by the 2%-of-travel tolerance); output selection = j0–j2 only. O appears as a context row, never in the scored table.
2. **Load attestation strengthened:** per-run receipt must include build log echo, pre-run and post-run SHA256 of the exact libctl file, `/proc/<milltask>/maps` snapshot including inode and library path, and the static lib file hash captured *during* execution (read from the mapped file while the process holds it).
3. **Frozen bytes preserved:** NC/HAL/INI/adapter/pristine/variant/patches all unchanged this round (verified by SHA256SUMS); no frozen input was replaced.

## 5. Amendment register

| File | Change |
|---|---|
| F1_1_AMENDMENT.md | new (this document) |
| F1_COMPARATOR_FREEZE.md | §2.3 replaced by v2 rules (R1/R2/R3); §2.6 gains the frozen FULL-B tree; §3 precedence + illustrations; O non-scored note |
| F1_GOLD_STANDARD_AND_EXPECTATIONS.md | §1 truth chain item 4 replaced (no requested-vel in gold); §2 table updated; new §2bis fourfold states + channel map + ACTIVATION-UNCONFIRMED; §4 adds t_exec98 / row-count predictions |
| F1_PROTOCOL_FREEZE.md | amendment register + attestation strengthening (§2) + package listing |
| receipts/F1_1_RETROSPECTIVE_RECEIPT.json | new (13 captures, non-scored) |
| tools/f11_retrospective_check.py | new (read-only analysis script) |
