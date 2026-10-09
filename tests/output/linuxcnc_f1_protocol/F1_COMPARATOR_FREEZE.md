# F1 Comparator Freeze — Fair Access, Verdict Rules, Scoring (Frozen Proposal)

**Package:** `linuxcnc_f1_protocol` · **Status:** prospective protocol — NO EXPERIMENT AUTHORIZED
**Taskbook:** F1_PROTOCOL_PREPARATION_TASKBOOK Work 3 (critical item).
**Responds to:** F0_INDEPENDENT_GATE_DECISION §3.1/§3.2 (fair access vs disabled capability; source inspection is supplied information).

---

## 1. Single shared input packet (identical for all six workflows)

Every workflow receives byte-identical inputs:

1. the 10-channel 1 kHz capture CSV (layout per F1_CAPTURE_AND_WINDOW_SPEC);
2. `g93_probe.ngc` / `.ini` / `.hal` / `.var` / `.tbl` (sha-frozen);
3. **pristine source snapshots** (`interp_inverse.cc` c98db3d4…, `interp_convert.cc` f2e7cf41…) **and all variant sources + unified diffs + hashes** (`patches/`, `variants/`, incl. the T seed and the W tag diff);
4. per-run provenance: run ID, libctl libsha, `/proc/<milltask>/maps` snapshot, build command echo;
5. block-selection metadata (window ordinal bounds per NC line).

**No workflow is denied information another has.** Workflows differ *only* in their configured checks and judgment policy — that configuration difference is precisely the treatment under study. In particular: FULL-B receives the same source diffs and provenance as I+Q and Q, and is expected to act on them; Q is explicitly **not** a no-information baseline (with a confirmed exercised guard, the diffs nearly reveal the engineered condition — this is declared, not hidden).

## 2. Workflow verdict rules (frozen pseudocode)

Common notation: arc window W₁₆ (pline==16); oracle `fu* = L·F`, `rv* = fu*/60`; tolerance τ = 2%; row time ts = 1 ms; `D_path = Σ|Δpos|` over W₁₆ (polygonal executed path length ≈ L).

### 2.1 O — selected-output oracle (F1.1: demoted to **non-scored context workflow**)
```
input used: j0..j2 columns only
time base:  data-row ordinal × 1 ms (deterministic; no phase alignment to events)
reference:  NC-nominal positions, constant-velocity-per-block model
            (accel ramps uncompensated — covered by the coarse tolerance)
verdict:    per-axis max |pos(t) − pos_nominal(t)| ≤ 2% of axis travel ?
            → {OUTPUT_CONFORMANT | OUTPUT_DEVIANT(timing/trajectory)}
cause vocabulary: trajectory/timing only. No witness, stage, or metadata claim possible.
```
O is deterministic but coarse; it appears as a context row and is **excluded from the scored table** (F1.1 Work D.1). Predicted: T0 cells conformant; T1 cells deviant (arc timing 0.500→0.476 s shifts downstream time alignment). **Banned claim:** "T1 is masked from the output because W changes only metadata" — output masking is not asserted; O's limit is that it can never adjudicate witness validity or stage responsibility.

### 2.2 I(P1-only) — restricted-signal ablation (explanatory; NEVER a primary comparator)
```
input used: feed_upm column only (plus oracle)
verdict:    any block |fu_med/(L·F) − 1| > τ → "conversion violation (naive attribution)"
```
Role: demonstrates the omitted-information failure mode (identical 600.00 signatures in T0W1/T1W1). Reported in an ablation column only; any headline use is a protocol violation.

### 2.3 I(P1+P2) — dual-channel predicate with v2 arbitration (PRIMARY comparator; F1.1 amended)

> **v1 retracted (F1.1):** the original whole-window integration test (`Σ channel·ts` vs path length) is category-invalid for command-level channels — `requested-vel`/`feed-upm` hold the segment's commanded target across the ≈35 ms exact-stop tail inside the 535-row window, producing a uniform +6.85% bias that fails normal historical runs (independently confirmed on 13 captures; F1_1_AMENDMENT §1). The replacement below uses like-for-like comparisons only.

```
R1 kinematic integrity (realized channels only):
    | Σ(cur_vel_k · ts) / D_path − 1 | ≤ τ,   D_path = Σ‖Δ(j0,j1,j2)‖ over W₁₆
R2 command agreement (witness level, medians over normal rows ovr≥0.99 ∧ fu>1e-9):
    | (fu_med/60) / rv_med − 1 | ≤ τ
R3 T-indication (command channel vs NC oracle):
    | rv_med / (L·F/60) − 1 | ≤ τ

procedure:  R1 fail → run integrity VOID (not a verdict)
            witness impeached ⇔ ¬R2                 (behavioral, cause-free)
            T-state = R3                             (OK | VIOLATED)
W-cause: NOT CERTIFIABLE (no source consultation configured)
```
Source-semantic grounding: `requested-vel` = planner segment nominal command (`tp.c:2629`, `control.c:1944`); `current-vel` = realized segment velocity (`tp.c:2630`); `motor-pos-cmd` = per-cycle realized position command (`control.c:2011–2016`); medians are robust to window overhang (historically exact: 1884.955566 / 600.00 / 31.415927). Retrospective validation on 13 historical captures (receipts/F1_1_RETROSPECTIVE_RECEIPT.json, NON-SCORED): R1 0.0150% PASS ×13; R2 separates W0 (PASS ×3) from W1 (FAIL ×10) exactly; R3 PASS ×13. I(P1+P2) thus has **full behavioral attribution capability without any source access**; its structural limit remains: it cannot say *why* a channel is off (stage misbinding vs any other cause).

### 2.4 Q — source-only audit (with activation qualification)
```
input used: diffs, snapshots, provenance, NC (no capture required except activation check)
verdict:    W_cfg = (tag append present after converter at all 3 arc routes ? W0 : W1)
            T_cfg = (guarded 1.05 seed present in inverse_time_rate_arc ? T1 : T0)
            activation: UNQUALIFIED unless runtime motion-side signature supplied
            (in this study activation evidence = requested-vel ratio, which is
             independent of the audited witness; using it is declared, and Q
             must NOT consume any O/I/I+Q/FULL-B verdict)
```
Q reports the *configured* state and binding certification; execution claims require the activation qualification. It never reads comparator outputs.

### 2.5 I+Q — source-certified runtime causality assignment (method under study)
```
gate:      if Q certifies W1 (tag provably written pre-conversion):
               fu declared UNTRUSTED (excluded from T attribution)
           else fu ADMISSIBLE
T-state:   from admissible channels: rv ratio (always admissible — motion-side,
           not tag-derived); if fu admissible, corroborate
verdict:   joint (T, W) + cause citation (source line anchors for W;
           source diff + motion-side ratio for T)
abstain:   if no admissible witness remained for some dimension → ABSTAIN(dim)
```

### 2.6 FULL-B — adequately configured conventional contract/provenance monitor (PRIMARY comparator; deterministic tree frozen in F1.1)
```
FULL-B(capture, NC, oracle, diffs, provenance):
  0. integrity: non-data-lines==1 ∧ parse_fail==0 ∧ overruns==0 ∧ P4 1:1
       else → INTEGRITY_FAIL (not scored as a verdict)
  1. R1 kinematic check: fail → flag CAPTURE_INCONSISTENT → ABSTAIN(both)
  2. provenance: libsha/maps vs manifest; mismatch → flag
       LOADED-CODE-DEVIATES (may name variant from supplied diffs) → ABSTAIN(both)
  3. W (source review of supplied diffs):
       tag-after-converter at 3 arc routes → W=correct-stage
       stock :479 order                     → W=wrong-stage
  4. T: R2 pass → witness coherent, T from R3
       R2 fail → witness UNTRUSTED-as-rate (behavioral), T from R3
  5. required verdict schema (fixed):
       {T_state, W_state, W_evidence: source-cited|behavioral,
        T_evidence: oracle-ratio|source-diff|both, flags[], abstentions[]}
```
Q and I+Q (§2.4/§2.5) use the same required verdict schema; every workflow's classification is reproducible from these trees without author discretion. FULL-B is *expected* to match I+Q. **If it does, the result is reported as equivalence** (parity), and the paper's claim is the auditing procedure + bounded causal case knowledge — not detection superiority. No post-hoc weakening of FULL-B to manufacture a gap is permitted; any such temptation is a STOP condition.

## 3. Scoring freeze (before any run; precedence fixed in F1.1)

Per dimension d ∈ {T, W}: label_d ∈ {matches gold, contradicts gold, abstain}; anchored_d = the decisive evidence channel discriminates the cells the verdict claims to separate (pre-declared equivalence: `feed-upm` is non-discriminating between T0W1 and T1W1).

**Priority (first match wins):**

1. any dimension label contradicts gold → **FALSE_ATTRIBUTION** (regardless of anchoring)
2. all decided labels match gold, any decisive evidence unanchored → **UNANCHORED**
3. both dimensions match, anchored → **CORRECT_JOINT**
4. no contradiction, ≥1 abstain ∧ ≥1 match → **PARTIAL**
5. all abstain → **ABSTAIN**

Integrity failures (R1, provenance, parse) are VOID — never scored as outcomes. An "unsupported correct label" (matches gold with weaker-but-anchored evidence, e.g., behavioral-only W impeachment) is CORRECT_JOINT with `W_evidence: behavioral`; the cause-support axis stays descriptive-secondary and is never folded into the primary count.

**Pre-declared illustrations (expectations only, not run-scored):**
- I(P1-only) @T0W1: T label "violated" contradicts gold → FALSE_ATTRIBUTION
- I(P1-only) @T1W1: label matches, fu non-discriminating in W1 → UNANCHORED
- I(P1+P2) @T1W1: T correct (R3, anchored), W abstain → PARTIAL
- FULL-B @T1W1: T + W correct, anchored → CORRECT_JOINT
- R1 failure anywhere → VOID (integrity)

Primary comparative result: workflows × cells table of these outcomes (I(P1+P2), Q, I+Q, FULL-B as primary rows; O context row, non-scored; I(P1-only) ablation column). False causal assignments, abstentions, and coverage each enumerated explicitly.

**Anti-rigging rules (frozen):**
1. I(P1-only) never appears in headline comparisons.
2. If I+Q's advantage over FULL-B exists only where FULL-B was denied information or checks, that is a protocol failure → STOP, not a result.
3. Parity is an acceptable, pre-declared outcome; the study's measurable content is the bounded causal composition + attribution/abstention accounting, not detection superiority (F1_1_AMENDMENT §3.3).
4. No scoring rule may be edited after the first prospective run; corrections require re-review and re-freeze.

## 4. What this design can and cannot show

- Can: a bounded, controlled, same-unit demonstration of how stage-responsibility checking and native-witness source auditing compose; an explicit measurement of which workflow configurations attribute causes correctly, abstain, or err; a clean equivalence/inequality statement versus a fully equipped conventional monitor.
- Cannot: natural defect rates, population inference, N=12 independence, uniqueness or superiority theorems, G94 generality.
