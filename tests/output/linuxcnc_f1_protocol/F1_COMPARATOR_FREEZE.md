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

### 2.1 O — selected-output oracle
```
input used: j0..j2 columns only
reference:  NC-nominal time-parameterized positions (block times from NC)
verdict:    per-axis max |pos(t) − pos_nominal(t)| ≤ 2% of axis travel ?
            → {OUTPUT_CONFORMANT | OUTPUT_DEVIANT(timing/trajectory)}
cause vocabulary: trajectory/timing only. No witness, stage, or metadata claim possible.
```
Predicted: T0 cells conformant; T1 cells deviant (arc timing 0.500→0.476 s shifts downstream time alignment). **Banned claim:** "T1 is masked from the output because W changes only metadata" — output masking is not asserted; O's limit is that it can never adjudicate witness validity or stage responsibility.

### 2.2 I(P1-only) — restricted-signal ablation (explanatory; NEVER a primary comparator)
```
input used: feed_upm column only (plus oracle)
verdict:    any block |fu_med/(L·F) − 1| > τ → "conversion violation (naive attribution)"
```
Role: demonstrates the omitted-information failure mode (identical 600.00 signatures in T0W1/T1W1). Reported in an ablation column only; any headline use is a protocol violation.

### 2.3 I(P1+P2) — dual-channel predicate with cross-motion arbitration (PRIMARY comparator)
```
for channel c in {fu := feed_upm/60·ovr, rv := req_vel}:
    implied_path(c)  = Σ_k c_k · ts          (over W₁₆)
    consistent(c)    = |implied_path(c)/D_path − 1| ≤ τ
    ratio(c)         = median(c)_normal / oracle_c
T-state:
    if consistent(rv):  T = (|ratio(rv) − 1| ≤ τ ? OK : VIOLATED)
    elif consistent(fu): T = (|ratio(fu) − 1| ≤ τ ? OK : VIOLATED)
    else: T = ABSTAIN
witness report (behavioral, cause-free):
    fu impeached iff ¬consistent(fu); rv impeached iff ¬consistent(rv)
W-cause: NOT CERTIFIABLE (no source consultation configured)
```
Predicted: fu impeached exactly in W1 cells (implied 5.0 mm vs D_path ≈ 15.708 mm); rv consistent in all cells; T-state correct in all four cells. I(P1+P2) thus has **full behavioral attribution capability without any source access** — it is the fair conventional-monitor representative for detection. Its structural limit: it cannot say *why* a channel is off (stage misbinding vs any other cause).

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

### 2.6 FULL-B — adequately configured conventional contract/provenance monitor (PRIMARY comparator)
```
configured with: everything I(P1+P2) has (2.3 arbitration)
              +  provenance checks (libsha/maps vs manifest; flags loaded-code
                 deviation and may name it from the supplied diffs)
              +  source review acting on the same diffs (may cite stage order)
verdict:       joint (T, W) + causes, at least as expressive as I+Q
```
FULL-B is *expected* to match I+Q. **If it does, the result is reported as equivalence** (parity), and the paper's claim is the auditing procedure + bounded causal case knowledge — not detection superiority. No post-hoc weakening of FULL-B to manufacture a gap is permitted; any such temptation is a STOP condition.

## 3. Scoring freeze (before any run)

Per cell × workflow, one five-way outcome:

| Outcome | Definition |
|---|---|
| CORRECT_JOINT | both T and W dimensions match gold, with evidence the procedure can discriminate |
| PARTIAL | exactly one dimension matches (the other wrong) |
| FALSE_ATTRIBUTION | a cause is named that contradicts gold (e.g., "conversion violation" in T0W1; "witness error" in T1W0) |
| ABSTAIN | workflow declines a dimension (counted, never merged with wrong) |
| UNANCHORED | verdict label coincidentally matches gold but rests on a channel that takes the **same value in both T arms of that W level** (non-discriminating evidence) — e.g., any fu-only T-verdict in W1 cells. Counted separately; **not** counted as CORRECT |

Primary comparative result: the workflows × cells table of these outcomes (I(P1+P2), Q, I+Q, FULL-B as primary rows; O context row; I(P1-only) ablation column). Secondary descriptive axis: cause-support level (source-cited / behavioral-only / none). False causal assignments, abstentions, and coverage are each enumerated explicitly.

**Anti-rigging rules (frozen):**
1. I(P1-only) never appears in headline comparisons.
2. If I+Q's advantage over FULL-B exists only where FULL-B was denied information or checks, that is a protocol failure → STOP, not a result.
3. Parity is an acceptable, pre-declared outcome.
4. No scoring rule may be edited after the first prospective run; corrections require re-review and re-freeze.

## 4. What this design can and cannot show

- Can: a bounded, controlled, same-unit demonstration of how stage-responsibility checking and native-witness source auditing compose; an explicit measurement of which workflow configurations attribute causes correctly, abstain, or err; a clean equivalence/inequality statement versus a fully equipped conventional monitor.
- Cannot: natural defect rates, population inference, N=12 independence, uniqueness or superiority theorems, G94 generality.
