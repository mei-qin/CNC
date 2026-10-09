# F0 Baseline Fairness — Comparator Workflows and Matched Access (Read-only)

**Package:** `linuxcnc_f0_interaction` · **Date:** 2026-10-09
**Authority:** static feasibility documentation only; no workflow is executed or scored in F0. Expected classifications below are *predictions for protocol review*, derived from frozen historical signatures and source reading; the two T1 columns have no historical runs.
**Taskbook anchor:** MICRO_STUDY_F0_FEASIBILITY_TASKBOOK G5; S4-B REFRAMING_GATE_DECISION §4 ("The evaluated workflows must receive matched raw native access").

---

## 1. Matched raw access (common to every workflow)

Every workflow listed below — including the conventional baselines — receives the **same** raw native evidence:

- the complete 10-channel 1 kHz sampler capture (positions, current-vel, requested-vel, feed-upm, distance-to-go, motion-type, program-line, feed-override);
- the probe NC program (`g93_probe.ngc`, sha-frozen) and INI/HAL configuration (sha-frozen);
- the per-run identity receipts (libsha, `/proc/<milltask>/maps` snapshot) — i.e., provenance metadata is available to all workflows on equal terms;
- identical block-window construction (`adapter_l15.py::windows()`), identical frozen tolerances.

No workflow gets privileged channel access. What differs is **which checks each workflow is configured to perform** and **whether it is allowed to consult source-level binding evidence** — that is the treatment under study, not an access asymmetry.

## 2. Workflow definitions

| ID | Workflow | Configuration | Source certification |
|---|---|---|---|
| **O** | Frozen selected-output oracle | Judges only the selected output record (joint position trajectories / final selected trace), as in the B3 C0/C2 observational-equivalence design | none |
| **I** | Internal-diagnostic stage predicate, uncertified | Evaluates P1-type stage predicate(s) on native internal diagnostics (feed-upm witness vs NC oracle; optionally P2 requested-vel as a second internal channel) with frozen tolerances; treats admitted diagnostic fields as semantically valid | **none** — trusts declared field semantics |
| **Q** | Independent source audit | Static review of the evidence-production binding: tag write sites vs conversion sites (`interp_convert.cc:479/:602/:689/:847`), `write_state_tag` field capture, `control.c:1872` pin binding; certifies/denies witness stage correctness and reads the T diff. Makes no runtime stage-predicate claim | is the source review |
| **I+Q** | Joint responsibility + source-adjudication | I's runtime predicates **gated by** Q's binding certification: a witness whose production stage is certified wrong is reported UNTRUSTED and its predicate outcomes are not used for cause attribution; transformation status is then attributed only from certified-admissible evidence (motion-side channel vs oracle), with ABSTAIN when no admissible witness remains | yes, as gating input |
| **FULL-B** | Fully capable conventional contract/provenance monitor | A traditional runtime-monitor workflow given the same raw access and configured, without the paper's responsibility-first framing, to (a) check native signals against NC-derived contracts, (b) cross-check redundant signal pairs for consistency, and (c) consume provenance/build metadata. Represents "adequately configured conventional monitoring" | provenance consumption, but not the paper's stage-responsibility adjudication procedure |

**Parity stance (per gate §4):** if FULL-B's dispositions equal I+Q's on all cells, the outcome is reported as equivalence; the contribution is then positioned as a case-grounded auditing methodology, not a superiority claim. F0 takes no position on which will occur.

## 3. Expected classification table (predictions; H = historically anchored)

| Cell | O | I (P1 only) | I (P1+P2) | Q (static) | I+Q (joint) | FULL-B |
|---|---|---|---|---|---|---|
| T0W0 | no violation detectable in selected output | PASS | PASS | binding certified; no T diff | **PASS (qualified)** | expected PASS |
| T0W1 | no violation detectable (motion unchanged — H: G4 invariance) | **FAIL** (fu 600.00, ratio 0.3183 — H) | FAIL + PASS dissociation (H: F-1) | binding **denied** (tag precedes conversion) | **witness UNTRUSTED; transformation OK** (rv==oracle) | either impeaches fu via cross-check or reports conflict |
| T1W0 | timing shift 1/1.05 on one block may or may not exceed selected-output granularity (cf. B3 masked-output result) | FAIL (fu 1979.20, ratio 1.0500) | FAIL + FAIL coherent at 1.05 (WP-B analogue) | T diff found; binding certified | **transformation VIOLATED** (witness certified, deviates at seeded value) | expected FAIL with cause from redundancy check |
| T1W1 | as T1W0 | **FAIL (fu 600.00 — identical signature to T0W1)** | FAIL(stale)+FAIL(1.05) mixed signature | T diff found; binding denied | **witness UNTRUSTED; transformation VIOLATED via motion-side channel** | as T0W1, plus must decide which channel to trust |

## 4. Indistinguishability and false-attribution analysis

1. **O** cannot separate any cell pair whose selected outputs match within frozen granularity; in particular the W axis is invisible to O by construction (W changes only which value a metadata field carries). Bounded instance of the B3/P1 result; no new theorem claimed.
2. **I (P1 only)** produces *byte-identical* fu signatures in T0W1 and T1W1 (600.00 FAIL). Any cause label it attaches ("conversion violated") is correct in T1W1 only by luck and wrong in T0W1 — a false cause attribution in ≥1 of the two cells, and it cannot know which. This is the primary measurable mechanism cell.
3. **I (P1+P2)** gains a dissociation signature but, lacking source certification, faces two conflicting native channels with no arbiter for *which channel is semantically valid*: in T1W1 it observes fu-stale + rv-1.05 and must either abstain or pick a channel by convention. Reporting rules for this workflow must be fixed in F1 *before* runs (pick-first-channel / pick-oracle-matching-channel / abstain — the choice changes its confusion table and must not be tuned post hoc).
4. **Q alone** certifies binding correctness and reads the T diff, but without runtime evidence cannot confirm that the seeded path actually executed (guard activation) — hence "no claim that Q alone checks stage predicates".
5. **I+Q** resolves all four cells with explicit causes, at the cost of one declared abstention class: when the only available witness is certified wrong-stage and no independent admissible channel exists, transformation status is reported NOT-ATTRIBUTABLE-FROM-AVAILABLE-WITNESS. In this design the motion-side channel exists, so the abstention is exercised only in the limit; the rule is stated so the procedure does not silently manufacture certainty elsewhere.
6. **FULL-B** may replicate I+Q. The study's value claim is then organizational/audit-methodological (explicit stage-responsibility gating, countable abstentions, source-grounded cause labels), reported as parity — not accuracy superiority.

## 5. Counting rules (to be frozen in F1, restated here for review)

- Unit of analysis = **cell** (4). Technical replicates (3/cell) verify signature determinism only.
- Primary observable = count of independently supported **correct cause dispositions** among 4 cells, per workflow.
- **ABSTAIN counted separately** from false attribution and from correct disposition (three-way outcome per cell×workflow).
- Truth for scoring = pre-declared per-cell state from source diffs + NC oracle + (historically anchored) motion-side channel behavior — never the verdict of the workflow being scored, and never the I+Q verdict itself (no outcome defined by the presence of the treatment).
- Negative/null results recorded and reported; no post-hoc denominator mixing with B3/LC-D1/WP-B archives.
