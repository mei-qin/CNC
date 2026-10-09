# F1 Capture Layout & Window Selection Specification (Frozen Proposal)

**Package:** `linuxcnc_f1_protocol` · **Status:** prospective protocol for independent review — NO EXPERIMENT AUTHORIZED
**Taskbook:** F1_PROTOCOL_PREPARATION_TASKBOOK Work 1 (via `F0_Independent_Protocol_Review_Package.zip`, sha256 `cb8294074e43a514ad3f0b9c2209409ee81cbdf6091e23628df921ebaf2e01f3`)
**Corrects:** F0_SOURCE_BASIS §2 sample-index statement (erratum, see §1).

---

## 1. Erratum — capture has no explicit sample-index column

F0_SOURCE_BASIS §2 stated "column 0 = sample index", following the `g93_probe.hal` header comment describing `halsampler -t`. **This is factually incorrect for the actual captures.** Independent verification (RAW_CAPTURE_LAYOUT_RECEIPT.json, four historical files g1s1/g1s2/g1s3/p1s1) and this session's re-inspection establish:

- Line 1 of each capture is the literal single-token string `overrun` (present exactly once, at position 0, in all four historical files).
- Every subsequent line holds **10 whitespace-separated signal values** — no explicit sample-id column.
- Row counts 11211–11215 data rows; `capture_p1s1.csv` sha256 `b120517c…` matches the F1-CLOSURE-FREEZE-1 manifest.

**Frozen interpretation:** the cycle key is the **0-based data-row ordinal** (k = 0, 1, 2, … after filtering, defined below). All ten values in one data row were latched by `sampler.0` on the same servo-thread pass at 1 kHz — a row is the same-cycle unit. A shared sampler row synchronizes channels **but does not by itself authenticate producer-stage correctness** of any channel (that is the object under study).

## 2. Pin mapping (exact, frozen)

| HAL pin (0-based) | net (g93_probe.hal) | Data column (1-based in row) | adapter COLS name | Type |
|---|---|---|---|---|
| sampler.0.pin.0 | Xpos (joint.0.motor-pos-cmd) | 1 | j0 | float |
| sampler.0.pin.1 | Ypos | 2 | j1 | float |
| sampler.0.pin.2 | Zpos | 3 | j2 | float |
| sampler.0.pin.3 | l15:cv = motion.current-vel | 4 | cur_vel | float |
| sampler.0.pin.4 | l15:rv = motion.requested-vel | 5 | req_vel | float |
| sampler.0.pin.5 | l15:fu = motion.feed-upm | 6 | feed_upm | float |
| sampler.0.pin.6 | l15:dtg = motion.distance-to-go | 7 | dtg | float |
| sampler.0.pin.7 | l15:mt = motion.motion-type | 8 | mtype | s32 |
| sampler.0.pin.8 | l15:pl = motion.program-line | 9 | pline | s32 |
| sampler.0.pin.9 | ovr = halui.feed-override.value | 10 | ovr | float |

## 3. Parse and filtering rules (frozen; superset of frozen adapter behavior)

The runtime checker `adapter_l15.py` (sha256 `eb6a6690…`, byte-frozen, unchanged) already skips any line whose whitespace token count ≠ 10 — which silently drops the `overrun` header. The F1 protocol keeps the adapter untouched and **adds run-level integrity accounting** around it:

1. Count total lines, data lines (10 tokens), and non-data lines per capture; archive with the run.
2. Exactly one non-data line (the `overrun` header at position 0) is expected; **any additional non-data line ⇒ run integrity FAIL ⇒ VOID** (investigate, do not relabel).
3. A 10-token line that fails float parsing ⇒ integrity FAIL ⇒ VOID (the adapter would silently skip it; the F1 accounting catches this).
4. `halcmd show param sampler.0.overruns` must read **0** at export time; record and archive the value. Non-zero ⇒ VOID.
5. Data-row count must be < FIFO depth (65536); expected ≈ 11.2k rows for the 12.0 s nominal program (historical range 11211–11215 for T0; T1 conditions are expected ≈ 24 rows shorter, see §5).
6. Empty capture ⇒ VOID.

## 4. Block/window selection (frozen; identical rule across all four conditions)

Selection algorithm (this is exactly `adapter_l15.py::windows()`, restated normatively):

- **Window** = maximal span of *contiguous* data-row ordinals (k increments by 1) in which `mtype ∈ {2.0, 3.0}` (coordinated feed / arc) and `pline` is constant. An ordinal gap or an `mtype ∉ {2,3}` row closes the window (transient/queue-boundary handling).
- **Arc analysis window** = the window with `pline == 16`. NC blocks map 1:1 to windows (P4 predicate: windows match NC motion lines exactly, monotone, complete); any unmatched or duplicated motion line ⇒ VOID (P4 is a hard gate here, as historically).
- **Per-window normal rows** = rows with `ovr ≥ 0.99` and `feed_upm > 1e-9`; per-window statistics are medians over normal rows (frozen).
- **The same selection rule applies under all conditions.** The protocol explicitly does **NOT** require identical row counts, identical row bounds, or identical physical durations across conditions: T1 intentionally shortens the arc block from 0.500 s to 0.500/1.05 = 0.4762 s (~476 vs ~500 rows). Window length is an *outcome*, never a gate.

## 5. Line-16 guard: static proof and activation requirement

**Static proof that `settings->sequence_number == 16` selects the arc block:**

1. `interp_execute.cc:241` (read-verified in the live tree): `block->line_number = settings->sequence_number;` at the head of `Interp::execute_block` — the sequence number equals the NC source line counter for the block being executed.
2. `g93_probe.ngc` (sha256 `fdcc31b2…`): lines 1–10 comments, 11 `G21 G90 G61 G93`, 12 `M53 P1`, 13–18 motion blocks B1–B6. **Line 16 = `F120 G2 X10 Y0 I0 J-10`** — the only arc.
3. The guard executes inside `inverse_time_rate_arc`, called from `convert_arc2` during execution of the line-16 block ⇒ `sequence_number == 16` there.
4. Empirical corroboration (straight twin, WP-B): seed guarded by the identical predicate produced `failing_lines=[16]` exactly, 3/3 runs; L1.5 P4 window/line 1:1 matching confirms the same identity chain.

**Execution is not proven by a diff.** A static diff certifies configuration, not activation. Therefore: a **guard-activation dry run** (one seeded straight-line canary or direct T1W1 probe inspection) is required after explicit F1 execution approval, marked **VOID / non-scored**, before any formal run (WP-B §9 precedent: an NC header edit once silently shifted the target off line 16).

## 6. G94 scope limitation (per independent review §4)

`inverse_time_rate_arc` returns early when `feed_mode != INVERSE_TIME` (interp_inverse.cc:62; call sites are bare calls with discarded returns — read-verified). Under G94, the W patch's appended `write_canon_state_tag` therefore executes with an unchanged `settings->feed_rate` — i.e., it emits a **duplicate observable metadata write with the same value**, not "nothing". **No general G94-inertness claim is made.** The microstudy runs G93 exclusively (NC line 11); no G94 assertion beyond the inspected-source statement above.

## 7. Files this spec freezes by reference

`g93_probe.ngc` (fdcc31b2…), `g93_probe.hal` (660550b6…), `g93_probe.ini/.var/.tbl` (F-1 freeze), `adapter_l15.py` (eb6a6690…), `l15_drive.py` drive timeline (F-1 lineage). Any byte change to any of these ⇒ this protocol is void and must be re-reviewed.
