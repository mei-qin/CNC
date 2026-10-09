# F0 Source Basis — G93 Arc Paired Interaction (Read-only)

**Package:** `linuxcnc_f0_interaction` (repo `CNC`, branch `CNC_V1`)
**Authority:** static source inspection and feasibility reporting only. No build, no runtime, no patching, no test execution, no criterion change, no scored claim. (MICRO_STUDY_F0_FEASIBILITY_TASKBOOK, sha256-anchored via `S4_B_Reframing_Interaction_Test_Package.zip` = `40546429ecddcd566961de735a7f40a5b398cff4255368afdd1c51a21d42befe`)
**Date:** 2026-10-09
**Inspector session:** all source reads performed read-only in WSL `Debian-L15` and in frozen ZIP archives; no file in the subject tree or in any frozen archive was modified.

---

## 1. Subject identity

| Item | Value | Receipt |
|---|---|---|
| Subject software | LinuxCNC `1:2.9.4-2+deb13u1` (Debian trixie apt source), RIP-tree execution | F1_G0_RECORD; deb sha256 `e14c9399fdda0472f3392bc99a4b2fac775cc1893ce322d3c44549200e3819af` |
| Source tree (live, read-verified) | `/home/l15/linuxcnc-2.9.4` in WSL distro `Debian-L15` | this session |
| `interp_convert.cc` | sha256 `f2e7cf41ec6bc4b1cd86bf6a5620c7d6028314f66a2980491ffab19156a506cb` — byte-identical to `interp_convert.cc.stock` in `S2_Binding_Assurance_Package.zip` `.../LC_D1/linuxcnc_l15/f1_closure/` and to the live tree's own `.stock` copy | F1_G2_PATCH_PREREG hygiene hash; independently recomputed this session (archive + live tree) |
| `interp_inverse.cc` | sha256 `c98db3d4…` (live tree = `.stock` copy, empty diff) | WP-B prereg §10; recomputed this session |
| stock milltask (G1 reference) | sha256 `d90c4b5796569446bee475b09c7b479f9013275af2271b19eb98588b4e42d2cb` | F1_G0_RECORD |
| Patched (W0) historical identity | source `6ab767c785b1d402e4795c67766c405e6fd8c2390de63bda7262688696fcda20`, lib `9972b40d8f216e7f28a35be4de69fd2a3371dcab4cf9c59cf403ccdc6fb3b4e6` | **F1-CLOSURE-FREEZE-1 `FREEZE_MANIFEST.json` lineage block** — this *corrects* the preliminary screen's "patched-run binary provenance not in package": the hashes are recorded in the freeze manifest (the patched source *file* is not a package member; the manifest carries its digest) |
| Tree-built stock lib (WP-B) | `d3e4ecd4…` (rebuild byte-reproducible); deb-packaged lib `3a8ce231…` | WP-B source_identity |
| Checker | `adapter_l15.py` sha256 `eb6a669039403126a3c2b770b9c4e62ea50336c6224cd0fb3dd5cf7827125905` — repo copy `scripts/linuxcnc_l15/adapter_l15.py` verified byte-identical this session | WP-B lineage_audit |
| Probe NC / HAL | `g93_probe.ngc` sha256 `fdcc31b2…`, `g93_probe.hal` sha256 `660550b6…` (recomputed live this session) | live `/home/l15/` |

Version discipline: 2.9.4 (frozen subject) and 2.9.10 (CS-PRESENT reproduction) are **not interchangeable**; the prospective 2×2 runs on 2.9.4 only. The CS-PRESENT result remains a separate portability observation.

## 2. Common stage and unit (G1)

**Fixed eligibility unit:** one G93 inverse-time arc block — `g93_probe.ngc` source line 16: `F120 G2 X10 Y0 I0 J-10` (start (0,10), center (0,0), r=10, quarter circle).

- Arc length `L = r·π/2 = 15.70796327 mm`; `F = 120 min⁻¹`.
- Original semantic formula (responsibility R): `fu = L·F = 1884.9556 mm/min`; motion-side velocity `rv = fu/60 = 31.415927 mm/s`.
- Program timeline (NC header, adapter-confirmed): B1..B6 block times 1.0/2.0/1.0/0.5/2.0/1.0 s; arc block nominal 0.5 s; under T1 seed the arc block shortens to `0.5/1.05 = 0.4762 s` (WP-B straight analogue: 12.0 s → 11.5 s IDLE shift, ratio-exact).
- **Native diagnostic witness** (audited object): `motion.feed-upm` HAL pin = tag field `GM_FIELD_FLOAT_FEED` of the *executing* motion command (§4). Units mm/min.
- **Independent converted-motion witness** (not tag-derived): `motion.requested-vel` pin (mm/s), reflecting the actually commanded interpolated feed (from the SET_FEED_RATE trajectory command stream), plus the offline NC oracle `L·F` recomputed by `adapter_l15.py` `parse_nc()` from NC geometry alone (regex + Euclidean/arc-length math, zero LinuxCNC dependency).
- **Same-cycle key:** all ten channels are sampled by one `sampler.0` on one `servo-thread` at 1 kHz; each capture row (column 0 = sample index) is one servo cycle — channels within a row are same-cycle by construction, keyed by row index. No NML/HAL stream is promoted across cycles without this key.
- **Block start/end:** per-block windows are the contiguous sampler-row spans with motion-type ∈ {2,3} and constant program-line (pin.8), produced by `adapter_l15.py::windows()` (L98–115); window construction is keyed on sampler rows, **not on tag count**, so it is structurally identical under W0 (two tag writes per arc block) and W1 (one).
- Run indexing: 3 technical replicates per cell (prospective); replicates never counted as independent units (unit = cell).

## 3. T-axis intervention (arc conversion violation) — the previously BLOCKED item

**Anchor (read-verified this session, `interp_inverse.cc` L47–73):**

```c
58 {
59   double length;
60   double rate;
62   if (settings->feed_mode != INVERSE_TIME) return -1;
64   length = find_arc_length(x1, y1, z1, cx, cy, turn, x2, y2, z2);
65   if (length == 0){
66     rate = 0.1; // See https://github.com/LinuxCNC/linuxcnc/issues/2410
67   } else {
68     rate = length * block->f_number;
69   }
70   enqueue_SET_FEED_RATE(rate);
71   settings->feed_rate = rate;
72   return INTERP_OK;
73 }
```

- **Seed design (prospective, mirrors WP-B straight twin at the same file):** after L68 insert `if (settings->sequence_number == 16) { rate = rate * 1.05; }` with the WP-B comment convention marking it an experimental intervention, not a LinuxCNC defect.
- Guard validity: `settings->sequence_number` is the NC source line counter — `interp_execute.cc:241` assigns it to `block->line_number`; empirically confirmed by WP-B (T1 failing_lines=[16] exactly) and by L1.5 P4 window/line 1:1 matching.
- Structure parallelism: `inverse_time_rate_straight` (L93–136) is line-for-line parallel (feed-mode guard → length → `rate = length * block->f_number` → `enqueue_SET_FEED_RATE` → `settings->feed_rate`); the straight twin was seeded successfully in WP-B (qualified FAIL 3/3/3, ratio exactly 1.0500). The arc seed is therefore source-homologous, not a new mechanism.
- G94 inertness: L62 returns `-1` before any side effect when feed mode ≠ INVERSE_TIME; the call sites (`interp_convert.cc:602` et al.) are bare calls whose return value is discarded, so in G94 the function is a no-op — the probe program is all-G93 from line 11 onward, and the W patch is inert in G94 blocks generally (F1_G2_PATCH_PREREG's append-overwrite rationale).
- **New source variants required (no historical artifact):** arc-seeded `interp_inverse.cc` (for T1W1) and arc-seeded + W-patched combination (for T1W0); hashes to be recorded at F1 build time under the WP-B `source_identity` protocol. This is bounded, single-file, ≤6-line diff work on a proven template — not new tool framework.

## 4. W-axis intervention (witness production stage) and the native witness chain

**Stock (W1, wrong stage):** `interp_convert.cc:479` `write_canon_state_tag(block, settings);` in `Interp::convert_arc` dispatch, *before* plane/cutter-comp routing to `convert_arc2` / `convert_arc_comp1` / `convert_arc_comp2`, which call the converter at `:602–603` / `:689` / `:847` respectively and then enqueue the motion (`ARC_FEED` :605; `enqueue_ARC_FEED` :744/:956–978). The tag therefore captures `settings->feed_rate` as left by the *previous* block (B3, line 15: L=10, F=60 → 600.00) — the F-1/CS-PRESENT stale-witness signature (fu=600.00 vs oracle 1884.9556, ratio 0.3183).

**Patch (W0, correct stage; pre-registered blueprint, not new design):** F1_G2_PATCH_PREREG — append `write_canon_state_tag(block, settings);` immediately after each of the three `inverse_time_rate_arc(...)` calls; 3 insertions, 0 deletions; `:479` retained. Empirically executed as F-1 G3 (p1s1–3: arc fu 600.00 → 1884.9556, S1–S5 all OK) with motion invariance G4 (preregistered per-axis E∞ criterion passed unchanged; window-length equality replaced post hoc by control-calibrated envelope — reported as post hoc in the freeze manifest).

**Witness chain (every link read-verified this session):**

1. `interp_write.cc:224` `write_state_tag`: `state.fields[GM_FIELD_LINE_NUMBER] = settings->sequence_number` (L229); `state.fields_float[GM_FIELD_FLOAT_FEED] = settings->feed_rate` (L338).
2. `interp_write.cc:344` `write_canon_state_tag` → `interp_convert.cc:5801` `update_tag` → canon `UPDATE_TAG` — the tag becomes the pending canon-queue tag carried by subsequently enqueued canonical calls until the next `UPDATE_TAG` (**last-write-wins**). Under W0 the second (post-conversion) write is the one pending when `ARC_FEED` is enqueued; under W1 the sole `:479` write governs.
3. Task → motion: the tag travels with the queued motion command; at execution `src/emc/motion/control.c:1872` writes `*(emcmot_hal_data->feed_upm) = emcmotStatus->tag.fields_float[GM_FIELD_FLOAT_FEED]` — the pin value during arc execution is the tag of the *executing* arc command.
4. `g93_probe.hal`: `motion.feed-upm → sampler.0.pin.5`, `motion.requested-vel → pin.4`, `motion.program-line → pin.8`; single sampler, servo-thread, 1 kHz, depth 65536.

The three-level historical causal chain stock(600.00, FAIL) → patch(1884.9556, PASS) → revert(600.00, FAIL) (F1-CLOSURE-FREEZE-1, G5 revert by libctl swap) pins this mechanism empirically; the 2×2 prospective runs re-verify it per cell via the G4 load gate (§6).

## 5. Toggle independence and the declared cross-term (G2)

- **Disjoint anchors:** T seed = `interp_inverse.cc:68` (function-local `rate`); W patch = `interp_convert.cc:602/:689/:847` append sites. Different files, different functions, no shared mutable state at the edit sites.
- **T does not silently move W's stage:** the seed multiplies the local `rate` after `rate = length * block->f_number`; it never writes `block->f_number`, never reorders tag writes. Under W1 the tag still captures the pre-conversion `settings->feed_rate` — the seed value itself never reaches the stock-positioned tag (masking, below).
- **W does not modify conversion logic:** `write_canon_state_tag` only reads `settings`/`block` and enqueues metadata; it writes nothing that feeds back into `inverse_time_rate_arc` or motion values. Historical G4 motion invariance (p1s vs g1r, per-axis E∞ ≤ 2% travel unchanged) corroborates.
- **Declared cross-term (W0 × T1):** with the tag appended after conversion, the witness faithfully reports the *violated* conversion: expected fu = 1884.9556 × 1.05 = **1979.2034** (rv = 32.986723). This is a pre-declared predicted value of the correct-stage witness under T1, not a confound.
- **Designed masking (W1 × T1):** the wrong-stage witness is structurally blind to the conversion (any T): fu stays 600.00, identical to T0W1. This indistinguishability is the object of the study, and the cell remains adjudicable via the independent channels (§3 truth basis).
- **Alternative explanations / isolation:** build/harness variation is excluded prospectively by the single-switchable-library protocol + per-run `/proc/<milltask>/maps` hard gate + per-run libsha receipts + T0W1 stock-equivalence check; NC and window definitions are byte-frozen (NC sha, adapter sha, HAL sha); revert logic is binary-level (libctl swap) per F-1 G5 / WP-B T2 precedent.

## 6. Build → loaded identity per prospective run (G4)

Protocol (WP-B `oneshot_wpb.sh` lineage, unchanged):

1. Four conditions, each a distinct `librs274.so.0` artifact switched via `~/libctl/` (the only runtime-variable component):
   - T0W0 = W-patched `interp_convert` (byte-identical patch ⇒ source sha must reproduce `6ab767c7`; lib distinct from `9972b40d` only if build env differs — record actual)
   - T0W1 = pristine tree (stock reference; F1 must fix **one** stock lib identity — tree-built `d3e4ecd4` or deb `3a8ce231` — and use it for both stock-facing conditions and revert checks)
   - T1W1 = arc-seeded `interp_inverse` only (new hashes at build time)
   - T1W0 = arc-seeded `interp_inverse` + W-patched `interp_convert` (new hashes at build time)
2. Per run: record libsha12, abort unless `/proc/<milltask>/maps` shows the libctl library (hard gate), archive maps snapshot, capture CSV, adapter report.
3. All four conditions: same deb version anchor, same INI/HAL/NC (hashes §1), same tolerance set (TOL_PIN 2%, TOL_OVR 3%, H1 1%, HOLD 0.1 s — frozen in `adapter_l15.py`).
4. Known environmental hazards carried forward from WP-B/CS operational records (SysV/NML shm residue, cross-distro port collisions, `set -u` vs rip-environment): mitigation scripts already exist (`wpb_clear.sh`); they are operational hygiene, not design changes.

## 7. Pre-declared expected values (unscored; historical values marked)

| Quantity | Formula | T0W0 | T0W1 | T1W0 | T1W1 |
|---|---|---|---|---|---|
| arc fu oracle | L·F | 1884.9556 | 1884.9556 | 1884.9556 | 1884.9556 |
| feed-upm witness (tag) | — | 1884.9556 *(historical p1s)* | 600.00 *(historical g1r, CS-PRESENT)* | **1979.2034** (declared cross-term) | **600.00** (masked, ≡ T0W1 signature) |
| P1 (fu vs oracle, 2%) | — | PASS | FAIL 0.3183 | FAIL 1.0500 | FAIL 0.3183 |
| requested-vel | L·F/60 (×1.05 if T1) | 31.415927 | 31.415927 | **32.986723** | **32.986723** |
| P2 (rv vs oracle, 2%) | — | PASS | PASS | FAIL 1.0500 | FAIL 1.0500 |
| arc block time | 0.5 s / (1.05 if T1) | 0.5 s | 0.5 s | 0.4762 s | 0.4762 s |

No cell's expected values are derived from the audited witness itself; truth anchors are the NC oracle, the source diffs, and the motion-side channel. These are **feasibility-level predictions for protocol review, not results**; the two T1 columns have no historical arc runs.

## 8. Open items transferred to F1 (explicit; no hidden conditional successes)

1. Arc-seed guard activation must be confirmed before formal runs (WP-B §9 precedent: header-line count once silenced the straight seed). Bounded dry-run check, VOID-able, inside F1 authority.
2. New source/lib hashes for T1 conditions recorded at build time; byte-reproduction check of `6ab767c7` for the W patch.
3. Per-run maps snapshots archived for **all** cells (closing the class of gap that F-1 closed at manifest level only).
4. Choice of single stock reference lib fixed in the F1 freeze.
5. F1 must re-state the adjudication rules (witness-untrusted vs transformation-violated vs abstain) and the ABSTAIN count as a first-class outcome, per the S4-B gate decision §4.
