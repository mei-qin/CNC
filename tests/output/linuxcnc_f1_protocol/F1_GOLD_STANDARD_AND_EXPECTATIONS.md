# F1 Gold Standard & Pre-Declared Expectations (Frozen Proposal)

**Package:** `linuxcnc_f1_protocol` · **Status:** prospective protocol — NO EXPERIMENT AUTHORIZED
**Taskbook:** F1_PROTOCOL_PREPARATION_TASKBOOK Work 2.

---

## 1. Gold-standard definition (fixed before any reading)

The gold-standard state of a run is the **two-dimensional configured-and-executed fact**:

- **T ∈ {T0 correct conversion, T1 seeded violation}** — whether the arc conversion for NC line 16 computed `L·F` or `1.05·L·F`;
- **W ∈ {W0 correct-stage tag, W1 wrong-stage tag}** — whether the state tag governing the executing arc motion was written after (appended) or before (stock `:479`) the inverse-time conversion.

The gold state is established **only** from the following evidence chain, none of which is any comparator's verdict:

1. **Version-controlled source identity**: per-run libctl library sha256, matched against the variant manifest (pristine/patched source hashes, §3); contemporaneous `/proc/<milltask>/maps` snapshot proving that library was loaded (hard gate — run aborts otherwise, WP-B/F-1 lineage).
2. **Source diffs** (`patches/F1_T_seed_interp_inverse.patch`, `patches/F1_W_tag_interp_convert.patch` — the W diff byte-reproduces the historically executed patch, sha `6ab767c7…`).
3. **NC geometry oracle**: `L·F` recomputed offline from `g93_probe.ngc` alone (regex + arc-length math, zero LinuxCNC dependency).
4. **Independent motion-side channel**: `requested-vel` (sampler pin.4) over the arc window vs oracle — confirms *execution* of the configured T state (activation evidence), and is not derived from the tag witness (`feed-upm`, the audited object).

Truth is never assigned from O/I/Q/I+Q/FULL-B outputs, and never from the `feed-upm` witness.

## 2. Configuration vs execution; disagreement handling (frozen)

A diff proves configuration; execution requires runtime corroboration. Per-run consistency checks and their dispositions:

| Check | Expected (given loaded variant) | On mismatch |
|---|---|---|
| libctl sha + maps | equals variant manifest entry | run already aborted at load gate |
| arc `requested-vel` ratio vs oracle | T0: 1.000 (±2% tol); T1: 1.050 (±2% tol) | **VOID / NON-ADJUDICABLE** — e.g., a T1-lib run with rv ratio 1.000 means the guard never fired; never relabel to T0 |
| arc `feed-upm` signature | W0: oracle-consistent (1884.9556 or 1979.2034); W1: stale 600.00 | **VOID / NON-ADJUDICABLE** — e.g., a W0-lib run showing 600.00 means the patch did not govern; investigate as anomaly, do not relabel |
| P4 block identity | 6 windows 1:1 with NC lines 13–18 | **VOID** |
| capture integrity | §3 of F1_CAPTURE_AND_WINDOW_SPEC | **VOID** |

**Priority rule:** no evidence source "overrides" another; any provenance↔runtime contradiction voids the run. Voided runs are reported with their anomaly; they are never re-used under a different cell label. (A genuine, reproducible contradiction would itself be reportable as an unexpected finding — not absorbed.)

## 3. Variant identity manifest (byte-grounded this package)

| Variant | interp_inverse.cc | interp_convert.cc | librs274.so.0 |
|---|---|---|---|
| T0W1 (stock) | pristine `c98db3d4…` | pristine `f2e7cf41…` | **stock reference lib frozen = tree-built `d3e4ecd4…`** (WP-B source_identity; byte-reproducible). Deb-packaged `3a8ce231…` is reference-only, not run. |
| T0W0 | pristine `c98db3d4…` | patched **`6ab767c7…`** — reconstructed this session from pristine bytes + W patch; byte-equal to the historically executed F-1 G3 patched source (manifest-verified) | build at F1 execution; sha to be recorded before any run |
| T1W1 | seeded **`2096776f…`** (new; constructed this session, `variants/T1W1_interp_inverse.cc`) | pristine `f2e7cf41…` | build at F1 execution; sha to be recorded |
| T1W0 | seeded `2096776f…` | patched `6ab767c7…` | build at F1 execution; sha to be recorded |

Pristine snapshots and both variant sources are package members (`pristine/`, `variants/`); unified diffs in `patches/` (T, W, and combined). Reconstruction script: `tools/make_variants.py` (text-only). **Historical manifest pairs are precedent only** — all four prospective libraries require fresh build receipts (exact build commands echoed and archived) plus per-run load attestation.

## 4. Pre-declared expected values (predictions; no observations here)

Fixed unit: `g93_probe.ngc` line 16 arc, `L = 15.70796327 mm`, `F = 120` ⇒ oracle `fu* = L·F = 1884.9556 mm/min`, `rv* = fu*/60 = 31.415927 mm/s`; stale W1 witness value = previous block's converted rate = 600.00 mm/min (B3: L=10, F=60).

| Cell | arc feed-upm (predicted) | arc requested-vel (predicted) | P1 (fu vs oracle, 2%) | P2 (rv vs oracle, 2%) | arc block time |
|---|---|---|---|---|---|
| T0W0 | 1884.9556 *(historically observed, p1s)* | 31.415927 *(historical)* | PASS | PASS | 0.500 s |
| T0W1 | 600.00 *(historical g1r/CS)* | 31.415927 *(historical)* | FAIL 0.3183 | PASS | 0.500 s |
| T1W0 | **1979.2034** (declared cross-term: correct-stage witness reports the violated value) | **32.986723** | FAIL 1.0500 | FAIL 1.0500 | 0.4762 s |
| T1W1 | **600.00** (masked; byte-identical fu signature to T0W1 by mechanism) | **32.986723** | FAIL 0.3183 | FAIL 1.0500 | 0.4762 s |

All T1 quantities are predictions without historical arc runs. These expectations freeze the *gold-side* reading rules only; they are not comparator outputs and confer no scored claim. Three technical replicates per cell serve signature determinism (cells, not runs, are the units); no N=12 defect-count or effect-rate claim is made or implied.

## 5. Non-goals

No new scored data, no manuscript main-effect claim, no EAV-1 restart, no edits to B3/F-1/WP-B/CS archives, no G94 claims (see F1_CAPTURE_AND_WINDOW_SPEC §6).
