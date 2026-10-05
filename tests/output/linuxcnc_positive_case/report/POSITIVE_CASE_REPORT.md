# LinuxCNC Positive Transformation-Violation Case — Final Report (WP-B, LC-POSX-1)

Date: 2026-10-05 · Subject: LinuxCNC 2.9.4-2+deb13u1 (WSL Debian-L15, frozen
F-1 runtime discipline) · Prereg: `prereg/prereg_positive_transform_violation.md`
(frozen 04:35 +08 before formal runs) · Verdicts: `verdicts/LC-T{0,1,2}.json`

## Outcome (frozen expectations, 9/9 runs as predicted)

| Condition | lib sha12 | Reps | Qualification | Semantic (P1) | P2 profile | Verdict |
|---|---|---|---|---|---|---|
| LC-T0 stock | d3e4ecd4 | 3 | PASS ×3 | PASS ×3 (6/6 blocks exact) | PASS | **PASS** |
| LC-T1 seeded | 64557990 | 3 | PASS ×3 | **FAIL ×3 — line 16 only, fu 630.00, ratio 1.0500** | FAIL ×3 (line 16, rv 10.5000, ratio 1.0500) | **FAIL** |
| LC-T2 revert | d3e4ecd4 | 3 | PASS ×3 | PASS ×3 (6/6 blocks exact) | PASS | **PASS** |

Causal chain (per-block fu_med, bit-stable within each condition):

    stock 600.00 (PASS)  ->  seeded 630.00 (FAIL, ratio exactly 1.0500)  ->  revert 600.00 (PASS)

Success criterion (taskbook §12.1): Qualification_seed = PASS **and**
SemanticVerdict_seed = FAIL, with stock and revert PASS — **satisfied**.
Isolation showcase inside every seeded run: block 15 PASS (600.00) → block 16
FAIL (630.00) → blocks 17–18 PASS (424.26 / 848.53). Program duration moved
12.0 s → 11.5 s (the seeded block's traverse shortened by exactly 1/1.05),
corroborating that the real transformation — not just the diagnostic — changed.

## Discriminator against the existing Class-C case (the point of WP-B)

| | Existing F-1 (witness-level) | WP-B LC-T1 (transformation-level) |
|---|---|---|
| Intervention | none (natural), arc state-tag captured pre-conversion | seeded δ=+5% inside the conversion, target block only |
| feed-upm (P1) | FAIL (stale 600 vs 1884.96 on arc) | FAIL (630 vs 600, ratio 1.05, straight) |
| requested-vel (P2) | PASS (motion correct) | **FAIL at the same 1.05** (motion really changed) |
| Interpretation | two native witnesses diverge → qualified inconsistency → source adjudication → Class C binding misbinding | witnesses coherent at the violated value; binding correct; evidence admissible → **qualified semantic FAIL** |

The two-channel signature (P1 and P2 failing together at an identical ratio,
vs F-1's P1-only failure) is what operationally separates a transformation-level
cause from a witness-level cause on an independent controller, under unchanged
checker and binding.

## The ten questions (taskbook §17)

**1. What responsibility was violated?**
The G93 inverse-time feed transformation for straight moves: the post-conversion
rate f_u = L·F that the interpreter must produce for each inverse-time block
(R: every G93 straight block's commanded rate equals its path length times its
F word).

**2. Where is the seed, in the real transformation producer?**
`src/emc/rs274ngc/interp_inverse.cc`, `Interp::inverse_time_rate_straight`, at
the stock statement `rate = length * block->f_number;` — the sole computation
of f_u on the straight path, feeding BOTH `enqueue_SET_FEED_RATE(rate)`
(commanded motion) and `settings->feed_rate` (state-tag feed field that drives
the feed-upm witness). Patch: `patches/stock_to_seed.patch` (+9 guarded lines);
source sha c98db3d4 (stock) → b22bdaff (seeded).

**3. Why is this not a witness mutation?**
The witness chain — `motion.feed-upm` pin → sampler channel → per-block
windows — is byte-identical across all conditions (`g93_probe.hal`,
660550b6…). The seed changes a VALUE inside the producer, not the producer's
identity, capture location, or ordering. The F-1 case, by contrast, was exactly
a capture-ordering (witness-semantics) problem. Here the stock
conversion-then-tag order is untouched (pristine `interp_convert.cc`
f2e7cf41… in all three conditions).

**4. Why is the binding still semantically correct?**
On the straight path the canonical state tag is packaged AFTER the conversion
(stock order), so `motion.feed-upm` observes the current block's post-conversion
rate — the declared binding "post-conversion inverse-time rate of the executing
block" is true of the actual producer in all three conditions. Empirically:
stock/revert straight blocks read back exactly L·F (240/300/600/600/424.26/848.53),
and the seeded block reads back exactly 1.05·L·F through the same chain.
Full argument: `audit/binding_audit.txt`.

**5. Why does qualification still pass?**
P4 (block identity 1:1/monotone/complete) PASS in 9/9 runs; P6 (feedhold
feature window ≈1.0 s) PASS in 9/9; every NC block has a capture window with
evaluated P1 samples (witness alive per block); trace valid; no aborts. The
seed changed a value, not the evidence structure.

**6. How is the expected value computed independently?**
`adapter_l15.py::parse_nc` derives L and F from the NC file geometry alone
(regex word parse + Euclidean distances; no LinuxCNC code, no interp state,
no link to the seeded library) and forms f_u_exp = L·F and v_exp = L·F/60. The
checker is byte-identical (sha eb6a6690…) to the one used in every L1.5/F-1
run; tolerances (2%) untouched. Full argument: `audit/lineage_audit.txt`.

**7. Stock verdict?** PASS ×3 — all six blocks exact, P1–P6 PASS.

**8. Seeded verdict?** FAIL ×3 — qualification PASS, semantic P1 FAIL on
exactly the preregistered block (line 16, ratio 1.0500 ∈ [1.045, 1.055]);
P2 fails on the same block at the same ratio (motion changed — as it must when
the transformation itself is wrong); P3/P4/P5/P6 PASS.

**9. Did the revert restore?** Yes — PASS ×3, with the runtime library
byte-identical to LC-T0 (d3e4ecd4…, binary-level swap; build-tree source
verified restored to pristine c98db3d4…).

**10. What does this prove — and not prove?**

Proves: on an independently developed controller, with an unchanged, audited,
semantically correct witness binding and fully admissible evidence, a genuine
transformation-responsibility violation is detected by the frozen checker as a
qualified semantic FAIL (and the method's two-channel signatures separate
witness-level from transformation-level causes).

Does NOT prove (and must not be claimed): defect detection rate or recall;
that all G93 defects are detectable; that LinuxCNC contains such a natural
defect (the seeded condition is a marked experimental intervention); coverage
of arbitrary transformation bugs; any physical machining safety impact.

## Protocol deviations (all recorded)

1. Pre-freeze NC header trim (block line layout 19–24 → 13–18, B4 at 16) —
   input-authoring correction before freezing; dry-runs repeated.
2. Clearing-ritual hardening (`wpb_clear.sh` + SysV shm purge + one cross-distro
   port-5007 cleanup) after two voided dry boots (environment failures, kept in
   `dry/`); no formal run affected.
3. AMENDMENT 01: one-character syntax repair in the frozen aggregator's print
   statement (discovered at first execution, before it had produced output; no
   computation changed).

## Reproducibility anchors

Libraries: stock-tree d3e4ecd4… / seeded 64557990… (same build config; stock
rebuild byte-reproducible); per-run loaded-image proof via
`/proc/<milltask>/maps` HARD GATE; per-run lib sha sidecars in `runs/`.
Full ledger: `hashes/SHA256SUMS` (91 entries). Every capture, report JSON/MD,
drive log, and the frozen prereg/patches/audits are included in this package.
