RQ3-B paper draft (drop-in; numbers traced to verdicts/LC-T*.json, 2026-10-05)
Suggested placement: LinuxCNC subsection (after the existing F-1 closure /
LC-D1 material), plus one row-pair in the RQ3 conceptual matrix. Labels follow
the frozen experiment naming LC-T0/T1/T2. Wording constraints respected: the
seeded condition is an intervention, never a LinuxCNC defect; no detection-rate
claims.

------------------------------------------------------------------------------
Paragraph (result, ~190 words):
------------------------------------------------------------------------------
To complement the witness-binding case, we added a preregistered positive
control on the same frozen LinuxCNC 2.9.4 subject: a controlled violation of
the transformation responsibility itself, leaving witness binding and evidence
qualification intact. The seed multiplies the post-conversion inverse-time
rate f_u = L·F by 1.05 inside the interpreter's actual conversion computation
(interp_inverse.cc, inverse_time_rate_straight), restricted to one
preregistered G93 block of a straight-only probe program, so both the
diagnostic feed-upm witness and the commanded-velocity channel derive from the
altered value through their stock producer chain. Under the unchanged frozen
checker and oracle (expected values recomputed from NC geometry only), the
stock condition passed on every block in 3/3 runs; the seeded condition kept
full qualification (block identity, feedhold window, per-block evidence all
present) yet failed the semantic predicate on exactly the targeted block
(630.00 vs. L·F = 600.00, ratio 1.0500), with the motion-side predicate
failing at the identical ratio; reverting to the byte-identical stock library
restored PASS in 3/3 runs. Against the earlier case — where feed-upm diverged
while commanded motion stayed correct — the two channels here fail together
at the seeded value, operationally separating transformation-level from
witness-level causes under one unchanged evidence discipline.

------------------------------------------------------------------------------
Matrix row (RQ3 conceptual matrix, four LinuxCNC conditions):
------------------------------------------------------------------------------
| LinuxCNC condition          | Binding semantics | Qualification | Transformation responsibility | Verdict / interpretation |
|-----------------------------|-------------------|---------------|-------------------------------|---------------------------|
| Stock (LC-T0)               | correct           | PASS          | satisfied                      | PASS                       |
| Pre-conversion witness (F-1)| incorrect role    | appears qualified under declared binding | nominal | inconsistency → adjudication → Class C |
| Seeded violation (LC-T1)    | correct           | PASS          | violated (+5%, one block)      | qualified FAIL             |
| Revert (LC-T2)              | correct           | PASS          | satisfied                      | PASS                       |

------------------------------------------------------------------------------
One-line method sentence (for the study-design/protocol paragraph):
------------------------------------------------------------------------------
The seeded study follows a stock→seed→revert design (3/3/3 runs), with the
runtime library as the only switched component (per-run loaded-image
verification), a preregistration frozen before formal runs, and stock/revert
sharing one byte-identical library artifact so that the seed patch is the sole
difference under a single build configuration.

------------------------------------------------------------------------------
Reviewer-response logic (rebuttal-ready, taskbook §20):
------------------------------------------------------------------------------
- To M2: RQ1 is intentionally a mechanism-isolation ablation, not a competitive
  baseline study; external positive validity is provided on LinuxCNC, where the
  same qualification machinery yields a qualified FAIL for a source-level
  transformation violation under an unchanged, valid binding.
- To M4: the original LinuxCNC case establishes the residual binding-trust
  boundary; this positive case holds the binding fixed and correct while
  modifying the transformation itself, distinguishing a witness-level
  inconsistency requiring semantic adjudication from a genuine responsibility
  violation under admissible evidence.

Claim boundaries (must accompany the result): the case demonstrates detection
of a true, seeded responsibility violation under admissible evidence; it does
not estimate detection rates, does not show that LinuxCNC contains such a
defect naturally, and makes no physical-safety claim.
