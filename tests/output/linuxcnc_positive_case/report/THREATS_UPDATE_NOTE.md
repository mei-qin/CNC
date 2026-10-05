Threats to validity / construct-validity update note (WP-B, 2026-10-05)
Suggested placement: construct-validity paragraph (the §7.4-style "builder-
evaluator" discussion). Three to five sentences; coordinates with the existing
"residual exposure belongs to seeded-deviation studies" sentence.

------------------------------------------------------------------------------
Draft text:
------------------------------------------------------------------------------
Construct validity for the transformation-level claim is supported by a
preregistered seeded-deviation study on the independent LinuxCNC subject
(LC-T0/T1/T2, 3/3/3 runs): the seeded condition modifies only the conversion
computation of one preregistered G93 block (+5%), under an unchanged witness
binding, unchanged checker and tolerances, and an oracle recomputed solely
from NC geometry, with the runtime library as the only switched component and
its loaded image verified in every run. Because stock and revert share one
byte-identical library artifact and differ from the seeded build only by the
preregistered patch under a single build configuration, the observed verdict
flip (PASS → qualified FAIL → PASS) is attributable to the seeded
responsibility violation rather than to build or harness variation. The
remaining construct caveat is the deliberate coupling of witness and motion
channels: a real transformation violation changes both, so the method's
diagnosis of transformation-level (vs. witness-level) causes rests on the
two-channel coherence signature rather than on the semantic predicate alone;
this is by design and is reported as such. No detection-rate or
natural-defect claim is made.
------------------------------------------------------------------------------

Anchors for reviewers: prereg freeze record (hashes of NC, checker, scripts,
libraries) precedes all formal runs; deviations are limited to a pre-freeze
input-authoring correction, environment-failure voided boots (kept), and a
syntax-only repair of the aggregation print (AMENDMENT 01); all 9 formal runs
match the frozen expected-outcome table.
