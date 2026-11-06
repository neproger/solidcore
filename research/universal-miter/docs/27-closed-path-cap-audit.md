# 27 — Closed-path cap audit: the arc–arc case was capped incorrectly

Status: the cap-plane mismatch is explained by treating a **closed path** as an
open, capped path. No boundary contract change is needed for this case.

## 1. Measured flag

The cap audit prints `normalized.closed`:

| Case | closed | prepared start | prepared end |
|---|---|---|---|
| line–arc | `false` | original start | original end |
| arc–arc (matrix pair) | `true` | `(-1.0000000000, 1.7071067812, 0)` | `(-1.7071067812, 1.0000000000, 0)` |

The arc–arc matrix pair starts at `(-1,1)` and ends at `(-1,1)`. It is a closed
loop, so `normalize_planar_sweep` sets `closed = true`, extends the closure, and
sets `cap_start = cap_end = false`.

`build_direct_chain` ignored that flag and built start/end caps from the
original endpoints. The side surface is the extended closure geometry, so its
U boundary is `0.7071` from the original cap plane. That is the measured
cap-plane deviation and the `1.4142 = 2 × 0.7071` tolerance growth.

## 2. Consequence

The current arc–arc direct test is invalid for the cap path:

- a closed path has **no caps**;
- its start/end meet at the closure joint;
- the closure joint must be part of the topology, not replaced by two caps.

The cap-profile edge/tolerance audit was measuring the wrong construction.

## 3. Passing comparison

- line–arc is open (`closed=false`), prepared and original endpoints coincide,
  so caps are valid and cap-plane deviation is `~1e-17`.

## 4. Next

1. Add an **open** arc–arc matrix case (`start != end`) to validate two-piece
   joint + caps without closure.
2. Separately implement the closed-path case: no caps, closure joint between
   the last and first segments, and a shared closure seam.
3. Do not change cap planes or tolerances; do not cap a closed path.

The production semantics are now clear: closed paths have no end caps; open
paths cap at their final path boundaries.

## 5. Open arc–arc follow-up

An open arc–arc test was added (`path_b` ends at 45° instead of 90°):

- `closed = false`;
- both caps are valid;
- cap-plane mismatch and `sqrt(2)` tolerance inflation are gone;
- `check_brep` still reports 4 failures, first an `Edge/Edge Gap` of
  `3.488e-5` against tolerance `2e-5`.

This is a separate, much smaller 3D endpoint/trim gap, not the closed-path cap
defect. The open test is currently ignored until that gap is audited.