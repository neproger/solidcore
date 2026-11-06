# 39 â€” Arcâ€“Arc refusal contract

Status: research closure. No ArcArcMiter, extension surface or adaptive cut is
implemented. Boundary Solver, Direct B-rep Builder and tolerances are unchanged.

## 1. Scope of Exact Arcâ€“Arc

This contract applies to two bounded planar circular arcs with a circular
profile and no tangent extension:

- both path curves are the original bounded arcs;
- both source surfaces are the exact Torus bands on those original domains;
- the profile is the same circular profile on both sides;
- no extension surface exists.

## 2. Mathematical existence condition

For a fixed profile angle `theta`, let `rho_i(theta)` be the offset-circle
radius on arc `i`. A valid common seam point requires

```text
center1 + rho1(theta) radial1(u1)
  = center2 + rho2(theta) radial2(u2)

u1 in D1, u2 in D2
```

with both points inside the original angular domains. Only same-theta
intersections are admissible. A generic Torus/Torus intersection may contain
different profile phases and is not a valid miter boundary.

A full seam exists only if every profile sample has a continuous branch of such
points. Pointwise existence does not imply a continuous boundary.

## 3. Oracle result

The independent circleâ€“circle oracle (`tests/arc_arc_oracle.rs`) sampled 68
profile angles without using the rail solver:

```text
missing intersections:                         0
outside both original arc domains:            33
multiple branches:                             0
branch discontinuities:                        0
first outside sample:                         35
```

Therefore the current Open Arcâ€“Arc case has no full exact common boundary on
the original bounded arcs. This is a geometric limitation, not a Torus or
Boundary Solver numerical failure.

The `33/68` number is a sampled diagnostic, not a proof of the exact continuous
uncovered interval. A later critical-angle analysis must refine it.

## 4. Mandatory typed refusal

The Exact-source behaviour is:

```text
UncoveredProfile / OutsideOriginalArcDomain
```

with no fallback to:

- tangent extension;
- `HybridAtJunction` for this case;
- Level B approximation;
- a different boundary strategy;
- a Direct B-rep attempt with an unvalidated seam.

Diagnostics must retain at least:

- first failing profile angle/parameter;
- number of covered and uncovered samples;
- covered and uncovered angular ranges;
- reason: no circle intersection, outside original arc domain, degenerate
  offset circle, multiple branch or branch discontinuity;
- both arc domains and both source UV domains.

## 5. Proposed refusal API

Names are provisional; the final API should preserve the reason:

```rust
enum ArcArcRefusal {
    UncoveredProfile { first_theta: f64, covered: usize, uncovered: usize },
    OutsideOriginalArcDomain { first_theta: f64 },
    NoCircleIntersection { first_theta: f64 },
    DegenerateOffsetCircle { first_theta: f64 },
    MultipleBranches { first_theta: f64 },
    BranchDiscontinuity { first_theta: f64 },
}
```

The refusal must be distinguishable from a numerical solver error and from a
Direct B-rep topology error.

## 6. Acceptance gate

`boundary_validation::validate_joint_boundary` remains the independent gate. A
candidate is accepted only when every sampled seam point:

- lies on both bounded source surfaces;
- lies in both UV domains;
- is continuous with its neighbours;
- has matching shared vertices and a closed seam loop.

`HybridAtJunction` may remain available for sources that actually have the
required extension surfaces, but its result must pass this gate. It is not an
Exact Arcâ€“Arc strategy.

## 7. Separate future contracts

These are separate constructions and must not be mixed into `ArcArcMiter`:

| Strategy | Contract |
|---|---|
| Explicit extension surface | builds an explicitly owned extension geometry; seam must lie on it |
| Adaptive cut / bridge | changes the piece boundaries and adds documented bridge geometry |
| Exact source | only original bounded surfaces; refuses when no common boundary exists |

## 8. Status

| Component | Status |
|---|---|
| Exact Torus geometry | PASS |
| Indexed profile correspondence | PASS |
| Open Arcâ€“Arc exact boundary | NOT PRESENT |
| Open Arcâ€“Arc typed refusal | REQUIRED |
| ArcArcMiter implementation | NOT STARTED â€” and not appropriate for this case |
| Boundary Solver | unchanged |
| Direct B-rep Builder | unchanged |
| Tolerances | unchanged |

## High-level integration (2026-11-06)

uild_planar_sweep now validates every solved boundary with alidate_joint_boundary before uild_direct_chain_surfaces. A validation failure is
PlanarSweepError::BoundaryValidation(...); the Direct builder is not called.

The open Arc–Arc test is no longer ignored: high_level_open_arc_arc_returns_typed_refusal asserts a typed validation refusal.

Observed refusal:

`	ext
boundary validation: seam sample 1 of segment 2 is not on surface 0: gap=2.1086334742979336e-2
`

Existing high-level tests remain positive:

`	ext
Exact Line–Line–Line     max_gap=8.854e-7   PASS
Exact Line–Arc–Line      max_gap=7.136e-8   PASS
Mixed Exact/Level B      max_gap=1.320e-6   PASS
`

