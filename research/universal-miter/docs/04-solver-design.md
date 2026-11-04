# 04 — Solver design

This is the design of the solver to be implemented in the next stage. The
current prototype only fixes the interface; `harness::run_case` returns
`NotImplemented`.

## Two levels

**Level A — mathematical prototype** (`prototype/src`, no kernel dependency):

- `curve`: `CurveSpec` data for line, circle arc, quadratic/cubic Bezier,
  B-spline.
- `differential_geometry`: point, first/second derivative, tangent, left
  normal, signed curvature and speed at one parameter.
- `rail_solver`: the rail system and its trace.
- `miter_curve`: the sampled seam.
- `validation`: residuals and invariance report.
- `harness`: the same case through both strategies.

**Level B — SolidCore geometry prototype** (`solidcore_api`, feature
`solidcore`): converts `CurveSpec` to the real `Curve`/`BSplineCurve`, builds
the test sweep surfaces, runs the seam, applies `ProjectAndTrim`, and checks
closure, manifold and volume. It is off by default and never part of the
production build.

## Geometry

Let `P_i` be the extended path piece (original curve plus the straight G1
tangent extension at the join), `t_i` its unit tangent, `n_i = N x t_i` the left
normal, `kappa_i` the signed curvature (`t' = kappa n`). The material offset is
`alpha`, the extrusion along `N` is `beta`.

```
rail:       R_{i,alpha}(v) = P_i(v) + alpha n_i(v)
rail system: G(v, w, alpha) = P_1(v) + alpha n_1(v) - P_2(w) - alpha n_2(w) = 0
seam point:  X(alpha) = P_1(v) + alpha n_1(v) = P_2(w) + alpha n_2(w)
surface:     M = { X(alpha) + beta N }
```

In the reach `|alpha| < 1/|kappa_i|` the seam is also the branch through the
corner of `d1(X) - d2(X) = 0` (signed distances). The rail form is primary: it
keeps the material correspondence where the nearest-point distance no longer
describes the sweep.

## Newton at fixed alpha

```
d/dv (P_1 + alpha n_1) = (1 - alpha kappa_1) P_1'
d/dw (P_2 + alpha n_2) = (1 - alpha kappa_2) P_2'
J = [ (1 - alpha kappa_1) P_1' , -(1 - alpha kappa_2) P_2' ]
solve J [dv, dw]^T = -G
```

Damped step (halve while `|G|` does not decrease). Initial guess from the
previous `alpha` (continuation), not from a type-specific formula.

## Continuation in alpha

Differentiate `G = 0`:

```
(1 - alpha kappa_1) v' P_1' - (1 - alpha kappa_2) w' P_2' = n_2 - n_1
X'(alpha) = n_1 + (1 - alpha kappa_1) v' P_1' = n_2 + (1 - alpha kappa_2) w' P_2'
```

At `alpha = 0` this gives the tangent `t_2 - t_1` (the current line's
direction). Trace both directions from the corner with an adaptive step,
correcting with the Newton above. The branch is fixed by the continuation; the
zero set is regular at `alpha = 0` (`grad F = n_1 - n_2 != 0` for
`t_1 != t_2`).

## Stops and refusals

`det J` vanishes in two independent ways:

1. **Curvature cusp** `1 - alpha kappa_i = 0`.
2. **Rail fold** `cross(P_1', P_2') = 0`: the offset rails are tangent, so the
   same-`alpha` intersection disappears.

The validated cases show the fold binds first:

| case | fold | cusp | `1 - alpha kappa` at fold |
|------|------|------|---------------------------|
| line-circle (R = 1, 90 deg) | 0.5 | 1 | 0.5 |
| arc-arc (R = 1, symmetric) | `1 - sqrt(2)/2 = 0.292893` | 1 | 0.707107 |
| arc-arc (R1 = 1, R2 = 1.4) | 0.3398 | 1 | ~0.66 / 0.76 |

A stop is a **refusal** (`MiterStatus::Refused`), not a fallback. Other stops:
seam outside the extended piece, non-planar path, undefined plane normal,
scaling sweep, non-unique closest point (self-intersecting path).

## Extension policy

The model uses the **extended** pieces, because the swept solid is built on
them. SolidCore's extension is a straight G1 tangent. On the side of the profile
whose `alpha` runs past a piece end, the seam therefore lies on a straight line
and the seam curvature jumps at `alpha = 0` (see `02` and `06`). The solver must
record which piece segment (original or extension) produced each foot point;
the owner must confirm the policy before implementation.

## Diagnostics

Every run reports, per sample: `|G|`, `|d1 - alpha|`, `|d2 - alpha|`, distance
to the current line (the tangent at the corner), and the segment of each foot
point. The trace reports the first stop, the minimum `|det J|`, the Newton
iteration count and the number of samples. Level B adds closure, manifold,
volume, B-rep counts and time.

## Interface

`harness.rs` fixes the shared types:

```rust
pub enum MiterStrategy { Original, Universal }

pub struct MiterTestCase {
    pub id: String,
    pub curve_a: CurveSpec,
    pub curve_b: CurveSpec,
    pub profile: ProfileSpec,
    pub expected: Option<ExpectedGeometry>,
}

pub struct MiterResult {
    pub strategy: MiterStrategy,
    pub status: MiterStatus,       // Ok | Refused(..) | NotImplemented
    pub curve: Option<MiterCurve>,
    pub diagnostics: MiterDiagnostics,
}
```

The **same** case is run through both strategies. Topology is reported but not
used as proof of equivalence. The current stage only wires the interface.
