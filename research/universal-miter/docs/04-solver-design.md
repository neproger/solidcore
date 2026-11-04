# 04 — Solver design

**Status: level A is implemented and passes Gate 1.** `MiterStrategy::Universal`
runs the rail solver. The Level A solver is the **joint component** of the
alternative **Planar Sweep** (`create_sweep_along_planar_path2`), not a
replacement for the general `create_curve_sweep`. `MiterStrategy::Original` is
the joint-level harness's placeholder; the operation-level `Original` is the
existing `create_sweep_along_planar_path` in the comparison (`09-sweep2-scope.md`).

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

**Level B — Planar Sweep2 prototype** (`solidcore_api`, feature `solidcore`):
the kernel-curve adapter and the independent Planar Sweep construction
(`create_sweep_along_planar_path2`). It converts the real curves through the
`PlanarCurve` adapter, builds Sweep2's own swept surfaces and joints, applies
the kernel trim/topology tools, and checks closure, manifold and volume. It is
off by default and never part of the production build. The scope and contract
are in `09-sweep2-scope.md`.

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

## Implementation (level A)

Modules and their responsibilities:

- `curve`: the common differential interface (`CurveSpec::differential`) for
  line, circular arc, quadratic/cubic Bezier and clamped B-spline. The B-spline
  derivative curves are built from the derivative control points; de Boor is
  iterative and bounds-checked. No miter code branches on the variant.
- `differential_geometry`: point, first/second derivative, tangent, left normal,
  signed curvature and speed; the 2D helpers.
- `rail_solver`: `Piece` (original curve plus straight G1 tangent extension), the
  damped fixed-alpha Newton, the two-direction continuation and the explicit
  stops; `solve_sequence` for uniform-grid invariance tests.
- `miter_curve`: the sampled seam, with the segment (original/start/end
  extension) for each piece.
- `validation`: the independent closest-point signed distances, the analytic
  residuals and the diagnostics.
- `harness`: `MiterStrategy`, `MiterTestCase`, `MiterResult`; `Universal` is
  implemented, `Original` is level B.

Implementation details:

- A piece parameter `s` maps affinely to the curve parameter; the extensions are
  the production policy: straight `SM_CT_G1`, `extension = 2 * sqrt(2) *
  max(profile alpha span, profile beta span)`.
- Newton tolerance `1e-13 * max(1, |corner|, |alpha|)`, determinant floor
  `1e-14 * scale^2`, damped line search with 24 halvings, at most 60 iterations.
- The continuation starts at `alpha = 0` with `v = end(P1)`, `w = start(P2)`,
  predicts with the ODE and corrects with the Newton. It traces negative and
  positive directions independently; a stop on one side does not abort the
  other.
- Stops: `Fold` (tangent cross below `1e-4`), `CurvatureCusp` (`|1 - alpha
  kappa|` below `1e-6`), `OutsidePiece`, `TangentJoin`, `ReachedAlphaBound`,
  `NoConvergence`. On a failed step the continuation halves the step down to
  `1e-10`; then it classifies the last successful state. No straight-line
  fallback exists.
- The `Fold` detection is also checked at every successful point; the two
  curvature factors are checked separately so a cusp is never reported as a
  fold. In the tested corner geometries the fold binds first; the cusp branch is
  covered by the `stop_classifier_distinguishes_cusp_fold_and_non_convergence`
  unit test.
- The independent check in `validation` uses a 256-sample closest-point search,
  golden-section refinement and Newton polishing, then the signed distance from
  the tangent at the closest point. It never uses the rail Jacobian.

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
used as proof of equivalence. Level A implements `Universal`; `Original` stays
`NotImplemented` until level B can run the production construction on the real
B-rep. The Gate 1 evidence is in `06-experiment-results.md`.
