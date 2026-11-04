# 01 — Problem definition

## Question

Planar Sweep joins two swept pieces at a path corner. The current construction
always cuts with a **plane**. For a curved path piece the true joint between the
two swept solids is generally a curved (extruded) surface, so the plane is only
an approximation.

The research question is:

> Is there one mathematical construction that produces the miter curve for any
> pair of parametric path pieces — line, arc, quadratic/cubic Bezier, B-spline,
> NURBS — without branching on the segment type, and can it be validated on the
> real SolidCore geometry without changing the production Sweep?

## Scope

- The joint between **two adjacent path pieces of one planar path**, in the path
  plane.
- A planar profile with material coordinates `(alpha, beta)`; `alpha` is the
  in-plane offset, `beta` the extrusion along the path-plane normal.
- The **WR-407 frame**: `x = tangent`, `z = plane normal`, `y = N x t`
  (`Kernel::fixes.sweep_frame_on_planar_path_normal`).
- No scaling sweep, no non-planar path, no arbitrary profile out of its plane.
- Closed paths are included at every join, including the closing join.

## Non-goals (this stage)

- Replacing the production `planar_path.rs` or changing the public API.
- Building B-rep topology beyond what the validation needs.
- A UI, SolidCraft integration, or a new public kernel operation.
- Solving tangential or degenerate joins that have no well-defined miter:
  those are refusals, not fallbacks.

## Definitions

- **Original strategy**: the production `CreateSweepAlongPlanarPath` +
  `ProjectAndTrim` plane cut.
- **Universal strategy**: the rail-based miter curve from
  [`03-mathematical-model.md`](03-mathematical-model.md).
- **Rail**: `R_{i,alpha}(v) = P_i(v) + alpha n_i(v)`, the signed-offset curve of
  path piece `i`.
- **Seam / miter curve**: the locus of the same material `alpha` on both rails,
  `G(v, w, alpha) = 0`, projected to the path plane and extruded along `N`.
- **Fold**: the two rails become tangent (`t1 || t2`); the same-`alpha`
  intersection disappears. This is the practical miter limit.
- **Cusp**: `1 - alpha kappa = 0`; the offset rail loses regularity. It is a
  different event from the fold and, in the validated cases, binds later.

## Fixed constraints (proposed)

| # | Constraint | Reason |
|---|------------|--------|
| 1 | Arc = circular and elliptical; closed form only for circles | the rail equation is type-agnostic; the analytic reference is cheaper for circles |
| 2 | Arbitrary planar profile | the seam depends only on `alpha`; `beta` is a common translation along `N` |
| 3 | WR-407 frame only | without it the sweep frame is not `(t, n, N)` and the rail formula does not describe the sweep |
| 4 | Refusal is a correct outcome | fold, cusp, seam outside the piece, non-planar path: no fallback |
| 5 | First matrix: line, circle arc, quadratic Bezier, cubic Bezier; B-spline/NURBS as representation checks | four families give the 10 unordered pairs (including same-type) |

The owner must confirm these before the algorithm is implemented. The model
document records them as the assumptions of the proof.

## Success criteria

A run is not accepted because "the curve was built". The acceptance checks are:

1. **Rail residual**: `|G(v, w, alpha)|` at every sample.
2. **Independent distance**: `d1(X) = d2(X) = alpha` by closest-point
   projection, not by the rail normal.
3. **Analytic reference** where one exists:
   - `Line-Line`: bisector line;
   - `Line-Circle`: parabola;
   - `Circle-Circle`: quartic / Cartesian oval.
4. **Invariance**: splitting a piece at an interior parameter, affine
   reparameterisation, and an exact B-spline representation of the same point
   set must not change the seam.
5. **B-rep checks** at level B: the two trimmed pieces unite into a closed,
   manifold solid; the volume is consistent; the seam lies on both swept
   surfaces.
6. **Stability and cost**: the fold/cusp event is detected before the solver
   produces a wrong point; the run time is recorded.

Equal face/edge counts are not a pass criterion.

## Comparison metrics

Both strategies run the same `MiterTestCase` and report, independently:

- geometry: point-set distance to the reference, seam length, curvature;
- accuracy: residuals 1–3 above;
- topology: face/edge/vertex counts (reported, not used as proof);
- validity: closure, manifold, self-intersection;
- volume: analytic value where available, otherwise the trimmed-solid volume;
- stability: iterations, condition number, stop event;
- performance: wall time and allocations, reported separately from correctness.

## Deliverables of this stage

- The isolated module and the two-level prototype skeleton.
- The independent Python validator (unchanged from the research stage) and the
  immutable baseline report.
- The docs below. No production change and no algorithm implementation yet.
