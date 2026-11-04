# 07 — Integration plan (future, gated)

Nothing here is executed in this stage. The production `planar_path.rs` and the
public API stay unchanged until every gate below is green.

## Gate 0 — mathematical reference (done)

The Python validator and the model agree within the reach; the fold and the
extension policy are documented. See `06-experiment-results.md`.

## Gate 1 — Rust level A matches the Python reference

Implement the level A solver in `prototype/src` (no kernel dependency).
Acceptance:

- rail residual and independent signed distances at `<= 1e-12` absolute on the
  matrix in `05-test-matrix.md`;
- analytic `Line–Line`, `Line–Circle`, `Circle–Circle` within `1e-12`;
- subdivision, affine reparameterisation and B-spline-representation invariances
  within `1e-12`;
- the fold and cusp stop at the analytic `alpha`;
- both directions traced; a stop in one direction does not abort the other.

The Python validator stays the independent reference; do not port it line by
line.

## Gate 2 — level B on the real curve types

Convert `CurveSpec` to `solidcore::curve::curve::Curve` and evaluate point,
first and second derivatives with the kernel's `evaluate`. Build the same seam
and compare with level A. Acceptance: point-set difference within the kernel
tolerance (`1e-9` relative where the difftest rule applies).

The real types and calls to use:

- `solidcore::curve::curve::Curve`, `CurveKind`, `CurveEvaluate`;
- `solidcore::nurbs::b_spline_curve::p1::BSplineCurve`;
- `Curve::evaluate(k, t, 2, ..)` for the differential data;
- `global_point_solve` (curve) for the independent closest-point check;
- `PrimitiveCreation::create_curve_sweep` /
  `BSplineSurface::create_sweep_surface` to build the test sweep pieces;
- `project_and_trim` (`topology/brep/p10.rs`) with the traced curve and
  `direction = N`;
- `brep_intersect_mut`, `stitch_and_orient` and the existing mass properties for
  closure/volume checks.

These are the same mechanisms the production uses; do not duplicate them in the
prototype.

## Gate 3 — one join, real B-rep

Build two extended swept pieces for one join, trim both with the traced curve,
and check:

- the union is one closed, manifold solid;
- the seam lies on both swept surfaces within tolerance;
- the volume agrees with the analytic value where one exists, and with a
  refined reference otherwise;
- the residuals of `06` hold on the final B-rep faces;
- the run is deterministic and independent of the piece parameterisation.

Test scaffolding: add cases under `crates/solidcore/tests/kernel_cases/` only
when the implementation is ready; for the research stage, use fixtures in
`fixtures/`. Existing related tests:
`wr_407_swept_arc_walls_join_without_fold.rs`,
`wr492_mitred_cylinders.rs`, `issue_1450_curve_sweep_profile_placement.rs`,
`issue_1937_bottle_sweep.rs`.

## Gate 4 — full planar path

- several joins in a row, including two consecutive mitres;
- a closed path, including the closing join;
- open profiles (`planar_path_uncapped_open_profiles`) and caps;
- the WR-407 flag on and the documented out-of-model behaviour off;
- both the straight G1 extension policy and the owner-chosen alternative;
- refusals at the fold/cusp/domain limit, with no fallback.

## Gate 5 — production integration (separate PR, future)

Only after gates 1–4:

- put the result behind a `Kernel::fixes` flag (per `AGENTS.md` §4b), never a
  silent replacement;
- add a regression test and a `plan/upstream-issues/` addendum;
- keep the old path until the new construction is in and parity is shown, then
  delete the old path and the solidcore-only `WR-` patches;
- difftest: the reference for a redesigned construction is the SolidCraft/fork
  result, not the old plane; never loosen the `1e-9` tolerance.

## Risks

1. **Extension policy.** The current straight G1 extension changes the seam on
   one side of the profile. This is the first decision to fix.
2. **Fold predicate.** Testing only `1 - alpha kappa` misses the fold; the
   implementation must test `cross(P_1', P_2')` and the piece domain.
3. **B-rep imprint.** Passing a traced curve instead of a line to
   `ProjectAndTrim` changes the imprint; the closure/manifold checks must be on
   the real B-rep, not only on the curve.
4. **Representation.** The seam must be independent of the B-spline
   representation and knot insertion; the production path currently converts
   several kinds to a B-spline.
5. **Performance.** Continuation + trim may cost more than the two-point line;
   measure separately from correctness.

## Isolation checklist

- The root `Cargo.toml` is not modified; `research/universal-miter/Cargo.toml`
  is its own workspace.
- The default prototype build has no `solidcore` dependency.
- The production `planar_path.rs` is untouched.
- The Python validator and `results/reports/initial-validation.txt` are
  unchanged.
