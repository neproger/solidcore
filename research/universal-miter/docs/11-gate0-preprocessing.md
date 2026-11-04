# 11 — Gate 0: preprocessing parity (PASS)

Status: **Gate 0 is green.** The alternative normalizer reproduces the production
Planar Sweep preprocessing and shares the same normalized input. No swept
surface, joint or B-rep of the alternative is built. Production, the public API,
the Python validator and the baseline are unchanged.

Test command:

```sh
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml \
  --features solidcore --test gate0 -- --nocapture
```

**9 tests pass.** The report is
[`v0-gate0-preprocessing.txt`](../results/reports/v0-gate0-preprocessing.txt).

## 1. What is compared

The existing Planar Sweep is the production reference. Because its preparation
functions are partly private, the alternative normalizer
(`prototype/src/planar_sweep2.rs`) is compared with:

- the **production cap face** of
  `create_sweep_along_planar_path` (profile placement/orientation, cap
  configuration), extracted from the real B-rep;
- the **source-faithful** normalization steps (`as_bspline` mapping, extension,
  whole-path placement, preparation, cap closure, planarity), reproduced locally;
- the public single-segment wrapper for the WR-660 contrast.

The normalizer does not build a swept surface or a B-rep.

## 2. Gate 0 results

| Check | Result |
|-------|--------|
| Profile position: start cap vs normalized profile | `0.0` (cap face vertices vs the world profile corners) |
| Profile position: end cap vs translated profile | `0.0` |
| `move_profile = false` start cap vs normalized profile | `0.0` |
| Profile orientation | the cap corner set and cap plane normal; exact match |
| Plane normal | `Some((-0.7071068, 0, 0.7071068))` for the tilted path; sampled deviation `0.0` |
| Segmentation | 2 path pieces, 2 placements; original order kept |
| Extension | `2.828427` (`2√2 · max_dimension` of the rectangle) |
| Closed / periodic | closed square: `closed = true`, `periodic = false`, no caps |
| Caps | `cap_ends` 0/1/2/3; start/end cap presence; closed path has no caps |
| Rational weights | circle base rational; weights `[1, 0.7071…, 1, …]` preserved in the normalized and prepared profiles and in the production cap boundary |
| Tolerances | `ZONE_TOL_3D = 1e-5`; planarity at the same tolerance; non-planar path refused |
| Whole-path WR-660 | production cap vs whole-path normalizer `0.0`; vs single-segment wrapper `0.4279` |
| `prepare_curve_sweep` | preparation only; **private**; reproduced as `PreparedPiece` |
| Reproducibility | repeated runs identical (the deterministic Level A suite) |

The WR-660 result is the important one: the public single-segment wrapper does
**not** reproduce the production placement, while the whole-path normalizer
matches the production cap exactly. This confirms the audit's warning.

## 3. Findings

1. **`prepare_curve_sweep` and `PreparedCurveSweep` are private.** The function
   body is preparation only (no `create_sweep_surface`, no `N_CreateSweepScale`,
   no joint), but the module `p3::geometry` is private. The research crate cannot
   call it. The normalizer reproduces the Planar Sweep preparation as a local
   `PreparedPiece`; the Gate 0 tests compare its output with the production cap.
   This is the only option without a public API change; a gated/public helper
   can replace it later if the owner prefers.
2. **`as_bspline`/`bspline` are private.** The five-arm mapping is reproduced
   from the public `Curve::kind`/`base`; no approximation is added.
3. **The whole-path placement is private.** Reproduced; the public wrapper
   differs for WR-660 (0.4279), the production cap matches the reproduction
   (0.0).
4. **Rational bases are preserved.** The circle weights survive normalization,
   preparation and the production cap boundary. The kernel evaluation must be
   used; `CurveSpec::BSpline` is non-rational and remains unsuitable.
5. **Planarity is not a production predicate.** `compute_crvs_normal` is an
   average normal; the strict check uses 65 sampled points per path against the
   production normal and refuses past `ZONE_TOL_3D`. A non-planar path is
   refused; `Curve::is_planar` is reported for comparison.
6. **Caps.** The alternative uses the confirmed WR-400 reference: caps only for
   a closing profile. The cap face is identified in the tests by the plane
   normal and the nearest face centroid, because side faces can share the normal
   on a multi-segment path.

## 4. Gate 0 verdict

**PASS** for the shared preprocessing contract, with the documented
`prepare_curve_sweep` visibility caveat. The alternative may now build its own
swept surfaces, analytical joints and B-rep on this normalized input.

## 5. Proposed Sweep2 architecture (approval requested)

```
create_sweep_along_planar_path2
├── normalized input   Gate 0 `NormalizedPlanarSweep` (profiles/paths original
│                      and B-spline, prepared piece, closed/periodic, extension,
│                      plane normal, caps, tolerance)
├── curve adapter      `PlanarCurve` trait: `CurveSpec` (Level A) and
│                      `KernelCurve` (real Curve via `Curve::evaluate`)
├── path               extended pieces, path-plane frame, profile placement
├── surfaces           Sweep2's own swept surfaces: analytic sampling
│                      `P(v)+alpha(u)n(v)+beta(u)N`, NURBS fitting; no
│                      `create_sweep_surface`/`N_CreateSweepScale`
├── joints             Level A rail seam per join; `gamma × N`; trim with
│                      `project_and_trim` (kernel tool) or direct faces
├── B-rep              `PrimitiveModel::build_new`/`new_brep` +
│                      `BrepConstructor`/`create_faces_with_curves`; caps;
│                      stitch/union; orient
└── validation         closure, manifold, orientation, seam on both surfaces,
                       no self-intersection, volume, determinism
```

Rules:

- The alternative never calls `create_sweep_along_planar_path`,
  `create_curve_sweep`, `create_sweep_surface` or `N_CreateSweepScale`; the
  production Planar Sweep is only the `Original` in the comparison.
- `ProjectAndTrim`, stitch, boolean and the low-level B-rep tools are allowed.
- The Level A solver is reused through the `PlanarCurve` adapter; its algorithm
  and Gate 1 tests are unchanged.
- The first slice after approval: adapter, one joint (line–line, line–circle,
  symmetric/asymmetric arc–arc, line–Bezier, Bezier–Bezier), independent
  surfaces, the seam on both surfaces, `project_and_trim`, caps, B-rep
  validation, then `Original` vs `Alternative` on the same inputs.
- The full matrix then adds convex/reflex turns, scales, G1, folds, open/closed
  profiles, `cap_ends` 0..3, multiple joints and a closed path.

## 6. Decisions requested

1. Approve Gate 0 and the shared `NormalizedPlanarSweep` contract.
2. Approve reproducing the private `prepare_curve_sweep` as `PreparedPiece`
   (recommended) or ask for a gated/public helper.
3. Approve the architecture in §5 and the first sliced implementation.
4. Confirm that no production/public API change is made before the final gate.

No swept surface, joint or B-rep is started before this approval.
