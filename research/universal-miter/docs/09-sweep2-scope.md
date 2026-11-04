# 09 — Planar Sweep2: contract and architecture

Status: **proposal for the owner; no Planar Sweep2 implementation is started.**
Level A is audited and green (`08-level-a-audit.md`); the input-normalization
audit is `10-preprocessing-audit.md`. This document fixes the exact contract of
the existing Planar Sweep and the architecture of the alternative.
Implementation starts only after approval.

## 1. The correct comparison

The object of development is the **specialised Planar Sweep**, not the general
`create_curve_sweep`:

| Implementation | Role |
|----------------|------|
| `create_sweep_along_planar_path()` | existing SolidCore Planar Sweep |
| `create_sweep_along_planar_path2()` | our independent analytical Planar Sweep |

The general `create_curve_sweep()` is **out of scope**: not the object of
replacement and not the `Original` of this comparison. Spatial paths are out of
scope; planarity is part of the operation's contract.

`create_sweep_along_planar_path2` must:

1. accept the same input contract as the existing Planar Sweep;
2. return a compatible `Brep`;
3. build its own swept surfaces, analytical joints and final topology;
4. never call the existing Planar Sweep as a fallback or use its construction
   algorithm;
5. use low-level SolidCore geometry/topology tools where appropriate;
6. leave the production implementation and public API unchanged at this stage.

Level A is the **joint component** of this Planar Sweep, not a replacement for
any Sweep operation.

## 2. Exact contract of the existing Planar Sweep

### 2.1 Public signature

`crates/solidcore/src/sweep/api_brep.rs:345`:

```rust
pub fn create_sweep_along_planar_path(
    k: &mut Kernel,
    profile_curves: &[&Curve],
    path_curves: &[&Curve],
    move_profile: bool,
    cap_ends: i32,
) -> Result<Brep>
```

The conceptual `(profile, path, options)` is this signature: the path is
`path_curves`, the profile is `profile_curves`, and `move_profile`/`cap_ends`
are the options. The alternative must use the same signature and the same
return type.

### 2.2 Inputs

| Input | Contract |
|-------|----------|
| `k` | the kernel; the function creates a new B-rep in the model's context; the kernel is mutated; the input curves are cloned and not changed |
| `profile_curves` | non-empty; planar profile boundary; the profile is anchored by `move_profile` and placed normal to the path |
| `path_curves` | non-empty; ordered and endpoint-connected; planar; each must be B-spline-reducible |
| `move_profile` | `true`: the profile **center** is the anchor at the path start; `false`: the profile's **first point** |
| `cap_ends` | `0` none, `1` profile/start, `2` far end, `3` both; ignored for a closed path |
| `tol` | the API uses `ZONE_TOL_3D` |

Path kind restriction: `as_bspline`
(`primitive/primitive_creation/p2.rs:49-58`) accepts `BSpline`, `Line`,
`Ellipse`, `Parabola`, `Hyperbola`; a non-B-spline path is `InvalidInput`
(`planar_path.rs:314-320`). Profile kind: the profile placement accepts any
`Curve`, but the sweep preparation filters to B-spline-reducible profiles
(`prepare_curve_sweep`; the function is private, so the alternative reproduces
the filter); non-B-spline profile curves are currently dropped. That is a
contract ambiguity to confirm below.

### 2.3 Geometry and preparation

- **Closed / periodic.** `closed` when the first start point and last end point
  are within `tol`; `periodic` additionally when the first and last tangents are
  parallel (`is_parallel_to(..., 2.0)`).
- **Extension.** `extend = 2 * sqrt(2) * max_dimension` of the profile; each path
  piece is extended at a non-parallel join with `CreateExtendedCurve(..,
  SM_CT_G1, ..)`, a **straight tangent** extension. The owner fixed G1 for v1.
- **Profile placement.** `position_profile_curves` (public,
  `primitive_creation/p1.rs:291`): centre to origin, profile normal to `X`, then
  a frame with `x = tangent` at the piece start. The WR-660 flag places the
  profile in the whole path's plane frame; straight paths use the legacy
  placement.
- **Sweep.** `CreateCurveSweep` with `orient_profile_perpendicular_to_path =
  false`, `profile_at_path_start = false`, `move_to_profile = false`; the surface
  is `S(u,v) = T(v) + M(v) C(u)` with the WR-407 frame `(t, n, N)` when the flag
  is on.
- **Mitre.** At a non-parallel join, the current implementation cuts with a
  plane: a temporary line along `normalize(-t_before + t_after)`, or a line
  through two sampled `BrepIntersect` points, then `ProjectAndTrim` with
  `direction = N`. This is the algorithm the alternative replaces.
- **Caps.** `cap_start` / `cap_end` from `cap_ends`; a closed path has no end
  caps; `cap_profile_ends` (WR-400) requests caps only for a closing profile when
  `planar_path_uncapped_open_profiles` is on; otherwise caps are always requested
  and the failure is ignored (upstream).
- **Result.** One capped sweep per path piece, mitred, united, stitched and
  classified into one `Brep`.

### 2.4 Kernel flags that are part of the contract

| Flag | Effect | Proposed reference |
|------|--------|--------------------|
| `sweep_frame_on_planar_path_normal` (WR-407) | frame `z = N`; the Level A model assumes it | **on** |
| `planar_path_sweep_statuses` | mandatory statuses propagate; temporaries released | **on** |
| `planar_path_profile_placement_statuses` | placement failures propagate | **on** |
| `planar_path_profile_in_path_frame` (WR-660) | profile placement in the path plane frame | **on** |
| `planar_path_uncapped_open_profiles` (WR-400) | cap only a closing profile | **on** |

The `Kernel::fixes` defaults are mostly off; the reference set above is the
proposed comparison configuration. `Original` and the alternative run with the
same `Kernel` flags.

### 2.5 Output, ownership and errors

- Output: `Result<Brep>`; one B-rep, built by `PrimitiveModel::build_new`
  (`primitive_creation/model.rs:162`), with `refresh_cached_topology` and the
  result owned by the caller.
- `InvalidInput` for empty profile/path or `cap_ends` outside `0..=3`; a
  non-B-spline path is `InvalidInput`; the internal `SM_ERR` for a failed union,
  stitch or classification.
- The public doc still says [`SolidError::Unported`] (`SmBrep::SmBrep`, until the
  B-rep constructor is ported). The body is implemented; the sentence is stale.
  The research will verify the actual status on each matrix case and record a
  deferred case if an unported callee is reached.

### 2.6 Ambiguities to confirm before implementation

1. **Non-B-spline profiles.** The current preparation drops them; should
   Sweep2 reproduce the drop or reject them explicitly? Recommendation: reject
   explicitly with `InvalidInput`, and use only B-spline-reducible profiles in
   the comparison.
2. **Non-planar paths.** The current function assumes a plane; should Sweep2
   validate planarity and refuse? Recommendation: yes, explicit refusal.
3. **Straight path.** A single straight path has no plane normal and no joint;
   Sweep2 builds one sweep and the requested caps, no seam.
4. **Reference flags.** Confirm the set in §2.4, in particular WR-407 on.
5. **Cap behaviour for open profiles.** Confirm the WR-400 reference (cap only a
   closing profile) vs the upstream always-request behaviour.

The exact conversion, parameter, planarity and segmentation findings, and the
proposed common preprocessing contract, are in `10-preprocessing-audit.md`.

## 3. The alternative `create_sweep_along_planar_path2`

Research signature (same contract; the function lives in the research crate,
not in the kernel's public API at this stage):

```rust
pub fn create_sweep_along_planar_path2(
    k: &mut Kernel,
    profile_curves: &[&Curve],
    path_curves: &[&Curve],
    move_profile: bool,
    cap_ends: i32,
) -> Result<Brep>
```

- Same inputs, same `Brep` output, same cap/move/closed-path semantics, same
  tolerance and the confirmed flag set.
- Independent swept surfaces, joints and topology. It must not call
  `create_sweep_along_planar_path`, `create_curve_sweep`,
  `BSplineSurface::create_sweep_surface`, or `N_CreateSweepScale`.
- The `Original` in the comparison is the existing `create_sweep_along_planar_path`
  (public API), run with the same kernel and flags. It is not
  `create_curve_sweep`.

## 4. Architecture

```
create_sweep_along_planar_path2
├── contract        validate inputs, cap/move, closed/periodic, extension,
│                   plane normal, confirmed flags
├── curve adapter   real Curve/BSplineCurve; Curve::evaluate(k, t, 2, ..);
│                   project to the path plane; no duplicate B-spline/de Boor
├── path            extend pieces (G1), path plane frame, profile placement
├── surfaces        Sweep2's own swept surfaces; analytic for line/circle/plane,
│                   NURBS fitting for the general profile/path
├── joints          Level A rail seam for each join; seam extruded along N;
│                   trim with project_and_trim (kernel tool) or direct faces
├── B-rep           PrimitiveModel::build_new/new_brep + BrepConstructor or
│                   create_faces_with_curves; caps; stitch/union; orient
└── validation      closure, manifold, orientation, seam on both surfaces,
                    volume, determinism
```

### 4.1 Level A adapter without rewriting the solver

Introduce a small trait:

```rust
pub trait PlanarCurve {
    fn natural_interval(&self) -> Result<[f64; 2], GeometryError>;
    fn differential(&self, u: f64) -> Result<DifferentialData, GeometryError>;
}
```

- `CurveSpec` implements it (the existing Level A curves).
- `KernelCurve<'a>` implements it with `&'a Kernel` and the real `Curve`:
  `Curve::evaluate(k, t, 2, ..)`, projected onto the path-plane frame; the
  tangent, normal and signed curvature from the projected derivatives.
- `Piece<C: PlanarCurve>` keeps the same extension/parameter map; the
  `rail_solver` and `validation` become generic over the trait. The algorithm
  and the Gate 1 tests are unchanged; the kernel adapter is the new
  implementation.

### 4.2 Swept surfaces

For a planar path and the WR-407 frame:

```
S(u, v) = P(v) + alpha(u) n(v) + beta(u) N
```

Sweep2 samples this analytical surface (profile parameter `u`, path parameter
`v`) and fits a tensor-product B-spline within the approximation tolerance; exact
for line/circle/plane cases where the representation permits. It does not call
the existing `create_sweep_surface`/`N_CreateSweepScale`; the fitted surface is
Sweep2's own construction. The kernel's generic NURBS fitting/surface tools are
allowed.

### 4.3 Joints, caps, topology

- For each non-parallel join, the Level A rail solver produces the seam `gamma`
  in the path plane; the miter surface is `gamma x N`; the existing
  `project_and_trim` (public kernel tool) projects it with `direction = N` and
  trims both pieces. This is not the existing Planar Sweep's plane logic.
- Caps: the profile boundary at the start/end; `create_planar_faces_with_3d_curves`
  or `BrepConstructor`, honouring the confirmed cap contract.
- B-rep: `PrimitiveModel::build_new` (or `new_brep`) + `BrepConstructor` /
  `create_faces_with_curves`; `stitch_faces` or the public boolean for the
  union; `is_manifold_solid`, `integrity::check`, `is_outward_solid`, mass/volume,
  and determinism.

### 4.4 Allowed / forbidden reuse

Allowed (low-level tools): `Curve::evaluate`, `Curve::kind`/`base`,
`position_profile_curves`, `extend_path_curves_with_kernel`,
`compute_crvs_normal`, `Curve::is_planar`,
`CompositeCurve::build_composites_from_curves`, `BSplineSurface`
constructors/fitting, `BrepConstructor`, `new_brep`,
`PrimitiveModel::build_new`, `create_faces_with_curves`, `project_and_trim`,
`stitch_faces`, boolean/union, `integrity::check`, `is_manifold_solid`,
`is_outward_solid`, mass. The private `as_bspline`/`bspline` and
`prepare_curve_sweep`/`PreparedCurveSweep` are reproduced in the research
normalizer (`PreparedPiece`); see `10-preprocessing-audit.md` and
`11-gate0-preprocessing.md`.

Forbidden for the alternative: `create_sweep_along_planar_path`,
`PrimitiveCreation::create_sweep_along_planar_path`, `create_curve_sweep`,
`create_sweep_surface`, `N_CreateSweepScale`, `N_CreateTransSweepSrf`. The
`Original` comparison may call the public existing Planar Sweep; that is the
point of the comparison.

## 5. Public API audit (read-only)

The research crate is separate and can use only `pub` items. The audit found
enough public surface to build an independent B-rep without changing the kernel:

| Need | Public API |
|------|------------|
| New B-rep / model | `primitive::primitive_creation::model::PrimitiveModel::build_new`, `PrimitiveCreation`, `topology::brep::p3::new_brep` |
| Faces/loops/edges/vertices/shells | `topology::brep_constructor::BrepConstructor` |
| Faces from surfaces/curves | `topology::brep::p8::{create_face_from_surface, create_faces_from_surface, create_faces_with_curves, make_faces_with_curves, create_planar_faces_with_3d_curves}` |
| Trim | `topology::brep::p10::{project_and_trim, make_topology_from_data}`, `create_parallel_projection_curves` |
| Intersect | `topology::brep::p1::brep_intersect_mut` |
| Stitch / topology | `topology::brep::p4::stitch_faces`, `p9::{make_vertex_loop, make_edge_in_face_with_vu}` |
| Validity | `topology::brep::p2::is_manifold_solid`, `topology::integrity::check`, `topology::brep::is_outward_solid` |
| Mass / volume | `mass::integrator` |
| Curve evaluation | `curve::curve::Curve`, `CurveEvaluate`, `Curve::evaluate`, `BSplineCurve` |
| Surface construction | `nurbs::b_spline_surface::BSplineSurface` and its fitting tools |
| Existing Planar Sweep, for `Original` only | `sweep::api_brep::create_sweep_along_planar_path` |

The `BrepConstructor` is public, so no feature-gated production module is
required for the B-rep builder. The remaining risk is the constructor's
sequencing/status contract; that is a Level B integration test, not a public API
change.

## 6. Comparison harness and metrics

```rust
pub enum PlanarSweepStrategy {
    Original,     // existing create_sweep_along_planar_path
    Alternative,  // create_sweep_along_planar_path2
}
```

One `PlanarSweepCase` (profiles, paths, `move_profile`, `cap_ends`, the
confirmed kernel flags) runs through both strategies. Compare:

- geometry: seam/volume against the analytic or refined reference;
- B-rep: closure, manifold, orientation, self-intersection, face/edge counts
  (reported, **not** proof of equivalence);
- stability: fold/refusal on the same inputs;
- determinism: repeated runs;
- performance: wall time, measured separately from correctness.

The `Original` is the public Planar Sweep on the same `Kernel` flags. The
universal-miter `MiterStrategy` remains the joint-level Level A harness; it is
not the operation-level comparison.

## 7. Test matrix

Start with:

1. Line–Line.
2. Line–Circle.
3. Symmetric Arc–Arc.
4. Asymmetric Arc–Arc.
5. Line–Bezier.
6. Bezier–Bezier.

Then add: convex/reflex turns; different radii; scales `1e-3`/`1e3`; G1
extension; fold refusal; open and closed profiles; `cap_ends` 0..3;
`move_profile` true/false; several consecutive joints; a closed path; the
confirmed flag set. B-rep checks on every successful case; a documented refusal
is a pass for the refusal cases.

## 8. First slice after approval

1. Confirm the contract ambiguities and the reference flags (§2.6).
2. Add the `PlanarCurve` adapter trait and the kernel `Curve` adapter; keep the
   Level A solver and its tests unchanged.
3. Implement the input/path preparation (reuse the public low-level placement
   and extension, or reproduce the semantics).
4. Build the independent swept surfaces for one joint; verify the seam lies on
   both surfaces.
5. Compute the Level A seam; trim with `project_and_trim`; build the B-rep;
   validate closure/manifold/orientation/volume.
6. Add the `PlanarSweepStrategy` comparison and the first matrix cases.
7. Extend to several joints and a closed path.

No production integration.

## 9. Decisions requested

1. Confirm scope: Planar Sweep only; `Original` =
   `create_sweep_along_planar_path`; general `create_curve_sweep` out of scope;
   spatial paths out of scope.
2. Confirm the reference flags in §2.4 (WR-407 and the four Planar Sweep fixes).
3. Confirm the contract ambiguities in §2.6:
   - non-B-spline profiles: explicit rejection vs the current drop;
   - non-planar paths: explicit refusal;
   - straight path and cap behaviour;
   - the stale `Unported` note (record the actual status).
4. Confirm the architecture in §4, including the `PlanarCurve` adapter refactor
   and the low-level reuse list.
5. Confirm the research function location: `prototype`, `solidcore` feature,
   not the kernel public API at this stage.

No Planar Sweep2 implementation starts before this approval.
