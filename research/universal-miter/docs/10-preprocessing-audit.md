# 10 — Preprocessing audit of the existing Planar Sweep

Scope: read-only audit of the input normalization used by
`create_sweep_along_planar_path()`. No implementation and no production change.
The goal is a common preprocessing contract for `Original` and the alternative
`create_sweep_along_planar_path2`.

## 1. Pipeline and call sites

```
create_sweep_along_planar_path          sweep/api_brep.rs:345
  └── PrimitiveCreation::create_sweep_along_planar_path
                                         planar_path.rs:265
        ├── as_bspline(path)             planar_path.rs:317  (private, p2.rs:49)
        ├── closed / periodic            planar_path.rs:339-342
        ├── extend_path_curves_with_kernel
                                         planar_path.rs:373-383 (p1.rs:193)
        ├── position_profile_curves      planar_path.rs:417-428 (p1.rs:299)
        ├── create_curve_sweep           planar_path.rs:435-444
        │     ├── prepare_curve_sweep    p3/geometry.rs:186
        │     │     └── bspline(profile) p3/geometry.rs:42 (private)
        │     └── create_sweep_surface   nurbs/b_spline_surface_n_lib/p1.rs:1683
        ├── mitre / BrepIntersect / GlobalCurveSolve
        │                                planar_path.rs:449-604
        ├── ProjectAndTrim               planar_path.rs:605-617
        └── union / stitch / classify    planar_path.rs:713+
```

`as_bspline` is **private** (`primitive_creation/p2.rs:49`); `bspline` is
**`pub(super)`** (`p3/geometry.rs:42`). The research crate cannot call either;
it can reproduce the same five-arm mapping from the public `Curve::kind()` and
the public `base()` methods.

## 2. Where conversion happens and which kinds are supported

`as_bspline` / `bspline` accept exactly:

| Kind | Base returned | Exactness |
|------|---------------|-----------|
| `BSpline` | clone of the `BSplineCurve` | exact |
| `Line` | `Line::base()` (`line.rs:518`) | exact degree-1 B-spline |
| `Ellipse` (and circle) | `Ellipse::base()` (`ellipse/p2.rs:941`) | exact rational quadratic; the module doc says the analytic ellipse is kept with its rational quadratic B-spline |
| `Parabola` | `Parabola::base()` (`parabola.rs:254`) | exact rational quadratic rebuilt from the analytic data (module doc) |
| `Hyperbola` | `Hyperbola::base()` (`hyperbola.rs:199`) | exact rational quadratic (module doc) |
| `Hermite`, `Conic`, `Projected`, `Iso`, `Offset`, `CrvInVolume`, `CrvOnSurf`, `TangentField`, `Composite` | `None` | unsupported |

Consequences:

- **Path:** `planar_path.rs:317` calls `as_bspline(path)`; an unsupported path
  kind is `InvalidInput` (`planar_path.rs:314-320`).
- **Profile:** `position_profile_curves` accepts any `Curve`; `create_curve_sweep`
  later filters the profiles through `bspline` (`prepare_curve_sweep`,
  `p3/geometry.rs:199`). An unsupported profile is **silently dropped**; if none
  remain, the sweep fails (`prepare_curve_sweep` has no B-spline profile).
- `as_bspline` is also used by `taper.rs`; that is a different primitive and not
  part of this contract.

## 3. Are the conversions exact?

`as_bspline` adds **no approximation**: it returns the curve's own B-spline base
(a clone). The exactness is therefore the exactness of the base:

- Line: exact degree-1 B-spline.
- Ellipse/Circle: exact rational quadratic B-spline (module doc; the analytic
  curve is kept with its rational base).
- Parabola/Hyperbola: exact rational quadratic rebuilt from the analytic data
  (module docs).
- B-spline: exact clone.
- The unported kinds are not converted at all.

The base may be **rational** (weights). The Level A `CurveSpec::BSpline` has no
weights, so a rational base cannot be converted to `CurveSpec::BSpline` without
losing the circle/ellipse. The kernel adapter must use `Curve::evaluate` for the
real curves (as planned), not a lossy conversion to `CurveSpec::BSpline`.

## 4. Parameter range, periodicity, direction, accuracy

The base clone preserves:

- control points, weights, knot vector, degree;
- the base's natural interval (`Curve::natural_interval`, `p2.rs:1131`);
- the B-spline parameter direction and periodicity;
- the geometric point set and its derivatives (same evaluation).

The `as_bspline` conversion does **not** carry:

- the `CurveKind` marker (Line/Ellipse/Parabola/Hyperbola);
- the owner, notification identity and attributes (`Curve::copy` is different);
- the analytic STEP parameterization of a conic (`Ellipse::step_interval`,
  `Parabola::step_interval`, ...); the B-spline natural interval is the one the
  Planar Sweep uses;
- the analytic kind's `inside_out` flag and any analytic overrides.

So the normalized path has the same geometry and the same B-spline interval as
the base, but the original analytic type is not recoverable from the normalized
`BSplineCurve` alone. The research should keep the original `Curve` (its
`CurveKind` and public base/analytic accessors) beside the normalized base.
For the Level A solver this is not required (it is type-agnostic and uses the
kernel evaluation); for analytic references and diagnostics it is useful.

## 5. Planarity, closedness, segmentation

### Planarity

- `compute_crvs_normal` (`curve/curve/p3.rs:297`) builds an **average normal**
  from sampled chord cross products. It is not a planarity predicate: a
  non-planar path can still yield a unit normal (or a zero normal, which becomes
  `(0,0,1)` unless the status fix is on). It explicitly errors only for a single
  linear curve.
- `Curve::is_planar` (`curve/curve/p6.rs:595`) returns a `Planar` result with a
  normal and deviation. The Planar Sweep uses it only inside the WR-407 frame
  (`prepare_curve_sweep`, `p3/geometry.rs:283-302`); it does not reject a
  non-planar path.
- Therefore the existing Planar Sweep **assumes** planarity. The alternative
  must add an explicit planarity validation (sample points to the fitted plane,
  tolerance `ZONE_TOL_3D`) and refuse, or confirm that the contract accepts the
  current average-normal behavior.

### Closed / periodic

`planar_path.rs:339-342`:

```
closed   = start_point.distance(end_point) < tol
periodic = closed && start_tangent.is_parallel_to(end_tangent, 2.0)
```

`tol = ZONE_TOL_3D`; the 2.0 is the parallel-angle tolerance used by the
existing code. A closed path has no end caps; the closing join trims the end of
the last and the beginning of the first. A periodic path is closed with matching
tangents. `extend_path_curves` uses the same flags for its end extensions.

### Segmentation

The input `path_curves` **are** the segments; the function does not split a
single path curve. It:

- extends each piece at a non-parallel join by `2 * sqrt(2) * max_dimension`
  (profile bbox), G1 straight (owner-fixed);
- builds one capped sweep per path piece;
- mitres at non-parallel joins;
- for a closed path, handles the closing join;
- unions/stitches/classifies the pieces into one B-rep.

The profile is not split at discontinuities in the Planar Sweep:
`create_curve_sweep` is called with `test_continuity = false`; the
`prepare_curve_sweep` split is skipped.

## 6. Profile preparation and local frames

Two preparation blocks:

1. **`position_profile_curves`** (public wrapper, `p1.rs:299`; private impl
   `pub(super)`, `:317`; legacy `:373-472`): copies the profiles; translates the
   profile bbox centre to the origin (`:405-410`); rotates the profile normal to
   `X` (`:419-430`); then places the profile at the path start with
   `x = tangent`, `y` from `MakeUnitOrthoVectors` (`:432-472`). The WR-660 flag
   performs the legacy placement in the **whole path's** plane frame and maps it
   back (`:329-370`).

   **Visibility caveat.** The Planar Sweep calls the private
   `position_profile_curves_impl` with the **whole** `paths` list
   (`planar_path.rs:417-428`). The public `position_profile_curves` wrapper calls
   the same impl with only the current path as `whole_path`. For the WR-660
   branch the two are not identical when the whole path determines a plane that
   a single segment does not (for example a non-XY straight segment followed by a
   turn). The alternative must reproduce the whole-path placement or use a
   public helper if one is added; it must not assume the public wrapper is the
   exact Planar Sweep placement.
2. **`prepare_curve_sweep`** (`p3/geometry.rs:186`): the function is `pub`, but
   its module `geometry` is **private** (`p3.rs:26`), and `PreparedCurveSweep`
   is private too. The research crate cannot call it. It filters the profiles
   to B-spline bases; optionally splits profiles (`test_continuity`, false for
   the Planar Sweep); no scaling; translates the path start to the origin;
   selects the profile reference (`move_profile_center_to_path`: bbox centre or
   the first profile point) and translates it to the origin; computes the
   start-tangent frame, with the WR-407 option replacing the frame `z` by the
   path plane normal; rotates the path and profiles; optionally orients the
   profile perpendicular to the path (false here); returns the prepared
   profiles/path and the inverse placements. The normalizer reproduces the same
   preparation as `PreparedPiece` (Planar Sweep options: no split, no scale, no
   perpendicular orientation).

The sweep surface then uses the prepared profiles/path. The production
`PreparedCurveSweep` is a **preparation** type; it does not build the surface or
the joint. It is private, so the research copies its fields into the local
`PreparedPiece`; no surface/joint code is copied.

`move_profile` in the public API is passed to `create_curve_sweep` as
`move_profile_center_to_path`; `position_profile_curves` already centres the
profile bbox at the path start, and `prepare_curve_sweep` selects the reference
that becomes the origin. The exact combined effect must be covered by a
comparison test; the audit does not change it.

## 7. Shared infrastructure vs the existing Planar Sweep algorithm

| Function / block | Classification | Research use |
|------------------|----------------|--------------|
| `Curve::kind`, `Curve::base` (`base` on Line/Ellipse/Parabola/Hyperbola), `Curve::evaluate` | geometry infrastructure | reuse |
| `as_bspline` / `bspline` mapping | normalization; **private** | reproduce the five-arm mapping from `kind`/`base` |
| `extend_path_curves_with_kernel` (`p1.rs:193`) | normalization; public | reuse |
| `position_profile_curves` (`p1.rs:299`) / `position_profile_curves_impl` | normalization; public wrapper, private impl; the Planar Sweep uses the impl with the whole path | reproduce the whole-path placement or use it through a public helper; the wrapper alone is not equivalent for WR-660 |
| `prepare_curve_sweep` (`p3/geometry.rs:186`), `PreparedCurveSweep` | preparation inside `CreateCurveSweep`; **private module/type**; no surface/joint | reproduce the prepared frame (`PreparedPiece`) or add a public helper |
| `compute_crvs_normal` (`p3.rs:297`) | geometric query (average normal) | reuse for the reference; add strict planarity separately |
| `Curve::is_planar` (`p6.rs:595`) | geometric query | reuse for strict planarity |
| `CompositeCurve::build_composites_from_curves` (`composite_curve/p1.rs:1159`) | profile grouping; public | reuse if the profile has several curves |
| `PrimitiveModel::build_new`, `new_brep`, `BrepConstructor`, `create_faces_with_curves`, `project_and_trim`, `stitch_faces`, `integrity::check`, `is_manifold_solid`, `is_outward_solid`, `mass::integrator` | low-level topology/validation | reuse for the alternative B-rep |
| `create_curve_sweep`, `create_sweep_surface`, `N_CreateSweepScale` | **existing Sweep geometry** | **forbidden for the alternative** |
| `planar_path` mitre (`:449-604`), trim (`:605-617`), union/classify (`:713+`) | **existing Planar Sweep algorithm** | **forbidden for the alternative** |
| `create_sweep_along_planar_path` | existing Planar Sweep | `Original` only |

The `ProjectAndTrim`, stitch and boolean are kernel tools, not the Planar Sweep
algorithm. The alternative's own surfaces and seam feed them; the alternative
does not use the existing mitre logic.

## 8. Proposed common preprocessing contract

Both implementations should obtain an equivalent normalized input. The research
proposes one normalizer (reusing the public functions; reproducing only the
private five-arm mapping):

```rust
pub struct NormalizedPlanarSweep {
    /// Original profile curves (kinds kept for analytic references).
    pub profiles_original: Vec<Curve>,
    /// Profile bases before placement (normalized B-splines).
    pub profiles_bspline: Vec<BSplineCurve>,
    /// Original path curves (kinds kept for diagnostics).
    pub path_original: Vec<Curve>,
    /// Path pieces after `as_bspline` and G1 extension.
    pub path_bspline: Vec<BSplineCurve>,
    /// Placed profiles and the local-frame preparation (reproduced; the kernel
    /// `PreparedCurveSweep` is private).
    pub prepared: PreparedPiece,
    pub closed: bool,
    pub periodic: bool,
    pub extension: f64,
    pub plane_normal: Option<Vec3>,
    pub cap_start: bool,
    pub cap_end: bool,
    pub cap_profile: bool,
    pub move_profile: bool,
}
```

Normalization steps, in the existing order:

1. validate non-empty profile/path and `cap_ends`; `tol = ZONE_TOL_3D`;
2. map each path through the `as_bspline`-equivalent (kind/base);
3. map each profile through the same mapping (keep the original curves);
4. `closed`/`periodic` from the first/last point and tangent;
5. compute `max_dimension` and extend the path pieces (G1, `2√2 *
   max_dimension`);
6. the whole-path profile placement (the `position_profile_curves_impl`
   behavior; the public wrapper is not equivalent for WR-660);
7. the reproduced `prepare_curve_sweep` preparation with the Planar Sweep options:
   `translational_sweep = false`, `move_profile_center_to_path = move_profile`,
   `orient_profile_perpendicular_to_path = false`, `move_to_profile = false`,
   `profile_at_path_start = false`, `test_continuity = false`, no scale;
8. the plane normal via `compute_crvs_normal` at the first mitre (not needed for
   a single straight path);
9. `cap_profile` via the same `profile_closes` logic and the confirmed WR-400
   flag;
10. keep the original `Curve`s beside the normalized bases.

The `Original` comparison calls the existing public
`create_sweep_along_planar_path` with the same inputs and flags; its internal
normalization is the same functions in the same order. The research should add
a normalization parity test: the prepared path/profiles/frame from the shared
normalizer must match what the existing function uses (compare points, intervals,
frame axes and placements; not the B-rep).

## 9. Key findings and risks

1. **Rational bases.** `as_bspline` returns a possibly rational `BSplineCurve`.
   The Level A `CurveSpec::BSpline` is non-rational; converting a circle/ellipse
   to it loses the geometry. The kernel adapter must evaluate the real curve
   through `Curve::evaluate`, not convert to `CurveSpec::BSpline`.
2. **Private helpers.** `as_bspline` and `bspline` are private. Reproducing
   their five-arm mapping is small and does not change the public API; do not
   copy more of `planar_path.rs`.
3. **Profile drop.** A non-B-spline profile is silently dropped. The alternative
   should reject unsupported profiles explicitly, or the comparison must use
   only B-spline-reducible profiles; the decision is open.
4. **Planarity.** There is no strict planarity refusal. `compute_crvs_normal` is
   an average normal; `Curve::is_planar` exists but is used only for the WR-407
   frame. The alternative should validate and refuse non-planar paths.
5. **Analytic kind.** The normalized base preserves the geometry but not the
   `CurveKind`, the analytic STEP parameterization, the owner/identity or the
   `inside_out` flag. Keep the original `Curve` beside the base for analytic
   references.
6. **`prepare_curve_sweep` visibility.** The function body is preparation only
   (no `create_sweep_surface`, no `N_CreateSweepScale`, no joint), but its module
   `p3::geometry` and `PreparedCurveSweep` are private. The research crate cannot
   call it. The normalizer reproduces the same preparation as `PreparedPiece`
   (the Planar Sweep options pass through the same transforms); the Gate 0 test
   compares the reproduced profile/frame with the production cap. The owner can
   add a public/gated helper later, but no public API change is made now.
7. **`move_profile` and the private placement.** The combined effect of the
   whole-path `position_profile_curves_impl` and `prepare_curve_sweep` must be
   covered by a parity test; the audit keeps the existing semantics. The public
   wrapper `position_profile_curves` is not a drop-in for the Planar Sweep's
   WR-660 placement. The cap helpers `profile_closes` and `cap_profile_ends`
   (`planar_path.rs:108-136`) are private and must be reproduced.
8. **Stale doc.** `create_sweep_along_planar_path`'s doc still mentions
   `Unported`/`SmBrep::SmBrep`; the body is implemented and the API test builds a
   real B-rep for circle-line. Record the actual status per matrix case.

## 10. Decisions requested

1. Confirm the shared normalization contract in §8 and the reuse list in §7.
2. Confirm the strict-planarity refusal for the alternative (recommended) and
   the profile policy (explicit rejection recommended).
3. Confirm keeping the original `Curve`/analytic kind beside the normalized
   base.
4. Confirm reproducing the private `prepare_curve_sweep` as `PreparedPiece`
   (the only option without a public API change), or expose a gated/public
   helper.
5. Confirm that `ProjectAndTrim`, stitch and boolean are allowed kernel tools
   while `create_curve_sweep`/`create_sweep_surface` and the existing mitre code
   are forbidden.

No Planar Sweep2 implementation starts before these decisions.
