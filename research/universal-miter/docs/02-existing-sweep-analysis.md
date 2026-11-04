# 02 — Existing Planar Sweep analysis

All references are to the Rust port (the C++ reference is not consulted here).

## Pipeline

`SmApiCreateSweepAlongPlanarPath` → `sweep/api_brep.rs:345` →
`PrimitiveCreation::create_sweep_along_planar_path`
(`crates/solidcore/src/primitive/primitive_creation/p2/planar_path.rs:265`).

1. **Inputs.** Every path must reduce to a B-spline: `as_bspline`
   (`primitive/primitive_creation/p2.rs:49-58`) accepts `BSpline`, `Line`,
   `Ellipse`, `Parabola`, `Hyperbola`; other `CurveKind`s are rejected.
2. **Closure.** `closed` / `periodic` from the first and last point and tangent
   (`planar_path.rs:339-342`).
3. **Extension.** `extend = 2*sqrt(2)*max_dimension` (`:369-370`); each piece is
   extended at a non-parallel join with `CreateExtendedCurve(.., SM_CT_G1, ..)`
   (`:373-383`, `p1.rs:209-276`). `SM_CT_G1` is **linear**
   (`nurbs/b_spline_curve_n_lib.rs:663-731`): a straight tangent extension.
4. **Profile placement.** Centre to origin, profile normal to `X`, then a frame
   with `x = tangent` (`p1.rs:317,373-472`). The profile plane is normal to the
   path.
5. **Sweep.** `CreateCurveSweep` (`p3/construct.rs:334`) builds
   `S(u,v) = T(v) + M(v) C(u)` through `N_CreateSweepScale`
   (`nurbs/srf_adv/p6.rs:171-186,449-463`):
   `x = T'/|T'|`, `z` from the projection-normal transport
   (`nurbs/srf_adv/p4.rs:117-166`), `y = z x x`. On a planar path the frame is
   `(t, n, N)` only with WR-407 (`p3/geometry.rs:272-302`).
6. **Mitre.** At a join with non-parallel tangents (`planar_path.rs:449-478`):
   ```
   average = normalize(-t_before + t_after)
   flip average if the ccw angle is negative (reflex corner)
   temp_cutting = line from the corner along average, extended backwards
   ```
   - both pieces linear → `temp_cutting` (`:485-490`);
   - otherwise `BrepIntersect` of the two extended solids (`:498-503`), one
     `GlobalCurveSolve` per intersection curve (`:521-545`), the two closest
     points (`:570-583`), a straight line through them (`:584-597`), else the
     temporary line (`:602-604`).
7. **Trim.** `ProjectAndTrim(cutting, direction = N)` (`:203-236,605-617`).
   It builds the **plane** spanned by the curve and `N`, projects it onto the
   faces, imprints and trims (`topology/brep/p10.rs:2095-2148`).
8. **Closed path.** The closing join is handled separately (`:628-709`); the
   pieces are then unioned (`:713+`).

## What the algorithm really is

The only variable part is the numerical `BrepIntersect` + `GlobalCurveSolve`
search. There is no formula for the miter. **The cut is always a plane.**

- `Line-Line`: the plane is the exact bisector; the current line is exact.
- Curved pieces: the plane is a chord through two sampled intersection points.
  It depends on the extension length, the intersection sampling and the solver
  tolerance, not only on the geometry.
- Subdividing a B-spline piece, reparameterising it, or changing the extension
  can change the result for the same point set.

## Why the current line is not arbitrary

For the mathematical model (see `03-mathematical-model.md`), the tangent of the
true seam at the corner is

```
X - O  ||  t_2 - t_1  =  -t_1 + t_2,
```

which is exactly the `average` at `planar_path.rs:456`. So the current line is
the **first-order tangent** of the true miter curve at the corner. It is exact
only when the curve is straight; otherwise the error is `O(alpha^2)`. The
independent validator measures this: `Line-Line` and the symmetric `Arc-Arc`
pair are on the line to `~1e-14`, the asymmetric `Arc-Arc` deviates by `5.6e-2`
at `alpha = 0.34`, and `Line-Arc` by `3.5e-1` at `alpha = 0.5`.

## Important consequence: the extension is part of the construction

Because the sweep is built on the **extended** pieces, the side of the seam
whose `alpha` runs past a piece end lies on the straight tangent extension, not
on the original curve. The validator's asymmetric `Arc-Arc` comparison:

```
straight (SolidCore): X(-0.4) = (0.400000, -0.400000), kappa(gamma) on alpha<0 = 0
natural continuation: X(-0.4) = (0.365349, -0.351488), kappa(gamma) on alpha<0 = -0.062901
difference: 5.96e-2
```

On `alpha > 0` both modes agree (`kappa = -0.177957`). The real seam therefore
has a curvature jump at `alpha = 0`. The owner must decide whether the intended
miter is the straight-extension one (current construction) or a
curvature-continuous continuation.

## Limits of the current construction

1. The seam is a plane though the true miter is generally a surface.
2. Two sampled points cannot represent the curvature of a curved seam.
3. No miter-limit check: the fold (`t1 || t2`) and the curvature cusp
   (`1 - alpha kappa = 0`) are not detected.
4. The result can change under a re-representation of the same geometry.
5. A failed `BrepIntersect` / `GlobalCurveSolve` falls back to the tangent line
   when `planar_path_sweep_statuses` is off, and is an error when it is on.
6. The API only accepts path kinds reducible to a B-spline; the "any curve"
   requirement is not met at the entry.
