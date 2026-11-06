# 29 — Surface Capability Audit

## Purpose

Decide whether the exact analytic path is usable before changing the Direct B-rep
Builder: can SolidCore create the required analytic surfaces, convert them to NURBS,
split them, and make faces directly on them without losing geometry?

Implementation: `crates/solidcore/tests/surface_capability_audit.rs`.

## Method

For `Plane`, bounded `Cone`/cylinder, `Torus`, `SurfOfExtrusion` and
`SurfOfRevolution`:

1. Create the analytic surface with its NURB.
2. Split at the middle of the NURB U domain and of the NURB V domain.
3. Assert the split pieces are `SurfaceKind::BSpline` (rational NURBS).
4. Compare the original and both pieces on a 17 x 17 grid over each piece's full
   NURB domain, in 3D.
5. Create a four-sided face directly on each analytic `Surface` with
   `make_face_with_curves`, then check its surface/loops.
6. Check a complete analytic primitive body (cylinder + two planes) with
   `check_brep` at `Level2`.

## Results

### Exact conversion and split

The original surface is evaluated through its NURB parameterization and each piece
through the split NURB domain. Maximum 3D deviation over the full grid:

| Surface             | U split max deviation | V split max deviation |
|---------------------|----------------------:|----------------------:|
| Plane               | `3.67e-15`           | `1.99e-15`           |
| Cylinder (`Cone`)   | `3.58e-15`           | `2.74e-15`           |
| Torus               | `0`                  | `0`                  |
| SurfOfExtrusion     | `1.83e-15`           | `1.83e-15`           |
| SurfOfRevolution    | `0`                  | `1.37e-15`           |

`split_at` converts every analytic input to rational NURBS pieces and preserves the
geometry to floating-point roundoff. Keeping the analytic type after a split is
therefore unnecessary.

### Face creation

`make_face_with_curves` creates a face directly from every tested analytic `Surface`.
Each face has one loop and retains the analytic surface kind (`Plane`, `Cone` with
`is_cylinder`, `Torus`, `SurfOfExtrusion`, `SurfOfRevolution`). The same test also
creates faces from the split NURB pieces (both halves of every surface), where the
face keeps `SurfaceKind::BSpline`.

A detached single face is not a valid complete B-rep: `check_brep` reports missing
shell/face-owner/UV-trim-owner structure. This is a topology-assembly limitation of
the audit fixture, not a surface or geometry failure.

A complete analytic primitive body — cylinder + two plane caps — passes
`check_brep` at `Level2` with zero failures. Its faces are `["Cylinder", "Plane",
"Plane"]`.

## Conclusion

The exact analytic path is viable:

```text
analytic sweep -> analytic Surface -> split_at -> rational NURBS pieces -> face
```

No Level B approximation is needed for `Plane`, `Cone`/cylinder, `Torus`,
`SurfOfExtrusion` or `SurfOfRevolution` when their geometry is available. The
Direct B-rep Builder should accept `Surface`, not only `BSplineSurface`: the current
restriction is artificial.

## Next

Stage B: build analytic sweep surfaces for line + circular profile and
arc + circular profile, split them into vertical patches, and feed those patches into
the existing Direct B-rep Builder. Use Level B only for profiles with no analytic
surface representation.
