# 25 — Cap boundary consistency audit

Status: the current blocker is a **contract mismatch between the side-surface U
boundary and the cap plane**, not a UV trim or endpoint defect.

## 1. Measurements

| Case | Original start | Prepared path-piece start | Cap-plane deviation |
|---|---|---|---|
| line–arc | `(0,0,0)` | `(0,0,0)` | `1.5e-17` |
| line–arc end | `(0,1,0)` | `(0,1,0)` | `8.1e-17` |
| arc–arc | `(-1,1,0)` | `(-1.0000000000, 1.7071067812, 0)` | `0.7071067812` |
| arc–arc end | `(-1,1,0)` | `(-1.7071067812, 1.0000000000, 0)` | `0.7071067812` |

For arc–arc, the side-surface U boundary is at the **prepared path endpoint**,
which is shifted by `0.7071067812` from the original path endpoint. The cap
plane is built at the **original** endpoint. The cap profile edge therefore
misses the cap plane by `0.7071` over its full curve; endpoint deviations are
zero.

The cap-face trim/tolerance path then raises edge and vertex tolerances by
`2 * 0.7071067812 = 1.4142135624`.

## 2. Root cause

`build_direct_chain` mixes two path boundaries:

- side surface: built over `NormalizedPlacement::path_piece`;
- cap plane: built from the original `path_start` / `path_end`.

These agree for line–arc in this configuration, but not for arc–arc. The cap
profile edges are extracted from the side surface U boundary, so they are on the
prepared boundary, not on the cap plane.

## 3. Intended cap semantics

Two consistent choices exist:

- **A.** The prepared/extension boundary is the final boundary: build the cap
  plane from the actual side-surface U boundary.
- **B.** The original path endpoint is final and the prepared boundary is
  internal: trim the side surface to the original endpoint before extracting
  cap edges.

The current Direct B-rep Builder does neither; it takes the side-surface
boundary for the profile edges and the original path endpoint for the plane.

The production preprocessing semantics must decide A or B. This audit does not
choose a fix yet.

## 4. Passing comparison

- line–arc: prepared and original endpoints coincide; cap-plane deviation is
  essentially zero; cap face passes.
- arc–arc: prepared endpoints are offset by `0.7071`; cap-plane deviation is
  `0.7071`; tolerance grows to `1.4142`.

This is the first divergence between passing and failing cases.

## 5. Next

Trace `normalize_planar_sweep` / `prepare_piece` and production
`create_curve_sweep` to decide whether the prepared boundary or the original
path endpoint is the intended cap boundary, then make the cap plane and cap
profile edges use that one boundary consistently.

Do not adjust tolerances, UV trims or Boundary/Surface geometry.
