# AOIM Line–Arc — independent geometric research

Experimental Rust implementation of the **Analytical Offset-Intersection Miter (AOIM)** concept associated with [solidcore issue #447](https://github.com/tangency-labs/solidcore/issues/447).

This code is **standalone** and intentionally NOT part of the SolidCore kernel or its Rust↔C++ difftests.

## Geometry and conventions

The joint is the origin. The incoming segment travels along `+x` and is supported by `y = 0`. The next segment is supported by a circle of radius `R` through the origin with center

`C = (R sin α, R cos α)`.

The outgoing tangent at the joint is `(cos α, −sin α)`: the joint turns clockwise by `α`. Because `C` is on the left of that tangent, the arc itself curves counter-clockwise. The junction is therefore **inflected** (right kink, then left-curving arc), not a continuing-turn fillet. The opposite-curvature case is modelled separately by `ContinuingLineArc` in `src/continuing.rs`.

For signed offset `d` to the right of travel:

- incoming supporting line: `y = −d`
- circular supporting offset: `(x − R sin α)² + (y − R cos α)² = (R + d)²`

Eliminating `d` produces the parabola:

`x² − 2R sin(α)·x + 2R(1 − cos α)·y = 0`.

The implemented local root is

`x = R sin α − sqrt(4R sin²(α/2) · (R cos²(α/2) + d))`,  `y = −d`.

The half-angle form is algebraically identical to the old square root but avoids cancellation in `1 − cos α`.

## Established domain facts

**Existence.** A real intersection exists iff

`d ≥ d_min = −R (1 + cos α) / 2 = −R cos²(α/2)`.

At `d = d_min` the two offset curves are tangent at the parabola vertex `(R sin α, R cos²(α/2))`; below it there is no real intersection.

**Finite segments.** The branch selected by the minus sign behaves as follows:

| offset | supporting line | supporting arc |
|---|---|---|
| `d > 0` | incoming ray, `x < 0` | finite arc span, `0 < φ − φ_start < α` |
| `d = 0` | joint `(0,0)` | joint |
| `d_min < d < 0` | forward extension, `x > 0` | extension before the joint |

So the finite-body seam exists only for `d ≥ 0`; `d < 0` requires extending **both** segments and is available only on the small interval `(d_min, 0)`.

**Sharp corners.** As `α → π`, `d_min → 0⁻`. For `α = 150°`, `d_min ≈ −0.067 R`. A symmetric profile spanning `[−0.5R, 0.5R]` therefore has no AOIM seam point for about 43 % of its offsets. This is a genuine coverage limit for this branch, not a rounding error.

**Tangent.** The miter tangent at the joint is `cot(α/2)`, the *external* bisector of the incoming and outgoing tangents. The internal bisector is `−tan(α/2)`. The test suite checks both slopes against the two geometric bisectors rather than against the parabola's derivative.

## Hybrid at the junction

When AOIM has no point (`d < d_min`), the proposed fallback is a **tangent continuation of the arc offset at the joint**, not an arbitrary straight line.

The arc-side joint point and tangent are

`Q0(d) = (−d sin α, −d cos α)`,  `u = (cos α, −sin α)`.

The tangent continuation meets the line offset `y = −d` at

`Q(d) = Q0(d) + t u`,  `t = d tan(α/2)`,

which simplifies to

`Q(d) = (−d tan(α/2), −d)`.

For `d < 0`, `t < 0`: the extension is **backward** along the arc tangent, into the corner. For `d > 0`, `t > 0`: the continuation runs forward into the arc and is not used here.

The hybrid seam is therefore

- AOIM curve for `d ≥ 0`;
- straight external bisector `Q(d)` for `d < 0`.

Both branches pass through `P` and have the same tangent there, so the seam is **C1 (G1) at the junction**. Curvature jumps at `P`; G2 is not claimed.

The line body is extended forward to `x = Q_x(d)` and the arc body is extended backward along its tangent to `Q(d)`. Both extensions are real sweep trajectories, so this is a local change of the swept geometry, not just a longer miter.

## 2D region verification

`src/sweep.rs` builds, for a given profile `[d0, d1]`:

- the untrimmed line strip and finite arc sector (`build_untrimmed`);
- the trimmed line and arc regions, including the hybrid prolongations;
- the expected outer contour;
- a seam sampled from `d0` to `d1`;
- polygon areas, area balance, self-intersection, and a 2D raster comparison.

The raster metrics are `overlap`, `gap`, and symmetric-difference area between the union of the two trimmed parts and the expected outer contour. Area balance is `line + arc − expected`; it is independent of the raster resolution.

Representative results for `R = 4`, `α = 150°`, `d ∈ [−2, 2]`:

| construction | result |
|---|---|
| untrimmed bodies | overlap area ≈ `9.665` |
| original AOIM | **uncovered**: first missing `d = −2.0`; only ≈ 56.7 % of the profile has a seam |
| hybrid at junction | `line=83.024`, `arc=44.174`, `expected=127.198`; balance `+2.84e-14`; raster overlap/gap/sym `0`; no self-intersection |
| limit tangent | **uncovered**: the vertex tangent is horizontal and never meets `y = −d` below `d_min` |
| limit bisector | closes by area, but the connector is not tangent and its direction varies with `d`; rejected as a linear sweep continuation |

At `α = 90°` the original AOIM can be closed by circular extension for `d < 0`; the sampled polygon reports a tangential self-contact at `d_min`, while the hybrid has no self-intersection. At `α = 30°` the original AOIM also closes, and the hybrid agrees on a positive-only profile.

The hybrid remains balanced and non-self-intersecting for symmetric, positive-only, negative-only, wide (`d ∈ [−3.5, 3.5]`), near-collapse (`d0 = −0.9975R`) and shallow/steep profiles from `α = 1°` to `179°`.

## Counterexample: continuing-turn arc

The opposite curvature is in `src/continuing.rs`. It has center `(−R sin α, −R cos α)` and offset radius `R − d`, so its AOIM existence interval is the other side:

`d ≤ d_max = R cos²(α/2)`.

The missing side is now the **concave/inside** side (`d > d_max`). There, `t = d tan(α/2) > 0`: the only tangent intersection lies **forward** of the joint, and the backward tangent ray moves away from the line offsets. For a symmetric profile the forward extension also lies inside the line strip. Thus the hybrid-at-junction is not a retrograde prolongation for this curvature; it would introduce a cusp at `Q0` and/or overlap the line body.

This is a genuine counterexample to a universal “switch at `P`” rule. The admissible side is determined by the sign of `t(d)`: a tangent-linear hybrid is valid only where the uncovered offsets have `t(d) < 0`.

## Tests

```sh
cd research/aoim-line-arc
cargo test
```

The suite currently has **33 tests**. They cover:

- the original AOIM algebra, independent bisection, focus/directrix, existence limit, tangency, scaling, and invalid inputs;
- hybrid seam construction, tangent parameter sign, C1 continuity at `P`, and boundary sharing;
- untrimmed overlap, 2D area balance, raster gap/overlap/symmetric difference, and self-intersection;
- the `α = 150°` uncovered-profile counterexample;
- the limit-tangent and limit-bisector negative controls;
- wide, near-collapse, shallow, and steep profiles;
- the continuing-turn counterexample (`d_max`, forward tangent parameter, backward-ray distance, extension inside the line body).

The original checks that substituted `P(d)` back into the parabola were **self-confirming**: both sides came from the same two offset equations. They are kept, but the independent bisection, geometric bisector, and 2D region checks are the ones that can detect a branch, sign, or prolongation error.

## Visualize and inspect metrics

```sh
cargo run --example visualize
cargo run --example metrics
```

`visualize` writes `out/index.html`, per-angle constructions, and four-panel comparisons:

- `out/aoim-vs-hybrid-30.svg`
- `out/aoim-vs-hybrid-90.svg`
- `out/aoim-vs-hybrid-150.svg`

Each comparison has: original construction, hybrid construction, original regions, and hybrid trimmed regions. Purple is the linear seam on the extensions; green is the AOIM seam on the finite side; blue/orange are the line/arc bodies. The `out/` directory is gitignored.

`metrics` prints the area/raster table and connector kinematics (`t(d)`, limit-connector tangent error and direction spread, internal-bisector error, continuing-turn parameters).

## Research status / next step

**Confirmed for the inflected Line–Arc model:** the tangent continuation at the joint is a valid local linear prolongation on the side where `t(d) < 0`; the hybrid seam closes both 2D regions with no area overlap or gap, C1 at `P`, and no self-intersection for the tested profiles.

**Refuted as a universal rule:** switching at `P` does not transfer to the continuing-turn (concave) side, because the required tangent continuation points forward, not backward. A limit switch with a non-tangent connector can close the area but is not a linear sweep continuation.

**Not yet validated:** Arc–Arc, spline segments, 3D frames, trimmed surface domains, B-rep topology/sewing, or performance relative to SMLib.

**Next:**

1. Add a curvature-sign parameter to the model and the region builder; choose the admissible hybrid side automatically from `sign(t(d))`.
2. For the continuing-turn side, compare alternatives: trim the line body instead of the arc, or allow a ruled (non-linear) local connector, and measure whether they are valid sweeps.
3. Extend the 2D region test to Arc–Arc and a moving frame.
4. Only then attempt two neighbouring B-rep pieces with shared topology.

**No upstream PR is intended until that evidence exists.**
