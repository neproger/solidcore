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

## Adaptive path regularization

`AdaptivePathAOIM` is a third, independent strategy. It does not replace either
previous construction.

The original corner `Line -> P -> Arc` is replaced by

`Line -> A -> straight tangent -> Q -> remaining arc`,

where `A` is a new line end to the rear of `P` and `Q` is a tangency point on
the original arc. The straight segment `A -> Q` is tangent to the circle at
`Q`, so the junction at `Q` is G1. The junction at `A` is a genuine line–line
corner; its miter is straight.

For a chosen turn angle `theta` at `A`:

- `A = (-s, 0)`,
- `s = (cos(theta) C_y - sin(theta) C_x - R) / sin(theta)`,
- tangent length `t = (C_x + s) cos(theta) + C_y sin(theta)`,
- `Q = A + t (cos(theta), sin(theta))`.

The removed arc angle is `arc_parameter(Q)`; the remaining arc is the original
arc from `Q` to its end.

The solver scans `theta` from the original angle towards zero and selects the
**least path deviation** candidate that satisfies both bounds. It rejects a cut
if the straight-body quadrilateral `seam -> Q0(d0) -> Q0(d1) -> seam` is
self-intersecting; this is important for moderate angles, where a too-small cut
would make the line–line seam cross the `Q` cross-section. It also reports
infeasibility rather than returning such a cut.

Parameters:

- `max_miter_length`: maximum straight line–line miter chord across the profile.
- `max_path_deviation`: maximum Hausdorff deviation between the original local
  path `A -> P -> arc(P..Q)` and the new segment `A -> Q`.
- `preserve_endpoints`: the far line start and far arc end are preserved; the
  original joint `P` is intentionally removed. The straight-transition family
  always satisfies this by shortening both segments; `false` does not enable
  moving the far endpoints.
- `preserve_tangency`: cannot be satisfied by a straight transition, because
  `A` is a G0 corner. Returning `TangencyRequired` is deliberate.

If the untouched line–line miter already fits `max_miter_length`, the solver
returns `NoCutNeeded` instead of making a cosmetic cut.

Comparison for `R = 4`, profile `[-2, 2]` (`chord` = miter length across the
profile, `extent` = maximum distance from the corner, `dev` = path deviation):

| alpha | line length | analytical | hybrid chord / extent | adaptive chord / extent | dev | removed arc | hybrid -> adaptive area |
|---|---:|---|---|---|---:|---:|---:|
| 30° | 20 | closed, chord 4.153 | 4.127 / 2.071 | 4.092 / 2.046 | 0.019 | 5.61° | 88.382 -> 88.359 |
| 90° | 20 | closed, chord 6.928, self-touch | 5.420 / 2.828 | 4.898 / 2.449 | 0.230 | 19.52° | 105.358 -> 104.378 |
| 150° | 20 | uncovered below d_min | 11.971 / 7.727 | 6.495 / 3.247 | 1.260 | 46.03° | 127.198 -> 114.290 |
| 170° | 100 | uncovered below d_min | 28.129 / 22.947 | 6.498 / 3.249 | 2.445 | 65.98° | 479.510 -> 431.513 |
| 175° | 500 | uncovered below d_min | 51.277 / 45.851 | 6.498 / 3.249 | 2.779 | 70.98° | 2125.470 -> 2030.361 |
| 179° | 500 | uncovered below d_min | 234.799 / 229.186 | 6.492 / 3.246 | 3.059 | 75.07° | 2492.150 -> 2029.298 |

For all primary adaptive cases above, the two/three trimmed parts close with
`line + parts - expected ~ 1e-15`, raster overlap/gap/symdiff zero at the
reported resolution, and no self-intersection. The area change is real: the
path is shortened and part of the arc is replaced, so the adaptive strategy is
not shape-preserving. The trade-off is explicit in the `dev` and `removed arc`
columns.

Tight bounds fail honestly. For `alpha = 150°`, `max_miter_length = 4.5` and
`max_path_deviation = 1.0` returns `NoFeasibleCut`; the Pareto scan shows that
the miter can only be reduced further by accepting a larger path deviation.
`preserve_tangency = true` returns `TangencyRequired` for every straight cut.

This experiment is for the inflected `LineArc` model. The continuing-turn
model is not yet wired into `AdaptivePath`; its hybrid counterexample below is
the reason a curvature-aware adaptive builder is still required.

### Event-driven variant: what the first event actually is

`AdaptivePath::solve_event_driven` implements the contact-limited idea by
selecting the first topology-valid cut instead of a cut chosen only by the
miter cap.

The first candidate event is the AOIM loss at `d_min`. Its contact point is the
parabola vertex `V`. Relative to the circle center, `V` is at angle `π/2`, while
the finite arc starts at `3π/2 - α`; its arc parameter is `α - π`, which is
negative for every valid `α`. **The AOIM contact event therefore occurs before
the finite arc begins and does not define an arc cut.** Preserving the arc until
that event means using the hybrid at the joint, not making an adaptive cut.

The first event that *does* constrain the adaptive cut is the topology of the
straight-body quadrilateral: the line–line seam and the `Q` cross-section stop
crossing. The existing adaptive solver already uses exactly this condition as a
validity constraint; it is why a too-small cut is rejected.

Comparison (same inputs as the table above):

| alpha | miter-bounded removed / miter / dev | event-driven removed / miter / dev |
|---|---|---|
| 30° | 5.61° / 4.092 / 0.019 | 5.61° / 4.092 / 0.019 |
| 90° | 19.52° / 4.898 / 0.230 | 19.52° / 4.898 / 0.230 |
| 150° | 46.03° / 6.495 / 1.260 | 45.94° / 6.502 / 1.256 |
| 170° | 65.98° / 6.498 / 2.445 | 61.20° / 6.871 / 2.190 |
| 175° | 70.98° / 6.498 / 2.779 | 65.73° / 6.911 / 2.496 |
| 179° | 75.07° / 6.492 / 3.059 | 69.59° / 6.923 / 2.762 |

Conclusions:

- At `30/90/150°` the miter-bounded cut already **is** the first valid cut;
  there is no evidence that it cuts too early.
- At `170/175/179°` the event-driven cut saves roughly `4.8–5.5°` of arc and
  `0.25–0.30` path deviation, but the miter rises to `6.87–6.92`, slightly above
  the `6.5` cap. This is a real but small Pareto improvement.
- The apparent “too early” cut at `179°` is the price of the user’s miter cap:
  preserving the arc longer makes the miter longer, not invalid. If the cap is
  relaxed to about `7.0`, the event-driven cut is preferable; if the cap is
  `6.5`, the current miter-bounded cut is necessary.
- The AOIM-loss event is not a substitute for a miter cap; the topology event
  is the proper lower bound on the cut.

### Strategy summary

| strategy | strong point | limitation | preferred use |
|---|---|---|---|
| `Analytical` AOIM | exact offset-intersection curve; no path change; no area change | no point for `d < d_min`; circular extension can touch itself at `d_min` | profiles wholly inside the AOIM existence range |
| `HybridAtJunction` | covers the missing side; no path change; G1 at `P`; zero gap/overlap in 2D | long sharp wedge for `alpha` near `180°`; area grows | when path modification is forbidden and the wedge is acceptable |
| `HybridAtLimitTangent` | preserves the AOIM curve to the limit | the vertex tangent is parallel to the line offsets, so no seam exists below the limit | diagnostic only |
| `HybridAtLimitBisector` | can close the area | connector is not tangent and varies with `d`; not a linear sweep continuation | diagnostic only |
| `AdaptivePath` | bounds the miter; controlled, measurable path deviation; 2D-valid | changes the path and area; G0 at `A`; infeasible for tight bounds; requires explicit CAD authorisation | sharp/opposite tangents where the long wedge is unacceptable |
| `solve_event_driven` | first topology-valid cut; minimal arc modification for a relaxed miter cap | miter may exceed a tight cap; same path/area change; G0 at `A` | when the miter cap can be relaxed to `~7` and minimal path modification is preferred |

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

The suite currently has **47 tests**. They cover:

- the original AOIM algebra, independent bisection, focus/directrix, existence limit, tangency, scaling, and invalid inputs;
- hybrid seam construction, tangent parameter sign, C1 continuity at `P`, and boundary sharing;
- untrimmed overlap, 2D area balance, raster gap/overlap/symmetric difference, and self-intersection;
- the `α = 150°` uncovered-profile counterexample;
- the limit-tangent and limit-bisector negative controls;
- wide, near-collapse, shallow, and steep profiles;
- the continuing-turn counterexample (`d_max`, forward tangent parameter, backward-ray distance, extension inside the line body);
- adaptive-path cuts for `30/90/150/170/175/179°`, bounded miter and deviation, G1 at `Q`, G0 at `A`, far-endpoint preservation, dense independent path-deviation confirmation, and infeasible/tangency-required/no-cut cases;
- event-driven first-valid cuts, the `150/179°` comparison against the miter-bounded cut, and the proof-by-parameter that the AOIM contact event lies before the finite arc;
- adaptive three-part area balance, raster overlap/gap/symdiff, self-intersection, and the comparison that the adaptive miter is shorter than the hybrid wedge.

The original checks that substituted `P(d)` back into the parabola were **self-confirming**: both sides came from the same two offset equations. They are kept, but the independent bisection, geometric bisector, and 2D region checks are the ones that can detect a branch, sign, or prolongation error.

## Visualize and inspect metrics

```sh
cargo run --example visualize
cargo run --example metrics
cargo run --example adaptive
cargo run --example compare
```

`visualize` writes `out/index.html`, per-angle constructions, four-panel
original-vs-hybrid comparisons, and adaptive-path figures:

- `out/aoim-vs-hybrid-30.svg`, `-90.svg`, `-150.svg`
- `out/aoim-adaptive-150.svg`, `-179.svg`

The adaptive figures show the original path (gray dashed), the hybrid wedge
(orange dashed), the miter-bounded modified path `line -> A -> tangent -> Q ->
arc` in purple, the event-driven first-valid cut in teal dashed, the line-line
miter at `A`, the `Q` cross-section, and the three trimmed regions with metrics.
The event-driven outlines are dashed teal; blue is the line body and orange is
the remaining arc. The `out/` directory is gitignored.

`metrics` prints the hybrid area/raster table and connector kinematics.
`adaptive` prints the cut search and adaptive region metrics for all angles.
`compare` runs analytical, hybrid, miter-bounded adaptive, and event-driven
adaptive on the same `R`, profile, and line length, printing miter chord/extent,
path deviation, area difference, overlap/gap/symdiff, and self-intersection.

## Research status / next step

**Confirmed for the inflected Line–Arc model:** the tangent continuation at the
joint is a valid local linear prolongation on the side where `t(d) < 0`; the
hybrid seam closes both 2D regions with no area overlap or gap, C1 at `P`, and
no self-intersection for the tested profiles.

**Confirmed for the adaptive path:** shortening the line to `A` and inserting a
tangent segment `A -> Q` gives a valid 2D sweep with a bounded line–line miter.
It works for `30/90/150/170/175/179°` within the reported bounds; the cut is
G0 at `A` and G1 at `Q`; tight bounds and `preserve_tangency = true` fail
explicitly. The price is a measurable path deviation and area change.

**Refuted as a universal rule:** switching at `P` does not transfer to the
continuing-turn (concave) side, because the required tangent continuation
points forward, not backward. A limit switch with a non-tangent connector can
close the area but is not a linear sweep continuation. `AdaptivePath` is not
yet implemented for the continuing-turn model.

**Not yet validated:** Arc–Arc, spline segments, 3D frames, trimmed surface
domains, B-rep topology/sewing, or performance relative to SMLib.

**Next:**

1. Add a curvature-sign parameter to the model and both region builders; choose
   the admissible hybrid side automatically from `sign(t(d))`.
2. Wire `AdaptivePath` into the continuing-turn model; check whether the tangent
   segment and the remaining right-curving arc still produce valid regions.
3. For the continuing-turn side, compare alternatives: trim the line body
   instead of the arc, or allow a ruled (non-linear) local connector.
4. Extend the 2D region test to Arc–Arc and a moving frame.
5. Only then attempt two neighbouring B-rep pieces with shared topology.

**No upstream PR is intended until that evidence exists.**


