# AOIM Line–Arc — independent geometric research

Experimental Rust implementation of the **Analytical Offset-Intersection Miter (AOIM)** concept associated with [solidcore issue #447](https://github.com/tangency-labs/solidcore/issues/447).

This code is **standalone** and intentionally NOT part of the SolidCore kernel or its Rust↔C++ difftests.

## Geometry and conventions

The joint is the origin. The incoming segment travels along `+x` and is supported by `y = 0`. The next segment is supported by a circle of radius `R` through the origin with center

`C = (R sin α, R cos α)`.

The outgoing tangent at the joint is `(cos α, −sin α)`: the joint turns clockwise by `α`. Because `C` is on the left of that tangent, the arc itself curves counter-clockwise. The junction is therefore **inflected** (right kink, then left-curving arc), not a continuing-turn fillet. The mirror case (right-curving arc, `C = (−R sin α, −R cos α)`) is **not** modelled.

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

## Tests

```sh
cd research/aoim-line-arc
cargo test
```

The suite currently has 12 tests:

- zero-offset junction and both supporting loci;
- independent `φ`-bisection solver (no discriminant/square root/parabola) compared with the closed form;
- focus/directrix identity `|P − C| = R − y`;
- existence limit and tangency at the parabola vertex;
- stable behaviour for `d` within `1e-12 R` of `d_min`;
- radius-scaling invariance over six orders of magnitude;
- finite-ray / arc-span classification on both sides of `d = 0`;
- the `α = 150°` uncovered-profile counterexample;
- external vs internal bisector tangent check;
- curvature/inflection convention;
- invalid, non-finite, and overflowed inputs.

The original checks that substituted `P(d)` back into the parabola were **self-confirming**: both sides came from the same two offset equations. They are kept, but the independent bisection and bisector tests are the ones that can actually detect a branch or sign error.

## Visualize

```sh
cargo run --example visualize
```

This writes `out/index.html` and `out/aoim-line-arc-{30,90,150}.svg` (the `out/`
directory is gitignored). Each figure has two panels:

- top: the supporting line, the arc, representative offset lines/circles, the
  intersection points `P(d)`, and the miter branch;
- bottom: the finite line-strip and arc-annulus sweep regions, the sampled seam,
  and a profile-offset bar.

Colour key: blue = line sweep, orange = arc sweep, green = seam on finite bodies
(`d >= 0`), dashed orange = seam on supporting extensions (`d < 0`), red = no real
intersection. The `150°` figure is the clearest counterexample: the red part of
the profile bar has no AOIM point at all, and the seam stops at the parabola
vertex instead of reaching the profile edge.

## Research status / next step

**Confirmed:** the selected offset intersections are the equidistant locus of the supporting line and circle on `d ≥ d_min`; the closed form agrees with an independent numerical solver; the finite-segment split and the sharp-corner coverage limit are reproducible.

**Not yet validated:** a common trimmed boundary of two whole 2D swept regions, overlap/gap area, self-intersection, trimmed surface domains, or any speed/memory advantage over SMLib. The inflected-vs-continuing-turn sign convention must be generalised before Arc–Arc.

**Next experiment (2D sweep reference):**

1. Define the sweep with a fixed 2D profile in the `(offset, height)` plane and both segment frames explicitly.
2. Build the line-sweep and arc-sweep regions on a dense grid, including the `d` range where AOIM has no seam point.
3. Trim each region by the sampled `P(d)` curve and compare the two boundaries: maximum gap, maximum overlap, and symmetric-difference area.
4. Run it on `α = 30°, 90°, 150°` and profiles that cross `d_min`; a single counterexample is a valid result.
5. Only if the 2D test passes should B-rep topology be attempted.

**No upstream PR is intended until that evidence exists.**
