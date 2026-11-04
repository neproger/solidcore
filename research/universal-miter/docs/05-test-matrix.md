# 05 — Test matrix

One case is a pair of path pieces, a planar profile, and an optional independent
reference. The same case runs through `MiterStrategy::Original` and
`MiterStrategy::Universal`.

## Pairs

Five families, all ten unordered pairs including same-type:

| # | Pair | Analytic reference |
|---|------|--------------------|
| 1 | Line–Line | bisector line |
| 2 | Line–Circle arc | parabola: `|X - C| = R ± d_line` |
| 3 | Line–Quadratic Bezier | none; rail root + invariants |
| 4 | Line–Cubic Bezier | none; rail root + invariants |
| 5 | Circle arc–Circle arc | quartic / Cartesian oval |
| 6 | Circle arc–Quadratic | none |
| 7 | Circle arc–Cubic | none |
| 8 | Quadratic–Quadratic | none |
| 9 | Quadratic–Cubic | none |
| 10 | Cubic–Cubic | none |

`B-spline` and `NURBS` are **representation checks**: a curve that is exactly
representable as a clamped B-spline must produce the same seam point set as its
native variant. They are not new geometry in the first matrix. An elliptical arc
is supported by the equations but has no closed-form reference; it is verified
numerically.

## Conditions per pair

- **Turn**: 90 deg convex, 90 deg reflex, 30 deg, 150 deg, 1 deg, 0 deg (skip,
  tangent join), 180 deg (refuse, reversal).
- **Limit**: `alpha_max / alpha_fold` in `{0.1, 0.5, 0.9, 1.0, 1.1}`; the last
  must refuse. Also `alpha_max / (1/|kappa|)` in the same set.
- **Scale**: `1e-3, 1, 1e3, 1e6`; origin far from zero.
- **Placement**: XY, tilted plane, arbitrary orientation; WR-407 on (the model)
  and off (out of model; refuse or record a documented divergence).
- **Parameterisation**: uniform / non-uniform knots, reversed direction,
  rational weights, knot insertion, piece split at an interior parameter.
- **Geometry**: inflection, curvature jump at the join, G1 join, a piece that
  self-intersects, a zero-length piece.
- **Profile**: thin wall, wide wall near the fold, pipe (closed circle), open
  sheet, multi-curve profile.
- **Extension** (SolidCore): straight G1 (the current construction) vs natural
  continuation; record which one a case uses.

## Metrics

Per sample (`alpha`):

- `|G(v, w, alpha)|`;
- `|d1(X) - alpha|`, `|d2(X) - alpha|` by independent closest-point projection;
- distance to the current cutting line (the tangent at the corner);
- the segment of each foot point (original or extension).

Per run:

- point-set distance to the analytic or independent reference;
- seam length and curvature, including the jump at `alpha = 0`;
- stop event and `alpha` where it happened; minimum `|det J|`;
- Newton iterations, wall time;
- level B: face/edge/vertex counts, closure, manifold, volume, self-intersection.

## Acceptance

- Residuals `<= 1e-9` relative for the value checks where the kernel difftest
  tolerance applies; the mathematical prototype is expected to reach `~1e-12`.
- Invariance: subdivision / reparameterisation / representation differences at
  the level of the solver tolerance.
- A correct refusal is a pass for the refusal cases.
- Equal topology is **not** a pass criterion.

## Fixture format (proposed)

One JSON file per case, deterministic and self-contained:

```json
{
  "id": "line-arc-90-convex",
  "curve_a": { "kind": "line", "start": [-2.0, 0.0], "end": [0.0, 0.0] },
  "curve_b": {
    "kind": "circle_arc",
    "center": [-1.0, 0.0],
    "radius": 1.0,
    "start_angle": 0.0,
    "sweep_angle": 1.5707963267948966
  },
  "profile": { "alpha": [-0.5, 0.5], "beta": [0.0, 1.0] },
  "expected": {
    "kind": "parabola",
    "focus": [-1.0, 0.0],
    "directrix_point": [0.0, 0.0],
    "directrix_direction": [1.0, 0.0],
    "radius": 1.0
  },
  "notes": "inner side on the arc, outer side on the straight extensions"
}
```

Serde support is added with the runner; the format is fixed now so fixtures can
be written independently. Fixtures live under `fixtures/<family>/` and are loaded
by both the Rust prototype and, where applicable, the Python validator.

## Level A implemented tests

`prototype/tests/matrix.rs` implements the matrix above (20 tests, including
the audit regressions):

- the ten pairs at `alpha in [-0.2, 0.2]`;
- the analytic `Line–Line`, `Line–Circle` (parabola) and `Circle–Circle`
  (signed-distance quartic), plus the asymmetric circle pair;
- a reflex turn;
- subdivision at `u = 0.5`, affine reparameterisation and exact B-spline
  representation at a common alpha grid;
- reversed curves and scales `1e-3`/`1e3`;
- the G1 extension segments (original vs start/end extension);
- the line–arc and arc–arc folds with the negative branch continuing;
- an out-of-piece seam (`OutsidePiece`);
- the curvature-cusp/fold/non-convergence classifier (unit test);
- the audit regressions: closest-point independence, repeated-run determinism,
  the fold approach from both sides, and relative-residual assertions.

The Gate 1 numbers and the PASS verdict are in `06-experiment-results.md`.

## Baseline

`results/reports/initial-validation.txt` is the immutable first run of the
Python validator. New runs:

```sh
python research/universal-miter/prototype-python/run_report.py \
  --version v0 --label matrix --param pair=line-arc --param turn=90
```

The report records the version, label, parameters and validator hash. Do not
hand-edit a report.
