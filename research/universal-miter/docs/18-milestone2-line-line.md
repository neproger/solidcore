# 18 — Milestone 2: Universal Boundary Construction Pipeline (PASS)

Status: **PASS** for `ExactMiter` on line–line and exact line–arc, and **PASS
for the `HybridAtJunction` boundary/topology** on a line–arc whose missing
profile side has no exact offset intersection. The hybrid volume is a
diagnostic, not a correctness criterion: it belongs to a different joint
construction and is not compared to the ordinary sweep. The boundary,
topology, orientation and validation are confirmed.

All three produce one direct B-rep body, one solid region and one shell, with a
shared seam boundary and no Boolean, `ProjectAndTrim` or `StitchFaces`. The
limit-tangent negative control refuses with a typed error instead of covering
the missing side.

The test is `prototype/tests/direct_joint.rs`. Reports:
`results/reports/v11-milestone2-line-line.txt`,
`results/reports/v12-universal-boundary-line-arc.txt` and
`results/reports/v13-universal-boundary-hybrid.txt`. The v12 label called the
90° line–arc `ExactMiter`; v13 keeps the same result but records the corrected
strategy, because that seam uses the junction tangent extension.

## 1. Separation of responsibilities

| Component | File | Responsibility |
|---|---|---|
| Boundary solver | `prototype/src/boundary.rs` | mathematical joint boundary; strategy contract and selection; no B-rep |
| Surface builder | `prototype/src/level_b_geometry.rs` | NURBS swept surfaces |
| B-rep builder | `prototype/src/direct_brep.rs` | shared vertices/edges/faces, caps and shell assembly |

The boundary solver does not know about `Brep`, `make_face_topology` or caps.
It receives each segment through the common `PathFrame`/`BoundaryInput`
interface: position, tangent, normal, curvature, derivatives, valid parameter
domain and the placed profile. The primitive kind is not an input to strategy
selection.

## 2. Strategy contract and classification

Every strategy implements the same contract:

```
BoundaryStrategy::solve(BoundaryInput) -> JointBoundary | BoundaryError
```

`JointBoundary` carries the strategy identifier, the `K` seam curves, the `K`
shared seam vertices and diagnostics. `BoundaryDiagnostics` records the selected
strategy, the maximum rail residual, the sample and segment counts, the exact
and junction sample counts, the first junction `alpha`, and the covered profile
`alpha` range. A refusal is a typed `BoundaryError::UncoveredProfile` carrying
the strategy and the first missing `alpha`; there is no silent approximation.

The classification is part of one extended rail solve; no strategy is tried
and then replaced by another:

- a seam sample on an extension of a **straight** piece is still an exact
  offset intersection, because that extension is the same supporting line;
- a seam sample on an extension of a **curved** piece is the junction tangent
  continuation and therefore belongs to `HybridAtJunction`;
- `ExactMiter` refuses if any sample belongs to the second category;
- `HybridAtJunction` accepts both categories and uses one shared seam on the
  two extended side surfaces;
- `HybridAtLimitTangent` is the AOIM negative control and refuses when the
  second category is needed.

`Piece::is_linear` supplies the straight/curved classification by sampling the
curvature of the original piece; it is not a primitive-name switch.

| Case | Selected strategy | Samples | Exact / junction | First junction `alpha` | Max rail residual |
|---|---|---|---|---|---|
| line–line | `ExactMiter` | 68 | 68 / 0 | — | `2.289e-16` |
| line–arc exact | `ExactMiter` | 68 | 68 / 0 | — | `0.000e0` |
| line–arc hybrid | `HybridAtJunction` | 68 | 36 / 32 | `-0.022463609` | `1.968e-14` |

## 3. Direct two-piece B-rep

For each piece the Level B surface of the extended path is used. The seam
vertices are dropped on both side surfaces (`Surface::drop_point`) to obtain
their `(U, V)` parameters. The V parameter is used for the rails and the outer
profile segments; **do not assume it equals the profile contour fraction**.
The seam U trims the rails between the outer profile loops and the seam.

The hybrid seam lies on the line end extension and the arc start extension;
both extended side surfaces contain it, so the B-rep keeps the same shared
seam topology as the exact case.

Topology (`K = 4`), identical in all three cases:
- side faces: `2K = 8`; caps: 2; total faces `2K + 2 = 10`;
- edges: outer start `K`, outer end `K`, seam `K`, rails `2K`; total `5K = 20`;
- vertices: start `K`, end `K`, seam `K`; total `3K = 12`;
- each edge has 4 edge-uses; `V - E + F = 12 - 20 + 10 = 2` per shell.

No Boolean, `ProjectAndTrim` or `StitchFaces`; the seam edges are shared by the
two adjacent side-face patches.

## 4. Results

### Line–line (`ExactMiter`)

| Check | Value |
|---|---|
| Surface deviation A / B | `4.657e-15` / `4.621e-15` |
| Boundary deviation from both surfaces | `8.854e-7` |
| Faces / edges / vertices | 10 / 20 / 12 |
| Regions / shells | 2 / 2 |
| `check_brep` | pointers 1000, assert true, 0 failures |
| `integrity::check` | pass |
| Edge uses per edge | 4 |
| `is_manifold_solid` / `is_outward_solid` | true / true |
| Volume | `3.926987504e-1` |
| Analytic `area × centreline length` | `3.926990817e-1` |
| Production reference | `3.926990634e-1` |
| Relative to analytic / production | `8.437e-7` / `7.972e-7` |

### Line–arc exact (`ExactMiter`)

The path is a 5-long line followed by a 150° arc of radius 4; the profile
radius is 0.25. Every sampled offset intersection lies on both original
trajectories.

| Check | Value |
|---|---|
| Surface deviation, line / arc | `1.688e-14` / `7.480e-5` |
| Boundary deviation from both surfaces | `7.136e-8` |
| Faces / edges / vertices | 10 / 20 / 12 |
| Regions / shells | 2 / 2 |
| `check_brep` | pointers 1000, assert true, 0 failures |
| Edge uses per edge | 4 |
| `is_manifold_solid` / `is_outward_solid` | true / true |
| Volume | `3.037933872` |
| Analytic `area × centreline length` | `3.037915288` |
| Relative to analytic | `6.117e-6` |

### Line–arc hybrid (`HybridAtJunction`)

The path is a 1-long line followed by a 90° arc of radius 1; the profile radius
is 0.25. The negative side has no exact offset intersection; 32 of the 68
samples use the junction tangent extension.

| Check | Value |
|---|---|
| Surface deviation, line / arc | `4.657e-15` / `5.591e-5` |
| Boundary deviation from both surfaces | `3.963e-5` |
| Faces / edges / vertices | 10 / 20 / 12 |
| Regions / shells | 2 / 2 |
| `check_brep` | pointers 1000, assert true, 0 failures |
| Edge uses per edge | 4 |
| `is_manifold_solid` / `is_outward_solid` | true / true |
| Volume | `5.037627012e-1` |
| Analytic `area × centreline length` | `5.047746784e-1` |
| Relative to analytic | `2.005e-3` |

## 5. Volume is diagnostic

- Line–line and exact line–arc may be compared with `area × centreline length`
  because their seam is the exact common boundary; they measure `8.437e-7` and
  `6.117e-6`.
- Hybrid line–arc measures `2.005e-3` in the same diagnostic, but this is **not
  a failure**: `HybridAtJunction` is a different joint construction, so the
  solid geometry and its volume legitimately differ from an ordinary sweep.
  The B-rep is judged by the shared boundary, closure, manifold, orientation
  and validation, not by that number.
- The measurements remain in `19-line-arc-volume-diagnostic.md` for traceability
  only; no independent volume reference is required.

`brep_volume_default` uses integration accuracy `1e-4`. The `1e-9` limit from
Milestone 1 remains a kernel integrator convergence limit, not a geometric
tolerance. The production line–arc B-rep is non-manifold / effectively zero
volume and is not used as a reference.

## 6. Findings

1. **Strategy separation is real.** The 90° line–arc has an exact side and a
   missing side; the first exact run silently used the extension and looked
   like `ExactMiter`. Classifying extensions by the underlying piece
   linearity makes the distinction explicit.
2. **`HybridAtJunction` is a valid direct construction.** Its seam lies on both
   extended side surfaces, the B-rep keeps the same shared topology, and the
   shell validates. No Boolean, trim or stitch was needed.
3. **The exact line–arc is genuinely exact.** A 150° arc of radius 4 with a
   5-long line has all 68 samples on both original trajectories; it validates
   and its volume is `6.1e-6` from the analytic reference.
4. **Seam V from the surface, not from the profile fraction.** Using `i/K` for
   the side-surface V produced edge/loop validation gaps; dropping the seam
   vertices on both surfaces and using `result_uv.y` fixes it.
5. **p6 closure tail.** The last `make_face_topology` still reports the
   `TestClosureSplitRegion` Unported after the split. The split completes; the
   exact Unported is accepted and the shell is validated independently.
6. **Orientation.** Side faces use `OrientType::Opposite` (Level B surface
   normal is inward); caps use outward plane normals and `Same`.

## 7. Limitations

- `ExactMiter` and `HybridAtJunction` only; the bisector-limit and adaptive-cut
  constructions are separate next strategies.
- The limit-tangent negative control is a typed refusal, not a covering
  strategy.
- One joint; two segments; circular profile, `K=4`.
- The seam curve between samples is interpolated; the sample points are exact.
- Hybrid line–arc volume differs from `area × centreline length` because it is
  a different joint construction; it is diagnostic only, not a gate.
- p6 tail and the `1e-9` volume integrator limit remain kernel gaps.

## 8. Next

Add the separate `HybridAtLimitBisector` and `AdaptiveCut` strategies behind
the same contract, each with its own geometric formulation and validation; then
exercise arc–arc, line–Bezier and spline–spline as test configurations. Keep
`ExactMiter`, `HybridAtJunction` and the direct B-rep builder unchanged.
