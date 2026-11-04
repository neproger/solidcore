# 08 — Level A audit

Scope: read-only audit of the Level A rail solver, with research-only fixes and
regression tests. The mathematical model is unchanged. Production code, the
Python validator and the immutable baseline are untouched.

**Verdict: PASS for the Gate 1 matrix, with the documented cusp caveat.**
The numerical accuracy is not obtained by misclassifying singular cases; the
independent checks and the near-fold behaviour are explicit and tested.

## 1. Fold and curvature cusp are classified independently

`rail_solver::classify_stop` evaluates, in this order:

```
cusp = min(|1 - alpha*kappa_1|, |1 - alpha*kappa_2|)
fold = |cross(t_1, t_2)|
cusp < 1e-6  -> CurvatureCusp
fold < 1e-4  -> Fold
otherwise    -> NoConvergence
```

A small `det J` alone never selects `Fold`:

- `det_eps` only routes a step to `classify_stop`; it does not choose the stop;
- a successful step checks the cusp first, then the fold, then the determinant;
- the `OutsidePiece` failure also runs `classify_stop` before defaulting to
  `OutsidePiece`, so a seam that reaches the piece end exactly at the fold is
  reported as `Fold`.

The unit test
`rail_solver::tests::stop_classifier_distinguishes_cusp_fold_and_non_convergence`
covers `1 - alpha*kappa = 0`, parallel tangents, and a near-degenerate state
where both factors are moderate (it must be `NoConvergence`, not `Fold`).

**Caveat.** The tested corner geometries reach the fold before the cusp: at the
line–arc and arc–arc folds the cusp factors are `0.5` and `0.707`. A full trace
that stops at the cusp is therefore not demonstrated. The classifier is verified
directly; the cusp-first geometry is a documented gap, not a hidden fallback.

## 2. Signed-distance validation is independent

`rail_solver::Piece::closest(point)`:

- samples the **extended piece** on a 256-point grid;
- refines with golden section (100 iterations);
- polishes with Newton on `(P - X) . P' = 0`;
- signs the distance with the normal at the closest point.

It receives only the point `X`. It never receives or uses `v`, `w`, `J`, the
residual, or the rail solution. The rail solution supplies only `X`; the check
is a geometric projection, not a rearrangement of the rail equation.

Regression test `closest_point_is_a_geometric_projection`: a line piece returns
`+0.75` and `-0.25` for points above and below it, without any rail state.

## 3. Absolute and relative residuals

`MiterDiagnostics` now carries:

- `characteristic_scale` (the trace magnitude, at least 1);
- `max_relative_rail_residual`, `max_relative_signed_distance_error_a/b`
  (absolute maximum divided by the scale);
- `max_newton_iterations`, `min_step`, `max_step` (the continuation behaviour);
- the existing absolute maxima.

The solver tolerance is `1e-13 * magnitude` and the determinant floor is
`1e-14 * magnitude^2`, so the numerical error scales with the geometry. The
matrix tests assert both absolute and relative residuals at `1e-12`; the scale
test runs at `1e-3` and `1e3` and compares the normalised point sets.

## 4. Behaviour approaching the fold

The continuation halves the step down to `1e-10`, then classifies the last
successful state. The fold diagnostics from the matrix run:

| case | min `\|det J\|` | max Newton iters | step range |
|------|-----------------|------------------|------------|
| line–arc fold | `1.263e-4` | 6 | `1.12e-9` … `1.97e-1` |
| arc–arc fold | `8.735e-5` | 5 | `6.88e-10` … `1.21e-1` |

Regression test `fold_is_approached_from_both_sides`:

- `alpha_max = 0.49` → `Ok`, seam reaches the bound;
- `alpha_max = 0.51` → `Refused(Fold)`, positive stop `Fold`;
- `solve_sequence` at `0.51` returns an error: there is no rail intersection
  above the fold.

The fold is a limit; the last sample is the boundary. There is no straight-line
fallback above it.

## 5. Independent branches

`RailTrace` carries `negative_stop` and `positive_stop` separately. The fold
tests show the negative branch reaching its `alpha` bound while the positive
branch folds; a stop in one direction does not abort the other. The seam samples
are sorted by `alpha` and keep the original/extension segment of each piece.

## 6. Reproducibility

`repeated_runs_are_identical` runs the same case twice and compares the whole
`MiterResult`. The solver has no RNG, no clock and no global state; the only
inputs are the curve data, the profile and the extension. The result is
deterministic.

## 7. Findings and changes

- Added relative residuals, per-sample Newton iterations and adaptive step, and
  trace-level `min_step`/`max_step`/`max_iterations` (research diagnostics only).
- Added the audit regression tests: closest-point independence, reproducibility,
  fold approach from both sides, and relative-residual assertions.
- No change to the mathematical model, the Python validator or the baseline.
- No production code touched.

## 8. Residual limitations

1. **Cusp-first trace** is not demonstrated end-to-end; the classifier is tested.
2. **Fold threshold** `1e-4` is a numerical detection tolerance. The model's
   fold is exact; the threshold decides when the numerical branch is called
   folded. It is documented and does not alter the residual check.
3. **Planar only.** Level A describes the planar-path, WR-407 model; spatial
   paths are outside this model (see `09-sweep2-scope.md`).
4. **Original not implemented.** `MiterStrategy::Original` is the production
   construction and belongs to the comparison/Level B, not Level A.

## 9. Evidence

```sh
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml \
  --test matrix gate1_report -- --nocapture
cargo check --manifest-path research/universal-miter/prototype/Cargo.toml --features solidcore
```

22 tests pass (1 rail-stop classifier, 20 matrix/audit, 1 smoke). The maximum
absolute rail residual is `4.95e-14`; the maximum relative residual is
`4.95e-14` at scale 1. The Gate 1 matrix is green.
