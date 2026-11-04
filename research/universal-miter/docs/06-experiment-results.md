# 06 — Experiment results

Two independent implementations:

- the Python validator
  ([`miter_validator.py`](../prototype-python/miter_validator.py)), whose first
  run is preserved unchanged in
  [`initial-validation.txt`](../results/reports/initial-validation.txt);
- the Rust level A prototype (`prototype/`), whose Gate 1 run is
  [`v1-gate1-rust.txt`](../results/reports/v1-gate1-rust.txt) and whose audit
  run is [`v1a-gate1-audit-rust.txt`](../results/reports/v1a-gate1-audit-rust.txt).

The Python validator is independent of SolidCore: it implements the curves, the
closest point and the rail solve from first principles.

Reproduce the Python reference:

```sh
python research/universal-miter/prototype-python/miter_validator.py
```

## Core cases

`G` is solved by continuation in `alpha`; `d1`, `d2` are checked by an
independent closest-point projection (golden section + Newton on
`(P - X).P'`); `d_line` is the distance to the current cutting line (the tangent
at the corner).

| case | points | alpha range | max `|d1-alpha|` | max `|d2-alpha|` | max `|G|` | max `d_line` | min `|det J|` | stop |
|------|--------|-------------|------------------|------------------|-----------|--------------|---------------|------|
| line–line | 37 | [-0.58, +0.58] | 0 | 3.2e-15 | 3.2e-15 | 2.3e-15 | 4.0 | — |
| line–arc | 57 | [-0.86, +0.50] | 0 | 1.0e-13 | 1.0e-13 | 3.5e-1 | 3.7e-5 | fold @ +0.500 |
| arc–arc (symmetric) | 45 | [-0.86, +0.29] | 5.6e-17 | 1.3e-14 | 1.3e-14 | 6.6e-15 | 1.4e-4 | fold @ +0.293 |
| arc–arc (asymmetric) | 46 | [-0.86, +0.34] | 1.7e-16 | 7.4e-14 | 7.4e-14 | 5.6e-2 | 1.1e-4 | fold @ +0.340 |
| quadratic–quadratic | 37 | [-0.58, +0.58] | 1.7e-16 | 3.2e-15 | 3.2e-15 | 1.2e-1 | 4.0 | — |
| cubic–cubic | 37 | [-0.58, +0.58] | 1.1e-16 | 4.5e-14 | 4.5e-14 | 1.4e-1 | 3.2 | — |
| B-spline–B-spline | 37 | [-0.58, +0.58] | 1.1e-16 | 4.4e-14 | 4.4e-14 | 1.4e-1 | 3.2 | — |

**Interpretation.** The rail solution and the independent signed-distance
definition agree to `1e-13` or better in every case. This is the experimental
confirmation of the model within its assumptions.

## Analytic checks

```
line–line          max distance to the exact bisector line       2.8e-16
line–arc           max |d_line - (R - |X-C|)| on the arc side    5.9e-14
arc–arc            max |(R1-|X-C1|) - (R2-|X-C2|)|               8.8e-14
arc–arc symmetric  max distance to the current line              4.4e-14
arc–arc asymmetric max |(R1-|X-C1|) - (R2-|X-C2|)|               7.8e-14
```

`Line–Line` and the **symmetric** `Arc–Arc` pair are exactly on the current
line. The asymmetric `Arc–Arc` and `Line–Arc` are not: the current-line
deviation is `O(alpha^2)` (`Line–Arc`: `0.354` at `alpha = 0.5`, coefficient
`1.414`; `Arc–Arc-asym`: `0.056` at `alpha = 0.34`, coefficient `0.484`). The
symmetric pair is a special symmetry and must not be used alone as evidence.

## Invariance

```
cubic vs its exact B-spline representation     2.8e-16
cubic split at u = 0.5 (corner unchanged)      2.0e-16
line reparameterised (u in [2,5])              1.3e-15
```

The solver does not branch on the segment class, the representation or the
parameterisation.

## Binding limits: fold, not cusp

At `alpha = 0.90, 0.99, 1.01` on the symmetric `Arc–Arc` (R = 1) the trace stops
at `0.292893 = 1 - sqrt(2)/2`, with `d1 - alpha = 0` and
`d2 - alpha = -6e-16` up to the stop. The curvature cusp at `alpha = 1` is never
reached: the **rail fold** binds first.

`det J` factors as

```
det J = -(1 - alpha kappa_1)(1 - alpha kappa_2) cross(P_1', P_2').
```

Two independent failure modes:

- curvature cusp: the first two factors vanish;
- rail fold: the last factor vanishes (the offset curves are tangent).

At the validated folds the curvature factors are 0.5 and 0.707, so the stop is a
fold, not a cusp. The refusal predicate must test `cross(P_1', P_2')` and the
piece domain, not only `|alpha| < 1/|kappa|`.

## Extension policy

SolidCore's `SM_CT_G1` extension is straight. The asymmetric `Arc–Arc` case
(`R1 = 1`, centre `(0,1)`; `R2 = 1.4`, centre `(-1.4,0)`):

```
straight: X(-0.4) = (0.400000, -0.400000),  kappa(gamma) on alpha<0 = 0
natural:  X(-0.4) = (0.365349, -0.351488),  kappa(gamma) on alpha<0 = -0.062901
difference at alpha = -0.4: 5.96e-2
```

On `alpha > 0` both modes give `kappa = -0.177957`. The real seam therefore has
a curvature jump at `alpha = 0`. This is not a numerical artefact: the swept
solid includes the straight extension up to the seam. For v1 the owner chose the
existing straight G1 policy; the Rust solver implements exactly it.


## What this proves and what it does not

**Proves:** the rail model `G = 0` and the signed-distance formulation describe
the same curve within the reach; the solver is type- and representation-
independent; the fold is the practical limit; the straight-extension policy
changes the seam.

**Does not prove:** that the production Planar Sweep can be replaced by the
traced seam without regressions. Level A is the joint component; the alternative
Planar Sweep (`create_sweep_along_planar_path2`) must build its own swept
surfaces and B-rep, and be checked for closure, manifold, volume and stability.
The scope and contract are in `09-sweep2-scope.md`; no Planar Sweep2
implementation is started.

---

## Rust level A — Gate 1 (PASS)

Commands (the prototype is a separate workspace; no production file is read or
written):

```sh
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml \
  --test matrix gate1_report -- --nocapture
```

**22 tests pass:** 1 rail-stop classifier unit test, 20 matrix/audit tests, 1
infrastructure smoke test. The matrix covers all ten pairs, the analytic
`Line–Line`, `Line–Arc` and `Arc–Arc` cases, a reflex turn, subdivision,
affine reparameterisation, B-spline representation, reversed curves, scales
`1e-3` and `1e3`, the G1 extension segments, the fold (line–arc and arc–arc),
an out-of-piece seam, and the independent closest-point distances. The audit
tests add closest-point independence, repeated-run determinism, the fold
approach from both sides, and relative-residual assertions
(`08-level-a-audit.md`).

| case | status | rail | `\|d1-alpha\|` | `\|d2-alpha\|` | current line | min `\|det J\|` | stop |
|------|--------|------|----------------|----------------|--------------|------------------|------|
| line-line | Ok | 3.3e-16 | 0 | 3.3e-16 | 2.2e-16 | 4.0 | reached bound |
| line-arc | Ok | 4.6e-15 | 0 | 4.6e-15 | 1.08e-1 | 1.405 | reached bound |
| arc-arc | Ok | 3.5e-15 | 5.6e-17 | 2.6e-15 | 1.7e-15 | 0.872 | reached bound |
| arc-arc-asym | Ok | 6.5e-15 | 2.8e-17 | 5.9e-15 | 6.38e-3 | 2.105 | reached bound |
| quad-quad | Ok | 1.8e-14 | 8.3e-17 | 1.6e-14 | 1.42e-2 | 4.0 | reached bound |
| cubic-cubic | Ok | 1.9e-14 | 2.8e-17 | 1.9e-14 | 1.22e-2 | 3.24 | reached bound |
| bspline-bspline | Ok | 1.9e-14 | 2.8e-17 | 1.9e-14 | 1.22e-2 | 3.24 | reached bound |
| line-arc fold | Refused(Fold) | 4.9e-14 | 0 | 5.0e-14 | 3.54e-1 | 1.26e-4 | positive fold at 0.500000; negative reached −0.600000 |
| arc-arc fold | Refused(Fold) | 4.3e-14 | 5.6e-17 | 4.3e-14 | 2.1e-14 | 8.74e-5 | positive fold at 0.292893; negative reached −0.400000 |
| line-line outside | Refused(OutsidePiece) | 3.1e-16 | 0 | 3.1e-16 | 2.2e-16 | 0.2 | positive outside at α = 0.100000 |

Gate 1 criteria:

- rail and independent signed-distance residuals: maximum `4.95e-14` — below
  the `1e-12` gate;
- `Line–Line` is the exact bisector (`2.2e-16`), `Line–Circle` the analytic
  parabola, `Circle–Circle` the analytic signed-distance quartic;
- subdivision, affine reparameterisation and the exact B-spline representation
  change the seam by at most `1e-12` (the tests assert this at a common alpha
  grid);
- the fold is detected at `0.500000` for line–arc and `0.292893 = 1 − √2/2` for
  arc–arc; the negative branch continues independently in both cases;
- the curvature-cusp classifier is unit-tested (fold and cusp are distinguished
  by `t1 × t2` and `1 − ακ`); in the tested corner geometries the fold binds
  first, so no trace reaches the cusp;
- an out-of-piece seam is refused at the piece end, not replaced by a fallback;
- `Original` remains `NotImplemented` at level A; that is intentional and is the
  level B comparison.

### Audit additions

The Level A audit (`08-level-a-audit.md`) added the quantitative near-fold and
scale diagnostics:

- `characteristic_scale` and `max_relative_*` residuals (absolute divided by the
  scale); at unit scale the relative values equal the absolute ones, and the
  `1e-3`/`1e3` scale test checks the normalised point set at `1e-12`;
- per-sample Newton iterations and adaptive step, and trace-level
  `min_step`/`max_step`/`max_iterations`;
- near the line–arc fold: `min |det J| = 1.26e-4`, 6 Newton iterations,
  steps `1.12e-9 … 1.97e-1`;
- near the arc–arc fold: `min |det J| = 8.74e-5`, 5 iterations,
  steps `6.88e-10 … 1.21e-1`;
- the fold approach from below (`0.49` Ok) and above (`0.51` refused as `Fold`)
  is a regression test, and `solve_sequence` above the fold errors: there is no
  rail intersection to fall back to.

### Comparison with the Python validator

The Rust and Python implementations agree on the defining results:

- `Line–Line` is on the exact bisector; the symmetric `Arc–Arc` seam is on the
  current line (`1.7e-15` in Rust, `4.4e-14` in Python);
- the fold alphas agree: `0.500000` vs `0.500` (line–arc) and `0.292893` vs
  `0.293` (arc–arc);
- the residuals are the same order (`~1e-14`), well below `1e-12`;
- the curved cases deviate from the current line in both implementations:
  `Line–Arc` by `0.108` at `alpha = 0.4` (Python reports `0.354` at `0.5`),
  `Arc–Arc-asym` by `6.4e-3` at `0.2`.

The Rust implementation is not a line-by-line port of the Python validator; the
Python baseline remains the independent reference.

### Gate 1 verdict

**PASS.** The Rust level A solver reproduces the independent mathematical model
on the full matrix, with explicit folds, cusps and out-of-domain refusals, and
no fallback. The Level A audit (`08-level-a-audit.md`) confirms the independent
closest-point validation, the relative residuals and the near-fold behaviour.
Level B / Sweep2 is not started: the scope decision is in
`09-sweep2-scope.md`.
