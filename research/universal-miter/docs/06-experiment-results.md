# 06 — Experiment results (Python validator)

Source: [`miter_validator.py`](../prototype-python/miter_validator.py), run
output preserved unchanged in
[`initial-validation.txt`](../results/reports/initial-validation.txt). The
validator is independent of SolidCore: it implements the curves, the closest
point and the rail solve from first principles.

Reproduce:

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
solid includes the straight extension up to the seam. The owner must choose the
extension policy before the solver is implemented.

## What this proves and what it does not

**Proves:** the rail model `G = 0` and the signed-distance formulation describe
the same curve within the reach; the solver is type- and representation-
independent; the fold is the practical limit; the straight-extension policy
changes the seam.

**Does not prove:** that the production B-rep construction can be replaced by
the traced curve without regressions. That requires level B on the real kernel:
`ProjectAndTrim` with the traced curve, closure, manifold, volume, and the
comparison with `Original` on the same cases.
