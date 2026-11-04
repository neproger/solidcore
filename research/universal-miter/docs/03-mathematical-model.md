# Planar-sweep miter — mathematical model

Status: research model + independent validator. **No SolidCore source is changed.**
Validator: [`miter_validator.py`](../prototype-python/miter_validator.py). The
immutable run log is
[`initial-validation.txt`](../results/reports/initial-validation.txt). Both are
from that script (Python 3.12, stdlib only, no SolidCore imports).

---

## 0. Fixed constraints (proposed; owner to confirm)

| # | Question | Decision used by this model |
|---|----------|-----------------------------|
| 1 | Arc = circular or elliptical? | Both are supported by the solver. A **closed-form reference exists only for circles**; elliptical arcs are verified numerically (same equations). |
| 2 | Profile: wall only or arbitrary planar? | **Arbitrary planar profile.** The model only uses the profile's material coordinates `(alpha, beta)`; a wall, a pipe and an open sheet are all the same seam surface. The validator exercises `alpha` intervals; the `beta` extrusion is a common translation. |
| 3 | Only the WR-407 frame? | **Yes for this model.** The rail formula holds when the sweep frame is the Darboux frame of the path plane: `z = N` (plane normal), `x = t`, `y = N x t`. That is `Kernel::fixes.sweep_frame_on_planar_path_normal` (WR-407, #434). The upstream frame (without the flag) is out of scope. |
| 4 | A correct refusal at the miter limit? | **Yes.** Refuse when the two rails stop intersecting (fold), when the offset reaches the curvature cusp, when the seam leaves the piece, or when the path is not planar. No fallback construction. |
| 5 | First test matrix | `{Line, circular arc, quadratic Bezier, cubic Bezier}` — all 10 unordered pairs including same-type. B-spline/NURBS enter as **representation-invariance** checks of the same geometry, not as new geometry. |

---

## 1. What the current implementation does

Entry: `SmApiCreateSweepAlongPlanarPath` -> `sweep/api_brep.rs:345` ->
`PrimitiveCreation::create_sweep_along_planar_path`
(`crates/solidcore/src/primitive/primitive_creation/p2/planar_path.rs:265`).

Per path piece:

1. The path is extended at a non-parallel join by `2*sqrt(2)*max_dimension`
   (`planar_path.rs:369-383`) with `CreateExtendedCurve(.., SM_CT_G1, ..)`, which is a
   **straight tangent extension** (`nurbs/b_spline_curve_n_lib.rs:663-731`).
2. The profile is positioned at the piece start: centre to origin, profile normal to X,
   then a frame with `x = tangent` (`primitive_creation/p1.rs:317,373-472`).
3. `CreateCurveSweep` builds `S(u,v) = T(v) + M(v) C(u)` with
   `x = T'/|T'|`, `z` from the projection-normal transport, `y = z x x`
   (`nurbs/srf_adv/p6.rs:171-186,449-463`; `nurbs/srf_adv/p4.rs:117-166`).
   For a planar path the frame is `(t, n, N)` only with WR-407
   (`primitive_creation/p3/geometry.rs:272-302`).
4. At a join: `average = normalize(-t_before + t_after)`, flipped for a reflex turn;
   a temporary cutting line is built through the corner along `average`
   (`planar_path.rs:449-478`).
   - both pieces linear -> use it as is (`:485-490`);
   - otherwise `BrepIntersect` of the two extended solids (`:498-503`), one
     `GlobalCurveSolve` per intersection curve (`:530-545`), keep the two closest
     points (`:570-583`), build a straight line through them (`:584-597`);
     fall back to the temporary line if that fails (`:602-604`).
5. `ProjectAndTrim` with `direction = N` (`:203-236,605-617`). This builds the **plane**
   spanned by the curve and `N`, projects it onto the faces, imprints and trims
   (`topology/brep/p10.rs:2095-2148`).

So the current joint is always a plane. For `Line-Line` that plane is the exact
bisector. For curved pieces it is a chord through two sampled intersection points.

---

## 2. Geometry of the planar sweep

Let the path piece be `P_i(v)` in the plane `Pi` with unit normal `N`. Define

```
t_i(v) = P_i'(v) / |P_i'(v)|,
n_i(v) = N x t_i(v)              (left normal),
kappa_i(v) = cross(P_i', P_i'') / |P_i'|^3   (signed, t' = kappa n).
```

With the WR-407 frame a profile point with local coordinates `(alpha, beta)`
(`alpha` along `n`, `beta` along `N`) maps to

```
S_i(v, alpha, beta) = P_i(v) + alpha n_i(v) + beta N.          (1)
```

The **rail** at offset `alpha` is

```
R_{i,alpha}(v) = P_i(v) + alpha n_i(v).                        (2)
```

`R_{i,alpha}` is exactly the signed-offset curve of `P_i`: for `|alpha| < 1/|kappa_i|`
it is the level set of the signed distance `d_i` (positive on the left). This is a
consequence of the frame, not an assumption about the path type.

---

## 3. Miter surface and miter curve

A miter matches the **same material point** of the profile on both pieces:

```
P_1(v) + alpha n_1(v) + beta N = P_2(w) + alpha n_2(w) + beta N
   <=>   P_1(v) + alpha n_1(v) = P_2(w) + alpha n_2(w) =: X(alpha).    (3)
```

`beta` cancels, so the seam surface is the extrusion of a planar curve along `N`:

```
M = { X(alpha) + beta N }.                                      (4)
```

Since `X(alpha)` lies on both `alpha`-levels,

```
d_1(X(alpha)) = d_2(X(alpha)) = alpha,
```

and, within the reach, the seam is the branch through the corner `O` of

```
gamma = { X in Pi : d_1(X) - d_2(X) = 0 }.                      (5)
```

Equation (5) is the hypothesis from the task. It is a **theorem under the assumptions
of sections 0 and 2**, with two qualifications:

- the distances are **signed** (left normal), not absolute values;
- the form `d_1 - d_2 = 0` fails beyond the reach; the rail form

```
G(v, w, alpha) = P_1(v) + alpha n_1(v) - P_2(w) - alpha n_2(w) = 0   (6)
```

is the fundamental equation. It keeps the material correspondence even where the
nearest-point distance no longer describes the sweep.

### 3.1 Relation to the current cutting line

At `alpha = 0`, `X(0) = O` and

```
grad F = n_1 - n_2 = N x (t_1 - t_2),
tangent of gamma at O  ||  t_2 - t_1  =  -t_1 + t_2.
```

This is exactly the `average` of `planar_path.rs:456`. **The current line is the
first-order tangent of the true miter curve at the corner.** It is exact only when
`gamma` is a straight line (`Line-Line`, and the symmetric `Arc-Arc` pair below),
otherwise the error is `O(alpha^2)`. The validator measures it (section 6).

---

## 4. Solver

### 4.1 Newton for fixed `alpha`

From (6),

```
d/dv (P_1 + alpha n_1) = (1 - alpha kappa_1) P_1',
d/dw (P_2 + alpha n_2) = (1 - alpha kappa_2) P_2',
J = [ (1 - alpha kappa_1) P_1' ,  -(1 - alpha kappa_2) P_2' ],
det J = -(1 - alpha kappa_1)(1 - alpha kappa_2) cross(P_1', P_2').
```

Solve `J [dv, dw]^T = -G` with a damped step. Start from `alpha = 0`,
`v = end(P_1)`, `w = start(P_2)`.

### 4.2 Continuation

Differentiate (6) in `alpha`:

```
(1 - alpha kappa_1) v' P_1' - (1 - alpha kappa_2) w' P_2' = n_2 - n_1,   (7)
X'(alpha) = n_1 + (1 - alpha kappa_1) v' P_1' = n_2 + (1 - alpha kappa_2) w' P_2'.
```

At `alpha = 0`, (7) gives the first-order tangent `t_2 - t_1`. Integrate (7) in both
directions with an adaptive step, correcting with the Newton of 4.1. The branch is
fixed by the continuation from `alpha = 0` (the zero set is regular there:
`grad F = n_1 - n_2 != 0` for `t_1 != t_2`). No segment-type dispatch: only `P`, `t`,
`n`, `kappa`, all from the generic curve evaluator.

### 4.3 Parameters and extension

The implementation works with the **extended pieces** (section 1, step 1), because the
swept solid is built on them. With a general parameter the Jacobian entries carry the
speed factor `|P_i'|`; the zero set and (7) are unchanged. The validator implements
the actual `SM_CT_G1` policy: a **straight tangent segment**. Consequence in section 5.

---

## 5. Degeneracies, limits, refusals

`det J` vanishes in **two independent ways**:

1. **Curvature cusp** `1 - alpha kappa_i = 0` (`alpha = 1/|kappa_i|`): the offset rail
   loses regularity.
2. **Rail fold** `cross(P_1', P_2') = 0`: the two offset rails become tangent, so the
   same-`alpha` intersection disappears (a fold of the seam).

The fold is a **different event** from the cusp and, in the validated configurations,
it binds **first**:

| case | fold (analytic) | curvature cusp | `1 - alpha kappa` at the fold |
|------|-----------------|----------------|-------------------------------|
| line - circle (R = 1, 90 deg) | `alpha = 0.5` | `alpha = 1` | 0.5 |
| arc - arc (R = 1, symmetric) | `1 - sqrt(2)/2 = 0.292893` | `alpha = 1` | 0.707107 |
| arc - arc (R1 = 1, R2 = 1.4) | `0.3398` (computed) | `alpha = 1` | 0.66 / 0.76 |

So `|alpha| < 1/|kappa|` is **necessary but not sufficient**. The solver must also
require that the rail intersection exists and lies inside the extended pieces. When
it does not, the correct answer is a refusal, not a fallback.

Other refusals: `t_1 || t_2` (no join), `t_2 = -t_1` (reversal), a non-planar path or
an undefined plane normal, a seam leaving the piece domain, a self-intersecting path
(non-unique closest point), a scaling sweep (the rail is scaled), a non-planar profile.

### 5.1 Extension policy — a design decision, not a detail

SolidCore's extension is **straight** (`SM_CT_G1` is linear, `b_spline_curve_n_lib.rs:663-731`).
For a curved piece, half of the seam can lie on that straight extension: the side
whose `alpha` runs past the piece end. The validator's asymmetric `Arc-Arc` case:

```
straight extension:  X(-0.4) = (0.400000, -0.400000),  curvature(gamma) on alpha<0 = 0
natural continuation: X(-0.4) = (0.365349, -0.351488),  curvature(gamma) on alpha<0 = -0.062901
difference at alpha = -0.4: 5.96e-2
```

On `alpha > 0` both modes give the same curvature (`-0.177957`), because that side is
on the original arcs. The seam therefore has a **curvature jump at `alpha = 0`** in the
real construction. This is not an artefact of the model: the swept solid really uses the
straight extension up to the seam. The owner must choose:

- accept the straight-extension miter (matches the current construction), or
- change the extension to a curvature-continuous continuation (G2 / natural), which
  changes the part and needs its own reference.

The model must state which policy it describes; (1)-(7) are valid for either, with the
corresponding extended pieces.

---

## 6. Independent validation

`miter_validator.py` implements the geometry from first principles (lines, circular
arcs, quadratic/cubic Beziers, clamped B-splines) and checks:

- `|G|`, and independently `|d_i(X) - alpha|` by closest-point projection
  (golden section + Newton on `(P - X).P'`, not the rail normal);
- the analytic cases;
- representation / subdivision / reparameterisation invariance;
- the fold and the extension policy.

### 6.1 Core results

| case | points | alpha range | max `abs(d1-alpha)` | max `abs(d2-alpha)` | max `abs(F)` | current-line dev | min `abs(det J)` | events |
|------|--------|-------------|---------------------|---------------------|--------------|------------------|------------------|--------|
| line-line | 37 | [-0.58, +0.58] | 0 | 3.2e-15 | 3.2e-15 | 2.3e-15 | 4.0 | — |
| line-arc | 57 | [-0.86, +0.50] | 0 | 1.0e-13 | 1.0e-13 | 3.5e-1 | 3.7e-5 | fold @ +0.500 |
| arc-arc (symmetric) | 45 | [-0.86, +0.29] | 5.6e-17 | 1.3e-14 | 1.3e-14 | 6.6e-15 | 1.4e-4 | fold @ +0.293 |
| arc-arc (asymmetric) | 46 | [-0.86, +0.34] | 1.7e-16 | 7.4e-14 | 7.4e-14 | 5.6e-2 | 1.1e-4 | fold @ +0.340 |
| quad-quad | 37 | [-0.58, +0.58] | 1.7e-16 | 3.2e-15 | 3.2e-15 | 1.2e-1 | 4.0 | — |
| cubic-cubic | 37 | [-0.58, +0.58] | 1.1e-16 | 4.5e-14 | 4.5e-14 | 1.4e-1 | 3.2 | — |
| bspline-bspline | 37 | [-0.58, +0.58] | 1.1e-16 | 4.4e-14 | 4.4e-14 | 1.4e-1 | 3.2 | — |

The independent signed-distance check agrees with the rail solution to `1e-13` or
better in every case. This is the experimental confirmation that (6) and (5) describe
the same curve within the reach.

### 6.2 Analytic checks

```
line-line          max distance to the exact bisector line       2.8e-16
line-arc           max |d_line - (R - |X-C|)| on the arc side    5.9e-14
arc-arc            max |(R1-|X-C1|) - (R2-|X-C2|)|               8.8e-14
arc-arc (symmetric) max distance to the current line              4.4e-14
arc-arc (asymmetric) max |(R1-|X-C1|) - (R2-|X-C2|)|             7.8e-14
```

`Line-Line` and the symmetric `Arc-Arc` pair are exactly on the current line. The
asymmetric `Arc-Arc` and `Line-Arc` are not: the current-line deviation is
`O(alpha^2)` (e.g. `Line-Arc`: `0.354` at `alpha = 0.5`, coefficient `1.414`;
`Arc-Arc-asym`: `0.056` at `alpha = 0.34`, coefficient `0.484`).

### 6.3 Invariance

```
cubic vs its exact B-spline representation     2.8e-16
cubic split at u = 0.5 (corner unchanged)      2.0e-16
line reparameterised (u in [2,5])              1.3e-15
```

The solver is independent of the segment class, the representation and the
parameterisation, as required.

### 6.4 Binding limits

At `alpha = 0.90, 0.99, 1.01` on the symmetric `Arc-Arc` (R = 1) the trace stops at
`0.292893 = 1 - sqrt(2)/2`, with `d1 - alpha = 0` and `d2 - alpha = -6e-16` up to the
stop. The cusp at `alpha = 1` is never reached: the fold binds first. This is the
strongest correction to the initial model.

---

## 7. Test matrix

Pairs (5 families, all 10 unordered pairs including same-type):

```
Line-Line, Line-Arc, Line-Quad, Line-Cubic, Arc-Arc,
Arc-Quad, Arc-Cubic, Quad-Quad, Quad-Cubic, Cubic-Cubic
```

Conditions per pair:

- **turn**: 90 deg convex / 90 deg reflex / 30 / 150 / 1 / 0 (skip) / 180 (refuse);
- **alpha / limit**: `alpha_max/reach` and `alpha_max/alpha_fold` in
  `{0.1, 0.5, 0.9, 1.0, 1.1}`; the last must refuse;
- **scale**: `1e-3, 1, 1e3, 1e6`; far origin;
- **placement**: XY, tilted plane, arbitrary orientation; WR-407 on (the model) and
  off (out of model, refusal or documented divergence);
- **parameterisation**: uniform / non-uniform, reversed, rational weights, knot
  insertion, piece split;
- **geometry**: inflection, curvature jump at the join, G1 join, self-intersection,
  zero-length piece;
- **profile**: thin wall, wide wall near the fold, pipe, open sheet, multi-curve;
- **checks**: analytic where available, rail-root reference, membership of both
  sweep surfaces, closure/volume, invariance, convergence under `h/2`.

Analytic references: `Line-Line` (bisector), `Line-Circle` (parabola), `Circle-Circle`
(quartic / Cartesian oval). No closed form for Bezier/B-spline or ellipses; those use
the rail-root reference plus inversion and closure.

---

## 8. Governance and risks

- A new seam differs from the C++ plane construction. Per `AGENTS.md` §4b this is a
  **flagged redesign** (`Kernel::fixes`), with a regression test and a
  `plan/upstream-issues/` addendum; the difftest reference for the redesigned
  construction is the SolidCraft/fork result, not the old plane. Do not loosen the
  difftest tolerance.
- Production `planar_path.rs` is untouched. The next code step is a read-only
  prototype that replaces only the choice of the cutting curve (the traced `gamma`)
  and reuses `ProjectAndTrim` with `direction = N`.
- The extension policy (5.1) must be fixed before the prototype, because it changes
  the answer on one side of the profile.
- The miter limit is a **fold**, not only the cusp; the refusal predicate must test
  `cross(P_1', P_2')` and the piece domain, not just `1 - alpha kappa`.

---

## 9. Open decisions for the owner

1. Confirm the five constraints in section 0.
2. Accept the straight-extension miter, or change the extension to a
   curvature-continuous one (5.1).
3. Define the refusal message and the tolerance for the fold (the solver must not
   silently produce a plane).
4. Decide whether the first implementation is a read-only prototype (as recommended)
   or a flagged production path.
