# 37 â€” Boundary Solver mathematical model audit

Status: audit only. No Boundary Solver, Direct builder, Exact Torus or tolerance
changes.

## 1. The implemented contract

`BoundaryInput` provides one `PathFrame` and one placed profile at the first
segment start. For a profile sample with coordinates `(alpha, beta)` in that
frame, the rail solver solves the 2D system

```text
G(v, w, alpha) = P1(v) + alpha n1(v) - P2(w) - alpha n2(w) = 0
X(alpha)       = P1(v) + alpha n1(v) = P2(w) + alpha n2(w)
```

and the 3D seam point is

```text
S = PathFrame.origin + x(alpha) * frame.x + beta * frame.normal
```

`max_residual` is the residual of `G`, not the distance from `S` to either
source surface.

`JointBoundary` stores interpolated seam curves and the first point of each
profile segment. It does not store per-point alpha, beta, source U/V, source
surface membership or a common-boundary certificate.

## 2. What the audit found

### 2.1 Rail residual is not surface membership

`max_residual = 1.284e-13` proves that the 2D offset-rail equations are solved.
It does not prove that the lifted `S` lies on both exact source surfaces.

The seam mapping audit shows the distinction directly:

```text
vertex 0,1,2: source-surface gap <= 2.2e-16
vertex 3:     source-surface gap = 2.50e-1 on both pieces
```

All four points have valid U/V inside `[0,1]`. Two vertices are exact; one is
an interior-U exact point; one is not on either source surface.

### 2.2 Single profile/frame versus per-piece exact placement

The solver receives one profile placement and one global path frame. Exact
sources built from the original paths can have a different profile placement
and arc frame. The indexed audit confirms the Torus sections match their source
profile exactly (best error `2e-16`, shift `0`, reverse `false`).

The V=0.75 seam point is not a source-section point:

```text
piece 0: source U=1 V=.75 = (0, -0.25, 0); seam = (0.25, -0.25, 0)
piece 1: source U=0 V=.75 = (0.25, 0, 0);   seam = (0.25, -0.25, 0)
```

So the failure is in the boundary/profile contract, not in the Torus sections.

### 2.3 HybridAtJunction assumes extended source surfaces

The documented hybrid contract is:

> on the missing side the seam continues on the tangent extension at the joint;
> both extended side surfaces contain the seam.

That is valid for the Level B extended surfaces used in the original
lineâ€“arc test. It is not valid for the exact Torus source: an exact Torus ends
at the original arc end and has no tangent extension. A seam point generated on
the tangent extension is therefore outside the exact source surface, even when
its U/V projection is numerically inside the surfaceâ€™s parameter range.

This is a mathematical contract mismatch, not a Boundary Solver numerical
error:

```text
HybridAtJunction defines seam on extended pieces
Exact source defines surface only on original arc
```

The direct builderâ€™s refusal at that point is correct.

### 2.4 ExactMiter is also a 2D predicate

`ExactMiter` classifies by `Piece::is_linear` and exact rail samples. It does
not check that every resulting point lies on both 3D source surfaces. Its
`UncoveredProfile` decision is based on the 2D rail model.

## 3. Conclusion

The current solver is internally consistent, but its correctness condition is
not the required one for exact sources:

1. It solves a 2D offset-rail problem, then lifts with a fixed profile beta.
2. It does not verify shared-boundary membership on the actual source surfaces.
3. `HybridAtJunction` requires extended side surfaces; the exact Torus source
   has no such extension.
4. The exact source and boundary solver currently do not share a complete
   geometric contract for profile/frame placement.

The V=0.75 discrepancy is a symptom of this contract mismatch, not a reason to
change the Torus generator.

## 4. Independent oracle required

Before any further implementation, add an oracle that does not use the rail
solver:

For a proposed seam curve and two source surfaces:

1. Sample the seam (including segment boundaries and the closed-profile seam).
2. For each sample, project to both source surfaces with `drop_point`.
3. Require success, gap within the declared geometric tolerance, and U/V inside
   each source domain.
4. Check seam continuity and that adjacent samples share endpoints/vertices.
5. Check that the seam bounds the intended profile interval on both sides.
6. Compare the solver seam with this oracle and report the first failing sample,
   its piece, source U/V, and whether it is original or extension geometry.

For the exact-source pipeline the oracle must additionally require:

- the seam lies on the original arc portions, or
- an exact surface extension exists; otherwise the strategy must return a typed
  refusal.

## 5. Architectural decision needed

Choose one contract before porting further:

A. **Exact source only:** `HybridAtJunction` is not applicable to exact
   arcâ€“arc; return `UncoveredProfile` when the seam leaves the original source
   domain. Design a different valid joint construction for this case.
B. **Extended source:** exact sources must provide an exact extension surface
   (a tangent extension of a Torus is not a Torus), or the pipeline must build
   the extension as a separate exact/Level B piece.

Do not add another profile-placement or seam-point correction before this
decision.

## Test state

`high_level_open_arc_arc` remains ignored. Boundary Solver, Direct builder and
tolerances are unchanged.

## Contract A fixed

The accepted baseline is **A. Exact source only**: a candidate seam is valid only when every sample lies on both bounded source surfaces and inside both UV domains. HybridAtJunction may still run, but its result must pass the independent validator; if it depends on a tangent extension outside an exact source, it is not accepted.

## Validator API

oundary_validation::validate_joint_boundary now checks:

- source-surface membership with drop_point;
- source U/V domain membership;
- segment-to-segment continuity;
- stored seam-vertex consistency;
- seam-loop closure;
- maximum surface gap.

It does not use the rail solver or its residual.

## Oracle results

- Exact Line–Arc–Line: 68 samples, max source gap 7.136e-8, validator PASS.
- Mixed Exact/Level B Line–Ellipse–Line: 68 samples, max source gap 1.320e-6, validator PASS.
- Open Arc–Arc candidate: validator returns SurfaceMembershipFailure with gap 2.50e-1; the candidate remains rejected.

The open arc–arc strategy remains a separate construction problem. high_level_open_arc_arc stays ignored.

