# 44 — PlanarSweep architecture audit

Status: audit only. No new geometry strategy, no solver/builder change.

## 1. Where the code depends on profile shape

### Circular-profile dependencies

- `source_selection::is_circular_profile` gates ExactRevolution.
- `source_selection::classify_piece` maps line + supported profile to
  ExactExtrusion, circular arc + circular profile to ExactRevolution, and
  everything else to Level B.
- `source_selection::all_arc_paths` and `original_arc_profile` are a special
  source-placement path for arc-only chains.
- `exact_geometry::arc_sweep_surface` / `arc_sweep_surface_with_profile` build
  a Torus, which is inherently a circular-profile construction.
- `tests/arc_arc_oracle.rs` uses circle–circle geometry. It is a test oracle,
  not a pipeline branch.

### Not profile-specific

- `planar_sweep2` normalization.
- `boundary` rail solver and `BoundaryStrategy` interface.
- `boundary_validation` (generic `Surface`).
- `direct_brep` (generic `Surface`).
- `level_b_geometry` (generic profile).
- `GeometryPolicy` / provenance.

## 2. Universal operations vs specialized solvers

Universal:

- path/profile normalization;
- source selection interface;
- boundary solving on planar curves;
- boundary validation;
- direct topology assembly;
- policy/provenance/refusal handling.

Specialized solvers:

- exact extrusion (line sweep);
- exact Torus/revolution (circular profile);
- Level B NURBS approximation;
- circle–circle Arc–Arc oracle (test).

The specialized solvers are legitimate; the problem is that their selection
currently lives inside `source_selection` rather than behind a uniform source
contract.

## 3. Relation to the four 447 strategies

Implemented behind `BoundaryStrategy`:

- `ExactMiter`;
- `HybridAtJunction`;
- `HybridAtLimitTangent` (negative control).

Not implemented:

- `HybridAtLimitBisector`;
- `AdaptiveCut`.

`BoundarySolver` currently hardcodes the choice between ExactMiter and
HybridAtJunction from `junction_samples`, rather than selecting from a strategy
registry. Adding the remaining strategies would require extending selection and
the geometry policy; `build_planar_sweep` itself need not know each strategy.

## 4. Duplication

- The rail solver computes a 2D miter/offset boundary.
- Exact source builders compute Torus/Extrusion surfaces.
- The Arc–Arc circle oracle recomputes seam existence independently.
- `source_selection::original_arc_profile` and normalization both manipulate
  profile placement.
- `build_planar_sweep` converts original paths to B-spline for the boundary
  solver while exact sources may use original arc data.

The oracle duplication is intentional and must stay independent. The source
placement duplication is not architectural duplication yet, but indicates that
source construction needs a clearer contract.

## 5. Can a new profile or strategy be added without changing
   `build_planar_sweep`?

### New profile

- A new profile can already take the Level B path without changing
  `build_planar_sweep`.
- Adding exact support requires changes in `source_selection` (classification
  and source builder) and possibly `exact_geometry`; the high-level signature
  need not change.

### New boundary strategy

- A strategy can implement `BoundaryStrategy` without changing
  `build_planar_sweep` if `BoundarySolver`/selection is extended behind its
  existing call.
- The current solver selection is hardcoded, so that extension is missing.

### New generated geometry

- `GeometryPolicy` and provenance are in place.
- `PlanarSweepResult` can carry new generated pieces later.
- `build_planar_sweep` will still need an explicit policy-to-strategy dispatch;
  it must not become a branch per profile or curve combination.

## 6. Architectural risks

1. Profile-specific branches in source selection (`all_arc_paths`,
   circular-profile-only).
2. Strategy selection hardcoded in `BoundarySolver`.
3. Two coordinate systems: original source placement vs prepared boundary
   profile.
4. Generated geometry/provenance exists in the type model but not yet in the
   pipeline.
5. The low-level `direct_brep` bypass remains public (documented prototype).

## 7. Recommended boundary-framework shape

```text
BoundaryProblem
  surfaces A/B
  bounded UV domains
  profile correspondence
  geometry constraints
  geometry policy
        |
        v
BoundarySolver (strategy selection)
        |
        v
candidate JointBoundary
        |
        v
independent BoundaryValidator
        |
        v
Direct B-rep Builder
```

- `SourceProvider` / source registry chooses exact/Level B surface geometry;
  it does not select a joint strategy.
- `BoundarySolver` owns a strategy registry/factory over `BoundaryStrategy`.
- Every strategy returns a candidate plus diagnostics; it never bypasses the
  validator.
- `GeometryPolicy` gates generated extension/bridge strategies; ExactOnly
  remains the default.
- `build_planar_sweep` stays one pipeline; profile/strategy specifics live
  behind these contracts.

## 8. Conclusion

The current pipeline is functionally stable, but the architecture is at the
edge of a combinatorial branch set. Before adding Extension Surface or
Adaptive Cut/Bridge, extract the following contracts:

1. a profile/source-provider contract;
2. a strategy registry owned by the boundary solver;
3. a single `BoundaryProblem` input carrying surfaces, domains, profile mapping
   and policy;
4. generated-geometry provenance in the result.

Until then, keep Exact Contract A, typed refusal, and the current validator
unchanged.
