# 45 — BoundaryProblem contract and policy/provenance

Status: architectural refactor step. No new geometry strategy.

## 1. BoundaryProblem

`boundary::BoundaryProblem` is now the common input for boundary solving:

```rust
pub struct BoundaryProblem<'a> {
    pub input: BoundaryInput<'a>,
    pub surface_a: &'a Surface,
    pub surface_b: &'a Surface,
    pub domains: [Extent2d; 2],
    pub policy: GeometryPolicy,
}
```

`BoundaryStrategy` receives it through:

```rust
fn solve_problem(&self, problem: &BoundaryProblem<'_>) -> Result<JointBoundary>
```

The default implementation delegates to the existing source-agnostic `solve`;
future strategies can use the source surfaces, domains and policy directly.

`BoundarySolver::solve_problem` is used by `build_planar_sweep`. The exact
geometry and boundary behavior are unchanged.

## 2. GeometryPolicy

`GeometryPolicy` now lives in `boundary` and is re-exported by
`planar_pipeline`:

```rust
enum GeometryPolicy {
    ExactOnly,
    AllowExtension,
    AllowAdaptiveBridge,
}
```

`ExactOnly` is the default. The unimplemented policies return
`PlanarSweepError::StrategyNotImplemented`; they never switch to Exact, Hybrid,
Extension or Bridge silently.

## 3. Provenance

`source_selection::GeometryProvenance` is carried by every `SelectedSweep`:

```rust
enum GeometryProvenance {
    Original,
    Extension,
    Bridge,
}
```

Current results contain only `Original` pieces. Generated pieces/interfaces are
not produced yet; `PlanarSweepResult.geometry_policy` records the policy.

## 4. What did not change

- Exact Contract A.
- `validate_joint_boundary`.
- Typed refusal.
- Existing rail-solver mathematics.
- Direct B-rep Builder.
- Tolerances.
- Independent Arc–Arc oracle.

## 5. Remaining architecture work

1. SourceProvider registry: remove profile/source branching from
   `source_selection` internals.
2. Strategy registry/classification: remove the hardcoded
   `junction_samples` choice from `BoundarySolver`.
3. Generated pieces/interfaces in `PlanarSweepResult`.
4. Architecture tests for adding a source provider/strategy without changing
   `build_planar_sweep`.

## 6. Verification

- Exact/mixed high-level pipeline: `7 passed`, `0 ignored`.
- Open Arc–Arc returns the exact typed refusal.
- Geometry policy default and provenance tests pass.
- `clippy --tests -D warnings` and `rustfmt --check` are clean.

## 7. SourceProvider registry

source_selection now has:

- SourceInput (kernel, normalized sweep, placement, index, Level B options);
- SourceSupport::{Applicable, NotApplicable, Unsupported};
- SourceProvider { kind, supports, build };
- ExactExtrusionProvider, ExactRevolutionProvider, LevelBProvider;
- SourceProviderRegistry with explicit first-applicable selection.

classify_piece and select_sweep_sources use the registry. An exact provider that is applicable but fails to build returns SourceSelectionError; it is not silently retried as Level B.

The registry is still a fixed internal list in the high-level pipeline; a public injection point for arbitrary provider sets remains future work.


## 8. SourceProvider registry audit

SourceSupport now distinguishes:

- Applicable: an exact provider claims the piece;
- NotApplicable: outside its area;
- Unsupported: its area but unsupported;
- Fallback: an explicit fallback provider (Level B).

The registry order is explicit: ExactExtrusion, ExactRevolution, LevelB. More than one exact Applicable provider returns SourceSelectionError::ProviderAmbiguous; Level B is selected only when no exact provider applies. An applicable exact provider's build failure remains final and is never replaced by Level B.

classify_piece and select_sweep_sources both use the registry.

### Remaining source-truth risk

SourceInput carries both 
ormalized and placement. Exact Revolution still derives its arc/frame from the original path, while Exact Extrusion uses the placed prepared path. This is documented behavior, not yet a single source-of-truth contract. The next audit must state, per provider, which geometry is authoritative for the surface and which is authoritative for the boundary.

