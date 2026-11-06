# 43 — GeometryPolicy and provenance contract

Status: contract tests and API surface only. No extension or bridge algorithm.

## 1. Policy API

`planar_pipeline::GeometryPolicy`:

```rust
enum GeometryPolicy {
    ExactOnly,          // default
    AllowExtension,
    AllowAdaptiveBridge,
}
```

`build_planar_sweep_with_policy` is the explicit policy entry point.
`build_planar_sweep` remains the ExactOnly default wrapper.

For the unimplemented policies the result is:

```text
PlanarSweepError::StrategyNotImplemented(policy)
```

The pipeline does not silently choose Exact, HybridAtJunction, extension or a
bridge.

## 2. Provenance

`source_selection::GeometryProvenance` is carried by every `SelectedSweep`:

```rust
enum GeometryProvenance {
    Original,
    Extension,
    Bridge,
}
```

The current implementation produces only `Original` pieces. Any future
generated extension or bridge must be a separate piece with `Extension` or
`Bridge` provenance; it must not be relabeled as original.

`PlanarSweepResult` carries `geometry_policy`. Generated pieces/interfaces are
not produced yet; the data model is ready to add them without changing the
ExactOnly path.

## 3. Contract tests

- default policy is `ExactOnly`;
- ExactOnly provenance is `Original` for all pieces;
- `AllowExtension` returns `StrategyNotImplemented` without building;
- `AllowAdaptiveBridge` returns `StrategyNotImplemented` without building;
- the existing validated Exact tests remain green.

## 4. Guardrails

- policy never leaks into ExactOnly;
- an unimplemented policy never falls back to another strategy;
- generated geometry must never be reported as original;
- the independent validator will receive actual original/generated Surface
  objects when those strategies exist;
- false exactness is forbidden: exact NURBS generated geometry is not the same
  guarantee as preserving the original sweep.

## 5. Status

- Exact Contract A: unchanged.
- GeometryPolicy: public, ExactOnly default.
- Provenance: present on selected sources, all Original.
- Extension/Bridge implementations: not started.
- Solver/builder/validator/tolerances unchanged.
