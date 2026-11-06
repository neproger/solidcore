# 47 — SourceGeometry contract and BoundaryProblem migration

Status: architectural implementation. No solver/builder/validation change.

## 1. SourceGeometry

`source_selection::SourceGeometry` carries one coherent geometry view:

```rust
struct SourceGeometry<'a> {
    surface: &'a Surface,
    path: &'a Curve,
    path_domain: Extent1d,
    profile: &'a Curve,
    profile_domain: Extent1d,
    surface_domain: Extent2d,
    coverage: SourceCoverage,
    provenance: GeometryProvenance,
    path_correspondence: PathCorrespondence,
}
```

`SourceCoverage`:

```rust
OriginalOnly | WithExtension
```

`PathCorrespondence`:

```rust
Identity | Independent
```

Current provider rules:

- Exact Revolution: `OriginalOnly`, `Identity`;
- Exact Extrusion: `WithExtension`, `Independent`;
- Level B: `WithExtension`, `Independent`.

`SelectedSweep` stores the declared path/domain/coverage/correspondence and
exposes `geometry()`. `SourceGeometry` does not invent a parameter map for an
extended path; `Independent` makes the absence explicit.

## 2. BoundaryProblem migration

Before:

```rust
BoundaryProblem {
    input,
    surface_a,
    surface_b,
    domains,
    policy,
}
```

After:

```rust
BoundaryProblem {
    input,
    source_a: &SourceGeometry,
    source_b: &SourceGeometry,
    policy,
}
```

The boundary problem no longer receives independent `surface` and `domain`
arguments that could come from different geometry objects. The high-level
pipeline constructs both source geometries from the selected pieces and hands
them to `BoundarySolver::solve_problem`.

## 3. Contract tests

`source_selection_builds_mixed_exact_chain_inputs` checks:

- `surface_domain` matches the selected surface domain;
- `profile_domain` matches `profile.natural_interval()`;
- `path_domain` matches `path.natural_interval()`;
- all current provenance is `Original`;
- Exact Revolution is `OriginalOnly/Identity`;
- Exact Extrusion and Level B are `WithExtension/Independent`.

## 4. Caveats

- `SelectedSweep` still carries legacy convenience fields (`surface`, `profile`,
  path endpoints) alongside the geometry contract. They are populated together
  in `finalize_selected`; a later cleanup can remove the duplicate accessors.
- `BoundarySolver` still uses `BoundaryInput` for its existing rail mathematics.
  Its `path_a`/`path_b` are the original paths and are documented as such;
  `SourceGeometry.path` may be the extended path when coverage is
  `WithExtension`. There is no implied parameter map.
- The pipeline takes validator surfaces, builder surfaces and cap path frames
  from `SourceGeometry`.
- Public registry injection, generated pieces, Bridge/Extension and
  `BoundaryStrategyRegistry` remain unimplemented.

## 5. Verification

```text
source_selection:                3 passed
exact line-arc-line:             PASS
open arc-arc typed refusal:      PASS
clippy --tests -D warnings:      clean
rustfmt --check:                 clean
```

Exact Contract A, validator, solver, Direct builder and tolerances are
unchanged.
