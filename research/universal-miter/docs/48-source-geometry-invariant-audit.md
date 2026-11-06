# 48 — SourceGeometry invariant audit

Status: audit only. No new geometry strategy.

## 1. Proven invariants

For every selected piece, `SourceGeometry` is built from one `SelectedSweep`:

- `surface_domain` is obtained from `surface.natural_uv_domain_with_nurb`;
- `path_domain` is `path.natural_interval()`;
- `profile_domain` is `profile.natural_interval()`;
- `provenance` is `Original`;
- provider coverage/correspondence is explicit:
  - Exact Revolution: `OriginalOnly` + `Identity`;
  - Exact Extrusion: `WithExtension` + `Independent`;
  - Level B: `WithExtension` + `Independent`.

The boundary problem now receives:

```rust
BoundaryProblem {
    input,
    source_a: &SourceGeometry,
    source_b: &SourceGeometry,
    policy,
}
```

and no longer independent `surface`/`domains` arguments.

The high-level pipeline takes the validator surfaces, the Direct builder
surfaces and the cap path frames from `SourceGeometry`. This avoids a second
surface source and a second cap-path source for those consumers.

## 2. Remaining path distinction

### 2.1 BoundaryInput path is deliberately the original path

`BoundaryProblem.input` contains:

- `path_a`/`path_b` built from `normalized.path_original`;
- `profile` from `SourceGeometry.profile`.

`SourceGeometry.path` for Exact Extrusion and Level B is the prepared/extended
`placement.path_piece`. Therefore the boundary problem still contains two path
representations:

```text
SourceGeometry.path     = prepared/extended path
BoundaryInput.path_a/b  = original path, used only for rail extension
```

The fields and `BoundaryProblem` docs now state this explicitly. The exact
revolution path is original in both places. `Independent` explains the absence
of a parameter map but does not make the two inputs interchangeable. A strategy
that needs a map must obtain it explicitly or refuse.

### 2.2 Caps now use SourceGeometry.path

`build_planar_sweep` derives the first/last cap positions and tangents with
`SourceGeometry::path_ends`, not `SelectedSweep.path_start/path_end`.

### 2.3 Legacy SelectedSweep fields

`SelectedSweep` still exposes `surface`, `profile`, path endpoints and domains
alongside `geometry()`. They are consistent in `finalize_selected`, but the
duplicate storage can still diverge after a future change.

### 2.4 BoundaryInput profile

The pipeline supplies `BoundaryProblem.input.profile` from
`SourceGeometry.profile` (through the first selected piece). The field is still
separate in `BoundaryInput`, so a future caller could pass a different profile.

## 3. Required cleanup before BoundaryClassification

1. Remove the legacy `SelectedSweep` convenience fields or make them methods
   over the canonical geometry storage.
2. Add a negative test that tries to pair an extended source path/surface with
   an original-only boundary input and requires refusal.
3. Decide whether `BoundaryInput` keeps the explicit original path or receives
   a declared `SourceCoverage` for its path.

## 4. Status

```text
SourceProvider Registry        PASS
SourceGeometry contract        IMPLEMENTED
BoundaryProblem migration      PASS for surface/domains
Geometric equivalence          PARTIAL
Cap path from SourceGeometry   PASS
BoundaryInput profile          PASS (pipeline)
Original-path declaration      PASS (documented)
Legacy field cleanup           REQUIRED
BoundaryClassification         NOT STARTED
BoundaryStrategyRegistry       NOT STARTED
```

## 5. Verification

- source selection: 3 passed;
- exact line-arc-line: PASS;
- open arc-arc typed refusal: PASS;
- clippy/rustfmt: clean.
