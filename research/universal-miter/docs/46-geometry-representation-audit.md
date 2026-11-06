# 46 — Geometry representation audit: kernel ↔ normalized ↔ placement

Status: audit only. No geometry, solver, builder or tolerance change.

## 1. The three representations

| Name | Meaning | Main producer |
|---|---|---|
| kernel/original | The caller's input curves and frames | `path_original`, profile inputs |
| normalized | Prepared paths/profiles, extended paths, local frames | `planar_sweep2::normalize_planar_sweep`, `prepare_piece` |
| placement | Per-piece world placement and prepared profile data | `NormalizedPlacement` |

## 2. Actual current data flow

| Consumer | Geometry used |
|---|---|
| Exact Revolution | `normalized.path_original` (original arc/frame), plus a profile derived from placement and original start |
| Exact Extrusion | `placement.path_piece` (extended path) and `placement.world_profiles` |
| Level B | `placement.path_piece` (extended path) and `placement.world_profiles` |
| Boundary Solver | `normalized.path_original` converted to B-spline by the high-level pipeline; `BoundaryProblem.input.profile` |
| Validation / Direct builder | Actual `Surface` objects from selected sources |
| Caps | `SelectedSweep.path_start/end/tangents` and source-surface sections |

## 3. Audit findings

### 3.1 Original and extended paths are different domains

`path_piece` is the extended path. It is not the same parameter curve as
`path_original`; observed open arc endpoints differ:

```text
piece 0 original end = (0, 0)
piece 0 prepared end = (0.7071, 0)
piece 1 original start = (0, 0)
piece 1 prepared start = (0, -0.7071)
```

So an Exact Extrusion/Level B surface built on `path_piece` and a boundary built
on `path_original` are not guaranteed to share the same coverage. The line cases
work because the extension is straight and compatible, not because the
representations are identical.

### 3.2 Profile placement has two paths

- `world_profiles` is the prepared profile mapped back by
  `rotation_back` and `path_back`.
- Exact Revolution additionally uses `original_arc_profile`, which moves the
  placement profile to the original start.
- Exact Extrusion and Level B use `world_profiles` directly.

These are not one canonical profile representation.

### 3.3 Parameter and orientation preservation

`prepare_piece` applies:

1. path translation to origin;
2. profile reference translation;
3. a path-start frame rotation;
4. inverse transforms for `world_profiles`.

Rigid transforms preserve parameterization in principle, but the path extension
changes the domain before this step. The `path_original` ↔ `path_piece`
correspondence is therefore not established by a single parameter map in the
current types.

### 3.4 Boundary/Source contract

The source surface and Boundary Solver can currently use different path
coverage:

- source: original for Exact Revolution, prepared/extended for Exact Extrusion
  and Level B;
- boundary: original paths;
- HybridAtJunction: tangent extensions inside the solver;
- exact Torus: no extension surface.

The independent validator now catches the resulting surface membership failures,
but the geometry contract is still implicit.

## 4. Required contract before BoundaryStrategyRegistry

Every `SourceProvider` must return a source contract that includes:

- the authoritative path representation (`original` or `prepared`);
- that path's parameter domain and orientation;
- the source-surface UV domain;
- the exact profile placement and its frame;
- whether tangent extension coverage is included;
- the mapping used by the boundary solver, if different.

`BoundaryProblem` must carry the same authoritative representation, not merely a
reference to `BoundaryInput` plus surfaces.

The boundary solver must either:

- use the source's original bounded path and refuse uncovered seams; or
- use an explicitly declared extension geometry that is also present in the
  source contract.

It must not mix original and prepared paths implicitly.

## 5. Recommended next refactor

Add a `SourceGeometry`/`BoundaryGeometry` representation contract:

```text
SourceProvider
  -> SourceGeometry {
        surface,
        path,
        path_domain,
        profile,
        profile_domain,
        coverage: OriginalOnly | WithExtension,
        provenance
     }
          |
          v
BoundaryProblem {
        geometry_a,
        geometry_b,
        policy
     }
```

The boundary solver consumes the declared geometry, and the validator checks the
same surfaces and domains. The high-level pipeline then has one authoritative
representation per joint.

## 6. Guardrails

- Do not silently reapply `placement`.
- Do not substitute prepared path for original path without declaring it.
- Do not allow a source provider to build an extended surface and then let the
  boundary use the original bounded path without flagging the mismatch.
- Keep Exact Contract A and typed refusal unchanged.
- Keep the independent Arc–Arc oracle test-only.

## 7. Status

- SourceProvider registry: implemented, explicit order/fallback.
- Geometry representation contract: **not yet unified**.
- BoundaryStrategyRegistry: not started.
- Current behavior remains green through validation and refusal tests.
