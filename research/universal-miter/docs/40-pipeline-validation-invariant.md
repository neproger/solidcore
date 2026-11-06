# 40 — High-level pipeline validation invariant audit

Status: audit and regression checks. No solver, builder or tolerance change.

## 1. Paths that can produce `PlanarSweepResult`

Only `planar_pipeline::build_planar_sweep` constructs `PlanarSweepResult`.

`normalized_input` returns only `NormalizedPlanarSweep`; it constructs no
surface or B-rep.

`source_selection::select_sweep_sources` returns selected surfaces and
diagnostics, not a B-rep.

The low-level `direct_brep` entry points are intentionally public for the
prototype, but they are not a PlanarSweep high-level result path. The high-level
contract is the `build_planar_sweep` path and always validates before calling
`build_direct_chain_surfaces`.

## 2. Validation barrier

The pipeline order is:

```text
normalize
  -> select sources
  -> solve every JointBoundary
  -> validate every JointBoundary against its two sources
  -> only then build_direct_chain_surfaces
```

A validation error becomes:

```text
PlanarSweepError::BoundaryValidation(BoundaryValidationError)
```

It preserves the validator fields: segment, sample, surface, gap, U/V and the
specific reason.

## 3. Error stability

The open Arc–Arc negative test now asserts the exact classification and fields:

```text
PlanarSweepError::BoundaryValidation(
    BoundaryValidationError::SurfaceMembershipFailure {
        segment: 2,
        sample: 1,
        surface: 0,
        gap: 2.1086334742979336e-2,
    }
)
```

It does not accept an arbitrary validation failure. The test also proves the
pipeline returns the validation error rather than `PlanarSweepError::Chain`; the
direct builder is after the validator and is not reached.

## 4. Source-type coverage

The validator takes a generic `Surface`, so it applies to every source kind:

- `SurfaceKind::Torus` — exact open Arc–Arc, validator rejects the candidate;
- `SurfaceKind::SurfOfExtrusion` — exact line pieces;
- `SurfaceKind::BSpline` — Level B mixed pieces.

Positive high-level tests cover all three categories:

```text
Exact Line–Line–Line     max_gap=8.854e-7   PASS
Exact Line–Arc–Line      max_gap=7.136e-8   PASS
Mixed Exact/Level B      max_gap=1.320e-6   PASS
```

## 5. Remaining bypass boundary

The prototype's low-level `build_direct_chain_surfaces`, `build_direct_joint`
and `build_direct_single` remain callable directly. They are not the PlanarSweep
public contract; callers using them bypass source selection and boundary
validation by design. Production integration must use `build_planar_sweep`, or
the low-level API would need an internal-only visibility change later.

## 6. Status

- validation is a mandatory barrier on the high-level path;
- the negative Open Arc–Arc case is a normal regression test, not ignored;
- diagnostic fields survive into the public `PlanarSweepError`;
- no Boundary Solver, Direct builder or tolerance change.
