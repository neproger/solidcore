# 41 — API boundary audit

Status: documentation/visibility audit. No solver, builder or tolerance change.

## Public exports

The research crate has one public module namespace. Exports relevant to
PlanarSweep:

| Export | Status | Responsibility |
|---|---|---|
| `planar_pipeline::build_planar_sweep` | **validated high-level API** | normalize, source select, solve, validate, build |
| `planar_pipeline::PlanarSweepResult` | high-level result | created only by `build_planar_sweep` |
| `planar_pipeline::PlanarSweepError` | high-level error | preserves stage and validator reason |
| `boundary_validation::validate_joint_boundary` | public acceptance gate | independent source membership/continuity check |
| `direct_brep::*` | **prototype low-level API** | accepts solved surfaces/boundaries; no source selection or validation |
| `boundary::BoundarySolver` | prototype building block | candidate boundary only |
| `source_selection::select_sweep_sources` | prototype building block | source/surface selection only |
| `exact_geometry`, `level_b_geometry`, `planar_sweep2` | prototype building blocks | geometry/normalization only |

## External reachability

Because this is an out-of-workspace research crate, all of the above are
reachable by an external crate when the `solidcore` feature is enabled. There is
no `pub(crate)` firewall today.

That is an accepted prototype boundary, not a production API guarantee:

- `direct_brep` is explicitly marked **prototype low-level API** in its module
  docs;
- the root crate docs state that the validated high-level path is
  `build_planar_sweep`;
- callers of the low-level builders own source and boundary correctness.

## Bypass status

The low-level `direct_brep` entry points can bypass validation. This does not
violate the current high-level contract because they do not create a
`PlanarSweepResult`. Production integration must use `build_planar_sweep`, or a
future production crate must narrow these modules to `pub(crate)`/internal.

## Verified invariants

- `PlanarSweepResult` is constructed only in `build_planar_sweep`.
- Every `JointBoundary` is validated before `build_direct_chain_surfaces`.
- Validation errors survive as
  `PlanarSweepError::BoundaryValidation(BoundaryValidationError)`.
- The Open Arc–Arc negative test checks the exact error fields and does not
  receive `PlanarSweepError::Chain`.
- Positive regression cases cover Torus, SurfOfExtrusion and BSpline sources.

## Decision

Keep the low-level functions public for the research prototype and explicitly
label them prototype/internal. Do not treat them as the production PlanarSweep
API. Narrowing visibility is a separate production-integration decision.
