# 26 — Prepared boundary contract audit

Status: the cap failure is a boundary-contract mismatch. The audit does not
choose the final semantics without tracing preprocessing; both current sources
are now identified.

## 1. Measured endpoints

| Case | Original start | Prepared path-piece start | Original end | Prepared path-piece end |
|---|---|---|---|---|
| line–arc | `(0,0,0)` | `(0,0,0)` | `(0,1,0)` | `(0,1,0)` |
| arc–arc | `(-1,1,0)` | `(-1.0000000000, 1.7071067812, 0)` | `(-1,1,0)` | `(-1.7071067812, 1.0000000000, 0)` |

Side surface U domains are `[0,1]`; its boundary is at the prepared path-piece
endpoint. Cap profile edges are sampled from that U boundary. Cap planes are
built from the original path endpoints.

## 2. Measured cap-plane deviation

| Case | Start plane | End plane |
|---|---|---|
| line–arc | `1.5e-17` | `8.1e-17` |
| arc–arc | `0.7071067812` | `0.7071067812` |

The cap profile edge curve lies off the arc–arc cap plane by `0.7071`; its
endpoints still coincide with the vertices. SolidCore raises edge/vertex
tolerance to `2 * 0.7071 = 1.4142135624` during cap-face trim creation.

## 3. Contract

The current `NormalizedPlacement` has no explicit distinction between:

- `original_path_domain`;
- `prepared_path_domain` (the extended/positioned `path_piece` used by the
  surface builder);
- `final_sweep_domain` (the boundary the B-rep caps must represent).

`build_direct_chain` takes the side surface from the prepared path but the cap
plane from the original path. That is the defect.

## 4. Decision still needed

Variant A (prepared boundary final) is viable only if preprocessing guarantees
that `NormalizedPlacement::path_piece` is the intended final sweep boundary.
The current audit shows that it is different for arc–arc, but does not prove
whether the difference is an intentional final extension or an internal joint
aid.

Variant B (original endpoint final) requires trimming the side surface to the
original endpoint before extracting cap edges and rebuilding the affected
rails/trims.

## 5. Next

Trace `extend_path_curves_with_kernel`, `normalize_planar_sweep` and production
curve-sweep cap semantics to decide A or B. Then expose an explicit
`PreparedBoundaryContract` to `build_direct_chain`; the builder must not infer
which endpoint is final.

Do not change tolerances, UV trims, Boundary Solver or Surface Builder before
that decision.
