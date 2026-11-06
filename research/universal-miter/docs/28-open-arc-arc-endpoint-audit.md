# 28 — Open arc–arc endpoint/trim audit

Status: the open arc–arc blocker is localized to **curve endpoints at the cap/side
junctions**, not to the closed-path cap plane or the periodic UV branch.

## Measured failures

`check_brep`: 4 failures, B-rep tolerance `2e-5`.

First failure: `Edge/Edge Gap = 3.48811487814872e-5`.

| Face | Loop | Transition | Gap | Current curve end | Next curve start |
|---|---|---|---|---|---|
| `FaceId(4)` | `LoopuseId(9)` | `EdgeId(16)` → `EdgeId(1)` | `3.488114878e-5` | `(-0.2928932188, 0.7071067812, -0.25)` | `(-0.2929178835, 0.7070821165, -0.2499999976)` |
| `FaceId(4)` | `LoopuseId(9)` | `EdgeId(8)` → `EdgeId(16)` | `3.488289081e-5` | `(0, ~0, -0.25)` | `(3.488236577e-5, 1.913872469e-7, -0.25)` |
| `FaceId(0)` | `LoopuseId(1)` | `EdgeId(12)` → `EdgeId(8)` | `3.963312448e-5` | `(2.039375661e-7, 3.963259978e-5, -0.25)` | `(0, ~0, -0.25)` |

The corresponding loop gaps are of the same order as the Level B side-surface
deviation (`4.280e-5`). There is no `sqrt(2)` tolerance inflation and no cap
plane mismatch in the open case.

## Classification

The gap is between **3D curve endpoints** of adjacent edges in a face loop. It
is not yet established whether the shared vertex lies on both curve ends; that
is the next measurement. The likely location is a cap-profile-edge ↔ rail
junction, where the edge curves are created from surface/patch boundaries.

## Next

For the specific pairs (`EdgeId(16)`/`EdgeId(1)` and `EdgeId(8)`/`EdgeId(16)`):

1. compare each curve endpoint with its shared vertex;
2. compare the UV trim endpoints and the patch boundary parameter;
3. check edgeuse orientation and curve interval;
4. compare with the passing open line–arc loop.

Do not snap, shift or raise tolerance. The open arc–arc test remains ignored
until these endpoint sources are established.
