# 21 — Arc–arc matrix: boundary PASS, direct B-rep blocked

Status: **arc–arc boundary matrix PASS; arc–arc direct B-rep BLOCKED; 3D
geometry audit PASS (`22-arc-arc-geometric-consistency-audit.md`).** The
N-segment builder remains PASS for the tested line–line–line chain.

## 1. Boundary results

Test pair: center `(0,1)`, radius `1`, `-180°..-90°`; center `(-1,0)`,
radius `1`, `0°..90°`; circular profile radius `0.25`, `K=4`.

| Check | Result |
|---|---|
| `HybridAtJunction` | selected |
| exact / junction samples | 36 / 32 |
| first junction `alpha` | `-0.022463609...` |
| max rail residual | `6.995e-14` |
| `ExactMiter` | typed `UncoveredProfile` refusal |

So the existing strategy pipeline handles the arc–arc boundary: the exact side
is exact and the missing side uses the junction tangent continuation. No new
strategy was added.

## 2. Direct B-rep failure

Passing the hybrid `JointBoundary` to `build_direct_chain` builds the topology
(2 pieces, `K=4`), but `check_brep` reports 11 failures:

- `UVEdge/UVEdge Gap`: V-direction gap `0.7499832976013534` (tolerance `0.1`);
- `Edge tolerance relative to brep tolerance`: `1.4142135623730947`;
- `Vertex tolerance relative to brep tolerance`: `1.4142135623730947`;
- associated bad-edge pointer checks.

This is not a boundary-solver failure. The seam points drop onto both side
surfaces with gaps around `1e-9`.

## 3. Localized cause

The arc–arc seam drops to the **other branch of the periodic profile V**:

| Side | Seam V values at the four profile boundaries |
|---|---|
| a | `0.9999832976`, `0.2500000000`, `0.5000167024`, `0.7500000000` |
| b | `0.9999832976`, `0.2500000000`, `0.5000167024`, `0.7500000000` |

The line–line and line–arc cases have `0.0, 0.25, 0.5, 0.75`: their first
profile boundary is at `V = 0`. The arc–arc case has the first boundary at
`V ≈ 1` even though `V = 0` and `V = 1` are the same geometric profile point.
The direct builder's cap/rail/seam trims therefore mix the two periodic
representatives, and the face loops report a V gap of about `0.75`.

A trial canonicalization of the seam V near `v_max` to `v_min` did not fix the
face-loop trims. A second trial supplied explicit seam UV trim curves to
`FaceTopologyOptions` (with `calculate_trims = true` and the canonical V
branch); `check_brep` still reports the same 11 failures. The UV-curve path is
therefore not yet reaching the trim in the needed form, or the gap is created
elsewhere in the loop/trim construction.

## 4. What works

- All existing line–line, exact line–arc and hybrid line–arc direct bodies.
- The N=3 line–line–line chain.
- Arc–arc boundary construction and typed exact refusal.

## 5. Required fix before adaptive work

The direct builder must use one periodic representative consistently across:

1. the seam curve's UV trim,
2. the rails created from the seam V,
3. the start/end cap profile edges.

Options to investigate:

- normalize the periodic profile parameter to `[0,1)` in the builder, including
  the seam curve's UV projection, not just the rail V;
- construct the seam/cap/rail trims from **surface iso/edge curves** at the
  chosen V branch instead of a free 3D interpolated seam curve;
- have the boundary solver return the seam UV branch, so the builder does not
  have to infer it.

Only after this is fixed should the arc–arc matrix be repeated with
`build_direct_chain`, followed by N=3 arc–arc–arc. AdaptiveCut remains blocked
by this B-rep parameterization issue, not by a strategy issue.

## 6. Current decision

- Keep `ExactMiter` and `HybridAtJunction` unchanged.
- Keep the N-segment chain unchanged.
- Do not claim arc–arc direct PASS.
- Do not add AdaptiveCut until the periodic V seam trim is fixed.
