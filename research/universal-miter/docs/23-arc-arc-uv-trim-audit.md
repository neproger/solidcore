# 23 — Arc–arc UV trim audit: localized transition

Status: direct B-rep still **BLOCKED**, but the failing transition is now
localized to a specific side face/loop and the periodic-profile V branch.

## 1. Target

Same arc–arc hybrid case as `21-arc-arc-direct-blocked.md`, built through
`build_direct_chain`. The 3D geometry audit (`22`) already showed the miter
curve lies on both surfaces (`3.779e-6`), so this audit only inspects UV edge
trims.

Run manually:

```sh
cargo test --features solidcore --test direct_joint -- --ignored --nocapture arc_arc_uv_trim_audit
```

## 2. Failing loop

`check_brep` reports the V gap `0.7499832976013534`. The audit prints the
loops whose adjacent UV trim endpoints differ by more than `1e-3`.

The relevant faces are the two **side faces of piece 0** and the adjacent caps
(`FaceId(0)`, `FaceId(1)`, `FaceId(2)`, `FaceId(3)`). On `FaceId(0)`,
`LoopuseId(1)`, the side face has four UV trims:

| Loop index | Edge | UV start | UV end | Role |
|---|---|---|---|---|
| 1 | `EdgeId(12)` | `(0.760359, 0.999983)` | `(0.760359, 0.999983)` | rail at profile boundary 0 |
| 2 | `EdgeId(8)` | `(0.760351, 0.0)` | `(0.612640, 0.25)` | seam edge on the arc side |
| 3 | `EdgeId(13)` | `(0.0, 0.25)` | `(0.612640, 0.25)` | rail at profile boundary 1 |

The loop's periodic transition is between:

- the rail `EdgeId(12)` at `V ≈ 0.999983`, and
- the seam `EdgeId(8)` / rail `EdgeId(13)` at `V = 0.0` / `0.25`.

The direct builder's rail for profile boundary 0 is on `V ≈ 1`, while the
shared seam trim on the arc side is projected onto `V = 0`. The 3D seam is the
same curve (drop gap `~1e-9`), but the **UV trim coefficients are on opposite
periodic representatives**. That is the `0.749983...` gap: `0.999983 - 0.25`.

## 3. Which operation

The failure is not in the 3D seam curve and not in the rail/edge vertex
assignment. The edge trims are created by
`create_uv_trim_curve`/`create_uv_trim_curves` from the shared 3D edge, after
the side face already exists. The seam trim on the arc side is parameterized
with `V≈1`, while the builder's rail at profile boundary 0 is an iso-curve at
`V≈0`. The loop then closes with two different periodic values.

Two attempts did not change the check result:

- explicit `uv_curves` (canonical seam UV branch passed to
  `FaceTopologyOptions`);
- a post-trim pass that shifts constant-V rail UV trims near `v_max` to
  `v_min` on side faces.

The trim stage still leaves the same loop transition. This is recorded as an
attempt, not claimed as a fix.

## 4. Required fix

The fix belongs in the periodic profile trim construction, not in the Boundary
Solver or the Surface Builder:

- make the seam UV trim, the rail at profile boundary 0, and the cap profile
  edge use the **same periodic V representative**;
- apply that consistently in the trim stage (or supply a UV curve that the
  trim stage actually accepts);
- do not special-case arc–arc; line–line and line–arc must keep their current
  `V=0` behavior.

The architectural follow-up is `24-direct-brep-architecture-audit.md`: the
side surfaces should be split into K non-periodic patches as in
`build_direct_single`, rather than repairing a loop over a full periodic
surface.
