# 19 — Hybrid line–arc volume diagnostic (closed)

Status: **closed as a construction difference.** The hybrid volume is not
compared with the ordinary sweep: `HybridAtJunction` deliberately uses a
different joint construction, so the body and its volume differ. This record
keeps the measurements for traceability; the volume is not a correctness gate.

## Observation

| Case | Direct volume | Reference `area × centreline` | Relative | Seam |
|---|---|---|---|---|
| line–line exact | `3.926987504e-1` | `3.926990817e-1` | `8.437e-7` | exact |
| line–arc exact | `3.037933872` | `3.037915288` | `6.117e-6` | exact |
| line–arc hybrid | `5.037627012e-1` (base) / `5.037683241e-1` (fine) | `5.047746784e-1` | `2.005e-3` / `1.994e-3` | 32 junction samples |

The hybrid case has 32 of 68 seam samples on the junction tangent extension;
the exact line–arc has all 68 on both original trajectories.

## Diagnostic results

### 1. Volume integration is not the cause

On one fixed hybrid B-rep, `brep_volume_default` gives:

| Accuracy | Volume |
|---|---|
| `1e-3` | `5.037753825530e-1` |
| `1e-4` | `5.037627012363e-1` |
| `1e-5` | `5.037597106550e-1` (base surface) |

The `1e-4`–`1e-5` change is `3.0e-6` absolute (`5.9e-6` relative), not the
`1.0e-3` gap to the `area × centreline` reference.

### 2. Level B surface accuracy is not the cause

Rebuilding the same hybrid body with a finer Level B setting (`tolerance 1e-5`,
`max_refinements 8`, `max_sections 257`) gives:

| Quantity | Base (`1e-4`, 129) | Fine (`1e-5`, 257) |
|---|---|---|
| Arc-side surface deviation | `5.591e-5` | `1.267e-6` |
| Boundary deviation from both surfaces | `3.963e-5` | `1.908e-6` |
| Volume | `5.037627012e-1` | `5.037683241e-1` |
| Relative to reference | `2.005e-3` | `1.994e-3` |

The boundary deviation improves by ~20×; the volume relative changes by
`1.1e-5` relative. The 2e-3 residual is not the NURBS surface approximation.

A coarse Level B setting (`tolerance 1e-3`, 65 sections) cannot even drop the
seam vertices within the builder tolerance (`the seam point is not on the side
surface`), confirming that the base/fine settings are the meaningful range.

### 3. Seam and parametric ranges are valid

Base Level B, hybrid body:

| Side | Surface U domain | Seam U range | Seam V range | Max drop gap |
|---|---|---|---|---|
| line | `[0, 1]` | `[0.414214, 0.732233]` | `[0, 0.75]` | `2.3e-16` |
| arc / extension | `[0, 1]` | `[0.204780, 0.464609]` | `[0, 0.75]` | `3.963e-5` |

At the fine setting the arc-side seam U range is `[0.204769, 0.464578]` and the
maximum drop gap is `1.06e-6`. The seam on the arc side lies in the start
extension U region, as the junction construction requires; it is inside the
surface domain, the V sequence is ordered, and the drop gaps behave as a
surface-approximation residual. There is no out-of-domain or reversed
parametric seam.

### 4. The exact/hybrid transition is continuous

The sampled profile switches from exact samples to junction samples at
`alpha = -0.022463609...`; the seam passes through the joint at `alpha = 0`.
The B-rep validation, manifold, outward orientation and topology checks pass in
both the base and fine runs.

## Conclusion

The residual is **not** explained by:

- volume integration (stable under accuracy);
- Level B surface approximation (stable under a 20× better boundary);
- an out-of-domain or reversed parametric seam;
- a topology or orientation failure.

The exact line–arc with the same builder and profile is `6.117e-6`; the hybrid
is `1.99e-3`. The exact comparison is valid because its seam is the common
offset boundary. The hybrid comparison is not: `HybridAtJunction` uses a
different joint construction, so its body and volume legitimately differ from
an ordinary sweep. The diagnostic is closed; no independent hybrid volume
reference is required.

The hybrid volume is stable at `~5.0377e-1` (effective centerline length
`~2.5655` versus the original `2.5708`). The B-rep is judged by:

- the shared exact boundary on both extended surfaces;
- no gaps or overlaps at the seam;
- closure, manifold and outward orientation;
- `check_brep` and `integrity`;
- the direct topology without Boolean, trim or stitch.

## Current decision

- Do not use ordinary-sweep volume as a HybridAtJunction correctness gate.
- Keep the volume as a diagnostic only.
- Keep the boundary/topology checks as the gate.
- Move on to the next Universal Boundary strategy.
