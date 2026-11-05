# 20 — N-segment direct B-rep builder (PASS)

Status: **PASS** for an open chain of `N` path pieces with `N - 1` shared
joints. The first test is `N = 3` line–line–line with `K = 4` profile
contour segments. The existing 2-segment `build_direct_joint` is now a wrapper
around the same chain builder, so all exact and hybrid joint tests exercise the
general topology.

The builder accepts already solved surfaces and joint boundaries. It does not
know whether `ExactMiter`, `HybridAtJunction` or a future strategy produced
them.

## 1. API

```rust
build_direct_chain(
    kernel,
    side_surfaces: &[BSplineSurface],   // N
    boundaries: &[JointBoundary],       // N - 1
    path_start, path_start_tangent,
    path_end, path_end_tangent,
    segments,                           // K profile contour segments
) -> Result<Brep>
```

`N = 1` delegates to `build_direct_single`. `build_direct_joint` is the
`N = 2` specialization and delegates to the chain.

## 2. Topology

For `N` pieces and `K` profile segments:

- side faces: `N * K`; caps: 2; total `N*K + 2`;
- edges: start cap `K`, end cap `K`, joint seams `(N - 1) * K`, rails `N * K`;
  total `(2N + 1) * K`;
- vertices: start cap `K`, end cap `K`, joint seams `(N - 1) * K`;
  total `(N + 1) * K`;
- every edge has 4 edge-uses;
- Euler: `V - E + F = 2` per shell.

Every joint owns one set of seam vertices and seam edges, shared by the two
adjacent side surfaces. Every intermediate surface owns one rail per profile
boundary. No joint face is duplicated. The start and end surfaces share their
profile edges with the caps.

## 3. N = 3 result

Path: `(0,0) -> (1,0) -> (1,1) -> (2,1)`, circular profile radius `0.25`,
`K = 4`, circular profile segments. Both joints select `ExactMiter`.

| Check | Value |
|---|---|
| Faces / edges / vertices | 14 / 28 / 16 |
| Regions / shells | 2 / 2 |
| Edge uses per edge | 4 |
| Euler per shell | `16 - 28 + 14 = 2` |
| `check_brep` | 0 failures, assert true |
| `integrity::check` | pass |
| `is_manifold_solid` / `is_outward_solid` | true / true |
| Volume | `5.890492241e-1` |
| Analytic `area × centreline length` | `5.890486225e-1` |
| Relative | `1.021e-6` |

The 2-segment exact line–line, exact line–arc and hybrid line–arc tests are
unchanged numerically and now run through the same chain builder.

## 4. Findings

1. **Profile-loop cap closure.** The last cap/profile edge must close at
   `v_max`, not at the first seam's `V = 0`. Using `V = 0` for the closing edge
   produced loop edge gaps at the profile boundary `K-1 -> 0`; using `v_max`
   fixed them. The surface profile is a periodic loop with both parameter ends
   at the same geometric point.
2. **Profile V consistency.** Each intermediate surface is checked for the same
   profile V at both adjacent joints. A side surface with different V at its two
   ends needs a non-iso rail; the builder refuses instead of silently making a
   wrong rail.
3. **Shared seam ownership.** Joint vertices/edges are created once and used by
   both adjacent faces. This keeps four edge-uses per edge and no duplicate
   joint faces.
4. **Strategy independence.** The reference test uses two `ExactMiter`
   boundaries. The 2-segment hybrid test reaches the same builder through the
   wrapper, so the topology is independent of the boundary strategy.

## 5. Limitations

- N=3 test is line–line–line; an intermediate curved piece with two joints is
  next.
- The profile V at both ends of one surface must match within `1e-6`; otherwise
  the builder returns a typed error.
- Open paths only; closed/periodic path caps and seam closure are not in this
  step.
- No Boolean, trim or stitch is used; the builder still relies on the direct
  `make_shell_vertex`/`make_wire_edge`/`make_face_topology` path.

## 6. Next

The arc–arc boundary matrix is `HybridAtJunction` PASS, but the arc–arc direct
B-rep is blocked on the periodic profile V seam trim
(`21-arc-arc-direct-blocked.md`). Fix that before the N=3 arc chain or the
adaptive path cut.
