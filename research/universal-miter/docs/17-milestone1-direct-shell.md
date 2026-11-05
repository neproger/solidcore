# 17 — Milestone 1: Direct shell, N=1 (PASS)

Status: **PASS** for one path piece with a circular profile: straight and arc
paths, `K = 4` profile contour segments. One B-rep, one solid region and one
shell, no Boolean, `ProjectAndTrim` or `StitchFaces` in the construction.

The test is `prototype/tests/direct_shell.rs`; the report is
`results/reports/v10-milestone1-direct-shell.txt`.

## 1. Construction

```
build_swept_surface (Level B)
  -> split the NURBS surface at V into K profile patches
  -> from each patch: U-min/U-max profile segments; full-surface V rails
  -> make_shell_vertex: 2K shared vertices (rail start/end)
  -> make_wire_edge: K start segments, K end segments, K rails
  -> make_face_topology: K side patches
  -> make_face_topology: start cap (start loop reversed), end cap
  -> region finalization; refresh
```

No `create_faces_with_curves`, `stitch_faces`, `project_and_trim` or Boolean.

### Orientation

The Level B NURBS side surface normal is inward relative to the material, so
the side faces use `OrientType::Opposite`; the caps use their outward plane
normals with `OrientType::Same`. The start cap loop is reversed and the end cap
loop is forward, as in the AOIM prism. This is a B-rep orientation attribute,
not a geometry change.

### Region finalization

`reset_infinite_region` (the pre-split infinite region) and
`set_region_is_void_flags_for_nested_solids` are used; `brep.refresh_cached_topology`
follows. These are topology finalization, not a joint mechanism.

## 2. Results

Test command:

```sh
cargo test --manifest-path research/universal-miter/prototype/Cargo.toml \
  --features solidcore --test direct_shell -- --nocapture
```

| Check | Straight | Arc (R=2, 90°) |
|---|---|---|
| Surface deviation (`max_deviation`) | `6.598e-15` | `4.347e-6` |
| Faces / edges / vertices | 6 / 12 / 8 | 6 / 12 / 8 |
| Regions / shells | 2 / 2 | 2 / 2 |
| `check_brep` pointers / assert / failures | 1000 / true / 0 | 1000 / true / 0 |
| `integrity::check` | pass | pass |
| Edge uses per edge | 4 | 4 |
| `is_manifold_solid` | true | true |
| `is_outward_solid` | true | true |
| Volume | `3.926991401e-1` | `6.168519280e-1` |
| Analytic volume | `3.926990817e-1` | `6.168502751e-1` |
| Production reference | `3.926990817e-1` | `6.168502686e-1` |
| Relative to analytic | `1.486e-7` | `2.680e-6` |

The analytic volume is `area × centreline length`: `π r² L` for the straight
and `π r² (π R / 2)` for the arc. The production reference is the existing
`create_sweep_along_planar_path` on the same inputs, used only as a comparison.

The two regions are the expected void/solid pair; each shell has all 6 faces
and 6 faceuses; each edge has 4 edge-uses (2 `Same` + 2 `Opposite`). Euler:
`V - E + F = 8 - 12 + 6 = 2` per closed shell.

## 3. Volume-query accuracy

`brep_volume_default` is called with integration accuracy `1e-3`, the same
order `is_outward_solid` uses for the volume sign. With `1e-9` the integrator
returns `SM_ERR` (no convergence) on the direct NURBS faces; that is a numerical
integration limit, not a geometric tolerance. At `1e-3` the volume matches the
analytic and production references to the printed digits (`1.5e-7` straight,
`2.7e-6` arc). The test asserts `<= 1e-5` relative.

## 4. API findings

1. **p6 closure tail.** The last `make_face_topology` still reports
   `Unported("SmBrep::TestClosureSplitRegion", "topology.brep_p6")` after the
   split. The split and the two shells/regions complete; the caller treats only
   that exact Unported as the simple-prism completion and validates the shell
   independently. This remains a kernel port gap for general cases.
2. **Side-face orientation.** With `OrientType::Same` on the side NURBS
   patches, `check_brep` reports `Face Loop has bad orientation` and
   `is_outward_solid` is false. `OrientType::Opposite` fixes the orientation
   without changing geometry. The Level B surface normal is inward for this
   profile placement.
3. **`Shell.faces` cache.** After refresh both shells report `faces=6`,
   `faceuses=6`; the empty-cache finding from the older AOIM probe does not
   reproduce here.
4. **Volume accuracy.** `1e-9` fails to converge; `1e-3` matches the
   references. This is documented, not silently loosened.

## 5. Limitations

- N=1 only; no joints yet.
- Circular profile, `K=4`; no arbitrary or multi-curve profile loops.
- No open profile, holes or closed/periodic path.
- The p6 `TestClosureSplitRegion` tail is still unported; the simple prism
  split completes, but the general tail is not available.
- The volume query's convergence limit at `1e-9` is a kernel numeric limit.
- The production sweep is used only as the volume reference; the direct
  construction never calls it.

## 6. Next

Milestone 2: the boundary-strategy interface and the exact line–line joint with
shared topology, followed by the curved/hybrid strategies in Milestone 3.
