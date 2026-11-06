# 35 — Exact Torus Source Geometry Audit

## Purpose

Check whether the exact Torus used for open Arc–Arc is the same geometry as the
analytic sweep, before changing seam or UV handling.

Test: `prototype/tests/torus_audit.rs`.

## Inputs

The same open Arc–Arc case:

```text
path A: center (0, 1), radius 1, -180..-90
path B: center (-1, 0), radius 1, 0..45
profile: circle radius 0.25
```

The audit compares, for each selected piece:

- source kind and surface kind;
- path start;
- profile samples and profile centre;
- surface U/V domains;
- surface-evaluated points against the analytic sweep at U = 0, .25, .5,
  .75, 1 and V = 0, .25, .5, .75, 1;
- surface centreline at each U sample.

## Frame audit

The audit prints original and prepared path endpoints separately:

```text
piece 0: original start (-1, 1), prepared start (-1, 1)
         original end   ( 0, 0), prepared end   (0.7071, 0)
piece 1: original start ( 0, 0), prepared start (0, -0.7071)
         original end   (-0.2929, 0.7071), prepared end (-0.2929, 0.7071)
```

`placement.world_profiles` has its centre at the **prepared** start:

```text
piece 0 world profile centre: (-1, 1)
piece 1 world profile centre: (0, -0.7071)
```

The exact Torus, however, takes its frame/angles from the **original** path.
For piece 1 the profile is therefore centred at `(0, -0.7071)` while the Torus
frame starts at `(0, 0)`: the two coordinate systems are different before any
seam handling.

Both pieces select `ExactRevolution` and build `Torus` surfaces. Domains are
`U=[0,1]`, `V=[0,1]`.

The audit finds large differences already at the source geometry, before any
joint topology:

```text
piece 0: max deviation 8.527e-1 at U=1, V=.75
piece 1: max deviation 8.527e-1 at U=0, V=.75
```

The surface centre at the path endpoints is also displaced from the analytic
sweep centre. This is consistent with the separate seam audit:

```text
piece 0 seam gap: 2.500e-1
piece 1 seam gap: 4.591e-1
```

The exact source currently uses:

- arc frame/radius from the original path;
- profile from `placement.world_profiles`.

Those are not yet proven to describe the same start position after path
extension, and the profile parameterization is not the same as the torus minor
parameterization. The audit therefore supports the hypothesis that the problem
is in the source frame/profile placement, not in the Boundary Solver.

## Not changed

- Boundary Solver: unchanged, still HybridAtJunction with residual 1.284e-13.
- Direct B-rep Builder: unchanged.
- Tolerances: unchanged.
- No seam-point or UV correction was added.

## Next

Align the exact Torus generator with the same original path start used for the
analytic sweep, then rerun `torus_source_geometry_audit` and open Arc–Arc through
`build_planar_sweep`. Do this before touching seam UV mapping or topology.

## Test state

`torus_source_geometry_audit` is a diagnostic test (prints, no geometry
assertion yet). `high_level_open_arc_arc` remains ignored with the same finding.

## Attempted source alignment (2026-11-06)

The all-arc source path now derives its contract profile at the original path start by translating the placement profile from the prepared start. The pipeline uses that source profile for the boundary solver.

Result: not accepted yet.

- line-arc and mixed exact/Level B chains still pass;
- open arc-arc seam gaps remain 2.500e-1 on both pieces;
- the source's profile centre is now at the original start, so translation alone is insufficient;
- the remaining mismatch is in the Torus generator's orientation/angular parameterisation, not in the Boundary Solver.

No solver, builder or tolerance change was made. high_level_open_arc_arc remains ignored.


## Indexed profile-vs-section audit (2026-11-06)

For both pieces and both U=0/U=1 sections, the source profile points were compared with Surface::evaluate(U, V) for direct order, reverse order and every cyclic shift.

`	ext
piece 0 U=0: best_error=2.29e-16 shift=0 reverse=false
piece 0 U=1: best_error=2.31e-16 shift=0 reverse=false
piece 1 U=0: best_error=6.21e-17 shift=0 reverse=false
piece 1 U=1: best_error=2.08e-16 shift=0 reverse=false
`

The source Torus sections now match the indexed profile exactly, with no shift and no reversal. The Torus generator and its V parameterisation are therefore not the remaining defect.

Direct seam-vertex-to-section distances show that only some seam vertices lie on the source sections:

`	ext
seam vertex 1: 3.29e-16
seam vertex 2: 3.31e-1
seam vertex 3: 3.29e-16
seam vertex 4: 2.50e-1
`

The remaining mismatch is in seam correspondence: the Boundary Solver seam loop includes points outside the exact source section / path coverage, so the next audit must compare seam point U/V and extension coverage, not change rc_sweep_surface.

