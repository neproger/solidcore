# 31 — Exact Multi-Surface Chain

## Goal

Extend the Exact Sweep from one side surface to a chain, while preserving the
old NURBS entry point:

- `build_direct_chain_surfaces(k, &[Surface], ...)`
- `build_direct_chain(k, &[BSplineSurface], ...)` remains as a wrapper.

The chain builder now accepts mixed analytic/NURBS surfaces. The only input
change is the surface source; the topology construction is unchanged.

## Test 1 — exact line–line–line

Three exact extrusion/cylinder surfaces, two ExactMiter boundaries.

```text
faces=14 edges=28 vertices=16
check_brep Level2: valid=true, 0 failures
volume=0.5890486341
expected=0.5890486225
relative=1.954e-8
```

## Test 2 — exact line–arc–line

One exact line surface, one exact partial torus, one exact line surface. Both
joints use the existing boundary solver; the torus uses the world profile from
the normalized arc placement and a relative major domain starting at 0.

Seam drop gaps:

```text
joint 0 left:  7.136e-8
joint 0 right: 7.136e-8
joint 1 left:  7.136e-8
joint 1 right: 7.136e-8
```

Result:

```text
faces=14 edges=28 vertices=16
check_brep Level2: valid=true, 0 failures
manifold=true outward=true
volume=3.430607280
expected=3.430614370
relative=2.067e-6
```

The expected volume is cross-section area times the path length; the residual
is the mass integrator at `1e-5` in this test, not a surface approximation. No
B-rep or surface tolerance was changed.

## Chain fix required by the exact torus

The exact torus can reach a joint with a rotated closed-profile parameterisation.
The old chain stored the canonicalized profile V for splitting; that made a
split fall on the V domain endpoint and left a patch missing. The chain now:

- keeps the actual profile V values for splits, caps and rails;
- canonicalizes only for the start/end V comparison;
- skips a split that lies on the V domain endpoint, since it is the closed
  profile seam rather than an interior split;
- uses the exact surface's full V domain for the resulting patch count.

## Regression checks

- `direct_joint` line–line, line–arc exact and line–arc hybrid still pass.
- The legacy `direct_chain_line_line_line` still passes.
- `build_direct_single` and `build_direct_chain` remain compatible wrappers.
- No Level B surface is used in either exact chain.

## Next

Wire exact source selection into the PlanarSweep2 path so line and arc pieces
choose the analytic source automatically, then keep the existing NURBS source
for unsupported profiles. Open Arc–Arc and closed-path closure remain separate.
