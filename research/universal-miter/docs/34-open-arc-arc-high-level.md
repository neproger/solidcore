# 34 — Open Arc–Arc through the High-Level Pipeline

## Input

The previous open Arc–Arc case, unchanged:

```text
path A: center (0, 1), radius 1, -180..-90 deg
path B: center (-1, 0), radius 1, 0..45 deg
profile: circle radius 0.25
segments: 4
```

Run through `build_planar_sweep`.

## Result

Source selection chooses:

```text
piece 0: ExactRevolution, SurfaceKind::Torus
piece 1: ExactRevolution, SurfaceKind::Torus
```

Boundary Solver:

```text
strategy = HybridAtJunction
exact = 36, junction = 32
max residual = 1.284e-13
```

The boundary solve itself succeeds. The chain build then fails before topology
assembly:

```text
PlanarSweepError::Chain(
  InvalidInput { reason: "the seam point is not on the side surface" }
)
```

Seam-to-side-surface drops:

```text
piece 0: U=[0,1] V=[0,1] gap = 2.500e-1
piece 1: U=[0,1] V=[0,1] gap = 4.591e-1
```

## Localisation

- Source Selection: PASS — both pieces are exact revolution toruses.
- Boundary Solver: PASS — HybridAtJunction, residual 1.3e-13.
- Direct topology: cannot start; the builder correctly refuses a seam point that
  is not on the selected side surface.
- Suspected area: the exact `Torus` surface / profile placement and its U/V
  parameterization relative to the solved seam. The gap is finite and large
  (about the profile radius for piece 0), so this is not a tolerance issue.

No Boundary Solver mathematics, Direct B-rep topology, or tolerance was changed.

## Test state

`high_level_open_arc_arc` is retained and ignored with this finding.
`open_arc_arc_source_boundary_audit` remains runnable and prints the same
source/boundary/gap diagnostics.

## Next

Audit the torus generator frame and seam UV mapping for this two-arc case. Fix the
exact source representation first; do not change the Boundary Solver or lower a
tolerance.
