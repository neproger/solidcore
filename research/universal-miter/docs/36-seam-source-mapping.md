# 36 — Seam-to-Source Mapping Audit

## Scope

Audit only. No Exact Torus, Boundary Solver, Direct B-rep or tolerance changes.

For every seam vertex of the open Arc–Arc boundary, project it onto both source
Torus surfaces and print:

- drop success and gap;
- projected U/V;
- whether U/V are inside the source domains;
- distance to original/prepared path endpoints;
- distance to the nearest source start/end section.

## Result

Boundary Solver remains successful:

```text
HybridAtJunction
exact=36 junction=32
residual=1.284e-13
```

Seam vertices:

```text
vertex 0: piece 0 U=1.000 V=0.000 gap=3.29e-16
          piece 1 U=0.000 V=0.000 gap=6.50e-17

vertex 1: piece 0 U=0.707 V=0.250 gap=2.22e-16
          piece 1 U=0.566 V=0.250 gap=2.00e-16

vertex 2: piece 0 U=1.000 V=0.500 gap=3.29e-16
          piece 1 U=0.000 V=0.500 gap=6.50e-17

vertex 3: piece 0 U=1.000 V=0.750 gap=2.50e-1
          piece 1 U=0.000 V=0.750 gap=2.50e-1
```

All U/V values are inside `[0,1]`. The problematic vertex is therefore not
outside the source U-domain and is not an original/extension U-coverage issue.

## Interpretation

- Vertices 0 and 2 are on both source surfaces.
- Vertex 1 is an interior-U seam point and is also on both surfaces.
- Vertex 3 is the only mismatch: it projects to `U=1/0, V=0.75` on both sides,
  but has a `0.25` gap on both sides.

This is a **profile/seam correspondence** problem at the V=0.75 seam point, not a
Torus section or V-direction problem. The indexed profile-vs-section audit already
showed the Torus sections match the source profile exactly with no shift or
reverse.

## Next

Trace how the Boundary Solver produces the V=0.75 seam vertex and compare that
point with the source profile point at the same index/alpha. Do not change the
Torus generator, Boundary Solver residual, or Direct builder yet.

## Test state

`high_level_open_arc_arc` remains ignored.

## V=0.75 point trace

`	ext
piece 0 profile V=.75 at original start = (-1.25, 1.0, 0)
piece 0 section U=1 V=.75             = (0, -0.25, 0)
piece 0 seam vertex                 = (0.25, -0.25, 0)

piece 1 profile V=.75 at original start = (0.25, 0, 0)
piece 1 section U=0 V=.75             = (0.25, 0, 0)
piece 1 seam vertex                 = (0.25, -0.25, 0)
`

The V=0.75 seam vertex is not the source section point at V=.75. On piece 0 it is displaced by 0.25 from the source section; on piece 1 the source section point is (0.25, 0) while the seam vertex is (0.25, -0.25). This is a profile/seam mapping mismatch, not a Torus parameterization failure.

