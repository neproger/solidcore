# 30 — Exact Sweep: Line and Arc + Circular Profile

## Goal

Prove the Stage B path end to end: an exact analytic side surface, split into
profile patches by the existing Direct B-rep Builder, produces a closed valid
solid. Level B is not used.

Implementation:

- `prototype/src/exact_geometry.rs`
- `prototype/src/direct_brep.rs` — `build_direct_single_surface` accepts `Surface`
- `prototype/tests/direct_exact.rs`

## Test A — line + circular profile

Exact side surface: `SurfOfExtrusion` of the circular profile along the line,
with the NURB U/V swapped so U = sweep and V = profile.

Result:

```text
faces=6 edges=12 vertices=8
manifold=true outward=true
check_brep Level2 valid=true failures=[]
volume=0.39269914006043183
expected=0.39269908169872414
relative=1.486e-7
```

The volume error is the mass integrator at its `1e-3` integration tolerance
(the same setting as the existing direct-shell tests), not a surface
approximation. No B-rep or surface tolerance was raised.

## Test B — arc + circular profile

Exact side surface: a partial `Torus` over the major-angle domain
`[start, end]` and the full minor angle `[0, 360]`, built inside-out so its
normal matches the Direct builder's side-face orientation.

Result:

```text
faces=6 edges=12 vertices=8
manifold=true outward=true
check_brep Level2 valid=true failures=[]
volume=0.6168519173226139
expected=0.6168502750680849
relative=2.662e-6
```

Again the residual is volume integration, not geometry.

## Consistency checks in the test

The exact tests assert:

- B-rep pointer/geometry validation at `Level2` with zero failures;
- manifold and outward orientation;
- expected face/edge/vertex counts;
- every edge curve endpoint agrees with its stored edge vertices within
  `1e-9` scale;
- closed-form volume within the integration tolerance.

The exact line and arc cases use the shared `build_direct_single_surface`
entry point; the old `build_direct_single` NURBS entry point delegates to it.
The existing line–line, line–arc and N=3 direct tests still pass.

## Bounds

- This is a single-piece exact sweep. The multi-piece chain still enters through
  the retained NURBS entry point.
- Open Arc–Arc and closed-path closure are untouched.
- The 17×17 Stage A grid was acceptance evidence, not a proof over the whole
  domain. These tests close that gap operationally for the two required cases:
  the resulting B-rep is built from the exact Surface and passes the kernel's
  full `check_brep` at `Level2`, including edge/face consistency.
