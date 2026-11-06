# 33 — High-Level PlanarSweep2 B-rep

## Pipeline

`planar_pipeline::build_planar_sweep` now runs one complete path:

```text
input profiles + paths
    -> normalize_planar_sweep
    -> select_sweep_sources
    -> PathFrame + BoundarySolver
    -> build_direct_chain_surfaces
    -> PlanarSweepResult { brep, sources, boundaries }
```

`PlanarSweepResult` retains every piece's source kind, surface, domains and
diagnostics, plus every joint's boundary and diagnostics. The stages remain
independent:

- source selection never falls back to Level B;
- boundary solving never retries another strategy;
- topology construction receives the selected surfaces and solved boundaries.

## Tests

`prototype/tests/planar_pipeline.rs`:

| Case | Selected sources | Result |
|---|---|---|
| Line–Line–Line, circle | Exact / Exact / Exact | PASS |
| Line–Arc–Line, circle | Exact / Exact / Exact | PASS |
| Line–Arc–Line, ellipse | Exact / Level B / Exact | PASS |

All three cases pass `check_brep` at `Level2`, are manifold and outward, and have
the expected 14 faces / 28 edges / 16 vertices with four profile segments.

The mixed case also checks the actual built surfaces:

- line pieces: `SurfaceKind::SurfOfExtrusion`;
- middle Level B piece: `SurfaceKind::BSpline`;
- middle diagnostics: `path_kind = CircleArc`, `circular_profile = false`.

## Boundaries preserved

- Boundary Solver and its ExactMiter/HybridAtJunction mathematics are untouched.
- Open Arc–Arc and closed-path closure are untouched.
- No B-rep or surface tolerance was changed.
- A source failure is returned as `PlanarSweepError::Source`; it is never
  converted into a Level B result.
- A boundary failure is returned as `PlanarSweepError::Boundary`; it is never
  retried with another strategy.

## Next

The high-level path is now proven for supported and mixed source cases. Open
Arc–Arc and closed-path closure remain separate next milestones.
