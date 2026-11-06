# 32 — PlanarSweep2 Source Selection

## What is selected

`source_selection::SweepSourceKind`:

- `ExactExtrusion` — linear path, exact extrusion/cylinder.
- `ExactRevolution` — circular arc path with a circular profile, exact torus/revolution.
- `LevelBApproximation` — anything else.

`classify_piece` uses geometric queries (`Curve::is_linear`, `Curve::is_arc`)
before any surface is built. It does not try an exact construction and fall back.
`select_sweep_sources` builds each selected source and returns its contract:

- `Surface`;
- the profile curve;
- actual path start/end and tangents;
- surface U/V domain;
- profile-closure flag;
- `SourceDiagnostics`.

An exact source that fails is returned as `SourceSelectionError::ExactBuildFailed`;
there is no silent Level B replacement.

## Tests

`prototype/tests/source_selection.rs`:

| Input | Selected sources |
|---|---|
| Line–Line–Line, circle | Exact / Exact / Exact |
| Line–Arc–Line, circle | Exact / Exact / Exact |
| Line–unsupported–Line, ellipse | Exact / Level B / Exact |
| Arc–Line–Arc, circle | Exact / Exact / Exact |

`source_selection_builds_mixed_exact_chain_inputs` also builds the selected
surfaces for Line–Line–Line, Line–Arc–Line and Arc–Line–Arc and checks their
actual U/V domains.

## Boundaries kept

- The Boundary Strategy Solver and its mathematics are unchanged.
- Open Arc–Arc, closed-path closure, and B-rep tolerance are unchanged.
- Level B is selected only for profiles/paths the classifier does not accept.

## Next

Wire `select_sweep_sources` into the high-level PlanarSweep2 chain builder,
then add a full B-rep case where a mixed Exact / Level B chain is assembled
through the same boundary solver.
