# PlanarSweep2 research

An isolated laboratory for **PlanarSweep2**: a Planar Sweep that analyses the
flat path first, constructs a mathematically valid boundary between adjacent
segments, and builds the B-rep directly from those boundaries. No Boolean,
`ProjectAndTrim` or `StitchFaces` forms the joints.

The main specification is
[`docs/16-planarsweep2-task.md`](docs/16-planarsweep2-task.md). The AOIM
research audit is
[`docs/15-aoim-447-audit.md`](docs/15-aoim-447-audit.md). **Milestone 1 is
PASS**: one direct shell for a straight/arc piece with a circular profile, one
solid region, exact volume against the analytic and production references; see
[`docs/17-milestone1-direct-shell.md`](docs/17-milestone1-direct-shell.md).
**Milestone 2 is PASS for `ExactMiter`** (line–line and exact line–arc) and
**PASS for the `HybridAtJunction` boundary/topology**. Its volume is a
diagnostic, not a gate: it belongs to a different joint construction
(`docs/19`). Both go through the same differential/offset pipeline; see
[`docs/18-milestone2-line-line.md`](docs/18-milestone2-line-line.md).
**N-segment direct builder is PASS** for an open 3-piece chain; see
[`docs/20-n-segment-direct-builder.md`](docs/20-n-segment-direct-builder.md).
The arc–arc boundary matrix passes and the 3D geometry audit passes
(`docs/22`), but the arc–arc direct B-rep is **blocked** on the periodic profile
V seam trim; see
[`docs/21-arc-arc-direct-blocked.md`](docs/21-arc-arc-direct-blocked.md).

Production code (`crates/solidcore/.../p2/planar_path.rs`) and the public API
are untouched. This directory is not a member of the root workspace.

## Foundations kept

- **Level A** (`prototype/src/curve.rs`, `differential_geometry.rs`,
  `rail_solver.rs`, `miter_curve.rs`, `validation.rs`, `harness.rs`): the planar
  rail/differential mathematics.
- **Gate 0 input preprocessing** (`prototype/src/planar_sweep2.rs`, feature
  `solidcore`): path/profile normalization and unification. It reproduces the
  production preprocessing where those helpers are private. The production
  Planar Sweep is used only as the parity reference for this preparation, not
  for the new joint construction.
- **Level B surface builder** (`prototype/src/level_b_geometry.rs`, feature
  `solidcore`): controlled NURBS swept surfaces and their accuracy checks.
- **Python reference** (`prototype-python/miter_validator.py` and
  `results/reports/initial-validation.txt`): the original independent
  mathematical reference and its immutable baseline. Preserved; not used as a
  source of C++/kernel state.

The AOIM research branch is a separate reference:
`research/aoim-line-arc` (`BREP_PLAN.md`, `BREP_REPORT_RU.md`,
`probes/brep_probe/`). It is the direct shared-topology B-rep prototype
(`make_shell_vertex` + `make_wire_edge` + `make_face_topology`), not part of
this workspace.

## Layout

```
research/universal-miter/
├── README.md
├── AGENTS.md
├── Cargo.toml                 separate workspace
├── docs/
│   ├── 15-aoim-447-audit.md
│   ├── 16-planarsweep2-task.md
│   ├── 17-milestone1-direct-shell.md
│   ├── 18-milestone2-line-line.md
│   ├── 19-line-arc-volume-diagnostic.md
│   ├── 20-n-segment-direct-builder.md
│   ├── 21-arc-arc-direct-blocked.md
│   ├── 22-arc-arc-geometric-consistency-audit.md
│   ├── 23-arc-arc-uv-trim-audit.md
│   └── 24-direct-brep-architecture-audit.md
├── prototype/
│   ├── Cargo.toml
│   ├── src/
│   │   ├── lib.rs
│   │   ├── curve.rs
│   │   ├── differential_geometry.rs
│   │   ├── rail_solver.rs
│   │   ├── miter_curve.rs
│   │   ├── validation.rs
│   │   ├── harness.rs
│   │   ├── planar_sweep2.rs       feature solidcore (Gate 0 preprocessing)
│   │   ├── level_b_geometry.rs    feature solidcore (NURBS surface builder)
│   │   ├── boundary.rs            feature solidcore (boundary strategies)
│   │   └── direct_brep.rs         feature solidcore (direct shell/joint/chain)
│   └── tests/
│       ├── smoke.rs
│       ├── matrix.rs
│       ├── gate0.rs               feature solidcore
│       ├── direct_shell.rs        feature solidcore
│       └── direct_joint.rs        feature solidcore
├── prototype-python/
│   ├── miter_validator.py
│   └── run_report.py
└── results/
    ├── reports/initial-validation.txt        immutable baseline
    ├── reports/v9-aoim-447-audit.txt
    ├── reports/v10-milestone1-direct-shell.txt
    ├── reports/v11-milestone2-line-line.txt
    ├── reports/v12-universal-boundary-line-arc.txt
    ├── reports/v13-universal-boundary-hybrid.txt
    ├── reports/v14-hybrid-volume-diagnostic.txt
    ├── reports/v15-n-segment-direct-builder.txt
    ├── reports/v16-arc-arc-matrix-blocked.txt
    ├── reports/v17-arc-arc-geometric-consistency.txt
    ├── reports/v18-arc-arc-uv-trim-audit.txt
    └── reports/v19-direct-brep-architecture-audit.txt
```

## Milestones

From `docs/16-planarsweep2-task.md`:

1. Direct shell, N=1 (AOIM shared topology with Level B NURBS).
2. Universal Boundary Construction Pipeline: shared differential interface,
   `ExactMiter` for any two planar segments, `HybridAtJunction` for the missing
   side, direct two-piece shell. Line–line and exact line–arc are the exact
   cases; the 90° line–arc exercises the junction continuation.
3. Remaining strategies: separate bisector-limit and adaptive-cut
   constructions, plus the limit-tangent negative control; then arc-arc and
   spline-spline configurations.
4. N segments and closed path: the open N-chain is implemented; closed/periodic
   closure is next.
5. Profile coverage and integration (replace or separate production method).

## Commands

```sh
export CARGO_BUILD_JOBS=2

cargo fmt   --manifest-path research/universal-miter/prototype/Cargo.toml --check
cargo build --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test  --manifest-path research/universal-miter/prototype/Cargo.toml
cargo check --manifest-path research/universal-miter/prototype/Cargo.toml --features solidcore
cargo test  --manifest-path research/universal-miter/prototype/Cargo.toml --features solidcore
```

`RUSTC_WRAPPER=sccache` may be set when sccache is installed; the repository
does not require it.

## Boundary

- Do not modify production files, the public API or the root workspace.
- Do not add this workspace to the root `Cargo.toml`.
- The mathematical strategy set and typed refusals are in
  `docs/16-planarsweep2-task.md`.
- Integration into production is a separate, gated step.
