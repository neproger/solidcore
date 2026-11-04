# Universal miter — research module

An isolated laboratory for the Planar Sweep miter and the alternative
`create_sweep_along_planar_path2`. The research target is the kernel geometry,
so it lives **inside SolidCore**, but it is not part of the production build and
it does not change the production Planar Sweep.

Production code (`crates/solidcore/.../p2/planar_path.rs`) and the public API
are untouched. The production build is unrelated to this directory.

## Status

- **Stage 1–3 research:** the existing construction, the mathematical model and
  the independent validation are in `docs/` and `results/`.
- **Level A implemented; Gate 1 PASS; audit PASS.** The Rust rail solver
  implements the differential geometry, the G1 offset rails, the damped
  fixed-alpha Newton, the two-direction continuation, the fold/cusp/domain
  refusals and the independent signed-distance validation. 22 tests pass; the
  Gate 1 evidence is in `docs/06-experiment-results.md`,
  `results/reports/v1-gate1-rust.txt` and
  `results/reports/v1a-gate1-audit-rust.txt`; the audit is
  `docs/08-level-a-audit.md`.
- **Gate 0 preprocessing parity PASS.** The normalizer in
  `prototype/src/planar_sweep2.rs` reproduces the production Planar Sweep
  preparation; 9 tests pass. The production cap matches the whole-path
  normalizer; the single-segment WR-660 wrapper differs; rational weights,
  planarity, caps and extension are covered. See
  `docs/11-gate0-preprocessing.md` and
  `results/reports/v0-gate0-preprocessing.txt`. The private
  `prepare_curve_sweep` is reproduced as `PreparedPiece`.
- **Planar Sweep2 surfaces are not started.** After Gate 0 approval the
  alternative builds its own swept surfaces, joints and B-rep.
  `Original` is the existing `create_sweep_along_planar_path`. The general
  `create_curve_sweep` and spatial paths are out of scope. The exact contract,
  architecture and open decisions are `docs/09-sweep2-scope.md`; the input
  normalization audit is `docs/10-preprocessing-audit.md`; approval is pending.
- The `MiterStrategy::Original` of the Level A joint harness deliberately remains
  `NotImplemented`; the operation-level `Original` is the existing Planar Sweep
  in the Planar Sweep2 comparison.
- The Python validator remains the independent mathematical reference and its
  baseline is `results/reports/initial-validation.txt`.

## Layout

```
research/universal-miter/
├── README.md                 this file
├── AGENTS.md                 rules for this sandbox
├── Cargo.toml                separate workspace (not a member of the root one)
├── docs/
│   ├── 01-problem-definition.md
│   ├── 02-existing-sweep-analysis.md
│   ├── 03-mathematical-model.md
│   ├── 04-solver-design.md
│   ├── 05-test-matrix.md
│   ├── 06-experiment-results.md
│   ├── 07-integration-plan.md
│   ├── 08-level-a-audit.md
│   ├── 09-sweep2-scope.md
│   ├── 10-preprocessing-audit.md
│   └── 11-gate0-preprocessing.md
├── prototype/                Rust level A (math) + level B (feature `solidcore`)
│   ├── Cargo.toml
│   ├── src/
│   │   ├── lib.rs
│   │   ├── curve.rs
│   │   ├── differential_geometry.rs
│   │   ├── rail_solver.rs
│   │   ├── miter_curve.rs
│   │   ├── validation.rs
│   │   ├── harness.rs
│   │   ├── planar_sweep2.rs  only with `--features solidcore`
│   │   └── solidcore_api.rs  only with `--features solidcore`
│   └── tests/
│       ├── smoke.rs
│       ├── matrix.rs
│       └── gate0.rs          only with `--features solidcore`
├── prototype-python/
│   ├── miter_validator.py    independent mathematical validator
│   └── run_report.py         versioned report wrapper (does not edit the validator)
├── fixtures/                 reproducible inputs (line, arc, bezier, bspline)
├── experiments/              baseline, invariance, singularities, profile variations
└── results/
    ├── reports/              versioned reports; initial-validation.txt is immutable
    ├── numerical/            raw numeric data
    └── visualizations/
```

## Commands

```sh
# Independent mathematical validator (level 0 reference)
python research/universal-miter/prototype-python/miter_validator.py

# Versioned report; never overwrites the baseline
python research/universal-miter/prototype-python/run_report.py \
  --version v0 --label matrix --param case=line-arc

# Rust level A (no kernel dependency)
cargo build --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test  --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test  --manifest-path research/universal-miter/prototype/Cargo.toml \
  --test matrix gate1_report -- --nocapture

# Rust level B (uses the real kernel; never part of the production build)
cargo check --manifest-path research/universal-miter/prototype/Cargo.toml --features solidcore
```

On the shared machine keep `CARGO_BUILD_JOBS=2`; the repository's
`.cargo/config.toml` sends artifacts to the root `bin/` (gitignored).

## Isolation guarantees

- `research/universal-miter/Cargo.toml` is its own workspace. The root workspace
  lists members explicitly and does not include it.
- The default prototype build has **no** dependency on `solidcore`. The real
  kernel is reachable only with the off-by-default `solidcore` feature.
- The production `planar_path.rs` is not modified. Integration is a later,
  separately gated step (`docs/07-integration-plan.md`).

## Comparing strategies

`prototype/src/harness.rs` defines the common interface:

```rust
pub enum MiterStrategy { Original, Universal }

pub struct MiterTestCase {
    pub id: String,
    pub curve_a: CurveSpec,
    pub curve_b: CurveSpec,
    pub profile: ProfileSpec,
    pub expected: Option<ExpectedGeometry>,
}

pub struct MiterResult {
    pub strategy: MiterStrategy,
    pub status: MiterStatus,
    pub curve: Option<MiterCurve>,
    pub diagnostics: MiterDiagnostics,
}
```

One case runs through both strategies. Geometry, accuracy, B-rep topology,
manifold, volume, numerical stability and time are compared **separately**.
Equal topology is not accepted as proof of geometric equivalence.
