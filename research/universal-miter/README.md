# Universal miter — research module

An isolated laboratory for the universal planar-sweep miter curve. The research
target is the kernel geometry, so it lives **inside SolidCore**, but it is not
part of the production build and it does not change the production Sweep.

Production code (`crates/solidcore/.../p2/planar_path.rs`) and the public API
are untouched. The production build is unrelated to this directory.

## Status

- **Stage 1–3 research:** the existing construction, the mathematical model and
  the independent validation are in `docs/` and `results/`.
- **This stage:** the isolated infrastructure only. The Rust prototype is a
  compiling skeleton; `harness::run_case` returns `NotImplemented` for both
  strategies. The algorithm is the next stage.
- The Python validator is the independent mathematical reference and already
  produces the baseline in `results/reports/initial-validation.txt`.

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
│   └── 07-integration-plan.md
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
│   │   └── solidcore_api.rs  only with `--features solidcore`
│   └── tests/smoke.rs
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
