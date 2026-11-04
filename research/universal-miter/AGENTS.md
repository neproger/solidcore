# AGENTS.md — research/universal-miter

This directory is an isolated research sandbox inside the SolidCore repository.
The root `AGENTS.md`, `CONTRIBUTING.md`, licence rules, determinism rules and
git rules still apply. The rules below add the isolation boundary.

## Boundary

- **Do not modify** `crates/solidcore/.../p2/planar_path.rs`, the public
  SolidCore API, or any production file for this research.
- Do not add this workspace to the root `Cargo.toml`. It has its own
  `Cargo.toml`; the root build must not see the prototype.
- The default prototype build must not depend on `solidcore`. The real kernel is
  available only through the `solidcore` feature, which is off by default and
  never enabled by the production build.
- Do not duplicate a kernel mechanism when a real SolidCore type or function can
  be used. Level A is independent by design (mathematics); level B uses the
  kernel.
- Integration into production is a separate, gated step
  (`docs/07-integration-plan.md`). Do not start it from this sandbox.

## Correctness

- A constructed miter curve is **not** evidence that it is correct. Every run
  must also check at least: the rail residual, both independent signed
  distances, the analytic reference where one exists, closure where a B-rep is
  built, and invariance under subdivision and reparameterisation.
- `d1 - d2 = 0` is valid only under the assumptions in
  `docs/03-mathematical-model.md` (WR-407 frame, planar path, same material
  `alpha`). It is **not** equivalent to the raw intersection of two sweep
  surfaces.
- Equal B-rep topology (face/edge counts) is not proof of geometric
  equivalence.
- A refusal at a documented limit (rail fold, curvature cusp, seam outside the
  piece) is a correct outcome. No fallback construction.

## Data and reports

- `results/reports/initial-validation.txt` is the immutable baseline. Do not
  edit or overwrite it.
- New runs go through `prototype-python/run_report.py` and carry a version, a
  label, the validator hash and the test parameters. Do not hand-edit a report.
- Keep fixtures reproducible: one file per case, fixed seeds, no clock in the
  algorithm.
- Do not commit build output (`bin/` is gitignored) or C++ reference output.

## Commands

```sh
cargo fmt   --manifest-path research/universal-miter/prototype/Cargo.toml --check
cargo build --manifest-path research/universal-miter/prototype/Cargo.toml
cargo test  --manifest-path research/universal-miter/prototype/Cargo.toml
cargo check --manifest-path research/universal-miter/prototype/Cargo.toml --features solidcore
python research/universal-miter/prototype-python/miter_validator.py
```

Use `CARGO_BUILD_JOBS=2` on the shared machine.
