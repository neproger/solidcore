# 07 — Planar Sweep2 plan (gated)

**Scope (refined 2026-11-04).** The object is the specialised **Planar Sweep**,
not the general `create_curve_sweep`:

- `Original` = the existing `create_sweep_along_planar_path()`;
- `Alternative` = `create_sweep_along_planar_path2()`.

Spatial paths are out of scope. The general `create_curve_sweep` is out of scope
and is not the `Original`. Level A (`universal-miter`) is the **joint component**
of the alternative, not a Sweep replacement. The exact contract, architecture
and open decisions are in `09-sweep2-scope.md`.

Gate 1 is executed. Gate 0 (preprocessing parity) is executed and green; Gate 2
is the approval gate; no swept surface, joint or B-rep starts before it. The
production `planar_path.rs` and the public API stay unchanged until the final
gate.

## Reference — mathematical model (done)

The Python validator and the model agree within the reach; the fold and the
extension policy are documented. See `06-experiment-results.md`.

## Gate 1 — Level A rail solver (done, PASS)

The level A solver is implemented in `prototype/src` (no kernel dependency).
The results are in `06-experiment-results.md`,
`results/reports/v1-gate1-rust.txt` and
`results/reports/v1a-gate1-audit-rust.txt`: 22 tests pass, the maximum rail and
signed-distance residual is `4.95e-14`, the folds are at `0.500000` (line–arc)
and `0.292893` (arc–arc), and the negative branch continues independently. The
audit is `08-level-a-audit.md`.

## Gate 0 — preprocessing parity (done, PASS)

`prototype/src/planar_sweep2.rs` normalizes the input exactly as the production
Planar Sweep prepares it and compares the result with the production cap face
and the source-faithful steps. Nine tests pass; the profile placement, plane
normal, segmentation, extension, closed/periodic, caps, rational weights,
tolerances and the whole-path WR-660 placement are covered. The production cap
matches the whole-path normalizer; the single-segment wrapper differs by 0.4279.
The private `prepare_curve_sweep` is reproduced as `PreparedPiece`. See
`11-gate0-preprocessing.md` and `results/reports/v0-gate0-preprocessing.txt`.

## Gate 2 — Planar Sweep2 contract and architecture (approval required)

`09-sweep2-scope.md` fixes:

- the exact public signature and the input contract of
  `create_sweep_along_planar_path`;
- the closed/periodic, extension, profile-placement, cap and flag behaviour;
- the output/ownership/error contract;
- the contract ambiguities (non-B-spline profiles, non-planar paths, straight
  path, cap behaviour, the stale `Unported` note);
- the architecture of the alternative and the public low-level API audit.

No implementation starts until the decisions in §9 of that document are
confirmed.

## Gate 3 — adapter and one-joint surfaces

- Add the `PlanarCurve` adapter trait; implement it for `CurveSpec` and for the
  real `solidcore::curve::curve::Curve` via `Curve::evaluate(k, t, 2, ..)`.
  Project to the path plane; do not duplicate the kernel's B-spline/de Boor.
- Keep the Level A solver and its Gate 1 tests unchanged.
- Prepare the path/profile with the confirmed contract (G1 extension, placement,
  flags).
- Build the two swept surfaces with Sweep2's own construction (analytic sampling
  and NURBS fitting; no `create_sweep_surface`/`N_CreateSweepScale`).
- Verify the Level A seam lies on both swept surfaces before any trim.

## Gate 4 — one joint and B-rep

- Trim with `project_and_trim` (kernel tool) using the analytical seam and
  `direction = N`, or build the trimmed faces directly.
- Add the caps with the confirmed cap contract.
- Build the B-rep with `PrimitiveModel::build_new` / `BrepConstructor` /
  `create_faces_with_curves`; stitch/union.
- Check closed, manifold, orientation, no self-intersection, seam on both
  surfaces, volume (analytic or refined reference), determinism.
- Compare with `Original` on the same inputs and Kernel flags.

## Gate 5 — full Planar Sweep

- several consecutive joints, including two mitres in a row;
- a closed path and the closing join;
- open/closed profiles; `cap_ends` 0..3; `move_profile` true/false;
- scales, convex/reflex turns, folds and refusals;
- the confirmed flag set, including WR-407 on;
- `Original` vs `Alternative`: geometry, closure, manifold, volume, counts,
  time, stability;
- reproducible fixtures and regression tests.

## Gate 6 — production integration (separate PR, future)

Only after the gates above and the three production criteria:

1. the mathematical seam matches the independent reference;
2. the real B-rep is closed, manifold and geometrically correct;
3. the alternative is stable over several consecutive joints and a closed path.

Then:

- put the result behind a `Kernel::fixes` flag (per `AGENTS.md` §4b), never a
  silent replacement;
- add a regression test and a `plan/upstream-issues/` addendum;
- keep the old path until the new construction is in and parity is shown, then
  delete the old path and the solidcore-only `WR-` patches;
- difftest: the reference for a redesigned construction is the SolidCraft/fork
  result, not the old plane; never loosen the `1e-9` tolerance.

No production integration in this research.

## Risks

1. **B-rep topology.** The main risk is not the solver but the conversion of the
   analytical seam into a valid trimmed topology. Closure/manifold/orientation
   and a real volume check are mandatory.
2. **Extension policy.** G1 straight extension is fixed for v1; G2/natural
   continuation is out of scope.
3. **Fold predicate.** Testing only `1 - alpha kappa` misses the fold; test
   `cross(P_1', P_2')` and the piece domain.
4. **Representation.** The seam must be independent of the B-spline
   representation and knot insertion.
5. **Performance.** Continuation + trim + fitting may cost more than the
   two-plane line; measure separately from correctness.
6. **Contract ambiguities.** Non-B-spline profiles, non-planar paths, straight
   paths and cap behaviour must be fixed before the first slice.

## Isolation checklist

- The root `Cargo.toml` is not modified; `research/universal-miter/Cargo.toml`
  is its own workspace.
- The default prototype build has no `solidcore` dependency.
- The production `planar_path.rs` and the public API are untouched.
- The Python validator and `results/reports/initial-validation.txt` are
  unchanged.
