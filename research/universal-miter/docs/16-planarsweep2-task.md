# 16 — PlanarSweep2: task definition

Status: **main specification for PlanarSweep2.** The earlier documents are
inputs to this task, not separate projects. **Milestone 1 is PASS** for a
single straight/arc piece with a circular profile. **Milestone 2 is PASS for
`ExactMiter`** (line–line and an exact line–arc) and **PASS for the
`HybridAtJunction` boundary/topology** (line–arc with a missing exact side).
The hybrid volume difference is a diagnostic, not a correctness gate: it is a
different joint construction from an ordinary sweep. See
`17-milestone1-direct-shell.md`, `18-milestone2-line-line.md` and
`19-line-arc-volume-diagnostic.md`.

## 1. Goal in one sentence

Build a SolidCore Planar Sweep that analyses the flat path first, computes a
mathematically valid boundary between adjacent path segments, and constructs the
B-rep directly from those boundaries, without Boolean, `ProjectAndTrim` or
`StitchFaces` forming the joints.

PlanarSweep2 is **not** a miter algorithm and not a collection of per-primitive
algorithms. It is a universal boundary-construction pipeline over the
differential geometry of the path. A miter is one boundary construction among
several; the primitive names (Line, Arc, Ellipse, Bezier, B-spline) select
nothing in the algorithm.

Every supported planar segment is represented through one interface:

```
P(t), t in [a, b]
T(t), N(t), kappa(t), derivatives, valid parameter domain
```

The profile defines a family of offset trajectories `Q(t, s) = P(t) + s N(t)`.
The joint boundary is the common boundary of the corresponding offset
trajectories of two adjacent segments and their swept surfaces.

## 2. Method

```
flat path sketch
  -> curve normalization (common representation)
  -> differential geometry (position, tangent, normal, curvature, derivatives,
     valid parameter domain)
  -> universal boundary solver (offset trajectories of adjacent segments)
  -> boundary strategy selection (exact, hybrid, adaptive)
  -> boundary curves/surfaces for every segment
  -> swept surfaces bounded by those boundaries
  -> shared B-rep topology
  -> sweep solid
```

The joint geometry is computed before any 3D body exists. The swept surfaces
are built with their final boundaries; there is no intersection search and no
trim of finished solids.

Inputs/outputs are the Planar Sweep contract: one profile, a flat path of
ordered segments, `move_profile`, `cap_ends`, and a `Brep` result.

## 3. Foundations (not rewritten)

- **AOIM (#447)** — analytical joint geometry, the strategy set below, and the
  direct shared-topology B-rep reference (`research/aoim-line-arc`).
- **Universal Miter / Level A** — exact planar offset/rail geometry, folds,
  cusps, domain stops, signed offsets.
- **Gate 0** — path/profile normalization.
- **Level B Surface Builder** — NURBS swept surface with exact boundaries.
- **v4–v8** — B-rep, topology and validation knowledge. The v8 production
  mechanism (natural faces + trim/boolean) is a comparison/validation path, not
  the target.

## 4. Boundary strategies (AOIM-derived)

The joint boundary is chosen from a strategy set, by geometric analysis of the
offset trajectories and the profile, never from the primitive types. Exactness
may be analytical for one pair and numerical for another; the interface,
correctness predicates and pipeline stay the same. The initial set is the one
researched in AOIM:

| Strategy | Boundary construction | Role |
|---|---|---|
| `ExactMiter` | Exact common boundary of the corresponding offset trajectories; line–arc is the parabola | Normal exact case; refuses if any profile sample needs an extension |
| `HybridAtJunction` | Exact where the common boundary exists; on the missing side, continue on the tangent at the joint (the extended pieces' tangent line) | Valid hybrid boundary; one shared seam on both extended side surfaces |
| `HybridAtLimitTangent` | AOIM limit-tangent negative control: a tangent at the limit vertex is parallel to the line offsets below the limit, so it cannot form a common boundary | Typed refusal, not a fallback |
| `HybridAtLimitBisector` | Continue from the limit along the junction bisector | Alternative hybrid boundary (separate construction) |
| `AdaptiveCut` | Choose a cut/bridge within documented miter-length and path-deviation bounds; may locally adjust the path | Adaptive boundary for cases the analytic miter cannot cover |

Additional boundary constructions may be added behind the same interface, but
only with their own geometric proof and tests.

### Selection, not trial

The strategy is selected from geometry **before** building:

- existence and extent of the offset intersection on both adjacent segments;
- whether the chosen boundary covers the whole profile interval;
- common-boundary/no-gap/no-overlap predicates against both adjacent surfaces;
- closure and manifold predicates for the resulting shell;
- folds, curvature cusps, self-intersections and domain stops;
- documented miter-length/path-deviation bounds for adaptive cuts;
- topology compatibility (shared vertices/edges/loops).

The selection is deterministic and explainable from the input. It is not a
try-then-fallback chain: a strategy is not run speculatively and replaced when
it fails. If no strategy satisfies the geometric and topological predicates,
PlanarSweep2 returns a typed error. No silent approximation, no magic
tolerance, no special-case fallback.

`HybridAtJunction` is the valid hybrid construction: it was the successful
AOIM hybrid and is now implemented. `HybridAtLimitTangent` was a negative
control in the AOIM probe and remains a typed refusal: below its limit the
vertex tangent is parallel to the line offsets and cannot form a common
boundary. `HybridAtLimitBisector` and `AdaptiveCut` are separate constructions
with their own geometry, selection predicates and validation.

### Special joint cases

- **Tangent-continuous joint.** No miter boundary; the adjacent surfaces share
  the cross-section boundary directly.
- **Closed/periodic path.** No end caps; the last and first segments follow the
  same boundary rules; a tangent-continuous closure has no seam.
- **No valid boundary.** Typed error (for example `NoValidJointBoundary` with
  the failed predicates), not an approximation.

## 5. Requirements

1. Same input/output contract as `create_sweep_along_planar_path`.
2. Path analysis before 3D: segmentation, tangent continuity, closed/periodic,
   extension, plane frame, profile placement.
3. Boundary strategy analysis and selection as in section 4, including the
   hybrid and adaptive constructions.
4. Side surfaces constructed directly between the incoming and outgoing
   boundaries (Level B), not trimmed after a full-body intersection.
5. One B-rep assembled with shared vertices/edges/faces
   (`make_shell_vertex` + `make_wire_edge` + `make_face_topology`); caps from
   the outer profile loops.
6. No `BrepIntersect` / Boolean union / `ProjectAndTrim` / `StitchFaces` as the
   joint mechanism.
7. Supported path segments: at least Line, Arc/Circle, Ellipse, Bezier and the
   B-spline-reducible kinds the existing sweep accepts. Supported profiles:
   the arbitrary valid planar profiles of the input, not only rectangles.
8. Accuracy: the selected boundary must be a true common boundary of both
   adjacent surfaces within the kernel tolerance; no loosened comparison.
9. Typed refusals at documented limits; no silent approximation, no dead
   flags.

**Mathematical universality is not implementation proof.** The strategy set may
be mathematically applicable to a broad class, but each curve kind, profile
class, topology case and hybrid construction needs its own tests.

## 6. Validation

Geometry and topology are checked independently:

- the selected boundary is on **both** adjacent surfaces (not just one);
- no gap, no overlap, no uncovered profile interval;
- volume against `area × centreline length` or the analytic/refined reference;
- positive volume and outward orientation;
- closed manifold, 4 edge-uses per edge, void/solid region pair;
- `integrity::check`;
- shared edge/vertex ownership, no duplicate coincident joint edges;
- caps only at the outer ends (closed path: none);
- determinism and scale invariance;
- N=1, N=2, N=3 and a longer path; line–line, line–arc, arc–arc and mixed;
- circle/rectangle/ellipse/Bezier/B-spline profiles, open/closed profiles;
- each strategy: exact, tangent-limit, bisector-limit, adaptive cut;
- the typed-error cases.

The old p6 `TestClosureSplitRegion` split tail and `Shell.faces` cache findings
remain visible kernel gaps. A completing split is not a substitute for the
tail; do not claim volume/orientation for the direct path until measured.

## 7. Milestones

1. **Direct shell, N=1**: AOIM `make_shell_vertex`/`make_wire_edge`/
   `make_face_topology` with a Level B NURBS surface and circular profile;
   validate volume/topology against the v8 reference.
2. **Universal Boundary Construction Pipeline**: the shared differential
   interface, `ExactMiter` for any two planar segments, `HybridAtJunction` for
   the missing side, the direct two-piece shell and the validation. Line-line
   and an exact line-arc are the exact cases; the 90° line-arc exercises the
   junction continuation.
3. **Remaining hybrid and adaptive strategies**: add the separate
   bisector-limit and adaptive-cut constructions behind the same
   `BoundaryStrategy` interface, with their own geometry and validation; keep
   the limit-tangent negative control as a typed refusal. Then arc–arc,
   line–Bezier, Bezier–spline and spline–spline as test configurations of the
   same pipeline.
4. **N segments and closed path**: the open N-segment direct chain is PASS
   (3 pieces, 2 shared joints; `20-n-segment-direct-builder.md`). The arc-arc
   boundary matrix passes, but the arc-arc direct B-rep is blocked on the
   periodic profile V seam trim (`21-arc-arc-direct-blocked.md`); fix that
   before the N=3 arc chain and closed/periodic closure.
5. **Profile coverage and integration**: arbitrary planar profiles, open
   profiles, caps, scale, performance; then decide Option A (replace
   `create_sweep_along_planar_path`) or Option B (separate PlanarSweep2).

Production and public APIs stay unchanged during the research.

## 8. Out of scope

- General `create_curve_sweep` and spatial paths.
- Replacing Level A, Gate 0 or Level B.
- Boolean/trim/stitch as the joint construction.
- Changing production until the final, separately gated integration step.

## 9. Definition of done

A PlanarSweep2 that meets the requirements above on the validation matrix, with
a geometrically valid boundary shared by both adjacent surfaces, a valid B-rep,
the AOIM strategy set (exact, hybrid limit, adaptive) selected by geometry, and
typed refusals when none is valid. Then, and only then, choose replacement or a
separate production method.
