# 15 — Audit of the AOIM (#447) research

Status: **audit complete.** The AOIM branch is the correct architectural
direction for the target Planar Sweep2, and its direct shared-topology B-rep
prototype is real, not speculative. It is not yet a general or certified
solution: the evidence is a manual probe for a rectangular/analytic class, and
volume, outward orientation and the `topology.brep_p6` closure tail were not
closed when the research was published.

Sources audited:

- issue #447, *Proposal: Analytical Curved Miter for Planar Sweep Joints
  (AOIM)*, and the 2026-10-22 comment by `neproger`;
- branch `research/aoim-line-arc` at `9920e32f` (preserved locally and on
  `neproger`), in particular
  `BREP_PLAN.md`, `BREP_REPORT_RU.md`, `src/sweep.rs`,
  `src/sweep3d.rs`, `src/adaptive.rs`, `src/continuing.rs`, and
  `probes/brep_probe/src/main.rs`;
- our current `research/universal-miter` documents and results.

No production file was changed. The AOIM branch was not deleted; it was simply
not carried into the current research branch.

## 1. What the AOIM research actually claims

The AOIM direction is:

```
profile -> segment 1 -> miter 1 -> segment 2 -> miter 2 -> ...
```

with the joint boundary computed from the corresponding offset trajectories
**before** any B-rep solid exists. The final shell is assembled from shared
vertices, edges and faces. The research explicitly avoids:

- `BrepIntersect`,
- Boolean union,
- `ProjectAndTrim`,
- `StitchFaces`,
- internal seam faces.

The published probe direct path is:

1. `topology::brep::p9::make_shell_vertex` for shared vertices.
2. `topology::brep::p3::make_wire_edge` for shared edges between existing
   vertices.
3. `topology::brep::p6::make_face_topology` for faces from ordered edge-use
   loops; an edge already used by a face gets the next edge-use pair.
4. `topology::edge::edgeuses`, `edgeuse::p1::vertexuse` and
   `vertexuse::vertex` to select the edge-use that starts at the required loop
   vertex.
5. Caps as planar faces from the same contour; line sides as planes, arc sides
   as cylinders, vertical extrusion.

The probe reports:

| Case | Contour | F / E / V |
|---|---:|---:|
| 90° baseline, profile `[0,2]` | 6 | 8 / 18 / 12 |
| 90° symmetric, `[-2,2]` | 7 | 9 / 21 / 14 |
| 30° symmetric | 7 | 9 / 21 / 14 |
| 150° symmetric | 7 | 9 / 21 / 14 |
| 179° symmetric | 7 | 9 / 21 / 14 |

Every edge has 4 edge-uses (2 `Same`, 2 `Opposite`); each face has one faceuse
in the void shell and one in the solid shell; the two shells are the expected
void/solid pair.

## 2. Claims versus evidence

| Claim | Evidence in the AOIM branch | Audit verdict |
|---|---|---|
| Exact analytical line–arc miter (parabola) | `src/sweep.rs` / `sweep3d.rs`; 68 tests in the 2D suite | **Supported for the 2D line/arc class.** Our Level A independently has `line_arc_is_the_analytic_parabola`. |
| No Boolean / trim / stitch in assembly | probe source uses only `make_shell_vertex`, `make_wire_edge`, `make_face_topology` | **Supported for the probe.** No production integration. |
| Shared vertices/edges/faces | probe counts and 4 edge-uses per edge | **Supported for the tested prisms.** |
| 30/90/150/179 and caps/sides-first orders | probe assertions | **Supported.** |
| One B-rep, one solid | 2 shells / 2 regions (void + solid) | **Supported as topology**, but see the p6 and `Shell.faces` caveats. |
| Volume | `brep_volume` was blocked by `SmSurface::DropCurve` | **Not certified at publication.** Expected `area * height`, but not measured. |
| Outward orientation | `orient_trimmed_surfaces` was blocked by `CalculateBoundingBox` | **Not certified at publication.** The probe checked orientation split counts, not outward normals. |
| General rectangular profiles/holes/reflex | not tested | **Not covered.** |
| Arbitrary profiles / NURBS surfaces | not tested | **Not covered.** The probe uses plane/cylinder patches. |
| Multiple path pieces / multiple joints | not tested | **Not covered.** The probe is a single line–arc prism. |
| Performance | not measured | **Not covered.** |
| Production readiness | no kernel changes; research branch only | **Not production.** |

The 68 passing tests are the 2D/adaptive/continuation research suite, not the
direct B-rep probe. The direct B-rep evidence is the manual probe and its
assertions; it is not part of CI.

## 3. Mathematical audit

The issue's line–arc derivation (offset line `y=-d`, offset circle with radius
`R+d`, eliminating `d`) gives the parabola

```
x^2 - 2 R sin(theta) x + 2 R (1 - cos(theta)) y = 0
```

and the branch through the path junction. Our Level A reproduces the same
line–arc parabola from the rail system and validates it against the analytic
reference. This is the strongest part of the AOIM research: the joint boundary
is genuinely curved and analytically known, so it should not be replaced by the
production bisector line.

The AOIM branch also contains non-AOIM strategies (`HybridAtJunction`,
`HybridAtLimitTangent`, `HybridAtLimitBisector`, adaptive cuts). These are
research alternatives/negative controls, not the target construction. The
target must select the exact AOIM branch or refuse at its documented limit; a
tangent/fallback contour would reintroduce approximation.

The probe's 3D solid is the **vertical extrusion of the 2D union region** for a
rectangular profile range and height. Its side patches are planes/cylinders and
its caps are planes. This is a strict subset of a general sweep: the AOIM
geometry and topology mechanism are validated, but the surface class is not the
circular/arbitrary profile NURBS class of our Level B.

## 4. Topology audit

The plan's expected counts for a closed contour of `N` line/arc segments are
`F = 2 + N`, `E = 3N`, `V = 2N`, with `V - E + F = 2` per shell and 4 edge-uses
per edge. The probe matches: 8/18/12 and 9/21/14. That is a valid
shared-topology prism, not a stitch result.

Caveats:

1. **`TestClosureSplitRegion` tail.** `make_face_topology` still returns
   `Unported("SmBrep::TestClosureSplitRegion", "topology.brep_p6")` after the
   split (`p6.rs:2396` in the current tree). The AOIM probe accepts this because
   the split already produced the two shells/regions and the void/solid flags;
   the omitted tail (`FindAndSetInfiniteRegion`, `IsInnerShell`, `RayFire`,
   empty-shell cleanup) is not exercised by the simple prism. This is a kernel
   port gap, not a geometry failure, but it must stay visible and must not be
   papered over for general cases.
2. **`Shell.faces` cache.** The new shell's `Shell.faces` list was empty while
   the faceuse list had all faces. The probe's queries use the faceuse list.
   This cache discrepancy must be checked before claiming general validity.
3. **`reset_infinite_region`.** The probe uses it as a no-op confirmation for
   the simple case; it is not a replacement for the p6 tail in nested/wire/
   vertex-only cases.
4. **Orientation.** The bottom cap loop must be reversed relative to the top
   cap; otherwise the faceuse split is 7/1 instead of 8/0. The probe checks
   orientation counts, not outward-normal certification.

## 5. Relation to our current research

| Our area | Relation to AOIM |
|---|---|
| Level A rail solver | Same exact miter for line–arc; more general (line/circle/ellipse/Bezier/B-spline, folds/cusps/domain stops). This is the **miter source** for the AOIM assembly. |
| Gate 0 preprocessing | Segment/profile preparation; compatible. |
| Level B Surface Builder | Produces the NURBS side surface. The AOIM probe used plane/cylinder patches for a rectangular profile; porting to NURBS is the new work. |
| v4 Boolean assembly | **Not the target.** |
| v5–v7 sequential B-rep | Useful topology experiments; not the final architecture. |
| v8 production topology audit | Useful requirements and validation tooling, but `stitch_and_orient` + trim/boolean is the production path, **not** the AOIM path. The exact volumes there (`3.926991e-1`, `6.168503e-1`, two/three line–line) are validation references. |

The current single-piece natural-face + caps + `stitch_and_orient` result is a
production-style side branch. It proves that the kernel can produce exact
volume/orientation when the production assembly is used, but it does not
implement the AOIM architecture.

## 6. Verdict

1. **AOIM is the right target.** The research demonstrates the exact curved
   miter and a direct shared-topology B-rep for the tested rectangular/line–arc
   class without Boolean, trim or stitch.
2. **It is a probe, not a general builder.** The evidence is limited to
   rectangular/analytic vertical extrusions, one line–arc joint, and topology
   assertions; volume/outward were not certified at publication.
3. **Our Level A/B reach further.** Level A covers general planar miter curves;
   Level B covers general NURBS side surfaces. The missing piece is the AOIM
   topology assembly adapted to the NURBS/closed-profile class.
4. **Do not continue the v8 production trim/boolean path as the target.** Use it
   only as a reference/validation path.
5. **Revalidate the AOIM direct path on the current kernel.** The p6 tail,
   `Shell.faces` cache, volume and outward checks must be retested with the
   current APIs; do not assume the old blockers are fixed or still present.

## 7. Next work

1. Port the AOIM direct builder (`make_shell_vertex` + `make_wire_edge` +
   `make_face_topology`) into the current research branch (new
   `direct_brep.rs` or into `brep_builder.rs`), preserving the AOIM branch as
   the reference.
2. Feed it our Level A miter loops (split into open edges, as the AOIM contour
   segments) and Level B NURBS side surfaces.
3. Build the caps as planar faces from the outer profile loops; keep the AOIM
   loop reversal and orientation rules.
4. Validate on the current kernel: `4` edge-uses per edge, `2` shells / `2`
   regions, volume `area * length`, outward orientation, `integrity::check`,
   and the p6 `Unported` split handling; record `Shell.faces` cache state.
5. Generalize the counts and loops to N path pieces and a closed circular
   profile; test N=1,2,3, line–arc, mixed; preserve the AOIM 90° prism as a
   regression.
6. Keep the exact AOIM branch selection; do not use the hybrid/tangent
   strategies as a fallback. Refuse at documented limits.

## 8. Addendum: task refinement (strategy scope)

The task definition in `16-planarsweep2-task.md` broadens the strategy scope.
The audit above records what the AOIM branch did at publication:

- `HybridAtLimitTangent`, `HybridAtLimitBisector` and the adaptive cut were
  research alternatives/negative controls for evaluating the exact AOIM branch;
- the direct B-rep probe used the exact/hybrid-at-junction contour for the
  tested line-arc prisms.

That historical status does not exclude them from PlanarSweep2. PlanarSweep2 is
a **boundary-construction** algorithm, not a miter-only algorithm: the exact
miter, the tangent/bisector limit continuations and the adaptive cut are all
candidate boundary strategies, chosen deterministically from the geometry (offset
existence and extent, profile coverage, common-boundary/no-gap/no-overlap,
closure, folds/cusps, miter-length/path-deviation bounds, topology). They are
never used as a silent try-then-fallback chain. If no strategy satisfies the
geometric and topological predicates, the operation returns a typed error.

The audit's evidence still stands: each strategy, profile class and joint type
needs independent tests; mathematical applicability is not implementation proof.
