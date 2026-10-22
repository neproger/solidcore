# Direct B-rep integration plan

Status: design/audit only. No kernel changes are made by this plan.

The current 3D code is a faceted prism over the 2D line/arc union: it builds two
bodies, shares a vertical seam surface, then deletes the seam faces. The direct
B-rep construction below produces the final outer shell without a Boolean and
without ever creating the internal seam surface.

## Final geometry

For a profile offset range `d in [d0, d1]` and height range `z in [z0, z1]`, the
solid is the vertical extrusion of the 2D union region:

```
position(d, z) = (P(d).x, P(d).y, z)
```

where `P(d)` is the line/arc sweep trajectory in the path plane. The final shell
has:

- two cap faces at `z = z0` and `z = z1`, each one planar face bounded by the
  outer contour of the union region;
- one side face per outer-contour segment, extruded vertically;
- no internal seam face, no two-body intermediate, no Boolean.

The internal seam is used only to compute the union region and the outer
contour. It is not represented in the B-rep.

For the ordinary case (`d >= 0`, no tangent prolongation) the outer contour has
six segments:

```
line  (-L, -d1) -> seam_end
arc   seam_end  -> outer_end
line  outer_end -> inner_end
arc   inner_end -> seam_start
line  seam_start-> (-L, -d0)
line  (-L, -d0) -> (-L, -d1)       // start cap
```

For `d < 0` under `HybridAtJunction`, `arc_boundary` prefixes a straight
tangent segment; the contour builder must therefore be a general
`Vec<ContourSegment>` of line/arc segments, not a hard-coded six-edge loop.
Each arc segment maps to a vertical cylindrical patch; each line segment maps
to a vertical planar patch.

Expected topology for a closed contour of `N` line/arc segments: `2 + N`
faces, `3N` edges, `2N` vertices, one void shell and one solid shell,
`V - E + F = 2` for each closed shell. The six-segment baseline is
8/18/12; the symmetric `[-2, 2]` contour has seven segments and is 9/21/14.
Volume is exactly `union_area * (z1 - z0)`.

## Direct shared-topology construction (validated)

The supported public path is:

1. `topology::brep::p9::make_shell_vertex` creates a vertex.
2. `topology::brep::p3::make_wire_edge` creates an edge between two existing
   vertices with a 3D curve and interval.
3. `topology::brep::p6::make_face_topology` creates a face from ordered
   `EdgeuseId` loops. For a wire edge it consumes the existing edge-use pair;
   for an edge already used by a face it creates the next edge-use pair on the
   same edge. This is the direct-sharing primitive.
4. `topology::edge::edgeuses`, `topology::edgeuse::p1::vertexuse` and
   `topology::vertexuse::vertex` select the edge use that starts at the next
   loop vertex.

This path was validated with public APIs only:

- Two adjacent squares: 2 faces, 7 edges, 6 vertices (shared edge).
- A box: 6 faces, 12 edges, 8 vertices; every edge has 4 edge uses; one shell
  and one region (a useful contrast: both faceuses of each face are still in
  one shell, so it is not a split solid).
- A cylindrical face from two vertical lines and two circle arcs: 1 face,
  4 edges, 4 vertices.
- The 90-degree AOIM prism completes as a two-region solid: 8 faces,
  18 edges, 12 vertices, 4 edge uses per edge, one void region and one solid
  region, with each face present once in each shell.

No `StitchFaces`, `GlueVertices`, Boolean, `project_and_trim` or seam surface
is used. The split already leaves the correct flags; `reset_infinite_region` is
only a before/after no-op consistency check for this simple case.

## Face construction

### Caps

Build the contour as wire edges at `z0` and `z1`, then call
`make_face_topology` with the ordered edge uses. Use a plane surface:

- bottom: outward normal `(0, 0, -1)`;
- top: outward normal `(0, 0, 1)`.

The top cap loop follows the outer contour. The bottom cap loop must be
reversed because its outward normal is `-Z`; using the same order flips one
faceuse relative to the other seven (observed as 7/1 instead of 8/0).

### Line side faces

Each vertical extrusion of a contour line is a planar rectangle. Build its four
wire edges, use a plane through the segment with the outward side normal, and
call `make_face_topology` with the loop

```
bottom start -> bottom end -> top end -> top start -> bottom start
```

### Arc side faces

Each vertical extrusion of a contour arc is a cylindrical rectangular patch.
Build the bottom arc, top arc and two vertical line edges as wire edges. Create
the cylinder with `surface::cone::p1::Cone::new_cylinder_bounded`:

- origin at the arc center with `z = z0`;
- local `x_axis` at the arc start, `y_axis` along the arc tangent;
- constructor angles in **degrees**, start `0`, end `span_degrees`;
- height `z1 - z0`;
- then call `make_nurb(&k)` (otherwise the natural UV domain is empty).

Call `make_face_topology` with the cylinder surface and the loop

```
bottom arc -> vertical at end -> top arc reversed -> vertical at start
```

The edges lie analytically on the cylinder, so no projection or UV trimming is
needed for topology construction.

### Orientation

Choose each surface normal outward and use the matching `OrientType`. The cap
loops must agree with their surface normals; reversing the bottom loop is what
turns the orientation split from 7/1 into 8/0. `orient_trimmed_surfaces` would
be the final outward-normal check, but it currently stops at
`CalculateBoundingBox` (`query.brep`), so it is not part of the passing probe.

## Assembly

1. Build the 2D union and an analytic `Vec<ContourSegment>` with exact shared
   endpoints.
2. Create the `2N` cap vertices with `make_shell_vertex`.
3. Create the `2N` cap edges and `N` vertical edges with `make_wire_edge`.
4. Create the bottom cap, top cap and `N` side faces with
   `make_face_topology`.
5. For every loop edge, select the edge use starting at the required vertex,
   so the loop direction is explicit.
6. Verify:
   - faces `2 + N`, edges `3N`, vertices `2N`;
   - every edge has 4 edge uses;
   - one shell and one region;
   - no internal seam face;
   - volume `area * (z1 - z0)`;
   - outward normals;
   - the same result for `30/90/150/179` degrees and for negative-profile
     portions.
7. Verify edge-use orientation (`2 Same + 2 Opposite` per edge) and the
   faceuse split (`one faceuse per shell`). Outward-normal unification is
   currently blocked at `CalculateBoundingBox`; do not claim it.

No `stitch_and_orient`, no `glue_*`, no Boolean and no `SurfOfExtrusion` is
required.

## Why this supersedes the stitch path

`topology::brep::p5::stitch_and_orient` still stops at
`SmBrep::StitchFaces` (`topology.brep_p4`). Calling the already-ported
`heal::stitch::p2::Stitch::do_stitching` directly gets past that wrapper but
stops at `SmBrep::m_pVertexListHead` (`topology.brep_p1`), reached through
`topology.brep_p5`'s `GlueVertices` storage erasure. The stitch path is
therefore blocked by kernel storage/list integration, not by AOIM geometry.

The direct path avoids both gaps. `make_face_topology` is public and completes
the shared faces of the AOIM prism. When the last face closes the shell,
`TestClosureSplitRegion` still returns `Unported` from its omitted tail after
`CalculateBoundingBox` (`SmBrep.cpp:27966-28583`), but the split itself is
complete: it creates the void and solid shells, each face gets one faceuse in
each, and the old infinite region remains correct. The two shells/two regions
are the expected void/solid pair, not a merge failure.

For this simple prism none of the omitted tail is needed: the split already
leaves `Region0 void=1`, `Region1 void=0` and `infinite=Region0`. The probe
calls `reset_infinite_region` only to confirm the pointer and flags; before and
after are identical. `IsInnerShell`, `RayFire` distribution and empty-shell
cleanup are still required for general cases.

## Ownership

From the checked-in plan only (no `gh` client is available in this workspace):

- `topology.brep_p4`: `todo`, `claimed_by = ""`; owns `StitchFaces`, but the
  direct path does not use it.
- `topology.brep_p1`: `partial`, claimed by bluecmd (PR #145).
- `topology.brep_p5`: `partial` (PR #194), no current claim recorded.
- `topology.brep_p6`: `partial`, no current claim recorded; its
  `TestClosureSplitRegion` tail is incomplete, but the simple prism is already
  correctly split; `reset_infinite_region` only confirms it.
- `topology.brep_p7`: `partial` (PR #155), no current claim recorded.
- `heal.stitch_p1`: `partial`, claimed by bluecmd (PR #196).
- `heal.stitch_p2`: `partial`, claimed by bluecmd (PR #205).
- `intersect.curve_class_p4`: `partial` (PR #137), no current claim recorded.

The direct path needs none of the stitch owners. It still needs the full p6
closure tail for general cases; the simple prism already has the correct
infinite-region flags, and `reset_infinite_region` only confirms them.

## Probe results

The probes are preserved in `probes/brep_probe/` and reproduce with (Russian technical report: [`BREP_REPORT_RU.md`](BREP_REPORT_RU.md)):

```sh
cd research/aoim-line-arc/probes/brep_probe
cargo run --offline
```

Current output is summarized by case (the probe also prints per-face orientation):

| case | profile | contour | F / E / V | area |
|---|---|---:|---:|---:|
| 90° baseline | `[0, 2]` | 6 | 8 / 18 / 12 | 51.933998 |
| 90° symmetric | `[-2, 2]` | 7 | 9 / 21 / 14 | 105.358356 |
| 30° | `[-2, 2]` | 7 | 9 / 21 / 14 | 88.382434 |
| 150° | `[-2, 2]` | 7 | 9 / 21 / 14 | 127.197778 |
| 179° (`line_length=500`) | `[-2, 2]` | 7 | 9 / 21 / 14 | 2492.149982 |

For every case:

- each edge has 4 edge uses, with 2 `Same` and 2 `Opposite`;
- each face has one faceuse in the void shell and one in the solid shell;
- the two shells are `Region0` (void) and `Region1` (solid); the split already
  leaves the flags correct;
- both `caps-first` and `sides-first` orders pass;
- the last face still reports the p6 `TestClosureSplitRegion` tail, but the
  split is complete and the probe's assertions pass.

Other observations:

- `create_pyramid` builds all planar faces and then stops at
  `SmBrep::StitchFaces`; face construction is not the problem for the old path.
- Independent faces do not share topology (8 edges, 8 vertices for two
  squares).
- `DoStitching` is blocked earlier, at the p5/p1 vertex-list storage gap.
- The direct wire-edge + `make_face_topology` path shares edges and builds a
  box; its single shell contains both faceuses of every face, so it is not a
  split solid (a useful contrast, not the target).
- Reversing the bottom cap loop fixes the cap orientation; without it the split
  is 7/1 instead of 8/0 or 9/0.
- The negative `d0` adds a tangent Line–Arc segment, hence 7 contour segments
  and 9 faces for the symmetric profile.
- `brep_volume` is blocked by `SmSurface::DropCurve`
  (`intersect.surface_drop_lib_p3`), so volume is not numerically certified.
- `orient_trimmed_surfaces` is blocked by `CalculateBoundingBox`
  (`query.brep`), so outward-normal unification is not certified.
- The new shell has one faceuse per face but its `Shell.faces` list is empty;
  the faceuse list is authoritative for the queries used here, but this cache
  should be checked when the p6 tail is completed.
## Implementation order

1. Keep the plan and probes in this research branch; do not claim
   `topology.brep_p4`, `topology.brep_p6` or implement replacement topology
   storage.
2. Keep the 90° baseline as the regression test. The generalized builder has
   also passed 90/30/150/179 with `[-2, 2]` and the Line–Arc tangent segment;
   both face orders pass.
3. Keep the bottom-cap loop reversal and the checked `reset_infinite_region`
   confirmation. Next profile class is L/T; do not change the direct topology
   strategy.
4. Cross-check volume and area against the existing 2D hybrid and 3D mesh.
   Volume validation remains blocked until `SmSurface::DropCurve` and
   `CalculateBoundingBox` are available; do not substitute a guessed volume.
5. Document the p6 tail and `Shell.faces` cache findings in the Russian report
   and in `plan/findings.md` if the project accepts research findings. Do not
   port them in this branch.
6. Do not fall back to `StitchFaces`; its path is blocked earlier by kernel
   storage work.

## Decision

Direct shared-topology construction is the chosen architecture and has produced
closed AOIM B-reps for 90/30/150/179 with exact plane/cylinder side surfaces. The remaining gaps are
validation and generic-case completion:
`TestClosureSplitRegion` tail (`topology.brep_p6`), `CalculateBoundingBox`
(`query.brep`) and `SmSurface::DropCurve`
(`intersect.surface_drop_lib_p3`). None of them is AOIM geometry; none requires
returning to `StitchFaces`.
