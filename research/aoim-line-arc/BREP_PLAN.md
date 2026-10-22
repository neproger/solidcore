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

Expected topology for the six-segment rectangle: 8 faces, 18 edges, 12
vertices, one closed shell, `V - E + F = 2`. Volume is exactly
`union_area * (z1 - z0)`.

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
- A closed box: 6 faces, 12 edges, 8 vertices; every edge has 4 edge uses; one
  shell and one region.
- A cylindrical face from two vertical lines and two circle arcs: 1 face,
  4 edges, 4 vertices.

No `StitchFaces`, `GlueVertices`, Boolean, `project_and_trim` or seam surface
is used.

## Face construction

### Caps

Build the contour as wire edges at `z0` and `z1`, then call
`make_face_topology` with the ordered edge uses. Use a plane surface:

- bottom: outward normal `(0, 0, -1)`;
- top: outward normal `(0, 0, 1)`.

The cap loop is the ordered outer contour. There is one loop and one face per
cap.

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

Choose each surface normal outward and use the matching `OrientType`. If the
assembled shell has inconsistent normals, `topology::brep::p3::orient_trimmed_surfaces`
is ported; run it as a validation/fix-up step. Do not use it to replace the
direct topology.

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
7. Only if normals are inconsistent, run `orient_trimmed_surfaces` and
   re-check.

No `stitch_and_orient`, no `glue_*`, no Boolean and no `SurfOfExtrusion` is
required.

## Why this supersedes the stitch path

`topology::brep::p5::stitch_and_orient` still stops at
`SmBrep::StitchFaces` (`topology.brep_p4`). Calling the already-ported
`heal::stitch::p2::Stitch::do_stitching` directly gets past that wrapper but
stops at `SmBrep::m_pVertexListHead` (`topology.brep_p1`), reached through
`topology.brep_p5`'s `GlueVertices` storage erasure. The stitch path is
therefore blocked by kernel storage/list integration, not by AOIM geometry.

The direct path avoids both gaps. `make_face_topology` is public and has already
built the shared faces of the AOIM prism. Its remaining blocker is different:
when the last face closes the shell, `TestClosureSplitRegion`
(`topology.brep_p6`) reaches its omitted tail after `CalculateBoundingBox`
(`SmBrep.cpp:27966-28583`). That tail performs infinite-region testing, the
inner-shell choice, redistribution of faces/wires/shells joined only by vertices
and empty-shell/region cleanup. The prism is left with 2 shells and 2 regions;
the face/edge topology is otherwise complete.

## Probe results

The probes are preserved in `probes/brep_probe/` and reproduce with:

```sh
cd research/aoim-line-arc/probes/brep_probe
cargo run --offline
```

Current output:

```
PLANAR_ARC_CAP ok face=0
TWO_FACES faces=2 edges=8 vertices=8
CYLINDER_PATCH ok face=0 u_max=1.570796 v_max=2.000000
PYRAMID blocker=unported: upstream `SmBrep::StitchFaces` (unit topology.brep_p4) is not ported yet
DO_STITCHING blocker=unported: upstream `SmBrep::m_pVertexListHead` (unit topology.brep_p1) is not ported yet
DIRECT_FACE_A face=0
DIRECT_FACE_B face=1
DIRECT_SHARED faces=2 edges=7 vertices=6
BOX_FACE bottom face=0
BOX_FACE top face=1
BOX_FACE front face=2
BOX_FACE right face=3
BOX_FACE back face=4
BOX_FACE left face=5
BOX faces=6 edges=12 vertices=8
BOX_EDGE_USES min=4 max=4 all=[4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]
BOX_SHELLS shells=1 regions=1
DIRECT_CYLINDER_FACE face=0 faces=1 edges=4 vertices=4
AOIM_FACE bottom face=0
AOIM_FACE top face=1
AOIM_FACE side0 face=2
AOIM_FACE side1 face=3
AOIM_FACE side2 face=4
AOIM_FACE side3 face=5
AOIM_FACE side4 face=6
AOIM_FACE side5 err=Unported { upstream: "SmBrep::TestClosureSplitRegion (after CalculateBoundingBox, SmBrep.cpp:27966-28583)", unit: "topology.brep_p6" }
DIRECT_AOIM_PRISM faces=8 edges=18 vertices=12 edgeuses=4..4 shells=2 regions=2 area=51.933998
```

Interpretation:

- `create_pyramid` builds all planar faces and then stops at
  `SmBrep::StitchFaces`; face construction is not the issue for the old path.
- Independent faces do not share topology (8 edges, 8 vertices for two
  squares).
- `DoStitching` is blocked earlier, at the p5/p1 vertex-list storage gap.
- The direct wire-edge + `make_face_topology` path shares edges and builds a
  box with correct manifold edge-use counts.
- A cylindrical face with arc boundaries can be built the same way.
- The AOIM prism builds all 8 faces and shares every edge (4 edge uses per
  edge), but the final closure stops at the unported `TestClosureSplitRegion`
  tail and leaves 2 shells/2 regions.

## Implementation order

1. Keep the plan and probes in this research branch; do not claim
   `topology.brep_p4`, `topology.brep_p6` or implement replacement topology
   storage.
2. The 90-degree six-segment prism probe is already in place and reaches the
   real blocker: all faces/edges are built, but the last face's closure split is
   unported.
3. Stop B-rep integration here until `TestClosureSplitRegion` has its omitted
   tail. Re-run the probe then; if it completes, extend the same builder to
   `30/150/179`, negative-profile and L/T-style contours.
4. Cross-check volume and area against the existing 2D hybrid and 3D mesh only
   after the closure split succeeds.
5. Do not fall back to `StitchFaces`; its path is blocked earlier by the same
   kernel storage work.

## Decision

Use direct shared-topology construction as the architecture. It avoids
`StitchFaces` and matches the AOIM goal: `AOIM geometry -> shared B-rep topology
-> closed shell`. The remaining block is the kernel's closed-region split
(`topology.brep_p6`), not AOIM geometry and not face/edge sharing. Stop here
until that core operation is available.
