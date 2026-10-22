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

## Face construction

### Caps

Use `topology::brep::p8::create_planar_face_with_3d_curves` for each cap
(`z0`, `z1`) with the analytic contour curves in order. The function accepts a
mixed line/arc loop.

Probe result: a planar face with one circle arc and three lines succeeds. A
two-edge line+arc loop returned `SM_ERR`; use loops with at least three edges
(the outer contour has six or more).

### Line side faces

Each vertical extrusion of a contour line segment is a planar rectangle. Build
it with `create_planar_face_with_3d_curves` using four `Curve::Line` edges, or
with a plane surface and `create_face_from_surface`. The four-line form avoids
having to choose a plane UV frame.

### Arc side faces

Each vertical extrusion of a contour arc is a cylindrical rectangular patch in
`(angle, z)` parameter space. Build the cylinder with
`surface::cone::p1::Cone::new_cylinder_bounded`:

- origin at the arc center with `z = z0`;
- local `x_axis` pointing to the arc start and `y_axis` along the arc tangent;
- constructor angles in **degrees**; use `start = 0`, `end = span_degrees`;
- height `z1 - z0`;
- then call `make_nurb(&k)`; without it the surface has an empty natural UV
  domain and `create_face_from_surface` fails with `SM_ERR`.

Use `create_face_from_surface` with the surface and `Extent2d`:

```
U (radians): 0 .. span_radians
V:           0 .. (z1 - z0)
```

Probe result: this produces a valid cylindrical face. The constructor's
angles are degrees, while the natural UV domain is radians; do not pass
radians to `new_cylinder_bounded`.

`SurfOfExtrusion` is not in `SurfaceKind` and is not needed: the final shell
has no seam face.

## Assembly

1. Build the 2D union and an analytic `Vec<ContourSegment>` with exact shared
   endpoints. Reuse the same vertex coordinates across adjacent faces.
2. Create one `Brep` with `topology::brep::p3::new_brep` (or
   `Brep::new_with_infinite_region`). Use the infinite region for face
   creation, as the existing face builders do.
3. Add the two cap faces and all side faces.
4. Call `topology::brep::p5::stitch_and_orient` with
   `StitchAndOrientOptions::default()`.
5. Verify: one closed shell/region, manifold edges, 8 faces (six-segment
   case), no interior seam, volume `area * height`, outward normals, and the
   same result for `30/90/150/179` degrees and for a rectangle with negative
   profile portions.

No `project_and_trim`, no Boolean, and no `SurfOfExtrusion` is required.

## Current blocker

`topology::brep::p5::stitch_and_orient` calls `p5::stitch_faces`, which is a
stub:

```
UNPORTED(topology.brep_p4): SmBrep::StitchFaces
```

`SmBrep::StitchFaces` is listed under `topology.brep_p4` in
`harness/callgraph/units.json`, but `plan/units/topology.brep_p4.toml` has
`status = "todo"` and its notes list only `DeleteEdge`, `ShrinkGeometry`, the
destructor, and `DeleteVertex`. The plan metadata omits `StitchFaces`; it must
be added to that unit's scope when the unit is claimed.

Probe results:

- `primitive::api_primitives::create_pyramid` builds all five planar faces and
  then fails only with
  `Unported { upstream: "SmBrep::StitchFaces", unit: "topology.brep_p4" }`.
  Face construction and `MakeFaceTopology` are therefore not the blocker.
- Two adjacent planar faces created independently produce 8 edges and 8
  vertices, not shared topology. Stitching (or equivalent edge/vertex glue) is
  required.
- `topology.brep_p7::glue_edges_general` is partial; real coincident pairs can
  defer at `intersect.curve_class_p4` or `topology.brep_p4::DeleteEdge`.
  It is not a drop-in replacement for `StitchFaces`.
- `topology.brep_p4::DeleteEdge` is also unported, so a faithful
  `StitchFaces` port may need more than that one function.

`SmBrep::OrientTrimmedSurfaces` (`topology.brep_p3`) is ported, but some paths
still defer at `topology.face_p3` / `topology.face_p4`. The assembled solid
must be run through difftest before claiming orientation is green.

## Implementation order

1. Core prerequisite: claim `topology.brep_p4` and port `SmBrep::StitchFaces`
   (plus the `DeleteEdge`/glue dependencies it actually reaches); wire
   `topology.brep_p5::stitch_faces` to it. This is a kernel PR, not a research
   change.
2. In `research/aoim-line-arc`, add an analytic `outer_contour_segments`
   function (line/arc, exact endpoints) beside the existing sampled
   `expected_polygon`; keep the mesh code unchanged.
3. Add a B-rep prototype behind a separate example/binary using the public
   `solidcore` API. Start with the six-segment 90-degree rectangle, then
   `30/150/179`, then negative-profile and L/T-style contours.
4. Cross-check volume and area against the existing 2D hybrid and 3D mesh;
   reject if the B-rep volume or topology disagrees.
5. Only after the prototype is stable, consider moving the builder into the
   kernel/Sweep API. Do not port `CreateSweepAlongPlanarPath` or the mesh
   pipeline first.

## Decision needed

The next substantive step is a core port of `SmBrep::StitchFaces`. If that unit
is already owned elsewhere, the alternative is to keep the B-rep prototype
blocked and use the existing mesh only for research. A research-only manual
edge-sharing assembler would bypass the real pipeline and is not recommended.
