# AOIM B-rep probe

Public-API probes for the direct-shell design in `../../BREP_PLAN.md`. The probe
does not modify the kernel.

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

What the probes establish:

- A planar cap with a circle arc and three lines can be built with
  `create_planar_face_with_3d_curves`.
- A trimmed cylindrical patch can be built with `create_face_from_surface`
  after `Cone::make_nurb`. The constructor takes degrees; the natural UV domain
  is in radians.
- Independently created adjacent faces do not share topology: two squares give
  two faces, eight edges and eight vertices.
- `create_pyramid` builds its faces and stops at `SmBrep::StitchFaces`
  (`topology.brep_p4`).
- Calling the already-ported `heal::stitch::p2::Stitch::do_stitching` directly
  gets past the missing `StitchFaces` wrapper but stops earlier at
  `SmBrep::m_pVertexListHead` (`topology.brep_p1`), reached from
  `topology.brep_p5`'s `GlueVertices` storage erasure.
- The direct path works up to closure: `make_shell_vertex` + `make_wire_edge` +
  `make_face_topology` builds two squares with one shared edge (2/7/6), a box
  (6/12/8, four edge uses per edge, one shell and region) and a cylindrical face
  from two lines and two arcs (1/4/4). No stitch or glue is used.
- The 90-degree AOIM prism builds all 8 faces/18 edges/12 vertices with four
  edge uses per edge. The last side face stops in the unported
  `TestClosureSplitRegion` tail (`topology.brep_p6`), leaving 2 shells/2 regions.

If a probe assertion fails after a kernel update, the recorded blocker or API
has changed and `BREP_PLAN.md` needs to be updated.



