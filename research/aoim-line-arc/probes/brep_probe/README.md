# AOIM B-rep probe

Public-API probes for the direct-shell design in `../../BREP_PLAN.md`. The probe
does not modify the kernel.

```sh
cd research/aoim-line-arc/probes/brep_probe
cargo run --offline
```

Current output (abridged; the probe also prints per-face orientation):

```
PLANAR_ARC_CAP ok face=0
TWO_FACES faces=2 edges=8 vertices=8
CYLINDER_PATCH ok face=0 u_max=1.570796 v_max=2.000000
PYRAMID blocker=unported: upstream `SmBrep::StitchFaces` (unit topology.brep_p4) is not ported yet
DO_STITCHING blocker=unported: upstream `SmBrep::m_pVertexListHead` (unit topology.brep_p1) is not ported yet
DIRECT_SHARED faces=2 edges=7 vertices=6
BOX faces=6 edges=12 vertices=8
BOX_EDGE_USES min=4 max=4
BOX_SHELLS shells=1 regions=1
DIRECT_CYLINDER_FACE face=0 faces=1 edges=4 vertices=4
AOIM_FACE side5 err=Unported { upstream: "SmBrep::TestClosureSplitRegion (after CalculateBoundingBox, SmBrep.cpp:27966-28583)", unit: "topology.brep_p6" }
AOIM_REGION RegionId(0) void=1 shells=1
AOIM_REGION RegionId(1) void=0 shells=1
AOIM_SHELL ShellId(0) faceuses=8 same=8 opposite=0
AOIM_SHELL ShellId(12) faceuses=8 same=0 opposite=8
DIRECT_AOIM_PRISM order=caps-first faces=8 edges=18 vertices=12 edgeuses=4..4 shells=2 regions=2 area=51.933998
AOIM_RESET_INFINITE ok old_infinite=RegionId(0)
AOIM_EDGE_ORIENT same=2..2 opposite=2..2
AOIM_VOLUME err=Unported { upstream: "SmSurface::DropCurve", unit: "intersect.surface_drop_lib_p3" }
AOIM_FACE_SHELLS faces=8 each_face_two_distinct_shells=true
AOIM_ORIENT err=Unported { upstream: "SmBrep::CalculateBoundingBox", unit: "query.brep" }
AOIM_PROBE_PASS order=caps-first
...
AOIM_PROBE_PASS order=sides-first
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
- Calling `heal::stitch::p2::Stitch::do_stitching` directly gets past the
  missing `StitchFaces` wrapper but stops earlier at
  `SmBrep::m_pVertexListHead` (`topology.brep_p1`), reached from
  `topology.brep_p5`'s `GlueVertices` storage erasure.
- The direct path works: `make_shell_vertex` + `make_wire_edge` +
  `make_face_topology` builds two squares with one shared edge (2/7/6), a box
  (6/12/8, four edge uses per edge), a cylindrical face (1/4/4), and a closed
  90-degree AOIM prism (8/18/12, four edge uses per edge, two shells/regions).
- The AOIM prism still receives the p6 `TestClosureSplitRegion` error, but the
  split is already complete. `reset_infinite_region` on the known pre-split
  infinite region confirms the flags (before/after identical), and both face-order
  variants pass.
- `brep_volume` and `orient_trimmed_surfaces` remain blocked by
  `SmSurface::DropCurve` and `CalculateBoundingBox`; volume/orientation are not
  numerically certified.

See `../../BREP_REPORT_RU.md` for the detailed Russian report. If a probe
assertion fails after a kernel update, the recorded blocker or API has changed
and the report needs updating.
