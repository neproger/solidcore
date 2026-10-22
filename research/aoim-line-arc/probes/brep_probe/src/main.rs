//! Reproducible B-rep probes for the AOIM direct-shell design.
//!
//! These probes use only the public `solidcore` API. They are not part of the
//! kernel or its difftests; they record what the current kernel can and cannot
//! do before a direct B-rep builder is attempted.
//!
//! Run from this directory:
//!
//! ```sh
//! cargo run --offline
//! ```

use solidcore::curve::curve::Curve;
use solidcore::curve::ellipse::p2::Ellipse;
use solidcore::curve::line::Line;
use solidcore::error::{Result, SolidError};
use solidcore::geom::attribute::ObjectKey;
use solidcore::geom::extent1d::Extent1d;
use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::object::ObjectRef;
use solidcore::heal::stitch::p1::{DefaultStitchCallback, Stitch};
use solidcore::heal::stitch::p2::MinGapRequest;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::primitive::api_primitives::create_pyramid;
use solidcore::surface::cone::p1::Cone;
use solidcore::surface::surface::Surface;
use solidcore::topology::brep::{p1, p2, p3, p8};
use solidcore::topology::edge::NotifyEnv;
use solidcore::topology::topology_traverser::Marks;
use solidcore::topology::RegionId;
use solidcore::{Brep, EdgeId, FaceId};

fn face_ref(f: FaceId) -> ObjectRef {
    ObjectRef {
        type_code: 16002,
        key: ObjectKey(1 + (4u64 << 32) + u64::from(f.index())),
    }
}

fn new_brep(k: &mut Kernel) -> (Brep, Option<RegionId>) {
    let b = p3::new_brep(
        k,
        solidcore::geom::tol::USE_DEFAULT,
        solidcore::geom::tol::USE_DEFAULT,
    )
    .expect("new B-rep");
    let region = b.infinite_region();
    (b, region)
}

fn line(a: Point3, b: Point3) -> Curve {
    Curve::from(Line::create_line_segment(3, a, b, None).expect("line segment"))
}

fn add_planar_face(
    k: &mut Kernel,
    b: &mut Brep,
    region: Option<RegionId>,
    boundary: &[Curve],
    env: &mut NotifyEnv<'_>,
    marks: &mut Marks,
) -> Result<Option<FaceId>> {
    let this_face = |f: FaceId| face_ref(f);
    let mut fenv = p8::FaceBuildEnv {
        env,
        marks,
        this_face: &this_face,
    };
    p8::create_planar_face_with_3d_curves(k, b, region, boundary, 1e-6, &mut fenv)
}

fn counts(b: &Brep) -> (usize, usize, usize) {
    (
        p1::faces(b, None).expect("faces").len(),
        p1::edges(b, None).expect("edges").len(),
        p1::vertices(b, None).expect("vertices").len(),
    )
}

fn assert_unported(err: &SolidError, upstream: &str, unit: &str) {
    match err {
        SolidError::Unported {
            upstream: got_upstream,
            unit: got_unit,
        } => {
            assert_eq!(*got_upstream, upstream);
            assert_eq!(*got_unit, unit);
        }
        other => panic!("expected Unported({upstream}, {unit}), got {other:?}"),
    }
}

fn main() {
    probe_planar_arc_cap();
    probe_two_independent_faces();
    probe_cylinder_patch();
    probe_pyramid_blocker();
    probe_do_stitching_blocker();
    probe_direct_shared_topology();
    probe_closed_box();
    probe_direct_cylinder_face();
    probe_direct_aoim_prism();
}

fn probe_planar_arc_cap() {
    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let arc = Ellipse::circle_from_3_points(
        &k,
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(3.0, 1.0, 0.0),
        Point3::new(2.0, 2.0, 0.0),
        3,
        false,
    )
    .expect("circle arc");
    let boundary = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        Curve::from(arc),
        line(Point3::new(2.0, 2.0, 0.0), Point3::new(0.0, 2.0, 0.0)),
        line(Point3::new(0.0, 2.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
    ];

    let face = add_planar_face(&mut k, &mut b, region, &boundary, &mut env, &mut marks)
        .expect("planar line/arc cap")
        .expect("cap face");
    println!("PLANAR_ARC_CAP ok face={}", face.index());
}

fn probe_two_independent_faces() {
    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let a = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let c = [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    for pts in [a, c] {
        let boundary = [
            line(pts[0], pts[1]),
            line(pts[1], pts[2]),
            line(pts[2], pts[3]),
            line(pts[3], pts[0]),
        ];
        add_planar_face(&mut k, &mut b, region, &boundary, &mut env, &mut marks)
            .expect("square face")
            .expect("face id");
    }

    let (faces, edges, vertices) = counts(&b);
    println!("TWO_FACES faces={faces} edges={edges} vertices={vertices}");
    assert_eq!((faces, edges, vertices), (2, 8, 8));
}

fn probe_cylinder_patch() {
    let mut k = Kernel::default();
    let (mut b, _region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();
    let this_face = |f: FaceId| face_ref(f);
    let mut fenv = p8::FaceBuildEnv {
        env: &mut env,
        marks: &mut marks,
        this_face: &this_face,
    };

    let mut cylinder = Cone::new_cylinder_bounded(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.0,
        0.0,
        90.0,
        2.0,
        false,
        false,
        false,
    );
    cylinder.make_nurb(&k).expect("cylinder NURB");
    let surface = Surface::from(cylinder);
    let natural = surface.natural_uv_domain();
    let domain = Extent2d::from_bounds(0.0, 0.0, std::f64::consts::FRAC_PI_2, 2.0);
    assert!(domain.is_contained_by(&natural, 1e-12));

    let face = p8::create_face_from_surface(&mut k, &mut b, Some(surface), &domain, &mut fenv)
        .expect("cylinder patch");
    println!(
        "CYLINDER_PATCH ok face={} u_max={:.6} v_max={:.6}",
        face.index(),
        natural.u_max(),
        natural.v_max()
    );
}

fn probe_pyramid_blocker() {
    let mut k = Kernel::default();
    let err = create_pyramid(&mut k, Point3::new(0.0, 0.0, 0.0), 2.0, 3.0)
        .expect_err("pyramid must currently stop at stitching");
    assert_unported(&err, "SmBrep::StitchFaces", "topology.brep_p4");
    println!("PYRAMID blocker={err}");
}

fn probe_do_stitching_blocker() {
    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let a = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    ];
    let c = [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
    ];
    for pts in [a, c] {
        let boundary = [
            line(pts[0], pts[1]),
            line(pts[1], pts[2]),
            line(pts[2], pts[3]),
            line(pts[3], pts[0]),
        ];
        add_planar_face(&mut k, &mut b, region, &boundary, &mut env, &mut marks)
            .expect("square face")
            .expect("face id");
    }
    let before = counts(&b);

    let mut callback = DefaultStitchCallback;
    let mut stitch = Stitch::new(&mut callback, 1e-6, true, false);
    stitch.fast_edge_compare = true;
    stitch.ignore_problems = true;
    stitch.squeeze_small_edges = true;
    stitch.making_manifold_solid = true;
    stitch.remove_laminar_slivers = true;
    let err = stitch
        .do_stitching(&mut k, &mut b, None, None, MinGapRequest::default())
        .expect_err("the current heal stitch path must stop at the storage gap");
    assert_unported(&err, "SmBrep::m_pVertexListHead", "topology.brep_p1");
    let after = counts(&b);
    assert_eq!(before, after);
    println!("DO_STITCHING blocker={err}");
}

fn probe_direct_shared_topology() {
    use solidcore::math::Vec2;
    use solidcore::surface::plane::Plane;
    use solidcore::topology::brep::p3 as brep_p3;
    use solidcore::topology::brep::p6;
    use solidcore::topology::brep::p9;
    use solidcore::topology::edge;

    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);

    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let mut v = Vec::new();
    for p in points {
        let (_shell, vertex) =
            p9::make_shell_vertex(&mut b, region, p, &mut env).expect("shell vertex");
        v.push(vertex);
    }

    let mut edge = |a: usize, c: usize| {
        let curve = line(points[a], points[c]);
        let interval = Extent1d::from_min_max(0.0, 1.0);
        brep_p3::make_wire_edge(
            &mut k,
            &mut b,
            &mut env,
            region,
            v[a],
            v[c],
            curve,
            &interval,
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        )
        .expect("wire edge")
    };
    let ab = edge(0, 1);
    let bc = edge(1, 2);
    let cd = edge(2, 3);
    let da = edge(3, 0);
    let cf = edge(2, 5);
    let fe = edge(5, 4);
    let eb = edge(4, 1);

    let domain = Extent2d::from_bounds(0.0, 0.0, 2.0, 1.0);
    let plane = Plane::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec2::new(1.0, 1.0),
        &domain,
    );
    let surface = Surface::from(plane);
    let mut marks = Marks::default();
    let face_uses = |b: &Brep, e| {
        edge::edgeuses(b, e, None, None)
            .expect("edge uses")
            .into_iter()
            .next()
            .expect("edge use")
    };

    let a = [
        edge::edgeuses(&b, ab, None, None).unwrap()[0],
        edge::edgeuses(&b, bc, None, None).unwrap()[0],
        edge::edgeuses(&b, cd, None, None).unwrap()[0],
        edge::edgeuses(&b, da, None, None).unwrap()[0],
    ];
    let opts_a = p6::FaceTopologyOptions {
        region,
        edge_loops: &[4],
        edgeuses: &a,
        ..Default::default()
    };
    let topo_a = p6::make_face_topology(
        &mut k,
        &mut b,
        &mut marks,
        &mut env,
        Some(surface.clone()),
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        &opts_a,
    )
    .expect("face A");
    println!("DIRECT_FACE_A face={}", topo_a.face.index());

    let b_eus = [
        face_uses(&b, bc),
        face_uses(&b, cf),
        face_uses(&b, fe),
        face_uses(&b, eb),
    ];
    let opts_b = p6::FaceTopologyOptions {
        region,
        edge_loops: &[4],
        edgeuses: &b_eus,
        ..Default::default()
    };
    let topo_b = p6::make_face_topology(
        &mut k,
        &mut b,
        &mut marks,
        &mut env,
        Some(surface),
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        &opts_b,
    )
    .expect("face B");
    println!("DIRECT_FACE_B face={}", topo_b.face.index());

    let (faces, edges, vertices) = counts(&b);
    println!("DIRECT_SHARED faces={faces} edges={edges} vertices={vertices}");
}

fn probe_closed_box() {
    use solidcore::surface::plane::Plane;
    use solidcore::topology::brep::p3 as brep_p3;
    use solidcore::topology::brep::p6;
    use solidcore::topology::brep::p9;
    use solidcore::topology::edge;

    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let p = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 0.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(0.0, 1.0, 1.0),
    ];
    let mut v = Vec::new();
    for point in p {
        let (_shell, vertex) =
            p9::make_shell_vertex(&mut b, region, point, &mut env).expect("shell vertex");
        v.push(vertex);
    }
    let interval = Extent1d::from_min_max(0.0, 1.0);
    let mut wire = |a: usize, c: usize| {
        brep_p3::make_wire_edge(
            &mut k,
            &mut b,
            &mut env,
            region,
            v[a],
            v[c],
            line(p[a], p[c]),
            &interval,
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        )
        .expect("wire edge")
    };
    let bottom = [wire(0, 1), wire(1, 2), wire(2, 3), wire(3, 0)];
    let top = [wire(4, 5), wire(5, 6), wire(6, 7), wire(7, 4)];
    let vertical = [wire(0, 4), wire(1, 5), wire(2, 6), wire(3, 7)];

    let plane = |origin, normal| Surface::from(Plane::from_normal(origin, normal));
    let faces: [(
        &str,
        Surface,
        [EdgeId; 4],
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType,
    ); 6] = [
        (
            "bottom",
            plane(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, -1.0)),
            [bottom[0], bottom[1], bottom[2], bottom[3]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
        (
            "top",
            plane(Point3::new(0.0, 0.0, 1.0), Vec3::new(0.0, 0.0, 1.0)),
            [top[0], top[1], top[2], top[3]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
        (
            "front",
            plane(Point3::new(0.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0)),
            [bottom[0], vertical[1], top[0], vertical[0]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
        (
            "right",
            plane(Point3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0)),
            [bottom[1], vertical[2], top[1], vertical[1]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
        (
            "back",
            plane(Point3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)),
            [bottom[2], vertical[3], top[2], vertical[2]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
        (
            "left",
            plane(Point3::new(0.0, 0.0, 0.0), Vec3::new(-1.0, 0.0, 0.0)),
            [bottom[3], vertical[0], top[3], vertical[3]],
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        ),
    ];
    for (name, surface, edges, orientation) in faces {
        let edgeuses: Vec<_> = edges
            .iter()
            .map(|&e| edge::edgeuses(&b, e, None, None).unwrap()[0])
            .collect();
        let options = p6::FaceTopologyOptions {
            region,
            edge_loops: &[4],
            edgeuses: &edgeuses,
            ..Default::default()
        };
        match p6::make_face_topology(
            &mut k,
            &mut b,
            &mut marks,
            &mut env,
            Some(surface),
            orientation,
            &options,
        ) {
            Ok(face) => println!("BOX_FACE {name} face={}", face.face.index()),
            Err(e) => {
                println!("BOX_FACE {name} err={e:?}");
                return;
            }
        }
    }
    let (faces, edges, vertices) = counts(&b);
    println!("BOX faces={faces} edges={edges} vertices={vertices}");
    let mut min_uses = usize::MAX;
    let mut max_uses = 0;
    let mut edge_list = Vec::new();
    for e in p1::edges(&b, None).unwrap() {
        let n = edge::edgeuses(&b, e, None, None).unwrap().len();
        min_uses = min_uses.min(n);
        max_uses = max_uses.max(n);
        edge_list.push(n);
    }
    println!("BOX_EDGE_USES min={min_uses} max={max_uses} all={edge_list:?}");
    let shell_count = p1::shells(&mut k, &b, None).map(|v| v.len()).unwrap_or(0);
    let region_count = p1::regions(&b, None).map(|v| v.len()).unwrap_or(0);
    println!("BOX_SHELLS shells={shell_count} regions={region_count}");
}

fn edge_use_from(
    b: &Brep,
    edge: EdgeId,
    start: solidcore::VertexId,
) -> solidcore::topology::EdgeuseId {
    use solidcore::topology::edgeuse::p1 as eu1;
    use solidcore::topology::vertexuse as vu1;
    for eu in solidcore::topology::edge::edgeuses(b, edge, None, None).expect("edge uses") {
        let Some(vu) = eu1::vertexuse(b, eu).expect("edgeuse vertexuse") else {
            continue;
        };
        if vu1::vertex(b, vu).expect("vertexuse vertex") == Some(start) {
            return eu;
        }
    }
    panic!("no edge use starts at the requested vertex");
}

fn probe_direct_cylinder_face() {
    use solidcore::surface::cone::p1::Cone;
    use solidcore::topology::brep::p3 as brep_p3;
    use solidcore::topology::brep::p6;
    use solidcore::topology::brep::p9;

    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();
    let mut v = Vec::new();
    for p in [
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(1.0, 0.0, 1.0),
    ] {
        let (_shell, vertex) = p9::make_shell_vertex(&mut b, region, p, &mut env).expect("vertex");
        v.push(vertex);
    }
    let arc = |a, mid, c| {
        Curve::from(Ellipse::circle_from_3_points(&k, a, mid, c, 3, false).expect("arc"))
    };
    let bottom = arc(
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.7071067811865476, 0.7071067811865476, 0.0),
        Point3::new(0.0, 1.0, 0.0),
    );
    let top = arc(
        Point3::new(0.0, 1.0, 1.0),
        Point3::new(0.7071067811865476, 0.7071067811865476, 1.0),
        Point3::new(1.0, 0.0, 1.0),
    );
    let bottom_iv = bottom.natural_interval();
    let top_iv = top.natural_interval();
    let bottom_edge = brep_p3::make_wire_edge(
        &mut k,
        &mut b,
        &mut env,
        region,
        v[0],
        v[1],
        bottom,
        &bottom_iv,
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
    )
    .expect("bottom arc edge");
    let top_edge = brep_p3::make_wire_edge(
        &mut k,
        &mut b,
        &mut env,
        region,
        v[2],
        v[3],
        top,
        &top_iv,
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
    )
    .expect("top arc edge");
    let iv = Extent1d::from_min_max(0.0, 1.0);
    let vert_bc = brep_p3::make_wire_edge(
        &mut k,
        &mut b,
        &mut env,
        region,
        v[1],
        v[2],
        line(Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, 1.0, 1.0)),
        &iv,
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
    )
    .expect("bc");
    let vert_da = brep_p3::make_wire_edge(
        &mut k,
        &mut b,
        &mut env,
        region,
        v[3],
        v[0],
        line(Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 0.0, 0.0)),
        &iv,
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
    )
    .expect("da");

    let eus = [
        edge_use_from(&b, bottom_edge, v[0]),
        edge_use_from(&b, vert_bc, v[1]),
        edge_use_from(&b, top_edge, v[2]),
        edge_use_from(&b, vert_da, v[3]),
    ];
    let mut cylinder = Cone::new_cylinder_bounded(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        1.0,
        0.0,
        90.0,
        1.0,
        false,
        false,
        false,
    );
    cylinder.make_nurb(&k).expect("cylinder NURB");
    let options = p6::FaceTopologyOptions {
        region,
        edge_loops: &[4],
        edgeuses: &eus,
        ..Default::default()
    };
    match p6::make_face_topology(
        &mut k,
        &mut b,
        &mut marks,
        &mut env,
        Some(Surface::from(cylinder)),
        solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
        &options,
    ) {
        Ok(face) => {
            let (faces, edges, vertices) = counts(&b);
            println!(
                "DIRECT_CYLINDER_FACE face={} faces={faces} edges={edges} vertices={vertices}",
                face.face.index()
            );
        }
        Err(e) => println!("DIRECT_CYLINDER_FACE err={e:?}"),
    }
}

fn probe_direct_aoim_prism() {
    use aoim_line_arc_research::sweep::{build_regions, seam_point, Strategy};
    use aoim_line_arc_research::{LineArc, Point2};
    use solidcore::surface::plane::Plane;
    use solidcore::topology::brep::p3 as brep_p3;
    use solidcore::topology::brep::p6;
    use solidcore::topology::brep::p9;

    #[derive(Clone, Copy)]
    enum Seg {
        Line {
            a: Point2,
            b: Point2,
        },
        Arc {
            c: Point2,
            r: f64,
            a0: f64,
            a1: f64,
            reversed: bool,
        },
    }

    let model = LineArc::new(4.0, 90.0_f64.to_radians()).expect("model");
    let d0 = 0.0;
    let d1 = 2.0;
    let line_length = 20.0;
    let z0 = -1.0;
    let z1 = 1.0;
    let built = build_regions(
        &model,
        Strategy::HybridAtJunction,
        d0,
        d1,
        line_length,
        401,
        96,
    )
    .expect("2D regions");
    let center = model.center();
    let seam0 = seam_point(&model, Strategy::HybridAtJunction, d0).expect("seam d0");
    let seam1 = seam_point(&model, Strategy::HybridAtJunction, d1).expect("seam d1");
    let arc_angle = |d: f64| {
        let seam = seam_point(&model, Strategy::HybridAtJunction, d).expect("seam");
        (
            model.arc_start_angle() + model.arc_parameter(seam),
            model.arc_end_angle(),
        )
    };
    let arc_end = |radius: f64| Point2 {
        x: center.x + radius * model.arc_end_angle().cos(),
        y: center.y + radius * model.arc_end_angle().sin(),
    };
    let (outer_a0, outer_a1) = arc_angle(d1);
    let (inner_a0, inner_a1) = arc_angle(d0);
    let outer_end = arc_end(model.radius + d1);
    let inner_end = arc_end(model.radius + d0);
    let contour = [
        Point2 {
            x: -line_length,
            y: -d1,
        },
        seam1,
        outer_end,
        inner_end,
        seam0,
        Point2 {
            x: -line_length,
            y: -d0,
        },
    ];
    let segments = [
        Seg::Line {
            a: contour[0],
            b: contour[1],
        },
        Seg::Arc {
            c: center,
            r: model.radius + d1,
            a0: outer_a0,
            a1: outer_a1,
            reversed: false,
        },
        Seg::Line {
            a: contour[2],
            b: contour[3],
        },
        Seg::Arc {
            c: center,
            r: model.radius + d0,
            a0: inner_a0,
            a1: inner_a1,
            reversed: true,
        },
        Seg::Line {
            a: contour[4],
            b: contour[5],
        },
        Seg::Line {
            a: contour[5],
            b: contour[0],
        },
    ];
    let n = contour.len();

    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let mut bottom_v = Vec::with_capacity(n);
    let mut top_v = Vec::with_capacity(n);
    for p in contour {
        let (_s, vb) = p9::make_shell_vertex(&mut b, region, Point3::new(p.x, p.y, z0), &mut env)
            .expect("bottom vertex");
        let (_s, vt) = p9::make_shell_vertex(&mut b, region, Point3::new(p.x, p.y, z1), &mut env)
            .expect("top vertex");
        bottom_v.push(vb);
        top_v.push(vt);
    }

    let make_edge =
        |k: &mut Kernel, b: &mut Brep, env: &mut NotifyEnv<'_>, start, end, curve: Curve| {
            let interval = curve.natural_interval();
            brep_p3::make_wire_edge(
                k,
                b,
                env,
                region,
                start,
                end,
                curve,
                &interval,
                solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
            )
            .expect("wire edge")
        };
    let arc_curve = |k: &Kernel, c: Point2, r: f64, a0: f64, a1: f64, reversed: bool, z: f64| {
        let angle = |t: f64| Point3::new(c.x + r * t.cos(), c.y + r * t.sin(), z);
        let (s, e) = if reversed { (a1, a0) } else { (a0, a1) };
        Curve::from(
            Ellipse::circle_from_3_points(&k, angle(s), angle(0.5 * (a0 + a1)), angle(e), 3, false)
                .expect("arc"),
        )
    };

    let mut bottom_e = Vec::with_capacity(n);
    let mut top_e = Vec::with_capacity(n);
    for i in 0..n {
        let j = (i + 1) % n;
        let (cb, ct) = match segments[i] {
            Seg::Line { a, b: c } => (
                line(Point3::new(a.x, a.y, z0), Point3::new(c.x, c.y, z0)),
                line(Point3::new(a.x, a.y, z1), Point3::new(c.x, c.y, z1)),
            ),
            Seg::Arc {
                c,
                r,
                a0,
                a1,
                reversed,
            } => (
                arc_curve(&k, c, r, a0, a1, reversed, z0),
                arc_curve(&k, c, r, a0, a1, reversed, z1),
            ),
        };
        bottom_e.push(make_edge(
            &mut k,
            &mut b,
            &mut env,
            bottom_v[i],
            bottom_v[j],
            cb,
        ));
        top_e.push(make_edge(&mut k, &mut b, &mut env, top_v[i], top_v[j], ct));
    }
    let mut vertical_e = Vec::with_capacity(n);
    for i in 0..n {
        let curve = line(
            Point3::new(contour[i].x, contour[i].y, z0),
            Point3::new(contour[i].x, contour[i].y, z1),
        );
        vertical_e.push(make_edge(
            &mut k,
            &mut b,
            &mut env,
            bottom_v[i],
            top_v[i],
            curve,
        ));
    }

    let plane = |p: Point2, z: f64, normal: Vec3| {
        Surface::from(Plane::from_normal(Point3::new(p.x, p.y, z), normal))
    };
    let bottom_surface = plane(Point2 { x: 0.0, y: 0.0 }, z0, Vec3::new(0.0, 0.0, -1.0));
    let top_surface = plane(Point2 { x: 0.0, y: 0.0 }, z1, Vec3::new(0.0, 0.0, 1.0));
    let mut surfaces = Vec::with_capacity(n);
    for seg in segments {
        let surface = match seg {
            Seg::Line { a, b: c } => {
                let dx = c.x - a.x;
                let dy = c.y - a.y;
                plane(a, z0, Vec3::new(dy, -dx, 0.0))
            }
            Seg::Arc { c, r, a0, a1, .. } => {
                let mut cone = Cone::new_cylinder_bounded(
                    Point3::new(c.x, c.y, z0),
                    Vec3::new(a0.cos(), a0.sin(), 0.0),
                    Vec3::new(-a0.sin(), a0.cos(), 0.0),
                    r,
                    0.0,
                    (a1 - a0).to_degrees(),
                    z1 - z0,
                    false,
                    false,
                    false,
                );
                cone.make_nurb(&k).expect("cylinder NURB");
                Surface::from(cone)
            }
        };
        surfaces.push(surface);
    }

    let mut make_face = |name: &str,
                         surface: Surface,
                         edges: &[EdgeId],
                         starts: &[solidcore::VertexId],
                         loop_len: usize| {
        let eus: Vec<_> = edges
            .iter()
            .zip(starts)
            .map(|(&e, &s)| edge_use_from(&b, e, s))
            .collect();
        let options = p6::FaceTopologyOptions {
            region,
            edge_loops: &[loop_len],
            edgeuses: &eus,
            ..Default::default()
        };
        match p6::make_face_topology(
            &mut k,
            &mut b,
            &mut marks,
            &mut env,
            Some(surface),
            solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same,
            &options,
        ) {
            Ok(face) => println!("AOIM_FACE {name} face={}", face.face.index()),
            Err(e) => {
                println!("AOIM_FACE {name} err={e:?}");
                return;
            }
        }
    };

    make_face("bottom", bottom_surface, &bottom_e, &bottom_v, n);
    make_face("top", top_surface, &top_e, &top_v, n);
    for i in 0..n {
        let j = (i + 1) % n;
        let edges = [bottom_e[i], vertical_e[j], top_e[i], vertical_e[i]];
        let starts = [bottom_v[i], bottom_v[j], top_v[j], top_v[i]];
        make_face(&format!("side{i}"), surfaces[i].clone(), &edges, &starts, 4);
    }

    let (faces, edges, vertices) = counts(&b);
    let mut min_uses = usize::MAX;
    let mut max_uses = 0;
    for e in p1::edges(&b, None).unwrap() {
        let c = solidcore::topology::edge::edgeuses(&b, e, None, None)
            .unwrap()
            .len();
        min_uses = min_uses.min(c);
        max_uses = max_uses.max(c);
    }
    let shells = p1::shells(&mut k, &b, None).map(|v| v.len()).unwrap_or(0);
    let regions = p1::regions(&b, None).map(|v| v.len()).unwrap_or(0);
    println!("DIRECT_AOIM_PRISM faces={faces} edges={edges} vertices={vertices} edgeuses={min_uses}..{max_uses} shells={shells} regions={regions} area={:.6}", built.expected.area());
}
