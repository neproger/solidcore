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
    probe_direct_aoim_prism("90-baseline", 90.0, 0.0, 2.0, 20.0, false);
    probe_direct_aoim_prism("90-baseline-sides-first", 90.0, 0.0, 2.0, 20.0, true);
    probe_direct_aoim_prism("90-symmetric", 90.0, -2.0, 2.0, 20.0, false);
    probe_direct_aoim_prism("90-symmetric-sides-first", 90.0, -2.0, 2.0, 20.0, true);
    probe_direct_aoim_prism("30-symmetric", 30.0, -2.0, 2.0, 20.0, false);
    probe_direct_aoim_prism("30-symmetric-sides-first", 30.0, -2.0, 2.0, 20.0, true);
    probe_direct_aoim_prism("150-symmetric", 150.0, -2.0, 2.0, 20.0, false);
    probe_direct_aoim_prism("150-symmetric-sides-first", 150.0, -2.0, 2.0, 20.0, true);
    probe_direct_aoim_prism("179-symmetric", 179.0, -2.0, 2.0, 500.0, false);
    probe_direct_aoim_prism("179-symmetric-sides-first", 179.0, -2.0, 2.0, 500.0, true);
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
    for s in p1::shells(&mut k, &b, None).unwrap() {
        for fu in b.shell(s).unwrap().get_faceuses().unwrap() {
            let rec = b.faceuse(fu).unwrap();
            println!(
                "AOIM_FU shell={s:?} face={:?} orient={:?}",
                rec.face.map(|f| f.index()),
                rec.orientation
            );
        }
    }
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

fn probe_direct_aoim_prism(
    label: &str,
    angle_deg: f64,
    d0: f64,
    d1: f64,
    line_length: f64,
    side_first: bool,
) {
    use aoim_line_arc_research::sweep::{build_regions, seam_point, Strategy};
    use aoim_line_arc_research::{LineArc, Point2};
    use solidcore::surface::plane::Plane;
    use solidcore::topology::brep::p3 as brep_p3;
    use solidcore::topology::brep::p6;
    use solidcore::topology::brep::p9;

    #[derive(Clone, Copy)]
    enum Geom {
        Line,
        Arc { c: Point2, r: f64, a0: f64, a1: f64 },
    }

    #[derive(Clone, Copy)]
    struct Seg {
        a: usize,
        b: usize,
        geom: Geom,
    }

    fn add_point(points: &mut Vec<Point2>, p: Point2) -> usize {
        if let Some(i) = points
            .iter()
            .position(|q| (q.x - p.x).abs() <= 1e-12 && (q.y - p.y).abs() <= 1e-12)
        {
            i
        } else {
            points.push(p);
            points.len() - 1
        }
    }

    let model = LineArc::new(4.0, angle_deg.to_radians()).expect("model");
    let z0 = -1.0;
    let z1 = 1.0;
    let built = match build_regions(
        &model,
        Strategy::HybridAtJunction,
        d0,
        d1,
        line_length,
        401,
        96,
    ) {
        Ok(built) => built,
        Err(e) => {
            println!("AOIM_BUILD_ERR label={label} err={e:?}");
            return;
        }
    };
    let center = model.center();
    let arc_start = model.arc_start_angle();
    let arc_end = model.arc_end_angle();
    let seam = |d: f64| seam_point(&model, Strategy::HybridAtJunction, d).expect("seam");
    let radius = |d: f64| model.radius + d;
    let arc_start_for = |d: f64| {
        if d < 0.0 {
            arc_start
        } else {
            arc_start + model.arc_parameter(seam(d))
        }
    };
    let arc_end_point = |d: f64| Point2 {
        x: center.x + radius(d) * arc_end.cos(),
        y: center.y + radius(d) * arc_end.sin(),
    };

    let mut points = Vec::new();
    let mut segments = Vec::new();
    let p0 = add_point(
        &mut points,
        Point2 {
            x: -line_length,
            y: -d1,
        },
    );
    let s1 = add_point(&mut points, seam(d1));
    segments.push(Seg {
        a: p0,
        b: s1,
        geom: Geom::Line,
    });

    let outer_end = if d1 < 0.0 {
        let joint = add_point(&mut points, model.joint_arc_point(d1).expect("joint d1"));
        segments.push(Seg {
            a: s1,
            b: joint,
            geom: Geom::Line,
        });
        let end = add_point(&mut points, arc_end_point(d1));
        segments.push(Seg {
            a: joint,
            b: end,
            geom: Geom::Arc {
                c: center,
                r: radius(d1),
                a0: arc_start,
                a1: arc_end,
            },
        });
        end
    } else {
        let end = add_point(&mut points, arc_end_point(d1));
        segments.push(Seg {
            a: s1,
            b: end,
            geom: Geom::Arc {
                c: center,
                r: radius(d1),
                a0: arc_start_for(d1),
                a1: arc_end,
            },
        });
        end
    };

    let inner_end = add_point(&mut points, arc_end_point(d0));
    segments.push(Seg {
        a: outer_end,
        b: inner_end,
        geom: Geom::Line,
    });

    let s0 = if d0 < 0.0 {
        let joint = add_point(&mut points, model.joint_arc_point(d0).expect("joint d0"));
        segments.push(Seg {
            a: inner_end,
            b: joint,
            geom: Geom::Arc {
                c: center,
                r: radius(d0),
                a0: arc_start,
                a1: arc_end,
            },
        });
        let seam0 = add_point(&mut points, seam(d0));
        segments.push(Seg {
            a: joint,
            b: seam0,
            geom: Geom::Line,
        });
        seam0
    } else {
        let seam0 = add_point(&mut points, seam(d0));
        segments.push(Seg {
            a: inner_end,
            b: seam0,
            geom: Geom::Arc {
                c: center,
                r: radius(d0),
                a0: arc_start_for(d0),
                a1: arc_end,
            },
        });
        seam0
    };

    let p_last = add_point(
        &mut points,
        Point2 {
            x: -line_length,
            y: -d0,
        },
    );
    segments.push(Seg {
        a: s0,
        b: p_last,
        geom: Geom::Line,
    });
    segments.push(Seg {
        a: p_last,
        b: p0,
        geom: Geom::Line,
    });
    let n = points.len();
    assert_eq!(n, segments.len(), "contour is not a closed indexed chain");
    println!(
        "AOIM_CASE label={label} angle={angle_deg} d0={d0} d1={d1} contour={n} area={:.6}",
        built.expected.area()
    );

    let mut k = Kernel::default();
    let (mut b, region) = new_brep(&mut k);
    let mut state = p2::ArenaNotifyState::new(&b);
    let mut env = state.env(None);
    let mut marks = Marks::default();

    let mut bottom_v = Vec::with_capacity(n);
    let mut top_v = Vec::with_capacity(n);
    for p in &points {
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
    let arc_curve =
        |k: &Kernel, p: Point2, q: Point2, c: Point2, r: f64, a0: f64, a1: f64, z: f64| {
            let at = |t: f64| Point3::new(c.x + r * t.cos(), c.y + r * t.sin(), z);
            let p3 = Point3::new(p.x, p.y, z);
            let q3 = Point3::new(q.x, q.y, z);
            let mid = at(0.5 * (a0 + a1));
            Curve::from(Ellipse::circle_from_3_points(k, p3, mid, q3, 3, false).expect("arc"))
        };

    let mut bottom_e = Vec::with_capacity(n);
    let mut top_e = Vec::with_capacity(n);
    for (i, seg) in segments.iter().enumerate() {
        let (cb, ct) = match seg.geom {
            Geom::Line => (
                line(
                    Point3::new(points[seg.a].x, points[seg.a].y, z0),
                    Point3::new(points[seg.b].x, points[seg.b].y, z0),
                ),
                line(
                    Point3::new(points[seg.a].x, points[seg.a].y, z1),
                    Point3::new(points[seg.b].x, points[seg.b].y, z1),
                ),
            ),
            Geom::Arc { c, r, a0, a1 } => (
                arc_curve(&k, points[seg.a], points[seg.b], c, r, a0, a1, z0),
                arc_curve(&k, points[seg.a], points[seg.b], c, r, a0, a1, z1),
            ),
        };
        let _ = i;
        bottom_e.push(make_edge(
            &mut k,
            &mut b,
            &mut env,
            bottom_v[seg.a],
            bottom_v[seg.b],
            cb,
        ));
        top_e.push(make_edge(
            &mut k,
            &mut b,
            &mut env,
            top_v[seg.a],
            top_v[seg.b],
            ct,
        ));
    }
    let mut vertical_e = Vec::with_capacity(n);
    for i in 0..n {
        let curve = line(
            Point3::new(points[i].x, points[i].y, z0),
            Point3::new(points[i].x, points[i].y, z1),
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
    for seg in &segments {
        let surface = match seg.geom {
            Geom::Line => {
                let a = points[seg.a];
                let c = points[seg.b];
                let dx = c.x - a.x;
                let dy = c.y - a.y;
                plane(a, z0, Vec3::new(dy, -dx, 0.0))
            }
            Geom::Arc { c, r, a0, a1 } => {
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
            Ok(face) => println!("AOIM_FACE {label}:{name} face={}", face.face.index()),
            Err(e) => println!("AOIM_FACE {label}:{name} err={e:?}"),
        }
    };

    let order = if side_first {
        "sides-first"
    } else {
        "caps-first"
    };
    let bottom_loop_edges: Vec<EdgeId> = (0..n).rev().map(|i| bottom_e[i]).collect();
    let bottom_loop_starts: Vec<solidcore::VertexId> =
        (0..n).rev().map(|i| bottom_v[segments[i].b]).collect();
    if side_first {
        for i in 0..n {
            let j = segments[i].b;
            let a = segments[i].a;
            let edges = [bottom_e[i], vertical_e[j], top_e[i], vertical_e[a]];
            let starts = [bottom_v[a], bottom_v[j], top_v[j], top_v[a]];
            make_face(&format!("side{i}"), surfaces[i].clone(), &edges, &starts, 4);
        }
        make_face(
            "bottom",
            bottom_surface,
            &bottom_loop_edges,
            &bottom_loop_starts,
            n,
        );
        make_face("top", top_surface, &top_e, &top_v, n);
    } else {
        make_face(
            "bottom",
            bottom_surface,
            &bottom_loop_edges,
            &bottom_loop_starts,
            n,
        );
        make_face("top", top_surface, &top_e, &top_v, n);
        for i in 0..n {
            let j = segments[i].b;
            let a = segments[i].a;
            let edges = [bottom_e[i], vertical_e[j], top_e[i], vertical_e[a]];
            let starts = [bottom_v[a], bottom_v[j], top_v[j], top_v[a]];
            make_face(&format!("side{i}"), surfaces[i].clone(), &edges, &starts, 4);
        }
    }

    let (faces, edges, vertices) = counts(&b);
    let inf = b.infinite_region();
    for r in p1::regions(&b, None).unwrap() {
        let rec = b.region(r).unwrap();
        println!(
            "AOIM_REGION {label}:{r:?} void={} shells={}",
            rec.is_void,
            rec.shells.len()
        );
    }
    for s in p1::shells(&mut k, &b, None).unwrap() {
        let rec = b.shell(s).unwrap();
        let faceuses = rec.get_faceuses().unwrap_or_default();
        let same = faceuses
            .iter()
            .filter(|&&fu| {
                b.faceuse(fu).unwrap().orientation
                    == solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same
            })
            .count();
        let opposite = faceuses.len() - same;
        println!(
            "AOIM_SHELL {label}:{s:?} region={:?} faces={} faceuses={} same={same} opposite={opposite}",
            rec.region,
            rec.faces.len(),
            faceuses.len()
        );
    }
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
    println!(
        "DIRECT_AOIM_PRISM label={label} order={order} faces={faces} edges={edges} vertices={vertices} edgeuses={min_uses}..{max_uses} shells={shells} regions={regions} area={:.6}",
        built.expected.area()
    );
    println!("AOIM_INFINITE {label}:before={:?}", inf);
    if let Some(inf) = inf {
        match solidcore::topology::brep::p1::reset_infinite_region(
            &mut k,
            &mut b,
            Some(inf),
            solidcore::topology::brep::p1::SetRegionVoidType::InfiniteRegionOnly,
        ) {
            Ok(()) => println!("AOIM_RESET_INFINITE {label}:ok old_infinite={inf:?}"),
            Err(e) => println!("AOIM_RESET_INFINITE {label}:err={e:?}"),
        }
    }
    println!("AOIM_INFINITE {label}:after={:?}", b.infinite_region());
    for r in p1::regions(&b, None).unwrap() {
        let rec = b.region(r).unwrap();
        println!(
            "AOIM_REGION_AFTER {label}:{r:?} void={} shells={}",
            rec.is_void,
            rec.shells.len()
        );
    }
    use solidcore::topology::edgeuse::p1 as eu1;
    let mut min_same = usize::MAX;
    let mut min_opp = usize::MAX;
    let mut max_same = 0;
    let mut max_opp = 0;
    for e in p1::edges(&b, None).unwrap() {
        let mut same = 0;
        let mut opp = 0;
        for eu in solidcore::topology::edge::edgeuses(&b, e, None, None).unwrap() {
            match eu1::orientation(&b, eu).unwrap() {
                solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Same => same += 1,
                solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Opposite => opp += 1,
                solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType::Unknown => {}
            }
        }
        min_same = min_same.min(same);
        max_same = max_same.max(same);
        min_opp = min_opp.min(opp);
        max_opp = max_opp.max(opp);
    }
    println!("AOIM_EDGE_ORIENT {label}:same={min_same}..{max_same} opposite={min_opp}..{max_opp}");
    use solidcore::geom::attribute::AttributeStore;
    use solidcore::mass::integrator::properties::{brep_volume, PropertyAttributes};
    let store = AttributeStore::default();
    let key = |_e: solidcore::EntityRef| ObjectKey(0);
    let attributes = PropertyAttributes {
        store: &store,
        key_of: &key,
    };
    match brep_volume(&k, &b, 1e-6, &attributes) {
        Ok(volume) => println!(
            "AOIM_VOLUME {label}:{volume:.9} expected={:.9}",
            built.expected.area() * (z1 - z0)
        ),
        Err(e) => println!("AOIM_VOLUME {label}:err={e:?}"),
    }
    use std::collections::BTreeMap;
    let mut face_shells: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for s in p1::shells(&mut k, &b, None).unwrap() {
        for fu in b.shell(s).unwrap().get_faceuses().unwrap() {
            let face = b.faceuse(fu).unwrap().face.unwrap();
            face_shells
                .entry(face.index())
                .or_default()
                .push(u32::try_from(s.index()).unwrap());
        }
    }
    let each_face_two_shells = face_shells
        .values()
        .all(|shells| shells.len() == 2 && shells[0] != shells[1]);
    println!(
        "AOIM_FACE_SHELLS {label}:faces={} each_face_two_distinct_shells={each_face_two_shells}",
        face_shells.len()
    );
    let orient = solidcore::topology::brep::p3::orient_trimmed_surfaces(
        &mut k, &mut b, &mut env, &mut marks, true, false, None,
    );
    match orient {
        Ok(result) => println!("AOIM_ORIENT {label}:ok {result:?}"),
        Err(e) => println!("AOIM_ORIENT {label}:err={e:?}"),
    }
    assert_eq!(faces, 2 + n);
    assert_eq!(edges, 3 * n);
    assert_eq!(vertices, 2 * n);
    assert_eq!((shells, regions), (2, 2));
    assert!(each_face_two_shells);
    assert_eq!((min_same, max_same, min_opp, max_opp), (2, 2, 2, 2));
    let old_inf = inf.expect("the pre-split infinite region");
    let solid = p1::regions(&b, None)
        .unwrap()
        .into_iter()
        .find(|r| *r != old_inf)
        .expect("solid region");
    assert_eq!(b.region(old_inf).unwrap().is_void, 1);
    assert_eq!(b.region(solid).unwrap().is_void, 0);
    println!("AOIM_PROBE_PASS label={label} order={order}");
}
