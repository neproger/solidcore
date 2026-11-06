#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! Exact Sweep: analytic cylinder/torus side surface through the Direct B-rep Builder.

use solidcore::curve::curve::Curve;
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::kernel::Kernel;
use solidcore::mass::integrator::properties::brep_volume_default;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::primitive::api_curves;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;
use solidcore::topology::brep::is_outward_solid;
use solidcore::topology::brep::p2::is_manifold_solid;
use solidcore::topology::brep::validation::{CheckLevel, check_brep};
use solidcore::topology::integrity;

use universal_miter_prototype::boundary::{
    BoundaryInput, BoundarySolver, BoundaryStrategyKind, PathFrame,
};
use universal_miter_prototype::direct_brep::{
    DirectShell, build_direct_chain_surfaces, build_direct_single_surface,
};
use universal_miter_prototype::exact_geometry::{
    arc_sweep_surface, arc_sweep_surface_with_profile, line_sweep_surface,
};
use universal_miter_prototype::planar_sweep2::{normalize_planar_sweep, to_bspline};

fn line(start: Point3, end: Point3) -> BSplineCurve {
    api_curves::create_line_segment(start, end)
        .expect("line")
        .base_with_nurb()
        .expect("line nurb")
        .into_owned()
}

fn arc(center: Point3, radius: f64, start: f64, end: f64) -> BSplineCurve {
    api_curves::create_arc(center, radius, start, end).expect("arc")
}

fn endpoints(path: &BSplineCurve) -> (Point3, Vec3, Point3, Vec3) {
    let start_values = path
        .evaluate(path.natural_interval().min(), 1, true, false)
        .expect("path start");
    let end_values = path
        .evaluate(path.natural_interval().max(), 1, true, false)
        .expect("path end");
    (
        *start_values.first().expect("start point"),
        *start_values.get(1).expect("start tangent"),
        *end_values.first().expect("end point"),
        *end_values.get(1).expect("end tangent"),
    )
}

fn closed_profile(path: &BSplineCurve, radius: f64) -> BSplineCurve {
    let values = path
        .evaluate(path.natural_interval().min(), 1, true, false)
        .expect("profile path");
    let start = *values.first().expect("profile start");
    let tangent = values
        .get(1)
        .copied()
        .expect("profile tangent")
        .unitize()
        .expect("unit profile tangent");
    let normal = Vec3::Z.cross(tangent).unitize().expect("profile normal");
    let mut profile = api_curves::create_circle(Point3::ZERO, radius).expect("circle");
    let placement = Axis2Placement::new(start, normal, Vec3::Z);
    profile
        .transform(&placement, None)
        .expect("profile placement");
    profile
}

fn check_shell(k: &mut Kernel, shell: &mut DirectShell, expected: f64) {
    integrity::check(&shell.brep).expect("integrity");
    assert_eq!(shell.brep.faces().len(), shell.segments + 2);
    assert_eq!(shell.brep.edges().len(), 3 * shell.segments);
    assert_eq!(shell.brep.vertices().len(), 2 * shell.segments);
    let manifold = is_manifold_solid(&shell.brep).expect("manifold");
    let outward = is_outward_solid(k, &mut shell.brep).unwrap_or(false);
    let check = check_brep(k, &mut shell.brep, CheckLevel::Level2).expect("check_brep");
    println!(
        "faces={} edges={} vertices={} manifold={manifold} outward={outward} valid={} failures={:?}",
        shell.brep.faces().len(),
        shell.brep.edges().len(),
        shell.brep.vertices().len(),
        check.is_valid(),
        check.lines()
    );
    assert!(check.is_valid(), "check_brep failures: {:?}", check.lines());
    assert!(manifold);
    assert!(outward);
    for (edge_id, edge) in shell.brep.edges().iter() {
        let curve = edge.curve.as_ref().expect("edge curve");
        let interval = curve.natural_interval();
        let a = curve.evaluate_point(k, interval.min()).expect("edge start");
        let b = curve.evaluate_point(k, interval.max()).expect("edge end");
        let va = shell
            .brep
            .vertex(edge.vertices[0])
            .expect("start vertex")
            .point;
        let vb = shell
            .brep
            .vertex(edge.vertices[1])
            .expect("end vertex")
            .point;
        let error = (a - va).length().min((a - vb).length());
        let error = error.max((b - va).length().min((b - vb).length()));
        assert!(
            error <= 1.0e-9 * (1.0 + a.max_dimension() + b.max_dimension()),
            "edge {edge_id:?} endpoint does not match its vertices: {error}"
        );
    }
    let volume = brep_volume_default(k, &mut shell.brep, 1.0e-3).expect("volume");
    let relative = (volume - expected).abs() / expected.max(1.0e-30);
    assert!(
        relative <= 1.0e-5,
        "volume {volume} vs expected {expected} (relative {relative})"
    );
}

#[test]
fn exact_line_sweep_builds_analytic_cylinder_brep() {
    let mut k = Kernel::default();
    let path = line(Point3::ZERO, Point3::new(2.0, 0.0, 0.0));
    let profile = closed_profile(&path, 0.25);
    let (start, start_tangent, end, end_tangent) = endpoints(&path);
    let surface =
        line_sweep_surface(&k, &Curve::from(profile), start, end).expect("exact cylinder");
    let mut shell =
        build_direct_single_surface(&mut k, &surface, start, start_tangent, end, end_tangent, 4)
            .expect("direct shell");
    let expected = std::f64::consts::PI * 0.25 * 0.25 * 2.0;
    check_shell(&mut k, &mut shell, expected);
}

fn check_chain(
    k: &mut Kernel,
    brep: &mut solidcore::Brep,
    faces: usize,
    edges: usize,
    vertices: usize,
    expected: f64,
) {
    integrity::check(brep).expect("integrity");
    let check = check_brep(k, brep, CheckLevel::Level2).expect("check_brep");
    assert!(check.is_valid(), "check_brep failures: {:?}", check.lines());
    assert_eq!(brep.faces().len(), faces, "face count");
    assert_eq!(brep.edges().len(), edges, "edge count");
    assert_eq!(brep.vertices().len(), vertices, "vertex count");
    assert!(is_manifold_solid(brep).expect("manifold"));
    assert!(is_outward_solid(k, brep).unwrap_or(false));
    let volume = brep_volume_default(k, brep, 1.0e-5).expect("volume");
    let relative = (volume - expected).abs() / expected.max(1.0e-30);
    println!(
        "chain: faces={faces} edges={edges} vertices={vertices} volume={volume:.9e} \
         expected={expected:.9e} relative={relative:.3e}"
    );
    assert!(
        relative <= 1.0e-5,
        "volume {volume} vs expected {expected} (relative {relative})"
    );
}

#[test]
fn exact_chain_line_line_line() {
    let path0 = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let path1 = line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    let path2 = line(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0));
    let path_bs = [path0, path1, path2];
    let paths = [
        Curve::from(path_bs[0].clone()),
        Curve::from(path_bs[1].clone()),
        Curve::from(path_bs[2].clone()),
    ];
    let mut k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let normalized = normalize_planar_sweep(
        &k,
        &[Curve::from(
            api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"),
        )],
        &paths,
        true,
        3,
    )
    .expect("normalize");
    let boundary_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("profile");

    let mut surfaces = Vec::with_capacity(3);
    for placement in &normalized.placements {
        let profile = Curve::from(to_bspline(&placement.world_profiles[0]).expect("profile"));
        let (start, _, end, _) = endpoints(&placement.path_piece);
        surfaces.push(line_sweep_surface(&k, &profile, start, end).expect("exact line"));
    }

    let mut boundaries = Vec::with_capacity(2);
    for joint in 0..2 {
        let boundary = BoundarySolver::default()
            .solve(&BoundaryInput {
                kernel: &k,
                frame,
                profile: &boundary_profile,
                path_a: &path_bs[joint],
                path_b: &path_bs[joint + 1],
                extension: normalized.extension,
                segments: 4,
                samples_per_segment: 17,
            })
            .expect("boundary");
        assert_eq!(
            boundary.diagnostics.strategy,
            BoundaryStrategyKind::ExactMiter
        );
        boundaries.push(boundary);
    }

    let (start, start_tangent, _, _) = endpoints(&path_bs[0]);
    let (_, _, end, end_tangent) = endpoints(&path_bs[2]);
    let mut brep = build_direct_chain_surfaces(
        &mut k,
        &surfaces,
        &boundaries,
        start,
        start_tangent,
        end,
        end_tangent,
        4,
    )
    .expect("chain");
    let expected = std::f64::consts::PI * 0.25 * 0.25 * 3.0;
    check_chain(&mut k, &mut brep, 14, 28, 16, expected);
}

#[test]
fn exact_chain_line_arc_line() {
    let path0 = line(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0));
    let path1 = arc(Point3::new(5.0, 4.0, 0.0), 4.0, -90.0, 60.0);
    let arc_end = *path1
        .evaluate(path1.natural_interval().max(), 0, true, false)
        .expect("arc end")
        .first()
        .expect("arc end point");
    let tangent = path1
        .evaluate(path1.natural_interval().max(), 1, true, false)
        .expect("arc end tangent")
        .get(1)
        .copied()
        .expect("arc tangent")
        .unitize()
        .expect("unit tangent");
    let path2 = line(arc_end, arc_end + tangent * 2.0);
    let path_bs = [path0, path1, path2];
    let paths = [
        Curve::from(path_bs[0].clone()),
        Curve::from(path_bs[1].clone()),
        Curve::from(path_bs[2].clone()),
    ];
    let mut k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let normalized = normalize_planar_sweep(
        &k,
        &[Curve::from(
            api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"),
        )],
        &paths,
        true,
        3,
    )
    .expect("normalize");
    let boundary_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("profile");

    let mut surfaces = Vec::with_capacity(3);
    for (index, placement) in normalized.placements.iter().enumerate() {
        let (start, _, end, _) = endpoints(&placement.path_piece);
        if index == 1 {
            let profile =
                Curve::from(to_bspline(&placement.world_profiles[0]).expect("arc profile"));
            surfaces.push(
                arc_sweep_surface_with_profile(
                    &k,
                    Some(&profile),
                    Point3::new(5.0, 4.0, 0.0),
                    Vec3::new(0.0, -1.0, 0.0),
                    Vec3::X,
                    4.0,
                    0.25,
                    0.0,
                    150.0,
                )
                .expect("exact arc"),
            );
        } else {
            let profile = Curve::from(to_bspline(&placement.world_profiles[0]).expect("profile"));
            surfaces.push(line_sweep_surface(&k, &profile, start, end).expect("exact line"));
        }
    }

    let mut boundaries = Vec::with_capacity(2);
    for joint in 0..2 {
        boundaries.push(
            BoundarySolver::default()
                .solve(&BoundaryInput {
                    kernel: &k,
                    frame,
                    profile: &boundary_profile,
                    path_a: &path_bs[joint],
                    path_b: &path_bs[joint + 1],
                    extension: normalized.extension,
                    segments: 4,
                    samples_per_segment: 17,
                })
                .expect("boundary"),
        );
    }

    for (joint, boundary) in boundaries.iter().enumerate() {
        for (side, surface) in [("left", &surfaces[joint]), ("right", &surfaces[joint + 1])] {
            let domain = surface.natural_uv_domain_with_nurb(&k).expect("domain");
            let mut gap = 0.0_f64;
            for segment in &boundary.seam_segments {
                let interval = segment.natural_interval();
                for index in 0..=16 {
                    let point = segment
                        .evaluate_point(&k, interval.evaluate(f64::from(index) / 16.0))
                        .expect("seam point");
                    let drop = surface
                        .drop_point::<NoObject>(&k, point, &domain, &DropPointOptions::default())
                        .expect("seam drop");
                    gap = gap.max(drop.gap);
                }
            }
            println!("joint {joint} {side}: seam gap {gap:e}");
        }
    }

    let (start, start_tangent, _, _) = endpoints(&path_bs[0]);
    let (_, _, end, end_tangent) = endpoints(&path_bs[2]);
    let mut brep = build_direct_chain_surfaces(
        &mut k,
        &surfaces,
        &boundaries,
        start,
        start_tangent,
        end,
        end_tangent,
        4,
    )
    .expect("chain");
    let length = 5.0 + 4.0 * 5.0 * std::f64::consts::PI / 6.0 + 2.0;
    let expected = std::f64::consts::PI * 0.25 * 0.25 * length;
    check_chain(&mut k, &mut brep, 14, 28, 16, expected);
}

#[test]
fn exact_arc_sweep_builds_analytic_torus_brep() {
    let mut k = Kernel::default();
    let radius = 2.0;
    let profile_radius = 0.25;
    let start_deg = 0.0;
    let end_deg = 90.0;
    let path = arc(Point3::ZERO, radius, start_deg, end_deg);
    let (start, start_tangent, end, end_tangent) = endpoints(&path);
    let surface = arc_sweep_surface(
        &k,
        Point3::ZERO,
        Vec3::X,
        Vec3::Y,
        radius,
        profile_radius,
        start_deg,
        end_deg,
    )
    .expect("exact torus");
    let mut shell =
        build_direct_single_surface(&mut k, &surface, start, start_tangent, end, end_tangent, 4)
            .expect("direct shell");
    let length = radius * (end_deg - start_deg).to_radians();
    let expected = std::f64::consts::PI * profile_radius * profile_radius * length;
    check_shell(&mut k, &mut shell, expected);
}
