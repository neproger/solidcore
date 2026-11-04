#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::many_single_char_names
)]

//! Gate 0: preprocessing parity between the existing Planar Sweep and the
//! alternative normalizer. No swept surface, joint or B-rep of the alternative is
//! built here.

use solidcore::curve::curve::{Curve, CurveKind};
use solidcore::curve::hermite_curve::HermiteCurve;
use solidcore::geom::tol::ZONE_TOL_3D;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::primitive::api_curves;
use solidcore::primitive::primitive_creation::p1::position_profile_curves;
use solidcore::surface::surface::SurfaceKind;
use solidcore::sweep::api_brep::create_sweep_along_planar_path;
use solidcore::topology::face::p4::vertices as face_vertices;
use solidcore::topology::face::surface as face_surface;
use solidcore::topology::vertex::point as vertex_point;
use solidcore::topology::Brep;
use universal_miter_prototype::planar_sweep2::{normalize_planar_sweep, NormalizedPlanarSweep};

fn kernel() -> Kernel {
    let mut k = Kernel::default();
    k.fixes.sweep_frame_on_planar_path_normal = true;
    k.fixes.planar_path_sweep_statuses = true;
    k.fixes.planar_path_profile_placement_statuses = true;
    k.fixes.planar_path_profile_in_path_frame = true;
    k.fixes.planar_path_uncapped_open_profiles = true;
    k
}

fn line(start: Point3, end: Point3) -> Curve {
    Curve::from(api_curves::create_line_segment(start, end).expect("line"))
}

fn rectangle() -> Vec<Curve> {
    let a = Point3::new(0.0, 0.0, 0.0);
    let b = Point3::new(1.0, 0.0, 0.0);
    let c = Point3::new(1.0, 0.5, 0.0);
    let d = Point3::new(0.0, 0.5, 0.0);
    vec![line(a, b), line(b, c), line(c, d), line(d, a)]
}

fn circle() -> Curve {
    Curve::from(api_curves::create_circle(Point3::new(0.0, 0.0, 0.0), 1.0).expect("circle"))
}

fn hermite() -> Curve {
    let origin = Point3::new(0.0, 0.0, 0.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    Curve::from(HermiteCurve::new(
        origin,
        x,
        Point3::new(1.0, 1.0, 0.0),
        x,
        3,
    ))
}

fn tangent_at(k: &Kernel, path: &Curve, at_start: bool) -> Vec3 {
    let t = if at_start {
        path.natural_interval().min()
    } else {
        path.natural_interval().max()
    };
    path.evaluate(k, t, 1, true, true).expect("path eval")[1].unitize_or_z()
}

fn corners(k: &Kernel, curves: &[Curve]) -> Vec<Point3> {
    let mut out = Vec::new();
    for curve in curves {
        let domain = curve.natural_interval();
        out.push(curve.evaluate_point(k, domain.min()).expect("corner"));
        out.push(curve.evaluate_point(k, domain.max()).expect("corner"));
    }
    out
}

fn nearest_max(left: &[Point3], right: &[Point3]) -> f64 {
    left.iter()
        .map(|a| {
            right
                .iter()
                .map(|b| a.distance(*b))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max)
}

fn cap_faces(b: &Brep, tangent: Vec3) -> Vec<solidcore::topology::FaceId> {
    let tangent = tangent.unitize_or_z();
    let mut out = Vec::new();
    for face in b.face_list() {
        let surface = face_surface(b, *face).expect("face surface");
        let Some(surface) = surface else {
            continue;
        };
        if let SurfaceKind::Plane(plane) = surface.kind() {
            if plane.position().z_axis().dot(tangent).abs() > 1.0 - 1.0e-8 {
                out.push(*face);
            }
        }
    }
    out
}

fn face_points(b: &Brep, face: solidcore::topology::FaceId) -> Vec<Point3> {
    match face_vertices(b, face, None, false, None) {
        Ok(vertices) => vertices
            .into_iter()
            .filter_map(|vertex| vertex_point(b, vertex).ok())
            .collect(),
        Err(_) => Vec::new(),
    }
}

fn cap_face_near(b: &Brep, tangent: Vec3, target: Point3) -> Option<solidcore::topology::FaceId> {
    let mut best = None;
    let mut best_distance = f64::INFINITY;
    for face in cap_faces(b, tangent) {
        let points = face_points(b, face);
        if points.is_empty() {
            continue;
        }
        let mut centroid = Vec3::ZERO;
        for point in &points {
            centroid += *point;
        }
        centroid *= 1.0 / points.len() as f64;
        let distance = centroid.distance(target);
        if distance < best_distance {
            best_distance = distance;
            best = Some(face);
        }
    }
    best
}

fn cap_vertices_near(b: &Brep, tangent: Vec3, target: Point3) -> Vec<Point3> {
    cap_face_near(b, tangent, target)
        .map(|face| face_points(b, face))
        .unwrap_or_default()
}

fn run_original(
    k: &mut Kernel,
    profiles: &[Curve],
    paths: &[Curve],
    move_profile: bool,
    cap_ends: i32,
) -> Brep {
    let profile_refs: Vec<&Curve> = profiles.iter().collect();
    let path_refs: Vec<&Curve> = paths.iter().collect();
    create_sweep_along_planar_path(k, &profile_refs, &path_refs, move_profile, cap_ends)
        .expect("original planar sweep")
}

fn placement_corners(k: &Kernel, normalized: &NormalizedPlanarSweep, index: usize) -> Vec<Point3> {
    corners(k, &normalized.placements[index].world_profiles)
}

#[test]
fn rectangle_line_caps_and_profile_placement() {
    let k = kernel();
    let profiles = rectangle();
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let paths = [path.clone()];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 1).expect("normalize");
    assert!(!normalized.closed);
    assert!(!normalized.periodic);
    assert_eq!(normalized.placements.len(), 1);
    assert!(normalized.caps.start);
    assert!(!normalized.caps.end);
    assert!(normalized.caps.profile_closes);

    let world = placement_corners(&k, &normalized, 0);
    let start_point = path
        .evaluate_point(&k, path.natural_interval().min())
        .expect("start");
    let end_point = path
        .evaluate_point(&k, path.natural_interval().max())
        .expect("end");
    let start_tangent = tangent_at(&k, &path, true);
    let mut original_kernel = kernel();
    let original = run_original(&mut original_kernel, &profiles, &paths, true, 1);
    let cap = cap_vertices_near(&original, start_tangent, start_point);
    assert_eq!(cap_faces(&original, start_tangent).len(), 1);
    assert!(!cap.is_empty());
    let difference = nearest_max(&cap, &world);
    assert!(
        difference <= 1.0e-6,
        "start cap vs normalized profile: {difference}"
    );

    // No caps.
    let mut no_cap_kernel = kernel();
    let no_cap = run_original(&mut no_cap_kernel, &profiles, &paths, true, 0);
    assert!(cap_faces(&no_cap, start_tangent).is_empty());
    let no_cap_normalized =
        normalize_planar_sweep(&k, &profiles, &paths, true, 0).expect("normalize");
    assert!(!no_cap_normalized.caps.start);
    assert!(!no_cap_normalized.caps.end);

    // End cap: the straight sweep translates the start profile by the path vector.
    let end_tangent = tangent_at(&k, &path, false);
    let mut end_kernel = kernel();
    let end_body = run_original(&mut end_kernel, &profiles, &paths, true, 2);
    let end_cap = cap_vertices_near(&end_body, end_tangent, end_point);
    let offset = end_point - start_point;
    let expected_end: Vec<Point3> = world.iter().map(|point| *point + offset).collect();
    let difference = nearest_max(&end_cap, &expected_end);
    assert!(
        difference <= 1.0e-6,
        "end cap vs translated profile: {difference}"
    );
    assert_eq!(cap_faces(&end_body, end_tangent).len(), 1);

    // move_profile = false: the whole-path placement still feeds the cap.
    let normalized_first = normalize_planar_sweep(&k, &profiles, &paths, false, 1)
        .expect("normalize first-point profile");
    let world_first = placement_corners(&k, &normalized_first, 0);
    let mut first_kernel = kernel();
    let first_body = run_original(&mut first_kernel, &profiles, &paths, false, 1);
    let first_cap = cap_vertices_near(&first_body, start_tangent, start_point);
    let difference = nearest_max(&first_cap, &world_first);
    assert!(
        difference <= 1.0e-6,
        "move_profile=false cap vs normalized profile: {difference}"
    );
}

#[test]
fn line_line_segmentation_extension_and_caps() {
    let k = kernel();
    let profiles = rectangle();
    let paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        line(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)),
    ];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 3).expect("normalize");
    assert_eq!(normalized.path_original.len(), 2);
    assert_eq!(normalized.path_bspline.len(), 2);
    assert_eq!(normalized.placements.len(), 2);
    assert!(normalized.extension > 0.0);
    assert!(!normalized.closed);
    assert!(!normalized.periodic);
    assert!(normalized.caps.start);
    assert!(normalized.caps.end);

    let first_start = normalized.path_bspline[0]
        .evaluate_point(normalized.path_bspline[0].natural_interval().min())
        .expect("first start");
    let corner = Point3::new(2.0, 0.0, 0.0);
    let first_end = normalized.path_bspline[0]
        .evaluate_point(normalized.path_bspline[0].natural_interval().max())
        .expect("first end");
    assert!(
        first_start.distance(Point3::new(0.0, 0.0, 0.0)) < 1.0e-9,
        "the first piece should keep its original start"
    );
    assert!(
        first_end.distance(corner) > 0.5 * normalized.extension,
        "the first piece should be extended past the corner"
    );

    let mut original_kernel = kernel();
    let original = run_original(&mut original_kernel, &profiles, &paths, true, 3);
    let start_tangent = tangent_at(&k, &paths[0], true);
    let end_tangent = tangent_at(&k, &paths[1], false);
    let start_point = paths[0]
        .evaluate_point(&k, paths[0].natural_interval().min())
        .expect("start");
    let end_point = paths[1]
        .evaluate_point(&k, paths[1].natural_interval().max())
        .expect("end");
    let world_start = placement_corners(&k, &normalized, 0);
    let start_cap = cap_vertices_near(&original, start_tangent, start_point);
    assert!(nearest_max(&start_cap, &world_start) <= 1.0e-6);

    // The last placement is at the extended start of the second piece; the end
    // cap is that profile translated by the piece's end-start vector.
    let world_last = placement_corners(&k, &normalized, 1);
    let piece = &normalized.placements[1].path_piece;
    let piece_start = piece
        .evaluate_point(piece.natural_interval().min())
        .expect("piece start");
    let piece_end = piece
        .evaluate_point(piece.natural_interval().max())
        .expect("piece end");
    let offset = piece_end - piece_start;
    let expected_end: Vec<Point3> = world_last.iter().map(|point| *point + offset).collect();
    let end_cap = cap_vertices_near(&original, end_tangent, end_point);
    assert!(nearest_max(&end_cap, &expected_end) <= 1.0e-6);
}

#[test]
fn whole_path_placement_differs_from_the_single_segment_wrapper() {
    let k = kernel();
    let profiles = rectangle();
    let paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)),
        line(Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 1.0, 1.0)),
    ];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 1).expect("normalize");
    let world_whole = placement_corners(&k, &normalized, 0);

    let mut single_output = Vec::new();
    position_profile_curves(&k, &profiles, ZONE_TOL_3D, &paths[0], &mut single_output)
        .expect("single-path placement");
    let single_corners = corners(&k, &single_output);

    let start_tangent = tangent_at(&k, &paths[0], true);
    let start_point = paths[0]
        .evaluate_point(&k, paths[0].natural_interval().min())
        .expect("start");
    let mut original_kernel = kernel();
    let original = run_original(&mut original_kernel, &profiles, &paths, true, 1);
    let cap = cap_vertices_near(&original, start_tangent, start_point);
    assert!(!cap.is_empty());

    let whole_difference = nearest_max(&cap, &world_whole);
    let single_difference = nearest_max(&cap, &single_corners);
    assert!(
        whole_difference <= 1.0e-6,
        "whole-path profile vs cap: {whole_difference}"
    );
    assert!(
        single_difference > whole_difference,
        "the single-segment wrapper should not match the whole-path cap: \
         single={single_difference}, whole={whole_difference}"
    );
}

#[test]
fn circle_profile_keeps_rational_weights() {
    let k = kernel();
    let profiles = [circle()];
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let paths = [path.clone()];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 1).expect("normalize");

    let original_base = match profiles[0].kind() {
        CurveKind::BSpline(curve) => curve,
        other => panic!("unexpected circle kind: {other:?}"),
    };
    assert!(original_base.is_rational().expect("rational"));
    let normalized_base = &normalized.profiles_bspline[0];
    assert!(normalized_base.is_rational().expect("normalized rational"));
    let original_weights: Vec<f64> = original_base
        .control_points()
        .expect("circle control points")
        .iter()
        .map(|point| point.w)
        .collect();
    let normalized_weights: Vec<f64> = normalized_base
        .control_points()
        .expect("normalized control points")
        .iter()
        .map(|point| point.w)
        .collect();
    assert_eq!(original_weights, normalized_weights);

    let prepared_base = &normalized.placements[0].profiles_bspline[0];
    assert!(prepared_base.is_rational().expect("prepared rational"));
    let prepared_weights: Vec<f64> = prepared_base
        .control_points()
        .expect("prepared control points")
        .iter()
        .map(|point| point.w)
        .collect();
    assert_eq!(original_weights, prepared_weights);

    // The production cap boundary keeps a rational base as well.
    let start_tangent = tangent_at(&k, &path, true);
    let start_point = path
        .evaluate_point(&k, path.natural_interval().min())
        .expect("start");
    let mut original_kernel = kernel();
    let original = run_original(&mut original_kernel, &profiles, &paths, true, 1);
    let face = cap_face_near(&original, start_tangent, start_point).expect("start cap");
    let edges =
        solidcore::topology::face::edges(&original, face, None, false, None).expect("cap edges");
    assert!(!edges.is_empty());
    let curve = solidcore::topology::edge::curve(&original, edges[0])
        .expect("edge curve")
        .expect("edge has a curve");
    match curve.kind() {
        CurveKind::BSpline(base) => {
            assert!(base.is_rational().expect("cap rational"));
        }
        CurveKind::Ellipse(ellipse) => {
            assert!(ellipse.base().is_rational().expect("cap ellipse rational"));
        }
        _ => {}
    }
}

#[test]
fn closed_path_has_no_caps() {
    let k = kernel();
    let profiles = rectangle();
    let paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        line(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)),
        line(Point3::new(2.0, 2.0, 0.0), Point3::new(0.0, 2.0, 0.0)),
        line(Point3::new(0.0, 2.0, 0.0), Point3::new(0.0, 0.0, 0.0)),
    ];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 3).expect("normalize");
    assert!(normalized.closed);
    assert!(!normalized.periodic);
    assert_eq!(normalized.placements.len(), 4);
    assert!(!normalized.caps.start);
    assert!(!normalized.caps.end);

    // The closed path ignores cap_ends: the 0 and 3 results are the same B-rep.
    let mut zero_kernel = kernel();
    let zero = run_original(&mut zero_kernel, &profiles, &paths, true, 0);
    let mut three_kernel = kernel();
    let three = run_original(&mut three_kernel, &profiles, &paths, true, 3);
    assert_eq!(zero.face_list().len(), three.face_list().len());
    assert_eq!(zero.edge_list().len(), three.edge_list().len());
    assert_eq!(zero.vertex_list().len(), three.vertex_list().len());
}

#[test]
fn strict_planarity_rejects_a_non_planar_path() {
    let k = kernel();
    let profiles = rectangle();
    let planar_paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)),
        line(Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 1.0, 1.0)),
    ];
    let planar =
        normalize_planar_sweep(&k, &profiles, &planar_paths, true, 0).expect("planar path");
    assert!(planar.plane_normal.is_some());
    assert!(planar.planarity_deviation <= ZONE_TOL_3D);
    assert!(planar.planarity_is_planar);

    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.5),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let curved = Curve::from(api_curves::create_curve(&points).expect("non-planar curve"));
    let non_planar = [curved];
    let result = normalize_planar_sweep(&k, &profiles, &non_planar, true, 0);
    assert!(result.is_err(), "the non-planar path must be refused");
    let planar_result = non_planar[0].is_planar(&k, ZONE_TOL_3D).expect("is_planar");
    assert!(!planar_result.is_planar || planar_result.deviation > ZONE_TOL_3D);
}

#[test]
fn unsupported_curve_kinds_are_rejected() {
    let k = kernel();
    let profiles = rectangle();
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let unsupported_path = hermite();
    assert!(normalize_planar_sweep(&k, &profiles, &[unsupported_path], true, 0).is_err());
    let unsupported_profile = hermite();
    assert!(normalize_planar_sweep(&k, &[unsupported_profile], &[path], true, 0).is_err());
}

#[test]
fn gate0_report() {
    let k = kernel();
    let profiles = rectangle();

    // Rectangle along a straight line: start/end caps and move_profile.
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0));
    let normalized = normalize_planar_sweep(&k, &profiles, std::slice::from_ref(&path), true, 1)
        .expect("rectangle normalize");
    let world = placement_corners(&k, &normalized, 0);
    let start = path
        .evaluate_point(&k, path.natural_interval().min())
        .expect("start");
    let end = path
        .evaluate_point(&k, path.natural_interval().max())
        .expect("end");
    let mut start_kernel = kernel();
    let start_body = run_original(
        &mut start_kernel,
        &profiles,
        std::slice::from_ref(&path),
        true,
        1,
    );
    let start_cap = cap_vertices_near(&start_body, tangent_at(&k, &path, true), start);
    let start_difference = nearest_max(&start_cap, &world);
    let mut end_kernel = kernel();
    let end_body = run_original(
        &mut end_kernel,
        &profiles,
        std::slice::from_ref(&path),
        true,
        2,
    );
    let expected_end: Vec<Point3> = world.iter().map(|point| *point + (end - start)).collect();
    let end_cap = cap_vertices_near(&end_body, tangent_at(&k, &path, false), end);
    let end_difference = nearest_max(&end_cap, &expected_end);
    let normalized_first =
        normalize_planar_sweep(&k, &profiles, std::slice::from_ref(&path), false, 1)
            .expect("first-point normalize");
    let world_first = placement_corners(&k, &normalized_first, 0);
    let mut first_kernel = kernel();
    let first_body = run_original(
        &mut first_kernel,
        &profiles,
        std::slice::from_ref(&path),
        false,
        1,
    );
    let first_cap = cap_vertices_near(&first_body, tangent_at(&k, &path, true), start);
    let first_difference = nearest_max(&first_cap, &world_first);
    println!(
        "case=rectangle-line start={start_difference:.3e} end={end_difference:.3e} first={first_difference:.3e} caps_start={} caps_end={} extension={:.6}",
        normalized.caps.start, normalized.caps.end, normalized.extension
    );

    // Two-line path: segmentation, extension, start/end caps.
    let paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)),
        line(Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 2.0, 0.0)),
    ];
    let normalized = normalize_planar_sweep(&k, &profiles, &paths, true, 3).expect("two-line");
    let world_start = placement_corners(&k, &normalized, 0);
    let world_last = placement_corners(&k, &normalized, 1);
    let piece = &normalized.placements[1].path_piece;
    let piece_start = piece
        .evaluate_point(piece.natural_interval().min())
        .expect("piece start");
    let piece_end = piece
        .evaluate_point(piece.natural_interval().max())
        .expect("piece end");
    let expected_end: Vec<Point3> = world_last
        .iter()
        .map(|point| *point + (piece_end - piece_start))
        .collect();
    let mut original_kernel = kernel();
    let original = run_original(&mut original_kernel, &profiles, &paths, true, 3);
    let start_point = paths[0]
        .evaluate_point(&k, paths[0].natural_interval().min())
        .expect("start");
    let end_point = paths[1]
        .evaluate_point(&k, paths[1].natural_interval().max())
        .expect("end");
    let start_cap = cap_vertices_near(&original, tangent_at(&k, &paths[0], true), start_point);
    let end_cap = cap_vertices_near(&original, tangent_at(&k, &paths[1], false), end_point);
    let start_difference = nearest_max(&start_cap, &world_start);
    let end_difference = nearest_max(&end_cap, &expected_end);
    println!(
        "case=line-line start={start_difference:.3e} end={end_difference:.3e} pieces={} placements={} extension={:.6} closed={} periodic={}",
        normalized.path_bspline.len(),
        normalized.placements.len(),
        normalized.extension,
        normalized.closed,
        normalized.periodic
    );

    // Whole-path WR-660 vs the single-segment wrapper.
    let whole_paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)),
        line(Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 1.0, 1.0)),
    ];
    let whole = normalize_planar_sweep(&k, &profiles, &whole_paths, true, 1).expect("whole");
    let world_whole = placement_corners(&k, &whole, 0);
    let mut single_output = Vec::new();
    position_profile_curves(
        &k,
        &profiles,
        ZONE_TOL_3D,
        &whole_paths[0],
        &mut single_output,
    )
    .expect("wrapper");
    let single_corners = corners(&k, &single_output);
    let mut whole_kernel = kernel();
    let whole_body = run_original(&mut whole_kernel, &profiles, &whole_paths, true, 1);
    let whole_cap = cap_vertices_near(
        &whole_body,
        tangent_at(&k, &whole_paths[0], true),
        Point3::new(0.0, 0.0, 0.0),
    );
    println!(
        "case=whole-path whole={:.3e} single={:.3e}",
        nearest_max(&whole_cap, &world_whole),
        nearest_max(&whole_cap, &single_corners)
    );

    // Rational weights.
    let circle_profiles = [circle()];
    let circle_path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let circle_normalized = normalize_planar_sweep(
        &k,
        &circle_profiles,
        std::slice::from_ref(&circle_path),
        true,
        1,
    )
    .expect("circle");
    let rational = circle_normalized.profiles_bspline[0]
        .is_rational()
        .expect("rational flag");
    let weights = circle_normalized.profiles_bspline[0]
        .control_points()
        .expect("weights")
        .iter()
        .map(|point| point.w)
        .collect::<Vec<_>>();
    println!("case=circle rational={rational} weights={weights:?}");

    // Strict planarity.
    let planar_paths = [
        line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 1.0)),
        line(Point3::new(1.0, 0.0, 1.0), Point3::new(1.0, 1.0, 1.0)),
    ];
    let planar = normalize_planar_sweep(&k, &profiles, &planar_paths, true, 0).expect("planar");
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.5),
        Point3::new(2.0, 1.0, 0.0),
    ];
    let curved = Curve::from(api_curves::create_curve(&points).expect("non-planar"));
    let non_planar_refused = normalize_planar_sweep(&k, &profiles, &[curved], true, 0).is_err();
    let planar_is_planar = planar.planarity_is_planar;
    println!(
        "case=planarity normal={:?} deviation={:.3e} is_planar={} non_planar_refused={non_planar_refused}",
        planar.plane_normal, planar.planarity_deviation, planar_is_planar
    );
}

#[test]
fn normalization_keeps_the_preparation_local() {
    let k = kernel();
    let profiles = rectangle();
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let normalized = normalize_planar_sweep(&k, &profiles, &[path], true, 1).expect("normalize");
    let placement = &normalized.placements[0];
    assert!(!placement.prepared.profiles.is_empty());
    assert_eq!(
        placement.prepared.profiles.len(),
        placement.profiles_placed.len()
    );
    assert_eq!(
        placement.world_profiles.len(),
        placement.prepared.profiles.len()
    );
    assert!((normalized.tolerance - ZONE_TOL_3D).abs() < 1.0e-15);
    // The preparation contains transforms only; no B-rep or surface is built here.
    assert!(
        placement
            .prepared
            .rotation_back
            .origin()
            .distance(Point3::new(0.0, 0.0, 0.0))
            <= 1.0e-9
    );
}
