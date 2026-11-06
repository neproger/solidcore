#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! Audit: does the exact Torus source reproduce the analytic planar sweep?

use solidcore::curve::curve::p2::Curve;
use solidcore::kernel::Kernel;
use solidcore::math::{Point2, Point3, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
use solidcore::primitive::api_curves;

use universal_miter_prototype::boundary::PathFrame;
use universal_miter_prototype::level_b_geometry::SurfaceBuildOptions;
use universal_miter_prototype::planar_sweep2::normalize_planar_sweep;
use universal_miter_prototype::source_selection::select_sweep_sources;

fn arc(center: Point3, radius: f64, start: f64, end: f64) -> Curve {
    Curve::from(api_curves::create_arc(center, radius, start, end).expect("arc"))
}

fn circle(k: &Kernel) -> Curve {
    Curve::from(api_curves::create_analytic_circle(k, Point3::ZERO, 0.25).expect("circle"))
}

fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let c = angle.cos();
    let s = angle.sin();
    v * c + axis * (axis.dot(v) * (1.0 - c)) + axis.cross(v) * s
}

fn evaluated(surface: &solidcore::surface::surface::Surface, k: &Kernel, uv: Point2) -> Point3 {
    surface
        .evaluate(k, uv, 0, 0, EvaluateFlags::default())
        .expect("surface evaluation")[0][0]
}

#[test]
fn torus_source_geometry_audit() {
    let k = Kernel::default();
    let profile = circle(&k);
    let paths = vec![
        arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0),
        arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 45.0),
    ];
    let normalized = normalize_planar_sweep(&k, std::slice::from_ref(&profile), &paths, true, 3)
        .expect("normalize");
    let frame = PathFrame::new(&k, &normalized.path_original).expect("frame");
    let selected =
        select_sweep_sources(&k, &normalized, &SurfaceBuildOptions::default()).expect("sources");

    for (index, piece) in selected.iter().enumerate() {
        let path = &normalized.path_original[index];
        let arc = path.is_arc(&k, 8, 1.0e-6).expect("is_arc").expect("arc");
        let center = arc.0.origin();
        let prof_arc = piece
            .profile
            .is_arc(&k, 8, 1.0e-6)
            .expect("profile is_arc")
            .expect("profile arc");
        println!(
            "profile frame {index}: x={:?} y={:?} z={:?}",
            prof_arc.0.x_axis(),
            prof_arc.0.y_axis(),
            prof_arc.0.z_axis()
        );
        for (label, u, expected_center) in [
            ("start", 0.0, piece.path_start),
            ("end", 1.0, piece.path_end),
        ] {
            let mut sum = Vec3::ZERO;
            let mut min = f64::INFINITY;
            let mut max = 0.0_f64;
            for sample in 0..=16 {
                let v = f64::from(sample) / 16.0;
                let point = evaluated(&piece.surface, &k, Point2::new(u, v));
                sum += point;
                let distance = (point - expected_center).length();
                min = min.min(distance);
                max = max.max(distance);
            }
            println!(
                "section {index} {label}: center_avg={:?} min_r={min:.6} max_r={max:.6}",
                sum / 17.0
            );
        }
        let path_interval = normalized.placements[index].path_piece.natural_interval();
        let prepared_start = *normalized.placements[index]
            .path_piece
            .evaluate(path_interval.min(), 0, true, false)
            .expect("prepared start")
            .first()
            .expect("prepared start");
        let prepared_end = *normalized.placements[index]
            .path_piece
            .evaluate(path_interval.max(), 0, true, false)
            .expect("prepared end")
            .first()
            .expect("prepared end");
        let world_profile = normalized.placements[index]
            .world_profiles
            .first()
            .expect("world profile")
            .is_arc(&k, 8, 1.0e-6)
            .expect("profile arc")
            .expect("profile arc");
        let placed_profile = normalized.placements[index]
            .profiles_placed
            .first()
            .expect("placed profile")
            .is_arc(&k, 8, 1.0e-6)
            .expect("placed arc");
        println!(
            "input {index}: original_start={:?} original_end={:?} prepared_start={prepared_start:?} \
             prepared_end={prepared_end:?} world_profile_center={:?} world_profile_radius={} \
             placed_profile_center={:?}",
            piece.path_start,
            piece.path_end,
            world_profile.0.origin(),
            world_profile.1,
            placed_profile.as_ref().map(|a| a.0.origin()),
        );
        let profile_interval = piece.profile.natural_interval();
        let mut profile_sum = Vec3::ZERO;
        for sample in 0..=8 {
            let t = profile_interval.min()
                + (profile_interval.max() - profile_interval.min()) * f64::from(sample) / 8.0;
            profile_sum += piece.profile.evaluate_point(&k, t).expect("profile sample");
        }
        let profile_center = profile_sum / 9.0;
        println!(
            "torus {index}: path_start={:?} profile_center={profile_center:?} profile_offset={:?}",
            piece.path_start,
            profile_center - piece.path_start
        );
        let mut max = 0.0_f64;
        let mut max_u = 0.0;
        let mut max_v = 0.0;
        for u_index in 0..=4 {
            let u = f64::from(u_index) / 4.0;
            let path_interval = normalized.placements[index].path_piece.natural_interval();
            let path_param = path_interval.min() + (path_interval.max() - path_interval.min()) * u;
            let path_point = *normalized.placements[index]
                .path_piece
                .evaluate(path_param, 0, true, false)
                .expect("path point")
                .first()
                .expect("path point");
            let start_rel = piece.path_start - center;
            let point_rel = path_point - center;
            let delta = start_rel
                .cross(point_rel)
                .dot(frame.normal)
                .atan2(start_rel.dot(point_rel));
            let mut surface_sum = Vec3::ZERO;
            let mut surface_count = 0;
            for v_index in 0..=4 {
                let v = f64::from(v_index) / 4.0;
                let profile_param =
                    profile_interval.min() + (profile_interval.max() - profile_interval.min()) * v;
                let profile_point = piece
                    .profile
                    .evaluate_point(&k, profile_param)
                    .expect("profile point");
                let expected =
                    path_point + rotate(profile_point - piece.path_start, frame.normal, delta);
                let actual = evaluated(&piece.surface, &k, Point2::new(u, v));
                surface_sum += actual;
                surface_count += 1;
                let error = (expected - actual).length();
                if error > max {
                    max = error;
                    max_u = u;
                    max_v = v;
                }
            }
            let surface_center = surface_sum / f64::from(surface_count);
            println!(
                "torus {index} u={u:.3}: path={path_point:?} surface_center={surface_center:?} center_gap={:.9e}",
                (surface_center - path_point).length()
            );
        }
        println!(
            "torus {index}: kind={:?} path_center={center:?} path_radius={} profile_interval=[{:.6},{:.6}] \
             u={:?} v={:?} max={max:.9e} at u={max_u:.3} v={max_v:.3}",
            piece.kind,
            arc.1,
            profile_interval.min(),
            profile_interval.max(),
            piece.u_domain,
            piece.v_domain,
        );
    }
}

#[test]
fn indexed_profile_section_audit() {
    let k = Kernel::default();
    let profile = circle(&k);
    let paths = vec![
        arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0),
        arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 45.0),
    ];
    let normalized = normalize_planar_sweep(&k, std::slice::from_ref(&profile), &paths, true, 3)
        .expect("normalize");
    let selected =
        select_sweep_sources(&k, &normalized, &SurfaceBuildOptions::default()).expect("sources");
    let samples = 16usize;

    for (index, piece) in selected.iter().enumerate() {
        let path = &normalized.path_original[index];
        let arc = path.is_arc(&k, 8, 1.0e-6).expect("arc").expect("arc");
        let center = arc.0.origin();
        let axis = arc.0.x_axis().cross(arc.0.y_axis());
        let mut span = arc.3 - arc.2;
        if span <= 0.0 {
            span += 360.0;
        }
        let interval = piece.profile.natural_interval();
        let start_points: Vec<Point3> = (0..samples)
            .map(|i| {
                let t =
                    interval.min() + (interval.max() - interval.min()) * i as f64 / samples as f64;
                piece.profile.evaluate_point(&k, t).expect("profile point")
            })
            .collect();
        let end_points: Vec<Point3> = start_points
            .iter()
            .map(|point| center + rotate(*point - center, axis, span.to_radians()))
            .collect();
        for (label, u, expected) in [("U0", 0.0, &start_points), ("U1", 1.0, &end_points)] {
            let section: Vec<Point3> = (0..samples)
                .map(|i| {
                    evaluated(
                        &piece.surface,
                        &k,
                        Point2::new(u, i as f64 / samples as f64),
                    )
                })
                .collect();
            let mut best_error = f64::INFINITY;
            let mut best_shift = 0usize;
            let mut best_reverse = false;
            for reverse in [false, true] {
                for shift in 0..samples {
                    let mut max = 0.0_f64;
                    for (i, expected_point) in expected.iter().enumerate().take(samples) {
                        let section_index = if reverse {
                            (samples + shift - i % samples) % samples
                        } else {
                            (i + shift) % samples
                        };
                        max = max.max((*expected_point - section[section_index]).length());
                    }
                    if max < best_error {
                        best_error = max;
                        best_shift = shift;
                        best_reverse = reverse;
                    }
                }
            }
            println!(
                "section index {index} {label}: best_error={best_error:.9e} shift={best_shift} reverse={best_reverse}"
            );
        }
    }
}
