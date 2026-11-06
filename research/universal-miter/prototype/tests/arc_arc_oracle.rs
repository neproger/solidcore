#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! Independent circle–circle oracle for the Arc–Arc miter.
//!
//! It does not use the rail solver. For each profile sample it intersects the
//! two same-angle offset circles, filters both original arc domains, and tracks
//! a continuous branch.

use solidcore::curve::curve::p2::Curve;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
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

fn angle_in_arc(angle: f64, start: f64, end: f64) -> bool {
    let mut a = angle;
    while a < start - 1.0e-9 {
        a += 360.0;
    }
    while a > end + 1.0e-9 {
        a -= 360.0;
    }
    a >= start - 1.0e-9 && a <= end + 1.0e-9
}

fn circle_intersections(c1: Point3, r1: f64, c2: Point3, r2: f64) -> Vec<Point3> {
    let delta = c2 - c1;
    let distance = delta.length();
    if distance <= 1.0e-12 {
        return Vec::new();
    }
    if distance > r1 + r2 + 1.0e-9 || distance < (r1 - r2).abs() - 1.0e-9 {
        return Vec::new();
    }
    let a = (r1 * r1 - r2 * r2 + distance * distance) / (2.0 * distance);
    let h2 = r1 * r1 - a * a;
    if h2 < -1.0e-9 {
        return Vec::new();
    }
    let h = h2.max(0.0).sqrt();
    let unit = delta / distance;
    let base = c1 + unit * a;
    let normal = Vec3::Z.cross(unit).unitize().unwrap_or(Vec3::X);
    if h <= 1.0e-12 {
        vec![base]
    } else {
        vec![base + normal * h, base - normal * h]
    }
}

struct ArcInfo {
    center: Point3,
    radius: f64,
    start_deg: f64,
    end_deg: f64,
    frame: solidcore::geom::axis2_placement::Axis2Placement,
    offset_normal: Vec3,
}

fn arc_info(curve: &Curve, k: &Kernel) -> ArcInfo {
    let arc = curve.is_arc(k, 8, 1.0e-9).expect("is_arc").expect("arc");
    let start = arc.2.to_radians();
    let tangent = arc.0.x_axis() * -start.sin() + arc.0.y_axis() * start.cos();
    let normal = arc.0.z_axis().cross(tangent).unitize().expect("normal");
    ArcInfo {
        center: arc.0.origin(),
        radius: arc.1,
        start_deg: arc.2,
        end_deg: arc.3,
        frame: arc.0,
        offset_normal: normal,
    }
}

fn angle_at(arc: &ArcInfo, point: Point3) -> f64 {
    let relative = point - arc.center;
    relative
        .dot(arc.frame.y_axis())
        .atan2(relative.dot(arc.frame.x_axis()))
        .to_degrees()
}

fn offset_radius(arc: &ArcInfo, alpha: f64) -> f64 {
    let start = arc.start_deg.to_radians();
    let point = arc.center
        + arc.frame.x_axis() * (arc.radius * start.cos())
        + arc.frame.y_axis() * (arc.radius * start.sin());
    ((point + arc.offset_normal * alpha) - arc.center).length()
}

#[test]
fn open_arc_arc_circle_oracle() {
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
    let frame = PathFrame::new(&k, &normalized.path_original).expect("frame");
    let arc_a = arc_info(&normalized.path_original[0], &k);
    let arc_b = arc_info(&normalized.path_original[1], &k);
    let profile_interval = selected[0].profile.natural_interval();
    let samples = 68usize;
    let joint = selected[0].path_end;
    let mut previous: Option<Point3> = None;
    let mut missing = 0usize;
    let mut outside = 0usize;
    let mut multiple = 0usize;
    let mut discontinuous = 0usize;
    let mut first_missing = None;
    let mut first_discontinuous = None;

    for sample in 0..samples {
        let t = profile_interval.min()
            + (profile_interval.max() - profile_interval.min()) * sample as f64 / samples as f64;
        let point = selected[0].profile.evaluate_point(&k, t).expect("profile");
        let alpha = (point - frame.origin).dot(frame.y);
        let rho_a = offset_radius(&arc_a, alpha);
        let rho_b = offset_radius(&arc_b, alpha);
        if rho_a <= 1.0e-12 || rho_b <= 1.0e-12 {
            missing += 1;
            first_missing.get_or_insert((sample, "degenerate"));
            continue;
        }
        let points = circle_intersections(arc_a.center, rho_a, arc_b.center, rho_b);
        let valid: Vec<Point3> = points
            .iter()
            .copied()
            .filter(|candidate| {
                let a = angle_at(&arc_a, *candidate);
                let b = angle_at(&arc_b, *candidate);
                angle_in_arc(a, arc_a.start_deg, arc_a.end_deg)
                    && angle_in_arc(b, arc_b.start_deg, arc_b.end_deg)
            })
            .collect();
        if points.is_empty() {
            missing += 1;
            first_missing.get_or_insert((sample, "no-intersection"));
            continue;
        }
        if valid.is_empty() {
            outside += 1;
            first_missing.get_or_insert((sample, "outside-domains"));
            continue;
        }
        if valid.len() > 1 {
            multiple += 1;
        }
        let next = match previous {
            None => valid
                .iter()
                .copied()
                .min_by(|a, b| {
                    (*a - joint)
                        .length()
                        .partial_cmp(&(*b - joint).length())
                        .unwrap()
                })
                .expect("valid point"),
            Some(prev) => valid
                .iter()
                .copied()
                .min_by(|a, b| {
                    (*a - prev)
                        .length()
                        .partial_cmp(&(*b - prev).length())
                        .unwrap()
                })
                .expect("valid point"),
        };
        if let Some(prev) = previous {
            if (next - prev).length() > 0.5 {
                discontinuous += 1;
                first_discontinuous.get_or_insert(sample);
            }
        }
        previous = Some(next);
    }

    println!(
        "arc-arc circle oracle: samples={samples} missing={missing} outside={outside} \
         multiple={multiple} discontinuous={discontinuous} first_missing={first_missing:?} \
         first_discontinuous={first_discontinuous:?}"
    );
    assert!(
        missing + outside > 0,
        "the open arc-arc case unexpectedly has a full exact boundary"
    );
}
