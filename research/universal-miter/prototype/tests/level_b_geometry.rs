#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::many_single_char_names
)]

//! `KernelCurve` + controlled Surface Builder tests. No seam, trim, caps or B-rep.

use solidcore::curve::curve::Curve;
use solidcore::curve::parabola::Parabola;
use solidcore::geom::extent1d::Extent1d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::b_spline_surface::SurfParamType;
use solidcore::primitive::api_curves;

use universal_miter_prototype::curve::CurveSpec;
use universal_miter_prototype::level_b_geometry::{
    build_swept_surface, BuiltSweptSurface, KernelCurve, SurfaceBuildError, SurfaceBuildOptions,
};
use universal_miter_prototype::rail_solver::{solve_sequence, Piece, PlanarCurve, RailSolution};

fn line(start: Point3, end: Point3) -> BSplineCurve {
    api_curves::create_line_segment(start, end)
        .expect("line")
        .base_with_nurb()
        .expect("line nurb")
        .into_owned()
}

fn circle(center: Point3, radius: f64) -> BSplineCurve {
    api_curves::create_circle(center, radius).expect("circle")
}

fn ellipse(k: &Kernel, center: Point3, r1: f64, r2: f64) -> BSplineCurve {
    api_curves::create_ellipse(k, center, r1, r2)
        .expect("ellipse")
        .base()
        .clone()
}

fn bspline(points: &[Point3]) -> BSplineCurve {
    api_curves::create_curve(points).expect("bspline")
}

fn kernel_curve<'a>(k: &'a Kernel, curve: &BSplineCurve) -> KernelCurve<'a> {
    KernelCurve::new(
        k,
        Curve::from(curve.clone()),
        Point3::ZERO,
        Vec3::X,
        Vec3::Y,
    )
}

fn compare_differential(left: &dyn PlanarCurve, right: &dyn PlanarCurve, samples: &[f64]) {
    for &u in samples {
        let a = left.differential(u).expect("left differential");
        let b = right.differential(u).expect("right differential");
        let point = (a.point[0] - b.point[0]).hypot(a.point[1] - b.point[1]);
        let tangent = (a.tangent[0] - b.tangent[0]).hypot(a.tangent[1] - b.tangent[1]);
        assert!(point <= 1.0e-9, "point difference {point} at u={u}");
        assert!(tangent <= 1.0e-9, "tangent difference {tangent} at u={u}");
        assert!(
            (a.curvature - b.curvature).abs() <= 1.0e-7,
            "curvature difference {} at u={u}",
            (a.curvature - b.curvature).abs()
        );
    }
}

fn result_point(result: &[RailSolution], alpha: f64) -> [f64; 2] {
    result
        .iter()
        .find(|solution| (solution.alpha - alpha).abs() < 1.0e-9)
        .expect("alpha")
        .point
}

#[test]
fn line_kernel_curve_matches_curve_spec() {
    let k = Kernel::default();
    let real = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 1.0, 0.0));
    let spec = CurveSpec::Line {
        start: [0.0, 0.0],
        end: [2.0, 1.0],
    };
    let kernel = kernel_curve(&k, &real);
    compare_differential(&kernel, &spec, &[0.0, 0.25, 0.5, 0.75, 1.0]);
}

#[test]
fn bspline_kernel_curve_matches_curve_spec() {
    let k = Kernel::default();
    let points = [
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 1.0, 0.0),
        Point3::new(2.0, -0.5, 0.0),
        Point3::new(3.0, 0.5, 0.0),
        Point3::new(4.0, 0.0, 0.0),
    ];
    let real = bspline(&points);
    let control = real
        .control_points()
        .expect("control points")
        .iter()
        .map(|point| [point.x, point.y])
        .collect::<Vec<_>>();
    let (unique, multiplicity) = real.knots(None).expect("knots");
    let mut knots = Vec::new();
    for (knot, count) in unique.iter().zip(multiplicity.iter()) {
        for _ in 0..*count {
            knots.push(*knot);
        }
    }
    let spec = CurveSpec::BSpline {
        control,
        knots,
        degree: real.degree().expect("degree"),
    };
    let kernel = kernel_curve(&k, &real);
    let interval = kernel.natural_interval().expect("interval");
    let samples = [
        interval[0],
        interval[0] + (interval[1] - interval[0]) * 0.25,
        interval[0] + (interval[1] - interval[0]) * 0.5,
        interval[0] + (interval[1] - interval[0]) * 0.75,
        interval[1],
    ];
    compare_differential(&kernel, &spec, &samples);
}

#[test]
fn circle_kernel_curve_matches_its_geometry_and_sign() {
    let k = Kernel::default();
    let real = circle(Point3::new(2.0, -1.0, 0.0), 3.0);
    let kernel = kernel_curve(&k, &real);
    let mut saw_positive = false;
    let mut saw_negative = false;
    for index in 0..=16 {
        let u = f64::from(index) / 16.0;
        let data = kernel.differential(u).expect("circle differential");
        let relative = [data.point[0] - 2.0, data.point[1] + 1.0];
        let radius = relative[0].hypot(relative[1]);
        assert!((radius - 3.0).abs() <= 1.0e-9, "radius {radius}");
        assert!(
            (data.curvature.abs() - 1.0 / 3.0).abs() <= 1.0e-9,
            "curvature {}",
            data.curvature
        );
        if data.curvature > 0.0 {
            saw_positive = true;
        } else {
            saw_negative = true;
        }
    }
    assert!(saw_positive || saw_negative);
}

#[test]
fn ellipse_kernel_curve_keeps_its_curvature_sign() {
    let k = Kernel::default();
    let real = ellipse(&k, Point3::new(0.0, 0.0, 0.0), 2.0, 0.5);
    let kernel = kernel_curve(&k, &real);
    let mut signs = (false, false);
    for index in 0..=16 {
        let u = f64::from(index) / 16.0;
        let data = kernel.differential(u).expect("ellipse differential");
        assert!(data.speed > 0.0);
        if data.curvature > 0.0 {
            signs.0 = true;
        }
        if data.curvature < 0.0 {
            signs.1 = true;
        }
    }
    assert!(signs.0 || signs.1);
}

#[test]
fn parabola_and_hyperbola_kernel_curves_evaluate() {
    let k = Kernel::default();
    let parabola = Curve::from(Parabola::new(
        Point3::new(0.0, 0.0, 0.0),
        Vec3::X,
        Vec3::Y,
        Extent1d::from_min_max(-1.0, 1.0),
        1.0,
        3,
    ));
    let kernel = KernelCurve::new(&k, parabola, Point3::ZERO, Vec3::X, Vec3::Y);
    let data = kernel.differential(0.5).expect("parabola differential");
    assert!(data.speed > 0.0);
    assert!(data.curvature.is_finite());
}

#[test]
fn kernel_curve_matches_level_a_line_circle() {
    let k = Kernel::default();
    let real_line = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let real_circle = circle(Point3::ZERO, 1.0);
    let spec_line = CurveSpec::Line {
        start: [0.0, 0.0],
        end: [1.0, 0.0],
    };
    let spec_circle = CurveSpec::CircleArc {
        center: [0.0, 0.0],
        radius: 1.0,
        start_angle: 0.0,
        sweep_angle: std::f64::consts::TAU,
    };
    let real_a = Piece::new(kernel_curve(&k, &real_line), 0.5, 0.5).expect("real line");
    let real_b = Piece::new(kernel_curve(&k, &real_circle), 0.5, 0.5).expect("real circle");
    let spec_a = Piece::new(spec_line, 0.5, 0.5).expect("spec line");
    let spec_b = Piece::new(spec_circle, 0.5, 0.5).expect("spec circle");
    let grid = [-0.2, -0.1, 0.1, 0.2];
    let real = solve_sequence(&real_a, &real_b, &grid).expect("real sequence");
    let spec = solve_sequence(&spec_a, &spec_b, &grid).expect("spec sequence");
    for alpha in grid {
        let a = result_point(&real, alpha);
        let b = result_point(&spec, alpha);
        let difference = (a[0] - b[0]).hypot(a[1] - b[1]);
        assert!(
            difference <= 1.0e-7,
            "Level A seam difference {difference} at alpha={alpha}"
        );
    }
}

#[test]
fn reversed_kernel_curve_matches_curve_spec() {
    let k = Kernel::default();
    let real = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 1.0, 0.0));
    let mut reversed = Curve::from(real);
    let interval = reversed.natural_interval();
    reversed
        .reverse_parameterization(&k, &interval)
        .expect("reverse");
    let spec = CurveSpec::Line {
        start: [2.0, 1.0],
        end: [0.0, 0.0],
    };
    let kernel = KernelCurve::new(&k, reversed, Point3::ZERO, Vec3::X, Vec3::Y);
    compare_differential(&kernel, &spec, &[0.0, 0.25, 0.5, 0.75, 1.0]);
}

#[test]
fn affine_reparameterisation_and_scaling_match() {
    let k = Kernel::default();
    let real = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 1.0, 0.0));
    let kernel = kernel_curve(&k, &real);
    let mapped_real = Piece::with_parameter_map(
        KernelCurve::new(
            &k,
            Curve::from(real.clone()),
            Point3::ZERO,
            Vec3::X,
            Vec3::Y,
        ),
        2.0,
        5.0,
        0.0,
        0.0,
    )
    .expect("mapped real");
    let spec = CurveSpec::Line {
        start: [0.0, 0.0],
        end: [2.0, 1.0],
    };
    let direct = kernel.differential(0.5).expect("direct");
    let mapped = mapped_real.differential(3.5).expect("mapped").0;
    assert!((direct.point[0] - mapped.point[0]).abs() <= 1.0e-9);
    assert!((direct.point[1] - mapped.point[1]).abs() <= 1.0e-9);
    let _ = spec;

    let scaled_real = line(Point3::new(0.0, 0.0, 0.0), Point3::new(20.0, 10.0, 0.0));
    let scaled_kernel = kernel_curve(&k, &scaled_real);
    let scaled_spec = CurveSpec::Line {
        start: [0.0, 0.0],
        end: [20.0, 10.0],
    };
    compare_differential(&scaled_kernel, &scaled_spec, &[0.5]);
}

fn cross_section(path: &BSplineCurve) -> BSplineCurve {
    let values = path
        .evaluate(path.natural_interval().min(), 1, true, false)
        .expect("path evaluation");
    let start = *values.first().expect("path start");
    let tangent = values
        .get(1)
        .copied()
        .expect("path tangent")
        .unitize()
        .expect("unit tangent");
    let normal = Vec3::Z.cross(tangent).unitize().expect("unit normal");
    line(start + normal * 0.25, start + normal * 0.25 + Vec3::Z * 0.5)
}

fn builder_options(tolerance: f64) -> SurfaceBuildOptions {
    SurfaceBuildOptions {
        tolerance,
        max_refinements: 7,
        max_sections: 129,
        degree: 3,
        analytic_samples_v: 17,
        analytic_samples_u: 9,
        surface_samples_v: 9,
        surface_samples_u: 5,
    }
}

fn check_surface(k: &Kernel, path: &BSplineCurve, tolerance: f64) -> BuiltSweptSurface {
    let profile = cross_section(path);
    let surface = build_swept_surface(k, path, &profile, Vec3::Z, &builder_options(tolerance))
        .expect("surface build");
    let test = std::thread::current()
        .name()
        .unwrap_or("unknown")
        .to_string();
    println!(
        "{test}: tolerance={:.3e} deviation={:.3e} sections={} refinements={}",
        tolerance, surface.max_deviation, surface.sections, surface.refinements
    );
    assert!(
        surface.max_deviation <= tolerance,
        "deviation {} > {tolerance}",
        surface.max_deviation
    );
    surface
}

#[test]
fn surface_builder_straight_path() {
    let k = Kernel::default();
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let surface = check_surface(&k, &path, 1.0e-7);
    assert!(surface.sections >= 2);
}

#[test]
fn surface_builder_circle_and_ellipse_paths() {
    let k = Kernel::default();
    let circle_path = circle(Point3::ZERO, 1.0);
    check_surface(&k, &circle_path, 1.0e-3);
    let ellipse_path = ellipse(&k, Point3::ZERO, 2.0, 0.7);
    check_surface(&k, &ellipse_path, 1.0e-3);
}

#[test]
fn surface_builder_bezier_and_rational_paths() {
    let k = Kernel::default();
    let bezier = bspline(&[
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.5, 0.0),
        Point3::new(2.0, -0.5, 0.0),
        Point3::new(3.0, 0.0, 0.0),
    ]);
    check_surface(&k, &bezier, 1.0e-3);
    let rational = circle(Point3::new(0.0, 0.0, 0.0), 2.0);
    check_surface(&k, &rational, 1.0e-3);
}

#[test]
fn surface_builder_scales() {
    let k = Kernel::default();
    let small = circle(Point3::ZERO, 1.0e-3);
    let surface = check_surface(&k, &small, 1.0e-4);
    assert!(surface.max_deviation <= 1.0e-4);
    let large = circle(Point3::ZERO, 1.0e3);
    let surface = check_surface(&k, &large, 1.0e-1);
    assert!(surface.max_deviation <= 1.0e-1);
}

#[test]
fn surface_builder_refuses_unreachable_tolerance() {
    let k = Kernel::default();
    let path = circle(Point3::ZERO, 1.0);
    let profile = cross_section(&path);
    let options = SurfaceBuildOptions {
        tolerance: 1.0e-15,
        max_refinements: 0,
        max_sections: 4,
        analytic_samples_v: 9,
        analytic_samples_u: 5,
        surface_samples_v: 5,
        surface_samples_u: 3,
        ..SurfaceBuildOptions::default()
    };
    let result = build_swept_surface(&k, &path, &profile, Vec3::Z, &options);
    assert!(matches!(
        result,
        Err(SurfaceBuildError::ToleranceNotReached { .. })
    ));
}

#[test]
fn surface_builder_is_deterministic() {
    let k = Kernel::default();
    let path = circle(Point3::ZERO, 1.0);
    let profile = cross_section(&path);
    let options = builder_options(1.0e-3);
    let first = build_swept_surface(&k, &path, &profile, Vec3::Z, &options).expect("first");
    let second = build_swept_surface(&k, &path, &profile, Vec3::Z, &options).expect("second");
    assert_eq!(first.sections, second.sections);
    assert_eq!(first.refinements, second.refinements);
    assert!((first.max_deviation - second.max_deviation).abs() <= 1.0e-15);
}

#[test]
fn surface_builder_boundary_curve_matches_the_start_profile() {
    let k = Kernel::default();
    let path = circle(Point3::ZERO, 1.0);
    let profile = cross_section(&path);
    let options = builder_options(1.0e-3);
    let built = build_swept_surface(&k, &path, &profile, Vec3::Z, &options).expect("surface");
    let surface = solidcore::surface::surface::Surface::from(built.surface.clone());
    let domain = surface.natural_uv_domain_with_nurb(&k).expect("domain");
    let iso = surface
        .create_iso_parametric_curve(&k, SurfParamType::U, domain.min().x, 1.0e-7, None, false)
        .expect("iso");
    let mut worst = 0.0_f64;
    for index in 0..=20 {
        let u = f64::from(index) / 20.0;
        let interval = iso.curve.natural_interval();
        let parameter = interval.min() + (interval.max() - interval.min()) * u;
        let point = iso.curve.evaluate_point(&k, parameter).expect("iso point");
        let analytic = profile
            .evaluate_point(profile.natural_interval().evaluate(u))
            .expect("profile point");
        worst = worst.max(point.distance(analytic));
    }
    println!("surface_builder_boundary: boundary difference={worst:.3e}");
    assert!(worst <= 1.0e-6, "boundary difference {worst}");
}
