#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::many_single_char_names
)]

//! Gate 1 matrix for the level A rail solver.
//!
//! Every case checks the rail residual, both independent signed distances,
//! the analytic reference where one exists, and the invariances. A refusal is
//! accepted only for the cases that reach a documented limit.

use std::f64::consts::PI;

use universal_miter_prototype::curve::CurveSpec;
use universal_miter_prototype::harness::{
    run_case, ExpectedGeometry, MiterRefusal, MiterResult, MiterStatus, MiterStrategy,
    MiterTestCase, ProfileSpec,
};
use universal_miter_prototype::rail_solver::{
    solve_sequence, Piece, PieceSegment, RailSolution, RailStop,
};
use universal_miter_prototype::validation::expected_residual;

const TOL: f64 = 1.0e-12;

fn line(start: [f64; 2], end: [f64; 2]) -> CurveSpec {
    CurveSpec::Line { start, end }
}

fn arc(center: [f64; 2], radius: f64, start_angle: f64, sweep_angle: f64) -> CurveSpec {
    CurveSpec::CircleArc {
        center,
        radius,
        start_angle,
        sweep_angle,
    }
}

fn quad(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2]) -> CurveSpec {
    CurveSpec::QuadraticBezier { p0, p1, p2 }
}

fn cubic(p0: [f64; 2], p1: [f64; 2], p2: [f64; 2], p3: [f64; 2]) -> CurveSpec {
    CurveSpec::CubicBezier { p0, p1, p2, p3 }
}

fn bspline_from_cubic(control: [[f64; 2]; 4]) -> CurveSpec {
    CurveSpec::BSpline {
        control: control.to_vec(),
        knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        degree: 3,
    }
}

fn case(
    id: &str,
    curve_a: CurveSpec,
    curve_b: CurveSpec,
    alpha: [f64; 2],
    expected: Option<ExpectedGeometry>,
) -> MiterTestCase {
    MiterTestCase {
        id: id.to_owned(),
        curve_a,
        curve_b,
        profile: ProfileSpec {
            alpha,
            beta: [0.0, 1.0],
        },
        expected,
    }
}

fn run(case: &MiterTestCase) -> MiterResult {
    run_case(case, MiterStrategy::Universal).expect("universal run")
}

fn piece(spec: CurveSpec, ext_start: f64, ext_end: f64) -> Piece<CurveSpec> {
    Piece::new(spec, ext_start, ext_end).expect("piece")
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

fn check_samples(result: &MiterResult, tolerance: f64) {
    let diagnostics = &result.diagnostics;
    assert!(
        diagnostics.max_abs_rail_residual <= tolerance,
        "rail residual {:.3e} > {:.3e}",
        diagnostics.max_abs_rail_residual,
        tolerance
    );
    assert!(
        diagnostics.max_abs_signed_distance_error_a <= tolerance,
        "d1 residual {:.3e} > {:.3e}",
        diagnostics.max_abs_signed_distance_error_a,
        tolerance
    );
    assert!(
        diagnostics.max_abs_signed_distance_error_b <= tolerance,
        "d2 residual {:.3e} > {:.3e}",
        diagnostics.max_abs_signed_distance_error_b,
        tolerance
    );
    assert!(
        diagnostics.max_relative_rail_residual <= tolerance,
        "relative rail residual {:.3e} > {:.3e}",
        diagnostics.max_relative_rail_residual,
        tolerance
    );
    assert!(
        diagnostics.max_relative_signed_distance_error_a <= tolerance,
        "relative d1 residual {:.3e} > {:.3e}",
        diagnostics.max_relative_signed_distance_error_a,
        tolerance
    );
    assert!(
        diagnostics.max_relative_signed_distance_error_b <= tolerance,
        "relative d2 residual {:.3e} > {:.3e}",
        diagnostics.max_relative_signed_distance_error_b,
        tolerance
    );
    for sample in &diagnostics.samples {
        assert!(
            sample.rail_residual <= tolerance,
            "rail residual at alpha {:.6}: {:.3e} > {:.3e}",
            sample.alpha,
            sample.rail_residual,
            tolerance
        );
        assert!(
            sample.signed_distance_error_a <= tolerance,
            "d1 error at alpha {:.6}: {:.3e} > {:.3e}",
            sample.alpha,
            sample.signed_distance_error_a,
            tolerance
        );
        assert!(
            sample.signed_distance_error_b <= tolerance,
            "d2 error at alpha {:.6}: {:.3e} > {:.3e}",
            sample.alpha,
            sample.signed_distance_error_b,
            tolerance
        );
    }
}

fn check_analytic(case: &MiterTestCase, result: &MiterResult, tolerance: f64) {
    let Some(expected) = &case.expected else {
        return;
    };
    let curve = result.curve.as_ref().expect("curve");
    let mut checked = 0usize;
    for sample in &curve.samples {
        if sample.segment_a != PieceSegment::Original || sample.segment_b != PieceSegment::Original
        {
            continue;
        }
        let residual = expected_residual(expected, sample.point).expect("analytic residual");
        assert!(
            residual <= tolerance,
            "case {} alpha {:.6}: analytic residual {:.3e} > {:.3e}",
            case.id,
            sample.alpha,
            residual,
            tolerance
        );
        checked += 1;
    }
    assert!(
        checked > 0,
        "case {} has no original samples for the analytic check",
        case.id
    );
}

fn check_ok(case: &MiterTestCase, tolerance: f64) -> MiterResult {
    let result = run(case);
    assert_eq!(
        result.status,
        MiterStatus::Ok,
        "case {} was refused: {:?}",
        case.id,
        result.status
    );
    check_samples(&result, tolerance);
    check_analytic(case, &result, tolerance);
    result
}

fn line_pair() -> (CurveSpec, CurveSpec) {
    (line([-2.0, 0.0], [0.0, 0.0]), line([0.0, 0.0], [0.0, 2.0]))
}

fn arc_pair() -> (CurveSpec, CurveSpec) {
    (
        arc([0.0, 1.0], 1.0, -PI, PI * 0.5),
        arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5),
    )
}

fn quad_pair() -> (CurveSpec, CurveSpec) {
    (
        quad([-2.0, 1.0], [-1.0, 0.0], [0.0, 0.0]),
        quad([0.0, 0.0], [0.0, 1.0], [1.0, 2.0]),
    )
}

fn cubic_points() -> ([[f64; 2]; 4], [[f64; 2]; 4]) {
    (
        [[-2.0, 1.5], [-1.2, 0.2], [-0.6, 0.0], [0.0, 0.0]],
        [[0.0, 0.0], [0.0, 0.6], [0.2, 1.2], [1.5, 2.0]],
    )
}

fn cubic_pair() -> (CurveSpec, CurveSpec) {
    let (a, b) = cubic_points();
    (cubic(a[0], a[1], a[2], a[3]), cubic(b[0], b[1], b[2], b[3]))
}

fn line_line_expected() -> ExpectedGeometry {
    ExpectedGeometry::BisectorLine {
        through: [0.0, 0.0],
        direction: [-1.0, 1.0],
    }
}

#[test]
fn line_line_is_the_exact_bisector() {
    let (a, b) = line_pair();
    let test = case("line-line", a, b, [-0.5, 0.5], Some(line_line_expected()));
    let result = check_ok(&test, TOL);
    let curve = result.curve.expect("curve");
    for sample in &curve.samples {
        let residual = expected_residual(&line_line_expected(), sample.point).expect("line");
        assert!(residual <= TOL, "line-line residual {residual:.3e}");
    }
}

#[test]
fn line_arc_is_the_analytic_parabola() {
    let a = line([-2.0, 0.0], [0.0, 0.0]);
    let b = arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5);
    let expected = ExpectedGeometry::Parabola {
        focus: [-1.0, 0.0],
        directrix_point: [0.0, 0.0],
        directrix_direction: [1.0, 0.0],
        radius: 1.0,
    };
    let test = case("line-arc", a, b, [-0.4, 0.4], Some(expected));
    check_ok(&test, TOL);
}

#[test]
fn arc_arc_is_the_analytic_circle_pair() {
    let (a, b) = arc_pair();
    let expected = ExpectedGeometry::CirclePair {
        c1: [0.0, 1.0],
        r1: 1.0,
        c2: [-1.0, 0.0],
        r2: 1.0,
    };
    let test = case("arc-arc", a, b, [-0.25, 0.25], Some(expected));
    check_ok(&test, TOL);
}

#[test]
fn asymmetric_arc_arc_is_the_analytic_circle_pair() {
    let a = arc([0.0, 1.0], 1.0, -PI, PI * 0.5);
    let b = arc([-1.4, 0.0], 1.4, 0.0, PI * 0.5);
    let expected = ExpectedGeometry::CirclePair {
        c1: [0.0, 1.0],
        r1: 1.0,
        c2: [-1.4, 0.0],
        r2: 1.4,
    };
    let test = case("arc-arc-asymmetric", a, b, [-0.2, 0.2], Some(expected));
    check_ok(&test, TOL);
}

#[test]
fn reflex_turn_uses_the_signed_material_offset() {
    let a = line([2.0, 0.0], [0.0, 0.0]);
    let b = line([0.0, 0.0], [0.0, 2.0]);
    let expected = ExpectedGeometry::BisectorLine {
        through: [0.0, 0.0],
        direction: [1.0, 1.0],
    };
    let test = case("line-line-reflex", a, b, [-0.4, 0.4], Some(expected));
    check_ok(&test, TOL);
}

#[test]
fn all_ten_pairs_are_green() {
    let (line_a, line_b) = line_pair();
    let (arc_a, arc_b) = arc_pair();
    let (quad_a, quad_b) = quad_pair();
    let (cubic_a, cubic_b) = cubic_pair();
    let pairs = [
        ("line-line", line_a.clone(), line_b.clone()),
        ("line-arc", line_a.clone(), arc_b.clone()),
        ("line-quad", line_a.clone(), quad_b.clone()),
        ("line-cubic", line_a, cubic_b.clone()),
        ("arc-arc", arc_a.clone(), arc_b),
        ("arc-quad", arc_a.clone(), quad_b.clone()),
        ("arc-cubic", arc_a, cubic_b.clone()),
        ("quad-quad", quad_a.clone(), quad_b),
        ("quad-cubic", quad_a, cubic_b.clone()),
        ("cubic-cubic", cubic_a, cubic_b),
    ];
    for (id, a, b) in pairs {
        let test = case(id, a, b, [-0.2, 0.2], None);
        check_ok(&test, TOL);
    }
}

fn sequence(p1: &Piece<CurveSpec>, p2: &Piece<CurveSpec>, alphas: &[f64]) -> Vec<RailSolution> {
    solve_sequence(p1, p2, alphas).expect("sequence")
}

fn point_at(sequence: &[RailSolution], alpha: f64) -> Option<[f64; 2]> {
    sequence
        .iter()
        .find(|solution| (solution.alpha - alpha).abs() < 1.0e-12)
        .map(|solution| solution.point)
}

fn compare_sequences(
    left: &[RailSolution],
    right: &[RailSolution],
    left_scale: f64,
    right_scale: f64,
    alpha_sign: f64,
    tolerance: f64,
) {
    for solution in left {
        let alpha = solution.alpha * alpha_sign;
        let other = point_at(right, alpha).expect("matching alpha");
        let a = [
            solution.point[0] / left_scale,
            solution.point[1] / left_scale,
        ];
        let b = [other[0] / right_scale, other[1] / right_scale];
        assert!(
            distance(a, b) <= tolerance,
            "point difference at alpha {:.6}: {:.3e}",
            solution.alpha,
            distance(a, b)
        );
    }
}

#[test]
fn bspline_representation_does_not_change_the_seam() {
    let (cubic_a, cubic_b) = cubic_points();
    let grid = [-0.2, -0.1, 0.0, 0.1, 0.2];
    let p1 = piece(
        cubic(cubic_a[0], cubic_a[1], cubic_a[2], cubic_a[3]),
        0.0,
        3.0,
    );
    let p2 = piece(
        cubic(cubic_b[0], cubic_b[1], cubic_b[2], cubic_b[3]),
        3.0,
        0.0,
    );
    let n1 = piece(bspline_from_cubic(cubic_a), 0.0, 3.0);
    let n2 = piece(bspline_from_cubic(cubic_b), 3.0, 0.0);
    let base = sequence(&p1, &p2, &grid);
    let spline = sequence(&n1, &n2, &grid);
    compare_sequences(&base, &spline, 1.0, 1.0, 1.0, TOL);
}

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

fn split_cubic(points: [[f64; 2]; 4]) -> (CurveSpec, CurveSpec) {
    let [p0, p1, p2, p3] = points;
    let a = lerp(p0, p1, 0.5);
    let d = lerp(p1, p2, 0.5);
    let f = lerp(p2, p3, 0.5);
    let e = lerp(a, d, 0.5);
    let g = lerp(d, f, 0.5);
    let h = lerp(e, g, 0.5);
    (cubic(p0, a, e, h), cubic(h, g, f, p3))
}

#[test]
fn subdivision_does_not_change_the_seam() {
    let (cubic_a, cubic_b) = cubic_points();
    let grid = [-0.2, -0.1, 0.0, 0.1, 0.2];
    let (_, right) = split_cubic(cubic_a);
    let p1 = piece(
        cubic(cubic_a[0], cubic_a[1], cubic_a[2], cubic_a[3]),
        0.0,
        3.0,
    );
    let p1s = piece(right, 0.0, 3.0);
    let p2 = piece(
        cubic(cubic_b[0], cubic_b[1], cubic_b[2], cubic_b[3]),
        3.0,
        0.0,
    );
    let base = sequence(&p1, &p2, &grid);
    let split = sequence(&p1s, &p2, &grid);
    compare_sequences(&base, &split, 1.0, 1.0, 1.0, TOL);
}

#[test]
fn reparameterisation_does_not_change_the_seam() {
    let (line_a, line_b) = line_pair();
    let grid = [-0.2, -0.1, 0.0, 0.1, 0.2];
    let p1 = piece(line_a.clone(), 0.0, 3.0);
    let p2 = piece(line_b.clone(), 3.0, 0.0);
    let p1r = Piece::with_parameter_map(line_a, 2.0, 5.0, 0.0, 3.0).expect("piece");
    let p2r = Piece::with_parameter_map(line_b, 2.0, 5.0, 3.0, 0.0).expect("piece");
    let base = sequence(&p1, &p2, &grid);
    let reparameterised = sequence(&p1r, &p2r, &grid);
    compare_sequences(&base, &reparameterised, 1.0, 1.0, 1.0, TOL);
}

#[test]
fn reversed_path_mirrors_the_seam() {
    let (line_a, line_b) = line_pair();
    let grid = [-0.2, -0.1, 0.0, 0.1, 0.2];
    let p1 = piece(line_a.clone(), 0.0, 3.0);
    let p2 = piece(line_b.clone(), 3.0, 0.0);
    let reversed_a = line_b.reversed().expect("reversed");
    let reversed_b = line_a.reversed().expect("reversed");
    let p1r = piece(reversed_a, 0.0, 3.0);
    let p2r = piece(reversed_b, 3.0, 0.0);
    let base = sequence(&p1, &p2, &grid);
    let reversed = sequence(&p1r, &p2r, &grid);
    compare_sequences(&reversed, &base, 1.0, 1.0, -1.0, TOL);
}

#[test]
fn scales_scale_the_seam_and_the_residuals() {
    let (line_a, line_b) = line_pair();
    let base_grid = [-0.2, 0.0, 0.2];
    let base = sequence(
        &piece(line_a.clone(), 0.0, 3.0),
        &piece(line_b.clone(), 3.0, 0.0),
        &base_grid,
    );
    for scale in [1.0e-3, 1.0e3] {
        let a = line([-2.0 * scale, 0.0], [0.0, 0.0]);
        let b = line([0.0, 0.0], [0.0, 2.0 * scale]);
        let test = MiterTestCase {
            id: format!("line-line-scale-{scale:e}"),
            curve_a: a.clone(),
            curve_b: b.clone(),
            profile: ProfileSpec {
                alpha: [-0.5 * scale, 0.5 * scale],
                beta: [0.0, scale],
            },
            expected: Some(line_line_expected()),
        };
        let result = run(&test);
        assert_eq!(result.status, MiterStatus::Ok);
        let tolerance = 1.0e-12 * scale.max(1.0);
        check_samples(&result, tolerance);
        check_analytic(&test, &result, tolerance);
        let grid = [-0.2 * scale, 0.0, 0.2 * scale];
        let scaled = sequence(
            &piece(a, 0.0, 3.0 * scale),
            &piece(b, 3.0 * scale, 0.0),
            &grid,
        );
        for solution in &scaled {
            let base_alpha = solution.alpha / scale;
            let other = point_at(&base, base_alpha).expect("matching base alpha");
            let normalised = [solution.point[0] / scale, solution.point[1] / scale];
            assert!(
                distance(normalised, other) <= TOL,
                "scaled point difference at alpha {:.6}: {:.3e}",
                solution.alpha,
                distance(normalised, other)
            );
        }
    }
}

#[test]
fn fold_stops_the_positive_branch_and_keeps_the_negative_one() {
    let a = line([-2.0, 0.0], [0.0, 0.0]);
    let b = arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5);
    let test = case("line-arc-fold", a, b, [-0.6, 0.7], None);
    let result = run(&test);
    assert_eq!(
        result.status,
        MiterStatus::Refused(MiterRefusal::Fold),
        "status {:?}",
        result.status
    );
    assert_eq!(result.diagnostics.positive_stop, Some(RailStop::Fold));
    assert_eq!(
        result.diagnostics.negative_stop,
        Some(RailStop::ReachedAlphaBound)
    );
    let curve = result.curve.expect("curve");
    assert!(
        curve.samples.iter().any(|sample| sample.alpha <= -0.59),
        "the negative branch did not continue"
    );
    let max_positive = curve
        .samples
        .iter()
        .filter(|sample| sample.alpha > 0.0)
        .map(|sample| sample.alpha)
        .fold(0.0_f64, f64::max);
    assert!(
        max_positive <= 0.51,
        "the positive branch crossed the fold: {max_positive}"
    );
}

#[test]
fn seam_outside_the_piece_is_refused() {
    let a = line([-2.0, 0.0], [0.0, 0.0]);
    let b = line([0.0, 0.0], [0.0, 0.1]);
    let test = MiterTestCase {
        id: "line-line-outside".to_owned(),
        curve_a: a,
        curve_b: b,
        profile: ProfileSpec {
            alpha: [0.0, 1.0],
            beta: [0.0, 1.0],
        },
        expected: None,
    };
    let result = run(&test);
    assert_eq!(
        result.status,
        MiterStatus::Refused(MiterRefusal::OutsidePiece),
        "status {:?}",
        result.status
    );
    assert_eq!(
        result.diagnostics.positive_stop,
        Some(RailStop::OutsidePiece)
    );
}

#[test]
fn g1_extension_segments_are_reported() {
    let a = line([-2.0, 0.0], [0.0, 0.0]);
    let b = arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5);
    let test = case("line-arc-segments", a, b, [-0.4, 0.4], None);
    let result = check_ok(&test, TOL);
    let curve = result.curve.expect("curve");
    let mut original = false;
    let mut extension = false;
    for sample in &curve.samples {
        if sample.alpha > 0.05 {
            assert_eq!(sample.segment_a, PieceSegment::Original);
            assert_eq!(sample.segment_b, PieceSegment::Original);
            original = true;
        }
        if sample.alpha < -0.05 {
            assert_eq!(sample.segment_a, PieceSegment::EndExtension);
            assert_eq!(sample.segment_b, PieceSegment::StartExtension);
            extension = true;
        }
    }
    assert!(original, "no original samples");
    assert!(extension, "no extension samples");
}

#[test]
fn seam_samples_are_sorted_by_alpha() {
    let (a, b) = line_pair();
    let test = case("line-line-sorted", a, b, [-0.5, 0.5], None);
    let result = check_ok(&test, TOL);
    let curve = result.curve.expect("curve");
    let mut previous = f64::NEG_INFINITY;
    for sample in &curve.samples {
        assert!(sample.alpha >= previous, "samples are not sorted");
        previous = sample.alpha;
    }
}

#[test]
fn arc_arc_fold_is_detected() {
    let (a, b) = arc_pair();
    let test = case("arc-arc-fold", a, b, [-0.4, 0.4], None);
    let result = run(&test);
    assert_eq!(
        result.status,
        MiterStatus::Refused(MiterRefusal::Fold),
        "status {:?}",
        result.status
    );
    assert_eq!(result.diagnostics.positive_stop, Some(RailStop::Fold));
    assert_eq!(
        result.diagnostics.negative_stop,
        Some(RailStop::ReachedAlphaBound)
    );
    let curve = result.curve.expect("curve");
    assert!(
        curve.samples.iter().any(|sample| sample.alpha <= -0.39),
        "the negative branch did not continue"
    );
    let max_positive = curve
        .samples
        .iter()
        .filter(|sample| sample.alpha > 0.0)
        .map(|sample| sample.alpha)
        .fold(0.0_f64, f64::max);
    assert!(
        max_positive <= 0.30,
        "the positive branch crossed the fold: {max_positive}"
    );
}

#[test]
fn gate1_report() {
    let (line_a, line_b) = line_pair();
    let (arc_a, arc_b) = arc_pair();
    let (quad_a, quad_b) = quad_pair();
    let (cubic_a, cubic_b) = cubic_points();
    let cases = [
        case(
            "line-line",
            line_a.clone(),
            line_b.clone(),
            [-0.5, 0.5],
            Some(line_line_expected()),
        ),
        case(
            "line-arc",
            line_a.clone(),
            arc_b.clone(),
            [-0.4, 0.4],
            Some(ExpectedGeometry::Parabola {
                focus: [-1.0, 0.0],
                directrix_point: [0.0, 0.0],
                directrix_direction: [1.0, 0.0],
                radius: 1.0,
            }),
        ),
        case(
            "arc-arc",
            arc_a.clone(),
            arc_b.clone(),
            [-0.25, 0.25],
            Some(ExpectedGeometry::CirclePair {
                c1: [0.0, 1.0],
                r1: 1.0,
                c2: [-1.0, 0.0],
                r2: 1.0,
            }),
        ),
        case(
            "arc-arc-asym",
            arc([0.0, 1.0], 1.0, -PI, PI * 0.5),
            arc([-1.4, 0.0], 1.4, 0.0, PI * 0.5),
            [-0.2, 0.2],
            Some(ExpectedGeometry::CirclePair {
                c1: [0.0, 1.0],
                r1: 1.0,
                c2: [-1.4, 0.0],
                r2: 1.4,
            }),
        ),
        case(
            "quad-quad",
            quad_a.clone(),
            quad_b.clone(),
            [-0.2, 0.2],
            None,
        ),
        case(
            "cubic-cubic",
            cubic(cubic_a[0], cubic_a[1], cubic_a[2], cubic_a[3]),
            cubic(cubic_b[0], cubic_b[1], cubic_b[2], cubic_b[3]),
            [-0.2, 0.2],
            None,
        ),
        case(
            "bspline-bspline",
            bspline_from_cubic(cubic_a),
            bspline_from_cubic(cubic_b),
            [-0.2, 0.2],
            None,
        ),
        case(
            "line-arc-fold",
            line_a,
            arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5),
            [-0.6, 0.7],
            None,
        ),
        case("arc-arc-fold", arc_a, arc_b, [-0.4, 0.4], None),
        case(
            "line-line-outside",
            line([-2.0, 0.0], [0.0, 0.0]),
            line([0.0, 0.0], [0.0, 0.1]),
            [0.0, 1.0],
            None,
        ),
    ];
    for test in &cases {
        let result = run(test);
        let count = result.curve.as_ref().map_or(0, |curve| curve.samples.len());
        let max_positive = result.curve.as_ref().map_or(f64::NEG_INFINITY, |curve| {
            curve
                .samples
                .iter()
                .filter(|sample| sample.alpha > 0.0)
                .map(|sample| sample.alpha)
                .fold(f64::NEG_INFINITY, f64::max)
        });
        let min_negative = result.curve.as_ref().map_or(f64::INFINITY, |curve| {
            curve
                .samples
                .iter()
                .filter(|sample| sample.alpha < 0.0)
                .map(|sample| sample.alpha)
                .fold(f64::INFINITY, f64::min)
        });
        println!(
            "case={:20} status={:?} samples={:3} scale={:.3e} rail={:.3e} rel={:.3e} d1={:.3e} d2={:.3e} line={:.3e} min_det={:.3e} max_it={:2} steps=[{:.2e},{:.2e}] max_pos={:.6} min_neg={:.6} neg={:?} pos={:?}",
            test.id,
            result.status,
            count,
            result.diagnostics.characteristic_scale,
            result.diagnostics.max_abs_rail_residual,
            result.diagnostics.max_relative_rail_residual,
            result.diagnostics.max_abs_signed_distance_error_a,
            result.diagnostics.max_abs_signed_distance_error_b,
            result.diagnostics.max_distance_to_original_line,
            result.diagnostics.min_abs_determinant,
            result.diagnostics.max_newton_iterations,
            result.diagnostics.min_step,
            result.diagnostics.max_step,
            max_positive,
            min_negative,
            result.diagnostics.negative_stop,
            result.diagnostics.positive_stop
        );
    }
}

#[test]
fn repeated_runs_are_identical() {
    let (a, b) = line_pair();
    let test = case("repeatable", a, b, [-0.3, 0.3], None);
    let first = run(&test);
    let second = run(&test);
    assert_eq!(first, second, "the rail solver is not deterministic");
}

#[test]
fn closest_point_is_a_geometric_projection() {
    let line_piece = piece(line([-2.0, 0.0], [0.0, 0.0]), 0.0, 0.0);
    let (above, _, segment) = line_piece.closest([-1.0, 0.75]).expect("closest");
    assert!(
        (above - 0.75).abs() <= TOL,
        "signed distance above: {above}"
    );
    assert_eq!(segment, PieceSegment::Original);
    let (below, _, _) = line_piece.closest([-1.0, -0.25]).expect("closest");
    assert!(
        (below + 0.25).abs() <= TOL,
        "signed distance below: {below}"
    );
}

#[test]
fn fold_is_approached_from_both_sides() {
    let a = line([-2.0, 0.0], [0.0, 0.0]);
    let b = arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5);
    let below = case("below-fold", a.clone(), b.clone(), [-0.3, 0.49], None);
    assert_eq!(run(&below).status, MiterStatus::Ok);
    let above = case("above-fold", a.clone(), b, [-0.3, 0.51], None);
    let result = run(&above);
    assert_eq!(
        result.status,
        MiterStatus::Refused(MiterRefusal::Fold),
        "status {:?}",
        result.status
    );
    assert_eq!(result.diagnostics.positive_stop, Some(RailStop::Fold));
    assert!(
        solve_sequence(
            &piece(a, 0.0, 3.0),
            &piece(arc([-1.0, 0.0], 1.0, 0.0, PI * 0.5), 3.0, 0.0),
            &[0.51]
        )
        .is_err(),
        "the rail system should have no solution above the fold"
    );
}
