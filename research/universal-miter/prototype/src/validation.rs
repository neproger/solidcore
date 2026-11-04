//! Diagnostics of one miter run.
//!
//! The diagnostics are deliberately independent of the construction: a run is
//! not validated by "the curve was built". Each sample is checked against the
//! rail residual, the signed distances by closest-point projection, the current
//! cutting line and the analytic reference where one exists.

use crate::differential_geometry::{cross, dist, dot, norm, rot90, sub, unit};
use crate::harness::ExpectedGeometry;
use crate::rail_solver::{Piece, PieceSegment, PlanarCurve, RailSolution, RailStop, RailTrace};

/// Residuals of one validation sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ResidualSample {
    /// The material offset of the sample.
    pub alpha: f64,
    /// `|G(v, w, alpha)|`.
    pub rail_residual: f64,
    /// `|d1(X) - alpha|`, with `d1` the signed distance to the first extended
    /// piece (closest-point projection, not the rail normal).
    pub signed_distance_error_a: f64,
    /// `|d2(X) - alpha|`, the same for the second piece.
    pub signed_distance_error_b: f64,
    /// The distance to the current cutting line (the tangent at the corner).
    pub distance_to_original_line: f64,
    /// Which segment of the first piece is the closest point.
    pub segment_a: PieceSegment,
    /// Which segment of the second piece is the closest point.
    pub segment_b: PieceSegment,
}

/// A summary of the numerical run.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MiterDiagnostics {
    /// Per-sample residuals.
    pub samples: Vec<ResidualSample>,
    /// The smallest `|det J|` seen on the traced branch.
    pub min_abs_determinant: f64,
    /// The largest rail residual.
    pub max_abs_rail_residual: f64,
    /// The largest `|d1 - alpha|`.
    pub max_abs_signed_distance_error_a: f64,
    /// The largest `|d2 - alpha|`.
    pub max_abs_signed_distance_error_b: f64,
    /// The largest distance to the current cutting line.
    pub max_distance_to_original_line: f64,
    /// The characteristic length used for the relative residuals.
    pub characteristic_scale: f64,
    /// The largest rail residual divided by the characteristic length.
    pub max_relative_rail_residual: f64,
    /// The largest `|d1 - alpha|` divided by the characteristic length.
    pub max_relative_signed_distance_error_a: f64,
    /// The largest `|d2 - alpha|` divided by the characteristic length.
    pub max_relative_signed_distance_error_b: f64,
    /// The largest Newton iteration count of one adaptive step.
    pub max_newton_iterations: usize,
    /// The smallest adaptive step.
    pub min_step: f64,
    /// The largest adaptive step.
    pub max_step: f64,
    /// Newton iterations spent on the trace.
    pub solver_iterations: usize,
    /// The stop of the negative branch.
    pub negative_stop: Option<RailStop>,
    /// The stop of the positive branch.
    pub positive_stop: Option<RailStop>,
}

/// The result of the representation / subdivision / reparameterisation checks.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct InvarianceReport {
    /// Largest point difference after splitting a piece at an interior
    /// parameter (the corner face must not change).
    pub subdivision_max_point_difference: f64,
    /// Largest point difference after an affine reparameterisation.
    pub reparameterisation_max_point_difference: f64,
    /// Largest point difference after replacing a curve by an exact B-spline
    /// representation of the same point set.
    pub representation_max_point_difference: f64,
}

fn maximum(values: impl Iterator<Item = f64>) -> f64 {
    values.fold(0.0_f64, f64::max)
}

/// Validate a rail trace against the independent closest-point distances.
///
/// `current_line` is the tangent of the current implementation at the corner,
/// as a point and a unit direction. It is `None` for a tangent join.
///
/// # Errors
///
/// [`crate::curve::GeometryError`] from the closest-point projection.
pub fn validate_trace<C1: PlanarCurve, C2: PlanarCurve>(
    p1: &Piece<C1>,
    p2: &Piece<C2>,
    trace: &RailTrace,
    current_line: Option<([f64; 2], [f64; 2])>,
) -> Result<MiterDiagnostics, crate::curve::GeometryError> {
    let mut samples = Vec::with_capacity(trace.points.len());
    let mut min_determinant = f64::INFINITY;
    for solution in &trace.points {
        let (distance_a, _, segment_a) = p1.closest(solution.point)?;
        let (distance_b, _, segment_b) = p2.closest(solution.point)?;
        let line_distance = current_line.map_or(0.0, |(point, direction)| {
            cross(direction, sub(solution.point, point)).abs()
        });
        min_determinant = min_determinant.min(solution.determinant.abs());
        samples.push(ResidualSample {
            alpha: solution.alpha,
            rail_residual: norm(solution.residual),
            signed_distance_error_a: (distance_a - solution.alpha).abs(),
            signed_distance_error_b: (distance_b - solution.alpha).abs(),
            distance_to_original_line: line_distance,
            segment_a,
            segment_b,
        });
    }
    if !min_determinant.is_finite() {
        min_determinant = 0.0;
    }
    let max_abs_rail_residual = maximum(samples.iter().map(|sample| sample.rail_residual));
    let max_abs_signed_distance_error_a =
        maximum(samples.iter().map(|sample| sample.signed_distance_error_a));
    let max_abs_signed_distance_error_b =
        maximum(samples.iter().map(|sample| sample.signed_distance_error_b));
    let scale = trace.characteristic_scale.max(1.0e-300);
    Ok(MiterDiagnostics {
        max_abs_rail_residual,
        max_abs_signed_distance_error_a,
        max_abs_signed_distance_error_b,
        max_distance_to_original_line: maximum(
            samples
                .iter()
                .map(|sample| sample.distance_to_original_line),
        ),
        characteristic_scale: trace.characteristic_scale,
        max_relative_rail_residual: max_abs_rail_residual / scale,
        max_relative_signed_distance_error_a: max_abs_signed_distance_error_a / scale,
        max_relative_signed_distance_error_b: max_abs_signed_distance_error_b / scale,
        max_newton_iterations: trace.max_iterations,
        min_step: trace.min_step,
        max_step: trace.max_step,
        samples,
        min_abs_determinant: min_determinant,
        solver_iterations: trace.iterations,
        negative_stop: trace.negative_stop,
        positive_stop: trace.positive_stop,
    })
}

/// The residual of `point` against an analytic reference.
///
/// The line and circle references use the signed distance on the left of the
/// line and the inside of a counter-clockwise circle. The test cases construct
/// their references with those conventions.
#[must_use]
pub fn expected_residual(expected: &ExpectedGeometry, point: [f64; 2]) -> Option<f64> {
    match expected {
        ExpectedGeometry::BisectorLine { through, direction } => {
            let direction = unit(*direction)?;
            Some(cross(direction, sub(point, *through)).abs())
        }
        ExpectedGeometry::Parabola {
            focus,
            directrix_point,
            directrix_direction,
            radius,
        } => {
            let direction = unit(*directrix_direction)?;
            let normal = rot90(direction);
            let line_distance = dot(sub(point, *directrix_point), normal);
            Some((dist(point, *focus) - (*radius - line_distance)).abs())
        }
        ExpectedGeometry::CirclePair { c1, r1, c2, r2 } => {
            Some(((r1 - dist(point, *c1)) - (r2 - dist(point, *c2))).abs())
        }
        ExpectedGeometry::Samples(points) => points
            .iter()
            .map(|reference| dist(point, *reference))
            .fold(None, |best, value| match best {
                Some(current) => Some(current.min(value)),
                None => Some(value),
            }),
    }
}

/// A nearest-reference distance for a point set, or `None` for an empty set.
#[must_use]
pub fn nearest_sample_distance(points: &[[f64; 2]], point: [f64; 2]) -> Option<f64> {
    points
        .iter()
        .map(|reference| dist(point, *reference))
        .fold(None, |best, value| match best {
            Some(current) => Some(current.min(value)),
            None => Some(value),
        })
}

/// The rail residual of one solution, exposed for reports.
#[must_use]
pub fn rail_residual(solution: &RailSolution) -> f64 {
    norm(solution.residual)
}
