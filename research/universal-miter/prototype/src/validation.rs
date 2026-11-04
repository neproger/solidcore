//! Diagnostics of one miter run.
//!
//! The diagnostics are deliberately independent of the construction: a run is
//! not validated by "the curve was built". The same sample is checked against
//! the rail residual, the signed distances, the current cutting line and the
//! analytic reference where one exists.

use crate::rail_solver::RailStop;

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
    /// `|onesided_distance_to_current_line|`, the tangent of the current
    /// implementation at the corner.
    pub distance_to_original_line: f64,
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
    /// Newton iterations spent on the whole trace.
    pub solver_iterations: usize,
    /// The first binding stop, if the trace was cut short.
    pub stop: Option<RailStop>,
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
