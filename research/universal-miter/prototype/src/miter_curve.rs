//! The miter curve and its material coordinate.

use crate::rail_solver::{PieceSegment, RailSolution};

/// One point of the miter curve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MiterSample {
    /// The profile's material offset along the path-plane normal.
    pub alpha: f64,
    /// The point `X(alpha)` in the path plane.
    pub point: [f64; 2],
    /// Which segment of the first piece produced the point.
    pub segment_a: PieceSegment,
    /// Which segment of the second piece produced the point.
    pub segment_b: PieceSegment,
}

/// A sampled miter curve in the path plane.
///
/// With the WR-407 frame the full miter surface is this curve extruded along
/// the path-plane normal `N`; `beta` is a common translation.
#[derive(Clone, Debug, PartialEq)]
pub struct MiterCurve {
    /// The samples, sorted by `alpha`.
    pub samples: Vec<MiterSample>,
}

impl MiterCurve {
    /// Build the curve from a rail trace.
    pub fn from_solutions(solutions: &[RailSolution]) -> Self {
        Self {
            samples: solutions
                .iter()
                .map(|solution| MiterSample {
                    alpha: solution.alpha,
                    point: solution.point,
                    segment_a: solution.segment_a,
                    segment_b: solution.segment_b,
                })
                .collect(),
        }
    }
}
