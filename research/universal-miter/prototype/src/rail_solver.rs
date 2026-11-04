//! Data of the rail system `G(v, w, alpha) = 0`.
//!
//! The system matches the same material offset `alpha` on both path pieces:
//!
//! ```text
//! P1(v) + alpha n1(v) = P2(w) + alpha n2(w).
//! ```
//!
//! The solver is not implemented in this infrastructure task; these types fix
//! the interface that the level A implementation and the level B runner share.

use crate::differential_geometry::DifferentialData;

/// One solution of the rail system at a fixed `alpha`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RailSolution {
    /// The material offset along the profile's `alpha` axis.
    pub alpha: f64,
    /// Parameter on the first piece.
    pub v: f64,
    /// Parameter on the second piece.
    pub w: f64,
    /// The miter point `X(alpha)` in the path plane.
    pub point: [f64; 2],
    /// The residual `G(v, w, alpha)`.
    pub residual: [f64; 2],
    /// The determinant of the 2x2 Jacobian at the solution.
    pub determinant: f64,
    /// Differential data at `v`.
    pub data_a: DifferentialData,
    /// Differential data at `w`.
    pub data_b: DifferentialData,
}

/// Why a trace of the rail branch stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RailStop {
    /// The two offset rails became tangent (`t1` parallel to `t2`): a fold of
    /// the seam. The same-`alpha` intersection no longer exists.
    Fold,
    /// The offset rail reached its curvature cusp (`1 - alpha kappa = 0`).
    CurvatureCusp,
    /// The seam left the extended piece.
    OutsidePiece,
    /// The profile's `alpha` interval ended normally.
    ReachedAlphaBound,
    /// The path is not planar, or its plane normal is undefined.
    NonPlanarPath,
}

/// A traced branch of the miter curve, both directions from `alpha = 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct RailTrace {
    /// The solutions, sorted by `alpha`.
    pub points: Vec<RailSolution>,
    /// The first binding stop, if the trace did not cover the whole interval.
    pub stop: Option<RailStop>,
}
