//! The common comparison harness.
//!
//! One [`MiterTestCase`] runs through both [`MiterStrategy`] variants. The
//! comparison is a first-class part of the research: the same input, the same
//! metrics, separate results. Equal B-rep topology is **not** accepted as proof
//! of geometric equivalence.

use crate::curve::CurveSpec;
use crate::miter_curve::MiterCurve;
use crate::rail_solver::RailStop;
use crate::validation::MiterDiagnostics;

/// Which construction a case runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MiterStrategy {
    /// The production `CreateSweepAlongPlanarPath` / `ProjectAndTrim` path.
    Original,
    /// The universal rail-based miter under study.
    Universal,
}

/// The planar profile's material coordinates.
///
/// The seam is independent of `beta` (the extrusion along the path-plane
/// normal); `alpha` is the in-plane offset that selects the rail. A wall is an
/// `alpha` interval, a pipe an `alpha` interval at each `beta`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ProfileSpec {
    /// The in-plane material offset interval.
    pub alpha: [f64; 2],
    /// The extrusion interval along the path-plane normal.
    pub beta: [f64; 2],
}

/// An independent reference for a case, when a closed form exists.
#[derive(Clone, Debug, PartialEq)]
pub enum ExpectedGeometry {
    /// A straight bisector through `through` along `direction`.
    BisectorLine {
        /// A point on the line.
        through: [f64; 2],
        /// A direction along the line.
        direction: [f64; 2],
    },
    /// The line-circle bisector: `|X - focus| = radius - d_line` (a parabola).
    Parabola {
        /// The circle centre (focus of the parabola).
        focus: [f64; 2],
        /// A point on the line (directrix).
        directrix_point: [f64; 2],
        /// A direction along the line (directrix).
        directrix_direction: [f64; 2],
        /// The circle radius in the signed-distance equation.
        radius: f64,
    },
    /// A sample set produced by an independent solver (rail root or brute
    /// force), for cases without a closed form.
    Samples(Vec<[f64; 2]>),
}

/// One case, runnable by both strategies.
#[derive(Clone, Debug, PartialEq)]
pub struct MiterTestCase {
    /// Stable case ID (used in reports and fixtures).
    pub id: String,
    /// The first path piece, ending at the corner.
    pub curve_a: CurveSpec,
    /// The second path piece, starting at the corner.
    pub curve_b: CurveSpec,
    /// The planar profile.
    pub profile: ProfileSpec,
    /// An independent reference, when one exists.
    pub expected: Option<ExpectedGeometry>,
}

/// The outcome of one run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MiterStatus {
    /// The construction completed.
    Ok,
    /// The construction was refused at a documented limit.
    Refused(MiterRefusal),
    /// The research prototype does not implement the algorithm yet.
    NotImplemented,
}

/// Why a case was refused (a correct outcome, not a fallback).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MiterRefusal {
    /// The two offset rails became tangent (seam fold).
    Fold,
    /// The offset rail reached the curvature cusp (`1 - alpha kappa = 0`).
    CurvatureCusp,
    /// The seam left the extended piece.
    OutsidePiece,
    /// The path is not planar.
    NonPlanarPath,
    /// The path plane normal is undefined.
    UndefinedPlaneNormal,
    /// A scaling sweep changes the rail geometry.
    ScalingSweep,
}

/// The result of one strategy on one case.
#[derive(Clone, Debug, PartialEq)]
pub struct MiterResult {
    /// Which strategy produced the result.
    pub strategy: MiterStrategy,
    /// The outcome status.
    pub status: MiterStatus,
    /// The traced miter curve, when the run completed.
    pub curve: Option<MiterCurve>,
    /// Numerical diagnostics.
    pub diagnostics: MiterDiagnostics,
}

/// Errors of the harness itself (not of the geometry).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MiterError {
    /// The algorithm is not implemented in this infrastructure task.
    NotImplemented,
    /// The case is malformed (a doc-level condition, not an input curve).
    InvalidCase(&'static str),
}

impl core::fmt::Display for MiterError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotImplemented => f.write_str("the universal miter solver is not implemented"),
            Self::InvalidCase(reason) => f.write_str(reason),
        }
    }
}

impl std::error::Error for MiterError {}

/// Run one case with one strategy.
///
/// The infrastructure task only fixes the interface; the solver itself is the
/// next stage. Both strategies therefore return
/// [`MiterError::NotImplemented`].
///
/// # Errors
///
/// Always [`MiterError::NotImplemented`] in this stage.
pub fn run_case(
    _case: &MiterTestCase,
    _strategy: MiterStrategy,
) -> Result<MiterResult, MiterError> {
    Err(MiterError::NotImplemented)
}

/// The binding stop as a harness refusal, if the trace stopped at a limit.
#[must_use]
pub fn refusal_of(stop: RailStop) -> Option<MiterRefusal> {
    match stop {
        RailStop::Fold => Some(MiterRefusal::Fold),
        RailStop::CurvatureCusp => Some(MiterRefusal::CurvatureCusp),
        RailStop::OutsidePiece => Some(MiterRefusal::OutsidePiece),
        RailStop::NonPlanarPath => Some(MiterRefusal::NonPlanarPath),
        RailStop::ReachedAlphaBound => None,
    }
}
