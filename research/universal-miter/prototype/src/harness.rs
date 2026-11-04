//! The common comparison harness.
//!
//! One [`MiterTestCase`] runs through both [`MiterStrategy`] variants. The
//! comparison is a first-class part of the research: the same input, the same
//! metrics, separate results. Equal B-rep topology is **not** accepted as proof
//! of geometric equivalence.

use core::fmt;

use crate::curve::{CurveSpec, GeometryError};
use crate::differential_geometry::{sub, unit};
use crate::miter_curve::MiterCurve;
use crate::rail_solver::{self, Piece, RailStop};
use crate::validation::{self, MiterDiagnostics};

const SQRT_2: f64 = std::f64::consts::SQRT_2;

/// Which construction a case runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MiterStrategy {
    /// The production `CreateSweepAlongPlanarPath` / `ProjectAndTrim` path.
    ///
    /// It is intentionally not implemented at level A: it shares the production
    /// algorithm and is added at level B, on the real B-rep.
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
    /// A line-circle bisector: `|X - focus| = radius - d_line` (a parabola).
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
    /// Two circle arcs with the inside signed distance: `r1 - |X - c1| =
    /// r2 - |X - c2|` (a quartic / Cartesian oval).
    CirclePair {
        /// First circle centre.
        c1: [f64; 2],
        /// First circle radius.
        r1: f64,
        /// Second circle centre.
        c2: [f64; 2],
        /// Second circle radius.
        r2: f64,
    },
    /// A sample set produced by an independent solver, for cases without a
    /// closed form.
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
    /// The two pieces are tangent at the corner; no miter exists.
    TangentJoin,
    /// The Newton iteration did not converge away from a known singularity.
    NoConvergence,
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
    /// A curve or a piece is malformed.
    Geometry(GeometryError),
}

impl fmt::Display for MiterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NotImplemented => f.write_str("the universal miter solver is not implemented"),
            Self::InvalidCase(reason) => f.write_str(reason),
            Self::Geometry(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for MiterError {}

impl From<GeometryError> for MiterError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

/// Run one case with one strategy.
///
/// `Universal` runs the level A rail solver. `Original` reproduces the
/// production plane cut only at level B; at level A it returns
/// [`MiterError::NotImplemented`] rather than pretending to be independent.
///
/// # Errors
///
/// [`MiterError`] for an unparseable case, malformed curve data or a corner
/// where the pieces do not meet.
pub fn run_case(case: &MiterTestCase, strategy: MiterStrategy) -> Result<MiterResult, MiterError> {
    match strategy {
        MiterStrategy::Original => Err(MiterError::NotImplemented),
        MiterStrategy::Universal => run_universal(case),
    }
}

fn run_universal(case: &MiterTestCase) -> Result<MiterResult, MiterError> {
    if !case.profile.alpha[0].is_finite()
        || !case.profile.alpha[1].is_finite()
        || !case.profile.beta[0].is_finite()
        || !case.profile.beta[1].is_finite()
    {
        return Err(MiterError::InvalidCase("non-finite profile interval"));
    }
    let alpha_span = (case.profile.alpha[1] - case.profile.alpha[0]).abs();
    let beta_span = (case.profile.beta[1] - case.profile.beta[0]).abs();
    let max_dimension = alpha_span.max(beta_span).max(1.0e-12);
    let extension = 2.0 * SQRT_2 * max_dimension;
    let p1 = Piece::new(case.curve_a.clone(), 0.0, extension).map_err(MiterError::Geometry)?;
    let p2 = Piece::new(case.curve_b.clone(), extension, 0.0).map_err(MiterError::Geometry)?;
    let alpha_min = case.profile.alpha[0].min(0.0);
    let alpha_max = case.profile.alpha[1].max(0.0);
    let trace = rail_solver::trace(&p1, &p2, alpha_min, alpha_max).map_err(MiterError::Geometry)?;
    let (start_a, _) = p1
        .differential(p1.corner_end())
        .map_err(MiterError::Geometry)?;
    let (start_b, _) = p2
        .differential(p2.corner_start())
        .map_err(MiterError::Geometry)?;
    let current_line =
        unit(sub(start_b.tangent, start_a.tangent)).map(|direction| (start_a.point, direction));
    let diagnostics =
        validation::validate_trace(&p1, &p2, &trace, current_line).map_err(MiterError::Geometry)?;
    let refusal =
        refusal_from_stop(trace.negative_stop).or_else(|| refusal_from_stop(trace.positive_stop));
    let status = refusal.map_or(MiterStatus::Ok, MiterStatus::Refused);
    Ok(MiterResult {
        strategy: MiterStrategy::Universal,
        status,
        curve: Some(MiterCurve::from_solutions(&trace.points)),
        diagnostics,
    })
}

fn refusal_from_stop(stop: Option<RailStop>) -> Option<MiterRefusal> {
    match stop {
        None | Some(RailStop::ReachedAlphaBound) => None,
        Some(RailStop::Fold) => Some(MiterRefusal::Fold),
        Some(RailStop::CurvatureCusp) => Some(MiterRefusal::CurvatureCusp),
        Some(RailStop::OutsidePiece) => Some(MiterRefusal::OutsidePiece),
        Some(RailStop::TangentJoin) => Some(MiterRefusal::TangentJoin),
        Some(RailStop::NoConvergence) => Some(MiterRefusal::NoConvergence),
    }
}
