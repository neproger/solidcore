//! Mathematical joint-boundary construction (`PlanarSweep2` boundary level).
//!
//! The boundary solver is independent of the B-rep builder: it takes the two
//! adjacent path segments and the placed profile, analyses the joint geometry
//! and returns the shared 3D boundary. Every strategy implements the same
//! [`BoundaryStrategy`] contract; the solver classifies each profile sample by
//! the extended pieces before constructing the seam, so it does not try a
//! strategy and fall back to another.
//!
//! Strategies:
//! - [`ExactMiter`]: the seam lies on both original offset trajectories.
//! - [`HybridAtJunction`]: exact where it exists; on the missing side the seam
//!   continues on the tangent extension at the joint.
//! - [`HybridAtLimitTangent`]: the AOIM limit-tangent negative control. It
//!   refuses when the profile needs a junction continuation.

use core::fmt;

use solidcore::curve::curve::p3::compute_crvs_normal;
use solidcore::curve::curve::Curve;
use solidcore::error::SolidError;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::crv_fit::p1::ParamMethod;

use crate::curve::GeometryError;
use crate::level_b_geometry::KernelCurve;
use crate::rail_solver::{solve_sequence, Piece, PieceSegment, RailSolution, RailSolveError};

/// The local path-plane frame.
#[derive(Clone, Copy, Debug)]
pub struct PathFrame {
    /// The path start.
    pub origin: Point3,
    /// The first path tangent.
    pub x: Vec3,
    /// The first left normal, `normal x x`.
    pub y: Vec3,
    /// The path-plane normal.
    pub normal: Vec3,
}

impl PathFrame {
    /// Build the frame from the whole planar path.
    ///
    /// # Errors
    ///
    /// [`BoundaryError`] for a missing path, a non-planar path or a malformed
    /// first path evaluation.
    pub fn new(k: &Kernel, paths: &[Curve]) -> Result<Self, BoundaryError> {
        let normal = compute_crvs_normal(k, paths)?
            .unitize()
            .map_err(BoundaryError::Kernel)?;
        let first = paths
            .first()
            .ok_or(BoundaryError::Shape("the path has no first curve"))?;
        let values = first
            .evaluate(k, first.natural_interval().min(), 1, true, false)
            .map_err(BoundaryError::Kernel)?;
        let origin = *values
            .first()
            .ok_or(BoundaryError::Shape("the path start is missing"))?;
        let x = values
            .get(1)
            .copied()
            .ok_or(BoundaryError::Shape("the path start tangent is missing"))?
            .unitize()
            .map_err(BoundaryError::Kernel)?;
        let y = normal.cross(x).unitize().map_err(BoundaryError::Kernel)?;
        Ok(Self {
            origin,
            x,
            y,
            normal,
        })
    }

    /// Lift a 2D path-plane point to 3D.
    #[must_use]
    pub fn lift(self, point: [f64; 2]) -> Point3 {
        self.origin + self.x * point[0] + self.y * point[1]
    }
}

/// The mathematical input of a joint boundary.
pub struct BoundaryInput<'a> {
    /// The kernel.
    pub kernel: &'a Kernel,
    /// The path-plane frame.
    pub frame: PathFrame,
    /// The placed profile at the first segment start.
    pub profile: &'a BSplineCurve,
    /// The first segment (original path).
    pub path_a: &'a BSplineCurve,
    /// The second segment (original path).
    pub path_b: &'a BSplineCurve,
    /// The G1 extension used by the Level A pieces.
    pub extension: f64,
    /// The number of profile contour segments.
    pub segments: usize,
    /// Samples per profile segment.
    pub samples_per_segment: usize,
}

/// A strategy identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoundaryStrategyKind {
    /// Exact analytical miter for any two planar segments.
    ExactMiter,
    /// Exact where it exists, junction tangent continuation on the missing side.
    HybridAtJunction,
    /// AOIM limit-tangent negative control; refuses when a junction is needed.
    HybridAtLimitTangent,
}

/// Diagnostic data of a solved boundary.
#[derive(Clone, Copy, Debug)]
pub struct BoundaryDiagnostics {
    /// The strategy used.
    pub strategy: BoundaryStrategyKind,
    /// The maximum rail residual over the samples.
    pub max_residual: f64,
    /// The number of profile samples.
    pub samples: usize,
    /// The number of profile contour segments.
    pub segments: usize,
    /// Samples whose seam lies on both original offset trajectories.
    pub exact_samples: usize,
    /// Samples whose seam uses at least one tangent extension at the joint.
    pub junction_samples: usize,
    /// The first `alpha` (in profile order) that needs the junction continuation.
    pub first_junction_alpha: Option<f64>,
    /// The profile `alpha` parameter range covered by the boundary.
    pub alpha_range: [f64; 2],
}

/// The shared boundary between two adjacent swept surfaces.
#[derive(Clone, Debug)]
pub struct JointBoundary {
    /// The strategy that produced it.
    pub strategy: BoundaryStrategyKind,
    /// The seam loop split into profile-segment curves, in profile order.
    pub seam_segments: Vec<Curve>,
    /// The shared seam vertices at the profile segment boundaries.
    pub seam_vertices: Vec<Point3>,
    /// Diagnostics.
    pub diagnostics: BoundaryDiagnostics,
}

/// A boundary refusal or failure.
#[derive(Debug)]
pub enum BoundaryError {
    /// The rail solve failed.
    Rail(RailSolveError),
    /// The curve geometry is malformed.
    Geometry(GeometryError),
    /// A kernel call failed.
    Kernel(SolidError),
    /// The input shape is inconsistent.
    Shape(&'static str),
    /// The strategy has no common boundary over the requested profile.
    UncoveredProfile {
        /// The strategy that refused.
        strategy: BoundaryStrategyKind,
        /// The first `alpha` without a common boundary.
        first_missing: f64,
    },
}

impl fmt::Display for BoundaryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Rail(error) => write!(f, "{error}"),
            Self::Geometry(error) => write!(f, "{error}"),
            Self::Kernel(error) => write!(f, "{error}"),
            Self::Shape(reason) => f.write_str(reason),
            Self::UncoveredProfile {
                strategy,
                first_missing,
            } => write!(
                f,
                "{strategy:?} has no common boundary at alpha={first_missing}"
            ),
        }
    }
}

impl std::error::Error for BoundaryError {}

impl From<SolidError> for BoundaryError {
    fn from(value: SolidError) -> Self {
        Self::Kernel(value)
    }
}

impl From<RailSolveError> for BoundaryError {
    fn from(value: RailSolveError) -> Self {
        Self::Rail(value)
    }
}

impl From<GeometryError> for BoundaryError {
    fn from(value: GeometryError) -> Self {
        Self::Geometry(value)
    }
}

/// A mathematical joint-boundary strategy.
pub trait BoundaryStrategy {
    /// The strategy identifier.
    fn kind(&self) -> BoundaryStrategyKind;
    /// Solve the joint boundary, or refuse with a typed error.
    ///
    /// # Errors
    ///
    /// [`BoundaryError`] when the strategy is not applicable or the geometry is
    /// invalid.
    fn solve(&self, input: &BoundaryInput<'_>) -> Result<JointBoundary, BoundaryError>;
}

/// The exact analytical miter strategy.
///
/// It depends only on the Level A differential geometry, not on the primitive
/// names: Line, Arc, Ellipse, Bezier and B-spline joints all go through the
/// same rail solve. The seam must lie on both original offset trajectories; if
/// it needs a tangent extension, the strategy refuses instead of silently
/// becoming a hybrid.
#[derive(Clone, Copy, Debug, Default)]
pub struct ExactMiter;

/// The hybrid-at-junction strategy.
///
/// It keeps the exact common boundary where it exists. On the missing side the
/// seam lies on the incoming piece's end extension and the outgoing piece's
/// start extension: the tangent continuation at the joint. Both extended side
/// surfaces contain the seam, so the direct B-rep still has one shared edge.
#[derive(Clone, Copy, Debug, Default)]
pub struct HybridAtJunction;

/// The AOIM limit-tangent negative control.
///
/// The AOIM probe found that a tangent at the limit vertex is parallel to the
/// line offsets below the limit, so it cannot form a common boundary there.
/// This strategy therefore refuses when the profile needs a junction
/// continuation; it is kept as a typed negative control, not as a fallback.
#[derive(Clone, Copy, Debug, Default)]
pub struct HybridAtLimitTangent;

struct SolvedBoundary {
    alphas: Vec<f64>,
    solutions: Vec<RailSolution>,
    samples: Vec<Vec<(f64, f64)>>,
    corner: [f64; 2],
    max_residual: f64,
    exact_samples: usize,
    junction_samples: usize,
    first_junction_alpha: Option<f64>,
    alpha_range: [f64; 2],
}

fn is_junction(solution: &RailSolution, a_is_linear: bool, b_is_linear: bool) -> bool {
    (solution.segment_a != PieceSegment::Original && !a_is_linear)
        || (solution.segment_b != PieceSegment::Original && !b_is_linear)
}

fn solve_extended(input: &BoundaryInput<'_>) -> Result<SolvedBoundary, BoundaryError> {
    if input.segments < 2 || input.samples_per_segment < 2 {
        return Err(BoundaryError::Shape(
            "the boundary needs at least two segments and two samples",
        ));
    }
    let piece_a = Piece::new(
        KernelCurve::new(
            input.kernel,
            Curve::from(input.path_a.clone()),
            input.frame.origin,
            input.frame.x,
            input.frame.y,
        ),
        input.extension,
        input.extension,
    )?;
    let piece_b = Piece::new(
        KernelCurve::new(
            input.kernel,
            Curve::from(input.path_b.clone()),
            input.frame.origin,
            input.frame.x,
            input.frame.y,
        ),
        input.extension,
        input.extension,
    )?;

    let domain = input.profile.natural_interval();
    let mut alphas = Vec::new();
    let mut samples = Vec::new();
    for index in 0..input.segments {
        let mut segment = Vec::with_capacity(input.samples_per_segment);
        for sample in 0..input.samples_per_segment {
            let t = domain.evaluate(
                (index as f64 + sample as f64 / (input.samples_per_segment - 1) as f64)
                    / input.segments as f64,
            );
            let point = input.profile.evaluate_point(t)?;
            let alpha = (point - input.frame.origin).dot(input.frame.y);
            let beta = (point - input.frame.origin).dot(input.frame.normal);
            alphas.push(alpha);
            segment.push((alpha, beta));
        }
        samples.push(segment);
    }
    let corner = piece_a
        .differential(piece_a.corner_end())
        .map_err(|_| BoundaryError::Shape("the seam corner is not on the first piece"))?
        .0
        .point;
    let a_is_linear = piece_a.is_linear();
    let b_is_linear = piece_b.is_linear();
    let solutions = solve_sequence(&piece_a, &piece_b, &alphas)?;
    let max_residual = solutions
        .iter()
        .map(|solution| {
            (solution.residual[0] * solution.residual[0]
                + solution.residual[1] * solution.residual[1])
                .sqrt()
        })
        .fold(0.0_f64, f64::max);
    let junction_samples = alphas
        .iter()
        .filter(|alpha| {
            solutions.iter().any(|solution| {
                (solution.alpha - **alpha).abs() <= 1.0e-10
                    && is_junction(solution, a_is_linear, b_is_linear)
            })
        })
        .count();
    let first_junction_alpha = alphas.iter().find_map(|alpha| {
        solutions
            .iter()
            .find(|solution| {
                (solution.alpha - *alpha).abs() <= 1.0e-10
                    && is_junction(solution, a_is_linear, b_is_linear)
            })
            .map(|_| *alpha)
    });
    let alpha_range = [
        alphas.iter().copied().fold(f64::INFINITY, f64::min),
        alphas.iter().copied().fold(f64::NEG_INFINITY, f64::max),
    ];
    Ok(SolvedBoundary {
        exact_samples: alphas.len() - junction_samples,
        alphas,
        solutions,
        samples,
        corner,
        max_residual,
        junction_samples,
        first_junction_alpha,
        alpha_range,
    })
}

fn seam_point(
    input: &BoundaryInput<'_>,
    corner: [f64; 2],
    solutions: &[RailSolution],
    alpha: f64,
    beta: f64,
) -> Result<Point3, BoundaryError> {
    let x = if alpha.abs() < 1.0e-12 {
        corner
    } else {
        solutions
            .iter()
            .find(|solution| (solution.alpha - alpha).abs() <= 1.0e-10)
            .ok_or(BoundaryError::Shape("no rail solution for a seam alpha"))?
            .point
    };
    Ok(input.frame.lift(x) + input.frame.normal * beta)
}

fn build_boundary(
    input: &BoundaryInput<'_>,
    solved: &SolvedBoundary,
    strategy: BoundaryStrategyKind,
) -> Result<JointBoundary, BoundaryError> {
    let mut seam_segments = Vec::with_capacity(input.segments);
    let mut seam_vertices = Vec::with_capacity(input.segments);
    for segment in &solved.samples {
        let mut points = Vec::with_capacity(segment.len());
        for &(alpha, beta) in segment {
            points.push(seam_point(
                input,
                solved.corner,
                &solved.solutions,
                alpha,
                beta,
            )?);
        }
        seam_vertices.push(
            *points
                .first()
                .ok_or(BoundaryError::Shape("the seam segment has no points"))?,
        );
        let degree = if points.len() >= 3 { 3 } else { 1 };
        seam_segments.push(Curve::from(BSplineCurve::create_interpolating_curve(
            ParamMethod::ChordLength,
            3,
            degree,
            &points,
            &[],
            None,
            false,
        )?));
    }
    let diagnostics = BoundaryDiagnostics {
        strategy,
        max_residual: solved.max_residual,
        samples: solved.alphas.len(),
        segments: input.segments,
        exact_samples: solved.exact_samples,
        junction_samples: solved.junction_samples,
        first_junction_alpha: solved.first_junction_alpha,
        alpha_range: solved.alpha_range,
    };
    Ok(JointBoundary {
        strategy,
        seam_segments,
        seam_vertices,
        diagnostics,
    })
}

impl BoundaryStrategy for ExactMiter {
    fn kind(&self) -> BoundaryStrategyKind {
        BoundaryStrategyKind::ExactMiter
    }

    fn solve(&self, input: &BoundaryInput<'_>) -> Result<JointBoundary, BoundaryError> {
        let solved = solve_extended(input)?;
        if solved.junction_samples > 0 {
            return Err(BoundaryError::UncoveredProfile {
                strategy: self.kind(),
                first_missing: solved.first_junction_alpha.unwrap_or(f64::NAN),
            });
        }
        build_boundary(input, &solved, self.kind())
    }
}

impl BoundaryStrategy for HybridAtJunction {
    fn kind(&self) -> BoundaryStrategyKind {
        BoundaryStrategyKind::HybridAtJunction
    }

    fn solve(&self, input: &BoundaryInput<'_>) -> Result<JointBoundary, BoundaryError> {
        let solved = solve_extended(input)?;
        build_boundary(input, &solved, self.kind())
    }
}

impl BoundaryStrategy for HybridAtLimitTangent {
    fn kind(&self) -> BoundaryStrategyKind {
        BoundaryStrategyKind::HybridAtLimitTangent
    }

    fn solve(&self, input: &BoundaryInput<'_>) -> Result<JointBoundary, BoundaryError> {
        let solved = solve_extended(input)?;
        if solved.junction_samples > 0 {
            return Err(BoundaryError::UncoveredProfile {
                strategy: self.kind(),
                first_missing: solved.first_junction_alpha.unwrap_or(f64::NAN),
            });
        }
        build_boundary(input, &solved, self.kind())
    }
}

/// The boundary solver: classifies the profile and selects one strategy.
///
/// The classification is part of the same extended rail solve. If every seam
/// sample lies on both original offset trajectories, [`ExactMiter`] is selected.
/// If any sample needs the tangent extension at the joint, [`HybridAtJunction`]
/// is selected. A failed solve or an uncovered extension is a typed refusal;
/// no strategy is attempted and then replaced by another.
#[derive(Clone, Copy, Debug, Default)]
pub struct BoundarySolver {
    /// The exact analytical miter strategy.
    pub exact_miter: ExactMiter,
    /// The exact + junction-tangent strategy.
    pub hybrid_at_junction: HybridAtJunction,
    /// The limit-tangent negative control.
    pub hybrid_at_limit_tangent: HybridAtLimitTangent,
}

impl BoundarySolver {
    /// Solve the joint boundary, or refuse with a typed error.
    ///
    /// # Errors
    ///
    /// [`BoundaryError`] when no strategy is applicable or the geometry is
    /// invalid.
    pub fn solve(&self, input: &BoundaryInput<'_>) -> Result<JointBoundary, BoundaryError> {
        let solved = solve_extended(input)?;
        let strategy = if solved.junction_samples > 0 {
            BoundaryStrategyKind::HybridAtJunction
        } else {
            BoundaryStrategyKind::ExactMiter
        };
        build_boundary(input, &solved, strategy)
    }
}
