//! High-level `PlanarSweep2` pipeline: normalize, select sources, solve joints,
//! build the direct B-rep and return the source/boundary diagnostics.

use core::fmt;

use solidcore::curve::curve::p2::Curve;
use solidcore::error::{Result, SolidError};
use solidcore::kernel::Kernel;
use solidcore::topology::Brep;

pub use crate::boundary::GeometryPolicy;
use crate::boundary::{
    BoundaryError, BoundaryInput, BoundaryProblem, BoundarySolver, JointBoundary, PathFrame,
};
use crate::boundary_validation::{BoundaryValidationError, validate_joint_boundary};
use crate::direct_brep::build_direct_chain_surfaces;
use crate::level_b_geometry::SurfaceBuildOptions;
use crate::planar_sweep2::{normalize_planar_sweep, to_bspline};
use crate::source_selection::{SelectedSweep, SourceSelectionError, select_sweep_sources};

/// The default number of profile contour segments used by the pipeline.
pub const DEFAULT_SEGMENTS: usize = 4;

/// The finished direct B-rep and the decisions that produced it.
#[derive(Debug)]
pub struct PlanarSweepResult {
    /// Policy used for the result.
    pub geometry_policy: GeometryPolicy,
    /// The direct B-rep solid.
    pub brep: Brep,
    /// Each piece's source, surface contract and diagnostics.
    pub sources: Vec<SelectedSweep>,
    /// Each joint's solved boundary and diagnostics.
    pub boundaries: Vec<JointBoundary>,
}

/// A high-level `PlanarSweep2` failure, attributed to its stage.
#[derive(Debug)]
pub enum PlanarSweepError {
    /// Input normalization failed.
    Normalize(SolidError),
    /// Source selection or source construction failed.
    Source(SourceSelectionError),
    /// Boundary solving failed.
    Boundary(BoundaryError),
    /// Independent boundary validation rejected the solver candidate.
    BoundaryValidation(BoundaryValidationError),
    /// Direct topology construction failed.
    Chain(SolidError),
    /// A requested generated-geometry policy is not implemented.
    StrategyNotImplemented(GeometryPolicy),
    /// The normalized input has no pieces.
    Empty,
}

impl From<SolidError> for PlanarSweepError {
    fn from(error: SolidError) -> Self {
        Self::Normalize(error)
    }
}

impl From<SourceSelectionError> for PlanarSweepError {
    fn from(error: SourceSelectionError) -> Self {
        Self::Source(error)
    }
}

impl From<BoundaryError> for PlanarSweepError {
    fn from(error: BoundaryError) -> Self {
        Self::Boundary(error)
    }
}

impl From<BoundaryValidationError> for PlanarSweepError {
    fn from(error: BoundaryValidationError) -> Self {
        Self::BoundaryValidation(error)
    }
}

impl fmt::Display for PlanarSweepError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Normalize(error) => write!(f, "normalize: {error}"),
            Self::Source(error) => write!(f, "source: {error}"),
            Self::Boundary(error) => write!(f, "boundary: {error}"),
            Self::BoundaryValidation(error) => write!(f, "boundary validation: {error}"),
            Self::Chain(error) => write!(f, "chain: {error}"),
            Self::StrategyNotImplemented(policy) => {
                write!(f, "geometry policy {policy:?} is not implemented")
            }
            Self::Empty => f.write_str("the normalized input has no pieces"),
        }
    }
}

impl std::error::Error for PlanarSweepError {}

/// Normalize the input, choose a source for every piece, solve every joint with
/// the existing Boundary Solver, and build the direct chain.
///
/// Source selection and boundary strategy are independent decisions. An exact
/// source failure is returned as [`PlanarSweepError::Source`]; it is never
/// replaced by Level B. A boundary failure is not retried with another strategy.
///
/// # Errors
///
/// [`PlanarSweepError`] for the failing stage.
pub fn build_planar_sweep(
    k: &mut Kernel,
    profiles: &[Curve],
    paths: &[Curve],
    move_profile: bool,
    cap_ends: i32,
    segments: usize,
    options: &SurfaceBuildOptions,
) -> std::result::Result<PlanarSweepResult, PlanarSweepError> {
    let normalized = normalize_planar_sweep(k, profiles, paths, move_profile, cap_ends)?;
    let sources = select_sweep_sources(k, &normalized, options)?;
    if sources.is_empty() {
        return Err(PlanarSweepError::Empty);
    }

    let frame = PathFrame::new(k, &normalized.path_original)?;
    let boundary_profile = sources
        .first()
        .map(|piece| &piece.profile)
        .ok_or(PlanarSweepError::Empty)
        .and_then(|profile| to_bspline(profile).map_err(PlanarSweepError::Normalize))?;

    let mut path_bspline = Vec::with_capacity(normalized.path_original.len());
    for path in &normalized.path_original {
        path_bspline.push(to_bspline(path).map_err(PlanarSweepError::Normalize)?);
    }

    let mut boundaries = Vec::with_capacity(path_bspline.len().saturating_sub(1));
    for joint in 0..path_bspline.len().saturating_sub(1) {
        let path_a = path_bspline.get(joint).ok_or(PlanarSweepError::Empty)?;
        let path_b = path_bspline.get(joint + 1).ok_or(PlanarSweepError::Empty)?;
        let geometry_a = sources
            .get(joint)
            .ok_or(PlanarSweepError::Empty)?
            .geometry();
        let geometry_b = sources
            .get(joint + 1)
            .ok_or(PlanarSweepError::Empty)?
            .geometry();
        let problem = BoundaryProblem {
            input: BoundaryInput {
                kernel: k,
                frame,
                profile: &boundary_profile,
                path_a,
                path_b,
                extension: normalized.extension,
                segments,
                samples_per_segment: 17,
            },
            source_a: &geometry_a,
            source_b: &geometry_b,
            policy: GeometryPolicy::ExactOnly,
        };
        boundaries.push(BoundarySolver::default().solve_problem(&problem)?);
    }

    for (joint, boundary) in boundaries.iter().enumerate() {
        let geometry_a = sources
            .get(joint)
            .ok_or(PlanarSweepError::Empty)?
            .geometry();
        let geometry_b = sources
            .get(joint + 1)
            .ok_or(PlanarSweepError::Empty)?
            .geometry();
        validate_joint_boundary(
            k,
            boundary,
            geometry_a.surface,
            geometry_b.surface,
            options.tolerance,
            17,
        )?;
    }

    let first = sources.first().ok_or(PlanarSweepError::Empty)?;
    let last = sources.last().ok_or(PlanarSweepError::Empty)?;
    let first_geometry = first.geometry();
    let last_geometry = last.geometry();
    let (path_start, path_start_tangent, _, _) = first_geometry
        .path_ends(k)
        .map_err(PlanarSweepError::Source)?;
    let (_, _, path_end, path_end_tangent) = last_geometry
        .path_ends(k)
        .map_err(PlanarSweepError::Source)?;
    let surfaces: Vec<_> = sources
        .iter()
        .map(|piece| piece.geometry().surface.clone())
        .collect();
    let brep = build_direct_chain_surfaces(
        k,
        &surfaces,
        &boundaries,
        path_start,
        path_start_tangent,
        path_end,
        path_end_tangent,
        segments,
    )
    .map_err(PlanarSweepError::Chain)?;

    Ok(PlanarSweepResult {
        geometry_policy: GeometryPolicy::ExactOnly,
        brep,
        sources,
        boundaries,
    })
}

/// Build with an explicit geometry policy.
///
/// `ExactOnly` is the validated base pipeline. Extension and adaptive-bridge
/// geometry are not implemented yet, so those policies return
/// [`PlanarSweepError::StrategyNotImplemented`] instead of silently falling
/// back to Exact or another strategy.
///
/// # Errors
///
/// [`PlanarSweepError`] for the failing stage or an unimplemented policy.
pub fn build_planar_sweep_with_policy(
    k: &mut Kernel,
    profiles: &[Curve],
    paths: &[Curve],
    move_profile: bool,
    cap_ends: i32,
    segments: usize,
    options: &SurfaceBuildOptions,
    policy: GeometryPolicy,
) -> std::result::Result<PlanarSweepResult, PlanarSweepError> {
    if policy != GeometryPolicy::ExactOnly {
        return Err(PlanarSweepError::StrategyNotImplemented(policy));
    }
    build_planar_sweep(
        k,
        profiles,
        paths,
        move_profile,
        cap_ends,
        segments,
        options,
    )
}

/// Normalize only, without selecting sources or building.
///
/// # Errors
///
/// The normalization errors.
pub fn normalized_input(
    k: &Kernel,
    profiles: &[Curve],
    paths: &[Curve],
    move_profile: bool,
    cap_ends: i32,
) -> Result<crate::planar_sweep2::NormalizedPlanarSweep> {
    normalize_planar_sweep(k, profiles, paths, move_profile, cap_ends)
}
