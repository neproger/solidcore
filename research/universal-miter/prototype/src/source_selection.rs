//! Source selection for `PlanarSweep2`: exact analytic or Level B approximation.
//!
//! Selection is explicit: a linear path with a supported profile is an exact
//! extrusion; an open circular-arc path with a circular profile is an exact
//! revolution; everything else is Level B. An exact selection that fails to
//! build is returned as an error, never silently replaced by Level B.

use core::fmt;

use solidcore::curve::curve::p2::Curve;
use solidcore::error::{Result, SolidError};
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::geom::extent1d::Extent1d;
use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::tol::ZONE_TOL_3D;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::surface::surface::Surface;

use crate::exact_geometry::{arc_sweep_surface_with_profile, line_sweep_surface};
use crate::level_b_geometry::{SurfaceBuildError, SurfaceBuildOptions, build_swept_surface};
use crate::planar_sweep2::{NormalizedPlacement, NormalizedPlanarSweep, to_bspline};

const OP: &str = "research::source_selection";
const ARC_SAMPLES: u64 = 8;

type SelectionResult<T> = std::result::Result<T, SourceSelectionError>;

/// The geometry source chosen before any surface is built.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SweepSourceKind {
    /// Exact analytic extrusion/cylinder.
    ExactExtrusion,
    /// Exact analytic revolution/torus.
    ExactRevolution,
    /// Controlled Level B NURBS approximation.
    LevelBApproximation,
}

/// Why a piece was classified as it was.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceDiagnostics {
    /// `Line`, `CircleArc` or `Other`.
    pub path_kind: &'static str,
    /// The profile is a closed circular curve.
    pub circular_profile: bool,
    /// Human-readable reason for the choice.
    pub reason: &'static str,
}

/// Where a result piece came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GeometryProvenance {
    /// Original bounded input geometry.
    #[default]
    Original,
    /// Explicitly generated extension geometry.
    Extension,
    /// Explicitly generated bridge/cut transition geometry.
    Bridge,
}

/// Coverage declared by a source geometry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SourceCoverage {
    /// Only the original bounded source path/surface.
    #[default]
    OriginalOnly,
    /// An explicitly represented extension is part of the source geometry.
    WithExtension,
}

/// Relation between the original path and the source path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PathCorrespondence {
    /// Source path is the original path.
    #[default]
    Identity,
    /// Source path has a different domain/geometry; no parameter map is implied.
    Independent,
}

/// A coherent source geometry contract.
#[derive(Debug, Clone, Copy)]
pub struct SourceGeometry<'a> {
    /// The surface built for this piece.
    pub surface: &'a Surface,
    /// The path that underlies the surface.
    pub path: &'a Curve,
    /// The path parameter domain.
    pub path_domain: Extent1d,
    /// The profile used for the boundary problem.
    pub profile: &'a Curve,
    /// The profile parameter domain.
    pub profile_domain: Extent1d,
    /// The bounded surface UV domain.
    pub surface_domain: Extent2d,
    /// Declared coverage.
    pub coverage: SourceCoverage,
    /// Original or generated geometry.
    pub provenance: GeometryProvenance,
    /// Relation to the original path.
    pub path_correspondence: PathCorrespondence,
}

impl SourceGeometry<'_> {
    /// Start/end points and tangents of the declared source path.
    ///
    /// Consumers that need cap frames derive them from this path rather than
    /// from a separate copy of the path endpoints.
    ///
    /// # Errors
    ///
    /// [`SourceSelectionError`] if the path cannot be evaluated.
    pub fn path_ends(&self, kernel: &Kernel) -> SelectionResult<(Point3, Vec3, Point3, Vec3)> {
        Ok(curve_ends(kernel, self.path)?)
    }
}

/// One selected piece: its source, surface, domains and placement data.
#[derive(Debug, Clone)]
pub struct SelectedSweep {
    /// Selected geometry source.
    pub kind: SweepSourceKind,
    /// Whether the piece is original or explicitly generated.
    pub provenance: GeometryProvenance,
    /// The built side surface, with U = path and V = profile.
    pub surface: Surface,
    /// The profile curve at the start of the piece.
    pub profile: Curve,
    /// The declared source path.
    pub path: Curve,
    /// The declared path parameter domain.
    pub path_domain: Extent1d,
    /// Declared coverage.
    pub coverage: SourceCoverage,
    /// Relation to the original path.
    pub path_correspondence: PathCorrespondence,
    /// The path start.
    pub path_start: Point3,
    /// Tangent at the path start.
    pub path_start_tangent: Vec3,
    /// The path end.
    pub path_end: Point3,
    /// Tangent at the path end.
    pub path_end_tangent: Vec3,
    /// The actual surface U domain.
    pub u_domain: Extent2d,
    /// The actual surface V domain.
    pub v_domain: Extent2d,
    /// Whether the profile closes.
    pub profile_closes: bool,
    /// Selection diagnostics.
    pub diagnostics: SourceDiagnostics,
}

impl SelectedSweep {
    /// The coherent geometry view handed to the boundary problem.
    #[must_use]
    pub fn geometry(&self) -> SourceGeometry<'_> {
        SourceGeometry {
            surface: &self.surface,
            path: &self.path,
            path_domain: self.path_domain,
            profile: &self.profile,
            profile_domain: self.profile.natural_interval(),
            surface_domain: self.u_domain,
            coverage: self.coverage,
            provenance: self.provenance,
            path_correspondence: self.path_correspondence,
        }
    }
}

/// A source-selection or source-build failure.
#[derive(Debug)]
pub enum SourceSelectionError {
    /// The exact source was selected but its construction failed.
    ExactBuildFailed {
        /// Piece index.
        index: usize,
        /// Selected kind.
        kind: SweepSourceKind,
        /// Kernel error.
        error: SolidError,
    },
    /// The Level B source failed.
    LevelBBuildFailed {
        /// Piece index.
        index: usize,
        /// Level B error.
        error: SurfaceBuildError,
    },
    /// More than one exact provider claims the same piece.
    ProviderAmbiguous {
        /// Piece index.
        index: usize,
    },
    /// The normalized contract is inconsistent.
    Contract(&'static str),
    /// A Level B build needs a path plane normal.
    MissingPlaneNormal,
    /// A kernel query failed while collecting the source contract.
    Kernel(SolidError),
}

impl From<SolidError> for SourceSelectionError {
    fn from(error: SolidError) -> Self {
        Self::Kernel(error)
    }
}

impl fmt::Display for SourceSelectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ExactBuildFailed { index, kind, error } => {
                write!(f, "exact {kind:?} source failed for piece {index}: {error}")
            }
            Self::LevelBBuildFailed { index, error } => {
                write!(f, "Level B source failed for piece {index}: {error}")
            }
            Self::ProviderAmbiguous { index } => {
                write!(f, "multiple exact source providers apply to piece {index}")
            }
            Self::Contract(reason) => f.write_str(reason),
            Self::MissingPlaneNormal => f.write_str("Level B needs a path plane normal"),
            Self::Kernel(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for SourceSelectionError {}

/// A source-provider request for one normalized piece.
pub struct SourceInput<'a> {
    /// Kernel.
    pub kernel: &'a Kernel,
    /// Normalized sweep.
    pub normalized: &'a NormalizedPlanarSweep,
    /// The piece placement.
    pub placement: &'a NormalizedPlacement,
    /// Piece index.
    pub index: usize,
    /// Level B options.
    pub options: &'a SurfaceBuildOptions,
}

/// Whether a provider applies to a source request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceSupport {
    /// The provider applies and may build.
    Applicable,
    /// The provider does not apply.
    NotApplicable,
    /// The geometry is explicitly unsupported by this provider.
    Unsupported,
    /// The provider is an explicit fallback when no exact provider applies.
    Fallback,
}

/// A source-geometry provider.
pub trait SourceProvider {
    /// The source kind this provider builds.
    fn kind(&self) -> SweepSourceKind;
    /// Classify without building.
    fn supports(&self, input: &SourceInput<'_>) -> SourceSupport;
    /// Build the source. A failure of an applicable provider is final.
    ///
    /// # Errors
    ///
    /// [`SourceSelectionError`] when the selected provider cannot build.
    fn build(&self, input: &SourceInput<'_>) -> SelectionResult<SelectedSweep>;
}

/// Exact extrusion/cylinder provider.
#[derive(Debug, Default, Clone, Copy)]
pub struct ExactExtrusionProvider;

/// Exact revolution/Torus provider.
#[derive(Debug, Default, Clone, Copy)]
pub struct ExactRevolutionProvider;

/// Level B fallback provider.
#[derive(Debug, Default, Clone, Copy)]
pub struct LevelBProvider;

/// Explicit source-provider registry.
pub struct SourceProviderRegistry {
    providers: Vec<Box<dyn SourceProvider>>,
}

impl Default for SourceProviderRegistry {
    fn default() -> Self {
        Self {
            providers: vec![
                Box::new(ExactExtrusionProvider),
                Box::new(ExactRevolutionProvider),
                Box::new(LevelBProvider),
            ],
        }
    }
}

impl SourceProviderRegistry {
    /// The first applicable or fallback provider, or `None`.
    ///
    /// # Errors
    ///
    /// [`SourceSelectionError::ProviderAmbiguous`] when more than one exact
    /// provider claims the same piece.
    pub fn provider_for<'a>(
        &'a self,
        input: &SourceInput<'_>,
    ) -> SelectionResult<Option<&'a dyn SourceProvider>> {
        let mut exact = None;
        let mut fallback = None;
        for provider in self.providers.iter().map(Box::as_ref) {
            match provider.supports(input) {
                SourceSupport::Applicable => {
                    if exact.is_some() {
                        return Err(SourceSelectionError::ProviderAmbiguous { index: input.index });
                    }
                    exact = Some(provider);
                }
                SourceSupport::Fallback => {
                    if fallback.is_none() {
                        fallback = Some(provider);
                    }
                }
                SourceSupport::NotApplicable | SourceSupport::Unsupported => {}
            }
        }
        Ok(exact.or(fallback))
    }
}

/// Classify one normalized piece without building a surface.
#[must_use]
pub fn classify_piece(
    k: &Kernel,
    normalized: &NormalizedPlanarSweep,
    index: usize,
) -> SweepSourceKind {
    let Some(placement) = normalized.placements.get(index) else {
        return SweepSourceKind::LevelBApproximation;
    };
    let options = SurfaceBuildOptions::default();
    let input = SourceInput {
        kernel: k,
        normalized,
        placement,
        index,
        options: &options,
    };
    SourceProviderRegistry::default()
        .provider_for(&input)
        .ok()
        .flatten()
        .map_or(SweepSourceKind::LevelBApproximation, SourceProvider::kind)
}

fn is_exact_extrusion_candidate(input: &SourceInput<'_>) -> bool {
    if input.normalized.closed
        || input.placement.world_profiles.len() != 1
        || input.normalized.profiles_original.len() != 1
    {
        return false;
    }
    let path = classification_path(input.normalized, input.index, input.placement);
    path.is_linear(input.kernel, ZONE_TOL_3D, false)
        .is_ok_and(|linear| linear.linear)
}

fn is_exact_revolution_candidate(input: &SourceInput<'_>) -> bool {
    if input.normalized.closed
        || input.placement.world_profiles.len() != 1
        || input.normalized.profiles_original.len() != 1
    {
        return false;
    }
    let Some(profile) = input.normalized.profiles_original.first() else {
        return false;
    };
    if !is_circular_profile(input.kernel, profile) {
        return false;
    }
    let path = classification_path(input.normalized, input.index, input.placement);
    path.is_arc(input.kernel, ARC_SAMPLES, ZONE_TOL_3D)
        .is_ok_and(|arc| arc.is_some())
}

fn finalize_selected(
    input: &SourceInput<'_>,
    kind: SweepSourceKind,
    surface: Surface,
) -> SelectionResult<SelectedSweep> {
    let domain = surface
        .natural_uv_domain_with_nurb(input.kernel)
        .map_err(SourceSelectionError::Kernel)?;
    let contract_path = if kind == SweepSourceKind::ExactRevolution {
        classification_path(input.normalized, input.index, input.placement)
    } else {
        Curve::from(input.placement.path_piece.clone())
    };
    let (path_start, path_start_tangent, path_end, path_end_tangent) =
        curve_ends(input.kernel, &contract_path)?;
    let path_domain = contract_path.natural_interval();
    let coverage = if kind == SweepSourceKind::ExactRevolution {
        SourceCoverage::OriginalOnly
    } else {
        SourceCoverage::WithExtension
    };
    let path_correspondence = if kind == SweepSourceKind::ExactRevolution {
        PathCorrespondence::Identity
    } else {
        PathCorrespondence::Independent
    };
    let profile = if kind == SweepSourceKind::ExactRevolution
        && all_arc_paths(input.kernel, input.normalized)
    {
        original_arc_profile(input.kernel, input.placement, &contract_path)?
    } else {
        input
            .placement
            .world_profiles
            .first()
            .cloned()
            .ok_or(SourceSelectionError::Contract("missing world profile"))?
    };
    Ok(SelectedSweep {
        kind,
        provenance: GeometryProvenance::Original,
        surface,
        profile,
        path: contract_path,
        path_domain,
        coverage,
        path_correspondence,
        path_start,
        path_start_tangent,
        path_end,
        path_end_tangent,
        u_domain: domain,
        v_domain: domain,
        profile_closes: input.normalized.caps.profile_closes,
        diagnostics: diagnostics_for(input.kernel, input.normalized, input.index, kind),
    })
}

impl SourceProvider for ExactExtrusionProvider {
    fn kind(&self) -> SweepSourceKind {
        SweepSourceKind::ExactExtrusion
    }

    fn supports(&self, input: &SourceInput<'_>) -> SourceSupport {
        if is_exact_extrusion_candidate(input) {
            SourceSupport::Applicable
        } else {
            SourceSupport::NotApplicable
        }
    }

    fn build(&self, input: &SourceInput<'_>) -> SelectionResult<SelectedSweep> {
        let surface = build_exact_extrusion(input.kernel, input.placement, input.index)?;
        finalize_selected(input, self.kind(), surface)
    }
}

impl SourceProvider for ExactRevolutionProvider {
    fn kind(&self) -> SweepSourceKind {
        SweepSourceKind::ExactRevolution
    }

    fn supports(&self, input: &SourceInput<'_>) -> SourceSupport {
        if is_exact_revolution_candidate(input) {
            SourceSupport::Applicable
        } else {
            SourceSupport::NotApplicable
        }
    }

    fn build(&self, input: &SourceInput<'_>) -> SelectionResult<SelectedSweep> {
        let surface =
            build_exact_revolution(input.kernel, input.normalized, input.placement, input.index)?;
        finalize_selected(input, self.kind(), surface)
    }
}

impl SourceProvider for LevelBProvider {
    fn kind(&self) -> SweepSourceKind {
        SweepSourceKind::LevelBApproximation
    }

    fn supports(&self, _input: &SourceInput<'_>) -> SourceSupport {
        SourceSupport::Fallback
    }

    fn build(&self, input: &SourceInput<'_>) -> SelectionResult<SelectedSweep> {
        let surface = build_level_b(
            input.kernel,
            input.normalized,
            input.placement,
            input.index,
            input.options,
        )?;
        finalize_selected(input, self.kind(), surface)
    }
}

/// Build every selected piece. Exact failures are diagnostics, not fallbacks.
///
/// # Errors
///
/// [`SourceSelectionError`] when the normalized contract is inconsistent, an
/// exact source fails to build, or a Level B source fails.
pub fn select_sweep_sources(
    k: &Kernel,
    normalized: &NormalizedPlanarSweep,
    options: &SurfaceBuildOptions,
) -> SelectionResult<Vec<SelectedSweep>> {
    let registry = SourceProviderRegistry::default();
    let mut selected = Vec::with_capacity(normalized.placements.len());
    for (index, placement) in normalized.placements.iter().enumerate() {
        let input = SourceInput {
            kernel: k,
            normalized,
            placement,
            index,
            options,
        };
        let provider = registry
            .provider_for(&input)?
            .ok_or(SourceSelectionError::Contract("no source provider"))?;
        selected.push(provider.build(&input)?);
    }
    Ok(selected)
}

fn diagnostics_for(
    k: &Kernel,
    normalized: &NormalizedPlanarSweep,
    index: usize,
    kind: SweepSourceKind,
) -> SourceDiagnostics {
    let path_kind = normalized
        .placements
        .get(index)
        .map_or("MissingPath", |placement| {
            let path = classification_path(normalized, index, placement);
            if path
                .is_linear(k, ZONE_TOL_3D, false)
                .is_ok_and(|linear| linear.linear)
            {
                "Line"
            } else if path
                .is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
                .is_ok_and(|arc| arc.is_some())
            {
                "CircleArc"
            } else {
                "Other"
            }
        });
    let circular_profile = normalized
        .profiles_original
        .first()
        .is_some_and(|profile| is_circular_profile(k, profile));
    let reason = match kind {
        SweepSourceKind::ExactExtrusion => "line path and supported profile",
        SweepSourceKind::ExactRevolution => "circle arc and circular profile",
        SweepSourceKind::LevelBApproximation if normalized.closed => "closed path is deferred",
        SweepSourceKind::LevelBApproximation if !circular_profile => "profile is not a circle",
        SweepSourceKind::LevelBApproximation => "unsupported path/profile combination",
    };
    SourceDiagnostics {
        path_kind,
        circular_profile,
        reason,
    }
}

fn classification_path(
    normalized: &NormalizedPlanarSweep,
    index: usize,
    placement: &NormalizedPlacement,
) -> Curve {
    normalized
        .path_original
        .get(index)
        .cloned()
        .unwrap_or_else(|| Curve::from(placement.path_piece.clone()))
}

fn build_exact_extrusion(
    k: &Kernel,
    placement: &NormalizedPlacement,
    index: usize,
) -> SelectionResult<Surface> {
    let (start, _, end, _) = path_ends(&placement.path_piece)?;
    let profile = placement
        .world_profiles
        .first()
        .ok_or(SourceSelectionError::Contract("missing world profile"))?;
    line_sweep_surface(k, profile, start, end).map_err(|error| {
        SourceSelectionError::ExactBuildFailed {
            index,
            kind: SweepSourceKind::ExactExtrusion,
            error,
        }
    })
}

fn all_arc_paths(k: &Kernel, normalized: &NormalizedPlanarSweep) -> bool {
    !normalized.path_original.is_empty()
        && normalized.path_original.iter().all(|path| {
            path.is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
                .is_ok_and(|arc| arc.is_some())
        })
}

fn original_arc_profile(
    k: &Kernel,
    placement: &NormalizedPlacement,
    path: &Curve,
) -> SelectionResult<Curve> {
    let Some(_frame) = path
        .is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
        .map_err(SourceSelectionError::Kernel)?
    else {
        return Err(SourceSelectionError::Contract(
            "arc source without an arc path",
        ));
    };
    let profile = placement
        .world_profiles
        .first()
        .cloned()
        .ok_or(SourceSelectionError::Contract("missing world profile"))?;
    let prepared_start = path_ends(&placement.path_piece)?.0;
    let original_start = curve_ends(k, path)?.0;
    let translation = original_start - prepared_start;
    if translation.length() <= 1.0e-12 {
        return Ok(profile);
    }
    let mut move_placement = Axis2Placement::IDENTITY;
    move_placement.translate(translation);
    let mut profile = profile;
    profile.transform(k, &move_placement, None)?;
    Ok(profile)
}

fn build_exact_revolution(
    k: &Kernel,
    normalized: &NormalizedPlanarSweep,
    placement: &NormalizedPlacement,
    index: usize,
) -> SelectionResult<Surface> {
    let path = classification_path(normalized, index, placement);
    let Some((frame, radius, start_deg, end_deg)) = path
        .is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
        .map_err(SourceSelectionError::Kernel)?
    else {
        return Err(SourceSelectionError::Contract(
            "arc source without an arc path",
        ));
    };
    let profile = if all_arc_paths(k, normalized) {
        original_arc_profile(k, placement, &path)?
    } else {
        placement
            .world_profiles
            .first()
            .cloned()
            .ok_or(SourceSelectionError::Contract("missing world profile"))?
    };
    let Some((_, profile_radius, _, _)) = profile
        .is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
        .map_err(SourceSelectionError::Kernel)?
    else {
        return Err(SourceSelectionError::Contract(
            "arc source without a circular profile",
        ));
    };
    let start = start_deg.to_radians();
    let radial = frame.x_axis() * start.cos() + frame.y_axis() * start.sin();
    let tangent = frame.x_axis() * -start.sin() + frame.y_axis() * start.cos();
    let mut span = end_deg - start_deg;
    if span <= 1.0e-12 {
        span += 360.0;
    }
    arc_sweep_surface_with_profile(
        k,
        Some(&profile),
        frame.origin(),
        radial,
        tangent,
        radius,
        profile_radius,
        0.0,
        span,
    )
    .map_err(|error| SourceSelectionError::ExactBuildFailed {
        index,
        kind: SweepSourceKind::ExactRevolution,
        error,
    })
}

fn build_level_b(
    k: &Kernel,
    normalized: &NormalizedPlanarSweep,
    placement: &NormalizedPlacement,
    index: usize,
    options: &SurfaceBuildOptions,
) -> SelectionResult<Surface> {
    let normal = normalized
        .plane_normal
        .ok_or(SourceSelectionError::MissingPlaneNormal)?;
    let profile = placement
        .world_profiles
        .first()
        .ok_or(SourceSelectionError::Contract("missing world profile"))?;
    let profile = to_bspline(profile)?;
    build_swept_surface(k, &placement.path_piece, &profile, normal, options)
        .map(|built| Surface::from(built.surface))
        .map_err(|error| SourceSelectionError::LevelBBuildFailed { index, error })
}

fn is_circular_profile(k: &Kernel, profile: &Curve) -> bool {
    profile
        .is_arc(k, ARC_SAMPLES, ZONE_TOL_3D)
        .is_ok_and(|arc| arc.is_some())
}

fn path_ends(path: &BSplineCurve) -> Result<(Point3, Vec3, Point3, Vec3)> {
    curve_ends_without_kernel(&Curve::from(path.clone()))
}

fn curve_ends(k: &Kernel, path: &Curve) -> Result<(Point3, Vec3, Point3, Vec3)> {
    let start_values = path.evaluate(k, path.natural_interval().min(), 1, true, false)?;
    let end_values = path.evaluate(k, path.natural_interval().max(), 1, true, false)?;
    let start = *start_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
    let start_tangent = *start_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path start tangent missing"))?;
    let end = *end_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path end missing"))?;
    let end_tangent = *end_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path end tangent missing"))?;
    Ok((start, start_tangent, end, end_tangent))
}

fn curve_ends_without_kernel(path: &Curve) -> Result<(Point3, Vec3, Point3, Vec3)> {
    let solidcore::curve::curve::p2::CurveKind::BSpline(path) = path.kind() else {
        return Err(SolidError::invalid_input(OP, "path is not a B-spline"));
    };
    path_ends_without_kernel(path)
}

fn path_ends_without_kernel(path: &BSplineCurve) -> Result<(Point3, Vec3, Point3, Vec3)> {
    let start_values = path.evaluate(path.natural_interval().min(), 1, true, false)?;
    let end_values = path.evaluate(path.natural_interval().max(), 1, true, false)?;
    let start = *start_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
    let start_tangent = *start_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path start tangent missing"))?;
    let end = *end_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path end missing"))?;
    let end_tangent = *end_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path end tangent missing"))?;
    Ok((start, start_tangent, end, end_tangent))
}
