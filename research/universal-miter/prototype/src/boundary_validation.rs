//! Independent validation of a solved joint boundary against the actual source
//! surfaces.
//!
//! This module deliberately does not use the rail solver or its residual. It
//! samples the candidate seam, projects each sample onto both bounded source
//! surfaces, and checks UV-domain membership, continuity and shared vertices.
//! The Boundary Solver itself is unchanged.

use core::fmt;

use solidcore::error::SolidError;
use solidcore::kernel::Kernel;
use solidcore::math::{Point2, Point3};
use solidcore::surface::surface::Surface;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;

use crate::boundary::JointBoundary;

/// An independent validation failure of a candidate joint boundary.
#[derive(Debug)]
pub enum BoundaryValidationError {
    /// The boundary has no seam segment.
    EmptyBoundary,
    /// A source surface domain was not available.
    SourceDomainMissing {
        /// 0 for the first surface, 1 for the second.
        surface: usize,
    },
    /// A seam sample is not on one of the source surfaces.
    SurfaceMembershipFailure {
        /// Segment index.
        segment: usize,
        /// Sample index in the segment.
        sample: usize,
        /// 0 for the first surface, 1 for the second.
        surface: usize,
        /// Measured projection gap.
        gap: f64,
    },
    /// A seam sample projects to a point outside the source UV domain.
    OutsideSourceDomain {
        /// Segment index.
        segment: usize,
        /// Sample index in the segment.
        sample: usize,
        /// 0 for the first surface, 1 for the second.
        surface: usize,
        /// Projected UV.
        uv: Point2,
    },
    /// Consecutive seam segments do not meet within tolerance.
    DiscontinuousBoundary {
        /// Segment index whose end was checked.
        segment: usize,
        /// End-to-next-start gap.
        gap: f64,
    },
    /// A stored seam vertex does not match its segment endpoint.
    SharedVertexMismatch {
        /// Seam vertex index.
        vertex: usize,
        /// Stored vertex to segment-start gap.
        gap: f64,
    },
    /// The seam loop does not close within tolerance.
    OpenSeamLoop {
        /// Last-to-first gap.
        gap: f64,
    },
    /// A kernel query failed while validating.
    Kernel(SolidError),
}

impl From<SolidError> for BoundaryValidationError {
    fn from(error: SolidError) -> Self {
        Self::Kernel(error)
    }
}

impl fmt::Display for BoundaryValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyBoundary => f.write_str("no seam segment"),
            Self::SourceDomainMissing { surface } => {
                write!(f, "source surface {surface} has no domain")
            }
            Self::SurfaceMembershipFailure {
                segment,
                sample,
                surface,
                gap,
            } => write!(
                f,
                "seam sample {sample} of segment {segment} is not on surface {surface}: gap={gap:e}"
            ),
            Self::OutsideSourceDomain {
                segment,
                sample,
                surface,
                uv,
            } => write!(
                f,
                "seam sample {sample} of segment {segment} leaves surface {surface} domain: uv={uv:?}"
            ),
            Self::DiscontinuousBoundary { segment, gap } => {
                write!(f, "seam segment {segment} to next gap={gap:e}")
            }
            Self::SharedVertexMismatch { vertex, gap } => {
                write!(f, "seam vertex {vertex} to segment start gap={gap:e}")
            }
            Self::OpenSeamLoop { gap } => write!(f, "seam loop closes with gap={gap:e}"),
            Self::Kernel(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for BoundaryValidationError {}

/// Successful independent boundary validation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundaryValidation {
    /// Largest source-surface projection gap over all samples.
    pub max_gap: f64,
    /// Number of seam samples checked.
    pub samples: usize,
    /// Largest gap between consecutive segment endpoints.
    pub max_join_gap: f64,
    /// Largest stored seam vertex to segment-start gap.
    pub max_vertex_gap: f64,
    /// Last-to-first seam-loop closure gap.
    pub loop_gap: f64,
}

/// Validate `boundary` against the two bounded source surfaces.
///
/// `samples_per_segment` includes both segment endpoints. The function is
/// independent of the Boundary Solver's rail equations; it checks only the
/// resulting curves and the actual source surfaces.
///
/// # Errors
///
/// [`BoundaryValidationError`] when membership, domain, continuity or shared
/// vertices fail.
pub fn validate_joint_boundary(
    k: &Kernel,
    boundary: &JointBoundary,
    surface_a: &Surface,
    surface_b: &Surface,
    tolerance: f64,
    samples_per_segment: usize,
) -> Result<BoundaryValidation, BoundaryValidationError> {
    if boundary.seam_segments.is_empty() {
        return Err(BoundaryValidationError::EmptyBoundary);
    }
    let surfaces = [surface_a, surface_b];
    let domains = [
        surface_a.natural_uv_domain_with_nurb(k)?,
        surface_b.natural_uv_domain_with_nurb(k)?,
    ];
    let samples_per_segment = samples_per_segment.max(2);
    let mut max_gap = 0.0_f64;
    let mut samples = 0usize;
    let mut first_start: Option<Point3> = None;
    let mut previous_end: Option<Point3> = None;
    let mut max_join_gap = 0.0_f64;

    for (segment_index, segment) in boundary.seam_segments.iter().enumerate() {
        let interval = segment.natural_interval();
        let mut segment_start: Option<Point3> = None;
        let mut segment_end: Option<Point3> = None;
        for sample in 0..samples_per_segment {
            let t = interval.min()
                + (interval.max() - interval.min())
                    * (sample as f64 / (samples_per_segment - 1) as f64);
            let point = segment.evaluate_point(k, t)?;
            if sample == 0 {
                segment_start = Some(point);
            }
            if sample + 1 == samples_per_segment {
                segment_end = Some(point);
            }
            for (surface_index, surface) in surfaces.iter().enumerate() {
                let domain = domains.get(surface_index).ok_or(
                    BoundaryValidationError::SourceDomainMissing {
                        surface: surface_index,
                    },
                )?;
                let drop = surface.drop_point::<NoObject>(
                    k,
                    point,
                    domain,
                    &DropPointOptions::default(),
                )?;
                if !drop.success || drop.gap > tolerance {
                    return Err(BoundaryValidationError::SurfaceMembershipFailure {
                        segment: segment_index,
                        sample,
                        surface: surface_index,
                        gap: drop.gap,
                    });
                }
                if !domain.contains_point(drop.result_uv, tolerance) {
                    return Err(BoundaryValidationError::OutsideSourceDomain {
                        segment: segment_index,
                        sample,
                        surface: surface_index,
                        uv: drop.result_uv,
                    });
                }
                max_gap = max_gap.max(drop.gap);
            }
            samples += 1;
        }
        if let (Some(start), None) = (segment_start, first_start) {
            first_start = Some(start);
        }
        if let Some(previous) = previous_end {
            if let Some(start) = segment_start {
                let gap = (previous - start).length();
                max_join_gap = max_join_gap.max(gap);
                if gap > tolerance {
                    return Err(BoundaryValidationError::DiscontinuousBoundary {
                        segment: segment_index - 1,
                        gap,
                    });
                }
            }
        }
        previous_end = segment_end;
        if let Some(vertex) = boundary.seam_vertices.get(segment_index) {
            if let Some(start) = segment_start {
                let gap = (*vertex - start).length();
                if gap > tolerance {
                    return Err(BoundaryValidationError::SharedVertexMismatch {
                        vertex: segment_index,
                        gap,
                    });
                }
            }
        }
    }
    let first_start = first_start.ok_or(BoundaryValidationError::EmptyBoundary)?;
    let last_end = previous_end.ok_or(BoundaryValidationError::EmptyBoundary)?;
    let loop_gap = (first_start - last_end).length();
    if loop_gap > tolerance {
        return Err(BoundaryValidationError::OpenSeamLoop { gap: loop_gap });
    }
    let max_vertex_gap = boundary
        .seam_vertices
        .iter()
        .zip(boundary.seam_segments.iter())
        .map(|(vertex, segment)| {
            let interval = segment.natural_interval();
            segment
                .evaluate_point(k, interval.min())
                .map_or(f64::INFINITY, |start| (*vertex - start).length())
        })
        .fold(0.0_f64, f64::max);
    Ok(BoundaryValidation {
        max_gap,
        samples,
        max_join_gap,
        max_vertex_gap,
        loop_gap,
    })
}
