//! Level B geometry primitives: the real-curve adapter and the controlled swept
//! surface builder. No joint, trimming, caps or B-rep is built here.

use core::fmt;

use solidcore::curve::curve::Curve;
use solidcore::error::SolidError;
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::kernel::Kernel;
use solidcore::math::{Point2, Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::b_spline_surface::{BSplineSurface, SurfParamType};
use solidcore::nurbs::b_spline_surface_n_lib::p2::SkinnedSurfaceOptions;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;
use solidcore::surface::surface::Surface;

use crate::curve::GeometryError;
use crate::differential_geometry::{cross, norm, rot90, unit, DifferentialData};
use crate::rail_solver::PlanarCurve;

/// The real `solidcore` curve adapted to the Level A planar interface.
///
/// The curve's natural parameter is preserved unchanged. The 3D point and
/// derivatives are projected onto the local path-plane axes; the kernel's
/// rational weights are kept by `Curve::evaluate`.
#[derive(Clone)]
pub struct KernelCurve<'a> {
    kernel: &'a Kernel,
    curve: Curve,
    origin: Point3,
    x_axis: Vec3,
    y_axis: Vec3,
}

impl<'a> KernelCurve<'a> {
    /// Adapt `curve` to the local plane `(origin, x_axis, y_axis)`.
    pub fn new(
        kernel: &'a Kernel,
        curve: Curve,
        origin: Point3,
        x_axis: Vec3,
        y_axis: Vec3,
    ) -> Self {
        Self {
            kernel,
            curve,
            origin,
            x_axis,
            y_axis,
        }
    }

    /// The kernel's underlying curve.
    #[must_use]
    pub fn curve(&self) -> &Curve {
        &self.curve
    }
}

impl fmt::Debug for KernelCurve<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("KernelCurve")
            .field("curve", &self.curve)
            .finish()
    }
}

impl PlanarCurve for KernelCurve<'_> {
    fn natural_interval(&self) -> Result<[f64; 2], GeometryError> {
        let interval = self.curve.natural_interval();
        Ok([interval.min(), interval.max()])
    }

    fn differential(&self, u: f64) -> Result<DifferentialData, GeometryError> {
        let values = self
            .curve
            .evaluate(self.kernel, u, 2, true, false)
            .map_err(|_| GeometryError::InvalidCurve("kernel curve evaluation"))?;
        let point = *values
            .first()
            .ok_or(GeometryError::InvalidCurve("kernel curve has no point"))?;
        let first = *values
            .get(1)
            .ok_or(GeometryError::InvalidCurve("kernel curve has no tangent"))?;
        let second = values.get(2).copied().unwrap_or(Vec3::ZERO);

        let relative = point - self.origin;
        let point_2d = [relative.dot(self.x_axis), relative.dot(self.y_axis)];
        let first_2d = [first.dot(self.x_axis), first.dot(self.y_axis)];
        let second_2d = [second.dot(self.x_axis), second.dot(self.y_axis)];

        let speed = norm(first_2d);
        if !speed.is_finite() || speed <= 1.0e-15 {
            return Err(GeometryError::ZeroSpeed);
        }
        let tangent = unit(first_2d).ok_or(GeometryError::ZeroSpeed)?;
        let normal = rot90(tangent);
        let curvature = cross(first_2d, second_2d) / (speed * speed * speed);
        Ok(DifferentialData {
            point: point_2d,
            first: first_2d,
            second: second_2d,
            tangent,
            normal,
            curvature,
            speed,
        })
    }
}

/// Options of [`build_swept_surface`].
#[derive(Clone, Copy, Debug)]
pub struct SurfaceBuildOptions {
    /// The maximum accepted geometric deviation between the NURBS surface and
    /// the analytic sweep, measured two-sided.
    pub tolerance: f64,
    /// Maximum number of adaptive refinements.
    pub max_refinements: usize,
    /// Maximum number of cross sections.
    pub max_sections: usize,
    /// Degree in the sweep direction.
    pub degree: usize,
    /// Samples of the analytic surface along the path for the deviation check.
    pub analytic_samples_v: usize,
    /// Samples of the analytic surface along the profile for the deviation check.
    pub analytic_samples_u: usize,
    /// Samples of the NURBS surface along V (the profile) for the reverse check.
    pub surface_samples_v: usize,
    /// Samples of the NURBS surface along U (the sweep direction) for the reverse check.
    pub surface_samples_u: usize,
}

impl Default for SurfaceBuildOptions {
    fn default() -> Self {
        Self {
            tolerance: 1.0e-5,
            max_refinements: 6,
            max_sections: 128,
            degree: 3,
            analytic_samples_v: 81,
            analytic_samples_u: 41,
            surface_samples_v: 9,
            surface_samples_u: 5,
        }
    }
}

/// A built swept surface and its measured accuracy.
#[derive(Clone, Debug)]
pub struct BuiltSweptSurface {
    /// The interpolated/approximated NURBS surface. Its U is the sweep direction
    /// and V the profile; the cross sections are constant-U curves.
    pub surface: BSplineSurface,
    /// The measured two-sided maximum deviation.
    pub max_deviation: f64,
    /// The number of cross sections used.
    pub sections: usize,
    /// The number of adaptive refinements performed.
    pub refinements: usize,
}

/// An explicit surface-build refusal.
#[derive(Debug)]
pub enum SurfaceBuildError {
    /// A kernel call failed.
    Kernel(SolidError),
    /// The requested tolerance was not reached after the refinement budget.
    ToleranceNotReached {
        /// The requested tolerance.
        tolerance: f64,
        /// The measured deviation.
        deviation: f64,
        /// The number of cross sections used.
        sections: usize,
        /// The number of refinements used.
        refinements: usize,
    },
    /// The input is degenerate for the analytic sweep.
    Degenerate(&'static str),
}

impl fmt::Display for SurfaceBuildError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Kernel(error) => write!(f, "{error}"),
            Self::ToleranceNotReached {
                tolerance,
                deviation,
                sections,
                refinements,
            } => write!(
                f,
                "surface tolerance {tolerance} not reached: deviation {deviation} after {refinements} refinements and {sections} sections"
            ),
            Self::Degenerate(reason) => f.write_str(reason),
        }
    }
}

impl std::error::Error for SurfaceBuildError {}

impl From<SolidError> for SurfaceBuildError {
    fn from(value: SolidError) -> Self {
        Self::Kernel(value)
    }
}

/// The analytic sweep `S(v,u) = P(v) + alpha(u) n(v) + beta(u) N`.
struct AnalyticSweep {
    path: BSplineCurve,
    profile: BSplineCurve,
    start: Point3,
    start_tangent: Vec3,
    plane_normal: Vec3,
    start_normal: Vec3,
}

impl AnalyticSweep {
    fn new(
        path: &BSplineCurve,
        profile: &BSplineCurve,
        plane_normal: Vec3,
    ) -> Result<Self, SurfaceBuildError> {
        let values = path
            .evaluate(path.natural_interval().min(), 1, true, false)
            .map_err(SurfaceBuildError::Kernel)?;
        let start = *values
            .first()
            .ok_or(SurfaceBuildError::Degenerate("path has no start point"))?;
        let start_tangent = values
            .get(1)
            .copied()
            .ok_or(SurfaceBuildError::Degenerate("path has no start tangent"))?
            .unitize()
            .map_err(|_| SurfaceBuildError::Degenerate("path start tangent is zero"))?;
        let plane_normal = plane_normal
            .unitize()
            .map_err(|_| SurfaceBuildError::Degenerate("path plane normal is zero"))?;
        if plane_normal.dot(start_tangent).abs() > 1.0e-8 {
            return Err(SurfaceBuildError::Degenerate(
                "path start tangent is not in the path plane",
            ));
        }
        let start_normal = plane_normal.cross(start_tangent);
        Ok(Self {
            path: path.clone(),
            profile: profile.clone(),
            start,
            start_tangent,
            plane_normal,
            start_normal,
        })
    }

    fn point(&self, v: f64, u: f64) -> Result<Point3, SurfaceBuildError> {
        let path_values = self
            .path
            .evaluate(v, 1, true, false)
            .map_err(SurfaceBuildError::Kernel)?;
        let point = *path_values
            .first()
            .ok_or(SurfaceBuildError::Degenerate("path point missing"))?;
        let tangent = path_values
            .get(1)
            .copied()
            .ok_or(SurfaceBuildError::Degenerate("path tangent missing"))?
            .unitize()
            .map_err(|_| SurfaceBuildError::Degenerate("path tangent is zero"))?;
        let normal = self.plane_normal.cross(tangent);
        let profile_point = self
            .profile
            .evaluate_point(u)
            .map_err(SurfaceBuildError::Kernel)?;
        let alpha = (profile_point - self.start).dot(self.start_normal);
        let beta = (profile_point - self.start).dot(self.plane_normal);
        Ok(point + normal * alpha + self.plane_normal * beta)
    }

    fn section(&self, v: f64) -> Result<BSplineCurve, SurfaceBuildError> {
        let path_values = self
            .path
            .evaluate(v, 1, true, false)
            .map_err(SurfaceBuildError::Kernel)?;
        let point = *path_values
            .first()
            .ok_or(SurfaceBuildError::Degenerate("path point missing"))?;
        let tangent = path_values
            .get(1)
            .copied()
            .ok_or(SurfaceBuildError::Degenerate("path tangent missing"))?
            .unitize()
            .map_err(|_| SurfaceBuildError::Degenerate("path tangent is zero"))?;
        let normal = self.plane_normal.cross(tangent);
        let start = Axis2Placement::new(self.start, self.start_tangent, self.start_normal);
        let current = Axis2Placement::new(point, tangent, normal);
        let placement = Axis2Placement::from_to(&start, &current);
        let mut section = self.profile.clone();
        section
            .transform(&placement, None)
            .map_err(SurfaceBuildError::Kernel)?;
        Ok(section)
    }

    fn domain(&self) -> ([f64; 2], [f64; 2]) {
        let path = self.path.natural_interval();
        let profile = self.profile.natural_interval();
        ([path.min(), path.max()], [profile.min(), profile.max()])
    }
}

fn closest_analytic(
    analytic: &AnalyticSweep,
    point: Point3,
    samples_v: usize,
    samples_u: usize,
) -> Result<f64, SurfaceBuildError> {
    let (v_domain, u_domain) = analytic.domain();
    let best = |v: f64, u: f64| -> Result<f64, SurfaceBuildError> {
        Ok(analytic.point(v, u)?.distance(point))
    };
    let mut best_v = v_domain[0];
    let mut best_u = u_domain[0];
    let mut best_distance = f64::INFINITY;
    for i in 0..=samples_v {
        let v = v_domain[0] + (v_domain[1] - v_domain[0]) * (i as f64 / samples_v as f64);
        for j in 0..=samples_u {
            let u = u_domain[0] + (u_domain[1] - u_domain[0]) * (j as f64 / samples_u as f64);
            let distance = best(v, u)?;
            if distance < best_distance {
                best_distance = distance;
                best_v = v;
                best_u = u;
            }
        }
    }
    // A short coordinate-descent polish around the best grid node.
    let dv = (v_domain[1] - v_domain[0]) / samples_v as f64;
    let du = (u_domain[1] - u_domain[0]) / samples_u as f64;
    let mut lo_v = (best_v - dv).max(v_domain[0]);
    let mut hi_v = (best_v + dv).min(v_domain[1]);
    let mut lo_u = (best_u - du).max(u_domain[0]);
    let mut hi_u = (best_u + du).min(u_domain[1]);
    for _ in 0..40 {
        let v = golden(
            |v| best(v, (lo_u + hi_u) * 0.5).unwrap_or(f64::INFINITY),
            lo_v,
            hi_v,
        );
        let u = golden(
            |u| best((lo_v + hi_v) * 0.5, u).unwrap_or(f64::INFINITY),
            lo_u,
            hi_u,
        );
        lo_v = (v - (hi_v - lo_v) * 0.25).max(v_domain[0]);
        hi_v = (v + (hi_v - lo_v) * 0.25).min(v_domain[1]);
        lo_u = (u - (hi_u - lo_u) * 0.25).max(u_domain[0]);
        hi_u = (u + (hi_u - lo_u) * 0.25).min(u_domain[1]);
        let distance = best(v, u)?;
        if distance < best_distance {
            best_distance = distance;
        }
    }
    Ok(best_distance)
}

fn golden<F: Fn(f64) -> f64>(f: F, mut a: f64, mut b: f64) -> f64 {
    let ratio = (5.0_f64.sqrt() - 1.0) * 0.5;
    let mut c = b - ratio * (b - a);
    let mut d = a + ratio * (b - a);
    let mut fc = f(c);
    let mut fd = f(d);
    for _ in 0..80 {
        if (b - a).abs() < 1.0e-14 {
            break;
        }
        if fc < fd {
            b = d;
            d = c;
            fd = fc;
            c = b - ratio * (b - a);
            fc = f(c);
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + ratio * (b - a);
            fd = f(d);
        }
    }
    (a + b) * 0.5
}

fn measure_deviation(
    k: &Kernel,
    analytic: &AnalyticSweep,
    surface: &Surface,
    options: &SurfaceBuildOptions,
) -> Result<f64, SurfaceBuildError> {
    let domain = surface
        .natural_uv_domain_with_nurb(k)
        .map_err(SurfaceBuildError::Kernel)?;
    let (v_domain, u_domain) = analytic.domain();
    let mut worst: f64 = 0.0;

    // Analytic -> NURBS: geometric closest point, not a same-parameter comparison.
    for i in 0..=options.analytic_samples_v {
        let v = v_domain[0]
            + (v_domain[1] - v_domain[0]) * (i as f64 / options.analytic_samples_v as f64);
        for j in 0..=options.analytic_samples_u {
            let u = u_domain[0]
                + (u_domain[1] - u_domain[0]) * (j as f64 / options.analytic_samples_u as f64);
            let point = analytic.point(v, u)?;
            let drop = surface
                .drop_point::<NoObject>(k, point, &domain, &DropPointOptions::default())
                .map_err(SurfaceBuildError::Kernel)?;
            if !drop.success {
                return Err(SurfaceBuildError::Degenerate(
                    "NURBS drop point did not converge",
                ));
            }
            worst = worst.max(drop.gap);
        }
    }

    // NURBS -> analytic: the reverse direction, with an independent analytic
    // closest-point search, catches a fit that pulls away between sections.
    for i in 0..=options.surface_samples_u {
        let s = i as f64 / options.surface_samples_u as f64;
        let u = domain.min().x + (domain.max().x - domain.min().x) * s;
        for j in 0..=options.surface_samples_v {
            let t = j as f64 / options.surface_samples_v as f64;
            let v = domain.min().y + (domain.max().y - domain.min().y) * t;
            let point = surface
                .evaluate_point(k, Point2::new(u, v))
                .map_err(SurfaceBuildError::Kernel)?;
            // The reverse check only needs a coarse basin for the golden polish;
            // the forward check above is the dense acceptance sample.
            let distance = closest_analytic(analytic, point, 33, 17)?;
            worst = worst.max(distance);
        }
    }
    Ok(worst)
}

/// Build the swept surface through explicit cross sections at the path
/// parameters `sections`, without the adaptive tolerance loop.
///
/// The sections are interpolated (`approx_tol = 0`), so the input sections are
/// contained in the result; the tolerance is only measured here. `sections` is
/// used in the given order and must hold at least two finite parameters.
///
/// # Errors
///
/// [`SurfaceBuildError`] for a kernel failure, a degenerate input or fewer than
/// two sections.
pub fn build_swept_surface_with_sections(
    k: &Kernel,
    path: &BSplineCurve,
    profile: &BSplineCurve,
    plane_normal: Vec3,
    sections: &[f64],
    options: &SurfaceBuildOptions,
) -> Result<BuiltSweptSurface, SurfaceBuildError> {
    if sections.len() < 2 {
        return Err(SurfaceBuildError::Degenerate(
            "at least two cross sections are required",
        ));
    }
    if sections.iter().any(|value| !value.is_finite()) {
        return Err(SurfaceBuildError::Degenerate(
            "a cross section parameter is not finite",
        ));
    }
    let analytic = AnalyticSweep::new(path, profile, plane_normal)?;
    let mut curves = Vec::with_capacity(sections.len());
    for &v in sections {
        curves.push(analytic.section(v)?);
    }
    let surface = BSplineSurface::create_skinned_surface(
        k,
        &curves,
        SkinnedSurfaceOptions {
            synchronized: false,
            cross_section_surf_param: SurfParamType::U,
            approx_tol: 0.0,
            rail1: None,
            rail2: None,
            rail1_used_as_spine_curve: false,
            intersections_with_rails: None,
            deriv_surfs: None,
            degree: options.degree,
        },
    )
    .map_err(SurfaceBuildError::Kernel)?;
    let wrapped = Surface::from(surface.clone());
    let deviation = measure_deviation(k, &analytic, &wrapped, options)?;
    Ok(BuiltSweptSurface {
        surface,
        max_deviation: deviation,
        sections: sections.len(),
        refinements: 0,
    })
}

/// Build the swept surface of `profile` along `path` with the analytic frame
/// `(tangent, plane_normal x tangent, plane_normal)`.
///
/// The surface is interpolated through adaptive cross sections with
/// `BSplineSurface::create_skinned_surface`; the returned deviation is measured
/// geometrically in both directions, including points between the fit sections.
/// The profile must lie in the start normal plane `(normal, plane_normal)`; a
/// profile with a start-tangent component is outside this construction. The
/// result's U is the sweep direction and V is the profile.
///
/// # Errors
///
/// [`SurfaceBuildError`] for a kernel failure, a degenerate input or a tolerance
/// that cannot be reached within the refinement budget.
pub fn build_swept_surface(
    k: &Kernel,
    path: &BSplineCurve,
    profile: &BSplineCurve,
    plane_normal: Vec3,
    options: &SurfaceBuildOptions,
) -> Result<BuiltSweptSurface, SurfaceBuildError> {
    if !options.tolerance.is_finite() || options.tolerance <= 0.0 {
        return Err(SurfaceBuildError::Degenerate(
            "surface tolerance must be positive and finite",
        ));
    }
    let interval = path.natural_interval();
    let mut v_parameters = vec![interval.min(), interval.max()];
    let mut refinements = 0usize;
    loop {
        let mut built = build_swept_surface_with_sections(
            k,
            path,
            profile,
            plane_normal,
            &v_parameters,
            options,
        )?;
        built.refinements = refinements;
        let deviation = built.max_deviation;
        let sections_count = v_parameters.len();
        let can_refine = deviation > options.tolerance
            && refinements < options.max_refinements
            && sections_count < options.max_sections;
        if !can_refine {
            if deviation > options.tolerance {
                return Err(SurfaceBuildError::ToleranceNotReached {
                    tolerance: options.tolerance,
                    deviation,
                    sections: sections_count,
                    refinements,
                });
            }
            return Ok(built);
        }
        let mut refined = Vec::with_capacity(v_parameters.len() * 2 - 1);
        for pair in v_parameters.windows(2) {
            let first = *pair
                .first()
                .ok_or(SurfaceBuildError::Degenerate("empty section interval"))?;
            let second = *pair
                .get(1)
                .ok_or(SurfaceBuildError::Degenerate("empty section interval"))?;
            refined.push(first);
            refined.push((first + second) * 0.5);
        }
        if let Some(last) = v_parameters.last().copied() {
            refined.push(last);
        }
        v_parameters = refined;
        refinements += 1;
    }
}
