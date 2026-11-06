//! Exact analytic swept surfaces for a planar path and a circular profile.
//!
//! These replace the Level B approximation when the sweep has an exact
//! Cylinder/Extrusion or Torus/Revolution representation. The surface keeps
//! U = path direction and V = profile, which is the Direct B-rep Builder's
//! convention.

use solidcore::curve::curve::Curve;
use solidcore::error::SolidError;
use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::surface::surf_of_extrusion::SurfOfExtrusion;
use solidcore::surface::surface::Surface;
use solidcore::surface::torus::{Torus, TorusOptions};

const OP: &str = "research::exact_geometry";

/// The exact cylinder swept by `profile` along the line `path_start -> path_end`.
///
/// `profile` must be closed and planar in the plane normal to the path. The
/// returned surface has U = path and V = profile.
///
/// # Errors
///
/// A zero-length path, a profile without a NURB, or the kernel extrusion/NURB
/// construction.
pub fn line_sweep_surface(
    k: &Kernel,
    profile: &Curve,
    path_start: Point3,
    path_end: Point3,
) -> Result<Surface, SolidError> {
    let extrusion = path_end - path_start;
    if extrusion.length() <= 1.0e-12 {
        return Err(SolidError::invalid_input(OP, "the path has zero length"));
    }
    let profile_interval = profile.natural_interval();
    let origin = profile.evaluate_point(k, profile_interval.min())?;
    let domain = Extent2d::from_bounds(profile_interval.min(), 0.0, profile_interval.max(), 1.0);
    let mut surface = SurfOfExtrusion::new(k, profile.clone(), origin, extrusion, domain, true)?;
    surface.make_nurb(k)?;
    Ok(Surface::from(surface))
}

/// The exact torus of a circular profile swept along an arc.
///
/// `center` is the arc centre, `major_radius` the arc radius and
/// `(start_deg, end_deg)` the major-angle domain. `x_axis` is the radial
/// direction at `start_deg`; `y_axis` is the tangent there, so
/// `x_axis x y_axis` is the rotation axis. The returned surface has U = path
/// (major angle) and V = profile (minor angle). The generator is built
/// inside-out to match the Direct builder's side-face orientation.
///
/// # Errors
///
/// A degenerate radius/domain or the kernel torus construction.
pub fn arc_sweep_surface(
    k: &Kernel,
    center: Point3,
    x_axis: Vec3,
    y_axis: Vec3,
    major_radius: f64,
    minor_radius: f64,
    start_deg: f64,
    end_deg: f64,
) -> Result<Surface, SolidError> {
    arc_sweep_surface_with_profile(
        k,
        None,
        center,
        x_axis,
        y_axis,
        major_radius,
        minor_radius,
        start_deg,
        end_deg,
    )
}

/// [`arc_sweep_surface`] using `profile` as the exact minor-circle generator.
///
/// The profile is used when the caller already has the world-space profile at
/// `start_deg` and needs its parameterisation/orientation to match the joint
/// boundary exactly.
///
/// # Errors
///
/// Those of [`arc_sweep_surface`], plus the profile's natural interval when it
/// is used as the V domain.
#[allow(clippy::too_many_arguments)]
pub fn arc_sweep_surface_with_profile(
    k: &Kernel,
    profile: Option<&Curve>,
    center: Point3,
    x_axis: Vec3,
    y_axis: Vec3,
    major_radius: f64,
    minor_radius: f64,
    start_deg: f64,
    end_deg: f64,
) -> Result<Surface, SolidError> {
    if major_radius <= 1.0e-12 || minor_radius <= 1.0e-12 {
        return Err(SolidError::invalid_input(OP, "the arc radius is zero"));
    }
    if (end_deg - start_deg).abs() <= 1.0e-12 {
        return Err(SolidError::invalid_input(OP, "the arc domain is empty"));
    }
    let (v_min, v_max) = profile.map_or((0.0, 360.0), |profile| {
        let interval = profile.natural_interval();
        (interval.min(), interval.max())
    });
    let domain = Extent2d::from_bounds(start_deg, v_min, end_deg, v_max);
    let mut surface = Torus::new(
        k,
        center,
        x_axis,
        y_axis,
        domain,
        major_radius,
        minor_radius,
        TorusOptions {
            inside_out: true,
            circle_nurb: profile.cloned(),
            ..TorusOptions::default()
        },
    )?;
    surface.make_nurb(k)?;
    Ok(Surface::from(surface))
}
