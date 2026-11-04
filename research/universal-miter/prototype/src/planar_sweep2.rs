//! Gate 0: the normalized input for the alternative Planar Sweep.
//!
//! This module reuses the existing input preparation and reproduces the one
//! private whole-path placement; it builds no swept surface, joint or B-rep.
//! The production Planar Sweep is unchanged. See `docs/10-preprocessing-audit.md`
//! and `docs/09-sweep2-scope.md`.

use solidcore::curve::curve::p2::{Curve, CurveKind};
use solidcore::curve::curve::p3::compute_crvs_normal;
use solidcore::error::{Result, SolidError};
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::geom::extent3d::Extent3d;
use solidcore::geom::tol::ZONE_TOL_3D;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3, EFF_ZERO, EFF_ZERO_SQRT};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::b_spline_curve::p2::BoundingBoxRequest;
use solidcore::primitive::primitive_creation::p1::extend_path_curves_with_kernel;

const OP: &str = "research::planar_sweep2::normalize";

/// The preparation block of `CreateCurveSweep`, reproduced locally because the
/// production module (`p3::geometry`) is private to the kernel crate.
#[derive(Clone, Debug)]
pub struct PreparedPiece {
    /// Prepared profile bases in the local frame.
    pub profiles: Vec<BSplineCurve>,
    /// The path in the local frame.
    pub path: BSplineCurve,
    /// Undo the path rotation.
    pub rotation_back: Axis2Placement,
    /// Return to the original path start.
    pub path_back: Axis2Placement,
    /// Return to the original profile reference.
    pub profile_back: Axis2Placement,
    /// Move the profile reference to the origin.
    pub profile_to_origin: Axis2Placement,
    /// Turn the path start frame into the global frame.
    pub rotation: Axis2Placement,
}

/// The cap configuration the existing Planar Sweep derives for the input.
#[allow(clippy::struct_excessive_bools)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CapConfiguration {
    /// The start cap is requested and the profile closes.
    pub start: bool,
    /// The end cap is requested and the profile closes.
    pub end: bool,
    /// The profile closes (WR-400 reference; the profile can be capped).
    pub profile_closes: bool,
    /// The API's `cap_ends` selector (informational).
    pub cap_ends: i32,
    /// The API's `move_profile` selector (informational).
    pub move_profile: bool,
}

/// One path piece after the production placement and preparation.
#[derive(Clone, Debug)]
pub struct NormalizedPlacement {
    /// Profiles placed at the piece start by the whole-path placement.
    pub profiles_placed: Vec<Curve>,
    /// The prepared profile bases in the local frame.
    pub profiles_bspline: Vec<BSplineCurve>,
    /// The extended path piece.
    pub path_piece: BSplineCurve,
    /// The preparation block of the existing `CreateCurveSweep`, reproduced.
    pub prepared: PreparedPiece,
    /// The prepared profiles mapped back to world space by `rotation_back` and
    /// `path_back` (the cap profile at the path start of this piece).
    pub world_profiles: Vec<Curve>,
    /// Whether this piece asks for a start cap.
    pub cap_start: bool,
    /// Whether this piece asks for an end cap.
    pub cap_end: bool,
}

/// The normalized input both implementations must share.
#[derive(Clone, Debug)]
pub struct NormalizedPlanarSweep {
    /// Original profile curves (kinds kept for analytic references).
    pub profiles_original: Vec<Curve>,
    /// Normalized profile bases before placement.
    pub profiles_bspline: Vec<BSplineCurve>,
    /// Original path curves (kinds kept for diagnostics).
    pub path_original: Vec<Curve>,
    /// Path pieces after normalization and G1 extension.
    pub path_bspline: Vec<BSplineCurve>,
    /// Per-piece placement and preparation.
    pub placements: Vec<NormalizedPlacement>,
    /// Closed path (first start and last end within `ZONE_TOL_3D`).
    pub closed: bool,
    /// Closed with parallel first/last tangents.
    pub periodic: bool,
    /// The production extension length `2 * sqrt(2) * max_dimension`.
    pub extension: f64,
    /// The production path plane normal (`compute_crvs_normal`); `None` for a
    /// single straight path.
    pub plane_normal: Option<Vec3>,
    /// The maximum sampled distance to the plane through the production normal.
    pub planarity_deviation: f64,
    /// Every path curve reports planar at `ZONE_TOL_3D`.
    pub planarity_is_planar: bool,
    /// The cap configuration derived for the input.
    pub caps: CapConfiguration,
    /// The reference tolerance (`ZONE_TOL_3D`).
    pub tolerance: f64,
}

/// The private `as_bspline` mapping, reproduced from the public `CurveKind` and
/// `base` accessors (the production function is private).
///
/// # Errors
///
/// `InvalidInput` for a curve kind the existing Planar Sweep does not accept.
pub fn to_bspline(curve: &Curve) -> Result<BSplineCurve> {
    match curve.kind() {
        CurveKind::BSpline(c) => Ok(c.clone()),
        CurveKind::Line(c) => Ok(c.base().clone()),
        CurveKind::Ellipse(c) => Ok(c.base().clone()),
        CurveKind::Parabola(c) => Ok(c.base().clone()),
        CurveKind::Hyperbola(c) => Ok(c.base().clone()),
        _ => Err(SolidError::invalid_input(
            OP,
            "unsupported curve kind for the Planar Sweep normalization",
        )),
    }
}

/// Normalize the Planar Sweep input exactly as the existing implementation
/// prepares it, and report the shared contract fields.
///
/// # Errors
///
/// `InvalidInput` for empty inputs, a bad cap selector, an unsupported curve
/// kind or a non-planar path; the errors of the preparation.
pub fn normalize_planar_sweep(
    k: &Kernel,
    profiles: &[Curve],
    paths: &[Curve],
    move_profile: bool,
    cap_ends: i32,
) -> Result<NormalizedPlanarSweep> {
    if profiles.is_empty() || paths.is_empty() {
        return Err(SolidError::invalid_input(OP, "no profile or path curves"));
    }
    if !(0..=3).contains(&cap_ends) {
        return Err(SolidError::invalid_input(OP, "cap selector outside 0..=3"));
    }

    let profiles_bspline = profiles
        .iter()
        .map(to_bspline)
        .collect::<Result<Vec<_>>>()?;
    let path_bspline = paths.iter().map(to_bspline).collect::<Result<Vec<_>>>()?;

    let (closed, periodic) = closed_and_periodic(k, paths, ZONE_TOL_3D)?;
    let extension = extension_length(k, profiles)?;
    let extended = extend_path_curves_with_kernel(k, &path_bspline, closed, periodic, extension)?;
    let (plane_normal, planarity_deviation, planarity_is_planar) = plane_check(k, paths)?;
    if planarity_deviation > ZONE_TOL_3D {
        return Err(SolidError::invalid_input(
            OP,
            "path is not planar within ZONE_TOL_3D",
        ));
    }
    let profile_closes = profile_closes(k, profiles, ZONE_TOL_3D)?;
    let cap_profile = if k.fixes.planar_path_uncapped_open_profiles {
        profile_closes
    } else {
        true
    };
    let cap_start_requested = (cap_ends & 1) != 0;
    let cap_end_requested = (cap_ends & 2) != 0;

    let mut placements = Vec::with_capacity(extended.len());
    for (index, path_piece) in extended.iter().enumerate() {
        let path_curve = Curve::from(path_piece.clone());
        let mut profiles_placed = Vec::new();
        place_profiles_whole_path(k, profiles, &path_curve, paths, &mut profiles_placed)?;
        let prepared = prepare_piece(k, &profiles_placed, path_piece, move_profile)?;
        let world_profiles = prepared
            .profiles
            .iter()
            .map(|profile| {
                let mut curve = profile.clone();
                curve.transform(&prepared.rotation_back, None)?;
                curve.transform(&prepared.path_back, None)?;
                Ok(Curve::from(curve))
            })
            .collect::<Result<Vec<_>>>()?;
        let cap_start = !closed && index == 0 && cap_start_requested && cap_profile;
        let cap_end = !closed && index + 1 == extended.len() && cap_end_requested && cap_profile;
        placements.push(NormalizedPlacement {
            profiles_placed,
            profiles_bspline: prepared.profiles.clone(),
            path_piece: path_piece.clone(),
            prepared,
            world_profiles,
            cap_start,
            cap_end,
        });
    }

    Ok(NormalizedPlanarSweep {
        profiles_original: profiles.to_vec(),
        profiles_bspline,
        path_original: paths.to_vec(),
        path_bspline: extended,
        placements,
        closed,
        periodic,
        extension,
        plane_normal,
        planarity_deviation,
        planarity_is_planar,
        caps: CapConfiguration {
            start: !closed && cap_start_requested && cap_profile,
            end: !closed && cap_end_requested && cap_profile,
            profile_closes,
            cap_ends,
            move_profile,
        },
        tolerance: ZONE_TOL_3D,
    })
}

fn closed_and_periodic(k: &Kernel, paths: &[Curve], tol: f64) -> Result<(bool, bool)> {
    let first = paths
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "no path curves"))?;
    let last = paths
        .last()
        .ok_or_else(|| SolidError::invalid_input(OP, "no path curves"))?;
    let first_values = first.evaluate(k, first.natural_interval().min(), 1, true, true)?;
    let last_values = last.evaluate(k, last.natural_interval().max(), 1, true, true)?;
    let start_point = *first_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
    let start_tangent = *first_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path start tangent missing"))?;
    let end_point = *last_values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path end missing"))?;
    let end_tangent = *last_values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path end tangent missing"))?;
    let closed = start_point.distance(end_point) < tol;
    let periodic = closed && start_tangent.is_parallel_to(end_tangent, 2.0);
    Ok((closed, periodic))
}

fn extension_length(k: &Kernel, profiles: &[Curve]) -> Result<f64> {
    let mut total = Extent3d::new();
    let mut bounds = Extent3d::new();
    for profile in profiles {
        let boxes = profile.calculate_bounding_box(
            k,
            &profile.natural_interval(),
            BoundingBoxRequest {
                normal: true,
                ..BoundingBoxRequest::default()
            },
            true,
        )?;
        if let Some(normal) = boxes.normal {
            bounds = normal;
        }
        total.add_point(bounds.min());
        total.add_point(bounds.max());
    }
    let max_dimension = total.max_dimension();
    let mut extension = max_dimension * 2.0_f64.sqrt();
    extension *= 2.0;
    Ok(extension)
}

fn profile_closes(k: &Kernel, profiles: &[Curve], tol: f64) -> Result<bool> {
    let mut ends = Vec::new();
    for curve in profiles {
        let domain = curve.natural_interval();
        ends.push(curve.evaluate_point(k, domain.min())?);
        ends.push(curve.evaluate_point(k, domain.max())?);
    }
    Ok(ends.iter().enumerate().all(|(index, end)| {
        ends.iter()
            .enumerate()
            .any(|(other_index, other)| other_index != index && end.distance(*other) < tol)
    }))
}

/// The preparation block of `CreateCurveSweep` with the Planar Sweep options
/// (`test_continuity = false`, no scaling, no perpendicular orientation),
/// reproduced because `prepare_curve_sweep` and `PreparedCurveSweep` are private
/// to the kernel crate.
fn prepare_piece(
    k: &Kernel,
    profiles: &[Curve],
    path: &BSplineCurve,
    move_profile_center_to_path: bool,
) -> Result<PreparedPiece> {
    let mut path = path.clone();
    let mut profiles: Vec<BSplineCurve> = profiles
        .iter()
        .map(to_bspline)
        .collect::<Result<Vec<_>>>()?;

    let values = path.evaluate(path.natural_interval().min(), 1, true, false)?;
    let start = values
        .first()
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
    let mut path_to_origin = Axis2Placement::IDENTITY;
    path_to_origin.translate(-start);
    path.transform(&path_to_origin, None)?;

    let reference = if move_profile_center_to_path {
        let mut total = Extent3d::new();
        let mut bounds = Extent3d::new();
        for profile in &profiles {
            let curve = Curve::from(profile.clone());
            let boxes = curve.calculate_bounding_box(
                k,
                &curve.natural_interval(),
                BoundingBoxRequest {
                    normal: true,
                    ..BoundingBoxRequest::default()
                },
                true,
            )?;
            if let Some(normal) = boxes.normal {
                bounds = normal;
            }
            total.add_point(bounds.min());
            total.add_point(bounds.max());
        }
        total.mid()
    } else {
        let first = profiles
            .first()
            .ok_or_else(|| SolidError::invalid_input(OP, "no B-spline profiles"))?;
        first.evaluate_point(first.natural_interval().min())?
    };
    let mut profile_to_origin = Axis2Placement::IDENTITY;
    profile_to_origin.translate(-reference);
    for profile in &mut profiles {
        profile.transform(&profile_to_origin, None)?;
    }

    let values = path.evaluate(path.natural_interval().min(), 1, true, false)?;
    let tangent = values
        .get(1)
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "path tangent missing"))?;
    let [x, y, _] = tangent.make_unit_ortho_vectors(None)?;
    let mut from_x = x;
    let mut from_y = y;
    if k.fixes.sweep_frame_on_planar_path_normal {
        let plane = path.is_planar(ZONE_TOL_3D)?;
        if let Some(plane_normal) = plane.is_planar.then_some(plane.normal) {
            if let Ok(normal) = plane_normal.unitize() {
                if normal.dot(from_x).abs() < EFF_ZERO_SQRT {
                    let from_z = normal - normal.dot(from_x) * from_x;
                    if from_z.unitize().is_ok() {
                        from_y = from_z.cross(from_x);
                    } else {
                        let [x, y, _] = tangent.make_unit_ortho_vectors(None)?;
                        from_x = x;
                        from_y = y;
                    }
                }
            }
        }
    }
    let rotation = Axis2Placement::from_to(
        &Axis2Placement::new(Vec3::ZERO, from_x, from_y),
        &Axis2Placement::IDENTITY,
    );
    path.transform(&rotation, None)?;
    for profile in &mut profiles {
        profile.transform(&rotation, None)?;
    }
    Ok(PreparedPiece {
        profiles,
        path,
        rotation_back: rotation.invert(),
        path_back: path_to_origin.invert(),
        profile_back: profile_to_origin.invert(),
        profile_to_origin,
        rotation,
    })
}

fn plane_check(k: &Kernel, paths: &[Curve]) -> Result<(Option<Vec3>, f64, bool)> {
    let normal = compute_crvs_normal(k, paths).ok();
    let mut deviation = 0.0_f64;
    if let Some(plane_normal) = normal {
        let mut points: Vec<Point3> = Vec::new();
        for path in paths {
            let domain = path.natural_interval();
            for index in 0..=64usize {
                let t = domain.evaluate(index as f64 / 64.0);
                points.push(path.evaluate_point(k, t)?);
            }
        }
        if !points.is_empty() {
            let mut centroid = Vec3::ZERO;
            for point in &points {
                centroid += *point;
            }
            centroid *= 1.0 / points.len() as f64;
            for point in &points {
                deviation = deviation.max((*point - centroid).dot(plane_normal).abs());
            }
        }
    }
    let mut is_planar = true;
    for path in paths {
        let planar = path.is_planar(k, ZONE_TOL_3D)?;
        is_planar &= planar.is_planar;
    }
    Ok((normal, deviation, is_planar))
}

/// The private `position_profile_curves_impl` behavior: the profile is placed
/// with respect to the whole path list, using the current extended piece as the
/// frame path.
fn place_profiles_whole_path(
    k: &Kernel,
    profiles: &[Curve],
    path: &Curve,
    whole_path: &[Curve],
    output: &mut Vec<Curve>,
) -> Result<()> {
    if k.fixes.planar_path_profile_in_path_frame {
        if let Ok(normal) = compute_crvs_normal(k, whole_path) {
            if normal.x.abs() > EFF_ZERO || normal.y.abs() > EFF_ZERO {
                let first = whole_path
                    .first()
                    .ok_or_else(|| SolidError::invalid_input(OP, "no path curves"))?;
                let values = first.evaluate(k, first.natural_interval().min(), 1, true, true)?;
                let origin = *values
                    .first()
                    .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
                let tangent = values
                    .get(1)
                    .copied()
                    .ok_or_else(|| SolidError::invalid_input(OP, "path tangent missing"))?
                    .unitize()?;
                let x = tangent.cross(normal).unitize()?;
                let frame = Axis2Placement::new(origin, x, tangent);
                let inverse = frame.invert();
                let mut local_profiles = profiles.to_vec();
                for profile in &mut local_profiles {
                    profile.transform(k, &inverse, None)?;
                }
                let mut local_path = path.clone();
                local_path.transform(k, &inverse, None)?;
                let mut local_output = Vec::new();
                position_profile_curves_legacy(k, &local_profiles, &local_path, &mut local_output)?;
                for curve in &mut local_output {
                    curve.transform(k, &frame, None)?;
                }
                *output = local_output;
                return Ok(());
            }
        }
    }
    position_profile_curves_legacy(k, profiles, path, output)
}

fn position_profile_curves_legacy(
    k: &Kernel,
    profiles: &[Curve],
    path: &Curve,
    output: &mut Vec<Curve>,
) -> Result<()> {
    let mut total = Extent3d::new();
    let mut bounds = Extent3d::new();
    for profile in profiles {
        let boxes = profile.calculate_bounding_box(
            k,
            &profile.natural_interval(),
            BoundingBoxRequest {
                normal: true,
                ..BoundingBoxRequest::default()
            },
            true,
        )?;
        if let Some(normal) = boxes.normal {
            bounds = normal;
        }
        total.add_point(bounds.min());
        total.add_point(bounds.max());
        output.push(profile.copy(k)?);
    }
    let mut translation = Axis2Placement::IDENTITY;
    translation.set_canonical(-total.mid(), Vec3::X, Vec3::Y)?;
    for curve in output.iter_mut() {
        curve.transform(k, &translation, None)?;
    }
    let normal = compute_crvs_normal(k, profiles)?;
    let angle = Vec3::X.angle_between(normal)?;
    let mut orientation = Axis2Placement::IDENTITY;
    orientation.rotate_about_axis(-angle, Vec3::X.cross(normal));
    for curve in output.iter_mut() {
        curve.transform(k, &orientation, None)?;
    }
    let values = path.evaluate(k, path.natural_interval().min(), 1, true, true)?;
    let origin = *values
        .first()
        .ok_or_else(|| SolidError::invalid_input(OP, "path start missing"))?;
    let tangent = *values
        .get(1)
        .ok_or_else(|| SolidError::invalid_input(OP, "path tangent missing"))?;
    let mut y = Vec3::Z.cross(tangent);
    if tangent.z.abs() > tangent.x.abs() && tangent.z.abs() > tangent.y.abs() {
        y = Vec3::Y;
    }
    let [x, y, _] = tangent.make_unit_ortho_vectors(Some(y))?;
    let mut placement = Axis2Placement::IDENTITY;
    placement.set_canonical(origin, x, y)?;
    for curve in output.iter_mut() {
        curve.transform(k, &placement, None)?;
    }
    Ok(())
}
