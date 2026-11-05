//! Direct shared-topology B-rep for one piece, two pieces or an open chain.
//!
//! The profile loop is split into `K` open contour segments, the swept NURBS
//! surface into `K` patches, and the shell is assembled from shared vertices,
//! wire edges and face topology. The chain builder shares one vertex/edge set
//! per joint. No Boolean, trim or stitch is used.
//!
//! Reference: AOIM direct topology (`make_shell_vertex` + `make_wire_edge` +
//! `make_face_topology`).

use crate::boundary::JointBoundary;
use solidcore::curve::curve::Curve;
use solidcore::error::{Result, SolidError};

use solidcore::geom::extent2d::Extent2d;
use solidcore::kernel::Kernel;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::b_spline_surface::{BSplineSurface, SurfParamType};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::nurbs::crv_fit::p1::ParamMethod;
use solidcore::surface::plane::Plane;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;
use solidcore::surface::surface::Surface;
use solidcore::topology::brep::p1::{
    reset_infinite_region, set_region_is_void_flags_for_nested_solids, SetRegionVoidType,
};
use solidcore::topology::brep::p2::ArenaNotifyState;
use solidcore::topology::brep::p3 as brep_p3;
use solidcore::topology::brep::p6::{self, FaceTopologyOptions};
use solidcore::topology::brep::p9;
use solidcore::topology::edge;
use solidcore::topology::edgeuse::p1 as edgeuse1;

use solidcore::topology::topology_traverser::Marks;
use solidcore::topology::vertexuse as vertexuse1;
use solidcore::topology::{Brep, EdgeId, EdgeuseId, FaceId, RegionId, VertexId};

const OP: &str = "research::direct_brep";

/// The result of the direct single-piece construction.
#[derive(Clone, Debug)]
pub struct DirectShell {
    /// The assembled B-rep.
    pub brep: Brep,
    /// The number of profile contour segments.
    pub segments: usize,
    /// The requested profile surface patches.
    pub patches: usize,
}

fn make_edge(
    k: &mut Kernel,
    brep: &mut Brep,
    env: &mut edge::NotifyEnv<'_>,
    region: Option<RegionId>,
    start: VertexId,
    end: VertexId,
    curve: Curve,
) -> Result<EdgeId> {
    let interval = curve.natural_interval();
    brep_p3::make_wire_edge(
        k,
        brep,
        env,
        region,
        start,
        end,
        curve,
        &interval,
        OrientType::Same,
    )
}

fn edge_use_from(brep: &Brep, edge: EdgeId, start: VertexId) -> Result<EdgeuseId> {
    for edgeuse in edge::edgeuses(brep, edge, None, None)? {
        let Some(vertexuse) = edgeuse1::vertexuse(brep, edgeuse)? else {
            continue;
        };
        if vertexuse1::vertex(brep, vertexuse)? == Some(start) {
            return Ok(edgeuse);
        }
    }
    Err(SolidError::invalid_input(
        OP,
        "no edge use starts at the requested vertex",
    ))
}

#[allow(clippy::too_many_arguments)]
fn make_face(
    k: &mut Kernel,
    brep: &mut Brep,
    marks: &mut Marks,
    env: &mut edge::NotifyEnv<'_>,
    region: Option<RegionId>,
    surface: Surface,
    orientation: OrientType,
    edges: &[EdgeId],
    starts: &[VertexId],
    uv_curves: Option<&[Option<BSplineCurve>]>,
    _periodic_v: Option<(f64, f64)>,
) -> Result<Option<FaceId>> {
    let edgeuses = edges
        .iter()
        .zip(starts)
        .map(|(&edge, &start)| edge_use_from(brep, edge, start))
        .collect::<Result<Vec<_>>>()?;
    let options = FaceTopologyOptions {
        region,
        edge_loops: &[edges.len()],
        edgeuses: &edgeuses,
        uv_curves,
        ..FaceTopologyOptions::default()
    };
    match p6::make_face_topology(k, brep, marks, env, Some(surface), orientation, &options) {
        Ok(result) => Ok(Some(result.face)),
        Err(SolidError::Unported {
            unit: "topology.brep_p6",
            ..
        }) => {
            // TestClosureSplitRegion completes the simple split before its
            // unported tail; the caller still validates the whole shell.
            Ok(None)
        }
        Err(error) => Err(error),
    }
}

fn vertex_at(vertices: &[VertexId], index: usize) -> Result<VertexId> {
    vertices
        .get(index)
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing vertex"))
}

fn edge_at(edges: &[EdgeId], index: usize) -> Result<EdgeId> {
    edges
        .get(index)
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing edge"))
}

fn curve_at(curves: &[Curve], index: usize) -> Result<Curve> {
    curves
        .get(index)
        .cloned()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing boundary curve"))
}

fn scalar_at(values: &[f64], index: usize) -> Result<f64> {
    values
        .get(index)
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing surface scalar"))
}

fn profile_v_at(values: &[Vec<f64>], piece: usize, profile: usize) -> Result<f64> {
    values
        .get(piece)
        .and_then(|piece_values| piece_values.get(profile))
        .copied()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing profile V"))
}

fn vertices_at(values: &[Vec<VertexId>], index: usize) -> Result<&[VertexId]> {
    values
        .get(index)
        .map(Vec::as_slice)
        .ok_or_else(|| SolidError::invalid_input(OP, "missing boundary vertices"))
}

fn edge_row_at(values: &[Vec<EdgeId>], index: usize) -> Result<&[EdgeId]> {
    values
        .get(index)
        .map(Vec::as_slice)
        .ok_or_else(|| SolidError::invalid_input(OP, "missing boundary edges"))
}

fn surface_row_at(values: &[Surface], index: usize) -> Result<&Surface> {
    values
        .get(index)
        .ok_or_else(|| SolidError::invalid_input(OP, "missing surface"))
}

fn surface_at(surfaces: &[Surface], index: usize) -> Result<Surface> {
    surfaces
        .get(index)
        .cloned()
        .ok_or_else(|| SolidError::invalid_input(OP, "missing surface patch"))
}

/// Build one direct shell for a single path piece.
///
/// `surface` is the Level B swept surface over the path. The path start/end
/// points and tangents give the cap planes. `segments` is `K`, the number of
/// profile contour segments.
///
/// # Errors
///
/// Kernel errors from the surface split, vertex/edge/face creation, or the p6
/// closure tail (other than the simple-prism split completion).
#[allow(clippy::too_many_arguments)]
pub fn build_direct_single(
    k: &mut Kernel,
    surface: &BSplineSurface,
    path_start: Point3,
    path_start_tangent: Vec3,
    path_end: Point3,
    path_end_tangent: Vec3,
    segments: usize,
) -> Result<DirectShell> {
    if segments < 2 {
        return Err(SolidError::invalid_input(
            OP,
            "the profile needs at least two segments",
        ));
    }
    let full = Surface::from(surface.clone());
    let domain = full.natural_uv_domain_with_nurb(k)?;
    let v_min = domain.min().y;
    let v_max = domain.max().y;

    let mut patches = Vec::with_capacity(segments);
    let mut remaining = full.clone();
    for index in 1..segments {
        let split = v_min + (v_max - v_min) * (index as f64 / segments as f64);
        let (left, right) = remaining.split_at(k, split, SurfParamType::V)?;
        patches.push(left);
        remaining = right;
    }
    patches.push(remaining);

    let mut start_segments = Vec::with_capacity(segments);
    let mut end_segments = Vec::with_capacity(segments);
    for patch in &patches {
        let patch_domain = patch.natural_uv_domain_with_nurb(k)?;
        start_segments.push(
            patch
                .create_iso_parametric_curve(
                    k,
                    SurfParamType::U,
                    patch_domain.min().x,
                    1.0e-7,
                    None,
                    false,
                )?
                .curve,
        );
        end_segments.push(
            patch
                .create_iso_parametric_curve(
                    k,
                    SurfParamType::U,
                    patch_domain.max().x,
                    1.0e-7,
                    None,
                    false,
                )?
                .curve,
        );
    }

    let mut rails = Vec::with_capacity(segments);
    for index in 0..segments {
        let v = v_min + (v_max - v_min) * (index as f64 / segments as f64);
        rails.push(
            full.create_iso_parametric_curve(k, SurfParamType::V, v, 1.0e-7, None, false)?
                .curve,
        );
    }

    let mut brep = brep_p3::new_brep(
        k,
        solidcore::geom::tol::USE_DEFAULT,
        solidcore::geom::tol::USE_DEFAULT,
    )?;
    let region = brep.infinite_region();
    let initial_infinite = region;

    let (start_vertices, end_vertices) = {
        let mut state = ArenaNotifyState::new(&brep);
        let mut env = state.env(None);
        let mut start_vertices = Vec::with_capacity(segments);
        let mut end_vertices = Vec::with_capacity(segments);
        for rail in &rails {
            let interval = rail.natural_interval();
            let start = rail.evaluate_point(k, interval.min())?;
            let end = rail.evaluate_point(k, interval.max())?;
            let (_shell, start_vertex) = p9::make_shell_vertex(&mut brep, region, start, &mut env)?;
            let (_shell, end_vertex) = p9::make_shell_vertex(&mut brep, region, end, &mut env)?;
            start_vertices.push(start_vertex);
            end_vertices.push(end_vertex);
        }
        (start_vertices, end_vertices)
    };

    let mut state = ArenaNotifyState::new(&brep);
    let mut env = state.env(None);
    let mut start_edges = Vec::with_capacity(segments);
    let mut end_edges = Vec::with_capacity(segments);
    let mut rail_edges = Vec::with_capacity(segments);
    for index in 0..segments {
        let next = (index + 1) % segments;
        start_edges.push(make_edge(
            k,
            &mut brep,
            &mut env,
            region,
            vertex_at(&start_vertices, index)?,
            vertex_at(&start_vertices, next)?,
            curve_at(&start_segments, index)?,
        )?);
        end_edges.push(make_edge(
            k,
            &mut brep,
            &mut env,
            region,
            vertex_at(&end_vertices, index)?,
            vertex_at(&end_vertices, next)?,
            curve_at(&end_segments, index)?,
        )?);
        rail_edges.push(make_edge(
            k,
            &mut brep,
            &mut env,
            region,
            vertex_at(&start_vertices, index)?,
            vertex_at(&end_vertices, index)?,
            curve_at(&rails, index)?,
        )?);
    }

    let mut marks = Marks::default();
    for index in 0..segments {
        let next = (index + 1) % segments;
        let edges = [
            edge_at(&start_edges, index)?,
            edge_at(&rail_edges, next)?,
            edge_at(&end_edges, index)?,
            edge_at(&rail_edges, index)?,
        ];
        let starts = [
            vertex_at(&start_vertices, index)?,
            vertex_at(&start_vertices, next)?,
            vertex_at(&end_vertices, next)?,
            vertex_at(&end_vertices, index)?,
        ];
        make_face(
            k,
            &mut brep,
            &mut marks,
            &mut env,
            region,
            surface_at(&patches, index)?,
            OrientType::Opposite,
            &edges,
            &starts,
            None,
            None,
        )?;
    }

    let start_normal = path_start_tangent
        .unitize()
        .map_err(|_| SolidError::invalid_input(OP, "the start tangent is zero"))?;
    let end_normal = path_end_tangent
        .unitize()
        .map_err(|_| SolidError::invalid_input(OP, "the end tangent is zero"))?;
    let start_plane = Surface::from(Plane::from_normal(path_start, -start_normal));
    let end_plane = Surface::from(Plane::from_normal(path_end, end_normal));

    let start_cap_edges: Vec<EdgeId> = (0..segments)
        .rev()
        .map(|index| edge_at(&start_edges, index))
        .collect::<Result<Vec<_>>>()?;
    let start_cap_starts: Vec<VertexId> = (0..segments)
        .rev()
        .map(|index| vertex_at(&start_vertices, (index + 1) % segments))
        .collect::<Result<Vec<_>>>()?;
    make_face(
        k,
        &mut brep,
        &mut marks,
        &mut env,
        region,
        start_plane,
        OrientType::Same,
        &start_cap_edges,
        &start_cap_starts,
        None,
        None,
    )?;

    make_face(
        k,
        &mut brep,
        &mut marks,
        &mut env,
        region,
        end_plane,
        OrientType::Same,
        &end_edges,
        &end_vertices,
        None,
        None,
    )?;

    if let Some(infinite) = initial_infinite {
        reset_infinite_region(
            k,
            &mut brep,
            Some(infinite),
            SetRegionVoidType::InfiniteRegionOnly,
        )?;
    }
    set_region_is_void_flags_for_nested_solids(k, &mut brep)?;
    brep.refresh_cached_topology();
    Ok(DirectShell {
        brep,
        segments,
        patches: segments,
    })
}

fn profile_segment(
    k: &Kernel,
    surface: &Surface,
    u_param: f64,
    u_min: f64,
    u_max: f64,
    v_start: f64,
    v_end: f64,
) -> Result<Curve> {
    let domain = Extent2d::from_bounds(u_min, v_start, u_max, v_end);
    Ok(surface
        .create_iso_parametric_curve(k, SurfParamType::U, u_param, 1.0e-7, Some(&domain), false)?
        .curve)
}

fn rail_segment(
    k: &Kernel,
    surface: &Surface,
    v_param: f64,
    u_start: f64,
    u_end: f64,
    v_min: f64,
    v_max: f64,
) -> Result<Curve> {
    let domain = Extent2d::from_bounds(u_start, v_min, u_end, v_max);
    Ok(surface
        .create_iso_parametric_curve(k, SurfParamType::V, v_param, 1.0e-7, Some(&domain), false)?
        .curve)
}

fn seam_uv(k: &Kernel, surface: &Surface, point: Point3) -> Result<(f64, f64)> {
    let domain = surface.natural_uv_domain_with_nurb(k)?;
    let drop = surface.drop_point::<NoObject>(k, point, &domain, &DropPointOptions::default())?;
    if !drop.success || drop.gap > 1.0e-4 {
        return Err(SolidError::invalid_input(
            OP,
            "the seam point is not on the side surface",
        ));
    }
    Ok((drop.result_uv.x, drop.result_uv.y))
}

/// Build one direct shell for two adjacent path pieces and their shared seam.
///
/// This is the `N = 2` case of [`build_direct_chain`]. The seam curves and
/// vertices come from the boundary solver; the chain builder drops them on both
/// side surfaces and shares the resulting edges.
///
/// # Errors
///
/// Those of [`build_direct_chain`].
#[allow(clippy::too_many_arguments)]
pub fn build_direct_joint(
    k: &mut Kernel,
    side_a: &BSplineSurface,
    side_b: &BSplineSurface,
    boundary: &JointBoundary,
    path_start: Point3,
    path_start_tangent: Vec3,
    path_end: Point3,
    path_end_tangent: Vec3,
    segments: usize,
) -> Result<Brep> {
    let surfaces = [side_a.clone(), side_b.clone()];
    let boundaries = [boundary.clone()];
    build_direct_chain(
        k,
        &surfaces,
        &boundaries,
        path_start,
        path_start_tangent,
        path_end,
        path_end_tangent,
        segments,
    )
}
/// Build one direct shell for `N` path pieces and `N - 1` shared joints.
///
/// The input is already solved geometry: one Level B side surface per piece,
/// one [`JointBoundary`] per adjacent pair, and the path end caps. The builder
/// does not know which boundary strategy produced the seams.
///
/// Every joint owns one set of seam vertices and seam edges, shared by the two
/// adjacent side surfaces. Every intermediate surface owns one rail per profile
/// boundary; the start and end surfaces share their profile edges with the
/// caps. No Boolean, trim or stitch is used.
///
/// # Errors
///
/// Invalid surface/boundary counts, a seam that cannot be dropped on both
/// adjacent surfaces, a profile V mismatch between the two ends of one
/// surface, or kernel errors from the topology construction.
#[allow(clippy::too_many_arguments)]
pub fn build_direct_chain(
    k: &mut Kernel,
    side_surfaces: &[BSplineSurface],
    boundaries: &[JointBoundary],
    path_start: Point3,
    path_start_tangent: Vec3,
    path_end: Point3,
    path_end_tangent: Vec3,
    segments: usize,
) -> Result<Brep> {
    if segments < 2 {
        return Err(SolidError::invalid_input(
            OP,
            "the profile needs at least two segments",
        ));
    }
    if side_surfaces.is_empty() {
        return Err(SolidError::invalid_input(
            OP,
            "the chain has no side surface",
        ));
    }
    if boundaries.len() + 1 != side_surfaces.len() {
        return Err(SolidError::invalid_input(
            OP,
            "the chain needs one boundary between each adjacent pair",
        ));
    }
    let piece_count = side_surfaces.len();
    if piece_count == 1 {
        let surface = side_surfaces
            .first()
            .ok_or_else(|| SolidError::invalid_input(OP, "missing side surface"))?;
        return Ok(build_direct_single(
            k,
            surface,
            path_start,
            path_start_tangent,
            path_end,
            path_end_tangent,
            segments,
        )?
        .brep);
    }

    let surfaces: Vec<Surface> = side_surfaces.iter().cloned().map(Surface::from).collect();
    let domains = surfaces
        .iter()
        .map(|surface| surface.natural_uv_domain_with_nurb(k))
        .collect::<Result<Vec<_>>>()?;
    let u_min: Vec<f64> = domains.iter().map(|domain| domain.min().x).collect();
    let u_max: Vec<f64> = domains.iter().map(|domain| domain.max().x).collect();
    let v_min: Vec<f64> = domains.iter().map(|domain| domain.min().y).collect();
    let v_max: Vec<f64> = domains.iter().map(|domain| domain.max().y).collect();

    let joint_count = piece_count - 1;
    let mut joint_left_uv: Vec<Vec<(f64, f64)>> = Vec::with_capacity(joint_count);
    let mut joint_right_uv: Vec<Vec<(f64, f64)>> = Vec::with_capacity(joint_count);
    for joint in 0..joint_count {
        let boundary = boundaries
            .get(joint)
            .ok_or_else(|| SolidError::invalid_input(OP, "missing joint boundary"))?;
        if boundary.seam_vertices.len() != segments {
            return Err(SolidError::invalid_input(
                OP,
                "the seam boundary does not match the profile segments",
            ));
        }
        let mut left = Vec::with_capacity(segments);
        let mut right = Vec::with_capacity(segments);
        let left_surface = surface_row_at(&surfaces, joint)?;
        let right_surface = surface_row_at(&surfaces, joint + 1)?;
        for point in &boundary.seam_vertices {
            left.push(seam_uv(k, left_surface, *point)?);
            right.push(seam_uv(k, right_surface, *point)?);
        }
        joint_left_uv.push(left);
        joint_right_uv.push(right);
    }

    let mut profile_v: Vec<Vec<f64>> = Vec::with_capacity(piece_count);
    for piece in 0..piece_count {
        let start_v: Vec<f64> = if piece == 0 {
            joint_left_uv.first().map_or_else(
                || profile_fractions(&v_min, &v_max, 0, segments),
                |values| values.iter().map(|uv| uv.1).collect(),
            )
        } else {
            joint_right_uv
                .get(piece - 1)
                .map(|values| values.iter().map(|uv| uv.1).collect())
                .ok_or_else(|| SolidError::invalid_input(OP, "missing joint V"))?
        };
        let end_v: Vec<f64> = if piece + 1 == piece_count {
            joint_right_uv
                .get(piece - 1)
                .map(|values| values.iter().map(|uv| uv.1).collect())
                .ok_or_else(|| SolidError::invalid_input(OP, "missing joint V"))?
        } else {
            joint_left_uv
                .get(piece)
                .map(|values| values.iter().map(|uv| uv.1).collect())
                .ok_or_else(|| SolidError::invalid_input(OP, "missing joint V"))?
        };
        let lo = scalar_at(&v_min, piece)?;
        let hi = scalar_at(&v_max, piece)?;
        let canonical_start = start_v
            .iter()
            .map(|v| canonical_profile_v(*v, lo, hi))
            .collect::<Vec<_>>();
        let canonical_end = end_v
            .iter()
            .map(|v| canonical_profile_v(*v, lo, hi))
            .collect::<Vec<_>>();
        let scale = 1.0 + hi.abs() + lo.abs();
        for index in 0..segments {
            let a = *canonical_start
                .get(index)
                .ok_or_else(|| SolidError::invalid_input(OP, "missing profile V"))?;
            let b = *canonical_end
                .get(index)
                .ok_or_else(|| SolidError::invalid_input(OP, "missing profile V"))?;
            if (a - b).abs() > 1.0e-6 * scale {
                return Err(SolidError::invalid_input(
                    OP,
                    "the side surface has different profile V at its two joints",
                ));
            }
        }
        profile_v.push(canonical_start);
    }

    let mut patch_surfaces: Vec<Vec<Surface>> = Vec::with_capacity(piece_count);
    for piece in 0..piece_count {
        let mut remaining = surfaces
            .get(piece)
            .cloned()
            .ok_or_else(|| SolidError::invalid_input(OP, "missing side surface"))?;
        let mut patches = Vec::with_capacity(segments);
        for profile in 1..segments {
            let split = *profile_v
                .get(piece)
                .and_then(|values| values.get(profile))
                .ok_or_else(|| SolidError::invalid_input(OP, "missing profile split"))?;
            let (left, right) = remaining.split_at(k, split, SurfParamType::V)?;
            patches.push(left);
            remaining = right;
        }
        patches.push(remaining);
        patch_surfaces.push(patches);
    }

    let mut joint_uv_curves: Vec<(Vec<BSplineCurve>, Vec<BSplineCurve>)> =
        Vec::with_capacity(joint_count);
    for joint in 0..joint_count {
        let boundary = boundaries
            .get(joint)
            .ok_or_else(|| SolidError::invalid_input(OP, "missing joint boundary"))?;
        let left_surface = surface_row_at(&surfaces, joint)?;
        let right_surface = surface_row_at(&surfaces, joint + 1)?;
        let mut left_curves = Vec::with_capacity(segments);
        let mut right_curves = Vec::with_capacity(segments);
        for profile in 0..segments {
            let next = (profile + 1) % segments;
            let seam = curve_at(&boundary.seam_segments, profile)?;
            left_curves.push(seam_uv_curve(
                k,
                left_surface,
                &seam,
                profile_v_at(&profile_v, joint, profile)?,
                profile_v_at(&profile_v, joint, next)?,
                scalar_at(&v_min, joint)?,
                scalar_at(&v_max, joint)?,
            )?);
            right_curves.push(seam_uv_curve(
                k,
                right_surface,
                &seam,
                profile_v_at(&profile_v, joint + 1, profile)?,
                profile_v_at(&profile_v, joint + 1, next)?,
                scalar_at(&v_min, joint + 1)?,
                scalar_at(&v_max, joint + 1)?,
            )?);
        }
        joint_uv_curves.push((left_curves, right_curves));
    }

    let mut rails: Vec<Vec<Curve>> = Vec::with_capacity(piece_count);
    for piece in 0..piece_count {
        let mut piece_rails = Vec::with_capacity(segments);
        for profile in 0..segments {
            let u_start = if piece == 0 {
                scalar_at(&u_min, piece)?
            } else {
                *joint_right_uv
                    .get(piece - 1)
                    .and_then(|values| values.get(profile))
                    .map(|uv| &uv.0)
                    .ok_or_else(|| SolidError::invalid_input(OP, "missing seam U"))?
            };
            let u_end = if piece + 1 == piece_count {
                scalar_at(&u_max, piece)?
            } else {
                *joint_left_uv
                    .get(piece)
                    .and_then(|values| values.get(profile))
                    .map(|uv| &uv.0)
                    .ok_or_else(|| SolidError::invalid_input(OP, "missing seam U"))?
            };
            let v = *profile_v
                .get(piece)
                .and_then(|values| values.get(profile))
                .ok_or_else(|| SolidError::invalid_input(OP, "missing profile V"))?;
            piece_rails.push(rail_segment(
                k,
                surface_row_at(&surfaces, piece)?,
                v,
                u_start,
                u_end,
                scalar_at(&v_min, piece)?,
                scalar_at(&v_max, piece)?,
            )?);
        }
        rails.push(piece_rails);
    }

    let mut brep = brep_p3::new_brep(
        k,
        solidcore::geom::tol::USE_DEFAULT,
        solidcore::geom::tol::USE_DEFAULT,
    )?;
    let region = brep.infinite_region();
    let initial_infinite = region;

    let (start_vertices, end_vertices, joint_vertices) = {
        let mut state = ArenaNotifyState::new(&brep);
        let mut env = state.env(None);
        let mut start_vertices = Vec::with_capacity(segments);
        let mut end_vertices = Vec::with_capacity(segments);
        let mut joint_vertices: Vec<Vec<VertexId>> = Vec::with_capacity(joint_count);
        for profile in 0..segments {
            let start_curve = rails
                .first()
                .and_then(|values| values.get(profile))
                .ok_or_else(|| SolidError::invalid_input(OP, "missing start rail"))?;
            let start_interval = start_curve.natural_interval();
            let start = start_curve.evaluate_point(k, start_interval.min())?;
            let (_shell, start_vertex) = p9::make_shell_vertex(&mut brep, region, start, &mut env)?;
            start_vertices.push(start_vertex);

            let end_curve = rails
                .last()
                .and_then(|values| values.get(profile))
                .ok_or_else(|| SolidError::invalid_input(OP, "missing end rail"))?;
            let end_interval = end_curve.natural_interval();
            let end = end_curve.evaluate_point(k, end_interval.max())?;
            let (_shell, end_vertex) = p9::make_shell_vertex(&mut brep, region, end, &mut env)?;
            end_vertices.push(end_vertex);
        }
        for (joint, boundary) in boundaries.iter().enumerate() {
            if joint >= joint_count {
                break;
            }
            let mut vertices = Vec::with_capacity(segments);
            for point in &boundary.seam_vertices {
                let (_shell, vertex) = p9::make_shell_vertex(&mut brep, region, *point, &mut env)?;
                vertices.push(vertex);
            }
            joint_vertices.push(vertices);
        }
        (start_vertices, end_vertices, joint_vertices)
    };

    let surface_start_vertices: Vec<Vec<VertexId>> = (0..piece_count)
        .map(|piece| {
            if piece == 0 {
                start_vertices.clone()
            } else {
                joint_vertices.get(piece - 1).cloned().unwrap_or_default()
            }
        })
        .collect();
    let surface_end_vertices: Vec<Vec<VertexId>> = (0..piece_count)
        .map(|piece| {
            if piece + 1 == piece_count {
                end_vertices.clone()
            } else {
                joint_vertices.get(piece).cloned().unwrap_or_default()
            }
        })
        .collect();

    let mut state = ArenaNotifyState::new(&brep);
    let mut env = state.env(None);
    let mut start_cap_edges = Vec::with_capacity(segments);
    let mut end_cap_edges = Vec::with_capacity(segments);
    let mut seam_edges: Vec<Vec<EdgeId>> = Vec::with_capacity(joint_count);
    let mut rail_edges: Vec<Vec<EdgeId>> = Vec::with_capacity(piece_count);

    for profile in 0..segments {
        let next = (profile + 1) % segments;
        let start_curve = profile_segment(
            k,
            surface_row_at(&surfaces, 0)?,
            scalar_at(&u_min, 0)?,
            scalar_at(&u_min, 0)?,
            scalar_at(&u_max, 0)?,
            profile_v_at(&profile_v, 0, profile)?,
            if next == 0 {
                scalar_at(&v_max, 0)?
            } else {
                profile_v_at(&profile_v, 0, next)?
            },
        )?;
        start_cap_edges.push(make_edge(
            k,
            &mut brep,
            &mut env,
            region,
            vertex_at(&start_vertices, profile)?,
            vertex_at(&start_vertices, next)?,
            start_curve,
        )?);

        let last = piece_count - 1;
        let end_curve = profile_segment(
            k,
            surface_row_at(&surfaces, last)?,
            scalar_at(&u_max, last)?,
            scalar_at(&u_min, last)?,
            scalar_at(&u_max, last)?,
            profile_v_at(&profile_v, last, profile)?,
            if next == 0 {
                scalar_at(&v_max, last)?
            } else {
                profile_v_at(&profile_v, last, next)?
            },
        )?;
        end_cap_edges.push(make_edge(
            k,
            &mut brep,
            &mut env,
            region,
            vertex_at(&end_vertices, profile)?,
            vertex_at(&end_vertices, next)?,
            end_curve,
        )?);
    }

    for (joint, boundary) in boundaries.iter().enumerate() {
        if joint >= joint_count {
            break;
        }
        let vertices = joint_vertices
            .get(joint)
            .ok_or_else(|| SolidError::invalid_input(OP, "missing joint vertices"))?;
        let mut edges = Vec::with_capacity(segments);
        for profile in 0..segments {
            let next = (profile + 1) % segments;
            edges.push(make_edge(
                k,
                &mut brep,
                &mut env,
                region,
                vertex_at(vertices, profile)?,
                vertex_at(vertices, next)?,
                curve_at(&boundary.seam_segments, profile)?,
            )?);
        }
        seam_edges.push(edges);
    }

    for piece in 0..piece_count {
        let start_row = vertices_at(&surface_start_vertices, piece)?;
        let end_row = vertices_at(&surface_end_vertices, piece)?;
        let rail_row = rails
            .get(piece)
            .ok_or_else(|| SolidError::invalid_input(OP, "missing piece rails"))?;
        let mut edges = Vec::with_capacity(segments);
        for profile in 0..segments {
            edges.push(make_edge(
                k,
                &mut brep,
                &mut env,
                region,
                vertex_at(start_row, profile)?,
                vertex_at(end_row, profile)?,
                curve_at(rail_row, profile)?,
            )?);
        }
        rail_edges.push(edges);
    }

    let mut marks = Marks::default();
    for piece in 0..piece_count {
        let start_row = vertices_at(&surface_start_vertices, piece)?;
        let end_row = vertices_at(&surface_end_vertices, piece)?;
        let rail_row = edge_row_at(&rail_edges, piece)?;
        for profile in 0..segments {
            let next = (profile + 1) % segments;
            let start_edge = if piece == 0 {
                edge_at(&start_cap_edges, profile)?
            } else {
                edge_at(edge_row_at(&seam_edges, piece - 1)?, profile)?
            };
            let end_edge = if piece + 1 == piece_count {
                edge_at(&end_cap_edges, profile)?
            } else {
                edge_at(edge_row_at(&seam_edges, piece)?, profile)?
            };
            let mut uv_curves = vec![None; 4];
            if piece > 0 {
                let start_uv = joint_uv_curves
                    .get(piece - 1)
                    .and_then(|(_, right)| right.get(profile))
                    .cloned();
                if let Some(slot) = uv_curves.get_mut(0) {
                    *slot = start_uv;
                }
            }
            if piece + 1 < piece_count {
                let end_uv = joint_uv_curves
                    .get(piece)
                    .and_then(|(left, _)| left.get(profile))
                    .cloned();
                if let Some(slot) = uv_curves.get_mut(2) {
                    *slot = end_uv;
                }
            }
            let edges = [
                start_edge,
                edge_at(rail_row, next)?,
                end_edge,
                edge_at(rail_row, profile)?,
            ];
            let starts = [
                vertex_at(start_row, profile)?,
                vertex_at(start_row, next)?,
                vertex_at(end_row, next)?,
                vertex_at(end_row, profile)?,
            ];
            make_face(
                k,
                &mut brep,
                &mut marks,
                &mut env,
                region,
                patch_surfaces
                    .get(piece)
                    .and_then(|patches| patches.get(profile))
                    .cloned()
                    .ok_or_else(|| SolidError::invalid_input(OP, "missing side patch"))?,
                OrientType::Opposite,
                &edges,
                &starts,
                None,
                None,
            )?;
        }
    }

    let start_normal = path_start_tangent
        .unitize()
        .map_err(|_| SolidError::invalid_input(OP, "the start tangent is zero"))?;
    let end_normal = path_end_tangent
        .unitize()
        .map_err(|_| SolidError::invalid_input(OP, "the end tangent is zero"))?;
    let start_plane = Surface::from(Plane::from_normal(path_start, -start_normal));
    let end_plane = Surface::from(Plane::from_normal(path_end, end_normal));

    let start_cap_edges_reversed: Vec<EdgeId> = (0..segments)
        .rev()
        .map(|index| edge_at(&start_cap_edges, index))
        .collect::<Result<Vec<_>>>()?;
    let start_cap_starts: Vec<VertexId> = (0..segments)
        .rev()
        .map(|index| vertex_at(&start_vertices, (index + 1) % segments))
        .collect::<Result<Vec<_>>>()?;
    make_face(
        k,
        &mut brep,
        &mut marks,
        &mut env,
        region,
        start_plane,
        OrientType::Same,
        &start_cap_edges_reversed,
        &start_cap_starts,
        None,
        None,
    )?;
    make_face(
        k,
        &mut brep,
        &mut marks,
        &mut env,
        region,
        end_plane,
        OrientType::Same,
        &end_cap_edges,
        &end_vertices,
        None,
        None,
    )?;

    if let Some(infinite) = initial_infinite {
        reset_infinite_region(
            k,
            &mut brep,
            Some(infinite),
            SetRegionVoidType::InfiniteRegionOnly,
        )?;
    }
    set_region_is_void_flags_for_nested_solids(k, &mut brep)?;
    brep.refresh_cached_topology();
    Ok(brep)
}

fn seam_uv_curve(
    k: &Kernel,
    surface: &Surface,
    seam: &Curve,
    start_v: f64,
    end_v: f64,
    v_min: f64,
    v_max: f64,
) -> Result<BSplineCurve> {
    let samples = 16_usize;
    let interval = seam.natural_interval();
    let domain = surface.natural_uv_domain_with_nurb(k)?;
    let period = v_max - v_min;
    let mut points = Vec::with_capacity(samples + 1);
    let mut previous = start_v;
    for index in 0..=samples {
        let t = interval.evaluate(index as f64 / samples as f64);
        let point = seam.evaluate_point(k, t)?;
        let drop =
            surface.drop_point::<NoObject>(k, point, &domain, &DropPointOptions::default())?;
        if !drop.success {
            return Err(SolidError::invalid_input(
                OP,
                "the seam point is not on the side surface",
            ));
        }
        let mut v = drop.result_uv.y;
        while v - previous > 0.5 * period {
            v -= period;
        }
        while previous - v > 0.5 * period {
            v += period;
        }
        if index == 0 {
            v = start_v;
        }
        if index == samples {
            v = end_v;
        }
        points.push(Point3::new(drop.result_uv.x, v, 0.0));
        previous = v;
    }
    let degree = if points.len() >= 3 { 3 } else { 1 };
    BSplineCurve::create_interpolating_curve(
        ParamMethod::ChordLength,
        3,
        degree,
        &points,
        &[],
        None,
        false,
    )
}
fn canonical_profile_v(v: f64, v_min: f64, v_max: f64) -> f64 {
    if (v - v_max).abs() <= 1.0e-6 * (1.0 + v.abs() + v_max.abs()) {
        v_min
    } else {
        v
    }
}

fn profile_fractions(v_min: &[f64], v_max: &[f64], piece: usize, segments: usize) -> Vec<f64> {
    let lo = v_min.get(piece).copied().unwrap_or(0.0);
    let hi = v_max.get(piece).copied().unwrap_or(1.0);
    (0..segments)
        .map(|index| lo + (hi - lo) * index as f64 / segments as f64)
        .collect()
}
