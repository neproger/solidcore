#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! High-level `PlanarSweep2`: normalize -> source selection -> boundaries -> B-rep.

use solidcore::curve::curve::p2::Curve;
use solidcore::kernel::Kernel;
use solidcore::math::{Point2, Point3, Vec3};
use solidcore::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
use solidcore::primitive::api_curves;
use solidcore::topology::brep::is_outward_solid;
use solidcore::topology::brep::p2::is_manifold_solid;
use solidcore::topology::brep::validation::{CheckLevel, check_brep};
use solidcore::topology::integrity;

use solidcore::surface::surface::SurfaceKind;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;

use universal_miter_prototype::boundary::{BoundaryInput, BoundarySolver, PathFrame};
use universal_miter_prototype::boundary_validation::{
    BoundaryValidationError, validate_joint_boundary,
};
use universal_miter_prototype::level_b_geometry::SurfaceBuildOptions;
use universal_miter_prototype::planar_pipeline::{
    DEFAULT_SEGMENTS, GeometryPolicy, PlanarSweepError, PlanarSweepResult, build_planar_sweep,
    build_planar_sweep_with_policy,
};
use universal_miter_prototype::planar_sweep2::{normalize_planar_sweep, to_bspline};
use universal_miter_prototype::source_selection::{
    GeometryProvenance, SweepSourceKind, select_sweep_sources,
};

fn line(start: Point3, end: Point3) -> Curve {
    Curve::from(
        api_curves::create_line_segment(start, end)
            .expect("line")
            .base_with_nurb()
            .expect("line nurb")
            .into_owned(),
    )
}

fn arc(center: Point3, radius: f64, start: f64, end: f64) -> Curve {
    Curve::from(api_curves::create_arc(center, radius, start, end).expect("arc"))
}

fn circle(k: &Kernel) -> Curve {
    Curve::from(api_curves::create_analytic_circle(k, Point3::ZERO, 0.25).expect("circle"))
}

fn ellipse(k: &Kernel) -> Curve {
    Curve::from(api_curves::create_ellipse(k, Point3::ZERO, 0.3, 0.2).expect("ellipse"))
}

fn line_line_line() -> Vec<Curve> {
    vec![
        line(Point3::ZERO, Point3::new(1.0, 0.0, 0.0)),
        line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)),
        line(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
    ]
}

fn line_arc_line() -> Vec<Curve> {
    let sqrt3 = 3.0_f64.sqrt();
    let path0 = line(Point3::ZERO, Point3::new(5.0, 0.0, 0.0));
    let path1 = arc(Point3::new(5.0, 4.0, 0.0), 4.0, -90.0, 60.0);
    let end = Point3::new(7.0, 4.0 + 2.0 * sqrt3, 0.0);
    let tangent = Vec3::new(-sqrt3 / 2.0, 0.5, 0.0);
    let path2 = line(end, end + tangent * 2.0);
    vec![path0, path1, path2]
}

fn options() -> SurfaceBuildOptions {
    SurfaceBuildOptions {
        tolerance: 1.0e-5,
        max_refinements: 6,
        max_sections: 64,
        ..SurfaceBuildOptions::default()
    }
}

fn validate(
    k: &mut Kernel,
    result: &mut PlanarSweepResult,
    faces: usize,
    edges: usize,
    vertices: usize,
) {
    integrity::check(&result.brep).expect("integrity");
    let check = check_brep(k, &mut result.brep, CheckLevel::Level2).expect("check_brep");
    assert!(check.is_valid(), "check_brep failures: {:?}", check.lines());
    assert_eq!(result.brep.faces().len(), faces);
    assert_eq!(result.brep.edges().len(), edges);
    assert_eq!(result.brep.vertices().len(), vertices);
    assert!(is_manifold_solid(&result.brep).expect("manifold"));
    assert!(is_outward_solid(k, &mut result.brep).unwrap_or(false));

    for (joint, boundary) in result.boundaries.iter().enumerate() {
        let surface_a = &result.sources.get(joint).expect("left source").surface;
        let surface_b = &result.sources.get(joint + 1).expect("right source").surface;
        let validation = validate_joint_boundary(k, boundary, surface_a, surface_b, 1.0e-4, 17)
            .unwrap_or_else(|error| panic!("boundary oracle failed: {error}"));
        println!(
            "boundary oracle joint {joint}: samples={} max_gap={:.3e} join={:.3e} vertex={:.3e} loop={:.3e}",
            validation.samples,
            validation.max_gap,
            validation.max_join_gap,
            validation.max_vertex_gap,
            validation.loop_gap
        );
    }
}

fn kinds(result: &PlanarSweepResult) -> Vec<SweepSourceKind> {
    result.sources.iter().map(|piece| piece.kind).collect()
}

#[test]
fn open_arc_arc_source_boundary_audit() {
    let k = Kernel::default();
    let profile = circle(&k);
    let paths = vec![
        arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0),
        arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 45.0),
    ];
    let normalized = normalize_planar_sweep(&k, std::slice::from_ref(&profile), &paths, true, 3)
        .expect("normalize");
    let selected = select_sweep_sources(&k, &normalized, &options()).expect("sources");
    let frame = PathFrame::new(&k, &normalized.path_original).expect("frame");
    let boundary_profile = to_bspline(&selected[0].profile).expect("boundary profile");
    let path_bs: Vec<_> = normalized
        .path_original
        .iter()
        .map(|path| to_bspline(path).expect("path"))
        .collect();
    let boundary = BoundarySolver::default()
        .solve(&BoundaryInput {
            kernel: &k,
            frame,
            profile: &boundary_profile,
            path_a: &path_bs[0],
            path_b: &path_bs[1],
            extension: normalized.extension,
            segments: 4,
            samples_per_segment: 17,
        })
        .expect("boundary");
    println!(
        "open arc-arc boundary: strategy={:?} exact={} junction={} residual={:.3e}",
        boundary.diagnostics.strategy,
        boundary.diagnostics.exact_samples,
        boundary.diagnostics.junction_samples,
        boundary.diagnostics.max_residual
    );
    let validation = validate_joint_boundary(
        &k,
        &boundary,
        &selected[0].surface,
        &selected[1].surface,
        1.0e-4,
        17,
    );
    assert!(
        matches!(
            validation,
            Err(BoundaryValidationError::SurfaceMembershipFailure { .. })
        ),
        "open arc-arc oracle should reject the candidate: {validation:?}"
    );
    for (vertex_index, vertex) in boundary.seam_vertices.iter().enumerate() {
        println!(
            "seam vertex {vertex_index} {vertex:?}: to piece0 start {:.6} end {:.6}; to piece1 start {:.6}",
            (*vertex - selected[0].path_start).length(),
            (*vertex - selected[0].path_end).length(),
            (*vertex - selected[1].path_start).length()
        );
        for (surface_index, piece) in selected.iter().enumerate() {
            let domain = piece
                .surface
                .natural_uv_domain_with_nurb(&k)
                .expect("mapping domain");
            let drop = piece
                .surface
                .drop_point::<NoObject>(&k, *vertex, &domain, &DropPointOptions::default())
                .expect("mapping drop");
            let uv = drop.result_uv;
            let in_u = uv.x >= domain.min().x && uv.x <= domain.max().x;
            let in_v = uv.y >= domain.min().y && uv.y <= domain.max().y;
            println!(
                "  map piece {surface_index}: success={} gap={:.6e} uv={uv:?} \
                 u_domain=[{:.6},{:.6}] v_domain=[{:.6},{:.6}] in_u={in_u} in_v={in_v}",
                drop.success,
                drop.gap,
                domain.min().x,
                domain.max().x,
                domain.min().y,
                domain.max().y
            );
        }
        let prepared_start = normalized.placements[0]
            .path_piece
            .evaluate(
                normalized.placements[0].path_piece.natural_interval().min(),
                0,
                true,
                false,
            )
            .expect("prepared start")[0];
        let prepared_end = normalized.placements[0]
            .path_piece
            .evaluate(
                normalized.placements[0].path_piece.natural_interval().max(),
                0,
                true,
                false,
            )
            .expect("prepared end")[0];
        println!(
            "  path: original_start_d={:.6e} original_end_d={:.6e} prepared_start_d={:.6e} prepared_end_d={:.6e}",
            (*vertex - selected[0].path_start).length(),
            (*vertex - selected[0].path_end).length(),
            (*vertex - prepared_start).length(),
            (*vertex - prepared_end).length()
        );
        let mut best = f64::INFINITY;
        for u in [0.0, 1.0] {
            for sample in 0..=64 {
                let point = selected[0]
                    .surface
                    .evaluate(
                        &k,
                        Point2::new(u, f64::from(sample) / 64.0),
                        0,
                        0,
                        EvaluateFlags::default(),
                    )
                    .expect("section point")[0][0];
                best = best.min((*vertex - point).length());
            }
        }
        println!("  nearest section point distance {best:.9e}");
    }
    for (index, piece) in selected.iter().enumerate() {
        let interval = piece.profile.natural_interval();
        let t = interval.min() + (interval.max() - interval.min()) * 0.75;
        let profile_point = piece.profile.evaluate_point(&k, t).expect("profile .75");
        let u = if index == 0 { 1.0 } else { 0.0 };
        let section_point = piece
            .surface
            .evaluate(&k, Point2::new(u, 0.75), 0, 0, EvaluateFlags::default())
            .expect("section .75")[0][0];
        println!(
            "V=.75 piece {index}: profile={profile_point:?} section_u={u:.1}={section_point:?} \
             distance={:.9e}",
            (profile_point - section_point).length()
        );
    }
    for (index, piece) in selected.iter().enumerate() {
        let domain = piece
            .surface
            .natural_uv_domain_with_nurb(&k)
            .expect("domain");
        let mut gap = 0.0_f64;
        for segment in &boundary.seam_segments {
            let interval = segment.natural_interval();
            for sample in 0..=16 {
                let point = segment
                    .evaluate_point(&k, interval.evaluate(f64::from(sample) / 16.0))
                    .expect("seam point");
                let drop = piece
                    .surface
                    .drop_point::<NoObject>(&k, point, &domain, &DropPointOptions::default())
                    .expect("drop");
                gap = gap.max(drop.gap);
            }
        }
        println!(
            "open arc-arc piece {index}: kind={:?} type={} u=[{:.6},{:.6}] v=[{:.6},{:.6}] gap={gap:.3e}",
            piece.kind,
            surface_label(piece.surface.kind()),
            domain.min().x,
            domain.max().x,
            domain.min().y,
            domain.max().y
        );
    }
}

fn surface_label(kind: &SurfaceKind) -> &'static str {
    match kind {
        SurfaceKind::Torus(_) => "Torus",
        SurfaceKind::SurfOfExtrusion(_) => "SurfOfExtrusion",
        SurfaceKind::BSpline(_) => "BSpline",
        _ => "Other",
    }
}

#[test]
fn high_level_open_arc_arc_returns_typed_refusal() {
    let mut k = Kernel::default();
    let profile = circle(&k);
    let paths = vec![
        arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0),
        arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 45.0),
    ];
    let error = build_planar_sweep(
        &mut k,
        &[profile],
        &paths,
        true,
        3,
        DEFAULT_SEGMENTS,
        &options(),
    )
    .expect_err("open arc-arc must return a typed refusal");
    println!("open arc-arc typed refusal: {error}");
    assert!(
        matches!(
            error,
            PlanarSweepError::BoundaryValidation(
                BoundaryValidationError::SurfaceMembershipFailure { .. }
                    | BoundaryValidationError::OutsideSourceDomain { .. }
            )
        ),
        "unexpected refusal: {error:?}"
    );
}

#[test]
fn high_level_exact_line_line_line() {
    let mut k = Kernel::default();
    let profile = circle(&k);
    let mut result = build_planar_sweep(
        &mut k,
        &[profile],
        &line_line_line(),
        true,
        3,
        DEFAULT_SEGMENTS,
        &options(),
    )
    .expect("pipeline");
    assert_eq!(
        kinds(&result),
        vec![
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactExtrusion
        ]
    );
    assert!(
        result
            .sources
            .iter()
            .all(|piece| matches!(piece.surface.kind(), SurfaceKind::SurfOfExtrusion(_)))
    );
    validate(&mut k, &mut result, 14, 28, 16);
}

#[test]
fn high_level_exact_line_arc_line() {
    let mut k = Kernel::default();
    let profile = circle(&k);
    let mut result = build_planar_sweep(
        &mut k,
        &[profile],
        &line_arc_line(),
        true,
        3,
        DEFAULT_SEGMENTS,
        &options(),
    )
    .expect("pipeline");
    assert_eq!(
        kinds(&result),
        vec![
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactRevolution,
            SweepSourceKind::ExactExtrusion
        ]
    );
    assert!(matches!(
        result.sources[0].surface.kind(),
        SurfaceKind::SurfOfExtrusion(_)
    ));
    assert!(matches!(
        result.sources[1].surface.kind(),
        SurfaceKind::Torus(_)
    ));
    assert!(matches!(
        result.sources[2].surface.kind(),
        SurfaceKind::SurfOfExtrusion(_)
    ));
    validate(&mut k, &mut result, 14, 28, 16);
}

#[test]
fn high_level_mixed_exact_level_b_line_arc_line() {
    let mut k = Kernel::default();
    let profile = ellipse(&k);
    let mut result = build_planar_sweep(
        &mut k,
        &[profile],
        &line_arc_line(),
        true,
        3,
        DEFAULT_SEGMENTS,
        &options(),
    )
    .expect("pipeline");
    assert_eq!(
        kinds(&result),
        vec![
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::LevelBApproximation,
            SweepSourceKind::ExactExtrusion
        ]
    );
    assert_eq!(result.sources[1].diagnostics.path_kind, "CircleArc");
    assert!(!result.sources[1].diagnostics.circular_profile);
    assert!(matches!(
        result.sources[0].surface.kind(),
        SurfaceKind::SurfOfExtrusion(_)
    ));
    assert!(matches!(
        result.sources[1].surface.kind(),
        SurfaceKind::BSpline(_)
    ));
    assert!(matches!(
        result.sources[2].surface.kind(),
        SurfaceKind::SurfOfExtrusion(_)
    ));
    validate(&mut k, &mut result, 14, 28, 16);
}

#[test]
fn geometry_policy_defaults_to_exact_and_provenance_is_original() {
    let mut k = Kernel::default();
    let profile = circle(&k);
    let result = build_planar_sweep(
        &mut k,
        &[profile],
        &line_line_line(),
        true,
        3,
        DEFAULT_SEGMENTS,
        &options(),
    )
    .expect("exact policy pipeline");
    assert_eq!(result.geometry_policy, GeometryPolicy::ExactOnly);
    assert!(
        result
            .sources
            .iter()
            .all(|piece| piece.provenance == GeometryProvenance::Original)
    );
}

#[test]
fn unimplemented_geometry_policies_refuse_without_building() {
    let mut k = Kernel::default();
    let profile = circle(&k);
    let paths = vec![
        arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0),
        arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 45.0),
    ];
    for policy in [
        GeometryPolicy::AllowExtension,
        GeometryPolicy::AllowAdaptiveBridge,
    ] {
        let error = build_planar_sweep_with_policy(
            &mut k,
            std::slice::from_ref(&profile),
            &paths,
            true,
            3,
            DEFAULT_SEGMENTS,
            &options(),
            policy,
        )
        .expect_err("unimplemented policy must not build");
        assert!(matches!(
            error,
            PlanarSweepError::StrategyNotImplemented(actual) if actual == policy
        ));
    }
}
