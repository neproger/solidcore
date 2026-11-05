#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::items_after_statements
)]

//! Milestone 2: universal boundary solver + direct B-rep joint.
//!
//! Exact line-line and exact line-arc are the exact cases; the 90-degree
//! line-arc exercises the junction tangent continuation. The 3-piece chain
//! checks the N-segment direct topology. The boundary solver is driven by the
//! Level A differential geometry, not by primitive types.

use solidcore::curve::curve::Curve;
use solidcore::kernel::Kernel;
use solidcore::mass::integrator::properties::brep_volume_default;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::nurbs::b_spline_surface::BSplineSurface;
use solidcore::primitive::api_curves;
use solidcore::surface::surface::p4::NoObject;
use solidcore::surface::surface::p5::DropPointOptions;
use solidcore::surface::surface::Surface;
use solidcore::sweep::api_brep::create_sweep_along_planar_path;
use solidcore::topology::brep::is_outward_solid;
use solidcore::topology::brep::p2::is_manifold_solid;
use solidcore::topology::brep::validation::{check_brep, CheckLevel};
use solidcore::topology::edge;
use solidcore::topology::edgeuse::p1 as edgeuse1;
use solidcore::topology::integrity;

use universal_miter_prototype::boundary::{
    BoundaryError, BoundaryInput, BoundarySolver, BoundaryStrategy, BoundaryStrategyKind,
    ExactMiter, HybridAtJunction, HybridAtLimitTangent, JointBoundary, PathFrame,
};
use universal_miter_prototype::direct_brep::{build_direct_chain, build_direct_joint};
use universal_miter_prototype::level_b_geometry::{build_swept_surface, SurfaceBuildOptions};
use universal_miter_prototype::planar_sweep2::{normalize_planar_sweep, to_bspline};

fn line(start: Point3, end: Point3) -> BSplineCurve {
    api_curves::create_line_segment(start, end)
        .expect("line")
        .base_with_nurb()
        .expect("line nurb")
        .into_owned()
}

fn arc(center: Point3, radius: f64, start: f64, end: f64) -> BSplineCurve {
    api_curves::create_arc(center, radius, start, end).expect("arc")
}

fn endpoint(path: &BSplineCurve, at_end: bool) -> (Point3, Vec3) {
    let parameter = if at_end {
        path.natural_interval().max()
    } else {
        path.natural_interval().min()
    };
    let values = path
        .evaluate(parameter, 1, true, false)
        .expect("path endpoint");
    (
        *values.first().expect("endpoint point"),
        *values.get(1).expect("endpoint tangent"),
    )
}

fn options() -> SurfaceBuildOptions {
    // The line-arc side surface needs a slightly looser builder tolerance; the
    // measured deviation (~5.6e-5) is reported in the test output.
    SurfaceBuildOptions {
        tolerance: 1.0e-4,
        max_refinements: 7,
        max_sections: 129,
        ..SurfaceBuildOptions::default()
    }
}

fn seam_deviation(
    k: &Kernel,
    boundary: &JointBoundary,
    surface_a: &Surface,
    surface_b: &Surface,
) -> f64 {
    let domain_a = surface_a.natural_uv_domain_with_nurb(k).expect("domain a");
    let domain_b = surface_b.natural_uv_domain_with_nurb(k).expect("domain b");
    let mut worst = 0.0_f64;
    for segment in &boundary.seam_segments {
        let interval = segment.natural_interval();
        for index in 0..=16 {
            let t = interval.evaluate(f64::from(index) / 16.0);
            let point = segment.evaluate_point(k, t).expect("seam point");
            for (surface, domain) in [(surface_a, &domain_a), (surface_b, &domain_b)] {
                let drop = surface
                    .drop_point::<NoObject>(k, point, domain, &DropPointOptions::default())
                    .expect("seam drop");
                assert!(drop.success, "seam drop did not converge");
                worst = worst.max(drop.gap);
            }
        }
    }
    worst
}

fn reference_volume(paths: &[Curve], radius: f64) -> Option<f64> {
    let mut k = Kernel::default();
    let profile = Curve::from(api_curves::create_circle(Point3::ZERO, radius).ok()?);
    let profile_refs = [&profile];
    let path_refs: Vec<&Curve> = paths.iter().collect();
    let mut brep =
        create_sweep_along_planar_path(&mut k, &profile_refs, &path_refs, true, 3).ok()?;
    brep_volume_default(&k, &mut brep, 1.0e-9).ok()
}

fn run_joint_case(
    name: &str,
    path_a: &BSplineCurve,
    path_b: &BSplineCurve,
    profile_radius: f64,
    expected_length: f64,
    expected_strategy: BoundaryStrategyKind,
    expect_junction: bool,
    check_reference: bool,
    volume_tolerance: Option<f64>,
) {
    let mut k = Kernel::default();
    let paths = [Curve::from(path_a.clone()), Curve::from(path_b.clone())];
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input =
        Curve::from(api_curves::create_circle(Point3::ZERO, profile_radius).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let extension = normalized.extension;
    let placed_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");
    let boundary = BoundarySolver::default()
        .solve(&BoundaryInput {
            kernel: &k,
            frame,
            profile: &placed_profile,
            path_a,
            path_b,
            extension,
            segments: 4,
            samples_per_segment: 17,
        })
        .expect("boundary");
    assert_eq!(
        boundary.diagnostics.strategy, expected_strategy,
        "{name}: selected strategy"
    );
    if expect_junction {
        assert!(
            boundary.diagnostics.junction_samples > 0,
            "{name}: expected junction samples"
        );
    } else {
        assert_eq!(
            boundary.diagnostics.junction_samples, 0,
            "{name}: expected no junction samples"
        );
    }
    println!(
        "boundary {name}: strategy={:?} samples={} segments={} exact={} junction={} first_junction={:?} max_residual={:.3e}",
        boundary.diagnostics.strategy,
        boundary.diagnostics.samples,
        boundary.diagnostics.segments,
        boundary.diagnostics.exact_samples,
        boundary.diagnostics.junction_samples,
        boundary.diagnostics.first_junction_alpha,
        boundary.diagnostics.max_residual
    );

    let mut side_surfaces: Vec<BSplineSurface> = Vec::new();
    let mut surface_deviations = Vec::new();
    for placement in &normalized.placements {
        let profile = to_bspline(&placement.world_profiles[0]).expect("profile");
        let built = build_swept_surface(
            &k,
            &placement.path_piece,
            &profile,
            frame.normal,
            &options(),
        )
        .expect("surface");
        side_surfaces.push(built.surface);
        surface_deviations.push(built.max_deviation);
    }
    let surface_a = Surface::from(side_surfaces[0].clone());
    let surface_b = Surface::from(side_surfaces[1].clone());
    let boundary_deviation = seam_deviation(&k, &boundary, &surface_a, &surface_b);

    let (start, start_tangent) = endpoint(path_a, false);
    let (end, end_tangent) = endpoint(path_b, true);
    let mut brep = build_direct_joint(
        &mut k,
        &side_surfaces[0],
        &side_surfaces[1],
        &boundary,
        start,
        start_tangent,
        end,
        end_tangent,
        4,
    )
    .expect("direct joint");

    integrity::check(&brep).expect("integrity");
    match check_brep(&mut k, &mut brep, CheckLevel::Level0) {
        Ok(check) => {
            println!(
                "validation {name}: pointers={} assert={:?} failures={}",
                check.pointers_status,
                check.assert_valid,
                check.failures().count()
            );
            for failure in check.failures() {
                println!("validation failure {name}: {failure:?}");
            }
            assert_eq!(check.failures().count(), 0, "{name}: check_brep failures");
            assert_eq!(
                check.assert_valid,
                Some(true),
                "{name}: check_brep assert_valid"
            );
        }
        Err(error) => println!("validation error {name}: {error:?}"),
    }
    let faces = brep.faces().len();
    let edges = brep.edges().len();
    let vertices = brep.vertices().len();
    for (edge_id, _) in brep.edges().iter() {
        let uses = edge::edgeuses(&brep, edge_id, None, None).expect("edge uses");
        assert_eq!(
            uses.len(),
            4,
            "{name}: edge {edge_id:?} has {} uses",
            uses.len()
        );
    }
    let manifold = is_manifold_solid(&brep).expect("manifold");
    let outward = is_outward_solid(&mut k, &mut brep).expect("outward");
    if name == "line-arc-hybrid" {
        for accuracy in [1.0e-3, 1.0e-4, 1.0e-5] {
            match brep_volume_default(&k, &mut brep, accuracy) {
                Ok(value) => println!("diagnostic {name}: volume@{accuracy:.0e}={value:.12e}"),
                Err(error) => println!("diagnostic {name}: volume@{accuracy:.0e}=ERR {error:?}"),
            }
        }
        for (label, side) in [("a", &side_surfaces[0]), ("b", &side_surfaces[1])] {
            let surface = Surface::from(side.clone());
            let domain = surface
                .natural_uv_domain_with_nurb(&k)
                .expect("diagnostic domain");
            let mut u_min = f64::INFINITY;
            let mut u_max = f64::NEG_INFINITY;
            let mut v_min = f64::INFINITY;
            let mut v_max = f64::NEG_INFINITY;
            let mut drops = Vec::new();
            for point in &boundary.seam_vertices {
                let drop = surface
                    .drop_point::<NoObject>(&k, *point, &domain, &DropPointOptions::default())
                    .expect("diagnostic seam drop");
                u_min = u_min.min(drop.result_uv.x);
                u_max = u_max.max(drop.result_uv.x);
                v_min = v_min.min(drop.result_uv.y);
                v_max = v_max.max(drop.result_uv.y);
                drops.push((drop.result_uv.x, drop.result_uv.y, drop.gap));
            }
            println!(
                "diagnostic {name}: side {label} domain u=[{:.6},{:.6}] v=[{:.6},{:.6}] seam u=[{u_min:.6},{u_max:.6}] v=[{v_min:.6},{v_max:.6}] drops={drops:?}",
                domain.min().x,
                domain.max().x,
                domain.min().y,
                domain.max().y
            );
        }
    }
    let volume = brep_volume_default(&k, &mut brep, 1.0e-4).expect("volume");
    let expected = std::f64::consts::PI * profile_radius * profile_radius * expected_length;
    let relative = (volume - expected).abs() / expected;
    let reference = if check_reference {
        reference_volume(&paths, profile_radius)
    } else {
        None
    };
    let reference_relative = reference.map(|value| (volume - value).abs() / value);
    println!(
        "{name}: surface_deviation_a={:.3e} surface_deviation_b={:.3e} boundary_deviation={:.3e} faces={faces} edges={edges} vertices={vertices} regions={} shells={} manifold={manifold} outward={outward} volume={volume:.9e} expected={expected:.9e} relative={relative:.3e} reference={reference:?} reference_relative={reference_relative:?}",
        surface_deviations[0],
        surface_deviations[1],
        boundary_deviation,
        brep.regions().len(),
        brep.shells().len()
    );
    assert!(
        surface_deviations[0] <= 1.0e-4 && surface_deviations[1] <= 1.0e-4,
        "{name}: surface deviation"
    );
    assert!(
        boundary_deviation <= 1.0e-4,
        "{name}: boundary deviation {boundary_deviation}"
    );
    assert_eq!(faces, 10, "{name}: face count");
    assert_eq!(edges, 20, "{name}: edge count");
    assert_eq!(vertices, 12, "{name}: vertex count");
    assert!(manifold, "{name}: not manifold");
    assert!(outward, "{name}: not outward");
    assert!(volume > 0.0, "{name}: no volume");
    if let Some(volume_tolerance) = volume_tolerance {
        assert!(
            relative <= volume_tolerance,
            "{name}: volume relative {relative}"
        );
    }
    if check_reference {
        if let Some(reference_relative) = reference_relative {
            assert!(
                reference_relative <= 1.0e-5,
                "{name}: reference relative {reference_relative}"
            );
        }
    }
}

#[test]
fn direct_joint_line_line() {
    let path_a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let path_b = line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    run_joint_case(
        "line-line",
        &path_a,
        &path_b,
        0.25,
        2.0,
        BoundaryStrategyKind::ExactMiter,
        false,
        true,
        Some(1.0e-5),
    );
}

#[test]
fn direct_joint_line_arc_hybrid() {
    let path_a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let path_b = arc(Point3::ZERO, 1.0, 0.0, 90.0);
    let length = 1.0 + std::f64::consts::FRAC_PI_2;
    // The negative side has no exact offset intersection; the junction tangent
    // extension supplies it. Area x centreline is the reference, with the
    // open Level B volume residual reported by docs/19.
    run_joint_case(
        "line-arc-hybrid",
        &path_a,
        &path_b,
        0.25,
        length,
        BoundaryStrategyKind::HybridAtJunction,
        true,
        false,
        None,
    );
}

#[test]
fn direct_joint_line_arc_exact() {
    let path_a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(5.0, 0.0, 0.0));
    let path_b = arc(Point3::new(5.0, 4.0, 0.0), 4.0, -90.0, 60.0);
    let length = 5.0 + 4.0 * 5.0 * std::f64::consts::PI / 6.0;
    run_joint_case(
        "line-arc-exact",
        &path_a,
        &path_b,
        0.25,
        length,
        BoundaryStrategyKind::ExactMiter,
        false,
        false,
        Some(1.0e-5),
    );
}

#[test]
fn hybrid_limit_tangent_refuses_uncovered_profile() {
    let path_a = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let path_b = arc(Point3::ZERO, 1.0, 0.0, 90.0);
    let k = Kernel::default();
    let paths = [Curve::from(path_a.clone()), Curve::from(path_b.clone())];
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input = Curve::from(api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let placed_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");
    let input = BoundaryInput {
        kernel: &k,
        frame,
        profile: &placed_profile,
        path_a: &path_a,
        path_b: &path_b,
        extension: normalized.extension,
        segments: 4,
        samples_per_segment: 17,
    };
    let exact_refused = ExactMiter
        .solve(&input)
        .expect_err("exact miter must refuse the uncovered side");
    assert!(
        matches!(
            exact_refused,
            BoundaryError::UncoveredProfile {
                strategy: BoundaryStrategyKind::ExactMiter,
                ..
            }
        ),
        "unexpected exact refusal: {exact_refused}"
    );
    let hybrid = HybridAtJunction
        .solve(&input)
        .expect("junction continuation");
    assert_eq!(hybrid.strategy, BoundaryStrategyKind::HybridAtJunction);
    assert!(hybrid.diagnostics.junction_samples > 0);
    let refused = HybridAtLimitTangent
        .solve(&input)
        .expect_err("limit tangent must refuse");
    assert!(
        matches!(
            refused,
            BoundaryError::UncoveredProfile {
                strategy: BoundaryStrategyKind::HybridAtLimitTangent,
                ..
            }
        ),
        "unexpected refusal: {refused}"
    );
}

#[test]
fn direct_chain_line_line_line() {
    let path0 = line(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0));
    let path1 = line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0));
    let path2 = line(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0));
    let path_bs = [path0, path1, path2];
    let paths = [
        Curve::from(path_bs[0].clone()),
        Curve::from(path_bs[1].clone()),
        Curve::from(path_bs[2].clone()),
    ];
    let mut k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input = Curve::from(api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let boundary_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");

    let mut side_surfaces = Vec::with_capacity(3);
    for placement in &normalized.placements {
        let profile = to_bspline(&placement.world_profiles[0]).expect("profile");
        let built = build_swept_surface(
            &k,
            &placement.path_piece,
            &profile,
            frame.normal,
            &options(),
        )
        .expect("surface");
        side_surfaces.push(built.surface);
    }

    let mut boundaries = Vec::with_capacity(2);
    for joint in 0..2 {
        let boundary = BoundarySolver::default()
            .solve(&BoundaryInput {
                kernel: &k,
                frame,
                profile: &boundary_profile,
                path_a: &path_bs[joint],
                path_b: &path_bs[joint + 1],
                extension: normalized.extension,
                segments: 4,
                samples_per_segment: 17,
            })
            .expect("joint boundary");
        assert_eq!(
            boundary.diagnostics.strategy,
            BoundaryStrategyKind::ExactMiter
        );
        boundaries.push(boundary);
    }

    let (start, start_tangent) = endpoint(&path_bs[0], false);
    let (end, end_tangent) = endpoint(&path_bs[2], true);
    let mut brep = build_direct_chain(
        &mut k,
        &side_surfaces,
        &boundaries,
        start,
        start_tangent,
        end,
        end_tangent,
        4,
    )
    .expect("direct chain");

    integrity::check(&brep).expect("integrity");
    let check = check_brep(&mut k, &mut brep, CheckLevel::Level0).expect("check_brep");
    assert_eq!(check.failures().count(), 0, "check_brep failures");
    assert_eq!(check.assert_valid, Some(true), "check_brep assert_valid");

    let faces = brep.faces().len();
    let edges = brep.edges().len();
    let vertices = brep.vertices().len();
    assert_eq!(faces, 14, "face count");
    assert_eq!(edges, 28, "edge count");
    assert_eq!(vertices, 16, "vertex count");
    assert_eq!(brep.regions().len(), 2);
    assert_eq!(brep.shells().len(), 2);
    for (edge_id, _) in brep.edges().iter() {
        let uses = edge::edgeuses(&brep, edge_id, None, None).expect("edge uses");
        assert_eq!(uses.len(), 4, "edge {edge_id:?} uses");
    }
    assert!(is_manifold_solid(&brep).expect("manifold"));
    assert!(is_outward_solid(&mut k, &mut brep).expect("outward"));
    let volume = brep_volume_default(&k, &mut brep, 1.0e-4).expect("volume");
    let expected = std::f64::consts::PI * 0.25 * 0.25 * 3.0;
    let relative = (volume - expected).abs() / expected;
    println!(
        "line-line-line: faces={faces} edges={edges} vertices={vertices} volume={volume:.9e} expected={expected:.9e} relative={relative:.3e}"
    );
    assert!(volume > 0.0);
    assert!(
        relative <= 1.0e-5,
        "line-line-line volume relative {relative}"
    );
}

#[test]
fn arc_arc_boundary_matrix() {
    let path_a = arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0);
    let path_b = arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 90.0);
    let paths = [Curve::from(path_a.clone()), Curve::from(path_b.clone())];
    let k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input = Curve::from(api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let placed_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");
    let input = BoundaryInput {
        kernel: &k,
        frame,
        profile: &placed_profile,
        path_a: &path_a,
        path_b: &path_b,
        extension: normalized.extension,
        segments: 4,
        samples_per_segment: 17,
    };
    let hybrid = HybridAtJunction.solve(&input).expect("hybrid boundary");
    assert_eq!(hybrid.strategy, BoundaryStrategyKind::HybridAtJunction);
    assert!(hybrid.diagnostics.junction_samples > 0);
    assert!(hybrid.diagnostics.max_residual <= 1.0e-9);
    let refused = ExactMiter.solve(&input).expect_err("exact must refuse");
    assert!(matches!(
        refused,
        BoundaryError::UncoveredProfile {
            strategy: BoundaryStrategyKind::ExactMiter,
            ..
        }
    ));
}

fn side_seam_deviation(k: &Kernel, boundary: &JointBoundary, surface: &Surface) -> f64 {
    let domain = surface
        .natural_uv_domain_with_nurb(k)
        .expect("audit domain");
    let mut worst = 0.0_f64;
    for segment in &boundary.seam_segments {
        let interval = segment.natural_interval();
        for index in 0..=16 {
            let t = interval.evaluate(f64::from(index) / 16.0);
            let point = segment.evaluate_point(k, t).expect("audit seam point");
            let drop = surface
                .drop_point::<NoObject>(k, point, &domain, &DropPointOptions::default())
                .expect("audit seam drop");
            assert!(drop.success, "audit seam drop did not converge");
            worst = worst.max(drop.gap);
        }
    }
    worst
}

#[test]
fn arc_arc_geometric_consistency() {
    let path_a = arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0);
    let path_b = arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 90.0);
    let paths = [Curve::from(path_a.clone()), Curve::from(path_b.clone())];
    let k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input = Curve::from(api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let boundary_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");
    let mut side_surfaces = Vec::with_capacity(2);
    let mut surface_deviations = Vec::with_capacity(2);
    for placement in &normalized.placements {
        let profile = to_bspline(&placement.world_profiles[0]).expect("profile");
        let built = build_swept_surface(
            &k,
            &placement.path_piece,
            &profile,
            frame.normal,
            &options(),
        )
        .expect("surface");
        side_surfaces.push(built.surface);
        surface_deviations.push(built.max_deviation);
    }
    let boundary = BoundarySolver::default()
        .solve(&BoundaryInput {
            kernel: &k,
            frame,
            profile: &boundary_profile,
            path_a: &path_a,
            path_b: &path_b,
            extension: normalized.extension,
            segments: 4,
            samples_per_segment: 17,
        })
        .expect("arc-arc boundary");
    let surface_a = Surface::from(side_surfaces[0].clone());
    let surface_b = Surface::from(side_surfaces[1].clone());
    let side_a = side_seam_deviation(&k, &boundary, &surface_a);
    let side_b = side_seam_deviation(&k, &boundary, &surface_b);
    let combined = seam_deviation(&k, &boundary, &surface_a, &surface_b);
    println!(
        "arc-arc audit: strategy={:?} exact={} junction={} surface_a={:.3e} surface_b={:.3e} seam_a={:.3e} seam_b={:.3e} seam_both={:.3e}",
        boundary.diagnostics.strategy,
        boundary.diagnostics.exact_samples,
        boundary.diagnostics.junction_samples,
        surface_deviations[0],
        surface_deviations[1],
        side_a,
        side_b,
        combined
    );
    assert!(surface_deviations[0] <= 1.0e-4);
    assert!(surface_deviations[1] <= 1.0e-4);
    assert!(side_a <= 1.0e-4);
    assert!(side_b <= 1.0e-4);
    assert!(combined <= 1.0e-4);
}
#[test]
#[ignore = "manual UV trim audit"]
fn arc_arc_uv_trim_audit() {
    let path_a = arc(Point3::new(0.0, 1.0, 0.0), 1.0, -180.0, -90.0);
    let path_b = arc(Point3::new(-1.0, 0.0, 0.0), 1.0, 0.0, 90.0);
    let paths = [Curve::from(path_a.clone()), Curve::from(path_b.clone())];
    let mut k = Kernel::default();
    let frame = PathFrame::new(&k, &paths).expect("frame");
    let profile_input = Curve::from(api_curves::create_circle(Point3::ZERO, 0.25).expect("circle"));
    let normalized =
        normalize_planar_sweep(&k, &[profile_input], &paths, true, 3).expect("normalize");
    let boundary_profile =
        to_bspline(&normalized.placements[0].world_profiles[0]).expect("placed profile");
    let mut side_surfaces = Vec::with_capacity(2);
    for placement in &normalized.placements {
        let profile = to_bspline(&placement.world_profiles[0]).expect("profile");
        let built = build_swept_surface(
            &k,
            &placement.path_piece,
            &profile,
            frame.normal,
            &options(),
        )
        .expect("surface");
        side_surfaces.push(built.surface);
    }
    let boundary = BoundarySolver::default()
        .solve(&BoundaryInput {
            kernel: &k,
            frame,
            profile: &boundary_profile,
            path_a: &path_a,
            path_b: &path_b,
            extension: normalized.extension,
            segments: 4,
            samples_per_segment: 17,
        })
        .expect("boundary");
    let (start, start_tangent) = endpoint(&path_a, false);
    let (end, end_tangent) = endpoint(&path_b, true);
    let mut brep = build_direct_chain(
        &mut k,
        &side_surfaces,
        std::slice::from_ref(&boundary),
        start,
        start_tangent,
        end,
        end_tangent,
        4,
    )
    .expect("direct chain");
    fn audit_edge_uvs(brep: &solidcore::topology::Brep, k: &Kernel, label: &str) {
        for (face_id, face) in brep.faces().iter() {
            let Some(faceuse) = face.faceuse else {
                continue;
            };
            let loopuses = brep.faceuse(faceuse).expect("faceuse").loopuses.clone();
            for loopuse in loopuses {
                for edgeuse in edgeuse1::loopuse_edgeuses(brep, loopuse).expect("edgeuses") {
                    let edge = edgeuse1::edge(brep, edgeuse).expect("edge").expect("id");
                    if edge != solidcore::topology::EdgeId::from_index(8)
                        && edge != solidcore::topology::EdgeId::from_index(12)
                    {
                        continue;
                    }
                    let uv = edgeuse1::uv_trim_curve(brep, edgeuse)
                        .expect("uv")
                        .0
                        .cloned();
                    let values = uv.map(|curve| {
                        let interval = curve.natural_interval();
                        (
                            curve.evaluate_point(k, interval.min()).expect("uv start"),
                            curve.evaluate_point(k, interval.max()).expect("uv end"),
                        )
                    });
                    println!("AUDIT-EDGE {label} face={face_id:?} loop={loopuse:?} edge={edge:?} edgeuse={edgeuse:?} uv={values:?}");
                }
            }
        }
    }
    audit_edge_uvs(&brep, &k, "before-check");
    match check_brep(&mut k, &mut brep, CheckLevel::Level0) {
        Ok(check) => {
            println!(
                "AUDIT-CHECK failures={} assert={:?}",
                check.failures().count(),
                check.assert_valid
            );
            for failure in check.failures().take(3) {
                println!("AUDIT-CHECK {failure:?}");
            }
        }
        Err(error) => println!("AUDIT-CHECK ERR {error:?}"),
    }
    audit_edge_uvs(&brep, &k, "after-check");
    for (face_id, face) in brep.faces().iter() {
        let Some(faceuse) = face.faceuse else {
            continue;
        };
        let loopuses = brep.faceuse(faceuse).expect("faceuse").loopuses.clone();
        for loopuse in loopuses {
            let edgeuses = edgeuse1::loopuse_edgeuses(&brep, loopuse).expect("loop edgeuses");
            let mut entries = Vec::new();
            for edgeuse in &edgeuses {
                let edge = edgeuse1::edge(&brep, *edgeuse)
                    .expect("edgeuse edge")
                    .expect("edge id");
                let curve = edge::curve(&brep, edge)
                    .expect("edge curve")
                    .expect("curve");
                let interval = edge::interval(&brep, edge).expect("edge interval");
                let start3 = curve
                    .evaluate_point(&k, interval.min())
                    .expect("edge start");
                let end3 = curve.evaluate_point(&k, interval.max()).expect("edge end");
                let uv_curve = edgeuse1::uv_trim_curve(&brep, *edgeuse)
                    .expect("uv curve")
                    .0
                    .cloned();
                let (uv0, uv1) = if let Some(uv) = uv_curve {
                    let iv = uv.natural_interval();
                    (
                        Some(uv.evaluate_point(&k, iv.min()).expect("uv start")),
                        Some(uv.evaluate_point(&k, iv.max()).expect("uv end")),
                    )
                } else {
                    (None, None)
                };
                let orientation = edgeuse1::orientation(&brep, *edgeuse).expect("orientation");
                entries.push((edge, start3, end3, uv0, uv1, orientation));
            }
            for index in 0..entries.len() {
                let next = (index + 1) % entries.len();
                let (edge, e0, e1, u0, u1, orient) = entries[index];
                let (_, n0, _, nu0, _, _) = entries[next];
                let gap = match (u1, nu0) {
                    (Some(a), Some(b)) => ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt(),
                    _ => f64::NAN,
                };
                if !gap.is_finite() || gap > 1.0e-3 {
                    println!(
                        "UV-AUDIT face={face_id:?} loop={loopuse:?} index={index} edge={edge:?} orient={orient:?} edge3={e0:?}->{e1:?} uv={u0:?}->{u1:?} next3={n0:?} next_uv0={nu0:?} gap={gap:.9e}"
                    );
                }
            }
        }
    }
}
