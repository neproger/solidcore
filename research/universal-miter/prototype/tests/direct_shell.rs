#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! Milestone 1: direct shared-topology B-rep for one path piece.

use solidcore::curve::curve::Curve;
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::kernel::Kernel;
use solidcore::mass::integrator::properties::brep_volume_default;
use solidcore::math::{Point3, Vec3};
use solidcore::nurbs::b_spline_curve::p1::BSplineCurve;
use solidcore::primitive::api_curves;
use solidcore::sweep::api_brep::create_sweep_along_planar_path;
use solidcore::topology::brep::is_outward_solid;
use solidcore::topology::brep::p2::is_manifold_solid;
use solidcore::topology::brep::validation::{check_brep, CheckLevel};
use solidcore::topology::edge;
use solidcore::topology::integrity;

use universal_miter_prototype::direct_brep::{build_direct_single, DirectShell};
use universal_miter_prototype::level_b_geometry::{build_swept_surface, SurfaceBuildOptions};

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

fn closed_profile(path: &BSplineCurve, radius: f64) -> BSplineCurve {
    let values = path
        .evaluate(path.natural_interval().min(), 1, true, false)
        .expect("profile path");
    let start = *values.first().expect("profile start");
    let tangent = values
        .get(1)
        .copied()
        .expect("profile tangent")
        .unitize()
        .expect("unit profile tangent");
    let normal = Vec3::Z.cross(tangent).unitize().expect("profile normal");
    let mut profile = api_curves::create_circle(Point3::ZERO, radius).expect("circle");
    let placement = Axis2Placement::new(start, normal, Vec3::Z);
    profile
        .transform(&placement, None)
        .expect("profile placement");
    profile
}

fn endpoints(path: &BSplineCurve) -> (Point3, Vec3, Point3, Vec3) {
    let start_values = path
        .evaluate(path.natural_interval().min(), 1, true, false)
        .expect("path start");
    let end_values = path
        .evaluate(path.natural_interval().max(), 1, true, false)
        .expect("path end");
    (
        *start_values.first().expect("start point"),
        *start_values.get(1).expect("start tangent"),
        *end_values.first().expect("end point"),
        *end_values.get(1).expect("end tangent"),
    )
}

fn options() -> SurfaceBuildOptions {
    SurfaceBuildOptions {
        tolerance: 1.0e-5,
        max_refinements: 6,
        max_sections: 64,
        ..SurfaceBuildOptions::default()
    }
}

fn reference_volume(path: &BSplineCurve, radius: f64) -> f64 {
    let mut k = Kernel::default();
    let profile = Curve::from(api_curves::create_circle(Point3::ZERO, radius).expect("circle"));
    let path_curve = Curve::from(path.clone());
    let mut brep = create_sweep_along_planar_path(&mut k, &[&profile], &[&path_curve], true, 3)
        .expect("production reference");
    brep_volume_default(&k, &mut brep, 1.0e-9).expect("reference volume")
}

struct ShellChecks {
    faces: usize,
    edges: usize,
    vertices: usize,
    manifold: bool,
    outward: bool,
    volume: f64,
}

fn check_shell(k: &mut Kernel, shell: &mut DirectShell) -> ShellChecks {
    integrity::check(&shell.brep).expect("integrity");
    let faces = shell.brep.faces().len();
    let edges = shell.brep.edges().len();
    let vertices = shell.brep.vertices().len();
    for (edge_id, _) in shell.brep.edges().iter() {
        let uses = edge::edgeuses(&shell.brep, edge_id, None, None).expect("edge uses");
        assert_eq!(uses.len(), 4, "edge {edge_id:?} has {} uses", uses.len());
    }
    match check_brep(k, &mut shell.brep, CheckLevel::Level0) {
        Ok(check) => {
            println!(
                "validation: pointers={} assert={:?} failures={}",
                check.pointers_status,
                check.assert_valid,
                check.failures().count()
            );
            for failure in check.failures() {
                println!("validation failure: {failure:?}");
            }
        }
        Err(error) => println!("validation error: {error:?}"),
    }
    for (shell_id, shell_record) in shell.brep.shells().iter() {
        let faceuses = shell_record.get_faceuses().unwrap_or_default();
        println!(
            "shell {:?} faces_cache={} faceuses={}",
            shell_id,
            shell_record.faces.len(),
            faceuses.len()
        );
    }
    let manifold = is_manifold_solid(&shell.brep).expect("manifold");
    let outward_result = is_outward_solid(k, &mut shell.brep);
    let volume_result = brep_volume_default(k, &mut shell.brep, 1.0e-3);
    println!(
        "topology: regions={} shells={} faces={faces} edges={edges} vertices={vertices} manifold={manifold} outward={outward_result:?} volume={volume_result:?}",
        shell.brep.regions().len(),
        shell.brep.shells().len()
    );
    for (id, region) in shell.brep.regions().iter() {
        println!(
            "region {:?} void={} shells={}",
            id,
            region.is_void(),
            region.shells.len()
        );
    }
    let outward = outward_result.unwrap_or(false);
    let volume = volume_result.unwrap_or(f64::NAN);
    ShellChecks {
        faces,
        edges,
        vertices,
        manifold,
        outward,
        volume,
    }
}

fn run_case(name: &str, path: &BSplineCurve, radius: f64, segments: usize, expected: f64) {
    let mut k = Kernel::default();
    let profile = closed_profile(path, radius);
    let built = build_swept_surface(&k, path, &profile, Vec3::Z, &options()).expect("surface");
    let (start, start_tangent, end, end_tangent) = endpoints(path);
    let mut shell = build_direct_single(
        &mut k,
        &built.surface,
        start,
        start_tangent,
        end,
        end_tangent,
        segments,
    )
    .expect("direct shell");
    let checks = check_shell(&mut k, &mut shell);
    let reference = reference_volume(path, radius);
    let relative = (checks.volume - expected).abs() / expected.max(1.0e-30);
    println!(
        "{name}: surface_deviation={:.3e} faces={} edges={} vertices={} manifold={} outward={} volume={:.9e} expected={:.9e} reference={:.9e} relative={:.3e}",
        built.max_deviation,
        checks.faces,
        checks.edges,
        checks.vertices,
        checks.manifold,
        checks.outward,
        checks.volume,
        expected,
        reference,
        relative
    );
    assert!(
        built.max_deviation <= 1.0e-5,
        "{name}: surface deviation {}",
        built.max_deviation
    );
    assert_eq!(checks.faces, segments + 2, "{name}: face count");
    assert_eq!(checks.edges, 3 * segments, "{name}: edge count");
    assert_eq!(checks.vertices, 2 * segments, "{name}: vertex count");
    assert!(checks.manifold, "{name}: not manifold");
    assert!(checks.outward, "{name}: not outward");
    assert!(checks.volume > 0.0, "{name}: no volume");
    assert!(
        relative <= 1.0e-5,
        "{name}: volume differs from the analytic sweep by {relative}"
    );
    let reference_relative = (checks.volume - reference).abs() / reference.max(1.0e-30);
    assert!(
        reference_relative <= 1.0e-5,
        "{name}: volume differs from the production reference by {reference_relative}"
    );
}

#[test]
fn direct_shell_single_straight() {
    let path = line(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0));
    let expected = std::f64::consts::PI * 0.25 * 0.25 * 2.0;
    run_case("single straight", &path, 0.25, 4, expected);
}

#[test]
fn direct_shell_single_arc() {
    let path = arc(Point3::ZERO, 2.0, 0.0, 90.0);
    let expected = std::f64::consts::PI * 0.25 * 0.25 * (std::f64::consts::PI / 2.0 * 2.0);
    run_case("single arc", &path, 0.25, 4, expected);
}
