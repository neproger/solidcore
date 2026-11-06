//! Surface capability audit: analytic creation, exact NURBS conversion, split, face.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    clippy::needless_pass_by_value,
    clippy::cast_lossless
)]

use solidcore::curve::curve::Curve;
use solidcore::curve::line::Line;
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::object::ObjectRef;
use solidcore::math::{Point2, Point3, Vec2, Vec3};
use solidcore::nurbs::b_spline_surface::p1::SurfParamType;
use solidcore::nurbs::b_spline_surface_n_lib::p2::{EvaluateFlags, OrientType};
use solidcore::surface::cone::Cone;
use solidcore::surface::plane::Plane;
use solidcore::surface::surf_of_extrusion::SurfOfExtrusion;
use solidcore::surface::surf_of_revolution::SurfOfRevolution;
use solidcore::surface::surface::{Surface, SurfaceKind};
use solidcore::surface::torus::{Torus, TorusOptions};
use solidcore::topology::brep::p7::{FaceCurvesInput, make_face_with_curves};
use solidcore::topology::brep::validation::{CheckLevel, check_brep, check_brep_with_keys};
use solidcore::topology::edge::NotifyEnv;
use solidcore::topology::topology::NotifyRecords;
use solidcore::{Brep, EntityRef, Kernel};

fn point(surface: &Surface, k: &Kernel, uv: Point2) -> Point3 {
    let eval = surface
        .evaluate(k, uv, 0, 0, EvaluateFlags::default())
        .expect("surface evaluation");
    eval[0][0]
}

fn close(a: Point3, b: Point3, scale: f64) -> bool {
    (a - b).length() <= 1e-9 * scale.max(1.0)
}

fn audit_surface(label: &str, surface: Surface, k: &Kernel) {
    let before = surface.kind();
    assert!(
        !matches!(before, SurfaceKind::BSpline(_)),
        "{label}: audit input is already a plain B-spline"
    );

    let domain = surface
        .natural_uv_domain_with_nurb(k)
        .unwrap_or_else(|e| panic!("{label}: no NURB domain: {e}"));

    for dir in [SurfParamType::U, SurfParamType::V] {
        let (min, max) = match dir {
            SurfParamType::U => (domain.min().x, domain.max().x),
            SurfParamType::V => (domain.min().y, domain.max().y),
            _ => unreachable!(),
        };
        assert!(max > min, "{label} {dir:?}: empty NURB domain");

        let split = 0.5 * (min + max);
        let (left, right) = surface
            .split_at(k, split, dir)
            .unwrap_or_else(|e| panic!("{label} {dir:?}: split failed: {e}"));

        assert!(
            matches!(left.kind(), SurfaceKind::BSpline(_))
                && matches!(right.kind(), SurfaceKind::BSpline(_)),
            "{label} {dir:?}: split pieces are not B-spline/rational NURBS"
        );

        for (side, piece) in [("left", &left), ("right", &right)] {
            let pd = piece
                .natural_uv_domain_with_nurb(k)
                .unwrap_or_else(|e| panic!("{label} {dir:?} {side}: no domain: {e}"));
            let n = 17;
            let mut max = 0.0_f64;
            for i in 0..=n {
                for j in 0..=n {
                    let u = pd.min().x + (pd.max().x - pd.min().x) * (i as f64 / n as f64);
                    let v = pd.min().y + (pd.max().y - pd.min().y) * (j as f64 / n as f64);
                    let uv = Point2::new(u, v);
                    let got = point(piece, k, uv);
                    let want = point(&surface, k, uv);
                    let d = (got - want).length();
                    max = max.max(d);
                    assert!(
                        close(got, want, want.max_dimension()),
                        "{label} {dir:?} {side}: at {uv:?}: {got:?} != {want:?} (d={d})"
                    );
                }
            }
            println!("{label} {dir:?} {side}: max deviation {max:e}");
        }
    }
}

fn plane() -> Surface {
    let mut plane = Plane::new(
        Point3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec2::new(1.0, 1.0),
        &Extent2d::from_bounds(0.0, 0.0, 10.0, 10.0),
    );
    plane.make_nurb().unwrap();
    plane.into()
}

fn cylinder(k: &Kernel) -> Surface {
    let mut c = Cone::new_cylinder_bounded(
        Point3::ZERO,
        Vec3::X,
        Vec3::Y,
        2.0,
        0.0,
        180.0,
        10.0,
        false,
        false,
        false,
    );
    c.make_nurb(k).unwrap();
    c.into()
}

fn torus(k: &Kernel) -> Surface {
    Torus::create_canonical(
        k,
        &Axis2Placement::IDENTITY,
        5.0,
        1.0,
        TorusOptions::default(),
        None,
    )
    .unwrap()
    .into()
}

fn extrusion(k: &Kernel) -> Surface {
    let line = Line::from_points(Point3::new(2.0, 0.0, -2.0), Point3::new(2.0, 2.0, -2.0), 3);
    let mut s = SurfOfExtrusion::new(
        k,
        Curve::from(line),
        Point3::new(2.0, 0.0, -2.0),
        Vec3::Z,
        Extent2d::from_bounds(0.0, 0.0, 1.0, 10.0),
        false,
    )
    .unwrap();
    s.make_nurb(k).unwrap();
    s.into()
}

fn revolution(k: &Kernel) -> Surface {
    let mut line = Line::from_points(Point3::new(2.0, 0.0, -2.0), Point3::new(2.0, 0.0, 2.0), 3);
    line.make_nurb().unwrap();
    SurfOfRevolution::create_canonical(k, Curve::from(line), Point3::ZERO, Vec3::Z, false)
        .unwrap()
        .into()
}

#[test]
fn analytic_primitive_body_is_a_valid_brep() {
    let mut k = Kernel::default();
    let mut b =
        solidcore::primitive::api_primitives::create_cylinder(&mut k, Point3::ZERO, 2.0, 10.0)
            .unwrap();
    let kinds: Vec<String> = b
        .face_list()
        .iter()
        .map(|&f| {
            let kind = b.face(f).unwrap().surface.as_ref().unwrap().kind();
            match kind {
                SurfaceKind::Cone(c) if c.is_cylinder() => "Cylinder".to_string(),
                SurfaceKind::Plane(_) => "Plane".to_string(),
                other => format!("{other:?}"),
            }
        })
        .collect();
    let check = check_brep(&mut k, &mut b, CheckLevel::Level2).unwrap();
    println!(
        "cylinder body: faces={} kinds={kinds:?} valid={} failures=[{}]",
        b.face_list().len(),
        check.is_valid(),
        check.lines().join(" | ")
    );
    assert!(check.is_valid(), "analytic cylinder body is invalid");
}

fn make_one_face(label: &str, surface: Surface, k: &mut Kernel, analytic: bool) {
    let full = surface
        .natural_uv_domain_with_nurb(k)
        .unwrap_or_else(|e| panic!("{label}: no NURB domain: {e}"));
    let span = full.max() - full.min();
    let domain = Extent2d::from_bounds(
        full.min().x,
        full.min().y,
        full.min().x + 0.5 * span.x,
        full.min().y + 0.5 * span.y,
    );

    let iso = |param, t| {
        surface
            .create_iso_parametric_curve(k, param, t, 0.0, Some(&domain), false)
            .unwrap_or_else(|e| panic!("{label}: iso curve failed: {e}"))
            .curve
    };
    let curves = vec![
        Some(iso(SurfParamType::U, domain.min().x)),
        Some(iso(SurfParamType::V, domain.max().y)),
        Some(iso(SurfParamType::U, domain.max().x)),
        Some(iso(SurfParamType::V, domain.min().y)),
    ];

    let mut b = Brep::new_with_infinite_region().unwrap();
    let mut records = NotifyRecords::new();
    let mut attributes = AttributeStore::default();
    let key_of = |entity: EntityRef| ObjectRef {
        type_code: 0,
        key: ObjectKey(match entity {
            EntityRef::Face(f) => 1_000_000 + u64::from(f.index()),
            EntityRef::Edge(e) => 2_000_000 + u64::from(e.index()),
            EntityRef::Edgeuse(e) => 3_000_000 + u64::from(e.index()),
            EntityRef::Vertex(v) => 4_000_000 + u64::from(v.index()),
            EntityRef::Vertexuse(v) => 5_000_000 + u64::from(v.index()),
            EntityRef::Loop(l) => 6_000_000 + u64::from(l.index()),
            EntityRef::Loopuse(l) => 7_000_000 + u64::from(l.index()),
            EntityRef::Faceuse(f) => 8_000_000 + u64::from(f.index()),
            EntityRef::Shell(s) => 9_000_000 + u64::from(s.index()),
            EntityRef::Region(r) => 10_000_000 + u64::from(r.index()),
            EntityRef::Brep(_) => 11_000_000,
        }),
    };
    let face_ref = |face: solidcore::FaceId| ObjectRef {
        type_code: 0,
        key: ObjectKey(1_000_000 + u64::from(face.index())),
    };
    let mut env = NotifyEnv {
        records: &mut records,
        attributes: &mut attributes,
        context: None,
        key_of: &key_of,
    };
    let made = make_face_with_curves(
        k,
        &mut b,
        None,
        FaceCurvesInput {
            loops: vec![4],
            curves_3d: Some(curves),
            orientations: vec![
                OrientType::Same,
                OrientType::Same,
                OrientType::Opposite,
                OrientType::Opposite,
            ],
            ..Default::default()
        },
        surface,
        &domain,
        OrientType::Same,
        None,
        &face_ref,
        &mut env,
    )
    .unwrap_or_else(|e| panic!("{label}: face creation failed: {e}"));

    let face = made.face.unwrap_or_else(|| panic!("{label}: no face"));
    let record = b.face(face).unwrap();
    assert!(record.surface.is_some(), "{label}: face has no surface");
    assert_eq!(record.loops.len(), 1, "{label}: face loop count");
    assert_eq!(
        !matches!(
            record.surface.as_ref().unwrap().kind(),
            SurfaceKind::BSpline(_)
        ),
        analytic,
        "{label}: unexpected face surface kind"
    );
    match check_brep_with_keys(k, &mut b, CheckLevel::Level0, &key_of) {
        Ok(check) => println!(
            "{label}: face={face:?} check_brep valid={} failures=[{}] warnings={}",
            check.is_valid(),
            check.lines().join(" | "),
            check.warnings().count()
        ),
        Err(e) => println!("{label}: face={face:?} check_brep error={e}"),
    }
}

#[test]
fn analytic_surfaces_make_faces_directly() {
    let mut k = Kernel::default();
    make_one_face("plane", plane(), &mut k, true);
    make_one_face("cylinder", cylinder(&k), &mut k, true);
    make_one_face("torus", torus(&k), &mut k, true);
    make_one_face("extrusion", extrusion(&k), &mut k, true);
    make_one_face("revolution", revolution(&k), &mut k, true);
}

#[test]
fn split_nurb_pieces_make_faces() {
    let mut k = Kernel::default();
    let cases = [
        ("plane", plane()),
        ("cylinder", cylinder(&k)),
        ("torus", torus(&k)),
        ("extrusion", extrusion(&k)),
        ("revolution", revolution(&k)),
    ];
    for (label, surface) in cases {
        let domain = surface.natural_uv_domain_with_nurb(&k).unwrap();
        let split = 0.5 * (domain.min().x + domain.max().x);
        let (left, right) = surface.split_at(&k, split, SurfParamType::U).unwrap();
        make_one_face(&format!("{label}-left"), left, &mut k, false);
        make_one_face(&format!("{label}-right"), right, &mut k, false);
    }
}

#[test]
fn analytic_surfaces_convert_and_split_without_geometric_loss() {
    let k = Kernel::default();
    audit_surface("plane", plane(), &k);
    audit_surface("cylinder", cylinder(&k), &k);
    audit_surface("torus", torus(&k), &k);
    audit_surface("extrusion", extrusion(&k), &k);
    audit_surface("revolution", revolution(&k), &k);
}
