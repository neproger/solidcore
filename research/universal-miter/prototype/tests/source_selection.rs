#![cfg(feature = "solidcore")]
#![allow(
    missing_docs,
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::panic,
    clippy::indexing_slicing
)]

//! `PlanarSweep2` source selection: exact line/arc or Level B fallback.

use solidcore::curve::curve::p2::Curve;
use solidcore::kernel::Kernel;
use solidcore::math::Point3;
use solidcore::primitive::api_curves;

use universal_miter_prototype::level_b_geometry::SurfaceBuildOptions;
use universal_miter_prototype::planar_sweep2::{NormalizedPlanarSweep, normalize_planar_sweep};
use universal_miter_prototype::source_selection::{
    GeometryProvenance, PathCorrespondence, SourceCoverage, SweepSourceKind, classify_piece,
    select_sweep_sources,
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

fn normalized(k: &Kernel, profile: &Curve, paths: &[Curve]) -> NormalizedPlanarSweep {
    normalize_planar_sweep(k, std::slice::from_ref(profile), paths, true, 3).expect("normalize")
}

fn kinds(k: &Kernel, sweep: &NormalizedPlanarSweep) -> Vec<SweepSourceKind> {
    (0..sweep.placements.len())
        .map(|index| classify_piece(k, sweep, index))
        .collect()
}

fn line_line_line() -> Vec<Curve> {
    vec![
        line(Point3::ZERO, Point3::new(1.0, 0.0, 0.0)),
        line(Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)),
        line(Point3::new(1.0, 1.0, 0.0), Point3::new(2.0, 1.0, 0.0)),
    ]
}

fn line_arc_line(k: &Kernel) -> Vec<Curve> {
    let path0 = line(Point3::ZERO, Point3::new(5.0, 0.0, 0.0));
    let path1 = arc(Point3::new(5.0, 4.0, 0.0), 4.0, -90.0, 60.0);
    let path1_curve = path1.clone();
    let end = *path1_curve
        .evaluate(k, path1_curve.natural_interval().max(), 0, true, false)
        .expect("arc end")
        .first()
        .expect("end point");
    let tangent = path1_curve
        .evaluate(k, path1_curve.natural_interval().max(), 1, true, false)
        .expect("arc end tangent")
        .get(1)
        .copied()
        .expect("tangent")
        .unitize()
        .expect("unit tangent");
    vec![path0, path1, line(end, end + tangent * 2.0)]
}

fn arc_line_arc() -> Vec<Curve> {
    vec![
        arc(Point3::ZERO, 2.0, 0.0, 90.0),
        line(Point3::new(0.0, 2.0, 0.0), Point3::new(-2.0, 2.0, 0.0)),
        arc(Point3::new(-4.0, 2.0, 0.0), 2.0, 0.0, 90.0),
    ]
}

fn assert_kinds(actual: &[SweepSourceKind], expected: &[SweepSourceKind]) {
    assert_eq!(actual, expected);
}

#[test]
fn source_selection_chooses_exact_for_all_supported_pieces() {
    let k = Kernel::default();
    let profile = circle(&k);

    let ll = normalized(&k, &profile, &line_line_line());
    assert_kinds(
        &kinds(&k, &ll),
        &[
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactExtrusion,
        ],
    );

    let lal = normalized(&k, &profile, &line_arc_line(&k));
    assert_kinds(
        &kinds(&k, &lal),
        &[
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactRevolution,
            SweepSourceKind::ExactExtrusion,
        ],
    );

    let ala = normalized(&k, &profile, &arc_line_arc());
    assert_kinds(
        &kinds(&k, &ala),
        &[
            SweepSourceKind::ExactRevolution,
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactRevolution,
        ],
    );
}

#[test]
fn source_selection_falls_back_only_for_the_unsupported_middle_piece() {
    let k = Kernel::default();
    let profile = ellipse(&k);
    let lal = normalized(&k, &profile, &line_arc_line(&k));
    assert_kinds(
        &kinds(&k, &lal),
        &[
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::LevelBApproximation,
            SweepSourceKind::ExactExtrusion,
        ],
    );
}

#[test]
fn source_selection_builds_mixed_exact_chain_inputs() {
    let k = Kernel::default();
    let options = SurfaceBuildOptions {
        tolerance: 1.0e-5,
        max_refinements: 6,
        max_sections: 64,
        ..SurfaceBuildOptions::default()
    };

    let ll = normalized(&k, &circle(&k), &line_line_line());
    let selected = select_sweep_sources(&k, &ll, &options).expect("line chain sources");
    assert_eq!(selected.len(), 3);
    assert!(
        selected
            .iter()
            .all(|piece| piece.kind == SweepSourceKind::ExactExtrusion)
    );
    for piece in &selected {
        assert!(piece.u_domain.max().x > piece.u_domain.min().x);
        assert!(piece.v_domain.max().y > piece.v_domain.min().y);
        let geometry = piece.geometry();
        assert_eq!(geometry.surface_domain, piece.u_domain);
        assert_eq!(geometry.profile_domain, piece.profile.natural_interval());
        assert_eq!(geometry.path_domain, piece.path.natural_interval());
        assert_eq!(geometry.provenance, GeometryProvenance::Original);
        assert_eq!(piece.coverage, SourceCoverage::WithExtension);
        assert_eq!(piece.path_correspondence, PathCorrespondence::Independent);
    }

    let lal = normalized(&k, &circle(&k), &line_arc_line(&k));
    let selected = select_sweep_sources(&k, &lal, &options).expect("line-arc-line sources");
    assert_eq!(selected.len(), 3);
    assert_eq!(
        selected.iter().map(|piece| piece.kind).collect::<Vec<_>>(),
        vec![
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactRevolution,
            SweepSourceKind::ExactExtrusion,
        ]
    );
    for piece in &selected {
        assert_eq!(piece.geometry().provenance, GeometryProvenance::Original);
        if piece.kind == SweepSourceKind::ExactRevolution {
            assert_eq!(piece.coverage, SourceCoverage::OriginalOnly);
            assert_eq!(piece.path_correspondence, PathCorrespondence::Identity);
        } else {
            assert_eq!(piece.coverage, SourceCoverage::WithExtension);
            assert_eq!(piece.path_correspondence, PathCorrespondence::Independent);
        }
    }

    let ala = normalized(&k, &circle(&k), &arc_line_arc());
    let selected = select_sweep_sources(&k, &ala, &options).expect("arc-line-arc sources");
    assert_eq!(
        selected.iter().map(|piece| piece.kind).collect::<Vec<_>>(),
        vec![
            SweepSourceKind::ExactRevolution,
            SweepSourceKind::ExactExtrusion,
            SweepSourceKind::ExactRevolution,
        ]
    );
    for piece in &selected {
        assert_eq!(piece.geometry().provenance, GeometryProvenance::Original);
        if piece.kind == SweepSourceKind::ExactRevolution {
            assert_eq!(piece.coverage, SourceCoverage::OriginalOnly);
            assert_eq!(piece.path_correspondence, PathCorrespondence::Identity);
        } else {
            assert_eq!(piece.coverage, SourceCoverage::WithExtension);
            assert_eq!(piece.path_correspondence, PathCorrespondence::Independent);
        }
    }
}
