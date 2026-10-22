use aoim_line_arc_research::sweep3d::{build_profile_pair, ProfileSweep3};
use aoim_line_arc_research::{LineArc, Point2};

fn p(d: f64, z: f64) -> Point2 {
    Point2 { x: d, y: z }
}

fn report(name: &str, result: Result<ProfileSweep3, impl std::fmt::Debug>) {
    match result {
        Ok(sweep) => println!(
            "{name}: profile_area={:.4} mesh line={:.4} arc={:.4} combined={:.4} ref line={:.4} arc={:.4} combined={:.4} seam_faces={} seam_area={:.4}",
            sweep.reference.profile_area,
            sweep.pair.metrics.line_volume,
            sweep.pair.metrics.arc_volume,
            sweep.pair.metrics.combined_volume,
            sweep.reference.line_volume,
            sweep.reference.arc_volume,
            sweep.reference.combined_volume,
            sweep.pair.metrics.seam_face_count,
            sweep.pair.metrics.seam_area,
        ),
        Err(error) => println!("{name}: {error:?}"),
    }
}

fn main() {
    let model = LineArc::new(4.0, 150.0_f64.to_radians()).unwrap();
    let rectangle = [p(-2.0, -1.0), p(2.0, -1.0), p(2.0, 1.0), p(-2.0, 1.0)];
    let l_shape = [
        p(-2.0, -1.0),
        p(0.0, -1.0),
        p(0.0, 0.0),
        p(1.0, 0.0),
        p(1.0, 1.0),
        p(-2.0, 1.0),
    ];
    let t_shape = [
        p(-1.0, -1.0),
        p(1.0, -1.0),
        p(1.0, 0.0),
        p(2.0, 0.0),
        p(2.0, 1.0),
        p(-2.0, 1.0),
        p(-2.0, 0.0),
        p(-1.0, 0.0),
    ];
    for (name, profile) in [
        ("rectangle", &rectangle[..]),
        ("L", &l_shape[..]),
        ("T", &t_shape[..]),
    ] {
        report(name, build_profile_pair(model, profile, 20.0, 16, 48));
    }
}
