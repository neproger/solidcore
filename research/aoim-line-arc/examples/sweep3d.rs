use aoim_line_arc_research::sweep::Strategy;
use aoim_line_arc_research::sweep3d::{build_pair, Sweep3Params};
use aoim_line_arc_research::LineArc;

fn p(d0: f64, d1: f64, z0: f64, z1: f64, line_length: f64) -> Sweep3Params {
    Sweep3Params {
        d0,
        d1,
        z0,
        z1,
        line_length,
        seam_samples: 401,
        arc_steps: 96,
    }
}

fn main() {
    for (degrees, line_length) in [
        (30.0_f64, 20.0_f64),
        (90.0, 20.0),
        (150.0, 20.0),
        (179.0, 500.0),
    ] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        match build_pair(
            model,
            Strategy::HybridAtJunction,
            p(-2.0, 2.0, -1.0, 1.0, line_length),
        ) {
            Ok(pair) => {
                let metrics = pair.metrics;
                println!(
                    "{degrees}: line={:.3} arc={:.3} combined={:.3} expected={:.3} seam_faces={} seam_area={:.3} expected_seam={:.3}",
                    metrics.line_volume,
                    metrics.arc_volume,
                    metrics.combined_volume,
                    metrics.expected_combined_volume,
                    metrics.seam_face_count,
                    metrics.seam_area,
                    metrics.expected_seam_area,
                );
            }
            Err(error) => println!("{degrees}: {error:?}"),
        }
    }
}
