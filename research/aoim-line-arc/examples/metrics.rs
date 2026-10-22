//! Print the 2D region metrics used by the AOIM hybrid investigation.
//!
//! Run from `research/aoim-line-arc`:
//!
//! ```sh
//! cargo run --example metrics
//! ```

use aoim_line_arc_research::continuing::ContinuingLineArc;
use aoim_line_arc_research::sweep::{build_regions, build_untrimmed, RegionError, Strategy};
use aoim_line_arc_research::LineArc;

fn main() {
    let strategies = [
        ("analytical", Strategy::Analytical),
        ("hybrid-junction", Strategy::HybridAtJunction),
        ("hybrid-limit-tangent", Strategy::HybridAtLimitTangent),
        ("hybrid-limit-bisector", Strategy::HybridAtLimitBisector),
    ];
    let profiles = [
        ("symmetric", -2.0_f64, 2.0_f64),
        ("positive", 0.5, 2.0),
        ("negative", -2.0, -0.5),
    ];

    for degrees in [30.0_f64, 90.0, 150.0] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        println!("\n=== alpha = {degrees:.0} deg, R = 4 ===");
        for (profile_name, d0, d1) in profiles {
            println!("profile {profile_name}: d in [{d0}, {d1}]");
            match build_untrimmed(&model, d0, d1, 20.0, 96, 600) {
                Ok((_, _, metrics)) => println!(
                    "  untrimmed: line={:.4} arc={:.4} overlap={:.4} union={:.4} ({:.2e})",
                    metrics.line_area,
                    metrics.arc_area,
                    metrics.overlap_area,
                    metrics.union_area,
                    metrics.overlap_area / metrics.union_area.max(1.0),
                ),
                Err(error) => println!("  untrimmed: {error:?}"),
            }
            for (strategy_name, strategy) in strategies {
                match build_regions(&model, strategy, d0, d1, 20.0, 401, 96) {
                    Ok(regions) => {
                        let raster = regions.raster;
                        println!(
                            "  {strategy_name:22} line={:8.4} arc={:8.4} expected={:8.4} \
                             balance={:+.2e} ({:+.2e}) self={} overlap={:.2e} gap={:.2e} \
                             sym={:.2e} ({:.2e})",
                            regions.line.area(),
                            regions.arc.area(),
                            regions.expected.area(),
                            regions.area_balance,
                            regions.area_balance_fraction,
                            regions.self_intersecting,
                            raster.overlap_area / raster.expected_area,
                            raster.gap_area / raster.expected_area,
                            raster.symmetric_difference_area,
                            raster.symmetric_difference_fraction,
                        );
                        if regions.self_intersecting {
                            println!(
                                "    line self={:?} arc self={:?}",
                                regions.line.self_intersection_pairs(),
                                regions.arc.self_intersection_pairs()
                            );
                        }
                    }
                    Err(error) => {
                        let detail = match error {
                            RegionError::UncoveredProfile { first_missing } => {
                                format!("UncoveredProfile(first_missing={first_missing:.6})")
                            }
                            other => format!("{other:?}"),
                        };
                        println!("  {strategy_name:22} {detail}");
                    }
                }
            }
        }
    }
    print_kinematics();
}

fn print_kinematics() {
    println!("\n=== connector kinematics ===");
    for degrees in [30.0_f64, 90.0, 150.0] {
        let model = LineArc::new(4.0, degrees.to_radians()).unwrap();
        let half = 0.5 * model.corner_angle_rad;
        let tangent = half.sin() / half.cos();
        let joint_tangent = model.joint_tangent();
        let minimum = model.min_offset();
        let mut limit_cross_max: f64 = 0.0;
        let mut internal_cross_max: f64 = 0.0;
        let mut min_angle = f64::INFINITY;
        let mut max_angle = f64::NEG_INFINITY;
        for step in [1e-6_f64, 0.1, 0.5, 1.0] {
            let d = minimum - step;
            if model.radius + d <= 0.0 {
                continue;
            }
            let joint = model.joint_arc_point(d).unwrap();
            let limit = model.limit_linear_point(d).unwrap();
            let delta = (limit.x - joint.x, limit.y - joint.y);
            let cross = delta.0 * joint_tangent.y - delta.1 * joint_tangent.x;
            limit_cross_max = limit_cross_max.max(cross.abs());
            let angle = delta.1.atan2(delta.0);
            min_angle = min_angle.min(angle);
            max_angle = max_angle.max(angle);
            let internal = model.internal_junction_point(d).unwrap();
            let internal_delta = (internal.x - joint.x, internal.y - joint.y);
            let internal_cross =
                internal_delta.0 * joint_tangent.y - internal_delta.1 * joint_tangent.x;
            internal_cross_max = internal_cross_max.max(internal_cross.abs());
        }
        let continuing = ContinuingLineArc::new(4.0, degrees.to_radians()).unwrap();
        let maximum = continuing.max_offset();
        let forward_parameter = continuing.tangent_parameter(1.0);
        println!(
            "{degrees:5.1} deg: tan(a/2)={tangent:.4} hybrid t(-2)={:.3} t(+2)={:.3} | \
             limit cross_max={limit_cross_max:.3} angle_spread={:.3} | \
             internal cross_max={internal_cross_max:.3} | \
             continuing d_max={maximum:.3}, AOIM(d=1)={}, t(1)={forward_parameter:.3}",
            -2.0 * tangent,
            2.0 * tangent,
            max_angle - min_angle,
            continuing.intersect_offsets(1.0).is_err(),
        );
    }
}
