//! Infrastructure smoke test: the harness is wired, the algorithm is not.
//!
//! This test must stay independent of the kernel (level A). The level B check
//! is `cargo test --features solidcore`, handled separately.

use universal_miter_prototype::curve::CurveSpec;
use universal_miter_prototype::harness::{
    run_case, MiterError, MiterStrategy, MiterTestCase, ProfileSpec,
};

/// The test's failure value; `Result<(), _>` implements `Termination` for a
/// `Debug` error.
#[derive(Debug)]
struct UnexpectedResult;

/// A case that every strategy can accept as well-formed.
fn line_line_case() -> MiterTestCase {
    MiterTestCase {
        id: String::from("smoke-line-line"),
        curve_a: CurveSpec::Line {
            start: [-2.0, 0.0],
            end: [0.0, 0.0],
        },
        curve_b: CurveSpec::Line {
            start: [0.0, 0.0],
            end: [0.0, 2.0],
        },
        profile: ProfileSpec {
            alpha: [-0.5, 0.5],
            beta: [0.0, 1.0],
        },
        expected: None,
    }
}

#[test]
fn both_strategies_are_not_implemented_yet() -> Result<(), UnexpectedResult> {
    let case = line_line_case();
    for strategy in [MiterStrategy::Original, MiterStrategy::Universal] {
        match run_case(&case, strategy) {
            Err(MiterError::NotImplemented) => {}
            _ => return Err(UnexpectedResult),
        }
    }
    Ok(())
}
