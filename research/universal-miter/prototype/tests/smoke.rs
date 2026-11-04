#![allow(clippy::expect_used, clippy::unwrap_used, clippy::panic)]

//! Infrastructure smoke test: the harness is wired, `Original` is not
//! implemented at level A, and `Universal` runs a simple case.

use universal_miter_prototype::curve::CurveSpec;
use universal_miter_prototype::harness::{
    run_case, MiterError, MiterStatus, MiterStrategy, MiterTestCase, ProfileSpec,
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
fn original_is_not_implemented_and_universal_runs() -> Result<(), UnexpectedResult> {
    let case = line_line_case();
    match run_case(&case, MiterStrategy::Original) {
        Err(MiterError::NotImplemented) => {}
        _ => return Err(UnexpectedResult),
    }
    let Ok(result) = run_case(&case, MiterStrategy::Universal) else {
        return Err(UnexpectedResult);
    };
    if result.status != MiterStatus::Ok {
        return Err(UnexpectedResult);
    }
    if result.diagnostics.max_abs_rail_residual > 1.0e-9 {
        return Err(UnexpectedResult);
    }
    Ok(())
}
