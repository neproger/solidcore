//! Level B: the integration prototype on the real SolidCore types.
//!
//! Enabled only with `--features solidcore`; the production build never turns
//! it on. This module is a wiring placeholder: it proves that the isolated
//! research workspace can reach the kernel without modifying it. The runner
//! will be added after the mathematical prototype and the Python reference
//! agree.
//!
//! The intended real types are
//! [`solidcore::curve::curve::Curve`],
//! `solidcore::nurbs::b_spline_curve::p1::BSplineCurve`,
//! `solidcore::primitive::primitive_creation::p3::PrimitiveCreation::create_curve_sweep`,
//! and `solidcore::topology::brep::p10::project_and_trim`.

/// The kernel's curve type, re-exported for the integration runner.
pub type KernelCurve = solidcore::curve::curve::Curve;
