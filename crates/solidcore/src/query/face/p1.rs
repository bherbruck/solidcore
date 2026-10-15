//! Face queries and checked curve evaluation entry points.
//!
//! Upstream: `source/SM_API/src/SmApiCurves.cpp`, `source/SMLib/inc/SmCurve.h`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`evaluate_curve`] | `SmApiEvaluateCurve` |
//!
//! Deviations: a nullable curve is `Option<&Curve>`. The three optional output
//! pointers become a returned array `(position, first derivative, second
//! derivative)`; evaluation still requests all three, even when callers discard
//! some outputs. No recursion is introduced.

use crate::curve::curve::Curve;
use crate::geom::vector3d::Vec3;
use crate::{Result, SolidError};

/// Evaluate position and the first two derivatives within the natural domain.
///
/// Uses the API's inclusive range check and the upstream defaults: evaluation
/// from the left, with nonzero tangents enabled. Both settings affect repeated
/// knots and collapsed tangents.
///
/// # Errors
/// Returns invalid input for a null curve, NaN parameter or parameter outside the
/// natural domain. Propagates the concrete curve evaluator's error unchanged.
pub fn evaluate_curve(curve: Option<&Curve>, parameter: f64) -> Result<[Vec3; 3]> {
    const OP: &str = "query::face::p1::evaluate_curve";
    let curve = curve.ok_or_else(|| SolidError::invalid_input(OP, "null curve"))?;
    let domain = curve.natural_interval();
    // Keep the negated conjunction: NaN must fail the API's range check.
    if !(parameter >= domain.min() && parameter <= domain.max()) {
        return Err(SolidError::invalid_input(
            OP,
            "parameter outside natural domain",
        ));
    }
    let values = curve.evaluate(parameter, 2, true, true)?;
    let mut iter = values.into_iter();
    let missing = || SolidError::invalid_input(OP, "curve returned too few derivatives");
    Ok([
        iter.next().ok_or_else(missing)?,
        iter.next().ok_or_else(missing)?,
        iter.next().ok_or_else(missing)?,
    ])
}
