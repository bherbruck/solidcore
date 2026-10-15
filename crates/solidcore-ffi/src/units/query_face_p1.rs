//! FFI for `query.face_p1` (`SmApiEvaluateCurve`).
//!
//! Shim: `harness/cpp/shim/query_face_p1.cpp`.
//! Inputs construct a real null curve (kind 0), line (1) or Hermite curve (2).
//! `data` contains `(p1, d1, p2, d2)`, each xyz; lines use p1 and p2.

use crate::{check, FfiError};

mod sys {
    extern "C" {
        pub fn sc_query_face_p1_curve(
            kind: i32,
            data: *const f64,
            parameter: f64,
            mask: i32,
            out: *mut f64,
            operation_status: *mut i32,
        ) -> i32;
    }
}

/// Call the real API, retaining seeded outputs where optional pointers are null
/// or evaluation fails. `mask` bits 0, 1 and 2 request point, d1 and d2.
///
/// Geometry must be finite; parameter NaN/infinity is deliberately admitted so
/// the API's own natural-domain rejection is compared.
pub fn evaluate_curve(
    kind: i32,
    data: &[f64; 12],
    parameter: f64,
    mask: i32,
) -> Result<(i32, [f64; 9]), FfiError> {
    if !(0..=2).contains(&kind) || !(0..=7).contains(&mask) || data.iter().any(|x| !x.is_finite()) {
        return Err(FfiError::InvalidArgument(
            "curve construction or output mask".into(),
        ));
    }
    let mut values = [0.0; 9];
    let mut status = 0;
    // SAFETY: fixed arrays provide exactly 12 reads and 9 writes; kind and mask
    // are checked and all borrowed buffers remain live throughout the call.
    let result = unsafe {
        sys::sc_query_face_p1_curve(
            kind,
            data.as_ptr(),
            parameter,
            mask,
            values.as_mut_ptr(),
            &raw mut status,
        )
    };
    check(result)?;
    Ok((status, values))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn malformed_construction_and_masks_fail_before_cpp() {
        let data = [0.0; 12];
        for kind in [-1, 3, i32::MAX] {
            assert!(matches!(
                evaluate_curve(kind, &data, 0.5, 7),
                Err(FfiError::InvalidArgument(_))
            ));
        }
        for mask in [-1, 8, i32::MAX] {
            assert!(matches!(
                evaluate_curve(1, &data, 0.5, mask),
                Err(FfiError::InvalidArgument(_))
            ));
        }
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut data = data;
            data[0] = bad;
            assert!(matches!(
                evaluate_curve(1, &data, 0.5, 7),
                Err(FfiError::InvalidArgument(_))
            ));
        }
    }
}
