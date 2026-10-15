//! Native query.brep surface API wrappers.
//!
//! Input: present [surface block as `surface_surface_cache_p3`], uv, three xyz
//! output sentinels. Output: status, point/normal, du, dv; even errors preserve
//! the caller's sentinels. The existing validated surface builder bounds NURB
//! allocation and requires valid axes/knots. UV is passed without a finite/range
//! guard because these API entry points themselves reject NaN and outside values
//! before evaluating the kernel. No out-of-domain geometry is evaluated.
use super::surface_surface_cache_p1::Reader;
use super::surface_surface_cache_p3::sized_surface;
use crate::{call_with_buffer, FfiError};
mod sys {
    extern "C" {
        pub fn sc_query_brep_surface(
            op: i32,
            input: *const f64,
            n_in: i64,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}
/// Validate a surface API input (operation 0 point, 1 normal, 2 derivatives).
pub fn validate(op: u8, input: &[f64]) -> Result<(), FfiError> {
    if op > 2 {
        return Err(FfiError::InvalidArgument("surface operation".into()));
    }
    let mut rd = Reader { input, at: 0 };
    if rd.flag()? {
        sized_surface(&mut rd)?;
    }
    rd.num()?;
    rd.num()?;
    rd.finites(9)?;
    rd.done()
}
/// Invoke the actual surface API and return its status and unchanged-or-written outputs.
pub fn surface(op: u8, input: &[f64]) -> Result<Vec<f64>, FfiError> {
    validate(op, input)?;
    call_with_buffer(10, |buf, len| {
        // SAFETY: validate checked the complete layout, geometry and bounded counts;
        // UV is checked by the native API before geometry evaluation. Buffers live
        // throughout this call and the shim does not retain pointers.
        unsafe {
            sys::sc_query_brep_surface(
                i32::from(op),
                input.as_ptr(),
                input.len() as i64,
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
            )
        }
    })
}

mod classify_sys {
    extern "C" {
        pub fn sc_query_brep_classify(
            point: *const f64,
            tolerance: f64,
            boundary: i32,
            regions: i32,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}
/// Classify a point on a real empty B-rep with one to four constructed regions.
/// This fixture never
/// traverses nonempty geometry; all floating inputs are valid scalar payloads.
pub fn classify(
    point: &[f64; 3],
    tolerance: f64,
    boundary: bool,
    regions: u8,
) -> Result<Vec<f64>, FfiError> {
    if !(1..=4).contains(&regions) {
        return Err(FfiError::InvalidArgument("region count".into()));
    }
    call_with_buffer(14, |buf, len| {
        // SAFETY: point has three live doubles; boundary is a boolean; the shim
        // constructs a real B-rep and copies into the supplied live buffers.
        unsafe {
            classify_sys::sc_query_brep_classify(
                point.as_ptr(),
                tolerance,
                i32::from(boundary),
                i32::from(regions),
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
            )
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_malformed_layout_before_native_read_or_allocation() {
        for input in [
            vec![],
            vec![2.0],
            vec![1.0, 0.0, 0.0, 0.0, 1e30],
            vec![0.0; 11],
            vec![0.0; 13],
        ] {
            assert!(validate(0, &input).is_err());
        }
        assert!(validate(3, &[0.0; 12]).is_err());
    }
    #[test]
    fn lets_the_native_api_reject_nonfinite_parameters() {
        let mut input = [0.0; 12];
        input[1] = f64::NAN;
        input[2] = f64::INFINITY;
        assert!(validate(0, &input).is_ok());
    }
}
