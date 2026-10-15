//! Real B-rep box scripts and independent graph export for `query.brep`.
//! Layout: has-graph [standard P1 B-rep block], extra vertices/edges, three
//! tolerances, drop-wire-curves, step count, then (code, approx, tight, expand).
//! Counts are bounded before C++ allocates or reads them. `Export` constructs the
//! exact same graph without executing any box operation; setup failures remain
//! harness errors in the runner. No geometry approximation is used here.
use super::topology_edgeuse_p1::{brep_block, Reader};
use crate::{call_with_buffer, FfiError};
use std::ffi::CString;
mod sys {
    extern "C" {
        pub fn sc_query_brep_bbox(
            op: i32,
            path: *const std::ffi::c_char,
            input: *const f64,
            n_in: i64,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}
/// Validate a complete construction/script input before native reads or allocation.
pub fn validate(path: &str, input: &[f64]) -> Result<(), FfiError> {
    let mut rd = Reader { input, at: 0 };
    if rd.flag()? {
        brep_block(&mut rd, path)?;
    }
    rd.index(600)?;
    rd.index(120)?;
    rd.finites(3)?;
    rd.flag()?;
    let n = rd.index(16)?;
    for _ in 0..n {
        rd.index(4)?;
        rd.flag()?;
        rd.flag()?;
        rd.flag()?;
    }
    rd.done()
}
/// Construct a real graph and export it, or run the actual bounding-box script.
pub fn eval(export: bool, path: &str, input: &[f64]) -> Result<Vec<f64>, FfiError> {
    validate(path, input)?;
    let path =
        CString::new(path).map_err(|_| FfiError::InvalidArgument("path contains NUL".into()))?;
    call_with_buffer(128, |buf, len| {
        // SAFETY: full layout, counts, geometry and steps were validated; all
        // slices/CString and writable buffers live throughout the call. The
        // native shim owns and destroys the complete graph before returning.
        unsafe {
            sys::sc_query_brep_bbox(
                i32::from(!export),
                path.as_ptr(),
                input.as_ptr(),
                input.len() as i64,
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
    fn rejects_malformed_counts_before_native_reads_or_construction() {
        for (index, value) in [(1, 1e30), (2, f64::NAN), (7, f64::INFINITY)] {
            let mut input = [0.0; 8];
            input[index] = value;
            assert!(eval(true, "", &input).is_err());
        }
        assert!(eval(true, "", &[0.0; 7]).is_err());
        assert!(eval(true, "contains\0nul", &[0.0; 8]).is_err());
    }
}
