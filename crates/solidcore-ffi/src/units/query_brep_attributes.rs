//! Native constructor and copy checks for query.brep attributes.
use crate::{call_with_buffer, FfiError};
mod sys {
    extern "C" {
        pub fn sc_query_brep_attributes(
            kind: i32,
            id: u64,
            behavior: i32,
            mutate: i32,
            sort: u64,
            cls: i32,
            data: *const f64,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
            ints: *mut u64,
        ) -> i32;
    }
}
/// Original and copied payloads, with exact integer ids and bookkeeping.
pub struct Record {
    /// Constructor, original after mutation, and copied floating-point payloads.
    pub values: Vec<f64>,
    /// Original/copy ids, behaviours, sort indices and vertex classes.
    pub integers: [u64; 8],
}
/// Run a constructor, optional friend writes, and the virtual copy operation.
/// No uninitialised native field is read; edge sort is observed only after writing.
pub fn eval(
    kind: u8,
    id: u64,
    behavior: u8,
    mutate: bool,
    sort: u64,
    cls: u8,
    data: &[f64; 13],
) -> Result<Record, FfiError> {
    if kind > 2 || behavior > 4 || cls > 5 {
        return Err(FfiError::InvalidArgument("attribute payload".into()));
    }
    let mut integers = [0; 8];
    let values = call_with_buffer(39, |buf, len| {
        // SAFETY: flags are validated, data has 13 live doubles, integer output
        // has eight slots, and call_with_buffer supplies the remaining buffers.
        unsafe {
            sys::sc_query_brep_attributes(
                i32::from(kind),
                id,
                i32::from(behavior),
                i32::from(mutate),
                sort,
                i32::from(cls),
                data.as_ptr(),
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
                integers.as_mut_ptr(),
            )
        }
    })?;
    Ok(Record { values, integers })
}
