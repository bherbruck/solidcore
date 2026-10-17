//! FFI wrappers for unit `intersect.surface_drop_lib_p3`.
//!
//! Shim: `harness/cpp/shim/intersect_surface_drop_lib_p3.cpp`
//! (`sc_intersect_surface_drop_lib_p3_run`). Upstream:
//! `source/SMLib/src/SmSurfaceDropLib.cpp` (`SmSurface::DropCurve`, the base body,
//! called non-virtually so a plane does not take its override).
//!
//! [`run`] runs one [`Drop`] and returns its record:
//! `status max_drop max_dev ncurves curves` and, when the failure record is asked for,
//! the record: `fail_status from_call path has_context uv_domain[4] interval[2] tol
//! max_drop_arg max_dev_arg keep_all degen_curve degen_u degen_v contained max_drop
//! closed_u closed_v closure singularities approx_singularities npoles poles(x y z)
//! nnormals normals(x y z) nsols sols(u v) npoly poly(x y z) nknots knots ncurves
//! curves`. A status is `0` for `SM_SUCCESS`, else the `SmStatus`; a B-spline curve is
//! `degree ncp cpts(x y z w) nknots knots mults ivl_min ivl_max`; `closure` is 0 `U`,
//! 1 `V`, 2 `BOTH`, 3 `NEITHER`, 4 `UNKNOWN`; `path` is the `SmCreateUVTrimCurvePathType`
//! value. The surface and curve blocks are those of
//! [`super::intersect_surface_drop_lib_p2`] ([`SurfaceIn`], [`CurveIn`]).
//!
//! A surface of revolution whose `CreateCanonical` fails makes the record
//! [[`NO_SURFACE`]]. [`run`] accepts the surfaces and curves the p2 wrapper accepts,
//! finite domains, any tolerance with `|tol| <= MAX_COORD` (1e9; zero, negative and
//! `SM_USE_DEFAULT` included: `SmApproxTol3d` takes `fabs`) and any finite
//! interval: zero-length, inverted (`SmExtent1d` swaps the ends) and outside the curve's
//! parameter range (B-spline `Evaluate` clamps, a line and a circle extend, so the ignored
//! `Evaluate`s of `SmCurve::IsDegenerate` never fail). It rejects only a non-degenerate
//! interval of length `L` with `L / 8000 <= 2 ulp(max |end|)`: there upstream's span
//! subdivision (`(start + end) / 2`) cannot shrink a failing span to `L / 8000` and loops
//! forever (plan/upstream-issues.md HANG-16; the port bounds that depth,
//! `MAX_SPAN_DEPTH`).

use crate::{call_with_buffer, FfiError};

pub use super::intersect_surface_drop_lib_p2::{CurveIn, SurfaceIn, MAX_COORD, NOW};
use super::intersect_surface_drop_lib_p2::{encode_curve, encode_surface};

mod sys {
    extern "C" {
        pub fn sc_intersect_surface_drop_lib_p3_run(
            input: *const f64,
            n_in: i64,
            out: *mut f64,
            cap: i64,
            out_len: *mut i64,
        ) -> i32;
    }
}

/// The record of a drop whose surface of revolution could not be made.
pub const NO_SURFACE: f64 = -3.0;

/// One `SmSurface::DropCurve` call.
#[derive(Debug, Clone, PartialEq)]
pub struct Drop {
    /// The surface.
    pub surface: SurfaceIn,
    /// The 3D curve.
    pub curve: CurveIn,
    /// `crUVDomain`: `umin vmin umax vmax`.
    pub domain: [f64; 4],
    /// `crInterval`.
    pub interval: [f64; 2],
    /// `sApproxTol3d`.
    pub tol: f64,
    /// `bKeepAllDropCurves`.
    pub keep_all: bool,
    /// Pass an `SmDropCurveFail` record (else `NULL`).
    pub want_record: bool,
}

fn bad(msg: &str) -> FfiError {
    FfiError::InvalidArgument(msg.into())
}

fn finite(x: f64) -> Result<f64, FfiError> {
    if x.is_finite() && x.abs() <= MAX_COORD {
        Ok(x)
    } else {
        Err(bad("value not finite or too large"))
    }
}

/// Encodes `d` as the shim's input.
///
/// # Errors
///
/// [`FfiError::InvalidArgument`] for a rejected drop.
pub fn encode(d: &Drop) -> Result<Vec<f64>, FfiError> {
    let mut v = Vec::new();
    encode_surface(&d.surface, &mut v)?;
    encode_curve(&d.curve, &mut v)?;
    for &x in &d.domain {
        v.push(finite(x)?);
    }
    let [i0, i1] = d.interval;
    finite(i0)?;
    finite(i1)?;
    // HANG-16: a non-degenerate interval whose span the subdivision can never shrink
    // below `length / 8000` (a float spacing inside it is at least that) hangs upstream.
    // Zero-length, inverted (`SmExtent1d` swaps) and out-of-range intervals are fine.
    let (lo, hi) = if i0 <= i1 { (i0, i1) } else { (i1, i0) };
    let len = hi - lo;
    let m = lo.abs().max(hi.abs());
    if len > 0.0 && len / 8000.0 <= 2.0 * (m.next_up() - m) {
        return Err(bad("interval too short for the span subdivision (HANG-16)"));
    }
    v.push(i0);
    v.push(i1);
    let tol = finite(d.tol)?;
    v.push(tol);
    v.push(if d.keep_all { 1.0 } else { 0.0 });
    v.push(if d.want_record { 1.0 } else { 0.0 });
    Ok(v)
}

/// Runs `d` (see the module docs) and returns its record.
///
/// # Errors
///
/// [`FfiError::InvalidArgument`] for rejected inputs; [`FfiError::Status`] for a shim
/// failure.
pub fn run(d: &Drop) -> Result<Vec<f64>, FfiError> {
    let input = encode(d)?;
    let n_in = i64::try_from(input.len()).map_err(|_| bad("program too long"))?;
    call_with_buffer(4096, |buf: &mut [f64], len| {
        // SAFETY: `input` is valid for `n_in` reads; `buf` for `buf.len()` writes; `len`
        // is a valid i64; nothing is retained.
        unsafe {
            sys::sc_intersect_surface_drop_lib_p3_run(
                input.as_ptr(),
                n_in,
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

    fn drop_with(interval: [f64; 2], tol: f64) -> Drop {
        Drop {
            surface: SurfaceIn::Plane {
                o: [0.0; 3],
                x: [1.0, 0.0, 0.0],
                y: [0.0, 1.0, 0.0],
                scale: [1.0, 1.0],
                domain: [0.0, 0.0, 1.0, 1.0],
            },
            curve: CurveIn::Line {
                a: [0.0; 3],
                b: [1.0, 0.0, 0.0],
            },
            domain: [0.0, 0.0, 1.0, 1.0],
            interval,
            tol,
            keep_all: true,
            want_record: true,
        }
    }

    #[test]
    fn guard_rejects_only_the_hanging_intervals() {
        // accepted: zero-length, inverted, outside the range, any tolerance
        for (ivl, tol) in [
            ([0.5, 0.5], 1.0e-4),
            ([1.0, 0.0], 1.0e-4),
            ([-0.5, 2.0], 1.0e-4),
            ([2.0, 3.0], 1.0e-4),
            ([0.0, 1.0], 0.0),
            ([0.0, 1.0], -1.0),
            ([0.0, 1.0], 1.0e-12),
        ] {
            assert!(encode(&drop_with(ivl, tol)).is_ok(), "{ivl:?} {tol}");
        }
        // rejected (HANG-16): a non-degenerate interval of at most 16000 ulps
        let m = 1.0f64;
        let ulp = m.next_up() - m;
        assert!(encode(&drop_with([1.0, 1.0 + 100.0 * ulp], 1.0e-4)).is_err());
        assert!(encode(&drop_with([1.0 + 100.0 * ulp, 1.0], 1.0e-4)).is_err());
        assert!(encode(&drop_with([1.0, 1.0 + 20000.0 * ulp], 1.0e-4)).is_ok());
        assert!(encode(&drop_with([0.0, f64::MIN_POSITIVE], 1.0e-4)).is_ok());
        // non-finite stays rejected
        assert!(encode(&drop_with([0.0, f64::NAN], 1.0e-4)).is_err());
    }
}
