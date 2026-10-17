//! FFI wrappers for unit `topology.brep_p7`.
//!
//! Shim: `harness/cpp/shim/topology_brep_p7.cpp` (`sc_topology_brep_p7_eval`).
//! Upstream: `source/SMLib/src/SmBrep.cpp`.
//!
//! A curve block is `dim n p m cpts(x y z w)*(n+1) knots*(m+1)` (an `SmBSplineCurve`;
//! `w = NOW` non-rational, `z = NOZ` in 2D), a surface block `1 O[3] N[3]` (an `SmPlane`) or `0 oob form n
//! m p q r s cpts knots_u knots_v` (an `SmBSplineSurface`), as `sc_topology_fixture.h`
//! reads them. Outputs start with the status (`0` success, else the `SmStatus`) unless
//! noted; `-2` is "upstream would dereference null" (not called), `-3` "not run".
//!
//! | [`Op`]              | input | output |
//! |---------------------|-------|--------|
//! | `Read`              | `n byte*n file_type rebuild` | `status [counts]` (`SmBrep::ReadFromFile` of the bytes) |
//! | `MakeFaceWithCurves`| `surface u0 v0 u1 v1 orient region_null crv_on_surf nloops count* has3d has_uv ncurves (curve3d? curveuv? orient)* npts pt[3]*` | `-2 \| status [face shell region counts]` (`-2`: `crv_on_surf` TRUE with 3D but no UV curves, where upstream dereferences the null UV array) |
//! | `MakeFace`          | `kind side orient region_null` (`kind` 0: null surface, 1: a square wire loop on `z = 0`) | `-3 \| status [face shell region counts]` (`-3`: the square's lines cannot be made) |
//! | `Glue`              | `e1 e2 orient gap nesting`, B-rep block | `0 (-2 \| status counts)` (`-2`: `SM_OT_SAME`, equal intervals and one edge without curve) (`-1 status` when the B-rep build fails) |
//! | `Copy`              | `n use_merge`, B-rep block | `0 status nfaces counts` |
//! | `MergeSelf`         | `face tol`, B-rep block | `0 (-2 \| status nedges)` |
//!
//! `counts` are the region, shell, face, loop, edge and vertex counts of the B-rep made.
//!
//! [`eval`] accepts only what upstream handles without undefined behaviour: clamped
//! B-splines of degree 1 to 5 with at most 33 control points, positive (or `NOW`)
//! weights and finite values, planes with a non-zero normal, loop counts that add up to
//! the curves, indices inside the B-rep's export. For `Read` it applies the guard of
//! unit `topology.brep_data` to the bytes (a rejected stream is
//! [`FfiError::InvalidArgument`]) and, for bytes that are not an unmodified upstream test
//! file, runs `SmBrepData::ReadFromFile` first: when that read succeeds the topology it
//! read is not known to be safe for `MakeTopologyFromData`, and `[-3]` is returned
//! without calling the shim.

use super::topology_brep_data as bd;
use super::topology_vertexuse as fixture;
use crate::{call_with_buffer, FfiError};

/// Largest control point index.
pub const MAX_INDEX: usize = 32;
/// Largest degree.
pub const MAX_DEGREE: usize = 5;
/// Largest coordinate magnitude.
pub const MAX_COORD: f64 = 1.0e12;
/// Largest number of curves (and loops, loop points).
pub const MAX_CURVES: usize = 64;
/// Largest file size of `Read`.
pub const MAX_BYTES: usize = 1 << 22;

/// `NOW`: the weight of a non-rational control point.
const NOW: f64 = -f64::MAX;
/// `NOZ`: the z of a 2D control point.
const NOZ: f64 = f64::MAX;
/// "not run".
const NOT_RUN: f64 = -3.0;

/// Shim operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(missing_docs)]
pub enum Op {
    Read = 1,
    MakeFaceWithCurves = 2,
    MakeFace = 3,
    Glue = 4,
    Copy = 5,
    MergeSelf = 6,
}

mod sys {
    extern "C" {
        pub fn sc_topology_brep_p7_eval(
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

fn bad(msg: impl Into<String>) -> FfiError {
    FfiError::InvalidArgument(msg.into())
}

struct Reader<'a> {
    input: &'a [f64],
    at: usize,
}

impl Reader<'_> {
    fn num(&mut self) -> Result<f64, FfiError> {
        let v = self
            .input
            .get(self.at)
            .copied()
            .ok_or_else(|| bad("input too short"))?;
        self.at += 1;
        Ok(v)
    }

    fn finite(&mut self) -> Result<f64, FfiError> {
        let v = self.num()?;
        if !v.is_finite() || v.abs() > MAX_COORD {
            return Err(bad("non-finite or huge value"));
        }
        Ok(v)
    }

    fn index(&mut self, max: usize) -> Result<usize, FfiError> {
        let v = self.num()?;
        if v.fract() != 0.0 || v < 0.0 || v > max as f64 {
            return Err(bad(format!("bad index/count {v} (max {max})")));
        }
        Ok(v as usize)
    }

    fn signed(&mut self, max: usize) -> Result<Option<usize>, FfiError> {
        let v = self.num()?;
        if v == -1.0 {
            return Ok(None);
        }
        self.at -= 1;
        self.index(max).map(Some)
    }

    fn flag(&mut self) -> Result<bool, FfiError> {
        Ok(self.index(1)? == 1)
    }

    fn orient(&mut self) -> Result<(), FfiError> {
        // SM_OT_UNKNOWN (0), SM_OT_SAME (1) or SM_OT_OPPOSITE (2): upstream tests for
        // SAME and treats anything else as the other branch
        self.index(2).map(|_| ())
    }

    fn done(&self) -> Result<(), FfiError> {
        if self.at != self.input.len() {
            return Err(bad("trailing input"));
        }
        Ok(())
    }
}

/// Checks a clamped, non-decreasing knot vector of degree `p` with a non-empty domain.
fn check_knots(knots: &[f64], p: usize) -> Result<(), FfiError> {
    if knots.windows(2).any(|w| w.first() > w.last()) {
        return Err(bad("knots not non-decreasing"));
    }
    let (Some(&a), Some(&b)) = (knots.first(), knots.last()) else {
        return Err(bad("no knots"));
    };
    if b.partial_cmp(&a) != Some(core::cmp::Ordering::Greater) {
        return Err(bad("empty knot domain"));
    }
    let lo = knots.iter().take(p + 1).all(|&k| k == a);
    let hi = knots.iter().rev().take(p + 1).all(|&k| k == b);
    if !lo || !hi {
        return Err(bad("knots not clamped"));
    }
    Ok(())
}

fn points(rd: &mut Reader<'_>, count: usize, dim2: bool) -> Result<(), FfiError> {
    for _ in 0..count {
        rd.finite()?;
        rd.finite()?;
        if dim2 {
            // a 2D control point has z = NOZ
            if rd.num()? != NOZ {
                return Err(bad("2D control point z must be NOZ"));
            }
        } else {
            rd.finite()?;
        }
        let w = rd.num()?;
        if w != NOW && !(w.is_finite() && w > 0.0) {
            return Err(bad("weight must be NOW or positive"));
        }
    }
    Ok(())
}

fn curve(rd: &mut Reader<'_>) -> Result<(), FfiError> {
    let dim = rd.index(3)?;
    if dim < 2 {
        return Err(bad("curve dimension 2 or 3"));
    }
    let n = rd.index(MAX_INDEX)?;
    let p = rd.index(MAX_DEGREE)?;
    let m = rd.index(MAX_INDEX + MAX_DEGREE + 1)?;
    if p == 0 || p > n || m != n + p + 1 {
        return Err(bad("curve: need 1 <= p <= n, m = n+p+1"));
    }
    points(rd, n + 1, dim == 2)?;
    let knots = (0..=m)
        .map(|_| rd.finite())
        .collect::<Result<Vec<_>, _>>()?;
    check_knots(&knots, p)
}

fn surface(rd: &mut Reader<'_>) -> Result<(), FfiError> {
    if rd.index(1)? == 1 {
        for _ in 0..3 {
            rd.finite()?;
        }
        let n = [rd.finite()?, rd.finite()?, rd.finite()?];
        if n.iter().map(|c| c * c).sum::<f64>() < 1e-12 {
            return Err(bad("zero plane normal"));
        }
        return Ok(());
    }
    rd.flag()?;
    rd.index(11)?;
    let n = rd.index(MAX_INDEX)?;
    let m = rd.index(MAX_INDEX)?;
    let p = rd.index(MAX_DEGREE)?;
    let q = rd.index(MAX_DEGREE)?;
    let r = rd.index(MAX_INDEX + MAX_DEGREE + 1)?;
    let s = rd.index(MAX_INDEX + MAX_DEGREE + 1)?;
    if p == 0 || p > n || q == 0 || q > m || r != n + p + 1 || s != m + q + 1 {
        return Err(bad(
            "surface: need 1 <= p <= n, 1 <= q <= m, r = n+p+1, s = m+q+1",
        ));
    }
    points(rd, (n + 1) * (m + 1), false)?;
    let u = (0..=r)
        .map(|_| rd.finite())
        .collect::<Result<Vec<_>, _>>()?;
    let v = (0..=s)
        .map(|_| rd.finite())
        .collect::<Result<Vec<_>, _>>()?;
    check_knots(&u, p)?;
    check_knots(&v, q)
}

/// The arguments of [`eval`] for [`Op::Read`]: the bytes are an unmodified upstream test
/// file (`SmBrep::ReadFromFile` is run on them whatever the stream guard of the B-rep
/// data simulation stops at).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReadOptions {
    /// The bytes are an unmodified upstream test file.
    pub upstream_file: bool,
}

/// Runs `op` on `input` (see the module docs); `path` is the `.smb` file of a file
/// B-rep block (ignored otherwise).
///
/// # Errors
///
/// [`FfiError::InvalidArgument`] for a malformed input or an index outside the B-rep;
/// [`FfiError::Status`] for a shim failure.
pub fn eval(op: Op, path: &str, input: &[f64], read: ReadOptions) -> Result<Vec<f64>, FfiError> {
    let mut rd = Reader { input, at: 0 };
    match op {
        Op::Read => {
            let n = rd.index(MAX_BYTES)?;
            let mut bytes = Vec::new();
            for _ in 0..n {
                bytes.push(u8::try_from(rd.index(255)?).map_err(|_| bad("byte"))?);
            }
            let binary = rd.flag()?;
            rd.flag()?;
            rd.done()?;
            // the B-rep data guard (and, for a modified stream, its read status)
            let req = bd::Request {
                mode: bd::Mode::ReadFile,
                binary,
                version: 0,
                dim: 0,
                persist: false,
                source: 0,
                pre: 0,
                data: bytes,
                upstream_file: read.upstream_file,
            };
            let items = bd::run(&req)?;
            if !read.upstream_file && items.first() == Some(&bd::Item::Int(0)) {
                return Ok(vec![NOT_RUN]);
            }
        }
        Op::MakeFaceWithCurves => {
            surface(&mut rd)?;
            for _ in 0..4 {
                rd.finite()?;
            }
            rd.orient()?;
            rd.flag()?;
            rd.index(2)?;
            let nl = rd.index(MAX_CURVES)?;
            let mut total = 0usize;
            for _ in 0..nl {
                total += rd.index(MAX_CURVES)?;
            }
            let has3d = rd.flag()?;
            let hasuv = rd.flag()?;
            let nc = rd.index(MAX_CURVES)?;
            // (without curve arrays upstream fails before reading the counts)
            // (upstream reads only the first `total` curves: extra ones are unused)
            if (has3d || hasuv) && total > nc {
                return Err(bad("loop counts exceed the curves"));
            }
            for _ in 0..nc {
                if has3d {
                    curve(&mut rd)?;
                }
                if hasuv {
                    curve(&mut rd)?;
                }
                rd.orient()?;
            }
            let np = rd.index(MAX_CURVES)?;
            for _ in 0..(3 * np) {
                rd.finite()?;
            }
            rd.done()?;
        }
        Op::MakeFace => {
            rd.index(1)?;
            // (the shim builds the square's lines: they need distinct ends)
            let side = rd.finite()?;
            if side.partial_cmp(&0.0) != Some(core::cmp::Ordering::Greater) {
                return Err(bad("square side must be positive"));
            }
            rd.orient()?;
            rd.flag()?;
            rd.done()?;
        }
        Op::Glue | Op::Copy | Op::MergeSelf => {
            // the topology.vertexuse wrapper validates the block itself and exports
            let (block, args) = split_block(op, input)?;
            let export = fixture::eval(fixture::Op::Export, path, block)?;
            let Some(c) = fixture::counts(&export) else {
                return Ok(export);
            };
            let [_, _, _, n_f, _, _, _, n_e, ..] = c;
            let mut ar = Reader { input: args, at: 0 };
            match op {
                Op::Glue => {
                    for _ in 0..2 {
                        if let Some(i) = ar.signed(MAX_INDEX << 20)? {
                            if i >= n_e {
                                return Err(bad("edge index out of range"));
                            }
                        }
                    }
                    ar.orient()?;
                    ar.finite()?;
                    ar.flag()?;
                }
                Op::Copy => {
                    ar.index(1 << 20)?;
                    ar.flag()?;
                }
                _ => {
                    if ar.index(1 << 20)? >= n_f {
                        return Err(bad("face index out of range"));
                    }
                    ar.finite()?;
                }
            }
            ar.done()?;
        }
    }
    raw(op, path, input)
}

/// The number of op arguments before the B-rep block of a B-rep op.
const fn n_args(op: Op) -> usize {
    match op {
        Op::Glue => 5,
        Op::Copy | Op::MergeSelf => 2,
        _ => 0,
    }
}

/// Splits a B-rep op's input into its arguments (first) and the B-rep block.
fn split_block(op: Op, input: &[f64]) -> Result<(&[f64], &[f64]), FfiError> {
    let n = n_args(op);
    Ok((
        input.get(n..).ok_or_else(|| bad("input too short"))?,
        input.get(..n).ok_or_else(|| bad("input too short"))?,
    ))
}

fn raw(op: Op, path: &str, input: &[f64]) -> Result<Vec<f64>, FfiError> {
    let cpath = std::ffi::CString::new(path).map_err(|_| bad("path has a NUL byte"))?;
    call_with_buffer(256, |buf, len| {
        // SAFETY: cpath is a valid NUL-terminated string; input is valid for
        // input.len() reads and holds what the shim reads for `op` (validated by
        // `eval`); buf is valid for buf.len() writes and the shim honours the capacity;
        // len is valid.
        unsafe {
            sys::sc_topology_brep_p7_eval(
                op as i32,
                cpath.as_ptr(),
                input.as_ptr(),
                input.len() as i64,
                buf.as_mut_ptr(),
                buf.len() as i64,
                &raw mut *len,
            )
        }
    })
}
