//! Sequential reader over a flat `&[f64]` input record, shared by the unit
//! wrappers' `validate` functions.
//!
//! Each unit declares its own `Reader` with [`unit_reader!`]: a newtype over
//! [`Cursor`] that derefs to it. Methods a unit defines on its `Reader` take
//! precedence over the [`Cursor`] ones, so a unit with a stricter or looser
//! check keeps it unchanged.

use crate::FfiError;

fn bad(msg: impl Into<String>) -> FfiError {
    FfiError::InvalidArgument(msg.into())
}

/// Position in a flat input record, plus the coordinate bound of
/// [`Cursor::finite`].
pub(crate) struct Cursor<'a> {
    /// The whole record.
    pub(crate) input: &'a [f64],
    /// Index of the next value to read.
    pub(crate) at: usize,
    max_coord: f64,
}

impl<'a> Cursor<'a> {
    /// A cursor at the start of `input`; [`Cursor::finite`] accepts magnitudes up to
    /// `max_coord`.
    pub(crate) fn new(input: &'a [f64], max_coord: f64) -> Self {
        Self {
            input,
            at: 0,
            max_coord,
        }
    }

    /// The next value.
    pub(crate) fn num(&mut self) -> Result<f64, FfiError> {
        let v = self
            .input
            .get(self.at)
            .copied()
            .ok_or_else(|| bad("input too short"))?;
        self.at += 1;
        Ok(v)
    }

    /// The next `k` values.
    pub(crate) fn take(&mut self, k: usize) -> Result<&'a [f64], FfiError> {
        let end = self.at.checked_add(k).ok_or_else(|| bad("overflow"))?;
        let s = self
            .input
            .get(self.at..end)
            .ok_or_else(|| bad("input too short"))?;
        self.at = end;
        Ok(s)
    }

    /// The next `k` values, all finite.
    pub(crate) fn slice(&mut self, k: usize) -> Result<&'a [f64], FfiError> {
        let end = self.at.checked_add(k).ok_or_else(|| bad("overflow"))?;
        let s = self
            .input
            .get(self.at..end)
            .ok_or_else(|| bad("input too short"))?;
        if s.iter().any(|x| !x.is_finite()) {
            return Err(bad("non-finite input"));
        }
        self.at = end;
        Ok(s)
    }

    /// Skips the next `k` values.
    pub(crate) fn skip(&mut self, k: usize) -> Result<(), FfiError> {
        let end = self.at.checked_add(k).ok_or_else(|| bad("overflow"))?;
        if end > self.input.len() {
            return Err(bad("input too short"));
        }
        self.at = end;
        Ok(())
    }

    /// Fails unless the whole record was read.
    pub(crate) fn done(&self) -> Result<(), FfiError> {
        if self.at != self.input.len() {
            return Err(bad("trailing input"));
        }
        Ok(())
    }

    /// The next value as an integer in `0..=max`.
    pub(crate) fn index(&mut self, max: usize) -> Result<usize, FfiError> {
        let v = self.num()?;
        if v.fract() != 0.0 || v < 0.0 || v > max as f64 {
            return Err(bad(format!("bad index/count {v}")));
        }
        Ok(v as usize)
    }

    /// The next value as a flag: `0` or `1`.
    pub(crate) fn flag(&mut self) -> Result<bool, FfiError> {
        Ok(self.index(1)? == 1)
    }

    /// The next value, which must be one of `allowed`.
    pub(crate) fn choice(&mut self, allowed: &[f64]) -> Result<f64, FfiError> {
        let v = self.num()?;
        if !allowed.contains(&v) {
            return Err(bad(format!("bad flag {v}")));
        }
        Ok(v)
    }

    /// The next value, finite and of magnitude at most the unit's `MAX_COORD`.
    pub(crate) fn finite(&mut self) -> Result<f64, FfiError> {
        let v = self.num()?;
        if !v.is_finite() || v.abs() > self.max_coord {
            return Err(bad(format!("value {v} not finite or too large")));
        }
        Ok(v)
    }

    /// The next `k` values, each as [`Cursor::finite`].
    pub(crate) fn finites(&mut self, k: usize) -> Result<Vec<f64>, FfiError> {
        (0..k).map(|_| self.finite()).collect()
    }

    /// The next three values, each as [`Cursor::finite`].
    pub(crate) fn vec3(&mut self) -> Result<[f64; 3], FfiError> {
        Ok([self.finite()?, self.finite()?, self.finite()?])
    }

    /// [`Cursor::vec3`] with squared length at least `1e-12`.
    pub(crate) fn nonzero3(&mut self) -> Result<[f64; 3], FfiError> {
        let v = self.vec3()?;
        if v.iter().map(|c| c * c).sum::<f64>() < 1e-12 {
            return Err(bad("zero vector"));
        }
        Ok(v)
    }
}

/// Declares the unit's `Reader`: a newtype over [`Cursor`] built with
/// `Reader::new(input)`. Pass the unit's `MAX_COORD` when it reads coordinates
/// through [`Cursor::finite`].
macro_rules! unit_reader {
    () => {
        $crate::reader::unit_reader!(f64::INFINITY);
    };
    ($max_coord:expr) => {
        /// Sequential reader over the input record (see [`crate::reader::Cursor`]).
        struct Reader<'a>(crate::reader::Cursor<'a>);

        impl<'a> Reader<'a> {
            fn new(input: &'a [f64]) -> Self {
                Self(crate::reader::Cursor::new(input, $max_coord))
            }
        }

        impl<'a> ::core::ops::Deref for Reader<'a> {
            type Target = crate::reader::Cursor<'a>;
            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }

        impl ::core::ops::DerefMut for Reader<'_> {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.0
            }
        }
    };
}
pub(crate) use unit_reader;
