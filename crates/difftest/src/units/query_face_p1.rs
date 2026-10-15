//! Differential runner for `query.face_p1`.
//! The curve API subset compares its status and all seeded optional outputs.
//! Other owned face/shell/region queries are still being ported; no completion
//! claim is made from this subset.

use crate::case::CaseContext;
use crate::runner::default_cases;
use crate::{Case, HarnessError, Output, Rng, UnitDiff};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use solidcore::curve::curve::Curve;
use solidcore::curve::hermite_curve::HermiteCurve;
use solidcore::curve::line::Line;
use solidcore::geom::vector3d::Vec3;
use solidcore::query::face::p1;
use solidcore::SolidError;
use solidcore_ffi::units::query_face_p1 as ffi;

pub struct Runner;

#[derive(Serialize, Deserialize)]
#[serde(untagged)]
enum Parameter {
    Number(f64),
    Special(String),
}

impl Parameter {
    fn value(&self) -> Result<f64, HarnessError> {
        match self {
            Self::Number(x) => Ok(*x),
            Self::Special(x) if x == "NaN" => Ok(f64::NAN),
            Self::Special(x) if x == "+inf" => Ok(f64::INFINITY),
            Self::Special(x) if x == "-inf" => Ok(f64::NEG_INFINITY),
            _ => Err(HarnessError::new("unknown special parameter")),
        }
    }
}

#[derive(Serialize, Deserialize)]
struct Input {
    kind: i32,
    data: [f64; 12],
    parameter: Parameter,
    mask: i32,
}

fn output(status: i32, values: [f64; 9]) -> Output {
    Output::new()
        .int("status", i64::from(status))
        .values("out", values.to_vec())
}

impl UnitDiff for Runner {
    fn cases(&self, ctx: &CaseContext) -> Result<Vec<Case>, HarnessError> {
        default_cases(
            self,
            &CaseContext {
                seeds: ctx.seeds.max(600),
                ..ctx.clone()
            },
        )
    }

    fn id(&self) -> &'static str {
        "query.face_p1"
    }

    fn run_cpp(&self, case: &Case) -> Result<Output, HarnessError> {
        let input: Input = case.parse()?;
        let (status, values) = ffi::evaluate_curve(
            input.kind,
            &input.data,
            input.parameter.value()?,
            input.mask,
        )
        .map_err(|e| HarnessError::new(e.to_string()))?;
        Ok(output(status, values))
    }

    fn run_rust(&self, case: &Case) -> Result<Output, HarnessError> {
        let input: Input = case.parse()?;
        let v = |i: usize| Vec3::new(input.data[i], input.data[i + 1], input.data[i + 2]);
        let curve: Option<Curve> = match input.kind {
            0 => None,
            1 => Some(Line::from_points(v(0), v(6), 3).into()),
            2 => Some(HermiteCurve::new(v(0), v(3), v(6), v(9), 3).into()),
            _ => return Err(HarnessError::new("unknown curve kind")),
        };
        let mut values = [11., 13., 17., 19., 23., 29., 31., 37., 41.];
        match p1::evaluate_curve(curve.as_ref(), input.parameter.value()?) {
            Ok(result) => {
                for (i, vector) in result.iter().enumerate() {
                    if input.mask & (1 << i) != 0 {
                        values[i * 3..i * 3 + 3].copy_from_slice(&[vector.x, vector.y, vector.z]);
                    }
                }
                Ok(output(0, values))
            }
            Err(SolidError::InvalidInput { .. }) => Ok(output(1007, values)),
            Err(SolidError::Upstream { code, .. }) => Ok(output(code, values)),
            Err(e) => Ok(Output::unported(&e).unwrap_or_else(|| Output::err(e.to_string()))),
        }
    }

    fn generate(&self, rng: &mut Rng) -> Option<Value> {
        let scale = match rng.below(5) {
            0 => 1e-200,
            1 => 1e-6,
            2 => 1.0,
            3 => 1e6,
            _ => 1e100,
        };
        let mut data = [0.0; 12];
        for item in &mut data {
            *item = rng.uniform(-100.0, 100.0) * scale;
        }
        serde_json::to_value(Input {
            kind: 1 + rng.below(2) as i32,
            data,
            parameter: Parameter::Number(rng.unit()),
            mask: 1 + rng.below(7) as i32,
        })
        .ok()
    }
}
