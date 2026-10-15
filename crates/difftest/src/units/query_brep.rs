//! query.brep: actual `SM_API` evaluation entry points, status and failure atomicity.
//! Surface APIs, decimation attributes and B-rep query batches.
use super::surface_surface_cache_p1::{push3, status};
use super::surface_surface_cache_p3::{gen_surf, push_surf, surf_domain, surface_in, SurfIn};
use crate::case::CaseContext;
use crate::runner::default_cases;
use crate::{Case, HarnessError, Output, Rng, UnitDiff};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use solidcore::geom::vector2d::Vec2;
use solidcore::query::brep;
use solidcore_ffi::units::query_brep as ffi;

/// Native surface API comparison for `query.brep`.
pub struct Runner;
#[derive(Clone, Serialize, Deserialize)]
struct Inputs {
    surf: Option<SurfIn>,
    op: u8,
    uv: [f64; 2],
    /// 0 ordinary; 1/2 NaN u/v; 3/4 infinite u/v. JSON cannot encode NaN.
    #[serde(default)]
    special: u8,
    initial: [[f64; 3]; 3],
}
fn uv(i: &Inputs) -> [f64; 2] {
    let mut uv = i.uv;
    match i.special {
        1 => uv[0] = f64::NAN,
        2 => uv[1] = f64::NAN,
        3 => uv[0] = f64::INFINITY,
        4 => uv[1] = f64::NEG_INFINITY,
        _ => {}
    }
    uv
}
fn flat(i: &Inputs) -> Result<Vec<f64>, HarnessError> {
    let mut out = vec![if i.surf.is_some() { 1.0 } else { 0.0 }];
    if let Some(s) = &i.surf {
        push_surf(&mut out, s)?;
    }
    out.extend(uv(i));
    out.extend(i.initial.iter().flatten());
    Ok(out)
}
fn record(out: Vec<f64>) -> Output {
    Output::new().values("status_and_outputs", out)
}
impl UnitDiff for Runner {
    fn id(&self) -> &'static str {
        "query.brep"
    }
    fn run_cpp(&self, case: &Case) -> Result<Output, HarnessError> {
        if case.inputs.get("bbox_brep").is_some() {
            return bbox_cpp(&case.parse()?);
        }
        if case.inputs.get("point_classify").is_some() {
            let i: PointInput = case.parse()?;
            return ffi::classify(&i.point_classify, i.tolerance, i.boundary, i.regions)
                .map(record)
                .map_err(|e| HarnessError::new(e.to_string()));
        }
        if case.inputs.get("attribute_kind").is_some() {
            return attribute_cpp(&case.parse()?);
        }
        let i: Inputs = case.parse()?;
        let out = ffi::surface(i.op, &flat(&i)?).map_err(|e| HarnessError::new(e.to_string()))?;
        Ok(record(out))
    }
    fn run_rust(&self, case: &Case) -> Result<Output, HarnessError> {
        if case.inputs.get("bbox_brep").is_some() {
            return bbox_rust(&case.parse()?);
        }
        if case.inputs.get("point_classify").is_some() {
            return point_rust(&case.parse()?);
        }
        if case.inputs.get("attribute_kind").is_some() {
            return attribute_rust(&case.parse()?);
        }
        let i: Inputs = case.parse()?;
        let s = i.surf.as_ref().map(surface_in).transpose()?;
        let [u, v] = uv(&i);
        let uv = Vec2::new(u, v);
        let mut result_status = 0.0;
        let mut outputs: Vec<f64> = i.initial.iter().flatten().copied().collect();
        let evaluated = match i.op {
            0 => brep::evaluate_surface_point(s.as_ref(), uv).map(|p| {
                let mut out = Vec::new();
                push3(&mut out, p);
                out
            }),
            1 => brep::evaluate_surface_normal(s.as_ref(), uv).map(|p| {
                let mut out = Vec::new();
                push3(&mut out, p);
                out
            }),
            2 => brep::evaluate_surface_derivatives(s.as_ref(), uv).map(|(p, du, dv)| {
                let mut out = Vec::new();
                push3(&mut out, p);
                push3(&mut out, du);
                push3(&mut out, dv);
                out
            }),
            _ => return Err(HarnessError::new("surface operation")),
        };
        match evaluated {
            Ok(v) => {
                for (target, value) in outputs.iter_mut().zip(v) {
                    *target = value;
                }
            }
            Err(e) => {
                if let Some(deferred) = Output::unported(&e) {
                    return Ok(deferred);
                }
                result_status = status(&e);
            }
        }
        let mut result = vec![result_status];
        result.extend(outputs);
        Ok(record(result))
    }
    fn generate(&self, rng: &mut Rng) -> Option<Value> {
        for _ in 0..100 {
            let surf = gen_surf(rng);
            let [a, b, c, d] = surf_domain(&surf).ok()?;
            let fraction = match rng.below(5) {
                0 => 0.0,
                1 => 1.0,
                _ => rng.unit(),
            };
            let mut uv = [a + fraction * (c - a), b + rng.unit() * (d - b)];
            if rng.below(8) == 0 {
                uv[0] = a - (c - a);
            }
            let i = Inputs {
                surf: Some(surf),
                op: rng.below(3) as u8,
                uv,
                special: if rng.below(10) == 0 {
                    1 + rng.below(4) as u8
                } else {
                    0
                },
                initial: [
                    [17.0, -29.0, 41.0],
                    [53.0, -67.0, 79.0],
                    [83.0, -97.0, 101.0],
                ],
            };
            if ffi::validate(i.op, &flat(&i).ok()?).is_ok() {
                return serde_json::to_value(i).ok();
            }
        }
        None
    }
    fn cases(&self, ctx: &CaseContext) -> Result<Vec<Case>, HarnessError> {
        let mut cases = default_cases(
            self,
            &CaseContext {
                seeds: ctx.seeds.max(600),
                ..ctx.clone()
            },
        )?;
        // Preserve all 600 existing API seeds and add a separate reproducible
        // B-rep box stream; these cases never replace a previous regression.
        for index in 0..ctx.seeds.max(96) {
            let seed = crate::rng::fnv1a64("query.brep.bounding_boxes").wrapping_add(index as u64);
            let input = generate_bbox(&mut Rng::new(seed));
            cases.push(Case {
                unit: self.id().into(),
                name: format!("bbox_gen_{index:04}"),
                source: crate::case::CaseSource::Generated { seed },
                inputs: serde_json::to_value(input)
                    .map_err(|e| HarnessError::new(e.to_string()))?,
            });
        }
        Ok(cases)
    }
}

#[derive(Serialize, Deserialize)]
struct AttributeInput {
    attribute_kind: u8,
    #[serde(default)]
    special: u8,
    id: u64,
    behavior: u8,
    mutate: bool,
    sort: u64,
    class: u8,
    data: [f64; 13],
}
fn attr_record(values: Vec<f64>, integers: [u64; 8]) -> Output {
    // Preserve all 64 identity bits without routing them through a double or
    // overflowing difftest's signed integer representation.
    let integers: Vec<i64> = integers
        .into_iter()
        .flat_map(|v| [i64::from(v as u32), i64::from((v >> 32) as u32)])
        .collect();
    Output::new()
        .values("payloads", values)
        .ints("integer_words", integers)
}
fn attribute_data(i: &AttributeInput) -> [f64; 13] {
    let mut d = i.data;
    match i.special {
        1 => d[0] = f64::NAN,
        2 => d[0] = f64::INFINITY,
        3 => d[0] = f64::NEG_INFINITY,
        _ => {}
    }
    d
}
fn attribute_cpp(i: &AttributeInput) -> Result<Output, HarnessError> {
    let record = solidcore_ffi::units::query_brep_attributes::eval(
        i.attribute_kind,
        i.id,
        i.behavior,
        i.mutate,
        i.sort,
        i.class,
        &attribute_data(i),
    )
    .map_err(|e| HarnessError::new(e.to_string()))?;
    Ok(attr_record(record.values, record.integers))
}
fn attr_payload(a: &solidcore::geom::attribute::Attribute) -> Vec<f64> {
    use solidcore::geom::attribute::AttributeValue;
    let mut out = Vec::new();
    match a.value() {
        AttributeValue::DecimateVertex(v) => {
            out.extend([v.vertex_distance, v.max_face_error]);
            push3(&mut out, v.average_plane_normal);
        }
        AttributeValue::QuadricVertex(v) => {
            for row in v.matrix {
                push3(&mut out, row);
            }
            push3(&mut out, v.linear);
            out.push(v.constant);
        }
        AttributeValue::QuadricEdge(v) => {
            push3(&mut out, v.new_point);
            out.push(v.error);
        }
        _ => {}
    }
    out
}
fn attr_order(a: &solidcore::geom::attribute::Attribute) -> (u64, u64) {
    use solidcore::geom::attribute::AttributeValue;
    match a.value() {
        AttributeValue::DecimateVertex(v) => (v.sort_index, v.vertex_class as u64),
        AttributeValue::QuadricEdge(v) => (v.sort_index.unwrap_or(0), 0),
        _ => (0, 0),
    }
}
fn attribute_rust(i: &AttributeInput) -> Result<Output, HarnessError> {
    use solidcore::geom::attribute::{AttributeValue, Behavior};
    use solidcore::geom::vector3d::Vec3;
    use solidcore::query::brep::PolyVertexClass;
    let behavior =
        Behavior::from_code(i32::from(i.behavior)).map_err(|e| HarnessError::new(e.to_string()))?;
    let mut a = match i.attribute_kind {
        0 => brep::decimate_vertex_attribute(i.id, behavior),
        1 => brep::quadric_vertex_attribute(i.id, behavior),
        2 => brep::quadric_edge_attribute(i.id, behavior),
        _ => return Err(HarnessError::new("attribute kind")),
    };
    let mut values = attr_payload(&a);
    let d = attribute_data(i);
    if i.mutate {
        match a.value_mut() {
            AttributeValue::DecimateVertex(v) => {
                v.sort_index = i.sort;
                v.vertex_class = match i.class {
                    0 => PolyVertexClass::NotComputed,
                    1 => PolyVertexClass::Unknown,
                    2 => PolyVertexClass::Simple,
                    3 => PolyVertexClass::Boundary,
                    4 => PolyVertexClass::Interior,
                    5 => PolyVertexClass::Corner,
                    _ => return Err(HarnessError::new("vertex class")),
                };
                v.vertex_distance = d[0];
                v.max_face_error = d[1];
                v.average_plane_normal = Vec3::new(d[2], d[3], d[4]);
            }
            AttributeValue::QuadricVertex(v) => {
                v.matrix = [
                    Vec3::new(d[0], d[1], d[2]),
                    Vec3::new(d[3], d[4], d[5]),
                    Vec3::new(d[6], d[7], d[8]),
                ];
                v.linear = Vec3::new(d[9], d[10], d[11]);
                v.constant = d[12];
            }
            AttributeValue::QuadricEdge(v) => {
                v.new_point = Vec3::new(d[0], d[1], d[2]);
                v.error = d[3];
                v.sort_index = Some(i.sort);
            }
            _ => return Err(HarnessError::new("attribute value")),
        }
    }
    let copy = a.make_copy();
    values.extend(attr_payload(&a));
    values.extend(attr_payload(&copy));
    let (sort, cls) = attr_order(&a);
    let (csort, ccls) = attr_order(&copy);
    Ok(attr_record(
        values,
        [
            a.attribute_id(),
            copy.attribute_id(),
            a.behavior().code() as u64,
            copy.behavior().code() as u64,
            sort,
            csort,
            cls,
            ccls,
        ],
    ))
}

fn one_region() -> u8 {
    1
}
#[derive(Serialize, Deserialize)]
struct PointInput {
    #[serde(default = "one_region")]
    regions: u8,
    point_classify: [f64; 3],
    tolerance: f64,
    boundary: bool,
}
#[derive(Clone, Copy, PartialEq)]
enum EmptyObject {
    Brep,
    Region(u8),
}
impl solidcore::geom::solution_array::SolutionObject for EmptyObject {
    fn kind(&self) -> solidcore::geom::solution_array::ObjectKind {
        solidcore::geom::solution_array::ObjectKind::Other
    }
}
impl solidcore::intersect::point_class::ClassObject for EmptyObject {
    fn class_kind(&self) -> solidcore::intersect::point_class::ClassObjectKind {
        use solidcore::intersect::point_class::ClassObjectKind;
        match self {
            Self::Brep => ClassObjectKind::Other,
            Self::Region(_) => ClassObjectKind::Region,
        }
    }
}
// Only the B-rep and its real constructor region are represented. There are no
// curves, surfaces or vertices; requesting one is a harness/model error, never
// a fabricated geometry result. The standard missing-cache default is retained.
struct EmptyBrep {
    regions: u8,
}
fn wrong_empty_object() -> solidcore::SolidError {
    solidcore::SolidError::invalid_input(
        "difftest::query_brep::EmptyBrep",
        "object is not geometric",
    )
}
impl solidcore::intersect::topology_solver::p1::TopologyObjects<EmptyObject> for EmptyBrep {
    fn curve(&self, _: EmptyObject) -> solidcore::Result<&solidcore::curve::curve::Curve> {
        Err(wrong_empty_object())
    }
    fn analytic_curve_handle(&self, _: EmptyObject) -> Option<EmptyObject> {
        None
    }
    fn vertex_point(&self, _: EmptyObject) -> solidcore::Result<solidcore::math::Point3> {
        Err(wrong_empty_object())
    }
    fn vertex_tolerance(&self, _: EmptyObject) -> solidcore::Result<f64> {
        Err(wrong_empty_object())
    }
    fn curve_edge(&self, _: EmptyObject) -> solidcore::Result<Option<EmptyObject>> {
        Err(wrong_empty_object())
    }
    fn edge_curve(&self, _: EmptyObject) -> solidcore::Result<Option<EmptyObject>> {
        Err(wrong_empty_object())
    }
    fn edge_interval(
        &self,
        _: EmptyObject,
    ) -> solidcore::Result<solidcore::geom::extent1d::Extent1d> {
        Err(wrong_empty_object())
    }
    fn surface_face(&self, _: EmptyObject) -> solidcore::Result<Option<EmptyObject>> {
        Err(wrong_empty_object())
    }
    fn face_surface(&self, _: EmptyObject) -> solidcore::Result<EmptyObject> {
        Err(wrong_empty_object())
    }
    fn surface(&self, _: EmptyObject) -> solidcore::Result<&solidcore::surface::surface::Surface> {
        Err(wrong_empty_object())
    }
    fn face_uv_domain(
        &self,
        _: EmptyObject,
    ) -> solidcore::Result<solidcore::geom::extent2d::Extent2d> {
        Err(wrong_empty_object())
    }
    fn face_tolerance(&self, _: EmptyObject) -> solidcore::Result<f64> {
        Err(wrong_empty_object())
    }
    fn surface_owner(&self, _: EmptyObject) -> solidcore::Result<Option<EmptyObject>> {
        Err(wrong_empty_object())
    }
    fn shape_member(&self, _: EmptyObject) -> solidcore::geom::object::ShapeMember {
        solidcore::geom::object::ShapeMember::Other
    }
}
impl brep::BrepClassifyObjects<EmptyObject> for EmptyBrep {
    fn brep_object(&self) -> EmptyObject {
        EmptyObject::Brep
    }
    fn regions(&self) -> solidcore::Result<Vec<EmptyObject>> {
        Ok((0..self.regions).map(EmptyObject::Region).collect())
    }
}
impl solidcore::intersect::point_class::ClassGeometry<EmptyObject> for EmptyBrep {
    fn curve(&self, _: EmptyObject) -> Option<&solidcore::curve::curve::Curve> {
        None
    }
    fn surface(&self, _: EmptyObject) -> Option<&solidcore::surface::surface::Surface> {
        None
    }
    fn volume(&self, _: EmptyObject) -> Option<&solidcore::surface::volume::Volume> {
        None
    }
    fn point(&self, _: EmptyObject) -> Option<solidcore::math::Point3> {
        None
    }
}
fn point_rust(i: &PointInput) -> Result<Output, HarnessError> {
    use solidcore::geom::vector3d::Vec3;
    use solidcore::intersect::point_class::PointClassification;
    let k = solidcore::kernel::Kernel::default();
    let scene = EmptyBrep { regions: i.regions };
    let mut pc = PointClassification::new(17.0);
    pc.set_gap_3d(31.0);
    pc.set_t_param(47.0, false);
    pc.set_uv_param(Vec2::new(53.0, 59.0));
    pc.set_pre_snap_param(61.0);
    pc.set_sss_point(Vec3::new(67.0, 71.0, 73.0));
    let tolerance = solidcore::geom::tol::zone_tol3d(&k, i.tolerance);
    let [x, y, z] = i.point_classify;
    let outcome = brep::point_3d_classify(
        &k,
        &scene,
        Vec3::new(x, y, z),
        tolerance,
        i.boundary,
        &mut pc,
    );
    let status = match outcome {
        Ok(()) => 0.0,
        Err(e) => {
            if let Some(deferred) = Output::unported(&e) {
                return Ok(deferred);
            }
            status(&e)
        }
    };
    let gap = pc
        .gap_3d(&k, &scene, None)
        .map_err(|e| HarnessError::new(e.to_string()))?;
    let uv = pc.uv_param();
    let sss = pc.sss_point();
    Ok(record(vec![
        status,
        f64::from(pc.point_class().code()),
        if pc.object() == Some(EmptyObject::Region(0)) {
            0.0
        } else {
            -1.0
        },
        gap,
        pc.t_param(),
        uv.x,
        uv.y,
        f64::from(u8::from(pc.are_parameters_set())),
        pc.pre_snap_param(),
        f64::from(u8::from(pc.sss_point_flag())),
        sss.x,
        sss.y,
        sss.z,
        pc.src_zone_tol_3d(),
    ]))
}

#[derive(Clone, Serialize, Deserialize)]
struct BboxInput {
    bbox_brep: Option<super::topology_edgeuse_p1::BrepIn>,
    #[serde(default)]
    extra_vertices: usize,
    #[serde(default)]
    extra_edges: usize,
    tolerances: [f64; 3],
    #[serde(default)]
    drop_wire_curves: bool,
    steps: Vec<BoxStep>,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum BoxStep {
    Bounding {
        approximate: bool,
        tight: bool,
        expand: bool,
    },
    Tight,
    Lock {
        tight: bool,
    },
    Unlock,
    Default,
}
fn bbox_input(i: &BboxInput) -> Result<(String, Vec<f64>), HarnessError> {
    use super::topology_edgeuse_p1::{brep_block_of, path_of};
    let path = path_of(i.bbox_brep.as_ref())?;
    let mut flat = vec![f64::from(u8::from(i.bbox_brep.is_some()))];
    if i.bbox_brep.is_some() {
        flat.extend(brep_block_of(i.bbox_brep.as_ref(), false, false)?);
    }
    flat.extend([i.extra_vertices as f64, i.extra_edges as f64]);
    flat.extend(i.tolerances);
    flat.push(f64::from(u8::from(i.drop_wire_curves)));
    flat.push(i.steps.len() as f64);
    for step in &i.steps {
        let row = match *step {
            BoxStep::Bounding {
                approximate,
                tight,
                expand,
            } => [
                0.0,
                f64::from(u8::from(approximate)),
                f64::from(u8::from(tight)),
                f64::from(u8::from(expand)),
            ],
            BoxStep::Tight => [1.0, 0.0, 0.0, 0.0],
            BoxStep::Lock { tight } => [2.0, 0.0, f64::from(u8::from(tight)), 0.0],
            BoxStep::Unlock => [3.0, 0.0, 0.0, 0.0],
            BoxStep::Default => [4.0, 0.0, 0.0, 0.0],
        };
        flat.extend(row);
    }
    Ok((path, flat))
}
fn bbox_cpp(i: &BboxInput) -> Result<Output, HarnessError> {
    let (path, flat) = bbox_input(i)?;
    let record = solidcore_ffi::units::query_brep_bbox::eval(false, &path, &flat)
        .map_err(|e| HarnessError::new(e.to_string()))?;
    Ok(Output::new().values("counts_and_boxes", record))
}
fn bbox_rust(i: &BboxInput) -> Result<Output, HarnessError> {
    use solidcore::curve::curve::p2::{BSPLINE_CURVE_TYPE, LINE_TYPE};
    use solidcore::geom::extent3d::Extent3d;
    use solidcore::geom::vector3d::Vec3;
    let (path, flat) = bbox_input(i)?;
    let export = solidcore_ffi::units::query_brep_bbox::eval(true, &path, &flat)
        .map_err(|e| HarnessError::new(e.to_string()))?;
    let (mut graph, types) = super::topology_edgeuse_p1::import_with_types(&export)?;
    // Never compare a curve exported as another kind's B-spline stand-in.
    // Unknown surface payloads are not evaluated: face boxes have real unported
    // callees, and approximate boxes skip faces altogether.
    if types
        .edge_curve_types
        .iter()
        .any(|&t| t != -1 && t != BSPLINE_CURVE_TYPE && t != LINE_TYPE)
    {
        return Err(HarnessError::new(
            "box fixture has an unrepresented native edge curve kind",
        ));
    }
    let k = solidcore::kernel::Kernel::default();
    let mut record = vec![
        graph.vertices().len() as f64,
        graph.edges().len() as f64,
        graph.faces().len() as f64,
    ];
    let initial =
        Extent3d::from_points(Vec3::new(-17.0, -19.0, -23.0), Vec3::new(29.0, 31.0, 37.0));
    for step in &i.steps {
        let result = match *step {
            BoxStep::Bounding {
                approximate,
                tight,
                expand,
            } => brep::calculate_bounding_box(
                &k,
                &graph,
                brep::BoundingBoxOptions {
                    approximate,
                    tight,
                    expand_by_zone_tolerance: expand,
                },
            ),
            BoxStep::Tight => brep::calculate_tight_bounding_box(&k, &graph),
            BoxStep::Lock { tight } => {
                brep::lock_bounding_box(&k, &mut graph, tight).map(|()| initial)
            }
            BoxStep::Unlock => {
                brep::unlock_bounding_box(&mut graph);
                Ok(initial)
            }
            BoxStep::Default => {
                brep::calculate_bounding_box(&k, &graph, brep::BoundingBoxOptions::default())
            }
        };
        let bbox = match result {
            Ok(bbox) => {
                record.push(0.0);
                bbox
            }
            Err(e) => {
                if let Some(deferred) = Output::unported(&e) {
                    return Ok(deferred);
                }
                record.push(status(&e));
                initial
            }
        };
        push3(&mut record, bbox.min());
        push3(&mut record, bbox.max());
    }
    Ok(Output::new().values("counts_and_boxes", record))
}
fn generate_bbox(rng: &mut Rng) -> BboxInput {
    use super::topology_edgeuse_p1::{gen_curve, BrepIn};
    let scales = [1e-6, 1e-3, 1.0, 1e3, 1e6];
    let scale = *rng.choose(&scales).unwrap_or(&1.0);
    let curve = gen_curve(rng, 3, scale);
    let a = curve.u.first().copied().unwrap_or(0.0);
    let b = curve.u.last().copied().unwrap_or(1.0);
    let t0 = a + (b - a) * rng.uniform(0.0, 0.2);
    let t1 = b - (b - a) * rng.uniform(0.0, 0.2);
    let tight = rng.chance(0.25);
    BboxInput {
        bbox_brep: Some(BrepIn::Wire { curve, t0, t1 }),
        extra_vertices: rng.below(4) as usize,
        extra_edges: rng.below(3) as usize,
        tolerances: [rng.unit() * 1e-4, rng.unit() * 1e-3, 0.0],
        drop_wire_curves: rng.chance(0.1),
        steps: vec![
            BoxStep::Default,
            BoxStep::Bounding {
                approximate: rng.chance(0.5),
                tight,
                expand: rng.chance(0.5),
            },
            BoxStep::Lock { tight },
            BoxStep::Bounding {
                approximate: true,
                tight: !tight,
                expand: false,
            },
            BoxStep::Unlock,
            BoxStep::Tight,
        ],
    }
}
