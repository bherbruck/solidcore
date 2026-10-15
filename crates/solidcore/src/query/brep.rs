//! B-rep queries and checked surface evaluation entry points.
//!
//! Upstream: `source/SM_API/src/SmApiSurfaces.cpp`,
//! `source/SMLib/inc/SmPolyDecimate.h`, `source/SMLib/src/SmPolyDecimate.cpp`,
//! `source/SMLib/inc/SmPoly.h`, `source/SMLib/src/SmBrep.cpp`.
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`evaluate_surface_point`] | `SmApiEvaluateSurfacePoint` |
//! | [`evaluate_surface_normal`] | `SmApiEvaluateSurfaceNormal` |
//! | [`evaluate_surface_derivatives`] | `SmApiEvaluateSurfaceDerivatives` |
//! | [`PolyVertexClass`] | `SmPolyVertexClass` |
//! | [`decimate_vertex_attribute`], [`DecimateVertexData`] | `SmDecimateVertexAttr` constructor/destructor |
//! | [`quadric_vertex_attribute`], [`QuadricVertexData`] | `SmQuadricVertexAttr` constructor/destructor |
//! | [`quadric_edge_attribute`], [`QuadricEdgeData`] | `SmQuadricEdgeAttr` constructor/destructor |
//! | [`crate::geom::attribute::Attribute::make_copy`] | the three `MakeCopy` overrides |
//! | [`point_3d_classify`], [`BrepClassifyObjects`] | `SmBrep::Point3DClassify` |
//! | [`calculate_bounding_box`], [`BoundingBoxOptions`] | `SmBrep::CalculateBoundingBox` |
//! | [`calculate_tight_bounding_box`] | `SmBrep::CalculateTightBoundingBox` |
//! | [`lock_bounding_box`] | `SmBrep::LockBoundingBox` |
//! | [`unlock_bounding_box`] | `SmBrep::UnLockBoundingBox` |
//!
//! Deviations: nullable surfaces are `Option<&Surface>`. Surface evaluations and
//! box calculations return their outputs, preserving failure atomicity where
//! upstream does. Classification retains its in/out state, including writes
//! before failure. The locked box is owned by the B-rep. Unfiltered object lists
//! are arena views in list order. No input-driven recursion is added here.
//! Destructors use ownership rather than poisoning fields just before freeing.
//! The quadric edge constructor leaves its sort index uninitialised in C++; the
//! port represents that state by `None` instead of exposing undefined memory.

use crate::geom::extent3d::Extent3d;
use crate::geom::vector2d::Point2;
use crate::geom::vector3d::{Point3, Vec3};
use crate::kernel::Kernel;
use crate::nurbs::knots::Side;
use crate::surface::surface::Surface;
use crate::topology::{Brep, FaceId};
use crate::{Result, SolidError};

fn checked_surface<'a>(
    surface: Option<&'a Surface>,
    uv: Point2,
    op: &'static str,
) -> Result<&'a Surface> {
    let surface = surface.ok_or_else(|| SolidError::invalid_input(op, "null surface"))?;
    let domain = surface.natural_uv_domain();
    let lo = domain.min();
    let hi = domain.max();
    // Keep the negated conjunction: a NaN parameter must fail the API range test.
    if !(uv.x >= lo.x && uv.x <= hi.x && uv.y >= lo.y && uv.y <= hi.y) {
        return Err(SolidError::invalid_input(
            op,
            "parameter outside natural domain",
        ));
    }
    Ok(surface)
}

/// Evaluate a surface point after the API's inclusive natural-domain check.
///
/// # Errors
/// Returns invalid input for a null surface or an out-of-domain (including NaN)
/// parameter, otherwise propagates the surface evaluation error unchanged.
pub fn evaluate_surface_point(surface: Option<&Surface>, uv: Point2) -> Result<Point3> {
    checked_surface(surface, uv, "query::brep::evaluate_surface_point")?.evaluate_point(uv)
}

/// Evaluate the unit normal, with both derivative sides taken from the left.
///
/// # Errors
/// Those of [`evaluate_surface_point`] and the surface's normal evaluation.
pub fn evaluate_surface_normal(surface: Option<&Surface>, uv: Point2) -> Result<Vec3> {
    checked_surface(surface, uv, "query::brep::evaluate_surface_normal")?.evaluate_normal(
        uv,
        Side::Left,
        Side::Left,
    )
}

/// Evaluate `(point, du, dv)`, with both derivative sides taken from the left.
///
/// # Errors
/// Those of [`evaluate_surface_point`] and the surface's first derivatives.
pub fn evaluate_surface_derivatives(
    surface: Option<&Surface>,
    uv: Point2,
) -> Result<(Point3, Vec3, Vec3)> {
    checked_surface(surface, uv, "query::brep::evaluate_surface_derivatives")?
        .evaluate_1st_derivatives(uv, true, true)
}

/// The polygon vertex classification (`SmPolyVertexClass`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PolyVertexClass {
    /// `SM_PVC_NOT_COMPUTED`.
    NotComputed = 0,
    /// `SM_PVC_UNKNOWN`.
    Unknown = 1,
    /// `SM_PVC_SIMPLE`.
    Simple = 2,
    /// `SM_PVC_BOUNDARY`.
    Boundary = 3,
    /// `SM_PVC_INTERIOR`.
    Interior = 4,
    /// `SM_PVC_CORNER`.
    Corner = 5,
}

/// Decimation bookkeeping (`SmDecimateVertexAttr`).
#[derive(Debug, Clone, PartialEq)]
pub struct DecimateVertexData {
    /// `m_lSortIndex`; reset to zero when copied.
    pub sort_index: u64,
    /// `m_eVertClass`.
    pub vertex_class: PolyVertexClass,
    /// `m_dVertexDistance`.
    pub vertex_distance: f64,
    /// `m_dMaxFaceError`.
    pub max_face_error: f64,
    /// `m_vAveragePlaneNormal`.
    pub average_plane_normal: Vec3,
}

/// The quadratic error form (`SmQuadricVertexAttr`).
#[derive(Debug, Clone, PartialEq)]
pub struct QuadricVertexData {
    /// Three rows of `m_Q`.
    pub matrix: [Vec3; 3],
    /// `m_b`.
    pub linear: Vec3,
    /// `m_c`.
    pub constant: f64,
}

/// Edge contraction bookkeeping (`SmQuadricEdgeAttr`).
#[derive(Debug, Clone, PartialEq)]
pub struct QuadricEdgeData {
    /// `m_vNewPoint`.
    pub new_point: Point3,
    /// `m_dError`.
    pub error: f64,
    /// `m_lSortIndex`: unset until assigned; `MakeCopy` sets it to zero.
    pub sort_index: Option<u64>,
}

/// Construct `SmDecimateVertexAttr` (upstream default behaviour: `Copy`).
#[must_use]
pub fn decimate_vertex_attribute(
    id: u64,
    behavior: crate::geom::attribute::Behavior,
) -> crate::geom::attribute::Attribute {
    use crate::geom::attribute::{Attribute, AttributeValue};
    Attribute::with_value(
        id,
        behavior,
        AttributeValue::DecimateVertex(DecimateVertexData {
            sort_index: 0,
            vertex_class: PolyVertexClass::NotComputed,
            vertex_distance: -crate::math::BIG_DOUBLE,
            max_face_error: 0.0,
            average_plane_normal: Vec3::new(0.0, 0.0, 0.0),
        }),
    )
}

/// Construct `SmQuadricVertexAttr` (upstream default behaviour: `Copy`).
#[must_use]
pub fn quadric_vertex_attribute(
    id: u64,
    behavior: crate::geom::attribute::Behavior,
) -> crate::geom::attribute::Attribute {
    use crate::geom::attribute::{Attribute, AttributeValue};
    let zero = Vec3::new(0.0, 0.0, 0.0);
    Attribute::with_value(
        id,
        behavior,
        AttributeValue::QuadricVertex(QuadricVertexData {
            matrix: [zero; 3],
            linear: zero,
            constant: 0.0,
        }),
    )
}

/// Construct `SmQuadricEdgeAttr` (upstream default behaviour: `Copy`).
#[must_use]
pub fn quadric_edge_attribute(
    id: u64,
    behavior: crate::geom::attribute::Behavior,
) -> crate::geom::attribute::Attribute {
    use crate::geom::attribute::{Attribute, AttributeValue};
    Attribute::with_value(
        id,
        behavior,
        AttributeValue::QuadricEdge(QuadricEdgeData {
            new_point: Vec3::new(0.0, 0.0, 0.0),
            error: 0.0,
            sort_index: None,
        }),
    )
}

/// The B-rep and region handles and ray-fire access used by [`point_3d_classify`].
/// Geometry and cache trees use the existing topology solver adapter; handles
/// remain caller-owned and all equality tests retain object identity.
pub trait BrepClassifyObjects<O>:
    crate::intersect::topology_solver::p1::TopologyObjects<O>
{
    /// The B-rep being queried.
    fn brep_object(&self) -> O;
    /// Its complete region list in native list order (`GetRegions`).
    fn regions(&self) -> Result<Vec<O>>;
    /// `SmBrep::RayFire`: containing region, with the classification state written
    /// even on failure. The default marks the actual missing callee.
    fn ray_fire(
        &self,
        _kernel: &Kernel,
        _point: Point3,
        _direction: Vec3,
        _point_zone_tolerance: f64,
        _classification: &mut crate::intersect::point_class::PointClassification<O>,
    ) -> Result<Option<O>> {
        // UNPORTED(topology.brep_p2): SmBrep::RayFire
        Err(SolidError::unported("SmBrep::RayFire", "topology.brep_p2"))
    }
}

/// Classify a point against the B-rep boundary and its containing region.
/// `point_zone_tolerance` is an already constructed `SmZoneTol3d` value (see
/// [`crate::geom::tol::zone_tol3d`] for conversion from a raw tolerance).
///
/// The in/out classification preserves upstream's initial writes on failure and
/// retains its other fields. Boundary solving runs before the single-region
/// shortcut; region ray firing tries +X/-X, then +Y/-Y, then +Z/-Z only as needed.
///
/// # Errors
/// Those of boundary solving and classification loading. An unavailable callee
/// propagates `Unported`; ordinary ray-fire failures trigger the native retry
/// order, with a generic upstream error when no region is found.
pub fn point_3d_classify<O>(
    kernel: &Kernel,
    objects: &dyn BrepClassifyObjects<O>,
    point: Point3,
    point_zone_tolerance: f64,
    boundary_intersections: bool,
    classification: &mut crate::intersect::point_class::PointClassification<O>,
) -> Result<()>
where
    O: crate::intersect::point_class::ClassObject + crate::geom::solution_array::SolutionObject,
{
    use crate::error::ignored;
    use crate::geom::global_solver::{SolutionRequested, SolverOperation};
    use crate::intersect::point_class::{PointClassType, PointClassification};
    use crate::intersect::topology_solver::p1::TopologySolver;
    use crate::intersect::topology_solver::p2::TopologySolveOptions;
    const OP: &str = "query::brep::point_3d_classify";
    classification.set_class_object(PointClassType::Unknown, None);
    classification.set_src_zone_tol_3d(point_zone_tolerance);
    if boundary_intersections {
        let solutions = TopologySolver::brep_point_solve(
            kernel,
            objects,
            Some(objects.brep_object()),
            point,
            TopologySolveOptions {
                operation: SolverOperation::Intersect,
                requested: SolutionRequested::Single,
                tolerance_3d: point_zone_tolerance,
                best_answer_so_far_sq: 0.0,
                opt_vectors: &[],
            },
        )?;
        if let Some(solution) = solutions.as_slice().first() {
            let object = solution
                .objects
                .first()
                .copied()
                .flatten()
                .ok_or_else(|| SolidError::invalid_input(OP, "null intersection object"))?;
            classification.load_from_intersection(
                point_zone_tolerance,
                object,
                &solution.start,
                0,
                crate::math::BIG_DOUBLE,
            )?;
            return Ok(());
        }
    }
    let regions = objects.regions()?;
    if let [region] = regions.as_slice() {
        classification.set_class_object(PointClassType::Region, Some(*region));
        classification.set_gap_3d(0.0);
        classification.set_pre_snap_param(crate::math::BIG_DOUBLE);
        return Ok(());
    }
    let mut scratch = PointClassification::new(point_zone_tolerance);
    let mut region = None;
    for direction in [
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    ] {
        region = match ignored(objects.ray_fire(
            kernel,
            point,
            direction,
            point_zone_tolerance,
            &mut scratch,
        ))? {
            Some(region) => region,
            None => ignored(objects.ray_fire(
                kernel,
                point,
                -direction,
                point_zone_tolerance,
                &mut scratch,
            ))?
            .flatten(),
        };
        if region.is_some() {
            break;
        }
    }
    let region = region.ok_or_else(|| SolidError::sm_err(OP))?;
    classification.set_class_object(PointClassType::Region, Some(region));
    classification.set_deviation(0.0); // SM_USE_OLDTOL; SetDeviation delegates SetGap3d.
    classification.set_pre_snap_param(crate::math::BIG_DOUBLE);
    Ok(())
}

/// Options for [`calculate_bounding_box`], matching the three native defaults.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoundingBoxOptions {
    /// Skip faces; with more than 500 vertices AND more than 100 edges, also skip
    /// edges after including every vertex box (`bApprox`, default false).
    pub approximate: bool,
    /// Ask each edge and face for its tight box (`bTight`, default false).
    pub tight: bool,
    /// Expand each object's box by its own zone tolerance (default true).
    pub expand_by_zone_tolerance: bool,
}
impl Default for BoundingBoxOptions {
    fn default() -> Self {
        Self {
            approximate: false,
            tight: false,
            expand_by_zone_tolerance: true,
        }
    }
}
fn face_bounding_box(
    _kernel: &Kernel,
    _brep: &Brep,
    _face: FaceId,
    _tight: bool,
    _out: &mut Extent3d,
) -> Result<()> {
    // UNPORTED(query.face_p1): SmFace::CalculateBoundingBox
    Err(SolidError::unported(
        "SmFace::CalculateBoundingBox",
        "query.face_p1",
    ))
}
fn face_tight_bounding_box(
    _kernel: &Kernel,
    _brep: &Brep,
    _face: FaceId,
    _out: &mut Extent3d,
) -> Result<()> {
    // UNPORTED(query.face_p3): SmFace::CalculateTightBoundingBox
    Err(SolidError::unported(
        "SmFace::CalculateTightBoundingBox",
        "query.face_p3",
    ))
}

/// The B-rep box, respecting a previously locked box before reading geometry.
/// Unfiltered native vertex/edge/face lists are the corresponding arenas in list
/// order. Tolerance expansion is per object, not a final global expansion.
///
/// # Errors
/// Propagates the first failing edge or face box calculation. Face box callees
/// remain explicitly unported; no surface box is substituted for a face box.
pub fn calculate_bounding_box(
    kernel: &Kernel,
    brep: &Brep,
    options: BoundingBoxOptions,
) -> Result<Extent3d> {
    use crate::nurbs::b_spline_curve::BoundingBoxRequest;
    if let Some(locked) = brep.locked_bounding_box {
        return Ok(locked);
    }
    let mut bbox = Extent3d::new();
    for (_, vertex) in brep.vertices().iter() {
        let mut vbox = Extent3d::from_point(vertex.point);
        if options.expand_by_zone_tolerance {
            vbox.expand_absolute(vertex.tolerance);
        }
        bbox = bbox.union(&vbox);
    }
    if options.approximate && brep.vertices().len() > 500 && brep.edges().len() > 100 {
        return Ok(bbox);
    }
    let request = BoundingBoxRequest {
        normal: true,
        pseudo: false,
        polar: false,
    };
    for (id, edge) in brep.edges().iter() {
        if edge.curve.is_some() {
            let mut ebox = crate::query::edge::calculate_bounding_box(
                kernel,
                brep,
                id,
                request,
                options.tight,
            )?
            .normal
            .ok_or_else(|| SolidError::sm_err("query::brep::calculate_bounding_box"))?;
            if options.expand_by_zone_tolerance {
                ebox.expand_absolute(edge.tolerance);
            }
            bbox = bbox.union(&ebox);
        }
    }
    if !options.approximate {
        for (id, face) in brep.faces().iter() {
            let mut fbox = Extent3d::new();
            face_bounding_box(kernel, brep, id, options.tight, &mut fbox)?;
            if options.expand_by_zone_tolerance {
                fbox.expand_absolute(face.tolerance);
            }
            bbox = bbox.union(&fbox);
        }
    }
    Ok(bbox)
}

/// The tight B-rep box, without tolerance expansion; ignores the locked box.
/// Loose boxes contained by the accumulated box skip the expensive tight call.
/// As upstream, ordinary edge/face calculation failures are ignored, retaining
/// their defined output state; missing callees still propagate `Unported`.
///
/// # Errors
/// An unavailable face/edge callee or an invalid arena reference.
pub fn calculate_tight_bounding_box(kernel: &Kernel, brep: &Brep) -> Result<Extent3d> {
    use crate::error::ignored;
    use crate::nurbs::b_spline_curve::BoundingBoxRequest;
    let mut bbox = Extent3d::new();
    for (_, vertex) in brep.vertices().iter() {
        bbox.add_point(vertex.point);
    }
    let request = BoundingBoxRequest {
        normal: true,
        pseudo: false,
        polar: false,
    };
    for (id, _) in brep.edges().iter() {
        // SmEdge::CalculateBoundingBox always initialises the normal box first.
        // The normal-only curve routines write their final box after fallible
        // work; a composite ignores child statuses and returns its union. Thus
        // an ordinary error leaves this freshly initialised empty edge box.
        let mut ebox = match ignored(crate::query::edge::calculate_bounding_box(
            kernel, brep, id, request, false,
        ))? {
            Some(boxes) => boxes
                .normal
                .ok_or_else(|| SolidError::sm_err("query::brep::calculate_tight_bounding_box"))?,
            None => Extent3d::new(),
        };
        if ebox.is_contained_by(&bbox, 0.0) {
            continue;
        }
        // CalculateTightBoundingBox writes only on success; without a curve it
        // succeeds without touching the caller's existing loose box.
        if let Some(Some(tight)) = ignored(crate::query::edge::calculate_tight_bounding_box(
            kernel, brep, id,
        ))? {
            ebox = tight;
        }
        bbox = bbox.union(&ebox);
    }
    let mut face_box = Extent3d::new();
    for (id, _) in brep.faces().iter() {
        crate::error::ignore_status(face_bounding_box(kernel, brep, id, false, &mut face_box))?;
        if face_box.is_contained_by(&bbox, 0.0) {
            continue;
        }
        crate::error::ignore_status(face_tight_bounding_box(kernel, brep, id, &mut face_box))?;
        bbox = bbox.union(&face_box);
    }
    Ok(bbox)
}

/// Replace the locked box, computing a tight box when requested. The previous
/// lock is released BEFORE calculation, and stays released if calculation fails.
///
/// # Errors
/// Those of the selected box calculation.
pub fn lock_bounding_box(kernel: &Kernel, brep: &mut Brep, tight: bool) -> Result<()> {
    unlock_bounding_box(brep);
    let bbox = if tight {
        calculate_tight_bounding_box(kernel, brep)?
    } else {
        calculate_bounding_box(kernel, brep, BoundingBoxOptions::default())?
    };
    brep.locked_bounding_box = Some(bbox);
    Ok(())
}

/// Release the locked box, also succeeding when no box is locked.
pub fn unlock_bounding_box(brep: &mut Brep) {
    brep.locked_bounding_box = None;
}
