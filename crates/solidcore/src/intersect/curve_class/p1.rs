//! Curve classification, part 1: the interval and classification containers, their
//! accessors and storage management, the touch-list helpers, and the static helpers
//! that tighten classification-curve / edge intersections onto surf/surf/surf points.
//!
//! Upstream: `source/SMLib/src/SmCurveClass.cpp`, `source/SMLib/inc/SmCurveClass.h`;
//! the `SmCurveClassification` / `SmCurveInterval` overloads of `SmTol` in
//! `source/SMLib/src/SmTol.cpp`.
//!
//! | Rust                                                   | Upstream                                         |
//! |--------------------------------------------------------|--------------------------------------------------|
//! | [`IntervalPosition`]                                   | `SmIntervalPositionType`                         |
//! | [`CrvDirOnFaceType`]                                   | `SmCrvDirOnFaceType`                             |
//! | [`TouchData`]                                          | `SmTouchData`                                    |
//! | [`get_connected_touches`]                              | `SmTouchData::GetConnectedTouches`               |
//! | [`get_winding_count`]                                  | `SmTouchData::GetWindingCount`                   |
//! | [`CurveInterval`]                                      | `SmCurveInterval`                                |
//! | [`CurveInterval::new`], [`CurveInterval::with_interval`] | `SmCurveInterval(ctx, owner)`, `SmCurveInterval(ivl, ctx, owner)` |
//! | [`CurveInterval::reset`]                               | `SmCurveInterval::ReSet`                         |
//! | [`CurveInterval::set_interval`]                        | `SmCurveInterval::SetInterval`                   |
//! | `Clone` / `Copy` (derived)                             | `SmCurveInterval::operator=`                     |
//! | [`CurveInterval::set_interval_min`], [`CurveInterval::set_interval_max`] | `SetIntervalMin`, `SetIntervalMax` (private) |
//! | [`CurveInterval::interval`], [`CurveInterval::min`], [`CurveInterval::max`] | `GetInterval`, `GetMin`, `GetMax` |
//! | [`CurveClassification`]                                | `SmCurveClassification`                          |
//! | [`CurveClassification::new`]                           | `SmCurveClassification()`                        |
//! | `Drop` (implicit)                                      | `~SmCurveClassification`                         |
//! | [`CurveClassification::size`]                          | `GetSize`                                        |
//! | [`CurveClassification::face`]                          | `GetFace`                                        |
//! | [`CurveClassification::surface1`], [`CurveClassification::surface2`] | `GetSurface1`, `GetSurface2`        |
//! | [`CurveClassification::curve`]                         | `GetCurve`                                       |
//! | [`CurveClassification::set_interval`]               | `SetInterval` (inline)                           |
//! | [`CurveClassification::interval`]                      | `GetInterval`                                    |
//! | [`CurveClassification::sub_interval`]                  | `GetSubInterval`                                 |
//! | [`CurveClassification::uv_curve1`], [`CurveClassification::uv_curve2`] | `GetUVCurve1`, `GetUVCurve2`      |
//! | [`CurveClassification::any_uv_curve1`], [`CurveClassification::any_uv_curve2`] | `GetAnyUVCurve1`, `GetAnyUVCurve2` |
//! | [`CurveClassification::topology_intersector`]          | `GetTopologyIntersector`                         |
//! | [`CurveClassification::is_curve_current`]              | `GetIsCurveCurrent`                              |
//! | [`CurveClassification::ignore_tangency`]               | `GetIgnoreTangency`                              |
//! | [`CurveClassification::set_doing_cross_hatch`]         | `SetDoingCrossHatch`                             |
//! | [`CurveClassification::doing_cross_hatch`]             | `GetDoingCrossHatch`                             |
//! | [`CurveClassification::doing_fillets`]                 | `GetDoingFillets`                                |
//! | [`CurveClassification::other_classification`]          | `GetOtherClassification`                         |
//! | [`CurveClassification::ok_to_set_tolerance`]           | `GetOKToSetTolerance`                            |
//! | [`CurveClassification::tgt_loops_only`], [`CurveClassification::set_tgt_loops_only`] | `GetTgtLoopsOnly`, `SetTgtLoopsOnly` |
//! | [`CurveClassification::tgt_loops`], [`CurveClassification::add_tgt_loop`] | `GetTgtLoops`, `AddTgtLoop`   |
//! | [`CurveClassification::src_zone_tol_3d`]               | `GetSrcZoneTol3d` (private)                      |
//! | [`CurveClassification::set_src_zone_tol_3d`]           | `SetSrcZoneTol3d` (private; through `SmTol::SetZoneTol3d`) |
//! | [`CurveClassification::at`], [`CurveClassification::at_mut`] | `GetAt`, `operator[]`                      |
//! | [`CurveClassification::set_surface1`], [`CurveClassification::set_surface2`] | `SetSurface1`, `SetSurface2` |
//! | [`CurveClassification::set_face`]                      | `SetFace`                                        |
//! | [`CurveClassification::set_ignore_tangency`]           | `SetIgnoreTangency`                              |
//! | [`CurveClassification::set_is_curve_owned`], [`CurveClassification::is_curve_owned`] | `SetIsCurveOwned`, `m_bIsCurveOwned` |
//! | [`CurveClassification::set_ok_to_set_tolerance`]       | `SetOKToSetTolerance`                            |
//! | [`CurveClassification::set_fillet_data`]               | `SetFilletData`                                  |
//! | [`CurveClassification::set_uv_curve1`], [`CurveClassification::set_uv_curve2`] | `SetUVCurve1`, `SetUVCurve2` (no plan unit: unreachable from the entry points) |
//! | [`CurveClassification::type_code`], [`CURVE_CLASSIFICATION_TYPE_STRING`], [`CurveClassification::is_kind_of`] | `GetType`, `GetTypeString`, `IsKindOf` |
//! | [`CurveClassification::classify_crv_loc`]              | `GetClassifyCrvLoc(const SmPointClassification*, double)` |
//! | [`CurveClassification::set_curve`]                     | `SetCurve`                                       |
//! | [`CurveClassification::set_intersect_data`]            | `SetIntersectData`                               |
//! | [`CurveClassification::set_is_curve_current`]          | `SetIsCurveCurrent`                              |
//! | [`CurveClassification::remove_at`]                     | `RemoveAt`                                       |
//! | [`CurveClassification::set_size`], [`MAX_INTERVALS`]   | `SetSize`                                        |
//! | [`CurveClassification::set_point_class_parameters`]    | `SetPointClassParameters`                        |
//! | [`CurveClassification::find_best_squeeze_end`]         | `FindBestSqueezeEnd`                             |
//! | [`CurveClassification::is_referenced`]                 | `IsReferenced`                                   |
//! | [`src_zone_tol_3d_of`]                                 | `SmTol::GetSrcZoneTol3d(const SmCurveClassification*)`, `SmTol::GetSrcZoneTol3d(const SmCurveInterval*)` |
//! | [`interval_obj_zone_tol_3d`]                           | `SmTol::GetObjZoneTol3d(const SmCurveInterval*)` |
//! | `ScaledZero for CurveInterval`                         | `SmTol::GetScaledZero(const SmCurveInterval&)`   |
//! | [`set_xsect_curve_through_point`], [`XSectThroughPoint`] | `sm_SetXSectCurveThroughPoint` (static)        |
//! | [`set_xsect_edge_through_point`]                       | `sm_SetXSectEdgeThroughPoint` (static)           |
//! | [`get_cci_surface_gaps`], [`CciSurfaceGaps`]           | `sm_GetCCISurfaceGaps` (static)                  |
//! | [`get_cci_geometry`], [`CciGeometry`], [`TopologyHandles`] | `sm_GetCCIGeometry` (static)                |
//! | [`apply_point_solution`], [`ApplyPointSolutionInput`], [`ApplyPointSolution`] | `sm_ApplyPointSolution` (static) |
//! | [`make_consistent_classifications`]                    | `sm_MakeConsistentClassifications` (static)      |
//! | [`get_curve_curve_angle`]                              | `sm_GetCurveCurveAngle` (static)                 |
//! | [`cull_or_refine_cci_in_3d`]                           | `sm_CullOrRefineCCIIn3d` (static)                |
//! | [`compute_face_point_class`]                           | `sm_ComputeFacePointClass` (static)              |
//! | [`xsect_tol_2d_or_3d_for_classify_pair`]               | `sm_GetXSectTol2dOr3d_ForClassifyPair`           |
//! | [`object_dimension`]                                   | `sm_ObjectDimension` (static)                    |
//!
//! Deviations:
//! - **Object handles.** Every `SmObject` pointer the classification stores (curve,
//!   UV curves, surfaces, face, target loops, the other classification, the topology
//!   intersector) is a caller-chosen handle `O` ([`ClassObject`]), as in
//!   [`PointClassification`]; geometry behind a handle is reached through a
//!   [`ClassGeometry`]. `m_cpContext` is not stored (the kernel is passed explicitly).
//! - **No back pointer.** `SmCurveInterval::m_pCurveClassification` and
//!   `GetCurveClassification` are not ported: an interval lives inside its
//!   classification and is reached through it. The constructors take the owner (or
//!   `None`) only for its source zone tolerance (`SmTol::GetSrcZoneTol3d(cpOwner)`).
//! - **Storage.** `m_pData` / `m_lMaxSize` / `m_bIsBorrowed` become a `Vec` plus a flag
//!   recording whether upstream's array is allocated: growing from no array
//!   default-constructs the new intervals (their point classifications take the system
//!   zone tolerance, `SmTol::GetSrcZoneTol3d(NULL)`), growing an allocated array
//!   `ReSet`s them (`SM_UNINIT_TOL`). Borrowed arrays (`pOptBorrowedData`) are a
//!   memory-management feature of `ReSet` (unit `intersect.curve_class_p2`) and are not
//!   modelled. [`CurveClassification::set_size`] refuses more than [`MAX_INTERVALS`]
//!   intervals instead of allocating without bound.
//! - **Construction.** `with_curve_interval` initializes a single interval directly
//!   for boundary APIs; it does not co-port part 2's `ReSet` implementation.
//! - **Ownership of the curve.** `m_bIsCurveOwned` is kept as data; there is nothing to
//!   delete when a handle is replaced.
//! - **Undefined behaviour upstream** is an error here: `GetSubInterval`, `GetAt`,
//!   `operator[]`, `FindBestSqueezeEnd` past the size (upstream reads past `m_lSize`),
//!   `RemoveAt` on an empty classification (`m_lSize` wraps), a `NULL` curve where one is
//!   evaluated, an odd-length touch list in `GetWindingCount`, and missing surfaces or
//!   UV points where the static helpers dereference them. `RemoveAt` past the end of a
//!   non-empty classification drops the last interval, as upstream.
//! - **Out-params.** Status plus out-params become result structs; an out-param upstream
//!   leaves unwritten on some paths is an `Option` (`None` = not written) or is passed
//!   in and returned. `GetConnectedTouches` returns the connected touches (their count
//!   is the length). `sm_ApplyPointSolution` takes the classification as its curve
//!   (mutable: upstream edits it through a `const_cast`) and source zone tolerance,
//!   and its edge and mate as ids of the B-rep it is given; the edge step's
//!   `dTDeviation` keeps the classification curve's value when
//!   `sm_SetXSectEdgeThroughPoint` does not write it, as upstream.
//! - **`SmTemporaryChangeValue` on `GetOutOfBoundsEnabled`.** The surfaces a static
//!   helper evaluates out of bounds are copies with out-of-bounds evaluation enabled
//!   (B-spline surfaces and planes, the `SmBSplineSurface` kinds); the caller's
//!   surfaces are unchanged, as after upstream's restore.
//! - **Dead code.** `sm_ApplyPointSolution`'s `gwcUpdateCurves` / `gwcUpdateEdges` are
//!   constant `TRUE`, so its `FALSE` branches, and every `SM_DEBUG_CODE` block, are not
//!   ported; `SM_ASSERT*` only report.
//! - **Not ported here (other units).** `ReSet`, `InsertAt`, `operator=` and the other
//!   members of `SmCurveClassification` belong to units `intersect.curve_class_p2` ..
//!   `_p9`; `SmTouchData::operator==` (the derived `PartialEq`), `Homogenize`, `Dump`,
//!   `AssertValid` and `Draw` are not reachable from the in-scope entry points.
//! - **B-rep topology.** `sm_SetXSectEdgeThroughPoint`, `sm_GetCCIGeometry`,
//!   `sm_ComputeFacePointClass` and the edge path of `sm_ApplyPointSolution` take the
//!   [`Brep`] and the edge's or face's id;
//!   `sm_GetCCIGeometry` names the faces' surfaces, edge uses and UV trim curves with
//!   the caller's handles ([`TopologyHandles`]) and `sm_ComputeFacePointClass`
//!   classifies with [`EntityRef`] objects (as `SmFace::PointClassify`). The face
//!   surface cache's point test switch (`m_bPointTestEnabled`) has no counterpart:
//!   the solver tests no trim boundary of a surface without face owner and is
//!   `Unported` for one with a face owner. In `sm_SetXSectEdgeThroughPoint` the UV
//!   trim curve arrays built from the edge's faces are reset before use upstream and
//!   are not built. In
//!   `sm_ComputeFacePointClass` the best UV point starts at `(0, 0)` (upstream leaves
//!   it uninitialized; only read when every solution value is `SM_BIG_DOUBLE`).
//! - **Callees in units not ported yet** return [`SolidError::Unported`]: the
//!   vertex repair of `sm_MakeConsistentClassifications` (`SmEdge::GetVertex`,
//!   `SmVertex::GetEdges`), the Boolean mate of `sm_GetCCIGeometry`
//!   (`SmTopologyIntersector`), the surface point
//!   solves (`surface.surface_p4`, `surface.surface_p5`), the 3D-curve
//!   branch of `SmTol::MapTo2d(.., SmCurve, SmSurface)` (`SmSurface::DropPoint`,
//!   `surface.surface_p5`; its UV-curve branch is co-ported as
//!   [`map_to_2d_on_curve`]), and an `SmCrvOnSurf` over a
//!   surface that is not a B-spline (a dispatch gap of `curve.crv_on_surf`, whose
//!   `CrvOnSurf` holds only an `SmBSplineSurface`).
//! - **Recursion.** None.

use std::borrow::Cow;

use crate::curve::crv_on_surf::CrvOnSurf;
use crate::curve::curve::p2::{Curve, CurveKind};
use crate::curve::curve::p3::{local_curve_intersect, PointSolveOptions};
use crate::error::{ignored, EntityRef, Result, SolidError, SM_ERR};
use crate::geom::assert_array::labels::CURVE_CLASSIFICATION_TYPE;
use crate::geom::extent1d::Extent1d;
use crate::geom::global_solver::{SolutionRequested, SolverOperation};
use crate::geom::object;
use crate::geom::solution_array::{Solution, SolutionArray, SolutionObject, SolutionType};
use crate::geom::tol::{
    ang_tol_rad, eff_zero, eff_zero_param, xsect_tol3d, xsect_tol_3d_from_zones, zone_tol3d,
    zone_tol_3d, ScaledZero, SmallTopoType, UNINIT_TOL,
};
use crate::intersect::point_class::{
    ClassGeometry, ClassObject, ClassObjectKind, PointClassType, PointClassification,
};
use crate::kernel::Kernel;
use crate::math::{max2, Point2, Point3, Vec2, Vec3, BIG_DOUBLE, PI, UNDEF_DOUBLE};
use crate::nurbs::b_spline_curve::BSplineCurve;
use crate::nurbs::b_spline_surface_n_lib::p2::{EvaluateFlags, OrientType};
use crate::query::face::p2::{point_classify, PointClassifyOptions};
use crate::surface::surface::p1::map_to_2d_on_curve;
use crate::surface::surface::p2::surface_cache;
use crate::surface::surface::{Surface, SurfaceKind};
use crate::topology::edgeuse::p1::{set_uv_trim_curve, uv_trim_curve, UvTrimCurveArg};
use crate::topology::face::p2 as face;
use crate::topology::{edge, Brep, EdgeId, EdgeuseId, FaceId};

const OP_CC: &str = "intersect::curve_class::p1::CurveClassification";
const OP_TOUCH: &str = "intersect::curve_class::p1::get_winding_count";
const OP_XSECT: &str = "intersect::curve_class::p1::set_xsect_curve_through_point";
const OP_APPLY: &str = "intersect::curve_class::p1::apply_point_solution";
const OP_GAPS: &str = "intersect::curve_class::p1::get_cci_surface_gaps";
const OP_EDGE_XSECT: &str = "intersect::curve_class::p1::set_xsect_edge_through_point";
const OP_CCI: &str = "intersect::curve_class::p1::get_cci_geometry";
const OP_FACE_CLASS: &str = "intersect::curve_class::p1::compute_face_point_class";

/// The most intervals [`CurveClassification::set_size`] stores (far above anything a
/// classification reaches; upstream allocates whatever is asked for).
pub const MAX_INTERVALS: usize = 1 << 20;

/// `SM_MIN_ARRAY_SIZE` / `MIN_ARRAY_SIZE` of `SetSize`.
const MIN_ARRAY_SIZE: usize = 16;

/// `SmCurveClassification::GetTypeString`.
pub const CURVE_CLASSIFICATION_TYPE_STRING: &str = "SmCurveClassification_TYPE";

// ---- enums ------------------------------------------------------------------------------

/// Where a parameter falls relative to an interval (`SmIntervalPositionType`); the
/// discriminants are the upstream values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(i32)]
pub enum IntervalPosition {
    /// `SM_IP_UNINIT`: not yet classified.
    #[default]
    Uninit = 0,
    /// `SM_IP_START`: on the start point.
    Start = 1,
    /// `SM_IP_INSIDE`: between the start and end points.
    Inside = 2,
    /// `SM_IP_END`: on the end point.
    End = 3,
    /// `SM_IP_OUTSIDE`: outside the interval.
    Outside = 4,
}

impl IntervalPosition {
    /// The upstream enum value.
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }
}

/// The direction of a surface vector at a surface point relative to a face
/// (`SmCrvDirOnFaceType`); the discriminants are the upstream values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(i32)]
pub enum CrvDirOnFaceType {
    /// `SM_CD_UNINIT`.
    #[default]
    Uninit = 0,
    /// `SM_CD_NONE`.
    None = 1,
    /// `SM_CD_IN`: points into the face.
    In = 2,
    /// `SM_CD_ON`: tangent to a face boundary.
    On = 3,
    /// `SM_CD_OUT`: points out of the face.
    Out = 4,
    /// `SM_CD_LAMINA`: out of the surface over a lamina boundary.
    Lamina = 5,
    /// `SM_CD_SEAM`: out of the surface over a seam.
    Seam = 6,
    /// `SM_CD_POLE`: out of the surface over a pole marked by a vertex.
    Pole = 7,
    /// `SM_CD_POLE_NO_VERTEX`: out of the surface over a pole without a vertex.
    PoleNoVertex = 8,
}

impl CrvDirOnFaceType {
    /// The upstream enum value.
    #[must_use]
    pub const fn code(self) -> i32 {
        self as i32
    }

    /// The type with upstream value `code`, if any.
    #[must_use]
    pub const fn from_code(code: i32) -> Option<Self> {
        Some(match code {
            0 => Self::Uninit,
            1 => Self::None,
            2 => Self::In,
            3 => Self::On,
            4 => Self::Out,
            5 => Self::Lamina,
            6 => Self::Seam,
            7 => Self::Pole,
            8 => Self::PoleNoVertex,
            _ => return None,
        })
    }
}

// ---- touch data -------------------------------------------------------------------------

/// How the classification curve arrives at or departs from a face boundary at one of
/// its point classifications (`SmTouchData`). Touch lists hold arrive/depart pairs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TouchData<O> {
    /// `m_pConnectedTo`: the topology the point classification connects to.
    pub connected_to: Option<O>,
    /// `m_eLoopDir`.
    pub loop_dir: CrvDirOnFaceType,
    /// `m_lLoopIndx`: the index of the loop of `connected_to`.
    pub loop_index: u64,
    /// `m_dParam`: the classification curve parameter.
    pub param: f64,
    /// `m_lPtClassifyIndx`: the index of the point classification.
    pub pt_classify_index: u64,
    /// `m_pPtClassify`: the point classification (a handle).
    pub pt_classify: Option<O>,
}

/// The touches of `touches` connected to `connected_to`, in order
/// (`SmTouchData::GetConnectedTouches`; upstream's count is the length).
#[must_use]
pub fn get_connected_touches<O: PartialEq + Copy>(
    connected_to: Option<O>,
    touches: &[TouchData<O>],
) -> Vec<TouchData<O>> {
    // for every touch
    touches
        .iter()
        .filter(|t| t.connected_to == connected_to)
        .copied()
        .collect()
}

/// The winding count of a single loop's touch list (`SmTouchData::GetWindingCount`):
/// the number of unmatched in/out and out/in arrive/depart pairs, and
/// [`OrientType::Opposite`] (`SM_OT_LOWERDOMAIN`) for more in/out pairs,
/// [`OrientType::Same`] (`SM_OT_UPPERDOMAIN`) for more out/in pairs, else
/// [`OrientType::Unknown`].
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a list of odd length (upstream reads past its end).
pub fn get_winding_count<O>(touches: &[TouchData<O>]) -> Result<(u64, OrientType)> {
    if touches.len() % 2 != 0 {
        return Err(SolidError::invalid_input(
            OP_TOUCH,
            "touch list of odd length",
        ));
    }

    // init locals and output
    let mut in_out_cnt: u64 = 0;
    let mut out_in_cnt: u64 = 0;

    // for every touch
    for pair in touches.chunks_exact(2) {
        let (Some(a), Some(d)) = (pair.first(), pair.get(1)) else {
            continue;
        };
        if a.loop_dir == CrvDirOnFaceType::In && d.loop_dir == CrvDirOnFaceType::Out {
            in_out_cnt += 1;
        }
        if a.loop_dir == CrvDirOnFaceType::Out && d.loop_dir == CrvDirOnFaceType::In {
            out_in_cnt += 1;
        }
    }

    // all done - return WindingCount = unpaired In/Out to Out/In touches
    let orient = match in_out_cnt.cmp(&out_in_cnt) {
        core::cmp::Ordering::Greater => OrientType::Opposite,
        core::cmp::Ordering::Less => OrientType::Same,
        core::cmp::Ordering::Equal => OrientType::Unknown,
    };

    // smos_Labs(lInOutCnt - lOutInCnt) of the unsigned difference
    Ok((in_out_cnt.abs_diff(out_in_cnt), orient))
}

// ---- curve interval ---------------------------------------------------------------------

/// The classification of one homogeneous interval of the classification curve
/// (`SmCurveInterval`): its parameter range and the point classifications of its start,
/// interior and end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveInterval<O> {
    /// `m_vInterval`: the parameter interval (its ends may have been snapped, see the
    /// point classifications' pre-snap parameters).
    pub interval: Extent1d,
    /// `m_vStart`: the classification of the start point.
    pub start: PointClassification<O>,
    /// `m_vMid`: the classification of the interior.
    pub mid: PointClassification<O>,
    /// `m_vEnd`: the classification of the end point.
    pub end: PointClassification<O>,
}

impl<O: ClassObject> CurveInterval<O> {
    /// An interval with the "inside out" extent whose point classifications take the
    /// owner's source zone tolerance (`SmCurveInterval(cpContext, cpOwner)`;
    /// `SmTol::GetSrcZoneTol3d(cpOwner)`, the system zone tolerance without an owner).
    #[must_use]
    pub fn new(k: &Kernel, owner: Option<&CurveClassification<O>>) -> Self {
        Self::with_interval(k, &Extent1d::new(), owner)
    }

    /// An interval over `interval` (`SmCurveInterval(crInterval, cpContext, cpOwner)`).
    #[must_use]
    pub fn with_interval(
        k: &Kernel,
        interval: &Extent1d,
        owner: Option<&CurveClassification<O>>,
    ) -> Self {
        let tol = src_zone_tol_3d_of(k, owner);
        Self {
            interval: *interval,
            start: PointClassification::new(tol),
            mid: PointClassification::new(tol),
            end: PointClassification::new(tol),
        }
    }

    /// Resets the extent and the three point classifications (`ReSet`).
    pub fn reset(&mut self, k: &Kernel) {
        self.interval.init(); // m_cpContext value left alone by design
        self.start.reset(k, None);
        self.mid.reset(k, None);
        self.end.reset(k, None);
    }

    /// Sets the parameter interval (`SetInterval`).
    pub fn set_interval(&mut self, interval: &Extent1d) {
        self.interval = *interval;
    }

    /// Moves the start of the interval (`SetIntervalMin`, private upstream: use the
    /// classification's `EditIntervalMin`).
    ///
    /// # Errors
    ///
    /// Those of [`Extent1d::set_min_max`] (the interval would become negative); the
    /// interval is then unchanged.
    pub fn set_interval_min(&mut self, new_param: f64) -> Result<()> {
        let max = self.interval.max();
        self.interval.set_min_max(new_param, max)
    }

    /// Moves the end of the interval (`SetIntervalMax`, private upstream).
    ///
    /// # Errors
    ///
    /// As [`CurveInterval::set_interval_min`].
    pub fn set_interval_max(&mut self, new_param: f64) -> Result<()> {
        let min = self.interval.min();
        self.interval.set_min_max(min, new_param)
    }

    /// The parameter interval (`GetInterval`).
    #[must_use]
    pub const fn interval(&self) -> &Extent1d {
        &self.interval
    }

    /// The start parameter (`GetMin`).
    #[must_use]
    pub const fn min(&self) -> f64 {
        self.interval.min()
    }

    /// The end parameter (`GetMax`).
    #[must_use]
    pub const fn max(&self) -> f64 {
        self.interval.max()
    }
}

/// `SmTol::GetScaledZero(const SmCurveInterval&)`: the scaled zero of the interval.
impl<O> ScaledZero for CurveInterval<O> {
    fn scaled_zero(&self) -> f64 {
        self.interval.scaled_zero()
    }
}

// ---- curve classification ---------------------------------------------------------------

/// The classification of a curve against another object (`SmCurveClassification`):
/// the curve split into ascending homogeneous [`CurveInterval`]s spanning its interval.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::struct_excessive_bools)] // upstream's SmBoolean members
pub struct CurveClassification<O> {
    /// `m_bIsCurveOwned`.
    is_curve_owned: bool,
    /// `m_cpCurve`: the curve being classified.
    curve: Option<O>,
    /// `m_sSrcZoneTol3d`: the zone tolerance of the curve's source.
    src_zone_tol_3d: f64,
    /// `m_cpUVCurve1`.
    uv_curve1: Option<O>,
    /// `m_bIsCurveCurrent`.
    is_curve_current: bool,
    /// `m_cpSurface1`.
    surface1: Option<O>,
    /// `m_cpSurface2`.
    surface2: Option<O>,
    /// `m_bDoingCrossHatch`.
    doing_cross_hatch: bool,
    /// `m_bDoingFillets`.
    doing_fillets: bool,
    /// `m_bOKToSetTolerance`.
    ok_to_set_tolerance: bool,
    /// `m_bTgtLoopsOnly`.
    tgt_loops_only: bool,
    /// `m_sTgtLoops`.
    tgt_loops: Vec<O>,
    /// `m_cpOtherClassification`.
    other_classification: Option<O>,
    /// `m_cpFace`.
    face: Option<O>,
    /// `m_cpUVCurve2`.
    uv_curve2: Option<O>,
    /// `m_cpTI`.
    ti: Option<O>,
    /// `m_vInterval`: the whole interval classified.
    interval: Extent1d,
    /// `m_pData[0 .. m_lSize]`.
    data: Vec<CurveInterval<O>>,
    /// `m_pData != NULL`: whether upstream's interval array is allocated.
    allocated: bool,
    /// `m_bIgnoreTangency`.
    ignore_tangency: bool,
}

impl<O: ClassObject> CurveClassification<O> {
    /// An empty classification (`SmCurveClassification()`): no curve, no intervals,
    /// interval `[0, 1]`, source zone tolerance `SM_UNINIT_TOL`.
    #[must_use]
    pub fn new(k: &Kernel) -> Self {
        Self {
            is_curve_owned: false,
            curve: None,
            src_zone_tol_3d: zone_tol3d(k, UNINIT_TOL),
            uv_curve1: None,
            is_curve_current: true,
            surface1: None,
            surface2: None,
            doing_cross_hatch: false,
            doing_fillets: false,
            ok_to_set_tolerance: true,
            tgt_loops_only: false,
            tgt_loops: Vec::new(),
            other_classification: None,
            face: None,
            uv_curve2: None,
            ti: None,
            interval: Extent1d::from_min_max(0.0, 1.0),
            data: Vec::new(),
            allocated: false,
            ignore_tangency: false,
        }
    }

    /// Creates a classification with one unclassified interval for a curve.
    /// This initializes the same interval and source-zone state required by the
    /// boundary intersection APIs without invoking the separately owned `ReSet`.
    #[must_use]
    pub fn with_curve_interval(k: &Kernel, curve: O, interval: Extent1d, source_zone: f64) -> Self {
        let mut result = Self::new(k);
        result.set_curve(Some(curve), false);
        result.set_src_zone_tol_3d(source_zone);
        result.interval = interval;
        result
            .data
            .push(CurveInterval::with_interval(k, &interval, Some(&result)));
        result.allocated = true;
        result
    }

    /// Sets the native cross-hatch tolerance mode (`SetDoingCrossHatch`).
    pub fn set_doing_cross_hatch(&mut self, enabled: bool) {
        self.doing_cross_hatch = enabled;
    }

    // ---- simple data access --------------------------------------------------------

    /// The number of intervals (`GetSize`; the end-point count is one more).
    #[must_use]
    pub fn size(&self) -> usize {
        self.data.len()
    }

    /// The intervals.
    #[must_use]
    pub fn intervals(&self) -> &[CurveInterval<O>] {
        &self.data
    }

    /// Sets the complete classification domain (inline `SetInterval`).
    ///
    /// # Errors
    /// Returns the interval validation error instead of ignoring an invalid domain.
    pub fn set_interval(&mut self, interval: &Extent1d) -> Result<()> {
        self.interval.set_min_max(interval.min(), interval.max())
    }

    /// The face classified against (`GetFace`).
    #[must_use]
    pub const fn face(&self) -> Option<O> {
        self.face
    }

    /// `GetSurface1`.
    #[must_use]
    pub const fn surface1(&self) -> Option<O> {
        self.surface1
    }

    /// `GetSurface2`.
    #[must_use]
    pub const fn surface2(&self) -> Option<O> {
        self.surface2
    }

    /// The curve being classified (`GetCurve`).
    #[must_use]
    pub const fn curve(&self) -> Option<O> {
        self.curve
    }

    /// The whole interval classified (`GetInterval`).
    #[must_use]
    pub const fn interval(&self) -> Extent1d {
        self.interval
    }

    /// The parameter interval of interval `index` (`GetSubInterval`).
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for `index >= size()` (upstream reads past the
    /// intervals).
    pub fn sub_interval(&self, index: usize) -> Result<Extent1d> {
        Ok(self.at(index)?.interval)
    }

    /// The surface-1 UV curve while the curve is current (`GetUVCurve1`).
    #[must_use]
    pub const fn uv_curve1(&self) -> Option<O> {
        if self.is_curve_current {
            self.uv_curve1
        } else {
            None
        }
    }

    /// The surface-2 UV curve while the curve is current (`GetUVCurve2`).
    #[must_use]
    pub const fn uv_curve2(&self) -> Option<O> {
        if self.is_curve_current {
            self.uv_curve2
        } else {
            None
        }
    }

    /// `GetAnyUVCurve1`.
    #[must_use]
    pub const fn any_uv_curve1(&self) -> Option<O> {
        self.uv_curve1
    }

    /// `GetAnyUVCurve2`.
    #[must_use]
    pub const fn any_uv_curve2(&self) -> Option<O> {
        self.uv_curve2
    }

    /// `GetTopologyIntersector`.
    #[must_use]
    pub const fn topology_intersector(&self) -> Option<O> {
        self.ti
    }

    /// `GetIsCurveCurrent`.
    #[must_use]
    pub const fn is_curve_current(&self) -> bool {
        self.is_curve_current
    }

    /// `GetIgnoreTangency`.
    #[must_use]
    pub const fn ignore_tangency(&self) -> bool {
        self.ignore_tangency
    }

    /// `GetDoingCrossHatch`.
    #[must_use]
    pub const fn doing_cross_hatch(&self) -> bool {
        self.doing_cross_hatch
    }

    /// `GetDoingFillets`.
    #[must_use]
    pub const fn doing_fillets(&self) -> bool {
        self.doing_fillets
    }

    /// `GetOtherClassification`.
    #[must_use]
    pub const fn other_classification(&self) -> Option<O> {
        self.other_classification
    }

    /// `GetOKToSetTolerance`.
    #[must_use]
    pub const fn ok_to_set_tolerance(&self) -> bool {
        self.ok_to_set_tolerance
    }

    /// `m_bIsCurveOwned` (no upstream getter).
    #[must_use]
    pub const fn is_curve_owned(&self) -> bool {
        self.is_curve_owned
    }

    /// `GetTgtLoopsOnly`.
    #[must_use]
    pub const fn tgt_loops_only(&self) -> bool {
        self.tgt_loops_only
    }

    /// `SetTgtLoopsOnly`.
    pub fn set_tgt_loops_only(&mut self, tgt_loops_only: bool) {
        self.tgt_loops_only = tgt_loops_only;
    }

    /// `GetTgtLoops`.
    #[must_use]
    pub fn tgt_loops(&self) -> &[O] {
        &self.tgt_loops
    }

    /// `AddTgtLoop`.
    pub fn add_tgt_loop(&mut self, tgt_loop: O) {
        self.tgt_loops.push(tgt_loop);
    }

    /// The source zone tolerance (`GetSrcZoneTol3d`; its `SM_ASSERT_TOL` only reports).
    #[must_use]
    pub const fn src_zone_tol_3d(&self) -> f64 {
        self.src_zone_tol_3d
    }

    /// Sets the source zone tolerance (`SetSrcZoneTol3d`, reached through
    /// `SmTol::SetZoneTol3d`).
    pub fn set_src_zone_tol_3d(&mut self, zone_tol_3d: f64) {
        self.src_zone_tol_3d = zone_tol_3d;
    }

    // ---- curve interval access -------------------------------------------------------

    /// Interval `index` (`GetAt`, `operator[]`).
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for `index >= size()`.
    pub fn at(&self, index: usize) -> Result<&CurveInterval<O>> {
        self.data.get(index).ok_or_else(bad_index)
    }

    /// Interval `index`, mutable (`GetAt`, `operator[]`).
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for `index >= size()`.
    pub fn at_mut(&mut self, index: usize) -> Result<&mut CurveInterval<O>> {
        self.data.get_mut(index).ok_or_else(bad_index)
    }

    // ---- sets ------------------------------------------------------------------------

    /// `SetSurface1`.
    pub fn set_surface1(&mut self, surface1: Option<O>) {
        self.surface1 = surface1;
    }

    /// `SetSurface2`.
    pub fn set_surface2(&mut self, surface2: Option<O>) {
        self.surface2 = surface2;
    }

    /// `SetFace`.
    pub fn set_face(&mut self, face: Option<O>) {
        self.face = face;
    }

    /// `SetIgnoreTangency`.
    pub fn set_ignore_tangency(&mut self, ignore_tangency: bool) {
        self.ignore_tangency = ignore_tangency;
    }

    /// `SetIsCurveOwned`.
    pub fn set_is_curve_owned(&mut self, is_curve_owned: bool) {
        self.is_curve_owned = is_curve_owned;
    }

    /// `SetOKToSetTolerance`.
    pub fn set_ok_to_set_tolerance(&mut self, ok_to_set_tolerance: bool) {
        self.ok_to_set_tolerance = ok_to_set_tolerance;
    }

    /// Sets the fillet data (`SetFilletData`; upstream default `bDoingFillets = TRUE`).
    pub fn set_fillet_data(&mut self, other_classification: Option<O>, doing_fillets: bool) {
        self.other_classification = other_classification;
        self.doing_fillets = doing_fillets;
    }

    /// Sets the surface-1 UV curve (`SetUVCurve1`; deleting the old one,
    /// `bDeleteOldUVCurve`, has no counterpart for a handle).
    pub fn set_uv_curve1(&mut self, uv_curve1: Option<O>) {
        self.uv_curve1 = uv_curve1;
    }

    /// Sets the surface-2 UV curve (`SetUVCurve2`).
    pub fn set_uv_curve2(&mut self, uv_curve2: Option<O>) {
        self.uv_curve2 = uv_curve2;
    }

    /// Sets the curve (`SetCurve`; upstream default `bIsCurveOwned = FALSE`); the curve
    /// becomes current.
    pub fn set_curve(&mut self, curve: Option<O>, is_curve_owned: bool) {
        // when needed - free old curve (nothing to free for a handle)

        // make assignment
        self.curve = curve;
        self.is_curve_owned = is_curve_owned;
        self.is_curve_current = true;
    }

    /// Sets the surf/surf intersection data (`SetIntersectData`; upstream defaults
    /// `bIsCurveCurrent = TRUE`, `cpTI = NULL`).
    pub fn set_intersect_data(
        &mut self,
        surface1: Option<O>,
        surface2: Option<O>,
        uv_curve2: Option<O>,
        is_curve_current: bool,
        ti: Option<O>,
    ) {
        self.surface1 = surface1;
        self.surface2 = surface2;
        self.is_curve_current = is_curve_current;
        self.uv_curve2 = uv_curve2;
        self.ti = ti;
    }

    /// Marks the curve current or not (`SetIsCurveCurrent`); a curve that is no longer
    /// current loses its UV curves.
    pub fn set_is_curve_current(&mut self, current: bool) {
        self.is_curve_current = current;
        if !current {
            self.uv_curve1 = None;
            self.uv_curve2 = None;
        }
    }

    // ---- type --------------------------------------------------------------------------

    /// `GetType`.
    #[must_use]
    pub const fn type_code(&self) -> i64 {
        CURVE_CLASSIFICATION_TYPE
    }

    /// `IsKindOf`: its own type, then `SmObject`'s.
    #[must_use]
    pub const fn is_kind_of(&self, t: i64) -> bool {
        t == CURVE_CLASSIFICATION_TYPE || object::is_kind_of(t)
    }

    // ---- editing -----------------------------------------------------------------------

    /// Removes interval `index` (`RemoveAt`). An index past the end of a non-empty
    /// classification drops the last interval, as upstream.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for an empty classification (upstream's size
    /// wraps).
    pub fn remove_at(&mut self, index: usize) -> Result<()> {
        let size = self.data.len();
        if size == 0 {
            return Err(SolidError::invalid_input(
                OP_CC,
                "remove_at on an empty classification",
            ));
        }
        if index < size {
            self.data.remove(index);
        } else {
            // the shift loop does nothing; m_lSize -= 1
            self.data.truncate(size - 1);
        }
        Ok(())
    }

    /// Sets the number of intervals (`SetSize`). New intervals are default-constructed
    /// when no array is allocated yet, else reset; size 0 frees the array.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for more than [`MAX_INTERVALS`] intervals.
    pub fn set_size(&mut self, k: &Kernel, new_size: usize) -> Result<()> {
        if new_size > MAX_INTERVALS {
            return Err(SolidError::invalid_input(OP_CC, "too many intervals"));
        }

        // switch on NewSize
        if new_size == 0 {
            // shrink to nothing
            self.data.clear();
            self.allocated = false;
        } else if !self.allocated {
            // a new array of default-constructed intervals (no owner)
            let fresh = CurveInterval::new(k, None);
            self.data.clear();
            self.data.resize(new_size, fresh);
            self.allocated = true;
        } else if new_size > self.data.len() {
            // init any newly used SmCurveIntervals (within or beyond the allocation)
            let mut reset = CurveInterval::new(k, None);
            reset.reset(k);
            self.data.resize(new_size, reset);
        } else {
            self.data.truncate(new_size);
        }
        // (MIN_ARRAY_SIZE growth only sizes upstream's allocation)
        let _ = MIN_ARRAY_SIZE;
        Ok(())
    }

    // ---- queries ------------------------------------------------------------------------

    /// The location of a point classification with curve parameter `param`
    /// (`GetClassifyCrvLoc(const SmPointClassification*, double)`): its surf/surf/surf
    /// point when set, else the curve point.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] when the curve is needed but there is none or
    /// `geometry` cannot resolve it.
    pub fn classify_crv_loc(
        &self,
        geometry: &impl ClassGeometry<O>,
        point_class: &PointClassification<O>,
        param: f64,
    ) -> Result<Point3> {
        // When pointClassification has a stored point
        if point_class.sss_point_flag() {
            return Ok(point_class.sss_point());
        }

        // compute curve point at pointClassification parameter (status ignored)
        let curve = self.curve_geometry(geometry)?;
        Ok(ignored(curve.evaluate_point(param))?.unwrap_or(Vec3::UNDEF))
    }

    /// Makes sure every start and end point classification (and every interior one
    /// with `set_mids`) has its object parameters set (`SetPointClassParameters`;
    /// upstream default `bSetMids = FALSE`): the first start and every end from the
    /// curve point at their parameter, every other start copied from the previous end.
    ///
    /// # Errors
    ///
    /// The last failure of the curve evaluations and of
    /// [`PointClassification::compute_point_parameters`] (processing continues past
    /// them, as upstream); [`SolidError::Unported`] at once; an error for a missing
    /// curve where one is evaluated.
    pub fn set_point_class_parameters(
        &mut self,
        k: &Kernel,
        geometry: &impl ClassGeometry<O>,
        set_mids: bool,
    ) -> Result<()> {
        let num_cis = self.data.len();
        let mut ret: Result<()> = Ok(());
        let mut pt = Vec3::UNDEF;

        // record a failing status, propagate Unported at once
        let mut record = |r: Result<()>| -> Result<()> {
            match r {
                Ok(()) => Ok(()),
                Err(e) if e.is_unported() => Err(e),
                Err(e) => {
                    ret = Err(e);
                    Ok(())
                }
            }
        };

        for ii in 0..num_cis {
            // Start:
            let start_set = self.at(ii)?.start.are_parameters_set();
            if !start_set {
                if ii == 0 {
                    let param = self.at(ii)?.interval.min();
                    match self.curve_geometry(geometry)?.evaluate_point(param) {
                        Ok(p) => pt = p,
                        Err(e) => record(Err(e))?,
                    }
                    let r = self
                        .at_mut(ii)?
                        .start
                        .compute_point_parameters(k, geometry, pt);
                    record(r)?;
                } else {
                    let prev_end = self.at(ii - 1)?.end;
                    self.at_mut(ii)?.start = prev_end;
                }
            }

            // End:
            if !self.at(ii)?.end.are_parameters_set() {
                let param = self.at(ii)?.interval.max();
                match self.curve_geometry(geometry)?.evaluate_point(param) {
                    Ok(p) => pt = p,
                    Err(e) => record(Err(e))?,
                }
                let r = self
                    .at_mut(ii)?
                    .end
                    .compute_point_parameters(k, geometry, pt);
                record(r)?;
            }

            // Mid:
            if set_mids && !self.at(ii)?.mid.are_parameters_set() {
                let param = self.at(ii)?.interval.mid();
                match self.curve_geometry(geometry)?.evaluate_point(param) {
                    Ok(p) => pt = p,
                    Err(e) => record(Err(e))?,
                }
                let r = self
                    .at_mut(ii)?
                    .mid
                    .compute_point_parameters(k, geometry, pt);
                record(r)?;
            }
        } // End for each CurveInterval

        ret
    }

    /// Which end of interval `index` to squeeze out (`FindBestSqueezeEnd`): the end
    /// classified to the more general object kind (the first interval's start, or the
    /// last interval's end, then takes the other end's classification), else the end
    /// with the larger gap.
    ///
    /// # Errors
    ///
    /// [`SolidError::InvalidInput`] for `index >= size()`.
    pub fn find_best_squeeze_end(&mut self, index: usize) -> Result<IntervalPosition> {
        let last = self.data.len().saturating_sub(1);
        let ivl = self.at_mut(index)?;
        let start_class = ivl.start.point_class().code();
        let end_class = ivl.end.point_class().code();

        // when startPoint is more general than endPoint - squeeze startPoint
        if start_class < end_class {
            // When squeezing the first interval - set startPointClass = endPointClass
            if index == 0 {
                ivl.start = ivl.end;
            }
            return Ok(IntervalPosition::Start);
        }
        // when endPoint is more general than startPoint - squeeze endPoint
        else if start_class > end_class {
            // when squeezing the last interval - set endPointClass = startPointClass
            if index == last {
                ivl.end = ivl.start;
            }
            return Ok(IntervalPosition::End);
        }

        // arrive here when both endPoints classifiy to the same type of topology

        // squeeze the end with the greatest deviation
        Ok(if ivl.start.deviation() > ivl.end.deviation() {
            IntervalPosition::Start
        } else {
            IntervalPosition::End
        })
    }

    /// Whether `object` is the object of any point classification (`IsReferenced`);
    /// false for `None`.
    #[must_use]
    pub fn is_referenced(&self, object: Option<O>) -> bool {
        // return FALSE for NULL objects
        let Some(o) = object else {
            return false;
        };

        // for every interval
        self.data.iter().any(|ivl| {
            ivl.start.object() == Some(o)
                || ivl.mid.object() == Some(o)
                || ivl.end.object() == Some(o)
        })
    }

    /// The curve geometry, or an error for no curve.
    pub(super) fn curve_geometry<'g>(
        &self,
        geometry: &'g impl ClassGeometry<O>,
    ) -> Result<&'g Curve> {
        let c = self
            .curve
            .ok_or(SolidError::invalid_input(OP_CC, "no classification curve"))?;
        geometry
            .curve(c)
            .ok_or(SolidError::invalid_input(OP_CC, "unresolved curve handle"))
    }
}

fn bad_index() -> SolidError {
    SolidError::invalid_input(OP_CC, "interval index out of range")
}

// ---- SmTol overloads ----------------------------------------------------------------------

/// The source zone tolerance of a classification, the system zone tolerance for none
/// (`SmTol::GetSrcZoneTol3d(const SmCurveClassification*)`; also
/// `SmTol::GetSrcZoneTol3d(const SmCurveInterval*)` given the interval's owner).
#[must_use]
pub fn src_zone_tol_3d_of<O: ClassObject>(
    k: &Kernel,
    classification: Option<&CurveClassification<O>>,
) -> f64 {
    match classification {
        Some(c) => c.src_zone_tol_3d(),
        None => zone_tol_3d(k, SmallTopoType::NormalSize),
    }
}

/// The zone tolerance of an interval's interior object, the system zone tolerance for
/// no interval (`SmTol::GetObjZoneTol3d(const SmCurveInterval*)`).
///
/// # Errors
///
/// Those of [`PointClassification::obj_zone_tol_3d`].
pub fn interval_obj_zone_tol_3d<O: ClassObject>(
    k: &Kernel,
    geometry: &impl ClassGeometry<O>,
    interval: Option<&CurveInterval<O>>,
) -> Result<f64> {
    match interval {
        Some(i) => i.mid.obj_zone_tol_3d(k, geometry),
        None => Ok(zone_tol_3d(k, SmallTopoType::NormalSize)),
    }
}

/// `SmTol::GetZoneTol3d(const SmSurface*)`: the tolerance of the surface's face, the
/// system zone tolerance for a surface without one.
fn surface_zone_tol_3d(k: &Kernel, surface: &Surface) -> Result<f64> {
    crate::surface::surface::p1::zone_tol_3d(k, surface)
}

// ---- static helpers -----------------------------------------------------------------------

/// A copy of `surface` evaluating out of bounds when it is an `SmBSplineSurface` kind
/// (`SmTemporaryChangeValue<SmBoolean>(GetOutOfBoundsEnabled(), TRUE)`).
fn out_of_bounds(surface: &Surface) -> Cow<'_, Surface> {
    match surface.kind() {
        SurfaceKind::BSpline(s) => {
            let mut s = s.clone();
            *s.out_of_bounds_enabled_mut() = true;
            Cow::Owned(Surface::from(s))
        }
        SurfaceKind::Plane(p) => {
            let mut p = p.copy();
            *p.out_of_bounds_enabled_mut() = true;
            Cow::Owned(Surface::from(p))
        }
        SurfaceKind::SurfOfRevolution(s) => {
            // the SmBSplineSurface base flag only (not SetOutOfBoundsEnabled)
            let mut s = s.as_ref().clone();
            *s.base_mut().out_of_bounds_enabled_mut() = true;
            Cow::Owned(Surface::from(s))
        }
        SurfaceKind::CurveBounded(_) | SurfaceKind::Step(_) | SurfaceKind::SrfInVolume(_) => {
            Cow::Borrowed(surface)
        }
    }
}

/// `SmSurface::Evaluate(uv, 1, 1, TRUE, TRUE, TRUE, vals)` (status ignored upstream):
/// `vals[i][j]`, the upper half; `[1][1]` and the values of a failed evaluation stay
/// undefined.
fn surface_evaluate_11(surface: &Surface, uv: Vec2) -> Result<(bool, [[Vec3; 2]; 2])> {
    let mut vals = [[Vec3::UNDEF; 2]; 2];
    let r = surface.evaluate(
        uv,
        1,
        1,
        EvaluateFlags {
            only_upper_half: true,
            ..EvaluateFlags::default()
        },
    );
    let Some(d) = ignored(r)? else {
        return Ok((false, vals));
    };
    let at = |i: usize, j: usize| d.get(i).and_then(|r| r.get(j)).copied();
    for (i, j) in [(0, 0), (0, 1), (1, 0)] {
        if let (Some(v), Some(slot)) = (at(i, j), vals.get_mut(i).and_then(|r| r.get_mut(j))) {
            *slot = v;
        }
    }
    Ok((true, vals))
}

/// `SmCurve::Evaluate(t, 1, TRUE, pv)` (status ignored upstream): the point and
/// tangent, undefined for a failed evaluation.
fn curve_evaluate_1(curve: &Curve, t: f64) -> Result<[Vec3; 2]> {
    let mut pv = [Vec3::UNDEF; 2];
    if let Some(d) = ignored(curve.evaluate(t, 1, true, true))? {
        for (slot, v) in pv.iter_mut().zip(d) {
            *slot = v;
        }
    }
    Ok(pv)
}

/// The `SmBSplineCurve` part of a curve (`IsKindOf(SmBSplineCurve_TYPE)` and the cast).
fn bspline_part_mut(curve: &mut Curve) -> Option<&mut BSplineCurve> {
    match curve.kind_mut() {
        CurveKind::BSpline(c) => Some(c),
        CurveKind::Ellipse(c) => Some(c.base_mut()),
        CurveKind::Hyperbola(c) => Some(c.base_mut()),
        CurveKind::Parabola(c) => Some(c.base_mut()),
        CurveKind::Line(c) => Some(c.base_mut()),
        _ => None,
    }
}

/// The result of [`set_xsect_curve_through_point`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct XSectThroughPoint {
    /// `SM_SUCCESS` (`true`) or `SM_ERR` (`false`; "the curve was not set up to be
    /// tweaked", not a failure the caller must stop on).
    pub success: bool,
    /// `bModifiedCurve`: the curves were edited to pass through the point.
    pub modified_curve: bool,
    /// `dParam` out: the curve parameter of the intersection point.
    pub param: f64,
    /// `dDeviation`: the distance from the point to the curve (`None` when upstream
    /// leaves it unwritten).
    pub deviation: Option<f64>,
    /// `bSet3DPoint`: `point_3d` was set from the first surface.
    pub set_3d_point: bool,
    /// `*p3DPoint` out: the intersection point (the caller's value unless set).
    pub point_3d: Point3,
}

/// Makes a surf/surf intersection curve (and its UV trim curves) pass through the
/// surf/surf/surf intersection point given by `uv_points` on `surfaces`
/// (`sm_SetXSectCurveThroughPoint`): the point is the first surface's point there;
/// it is projected onto the curve near `param`, and when that is farther than the
/// intersection tolerance and the surfaces' intersection tangent is defined and nearly
/// parallel to the curve, a B-spline curve gets a through point
/// ([`BSplineCurve::set_intersection_curve_through_point`]). `point_3d` is the caller's
/// value of the output point.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for fewer than two UV points with two surfaces
/// (upstream reads past them); [`SolidError::Unported`] for a surface owned by a face
/// (its zone tolerance).
pub fn set_xsect_curve_through_point(
    k: &Kernel,
    src_curve: &mut Curve,
    curve_zone_tol_3d: f64,
    uv_curves: &mut [BSplineCurve],
    surfaces: &[&Surface],
    uv_points: &[Vec2],
    param: f64,
    point_3d: Point3,
) -> Result<XSectThroughPoint> {
    // init outputs - curve not modified
    let mut out = XSectThroughPoint {
        success: false,
        modified_curve: false,
        param,
        deviation: None,
        set_3d_point: false,
        point_3d,
    };

    // check state
    let ok_inputs = uv_curves.len() <= surfaces.len()
        && surfaces.len() >= 2
        && surfaces.len() == uv_points.len();

    // 1st the surface evaluations and setting p3DPoint
    let (Some(s0), Some(s1)) = (surfaces.first(), surfaces.get(1)) else {
        return Ok(out);
    };
    let (Some(&uv0), Some(&uv1)) = (uv_points.first(), uv_points.get(1)) else {
        return Err(SolidError::invalid_input(
            OP_XSECT,
            "fewer UV points than surfaces",
        ));
    };

    // temporarily allow out of bound evaluations for surface[0], surface[1]
    let (orig0, orig1) = (*s0, *s1);
    let s0 = out_of_bounds(orig0);
    let s1 = out_of_bounds(orig1);

    // evaluate each surface at the given intersection parameter points
    let (ok0, mut surf_val0) = surface_evaluate_11(&s0, uv0)?;
    let (_, mut surf_val1) = surface_evaluate_11(&s1, uv1)?;

    // tolerances
    let xsect_obj_zone_tol_3d = max2(
        surface_zone_tol_3d(k, orig0)?,
        surface_zone_tol_3d(k, orig1)?,
    );
    let xsect_tol_3d = xsect_tol_3d_from_zones(k, curve_zone_tol_3d, xsect_obj_zone_tol_3d);
    // (sXSectTol3dSq only feeds a warning)
    let ang_bound_rad = 6.0 * ang_tol_rad();
    let scaled_zero = eff_zero();

    // let surf/surf/surf xsect point = pSurface[0] evaluation
    if ok0 {
        out.set_3d_point = true;
        out.point_3d = surf_val0[0][0];
    }

    // Abort when inputs are confused (after setting output variables).
    if !ok_inputs {
        return Ok(out);
    }

    // project p3DPoint onto pSrcCurve3d to get its param value
    let natural = src_curve.natural_interval();
    out.param = natural.clamp_value(out.param);
    let projection = ignored(src_curve.local_point_solve::<usize>(
        &natural,
        SolverOperation::Minimize,
        out.point_3d,
        &PointSolveOptions::default(),
        out.param,
    ))?;

    // when the point fails to project to the curve
    let Some((true, projection)) = projection else {
        return Ok(out);
    };

    // grab the projected point's curve param and curve/point deviation values
    out.param = projection.start.param(0)?;
    let deviation = projection.start.solution_value;
    out.deviation = Some(deviation);

    // Note: sXSectTol3d = XSectTol3d(CurveSrcZoneTol3d,Surface)
    if deviation < xsect_tol_3d {
        out.success = true;
        return Ok(out);
    }

    // evaluate the curve at the param value for pos and tangent information
    let src_crv_pt = curve_evaluate_1(src_curve, out.param)?;

    surf_val0[1][0] = surf_val0[1][0].unitize_or_z();
    surf_val0[0][1] = surf_val0[0][1].unitize_or_z();
    surf_val1[1][0] = surf_val1[1][0].unitize_or_z();
    surf_val1[0][1] = surf_val1[0][1].unitize_or_z();
    let norm1 = surf_val0[1][0].cross(surf_val0[0][1]);
    let norm2 = surf_val1[1][0].cross(surf_val1[0][1]);
    let mut xsect_tangent = norm1.cross(norm2);

    // skip curve modification when the throughPoint is close enough to a tangent
    // surf/surf intersection point that the tangent vector can't easily be computed
    if xsect_tangent.length_squared() <= 100.0 * scaled_zero {
        out.success = true;
        return Ok(out);
    }

    // ensure Tangent direction is same as curve's direction
    if xsect_tangent.dot(src_crv_pt[1]) < 0.0 {
        xsect_tangent = -xsect_tangent;
    }

    // Surf/Surf XSectLine Tang direction is not nearly parallel to pSrcCurve3d's tangent
    let tang_angle = xsect_tangent.angle_between(src_crv_pt[1]).unwrap_or(0.0);
    if tang_angle > ang_bound_rad {
        out.success = true;
        return Ok(out);
    }

    // unitize the tangent vector
    xsect_tangent = xsect_tangent.unitize_or_z();

    // insert throughPoint=p3DPoint (tangent=sXSectTangent3d) at dParam into pSrcCurve3d
    if let Some(bs) = bspline_part_mut(src_curve) {
        let uv_opts: Vec<Option<Vec2>> = uv_points.iter().copied().map(Some).collect();
        // the surfaces as evaluated here (copies; evaluation does not use the owner)
        let evaluators: Vec<Surface> = surfaces
            .iter()
            .enumerate()
            .map(|(i, s)| match i {
                0 => s0.as_ref().clone(),
                1 => s1.as_ref().clone(),
                _ => (*s).clone(),
            })
            .collect();
        let mut new_param = out.param;
        let r = bs.set_intersection_curve_through_point(
            k,
            &mut new_param,
            out.point_3d,
            Some(xsect_tangent),
            uv_curves,
            &evaluators,
            &uv_opts,
            0.001,
        );
        out.param = new_param;
        // When Successful - reset dDeviation
        match ignored(r)? {
            Some(modified) => {
                out.modified_curve = modified;
                out.deviation = Some(0.0);
            }
            None => out.modified_curve = false,
        }
    }

    // all done - succesfully
    out.success = true;
    Ok(out)
}

/// `sm_SetXSectEdgeThroughPoint`: makes the curve of edge `e` of `b` pass through the
/// surf/surf/surf intersection point `through_point` (`uv_points` on `surfaces` are
/// its UV points): the point is projected onto the edge curve over the edge interval
/// near `param`; when the surfaces' intersection tangent is defined and nearly
/// parallel to the curve, and the point is farther from a B-spline curve than the
/// edge tolerance / 2000, the curve gets a through point
/// ([`BSplineCurve::set_intersection_curve_through_point`], without UV trim curves).
/// A curve whose natural interval was the edge interval gets its knots rescaled back
/// to it, and the UV trim curves of the edge's edge uses are removed (they no longer
/// fit). `point_3d` / `set_3d_point` of the result are unused (the caller's point and
/// `false`); `deviation` is `None` until the projection is found.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for an edge without curve (upstream dereferences it)
/// or fewer than two UV points with two surfaces; those of the topology accessors
/// for a dangling id; [`SolidError::Unported`] from the evaluations and the trim
/// curve removal.
pub fn set_xsect_edge_through_point(
    k: &Kernel,
    b: &mut Brep,
    e: EdgeId,
    through_point: Point3,
    surfaces: &[&Surface],
    uv_points: &[Vec2],
    param: f64,
) -> Result<XSectThroughPoint> {
    // init outputs - curve not modified
    let mut out = XSectThroughPoint {
        success: false,
        modified_curve: false,
        param,
        deviation: None,
        set_3d_point: false,
        point_3d: through_point,
    };

    // locals
    let curve_zone_tol_3d = edge::tolerance(b, e)?;
    let edge_interval = edge::interval(b, e)?;
    let Some(src_curve) = edge::curve(b, e)? else {
        return Err(SolidError::invalid_input(
            OP_EDGE_XSECT,
            "edge without curve",
        ));
    };
    let curve_interval = src_curve.natural_interval();
    let common_curve_edge_interval =
        edge_interval.min() == curve_interval.min() && edge_interval.max() == curve_interval.max();

    // check state - given or found valid pointers to edge, curve, points, and surfaces
    if surfaces.len() < 2 || uv_points.len() != surfaces.len() {
        return Ok(out);
    }

    // project pThroughPoint onto pCurve
    let projection = ignored(src_curve.local_point_solve::<usize>(
        &edge_interval,
        SolverOperation::Minimize,
        through_point,
        &PointSolveOptions::default(),
        out.param,
    ))?;

    // error - when no projected point is found
    let Some((true, projection)) = projection else {
        return Ok(out);
    };

    // grab the projected point's param and deviaton values
    out.param = projection.start.param(0)?;
    let projection_value = projection.start.solution_value;
    out.deviation = Some(projection_value);

    // evaluate the curve at the param value for pos and tangent information
    let src_crv_pt = curve_evaluate_1(src_curve, out.param)?;

    // temporarily allow out of bound evaluations for surface[0], surface[1]
    let (Some(orig0), Some(orig1), Some(&uv0), Some(&uv1)) = (
        surfaces.first(),
        surfaces.get(1),
        uv_points.first(),
        uv_points.get(1),
    ) else {
        return Ok(out);
    };
    let s0 = out_of_bounds(orig0);
    let s1 = out_of_bounds(orig1);
    let (_, mut surf_val0) = surface_evaluate_11(&s0, uv0)?;
    let (_, mut surf_val1) = surface_evaluate_11(&s1, uv1)?;

    // tolerances
    let xsect_tol_3d = xsect_tol_3d_from_zones(
        k,
        curve_zone_tol_3d,
        max2(
            surface_zone_tol_3d(k, orig0)?,
            surface_zone_tol_3d(k, orig1)?,
        ),
    );
    let xsect_tol_3d_sq = xsect_tol_3d * xsect_tol_3d;
    let scaled_zero = eff_zero();
    let ang_bound_rad = 6.0 * ang_tol_rad();

    // check state - surface xsect projections are near one another
    if through_point.distance_squared(surf_val0[0][0]) > xsect_tol_3d_sq
        || through_point.distance_squared(surf_val1[0][0]) > xsect_tol_3d_sq
        || surf_val0[0][0].distance_squared(surf_val1[0][0]) > xsect_tol_3d_sq
    {
        return Ok(out);
    }

    // set through point tangent = cross(surf1Normal,surf2Normal)
    surf_val0[1][0] = surf_val0[1][0].unitize_or_z();
    surf_val0[0][1] = surf_val0[0][1].unitize_or_z();
    surf_val1[1][0] = surf_val1[1][0].unitize_or_z();
    surf_val1[0][1] = surf_val1[0][1].unitize_or_z();
    let norm1 = surf_val0[1][0].cross(surf_val0[0][1]);
    let norm2 = surf_val1[1][0].cross(surf_val1[0][1]);
    let mut xsect_tangent = norm1.cross(norm2);

    if xsect_tangent.length_squared() <= scaled_zero {
        // the surfaces are tangent at this point: tangent = current curve tangent
        xsect_tangent = src_crv_pt[1];
    }

    // ensure Tangent direction is same as curve's direction
    if xsect_tangent.dot(src_crv_pt[1]) < 0.0 {
        xsect_tangent = -xsect_tangent;
    }

    // skip curve modification when the throughPoint is close enough to a tangent
    // surf/surf intersection point that the tangent vector can't easily be computed
    let tang_angle = xsect_tangent.angle_between(src_crv_pt[1]).unwrap_or(0.0);
    if xsect_tangent.length_squared() <= scaled_zero || tang_angle > ang_bound_rad {
        out.success = true;
        return Ok(out);
    }

    // unitize the tangent vector
    xsect_tangent = xsect_tangent.unitize_or_z();

    // force the 3DCurve to interpolate the point
    let is_bspline = matches!(
        src_curve.kind(),
        CurveKind::BSpline(_)
            | CurveKind::Ellipse(_)
            | CurveKind::Hyperbola(_)
            | CurveKind::Parabola(_)
            | CurveKind::Line(_)
    );
    if is_bspline && projection_value > curve_zone_tol_3d / 2000.0 {
        // The UV trim curve, surface and UV point arrays built from the edge's faces
        // are reset before use upstream (GWC:EXPERIMENT - leave the UV trim curves
        // alone), so they are not built here.
        let min_span_percent = 1.0e-6;
        let mut src_curve = b
            .edge_mut(e)?
            .curve
            .take()
            .ok_or_else(|| SolidError::invalid_input(OP_EDGE_XSECT, "edge without curve"))?;
        let mut new_param = out.param;
        let (r, flag) = match bspline_part_mut(&mut src_curve) {
            Some(bs) => bs.set_intersection_curve_through_point_flagged::<Surface>(
                k,
                &mut new_param,
                through_point,
                Some(xsect_tangent),
                &mut [],
                &[],
                &[],
                min_span_percent,
            ),
            None => (Ok(false), false),
        };
        out.param = new_param;
        let modified = match r {
            Ok(m) => Some(m),
            Err(err @ SolidError::Unported { .. }) => {
                b.edge_mut(e)?.curve = Some(src_curve);
                return Err(err);
            }
            Err(_) => None,
        };

        // If the curve was not changed, then we're done (a failed edit keeps the
        // bModifiedCurve it had set).
        if modified != Some(true) {
            b.edge_mut(e)?.curve = Some(src_curve);
            out.modified_curve = flag;
            out.success = true;
            return Ok(out);
        }

        // restore common curve/edge interval if its stopped being common by scaling
        // the curve's parameterization
        let edge_interval = edge::interval(b, e)?;
        let curve_interval = src_curve.natural_interval();
        let (e_min, e_max) = (edge_interval.min(), edge_interval.max());
        let (c_min, c_max) = (curve_interval.min(), curve_interval.max());
        if common_curve_edge_interval
            && ((e_min - c_min).abs() > scaled_zero || (e_max - c_max).abs() > scaled_zero)
        {
            // change the parameter values of every pSrcCurve3d knot (the UV trim
            // curve list is empty)
            if let Some(bs) = bspline_part_mut(&mut src_curve) {
                let r = bs.scale_knot_vector(e_min, e_max);
                if let Err(err) = ignored(r) {
                    b.edge_mut(e)?.curve = Some(src_curve);
                    return Err(err);
                }
            }
        }
        b.edge_mut(e)?.curve = Some(src_curve);

        // reset the deviation value
        out.deviation = Some(0.0);

        // remove obsolete UVTrimCurves from edge->Edgeuses
        let edgeuses = b.edge(e)?.edgeuses.clone();
        for eu in edgeuses {
            set_uv_trim_curve(b, eu, UvTrimCurveArg::Null, 0.0, true, None)?;
        }

        // inform the public that the curves were updated
        out.modified_curve = true;
    }

    // all done - successfully
    out.success = true;
    Ok(out)
}

/// The result of [`get_cci_surface_gaps`].
#[derive(Debug, Clone, PartialEq)]
pub struct CciSurfaceGaps {
    /// `SM_SUCCESS` (`true`), or `SM_ERR` when a point does not drop to a surface (the
    /// outputs are then partial).
    pub success: bool,
    /// `dCurveEdgeGap`: the curve point / edge point distance.
    pub curve_edge_gap: f64,
    /// `dCurveSurfaceGap`: the largest curve point / curve surface gap.
    pub curve_surface_gap: f64,
    /// `dEdgeSurfaceGap`: the largest edge point / edge surface gap.
    pub edge_surface_gap: f64,
    /// `rUVPoints`: the UV point of each drop.
    pub uv_points: Vec<Point2>,
}

/// The gaps between a classification-curve / edge-curve intersection and its surfaces
/// (`sm_GetCCISurfaceGaps`): the first two surfaces are the curve's, the rest the
/// edge's; a surface with a UV trim curve is dropped onto locally from the trim curve
/// point at the solution parameter, the others globally.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for fewer UV curve entries than surfaces;
/// [`SolidError::Unported`] for any surface: the surface point solves (units
/// `surface.surface_p5`, `surface.surface_p4`).
pub fn get_cci_surface_gaps<S: Copy>(
    k: &Kernel,
    class_curve_point: Point3,
    edge_curve_point: Point3,
    solution: &Solution<S>,
    surfaces: &[&Surface],
    uv_curves: &[Option<&Curve>],
    edge_surface_count: usize,
) -> Result<CciSurfaceGaps> {
    // NotUsed
    let _ = edge_surface_count;
    // init output
    let mut out = CciSurfaceGaps {
        success: false,
        curve_edge_gap: class_curve_point.distance(edge_curve_point),
        curve_surface_gap: 0.0,
        edge_surface_gap: 0.0,
        uv_points: Vec::new(),
    };

    // for every surface
    for (ii, surface) in surfaces.iter().enumerate() {
        // get target Point/Surface combination
        let uv_curve = *uv_curves.get(ii).ok_or(SolidError::invalid_input(
            OP_GAPS,
            "fewer UV curves than surfaces",
        ))?;
        let point = if ii < 2 {
            class_curve_point
        } else {
            edge_curve_point
        };

        // When posible use cheap localSolve else use expensive Global Solve
        let sol: Option<(f64, Point2)> = if let Some(uv_curve) = uv_curve {
            // get UVGuessPoint
            let t_param = if ii < 2 {
                solution.start.param(0)?
            } else {
                solution.start.param(1)?
            };
            let guess = ignored(uv_curve.evaluate_point(t_param))?.unwrap_or(Vec3::UNDEF);
            surface_local_point_solve(k, surface, point, Point2::new(guess.x, guess.y))?
        } else {
            // get classCurvePoint/surf dist without checking trim boundaries
            let _cache = surface_cache::<()>(k, surface, None)?;
            let size = surface_approximate_size(surface)?;
            match surface_global_point_solve(k, surface, size, point)? {
                Some(s) => Some(s),
                None => return Ok(out),
            }
        };

        // accumulate UVPoints and distances
        let Some((gap, uv)) = sol else {
            // Point failed to drop to connected surface
            return Ok(out);
        };
        out.uv_points.push(uv);

        // save max CurvePoint/Surface and EdgePoint/Surface gaps
        if ii < 2 {
            if gap > out.curve_surface_gap {
                out.curve_surface_gap = gap;
            }
        } else if gap > out.edge_surface_gap {
            out.edge_surface_gap = gap;
        }
    }

    // all done
    out.success = true;
    Ok(out)
}

/// The surfaces, edge uses and UV trim curves around a classification-curve / edge
/// intersection ([`get_cci_geometry`]).
#[derive(Debug, Clone, PartialEq)]
pub struct CciGeometry<O> {
    /// `pEMate`: the edge's Boolean mate.
    pub emate: Option<O>,
    /// `rSurfaces`: curve surfaces, edge surfaces, the common surface again, mate
    /// surfaces (`None` for a face without surface).
    pub surfaces: Vec<Option<O>>,
    /// `rEdgeuses`: the edge use of each surface (none for the curve's).
    pub edgeuses: Vec<Option<O>>,
    /// `rUVCurves`: the UV trim curve of each surface.
    pub uv_curves: Vec<Option<O>>,
    /// `lEdgeSurfaceCount`.
    pub edge_surface_count: usize,
}

/// The classification handles of B-rep geometry that [`get_cci_geometry`] stores next
/// to the classification's own handles (upstream stores the pointers).
pub trait TopologyHandles<O> {
    /// The handle of face `f`'s surface (`SmFace::GetSurface`), `None` without one.
    fn face_surface(&self, f: FaceId) -> Option<O>;
    /// The handle of edge use `eu`.
    fn edgeuse(&self, eu: EdgeuseId) -> O;
    /// The handle of the UV trim curve stored on edge use `eu`.
    fn uv_trim_curve(&self, eu: EdgeuseId) -> O;
}

/// `sm_GetCCIGeometry`: the surfaces, edge uses and UV trim curves at a
/// classification-curve / edge `e` intersection: the classification's surfaces (with
/// its UV curves), then the surface of every face of the edge that has an edge use
/// there (with its UV trim curve), and the surface the edge shares with the
/// classification once more. `handles` names the B-rep geometry.
///
/// # Errors
///
/// [`SolidError::Unported`] for a classification done for a Boolean (its topology
/// intersector's mate edge, unit `boolean.topology_intersector_p1`); those of the
/// topology accessors for a dangling id.
pub fn get_cci_geometry<O: ClassObject>(
    classification: &CurveClassification<O>,
    b: &Brep,
    e: EdgeId,
    handles: &impl TopologyHandles<O>,
) -> Result<CciGeometry<O>> {
    let mut out = CciGeometry {
        emate: None,
        surfaces: Vec::new(),
        edgeuses: Vec::new(),
        uv_curves: Vec::new(),
        edge_surface_count: 0,
    };

    // locals
    let surface1 = classification.surface1();
    let surface2 = classification.surface2();
    let mut common_surf: Option<usize> = None;

    // gather the ClassCurve Surfaces and UVTrimCurves
    if let Some(s) = surface1 {
        out.surfaces.push(Some(s));
        out.edgeuses.push(None);
        out.uv_curves.push(classification.uv_curve1());
    }
    if let Some(s) = surface2 {
        out.surfaces.push(Some(s));
        out.edgeuses.push(None);
        out.uv_curves.push(classification.uv_curve2());
    }

    // get Faces connected to Edge and Edge->BooleanMate
    let faces = edge::faces(b, e, None)?;
    let edge_face_count = faces.len();

    // When curve classification is being done as part of a boolean operation
    if classification.topology_intersector().is_some() {
        // UNPORTED(boolean.topology_intersector_p1): SmTopologyIntersector::GetOtherMate
        return Err(SolidError::unported(
            "SmTopologyIntersector::GetOtherMate",
            "boolean.topology_intersector_p1",
        ));
    }

    // the UV trim curve stored on an edge use or its mate (`GetUVTrimCurve`)
    let trim_curve = |eu: EdgeuseId| -> Result<Option<O>> {
        let (curve, owner) = uv_trim_curve(b, eu)?;
        Ok(match (curve, owner) {
            (Some(_), Some(owner)) => Some(handles.uv_trim_curve(owner)),
            _ => None,
        })
    };

    // gather unique surfaces and edgeuses connected to this intersection point
    for (ii, &face) in faces.iter().enumerate() {
        let surf = handles.face_surface(face);

        // when pSurf is not in sSurfaces list - add it
        if !out.surfaces.contains(&surf) {
            // (the faces of a Boolean mate are not reached: see above)
            if let Some(eu) = edge::edgeuse_of_face(b, e, Some(face))? {
                out.surfaces.push(surf);
                out.edgeuses.push(Some(handles.edgeuse(eu)));
                out.uv_curves.push(trim_curve(eu)?);
            }
            out.edge_surface_count += 1;
        }

        // remember an edge->Surface which is shared with the curve (nonManifold may
        // have 2); upstream compares the pointers, so two NULLs match
        if surf == surface1 || surf == surface2 {
            common_surf = Some(ii);
        }

        // when we are done with edgeSurfaces
        if Some(ii) == edge_face_count.checked_sub(1) {
            if let Some(common) = common_surf.filter(|&c| c < edge_face_count) {
                // add the commonSurf a 2nd time
                let face = *faces
                    .get(common)
                    .ok_or_else(|| SolidError::invalid_input(OP_CCI, "face index"))?;
                let surf = handles.face_surface(face);
                if let Some(eu) = edge::edgeuse_of_face(b, e, Some(face))? {
                    out.surfaces.push(surf);
                    out.edgeuses.push(Some(handles.edgeuse(eu)));
                    out.uv_curves.push(trim_curve(eu)?);
                    out.edge_surface_count += 1;
                }
            }
        }
    } // end iter every face attached to this edge

    // all done
    Ok(out)
}

/// The inputs of [`apply_point_solution`] other than the curves and the solution.
#[derive(Debug, Clone, Copy)]
pub struct ApplyPointSolutionInput<'a> {
    /// `SmTol::GetSrcZoneTol3d(pCurveClassification)` (see [`src_zone_tol_3d_of`]).
    pub src_zone_tol_3d: f64,
    /// `pEdge`: the edge of the B-rep given to [`apply_point_solution`], `None` to
    /// leave the edge curve alone.
    pub edge: Option<EdgeId>,
    /// `pEMate`: the edge's Boolean mate (only edited with `edge`).
    pub emate: Option<EdgeId>,
    /// `pClassCurveSurface1`, `pClassCurveUVPoint1`.
    pub class_curve_surface1: Option<(&'a Surface, Point2)>,
    /// `pClassCurveSurface2`, `pClassCurveUVPoint2`.
    pub class_curve_surface2: Option<(&'a Surface, Point2)>,
    /// `pEdgeCurveSurface1`, `pEdgeCurveUVPoint1`.
    pub edge_curve_surface1: Option<(&'a Surface, Point2)>,
    /// `pEdgeCurveSurface2`, `pEdgeCurveUVPoint2`.
    pub edge_curve_surface2: Option<(&'a Surface, Point2)>,
}

/// The result of [`apply_point_solution`].
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(clippy::struct_excessive_bools)] // upstream's SmBoolean out-params
pub struct ApplyPointSolution {
    /// `SM_SUCCESS` (`true`) or `SM_ERR` (no surface point could be evaluated).
    pub success: bool,
    /// `rSSSPoint` out: the surf/surf/surf point (the caller's value unless set).
    pub sss_point: Point3,
    /// `bSSSPointFlag`: use the surf/surf/surf point.
    pub sss_point_flag: bool,
    /// `bModifiedCurve`: the classification curve was modified.
    pub modified_curve: bool,
    /// `bModifiedEdgeCurve`.
    pub modified_edge_curve: bool,
    /// `bModifiedMateCurve`.
    pub modified_mate_curve: bool,
    /// `dDeviation`: the largest curve / surf/surf/surf point distance.
    pub deviation: f64,
}

/// Moves a classification-curve / edge intersection onto the surf/surf/surf point of
/// its surfaces (`sm_ApplyPointSolution`): the point is the first available surface
/// point; the classification curve (when given; it needs both its surfaces) is made to
/// pass through it ([`set_xsect_curve_through_point`]) and the solution's first
/// parameter moves to it; then the edge of `brep` (when given) and its mate are made
/// to pass through it ([`set_xsect_edge_through_point`] with the edge surfaces given)
/// and the solution's second parameter moves to the edge's. `sss_point` is the
/// caller's value of the output point.
///
/// # Errors
///
/// [`SolidError::InvalidInput`] for a classification curve without both of its
/// surfaces (upstream dereferences the missing one) or an edge without `brep`; those
/// of [`set_xsect_curve_through_point`] and [`set_xsect_edge_through_point`].
pub fn apply_point_solution<S: Copy>(
    k: &Kernel,
    src_curve: Option<&mut Curve>,
    brep: Option<&mut Brep>,
    input: &ApplyPointSolutionInput<'_>,
    solution: &mut Solution<S>,
    sss_point: Point3,
) -> Result<ApplyPointSolution> {
    // init output
    let mut out = ApplyPointSolution {
        success: false,
        sss_point,
        sss_point_flag: false,
        modified_curve: false,
        modified_edge_curve: false,
        modified_mate_curve: false,
        deviation: 0.0,
    };

    // locals
    let src_zone_tol_3d = input.src_zone_tol_3d;
    let mut t_deviation = 0.0;

    // let rSSSPoint = pClassCurveSurface1 evaluation at the intersection point
    let has_src = src_curve.is_some();
    let eval = if let (true, Some((s, uv))) = (has_src, input.class_curve_surface1) {
        Some(s.evaluate_point(uv))
    } else if let (true, Some((s, uv))) = (has_src, input.class_curve_surface2) {
        Some(s.evaluate_point(uv))
    } else if let (true, Some((s, uv))) = (
        input.edge.is_some() || input.emate.is_some(),
        input.edge_curve_surface1,
    ) {
        Some(s.evaluate_point(uv))
    } else {
        None
    };
    match eval {
        None => return Ok(out),
        Some(Ok(p)) => out.sss_point = p,
        Some(Err(e)) if e.is_unported() => return Err(e),
        Some(Err(SolidError::Upstream { code, .. })) if code == SM_ERR => return Ok(out),
        Some(Err(_)) => {}
    }

    // set closest point on curve to surf/surf/surf intersection point
    if let Some(src_curve) = src_curve {
        // set up sm_SetXSectCurveThroughPoint arguments
        let param = solution.start.param(0)?; // init guess = curve/curve xsect solution
        let (Some((s1, uv1)), Some((s2, uv2))) =
            (input.class_curve_surface1, input.class_curve_surface2)
        else {
            return Err(SolidError::invalid_input(
                OP_APPLY,
                "classification curve without both surfaces",
            ));
        };

        // ensure surf/surf/surf solution is through point on classCurve
        let r = set_xsect_curve_through_point(
            k,
            src_curve,
            src_zone_tol_3d,
            &mut [],
            &[s1, s2],
            &[uv1, uv2],
            param,
            out.sss_point,
        )?;
        out.modified_curve = r.modified_curve;
        out.sss_point = r.point_3d;

        // inform the public - remove obsolete UVTrimCurves
        if out.modified_curve {
            out.sss_point_flag = true;
        }

        // move pSrcCurve3d parameter in rSolution to surf/surf/surf xsect point
        *solution.start.param_mut(0)? = r.param;

        // increase deviation to include distance to this curve from rSSSPoint
        // (dTDeviation keeps its value when it is not set)
        t_deviation = r.deviation.unwrap_or(t_deviation);
        out.deviation = max2(out.deviation, t_deviation);
    }

    // set closest point on pEdge to surf/surf/surf intersection point
    if let Some(e) = input.edge {
        let Some(b) = brep else {
            return Err(SolidError::invalid_input(OP_APPLY, "edge without B-rep"));
        };

        // set arguments for sm_SetXSectEdgeThroughPoint() call.
        let param = solution.start.param(1)?;
        let mut surfaces: Vec<&Surface> = Vec::new();
        let mut uv_points: Vec<Vec2> = Vec::new();
        if let Some((s, uv)) = input.edge_curve_surface1 {
            surfaces.push(s);
            uv_points.push(uv);
        }
        if let Some((s, uv)) = input.edge_curve_surface2 {
            surfaces.push(s);
            uv_points.push(uv);
        }

        // fix the pEdge and its UVTrimCurves - edge is only modified if it is piecewise C0
        let r = set_xsect_edge_through_point(k, b, e, out.sss_point, &surfaces, &uv_points, param)?;
        out.modified_edge_curve = r.modified_curve;

        // move pEdge parameter in rSolution to surf/surf/surf xsect point
        *solution.start.param_mut(1)? = r.param;

        // increase deviation to include distance to this edge from rSSSPoint
        t_deviation = r.deviation.unwrap_or(t_deviation);
        out.deviation = max2(out.deviation, t_deviation);

        // inform the public - remove obsolete UVTrimCurves
        if out.modified_edge_curve {
            out.sss_point_flag = true;
        }

        // when there is an associated pEMate
        if let Some(emate) = input.emate {
            // use edge's solution to get a good guess on the mate
            let param1 = edge::interval(b, e)?
                .inversion(r.param)
                .parameter
                .unwrap_or(UNDEF_DOUBLE);
            let param1 = edge::interval(b, emate)?.evaluate(param1);

            // fix the pEMate and its UVTrimCurves
            let rm = set_xsect_edge_through_point(
                k,
                b,
                emate,
                out.sss_point,
                &surfaces,
                &uv_points,
                param1,
            )?;
            out.modified_mate_curve = rm.modified_curve;

            // increase deviation to include distance to this edge from rSSSPoint
            t_deviation = rm.deviation.unwrap_or(t_deviation);
            out.deviation = max2(out.deviation, t_deviation);
        }
    }

    // all done
    out.success = true;
    Ok(out)
}

/// Makes every interval whose interior classifies to an edge and whose start or end
/// classifies to a vertex use a vertex of that edge
/// (`sm_MakeConsistentClassifications`).
///
/// # Errors
///
/// [`SolidError::Unported`] for such an interval: the edge's vertices
/// (`SmEdge::GetVertex`, unit `topology.edge`).
pub fn make_consistent_classifications<O: ClassObject>(
    classification: &mut CurveClassification<O>,
) -> Result<()> {
    // for every interval
    for ivl in classification.intervals() {
        // If interval classifies to an Edge
        if ivl.mid.point_class() == PointClassType::Edge
            && (ivl.start.point_class() == PointClassType::Vertex
                || ivl.end.point_class() == PointClassType::Vertex)
        {
            // UNPORTED(topology.edge): SmEdge::GetVertex
            return Err(SolidError::unported("SmEdge::GetVertex", "topology.edge"));
        }
    }
    Ok(())
}

/// The angle in `[0, PI/2]` between the tangents of two curves at their parameters
/// (`sm_GetCurveCurveAngle`); `0` without either curve or when either tangent is
/// shorter than the scaled zero of the first point. `deviation` is only used by debug
/// code upstream.
///
/// # Errors
///
/// [`SolidError::Unported`] from the evaluations.
pub fn get_curve_curve_angle(
    this_curve: Option<&Curve>,
    this_param: f64,
    other_curve: Option<&Curve>,
    other_param: f64,
    deviation: f64,
) -> Result<f64> {
    let _ = deviation;
    // no work - no curves
    let (Some(this_curve), Some(other_curve)) = (this_curve, other_curve) else {
        return Ok(0.0);
    };

    // Evaluate the Curve locations
    let this_pv = curve_evaluate_1(this_curve, this_param)?;
    let other_pv = curve_evaluate_1(other_curve, other_param)?;

    // check for zero-length tangents.
    let scaled_zero = this_pv[0].scaled_zero();
    if this_pv[1].length() < scaled_zero {
        return Ok(0.0);
    }
    if other_pv[1].length() < scaled_zero {
        return Ok(0.0);
    }

    // get the angle between the tangents
    let mut angle = this_pv[1].angle_between(other_pv[1]).unwrap_or(0.0);

    // return angle from 0 to PI/2
    if angle < 0.0 {
        angle = -angle;
    }
    if angle > PI / 2.0 {
        angle = (PI - angle).abs();
    }

    // all done
    Ok(angle)
}

/// Tightens or culls the UV curve / trim curve intersections `solutions` with the
/// equivalent 3D intersections (`sm_CullOrRefineCCIIn3d`): a point solution takes the
/// 3D curve/curve intersection of the UV curve on `surface` with `curve_2` when that is
/// tighter, and is removed when there is none within `xsect_tol_3d`; a range solution
/// is removed when the curves are farther apart than `xsect_tol_3d`. As upstream, the
/// solution after a removed one is skipped.
///
/// # Errors
///
/// Those of the 3D intersection and distance computations;
/// [`SolidError::Unported`] for a surface that is not a B-spline surface (the ported
/// `SmCrvOnSurf` holds only an `SmBSplineSurface`, unit `curve.crv_on_surf`).
pub fn cull_or_refine_cci_in_3d<S: SolutionObject>(
    k: &Kernel,
    surface: &Surface,
    uv_curve_1: &Curve,
    interval_1: &Extent1d,
    curve_2: &Curve,
    interval_2: &Extent1d,
    xsect_tol_3d: f64,
    solutions: &mut SolutionArray<S>,
) -> Result<()> {
    // project UVCurve into 3d space
    let SurfaceKind::BSpline(bs) = surface.kind() else {
        // DISPATCH GAP: SmCrvOnSurf over a non-B-spline SmSurface (curve.crv_on_surf,
        // partial; no UNPORTED marker: the ported CrvOnSurf holds an SmBSplineSurface,
        // not any Surface, as recorded in curve/crv_on_surf.rs and plan/findings.md)
        return Err(SolidError::unported(
            "SmSurface (non-B-spline) in SmCrvOnSurf",
            "curve.crv_on_surf",
        ));
    };
    let curve_1 = Curve::from(CrvOnSurf::new(k, uv_curve_1.clone(), bs.clone(), None, 0)?);

    // for every solution
    let mut i = 0;
    while i < solutions.len() {
        let Some(sol) = solutions.get(i).copied() else {
            break;
        };

        // when the solution is a point (not an interval)
        if sol.solution_type == SolutionType::SingleValue {
            // get the 3dCurve/3Dcurve intersection
            let found = local_curve_intersect(
                &curve_1,
                interval_1,
                curve_2,
                interval_2,
                xsect_tol_3d,
                sol.start.param(0)?,
                sol.start.param(1)?,
            )?;

            // when 3dCurve/3dCurve intersection is tighter than 2dCurve/2dCurve intersection
            if let Some(x) = found {
                if x.deviation < sol.start.solution_value {
                    if let Some(s) = solutions.get_mut(i) {
                        s.start.solution_value = x.deviation;
                        *s.start.param_mut(0)? = x.my_param;
                        *s.start.param_mut(1)? = x.other_param;
                    }
                }
            }

            // when 3dCurve/3dCurve intersection is not good enough
            if found.is_none_or(|x| x.deviation > xsect_tol_3d) {
                solutions.remove_at(i, 1);
            }
        }
        // when the solution is a coincident curve pair
        else if sol.solution_type == SolutionType::RangeOfValues {
            // get quick max edge/UVTrimCurve 3d distance estimate at 8 sample points
            let sol_interval_1 = Extent1d::from_min_max(sol.start.param(0)?, sol.end.param(0)?);
            let mut max_dist = curve_1
                .curve_max_distance_between(
                    k,
                    &sol_interval_1,
                    curve_2,
                    sol.start.param(1)?,
                    sol.end.param(1)?,
                    8,
                    None,
                )?
                .distance;

            // when the quick estimate is good but not very tight
            if max_dist < xsect_tol_3d && max_dist > xsect_tol_3d / 100.00 {
                // get max dist value possible between curves by setting samplePointCount = 0
                max_dist = curve_1
                    .curve_max_distance_between(
                        k,
                        &sol_interval_1,
                        curve_2,
                        sol.start.param(1)?,
                        sol.end.param(1)?,
                        0,
                        None,
                    )?
                    .distance;
            }

            // when max edge/UVTrimCurve 3d distance is not good enough
            if max_dist > xsect_tol_3d {
                solutions.remove_at(i, 1);
            }
        }
        i += 1;
    }

    // all done
    Ok(())
}

/// Classifies a 3D point expected on face `f` of `b` to it (`sm_ComputeFacePointClass`):
/// the point is projected onto the face's surface over the face's UV domain (within
/// `src_zone_tol_3d`, then 10 and 100 times the intersection tolerance of
/// `src_zone_tol_3d` and the face's), and the nearest projection is classified against
/// the face boundary into `point_class` ([`point_classify`]; its status is ignored, as
/// upstream). Returns `SM_SUCCESS` (`true`), or `SM_ERR` (`false`) when the point does
/// not project.
///
/// # Errors
///
/// [`SolidError::null_pointer`] for a face without surface (`NER`); those of the
/// surface point solve (`SER`) and of the topology accessors for a dangling id.
pub fn compute_face_point_class(
    k: &Kernel,
    b: &Brep,
    point: Point3,
    src_zone_tol_3d: f64,
    f: FaceId,
    point_class: &mut PointClassification<EntityRef>,
) -> Result<bool> {
    // locals
    let surface = face::surface(b, f)?.ok_or(SolidError::null_pointer(OP_FACE_CLASS))?;

    // Project Point to Face->Surface. Upstream switches the surface cache's point
    // testing off around the solves; the port's solver tests no trim boundaries for a
    // surface without face owner, and one with a face owner is `Unported` there.
    let _cache = surface_cache::<()>(k, surface, None)?;
    let xsect_tol_3d = xsect_tol_3d_from_zones(k, src_zone_tol_3d, face::tolerance(b, f)?);
    let uv_domain = face::uv_domain(b, f)?;
    let solve = |tol: f64| {
        surface.global_point_solve::<crate::surface::surface::p4::NoObject>(
            k,
            Some(crate::surface::surface::p4::NoObject),
            &uv_domain,
            SolverOperation::Intersect,
            point,
            tol,
            None,
            SolutionRequested::All,
        )
    };
    let mut solutions = solve(src_zone_tol_3d)?;

    // If global point solve fails, try again with a bigger tolerance.
    if solutions.is_empty() {
        solutions = solve(10.0 * xsect_tol_3d)?;

        // If point still fails to project, give it one more try.
        if solutions.is_empty() {
            solutions = solve(100.0 * xsect_tol_3d)?;

            // signal error and give up if point still fails to project
            if solutions.is_empty() {
                return Ok(false);
            }
        }
    }

    // Find the best solution, and classify it to the face.
    let mut best_dist = BIG_DOUBLE;
    let mut uv = Point2::new(0.0, 0.0);
    for sol in solutions.as_slice() {
        if sol.start.solution_value < best_dist {
            best_dist = sol.start.solution_value;
            uv = Point2::new(sol.start.param(0)?, sol.start.param(1)?);
        }
    }

    ignored(point_classify(
        k,
        b,
        f,
        uv,
        point_class,
        PointClassifyOptions {
            src_zone_tol_3d,
            do_boundary_intersections: true,
            use_uv_classification: true,
            ..PointClassifyOptions::default()
        },
    ))?;

    // all done
    Ok(true)
}

/// The tolerance within which two points of a classification curve coincide, in the
/// space the classification works in (`sm_GetXSectTol2dOr3d_ForClassifyPair`): for a
/// 3D curve the intersection tolerance of the two zone tolerances; for a UV curve on
/// `surface_1` the two tolerances mapped to UV space; for a UV curve without surface
/// `SM_EFF_ZERO_PARAM`.
///
/// # Errors
///
/// [`SolidError::Unported`] from the surface evaluation of `SmTol::MapTo2d`, and for
/// a curve of dimension other than 2 or 3 on a surface (`SmSurface::DropPoint`, unit
/// `surface.surface_p5`).
pub fn xsect_tol_2d_or_3d_for_classify_pair(
    k: &Kernel,
    src_curve: &Curve,
    surface_1: Option<&Surface>,
    src_curve_or_obj1_param: f64,
    src_curve_or_obj1_zone_tol_3d: f64,
    obj2_src_curve_param: f64,
    obj2_zone_tol_3d: f64,
) -> Result<f64> {
    // tolerance cases:
    if src_curve.dim() == 3 {
        // Curve3d
        Ok(xsect_tol_3d_from_zones(
            k,
            src_curve_or_obj1_zone_tol_3d,
            obj2_zone_tol_3d,
        ))
    } else if let Some(surface_1) = surface_1 {
        // Curve2d with m_cpSurface1
        let obj2_zone_tol_2d = zone_tol3d(
            k,
            map_to_2d_on_curve(
                k,
                obj2_zone_tol_3d,
                obj2_src_curve_param,
                src_curve,
                surface_1,
            )?,
        );
        let obj1_zone_tol_2d = zone_tol3d(
            k,
            map_to_2d_on_curve(
                k,
                src_curve_or_obj1_zone_tol_3d,
                src_curve_or_obj1_param,
                src_curve,
                surface_1,
            )?,
        );
        Ok(xsect_tol_3d_from_zones(
            k,
            obj1_zone_tol_2d,
            obj2_zone_tol_2d,
        ))
    } else {
        // Curve2d No m_cpSurface1
        Ok(xsect_tol3d(k, eff_zero_param()))
    }
}

/// The dimension of a topology object (`sm_ObjectDimension`): face 2, edge 1, vertex
/// 0, anything else (or none) 4.
#[must_use]
pub fn object_dimension<O: ClassObject>(object: Option<O>) -> i32 {
    match object.map(|o| o.class_kind()) {
        Some(ClassObjectKind::Face) => 2,
        Some(ClassObjectKind::Edge) => 1,
        Some(ClassObjectKind::Vertex) => 0,
        _ => 4,
    }
}

// ---- callees in units not ported yet ---------------------------------------------------

/// `SmSurface::LocalPointSolve(domain, SM_SO_MINIMIZE, point, guess, found, sol)`:
/// the gap and UV point of the answer, if found.
fn surface_local_point_solve(
    k: &Kernel,
    surface: &Surface,
    point: Point3,
    guess: Point2,
) -> Result<Option<(f64, Point2)>> {
    let domain = surface.natural_uv_domain();
    // (the status is ignored upstream, only bFoundAnswer is read)
    let Some(r) = ignored(surface.local_point_solve::<()>(
        k,
        &domain,
        SolverOperation::Minimize,
        point,
        guess,
    ))?
    else {
        return Ok(None);
    };
    if r.found {
        Ok(Some((
            r.solution.start.solution_value,
            Point2::new(r.solution.start.param(0)?, r.solution.start.param(1)?),
        )))
    } else {
        Ok(None)
    }
}

/// `SmSurface::ApproximateSize`: upstream ignores the status, leaving the size zero.
fn surface_approximate_size(surface: &Surface) -> Result<Vec2> {
    match surface.approximate_size(None) {
        Ok(size) => Ok(size),
        Err(e @ SolidError::Unported { .. }) => Err(e),
        Err(_) => Ok(Vec2::ZERO),
    }
}

/// `SmSurface::GlobalPointSolve(domain, SM_SO_MINIMIZE, point, 0.001 * |size|, &maxDrop,
/// SM_SR_SINGLE, sols)` over the natural domain (`maxDrop` is `1.0`): the gap and UV
/// point of the first solution, if any. The status is ignored (`SmCurveClass.cpp:1045`
/// checks only the solution count), so a failed solve is an empty result. Upstream's
/// temporary switch-off of the surface cache's point test (`m_bPointTestEnabled`,
/// `:1040`) is moot here: the solver builds no cache for a surface that owns no face,
/// and a face-owned one is `Unported` in the solver (`UNPORTED(query.face_p2)`; the
/// switch must be added with it).
fn surface_global_point_solve(
    k: &Kernel,
    surface: &Surface,
    size: Vec2,
    point: Point3,
) -> Result<Option<(f64, Point2)>> {
    let tol = 0.001 * size.length();
    let solutions = ignored(
        surface.global_point_solve::<crate::surface::surface::p4::NoObject>(
            k,
            Some(crate::surface::surface::p4::NoObject),
            &surface.natural_uv_domain(),
            SolverOperation::Minimize,
            point,
            tol,
            Some(1.0),
            SolutionRequested::Single,
        ),
    )?
    .unwrap_or_default();
    Ok(solutions.as_slice().first().map(|s| {
        (
            s.start.solution_value,
            Point2::new(
                s.start.parameters.first().copied().unwrap_or(0.0),
                s.start.parameters.get(1).copied().unwrap_or(0.0),
            ),
        )
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq)]
    struct H(usize, ClassObjectKind);

    impl ClassObject for H {
        fn class_kind(&self) -> ClassObjectKind {
            self.1
        }
    }

    fn touch(dir: CrvDirOnFaceType, to: usize) -> TouchData<H> {
        TouchData {
            connected_to: Some(H(to, ClassObjectKind::Loop)),
            loop_dir: dir,
            loop_index: 0,
            param: 0.0,
            pt_classify_index: 0,
            pt_classify: None,
        }
    }

    #[test]
    fn winding_count() {
        use CrvDirOnFaceType as D;
        let l = [
            touch(D::In, 0),
            touch(D::Out, 0),
            touch(D::Out, 1),
            touch(D::In, 1),
            touch(D::In, 0),
            touch(D::Out, 0),
        ];
        assert_eq!(get_winding_count(&l).ok(), Some((1, OrientType::Opposite)));
        assert!(get_winding_count(&l[..3]).is_err());
        assert_eq!(
            get_connected_touches(Some(H(1, ClassObjectKind::Loop)), &l).len(),
            2
        );
    }

    #[test]
    fn set_size_and_remove() {
        let k = Kernel::default();
        let mut c: CurveClassification<H> = CurveClassification::new(&k);
        assert_eq!(c.interval().min(), 0.0);
        assert!(c.remove_at(0).is_err());
        c.set_size(&k, 3).ok();
        assert_eq!(c.size(), 3);
        // default constructed: system zone tolerance
        assert_eq!(
            c.at(0).map(|i| i.start.src_zone_tol_3d()).ok(),
            Some(1.0e-5)
        );
        c.remove_at(7).ok();
        assert_eq!(c.size(), 2);
        c.set_size(&k, 4).ok();
        // reset: uninit tolerance
        assert_eq!(
            c.at(3).map(|i| i.start.src_zone_tol_3d()).ok(),
            Some(UNINIT_TOL)
        );
        assert!(c.sub_interval(4).is_err());
        assert!(c.set_size(&k, MAX_INTERVALS + 1).is_err());
        c.set_size(&k, 0).ok();
        c.set_size(&k, 1).ok();
        assert_eq!(
            c.at(0).map(|i| i.start.src_zone_tol_3d()).ok(),
            Some(1.0e-5)
        );
    }

    #[test]
    fn squeeze_end_and_references() {
        let k = Kernel::default();
        let mut c: CurveClassification<H> = CurveClassification::new(&k);
        c.set_size(&k, 2).ok();
        let v = H(5, ClassObjectKind::Vertex);
        if let Ok(i) = c.at_mut(0) {
            i.start.set_class_object(PointClassType::Face, None);
            i.end.set_class_object(PointClassType::Vertex, Some(v));
        }
        assert_eq!(
            c.find_best_squeeze_end(0).ok(),
            Some(IntervalPosition::Start)
        );
        assert_eq!(
            c.at(0).map(|i| i.start.point_class()).ok(),
            Some(PointClassType::Vertex)
        );
        assert!(c.is_referenced(Some(v)));
        assert!(!c.is_referenced(None));
        assert_eq!(object_dimension(Some(v)), 0);
        assert_eq!(object_dimension::<H>(None), 4);
    }

    // ---- B-rep topology helpers ----------------------------------------------------

    struct Handles;

    impl TopologyHandles<H> for Handles {
        fn face_surface(&self, f: FaceId) -> Option<H> {
            Some(H(100 + f.index() as usize, ClassObjectKind::Surface))
        }
        fn edgeuse(&self, eu: EdgeuseId) -> H {
            H(eu.index() as usize, ClassObjectKind::Edgeuse)
        }
        fn uv_trim_curve(&self, eu: EdgeuseId) -> H {
            H(eu.index() as usize, ClassObjectKind::Curve)
        }
    }

    fn plane_z() -> Surface {
        Surface::from(crate::surface::plane::Plane::from_normal(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ))
    }

    /// A B-rep with one use-less edge (on `curve`) and one use-less face on the plane
    /// `z = 0` over its natural domain.
    fn stand_in_brep(curve: Option<Curve>) -> (Brep, EdgeId, FaceId) {
        let mut b = Brep::new();
        let shell = b.add_shell().unwrap();
        let v = b.add_vertex(Point3::new(0.0, 0.0, 0.0)).unwrap();
        let interval = curve
            .as_ref()
            .map_or_else(Extent1d::new, Curve::natural_interval);
        let e = b
            .add_edge_record(crate::topology::Edge {
                vertices: [v, v],
                edgeuses: Vec::new(),
                interval,
                tolerance: 1e-5,
                curve,
            })
            .unwrap();
        let surface = plane_z();
        let uv_domain = surface.natural_uv_domain();
        let f = b
            .add_face_record(crate::topology::Face {
                surface: Some(surface),
                uv_domain,
                tolerance: 1e-5,
                ..crate::topology::Face::new(shell)
            })
            .unwrap();
        (b, e, f)
    }

    #[test]
    fn cci_geometry_of_an_edge_without_faces() {
        let k = Kernel::default();
        let (b, e, _) = stand_in_brep(None);
        let mut cc = CurveClassification::<H>::new(&k);
        let s1 = H(1, ClassObjectKind::Surface);
        let s2 = H(2, ClassObjectKind::Surface);
        cc.set_intersect_data(Some(s1), Some(s2), None, true, None);
        let g = get_cci_geometry(&cc, &b, e, &Handles).unwrap();
        assert_eq!(g.surfaces, vec![Some(s1), Some(s2)]);
        assert_eq!(g.edgeuses, vec![None, None]);
        assert_eq!(g.edge_surface_count, 0);
        assert_eq!(g.emate, None);

        // a Boolean's classification needs the topology intersector
        cc.set_intersect_data(
            Some(s1),
            Some(s2),
            None,
            true,
            Some(H(9, ClassObjectKind::Other)),
        );
        assert!(matches!(
            get_cci_geometry(&cc, &b, e, &Handles),
            Err(SolidError::Unported { .. })
        ));
    }

    #[test]
    fn face_point_class_near_and_far_from_a_plane_face() {
        let k = Kernel::default();
        let (b, _, f) = stand_in_brep(None);
        let mut pc = PointClassification::new(1e-5);
        let ok = compute_face_point_class(&k, &b, Point3::new(0.25, 0.5, 1e-7), 1e-5, f, &mut pc)
            .unwrap();
        assert!(ok);
        // a face without loops has nothing inside
        assert_eq!(pc.point_class(), PointClassType::Unknown);
        // nothing near
        let mut pc = PointClassification::new(1e-5);
        let far = compute_face_point_class(&k, &b, Point3::new(0.25, 0.5, 3.0), 1e-5, f, &mut pc)
            .unwrap();
        assert!(!far);

        // a face without surface
        let mut b2 = b.clone();
        b2.face_mut(f).unwrap().surface = None;
        let mut pc = PointClassification::new(1e-5);
        assert!(
            compute_face_point_class(&k, &b2, Point3::new(0.0, 0.0, 0.0), 1e-5, f, &mut pc)
                .is_err()
        );
    }

    #[test]
    fn edge_through_point_checks_its_inputs() {
        let k = Kernel::default();
        let (mut b, e, _) = stand_in_brep(None);
        let p = plane_z();
        let surfaces = [&p, &p];
        let uvs = [Vec2::new(0.0, 0.0), Vec2::new(0.0, 0.0)];
        // an edge without curve (upstream dereferences it)
        assert!(matches!(
            set_xsect_edge_through_point(
                &k,
                &mut b,
                e,
                Point3::new(0.0, 0.0, 0.0),
                &surfaces,
                &uvs,
                0.0
            ),
            Err(SolidError::InvalidInput { .. })
        ));

        let line = Curve::from(crate::curve::line::Line::bounded(
            Point3::new(0.0, 0.0, 0.0),
            Vec3::new(1.0, 0.0, 0.0),
            &Extent1d::from_min_max(0.0, 1.0),
            &crate::curve::line::BoundedLineOptions::default(),
        ));
        let (mut b, e, _) = stand_in_brep(Some(line));
        // one surface: SM_ERR, nothing written
        let r = set_xsect_edge_through_point(
            &k,
            &mut b,
            e,
            Point3::new(0.5, 0.0, 0.0),
            &surfaces[..1],
            &uvs[..1],
            0.3,
        )
        .unwrap();
        assert!(!r.success && !r.modified_curve && r.deviation.is_none());
        assert_eq!(r.param, 0.3);
        // a point on the edge: projected, nothing to edit
        let r = set_xsect_edge_through_point(
            &k,
            &mut b,
            e,
            Point3::new(0.5, 0.0, 0.0),
            &surfaces,
            &uvs,
            0.3,
        );
        let r = r.unwrap();
        assert!(r.deviation.is_some());
        assert!(!r.modified_curve);
    }
}
