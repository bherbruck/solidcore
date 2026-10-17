//! Runner for `offset.offset_executive_p2` (draft PR #201).
//!
//! Upstream: `source/SMLib/src/SmOffsetExecutive.cpp`,
//! `source/SM_API/src/SmApiBrep.cpp`; native adapter:
//! `harness/cpp/shim/offset_offset_executive_p2.cpp` (snapshot protocol v1).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`Runner`] static offset/shell | `SmOffsetExecutive::OffsetBrep`, `ShellBrep` |
//! | [`Runner`] member solid/extended | `DoSolidOffset`, `DoExtendedOffset` |
//! | [`Runner`] API shell/offset | `SmApiShellBrepFull`, `SmApiOffsetBrepFull` |
//!
//! Deviations: independent serde/native DTO conversion and independent Rust input
//! construction, not native operation replay. Ordinary operation statuses are
//! INTEGER observations, not `Output::error` (which would suppress geometry/state
//! comparison). ONLY `ffi::Outcome::Rejected` is a constructor rejection; all FFI
//! guard/transport/layout errors are harness errors. Actual kernel `Unported`
//! errors propagate unchanged, including calls before tolerance guards/copying.
//! No topology clone stands in for the API shell's missing copy constructor.
//!
//! Exact current parity scope: member null (1006), API null (1007), and API shell
//! foreign/null selections (1007), including mixed/duplicate source-face-zero
//! selections and independent arenas whose local face-zero ids collide. Compare
//! all exposed executive scalar getters/flags, Boolean context before/after,
//! ordered selector identities, input/result lifetime/ownership and presence.
//! Input geometry projection compares B-rep tolerance, representable counts and
//! analytic kinds; sheets additionally compare face/natural domains and three
//! evaluated points/normals; wires compare natural/trim intervals, endpoint
//! evaluations and vertex bounds. Per-entity effective tolerances, region void /
//! infinite-region / editing metadata and traversal adjacency are NOT invented.
//! The COMPLETE native integer/numeric input blocks must remain bitwise unchanged
//! on those early paths; independently, Rust checks its complete Debug-visible
//! arena state and samples before/after the SAME owned input, not a reconstruction.
//!
//! Box input scaffolding has real shared vertices/edges, six analytic Plane faces,
//! mated face/loop uses and two region/shell records, but does NOT reproduce native
//! stitching/orientation/UV reparameterization or promise face-index geometric
//! correspondence after stitching. Box parity is deliberately only counts of
//! faces/edges/vertices, analytic kinds, vertex bounds, provenance and purity.
//! Sheet boundary scaffolding follows p1's typed rectangular fixture pattern;
//! its Line boundaries are not claimed equivalent to native isocurve kinds.
//! No topology is globally sorted. No arbitrary returned-Brep shape equivalence
//! is claimed: a newly reachable non-deferred result/temporary fails closed until
//! an output adapter is implemented, rather than guessing its missing metadata.
//! Static/member shell lists currently support source faces only; nullable and
//! foreign lists are exercised through the API's origin-qualified interface.
//! Positive B-rep tolerances avoid the unported automatic `SetTolerance` path;
//! member approximation tolerance still covers zero, negative and -1 sentinel.
//! All numeric keys have ZERO absolute floor; signed-zero inputs are retained.
//!
//! This unit is necessarily PARTIAL: current positive geometry operations reach
//! genuine missing manifold/copy/infinite-region/void/shrink/merge/stitch work.
//! Empty rounded offset still calls `ShrinkGeometry`; static shell queries manifold
//! BEFORE its strict radius guard; valid API shell selections copy BEFORE shelling.
//! Fixtures retain those native paths for future wiring, not manufactured success.
//! Source test references are reductions of tests read at
//! `tests/SM_API_test/src/TestSmBrep.cpp:2544-2758` and
//! `tests/prog_test/src/smOffsetTest.cpp:190-295`, not volume parity assertions.
//! No new input-dependent recursion is introduced.

use serde::{Deserialize, Serialize};
use serde_json::Value;
use solidcore::curve::curve::{Curve, CurveKind};
use solidcore::curve::line::Line;
use solidcore::geom::extent1d::Extent1d;
use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::object::FaceKey;
use solidcore::kernel::Kernel;
use solidcore::math::Vec3;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::nurbs::knots::Side;
use solidcore::offset::offset_executive::p1::{OffsetExecutive, OffsetOperation};
use solidcore::offset::offset_executive::p2::{
    self, OffsetOptions, ShellOptions, SolidOffsetOptions,
};
use solidcore::query::api_queries::BrepRef;
use solidcore::surface::plane::Plane;
use solidcore::surface::surface::{Surface, SurfaceKind};
use solidcore::topology::{
    Brep, BrepId, Edge, EdgeId, Edgeuse, EdgeuseOwner, Face, FaceId, Faceuse, Loop, LoopEdge,
    Loopuse, LoopuseStart, Region, Shell, ShellId, Vertex, VertexId, Vertexuse, VertexuseOwner,
};
use solidcore::SolidError;
use solidcore_ffi::units::offset_offset_executive_p2 as ffi;

use crate::runner::default_cases;
use crate::util::{p2 as point2, v3};
use crate::{Case, CaseContext, HarnessError, Output, Rng, UnitDiff};

/// Minimum reproducible generated inputs, even when `--seeds 0` is requested.
pub const MIN_GENERATED: usize = 240;
/// The six real native entrypoints and their Rust counterparts.
pub struct Runner;

type HResult<T> = Result<T, HarnessError>;
type RResult<T> = Result<T, RunError>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Operation {
    StaticOffset,
    StaticShell,
    Solid,
    Extended,
    ApiShell,
    ApiOffset,
}
impl Operation {
    const fn native(self) -> ffi::Operation {
        match self {
            Self::StaticOffset => ffi::Operation::StaticOffset,
            Self::StaticShell => ffi::Operation::StaticShell,
            Self::Solid => ffi::Operation::Solid,
            Self::Extended => ffi::Operation::Extended,
            Self::ApiShell => ffi::Operation::ApiShell,
            Self::ApiOffset => ffi::Operation::ApiOffset,
        }
    }
    const fn member(self) -> bool {
        matches!(self, Self::Solid | Self::Extended)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Fixture {
    Empty,
    Wire {
        start: [f64; 3],
        end: [f64; 3],
        domain: [f64; 2],
        interval: [f64; 2],
    },
    PlaneSheet {
        origin: [f64; 3],
        x_axis: [f64; 3],
        y_axis: [f64; 3],
        scale: [f64; 2],
        domain: [f64; 4],
    },
    Box {
        origin: [f64; 3],
        size: [f64; 3],
    },
}
impl Fixture {
    fn native(&self) -> ffi::Fixture {
        match *self {
            Self::Empty => ffi::Fixture::EmptyBrep,
            Self::Wire {
                start,
                end,
                domain,
                interval,
            } => ffi::Fixture::Wire {
                start,
                end,
                domain,
                interval,
            },
            Self::PlaneSheet {
                origin,
                x_axis,
                y_axis,
                scale,
                domain,
            } => ffi::Fixture::PlaneSheet {
                origin,
                x_axis,
                y_axis,
                scale,
                domain,
            },
            Self::Box { origin, size } => ffi::Fixture::Box { origin, size },
        }
    }
    const fn face_count(&self) -> usize {
        match self {
            Self::Empty | Self::Wire { .. } => 0,
            Self::PlaneSheet { .. } => 1,
            Self::Box { .. } => 6,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(
    tag = "origin",
    content = "index",
    rename_all = "snake_case",
    deny_unknown_fields
)]
enum Selection {
    Source(usize),
    Foreign(usize),
    Null,
}
impl Selection {
    const fn native(self) -> ffi::FaceSelection {
        match self {
            Self::Source(index) => ffi::FaceSelection::Source(index),
            Self::Foreign(index) => ffi::FaceSelection::Foreign(index),
            Self::Null => ffi::FaceSelection::Null,
        }
    }
}

// Five independent native flags, not alternatives in a state machine.
#[allow(clippy::struct_excessive_bools)]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
struct Config {
    offset_distance: f64,
    member_distance: f64,
    member_approx_tol: f64,
    member_operation: u32,
    inset: bool,
    extend_convex: bool,
    merge_results: bool,
    create_offset_solid: bool,
    skip_stitch_merge: bool,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            offset_distance: 0.1,
            member_distance: 0.1,
            member_approx_tol: 1e-6,
            member_operation: 0,
            inset: false,
            extend_convex: false,
            merge_results: false,
            create_offset_solid: false,
            skip_stitch_merge: false,
        }
    }
}
impl Config {
    fn native(&self) -> ffi::Config {
        ffi::Config {
            offset_distance: self.offset_distance,
            member_distance: self.member_distance,
            member_approx_tol: self.member_approx_tol,
            member_operation: self.member_operation,
            inset: self.inset,
            extend_convex: self.extend_convex,
            merge_results: self.merge_results,
            create_offset_solid: self.create_offset_solid,
            skip_stitch_merge: self.skip_stitch_merge,
        }
    }
    const fn offset(&self) -> OffsetOptions {
        OffsetOptions {
            distance: self.offset_distance,
            extend_convex_edges: self.extend_convex,
            merge_results: self.merge_results,
        }
    }
    const fn shell(&self) -> ShellOptions {
        ShellOptions {
            offset: self.offset(),
            create_offset_solid: self.create_offset_solid,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Inputs {
    operation: Operation,
    source: Option<Fixture>,
    #[serde(default)]
    foreign: Option<Fixture>,
    #[serde(default)]
    faces: Vec<Selection>,
    tolerance: f64,
    #[serde(default)]
    config: Config,
}
impl Inputs {
    fn native(&self) -> ffi::Request {
        ffi::Request {
            operation: self.operation.native(),
            source: self.source.as_ref().map(Fixture::native),
            foreign: self.foreign.as_ref().map(Fixture::native),
            faces: self.faces.iter().copied().map(Selection::native).collect(),
            tolerance: self.tolerance,
            config: self.config.native(),
        }
    }
}

fn bad(message: impl Into<String>) -> HarnessError {
    HarnessError::new(message)
}
fn setup(error: impl core::fmt::Display) -> HarnessError {
    bad(format!("Rust input fixture: {error}"))
}
fn at<T>(slice: &[T], index: usize) -> HResult<&T> {
    slice
        .get(index)
        .ok_or_else(|| bad("fixture/snapshot index outside its traversal"))
}
fn finite(values: &[f64]) -> HResult<()> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(bad("nonfinite fixture scalar"))
    }
}
fn interval(range: [f64; 2], strict: bool) -> HResult<()> {
    finite(&range)?;
    let [a, b] = range;
    if a > b || (strict && a == b) || !(b - a).is_finite() {
        return Err(bad("invalid finite fixture interval"));
    }
    Ok(())
}
fn validate(input: &Inputs) -> HResult<()> {
    let cfg = &input.config;
    finite(&[
        input.tolerance,
        cfg.offset_distance,
        cfg.member_distance,
        cfg.member_approx_tol,
    ])?;
    if input.tolerance <= 0.0 {
        return Err(bad(
            "automatic B-rep tolerance is outside this Rust fixture projection",
        ));
    }
    if cfg.member_operation > 4 || (input.operation.member() && cfg.member_distance < 0.0) {
        return Err(bad("invalid native member constructor parameters"));
    }
    if input.source.is_none()
        && matches!(
            input.operation,
            Operation::StaticOffset | Operation::StaticShell
        )
    {
        return Err(bad(
            "static offset/shell dereferences null before validation",
        ));
    }

    if input.faces.len() > ffi::MAX_SELECTIONS {
        return Err(bad("too many selected faces"));
    }
    for selection in &input.faces {
        let (fixture, index) = match *selection {
            Selection::Source(index) => (input.source.as_ref(), index),
            Selection::Foreign(index) => (input.foreign.as_ref(), index),
            Selection::Null => continue,
        };
        if index >= fixture.map_or(0, Fixture::face_count) {
            return Err(bad("selected face outside its origin fixture"));
        }
    }
    // Match the native/FFI guard: null member/API inputs and API-shell invalid
    // origins return before extension arithmetic, but still validate every DTO.
    let invalid_api_selection = input.operation == Operation::ApiShell
        && input
            .faces
            .iter()
            .any(|face| !matches!(face, Selection::Source(_)));
    if input.source.is_some()
        && !invalid_api_selection
        && (input.operation == Operation::Extended || cfg.extend_convex)
    {
        let magnitude = if input.operation.member() {
            cfg.member_distance
        } else {
            cfg.offset_distance.abs()
        };
        finite(&[5.0 * magnitude])?;
    }
    for fixture in input.source.iter().chain(&input.foreign) {
        match *fixture {
            Fixture::Empty => {}
            Fixture::Wire {
                start,
                end,
                domain,
                interval: trim,
            } => {
                finite(&start)?;
                finite(&end)?;
                interval(domain, true)?;
                interval(trim, false)?;
                let [d0, d1] = domain;
                let [t0, t1] = trim;
                if t0 < d0 || t1 > d1 {
                    return Err(bad("wire trim outside curve domain"));
                }
                finite(&[(v3(end) - v3(start)).length_squared()])?;
            }
            Fixture::PlaneSheet {
                origin,
                x_axis,
                y_axis,
                scale,
                domain,
            } => {
                finite(&origin)?;
                finite(&x_axis)?;
                finite(&y_axis)?;
                finite(&scale)?;
                let [u0, v0, u1, v1] = domain;
                interval([u0, u1], true)?;
                interval([v0, v1], true)?;
                let [su, sv] = scale;
                finite(&[
                    v3(x_axis).length_squared(),
                    v3(y_axis).length_squared(),
                    su * u0,
                    su * u1,
                    sv * v0,
                    sv * v1,
                ])?;
            }
            Fixture::Box { origin, size } => {
                finite(&origin)?;
                finite(&size)?;
                for (a, b) in origin.iter().zip(size) {
                    finite(&[a + b, b * b])?;
                }
            }
        }
    }
    Ok(())
}

#[derive(Debug)]
enum RunError {
    Harness(HarnessError),
    Kernel(SolidError),
}
impl From<HarnessError> for RunError {
    fn from(error: HarnessError) -> Self {
        Self::Harness(error)
    }
}
impl From<SolidError> for RunError {
    fn from(error: SolidError) -> Self {
        Self::Kernel(error)
    }
}
// Preserve all ordinary codes; in particular 1001 != 1006 != 1007. A real
// Unported must escape even when the static offset collapses ordinary errors.
fn operation<T>(result: solidcore::Result<T>) -> RResult<(i32, Option<T>)> {
    match result {
        Ok(value) => Ok((ffi::NATIVE_SUCCESS, Some(value))),
        Err(SolidError::Upstream { code, .. }) => Ok((code, None)),
        Err(error) => Err(RunError::Kernel(error)),
    }
}
fn operation_type(code: u32) -> HResult<OffsetOperation> {
    match code {
        0 => Ok(OffsetOperation::SolidOffset),
        1 => Ok(OffsetOperation::NonManifoldOffset),
        2 => Ok(OffsetOperation::Shell),
        3 => Ok(OffsetOperation::LocalOperation),
        4 => Ok(OffsetOperation::MachineOffset),
        _ => Err(bad("member operation outside native enum")),
    }
}
fn executive(cfg: &Config) -> HResult<OffsetExecutive> {
    let mut executive = OffsetExecutive::new(
        operation_type(cfg.member_operation)?,
        cfg.member_distance,
        cfg.member_approx_tol,
    );
    executive.set_inset(cfg.inset);
    executive.set_extend_convex_edges(cfg.extend_convex);
    executive.set_merge_results(cfg.merge_results);
    executive.set_create_offset_solid(cfg.create_offset_solid);
    Ok(executive)
}
fn executive_state(executive: &OffsetExecutive) -> ffi::ExecutiveState {
    ffi::ExecutiveState {
        operation: executive.operation() as i32,
        inset: executive.inset(),
        extend_convex: executive.extend_convex_edges(),
        merge_results: executive.merge_results(),
        create_offset_solid: executive.create_offset_solid(),
        distance: executive.offset_distance(),
        approx_tol: executive.approximation_tol_3d(),
        tangency_degrees: executive.tangency_tol_degrees(),
    }
}

fn line(start: Vec3, end: Vec3, domain: [f64; 2], dimension: u64) -> HResult<Curve> {
    let domain = Extent1d::from_min_max(domain[0], domain[1]);
    Ok(Curve::from(
        Line::create_line_segment(dimension, start, end, Some(&domain)).map_err(setup)?,
    ))
}
fn add_vertex(brep: &mut Brep, point: Vec3) -> HResult<VertexId> {
    brep.add_vertex_record(Vertex {
        point,
        tolerance: brep.tolerance(),
        ..Vertex::default()
    })
    .map_err(setup)
}
fn add_edge(
    brep: &mut Brep,
    vertices: [VertexId; 2],
    curve: Curve,
    trim: [f64; 2],
) -> HResult<EdgeId> {
    brep.add_edge_record(Edge {
        vertices,
        curve: Some(curve),
        interval: Extent1d::from_min_max(trim[0], trim[1]),
        edgeuses: Vec::new(),
        tolerance: brep.tolerance(),
    })
    .map_err(setup)
}
fn add_use(
    brep: &mut Brep,
    edge: EdgeId,
    vertex: VertexId,
    owner: EdgeuseOwner,
    orientation: OrientType,
    uv_trim_curve: Option<Curve>,
) -> HResult<solidcore::topology::EdgeuseId> {
    let use_id = brep
        .add_edgeuse_record(Edgeuse {
            edge: Some(edge),
            owner,
            orientation,
            uv_trim_curve,
            ..Edgeuse::default()
        })
        .map_err(setup)?;
    let vertexuse = brep
        .add_vertexuse_record(Vertexuse {
            vertex: Some(vertex),
            owner: VertexuseOwner::Edgeuse(Some(use_id)),
            ..Vertexuse::default()
        })
        .map_err(setup)?;
    brep.vertex_mut(vertex)
        .map_err(setup)?
        .vertexuses
        .push(vertexuse);
    brep.edgeuse_mut(use_id).map_err(setup)?.vertexuse = Some(vertexuse);
    brep.edge_mut(edge).map_err(setup)?.edgeuses.push(use_id);
    Ok(use_id)
}
fn add_shell(brep: &mut Brep, region: solidcore::topology::RegionId) -> HResult<ShellId> {
    brep.add_shell_record(Shell {
        region: Some(region),
        ..Shell::default()
    })
    .map_err(setup)
}

// Rectangular typed input scaffold, adapted from p1's face_with_edges, with a
// shared shell rather than four extra wire shells. It is NOT an offset result.
fn rectangular_face(
    brep: &mut Brep,
    surface: Surface,
    shells: [ShellId; 2],
    vertices: [VertexId; 4],
    edges: [(EdgeId, bool); 4],
) -> HResult<FaceId> {
    let domain = surface.natural_uv_domain();
    let [up_shell, down_shell] = shells;
    let face = brep
        .add_face_record(Face {
            surface: Some(surface),
            uv_domain: domain,
            rectangular_trim: 1,
            tolerance: brep.tolerance(),
            ..Face::new(up_shell)
        })
        .map_err(setup)?;
    brep.shell_mut(up_shell).map_err(setup)?.faces.push(face);
    if down_shell != up_shell {
        brep.shell_mut(down_shell).map_err(setup)?.faces.push(face);
    }
    let up = brep
        .add_faceuse_record(Faceuse {
            face: Some(face),
            shell: Some(up_shell),
            orientation: OrientType::Same,
            ..Faceuse::default()
        })
        .map_err(setup)?;
    let down = brep
        .add_faceuse_record(Faceuse {
            face: Some(face),
            shell: Some(down_shell),
            mate: Some(up),
            orientation: OrientType::Opposite,
            ..Faceuse::default()
        })
        .map_err(setup)?;
    brep.faceuse_mut(up).map_err(setup)?.mate = Some(down);
    brep.face_mut(face).map_err(setup)?.faceuse = Some(up);
    let loop_id = brep
        .add_loop_record(Loop {
            face,
            edges: edges
                .iter()
                .map(|&(edge, forward)| LoopEdge { edge, forward })
                .collect(),
            loopuse: None,
        })
        .map_err(setup)?;
    brep.face_mut(face).map_err(setup)?.loops.push(loop_id);
    let up_loop = brep
        .add_loopuse_record(Loopuse {
            faceuse: Some(up),
            loop_: Some(loop_id),
            orientation: OrientType::Same,
            ..Loopuse::default()
        })
        .map_err(setup)?;
    let down_loop = brep
        .add_loopuse_record(Loopuse {
            faceuse: Some(down),
            loop_: Some(loop_id),
            mate: Some(up_loop),
            orientation: OrientType::Same,
            ..Loopuse::default()
        })
        .map_err(setup)?;
    brep.loopuse_mut(up_loop).map_err(setup)?.mate = Some(down_loop);
    brep.loop_mut(loop_id).map_err(setup)?.loopuse = Some(up_loop);
    let lo = domain.min();
    let hi = domain.max();
    let uv = [[lo.x, lo.y], [hi.x, lo.y], [hi.x, hi.y], [lo.x, hi.y]];
    for (faceuse, loopuse, reverse) in [(up, up_loop, false), (down, down_loop, true)] {
        brep.faceuse_mut(faceuse)
            .map_err(setup)?
            .loopuses
            .push(loopuse);
        let mut uses = Vec::new();
        for index in 0..4 {
            let next = (index + 1) % 4;
            let &(edge, forward) = at(&edges, index)?;
            let start = *at(&vertices, if reverse { next } else { index })?;
            let orientation = if forward == reverse {
                OrientType::Opposite
            } else {
                OrientType::Same
            };
            let a = *at(&uv, if reverse { next } else { index })?;
            let b = *at(&uv, if reverse { index } else { next })?;
            let trim = line(
                Vec3::new(a[0], a[1], 0.0),
                Vec3::new(b[0], b[1], 0.0),
                [0.0, 1.0],
                2,
            )?;
            uses.push(add_use(
                brep,
                edge,
                start,
                EdgeuseOwner::Loopuse(Some(loopuse)),
                orientation,
                Some(trim),
            )?);
        }
        if reverse {
            uses.reverse();
        }
        brep.loopuse_mut(loopuse).map_err(setup)?.start =
            LoopuseStart::Edgeuse(uses.first().copied());
        for (index, &use_id) in uses.iter().enumerate() {
            let ccw = *at(&uses, (index + 1) % uses.len())?;
            let cw = *at(&uses, (index + uses.len() - 1) % uses.len())?;
            let record = brep.edgeuse_mut(use_id).map_err(setup)?;
            record.ccw = Some(ccw);
            record.cw = Some(cw);
        }
    }
    Ok(face)
}

fn build(fixture: &Fixture, tolerance: f64) -> HResult<Brep> {
    let mut brep = Brep::new();
    brep.set_tolerance(tolerance);
    // Actual typed region identity, NOT an invented IsVoid or infinite-region flag.
    let region = brep
        .add_region_record(Region { in_brep: true })
        .map_err(setup)?;
    match *fixture {
        Fixture::Empty => {}
        Fixture::Wire {
            start,
            end,
            domain,
            interval,
        } => {
            let curve = line(v3(start), v3(end), domain, 3)?;
            let [t0, t1] = interval;
            let a = add_vertex(&mut brep, curve.evaluate_point(t0).map_err(setup)?)?;
            let b = add_vertex(&mut brep, curve.evaluate_point(t1).map_err(setup)?)?;
            let shell = add_shell(&mut brep, region)?;
            let edge = add_edge(&mut brep, [a, b], curve, interval)?;
            let first = add_use(
                &mut brep,
                edge,
                a,
                EdgeuseOwner::Shell(Some(shell)),
                OrientType::Same,
                None,
            )?;
            let second = add_use(
                &mut brep,
                edge,
                b,
                EdgeuseOwner::Shell(Some(shell)),
                OrientType::Opposite,
                None,
            )?;
            brep.edgeuse_mut(first).map_err(setup)?.ccw = Some(second);
            brep.edgeuse_mut(first).map_err(setup)?.cw = Some(second);
            brep.edgeuse_mut(second).map_err(setup)?.ccw = Some(first);
            brep.edgeuse_mut(second).map_err(setup)?.cw = Some(first);
        }
        Fixture::PlaneSheet {
            origin,
            x_axis,
            y_axis,
            scale,
            domain,
        } => {
            let [u0, v0, u1, v1] = domain;
            let surface = Surface::from(Plane::new(
                v3(origin),
                v3(x_axis),
                v3(y_axis),
                point2(scale),
                &Extent2d::from_bounds(u0, v0, u1, v1),
            ));
            let corners = [[u0, v0], [u1, v0], [u1, v1], [u0, v1]];
            let mut vertices = Vec::new();
            for uv in corners {
                vertices.push(add_vertex(
                    &mut brep,
                    surface.evaluate_point(point2(uv)).map_err(setup)?,
                )?);
            }
            let shell = add_shell(&mut brep, region)?;
            let mut edges = Vec::new();
            for index in 0..4 {
                let a = *at(&vertices, index)?;
                let b = *at(&vertices, (index + 1) % 4)?;
                let curve = line(
                    brep.vertex(a).map_err(setup)?.point,
                    brep.vertex(b).map_err(setup)?.point,
                    [0.0, 1.0],
                    3,
                )?;
                edges.push((add_edge(&mut brep, [a, b], curve, [0.0, 1.0])?, true));
            }
            rectangular_face(
                &mut brep,
                surface,
                [shell, shell],
                [
                    *at(&vertices, 0)?,
                    *at(&vertices, 1)?,
                    *at(&vertices, 2)?,
                    *at(&vertices, 3)?,
                ],
                [
                    *at(&edges, 0)?,
                    *at(&edges, 1)?,
                    *at(&edges, 2)?,
                    *at(&edges, 3)?,
                ],
            )?;
        }
        Fixture::Box { origin, size } => build_box(&mut brep, region, origin, size)?,
    }
    Ok(brep)
}

// Input-only rectilinear scaffold. Unlike the missing SmBrep copy constructor,
// this creates distinct analytic geometry and typed topology from DTO scalars.
fn build_box(
    brep: &mut Brep,
    region: solidcore::topology::RegionId,
    origin: [f64; 3],
    size: [f64; 3],
) -> HResult<()> {
    if size.iter().any(|value| *value <= 0.0) {
        return Err(bad("box fixture constructor requires positive dimensions"));
    }
    let [x, y, z] = origin;
    let [dx, dy, dz] = size;
    let corners = [
        [x, y, z],
        [x + dx, y, z],
        [x + dx, y + dy, z],
        [x, y + dy, z],
        [x, y, z + dz],
        [x + dx, y, z + dz],
        [x + dx, y + dy, z + dz],
        [x, y + dy, z + dz],
    ];
    let mut vertices = Vec::new();
    for point in corners {
        vertices.push(add_vertex(brep, v3(point))?);
    }
    let inside = brep
        .add_region_record(Region { in_brep: true })
        .map_err(setup)?;
    let shells = [add_shell(brep, region)?, add_shell(brep, inside)?];
    let mut edges: Vec<(usize, usize, EdgeId)> = Vec::new();
    // Plane construction order follows SmPrimitiveCreation.cpp:287-344. Native
    // post-stitch face traversal/order is intentionally NOT inferred from it.
    for (quad, point, x_axis, y_axis, domain) in [
        ([0, 1, 2, 3], origin, Vec3::X, Vec3::Y, [dx, dy]),
        ([4, 5, 6, 7], [x, y, z + dz], Vec3::X, Vec3::Y, [dx, dy]),
        ([0, 3, 7, 4], origin, Vec3::Y, Vec3::Z, [dy, dz]),
        ([1, 2, 6, 5], [x + dx, y, z], Vec3::Y, Vec3::Z, [dy, dz]),
        ([0, 1, 5, 4], origin, Vec3::X, Vec3::Z, [dx, dz]),
        ([3, 2, 6, 7], [x, y + dy, z], Vec3::X, Vec3::Z, [dx, dz]),
    ] {
        let mut boundary = Vec::new();
        for index in 0..4 {
            let a = *at(&quad, index)?;
            let b = *at(&quad, (index + 1) % 4)?;
            if let Some(&(first, _, edge)) = edges.iter().find(|&&(first, second, _)| {
                (first == a && second == b) || (first == b && second == a)
            }) {
                boundary.push((edge, first == a));
            } else {
                let curve = line(v3(*at(&corners, a)?), v3(*at(&corners, b)?), [0.0, 1.0], 3)?;
                let edge = add_edge(
                    brep,
                    [*at(&vertices, a)?, *at(&vertices, b)?],
                    curve,
                    [0.0, 1.0],
                )?;
                edges.push((a, b, edge));
                boundary.push((edge, true));
            }
        }
        let surface = Surface::from(Plane::new(
            v3(point),
            x_axis,
            y_axis,
            point2([1.0, 1.0]),
            &Extent2d::from_bounds(0.0, 0.0, domain[0], domain[1]),
        ));
        rectangular_face(
            brep,
            surface,
            shells,
            [
                *at(&vertices, quad[0])?,
                *at(&vertices, quad[1])?,
                *at(&vertices, quad[2])?,
                *at(&vertices, quad[3])?,
            ],
            [
                *at(&boundary, 0)?,
                *at(&boundary, 1)?,
                *at(&boundary, 2)?,
                *at(&boundary, 3)?,
            ],
        )?;
    }
    Ok(())
}

const SOURCE: BrepId = BrepId::from_index(17);
const FOREIGN: BrepId = BrepId::from_index(43);
fn face_at(brep: Option<&Brep>, index: usize) -> HResult<FaceId> {
    brep.and_then(|brep| brep.faces().iter().nth(index).map(|(id, _)| id))
        .ok_or_else(|| bad("selected face absent from Rust origin traversal"))
}
fn selections(
    input: &Inputs,
    source: Option<&Brep>,
    foreign: Option<&Brep>,
) -> HResult<Vec<Option<FaceKey>>> {
    input
        .faces
        .iter()
        .map(|selection| {
            Ok(match *selection {
                Selection::Source(index) => Some((SOURCE, face_at(source, index)?)),
                Selection::Foreign(index) => Some((FOREIGN, face_at(foreign, index)?)),
                Selection::Null => None,
            })
        })
        .collect()
}
fn local_faces(selections: &[Option<FaceKey>]) -> HResult<Vec<FaceId>> {
    selections
        .iter()
        .map(|selection| match selection {
            Some((owner, id)) if *owner == SOURCE => Ok(*id),
            _ => Err(bad(
                "static/member shell lists require source-only faces in this Rust adapter",
            )),
        })
        .collect()
}
fn observe_selections(
    selections: &[Option<FaceKey>],
    source: Option<&Brep>,
    foreign: Option<&Brep>,
) -> Vec<[i64; 2]> {
    selections
        .iter()
        .map(|selection| {
            let Some((owner, face)) = selection else {
                return [2, -1];
            };
            let (origin, brep) = if *owner == SOURCE {
                (0, source)
            } else {
                (1, foreign)
            };
            let index = brep.and_then(|brep| brep.faces().iter().position(|(id, _)| id == *face));
            index.map_or([4, -1], |index| [origin, index as i64])
        })
        .collect()
}

#[derive(Debug, Clone, PartialEq)]
struct Projection {
    counts: Vec<i64>,
    kinds: Vec<i64>,
    statuses: Vec<i64>,
    values: Vec<f64>,
}
fn projected_counts(fixture: &Fixture, counts: [i64; 7]) -> Vec<i64> {
    let [_, _, faces, loops, edges, uses, vertices] = counts;
    if matches!(fixture, Fixture::Box { .. }) {
        vec![faces, edges, vertices]
    } else {
        vec![faces, loops, edges, uses, vertices]
    }
}
fn bounds(points: impl Iterator<Item = [f64; 3]>) -> Vec<f64> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    let mut any = false;
    for point in points {
        any = true;
        for ((lo, hi), value) in lo.iter_mut().zip(&mut hi).zip(point) {
            *lo = lo.min(value);
            *hi = hi.max(value);
        }
    }
    if any {
        lo.into_iter().chain(hi).collect()
    } else {
        Vec::new()
    }
}
fn xyz(point: Vec3) -> [f64; 3] {
    [point.x, point.y, point.z]
}
fn domain_values(domain: Extent2d) -> [f64; 4] {
    let lo = domain.min();
    let hi = domain.max();
    [lo.x, lo.y, hi.x, hi.y]
}
fn rust_projection(fixture: &Fixture, brep: &Brep) -> HResult<Projection> {
    let counts = [
        brep.regions().len() as i64,
        brep.shells().len() as i64,
        brep.faces().len() as i64,
        brep.loops().len() as i64,
        brep.edges().len() as i64,
        brep.edgeuses().len() as i64,
        brep.vertices().len() as i64,
    ];
    let mut result = Projection {
        counts: projected_counts(fixture, counts),
        kinds: Vec::new(),
        statuses: Vec::new(),
        values: vec![brep.tolerance()],
    };
    for (_, face) in brep.faces().iter() {
        let surface = face
            .surface
            .as_ref()
            .ok_or_else(|| bad("fixture face has no real surface"))?;
        result.kinds.push(match surface.kind() {
            SurfaceKind::Plane(_) => 1,
            SurfaceKind::BSpline(_) => 2,
            _ => 3,
        });
        if matches!(fixture, Fixture::PlaneSheet { .. }) {
            result.values.extend(domain_values(face.uv_domain));
            result
                .values
                .extend(domain_values(surface.natural_uv_domain()));
            let lo = face.uv_domain.min();
            let hi = face.uv_domain.max();
            for fraction in [0.0, 0.5, 1.0] {
                let uv = point2([
                    (1.0 - fraction) * lo.x + fraction * hi.x,
                    (1.0 - fraction) * lo.y + fraction * hi.y,
                ]);
                let point = surface.evaluate_point(uv).map_err(setup)?;
                let normal = surface
                    .evaluate_normal(uv, Side::Left, Side::Left)
                    .map_err(setup)?;
                result.statuses.extend([i64::from(ffi::NATIVE_SUCCESS); 2]);
                result.values.extend(xyz(point));
                result.values.extend(xyz(normal));
            }
        }
    }
    if matches!(fixture, Fixture::Wire { .. }) {
        for (_, edge) in brep.edges().iter() {
            let curve = edge
                .curve
                .as_ref()
                .ok_or_else(|| bad("fixture wire has no real curve"))?;
            result.kinds.push(match curve.kind() {
                CurveKind::Line(_) => 1,
                CurveKind::BSpline(_) => 2,
                _ => 3,
            });
            let domain = curve.natural_interval();
            result.values.extend([
                edge.interval.min(),
                edge.interval.max(),
                domain.min(),
                domain.max(),
            ]);
            for parameter in [edge.interval.min(), edge.interval.max()] {
                let point = curve.evaluate_point(parameter).map_err(setup)?;
                result.statuses.push(i64::from(ffi::NATIVE_SUCCESS));
                result.values.extend(xyz(point));
            }
        }
    }
    if matches!(fixture, Fixture::Box { .. } | Fixture::Wire { .. }) {
        result.values.extend(bounds(
            brep.vertices().iter().map(|(_, vertex)| xyz(vertex.point)),
        ));
    }
    Ok(result)
}

// The FFI decoder separates the protocol's integers and floats. This second,
// bounded decoder validates the ENTIRE topology/geometry layout before selecting
// representable input fields; dropping unsupported fields must not mask truncation.
struct BlockReader<'a> {
    integers: &'a [i64],
    values: &'a [f64],
}
impl BlockReader<'_> {
    fn integer(&mut self, min: i64, max: i64) -> HResult<i64> {
        let (value, rest) = self
            .integers
            .split_first()
            .ok_or_else(|| bad("truncated native topology block"))?;
        if !(min..=max).contains(value) {
            return Err(bad("invalid native topology integer"));
        }
        self.integers = rest;
        Ok(*value)
    }
    fn count(&mut self) -> HResult<usize> {
        Ok(self.integer(0, (ffi::MAX_OUTPUT_VALUES / 64) as i64)? as usize)
    }
    fn id(&mut self, expected: usize) -> HResult<()> {
        self.integer(expected as i64, expected as i64).map(|_| ())
    }
    fn reference(&mut self, count: usize) -> HResult<()> {
        self.integer(-2, count as i64 - 1).map(|_| ())
    }
    fn list(&mut self, count: usize) -> HResult<()> {
        let length = self.count()?;
        if length > self.integers.len() {
            return Err(bad("truncated native adjacency list"));
        }
        for _ in 0..length {
            self.reference(count)?;
        }
        Ok(())
    }
    fn numbers(&mut self, count: usize) -> HResult<Vec<f64>> {
        let values = self
            .values
            .get(..count)
            .ok_or_else(|| bad("truncated native geometry block"))?;
        self.values = self
            .values
            .get(count..)
            .ok_or_else(|| bad("truncated native geometry block"))?;
        Ok(values.to_vec())
    }
    fn status(&mut self) -> HResult<i64> {
        self.integer(i64::from(i32::MIN), i64::from(i32::MAX))
    }
}
#[derive(Debug)]
struct SampleRecord {
    kind: i64,
    statuses: Vec<i64>,
    values: Vec<f64>,
}
#[derive(Debug)]
struct DecodedBrep {
    counts: [i64; 7],
    tolerance: f64,
    faces: Vec<SampleRecord>,
    edges: Vec<SampleRecord>,
    vertices: Vec<[f64; 3]>,
}
fn decode_block(block: &ffi::BrepSnapshot) -> HResult<DecodedBrep> {
    if block.integers.len().saturating_add(block.values.len()) > ffi::MAX_OUTPUT_VALUES {
        return Err(bad("native B-rep block exceeds output budget"));
    }
    let mut rd = BlockReader {
        integers: &block.integers,
        values: &block.values,
    };
    let mut counts = [0; 7];
    for count in &mut counts {
        *count = rd.count()? as i64;
    }
    let [regions, shells, faces, loops, edges, uses, vertices] = counts.map(|count| count as usize);
    if counts.iter().sum::<i64>() > (ffi::MAX_OUTPUT_VALUES / 64) as i64 {
        return Err(bad("native topology aggregate exceeds budget"));
    }
    rd.reference(regions)?; // Native-only infinite region: validated, not fabricated in Rust.
    rd.integer(0, 1)?; // Native-only editing flag.
    rd.integer(0, 1)?; // Validate-only flag; full block is still checked for purity.
    let tolerance = *at(&rd.numbers(1)?, 0)?;
    for index in 0..regions {
        rd.id(index)?;
        rd.integer(0, 1)?;
    }
    for index in 0..shells {
        rd.id(index)?;
        rd.reference(regions)?;
    }
    let mut face_records = Vec::new();
    for index in 0..faces {
        rd.id(index)?;
        let kind = rd.integer(0, 3)?;
        rd.reference(shells)?;
        rd.reference(shells)?;
        rd.integer(-1, 3)?;
        rd.integer(-1, 3)?;
        rd.list(loops)?;
        let statuses = (0..6).map(|_| rd.status()).collect::<HResult<_>>()?;
        face_records.push(SampleRecord {
            kind,
            statuses,
            values: rd.numbers(27)?,
        });
    }
    for index in 0..loops {
        rd.id(index)?;
        rd.reference(faces)?;
        rd.list(uses)?;
    }
    let mut edge_records = Vec::new();
    for index in 0..edges {
        rd.id(index)?;
        rd.reference(vertices)?;
        rd.reference(vertices)?;
        let kind = rd.integer(0, 3)?;
        rd.list(uses)?;
        let statuses = (0..2).map(|_| rd.status()).collect::<HResult<_>>()?;
        edge_records.push(SampleRecord {
            kind,
            statuses,
            values: rd.numbers(11)?,
        });
    }
    for index in 0..uses {
        rd.id(index)?;
        for count in [edges, faces, shells, uses, uses] {
            rd.reference(count)?;
        }
        rd.integer(-1, 3)?;
    }
    let mut vertex_points = Vec::new();
    for index in 0..vertices {
        rd.id(index)?;
        let values = rd.numbers(4)?;
        vertex_points.push([*at(&values, 0)?, *at(&values, 1)?, *at(&values, 2)?]);
    }
    if !rd.integers.is_empty() || !rd.values.is_empty() {
        return Err(bad("trailing native B-rep snapshot fields"));
    }
    Ok(DecodedBrep {
        counts,
        tolerance,
        faces: face_records,
        edges: edge_records,
        vertices: vertex_points,
    })
}
fn native_projection(fixture: &Fixture, decoded: &DecodedBrep) -> HResult<Projection> {
    let mut result = Projection {
        counts: projected_counts(fixture, decoded.counts),
        kinds: Vec::new(),
        statuses: Vec::new(),
        values: vec![decoded.tolerance],
    };
    for face in &decoded.faces {
        result.kinds.push(face.kind);
        if matches!(fixture, Fixture::PlaneSheet { .. }) {
            result.statuses.extend(&face.statuses);
            result.values.extend(
                face.values
                    .get(1..)
                    .ok_or_else(|| bad("missing face geometry"))?,
            );
        }
    }
    if matches!(fixture, Fixture::Wire { .. }) {
        for edge in &decoded.edges {
            result.kinds.push(edge.kind);
            result.statuses.extend(&edge.statuses);
            result.values.extend(
                edge.values
                    .get(1..)
                    .ok_or_else(|| bad("missing edge geometry"))?,
            );
        }
    }
    if matches!(fixture, Fixture::Box { .. } | Fixture::Wire { .. }) {
        result
            .values
            .extend(bounds(decoded.vertices.iter().copied()));
    }
    Ok(result)
}
fn project(output: Output, key: &str, projection: Option<Projection>) -> Output {
    let Some(projection) = projection else {
        return output.int(&format!("{key}.present"), 0);
    };
    output
        .int(&format!("{key}.present"), 1)
        .ints(&format!("{key}.counts"), projection.counts)
        .ints(&format!("{key}.kinds"), projection.kinds)
        .ints(&format!("{key}.evaluation_status"), projection.statuses)
        .values(&format!("{key}.geometry"), projection.values)
}
fn scalar_output(
    status: i32,
    input_transferred: bool,
    lifetime: i32,
    result_origin: i32,
    doing_boolean: [bool; 2],
    executive: Option<&ffi::ExecutiveState>,
    selected_before: &[[i64; 2]],
    selected_after: &[[i64; 2]],
) -> Output {
    let mut output = Output::new()
        .int("status", i64::from(status))
        .ints(
            "ownership",
            vec![
                i64::from(input_transferred),
                i64::from(lifetime),
                i64::from(result_origin),
            ],
        )
        .ints("doing_boolean", doing_boolean.map(i64::from).to_vec())
        .ints(
            "selected.before",
            selected_before
                .iter()
                .flatten()
                .copied()
                .collect::<Vec<_>>(),
        )
        .ints(
            "selected.after",
            selected_after.iter().flatten().copied().collect::<Vec<_>>(),
        )
        .int("executive.present", i64::from(executive.is_some()));
    if let Some(state) = executive {
        output = output
            .ints(
                "executive.flags",
                vec![
                    i64::from(state.operation),
                    i64::from(state.inset),
                    i64::from(state.extend_convex),
                    i64::from(state.merge_results),
                    i64::from(state.create_offset_solid),
                ],
            )
            .values(
                "executive.geometry_parameters",
                vec![state.distance, state.approx_tol, state.tangency_degrees],
            )
            .ints(
                "executive.signed_zero",
                vec![
                    i64::from(state.distance == 0.0 && state.distance.is_sign_negative()),
                    i64::from(state.approx_tol == 0.0 && state.approx_tol.is_sign_negative()),
                ],
            );
    }
    output
}
fn unchanged(before: Option<&ffi::BrepSnapshot>, after: Option<&ffi::BrepSnapshot>) -> bool {
    match (before, after) {
        (None, None) => true,
        (Some(a), Some(b)) => {
            a.integers == b.integers
                && a.values.len() == b.values.len()
                && a.values
                    .iter()
                    .zip(&b.values)
                    .all(|(a, b)| a.to_bits() == b.to_bits())
        }
        _ => false,
    }
}
fn native_output(input: &Inputs, snapshot: &ffi::Snapshot) -> HResult<Output> {
    if snapshot.selected_before.len() != input.faces.len()
        || snapshot.selected_after.len() != input.faces.len()
        || snapshot.executive.is_some() != input.operation.member()
    {
        return Err(bad("native executive/selector snapshot shape mismatch"));
    }
    let mut output = scalar_output(
        snapshot.status,
        snapshot.input_transferred,
        snapshot.source_lifetime,
        snapshot.result_origin,
        snapshot.doing_boolean,
        snapshot.executive.as_ref(),
        &snapshot.selected_before,
        &snapshot.selected_after,
    )
    .ints(
        "input.pure",
        vec![
            i64::from(unchanged(
                snapshot.source_before.as_ref(),
                snapshot.source_after.as_ref(),
            )),
            i64::from(unchanged(
                snapshot.foreign_before.as_ref(),
                snapshot.foreign_after.as_ref(),
            )),
        ],
    );
    for (key, fixture, block) in [
        (
            "source.before",
            input.source.as_ref(),
            snapshot.source_before.as_ref(),
        ),
        (
            "source.after",
            input.source.as_ref(),
            snapshot.source_after.as_ref(),
        ),
        (
            "foreign.before",
            input.foreign.as_ref(),
            snapshot.foreign_before.as_ref(),
        ),
        (
            "foreign.after",
            input.foreign.as_ref(),
            snapshot.foreign_after.as_ref(),
        ),
    ] {
        let decoded = block.map(decode_block).transpose()?;
        let projection = match (fixture, decoded.as_ref()) {
            (Some(fixture), Some(decoded)) => Some(native_projection(fixture, decoded)?),
            (_, None) => None,
            (None, Some(_)) => return Err(bad("native snapshot invented an input allocation")),
        };
        output = project(output, key, projection);
    }
    for (key, block) in [
        ("result", &snapshot.result),
        ("executive_offset", &snapshot.executive_offset),
    ] {
        output = output.int(&format!("{key}.present"), i64::from(block.is_some()));
        if let Some(block) = block {
            decode_block(block)?;
            output = output
                .ints(&format!("{key}.native_topology"), block.integers.clone())
                .values(&format!("{key}.native_geometry"), block.values.clone());
        }
    }
    Ok(output)
}
fn native_outcome(input: &Inputs, outcome: ffi::Outcome) -> HResult<Output> {
    match outcome {
        ffi::Outcome::Rejected { status } => Ok(Output::rejected(format!(
            "native fixture constructor status_{status}"
        ))),
        ffi::Outcome::Built(snapshot) => native_output(input, &snapshot),
    }
}

fn rust_output(input: &Inputs) -> RResult<Output> {
    validate(input)?;
    let mut source = input
        .source
        .as_ref()
        .map(|fixture| build(fixture, input.tolerance))
        .transpose()?;
    let foreign = input
        .foreign
        .as_ref()
        .map(|fixture| build(fixture, input.tolerance))
        .transpose()?;
    let keys = selections(input, source.as_ref(), foreign.as_ref())?;
    let selected_before = observe_selections(&keys, source.as_ref(), foreign.as_ref());
    let source_state_before = source.as_ref().map(|brep| format!("{brep:?}"));
    let foreign_state_before = foreign.as_ref().map(|brep| format!("{brep:?}"));
    let source_before = input
        .source
        .as_ref()
        .zip(source.as_ref())
        .map(|(fixture, brep)| rust_projection(fixture, brep))
        .transpose()?;
    let foreign_before = input
        .foreign
        .as_ref()
        .zip(foreign.as_ref())
        .map(|(fixture, brep)| rust_projection(fixture, brep))
        .transpose()?;
    let mut kernel = Kernel::default();
    let boolean_before = kernel.context.doing_boolean();
    let mut exec = input
        .operation
        .member()
        .then(|| executive(&input.config))
        .transpose()?;
    if let Some(exec) = &mut exec {
        exec.set_shell_faces(&local_faces(&keys)?);
    }
    let mut transferred = false;
    let result = match input.operation {
        Operation::StaticOffset => p2::offset_brep(
            &mut kernel,
            source
                .as_ref()
                .ok_or_else(|| bad("static offset null source"))?,
            input.config.offset(),
        ),
        Operation::StaticShell => {
            let original = source
                .take()
                .ok_or_else(|| bad("static shell null source"))?;
            let outcome = OffsetExecutive::shell_brep(
                &mut kernel,
                original,
                input.config.shell(),
                &local_faces(&keys)?,
            );
            transferred = outcome.input_ownership_transferred();
            source = outcome.retained_input;
            outcome.result
        }
        Operation::Solid => exec
            .as_mut()
            .ok_or_else(|| bad("missing Rust executive"))?
            .do_solid_offset(
                &mut kernel,
                source.as_ref(),
                SolidOffsetOptions {
                    skip_stitch_merge: input.config.skip_stitch_merge,
                },
            ),
        Operation::Extended => exec
            .as_mut()
            .ok_or_else(|| bad("missing Rust executive"))?
            .do_extended_offset(&mut kernel, source.as_ref()),
        Operation::ApiShell => p2::api_shell_brep_full(
            &mut kernel,
            source.as_ref().map(|brep| BrepRef {
                brep,
                handle: SOURCE,
            }),
            input.config.shell(),
            &keys,
        ),
        Operation::ApiOffset => {
            p2::api_offset_brep_full(&mut kernel, source.as_ref(), input.config.offset())
        }
    };
    let (status, result) = operation(result)?;
    if result.is_some()
        || exec
            .as_ref()
            .is_some_and(|exec| exec.offset_brep().is_some())
    {
        return Err(bad("newly reachable non-deferred result/temporary needs a full Rust output adapter; not shape parity").into());
    }
    let state = exec.as_ref().map(executive_state);
    let selected_after = observe_selections(&keys, source.as_ref(), foreign.as_ref());
    let source_after = input
        .source
        .as_ref()
        .zip(source.as_ref())
        .map(|(fixture, brep)| rust_projection(fixture, brep))
        .transpose()?;
    let foreign_after = input
        .foreign
        .as_ref()
        .zip(foreign.as_ref())
        .map(|(fixture, brep)| rust_projection(fixture, brep))
        .transpose()?;
    let source_pure = source_state_before == source.as_ref().map(|brep| format!("{brep:?}"));
    let foreign_pure = foreign_state_before == foreign.as_ref().map(|brep| format!("{brep:?}"));
    let lifetime = if input.source.is_none() {
        0
    } else if source.is_some() {
        1
    } else {
        3
    };
    let mut output = scalar_output(
        status,
        transferred,
        lifetime,
        0,
        [boolean_before, kernel.context.doing_boolean()],
        state.as_ref(),
        &selected_before,
        &selected_after,
    )
    .ints(
        "input.pure",
        vec![i64::from(source_pure), i64::from(foreign_pure)],
    )
    .int("result.present", 0)
    .int("executive_offset.present", 0);
    for (key, projection) in [
        ("source.before", source_before),
        ("source.after", source_after),
        ("foreign.before", foreign_before),
        ("foreign.after", foreign_after),
    ] {
        output = project(output, key, projection);
    }
    Ok(output)
}

impl UnitDiff for Runner {
    fn id(&self) -> &'static str {
        "offset.offset_executive_p2"
    }
    fn run_cpp(&self, case: &Case) -> HResult<Output> {
        let input: Inputs = case.parse()?;
        // FFI argument errors are NEVER constructor rejections or native statuses.
        let outcome = ffi::eval(&input.native())
            .map_err(|error| bad(format!("native p2 adapter: {error}")))?;
        native_outcome(&input, outcome)
    }
    fn run_rust(&self, case: &Case) -> HResult<Output> {
        match rust_output(&case.parse()?) {
            Ok(output) => Ok(output),
            Err(RunError::Harness(error)) => Err(error),
            Err(RunError::Kernel(error)) => Ok(Output::unported(&error)
                .unwrap_or_else(|| Output::err(format!("Rust operation: {error}")))),
        }
    }
    fn cases(&self, context: &CaseContext) -> HResult<Vec<Case>> {
        let mut context = context.clone();
        context.seeds = context.seeds.max(MIN_GENERATED);
        default_cases(self, &context)
    }
    fn generate(&self, rng: &mut Rng) -> Option<Value> {
        serde_json::to_value(generated(rng)).ok()
    }
}

fn sheet(origin: [f64; 3], scale: f64, axis: u64) -> Fixture {
    let (x_axis, y_axis) = match axis % 3 {
        0 => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
        1 => ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
        _ => ([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]),
    };
    Fixture::PlaneSheet {
        origin,
        x_axis,
        y_axis,
        scale: [scale, 2.0 * scale],
        domain: [-1.0, -0.5, 2.0, 1.5],
    }
}
fn wire(origin: [f64; 3], scale: f64) -> Fixture {
    let [x, y, z] = origin;
    Fixture::Wire {
        start: origin,
        end: [x + 3.0 * scale, y + scale, z - 2.0 * scale],
        domain: [-2.0, 3.0],
        interval: [-1.5, 2.25],
    }
}
fn generated(rng: &mut Rng) -> Inputs {
    // Seven of twelve categories call valid nonempty geometry paths. Early-return
    // cases carry real foreign/source geometry, not an inflated empty/guard corpus.
    let category = rng.below(12);
    let scale = match rng.below(5) {
        0 => 0.01,
        1 => 0.1,
        2 => 1.0,
        3 => 10.0,
        _ => 100.0,
    };
    let origin = rng.point3(-3.0, 3.0).map(|value| value * scale);
    let flags = rng.below(32);
    let tolerance = scale * 1e-7;
    let distance = match rng.below(8) {
        0 => -0.0,
        1 => 0.0,
        2 => 2.0 * tolerance,
        3 => -(2.0 * tolerance),
        _ => scale * rng.uniform(0.02, 0.15) * if rng.chance(0.5) { -1.0 } else { 1.0 },
    };
    let mut input = Inputs {
        operation: Operation::ApiOffset,
        source: Some(sheet(origin, scale, rng.below(3))),
        foreign: None,
        faces: Vec::new(),
        tolerance,
        config: Config {
            offset_distance: distance,
            member_distance: scale * rng.uniform(0.01, 0.2),
            member_approx_tol: match rng.below(7) {
                0 => -1.0,
                1 => -0.0,
                2 => 0.0,
                _ => tolerance,
            },
            member_operation: rng.below(5) as u32,
            inset: flags & 1 != 0,
            extend_convex: flags & 2 != 0,
            merge_results: flags & 4 != 0,
            create_offset_solid: flags & 8 != 0,
            skip_stitch_merge: flags & 16 != 0,
        },
    };
    match category {
        0 | 1 => {
            input.operation = if category == 0 {
                Operation::Solid
            } else {
                Operation::Extended
            };
            input.source = None;
            let scale = match rng.below(4) {
                0 => 1e-9,
                1 => 1e9,
                _ => scale,
            };
            input.foreign = Some(wire([2.0 * scale, -scale, 3.0 * scale], scale));
            input.config.member_distance = match rng.below(5) {
                0 => -0.0,
                1 => 0.0,
                2 => 1e-150,
                3 => 1e100,
                _ => scale * 0.125,
            };
        }
        2 => {
            input.operation = if rng.chance(0.5) {
                Operation::ApiShell
            } else {
                Operation::ApiOffset
            };
            input.source = None;
            input.foreign = Some(sheet(origin, scale, rng.below(3)));
        }
        3 | 4 => {
            input.operation = Operation::ApiShell;
            input.foreign = Some(sheet(
                [origin[0] + 7.0 * scale, origin[1], origin[2]],
                scale,
                rng.below(3),
            ));
            input.faces = match rng.below(4) {
                0 => vec![
                    Selection::Source(0),
                    Selection::Source(0),
                    Selection::Foreign(0),
                ],
                1 => vec![Selection::Foreign(0), Selection::Null, Selection::Source(0)],
                2 => vec![Selection::Null, Selection::Foreign(0), Selection::Source(0)],
                _ => vec![Selection::Source(0), Selection::Null, Selection::Source(0)],
            };
        }
        5 | 6 | 11 => {
            input.operation = if category == 5 {
                Operation::ApiShell
            } else if category == 6 {
                Operation::ApiOffset
            } else {
                Operation::StaticShell
            };
            input.source = Some(Fixture::Box {
                origin,
                size: [2.0 * scale, 3.0 * scale, 4.0 * scale],
            });
            // Constructive boxes stay at moderate scales; extreme getter/guard
            // scalars are covered on genuine early paths rather than assuming
            // offset/stitch convergence over arbitrary native scales.
            input.config.offset_distance = if rng.chance(0.5) {
                0.05 * scale
            } else {
                -0.05 * scale
            };
            if category != 6 && rng.chance(0.7) {
                let index = rng.below(6) as usize;
                input.faces = vec![Selection::Source(index), Selection::Source(index)];
            }
        }
        7 => input.operation = Operation::StaticOffset,
        8 => {
            input.operation = Operation::StaticShell;
            if rng.chance(0.5) {
                input.faces = vec![Selection::Source(0)];
            }
        }
        9 => {
            input.operation = Operation::Solid;
            if rng.chance(0.3) {
                input.faces = vec![Selection::Source(0), Selection::Source(0)];
            }
        }
        _ => {
            input.operation = Operation::Extended;
            if rng.chance(0.5) {
                input.source = Some(wire(origin, scale));
            }
        }
    }
    if input.operation.member() && input.source.is_some() {
        input.config.member_approx_tol = tolerance;
    }
    input
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    use crate::case::repo_root;
    use crate::CaseSource;

    fn input(operation: Operation) -> Inputs {
        Inputs {
            operation,
            source: Some(sheet([1.0, -2.0, 3.0], 2.0, 0)),
            foreign: None,
            faces: Vec::new(),
            tolerance: 1e-6,
            config: Config::default(),
        }
    }
    fn case(input: &Inputs) -> Case {
        Case {
            unit: Runner.id().into(),
            name: "unit-test".into(),
            source: CaseSource::Inline,
            inputs: serde_json::to_value(input).unwrap(),
        }
    }
    fn empty_block(tolerance: f64) -> ffi::BrepSnapshot {
        ffi::BrepSnapshot {
            integers: vec![1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            values: vec![tolerance],
        }
    }
    fn empty_snapshot(status: i32) -> ffi::Snapshot {
        ffi::Snapshot {
            status,
            input_transferred: false,
            source_lifetime: 1,
            result_origin: 0,
            doing_boolean: [false, false],
            executive: None,
            selected_before: Vec::new(),
            selected_after: Vec::new(),
            source_before: Some(empty_block(1e-6)),
            source_after: Some(empty_block(1e-6)),
            foreign_before: None,
            foreign_after: None,
            result: None,
            executive_offset: None,
        }
    }

    #[test]
    fn serde_native_dtos_keep_all_operations_origins_and_raw_signed_distances() {
        for operation in [
            Operation::StaticOffset,
            Operation::StaticShell,
            Operation::Solid,
            Operation::Extended,
            Operation::ApiShell,
            Operation::ApiOffset,
        ] {
            for distance in [-1e100, -0.25, -1e-150, -0.0, 0.0, 0.25] {
                let mut original = input(operation);
                original.foreign = original.source.clone(); // DTO only, never a Brep copy.
                original.faces = vec![
                    Selection::Source(0),
                    Selection::Source(0),
                    Selection::Foreign(0),
                    Selection::Null,
                ];
                original.config.offset_distance = distance;
                original.config.member_distance = 0.375;
                original.config.inset = true;
                let decoded: Inputs =
                    serde_json::from_value(serde_json::to_value(&original).unwrap()).unwrap();
                assert_eq!(decoded, original);
                let request = decoded.native();
                assert_eq!(request.operation, operation.native());
                assert_eq!(request.config.offset_distance.to_bits(), distance.to_bits());
                assert_eq!(request.config.member_distance, 0.375);
                assert!(request.config.inset);
                assert_eq!(
                    request.faces,
                    vec![
                        ffi::FaceSelection::Source(0),
                        ffi::FaceSelection::Source(0),
                        ffi::FaceSelection::Foreign(0),
                        ffi::FaceSelection::Null
                    ]
                );
            }
        }
        let mut json = serde_json::to_value(input(Operation::ApiShell)).unwrap();
        json["unexpected"] = Value::Bool(true);
        assert!(serde_json::from_value::<Inputs>(json).is_err());
        assert!(serde_json::from_value::<Selection>(
            serde_json::json!({"origin":"source", "index":-1})
        )
        .is_err());
        assert!(
            serde_json::from_value::<Selection>(serde_json::json!({"origin":"foreign"})).is_err()
        );
        assert!(serde_json::from_value::<Selection>(
            serde_json::json!({"origin":"null", "index":0})
        )
        .is_err());
    }

    #[test]
    fn source_face_origin_cannot_alias_foreign_face_zero_and_order_is_preserved() {
        let mut input = input(Operation::ApiShell);
        input.foreign = Some(sheet([20.0, 3.0, -4.0], 3.0, 1));
        let source = build(input.source.as_ref().unwrap(), input.tolerance).unwrap();
        let foreign = build(input.foreign.as_ref().unwrap(), input.tolerance).unwrap();
        assert_eq!(
            face_at(Some(&source), 0).unwrap(),
            face_at(Some(&foreign), 0).unwrap()
        );
        for selectors in [
            vec![
                Selection::Source(0),
                Selection::Source(0),
                Selection::Foreign(0),
            ],
            vec![Selection::Source(0), Selection::Null, Selection::Foreign(0)],
            vec![Selection::Null, Selection::Foreign(0), Selection::Source(0)],
        ] {
            input.faces = selectors.clone();
            let keys = selections(&input, Some(&source), Some(&foreign)).unwrap();
            let expected: Vec<_> = selectors
                .iter()
                .map(|selection| match selection {
                    Selection::Source(index) => [0, *index as i64],
                    Selection::Foreign(index) => [1, *index as i64],
                    Selection::Null => [2, -1],
                })
                .collect();
            assert_eq!(
                observe_selections(&keys, Some(&source), Some(&foreign)),
                expected
            );
            input.config.offset_distance = 0.0;
            let output = Runner.run_rust(&case(&input)).unwrap();
            assert_eq!(output.ints["status"], [1007]);
            assert_eq!(output.ints["input.pure"], [1, 1]);
            assert_eq!(output.ints["ownership"], [0, 1, 0]);
            assert_eq!(
                output.ints["selected.before"],
                output.ints["selected.after"]
            );
            assert_eq!(
                output.values["source.before.geometry"],
                output.values["source.after.geometry"]
            );
            assert_eq!(
                output.values["foreign.before.geometry"],
                output.values["foreign.after.geometry"]
            );
            assert_ne!(
                output.values["source.before.geometry"],
                output.values["foreign.before.geometry"]
            );
            assert!(output.error.is_none() && output.deferred.is_none());
        }
        let mut valid = input;
        valid.faces = vec![Selection::Source(0), Selection::Source(0)];
        let output = Runner.run_rust(&case(&valid)).unwrap();
        assert_eq!(output.deferred.unwrap().symbol, "SmBrep::SmBrep");
    }

    #[test]
    fn member_null_preserves_every_flag_enum_magnitude_and_constructor_scalar() {
        for operation in [Operation::Solid, Operation::Extended] {
            for bits in 0..32 {
                let mut input = input(operation);
                input.source = None;
                input.foreign = Some(wire([4.0, -2.0, 7.0], 3.0));
                input.config.inset = bits & 1 != 0;
                input.config.extend_convex = bits & 2 != 0;
                input.config.merge_results = bits & 4 != 0;
                input.config.create_offset_solid = bits & 8 != 0;
                input.config.skip_stitch_merge = bits & 16 != 0;
                input.config.member_operation = bits % 5;
                input.config.member_distance = match bits % 4 {
                    0 => -0.0,
                    1 => 1e-150,
                    2 => 1e100,
                    _ => 0.375,
                };
                input.config.member_approx_tol = match bits % 4 {
                    0 => -0.0,
                    1 => -1.0,
                    2 => -0.25,
                    _ => 1e-14,
                };
                let output = Runner.run_rust(&case(&input)).unwrap();
                assert_eq!(output.ints["status"], [1006]);
                assert_eq!(
                    output.ints["executive.flags"],
                    [
                        i64::from(bits % 5),
                        i64::from(bits & 1 != 0),
                        i64::from(bits & 2 != 0),
                        i64::from(bits & 4 != 0),
                        i64::from(bits & 8 != 0)
                    ]
                );
                assert_eq!(
                    output.values["executive.geometry_parameters"][0].to_bits(),
                    input.config.member_distance.to_bits()
                );
                assert_eq!(
                    output.values["executive.geometry_parameters"][1].to_bits(),
                    input.config.member_approx_tol.to_bits()
                );
                assert_eq!(output.values["executive.geometry_parameters"][2], 2.0);
                assert_eq!(output.ints["doing_boolean"], [0, 0]);
                assert_eq!(output.ints["ownership"], [0, 0, 0]);
                assert_eq!(output.ints["input.pure"], [1, 1]);
                assert_eq!(output.ints["executive_offset.present"], [0]);
                assert!(!output.values["foreign.before.geometry"].is_empty());
            }
        }
    }

    #[test]
    fn api_null_is_1007_not_member_ner_or_a_constructor_rejection() {
        for operation in [Operation::ApiShell, Operation::ApiOffset] {
            let mut input = input(operation);
            input.source = None;
            input.foreign = Some(wire([0.0, 3.0, 5.0], 2.0));
            let output = Runner.run_rust(&case(&input)).unwrap();
            assert_eq!(output.ints["status"], [1007]);
            assert_eq!(output.ints["executive.present"], [0]);
            assert_eq!(output.ints["ownership"], [0, 0, 0]);
            assert!(output.rejected.is_none() && output.error.is_none());
            assert!(!output.values["foreign.before.geometry"].is_empty());
        }
    }

    #[test]
    fn ordinary_statuses_remain_integer_observations_and_unported_never_becomes_1001() {
        let mut input = input(Operation::ApiOffset);
        input.source = Some(Fixture::Empty);
        let mut outputs = Vec::new();
        for status in [1001, 1006, 1007] {
            let error = SolidError::Upstream {
                operation: "test",
                code: status,
                entity: None,
            };
            assert_eq!(operation::<Brep>(Err(error)).unwrap().0, status);
            let output = native_outcome(
                &input,
                ffi::Outcome::Built(Box::new(empty_snapshot(status))),
            )
            .unwrap();
            assert!(output.error.is_none() && output.rejected.is_none());
            assert_eq!(output.ints["status"], [i64::from(status)]);
            assert_eq!(output.values["source.before.geometry"], [1e-6]);
            outputs.push(output);
        }
        assert_ne!(outputs[0], outputs[1]);
        assert_ne!(outputs[1], outputs[2]);
        let error = SolidError::unported("SmBrep::ShrinkGeometry", "topology.brep_p4");
        assert!(matches!(
            operation::<Brep>(Err(error)),
            Err(RunError::Kernel(SolidError::Unported { .. }))
        ));
        let rejected = native_outcome(&input, ffi::Outcome::Rejected { status: 1007 }).unwrap();
        assert!(rejected.rejected.is_some());
        assert!(rejected.ints.is_empty() && rejected.values.is_empty());
    }

    #[test]
    fn ffi_argument_error_is_a_harness_error_not_output_rejected() {
        // These fail before native allocation, and do not rely on native geometry
        // convergence or execute one of the positive operation paths.
        let mut input = input(Operation::StaticOffset);
        input.source = None;
        assert!(Runner.run_cpp(&case(&input)).is_err());
        input.operation = Operation::Solid;
        input.config.member_distance = -1.0;
        assert!(Runner.run_cpp(&case(&input)).is_err());
    }

    #[test]
    fn irrelevant_extension_overflow_does_not_hide_native_early_returns() {
        for operation in [
            Operation::Solid,
            Operation::Extended,
            Operation::ApiShell,
            Operation::ApiOffset,
        ] {
            let mut input = input(operation);
            input.source = None;
            input.config.member_distance = f64::MAX;
            input.config.offset_distance = -f64::MAX;
            input.config.extend_convex = true;
            let case = case(&input);
            assert_eq!(
                Runner.run_rust(&case).unwrap(),
                Runner.run_cpp(&case).unwrap()
            );
            input.config.member_distance = -1.0;
            if operation.member() {
                assert!(validate(&input).is_err());
            }
        }
        let mut input = input(Operation::ApiShell);
        input.source = Some(sheet([0.0; 3], 1.0, 0));
        input.foreign = input.source.clone();
        input.faces = vec![Selection::Foreign(0), Selection::Null];
        input.config.extend_convex = true;
        for distance in [-f64::MAX, f64::MAX] {
            input.config.offset_distance = distance;
            let case = case(&input);
            assert_eq!(
                Runner.run_rust(&case).unwrap(),
                Runner.run_cpp(&case).unwrap()
            );
        }
        input.faces.push(Selection::Source(usize::MAX));
        assert!(validate(&input).is_err());
    }

    #[test]
    fn block_decode_validates_even_fields_excluded_from_the_projection() {
        let fixture = Fixture::Empty;
        let block = empty_block(3e-12);
        let projection = native_projection(&fixture, &decode_block(&block).unwrap()).unwrap();
        let brep = build(&fixture, 3e-12).unwrap();
        assert_eq!(projection, rust_projection(&fixture, &brep).unwrap());
        let mut broken = block.clone();
        broken.integers.pop(); // Missing native-only region void flag.
        assert!(decode_block(&broken).is_err());
        let mut broken = block.clone();
        broken.integers[11] = 2; // Invalid flag, not silently skipped.
        assert!(decode_block(&broken).is_err());
        let mut broken = block.clone();
        broken.integers[7] = 1; // Invalid infinite-region traversal id.
        assert!(decode_block(&broken).is_err());
        let mut broken = block.clone();
        broken.integers[0] = i64::MAX;
        assert!(decode_block(&broken).is_err());
        let mut broken = block.clone();
        broken.values.push(0.0);
        assert!(decode_block(&broken).is_err());
        let mut broken = block;
        broken.integers.push(0);
        assert!(decode_block(&broken).is_err());
    }

    #[test]
    fn sheet_projection_keeps_domains_points_normals_kinds_and_evaluation_statuses() {
        // A small synthetic decoder record, not a C++ operation result used by
        // run_rust. The count/adjacency framing must be consumed in native order.
        let mut ints = vec![
            1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 2, 0,
        ];
        ints.extend([1000; 6]);
        let mut values = vec![1e-6, 1e-6];
        values.extend([0.0, 0.0, 2.0, 4.0]);
        values.extend([0.0, 0.0, 2.0, 4.0]);
        values.extend([
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 0.0, 0.0, 0.0, 1.0, 2.0, 4.0, 0.0, 0.0, 0.0,
            1.0,
        ]);
        let mut block = ffi::BrepSnapshot {
            integers: ints,
            values,
        };
        let decoded = decode_block(&block).unwrap();
        let fixture = sheet([0.0; 3], 1.0, 0);
        let projection = native_projection(&fixture, &decoded).unwrap();
        assert_eq!(projection.kinds, [1]);
        assert_eq!(projection.statuses, [1000; 6]);
        assert_eq!(projection.values.len(), 27);
        assert_eq!(projection.values[1..5], [0.0, 0.0, 2.0, 4.0]);
        assert_eq!(projection.values[21..27], [2.0, 4.0, 0.0, 0.0, 0.0, 1.0]);
        block.integers[20] = 1; // A counted loop list with no valid loop traversal.
        assert!(decode_block(&block).is_err());
        block.integers[20] = 0;
        block.values.pop();
        assert!(decode_block(&block).is_err());
    }

    #[test]
    fn native_only_input_metadata_mutations_are_detected_by_full_local_purity_check() {
        let mut input = input(Operation::ApiOffset);
        input.source = Some(Fixture::Empty);
        let mut snapshot = empty_snapshot(1007);
        let baseline = native_output(&input, &snapshot).unwrap();
        snapshot.source_after.as_mut().unwrap().integers[8] = 1;
        let changed = native_output(&input, &snapshot).unwrap();
        assert_eq!(baseline.values, changed.values);
        assert_eq!(baseline.ints["input.pure"], [1, 1]);
        assert_eq!(changed.ints["input.pure"], [0, 1]);
        snapshot.source_after = snapshot.source_before.clone();
        snapshot.source_after.as_mut().unwrap().values[0] = 2e-6;
        assert_eq!(
            native_output(&input, &snapshot).unwrap().ints["input.pure"],
            [0, 1]
        );
    }

    #[test]
    fn positive_geometry_and_strict_boundaries_reach_real_dependencies() {
        for (operation, fixture, extend, symbol) in [
            (
                Operation::StaticShell,
                Fixture::Empty,
                false,
                "SmBrep::IsManifoldSolid",
            ),
            (Operation::ApiShell, Fixture::Empty, false, "SmBrep::SmBrep"),
            (
                Operation::StaticOffset,
                Fixture::Empty,
                false,
                "SmBrep::ShrinkGeometry",
            ),
            (
                Operation::Solid,
                sheet([0.0; 3], 1.0, 0),
                false,
                "SmBrep::GetInfiniteRegion",
            ),
            (
                Operation::Extended,
                sheet([0.0; 3], 1.0, 0),
                true,
                "SmRegion::IsVoid",
            ),
        ] {
            for distance in [0.0, 1.9999999999999995e-6, 2e-6, 2.0000000000000003e-6] {
                let mut input = input(operation);
                input.source = Some(fixture.clone());
                input.config.offset_distance = distance;
                input.config.extend_convex = extend;
                let output = Runner.run_rust(&case(&input)).unwrap();
                assert_eq!(output.deferred.unwrap().symbol, symbol);
            }
        }
        let box_fixture = Fixture::Box {
            origin: [0.0; 3],
            size: [10.0; 3],
        };
        let brep = build(&box_fixture, 1e-6).unwrap();
        let projection = rust_projection(&box_fixture, &brep).unwrap();
        assert_eq!(projection.counts, [6, 12, 8]);
        assert_eq!(projection.kinds, [1; 6]);
        assert_eq!(projection.values, [1e-6, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0]);
        let mut input = input(Operation::ApiShell);
        input.source = Some(box_fixture);
        input.faces = vec![Selection::Source(0), Selection::Source(0)];
        assert_eq!(
            Runner
                .run_rust(&case(&input))
                .unwrap()
                .deferred
                .unwrap()
                .symbol,
            "SmBrep::SmBrep"
        );
    }

    #[test]
    fn fixed_fixtures_parse_and_case_budget_is_reproducible_without_running_cpp() {
        let context = CaseContext {
            root: repo_root(),
            seeds: 0,
        };
        let fixtures = context.fixture_cases(Runner.id()).unwrap();
        assert!(fixtures.len() >= 40);
        let mut operations = BTreeSet::new();
        for case in &fixtures {
            let input: Inputs = case.parse().unwrap();
            validate(&input).unwrap();
            operations.insert(input.operation.native() as i32);
        }
        assert_eq!(operations.len(), 6);
        let all = Runner.cases(&context).unwrap();
        assert_eq!(all.len(), fixtures.len() + MIN_GENERATED);
        assert_eq!(all, Runner.cases(&context).unwrap());
        assert!(Runner.abs_floors().is_empty());
    }

    #[test]
    fn generated_coverage_is_finite_diverse_nonempty_and_mostly_real_geometry_paths() {
        let mut operations = BTreeSet::new();
        let mut flags = BTreeSet::new();
        let mut enums = BTreeSet::new();
        let (mut positive, mut early, mut boxes, mut wires, mut negatives, mut negative_zeros) =
            (0, 0, 0, 0, 0, 0);
        for index in 0..MIN_GENERATED {
            let seed = CaseContext::seed_for(Runner.id(), index);
            let input = generated(&mut Rng::new(seed));
            validate(&input).unwrap();
            assert_eq!(input, generated(&mut Rng::new(seed)));
            let roundtrip: Inputs =
                serde_json::from_value(serde_json::to_value(&input).unwrap()).unwrap();
            assert_eq!(roundtrip, input);
            operations.insert(input.operation.native() as i32);
            let cfg = &input.config;
            flags.insert((
                cfg.inset,
                cfg.extend_convex,
                cfg.merge_results,
                cfg.create_offset_solid,
                cfg.skip_stitch_merge,
            ));
            enums.insert(cfg.member_operation);
            negatives += usize::from(cfg.offset_distance < 0.0);
            negative_zeros +=
                usize::from(cfg.offset_distance == 0.0 && cfg.offset_distance.is_sign_negative());
            boxes += usize::from(matches!(input.source, Some(Fixture::Box { .. })));
            wires += usize::from(
                input
                    .source
                    .iter()
                    .chain(&input.foreign)
                    .any(|fixture| matches!(fixture, Fixture::Wire { .. })),
            );
            let early_selection = input.operation == Operation::ApiShell
                && input
                    .faces
                    .iter()
                    .any(|face| !matches!(face, Selection::Source(_)));
            let output = Runner.run_rust(&case(&input)).unwrap();
            if input.source.is_none() || early_selection {
                early += 1;
                assert!(output.deferred.is_none());
                assert!(output.error.is_none());
                assert!(!output.values.is_empty());
            } else {
                positive += 1;
                assert!(
                    output.deferred.is_some(),
                    "nonempty geometry must not be guessed: {input:?}"
                );
            }
        }
        assert_eq!(operations.len(), 6);
        assert_eq!(enums.len(), 5);
        assert!(flags.len() >= 24);
        assert!(positive > MIN_GENERATED / 2 && early > MIN_GENERATED / 4);
        assert!(boxes > 25 && wires > 25 && negatives > 40 && negative_zeros > 5);
    }
}
