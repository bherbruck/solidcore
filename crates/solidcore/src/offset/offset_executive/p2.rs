//! Solid offset, extended offset, shell ownership and the full public API prefixes.
//!
//! Upstream: `source/SMLib/src/SmOffsetExecutive.cpp` (100–129, 146–359,
//! 379–1053, 1062–1924), `source/SMLib/inc/SmOffsetExecutive.h`,
//! `source/SM_API/src/SmApiBrep.cpp` (2350–2431, 2438–2463).
//!
//! | Rust | Upstream |
//! |------|----------|
//! | [`offset_brep`] | `SmOffsetExecutive::OffsetBrep` |
//! | [`OffsetExecutive::shell_brep`], [`ShellOutcome`] | both `SmOffsetExecutive::ShellBrep` overloads, including input ownership |
//! | [`OffsetExecutive::do_solid_offset`] | `SmOffsetExecutive::DoSolidOffset` |
//! | [`OffsetExecutive::do_extended_offset`] | `SmOffsetExecutive::DoExtendedOffset` |
//! | [`api_shell_brep_full`] | `SmApiShellBrepFull` |
//! | [`api_offset_brep_full`] | `SmApiOffsetBrepFull` |
//!
//! Deviations: original B-reps and the kernel are explicit arguments, not retained
//! pointers. Options replace scalar flag lists; owned results replace output pointers.
//! A shell outcome returns any separately retained input even on failure; dropping it
//! is the caller's choice. The pure shell API uses existing origin-qualified
//! `FaceKey`/`BrepRef` handles, with caller-unique `BrepId`s, not local face ids alone.
//! Static entry points take non-null references/owned values (null is undefined
//! upstream); member null is native 1006, API null/invalid selection is native 1007.
//! Arena order replaces owning face-list order, as in p1. Empty temporary allocation
//! and tolerance assignment use the existing `Brep` skeleton; its absent editing flag
//! is not simulated. Region void flags, infinite-region lookup, topology copies,
//! constructors and geometry creation are not guessed. Continuations after missing
//! calls are documented at those calls, including ignored-status calls. Scoped
//! Boolean changes and mark locks are restored on every `Result` exit. There is no
//! new recursion; all owned loops are bounded by existing arena/relation lengths.

use std::collections::BTreeSet;

use super::p1::{OffsetExecutive, OffsetOperation};
use crate::error::{Result, SolidError};
use crate::geom::context::{with_mark_lock, ALL_MARKS};
use crate::geom::object::FaceKey;
use crate::kernel::Kernel;
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::query::api_queries::{brep_is_manifold_solid, BrepRef};

use crate::topology::edgeuse::{p1 as eu, p2 as eu2};
use crate::topology::face::p2 as face;
use crate::topology::{Brep, BrepId, EdgeuseId, FaceId, RegionId};

const OP_OFFSET: &str = "offset::offset_executive::p2::offset_brep";
const OP_SHELL: &str = "offset::offset_executive::p2::shell_brep";
const OP_SOLID: &str = "offset::offset_executive::p2::do_solid_offset";
const OP_EXTENDED: &str = "offset::offset_executive::p2::do_extended_offset";
const OP_API_SHELL: &str = "offset::offset_executive::p2::api_shell_brep_full";
const OP_API_OFFSET: &str = "offset::offset_executive::p2::api_offset_brep_full";

/// Signed distance and independent convex/concave offset choices.
///
/// The all-zero default matches the executive's false flags; these arguments have
/// no defaults on the upstream static functions. Negative zero is not an inset.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct OffsetOptions {
    /// `dOffsetDistance`: negative for an inset; the creator receives its absolute value.
    pub distance: f64,
    /// `bDoExtendedOffset`: extend/intersect convex offsets instead of rounding them.
    pub extend_convex_edges: bool,
    /// `bDoSelfInt`: merge and trim concave self-intersections instead of stitching.
    pub merge_results: bool,
}

/// Shell options, including whether to join sheet boundaries to their offsets.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct ShellOptions {
    /// Distance and convex/concave offset flags, forwarded without modification.
    pub offset: OffsetOptions,
    /// `bCreateOffsetSolid`: construct ruled side walls at lamina edges.
    pub create_offset_solid: bool,
}

/// Options for the direct solid-offset member, not for the static entry points.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct SolidOffsetOptions {
    /// `bSkipStitchMerge` (default false). Extended dispatch ignores this flag upstream.
    pub skip_stitch_merge: bool,
}

/// Owned shell result and the input allocation, if native code would retain it.
///
/// `retained_input` is present on failures before consumption and on the successful
/// non-manifold/no-side-walls return. It is absent if the input was consumed or
/// returned as the result: the same B-rep is never represented by two owned values.
/// Currently the real manifold query defers before ownership can transfer.
#[derive(Debug)]
pub struct ShellOutcome {
    /// Native shell status and, on success, its owned B-rep.
    pub result: Result<Brep>,
    /// Input still owned separately by the caller, including on early errors.
    pub retained_input: Option<Brep>,
}

impl ShellOutcome {
    /// The ownership-reporting overload's `rbInputOwnershipTransferred` value.
    pub const fn input_ownership_transferred(&self) -> bool {
        self.retained_input.is_none()
    }
}

/// Static offset entry point, with a non-null original and an owned result.
///
/// Only ordinary inner failures collapse to native `SM_ERR` (1001).
/// [`SolidError::Unported`] propagates so missing dependencies remain deferred.
pub fn offset_brep(kernel: &mut Kernel, original: &Brep, options: OffsetOptions) -> Result<Brep> {
    let mut executive = configured_executive(original, options);
    collapse_offset_status(executive.do_solid_offset(
        kernel,
        Some(original),
        SolidOffsetOptions::default(),
    ))
}

impl OffsetExecutive {
    /// Ownership-reporting static shell entry point (the thin overload needs no
    /// separate Rust method). `faces` are ids in the owned input, not in a copy.
    ///
    /// The manifold query deliberately precedes the strict distance guard. No
    /// caller-visible input is discarded on an early error. Unlike static offset,
    /// ordinary inner errors are returned unchanged.
    pub fn shell_brep(
        kernel: &mut Kernel,
        original: Brep,
        options: ShellOptions,
        faces: &[FaceId],
    ) -> ShellOutcome {
        let result = (|| {
            // cpp:206–215: do not move the guard before this real query.
            let radius = options.offset.distance.abs();
            let tolerance = original.tolerance();
            let input_is_manifold = brep_is_manifold_solid(&original)?;
            check_shell_radius(radius, tolerance)?;

            // cpp:217–251: the helper's scope drops any separately retained
            // executive temporary before querying the result's manifold status.
            let mut shell = run_shell_offset(kernel, &original, options, faces)?;
            if !brep_is_manifold_solid(&shell)? && !options.create_offset_solid {
                return Ok(shell);
            }
            OffsetExecutive::replace_implicit_offsets(kernel, &mut shell, tolerance)?;

            if options.create_offset_solid && !input_is_manifold {
                // Continuation cpp:300–325: only after MergeBrep succeeds is the
                // original consumed. StitchFaces starts at tolerance / 10.0 and
                // runs at most four times, multiplying by 10.0 after each call;
                // stop when no lamina edges remain or the next tolerance exceeds
                // abs(distance) / 2.0. Preserve retained input on MergeBrep error,
                // but report ownership transferred on a later stitch error.
                // UNPORTED(topology.brep_p3): SmBrep::MergeBrep
                return Err(SolidError::unported(
                    "SmBrep::MergeBrep",
                    "topology.brep_p3",
                ));
            }

            // Continuation cpp:328–346: ManifoldBoolean(DIFFERENCE), with tolerance
            // and 20.0 * PI / 180.0. Negative distance uses (original, shell),
            // otherwise (shell, original). Input ownership transfers only on
            // success; ordinary Boolean errors propagate unchanged. Both signs
            // reach the same unavailable function, so do not fake either result.
            // UNPORTED(boolean.merge_p4): SmMerge::ManifoldBoolean
            Err(SolidError::unported(
                "SmMerge::ManifoldBoolean",
                "boolean.merge_p4",
            ))
        })();
        // Every currently executable return retains the input. Consumption and
        // returned-input cases are beyond the explicit missing calls above.
        ShellOutcome {
            result,
            retained_input: Some(original),
        }
    }

    /// Builds a rounded solid-offset prefix, or dispatches to extended offset first.
    ///
    /// A null original returns native 1006. With extended dispatch the Boolean
    /// context flag is untouched, and `skip_stitch_merge` is not examined. Otherwise
    /// the flag is true throughout the operation and restored on every result.
    /// Even an empty input reaches `SmBrep::ShrinkGeometry`, before the skip flag.
    pub fn do_solid_offset(
        &mut self,
        kernel: &mut Kernel,
        original: Option<&Brep>,
        _options: SolidOffsetOptions,
    ) -> Result<Brep> {
        // cpp:411–418: dispatch must precede NER and the Boolean scope.
        if self.extend_convex_edges() {
            return self.do_extended_offset(kernel, original);
        }
        let original = required(original, OP_SOLID)?;
        with_doing_boolean(kernel, |kernel| {
            self.allocate_offset_brep();
            for (id, _) in original.faces().iter() {
                if self.shell_faces().contains(&id) {
                    self.copy_shell_face_prefix(kernel, original, id)?;
                } else {
                    self.offset_face_prefix(original, id, false)?;
                }
            }

            // cpp:608 is unconditional, even with zero faces and skip=true.
            // Continuation cpp:648–848: gather original edgeuses in arena/radial
            // order (the upstream GetAllSeconds test is global, not per-use),
            // claim/release a mark, call p1 BuildOffsetOfEdgeuses with ignored
            // ordinary statuses, optionally build lamina side walls (also ignored),
            // then BuildOffsetOfVertex and its trimmed/natural-domain cap faces.
            // cpp:874–1051: only then may skip_stitch_merge transfer the temporary;
            // otherwise PiecewiseMerge, ignored-status StitchIntoSolid, and
            // MakeManifold. A successful MakeManifold labels every region by
            // GetInfiniteRegion equality; failure is ignored. Finally release the
            // temporary and return the real result. None may be replaced by an
            // empty success here. `_options` is first used at that continuation.
            // UNPORTED(topology.brep_p4): SmBrep::ShrinkGeometry
            Err(SolidError::unported(
                "SmBrep::ShrinkGeometry",
                "topology.brep_p4",
            ))
        })
    }

    /// Builds an extended-offset prefix with the original's face-selection order.
    ///
    /// Unlike rounded offset, this does not set `doing_boolean`. Empty inputs still
    /// claim/release a traversal mark, remove excess faces when requested and call
    /// p1's real `piecewise_merge`; they never bypass stitching with an empty result.
    pub fn do_extended_offset(
        &mut self,
        kernel: &mut Kernel,
        original: Option<&Brep>,
    ) -> Result<Brep> {
        let original = required(original, OP_EXTENDED)?;
        self.allocate_offset_brep();
        for (id, _) in original.faces().iter() {
            if self.shell_faces().contains(&id) {
                self.copy_shell_face_prefix(kernel, original, id)?;
            } else {
                self.offset_face_prefix(original, id, true)?;
            }
        }

        let mapped = self.mapped_original_edgeuses();
        with_mark_lock(kernel, ALL_MARKS, |kernel, _mark_type| {
            // No face constructor is reachable past the boundaries above yet.
            // Thus fresh executives have no mapped edges or result faces here.
            // Still validate caller-installed stale mappings in the native order;
            // the new empty result arena must not make local ids look live.
            let mut marked = BTreeSet::new();
            if self.extend_convex_edges() {
                for &edgeuse in &mapped {
                    let radial = required(eu::radial(original, edgeuse)?, OP_EXTENDED)?;
                    if marked.contains(&radial) {
                        continue;
                    }
                    let Some(radial) = select_radial(original, radial, &mapped, &marked)? else {
                        continue;
                    };
                    if eu2::is_tangent_sector(
                        original,
                        edgeuse,
                        self.tangency_tol_degrees(),
                        false,
                        true,
                    )? {
                        self.mapped_edge_prefix(kernel, original, edgeuse, radial)?;
                        marked.insert(radial);
                        marked.insert(required(eu::mate(original, radial)?, OP_EXTENDED)?);
                        marked.insert(edgeuse);
                        marked.insert(required(eu::mate(original, edgeuse)?, OP_EXTENDED)?);
                    }
                }
            }

            for &edgeuse in &mapped {
                let mut radial = required(eu::radial(original, edgeuse)?, OP_EXTENDED)?;
                marked.insert(edgeuse);
                if marked.contains(&radial) {
                    continue;
                }
                let edge = required(eu::edge(original, edgeuse)?, OP_EXTENDED)?;
                if !eu::edge_is_lamina(original, edge)? {
                    let Some(selected) = select_radial(original, radial, &mapped, &marked)? else {
                        continue;
                    };
                    radial = selected;
                }
                self.mapped_edge_prefix(kernel, original, edgeuse, radial)?;
            }

            // cpp:1580: retain the same temporary on both success and failure.
            let mut temporary = required(self.take_offset_brep(), OP_EXTENDED)?;
            let status = self.remove_excess_faces(kernel, original, &mut temporary);
            self.set_offset_brep(temporary);
            status?;

            if self.create_offset_solid() {
                for &edgeuse in &mapped {
                    let edge = required(eu::edge(original, edgeuse)?, OP_EXTENDED)?;
                    if eu::edge_is_lamina(original, edge)? {
                        // SE, not SER, upstream: propagate Unported, ignore other errors.
                        crate::error::ignore_status(
                            self.build_side_wall_of_edgeuse(kernel, Some(edgeuse)),
                        )?;
                    }
                }
            }

            // cpp:1613–1703: every currently reachable result is empty. Once the
            // face-construction boundaries are wired, this continuation must walk
            // non-marked/non-closing result faces, GetCreateExtendedSurface, then
            // ReplaceSurface(UNSURE, NULL, FALSE) before calculating the trim domain
            // with bLimitToCurrentDomain=FALSE. Keep the original surface if trim
            // fits its natural domain, the extension if trim fits that domain;
            // otherwise restore the original and return SM_ERR. Do not synthesize
            // replacement surfaces or omit this work for nonempty results.
            self.extend_result_faces_prefix(kernel)?;

            let mut result = self.piecewise_merge(kernel, original)?;
            if !self.create_offset_solid() {
                self.remove_excess_faces(kernel, original, &mut result)?;
            }
            // cpp:1772 is called even after stitching in PiecewiseMerge. Its
            // ordinary status is only asserted, but Unported must still defer.
            // Continuation cpp:1836–1920: MakeManifold status is ignored; on success
            // FindAndSetInfiniteRegion when >1 region (editing temporarily false),
            // refresh regions, set void flags by infinite-region identity, then
            // transfer result ownership and clear the executive temporary.
            // UNPORTED(heal.stitch_p2): SmStitch::StitchIntoSolid
            Err(SolidError::unported(
                "SmStitch::StitchIntoSolid",
                "heal.stitch_p2",
            ))
        })?
    }

    fn allocate_offset_brep(&mut self) {
        let mut temporary = Brep::new();
        temporary.set_tolerance(self.approximation_tol_3d());
        self.set_offset_brep(temporary);
    }

    fn copy_shell_face_prefix(&self, kernel: &Kernel, original: &Brep, id: FaceId) -> Result<()> {
        // Both the 3D and UV outputs are requested, in this order, upstream.
        let _curves =
            face::create_curves_from_face(kernel, original, id, OrientType::Same, true, true)?;
        let source_surface = required(original.face(id)?.surface.as_ref(), OP_SOLID)?;
        let _surface = source_surface.copy_and_add_analytics(kernel)?;
        let temporary = required(self.offset_brep(), OP_SOLID)?;
        infinite_region(temporary)?;

        // Continuation cpp:482–506 / 1166–1207: MakeFaceWithCurves, using both
        // copied curve arrays, original UV domain and SAME orientation. Solid uses
        // the original upward faceuse's region IsVoid to select its mate, without
        // applying inset. Extended first notifies Copy, then selects by original
        // GetInfiniteRegion and applies inset. Both relate faceuse->face and call
        // p1 MapEdgeuses. Do not use a topology/geometry clone as this constructor.
        // UNPORTED(topology.brep_p7): SmBrep::MakeFaceWithCurves
        Err(SolidError::unported(
            "SmBrep::MakeFaceWithCurves",
            "topology.brep_p7",
        ))
    }

    fn offset_face_prefix(&self, original: &Brep, id: FaceId, extended: bool) -> Result<()> {
        let operation = if extended { OP_EXTENDED } else { OP_SOLID };
        let mut faceuse = required(face::upward_faceuse(original, id)?, operation)?;
        let shell = required(eu::faceuse_shell(original, faceuse)?, operation)?;
        let region = eu::shell_region(original, shell)?;
        // Do not unify these predicates: extended offset tests void, rounded
        // offset compares region identity. They defer at different real calls.
        let use_mate = if extended {
            !region_is_void(original, required(region, operation)?)?
        } else {
            region != Some(infinite_region(original)?)
        };
        if use_mate {
            faceuse = required(eu::faceuse_mate(original, faceuse)?, operation)?;
        }
        if self.inset() {
            // Validate the final mate now; geometry creation below is deferred.
            required(eu::faceuse_mate(original, faceuse)?, operation)?;
        }

        // FaceuseOffset(context, selected faceuse, TRUE, surface, SSI data), using
        // this executive's immutable distance/approximation/tangency snapshot.
        // Continuation cpp:559–567: rounded offset skips a null surface; extended
        // cpp:1296–1312 instead returns SM_ERR for null or for SSI count >0 with
        // merge_results, or >5 with extend_convex_edges. Then call p1
        // BuildOffsetOfFaceuse, preserving both surface and SSI data. Geometry
        // creation belongs to the open geometry-creation unit, not this prefix.
        // UNPORTED(offset.offset_geometry_creation): SmOffsetGeometryCreation::FaceuseOffset
        Err(SolidError::unported(
            "SmOffsetGeometryCreation::FaceuseOffset",
            "offset.offset_geometry_creation",
        ))
    }

    fn mapped_edge_prefix(
        &mut self,
        kernel: &Kernel,
        original: &Brep,
        edgeuse: EdgeuseId,
        radial: EdgeuseId,
    ) -> Result<()> {
        // The executable prefix of p1 BuildOffsetOfEdgeuses. All successful face
        // constructors above are unavailable, so a preinstalled mapping can only
        // refer to a stale result id: synchronization detects it before any copy
        // notification. No empty notification/attribute store is fabricated here.
        let _edge = required(eu::edge(original, edgeuse)?, OP_EXTENDED)?;
        let first_face = eu::face(original, edgeuse)?;
        let second_face = eu::face(original, radial)?;
        if first_face.is_some_and(|id| self.shell_faces().contains(&id))
            && second_face.is_some_and(|id| self.shell_faces().contains(&id))
            && !self.extend_convex_edges()
        {
            return Ok(());
        }
        self.synchronize_offset_edgeuses(kernel, Some(edgeuse), Some(radial))?;
        // Continuation: invoke p1 BuildOffsetOfEdgeuses with actual original/result
        // notification records once face construction can create live mappings;
        // preserve Copy notifications before the geometry creator and use the
        // claimed mark without advancing it. There is no live mapped pair at this
        // boundary in the current prefix, and no geometric success is claimed.
        // UNPORTED(offset.offset_geometry_creation): SmOffsetGeometryCreation::EdgeuseOffset
        Err(SolidError::unported(
            "SmOffsetGeometryCreation::EdgeuseOffset",
            "offset.offset_geometry_creation",
        ))
    }

    fn extend_result_faces_prefix(&mut self, kernel: &Kernel) -> Result<()> {
        let temporary = required(self.take_offset_brep(), OP_EXTENDED)?;
        let status = (|| {
            for (id, face) in temporary.faces().iter() {
                if self.closing_faces().contains(&id) {
                    continue;
                }
                let surface = required(face.surface.as_ref(), OP_EXTENDED)?;
                // A surface is a concrete, face-owned value. The same canonical
                // arena identity as p1's cache adapter is stable within this run.
                let key = crate::topology::brep_cache::arena_identity(
                    crate::topology::brep_cache::CacheObject::Face(id),
                );
                self.get_create_extended_surface(kernel, key.key.0, surface)?;
                // Continuation documented at cpp:1613 above. In particular, domain
                // calculation must not run before the temporary replacement.
                // UNPORTED(topology.brep_p2): SmBrep::ReplaceSurface
                return Err(SolidError::unported(
                    "SmBrep::ReplaceSurface",
                    "topology.brep_p2",
                ));
            }
            Ok(())
        })();
        self.set_offset_brep(temporary);
        status
    }
}

/// Pure full-shell API over a caller-identified original arena.
///
/// Each selection is `Some((origin_brep_id, face_id))`; `None` represents native
/// null. The caller must assign distinct `BrepId`s to distinct live B-reps. Every
/// selection is validated against the original before attempting its real copy,
/// including when distance is too small. Duplicates/order are not normalized.
/// Source data is immutable and no working copy is faked with `Brep::clone`.
pub fn api_shell_brep_full(
    _kernel: &mut Kernel,
    original: Option<BrepRef<'_, BrepId>>,
    _options: ShellOptions,
    faces: &[Option<FaceKey>],
) -> Result<Brep> {
    let original = original.ok_or_else(|| invalid_input(OP_API_SHELL))?;
    validate_shell_faces(original, faces)?;

    // cpp:2389–2419: the real SmBrep copy constructor and SmCopyBrepMap precede
    // ShellBrep (and hence its distance guard/manifold query). On success, map
    // every selection in order, returning SM_ERR for a missing/non-face mapping;
    // run shell_brep on the private copy with options unchanged. Retained working
    // input is dropped by RAII on both success and failure. A transferred input
    // must never also be cleaned up, and a null successful result is SM_ERR.
    // `_options` is forwarded only after the unavailable copy above succeeds.
    // UNPORTED(topology.brep_p3): SmBrep::SmBrep
    Err(SolidError::unported("SmBrep::SmBrep", "topology.brep_p3"))
}

/// Full-offset API: guard native null with 1007, then call the static executive.
/// Unlike the shell API this wrapper does not make a topology copy.
pub fn api_offset_brep_full(
    kernel: &mut Kernel,
    original: Option<&Brep>,
    options: OffsetOptions,
) -> Result<Brep> {
    let original = original.ok_or_else(|| invalid_input(OP_API_OFFSET))?;
    offset_brep(kernel, original, options)
}

fn configured_executive(original: &Brep, options: OffsetOptions) -> OffsetExecutive {
    let mut executive = OffsetExecutive::new(
        OffsetOperation::SolidOffset,
        options.distance.abs(),
        original.tolerance(),
    );
    executive.set_inset(options.distance < 0.0);
    executive.set_extend_convex_edges(options.extend_convex_edges);
    executive.set_merge_results(options.merge_results);
    executive
}

fn configured_shell_executive(
    original: &Brep,
    options: ShellOptions,
    faces: &[FaceId],
) -> OffsetExecutive {
    let mut executive = configured_executive(original, options.offset);
    executive.set_shell_faces(faces);
    executive.set_create_offset_solid(options.create_offset_solid);
    executive
}

fn run_shell_offset(
    kernel: &mut Kernel,
    original: &Brep,
    options: ShellOptions,
    faces: &[FaceId],
) -> Result<Brep> {
    let mut executive = configured_shell_executive(original, options, faces);
    // Unlike offset_brep, shelling must not collapse ordinary inner statuses.
    executive.do_solid_offset(kernel, Some(original), SolidOffsetOptions::default())
}

fn collapse_offset_status<T>(result: Result<T>) -> Result<T> {
    result.map_err(|error| {
        if error.is_unported() {
            error
        } else {
            SolidError::sm_err(OP_OFFSET)
        }
    })
}

fn check_shell_radius(radius: f64, tolerance: f64) -> Result<()> {
    if radius < 2.0 * tolerance {
        Err(invalid_input(OP_SHELL))
    } else {
        Ok(())
    }
}

fn with_doing_boolean<T>(
    kernel: &mut Kernel,
    run: impl FnOnce(&mut Kernel) -> Result<T>,
) -> Result<T> {
    let previous = kernel.context.doing_boolean();
    kernel.context.set_doing_boolean(true);
    let result = run(kernel);
    kernel.context.set_doing_boolean(previous);
    result
}

fn validate_shell_faces(original: BrepRef<'_, BrepId>, faces: &[Option<FaceKey>]) -> Result<()> {
    for selection in faces {
        let Some((owner, face)) = selection else {
            return Err(invalid_input(OP_API_SHELL));
        };
        // Comparing FaceId alone would accept foreign face 0 as source face 0.
        if *owner != original.handle || original.brep.faces().get(*face).is_none() {
            return Err(invalid_input(OP_API_SHELL));
        }
    }
    Ok(())
}

fn select_radial(
    original: &Brep,
    radial: EdgeuseId,
    mapped: &[EdgeuseId],
    marked: &BTreeSet<EdgeuseId>,
) -> Result<Option<EdgeuseId>> {
    // cpp:1459 / 1532 evaluates GetMate before checking radial membership.
    let Some(mate) = eu::mate(original, radial)? else {
        return Ok(Some(radial));
    };
    if mapped.contains(&mate) && !mapped.contains(&radial) {
        Ok((!marked.contains(&mate)).then_some(mate))
    } else {
        Ok(Some(radial))
    }
}

fn infinite_region(_brep: &Brep) -> Result<RegionId> {
    // UNPORTED(topology.brep_p1): SmBrep::GetInfiniteRegion
    Err(SolidError::unported(
        "SmBrep::GetInfiniteRegion",
        "topology.brep_p1",
    ))
}

fn region_is_void(brep: &Brep, region: RegionId) -> Result<bool> {
    brep.region(region)?;
    // UNPORTED(topology.shell): SmRegion::IsVoid
    Err(SolidError::unported("SmRegion::IsVoid", "topology.shell"))
}

fn required<T>(value: Option<T>, operation: &'static str) -> Result<T> {
    value.ok_or_else(|| SolidError::null_pointer(operation))
}

fn invalid_input(operation: &'static str) -> SolidError {
    SolidError::Upstream {
        operation,
        code: 1007,
        entity: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::{EntityRef, SM_ERR, SM_ERR_NULL_POINTER};
    use crate::geom::context::{MarkLock, MarkType};
    use crate::geom::extent2d::Extent2d;
    use crate::math::{Vec3, PI};
    use crate::surface::plane::Plane;
    use crate::surface::surface::Surface;
    use crate::topology::{Faceuse, FaceuseId, Region, Shell};

    fn executive() -> OffsetExecutive {
        OffsetExecutive::new(OffsetOperation::SolidOffset, 0.125, 1.0e-6)
    }

    fn assert_status<T: core::fmt::Debug>(result: Result<T>, code: i32) {
        assert!(
            matches!(result.unwrap_err(), SolidError::Upstream { code: actual, .. } if actual == code)
        );
    }

    fn assert_dependency<T: core::fmt::Debug>(result: Result<T>, symbol: &str, owner: &str) {
        assert!(
            matches!(result.unwrap_err(), SolidError::Unported { upstream, unit } if upstream == symbol && unit == owner)
        );
    }

    fn one_face(surface: bool) -> (Brep, FaceId) {
        let mut brep = Brep::new();
        let region = brep.add_region_record(Region { in_brep: true }).unwrap();
        let shell = brep
            .add_shell_record(Shell {
                region: Some(region),
                ..Shell::default()
            })
            .unwrap();
        let face = brep.add_face(shell).unwrap();
        let first = brep
            .add_faceuse_record(Faceuse {
                shell: Some(shell),
                face: Some(face),
                orientation: OrientType::Same,
                ..Faceuse::default()
            })
            .unwrap();
        let second = brep
            .add_faceuse_record(Faceuse {
                shell: Some(shell),
                face: Some(face),
                mate: Some(first),
                orientation: OrientType::Opposite,
                ..Faceuse::default()
            })
            .unwrap();
        brep.faceuse_mut(first).unwrap().mate = Some(second);
        let record = brep.face_mut(face).unwrap();
        record.faceuse = Some(first);
        record.uv_domain = Extent2d::from_bounds(-1.0, -1.0, 1.0, 1.0);
        if surface {
            record.surface = Some(Surface::from(Plane::from_normal(Vec3::ZERO, Vec3::Z)));
        }
        (brep, face)
    }

    fn source(brep: &Brep, id: u32) -> BrepRef<'_, BrepId> {
        BrepRef {
            brep,
            handle: BrepId::from_index(id),
        }
    }

    #[test]
    fn static_snapshot_preserves_sign_absolute_distance_tolerance_and_flags() {
        let mut brep = Brep::new();
        brep.set_tolerance(3.25e-7);
        for distance in [0.125, -0.125, 0.0, -0.0, f64::INFINITY, f64::NEG_INFINITY] {
            for extend_convex_edges in [false, true] {
                for merge_results in [false, true] {
                    let options = OffsetOptions {
                        distance,
                        extend_convex_edges,
                        merge_results,
                    };
                    let exec = configured_executive(&brep, options);
                    assert_eq!(exec.operation(), OffsetOperation::SolidOffset);
                    assert_eq!(exec.offset_distance().to_bits(), distance.abs().to_bits());
                    assert_eq!(exec.inset(), distance < 0.0);
                    assert_eq!(exec.approximation_tol_3d(), brep.tolerance());
                    assert_eq!(exec.tangency_tol_degrees(), (2.0 * PI / 180.0) * 180.0 / PI);
                    assert_eq!(exec.extend_convex_edges(), extend_convex_edges);
                    assert_eq!(exec.merge_results(), merge_results);
                    assert!(!exec.create_offset_solid());
                    assert!(exec.shell_faces().is_empty());
                }
            }
        }
        let exec = configured_executive(
            &brep,
            OffsetOptions {
                distance: f64::NAN,
                ..OffsetOptions::default()
            },
        );
        assert!(exec.offset_distance().is_nan());
        assert!(!exec.inset());
    }

    #[test]
    fn shell_snapshot_keeps_all_flags_and_the_ordered_duplicate_face_list() {
        let mut original = Brep::new();
        original.set_tolerance(6.5e-7);
        let faces = [
            FaceId::from_index(9),
            FaceId::from_index(3),
            FaceId::from_index(9),
        ];
        for extend_convex_edges in [false, true] {
            for merge_results in [false, true] {
                for create_offset_solid in [false, true] {
                    let options = ShellOptions {
                        offset: OffsetOptions {
                            distance: -0.25,
                            extend_convex_edges,
                            merge_results,
                        },
                        create_offset_solid,
                    };
                    let exec = configured_shell_executive(&original, options, &faces);
                    assert_eq!(exec.operation(), OffsetOperation::SolidOffset);
                    assert_eq!(exec.offset_distance(), 0.25);
                    assert!(exec.inset());
                    assert_eq!(exec.approximation_tol_3d(), original.tolerance());
                    assert_eq!(exec.extend_convex_edges(), extend_convex_edges);
                    assert_eq!(exec.merge_results(), merge_results);
                    assert_eq!(exec.create_offset_solid(), create_offset_solid);
                    assert_eq!(exec.shell_faces(), &faces);
                }
            }
        }
    }

    #[test]
    fn shell_inner_ordinary_status_is_not_collapsed_like_static_offset() {
        let mut original = Brep::new();
        let shell = original.add_shell().unwrap();
        original.add_face(shell).unwrap();
        let mut kernel = Kernel::default();
        // Exercise exactly the post-query shell helper: the real public shell
        // entry point must still defer at IsManifoldSolid before reaching it.
        assert_status(
            run_shell_offset(&mut kernel, &original, ShellOptions::default(), &[]),
            SM_ERR_NULL_POINTER,
        );
        assert_status(
            offset_brep(&mut kernel, &original, OffsetOptions::default()),
            SM_ERR,
        );
        assert!(!kernel.context.doing_boolean());
    }

    #[test]
    fn defaults_are_false_and_the_direct_skip_default_is_false() {
        assert_eq!(OffsetOptions::default().distance, 0.0);
        assert!(!OffsetOptions::default().extend_convex_edges);
        assert!(!OffsetOptions::default().merge_results);
        assert!(!ShellOptions::default().create_offset_solid);
        assert!(!SolidOffsetOptions::default().skip_stitch_merge);
    }

    #[test]
    fn native_api_null_and_member_null_statuses_are_distinct() {
        let mut kernel = Kernel::default();
        assert_status(
            api_offset_brep_full(&mut kernel, None, OffsetOptions::default()),
            1007,
        );
        assert_status(
            api_shell_brep_full(&mut kernel, None, ShellOptions::default(), &[None]),
            1007,
        );
        for extended in [false, true] {
            let mut exec = executive();
            exec.set_extend_convex_edges(extended);
            assert_status(
                exec.do_solid_offset(&mut kernel, None, SolidOffsetOptions::default()),
                SM_ERR_NULL_POINTER,
            );
            assert!(exec.offset_brep().is_none());
            assert!(!kernel.context.doing_boolean());
        }
        assert_status(
            executive().do_extended_offset(&mut kernel, None),
            SM_ERR_NULL_POINTER,
        );
    }

    #[test]
    fn extended_dispatch_ignores_skip_and_never_enters_boolean_scope() {
        let (original, _) = one_face(false);
        for previous in [false, true] {
            for skip_stitch_merge in [false, true] {
                let mut kernel = Kernel::default();
                kernel.context.set_doing_boolean(previous);
                let mut exec = executive();
                exec.set_extend_convex_edges(true);
                assert_dependency(
                    exec.do_solid_offset(
                        &mut kernel,
                        Some(&original),
                        SolidOffsetOptions { skip_stitch_merge },
                    ),
                    "SmRegion::IsVoid",
                    "topology.shell",
                );
                assert_eq!(kernel.context.doing_boolean(), previous);
                assert_eq!(exec.offset_brep().unwrap().tolerance(), 1.0e-6);
            }
        }
    }

    #[test]
    fn boolean_scope_restores_on_success_ordinary_error_and_deferred_error() {
        for previous in [false, true] {
            let mut kernel = Kernel::default();
            kernel.context.set_doing_boolean(previous);
            assert_eq!(
                with_doing_boolean(&mut kernel, |kernel| {
                    assert!(kernel.context.doing_boolean());
                    Ok(37)
                }),
                Ok(37)
            );
            assert_eq!(kernel.context.doing_boolean(), previous);
            for error in [
                SolidError::sm_err(OP_SOLID),
                SolidError::unported("SmBrep::ShrinkGeometry", "topology.brep_p4"),
            ] {
                let result: Result<()> = with_doing_boolean(&mut kernel, |kernel| {
                    assert!(kernel.context.doing_boolean());
                    Err(error.clone())
                });
                assert_eq!(result, Err(error));
                assert_eq!(kernel.context.doing_boolean(), previous);
            }
        }
    }

    #[test]
    fn rounded_face_offset_stops_at_real_infinite_region_before_geometry_creation() {
        let (original, _) = one_face(false);
        let mut kernel = Kernel::default();
        for inset in [false, true] {
            let mut exec = executive();
            exec.set_inset(inset);
            assert_dependency(
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default()),
                "SmBrep::GetInfiniteRegion",
                "topology.brep_p1",
            );
            assert!(!kernel.context.doing_boolean());
            assert!(exec.offset_brep().unwrap().faces().is_empty());
        }
    }

    #[test]
    fn faces_are_processed_in_arena_order_without_grouping_selected_faces_first() {
        let (mut original, first) = one_face(false);
        let shell = original.face(first).unwrap().shell;
        let second = original.add_face(shell).unwrap();
        let mut kernel = Kernel::default();
        for extended in [false, true] {
            let mut exec = executive();
            exec.set_extend_convex_edges(extended);
            exec.set_shell_faces(&[second]);
            assert_dependency(
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default()),
                if extended {
                    "SmRegion::IsVoid"
                } else {
                    "SmBrep::GetInfiniteRegion"
                },
                if extended {
                    "topology.shell"
                } else {
                    "topology.brep_p1"
                },
            );
            let mut exec = executive();
            exec.set_extend_convex_edges(extended);
            exec.set_shell_faces(&[first]);
            // The first face's copy prefix reaches its missing surface before
            // even inspecting the malformed second face's absent faceuse.
            assert_status(
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default()),
                SM_ERR_NULL_POINTER,
            );
        }
    }

    #[test]
    fn empty_solid_always_shrinks_before_skip_or_merge() {
        let original = Brep::new();
        for previous in [false, true] {
            for skip_stitch_merge in [false, true] {
                for merge in [false, true] {
                    let mut kernel = Kernel::default();
                    kernel.context.set_doing_boolean(previous);
                    let mut exec = executive();
                    exec.set_merge_results(merge);
                    exec.set_create_offset_solid(true);
                    assert_dependency(
                        exec.do_solid_offset(
                            &mut kernel,
                            Some(&original),
                            SolidOffsetOptions { skip_stitch_merge },
                        ),
                        "SmBrep::ShrinkGeometry",
                        "topology.brep_p4",
                    );
                    assert_eq!(kernel.context.doing_boolean(), previous);
                    let result = exec.offset_brep().unwrap();
                    assert!(result.faces().is_empty());
                    assert_eq!(result.tolerance(), exec.approximation_tol_3d());
                    assert_eq!(kernel.context.current_mark(MarkType::Mark), 1);
                }
            }
        }
    }

    #[test]
    fn copied_face_requests_real_curves_and_surface_before_infinite_region() {
        let mut kernel = Kernel::default();
        for extended in [false, true] {
            let (original, face) = one_face(false);
            let mut exec = executive();
            exec.set_shell_faces(&[face, face]);
            exec.set_extend_convex_edges(extended);
            assert_status(
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default()),
                SM_ERR_NULL_POINTER,
            );
            assert_eq!(exec.shell_faces(), &[face, face]);
            assert!(!kernel.context.doing_boolean());
        }
        for extended in [false, true] {
            let (original, face) = one_face(true);
            let mut exec = executive();
            exec.set_shell_faces(&[face]);
            exec.set_extend_convex_edges(extended);
            assert_dependency(
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default()),
                "SmBrep::GetInfiniteRegion",
                "topology.brep_p1",
            );
            assert!(exec.offset_brep().unwrap().faces().is_empty());
        }
    }

    #[test]
    fn empty_extended_uses_p1_stitch_choice_and_releases_its_mark_on_error() {
        let original = Brep::new();
        for create_solid in [false, true] {
            for previous in [false, true] {
                let mut kernel = Kernel::default();
                kernel.context.set_doing_boolean(previous);
                let mut exec = executive();
                exec.set_create_offset_solid(create_solid);
                assert_dependency(
                    exec.do_extended_offset(&mut kernel, Some(&original)),
                    if create_solid {
                        "SmStitch::StitchIntoShell"
                    } else {
                        "SmStitch::StitchIntoSolid"
                    },
                    "heal.stitch_p2",
                );
                assert!(exec.offset_brep().is_none());
                assert_eq!(kernel.context.current_mark(MarkType::Mark), 3);
                assert!(!kernel.context.is_mark_locked(MarkType::Mark));
                assert_eq!(kernel.context.doing_boolean(), previous);
            }
        }
    }

    #[test]
    fn extended_scope_preserves_an_outer_mark_lock_and_releases_its_own() {
        let original = Brep::new();
        let mut kernel = Kernel::default();
        let mut outer =
            MarkLock::new(std::slice::from_mut(&mut kernel.context), 0, ALL_MARKS).unwrap();
        assert_eq!(outer.mark_type(), MarkType::Mark);
        assert_dependency(
            executive().do_extended_offset(&mut kernel, Some(&original)),
            "SmStitch::StitchIntoSolid",
            "heal.stitch_p2",
        );
        assert!(kernel.context.is_mark_locked(MarkType::Mark));
        assert!(!kernel.context.is_mark_locked(MarkType::Mark2));
        assert_eq!(kernel.context.current_mark(MarkType::Mark), 3);
        assert_eq!(kernel.context.current_mark(MarkType::Mark2), 3);
        outer
            .release(std::slice::from_mut(&mut kernel.context))
            .unwrap();
    }

    #[test]
    fn stale_mapped_edgeuse_is_not_silently_skipped_on_the_empty_extended_path() {
        let original = Brep::new();
        for extended in [false, true] {
            let mut kernel = Kernel::default();
            let mut exec = executive();
            exec.set_extend_convex_edges(extended);
            let missing = EdgeuseId::from_index(9);
            exec.relate_edgeuses(missing, EdgeuseId::from_index(4));
            let result = exec.do_extended_offset(&mut kernel, Some(&original));
            assert!(matches!(
                result,
                Err(SolidError::NotFound { entity: EntityRef::Edgeuse(id), .. })
                    if id == missing
            ));
            assert!(exec.offset_brep().is_some());
            assert_eq!(kernel.context.current_mark(MarkType::Mark), 3);
            assert!(!kernel.context.is_mark_locked(MarkType::Mark));
        }
    }

    #[test]
    fn empty_extended_merge_reaches_remove_faces_even_without_candidate_faces() {
        let original = Brep::new();
        let mut kernel = Kernel::default();
        let mut exec = executive();
        exec.set_merge_results(true);
        assert_dependency(
            exec.do_extended_offset(&mut kernel, Some(&original)),
            "SmBrep::RemoveFaces",
            "topology.brep_p5",
        );
        assert!(exec.offset_brep().is_some());
        assert_eq!(kernel.context.current_mark(MarkType::Mark), 3);
        assert!(!kernel.context.is_mark_locked(MarkType::Mark));
    }

    #[test]
    fn nonempty_shell_list_skips_remove_faces_but_not_piecewise_merge_shrink() {
        let original = Brep::new();
        let mut kernel = Kernel::default();
        let mut exec = executive();
        exec.set_merge_results(true);
        exec.set_shell_faces(&[FaceId::from_index(0)]);
        assert_dependency(
            exec.do_extended_offset(&mut kernel, Some(&original)),
            "SmBrep::ShrinkGeometry",
            "topology.brep_p4",
        );
        assert!(exec.offset_brep().is_some());
        assert!(!kernel.context.is_mark_locked(MarkType::Mark));
    }

    #[test]
    fn static_offset_collapses_only_ordinary_statuses() {
        for error in [
            SolidError::null_pointer(OP_SOLID),
            SolidError::invalid_input(OP_SOLID, "bad topology"),
            SolidError::Upstream {
                operation: OP_SOLID,
                code: 1023,
                entity: None,
            },
        ] {
            assert_status(collapse_offset_status::<()>(Err(error)), SM_ERR);
        }
        assert_eq!(collapse_offset_status(Ok(17)), Ok(17));
        assert_dependency(
            collapse_offset_status::<()>(Err(SolidError::unported(
                "SmBrep::ShrinkGeometry",
                "topology.brep_p4",
            ))),
            "SmBrep::ShrinkGeometry",
            "topology.brep_p4",
        );

        let mut original = Brep::new();
        let shell = original.add_shell().unwrap();
        original.add_face(shell).unwrap(); // native invalid/dereferenced faceuse
        let mut kernel = Kernel::default();
        assert_status(
            offset_brep(&mut kernel, &original, OffsetOptions::default()),
            SM_ERR,
        );
        assert!(!kernel.context.doing_boolean());
    }

    #[test]
    fn static_and_api_offset_flags_select_the_real_empty_extended_path() {
        let original = Brep::new();
        for extended in [false, true] {
            let mut kernel = Kernel::default();
            let options = OffsetOptions {
                distance: -0.25,
                extend_convex_edges: extended,
                merge_results: false,
            };
            assert_dependency(
                api_offset_brep_full(&mut kernel, Some(&original), options),
                if extended {
                    "SmStitch::StitchIntoSolid"
                } else {
                    "SmBrep::ShrinkGeometry"
                },
                if extended {
                    "heal.stitch_p2"
                } else {
                    "topology.brep_p4"
                },
            );
            assert!(!kernel.context.doing_boolean());
        }
    }

    #[test]
    fn shell_manifold_query_precedes_strict_distance_guard_and_retains_input() {
        for distance in [0.0, -0.0, 1.0e-30, -1.0e-30, 0.25, -0.25] {
            let mut original = Brep::new();
            original.set_tolerance(1.0e-5);
            let mut kernel = Kernel::default();
            let outcome = OffsetExecutive::shell_brep(
                &mut kernel,
                original,
                ShellOptions {
                    offset: OffsetOptions {
                        distance,
                        extend_convex_edges: true,
                        merge_results: true,
                    },
                    create_offset_solid: true,
                },
                &[],
            );
            assert!(!outcome.input_ownership_transferred());
            assert_eq!(outcome.retained_input.as_ref().unwrap().tolerance(), 1.0e-5);
            assert_dependency(
                outcome.result,
                "SmBrep::IsManifoldSolid",
                "topology.brep_p2",
            );
            assert!(!kernel.context.doing_boolean());
            assert_eq!(kernel.context.current_mark(MarkType::Mark), 1);
        }
    }

    #[test]
    fn shell_distance_boundary_is_strict_without_extra_finite_checks() {
        let tolerance: f64 = 0.125;
        let boundary = 2.0 * tolerance;
        assert_status(check_shell_radius(0.0, tolerance), 1007);
        assert_status(check_shell_radius(-0.0, tolerance), 1007);
        assert_status(
            check_shell_radius(f64::from_bits(boundary.to_bits() - 1), tolerance),
            1007,
        );
        assert_eq!(check_shell_radius(boundary, tolerance), Ok(()));
        assert_eq!(
            check_shell_radius(f64::from_bits(boundary.to_bits() + 1), tolerance),
            Ok(())
        );
        assert_eq!(check_shell_radius(f64::INFINITY, tolerance), Ok(()));
        assert_eq!(check_shell_radius(f64::NAN, tolerance), Ok(()));
        assert_eq!(check_shell_radius(0.0, 0.0), Ok(()));
        assert_eq!(check_shell_radius(0.0, -1.0), Ok(()));
    }

    #[test]
    fn shell_api_rejects_null_foreign_and_out_of_range_selection_before_copy() {
        let (original, face) = one_face(true);
        let mut kernel = Kernel::default();
        let good = Some((BrepId::from_index(7), face));
        for faces in [
            vec![None],
            vec![good, None],
            vec![Some((BrepId::from_index(8), face))],
            vec![good, Some((BrepId::from_index(8), face))],
            vec![Some((BrepId::from_index(7), FaceId::from_index(99)))],
        ] {
            assert_status(
                api_shell_brep_full(
                    &mut kernel,
                    Some(source(&original, 7)),
                    ShellOptions::default(),
                    &faces,
                ),
                1007,
            );
        }
        assert!(original.face(face).unwrap().surface.is_some());
        assert_eq!(original.faces().len(), 1);
        assert_eq!(kernel.context.current_mark(MarkType::Mark), 1);
    }

    #[test]
    fn foreign_face_zero_cannot_alias_source_face_zero() {
        let (original, original_face) = one_face(true);
        let (foreign, foreign_face) = one_face(true);
        assert_eq!(original_face, foreign_face);
        assert_eq!(original_face.index(), 0);
        let mut kernel = Kernel::default();
        assert_status(
            api_shell_brep_full(
                &mut kernel,
                Some(source(&original, 7)),
                ShellOptions::default(),
                &[Some((source(&foreign, 8).handle, foreign_face))],
            ),
            1007,
        );
        assert_dependency(
            api_shell_brep_full(
                &mut kernel,
                Some(source(&original, 7)),
                ShellOptions::default(),
                &[Some((BrepId::from_index(7), original_face))],
            ),
            "SmBrep::SmBrep",
            "topology.brep_p3",
        );
    }

    #[test]
    fn api_validates_every_selection_and_keeps_duplicates_before_actual_copy() {
        let (original, face) = one_face(true);
        let good = Some((BrepId::from_index(3), face));
        assert_eq!(
            validate_shell_faces(source(&original, 3), &[good, good]),
            Ok(())
        );
        let mut kernel = Kernel::default();
        for distance in [0.0, -0.0, -0.5, 0.5] {
            assert_dependency(
                api_shell_brep_full(
                    &mut kernel,
                    Some(source(&original, 3)),
                    ShellOptions {
                        offset: OffsetOptions {
                            distance,
                            extend_convex_edges: true,
                            merge_results: true,
                        },
                        create_offset_solid: true,
                    },
                    &[good, good],
                ),
                "SmBrep::SmBrep",
                "topology.brep_p3",
            );
        }
        // An empty list still copies; it does not jump to IsManifoldSolid or the
        // shell distance check. No source arena or geometry is changed.
        assert_dependency(
            api_shell_brep_full(
                &mut kernel,
                Some(source(&original, 3)),
                ShellOptions::default(),
                &[],
            ),
            "SmBrep::SmBrep",
            "topology.brep_p3",
        );
        assert_eq!(original.faces().len(), 1);
        assert_eq!(
            original.face(face).unwrap().uv_domain,
            Extent2d::from_bounds(-1.0, -1.0, 1.0, 1.0)
        );
    }

    #[test]
    fn malformed_face_error_restores_context_and_retains_allocated_temporary() {
        let mut original = Brep::new();
        let shell = original.add_shell().unwrap();
        let face = original.add_face(shell).unwrap();
        original.face_mut(face).unwrap().faceuse = Some(FaceuseId::from_index(99));
        for previous in [false, true] {
            let mut kernel = Kernel::default();
            kernel.context.set_doing_boolean(previous);
            let mut exec = executive();
            let result =
                exec.do_solid_offset(&mut kernel, Some(&original), SolidOffsetOptions::default());
            assert!(matches!(
                result,
                Err(SolidError::NotFound {
                    entity: EntityRef::Faceuse(id),
                    ..
                }) if id == FaceuseId::from_index(99)
            ));
            assert_eq!(kernel.context.doing_boolean(), previous);
            assert_eq!(exec.offset_brep().unwrap().tolerance(), 1.0e-6);
        }
    }
}
