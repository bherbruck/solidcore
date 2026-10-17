# Primitive creation fixture records

A case is `{ "op": N, "data": [...] }`. The checked FFI wrapper validates the
entire record before either engine runs. A curve block contains seven numbers:
`[kind, x, y, z, a, b, c]`. Kind 0 builds a real B-spline line from xyz to abc;
kind 1 builds a quadratic B-spline circular arc centred at xyz, radius a and
increasing angles b/c in degrees. These are the actual kinds used by the upstream
file-local helpers, not replacements for analytic topology fixtures.

| op | data |
|---:|------|
| 0 | `tag analytics clear copy`, then four maps of four target indices; `-1` means absent, `0..3` are stable fixture topology identities. The native sweep starts with two stale entries. |
| 1 | first curve, second curve, tolerance |
| 2 | `closed periodic extension_distance count`, then the original path curves |
| 3 | `profile_count existing_output_count tolerance`, path curve, profiles, existing output curves |
| 4 | primitive selector, seven parameters, frame origin XYZ, X axis XYZ, Y axis XYZ, initial edit flag, initial map flag |
| 5 | curve, rotation-axis base XYZ, axis direction XYZ |

Primitive selectors are 0 static rectangle, 1 static circle, 2 box, 3 cylindrical
box, 4 full cone, 5 full sphere, 6 full torus, 7 pole-free sphere, 8 elliptic-ended
cone, 9 empty curve sweep and 10 empty planar boolean. Parameters follow the
corresponding upstream signature; parameter 7 is the static primitive tolerance,
pole-free sphere tolerance or cap flag. For an empty boolean, parameter 1 is its
operation and parameter 6 seeds an already-populated result slot.

Maps compare counts and fixed-order lookups rather than native hash iteration.
Curve outputs compare their natural intervals and position/first derivative at
fractions 0, 1/4, 1/2, 3/4 and 1. Primitive records compare actual statuses, edit/map
state and topology counts where Rust reaches a defined result. A Rust `Unported`
classifies the whole case as deferred; native-only geometry/topology is not parity.

The full-span cylindrical box is rejected by the exact native three-point-circle
preflight (UB-57). Rejected inputs never count as passes. The resource/geometry
limits of these native fixture families are documented in the checked FFI module.

`overflow.json` retains eight fuzz regressions. Four profile placements preserve
X after `MakeUnitOrthoVectors` writes it and subsequently fails. Four two-point
G1 extensions are rejected before NLib's failed-normalization fallback reads a
third control point outside its array. The preflight follows the original join
and closed/periodic decisions, so huge lines that require no extension remain
accepted and compared in the two additional witnesses.
