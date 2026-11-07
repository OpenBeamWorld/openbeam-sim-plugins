/*
    This source file is part of Rigs of Rods

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

/// @file JBeamStructuralIR.h
/// @brief Bounded, deterministic structural subset of a resolved JBeam graph.

#pragma once

#include "JBeamExpressionEvaluator.h"
#include "JBeamPartResolver.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace RoR {
namespace BeamNG {

enum class JBeamStructuralSeverity
{
    WARNING,
    ERROR_SEVERITY
};

enum class JBeamStructuralDiagnosticCode
{
    INVALID_RESOLVED_GRAPH,
    RESOLVED_PART_LIMIT,
    ROW_LIMIT,
    NODE_LIMIT,
    BEAM_LIMIT,
    TRIANGLE_LIMIT,
    DIAGNOSTIC_LIMIT,
    RETAINED_BYTE_LIMIT,
    PRESERVED_VALUE_LIMIT,
    NORMALIZATION_ERROR,
    NORMALIZATION_WARNING,
    PART_BODY_NOT_OBJECT,
    DUPLICATE_SECTION,
    INVALID_SECTION,
    INVALID_TABLE_HEADER,
    DUPLICATE_TABLE_HEADER,
    INVALID_TABLE_ROW,
    MISSING_REQUIRED_FIELD,
    AMBIGUOUS_REQUIRED_FIELD,
    INVALID_FIELD_TYPE,
    EXPRESSION_ERROR,
    EXPRESSION_LIMIT,
    EXPRESSION_DISABLED,
    UNSUPPORTED_COMPONENT_VALUE,
    INVALID_COMPONENT_PATH,
    INVALID_VARIABLE_VALUE,
    NON_FINITE_NUMBER,
    INVALID_NODE_WEIGHT,
    INVALID_NODE_FRICTION,
    INVALID_BEAM_PARAMETER,
    DUPLICATE_NODE_ID,
    MISSING_NODE_REFERENCE,
    OPTIONAL_BEAM_SKIPPED,
    OPTIONAL_SURFACE_SKIPPED,
    DUPLICATE_VERTEX,
    DEGENERATE_BEAM,
    DEGENERATE_TRIANGLE,
    UNSUPPORTED_TRIANGLE_TYPE,
    SPECIAL_BEAM_TYPE_DISABLED,
    UNKNOWN_SECTION,
    UNKNOWN_FIELD,
    UNSUPPORTED_FIELD,
    MISSING_REF_NODES,
    DUPLICATE_REF_NODES,
    DEGENERATE_REF_NODES,
    MISALIGNED_REF_NODES,
    MISALIGNED_REF_CORNERS,
    INVALID_SLOT_NODE_OFFSET,
    AERODYNAMICS_UNRESOLVED
};

struct JBeamStructuralPartIdentity
{
    std::size_t part_preorder_index;
    std::string part_name;
    std::string package_path;

    JBeamStructuralPartIdentity();
};

struct JBeamStructuralProvenance
{
    /// Shared by every entity and diagnostic originating from one part.
    std::shared_ptr<const JBeamStructuralPartIdentity> part;
    /// Interned independently because malformed hand-built graphs may attach
    /// spans from a source other than the part's defining source.
    std::shared_ptr<const std::string> source_name;
    JBeamSourcePosition begin;
    JBeamSourcePosition end;

    JBeamStructuralProvenance();
    std::size_t PartPreorderIndex() const;
    const std::string& PartName() const;
    const std::string& PackagePath() const;
    const std::string& SourceName() const;
    JBeamSourceSpan SourceSpan() const;
};

enum class JBeamAeroScalarOrigin
{
    DOCUMENTED_DEFAULT,
    INHERITED_DEFAULT,
    POSITIONAL_CELL,
    ROW_LOCAL_OVERRIDE,
    PART_SCALE,
    DEFAULT_FROM_DRAG
};

/// Metadata only. Percentages are NOT dimensionless physical Cd/Cl values.
/// An unresolved authored expression never becomes a numeric default. Values
/// follow this IR's normalized table-local scope; cross-part modifier leakage
/// is not a runtime-admitted BeamNG merge profile. RESOLVED means numeric data,
/// not a physically admissible coefficient range or a force implementation.
struct JBeamAeroScalar
{
    JBeamAeroScalar();
    JBeamAeroScalarOrigin origin;
    bool resolved;
    double value;
    bool effective_resolved;
    double effective_value;
    JBeamStructuralProvenance provenance;
    std::shared_ptr<const JBeamValue> authored_value;
};

enum class JBeamAeroParameter
{
    DRAG_PERCENT,
    SKIN_DRAG_PERCENT,
    LIFT_PERCENT,
    STALL_ANGLE_RADIANS
};

struct JBeamAeroScaleAssignment
{
    JBeamAeroParameter parameter;
    JBeamAeroScalar scalar;
};

enum class JBeamAeroCapability
{
    RESOLVED_PARAMETERS_NOT_APPLIED,
    UNRESOLVED_PARAMETERS_NOT_APPLIED
};

enum class JBeamAeroAreaSource
{
    CURRENT_REFERENCED_NODE_POSITIONS,
    UNAVAILABLE_OPTIONAL_NODE_REFERENCE
};

struct JBeamTriangleAerodynamics
{
    JBeamTriangleAerodynamics();
    JBeamAeroScalar drag_percent;
    JBeamAeroScalar skin_drag_percent;
    JBeamAeroScalar lift_percent;
    JBeamAeroScalar stall_angle_rad;
    JBeamAeroCapability capability;
    JBeamAeroAreaSource area_source;
    /// Node a,b,c order defines the CCW front; both sides have authored aero.
    /// Quad halves retain (1,2,3)/(1,3,4), never sorted vertex order.
    /// Area must eventually come from those live nodes, not render meshes or
    /// node count. No runtime force, occlusion, or break lifecycle is bound.
};

/// Unsupported data is retained as an owned parsed value. Header-only
/// diagnostics have has_preserved_value == false.
struct JBeamStructuralDiagnostic
{
    JBeamStructuralDiagnosticCode code;
    JBeamStructuralSeverity severity;
    JBeamStructuralProvenance provenance;
    std::string section;
    std::size_t row_index;
    std::string field_name;
    std::string detail;
    bool has_preserved_value;
    std::shared_ptr<const JBeamValue> preserved_value;

    JBeamStructuralDiagnostic();
};

struct JBeamStructuralLimits
{
    std::size_t max_parts;
    /// Counts every post-header entry, including defaults and malformed rows.
    std::size_t max_rows;
    std::size_t max_nodes;
    /// Counts every authored beam row, including disabled optional/special rows.
    std::size_t max_beams;
    /// Counts emitted triangle primitives; each quad reserves two.
    std::size_t max_triangles;
    /// Zero still retains one terminal error diagnostic.
    std::size_t max_diagnostics;
    /// Bounds dynamic string/value payload retained by the result. Shared
    /// provenance strings are charged only once.
    std::size_t max_retained_bytes;
    /// Bounds graph work while measuring a preserved unsupported value. This
    /// also protects hand-built cyclic or unusually deep value graphs.
    std::size_t max_preserved_value_work_units;
    std::size_t max_preserved_value_depth;
    /// Per-expression bounds inherited by the allowlisted scalar evaluator.
    JBeamExpressionLimits expression_limits;
    /// Aggregate bounds across component discovery, inherited-variable
    /// materialization, and all structural-field evaluations. These prevent a
    /// large but individually valid table from multiplying the evaluator's
    /// per-expression allowance.
    std::size_t max_expression_evaluations;
    std::size_t max_expression_work_units;
    std::size_t max_component_nodes;
    std::size_t max_component_depth;
    /// Bounds canonical serialization. Serialization returns an empty string
    /// rather than constructing a partial or oversized identity.
    std::size_t max_canonical_output_bytes;
    /// Bounds top-level entities, diagnostics, and preserved-value nodes
    /// visited by canonical serialization, including hand-built IR objects.
    std::size_t max_canonical_work_units;

    JBeamStructuralLimits();
};

struct JBeamStructuralPart
{
    JBeamStructuralProvenance provenance;
    /// Resolved mounting translation, before mirroring X by the authored
    /// node/flexbody position. Shared with visual placement; never re-evaluate
    /// slot expressions in a different variable scope.
    double slot_node_offset[3] = {0.0, 0.0, 0.0};
    /// The part's resolved configuration, slot and authored-default variable
    /// assignments, each evaluated exactly as this tier evaluated them for
    /// its own fields (later assignments win). Null when conversion stopped
    /// before environments were built. Read it through
    /// EvaluateJBeamStructuralPartNumber; never re-derive the scope.
    std::shared_ptr<const std::vector<JBeamExpressionVariable> >
        expression_variables;
};

struct JBeamStructuralNode
{
    JBeamStructuralNode();

    std::string id;
    double x;
    double y;
    double z;
    double node_weight;
    bool node_weight_authored;
    /// Effective structural-node coefficients, not wheel-plan multipliers.
    /// BeamNG documents frictionCoef=1; absent slidingFrictionCoef follows
    /// that node's effective frictionCoef. Both must be finite/nonnegative.
    /// https://documentation.beamng.com/modding/vehicle/sections/nodes/#friction
    /// Presence includes normalized defaults/positional/local assignments in
    /// this part's scope. Retention does not imply a native contact law can
    /// apply two distinct coefficients; adapters must prove representability.
    double friction_coef;
    bool friction_coef_authored;
    double sliding_friction_coef;
    bool sliding_friction_coef_authored;
    /// BeamNG node collision defaults to enabled. The initial compatibility
    /// slice maps false to RoR's no-ground/static-contact node option and
    /// leaves explicit self-collision contacters disabled.
    bool collision;
    /// BeamNG defaults self-collision off. A true value is retained here but
    /// may lower only when `collision` is false; otherwise RoR needs a native
    /// group-aware node/triangle implementation rather than a generic
    /// contacter approximation.
    bool self_collision;
    /// BeamNG defaults static/heightmap collision on. A false value is retained
    /// here but may lower only when `collision` is false; RoR's current node
    /// flag cannot disable terrain while preserving external dynamic contact.
    bool static_collision;
    /// BeamNG documents `fixed` (boolean, default false) as "If the node is
    /// fixed in 3D space (Can't move at all)". It lowers exactly to RoR's
    /// `fixes` pin (nd_immovable, zero inverse mass throughout the solver):
    /// how a structure such as a wall is anchored to the world.
    /// https://documentation.beamng.com/modding/vehicle/sections/nodes/
    bool fixed;
    JBeamStructuralProvenance provenance;
    /// Authored `nodeMaterial` ("Physics material of the node, rubber, metal,
    /// etc."), empty when unauthored. RETAINED AND DISCLOSED, NOT CONSUMED:
    /// BeamNG uses it for a node's default sound events and emitted particle
    /// type, and RoR has no per-node material at all -- particles and surface
    /// sounds come from the ground model of the surface CONTACTED, not from
    /// the node. Mapping NM_METAL and friends onto RoR's friction/volume/
    /// surface triple would invent a conversion table nothing declares.
    ///
    /// That objection is about CONSUMPTION only. Retaining the authored value
    /// invents nothing, which is why it is kept here and reported, and why it
    /// stays out of the canonical digest: nothing reads it, so it cannot make
    /// two vehicles different simulations.
    std::string node_material;
    /// Authored `isExhaust` on a node: the engine group whose exhaust path
    /// starts here (`"mainEngine"` on the shipped cars; a Boolean `true`
    /// reads as that default group). CONSUMED by the exhaust-outlet lowering
    /// (JBeamExhaustOutlets.h), which is graphics-only, so it stays out of
    /// the canonical digest like `node_material`. Empty when unauthored.
    std::string exhaust_engine;
};

enum class JBeamStructuralBeamStatus
{
    ENABLED,
    PRESERVED_DISABLED_SPECIAL_TYPE,
    PRESERVED_DISABLED_OPTIONAL_REFERENCE
};

struct JBeamStructuralBeam
{
    std::string node_a;
    std::string node_b;
    std::size_t node_a_index;
    std::size_t node_b_index;
    std::string beam_type;
    bool optional;
    JBeamStructuralBeamStatus status;
    /// Explicit project-law role, not a tolerance-based repair. Set only for
    /// distinct original nodes at exactly the same position, with supported
    /// NORMAL/neutral authored parameters and no unsupported effective row
    /// fields. `beamStrength` may be unbounded (FLT_MAX) or a finite positive
    /// force at which the spring breaks; `beamDeform` must not be below it
    /// (a rest length of zero has no plastic reading, so the spring stays
    /// nonplastic in effect); a `breakGroup` detaches its group as on any
    /// beam. The two nodes remain independent.
    bool jbeam_zero_rest_spring;
    bool has_spring;
    double spring;
    bool has_damping;
    double damping;
    bool has_deform;
    bool deform_unbounded;
    double deform;
    bool has_strength;
    bool strength_unbounded;
    double strength;
    /// BeamNG's `deformLimitExpansion`: "Limits by how much of original
    /// length the beam can deform in expansion", "used to avoid excessive
    /// stretching", defaulting to `deformLimit`
    /// (<https://documentation.beamng.com/modding/vehicle/sections/beams/>).
    ///
    /// A MULTIPLE OF THE ORIGINAL REST LENGTH, not a length and not a delta:
    /// authored values sit above 1.0, and 1.1 accounts for 1032 of the
    /// occurrences across the three reference packages. It caps permanent
    /// deformation only and says nothing about the force at which yielding
    /// starts -- that is `beamDeform`, in newtons.
    ///
    /// Retained here so the value survives parsing; the physics ceiling that
    /// consumes it is BeamAxialResponse::RestLengthExpansionCeiling.
    /// BeamNG's `breakGroup`: "A beam gets automatically broken when another
    /// beam from the same breakGroup breaks"
    /// (<https://documentation.beamng.com/modding/vehicle/sections/beams/>).
    ///
    /// EMPTY MEANS NO GROUP, and that is not a formality: 332 of the authored
    /// occurrences across the reference packages are the empty string, which
    /// is a reset rather than a group name. Treating "" as a group would put
    /// a third of a vehicle's beams in one group and break them together on
    /// the first failure anywhere.
    ///
    /// Single-group string form only. The documented table form exists in the
    /// corpus (57 of 1282 occurrences) and is NOT modelled: it is refused by
    /// name rather than silently flattened, because a beam in two groups
    /// propagates between them and flattening would invent a topology.
    std::string break_group;
    bool has_deform_limit_expansion;
    double deform_limit_expansion;
    /// BeamNG's `deformGroup`: the name of the deformation group this beam
    /// belongs to. Paired with `deformation_trigger_ratio` below, it is one
    /// half of the authored breaking-glass/breaking-light mechanism -- the
    /// other half is on the flexbody row, which names the same group plus a
    /// base and a damaged material.
    ///
    /// EMPTY MEANS NO GROUP, exactly as `break_group` above: 527 of the 1506
    /// occurrences measured on the reference GT-R are the empty string, an
    /// authored reset. Interning "" would put a third of the vehicle in one
    /// group.
    ///
    /// Retained, not yet simulated. Every one of the 20 flexbody groups
    /// measured on that vehicle is named by at least one beam, so the loop is
    /// closed on the content side and this string is what closes it in the IR.
    std::string deform_group;
    /// BeamNG's `deformationTriggerRatio`: how far the beam must deform before
    /// its `deform_group` fires, as a fraction of the reference length. The
    /// reference vehicle authors 9 distinct values, 0.001 to 0.1.
    bool has_deformation_trigger_ratio;
    double deformation_trigger_ratio;
    bool has_precompression;
    double precompression;
    /// BeamNG's `precompressionRange`: the spring's free length MINUS its
    /// spawned geometric length, in METRES. Positive means the beam spawns
    /// already compressed and pushes its endpoints apart at rest; that
    /// preload is how a BeamNG coilover carries the body, not static
    /// deflection.
    ///
    /// UNIT, from the content rather than assumed: the parts that compute it
    /// derive it from a `"range"` variable declared with unit `"+m"` (metres)
    /// and add a metres constant to it.
    ///
    /// PRECEDENCE, also stated by the content: where a row carries both,
    /// `precompressionRange` supersedes the dimensionless
    /// `beamPrecompression` multiplier.
    bool has_precompression_range;
    double precompression_range;
    /// BeamNG's `beamPrecompressionTime`: the seconds over which the beam's
    /// rest length moves from its spawned geometric length to the
    /// precompressed length the fields above select. Authored on every
    /// alignment beam of the reference packages (0.5, twice 1). Zero or
    /// absent applies the precompression at spawn exactly as before; see
    /// physics/BeamPrecompressionRamp.h for the law.
    bool has_precompression_time;
    double precompression_time;
    /// SUPPORT-beam extension break ratio. BeamNG's documented default is
    /// 1.0, measured from the geometric spawned length (1.0 means 200%).
    /// On a BOUNDED beam the SAME authored number is the extension threshold
    /// of the dead zone, in the same units.
    bool has_long_bound;
    double long_bound;
    /// BOUNDED compression threshold, the mirror of `long_bound`. Same
    /// documented default of 1.0, same dimensionless multiple of the spawned
    /// length.
    bool has_short_bound;
    double short_bound;
    /// The same two thresholds stated in METRES. BeamNG's documentation is
    /// explicit that using either "will overwrite" its dimensionless
    /// counterpart, so the metres form is not an alternative spelling - it is
    /// the winner where a row carries both.
    bool has_long_bound_range;
    double long_bound_range;
    bool has_short_bound_range;
    double short_bound_range;
    /// Metres of travel past a threshold over which the limit constants are
    /// blended in. BeamNG's documented default is 1 metre, which its own
    /// documentation warns "might be too long for a lot of applications".
    bool has_bound_zone;
    double bound_zone;
    /// The second constant pair, reached at full engagement. BeamNG's
    /// documented defaults are 1.0 N/m and 1.0 N/m/s - numerically inert, and
    /// the reason a row that omits them is a plain spring-damper.
    bool has_limit_spring;
    double limit_spring;
    bool has_limit_damping;
    double limit_damping;
    JBeamStructuralProvenance provenance;
    /// Authored `name` ("Name of the beam. Used by some systems to identify
    /// the specific beam to use."), empty when unauthored. RETAINED AND
    /// DISCLOSED, NOT CONSUMED: in BeamNG it is a handle for vehicle Lua to
    /// query or mutate one beam, and RoR has no such lookup -- there is no
    /// name field on RigDef's beam or on the runtime beam, and nothing
    /// anywhere resolves a beam by name. A consumer would be that lookup.
    std::string name;
    /// Authored `isExhaust` on a beam: the engine group whose exhaust pipe
    /// this beam forms (see JBeamStructuralNode::exhaust_engine). CONSUMED by
    /// the exhaust-outlet lowering; graphics-only, outside the digest.
    std::string exhaust_engine;
    /// OpenBeam `breakFx` (not a BeamNG field): the effect a destructible
    /// structure's renderer emits when this beam's breakGroup fails --
    /// "mortar" (a dust puff) or "concrete" (dust and chips). CONSUMED by the
    /// graphics layer only; outside the digest, and ignored (disclosed in the
    /// import receipt) without a breakGroup or with an unknown kind.
    std::string break_fx;
    /// Authored `disableMeshBreaking` ("Disables mesh breaking. Mesh breaking
    /// allows polygons of a flexbody mesh to be dynamically removed when an
    /// underlying beam breaks."). RETAINED AND DISCLOSED, NOT CONSUMED: RoR
    /// has no mesh breaking to disable. A broken beam has no flexbody effect
    /// at all, so consuming this would mean first building the mechanism it
    /// switches off.
    bool disable_mesh_breaking = false;
    bool disable_mesh_breaking_authored = false;
    /// Authored `disableTriangleBreaking` ("Disables triangle breaking ...
    /// dynamically transitioned to a state without collision and aero when an
    /// underlying beam breaks."). RETAINED AND DISCLOSED, NOT CONSUMED: RoR
    /// already never breaks a triangle, and its cab-node guard -- a beam
    /// refuses to break while either end is a cab node with fewer than three
    /// live beams -- is the always-on inverse. So the field's `false` case is
    /// what would need building, not its `true` case.
    bool disable_triangle_breaking = false;
    bool disable_triangle_breaking_authored = false;
    /// Set when the field WAS authored but its value is not a Boolean. The
    /// F-22A's rudder states `"disableMeshBreaking":"true"` -- a quoted
    /// string. The authored value is deliberately NOT retained and NOT
    /// coerced: a quoted boolean has no single reading, and BeamNG's own Lua
    /// treats the non-empty string "false" as TRUE, so mapping it onto a
    /// truth value would invent a conversion nobody declared. The documented
    /// default applies instead, and the import receipt NAMES the refusal
    /// (`retained-not-simulated:beams.<field>:type-refused:N`), because an
    /// escape that hides what it excused is indistinguishable from silently
    /// accepting bad data.
    ///
    /// This is survivable only because nothing consumes these two fields. A
    /// type violation on a CONSUMED boolean still rejects the vehicle.
    bool disable_mesh_breaking_type_refused = false;
    bool disable_triangle_breaking_type_refused = false;
    /// BeamNG's progressive damping. `beamDampRebound` applies while the beam
    /// EXPANDS; the two `...Fast` rates apply once the rate of length change
    /// exceeds `beamDampVelocitySplit`. RETAINED, NOT CONSUMED: the beam path
    /// still installs one linear rate, so none of these is serialised into the
    /// canonical digest and none changes what a vehicle simulates today.
    bool has_damp_fast = false;
    double damp_fast = 0.0;
    bool has_damp_rebound = false;
    double damp_rebound = 0.0;
    bool has_damp_rebound_fast = false;
    double damp_rebound_fast = 0.0;
    bool has_damp_velocity_split = false;
    double damp_velocity_split = 0.0;
    /// RETAINED, NOT CONSUMED. BeamNG's first-order low-pass on damper force.
    /// `BeamAxialResponse::ComputeDamping` is memoryless, so a filter has
    /// nowhere to keep its previous output without new per-beam state.
    bool has_damp_cutoff_hz = false;
    double damp_cutoff_hz = 0.0;

    JBeamStructuralBeam();
};

/// Shared scalar admission for the zero-rest role. Original coordinate and
/// reference equality, effective-field support and the marker are checked by
/// their respective IR/lowering boundaries, not inferred from these values.
/// NORMAL type, unit precompression and bounds, no bound ranges/zone/limits;
/// an explicit `beamStrength` and an explicit `beamDeform`, each FLT_MAX or
/// finite positive. A deform below the strength is admitted: a zero rest
/// length has no plastic reading, and the lowering keeps the spring elastic
/// to its break (see `JBeamZeroRestSpring::ValidBreakThresholds`).
bool HasSupportedJBeamZeroRestSpringParameters(
    const JBeamStructuralBeam& beam) noexcept;
/// Exact finite original-coordinate equality, including signed-zero equality,
/// without fast-math flushing a nonzero subnormal difference into coincidence.
bool AreJBeamNodePositionsExactlyCoincident(
    const JBeamStructuralNode& first,
    const JBeamStructuralNode& second) noexcept;

enum class JBeamStructuralTriangleOrigin
{
    TRIANGLE,
    QUAD_FIRST,
    QUAD_SECOND
};

enum class JBeamStructuralTriangleStatus
{
    ENABLED,
    PRESERVED_DISABLED_OPTIONAL_REFERENCE
};

enum class JBeamStructuralTriangleType
{
    NORMALTYPE,
    NONCOLLIDABLE
};

/// How a triangle meets water, read from `openbeamBuoyancy`.
///
/// BeamNG's format has no buoyancy at all, so nothing here lowers a field
/// any BeamNG content authors, and an absent key leaves a triangle exactly
/// as it is today. The name carries the `openbeam` prefix precisely so it
/// can never be mistaken for a BeamNG field that we failed to model: it is
/// ours, it is the only way to reach RoR's buoyant-cab machinery from a
/// JBeam document, and a vessel authored for OpenBeam is what uses it.
enum class JBeamStructuralTriangleBuoyancy
{
    /// Absent or `NONE`: the triangle displaces nothing.
    NONE,
    /// `BUOYANT`: displacement and drag (RoR's cab option `b`).
    BUOYANT,
    /// `DRAG_ONLY`: drag without displacement (RoR's `r`) -- a rudder or a
    /// fin, which resists the water it moves through but holds nothing up.
    DRAG_ONLY,
    /// `NO_DRAG`: displacement without drag (RoR's `s`) -- an interior
    /// volume that must float but must not also brake the hull.
    NO_DRAG
};

struct JBeamStructuralTriangle
{
    JBeamStructuralTriangle();

    std::string node_a;
    std::string node_b;
    std::string node_c;
    std::size_t node_a_index;
    std::size_t node_b_index;
    std::size_t node_c_index;
    bool optional;
    JBeamStructuralTriangleStatus status;
    JBeamStructuralTriangleOrigin origin;
    JBeamStructuralTriangleType triangle_type;
    /// Authored `openbeamBuoyancy`; NONE when the key is absent, which is
    /// every BeamNG document ever written.
    JBeamStructuralTriangleBuoyancy buoyancy;
    std::size_t authored_row_index;
    JBeamTriangleAerodynamics aerodynamics;
    JBeamStructuralProvenance provenance;
    /// Authored `breakGroup`, empty when the triangle is in no group.
    /// BeamNG documents it as letting "triangles to break when triggered by
    /// beams in the same break group", so a triangle whose group has broken
    /// produces no aerodynamic force. String form only; the documented table
    /// form is refused by name rather than flattened, exactly as for beams,
    /// and an empty string is an authored RESET rather than a group.
    std::string break_group;
    /// Authored `groundModel` ("Ground model of the triangle. Determines
    /// friction behavior."), empty when unauthored. RETAINED AND DISCLOSED,
    /// NOT CONSUMED: this is a per-triangle property and RoR's only hook is
    /// `submesh_groundmodel`, ONE string per actor. Emitting one would mean
    /// choosing among a vehicle's authored values, which is a guess; the
    /// corpus authors plastic 90, metal 65 and glass 1, so a
    /// refuse-unless-they-all-agree rule would refuse far more often than it
    /// fires. Deliberately absent from the canonical digest, because nothing
    /// reads it and so it cannot make two vehicles different simulations.
    std::string ground_model;
};

struct JBeamStructuralRefFrame
{
    std::string reference;
    std::string back;
    std::string left;
    std::string up;
    std::string left_corner;
    std::string right_corner;
    std::size_t reference_index;
    std::size_t back_index;
    std::size_t left_index;
    std::size_t up_index;
    std::size_t left_corner_index;
    std::size_t right_corner_index;
    JBeamStructuralProvenance provenance;
};

struct JBeamStructuralIR
{
    std::vector<JBeamStructuralPart> parts;
    /// Documented `$components` leaves, flattened. Every part's expressions
    /// read them ahead of the part's own variables.
    std::vector<JBeamExpressionVariable> expression_components;
    std::vector<JBeamStructuralNode> nodes;
    std::vector<JBeamStructuralBeam> beams;
    std::vector<JBeamStructuralTriangle> triangles;
    /// All authored scale assignments in resolved-part preorder/source order.
    /// More than one assignment is deliberately not collapsed: public docs
    /// do not establish cross-part negation/merge pass ordering precisely.
    std::vector<JBeamAeroScaleAssignment> aerodynamic_scales;
    JBeamStructuralRefFrame ref_frame;
    bool has_ref_frame;
    std::size_t authored_row_count;
    /// Dynamic payload bytes charged while constructing this IR.
    std::size_t retained_byte_count;
    std::size_t canonical_output_byte_limit;
    std::size_t canonical_work_unit_limit;
    std::vector<JBeamStructuralDiagnostic> diagnostics;

    JBeamStructuralIR();
    bool IsValid() const;
};

/// Builds the dependency-free J2 structural subset in BeamNG's authored SI
/// coordinate system. Explicitly supported scalar fields resolve standalone
/// variables, namespace strings, and the allowlisted "$=" expression subset
/// against the resolved part's variables and globally merged scalar component
/// leaves. Unsupported fields and table-valued components remain inert.
JBeamStructuralIR BuildJBeamStructuralIR(
    const JBeamResolvedGraph& graph,
    const JBeamStructuralLimits& limits = JBeamStructuralLimits());

/// Reads one scalar field of the part at `part_index` as a number, in the
/// scope this tier gave that part's own fields: a finite number is itself; a
/// string starting with '$' is evaluated as this tier evaluates such a field
/// ("$=expr", or a bare "$variable") against the documented components and
/// the part's resolved variables. False - with the reason in `failure` - for
/// anything else, a non-numeric or non-finite result, an unevaluable
/// expression, or a part this IR kept no scope for. Later tiers (the
/// drivetrain's torque samples) use it so that a field has one reading.
bool EvaluateJBeamStructuralPartNumber(
    const JBeamStructuralIR& ir,
    std::size_t part_index,
    const JBeamValue& value,
    double& output,
    std::string* failure = NULL);

/// Stable identity material for the supported structural semantics, provenance,
/// disabled records, and deterministic diagnostic report.
std::string SerializeCanonicalJBeamStructuralIR(
    const JBeamStructuralIR& ir);

const char* JBeamStructuralDiagnosticCodeToString(
    JBeamStructuralDiagnosticCode code);

} // namespace BeamNG
} // namespace RoR
