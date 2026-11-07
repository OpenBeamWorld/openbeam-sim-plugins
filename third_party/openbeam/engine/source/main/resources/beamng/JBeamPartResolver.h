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

/// @file JBeamPartResolver.h
/// @brief Deterministic BeamNG part index and bounded slot-graph resolver.

#pragma once

#include "JBeamSyntax.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace RoR {
namespace BeamNG {

enum class JBeamResolveSeverity
{
    WARNING,
    ERROR_SEVERITY
};

enum class JBeamResolveDiagnosticCode
{
    PACKAGE_DOCUMENT_NOT_OBJECT,
    PART_NOT_OBJECT,
    EMPTY_PART_NAME,
    MISSING_SLOT_TYPE,
    INVALID_SLOT_TYPE,
    DUPLICATE_SLOT_TYPE,
    DUPLICATE_PART,
    MULTIPLE_MAIN_PARTS,
    DUPLICATE_SLOT_SECTION,
    INVALID_SLOT_TABLE,
    INVALID_SLOT_ROW,
    MISSING_SLOT_FIELD,
    INVALID_SLOT_FIELD,
    DUPLICATE_SLOT,
    INVALID_SLOT_VARIABLE,
    INVALID_SLOT_NODE_OFFSET,
    DUPLICATE_TUNING_VARIABLE_SECTION,
    INVALID_TUNING_VARIABLE_TABLE,
    INVALID_TUNING_VARIABLE_ROW,
    MISSING_TUNING_VARIABLE_FIELD,
    INVALID_TUNING_VARIABLE_FIELD,
    INVALID_TUNING_VARIABLE_OVERRIDE,
    DUPLICATE_TUNING_VARIABLE,
    TUNING_VARIABLE_LIMIT,
    INDEX_PART_LIMIT,
    CONFIGURATION_NOT_OBJECT,
    INVALID_CONFIGURATION_SECTION,
    DUPLICATE_CONFIGURATION_SECTION,
    INVALID_PART_SELECTION,
    DUPLICATE_PART_SELECTION,
    INVALID_CONFIGURATION_VARIABLE,
    DUPLICATE_CONFIGURATION_VARIABLE,
    INDEX_INVALID,
    ROOT_PART_NOT_FOUND,
    ROOT_PART_NOT_MAIN,
    AMBIGUOUS_PART,
    MISSING_REQUIRED_PART,
    MISSING_OPTIONAL_PART,
    /// A coreSlot whose selection is explicitly empty or the `"none"` sentinel. The
    /// author declared the slot required and then explicitly declined it -
    /// worth naming, but it is a DECLINED slot, not a missing part, and its
    /// status is EMPTY accordingly.
    DECLINED_REQUIRED_PART,
    SUBSTITUTED_PACKAGE_LOCAL_PART,
    PART_NOT_ALLOWED_IN_SLOT,
    SLOT_CYCLE,
    RESOLVE_DEPTH_LIMIT,
    RESOLVED_PART_LIMIT,
    RESOLVED_VARIABLE_LIMIT,
    UNUSED_PART_SELECTION,
    INDEX_INPUT_ENTRY_LIMIT,
    PART_SELECTION_LIMIT,
    REQUEST_VARIABLE_LIMIT,
    DIAGNOSTIC_LIMIT,
    /// An unavailable or incompatible explicit selection retried the exact
    /// authored slot default. The retry can still resolve empty or refuse;
    /// this is never evidence of a package-local heuristic substitution.
    RETRIED_AUTHORED_DEFAULT_PART,
    /// A `coreSlot` whose configuration selection was the empty string took
    /// the slot's declared default. BeamNG does not let a core slot be
    /// emptied from the part UI, so a blank entry in a shipped
    /// configuration is "unspecified", not the `"none"` decline: every
    /// configuration of the NLR ZX-6R leaves its `paint_design` core slot
    /// blank, and that slot's default is the part whose skin carries the
    /// rider's only suit, helmet, glove and boot textures.
    BLANK_CORE_SLOT_TOOK_DEFAULT,
    /// A configuration or slot set a numeric tuning variable outside the
    /// min/max its part declares. BeamNG bounds a jbeam variable by that
    /// range (its documentation: the value used inside jbeam is the one
    /// min/max define), so the nearer bound is used, with its exact
    /// spelling. The detail reads `given=<v>:declared=[<min>,<max>]:used=<v>`.
    TUNING_VARIABLE_OVERRIDE_CLAMPED,
    /// A part's own tuning-variable row declares a default outside its
    /// min/max. The same bound applies as for an override: the nearer bound
    /// becomes the default, with its exact spelling, and the detail reads
    /// like TUNING_VARIABLE_OVERRIDE_CLAMPED's.
    TUNING_VARIABLE_DEFAULT_CLAMPED,
    // Appended: prior serialized codes keep their values.
    SELECTED_SOURCE_REFUSED,
    SELECTED_SOURCE_POLICY_REFUSED
};

struct JBeamResolveDiagnostic
{
    JBeamResolveDiagnosticCode code;
    JBeamResolveSeverity severity;
    JBeamSourceSpan span;
    std::string part_name;
    std::string slot_name;
    std::string detail;

    JBeamResolveDiagnostic();
};

struct JBeamResolverLimits
{
    /// Counts every top-level object entry examined, including malformed
    /// entries which cannot become indexed parts.
    std::size_t max_input_entries;
    std::size_t max_indexed_parts;
    std::size_t max_resolved_parts;
    std::size_t max_depth;
    std::size_t max_slots_per_part;
    /// Applies both while parsing a .pc file and while resolving a manually
    /// constructed request.
    std::size_t max_request_selections;
    std::size_t max_request_variables;
    std::size_t max_variables_per_node;
    /// Per-table normalization gates used by slots, slots2, and authored
    /// variables. They prevent temporary normalized-row copies from escaping
    /// the resolver's deterministic resource contract.
    std::size_t max_table_normalize_work_units;
    std::size_t max_table_normalize_retained_bytes;
    /// Maximum retained diagnostics. Zero still retains one terminal
    /// diagnostic when any diagnostic is emitted so failure cannot appear
    /// successful.
    std::size_t max_diagnostics;

    JBeamResolverLimits();
};

struct JBeamPackageSource
{
    std::string package_path;
    JBeamValue document;
};

enum class JBeamSlotKind
{
    SLOTS,
    SLOTS2
};

enum class JBeamVariableOrigin
{
    AUTHORED_DEFAULT,
    CONFIGURATION,
    SLOT
};

/// Values are intentionally retained as parsed scalar JBeam values. In
/// particular, "$=" expressions and "$variable" strings are never evaluated
/// by this structural resolver.
struct JBeamVariableAssignment
{
    std::string name;
    JBeamValue value;
    JBeamSourceSpan span;
    JBeamVariableOrigin origin;
};

/// One selected part's documented tuning-variable row. Required range fields
/// are admitted as literal finite values. The duplicate-preserving raw row
/// remains in the owning part body so UI metadata outside this first semantic
/// subset is never lost or redundantly deep-copied.
struct JBeamTuningVariableDefinition
{
    std::string part_name;
    std::string package_path;
    std::string name;
    std::string type;
    std::string unit;
    std::string category;
    JBeamValue default_value;
    JBeamValue min_value;
    JBeamValue max_value;
    std::string title;
    std::string description;
    std::string sub_category;
    bool has_step_dis;
    JBeamValue step_dis;
    bool has_min_dis;
    JBeamValue min_dis;
    bool has_max_dis;
    JBeamValue max_dis;
    bool hide_in_ui;
    JBeamSourceSpan span;

    JBeamTuningVariableDefinition();
};

/// A slot's documented `nodeOffset` translation. BeamNG mounts one authored
/// part into a slot that states where the part belongs on the vehicle, so the
/// part itself authors its nodes around its own origin: the four wheels of a
/// car are one wheel part, mounted four times, each slot supplying the axle's
/// position. A component whose offset is dropped is not slightly wrong, it is
/// at the vehicle origin.
///
/// Each component is retained as an unevaluated scalar because packages author
/// them as expressions over tuning variables (`$=case($trackwidth_F == nil,
/// $trackoffset_F+0.29, $trackwidth_F)` is the reference package's front
/// track). Evaluation belongs to the structural tier, which owns the variable
/// scope; this tier only proves the shape.
struct JBeamSlotNodeOffset
{
    bool present;
    bool has_x;
    bool has_y;
    bool has_z;
    JBeamValue x;
    JBeamValue y;
    JBeamValue z;
    JBeamSourceSpan span;

    JBeamSlotNodeOffset();
};

struct JBeamSlotDefinition
{
    JBeamSlotKind kind;
    std::string name;
    std::vector<std::string> allow_types;
    std::vector<std::string> deny_types;
    std::string default_part;
    std::string description;
    bool core_slot;
    JBeamSourceSpan span;
    std::vector<JBeamVariableAssignment> variables;
    JBeamSlotNodeOffset node_offset;

    JBeamSlotDefinition();
};

struct JBeamPartDefinition
{
    std::string name;
    std::string package_path;
    JBeamSourceSpan name_span;
    JBeamValue body;
    /// Source order and duplicates are retained. Matching treats this as a set.
    std::vector<std::string> slot_types;
    std::vector<JBeamSlotDefinition> slots;
    /// Ordered definitions from the effective authored `variables` table.
    std::vector<JBeamTuningVariableDefinition> tuning_variables;
};

struct JBeamPackageIndex
{
    std::vector<JBeamPartDefinition> parts;
    std::vector<JBeamResolveDiagnostic> diagnostics;

    bool IsValid() const;
};

enum class JBeamExactPartLoadStatus { FOUND, MISSING, REFUSED };

struct JBeamExactPartLoad
{
    JBeamExactPartLoadStatus status = JBeamExactPartLoadStatus::REFUSED;
    JBeamPartDefinition part;
    std::size_t original_definition_count = 0U;
    std::vector<JBeamResolveDiagnostic> diagnostics;
};

/// Opt-in selected-input boundary. A provider must return only an exact
/// effective authored definition whose metadata has passed strict validation.
/// REFUSED is distinct from MISSING and stops before any default retry or
/// missing-part witness. Loading does not authorize physical import/derivation.
class JBeamExactPartSource
{
public:
    virtual ~JBeamExactPartSource() = default;
    virtual JBeamExactPartLoad LoadExactPart(const std::string& name) = 0;
};

/// Builds a canonical package index independent of input document enumeration.
/// Object-field order inside each document is retained because duplicate-key
/// last-assignment semantics are source-observable.
JBeamPackageIndex BuildJBeamPackageIndex(
    const std::vector<JBeamPackageSource>& sources,
    const JBeamResolverLimits& limits = JBeamResolverLimits());

/// Stable index identity material. Length-prefixed values preserve embedded
/// control bytes and every duplicate object field without executing content.
std::string SerializeCanonicalJBeamPackageIndex(
    const JBeamPackageIndex& index);

struct JBeamPartSelection
{
    std::string slot_name;
    std::string part_name;
    JBeamSourceSpan span;
};

struct JBeamResolveRequest
{
    /// Empty selects the package's unique slotType "main" part.
    std::string root_part_name;
    std::vector<JBeamPartSelection> part_selections;
    std::vector<JBeamVariableAssignment> variables;
    /// Explicit empty/"none" selections stay empty. An explicit named part
    /// that is missing or incompatible retries only the exact authored slot
    /// default before the optional approximation below. The original ordered
    /// request and explicit-selection provenance remain in the graph.
    /// Declared-tier approximation. Real-world packages name base-game parts
    /// as slot defaults; those parts are not redistributed with the mod, so
    /// the slot would otherwise stay empty and every field the missing part
    /// contributes (wheel radius, tyre width, ...) would be absent. When this
    /// is enabled and a slot declares a non-empty default that is absent from
    /// the package, one package-local candidate for the same slot type is
    /// substituted deterministically and reported with
    /// SUBSTITUTED_PACKAGE_LOCAL_PART. A slot with no declared default, or the
    /// documented explicit `"none"` default, is never substituted: those mean
    /// "deliberately empty", not "missing". Nothing is guessed silently and
    /// the flag is part of the canonical resolved-graph identity.
    bool substitute_missing_package_local_defaults = false;
};

struct JBeamConfigurationResult
{
    JBeamResolveRequest request;
    std::vector<JBeamResolveDiagnostic> diagnostics;

    bool IsValid() const;
};

/// Reads the documented .pc {"parts": {...}, "vars": {...}} subset.
/// Duplicate keys remain ordered and use last-assignment semantics.
JBeamConfigurationResult ParseJBeamConfiguration(
    const JBeamValue& configuration);

JBeamConfigurationResult ParseJBeamConfiguration(
    const JBeamValue& configuration,
    const JBeamResolverLimits& limits);

/// EMPTY and MISSING describe the slot outcome and must not be conflated.
///
///  * EMPTY - resolution selected an empty/"none" value, either explicitly
///    or through the exact authored default. No part is attached. An absent
///    explicit request can remain separately disclosed by
///    unresolved_requested_part; this is not the same as a deliberate decline.
///  * MISSING - the package asks for a part by name and does not ship it.
///    An undeclared core-slot default is also reported as MISSING.
///
/// Consumers needing an unavailable authored-name witness must use
/// UnresolvedJBeamSlotPart(), not infer that every EMPTY edge was explicitly
/// declined or that every unfilled/error edge authorizes derivation.
enum class JBeamResolvedSlotStatus
{
    RESOLVED,
    EMPTY,
    MISSING,
    NOT_ALLOWED,
    CYCLE,
    LIMIT_REJECTED,
    SOURCE_REFUSED
};

/// True for a slot selection that means "deliberately nothing": an absent
/// default or the documented `"none"` sentinel. Exposed because the
/// resolver, its substitution policy and every consumer must agree on it,
/// and the only way to guarantee that is for them to call one function.
bool IsDeclinedJBeamSlotSelection(const std::string& selection);

struct JBeamResolvedPartNode;

struct JBeamResolvedSlot
{
    JBeamSlotDefinition definition;
    bool explicitly_selected;
    std::string selected_part;
    /// Nonempty only for a genuinely absent, nondeclined explicit request
    /// that still has no attached child. Exact fallback-to-empty preserves
    /// this source witness; successful fallback/substitution clears it.
    /// Present but incompatible requests are never missing-part witnesses.
    /// The original ordered request is also retained on JBeamResolvedGraph.
    std::string unresolved_requested_part;
    JBeamResolvedSlotStatus status;
    std::shared_ptr<JBeamResolvedPartNode> child;

    JBeamResolvedSlot();
};

/// An unavailable authored name eligible for disclosed downstream analysis,
/// not authorization to attach a part. Only no-child MISSING/EMPTY edges can
/// provide it: absent explicit-request metadata first, otherwise a nondeclined
/// MISSING selected_part. Deliberate empty selections and all rejected/error
/// statuses return a stable empty string.
const std::string& UnresolvedJBeamSlotPart(const JBeamResolvedSlot& slot) noexcept;

struct JBeamResolvedPartNode
{
    JBeamPartDefinition definition;
    std::vector<JBeamVariableAssignment> inherited_variables;
    std::vector<JBeamResolvedSlot> slots;
};

struct JBeamResolvedGraph
{
    /// Full ordered request history is retained so duplicate .pc assignments
    /// cannot collapse to the same canonical graph identity.
    JBeamResolveRequest request;
    std::shared_ptr<JBeamResolvedPartNode> root;
    /// Active authored definitions in deterministic resolved-part preorder.
    /// Duplicate names are retained; later rows are effective unless a .pc or
    /// descendant slot assignment overrides them.
    std::vector<JBeamTuningVariableDefinition> tuning_variables;
    std::vector<JBeamResolveDiagnostic> diagnostics;
    std::size_t resolved_part_count;

    JBeamResolvedGraph();
    bool IsValid() const;
};

JBeamResolvedGraph ResolveJBeamPartGraph(
    const JBeamPackageIndex& index,
    const JBeamResolveRequest& request = JBeamResolveRequest(),
    const JBeamResolverLimits& limits = JBeamResolverLimits());

/// Lazy exact-winner resolution with the same slot/variable policy as the
/// legacy index API. An explicit root and disabled sibling substitution are
/// required because a selected namespace is not a heuristic candidate index.
JBeamResolvedGraph ResolveJBeamPartGraphFromExactSource(
    JBeamExactPartSource& source,
    const JBeamResolveRequest& request,
    const JBeamResolverLimits& limits = JBeamResolverLimits());

/// Stable semantic policy identity, independent of the observed BeamNG
/// documentation profile. Importers must bind it into derived identities.
const char* JBeamPartResolverPolicyIdentifier() noexcept;

/// Canonical pre-order graph serialization includes the selected definition's
/// full duplicate-preserving AST, all inherited variable assignments, empty or
/// rejected slot edges, deterministic diagnostics, and resolver policy identity.
std::string SerializeCanonicalJBeamResolvedGraph(
    const JBeamResolvedGraph& graph);

const JBeamVariableAssignment* FindEffectiveJBeamVariable(
    const JBeamResolvedPartNode& node,
    const std::string& name);

const JBeamTuningVariableDefinition* FindEffectiveJBeamTuningVariable(
    const JBeamResolvedGraph& graph,
    const std::string& name);

const char* JBeamResolveDiagnosticCodeToString(
    JBeamResolveDiagnosticCode code);

} // namespace BeamNG
} // namespace RoR
