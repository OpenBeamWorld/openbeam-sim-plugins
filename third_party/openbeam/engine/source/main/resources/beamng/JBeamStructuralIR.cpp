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

#include "JBeamStructuralIR.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <streambuf>
#include <tuple>
#include <utility>

namespace RoR {
namespace BeamNG {
namespace {

const std::size_t INVALID_INDEX = static_cast<std::size_t>(-1);

bool IsBinary64SubnormalMagnitude(const double& value)
{
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    const std::uint64_t magnitude =
        bits & UINT64_C(0x7fffffffffffffff);
    return magnitude != 0U &&
        (magnitude & UINT64_C(0x7ff0000000000000)) == 0U;
}

bool AddSize(std::size_t value, std::size_t& total)
{
    if (value > static_cast<std::size_t>(-1) - total)
    {
        return false;
    }
    total += value;
    return true;
}

bool AddProduct(
    std::size_t count,
    std::size_t element_size,
    std::size_t& total)
{
    if (count != 0U &&
        element_size >
            std::numeric_limits<std::size_t>::max() / count)
    {
        return false;
    }
    return AddSize(count * element_size, total);
}

struct PartContext
{
    const JBeamResolvedPartNode* node;
    JBeamStructuralProvenance provenance;
    /// Resolved configuration and slot-variable assignments only. Component
    /// leaves are owned once by StructuralBuilder and prepended on demand.
    std::shared_ptr<std::vector<JBeamExpressionVariable> > variables;
    /// Preorder index of the part whose slot mounted this one, and that slot's
    /// index inside it. The root part has no mounting slot.
    bool has_parent;
    std::size_t parent_part_index;
    std::size_t parent_slot_index;
    /// Accumulated slot `nodeOffset` translation from the root down to this
    /// part, evaluated in each mounting part's own variable scope. `offset_x`
    /// is a magnitude: BeamNG mirrors the lateral component by the sign of the
    /// node's own authored X so that one authored part serves both sides of
    /// the vehicle.
    bool has_node_offset;
    double offset_x;
    double offset_y;
    double offset_z;

    PartContext()
        : node(NULL)
        , has_parent(false)
        , parent_part_index(0U)
        , parent_slot_index(0U)
        , has_node_offset(false)
        , offset_x(0.0)
        , offset_y(0.0)
        , offset_z(0.0)
    {
    }
};

struct SectionOccurrence
{
    const JBeamValue* value;
    JBeamSourceSpan span;
};

struct PartSections
{
    PartContext part;
    std::map<std::string, std::vector<SectionOccurrence> > sections;
};

bool HasRejectedSeverity(const JBeamStructuralDiagnostic& diagnostic)
{
    switch (diagnostic.severity)
    {
    case JBeamStructuralSeverity::WARNING:
        return false;
    case JBeamStructuralSeverity::ERROR_SEVERITY:
        return true;
    default:
        // Hand-built or corrupted IR must not turn an unknown severity into
        // an accepted warning by falling through a negative comparison.
        return true;
    }
}

bool HasErrorsOrUnknownSeverity(
    const std::vector<JBeamStructuralDiagnostic>& diagnostics)
{
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        if (HasRejectedSeverity(diagnostics[i]))
        {
            return true;
        }
    }
    return false;
}

bool IsDiagnosticLimit(const JBeamStructuralDiagnostic& diagnostic)
{
    return diagnostic.code ==
        JBeamStructuralDiagnosticCode::DIAGNOSTIC_LIMIT;
}

JBeamStructuralDiagnostic MakeDiagnostic(
    JBeamStructuralDiagnosticCode code,
    JBeamStructuralSeverity severity,
    const JBeamStructuralProvenance& provenance,
    const std::string& section,
    std::size_t row_index,
    const std::string& field_name,
    const std::string& detail)
{
    JBeamStructuralDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.provenance = provenance;
    diagnostic.section = section;
    diagnostic.row_index = row_index;
    diagnostic.field_name = field_name;
    diagnostic.detail = detail;
    return diagnostic;
}

JBeamStructuralDiagnostic MakeDiagnosticLimit()
{
    return MakeDiagnostic(
        JBeamStructuralDiagnosticCode::DIAGNOSTIC_LIMIT,
        JBeamStructuralSeverity::ERROR_SEVERITY,
        JBeamStructuralProvenance(),
        std::string(),
        0U,
        std::string(),
        "Additional diagnostics were deterministically suppressed");
}

void PushDiagnostic(
    std::vector<JBeamStructuralDiagnostic>& diagnostics,
    std::size_t configured_limit,
    const JBeamStructuralDiagnostic& diagnostic)
{
    if (!diagnostics.empty() &&
        IsDiagnosticLimit(diagnostics.back()))
    {
        return;
    }
    if (configured_limit == 0U)
    {
        diagnostics.push_back(MakeDiagnosticLimit());
        return;
    }
    if (diagnostics.size() < configured_limit)
    {
        diagnostics.push_back(diagnostic);
        return;
    }
    diagnostics[configured_limit - 1U] = MakeDiagnosticLimit();
}

bool DiagnosticLess(
    const JBeamStructuralDiagnostic& left,
    const JBeamStructuralDiagnostic& right)
{
    if (IsDiagnosticLimit(left) != IsDiagnosticLimit(right))
    {
        return !IsDiagnosticLimit(left);
    }
    if (left.provenance.PartPreorderIndex() !=
        right.provenance.PartPreorderIndex())
    {
        return left.provenance.PartPreorderIndex() <
            right.provenance.PartPreorderIndex();
    }
    if (left.provenance.SourceName() !=
        right.provenance.SourceName())
    {
        return left.provenance.SourceName() <
            right.provenance.SourceName();
    }
    if (left.provenance.begin.byte_offset !=
        right.provenance.begin.byte_offset)
    {
        return left.provenance.begin.byte_offset <
            right.provenance.begin.byte_offset;
    }
    if (left.section != right.section)
    {
        return left.section < right.section;
    }
    if (left.row_index != right.row_index)
    {
        return left.row_index < right.row_index;
    }
    if (left.field_name != right.field_name)
    {
        return left.field_name < right.field_name;
    }
    if (left.code != right.code)
    {
        return static_cast<int>(left.code) <
            static_cast<int>(right.code);
    }
    return left.detail < right.detail;
}

void SortDiagnostics(std::vector<JBeamStructuralDiagnostic>& diagnostics)
{
    std::stable_sort(
        diagnostics.begin(), diagnostics.end(), DiagnosticLess);
}

bool IsFiniteDouble(const double& value)
{
    // std::isfinite may be folded to true under -ffast-math. Inspect the
    // IEEE-754 exponent instead so hostile hand-built ASTs remain rejected.
    static_assert(
        sizeof(double) == sizeof(std::uint64_t),
        "JBeam structural conversion requires binary64 doubles");
    volatile unsigned char stored[sizeof(double)];
    const unsigned char* const source =
        reinterpret_cast<const unsigned char*>(&value);
    for (std::size_t i = 0U; i < sizeof(double); ++i)
    {
        stored[i] = source[i];
    }
    std::uint64_t bits = 0U;
    unsigned char* const destination =
        reinterpret_cast<unsigned char*>(&bits);
    for (std::size_t i = 0U; i < sizeof(double); ++i)
    {
        destination[i] = stored[i];
    }
    return (bits & UINT64_C(0x7ff0000000000000)) !=
        UINT64_C(0x7ff0000000000000);
}

bool IsFiniteNonNegativeDouble(const double& value)
{
    if (!IsFiniteDouble(value)) return false;
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    // Integer classification also rejects negative subnormals under host
    // fast-math/flush-to-zero flags; either spelling of exact zero is valid.
    return (bits & UINT64_C(0x8000000000000000)) == 0U ||
        (bits & UINT64_C(0x7fffffffffffffff)) == 0U;
}

std::uint64_t PositiveOrderBits(const double& value)
{
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    const volatile std::uint64_t observed = bits;
    return observed;
}

bool IsFinitePositiveDouble(const double& value)
{
    const std::uint64_t bits = PositiveOrderBits(value);
    return bits > 0U && bits < UINT64_C(0x7ff0000000000000);
}

double Abs(double value)
{
    return value < 0.0 ? -value : value;
}

bool IsExpressionString(const JBeamValue& value)
{
    return value.type == JBeamValueType::STRING &&
        !value.scalar_text.empty() &&
        value.scalar_text[0] == '$';
}

bool IsAsciiAlpha(unsigned char value)
{
    return (value >= static_cast<unsigned char>('a') &&
            value <= static_cast<unsigned char>('z')) ||
        (value >= static_cast<unsigned char>('A') &&
         value <= static_cast<unsigned char>('Z'));
}

bool IsAsciiDigit(unsigned char value)
{
    return value >= static_cast<unsigned char>('0') &&
        value <= static_cast<unsigned char>('9');
}

/// The name of a `$variable`, which is a dictionary key in the authored
/// document rather than a C identifier: shipped content names its tuning
/// variables `$1steer` and `$3fdamp`, so the first byte takes the same run
/// as the rest. A COMPONENT PATH segment keeps the stricter rule below --
/// that is a path into the component tree, not a name the author chose.
bool IsExpressionVariableNameSegment(const std::string& value)
{
    if (value.empty())
    {
        return false;
    }
    for (std::size_t i = 0U; i < value.size(); ++i)
    {
        const unsigned char current =
            static_cast<unsigned char>(value[i]);
        if (!IsAsciiAlpha(current) &&
            !IsAsciiDigit(current) &&
            current != static_cast<unsigned char>('_'))
        {
            return false;
        }
    }
    return true;
}

bool IsComponentPathSegment(const std::string& value)
{
    if (value.empty())
    {
        return false;
    }
    const unsigned char first =
        static_cast<unsigned char>(value[0]);
    if (!IsAsciiAlpha(first) &&
        first != static_cast<unsigned char>('_'))
    {
        return false;
    }
    for (std::size_t i = 1U; i < value.size(); ++i)
    {
        const unsigned char current =
            static_cast<unsigned char>(value[i]);
        if (!IsAsciiAlpha(current) &&
            !IsAsciiDigit(current) &&
            current != static_cast<unsigned char>('_'))
        {
            return false;
        }
    }
    return true;
}

bool IsAsciiSpace(unsigned char value)
{
    return value == static_cast<unsigned char>(' ') ||
        value == static_cast<unsigned char>('\t') ||
        value == static_cast<unsigned char>('\n') ||
        value == static_cast<unsigned char>('\r');
}

bool ParseExactComponentReference(
    const JBeamValue& value,
    std::string& path)
{
    path.clear();
    if (value.type != JBeamValueType::STRING ||
        value.scalar_text.compare(0U, 2U, "$=") != 0)
    {
        return false;
    }

    std::size_t begin = 2U;
    while (begin < value.scalar_text.size() &&
           IsAsciiSpace(static_cast<unsigned char>(
               value.scalar_text[begin])))
    {
        ++begin;
    }
    std::size_t end = value.scalar_text.size();
    while (end > begin &&
           IsAsciiSpace(static_cast<unsigned char>(
               value.scalar_text[end - 1U])))
    {
        --end;
    }
    static const char COMPONENT_PREFIX[] = "$components.";
    static const std::size_t COMPONENT_PREFIX_SIZE =
        sizeof(COMPONENT_PREFIX) - 1U;
    if (end - begin <= COMPONENT_PREFIX_SIZE ||
        value.scalar_text.compare(
            begin, COMPONENT_PREFIX_SIZE, COMPONENT_PREFIX) != 0)
    {
        return false;
    }

    path.assign(
        value.scalar_text,
        begin + COMPONENT_PREFIX_SIZE,
        end - begin - COMPONENT_PREFIX_SIZE);
    std::size_t segment_begin = 0U;
    while (segment_begin < path.size())
    {
        const std::size_t separator = path.find('.', segment_begin);
        const std::size_t segment_end = separator == std::string::npos
            ? path.size()
            : separator;
        if (!IsComponentPathSegment(path.substr(
                segment_begin, segment_end - segment_begin)))
        {
            path.clear();
            return false;
        }
        if (separator == std::string::npos)
        {
            return true;
        }
        segment_begin = separator + 1U;
    }
    path.clear();
    return false;
}

bool IsRecognizedSection(const std::string& name)
{
    return name == "nodes" ||
        name == "beams" ||
        name == "triangles" ||
        name == "quads" ||
        name == "refNodes";
}

bool IsResolverMetadataSection(const std::string& name)
{
    return name == "slotType" ||
        name == "slots" ||
        name == "slots2" ||
        name == "components" ||
        name == "variables";
}

bool IsAerodynamicScaleName(const std::string& name)
{
    return name == "scaledragCoef" || name == "scaleskinDragCoef" ||
        name == "scaleliftCoef" || name == "scalestallAngle";
}

bool IsKnownUnsupportedField(
    const std::string& section,
    const std::string& field)
{
    if (section == "nodes")
    {
        return field == "group" ||
            field == "nodeMaterial" ||
            field == "volumeCoef" ||
            field == "surfaceCoef" ||
            field == "flashPoint" ||
            field == "smokePoint" ||
            field == "tag";
    }
    if (section == "beams")
    {
        return                         field == "dampCutoffHz" ||
            field == "soundFile" ||
            field == "colorFactor" ||
            field == "disableMeshBreaking" ||
            field == "disableTriangleBreaking" ||
            field == "name";
    }
    if (section == "triangles" || section == "quads")
    {
        return field == "dragCoef" ||
            field == "skinDragCoef" ||
            field == "liftCoef" ||
            field == "stallAngle" ||
            field == "groundModel" ||
            field == "pressureGroup" ||
            field == "collision" ||
            field == "optional" ||
            field == "name";
    }
    if (section == "refNodes")
    {
        return false;
    }
    return false;
}

bool IsImplementedField(
    const std::string& section,
    const std::string& field)
{
    if (section == "nodes")
    {
        return field == "id" ||
            field == "posX" ||
            field == "posY" ||
            field == "posZ" ||
            field == "nodeWeight" ||
            field == "frictionCoef" ||
            field == "slidingFrictionCoef" ||
            field == "collision" ||
            field == "selfCollision" ||
            field == "staticCollision" ||
            field == "fixed" ||
            field == "isExhaust";
    }
    if (section == "beams")
    {
        return field == "id1:" ||
            field == "isExhaust" ||
            field == "id1" ||
            field == "id2:" ||
            field == "id2" ||
            field == "beamType" ||
            field == "optional" ||
            field == "beamSpring" ||
            field == "beamDamp" ||
            field == "beamDeform" ||
            field == "deformLimitExpansion" ||
            field == "beamStrength" ||
            field == "beamPrecompression" ||
            field == "precompressionRange" ||
            field == "beamPrecompressionTime" ||
            field == "beamLongBound" ||
            field == "beamShortBound" ||
            field == "longBoundRange" ||
            field == "shortBoundRange" ||
            field == "boundZone" ||
            field == "beamLimitSpring" ||
            field == "beamLimitDamp" ||
            field == "beamDampFast" ||
            field == "beamDampRebound" ||
            field == "beamDampReboundFast" ||
            field == "beamDampVelocitySplit" ||
            // OpenBeam's own: consumed by the graphics layer, as isExhaust.
            field == "breakFx";
    }
    if (section == "triangles")
    {
        return field == "id1:" ||
            field == "id1" ||
            field == "id2:" ||
            field == "id2" ||
            field == "id3:" ||
            field == "id3" ||
            field == "breakGroup" ||
            field == "optional" ||
            field == "triangleType" ||
            // OpenBeam's own: how the surface meets water.
            field == "openbeamBuoyancy";
    }
    if (section == "quads")
    {
        return field == "id1:" ||
            field == "id1" ||
            field == "id2:" ||
            field == "id2" ||
            field == "id3:" ||
            field == "id3" ||
            field == "id4:" ||
            field == "id4" ||
            field == "breakGroup" ||
            field == "optional" ||
            field == "triangleType" ||
            // OpenBeam's own: how the surface meets water.
            field == "openbeamBuoyancy";
    }
    if (section == "refNodes")
    {
        return field == "ref:" ||
            field == "ref" ||
            field == "back:" ||
            field == "back" ||
            field == "left:" ||
            field == "left" ||
            field == "up:" ||
            field == "up" ||
            field == "leftCorner:" ||
            field == "leftCorner" ||
            field == "rightCorner:" ||
            field == "rightCorner";
    }
    return false;
}

class StructuralBuilder
{
public:
    StructuralBuilder(
        const JBeamResolvedGraph& graph,
        const JBeamStructuralLimits& limits)
        : m_graph(graph)
        , m_limits(limits)
        , m_resource_limit(false)
        , m_ref_row_count(0U)
        , m_authored_beam_rows(0U)
        , m_retained_bytes(0U)
        , m_semantic_bytes(0U)
        , m_expression_evaluations(0U)
        , m_expression_work_units(0U)
        , m_component_nodes(0U)
        , m_component_string_bytes(0U)
    {
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            m_aero_scale_count[i] = 0U;
            m_aero_scale_index[i] = 0U;
        }
        m_result.canonical_output_byte_limit =
            limits.max_canonical_output_bytes;
        m_result.canonical_work_unit_limit =
            limits.max_canonical_work_units;
    }

    JBeamStructuralIR Run()
    {
        if (!m_graph.IsValid())
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_RESOLVED_GRAPH,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                JBeamStructuralProvenance(),
                std::string(),
                0U,
                std::string(),
                "Structural conversion requires a valid resolved graph");
            Finish();
            return m_result;
        }
        CollectParts();
        if (!m_resource_limit)
        {
            BuildComponentEnvironment();
        }
        if (!m_resource_limit)
        {
            BuildPartVariableEnvironments();
        }
        if (!m_resource_limit)
        {
            ResolveSlotNodeOffsets();
            for (std::size_t i = 0U; i < m_parts.size(); ++i)
            {
                if (!ReserveSemantic(3U * sizeof(double))) break;
                m_result.parts[i].slot_node_offset[0] = m_parts[i].offset_x;
                m_result.parts[i].slot_node_offset[1] = m_parts[i].offset_y;
                m_result.parts[i].slot_node_offset[2] = m_parts[i].offset_z;
            }
            // Keep the scope every part's fields were evaluated in, so a
            // later tier evaluates a `$=` field of the same part (the
            // drivetrain's torque samples) in exactly that scope.
            if (!m_resource_limit &&
                ReserveSemantic(
                    m_component_string_bytes +
                    m_components.size() * sizeof(JBeamExpressionVariable)))
            {
                m_result.expression_components = m_components;
                for (std::size_t i = 0U; i < m_parts.size(); ++i)
                {
                    m_result.parts[i].expression_variables =
                        m_parts[i].variables;
                }
            }
        }
        if (!m_resource_limit)
        {
            DiscoverSections();
        }
        if (!m_resource_limit)
        {
            PreserveAerodynamicScales();
        }
        if (!m_resource_limit)
        {
            PreflightRows();
        }
        if (!m_resource_limit)
        {
            ProcessNodes();
        }
        if (!m_resource_limit)
        {
            ProcessRefNodes();
        }
        if (!m_resource_limit)
        {
            ProcessBeams();
        }
        if (!m_resource_limit)
        {
            ProcessTriangles();
        }
        if (!m_resource_limit)
        {
            ProcessQuads();
        }
        if (m_ref_row_count == 0U && !m_resource_limit)
        {
            Push(
                JBeamStructuralDiagnosticCode::MISSING_REF_NODES,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                RootProvenance(),
                "refNodes",
                0U,
                std::string(),
                "Exactly one refNodes frame is required");
        }
        else if (m_ref_row_count > 1U && !m_resource_limit)
        {
            Push(
                JBeamStructuralDiagnosticCode::DUPLICATE_REF_NODES,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                RootProvenance(),
                "refNodes",
                0U,
                std::string(),
                "More than one refNodes data row is ambiguous");
            m_result.has_ref_frame = false;
        }
        Finish();
        return m_result;
    }

private:
    const JBeamResolvedGraph& m_graph;
    const JBeamStructuralLimits& m_limits;
    JBeamStructuralIR m_result;
    std::vector<PartContext> m_parts;
    std::vector<PartSections> m_sections;
    std::map<std::string, std::size_t> m_node_indices;
    bool m_resource_limit;
    std::size_t m_ref_row_count;
    std::size_t m_authored_beam_rows;
    std::size_t m_retained_bytes;
    std::size_t m_semantic_bytes;
    std::size_t m_expression_evaluations;
    std::size_t m_expression_work_units;
    std::size_t m_component_nodes;
    std::size_t m_component_string_bytes;
    std::size_t m_aero_scale_count[4];
    std::size_t m_aero_scale_index[4];
    std::set<std::size_t> m_aero_unresolved_parts;
    std::vector<JBeamExpressionVariable> m_components;
    /// Exact flattened paths that still denote non-scalar values. false
    /// allows scalar descendants of an object; true also blocks descendants
    /// because an array or unsupported expression replaced the subtree.
    std::map<std::string, bool> m_non_scalar_components;
    /// Exact effective array-valued component leaves. These remain source
    /// owned. A whole recognized section can reference one directly; a row
    /// reference copies it only after the complete copy has passed the
    /// retained-memory preflight.
    std::map<std::string, std::shared_ptr<const JBeamValue> >
        m_component_arrays;
    std::map<
        std::string,
        std::shared_ptr<const std::string> > m_source_names;
    std::set<const JBeamValue*> m_measured_values;
    /// Authored blank resets already reported, by part, source span and
    /// field: see the blank branch of ReadRequiredNumber.
    std::set<std::tuple<
        std::size_t,
        std::string,
        std::size_t,
        std::size_t,
        std::string> > m_reported_blank_resets;

    enum class ValueMeasureStatus
    {
        OK,
        BYTE_LIMIT,
        COMPLEXITY_LIMIT,
        INVALID_GRAPH
    };

    struct PendingValue
    {
        const JBeamValue* value;
        std::size_t depth;
        std::size_t header_bytes;
    };

    struct ValueMeasurement
    {
        std::size_t bytes;
        std::set<const JBeamValue*> addresses;

        ValueMeasurement()
            : bytes(0U)
        {
        }
    };

    JBeamStructuralProvenance RootProvenance() const
    {
        return m_parts.empty()
            ? JBeamStructuralProvenance()
            : m_parts[0].provenance;
    }

    void RejectRetained()
    {
        JBeamStructuralDiagnostic diagnostic = MakeDiagnostic(
            JBeamStructuralDiagnosticCode::RETAINED_BYTE_LIMIT,
            JBeamStructuralSeverity::ERROR_SEVERITY,
            JBeamStructuralProvenance(),
            std::string(),
            0U,
            std::string(),
            "Structural IR exceeds the retained-byte budget");
        PushDiagnostic(
            m_result.diagnostics,
            m_limits.max_diagnostics,
            diagnostic);
        m_resource_limit = true;
    }

    bool ReserveRetained(std::size_t bytes)
    {
        std::size_t admitted = m_retained_bytes;
        if (!AddSize(m_semantic_bytes, admitted))
        {
            RejectRetained();
            return false;
        }
        if (bytes >
            m_limits.max_retained_bytes -
                std::min(
                    admitted,
                    m_limits.max_retained_bytes))
        {
            RejectRetained();
            return false;
        }
        m_retained_bytes += bytes;
        m_result.retained_byte_count = m_retained_bytes;
        return true;
    }

    bool ReserveSemantic(std::size_t bytes)
    {
        std::size_t admitted = m_retained_bytes;
        if (!AddSize(m_semantic_bytes, admitted) ||
            bytes >
                m_limits.max_retained_bytes -
                    std::min(
                        admitted,
                        m_limits.max_retained_bytes))
        {
            RejectRetained();
            return false;
        }
        m_semantic_bytes += bytes;
        return true;
    }

    ValueMeasureStatus MeasureValue(
        const JBeamValue& value,
        std::size_t remaining,
        bool honor_already_retained,
        ValueMeasurement& measurement) const
    {
        static const std::size_t SHARED_ALLOCATION_OVERHEAD = 64U;
        measurement = ValueMeasurement();
        std::size_t work_units = 0U;
        std::vector<PendingValue> pending;
        PendingValue root;
        root.value = &value;
        root.depth = 0U;
        root.header_bytes =
            sizeof(JBeamValue) + SHARED_ALLOCATION_OVERHEAD;
        pending.push_back(root);

        while (!pending.empty())
        {
            const PendingValue current = pending.back();
            pending.pop_back();
            if (honor_already_retained &&
                m_measured_values.find(current.value) !=
                    m_measured_values.end())
            {
                continue;
            }
            if (!measurement.addresses.insert(
                    current.value).second)
            {
                // Parsed JBeam is a tree. Repeated addresses indicate a
                // hand-built alias or cycle, neither of which has a canonical
                // retained-size ownership model.
                return ValueMeasureStatus::INVALID_GRAPH;
            }
            if (current.depth >
                    m_limits.max_preserved_value_depth ||
                work_units >=
                    m_limits.max_preserved_value_work_units)
            {
                return ValueMeasureStatus::COMPLEXITY_LIMIT;
            }
            ++work_units;

            std::size_t bytes = current.header_bytes;
            if (!AddSize(current.value->span.source_name.size(), bytes) ||
                !AddSize(current.value->scalar_text.size(), bytes))
            {
                return ValueMeasureStatus::BYTE_LIMIT;
            }
            if (current.value->type == JBeamValueType::ARRAY)
            {
                if (!AddProduct(
                        current.value->array_values.capacity(),
                        sizeof(JBeamValue),
                        bytes))
                {
                    return ValueMeasureStatus::BYTE_LIMIT;
                }
                const std::size_t child_count =
                    current.value->array_values.size();
                if (child_count >
                    m_limits.max_preserved_value_work_units -
                        std::min(
                            work_units,
                            m_limits.max_preserved_value_work_units))
                {
                    return ValueMeasureStatus::COMPLEXITY_LIMIT;
                }
                if (bytes > remaining - std::min(
                        measurement.bytes, remaining))
                {
                    return ValueMeasureStatus::BYTE_LIMIT;
                }
                measurement.bytes += bytes;
                for (std::size_t i = child_count; i > 0U; --i)
                {
                    PendingValue child;
                    child.value =
                        &current.value->array_values[i - 1U];
                    child.depth = current.depth + 1U;
                    // The array allocation already owns each child's fixed
                    // JBeamValue storage.
                    child.header_bytes = 0U;
                    pending.push_back(child);
                }
                continue;
            }
            if (current.value->type == JBeamValueType::OBJECT)
            {
                if (!AddProduct(
                        current.value->object_fields.capacity(),
                        sizeof(JBeamObjectField),
                        bytes))
                {
                    return ValueMeasureStatus::BYTE_LIMIT;
                }
                const std::size_t field_count =
                    current.value->object_fields.size();
                if (field_count >
                    m_limits.max_preserved_value_work_units -
                        std::min(
                            work_units,
                            m_limits.max_preserved_value_work_units))
                {
                    return ValueMeasureStatus::COMPLEXITY_LIMIT;
                }
                work_units += field_count;
                for (std::size_t i = 0U; i < field_count; ++i)
                {
                    const JBeamObjectField& field =
                        current.value->object_fields[i];
                    if (!AddSize(field.key.size(), bytes) ||
                        !AddSize(
                            field.key_span.source_name.size(), bytes))
                    {
                        return ValueMeasureStatus::BYTE_LIMIT;
                    }
                }
                if (bytes > remaining - std::min(
                        measurement.bytes, remaining))
                {
                    return ValueMeasureStatus::BYTE_LIMIT;
                }
                measurement.bytes += bytes;
                for (std::size_t i = field_count; i > 0U; --i)
                {
                    const JBeamObjectField& field =
                        current.value->object_fields[i - 1U];
                    if (field.value)
                    {
                        PendingValue child;
                        child.value = field.value.get();
                        child.depth = current.depth + 1U;
                        child.header_bytes =
                            sizeof(JBeamValue) +
                            SHARED_ALLOCATION_OVERHEAD;
                        pending.push_back(child);
                    }
                }
                continue;
            }
            if (bytes > remaining - std::min(
                    measurement.bytes, remaining))
            {
                return ValueMeasureStatus::BYTE_LIMIT;
            }
            measurement.bytes += bytes;
        }
        return ValueMeasureStatus::OK;
    }

    std::shared_ptr<const std::string> InternSourceName(
        const std::string& source_name)
    {
        const std::map<
            std::string,
            std::shared_ptr<const std::string> >::const_iterator found =
                m_source_names.find(source_name);
        if (found != m_source_names.end())
        {
            return found->second;
        }
        if (!ReserveRetained(source_name.size()))
        {
            return std::shared_ptr<const std::string>();
        }
        const std::shared_ptr<const std::string> shared(
            new std::string(source_name));
        m_source_names.insert(std::make_pair(source_name, shared));
        return shared;
    }

    JBeamStructuralProvenance ProvenanceWithSpan(
        const JBeamStructuralProvenance& provenance,
        const JBeamSourceSpan& span)
    {
        JBeamStructuralProvenance result = provenance;
        result.source_name = InternSourceName(span.source_name);
        result.begin = span.begin;
        result.end = span.end;
        return result;
    }

    void Push(
        JBeamStructuralDiagnosticCode code,
        JBeamStructuralSeverity severity,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        const std::string& detail)
    {
        std::size_t payload = 0U;
        if (!AddSize(section.size(), payload) ||
            !AddSize(field_name.size(), payload) ||
            !AddSize(detail.size(), payload))
        {
            RejectRetained();
            return;
        }
        if (code !=
                JBeamStructuralDiagnosticCode::RETAINED_BYTE_LIMIT &&
            !ReserveRetained(payload))
        {
            return;
        }
        PushDiagnostic(
            m_result.diagnostics,
            m_limits.max_diagnostics,
            MakeDiagnostic(
                code,
                severity,
                provenance,
                section,
                row_index,
                field_name,
                detail));
        if (!m_result.diagnostics.empty() &&
            IsDiagnosticLimit(m_result.diagnostics.back()))
        {
            m_resource_limit = true;
        }
    }

    void RejectValueMeasurement(
        ValueMeasureStatus status,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name)
    {
        if (status == ValueMeasureStatus::BYTE_LIMIT)
        {
            RejectRetained();
            return;
        }
        Push(
            JBeamStructuralDiagnosticCode::PRESERVED_VALUE_LIMIT,
            JBeamStructuralSeverity::ERROR_SEVERITY,
            provenance,
            section,
            row_index,
            field_name,
            status == ValueMeasureStatus::INVALID_GRAPH
                ? "Preserved value contains an alias or cycle"
                : "Preserved value exceeds its depth or work budget");
        m_resource_limit = true;
    }

    bool ReservePreserved(
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        const std::string& detail,
        const JBeamValue* value,
        bool honor_already_retained,
        bool commit_addresses)
    {
        std::size_t string_bytes = 0U;
        if (!AddSize(section.size(), string_bytes) ||
            !AddSize(field_name.size(), string_bytes) ||
            !AddSize(detail.size(), string_bytes))
        {
            RejectRetained();
            return false;
        }
        const std::size_t remaining =
            m_limits.max_retained_bytes -
                std::min(
                    m_retained_bytes,
                    m_limits.max_retained_bytes);
        if (string_bytes > remaining)
        {
            RejectRetained();
            return false;
        }

        ValueMeasurement measurement;
        if (value != NULL)
        {
            const ValueMeasureStatus status = MeasureValue(
                *value,
                remaining - string_bytes,
                honor_already_retained,
                measurement);
            if (status != ValueMeasureStatus::OK)
            {
                RejectValueMeasurement(
                    status,
                    provenance,
                    section,
                    row_index,
                    field_name);
                return false;
            }
        }
        if (measurement.bytes >
            remaining - string_bytes)
        {
            RejectRetained();
            return false;
        }
        if (!ReserveRetained(
                string_bytes + measurement.bytes))
        {
            return false;
        }
        if (commit_addresses)
        {
            m_measured_values.insert(
                measurement.addresses.begin(),
                measurement.addresses.end());
        }
        return true;
    }

    void EmitPreserved(
        JBeamStructuralDiagnosticCode code,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        const std::string& detail,
        const std::shared_ptr<const JBeamValue>& value)
    {
        JBeamStructuralDiagnostic diagnostic = MakeDiagnostic(
            code,
            JBeamStructuralSeverity::WARNING,
            provenance,
            section,
            row_index,
            field_name,
            detail);
        diagnostic.has_preserved_value = static_cast<bool>(value);
        diagnostic.preserved_value = value;
        PushDiagnostic(
            m_result.diagnostics,
            m_limits.max_diagnostics,
            diagnostic);
        if (!m_result.diagnostics.empty() &&
            IsDiagnosticLimit(m_result.diagnostics.back()))
        {
            m_resource_limit = true;
        }
    }

    void PushPreserved(
        JBeamStructuralDiagnosticCode code,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        const std::string& detail,
        const std::shared_ptr<const JBeamValue>& value)
    {
        if (!ReservePreserved(
                provenance,
                section,
                row_index,
                field_name,
                detail,
                value.get(),
                true,
                true))
        {
            return;
        }
        EmitPreserved(
            code,
            provenance,
            section,
            row_index,
            field_name,
            detail,
            value);
    }

    void PushPreservedCopy(
        JBeamStructuralDiagnosticCode code,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        const std::string& detail,
        const JBeamValue& value)
    {
        // Measure the complete fixed/dynamic tree before invoking its copy
        // constructor. Copies are charged independently because their embedded
        // array storage is newly allocated.
        if (!ReservePreserved(
                provenance,
                section,
                row_index,
                field_name,
                detail,
                &value,
                false,
                false))
        {
            return;
        }
        const std::shared_ptr<const JBeamValue> copy(
            new JBeamValue(value));
        EmitPreserved(
            code,
            provenance,
            section,
            row_index,
            field_name,
            detail,
            copy);
    }

    void Finish()
    {
        SortDiagnostics(m_result.diagnostics);
    }

    /// One not-yet-visited slot edge: the child part plus the coordinates of
    /// the slot that mounts it, so the mounting slot's `nodeOffset` can be
    /// found again once the child has its own preorder index.
    struct PendingPart
    {
        const JBeamResolvedPartNode* node;
        bool has_parent;
        std::size_t parent_part_index;
        std::size_t parent_slot_index;
    };

    void CollectParts()
    {
        std::vector<PendingPart> pending;
        std::set<const JBeamResolvedPartNode*> discovered;
        const JBeamResolvedPartNode* root = m_graph.root.get();
        PendingPart pending_root;
        pending_root.node = root;
        pending_root.has_parent = false;
        pending_root.parent_part_index = 0U;
        pending_root.parent_slot_index = 0U;
        pending.push_back(pending_root);
        discovered.insert(root);
        while (!pending.empty())
        {
            const PendingPart current = pending.back();
            const JBeamResolvedPartNode* node = current.node;
            pending.pop_back();
            if (node == NULL)
            {
                continue;
            }
            if (m_parts.size() >= m_limits.max_parts)
            {
                Push(
                    JBeamStructuralDiagnosticCode::RESOLVED_PART_LIMIT,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    JBeamStructuralProvenance(),
                    std::string(),
                    0U,
                    std::string(),
                    "Resolved graph exceeds the configured part limit");
                m_resource_limit = true;
                return;
            }
            PartContext part;
            part.node = node;
            part.has_parent = current.has_parent;
            part.parent_part_index = current.parent_part_index;
            part.parent_slot_index = current.parent_slot_index;
            part.variables.reset(
                new std::vector<JBeamExpressionVariable>());
            std::size_t identity_bytes = 0U;
            if (!AddSize(
                    node->definition.name.size(),
                    identity_bytes) ||
                !AddSize(
                    node->definition.package_path.size(),
                    identity_bytes))
            {
                RejectRetained();
                return;
            }
            if (!ReserveRetained(identity_bytes))
            {
                return;
            }
            std::shared_ptr<JBeamStructuralPartIdentity> identity(
                new JBeamStructuralPartIdentity());
            identity->part_preorder_index = m_parts.size();
            identity->part_name = node->definition.name;
            identity->package_path =
                node->definition.package_path;
            part.provenance.part = identity;
            part.provenance.source_name = InternSourceName(
                node->definition.name_span.source_name);
            if (m_resource_limit)
            {
                return;
            }
            part.provenance.begin =
                node->definition.name_span.begin;
            part.provenance.end = node->definition.name_span.end;
            const std::size_t this_part_index = m_parts.size();
            m_parts.push_back(part);
            JBeamStructuralPart output_part;
            output_part.provenance = part.provenance;
            m_result.parts.push_back(output_part);

            for (std::size_t i = node->slots.size(); i > 0U; --i)
            {
                const JBeamResolvedSlot& slot = node->slots[i - 1U];
                if (slot.child)
                {
                    const JBeamResolvedPartNode* child =
                        slot.child.get();
                    if (!discovered.insert(child).second)
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                INVALID_RESOLVED_GRAPH,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            part.provenance,
                            std::string(),
                            0U,
                            std::string(),
                            "Resolved part graph contains a cycle or "
                            "aliased child");
                        m_resource_limit = true;
                        return;
                    }
                    if (discovered.size() > m_limits.max_parts)
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                RESOLVED_PART_LIMIT,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            part.provenance,
                            std::string(),
                            0U,
                            std::string(),
                            "Resolved graph exceeds the configured "
                            "part limit");
                        m_resource_limit = true;
                        return;
                    }
                    PendingPart pending_child;
                    pending_child.node = child;
                    pending_child.has_parent = true;
                    pending_child.parent_part_index = this_part_index;
                    pending_child.parent_slot_index = i - 1U;
                    pending.push_back(pending_child);
                }
            }
        }
    }

    bool ChargeSemanticWork(
        std::size_t units,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        const std::string& field_name,
        const std::string& detail)
    {
        if (units >
            m_limits.max_expression_work_units -
                std::min(
                    m_expression_work_units,
                    m_limits.max_expression_work_units))
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                0U,
                field_name,
                detail);
            m_resource_limit = true;
            return false;
        }
        m_expression_work_units += units;
        return true;
    }

    bool IsSimpleExpressionVariableName(
        const std::string& name) const
    {
        return name.size() >= 2U &&
            name[0] == '$' &&
            IsExpressionVariableNameSegment(name.substr(1U)) &&
            name.size() <=
                m_limits.expression_limits.max_variable_name_bytes;
    }

    bool ComposeExpressionEnvironment(
        const std::vector<JBeamExpressionVariable>& local_variables,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        const std::string& field_name,
        JBeamExpressionEnvironment& environment)
    {
        if (m_components.size() >
                m_limits.expression_limits.max_variables ||
            local_variables.size() >
                m_limits.expression_limits.max_variables -
                    m_components.size())
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                0U,
                field_name,
                "Expression environment exceeds the configured "
                "variable limit");
            m_resource_limit = true;
            return false;
        }
        const std::size_t count =
            m_components.size() + local_variables.size();
        if (!ChargeSemanticWork(
                count,
                provenance,
                section,
                field_name,
                "Expression environment construction exceeds the "
                "aggregate work limit"))
        {
            return false;
        }
        environment.variables.reserve(count);
        environment.variables.insert(
            environment.variables.end(),
            m_components.begin(),
            m_components.end());
        environment.variables.insert(
            environment.variables.end(),
            local_variables.begin(),
            local_variables.end());
        return true;
    }

    bool FindUnsupportedComponentReference(
        const std::string& expression,
        std::string& rejected_path) const
    {
        std::size_t index = 2U;
        while (index < expression.size())
        {
            if (expression[index] == '\'')
            {
                ++index;
                while (index < expression.size())
                {
                    if (expression[index] == '\\' &&
                        index + 1U < expression.size())
                    {
                        index += 2U;
                        continue;
                    }
                    if (expression[index++] == '\'')
                    {
                        break;
                    }
                }
                continue;
            }
            if (expression[index] != '$')
            {
                ++index;
                continue;
            }
            const std::size_t begin = index++;
            if (index >= expression.size() ||
                (!IsAsciiAlpha(
                    static_cast<unsigned char>(expression[index])) &&
                 expression[index] != '_'))
            {
                continue;
            }
            ++index;
            while (index < expression.size())
            {
                const unsigned char current =
                    static_cast<unsigned char>(expression[index]);
                if (IsAsciiAlpha(current) ||
                    IsAsciiDigit(current) ||
                    current == static_cast<unsigned char>('_'))
                {
                    ++index;
                    continue;
                }
                if (current == static_cast<unsigned char>('.') &&
                    index + 1U < expression.size() &&
                    expression[index + 1U] != '.' &&
                    (IsAsciiAlpha(static_cast<unsigned char>(
                         expression[index + 1U])) ||
                     expression[index + 1U] == '_'))
                {
                    ++index;
                    continue;
                }
                break;
            }
            const std::string token =
                expression.substr(begin, index - begin);
            if (token.compare(0U, 12U, "$components.") != 0)
            {
                continue;
            }
            const std::string component_path = token.substr(12U);
            std::map<std::string, bool>::const_iterator exact =
                m_non_scalar_components.find(component_path);
            if (exact != m_non_scalar_components.end())
            {
                rejected_path = token;
                return true;
            }
            std::string::size_type separator =
                component_path.find_last_of('.');
            while (separator != std::string::npos &&
                   separator != 0U)
            {
                const std::string prefix =
                    component_path.substr(0U, separator);
                const std::map<std::string, bool>::const_iterator
                    parent = m_non_scalar_components.find(prefix);
                if (parent != m_non_scalar_components.end() &&
                    parent->second)
                {
                    rejected_path = "$components." + prefix;
                    return true;
                }
                separator = prefix.find_last_of('.');
            }
        }
        return false;
    }

    bool EvaluateExpressionText(
        const std::string& expression,
        const std::vector<JBeamExpressionVariable>& local_variables,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        JBeamExpressionValue& output,
        bool* undeclared_variable_only = NULL)
    {
        if (m_expression_evaluations >=
            m_limits.max_expression_evaluations)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Structural lowering exceeds the configured expression "
                "evaluation limit");
            m_resource_limit = true;
            return false;
        }
        if (expression.size() >
            m_limits.expression_limits.max_expression_bytes)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Expression exceeds the configured byte limit");
            return false;
        }
        if (!ChargeSemanticWork(
                expression.size(),
                provenance,
                section,
                field_name,
                "Expression scanning exceeds the aggregate semantic "
                "work limit"))
        {
            return false;
        }
        std::string unsupported_component;
        if (FindUnsupportedComponentReference(
                expression, unsupported_component))
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Expression references non-scalar component '" +
                    unsupported_component +
                    "', which is preserved but not allowlisted");
            return false;
        }

        JBeamExpressionEnvironment environment;
        if (!ComposeExpressionEnvironment(
                local_variables,
                provenance,
                section,
                field_name,
                environment))
        {
            return false;
        }
        if (m_expression_work_units >=
            m_limits.max_expression_work_units)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Structural lowering exhausted the aggregate expression "
                "work limit");
            m_resource_limit = true;
            return false;
        }

        JBeamExpressionLimits limits =
            m_limits.expression_limits;
        const std::size_t remaining_work =
            m_limits.max_expression_work_units -
                std::min(
                    m_expression_work_units,
                    m_limits.max_expression_work_units);
        limits.max_work_units =
            std::min(limits.max_work_units, remaining_work);
        ++m_expression_evaluations;
        const JBeamExpressionResult evaluated =
            EvaluateJBeamExpression(expression, environment, limits);
        if (evaluated.work_units > remaining_work)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Expression evaluator exceeded the aggregate work "
                "limit");
            m_resource_limit = true;
            return false;
        }
        m_expression_work_units += evaluated.work_units;
        if (!evaluated.IsValid())
        {
            const bool has_diagnostic =
                !evaluated.diagnostics.empty();
            const JBeamExpressionDiagnostic* diagnostic =
                has_diagnostic ? &evaluated.diagnostics[0] : NULL;
            const bool is_limit =
                diagnostic != NULL &&
                (diagnostic->code ==
                        JBeamExpressionDiagnosticCode::
                            EXPRESSION_SIZE_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::TOKEN_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::DEPTH_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::WORK_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::
                            FUNCTION_ARGUMENT_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::
                            STRING_SIZE_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::
                            OUTPUT_STRING_SIZE_LIMIT ||
                 diagnostic->code ==
                        JBeamExpressionDiagnosticCode::
                            ENVIRONMENT_LIMIT);
            std::ostringstream detail;
            detail.imbue(std::locale::classic());
            detail << "JBeam expression evaluation failed";
            if (diagnostic != NULL)
            {
                detail << " at decoded byte "
                    << diagnostic->byte_offset << " ("
                    << JBeamExpressionDiagnosticCodeToString(
                        diagnostic->code)
                    << "): " << diagnostic->message;
            }
            // An OPTIONAL field whose expression failed only because it
            // read a variable nobody declared is the same situation the
            // nil_means_absent idiom already covers, reached one step later:
            // an unset $variable yields nil, and here that nil was consumed
            // by arithmetic before it could be returned. Shipped content does
            // this - the GT-R's factory spoiler precompresses by
            // "$=$spoiler_angle_R/64+0.8125" against a variable the pack
            // never declares - and refusing it cost three whole
            // configurations over one optional beam modifier. Limits are
            // never downgraded, and a caller that did not opt in is
            // unaffected.
            const bool undeclared_only = !is_limit &&
                JBeamExpressionFailedOnUndeclaredVariableOnly(evaluated);
            if (undeclared_variable_only != NULL && undeclared_only)
            {
                *undeclared_variable_only = true;
                Push(
                    JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                    JBeamStructuralSeverity::WARNING,
                    provenance,
                    section,
                    row_index,
                    field_name,
                    detail.str() + " | the expression reads undeclared "
                    "variable '" + evaluated.undeclared_variables[0] +
                    "', so this optional field is treated as unauthored");
                return false;
            }
            Push(
                is_limit
                    ? JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT
                    : JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                detail.str());
            return false;
        }
        output = evaluated.value;
        return true;
    }

    const JBeamExpressionValue* FindLocalExpressionVariable(
        const std::vector<JBeamExpressionVariable>& variables,
        const std::string& name) const
    {
        for (std::size_t i = variables.size(); i > 0U; --i)
        {
            if (variables[i - 1U].name == name)
            {
                return &variables[i - 1U].value;
            }
        }
        return NULL;
    }

    bool ExpandNamespaceString(
        const std::string& authored,
        const std::vector<JBeamExpressionVariable>& variables,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        JBeamExpressionValue& output)
    {
        std::string expanded;
        const JBeamExpressionValue* prefix =
            FindLocalExpressionVariable(variables, "$prefix");
        const JBeamExpressionValue* suffix =
            FindLocalExpressionVariable(variables, "$suffix");
        if ((prefix != NULL &&
             prefix->type != JBeamExpressionValueType::NIL_VALUE &&
             prefix->type != JBeamExpressionValueType::STRING) ||
            (suffix != NULL &&
             suffix->type != JBeamExpressionValueType::NIL_VALUE &&
             suffix->type != JBeamExpressionValueType::STRING))
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Namespace variables $prefix and $suffix must be "
                "strings or nil");
            return false;
        }
        const std::size_t prefix_size =
            prefix != NULL &&
                    prefix->type == JBeamExpressionValueType::STRING
                ? prefix->string_value.size()
                : 0U;
        const std::size_t suffix_size =
            suffix != NULL &&
                    suffix->type == JBeamExpressionValueType::STRING
                ? suffix->string_value.size()
                : 0U;
        std::size_t output_size = prefix_size;
        if (!AddSize(authored.size() - 2U, output_size) ||
            !AddSize(suffix_size, output_size) ||
            output_size >
                m_limits.expression_limits.max_output_string_bytes)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Expanded namespace string exceeds the configured "
                "output limit");
            return false;
        }
        expanded.reserve(output_size);
        if (prefix_size != 0U)
        {
            expanded.append(prefix->string_value);
        }
        expanded.append(authored, 2U, std::string::npos);
        if (suffix_size != 0U)
        {
            expanded.append(suffix->string_value);
        }
        std::size_t namespace_work = output_size;
        if (!AddSize(1U, namespace_work))
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                field_name,
                "Namespace expansion work accounting overflowed");
            m_resource_limit = true;
            return false;
        }
        if (!ChargeSemanticWork(
                namespace_work,
                provenance,
                section,
                field_name,
                "Namespace expansion exceeds the aggregate semantic "
                "work limit"))
        {
            return false;
        }
        output = JBeamExpressionValue::String(expanded);
        return true;
    }

    bool ResolveScalarValue(
        const JBeamValue& value,
        const std::vector<JBeamExpressionVariable>& variables,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& field_name,
        JBeamExpressionValue& output,
        bool* undeclared_variable_only = NULL)
    {
        switch (value.type)
        {
        case JBeamValueType::NULL_VALUE:
            output = JBeamExpressionValue::Nil();
            return true;
        case JBeamValueType::BOOLEAN:
            output = JBeamExpressionValue::Boolean(
                value.boolean_value);
            return true;
        case JBeamValueType::NUMBER:
            if (!IsFiniteDouble(value.number_value))
            {
                Push(
                    JBeamStructuralDiagnosticCode::NON_FINITE_NUMBER,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    provenance,
                    section,
                    row_index,
                    field_name,
                    "Non-finite structural numbers are rejected");
                return false;
            }
            output = JBeamExpressionValue::Number(
                value.number_value == 0.0 ? 0.0 : value.number_value);
            return true;
        case JBeamValueType::STRING:
            if (value.scalar_text.compare(0U, 2U, "$.") == 0)
            {
                return ExpandNamespaceString(
                    value.scalar_text,
                    variables,
                    provenance,
                    section,
                    row_index,
                    field_name,
                    output);
            }
            if (IsExpressionString(value))
            {
                const std::string expression =
                    value.scalar_text.compare(0U, 2U, "$=") == 0
                        ? value.scalar_text
                        : std::string("$=") + value.scalar_text;
                return EvaluateExpressionText(
                    expression,
                    variables,
                    provenance,
                    section,
                    row_index,
                    field_name,
                    output,
                    undeclared_variable_only);
            }
            output = JBeamExpressionValue::String(
                value.scalar_text);
            return true;
        case JBeamValueType::ARRAY:
        case JBeamValueType::OBJECT:
            return false;
        }
        return false;
    }

    bool EraseComponentPath(
        std::map<std::string, JBeamExpressionValue>& values,
        const std::string& path,
        bool descendants,
        const JBeamStructuralProvenance& provenance)
    {
        const std::string prefix = path + ".";
        std::map<std::string, JBeamExpressionValue>::iterator iterator =
            values.begin();
        while (iterator != values.end())
        {
            if (!ChargeSemanticWork(
                    1U,
                    provenance,
                    "components",
                    path,
                    "Component merging exceeds the aggregate semantic "
                    "work limit"))
            {
                return false;
            }
            const bool exact = iterator->first == path;
            const bool child =
                descendants &&
                iterator->first.size() > prefix.size() &&
                iterator->first.compare(
                    0U, prefix.size(), prefix) == 0;
            if (exact || child)
            {
                values.erase(iterator++);
            }
            else
            {
                ++iterator;
            }
        }
        return true;
    }

    bool EraseNonScalarComponentPath(
        const std::string& path,
        bool descendants,
        const JBeamStructuralProvenance& provenance)
    {
        const std::string prefix = path + ".";
        std::map<std::string, bool>::iterator iterator =
            m_non_scalar_components.begin();
        while (iterator != m_non_scalar_components.end())
        {
            if (!ChargeSemanticWork(
                    1U,
                    provenance,
                    "components",
                    path,
                    "Component classification exceeds the aggregate "
                    "semantic work limit"))
            {
                return false;
            }
            const bool exact = iterator->first == path;
            const bool child =
                descendants &&
                iterator->first.size() > prefix.size() &&
                iterator->first.compare(
                    0U, prefix.size(), prefix) == 0;
            if (exact || child)
            {
                m_non_scalar_components.erase(iterator++);
            }
            else
            {
                ++iterator;
            }
        }
        return true;
    }

    bool EraseComponentArrayPath(
        const std::string& path,
        bool descendants,
        const JBeamStructuralProvenance& provenance)
    {
        const std::string prefix = path + ".";
        std::map<
            std::string,
            std::shared_ptr<const JBeamValue> >::iterator iterator =
                m_component_arrays.begin();
        while (iterator != m_component_arrays.end())
        {
            if (!ChargeSemanticWork(
                    1U,
                    provenance,
                    "components",
                    path,
                    "Component-row classification exceeds the aggregate "
                    "semantic work limit"))
            {
                return false;
            }
            const bool exact = iterator->first == path;
            const bool child =
                descendants &&
                iterator->first.size() > prefix.size() &&
                iterator->first.compare(
                    0U, prefix.size(), prefix) == 0;
            if (exact || child)
            {
                m_component_arrays.erase(iterator++);
            }
            else
            {
                ++iterator;
            }
        }
        return true;
    }

    bool SetComponentArray(
        const std::string& path,
        const std::shared_ptr<const JBeamValue>& value)
    {
        std::map<
            std::string,
            std::shared_ptr<const JBeamValue> >::iterator found =
                m_component_arrays.find(path);
        if (found != m_component_arrays.end())
        {
            found->second = value;
            return true;
        }
        std::size_t bytes = path.size();
        if (!AddSize(sizeof(std::shared_ptr<const JBeamValue>), bytes))
        {
            RejectRetained();
            return false;
        }
        if (!ReserveSemantic(bytes))
        {
            return false;
        }
        m_component_arrays.insert(std::make_pair(path, value));
        return true;
    }

    bool SetNonScalarComponent(
        const std::string& path,
        bool blocks_descendants)
    {
        std::map<std::string, bool>::iterator found =
            m_non_scalar_components.find(path);
        if (found != m_non_scalar_components.end())
        {
            found->second = blocks_descendants;
            return true;
        }
        std::size_t bytes = path.size();
        if (!AddSize(sizeof(bool), bytes))
        {
            RejectRetained();
            return false;
        }
        if (!ReserveSemantic(bytes))
        {
            return false;
        }
        m_non_scalar_components.insert(
            std::make_pair(path, blocks_descendants));
        return true;
    }

    void PreserveUnsupportedComponent(
        JBeamStructuralDiagnosticCode code,
        const PartContext& part,
        const std::string& path,
        const std::string& detail,
        const std::shared_ptr<const JBeamValue>& value)
    {
        PushPreserved(
            code,
            value
                ? ProvenanceWithSpan(part.provenance, value->span)
                : part.provenance,
            "components",
            0U,
            path,
            detail,
            value);
    }

    void MergeComponentValue(
        const PartContext& part,
        const std::string& path,
        const std::shared_ptr<const JBeamValue>& value,
        std::size_t depth,
        std::map<std::string, JBeamExpressionValue>& values)
    {
        if (m_resource_limit)
        {
            return;
        }
        if (m_component_nodes >= m_limits.max_component_nodes ||
            depth > m_limits.max_component_depth)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                part.provenance,
                "components",
                0U,
                path,
                "Component tree exceeds the configured node or depth "
                "limit");
            m_resource_limit = true;
            return;
        }
        ++m_component_nodes;
        if (!ChargeSemanticWork(
                1U,
                part.provenance,
                "components",
                path,
                "Component discovery exceeds the aggregate semantic "
                "work limit"))
        {
            return;
        }
        if (!value)
        {
            PreserveUnsupportedComponent(
                JBeamStructuralDiagnosticCode::
                    UNSUPPORTED_COMPONENT_VALUE,
                part,
                path,
                "Null component storage is preserved but unavailable "
                "to expressions",
                value);
            return;
        }
        if (value->type == JBeamValueType::OBJECT)
        {
            if (!EraseComponentPath(
                    values, path, false, part.provenance) ||
                !EraseComponentArrayPath(
                    path, false, part.provenance))
            {
                return;
            }
            if (!path.empty())
            {
                if (!SetNonScalarComponent(path, false))
                {
                    return;
                }
            }
            for (std::size_t i = 0U;
                 i < value->object_fields.size();
                 ++i)
            {
                const JBeamObjectField& field =
                    value->object_fields[i];
                if (!IsComponentPathSegment(field.key))
                {
                    PreserveUnsupportedComponent(
                        JBeamStructuralDiagnosticCode::
                            INVALID_COMPONENT_PATH,
                        part,
                        path.empty()
                            ? field.key
                            : path + "." + field.key,
                        "Component path segment cannot be represented by "
                        "the allowlisted scalar evaluator",
                        field.value);
                    continue;
                }
                const std::string child_path = path.empty()
                    ? field.key
                    : path + "." + field.key;
                const std::size_t component_prefix_bytes = 12U;
                if (m_limits.expression_limits.
                            max_variable_name_bytes <
                        component_prefix_bytes ||
                    child_path.size() >
                        m_limits.expression_limits.
                            max_variable_name_bytes -
                            component_prefix_bytes)
                {
                    PreserveUnsupportedComponent(
                        JBeamStructuralDiagnosticCode::
                            INVALID_COMPONENT_PATH,
                        part,
                        child_path,
                        "Flattened component path exceeds the "
                        "configured expression-variable name limit",
                        field.value);
                    continue;
                }
                MergeComponentValue(
                    part,
                    child_path,
                    field.value,
                    depth + 1U,
                    values);
            }
            return;
        }
        if (!EraseComponentPath(
                values, path, true, part.provenance) ||
            !EraseComponentArrayPath(
                path, true, part.provenance))
        {
            return;
        }
        if (!EraseNonScalarComponentPath(
                path, true, part.provenance))
        {
            return;
        }
        if (value->type == JBeamValueType::ARRAY)
        {
            if (!SetNonScalarComponent(path, true) ||
                !SetComponentArray(path, value))
            {
                return;
            }
            return;
        }
        if (value->type == JBeamValueType::STRING &&
            IsExpressionString(*value))
        {
            if (!SetNonScalarComponent(path, true))
            {
                return;
            }
            PreserveUnsupportedComponent(
                JBeamStructuralDiagnosticCode::
                    UNSUPPORTED_COMPONENT_VALUE,
                part,
                path,
                "Expression-valued component is preserved; recursive "
                "component evaluation is not allowlisted",
                value);
            return;
        }
        if (value->type == JBeamValueType::STRING &&
            value->scalar_text.size() >
                m_limits.expression_limits.max_string_bytes)
        {
            if (!SetNonScalarComponent(path, true))
            {
                return;
            }
            PreserveUnsupportedComponent(
                JBeamStructuralDiagnosticCode::
                    UNSUPPORTED_COMPONENT_VALUE,
                part,
                path,
                "Component string exceeds the scalar evaluator limit "
                "and is preserved without entering the environment",
                value);
            return;
        }

        JBeamExpressionValue scalar;
        if (!ResolveScalarValue(
                *value,
                std::vector<JBeamExpressionVariable>(),
                ProvenanceWithSpan(part.provenance, value->span),
                "components",
                0U,
                path,
                scalar))
        {
            return;
        }
        std::size_t admission_bytes = 12U;
        if (!AddSize(path.size(), admission_bytes) ||
            (scalar.type == JBeamExpressionValueType::STRING &&
             !AddSize(
                 scalar.string_value.size(), admission_bytes)))
        {
            RejectRetained();
            return;
        }
        if (!ReserveSemantic(admission_bytes))
        {
            return;
        }
        values[path] = scalar;
        if (values.size() >
            m_limits.expression_limits.max_variables)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                part.provenance,
                "components",
                0U,
                path,
                "Flattened component environment exceeds the "
                "configured variable limit");
            m_resource_limit = true;
        }
    }

    void BuildComponentEnvironment()
    {
        std::map<std::string, JBeamExpressionValue> values;
        for (std::size_t part_index = 0U;
             part_index < m_parts.size() && !m_resource_limit;
             ++part_index)
        {
            const PartContext& part = m_parts[part_index];
            const JBeamValue& body = part.node->definition.body;
            if (body.type != JBeamValueType::OBJECT)
            {
                continue;
            }
            for (std::size_t field_index = 0U;
                 field_index < body.object_fields.size();
                 ++field_index)
            {
                const JBeamObjectField& field =
                    body.object_fields[field_index];
                if (field.key != "components")
                {
                    continue;
                }
                if (!field.value ||
                    field.value->type != JBeamValueType::OBJECT)
                {
                    PreserveUnsupportedComponent(
                        JBeamStructuralDiagnosticCode::
                            UNSUPPORTED_COMPONENT_VALUE,
                        part,
                        "$components",
                        "The components section must be a dictionary; "
                        "its inert value is preserved",
                        field.value);
                    continue;
                }
                MergeComponentValue(
                    part,
                    std::string(),
                    field.value,
                    0U,
                    values);
            }
        }
        if (m_resource_limit)
        {
            return;
        }
        m_components.reserve(values.size());
        for (std::map<std::string, JBeamExpressionValue>::
                 const_iterator iterator = values.begin();
             iterator != values.end();
             ++iterator)
        {
            JBeamExpressionVariable variable;
            variable.name = "$components." + iterator->first;
            variable.value = iterator->second;
            std::size_t bytes = variable.name.size();
            if (variable.value.type ==
                JBeamExpressionValueType::STRING)
            {
                const std::size_t string_bytes =
                    variable.value.string_value.size();
                if (string_bytes >
                        m_limits.expression_limits.max_string_bytes ||
                    string_bytes >
                        m_limits.expression_limits.
                            max_environment_string_bytes -
                            std::min(
                                m_component_string_bytes,
                                m_limits.expression_limits.
                                    max_environment_string_bytes))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::
                            EXPRESSION_LIMIT,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        JBeamStructuralProvenance(),
                        "components",
                        0U,
                        variable.name,
                        "Flattened component strings exceed the "
                        "configured expression-environment limit");
                    m_resource_limit = true;
                    return;
                }
                if (!AddSize(string_bytes, bytes))
                {
                    RejectRetained();
                    return;
                }
                m_component_string_bytes += string_bytes;
            }
            if (!ReserveSemantic(bytes))
            {
                return;
            }
            m_components.push_back(variable);
        }
    }

    void BuildPartVariableEnvironments()
    {
        for (std::size_t part_index = 0U;
             part_index < m_parts.size() && !m_resource_limit;
             ++part_index)
        {
            PartContext& part = m_parts[part_index];
            const std::vector<JBeamVariableAssignment>& authored =
                part.node->inherited_variables;
            if (authored.size() >
                m_limits.expression_limits.max_variables -
                    std::min(
                        m_components.size(),
                        m_limits.expression_limits.max_variables))
            {
                Push(
                    JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    part.provenance,
                    "variables",
                    0U,
                    std::string(),
                    "Resolved part expression environment exceeds the "
                    "configured variable limit");
                m_resource_limit = true;
                return;
            }
            part.variables->reserve(authored.size());
            std::size_t environment_string_bytes =
                m_component_string_bytes;
            for (std::size_t i = 0U;
                 i < authored.size() && !m_resource_limit;
                 ++i)
            {
                const JBeamVariableAssignment& assignment =
                    authored[i];
                const JBeamStructuralProvenance provenance =
                    ProvenanceWithSpan(
                        part.provenance, assignment.span);
                if (!IsSimpleExpressionVariableName(
                        assignment.name))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::
                            INVALID_VARIABLE_VALUE,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        provenance,
                        "variables",
                        i,
                        assignment.name,
                        "Resolved variable name is not accepted by the "
                        "allowlisted evaluator");
                    continue;
                }
                JBeamExpressionValue value;
                if (!ResolveScalarValue(
                        assignment.value,
                        *part.variables,
                        provenance,
                        "variables",
                        i,
                        assignment.name,
                        value))
                {
                    if (assignment.value.type ==
                            JBeamValueType::ARRAY ||
                        assignment.value.type ==
                            JBeamValueType::OBJECT)
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                INVALID_VARIABLE_VALUE,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            provenance,
                            "variables",
                            i,
                            assignment.name,
                            "Resolved expression variables must be "
                            "scalar values");
                    }
                    continue;
                }
                JBeamExpressionVariable variable;
                variable.name = assignment.name;
                variable.value = value;
                std::size_t bytes = variable.name.size();
                if (variable.value.type ==
                    JBeamExpressionValueType::STRING)
                {
                    const std::size_t string_bytes =
                        variable.value.string_value.size();
                    if (string_bytes >
                            m_limits.expression_limits.max_string_bytes ||
                        string_bytes >
                            m_limits.expression_limits.
                                max_environment_string_bytes -
                                std::min(
                                    environment_string_bytes,
                                    m_limits.expression_limits.
                                        max_environment_string_bytes))
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                EXPRESSION_LIMIT,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            provenance,
                            "variables",
                            i,
                            assignment.name,
                            "Resolved variable strings exceed the "
                            "configured expression-environment limit");
                        m_resource_limit = true;
                        return;
                    }
                    if (!AddSize(string_bytes, bytes))
                    {
                        RejectRetained();
                        return;
                    }
                    environment_string_bytes += string_bytes;
                }
                if (!ReserveSemantic(bytes))
                {
                    return;
                }
                part.variables->push_back(variable);
            }
        }
    }

    bool ReadNodeOffsetComponent(
        const JBeamValue& value,
        const PartContext& mounting_part,
        const JBeamStructuralProvenance& provenance,
        const char* component,
        double& output)
    {
        const std::string field =
            std::string("nodeOffset.") + component;
        JBeamExpressionValue evaluated;
        if (!ResolveScalarValue(
                value,
                *mounting_part.variables,
                provenance,
                "slots",
                0U,
                field,
                evaluated))
        {
            return false;
        }
        // An absent tuning variable evaluates to nil, which is the documented
        // "component not offset" answer rather than an error; anything else
        // non-numeric is a shape the translation cannot express.
        if (evaluated.type == JBeamExpressionValueType::NIL_VALUE)
        {
            output = 0.0;
            return true;
        }
        if (evaluated.type != JBeamExpressionValueType::NUMBER ||
            !IsFiniteDouble(evaluated.number_value))
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_SLOT_NODE_OFFSET,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                "slots",
                0U,
                field,
                "Slot nodeOffset components must evaluate to a finite "
                "number");
            return false;
        }
        output = evaluated.number_value;
        return true;
    }

    /// Accumulates each mounting slot's `nodeOffset` from the root down. Parts
    /// are collected in preorder, so a part's parent is always resolved before
    /// it and one forward pass suffices.
    ///
    /// The offset is evaluated in the *mounting* part's variable scope, not
    /// the mounted part's: the slot row belongs to the parent, and the tuning
    /// variables it reads (`$trackoffset_F` and friends) are declared there.
    void ResolveSlotNodeOffsets()
    {
        for (std::size_t part_index = 0U;
             part_index < m_parts.size() && !m_resource_limit;
             ++part_index)
        {
            PartContext& part = m_parts[part_index];
            if (!part.has_parent)
            {
                continue;
            }
            // Defensive: preorder guarantees this, and a violated invariant
            // must not read an unresolved parent's offset.
            if (part.parent_part_index >= part_index)
            {
                Push(
                    JBeamStructuralDiagnosticCode::INVALID_RESOLVED_GRAPH,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    part.provenance,
                    "slots",
                    0U,
                    std::string(),
                    "Resolved part graph is not in parent-before-child "
                    "order");
                m_resource_limit = true;
                return;
            }
            const PartContext& parent = m_parts[part.parent_part_index];
            part.has_node_offset = parent.has_node_offset;
            part.offset_x = parent.offset_x;
            part.offset_y = parent.offset_y;
            part.offset_z = parent.offset_z;

            if (part.parent_slot_index >= parent.node->slots.size())
            {
                Push(
                    JBeamStructuralDiagnosticCode::INVALID_RESOLVED_GRAPH,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    part.provenance,
                    "slots",
                    0U,
                    std::string(),
                    "Resolved part records a mounting slot its parent does "
                    "not declare");
                m_resource_limit = true;
                return;
            }
            const JBeamSlotNodeOffset& offset =
                parent.node->slots[part.parent_slot_index]
                    .definition.node_offset;
            if (!offset.present)
            {
                continue;
            }

            double dx = 0.0;
            double dy = 0.0;
            double dz = 0.0;
            bool valid = true;
            if (offset.has_x)
            {
                valid = ReadNodeOffsetComponent(
                            offset.x, parent, part.provenance, "x", dx) &&
                        valid;
            }
            if (offset.has_y)
            {
                valid = ReadNodeOffsetComponent(
                            offset.y, parent, part.provenance, "y", dy) &&
                        valid;
            }
            if (offset.has_z)
            {
                valid = ReadNodeOffsetComponent(
                            offset.z, parent, part.provenance, "z", dz) &&
                        valid;
            }
            if (!valid)
            {
                // Fail closed for this subtree only: the part stays at the
                // parent's accumulated offset and the diagnostic already
                // records why. Placement of unrelated parts is unaffected.
                continue;
            }
            part.offset_x += dx;
            part.offset_y += dy;
            part.offset_z += dz;
            part.has_node_offset = part.has_node_offset ||
                                   dx != 0.0 || dy != 0.0 || dz != 0.0;
        }
    }

    /// Translates one authored node by its part's accumulated slot
    /// `nodeOffset`.
    ///
    /// The lateral component is mirrored by the sign of the node's own
    /// authored X. That is what lets a package author one wheel and mount it
    /// four times: the offset states how far the axle sits *outboard*, and
    /// each side takes it in its own direction. Adding it unmirrored would
    /// slide the whole axle sideways instead of setting a track width.
    ///
    /// A node authored exactly on the centreline takes no lateral offset:
    /// there is no side for it to be pushed towards, and picking one would be
    /// a guess.
    ///
    /// The sign is taken from the authored coordinate rather than the
    /// running total, so a stack of offsets cannot flip direction part-way by
    /// carrying a node across the centreline.
    static void ApplySlotNodeOffset(
        const PartContext& part,
        JBeamStructuralNode& node)
    {
        if (!part.has_node_offset)
        {
            return;
        }
        if (node.x > 0.0)
        {
            node.x += part.offset_x;
        }
        else if (node.x < 0.0)
        {
            node.x -= part.offset_x;
        }
        node.y += part.offset_y;
        node.z += part.offset_z;
    }

    bool PreflightComponentReferenceText(
        const JBeamValue& value,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index)
    {
        if (value.scalar_text.size() >
            m_limits.expression_limits.max_expression_bytes)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                ProvenanceWithSpan(provenance, value.span),
                section,
                row_index,
                "$components",
                "Component reference exceeds the configured expression "
                "byte limit");
            return false;
        }
        return ChargeSemanticWork(
            value.scalar_text.size(),
            provenance,
            section,
            "$components",
            "Component-reference scanning exceeds the aggregate semantic "
            "work limit");
    }

    const JBeamValue* ResolveStructuralSectionValue(
        const PartContext& part,
        const std::string& section,
        const std::shared_ptr<const JBeamValue>& value)
    {
        if (!value || !IsExpressionString(*value))
        {
            return value.get();
        }
        if (!PreflightComponentReferenceText(
                *value, part.provenance, section, 0U))
        {
            return NULL;
        }
        std::string path;
        if (!ParseExactComponentReference(*value, path))
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                ProvenanceWithSpan(part.provenance, value->span),
                section,
                0U,
                "$components",
                "A structural section expression must be an exact "
                "$=$components.path table reference");
            return NULL;
        }
        const std::map<
            std::string,
            std::shared_ptr<const JBeamValue> >::const_iterator found =
                m_component_arrays.find(path);
        if (found == m_component_arrays.end() || !found->second)
        {
            Push(
                JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                ProvenanceWithSpan(part.provenance, value->span),
                section,
                0U,
                "$components." + path,
                "Exact component table reference does not resolve to "
                "an effective array-valued component");
            return NULL;
        }
        return found->second.get();
    }

    void DiscoverSections()
    {
        for (std::size_t part_index = 0U;
             part_index < m_parts.size();
             ++part_index)
        {
            if (m_resource_limit)
            {
                return;
            }
            PartSections sections;
            sections.part = m_parts[part_index];
            const JBeamValue& body =
                sections.part.node->definition.body;
            if (body.type != JBeamValueType::OBJECT)
            {
                Push(
                    JBeamStructuralDiagnosticCode::PART_BODY_NOT_OBJECT,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    sections.part.provenance,
                    std::string(),
                    0U,
                    std::string(),
                    "Resolved part body must be an object");
                m_sections.push_back(sections);
                continue;
            }
            for (std::size_t field_index = 0U;
                 field_index < body.object_fields.size();
                 ++field_index)
            {
                const JBeamObjectField& field =
                    body.object_fields[field_index];
                if (IsRecognizedSection(field.key))
                {
                    const JBeamValue* effective_value =
                        ResolveStructuralSectionValue(
                            sections.part,
                            field.key,
                            field.value);
                    if (field.value &&
                        IsExpressionString(*field.value) &&
                        effective_value == NULL)
                    {
                        continue;
                    }
                    SectionOccurrence occurrence;
                    occurrence.value = effective_value;
                    occurrence.span = effective_value
                        ? effective_value->span
                        : field.key_span;
                    sections.sections[field.key].push_back(
                        occurrence);
                }
                else if (IsAerodynamicScaleName(field.key))
                {
                    PushPreserved(JBeamStructuralDiagnosticCode::UNSUPPORTED_FIELD,
                        ProvenanceWithSpan(sections.part.provenance,
                            field.value ? field.value->span : field.key_span),
                        field.key, field_index, field.key,
                        "Aerodynamic scale is retained as typed metadata; no native force consumer is applied",
                        field.value);
                    if (m_resource_limit) return;
                }
                else if (!IsResolverMetadataSection(field.key))
                {
                    PushPreserved(
                        JBeamStructuralDiagnosticCode::UNKNOWN_SECTION,
                        ProvenanceWithSpan(
                            sections.part.provenance,
                            field.value
                                ? field.value->span
                                : field.key_span),
                        field.key,
                        0U,
                        field.key,
                        "Section is outside the structural J2 subset",
                        field.value);
                    if (m_resource_limit)
                    {
                        return;
                    }
                }
            }
            m_sections.push_back(sections);
        }
    }

    bool IsValidTableHeader(const JBeamValue& value) const
    {
        if (value.type != JBeamValueType::ARRAY ||
            value.array_values.empty())
        {
            return false;
        }
        const JBeamValue& header = value.array_values[0];
        if (header.type != JBeamValueType::ARRAY ||
            header.array_values.empty())
        {
            return false;
        }
        for (std::size_t i = 0U;
             i < header.array_values.size();
             ++i)
        {
            if (header.array_values[i].type !=
                    JBeamValueType::STRING ||
                header.array_values[i].scalar_text.empty())
            {
                return false;
            }
        }
        return true;
    }

    /// True when every occurrence but the last is a well-formed table that
    /// declares no rows: its header followed by nothing but modifiers. A
    /// string there could be a component reference that expands into rows,
    /// so only objects -- which JBeam defines as modifiers -- qualify.
    bool AreLeadingOccurrencesEmptyTables(
        const std::vector<SectionOccurrence>& occurrences) const
    {
        for (std::size_t occurrence_index = 0U;
             occurrence_index + 1U < occurrences.size();
             ++occurrence_index)
        {
            const JBeamValue* value = occurrences[occurrence_index].value;
            if (value == NULL || !IsValidTableHeader(*value))
            {
                return false;
            }
            for (std::size_t row_index = 1U;
                 row_index < value->array_values.size();
                 ++row_index)
            {
                if (value->array_values[row_index].type !=
                        JBeamValueType::OBJECT)
                {
                    return false;
                }
            }
        }
        return true;
    }

    void PreflightRows()
    {
        static const char* const names[] = {
            "nodes", "beams", "triangles", "quads", "refNodes"
        };
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t name_index = 0U;
                 name_index < sizeof(names) / sizeof(names[0]);
                 ++name_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const std::string name(names[name_index]);
                std::vector<SectionOccurrence>& occurrences =
                    m_sections[part_index].sections[name];
                if (occurrences.size() > 1U &&
                    AreLeadingOccurrencesEmptyTables(occurrences))
                {
                    // BeamNG reads a part into a Lua table, where a repeated
                    // key keeps its last value -- the rule its documentation
                    // states for a repeated part name. Taking that reading is
                    // safe only when nothing else could have been meant:
                    // every earlier occurrence here is a table that declares
                    // no rows, so keeping the last loses no geometry, and a
                    // reader that concatenated them would build the same
                    // rows. The Germanwings A319 opens `triangles` with a
                    // header and a lone modifier before the section holding
                    // its 229 triangles.
                    Push(
                        JBeamStructuralDiagnosticCode::DUPLICATE_SECTION,
                        JBeamStructuralSeverity::WARNING,
                        m_sections[part_index].part.provenance,
                        name,
                        0U,
                        std::string(),
                        "Repeated structural section: every earlier "
                        "occurrence declares no rows, so the last is used");
                    occurrences.erase(
                        occurrences.begin(), occurrences.end() - 1);
                }
                else if (occurrences.size() > 1U)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::DUPLICATE_SECTION,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        name,
                        0U,
                        std::string(),
                        "Duplicate structural section is ambiguous");
                }
                for (std::size_t occurrence_index = 0U;
                     occurrence_index < occurrences.size();
                     ++occurrence_index)
                {
                    if (m_resource_limit)
                    {
                        return;
                    }
                    const JBeamValue* value =
                        occurrences[occurrence_index].value;
                    if (value == NULL)
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::INVALID_SECTION,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            m_sections[part_index].part.provenance,
                            name,
                            0U,
                            std::string(),
                            "Structural section has no value");
                        continue;
                    }
                    if (!IsValidTableHeader(*value))
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                INVALID_TABLE_HEADER,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            m_sections[part_index].part.provenance,
                            name,
                            0U,
                            std::string(),
                            "Structural section requires a non-empty "
                            "string table header");
                        continue;
                    }
                    const std::size_t row_count =
                        value->array_values.size() - 1U;
                    if (row_count >
                        m_limits.max_rows -
                            std::min(
                                m_result.authored_row_count,
                                m_limits.max_rows))
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::ROW_LIMIT,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            m_sections[part_index].part.provenance,
                            name,
                            0U,
                            std::string(),
                            "Structural tables exceed the configured "
                            "row limit");
                        m_resource_limit = true;
                        return;
                    }
                    m_result.authored_row_count += row_count;
                }
            }
        }
    }

    bool ExpandComponentRows(
        const JBeamValue& source,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        JBeamValue& output,
        bool& expanded)
    {
        expanded = false;
        if (source.type != JBeamValueType::ARRAY ||
            source.array_values.size() < 2U)
        {
            return true;
        }

        typedef std::pair<
            std::size_t,
            std::shared_ptr<const JBeamValue> > Replacement;
        std::vector<Replacement> replacements;
        for (std::size_t i = 1U;
             i < source.array_values.size();
             ++i)
        {
            if (!ChargeSemanticWork(
                    1U,
                    provenance,
                    section,
                    std::string(),
                    "Component-row expansion exceeds the aggregate "
                    "semantic work limit"))
            {
                return false;
            }
            std::string path;
            if (IsExpressionString(source.array_values[i]) &&
                !PreflightComponentReferenceText(
                    source.array_values[i],
                    provenance,
                    section,
                    i - 1U))
            {
                return false;
            }
            if (!ParseExactComponentReference(
                    source.array_values[i], path))
            {
                if (IsExpressionString(source.array_values[i]))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        ProvenanceWithSpan(
                            provenance,
                            source.array_values[i].span),
                        section,
                        i - 1U,
                        "$components",
                        "A structural table-entry expression must be an "
                        "exact $=$components.path row reference");
                    return false;
                }
                continue;
            }
            const std::map<
                std::string,
                std::shared_ptr<const JBeamValue> >::const_iterator found =
                    m_component_arrays.find(path);
            if (found == m_component_arrays.end())
            {
                Push(
                    JBeamStructuralDiagnosticCode::EXPRESSION_ERROR,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    ProvenanceWithSpan(
                        provenance,
                        source.array_values[i].span),
                    section,
                    i - 1U,
                    "$components." + path,
                    "Exact component row reference does not resolve to "
                    "an effective array-valued component");
                return false;
            }
            replacements.push_back(
                std::make_pair(i, found->second));
        }
        if (replacements.empty())
        {
            return true;
        }

        std::size_t admitted = m_retained_bytes;
        if (!AddSize(m_semantic_bytes, admitted))
        {
            RejectRetained();
            return false;
        }
        const std::size_t remaining =
            m_limits.max_retained_bytes -
                std::min(admitted, m_limits.max_retained_bytes);
        ValueMeasurement source_measurement;
        ValueMeasureStatus status = MeasureValue(
            source,
            remaining,
            false,
            source_measurement);
        if (status != ValueMeasureStatus::OK)
        {
            RejectValueMeasurement(
                status,
                provenance,
                section,
                0U,
                "$components");
            return false;
        }
        std::size_t bytes = source_measurement.bytes;
        for (std::size_t i = 0U; i < replacements.size(); ++i)
        {
            if (!replacements[i].second)
            {
                continue;
            }
            ValueMeasurement replacement_measurement;
            status = MeasureValue(
                *replacements[i].second,
                remaining - std::min(bytes, remaining),
                false,
                replacement_measurement);
            if (status != ValueMeasureStatus::OK)
            {
                RejectValueMeasurement(
                    status,
                    provenance,
                    section,
                    replacements[i].first,
                    "$components");
                return false;
            }
            if (!AddSize(replacement_measurement.bytes, bytes) ||
                bytes > remaining)
            {
                RejectRetained();
                return false;
            }
        }
        if (!ReserveSemantic(bytes))
        {
            return false;
        }

        output = source;
        for (std::size_t i = 0U; i < replacements.size(); ++i)
        {
            output.array_values[replacements[i].first] =
                *replacements[i].second;
        }
        expanded = true;
        return true;
    }

    const JBeamNormalizedTable* NormalizeSection(
        const PartSections& sections,
        const std::string& name,
        JBeamNormalizeResult& normalized)
    {
        const std::map<
            std::string,
            std::vector<SectionOccurrence> >::const_iterator found =
                sections.sections.find(name);
        if (found == sections.sections.end() ||
            found->second.size() != 1U ||
            found->second[0].value == NULL ||
            !IsValidTableHeader(*found->second[0].value))
        {
            return NULL;
        }
        JBeamValue expanded_value;
        bool expanded = false;
        if (!ExpandComponentRows(
                *found->second[0].value,
                sections.part.provenance,
                name,
                expanded_value,
                expanded))
        {
            return NULL;
        }
        const JBeamValue& normalization_input = expanded
            ? expanded_value
            : *found->second[0].value;
        std::size_t admitted_bytes = m_retained_bytes;
        if (!AddSize(m_semantic_bytes, admitted_bytes))
        {
            RejectRetained();
            return NULL;
        }
        const std::size_t remaining_bytes =
            m_limits.max_retained_bytes -
                std::min(
                    admitted_bytes,
                    m_limits.max_retained_bytes);
        const std::size_t remaining_diagnostics =
            m_limits.max_diagnostics -
                std::min(
                    m_result.diagnostics.size(),
                    m_limits.max_diagnostics);
        JBeamNormalizeLimits normalize_limits;
        normalize_limits.max_retained_bytes = remaining_bytes;
        // Every normalized work item retains or examines source state which
        // must fit the same remaining structural admission budget. This keeps
        // a tiny structural budget from silently invoking the normalizer's
        // unrelated 512 MiB / 12M-unit defaults.
        normalize_limits.max_work_units = std::min(
            normalize_limits.max_work_units,
            remaining_bytes);
        normalize_limits.max_diagnostics =
            std::max<std::size_t>(1U, remaining_diagnostics);
        normalized = NormalizeJBeamTables(
            normalization_input,
            normalize_limits);

        bool normalization_error = false;
        for (std::size_t i = 0U;
             i < normalized.diagnostics.size();
             ++i)
        {
            const JBeamDiagnostic& source =
                normalized.diagnostics[i];
            const JBeamStructuralProvenance provenance =
                ProvenanceWithSpan(
                    sections.part.provenance,
                    source.span);
            if (m_resource_limit)
            {
                return NULL;
            }
            std::string detail =
                std::string("Normalizer ") +
                JBeamDiagnosticCodeToString(source.code) +
                ": " + source.message;
            if (source.code ==
                JBeamDiagnosticCode::
                    NORMALIZE_RETAINED_BYTES_LIMIT)
            {
                Push(
                    JBeamStructuralDiagnosticCode::
                        RETAINED_BYTE_LIMIT,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    provenance,
                    name,
                    0U,
                    std::string(),
                    detail);
                m_resource_limit = true;
                return NULL;
            }
            if (source.code ==
                JBeamDiagnosticCode::DIAGNOSTIC_LIMIT)
            {
                Push(
                    JBeamStructuralDiagnosticCode::DIAGNOSTIC_LIMIT,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    provenance,
                    name,
                    0U,
                    std::string(),
                    detail);
                m_resource_limit = true;
                return NULL;
            }
            const bool is_error =
                source.severity == JBeamDiagnosticSeverity::ERROR_SEVERITY;
            Push(
                is_error
                    ? JBeamStructuralDiagnosticCode::
                        NORMALIZATION_ERROR
                    : JBeamStructuralDiagnosticCode::
                        NORMALIZATION_WARNING,
                is_error
                    ? JBeamStructuralSeverity::ERROR_SEVERITY
                    : JBeamStructuralSeverity::WARNING,
                provenance,
                name,
                0U,
                std::string(),
                detail);
            normalization_error =
                normalization_error || is_error;
            if (m_resource_limit)
            {
                return NULL;
            }
        }
        if (normalization_error || !normalized.IsValid())
        {
            m_resource_limit = true;
            return NULL;
        }
        const JBeamNormalizedTable* table = NULL;
        for (std::size_t i = 0U;
             i < normalized.tables.size();
             ++i)
        {
            if (normalized.tables[i].path == "$")
            {
                table = &normalized.tables[i];
                break;
            }
        }
        if (table == NULL)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_SECTION,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                sections.part.provenance,
                name,
                0U,
                std::string(),
                "Structural table normalization failed");
            return NULL;
        }
        std::set<std::string> headers;
        for (std::size_t i = 0U; i < table->columns.size(); ++i)
        {
            if (!headers.insert(table->columns[i].name).second)
            {
                Push(
                    JBeamStructuralDiagnosticCode::
                        DUPLICATE_TABLE_HEADER,
                    JBeamStructuralSeverity::ERROR_SEVERITY,
                    sections.part.provenance,
                    name,
                    0U,
                    table->columns[i].name,
                    "Duplicate table header is ambiguous for "
                    "structural conversion");
            }
        }
        return table;
    }

    void PreserveUnsupportedAssignments(
        const JBeamNormalizedTable& table,
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index)
    {
        for (std::size_t i = 0U;
             i < row.positional_assignments.size();
             ++i)
        {
            if (m_resource_limit)
            {
                return;
            }
            const JBeamFieldAssignment& assignment =
                row.positional_assignments[i];
            if (!IsImplementedField(section, assignment.name))
            {
                const bool known = IsKnownUnsupportedField(
                    section, assignment.name);
                PushPreserved(
                    known
                        ? JBeamStructuralDiagnosticCode::UNSUPPORTED_FIELD
                        : JBeamStructuralDiagnosticCode::UNKNOWN_FIELD,
                    ProvenanceWithSpan(provenance, assignment.span),
                    section,
                    row_index,
                    assignment.name,
                    known
                        ? "Field is recognized but not simulated by J2"
                        : "Unknown field is preserved but ignored",
                    assignment.value);
                if (m_resource_limit)
                {
                    return;
                }
            }
        }
        for (std::size_t i = 0U;
             i < row.row_local_assignments.size();
             ++i)
        {
            if (m_resource_limit)
            {
                return;
            }
            const JBeamFieldAssignment& assignment =
                row.row_local_assignments[i];
            if (!IsImplementedField(section, assignment.name))
            {
                const bool known = IsKnownUnsupportedField(
                    section, assignment.name);
                PushPreserved(
                    known
                        ? JBeamStructuralDiagnosticCode::UNSUPPORTED_FIELD
                        : JBeamStructuralDiagnosticCode::UNKNOWN_FIELD,
                    ProvenanceWithSpan(provenance, assignment.span),
                    section,
                    row_index,
                    assignment.name,
                    known
                        ? "Field is recognized but not simulated by J2"
                        : "Unknown field is preserved but ignored",
                    assignment.value);
                if (m_resource_limit)
                {
                    return;
                }
            }
        }
        std::size_t positional_count =
            row.raw_row.array_values.size();
        if (positional_count > table.columns.size() &&
            row.raw_row.array_values[positional_count - 1U].type ==
                JBeamValueType::OBJECT)
        {
            --positional_count;
        }
        for (std::size_t i = table.columns.size();
             i < positional_count;
             ++i)
        {
            if (m_resource_limit)
            {
                return;
            }
            std::ostringstream field_name;
            field_name.imbue(std::locale::classic());
            field_name << "<extra-cell-" << i << '>';
            const JBeamValue& value = row.raw_row.array_values[i];
            PushPreservedCopy(
                JBeamStructuralDiagnosticCode::UNKNOWN_FIELD,
                ProvenanceWithSpan(provenance, value.span),
                section,
                row_index,
                field_name.str(),
                "Extra positional cell is preserved but ignored",
                value);
            if (m_resource_limit)
            {
                return;
            }
        }
    }

    bool ZeroRestRowHasOnlyImplementedFields(
        const JBeamNormalizedTable& table,
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance)
    {
        const auto charge = [&]() {
            return ChargeSemanticWork(1U, provenance, "beams", "zero-rest",
                "Zero-rest effective-field admission exceeded its work limit");
        };
        const JBeamFieldAssignment* break_group = FindEffectiveJBeamField(row, "breakGroup");
        if (break_group != NULL && (!charge() || !break_group->value ||
            break_group->value->type != JBeamValueType::STRING))
            return false;
                // `breakGroup` is consumed by this role: a breakable zero-rest spring
        // detaches its group, and a group member breaks it, exactly as an
        // ordinary beam's group does. `deformGroup` stays outside: a rest
        // length of zero has no deformation ratio to trigger on.
        //
        // The progressive (velocity-split) damping fields stay outside too,
        // and they are listed explicitly rather than inherited from
        // `IsImplementedField` because ONE FIELD CAN HAVE TWO CONSUMERS THAT
        // DISAGREE. They are implemented on the ordinary beam path, where
        // ProgressiveBeamDamping selects the coefficient by axial speed;
        // JBeamZeroRestSpring's law takes a single damping coefficient and
        // never consults a split. Letting this role inherit the ordinary
        // path's answer would admit a zero-rest row that then silently ran at
        // its slow rate - and would change which beams EXIST, since a row
        // this predicate rejects is refused outright.
        const auto consumed = [](const std::string& name) {
            if (name == "beamDampFast" ||
                name == "beamDampRebound" ||
                name == "beamDampReboundFast" ||
                name == "beamDampVelocitySplit" ||
                // A zero-rest spring has no length to ramp; a row that asks
                // for one is not the role, and is refused rather than run.
                name == "beamPrecompressionTime")
            {
                return false;
            }
            return IsImplementedField("beams", name) || name == "breakGroup";
        };
        for (const JBeamFieldAssignment& field : row.positional_assignments)
        {
            if (!charge() || !consumed(field.name))
                return false;
        }
        for (const JBeamFieldAssignment& field : row.row_local_assignments)
        {
            if (!charge() || !consumed(field.name))
                return false;
        }
        if (row.inherited_assignment_count != 0U)
        {
            if (!row.inherited_assignment_storage ||
                !row.inherited_assignment_index ||
                row.inherited_assignment_count > row.inherited_assignment_storage->size())
                return false;
            // The shared index also contains FUTURE defaults. Only names
            // whose first assignment belongs to this row's active prefix
            // constrain it; a later modifier must not contaminate an earlier
            // supported spring. Lookup never scans the whole defaults history.
            for (const JBeamInheritedFieldIndex& field :
                    row.inherited_assignment_index->fields)
            {
                if (!charge()) return false;
                if (!consumed(field.name) &&
                    !field.assignment_indices.empty() &&
                    field.assignment_indices.front() < row.inherited_assignment_count)
                    return false;
            }
        }
        std::size_t positional_count = row.raw_row.array_values.size();
        if (positional_count > table.columns.size() &&
            row.raw_row.array_values.back().type == JBeamValueType::OBJECT)
            --positional_count;
        return positional_count <= table.columns.size();
    }

    void PreserveUnsupportedDefaults(
        const JBeamNormalizedTable& table,
        const JBeamStructuralProvenance& provenance,
        const std::string& section)
    {
        for (std::size_t entry_index = 0U;
             entry_index < table.entries.size();
             ++entry_index)
        {
            if (m_resource_limit)
            {
                return;
            }
            const JBeamNormalizedTableEntry& entry =
                table.entries[entry_index];
            if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DEFAULT_MODIFIER ||
                entry.raw_value.type != JBeamValueType::OBJECT)
            {
                if (m_resource_limit)
                {
                    return;
                }
                continue;
            }
            for (std::size_t field_index = 0U;
                 field_index < entry.raw_value.object_fields.size();
                 ++field_index)
            {
                const JBeamObjectField& field =
                    entry.raw_value.object_fields[field_index];
                if (!IsImplementedField(section, field.key))
                {
                    const bool known =
                        IsKnownUnsupportedField(section, field.key);
                    PushPreserved(
                        known
                            ? JBeamStructuralDiagnosticCode::
                                UNSUPPORTED_FIELD
                            : JBeamStructuralDiagnosticCode::UNKNOWN_FIELD,
                        ProvenanceWithSpan(
                            provenance,
                            field.value
                                ? field.value->span
                                : field.key_span),
                        section,
                        entry_index,
                        field.key,
                        known
                            ? "Default is recognized but not simulated "
                                "by J2"
                            : "Unknown default is preserved but ignored",
                        field.value);
                    if (m_resource_limit)
                    {
                        return;
                    }
                }
            }
        }
    }

    const std::vector<JBeamExpressionVariable>& VariablesFor(
        const JBeamStructuralProvenance& provenance) const
    {
        static const std::vector<JBeamExpressionVariable> empty;
        if (!provenance.part ||
            provenance.PartPreorderIndex() >= m_parts.size())
        {
            return empty;
        }
        const std::shared_ptr<std::vector<JBeamExpressionVariable> >&
            variables =
                m_parts[provenance.PartPreorderIndex()].variables;
        return variables ? *variables : empty;
    }

    /// `refNodes` carries the four axis columns BeamNG documents -- ref,
    /// back, left, up -- and MAY also carry leftCorner/rightCorner. Vehicles
    /// that ship only the four are ordinary content, not malformed: the
    /// Bell 407 and the J35D Gripen are two, and requiring the pair refused
    /// them outright with `missing-required-field@...:refNodes#0.leftCorner`.
    /// The pair is validation-only downstream -- it feeds the front-corner
    /// alignment check and nothing that simulates -- so its absence costs
    /// that one check and nothing else.
    bool HasRefCornerColumns(const JBeamNormalizedDataRow& row) const
    {
        return FindEffectiveJBeamField(row, "leftCorner:") != NULL ||
            FindEffectiveJBeamField(row, "leftCorner") != NULL ||
            FindEffectiveJBeamField(row, "rightCorner:") != NULL ||
            FindEffectiveJBeamField(row, "rightCorner") != NULL;
    }

    bool ReadRequiredString(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& primary,
        const std::string& alias,
        std::string& output)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* first =
            FindEffectiveJBeamField(row, primary);
        const JBeamFieldAssignment* second = alias.empty()
            ? NULL
            : FindEffectiveJBeamField(row, alias);
        if (first != NULL && second != NULL)
        {
            Push(
                JBeamStructuralDiagnosticCode::
                    AMBIGUOUS_REQUIRED_FIELD,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                primary,
                "Both canonical and alias fields are assigned");
            return false;
        }
        const JBeamFieldAssignment* assignment =
            first != NULL ? first : second;
        if (assignment == NULL || !assignment->value)
        {
            Push(
                JBeamStructuralDiagnosticCode::MISSING_REQUIRED_FIELD,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                primary,
                "Required string field is missing");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        JBeamExpressionValue resolved;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                section,
                row_index,
                primary,
                resolved))
        {
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::STRING ||
            resolved.string_value.empty())
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                primary,
                "Required identifier expression must resolve to a "
                "non-empty string");
            return false;
        }
        output = resolved.string_value;
        return true;
    }

    bool ReadRequiredNumber(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& name,
        double& output,
        bool* nil_means_absent = NULL,
        bool* blank_means_absent = NULL,
        bool* undeclared_means_absent = NULL)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, name);
        if (assignment == NULL || !assignment->value)
        {
            Push(
                JBeamStructuralDiagnosticCode::MISSING_REQUIRED_FIELD,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                name,
                "Required numeric field is missing");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        if ((name == "posX" || name == "posY" || name == "posZ" ||
             name == "frictionCoef" || name == "slidingFrictionCoef") &&
            assignment->value->type == JBeamValueType::NUMBER &&
            IsBinary64SubnormalMagnitude(
                assignment->value->number_value))
        {
            std::memcpy(
                &output, &assignment->value->number_value,
                sizeof(output));
            return true;
        }
        JBeamExpressionValue resolved;
        bool undeclared_only = false;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                section,
                row_index,
                name,
                resolved,
                undeclared_means_absent != NULL ? &undeclared_only : NULL))
        {
            // Only an opted-in optional field can reach this; the warning
            // naming the missing variable was already recorded.
            if (undeclared_means_absent != NULL && undeclared_only)
            {
                *undeclared_means_absent = true;
                return true;
            }
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::NUMBER)
        {
            // Real-world content uses the documented existence-check
            // idiom ("$=$maybe_defined") in optional numeric fields: a
            // nil result means the field is unauthored and the default
            // applies. Required fields still refuse a nil.
            if (nil_means_absent != NULL &&
                resolved.type == JBeamExpressionValueType::NIL_VALUE)
            {
                *nil_means_absent = true;
                Push(
                    JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                    JBeamStructuralSeverity::WARNING,
                    field_provenance,
                    section,
                    row_index,
                    name,
                    "Optional field expression resolved to nil; the "
                    "authored default applies");
                return true;
            }
            // The same existence-check idiom in its other authored spelling:
            // `case(<cond>, <value>, '')` selects the empty string for the
            // branch on which the field is meant to be unauthored. Only the
            // fields that opt in accept it, and only when it is EMPTY - any
            // other string is still a refusal.
            if (blank_means_absent != NULL &&
                resolved.type == JBeamExpressionValueType::STRING &&
                resolved.string_value.empty())
            {
                *blank_means_absent = true;
                // Reported once per authored blank, not once per row that
                // inherits it. A modifier reset such as
                // {"deformLimitExpansion":""} applies to every row after it,
                // so per-row copies of one warning grow with the table: the
                // Frix MF1's 89 authored resets made about 2,800, filled
                // the diagnostic budget, and the budget's overflow (an error,
                // since it could hide one) refused the whole vehicle. Every
                // row still reads the field as unauthored.
                if (m_reported_blank_resets.insert(std::make_tuple(
                        field_provenance.PartPreorderIndex(),
                        field_provenance.SourceName(),
                        field_provenance.begin.byte_offset,
                        field_provenance.end.byte_offset,
                        name)).second)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                        JBeamStructuralSeverity::WARNING,
                        field_provenance,
                        section,
                        row_index,
                        name,
                        "Optional field expression resolved to an empty "
                        "string; the field is treated as unauthored");
                }
                return true;
            }
            // A quoted number is still that number. Real BeamNG content mixes
            // the spellings in one table -- the F-22A writes
            // `"beamDeform":"400000"` beside a plain `25000000` -- and BeamNG
            // reads both the same. The coercion is whole-string and finite, so
            // a string that is only partly a number is still a refusal.
            double coerced = 0.0;
            if (resolved.type == JBeamExpressionValueType::STRING &&
                ParseWholeNumber(resolved.string_value, coerced))
            {
                output = coerced;
                return true;
            }
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                name,
                "Field expression must resolve to a number");
            return false;
        }
        output = resolved.number_value;
        return true;
    }

    bool ReadOptionalNumber(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& name,
        bool& has_value,
        double& output,
        bool allow_blank = false)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, name);
        has_value = assignment != NULL;
        if (assignment == NULL)
        {
            return true;
        }
        bool nil_means_absent = false;
        bool blank_means_absent = false;
        bool undeclared_means_absent = false;
        if (!ReadRequiredNumber(
                row,
                provenance,
                section,
                row_index,
                name,
                output,
                &nil_means_absent,
                allow_blank ? &blank_means_absent : NULL,
                &undeclared_means_absent))
        {
            return false;
        }
        if (nil_means_absent || blank_means_absent ||
            undeclared_means_absent)
        {
            has_value = false;
        }
        return true;
    }

    /// True when `text` is a complete, finite number and nothing else.
    /// Deliberately strict: the entire string must be consumed, so "12abc",
    /// " 12" and an out-of-range exponent are not numbers.
    static bool ParseWholeNumber(const std::string& text, double& output)
    {
        if (text.empty())
        {
            return false;
        }
        // Reject "nan", "inf" and their variants LEXICALLY rather than by
        // testing the parsed double: this translation unit is built with
        // fast-math, under which comparing against infinity or self-comparing
        // for NaN is undefined and folds away. Only digits, a sign, a decimal
        // point and an exponent marker may appear, which no spelling of a
        // non-finite value can satisfy.
        for (std::size_t i = 0U; i < text.size(); ++i)
        {
            const char c = text[i];
            const bool numeric = (c >= '0' && c <= '9') || c == '+' ||
                c == '-' || c == '.' || c == 'e' || c == 'E';
            if (!numeric)
            {
                return false;
            }
        }
        const char* const begin = text.c_str();
        char* end = NULL;
        errno = 0;
        const double value = std::strtod(begin, &end);
        // ERANGE covers both an overflow to infinity and an underflow.
        if (end != begin + text.size() || errno == ERANGE)
        {
            return false;
        }
        output = value;
        return true;
    }

    bool ReadOptionalBeamLimit(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        std::size_t row_index,
        const std::string& name,
        bool& has_value,
        bool& unbounded,
        double& output)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, name);
        has_value = assignment != NULL;
        unbounded = false;
        output = 0.0;
        if (assignment == NULL)
        {
            return true;
        }
        if (assignment->value &&
            assignment->value->type == JBeamValueType::STRING &&
            assignment->value->scalar_text == "FLT_MAX")
        {
            unbounded = true;
            return true;
        }
        return ReadRequiredNumber(
            row, provenance, "beams", row_index, name, output);
    }

    /// Retains an authored string field that nothing consumes yet. Absent
    /// leaves `output` empty, a string is kept verbatim, and anything else is
    /// a WARNING rather than a refusal: the vehicle is not rejected over a
    /// field no consumer reads, which is the failure that once refused a
    /// whole car over a blank modifier reset.
    ///
    /// Retention is deliberately NOT simulation. These values are reported on
    /// the import receipt and stay OUT of the canonical digest, because a
    /// field nothing reads cannot make two vehicles different simulations.
    bool ReadOptionalString(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& name,
        std::string& output)
    {
        output.clear();
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, name);
        if (assignment == NULL || !assignment->value)
        {
            return true;
        }
        if (assignment->value->type != JBeamValueType::STRING)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::WARNING,
                ProvenanceWithSpan(provenance, assignment->value->span),
                section,
                row_index,
                name,
                name + " must be a string; the value is not retained");
            return true;
        }
        output = assignment->value->scalar_text;
        return true;
    }

    /// `isExhaust`: the shipped cars author the engine group as a string
    /// (`"mainEngine"`); a Boolean `true` names that default group and
    /// `false` clears it. Any other type is a WARNING and not retained, as
    /// for every retained string: the field only places exhaust emitters.
    bool ReadOptionalExhaustEngine(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        std::string& output)
    {
        output.clear();
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, "isExhaust");
        if (assignment == NULL || !assignment->value)
        {
            return true;
        }
        if (assignment->value->type == JBeamValueType::STRING)
        {
            output = assignment->value->scalar_text;
            return true;
        }
        if (assignment->value->type == JBeamValueType::BOOLEAN)
        {
            if (assignment->value->boolean_value)
            {
                output = "mainEngine";
            }
            return true;
        }
        Push(
            JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
            JBeamStructuralSeverity::WARNING,
            ProvenanceWithSpan(provenance, assignment->value->span),
            section,
            row_index,
            "isExhaust",
            "isExhaust must be an engine group name or a Boolean; the value is not retained");
        return true;
    }

    /// Reads an optional Boolean field. A value that is not a Boolean
    /// REJECTS the vehicle, which is correct for every field a consumer
    /// reads: admitting a value we cannot read would be silently accepting
    /// bad data.
    ///
    /// `unconsumed_type_refused` is the opt-in escape for the retained-but-
    /// unconsumed family, and mirrors what `ReadOptionalString` already does
    /// for the string members of that same family. When it is supplied, a
    /// non-Boolean value is a WARNING instead: the documented default
    /// applies, the flag records that a value was refused, and the import
    /// receipt names it. The severity changes; the diagnostic does not.
    ///
    /// Only a field NOTHING consumes may pass it. The justification is not
    /// that the value is unimportant but that it is provably unread -- these
    /// fields are excluded from the canonical digest, which a test pins
    /// byte-for-byte, so no simulation can depend on the value we dropped.
    /// Every consumed call site passes NULL and keeps today's exact
    /// behaviour.
    bool ReadOptionalBoolean(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        const std::string& name,
        bool default_value,
        bool& output,
        bool* unconsumed_type_refused = NULL)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, name);
        output = default_value;
        if (assignment == NULL)
        {
            return true;
        }
        if (!assignment->value)
        {
            if (unconsumed_type_refused != NULL)
            {
                *unconsumed_type_refused = true;
                Push(
                    JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                    JBeamStructuralSeverity::WARNING,
                    provenance,
                    section,
                    row_index,
                    name,
                    name + " must be a Boolean; the value is not retained");
                return true;
            }
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                name,
                "Field expression must resolve to a Boolean");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        JBeamExpressionValue resolved;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                section,
                row_index,
                name,
                resolved))
        {
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::BOOLEAN)
        {
            if (unconsumed_type_refused != NULL)
            {
                *unconsumed_type_refused = true;
                Push(
                    JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                    JBeamStructuralSeverity::WARNING,
                    field_provenance,
                    section,
                    row_index,
                    name,
                    name + " must be a Boolean; the value is not retained");
                return true;
            }
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                name,
                "Field expression must resolve to a Boolean");
            return false;
        }
        output = resolved.boolean_value;
        return true;
    }

    bool ReadTriangleType(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        JBeamStructuralTriangleType& output)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, "triangleType");
        output = JBeamStructuralTriangleType::NORMALTYPE;
        if (assignment == NULL)
        {
            return true;
        }
        if (!assignment->value)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                "triangleType",
                "triangleType must resolve to a non-empty string");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        JBeamExpressionValue resolved;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                section,
                row_index,
                "triangleType",
                resolved))
        {
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::STRING ||
            resolved.string_value.empty())
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                "triangleType",
                "triangleType must resolve to a non-empty string");
            return false;
        }
        std::string normalized = resolved.string_value;
        if (!normalized.empty() && normalized[0] == '|')
        {
            normalized.erase(0U, 1U);
        }
        if (normalized == "NORMALTYPE")
        {
            output = JBeamStructuralTriangleType::NORMALTYPE;
            return true;
        }
        if (normalized == "NONCOLLIDABLE")
        {
            output = JBeamStructuralTriangleType::NONCOLLIDABLE;
            return true;
        }
        Push(
            JBeamStructuralDiagnosticCode::UNSUPPORTED_TRIANGLE_TYPE,
            JBeamStructuralSeverity::ERROR_SEVERITY,
            field_provenance,
            section,
            row_index,
            "triangleType",
            "Only NORMALTYPE and NONCOLLIDABLE are admitted by the "
            "documented J2 triangle subset");
        return false;
    }

    /// Reads `openbeamBuoyancy`, which is OpenBeam's own key and not a
    /// BeamNG one: the format has no buoyancy, so an absent key -- every
    /// BeamNG document -- leaves the triangle displacing nothing, and only
    /// a vessel authored for OpenBeam ever sets it. A value that is not one
    /// of the four documented spellings is refused by name rather than
    /// silently read as NONE, because a misspelled marker on a hull is a
    /// ship that sinks with no receipt saying why.
    bool ReadTriangleBuoyancy(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section,
        std::size_t row_index,
        JBeamStructuralTriangleBuoyancy& output)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, "openbeamBuoyancy");
        output = JBeamStructuralTriangleBuoyancy::NONE;
        if (assignment == NULL)
        {
            return true;
        }
        if (!assignment->value)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance,
                section,
                row_index,
                "openbeamBuoyancy",
                "openbeamBuoyancy must resolve to a non-empty string");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        JBeamExpressionValue resolved;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                section,
                row_index,
                "openbeamBuoyancy",
                resolved))
        {
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::STRING ||
            resolved.string_value.empty())
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                "openbeamBuoyancy",
                "openbeamBuoyancy must resolve to a non-empty string");
            return false;
        }
        std::string normalized = resolved.string_value;
        if (!normalized.empty() && normalized[0] == '|')
        {
            normalized.erase(0U, 1U);
        }
        if (normalized == "NONE")
        {
            output = JBeamStructuralTriangleBuoyancy::NONE;
            return true;
        }
        if (normalized == "BUOYANT")
        {
            output = JBeamStructuralTriangleBuoyancy::BUOYANT;
            return true;
        }
        if (normalized == "DRAG_ONLY")
        {
            output = JBeamStructuralTriangleBuoyancy::DRAG_ONLY;
            return true;
        }
        if (normalized == "NO_DRAG")
        {
            output = JBeamStructuralTriangleBuoyancy::NO_DRAG;
            return true;
        }
        Push(
            JBeamStructuralDiagnosticCode::UNSUPPORTED_TRIANGLE_TYPE,
            JBeamStructuralSeverity::ERROR_SEVERITY,
            field_provenance,
            section,
            row_index,
            "openbeamBuoyancy",
            "openbeamBuoyancy admits only NONE, BUOYANT, DRAG_ONLY and "
            "NO_DRAG");
        return false;
    }

    /// Reads `breakGroup`. Absent, empty, or an unmodelled table form all
    /// leave the beam ungrouped; only a non-empty string joins a group.
    /// `breakGroup` and `deformGroup` have identical shape: a single group
    /// name, an empty string that is an authored RESET rather than a group,
    /// and a documented table form that is refused by name rather than
    /// flattened. One reader serves both so the two cannot drift apart.
    bool ReadGroupName(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        std::size_t row_index,
        const char* section,
        const char* field_name,
        std::string& output)
    {
        output.clear();
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, field_name);
        if (assignment == NULL || !assignment->value)
        {
            return true;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->value->span);
        if (assignment->value->type == JBeamValueType::ARRAY)
        {
            // Documented, present in the corpus, and deliberately not
            // modelled. Naming it keeps the gap visible instead of letting a
            // multi-group beam look ungrouped for no stated reason.
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::WARNING,
                field_provenance,
                section,
                row_index,
                field_name,
                std::string(field_name) +
                    " table form is not modelled; the row is left "
                    "ungrouped");
            return true;
        }
        if (assignment->value->type != JBeamValueType::STRING)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                section,
                row_index,
                field_name,
                std::string(field_name) + " must be a string");
            return false;
        }
        // An empty string is an authored RESET, not a group named "".
        output = assignment->value->scalar_text;
        return true;
    }

    bool ReadBeamType(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        std::size_t row_index,
        std::string& output,
        JBeamStructuralBeamStatus& status)
    {
        if (m_resource_limit)
        {
            return false;
        }
        const JBeamFieldAssignment* assignment =
            FindEffectiveJBeamField(row, "beamType");
        output = "NORMAL";
        status = JBeamStructuralBeamStatus::ENABLED;
        if (assignment == NULL)
        {
            return true;
        }
        if (!assignment->value)
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                assignment->value
                    ? ProvenanceWithSpan(
                        provenance, assignment->value->span)
                    : provenance,
                "beams",
                row_index,
                "beamType",
                "beamType must be a non-empty string");
            return false;
        }
        const JBeamStructuralProvenance field_provenance =
            ProvenanceWithSpan(provenance, assignment->span);
        JBeamExpressionValue resolved;
        if (!ResolveScalarValue(
                *assignment->value,
                VariablesFor(provenance),
                field_provenance,
                "beams",
                row_index,
                "beamType",
                resolved))
        {
            return false;
        }
        if (resolved.type != JBeamExpressionValueType::STRING ||
            resolved.string_value.empty())
        {
            Push(
                JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                field_provenance,
                "beams",
                row_index,
                "beamType",
                "beamType expression must resolve to a non-empty "
                "string");
            return false;
        }
        output = resolved.string_value;
        std::string normalized = output;
        if (!normalized.empty() && normalized[0] == '|')
        {
            normalized.erase(0U, 1U);
        }
        if (normalized == "NORMAL")
        {
            output = "NORMAL";
            return true;
        }
        if (normalized == "SUPPORT")
        {
            output = "SUPPORT";
            return true;
        }
        // BOUNDED is an ordinary axial spring-damper carrying a SECOND
        // constant pair blended in past an authored travel threshold, which
        // BoundedLimitBeam.h implements exactly. It is admitted on the same
        // footing as NORMAL and SUPPORT; the remaining special types stay
        // preserved-but-disabled.
        if (normalized == "BOUNDED")
        {
            output = "BOUNDED";
            return true;
        }
        status =
            JBeamStructuralBeamStatus::
                PRESERVED_DISABLED_SPECIAL_TYPE;
        PushPreserved(
            JBeamStructuralDiagnosticCode::
                SPECIAL_BEAM_TYPE_DISABLED,
            ProvenanceWithSpan(provenance, assignment->span),
            "beams",
            row_index,
            "beamType",
            "Only NORMAL and SUPPORT beams are enabled by the J2 subset",
            assignment->value);
        return true;
    }

    bool ReadNodeFriction(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        std::size_t row_index,
        const std::string& name,
        double default_value,
        bool& authored,
        double& output)
    {
        const JBeamFieldAssignment* assignment = FindEffectiveJBeamField(row, name);
        authored = assignment != NULL;
        output = default_value;
        if (!authored) return true;
        // ResolveScalarValue intentionally returns false for aggregate values
        // without a diagnostic. A malformed friction field must still reject
        // the whole IR, even on a node no beam or reference frame uses.
        if (assignment->value &&
            (assignment->value->type == JBeamValueType::ARRAY ||
             assignment->value->type == JBeamValueType::OBJECT))
        {
            Push(JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                ProvenanceWithSpan(provenance, assignment->span),
                "nodes", row_index, name,
                "Authored node friction coefficient must be a scalar number");
            return false;
        }
        // Unlike optional beam limit resets, an authored nil, empty string,
        // unknown variable or failed expression is not a friction default.
        if (!ReadRequiredNumber(row, provenance, "nodes", row_index,
                name, output))
        {
            return false;
        }
        if (!IsFiniteNonNegativeDouble(output))
        {
            Push(JBeamStructuralDiagnosticCode::INVALID_NODE_FRICTION,
                JBeamStructuralSeverity::ERROR_SEVERITY,
                provenance, "nodes", row_index, name,
                "Authored node friction coefficient must be finite and nonnegative");
            return false;
        }
        return true;
    }

    void ProcessNodes()
    {
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            JBeamNormalizeResult normalized;
            const JBeamNormalizedTable* table = NormalizeSection(
                m_sections[part_index], "nodes", normalized);
            if (m_resource_limit)
            {
                return;
            }
            if (table == NULL)
            {
                continue;
            }
            PreserveUnsupportedDefaults(
                *table,
                m_sections[part_index].part.provenance,
                "nodes");
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t entry_index = 0U;
                 entry_index < table->entries.size();
                 ++entry_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const JBeamNormalizedTableEntry& entry =
                    table->entries[entry_index];
                if (entry.kind ==
                    JBeamNormalizedTableEntryKind::INVALID_ENTRY)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "nodes",
                        entry_index,
                        std::string(),
                        "Node table entry is not a data row or default");
                    continue;
                }
                if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DATA_ROW)
                {
                    continue;
                }
                PreserveUnsupportedAssignments(
                    *table,
                    entry.data_row,
                    m_sections[part_index].part.provenance,
                    "nodes",
                    entry_index);
                if (m_resource_limit)
                {
                    return;
                }
                if (m_result.nodes.size() >= m_limits.max_nodes)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::NODE_LIMIT,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "nodes",
                        entry_index,
                        std::string(),
                        "Node table exceeds the configured node limit");
                    m_resource_limit = true;
                    return;
                }
                JBeamStructuralNode node;
                node.x = 0.0;
                node.y = 0.0;
                node.z = 0.0;
                node.node_weight = 25.0;
                node.node_weight_authored = false;
                node.friction_coef = 1.0;
                node.friction_coef_authored = false;
                node.sliding_friction_coef = 1.0;
                node.sliding_friction_coef_authored = false;
                node.collision = true;
                node.self_collision = false;
                node.static_collision = true;
                node.fixed = false;
                node.provenance =
                    ProvenanceWithSpan(
                        m_sections[part_index].part.provenance,
                        entry.data_row.span);
                bool valid = ReadRequiredString(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "id",
                    std::string(),
                    node.id);
                valid = ReadRequiredNumber(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "posX",
                    node.x) && valid;
                valid = ReadRequiredNumber(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "posY",
                    node.y) && valid;
                valid = ReadRequiredNumber(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "posZ",
                    node.z) && valid;
                if (valid)
                {
                    ApplySlotNodeOffset(
                        m_sections[part_index].part, node);
                }
                const JBeamFieldAssignment* weight =
                    FindEffectiveJBeamField(
                        entry.data_row, "nodeWeight");
                if (weight != NULL)
                {
                    node.node_weight_authored = true;
                    valid = ReadRequiredNumber(
                        entry.data_row,
                        node.provenance,
                        "nodes",
                        entry_index,
                        "nodeWeight",
                        node.node_weight) && valid;
                    if (valid && !(node.node_weight > 0.0))
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::
                                INVALID_NODE_WEIGHT,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            node.provenance,
                            "nodes",
                            entry_index,
                            "nodeWeight",
                            "Authored nodeWeight must be positive");
                        valid = false;
                    }
                }
                valid = ReadNodeFriction(entry.data_row, node.provenance,
                    entry_index, "frictionCoef", 1.0,
                    node.friction_coef_authored, node.friction_coef) && valid;
                valid = ReadNodeFriction(entry.data_row, node.provenance,
                    entry_index, "slidingFrictionCoef", node.friction_coef,
                    node.sliding_friction_coef_authored,
                    node.sliding_friction_coef) && valid;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "collision",
                    true,
                    node.collision) && valid;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "selfCollision",
                    false,
                    node.self_collision) && valid;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "staticCollision",
                    true,
                    node.static_collision) && valid;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "fixed",
                    false,
                    node.fixed) && valid;
                // Retained, never consumed: see JBeamStructuralNode.
                valid = ReadOptionalString(
                    entry.data_row,
                    node.provenance,
                    "nodes",
                    entry_index,
                    "nodeMaterial",
                    node.node_material) && valid;
                valid = ReadOptionalExhaustEngine(
                    entry.data_row, node.provenance, "nodes",
                    entry_index, node.exhaust_engine) && valid;
                const std::map<std::string, std::size_t>::const_iterator
                    redefined = node.id.empty()
                        ? m_node_indices.end()
                        : m_node_indices.find(node.id);
                if (redefined != m_node_indices.end())
                {
                    // Real-world part combinations redefine node ids that
                    // an earlier part already published: a wheel part that
                    // mounts its own hub nodes at the slot's nodeOffset over
                    // the ones its hub part defined, a glass part repeating
                    // the body's pillar nodes. BeamNG appends every chosen
                    // part's rows to its parent's table in slot order - the
                    // resolved preorder - and then keys the merged `nodes`
                    // table by id, so a later row REPLACES the earlier one:
                    // position, weight, groups, every option
                    // (lua/common/jbeam/tableSchema.lua,
                    // processJbeamTableRow: `newList[newID] = newRow`).
                    // The last definition therefore wins here too. It keeps
                    // the index the first definition published, so no other
                    // node moves, and the superseded row stays visible as a
                    // warning with this row's exact source span.
                    Push(
                        JBeamStructuralDiagnosticCode::DUPLICATE_NODE_ID,
                        JBeamStructuralSeverity::WARNING,
                        node.provenance,
                        "nodes",
                        entry_index,
                        "id",
                        "Node identifier is already defined; this later "
                        "definition in resolved preorder replaces the "
                        "earlier one, as BeamNG's id-keyed node table does");
                }
                if (m_resource_limit)
                {
                    return;
                }
                if (valid && redefined != m_node_indices.end())
                {
                    m_result.nodes[redefined->second] = node;
                }
                else if (valid)
                {
                    if (!ReserveRetained(node.id.size()))
                    {
                        return;
                    }
                    const std::size_t index = m_result.nodes.size();
                    m_node_indices.insert(
                        std::make_pair(node.id, index));
                    m_result.nodes.push_back(node);
                }
            }
        }
    }

    bool ResolveNode(
        const std::string& id,
        std::size_t& index) const
    {
        const std::map<std::string, std::size_t>::const_iterator found =
            m_node_indices.find(id);
        if (found == m_node_indices.end())
        {
            index = INVALID_INDEX;
            return false;
        }
        index = found->second;
        return true;
    }

    bool AreSamePosition(
        std::size_t first,
        std::size_t second) const
    {
        const JBeamStructuralNode& a = m_result.nodes[first];
        const JBeamStructuralNode& b = m_result.nodes[second];
        return AreJBeamNodePositionsExactlyCoincident(a, b);
    }

    double SafeAbsoluteDifference(
        double first,
        double second) const
    {
        if ((first < 0.0) == (second < 0.0))
        {
            // Equal-sign finite operands cannot overflow subtraction.
            return Abs(first - second);
        }
        const double first_abs = Abs(first);
        const double second_abs = Abs(second);
        const double maximum = std::numeric_limits<double>::max();
        if (first_abs > maximum - second_abs)
        {
            return maximum;
        }
        return first_abs + second_abs;
    }

    bool TryScaledDifference(
        double point,
        double origin,
        double scale,
        double& difference) const
    {
        difference = 0.0;
        if (!(scale > 0.0) ||
            !IsFiniteDouble(scale) ||
            !IsFiniteDouble(point) ||
            !IsFiniteDouble(origin))
        {
            return false;
        }
        if ((point < 0.0) == (origin < 0.0))
        {
            // Preserve small translated deltas. Equal-sign finite subtraction
            // is safe even when both coordinates are near DBL_MAX.
            difference = (point - origin) / scale;
        }
        else
        {
            // Opposite-sign subtraction may overflow. The selected scale is at
            // least their saturated absolute difference, so division first is
            // safe and preserves the direction ratio. Volatile intermediates
            // prevent -ffast-math from reassociating this back into the
            // overflowing (point - origin) / scale expression.
            volatile double scaled_point = point / scale;
            volatile double scaled_origin = origin / scale;
            difference = scaled_point - scaled_origin;
        }
        return IsFiniteDouble(difference);
    }

    bool IsDegenerateTriangle(
        std::size_t first,
        std::size_t second,
        std::size_t third) const
    {
        const JBeamStructuralNode& a = m_result.nodes[first];
        const JBeamStructuralNode& b = m_result.nodes[second];
        const JBeamStructuralNode& c = m_result.nodes[third];
        double scale = SafeAbsoluteDifference(a.x, b.x);
        scale = std::max(
            scale, SafeAbsoluteDifference(a.y, b.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(a.z, b.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(a.x, c.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(a.y, c.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(a.z, c.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(b.x, c.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(b.y, c.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(b.z, c.z));
        if (!(scale > 0.0))
        {
            return true;
        }
        double ux = 0.0;
        double uy = 0.0;
        double uz = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        if (!TryScaledDifference(b.x, a.x, scale, ux) ||
            !TryScaledDifference(b.y, a.y, scale, uy) ||
            !TryScaledDifference(b.z, a.z, scale, uz) ||
            !TryScaledDifference(c.x, a.x, scale, vx) ||
            !TryScaledDifference(c.y, a.y, scale, vy) ||
            !TryScaledDifference(c.z, a.z, scale, vz))
        {
            return true;
        }
        const double cx = uy * vz - uz * vy;
        const double cy = uz * vx - ux * vz;
        const double cz = ux * vy - uy * vx;
        const double area_squared =
            cx * cx + cy * cy + cz * cz;
        return !IsFiniteDouble(area_squared) ||
            !(area_squared > 1.0e-24);
    }

    bool IsDegenerateFrame(
        std::size_t reference,
        std::size_t back,
        std::size_t left,
        std::size_t up) const
    {
        const JBeamStructuralNode& r = m_result.nodes[reference];
        const JBeamStructuralNode& b = m_result.nodes[back];
        const JBeamStructuralNode& l = m_result.nodes[left];
        const JBeamStructuralNode& u = m_result.nodes[up];
        double scale = SafeAbsoluteDifference(r.x, b.x);
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, b.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, b.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.x, l.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, l.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, l.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.x, u.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, u.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, u.z));
        if (!(scale > 0.0))
        {
            return true;
        }
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        double lx = 0.0;
        double ly = 0.0;
        double lz = 0.0;
        double ux = 0.0;
        double uy = 0.0;
        double uz = 0.0;
        if (!TryScaledDifference(b.x, r.x, scale, bx) ||
            !TryScaledDifference(b.y, r.y, scale, by) ||
            !TryScaledDifference(b.z, r.z, scale, bz) ||
            !TryScaledDifference(l.x, r.x, scale, lx) ||
            !TryScaledDifference(l.y, r.y, scale, ly) ||
            !TryScaledDifference(l.z, r.z, scale, lz) ||
            !TryScaledDifference(u.x, r.x, scale, ux) ||
            !TryScaledDifference(u.y, r.y, scale, uy) ||
            !TryScaledDifference(u.z, r.z, scale, uz))
        {
            return true;
        }
        const double cross_x = by * lz - bz * ly;
        const double cross_y = bz * lx - bx * lz;
        const double cross_z = bx * ly - by * lx;
        const double determinant =
            cross_x * ux + cross_y * uy + cross_z * uz;
        const double b_len2 = bx * bx + by * by + bz * bz;
        const double l_len2 = lx * lx + ly * ly + lz * lz;
        const double u_len2 = ux * ux + uy * uy + uz * uz;
        const double product = b_len2 * l_len2 * u_len2;
        return !IsFiniteDouble(determinant) ||
            !IsFiniteDouble(product) ||
            !(product > 0.0) ||
            determinant * determinant <= product * 1.0e-24;
    }

    bool IsAlignedFrame(
        std::size_t reference,
        std::size_t back,
        std::size_t left,
        std::size_t up) const
    {
        const JBeamStructuralNode& r = m_result.nodes[reference];
        const JBeamStructuralNode& b = m_result.nodes[back];
        const JBeamStructuralNode& l = m_result.nodes[left];
        const JBeamStructuralNode& u = m_result.nodes[up];
        double scale = SafeAbsoluteDifference(r.x, b.x);
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, b.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, b.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.x, l.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, l.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, l.z));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.x, u.x));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.y, u.y));
        scale = std::max(
            scale, SafeAbsoluteDifference(r.z, u.z));
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        double lx = 0.0;
        double ly = 0.0;
        double lz = 0.0;
        double ux = 0.0;
        double uy = 0.0;
        double uz = 0.0;
        if (!TryScaledDifference(b.x, r.x, scale, bx) ||
            !TryScaledDifference(b.y, r.y, scale, by) ||
            !TryScaledDifference(b.z, r.z, scale, bz) ||
            !TryScaledDifference(l.x, r.x, scale, lx) ||
            !TryScaledDifference(l.y, r.y, scale, ly) ||
            !TryScaledDifference(l.z, r.z, scale, lz) ||
            !TryScaledDifference(u.x, r.x, scale, ux) ||
            !TryScaledDifference(u.y, r.y, scale, uy) ||
            !TryScaledDifference(u.z, r.z, scale, uz))
        {
            return false;
        }
        // Real vehicles pick refNodes from actual body geometry, so the
        // documented +Y back / +X left / +Z up semantics hold as
        // direction dominance rather than exact axis alignment: each
        // axis node's documented component must be positive and strictly
        // dominate the other two components of its offset from the
        // reference node.
        return by > 0.0 &&
            Abs(bx) < by &&
            Abs(bz) < by &&
            lx > 0.0 &&
            Abs(ly) < lx &&
            Abs(lz) < lx &&
            uz > 0.0 &&
            Abs(ux) < uz &&
            Abs(uy) < uz;
    }

    bool AreAlignedFrontCorners(
        std::size_t reference,
        std::size_t back,
        std::size_t left,
        std::size_t left_corner,
        std::size_t right_corner) const
    {
        const JBeamStructuralNode& r = m_result.nodes[reference];
        const JBeamStructuralNode& b = m_result.nodes[back];
        const JBeamStructuralNode& l = m_result.nodes[left];
        const JBeamStructuralNode& lc =
            m_result.nodes[left_corner];
        const JBeamStructuralNode& rc =
            m_result.nodes[right_corner];
        double scale = SafeAbsoluteDifference(r.x, b.x);
        const JBeamStructuralNode* points[] = {
            &b, &l, &lc, &rc
        };
        for (std::size_t i = 0U; i < 4U; ++i)
        {
            scale = std::max(
                scale,
                SafeAbsoluteDifference(r.x, points[i]->x));
            scale = std::max(
                scale,
                SafeAbsoluteDifference(r.y, points[i]->y));
            scale = std::max(
                scale,
                SafeAbsoluteDifference(r.z, points[i]->z));
        }
        if (!(scale > 0.0))
        {
            return false;
        }
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        double lx = 0.0;
        double ly = 0.0;
        double lz = 0.0;
        double lcx = 0.0;
        double lcy = 0.0;
        double lcz = 0.0;
        double rcx = 0.0;
        double rcy = 0.0;
        double rcz = 0.0;
        if (!TryScaledDifference(b.x, r.x, scale, bx) ||
            !TryScaledDifference(b.y, r.y, scale, by) ||
            !TryScaledDifference(b.z, r.z, scale, bz) ||
            !TryScaledDifference(l.x, r.x, scale, lx) ||
            !TryScaledDifference(l.y, r.y, scale, ly) ||
            !TryScaledDifference(l.z, r.z, scale, lz) ||
            !TryScaledDifference(lc.x, r.x, scale, lcx) ||
            !TryScaledDifference(lc.y, r.y, scale, lcy) ||
            !TryScaledDifference(lc.z, r.z, scale, lcz) ||
            !TryScaledDifference(rc.x, r.x, scale, rcx) ||
            !TryScaledDifference(rc.y, r.y, scale, rcy) ||
            !TryScaledDifference(rc.z, r.z, scale, rcz))
        {
            return false;
        }
        const double left_front =
            lcx * bx + lcy * by + lcz * bz;
        const double right_front =
            rcx * bx + rcy * by + rcz * bz;
        const double left_side =
            lcx * lx + lcy * ly + lcz * lz;
        const double right_side =
            rcx * lx + rcy * ly + rcz * lz;
        return IsFiniteDouble(left_front) &&
            IsFiniteDouble(right_front) &&
            IsFiniteDouble(left_side) &&
            IsFiniteDouble(right_side) &&
            left_front < 0.0 &&
            right_front < 0.0 &&
            left_side > 0.0 &&
            right_side < 0.0;
    }

    void ProcessRefNodes()
    {
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            JBeamNormalizeResult normalized;
            const JBeamNormalizedTable* table = NormalizeSection(
                m_sections[part_index], "refNodes", normalized);
            if (m_resource_limit)
            {
                return;
            }
            if (table == NULL)
            {
                continue;
            }
            PreserveUnsupportedDefaults(
                *table,
                m_sections[part_index].part.provenance,
                "refNodes");
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t entry_index = 0U;
                 entry_index < table->entries.size();
                 ++entry_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const JBeamNormalizedTableEntry& entry =
                    table->entries[entry_index];
                if (entry.kind ==
                    JBeamNormalizedTableEntryKind::INVALID_ENTRY)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "refNodes",
                        entry_index,
                        std::string(),
                        "refNodes entry must be a data row or default");
                    continue;
                }
                if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DATA_ROW)
                {
                    continue;
                }
                ++m_ref_row_count;
                PreserveUnsupportedAssignments(
                    *table,
                    entry.data_row,
                    m_sections[part_index].part.provenance,
                    "refNodes",
                    entry_index);
                if (m_resource_limit)
                {
                    return;
                }
                JBeamStructuralRefFrame frame;
                frame.reference_index = INVALID_INDEX;
                frame.back_index = INVALID_INDEX;
                frame.left_index = INVALID_INDEX;
                frame.up_index = INVALID_INDEX;
                frame.left_corner_index = INVALID_INDEX;
                frame.right_corner_index = INVALID_INDEX;
                frame.provenance =
                    ProvenanceWithSpan(
                        m_sections[part_index].part.provenance,
                        entry.data_row.span);
                bool valid = ReadRequiredString(
                    entry.data_row,
                    frame.provenance,
                    "refNodes",
                    entry_index,
                    "ref:",
                    "ref",
                    frame.reference);
                valid = ReadRequiredString(
                    entry.data_row,
                    frame.provenance,
                    "refNodes",
                    entry_index,
                    "back:",
                    "back",
                    frame.back) && valid;
                valid = ReadRequiredString(
                    entry.data_row,
                    frame.provenance,
                    "refNodes",
                    entry_index,
                    "left:",
                    "left",
                    frame.left) && valid;
                valid = ReadRequiredString(
                    entry.data_row,
                    frame.provenance,
                    "refNodes",
                    entry_index,
                    "up:",
                    "up",
                    frame.up) && valid;
                // Both or neither: a row that names one corner but not the
                // other is malformed, and the second read reports exactly
                // which one is missing.
                const bool has_ref_corners =
                    HasRefCornerColumns(entry.data_row);
                if (has_ref_corners)
                {
                    valid = ReadRequiredString(
                        entry.data_row,
                        frame.provenance,
                        "refNodes",
                        entry_index,
                        "leftCorner:",
                        "leftCorner",
                        frame.left_corner) && valid;
                    valid = ReadRequiredString(
                        entry.data_row,
                        frame.provenance,
                        "refNodes",
                        entry_index,
                        "rightCorner:",
                        "rightCorner",
                        frame.right_corner) && valid;
                }
                const std::size_t ref_id_count = has_ref_corners ? 6U : 4U;
                const std::string ids[] = {
                    frame.reference,
                    frame.back,
                    frame.left,
                    frame.up,
                    frame.left_corner,
                    frame.right_corner
                };
                std::size_t* indices[] = {
                    &frame.reference_index,
                    &frame.back_index,
                    &frame.left_index,
                    &frame.up_index,
                    &frame.left_corner_index,
                    &frame.right_corner_index
                };
                if (valid)
                {
                    std::set<std::string> distinct;
                    for (std::size_t i = 0U; i < ref_id_count; ++i)
                    {
                        if (!distinct.insert(ids[i]).second)
                        {
                            Push(
                                JBeamStructuralDiagnosticCode::
                                    DUPLICATE_VERTEX,
                                JBeamStructuralSeverity::ERROR_SEVERITY,
                                frame.provenance,
                                "refNodes",
                                entry_index,
                                std::string(),
                                "refNodes identifiers must be distinct");
                            valid = false;
                            break;
                        }
                    }
                }
                if (valid)
                {
                    for (std::size_t i = 0U; i < ref_id_count; ++i)
                    {
                        if (!ResolveNode(ids[i], *indices[i]))
                        {
                            Push(
                                JBeamStructuralDiagnosticCode::
                                    MISSING_NODE_REFERENCE,
                                JBeamStructuralSeverity::ERROR_SEVERITY,
                                frame.provenance,
                                "refNodes",
                                entry_index,
                                std::string(),
                                "refNodes references an unknown node: " +
                                    ids[i]);
                            valid = false;
                        }
                    }
                }
                if (valid &&
                    IsDegenerateFrame(
                        frame.reference_index,
                        frame.back_index,
                        frame.left_index,
                        frame.up_index))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::
                            DEGENERATE_REF_NODES,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        frame.provenance,
                        "refNodes",
                        entry_index,
                        std::string(),
                        "refNodes axes must span a nondegenerate frame");
                    valid = false;
                }
                if (valid &&
                    !IsAlignedFrame(
                        frame.reference_index,
                        frame.back_index,
                        frame.left_index,
                        frame.up_index))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::
                            MISALIGNED_REF_NODES,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        frame.provenance,
                        "refNodes",
                        entry_index,
                        std::string(),
                        "refNodes must align with +Y back, +X left, "
                        "and +Z up");
                    valid = false;
                }
                if (valid && has_ref_corners &&
                    !AreAlignedFrontCorners(
                        frame.reference_index,
                        frame.back_index,
                        frame.left_index,
                        frame.left_corner_index,
                        frame.right_corner_index))
                {
                    // The pair is optional and feeds only this check, so a
                    // pair that is not a front-corner pair is treated as the
                    // absence it is equivalent to, rather than as grounds to
                    // reject the vehicle. BeamNG does not check this at all;
                    // the Germanwings A319 names `fusf` and `fus15b`, which
                    // are not a symmetric left/right pair. Dropping them is
                    // better than retaining values no consumer should trust.
                    Push(
                        JBeamStructuralDiagnosticCode::
                            MISALIGNED_REF_CORNERS,
                        JBeamStructuralSeverity::WARNING,
                        frame.provenance,
                        "refNodes",
                        entry_index,
                        std::string(),
                        "leftCorner and rightCorner are not forward of ref "
                        "on their documented sides; the optional corner pair "
                        "is dropped");
                    frame.left_corner.clear();
                    frame.right_corner.clear();
                    frame.left_corner_index = INVALID_INDEX;
                    frame.right_corner_index = INVALID_INDEX;
                }
                if (m_resource_limit)
                {
                    return;
                }
                if (valid && m_ref_row_count == 1U)
                {
                    std::size_t retained = 0U;
                    if (!AddSize(frame.reference.size(), retained) ||
                        !AddSize(frame.back.size(), retained) ||
                        !AddSize(frame.left.size(), retained) ||
                        !AddSize(frame.up.size(), retained) ||
                        !AddSize(
                            frame.left_corner.size(), retained) ||
                        !AddSize(
                            frame.right_corner.size(), retained))
                    {
                        RejectRetained();
                        return;
                    }
                    if (!ReserveRetained(retained))
                    {
                        return;
                    }
                    m_result.ref_frame = frame;
                    m_result.has_ref_frame = true;
                }
            }
        }
    }

    void ProcessBeams()
    {
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            JBeamNormalizeResult normalized;
            const JBeamNormalizedTable* table = NormalizeSection(
                m_sections[part_index], "beams", normalized);
            if (m_resource_limit)
            {
                return;
            }
            if (table == NULL)
            {
                continue;
            }
            PreserveUnsupportedDefaults(
                *table,
                m_sections[part_index].part.provenance,
                "beams");
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t entry_index = 0U;
                 entry_index < table->entries.size();
                 ++entry_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const JBeamNormalizedTableEntry& entry =
                    table->entries[entry_index];
                if (entry.kind ==
                    JBeamNormalizedTableEntryKind::INVALID_ENTRY)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "beams",
                        entry_index,
                        std::string(),
                        "Beam table entry is not a data row or default");
                    continue;
                }
                if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DATA_ROW)
                {
                    continue;
                }
                if (m_authored_beam_rows >= m_limits.max_beams)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::BEAM_LIMIT,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "beams",
                        entry_index,
                        std::string(),
                        "Beam table exceeds the configured beam limit");
                    m_resource_limit = true;
                    return;
                }
                ++m_authored_beam_rows;
                PreserveUnsupportedAssignments(
                    *table,
                    entry.data_row,
                    m_sections[part_index].part.provenance,
                    "beams",
                    entry_index);
                if (m_resource_limit)
                {
                    return;
                }
                JBeamStructuralBeam beam;
                beam.provenance =
                    ProvenanceWithSpan(
                        m_sections[part_index].part.provenance,
                        entry.data_row.span);
                bool valid = ReadRequiredString(
                    entry.data_row,
                    beam.provenance,
                    "beams",
                    entry_index,
                    "id1:",
                    "id1",
                    beam.node_a);
                valid = ReadRequiredString(
                    entry.data_row,
                    beam.provenance,
                    "beams",
                    entry_index,
                    "id2:",
                    "id2",
                    beam.node_b) && valid;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    beam.provenance,
                    "beams",
                    entry_index,
                    "optional",
                    false,
                    beam.optional) && valid;
                valid = ReadBeamType(
                    entry.data_row,
                    beam.provenance,
                    entry_index,
                    beam.beam_type,
                    beam.status) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamSpring",
                    beam.has_spring, beam.spring) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamDamp",
                    beam.has_damping, beam.damping) && valid;
                // allow_blank: a blank is a modifier RESET and this corpus
                // authors 20 blank dampCutoffHz values, so a reader that
                // refuses blanks refuses those rows outright.
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamDampFast",
                    beam.has_damp_fast, beam.damp_fast, true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamDampRebound",
                    beam.has_damp_rebound, beam.damp_rebound, true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamDampReboundFast",
                    beam.has_damp_rebound_fast,
                    beam.damp_rebound_fast, true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamDampVelocitySplit",
                    beam.has_damp_velocity_split,
                    beam.damp_velocity_split, true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "dampCutoffHz",
                    beam.has_damp_cutoff_hz,
                    beam.damp_cutoff_hz, true) && valid;
                valid = ReadOptionalBeamLimit(
                    entry.data_row, beam.provenance,
                    entry_index, "beamDeform", beam.has_deform,
                    beam.deform_unbounded, beam.deform) && valid;
                valid = ReadOptionalBeamLimit(
                    entry.data_row, beam.provenance,
                    entry_index, "beamStrength", beam.has_strength,
                    beam.strength_unbounded, beam.strength) && valid;
                valid = ReadGroupName(
                    entry.data_row, beam.provenance,
                    entry_index, "beams", "breakGroup",
                    beam.break_group) && valid;
                valid = ReadGroupName(
                    entry.data_row, beam.provenance,
                    entry_index, "beams", "deformGroup",
                    beam.deform_group) && valid;
                // Retained, never consumed: see JBeamStructuralBeam. The
                // authored flags come from PRESENCE, not from the value, so a
                // row that states the documented default is still recorded as
                // having stated it.
                valid = ReadOptionalString(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "name", beam.name) && valid;
                valid = ReadOptionalExhaustEngine(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, beam.exhaust_engine) && valid;
                valid = ReadOptionalString(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "breakFx", beam.break_fx) && valid;
                beam.disable_mesh_breaking_authored =
                    FindEffectiveJBeamField(
                        entry.data_row, "disableMeshBreaking") != NULL;
                valid = ReadOptionalBoolean(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "disableMeshBreaking", false,
                    beam.disable_mesh_breaking,
                    &beam.disable_mesh_breaking_type_refused) && valid;
                beam.disable_triangle_breaking_authored =
                    FindEffectiveJBeamField(
                        entry.data_row, "disableTriangleBreaking") != NULL;
                valid = ReadOptionalBoolean(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "disableTriangleBreaking", false,
                    beam.disable_triangle_breaking,
                    &beam.disable_triangle_breaking_type_refused) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "deformLimitExpansion",
                    beam.has_deform_limit_expansion,
                    beam.deform_limit_expansion,
                    // A blank value is a modifier RESET, not a malformed
                    // number: infgtr alone authors 381 of them. Without this
                    // every one is an invalid-field-type error and the whole
                    // vehicle is refused, which is what happened.
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "deformationTriggerRatio",
                    beam.has_deformation_trigger_ratio,
                    beam.deformation_trigger_ratio,
                    // Blank is a reset here for the same reason it is on
                    // deformLimitExpansion: these modifiers travel together
                    // through the same authored blocks.
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamPrecompression",
                    beam.has_precompression,
                    beam.precompression) && valid;
                // Metres, and it supersedes the multiplier above where both
                // are present - both facts stated by the content itself.
                // The variants that author it as an existence check select
                // the empty string on the branch where it does not apply, so
                // this is the one beam field that accepts a blank as absent.
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "precompressionRange",
                    beam.has_precompression_range,
                    beam.precompression_range,
                    true) && valid;
                // Seconds; travels in the same authored blocks as the
                // precompression it ramps, so a blank is the same reset.
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamPrecompressionTime",
                    beam.has_precompression_time,
                    beam.precompression_time,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamLongBound",
                    beam.has_long_bound,
                    beam.long_bound,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamShortBound",
                    beam.has_short_bound,
                    beam.short_bound,
                    true) && valid;
                // The whole |BOUNDED family accepts a blank as "unauthored"
                // for the same reason precompressionRange does, and for one
                // more: real content authors a modifier RESET,
                // {"longBoundRange":"", "shortBoundRange":""}, to end a run
                // of bounded rows and return the rows after it to plain
                // beams. Measured on an installed package (2026-09-02): the
                // hood part of 350Z_modland.zip does exactly that after two
                // numeric modifier runs, and refusing the blank refused the
                // whole vehicle - 58 error diagnostics on one part, every
                // one of them this shape - with nothing else wrong with it.
                // A blank is a documented existence-check outcome
                // (`case(<cond>, <value>, '')`) and the modifier reset is
                // the same value by another route; both mean "this row does
                // not author the field". Any other non-numeric value is
                // still a refusal. Each blank is recorded as a WARNING
                // diagnostic, so the count of rows it touched is visible.
                // Metres, and each supersedes its dimensionless counterpart
                // above where a row carries both - the documentation states
                // that precedence outright ("will overwrite beamLongBound").
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "longBoundRange",
                    beam.has_long_bound_range,
                    beam.long_bound_range,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "shortBoundRange",
                    beam.has_short_bound_range,
                    beam.short_bound_range,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "boundZone",
                    beam.has_bound_zone,
                    beam.bound_zone,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamLimitSpring",
                    beam.has_limit_spring,
                    beam.limit_spring,
                    true) && valid;
                valid = ReadOptionalNumber(
                    entry.data_row, beam.provenance, "beams",
                    entry_index, "beamLimitDamp",
                    beam.has_limit_damping,
                    beam.limit_damping,
                    true) && valid;
                const bool invalid_bound =
                    (beam.has_short_bound && beam.short_bound < 0.0) ||
                    (beam.has_long_bound_range &&
                        beam.long_bound_range < 0.0) ||
                    (beam.has_short_bound_range &&
                        beam.short_bound_range < 0.0) ||
                    (beam.has_bound_zone && beam.bound_zone < 0.0) ||
                    (beam.has_limit_spring && beam.limit_spring < 0.0) ||
                    (beam.has_limit_damping && beam.limit_damping < 0.0);
                // NORMAL uses the ordinary signed axial coefficient. This is
                // not a negative sentinel, default request or damping-only
                // alias. Specialized positive-law roles remain restricted.
                const bool invalid_spring =
                    beam.has_spring && beam.spring < 0.0 &&
                    beam.beam_type != "NORMAL";
                const bool invalid_damping =
                    beam.has_damping && beam.damping < 0.0;
                const bool invalid_deform =
                    beam.has_deform && !beam.deform_unbounded &&
                    beam.deform < 0.0;
                const bool invalid_strength =
                    beam.has_strength && !beam.strength_unbounded &&
                    beam.strength < 0.0;
                const bool invalid_precompression =
                    beam.has_precompression &&
                    !(beam.precompression > 0.0);
                const bool invalid_long_bound =
                    beam.has_long_bound && beam.long_bound < 0.0;
                if (valid &&
                    (invalid_spring ||
                     invalid_damping ||
                     invalid_deform ||
                     invalid_strength ||
                     invalid_precompression ||
                     invalid_long_bound ||
                     invalid_bound))
                {
                    Push(
                        JBeamStructuralDiagnosticCode::
                            INVALID_BEAM_PARAMETER,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        beam.provenance,
                        "beams",
                        entry_index,
                        std::string(),
                        "Non-NORMAL spring, damping, deform, strength, bound "
                        "thresholds, bound zone and limit constants must be "
                        "nonnegative; precompression must be positive");
                    valid = false;
                }
                if (valid && beam.node_a == beam.node_b)
                {
                    // A beam from a node to itself has zero length and no
                    // direction, so it can exert nothing: BeamNG builds it and
                    // it does nothing. Rejecting the whole vehicle over one is
                    // out of proportion -- the Germanwings A319 has two, in
                    // its gear doors -- so drop the row and say so. Nothing is
                    // guessed at and no beam is invented in its place.
                    Push(
                        JBeamStructuralDiagnosticCode::DUPLICATE_VERTEX,
                        JBeamStructuralSeverity::WARNING,
                        beam.provenance,
                        "beams",
                        entry_index,
                        std::string(),
                        "Beam endpoints are the same node; the beam is "
                        "zero-length and is dropped");
                    continue;
                }
                const bool has_a =
                    valid && ResolveNode(
                        beam.node_a, beam.node_a_index);
                const bool has_b =
                    valid && ResolveNode(
                        beam.node_b, beam.node_b_index);
                if (valid && (!has_a || !has_b))
                {
                    if (beam.optional)
                    {
                        beam.status =
                            JBeamStructuralBeamStatus::
                                PRESERVED_DISABLED_OPTIONAL_REFERENCE;
                        Push(
                            JBeamStructuralDiagnosticCode::
                                OPTIONAL_BEAM_SKIPPED,
                            JBeamStructuralSeverity::WARNING,
                            beam.provenance,
                            "beams",
                            entry_index,
                            std::string(),
                            "Optional beam with a missing node reference "
                            "is preserved but disabled");
                    }
                    else
                    {
                        // Real-world part combinations author beams to
                        // nodes owned by parts that are missing from the
                        // package (base-game content). BeamNG's loader
                        // skips such beams with a warning; the row is
                        // skipped deterministically here as well.
                        Push(
                            JBeamStructuralDiagnosticCode::
                                MISSING_NODE_REFERENCE,
                            JBeamStructuralSeverity::WARNING,
                            beam.provenance,
                            "beams",
                            entry_index,
                            std::string(),
                            "Beam references an unknown node and is "
                            "skipped");
                        valid = false;
                    }
                }
                if (valid && has_a && has_b &&
                    AreSamePosition(
                        beam.node_a_index, beam.node_b_index))
                {
                    if (beam.has_spring && beam.spring < 0.0)
                    {
                        // Signed stiffness is an ordinary finite-rest axial
                        // coefficient, not a zero-rest restraint law.
                        Push(
                            JBeamStructuralDiagnosticCode::INVALID_BEAM_PARAMETER,
                            JBeamStructuralSeverity::ERROR_SEVERITY,
                            beam.provenance, "beams", entry_index,
                            std::string(),
                            "Negative spring requires a finite-rest NORMAL beam");
                        valid = false;
                    }
                    else if (HasSupportedJBeamZeroRestSpringParameters(beam) &&
                        ZeroRestRowHasOnlyImplementedFields(
                            *table, entry.data_row, beam.provenance))
                    {
                        beam.jbeam_zero_rest_spring = true;
                    }
                    else
                    {
                        Push(
                            JBeamStructuralDiagnosticCode::DEGENERATE_BEAM,
                            JBeamStructuralSeverity::WARNING,
                            beam.provenance,
                            "beams",
                            entry_index,
                            std::string(),
                            "Coincident beam does not satisfy the supported "
                            "NORMAL/neutral zero-rest spring policy (explicit "
                            "FLT_MAX or finite positive beamStrength and "
                            "beamDeform, unit precompression and bounds, "
                            "breakGroup admitted); the row is skipped");
                        valid = false;
                    }
                }
                if (m_resource_limit)
                {
                    return;
                }
                if (valid)
                {
                    std::size_t retained = 0U;
                    if (!AddSize(beam.node_a.size(), retained) ||
                        !AddSize(beam.node_b.size(), retained) ||
                        !AddSize(beam.beam_type.size(), retained) ||
                        !AddSize(beam.break_group.size(), retained) ||
                        !AddSize(beam.deform_group.size(), retained) ||
                        !AddSize(beam.exhaust_engine.size(), retained) ||
                        !AddSize(beam.break_fx.size(), retained))
                    {
                        RejectRetained();
                        return;
                    }
                    if (!ReserveRetained(retained))
                    {
                        return;
                    }
                    m_result.beams.push_back(beam);
                }
            }
        }
    }

    JBeamAeroScalar ReadAerodynamicScalar(
        const std::shared_ptr<const JBeamValue>& value,
        const JBeamStructuralProvenance& provenance,
        const std::string& section, std::size_t row_index,
        const std::string& field, JBeamAeroScalarOrigin origin)
    {
        JBeamAeroScalar result;
        result.origin = origin;
        result.provenance = provenance;
        result.authored_value = value;
        const std::size_t diagnostic_begin = m_result.diagnostics.size();
        JBeamExpressionValue evaluated;
        const bool subnormal_literal = value && value->type == JBeamValueType::NUMBER &&
            IsBinary64SubnormalMagnitude(value->number_value);
        const bool resolved = value && !subnormal_literal && ResolveScalarValue(
            *value, VariablesFor(provenance), provenance,
            section, row_index, field, evaluated);
        // This descriptor is not physics admission. Keep unsupported authored
        // aero inert without newly rejecting otherwise admitted geometry.
        // Resource/complexity limits remain hard errors, never downgraded.
        for (std::size_t i = diagnostic_begin;
             i < m_result.diagnostics.size(); ++i)
        {
            JBeamStructuralDiagnostic& diagnostic = m_result.diagnostics[i];
            if (diagnostic.code == JBeamStructuralDiagnosticCode::EXPRESSION_ERROR ||
                diagnostic.code == JBeamStructuralDiagnosticCode::NON_FINITE_NUMBER)
            {
                diagnostic.severity = JBeamStructuralSeverity::WARNING;
            }
        }
        result.resolved = resolved &&
            evaluated.type == JBeamExpressionValueType::NUMBER &&
            IsFiniteDouble(evaluated.number_value) &&
            !IsBinary64SubnormalMagnitude(evaluated.number_value);
        if (result.resolved)
        {
            result.value = evaluated.number_value;
            result.effective_value = result.value;
            result.effective_resolved = true;
        }
        else if (!m_resource_limit)
        {
            Push(JBeamStructuralDiagnosticCode::AERODYNAMICS_UNRESOLVED,
                JBeamStructuralSeverity::WARNING, provenance, section,
                row_index, field,
                "Authored aerodynamic value is preserved unresolved; no default or force is substituted");
        }
        return result;
    }

    void PreserveAerodynamicScales()
    {
        static const char* const names[] = {
            "scaledragCoef", "scaleskinDragCoef", "scaleliftCoef", "scalestallAngle"
        };
        for (std::size_t p = 0U; p < m_parts.size() && !m_resource_limit; ++p)
        {
            const JBeamValue& body = m_parts[p].node->definition.body;
            for (std::size_t f = 0U; f < body.object_fields.size() && !m_resource_limit; ++f)
            {
                const JBeamObjectField& field = body.object_fields[f];
                for (std::size_t parameter = 0U; parameter < 4U; ++parameter)
                {
                    if (field.key != names[parameter]) continue;
                    if (!ReserveRetained(sizeof(JBeamAeroScaleAssignment))) return;
                    JBeamAeroScaleAssignment assignment;
                    assignment.parameter = static_cast<JBeamAeroParameter>(parameter);
                    assignment.scalar = ReadAerodynamicScalar(field.value,
                        ProvenanceWithSpan(m_parts[p].provenance,
                            field.value ? field.value->span : field.key_span),
                        field.key, f, field.key, JBeamAeroScalarOrigin::PART_SCALE);
                    m_aero_scale_index[parameter] = m_result.aerodynamic_scales.size();
                    ++m_aero_scale_count[parameter];
                    m_result.aerodynamic_scales.push_back(assignment);
                }
            }
        }
    }

    JBeamAeroScalar ReadAerodynamicField(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& provenance,
        const std::string& section, std::size_t row_index,
        const std::string& name, double default_value)
    {
        const JBeamFieldAssignment* assignment = FindEffectiveJBeamField(row, name);
        if (assignment == NULL)
        {
            JBeamAeroScalar result;
            result.resolved = result.effective_resolved = true;
            result.value = result.effective_value = default_value;
            return result;
        }
        JBeamAeroScalarOrigin origin = JBeamAeroScalarOrigin::INHERITED_DEFAULT;
        if (assignment->origin == JBeamFieldOrigin::POSITIONAL_CELL)
            origin = JBeamAeroScalarOrigin::POSITIONAL_CELL;
        else if (assignment->origin == JBeamFieldOrigin::ROW_LOCAL_OVERRIDE)
            origin = JBeamAeroScalarOrigin::ROW_LOCAL_OVERRIDE;
        return ReadAerodynamicScalar(assignment->value,
            ProvenanceWithSpan(provenance, assignment->span),
            section, row_index, name, origin);
    }

    void ApplyUnambiguousAerodynamicScale(JBeamAeroScalar& scalar, std::size_t parameter)
    {
        if (m_aero_scale_count[parameter] == 0U) return;
        // The documented mechanism multiplies the named MODIFIERS present in
        // the structure: "multiplying all modifiers with the name specified
        // in the rest of the string with the provided value".
        // https://documentation.beamng.com/modding/vehicle/intro_jbeam/jbeamsyntax/
        // A field nobody authored carries no modifier for the scale to reach,
        // so it keeps its documented default, unscaled. That is what the page
        // states rather than a choice between two readings, and it is what
        // admits a car whose panels default their drag instead of refusing
        // the whole vehicle's aerodynamics.
        if (!scalar.authored_value) return;
        scalar.effective_resolved = false;
        scalar.effective_value = 0.0;
        if (!scalar.resolved || m_aero_scale_count[parameter] != 1U) return;
        const JBeamAeroScalar& scale =
            m_result.aerodynamic_scales[m_aero_scale_index[parameter]].scalar;
        if (!scale.resolved) return;
        volatile double product = scalar.value * scale.value;
        const double stored_product = product;
        // Do not let a fast-math flush-to-zero become an authored zero.
        // Preserve out-of-profile extremes unresolved in strict builds too.
        if (!IsFiniteDouble(stored_product) || IsBinary64SubnormalMagnitude(stored_product) ||
            (stored_product == 0.0 && scalar.value != 0.0 && scale.value != 0.0)) return;
        scalar.effective_value = stored_product;
        scalar.effective_resolved = true;
    }

    void PreserveTriangleAerodynamics(const JBeamNormalizedDataRow& row,
        const std::string& section, std::size_t row_index,
        JBeamStructuralTriangle& triangle)
    {
        JBeamTriangleAerodynamics& aero = triangle.aerodynamics;
        const JBeamStructuralProvenance& provenance = triangle.provenance;
        // Official docs: percentages of an equal-area flat plate, not Cd/Cl.
        // https://documentation.beamng.com/modding/vehicle/sections/triangles/
        // Scope/row and scaling rules:
        // https://docs.beamng.com/modding/vehicle/intro_jbeam/jbeamsyntax/
        aero.drag_percent = ReadAerodynamicField(row, provenance, section,
            row_index, "dragCoef", 100.0);
        aero.skin_drag_percent = ReadAerodynamicField(row, provenance, section,
            row_index, "skinDragCoef", 0.0);
        aero.stall_angle_rad = ReadAerodynamicField(row, provenance, section,
            row_index, "stallAngle", 0.58);
        ApplyUnambiguousAerodynamicScale(aero.drag_percent, 0U);
        ApplyUnambiguousAerodynamicScale(aero.skin_drag_percent, 1U);
        ApplyUnambiguousAerodynamicScale(aero.stall_angle_rad, 3U);
        if (FindEffectiveJBeamField(row, "liftCoef") == NULL)
        {
            // Documented default: liftCoef defaults to dragCoef. The scale
            // pass multiplies the authored dragCoef modifier, and this
            // default is taken from that same field, so a defaulted lift
            // follows the SCALED drag and the authored lift/drag relation is
            // preserved. No liftCoef modifier exists on this row, so
            // scaleliftCoef has nothing here to multiply.
            aero.lift_percent = aero.drag_percent;
            aero.lift_percent.origin = JBeamAeroScalarOrigin::DEFAULT_FROM_DRAG;
            aero.lift_percent.authored_value.reset();
        }
        else
        {
            aero.lift_percent = ReadAerodynamicField(row, provenance, section,
                row_index, "liftCoef", 0.0);
            ApplyUnambiguousAerodynamicScale(aero.lift_percent, 2U);
        }
        const bool resolved = aero.drag_percent.effective_resolved &&
            aero.skin_drag_percent.effective_resolved &&
            aero.lift_percent.effective_resolved && aero.stall_angle_rad.effective_resolved;
        aero.capability = resolved ? JBeamAeroCapability::RESOLVED_PARAMETERS_NOT_APPLIED
            : JBeamAeroCapability::UNRESOLVED_PARAMETERS_NOT_APPLIED;
        if (!resolved && !m_resource_limit &&
            m_aero_unresolved_parts.insert(provenance.PartPreorderIndex()).second)
            Push(JBeamStructuralDiagnosticCode::AERODYNAMICS_UNRESOLVED,
                JBeamStructuralSeverity::WARNING, provenance, section, row_index,
                "aerodynamics", "Parameters retained only; unresolved value or scale/default pass ordering prevents aerodynamic admission");
        if (triangle.status == JBeamStructuralTriangleStatus::PRESERVED_DISABLED_OPTIONAL_REFERENCE)
            aero.area_source = JBeamAeroAreaSource::UNAVAILABLE_OPTIONAL_NODE_REFERENCE;
    }

    bool BuildTriangle(
        const JBeamNormalizedDataRow& row,
        const JBeamStructuralProvenance& part_provenance,
        const std::string& section,
        std::size_t entry_index,
        const std::string& first,
        const std::string& second,
        const std::string& third,
        bool optional,
        JBeamStructuralTriangleOrigin origin,
        JBeamStructuralTriangleType triangle_type,
        JBeamStructuralTriangle& triangle)
    {
        triangle.provenance =
            ProvenanceWithSpan(part_provenance, row.span);
        triangle.node_a_index = INVALID_INDEX;
        triangle.node_b_index = INVALID_INDEX;
        triangle.node_c_index = INVALID_INDEX;
        triangle.optional = optional;
        triangle.status = JBeamStructuralTriangleStatus::ENABLED;
        triangle.origin = origin;
        triangle.triangle_type = triangle_type;
        triangle.authored_row_index = entry_index;
        bool valid = ReadRequiredString(
            row, triangle.provenance, section, entry_index,
            first + ":", first, triangle.node_a);
        valid = ReadRequiredString(
            row, triangle.provenance, section, entry_index,
            second + ":", second, triangle.node_b) && valid;
        valid = ReadRequiredString(
            row, triangle.provenance, section, entry_index,
            third + ":", third, triangle.node_c) && valid;
        if (!valid)
        {
            return false;
        }
        if (m_resource_limit)
        {
            return false;
        }
        if (triangle.node_a == triangle.node_b ||
            triangle.node_a == triangle.node_c ||
            triangle.node_b == triangle.node_c)
        {
            // A triangle that names one node twice has no area and no
            // normal, so it carries no pressure, drag or lift and collides
            // as nothing: it is the limiting case of the zero-area surface
            // below, which real-world content already skips with a
            // warning. The Germanwings A319 has one in its fuselage. Drop
            // the row and say so; no triangle is invented in its place. A
            // quad never reaches here with a repeated node, because its
            // collapsed half is not built at all.
            Push(
                JBeamStructuralDiagnosticCode::DUPLICATE_VERTEX,
                JBeamStructuralSeverity::WARNING,
                triangle.provenance,
                section,
                entry_index,
                std::string(),
                "Triangle names one node twice; it has no area and the "
                "row is skipped");
            return false;
        }
        bool references_valid = true;
        if (!ResolveNode(
                triangle.node_a, triangle.node_a_index))
        {
            references_valid = false;
        }
        if (!ResolveNode(
                triangle.node_b, triangle.node_b_index))
        {
            references_valid = false;
        }
        if (!ResolveNode(
                triangle.node_c, triangle.node_c_index))
        {
            references_valid = false;
        }
        if (!references_valid)
        {
            if (triangle.optional)
            {
                triangle.status =
                    JBeamStructuralTriangleStatus::
                        PRESERVED_DISABLED_OPTIONAL_REFERENCE;
                Push(
                    JBeamStructuralDiagnosticCode::
                        OPTIONAL_SURFACE_SKIPPED,
                    JBeamStructuralSeverity::WARNING,
                    triangle.provenance,
                    section,
                    entry_index,
                    std::string(),
                    "Optional surface with a missing node reference "
                    "is preserved but disabled");
            }
            else
            {
                Push(
                    JBeamStructuralDiagnosticCode::
                        MISSING_NODE_REFERENCE,
                    JBeamStructuralSeverity::WARNING,
                    triangle.provenance,
                    section,
                    entry_index,
                    std::string(),
                    "Triangle references an unknown node and is skipped");
                return false;
            }
        }
        if (references_valid && IsDegenerateTriangle(
                triangle.node_a_index,
                triangle.node_b_index,
                triangle.node_c_index))
        {
            // Degenerate surfaces in real-world content are skipped with
            // a warning; a zero-area cab cannot produce a contact plane.
            Push(
                JBeamStructuralDiagnosticCode::DEGENERATE_TRIANGLE,
                JBeamStructuralSeverity::WARNING,
                triangle.provenance,
                section,
                entry_index,
                std::string(),
                "Triangle geometry is degenerate; the row is skipped");
            return false;
        }
        if (m_resource_limit)
        {
            return false;
        }
        std::size_t retained = 0U;
        if (!AddSize(triangle.node_a.size(), retained) ||
            !AddSize(triangle.node_b.size(), retained) ||
            !AddSize(triangle.node_c.size(), retained))
        {
            RejectRetained();
            return false;
        }
        if (!ReserveRetained(retained))
        {
            return false;
        }
        if (!ReadGroupName(row, triangle.provenance, entry_index,
                section.c_str(), "breakGroup", triangle.break_group))
        {
            return false;
        }
        // Retained, never consumed: see JBeamStructuralTriangle.
        if (!ReadOptionalString(row, triangle.provenance, section,
                entry_index, "groundModel", triangle.ground_model))
        {
            return false;
        }
        PreserveTriangleAerodynamics(row, section, entry_index, triangle);
        return true;
    }

    void ProcessTriangles()
    {
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            JBeamNormalizeResult normalized;
            const JBeamNormalizedTable* table = NormalizeSection(
                m_sections[part_index], "triangles", normalized);
            if (m_resource_limit)
            {
                return;
            }
            if (table == NULL)
            {
                continue;
            }
            PreserveUnsupportedDefaults(
                *table,
                m_sections[part_index].part.provenance,
                "triangles");
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t entry_index = 0U;
                 entry_index < table->entries.size();
                 ++entry_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const JBeamNormalizedTableEntry& entry =
                    table->entries[entry_index];
                if (entry.kind ==
                    JBeamNormalizedTableEntryKind::INVALID_ENTRY)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        std::string(),
                        "Triangle table entry is not a data row or default");
                    continue;
                }
                if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DATA_ROW)
                {
                    continue;
                }
                PreserveUnsupportedAssignments(
                    *table,
                    entry.data_row,
                    m_sections[part_index].part.provenance,
                    "triangles",
                    entry_index);
                if (m_resource_limit)
                {
                    return;
                }
                if (m_result.triangles.size() >=
                    m_limits.max_triangles)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::TRIANGLE_LIMIT,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        std::string(),
                        "Triangle tables exceed the configured limit");
                    m_resource_limit = true;
                    return;
                }
                bool optional = false;
                JBeamStructuralTriangleType triangle_type =
                    JBeamStructuralTriangleType::NORMALTYPE;
                JBeamStructuralTriangleBuoyancy buoyancy =
                    JBeamStructuralTriangleBuoyancy::NONE;
                if (!ReadOptionalBoolean(
                        entry.data_row,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        "optional",
                        false,
                        optional) ||
                    !ReadTriangleType(
                        entry.data_row,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        triangle_type) ||
                    !ReadTriangleBuoyancy(
                        entry.data_row,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        buoyancy))
                {
                    continue;
                }
                JBeamStructuralTriangle triangle;
                if (BuildTriangle(
                        entry.data_row,
                        m_sections[part_index].part.provenance,
                        "triangles",
                        entry_index,
                        "id1",
                        "id2",
                        "id3",
                        optional,
                        JBeamStructuralTriangleOrigin::TRIANGLE,
                        triangle_type,
                        triangle))
                {
                    triangle.buoyancy = buoyancy;
                    m_result.triangles.push_back(triangle);
                }
            }
        }
    }

    void ProcessQuads()
    {
        for (std::size_t part_index = 0U;
             part_index < m_sections.size();
             ++part_index)
        {
            JBeamNormalizeResult normalized;
            const JBeamNormalizedTable* table = NormalizeSection(
                m_sections[part_index], "quads", normalized);
            if (m_resource_limit)
            {
                return;
            }
            if (table == NULL)
            {
                continue;
            }
            PreserveUnsupportedDefaults(
                *table,
                m_sections[part_index].part.provenance,
                "quads");
            if (m_resource_limit)
            {
                return;
            }
            for (std::size_t entry_index = 0U;
                 entry_index < table->entries.size();
                 ++entry_index)
            {
                if (m_resource_limit)
                {
                    return;
                }
                const JBeamNormalizedTableEntry& entry =
                    table->entries[entry_index];
                if (entry.kind ==
                    JBeamNormalizedTableEntryKind::INVALID_ENTRY)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "quads",
                        entry_index,
                        std::string(),
                        "Quad table entry is not a data row or default");
                    continue;
                }
                if (entry.kind !=
                    JBeamNormalizedTableEntryKind::DATA_ROW)
                {
                    continue;
                }
                PreserveUnsupportedAssignments(
                    *table,
                    entry.data_row,
                    m_sections[part_index].part.provenance,
                    "quads",
                    entry_index);
                if (m_resource_limit)
                {
                    return;
                }
                if (m_limits.max_triangles -
                        std::min(
                            m_result.triangles.size(),
                            m_limits.max_triangles) < 2U)
                {
                    Push(
                        JBeamStructuralDiagnosticCode::TRIANGLE_LIMIT,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        m_sections[part_index].part.provenance,
                        "quads",
                        entry_index,
                        std::string(),
                        "Quad requires two triangle slots");
                    m_resource_limit = true;
                    return;
                }
                std::string id1;
                std::string id2;
                std::string id3;
                std::string id4;
                const JBeamStructuralProvenance provenance =
                    m_sections[part_index].part.provenance;
                bool valid = ReadRequiredString(
                    entry.data_row, provenance, "quads", entry_index,
                    "id1:", "id1", id1);
                valid = ReadRequiredString(
                    entry.data_row, provenance, "quads", entry_index,
                    "id2:", "id2", id2) && valid;
                valid = ReadRequiredString(
                    entry.data_row, provenance, "quads", entry_index,
                    "id3:", "id3", id3) && valid;
                valid = ReadRequiredString(
                    entry.data_row, provenance, "quads", entry_index,
                    "id4:", "id4", id4) && valid;
                bool optional = false;
                JBeamStructuralTriangleType triangle_type =
                    JBeamStructuralTriangleType::NORMALTYPE;
                valid = ReadOptionalBoolean(
                    entry.data_row,
                    provenance,
                    "quads",
                    entry_index,
                    "optional",
                    false,
                    optional) && valid;
                valid = ReadTriangleType(
                    entry.data_row,
                    provenance,
                    "quads",
                    entry_index,
                    triangle_type) && valid;
                JBeamStructuralTriangleBuoyancy buoyancy =
                    JBeamStructuralTriangleBuoyancy::NONE;
                valid = ReadTriangleBuoyancy(
                    entry.data_row,
                    provenance,
                    "quads",
                    entry_index,
                    buoyancy) && valid;
                if (!valid)
                {
                    continue;
                }
                std::set<std::string> distinct;
                distinct.insert(id1);
                distinct.insert(id2);
                distinct.insert(id3);
                distinct.insert(id4);
                // A quad that repeats one vertex is how BeamNG content
                // spells a triangle in a quads table: the fixed diagonal
                // collapses one of the two triangles to zero area and leaves
                // the other. The J35D Gripen does it three times, and
                // refusing it rejected the whole vehicle. Emit the surviving
                // triangle and drop the collapsed one.
                //
                // Fewer than three distinct vertices is different: that quad
                // is a line or a point and describes no surface at all, so it
                // is still refused rather than silently contributing nothing.
                if (distinct.size() < 3U)
                {
                    JBeamStructuralProvenance row_provenance =
                        ProvenanceWithSpan(
                            provenance, entry.data_row.span);
                    Push(
                        JBeamStructuralDiagnosticCode::DUPLICATE_VERTEX,
                        JBeamStructuralSeverity::ERROR_SEVERITY,
                        row_provenance,
                        "quads",
                        entry_index,
                        std::string(),
                        "Quad vertices must span at least a triangle");
                    continue;
                }
                const bool first_collapses =
                    id1 == id2 || id2 == id3 || id1 == id3;
                const bool second_collapses =
                    id1 == id3 || id3 == id4 || id1 == id4;

                // Documented fixed diagonal: (id1,id2,id3) and
                // (id1,id3,id4), preserving authored winding.
                JBeamStructuralTriangle first;
                JBeamStructuralTriangle second;
                // A collapsed triangle is never built: BuildTriangle refuses
                // repeated vertices in its own right, so building it would
                // reject the row the collapse is supposed to make admissible.
                bool first_valid = true;
                bool second_valid = true;
                if (!first_collapses)
                {
                    first_valid = BuildTriangle(
                        entry.data_row, provenance, "quads", entry_index,
                        "id1", "id2", "id3",
                        optional,
                        JBeamStructuralTriangleOrigin::QUAD_FIRST,
                        triangle_type,
                        first);
                }
                if (!second_collapses)
                {
                    second_valid = BuildTriangle(
                        entry.data_row, provenance, "quads", entry_index,
                        "id1", "id3", "id4",
                        optional,
                        JBeamStructuralTriangleOrigin::QUAD_SECOND,
                        triangle_type,
                        second);
                }
                if (m_resource_limit)
                {
                    return;
                }
                if (first_valid && second_valid)
                {
                    if (first.status ==
                            JBeamStructuralTriangleStatus::
                                PRESERVED_DISABLED_OPTIONAL_REFERENCE ||
                        second.status ==
                            JBeamStructuralTriangleStatus::
                                PRESERVED_DISABLED_OPTIONAL_REFERENCE)
                    {
                        first.status =
                            JBeamStructuralTriangleStatus::
                                PRESERVED_DISABLED_OPTIONAL_REFERENCE;
                        second.status =
                            JBeamStructuralTriangleStatus::
                                PRESERVED_DISABLED_OPTIONAL_REFERENCE;
                        first.aerodynamics.area_source = second.aerodynamics.area_source =
                            JBeamAeroAreaSource::UNAVAILABLE_OPTIONAL_NODE_REFERENCE;
                    }
                    first.buoyancy = buoyancy;
                    second.buoyancy = buoyancy;
                    if (!first_collapses)
                    {
                        m_result.triangles.push_back(first);
                    }
                    if (!second_collapses)
                    {
                        m_result.triangles.push_back(second);
                    }
                }
            }
        }
    }
};

class BoundedStringBuffer : public std::streambuf
{
public:
    explicit BoundedStringBuffer(std::size_t limit)
        : m_limit(limit)
        , m_exceeded(false)
    {
    }

    const std::string& Value() const
    {
        return m_value;
    }

    bool Exceeded() const
    {
        return m_exceeded;
    }

protected:
    virtual std::streamsize xsputn(
        const char* value,
        std::streamsize count)
    {
        if (count < 0)
        {
            m_exceeded = true;
            return 0;
        }
        const std::size_t unsigned_count =
            static_cast<std::size_t>(count);
        if (unsigned_count >
            m_limit - std::min(m_value.size(), m_limit))
        {
            m_exceeded = true;
            return 0;
        }
        m_value.append(value, unsigned_count);
        return count;
    }

    virtual int_type overflow(int_type character)
    {
        if (traits_type::eq_int_type(
                character, traits_type::eof()))
        {
            return traits_type::not_eof(character);
        }
        if (m_value.size() >= m_limit)
        {
            m_exceeded = true;
            return traits_type::eof();
        }
        m_value.push_back(
            traits_type::to_char_type(character));
        return character;
    }

private:
    std::size_t m_limit;
    bool m_exceeded;
    std::string m_value;
};

struct LengthPrefixedView
{
    const std::string* value;
};

LengthPrefixedView LengthPrefixed(const std::string& value)
{
    LengthPrefixedView view;
    view.value = &value;
    return view;
}

std::ostream& operator<<(
    std::ostream& output,
    const LengthPrefixedView& view)
{
    // Stream directly into the caller's bounded buffer. Returning a complete
    // std::string here would allocate an unbounded second copy before the
    // BoundedStringBuffer could reject it.
    output << view.value->size() << ':';
    output << *view.value;
    return output;
}

void AppendProvenance(
    const JBeamStructuralProvenance& provenance,
    std::ostream& output)
{
    output
        << provenance.PartPreorderIndex() << '\t'
        << LengthPrefixed(provenance.PartName()) << '\t'
        << LengthPrefixed(provenance.PackagePath()) << '\t'
        << LengthPrefixed(provenance.SourceName()) << '\t'
        << provenance.begin.byte_offset << '\t'
        << provenance.begin.line << '\t'
        << provenance.begin.column << '\t'
        << provenance.end.byte_offset << '\t'
        << provenance.end.line << '\t'
        << provenance.end.column;
}

bool AppendCanonicalValue(
    const JBeamValue& value,
    std::ostream& output,
    std::set<const JBeamValue*>& active,
    std::size_t depth,
    std::size_t& work_units,
    std::size_t max_work_units)
{
    static const std::size_t MAX_CANONICAL_VALUE_DEPTH = 256U;
    if (depth > MAX_CANONICAL_VALUE_DEPTH ||
        work_units >= max_work_units ||
        !active.insert(&value).second)
    {
        return false;
    }
    ++work_units;
    bool valid = true;
    switch (value.type)
    {
    case JBeamValueType::NULL_VALUE:
        output << 'n';
        break;
    case JBeamValueType::BOOLEAN:
        output << (value.boolean_value ? "b1" : "b0");
        break;
    case JBeamValueType::NUMBER:
        output << 'd' << LengthPrefixed(value.scalar_text);
        break;
    case JBeamValueType::STRING:
        output << 's' << LengthPrefixed(value.scalar_text);
        break;
    case JBeamValueType::ARRAY:
        output << 'a' << value.array_values.size() << '[';
        for (std::size_t i = 0U;
             valid && i < value.array_values.size();
             ++i)
        {
            valid = AppendCanonicalValue(
                value.array_values[i],
                output,
                active,
                depth + 1U,
                work_units,
                max_work_units);
            valid = valid && static_cast<bool>(output);
        }
        output << ']';
        break;
    case JBeamValueType::OBJECT:
        output << 'o' << value.object_fields.size() << '{';
        for (std::size_t i = 0U;
             valid && i < value.object_fields.size();
             ++i)
        {
            output << 'k'
                << LengthPrefixed(value.object_fields[i].key);
            if (value.object_fields[i].value)
            {
                valid = AppendCanonicalValue(
                    *value.object_fields[i].value,
                    output,
                    active,
                    depth + 1U,
                    work_units,
                    max_work_units);
            }
            else
            {
                output << 'x';
            }
            valid = valid && static_cast<bool>(output);
        }
        output << '}';
        break;
    }
    active.erase(&value);
    return valid && static_cast<bool>(output);
}

void AppendDouble(double value, std::ostream& output)
{
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    // Canonicalize only the two exact IEEE zero encodings. A floating-point
    // equality under fast math can also flush a nonzero subnormal to zero.
    if ((bits & UINT64_C(0x7fffffffffffffff)) == 0U) bits = 0U;
    const std::ios_base::fmtflags previous = output.flags();
    const char previous_fill = output.fill();
    output << std::hex << std::setw(16) << std::setfill('0') << bits;
    output.flags(previous);
    output.fill(previous_fill);
}

bool AppendAerodynamicScalar(const JBeamAeroScalar& scalar,
    std::ostream& output, std::size_t& work, std::size_t max_work)
{
    if (work >= max_work || !IsFiniteDouble(scalar.value) ||
        !IsFiniteDouble(scalar.effective_value) ||
        static_cast<unsigned>(scalar.origin) >
            static_cast<unsigned>(JBeamAeroScalarOrigin::DEFAULT_FROM_DRAG) ||
        (scalar.effective_resolved && !scalar.resolved)) return false;
    ++work;
    output << static_cast<int>(scalar.origin) << ':' << scalar.resolved << ':';
    AppendDouble(scalar.value, output);
    output << ':' << scalar.effective_resolved << ':';
    AppendDouble(scalar.effective_value, output);
    output << ':';
    AppendProvenance(scalar.provenance, output);
    output << ':' << static_cast<bool>(scalar.authored_value) << ':';
    if (scalar.authored_value)
    {
        std::set<const JBeamValue*> active;
        if (!AppendCanonicalValue(*scalar.authored_value, output, active,
                0U, work, max_work)) return false;
    }
    return static_cast<bool>(output);
}

} // namespace

JBeamStructuralPartIdentity::JBeamStructuralPartIdentity()
    : part_preorder_index(0U)
{
}

JBeamStructuralProvenance::JBeamStructuralProvenance()
{
}

JBeamAeroScalar::JBeamAeroScalar()
    : origin(JBeamAeroScalarOrigin::DOCUMENTED_DEFAULT)
    , resolved(false)
    , value(0.0)
    , effective_resolved(false)
    , effective_value(0.0)
{
}

JBeamTriangleAerodynamics::JBeamTriangleAerodynamics()
    : capability(JBeamAeroCapability::UNRESOLVED_PARAMETERS_NOT_APPLIED)
    , area_source(JBeamAeroAreaSource::CURRENT_REFERENCED_NODE_POSITIONS)
{
}

std::size_t JBeamStructuralProvenance::PartPreorderIndex() const
{
    return part ? part->part_preorder_index : 0U;
}

const std::string& JBeamStructuralProvenance::PartName() const
{
    static const std::string empty;
    return part ? part->part_name : empty;
}

const std::string& JBeamStructuralProvenance::PackagePath() const
{
    static const std::string empty;
    return part ? part->package_path : empty;
}

const std::string& JBeamStructuralProvenance::SourceName() const
{
    static const std::string empty;
    return source_name ? *source_name : empty;
}

JBeamSourceSpan JBeamStructuralProvenance::SourceSpan() const
{
    JBeamSourceSpan span;
    span.source_name = SourceName();
    span.begin = begin;
    span.end = end;
    return span;
}

JBeamStructuralDiagnostic::JBeamStructuralDiagnostic()
    : code(JBeamStructuralDiagnosticCode::INVALID_RESOLVED_GRAPH)
    , severity(JBeamStructuralSeverity::ERROR_SEVERITY)
    , row_index(0U)
    , has_preserved_value(false)
{
}

JBeamStructuralLimits::JBeamStructuralLimits()
    : max_parts(1024U)
    , max_rows(1000000U)
    , max_nodes(250000U)
    , max_beams(1000000U)
    , max_triangles(1000000U)
    , max_diagnostics(4096U)
    , max_retained_bytes(64U * 1024U * 1024U)
    , max_preserved_value_work_units(1000000U)
    , max_preserved_value_depth(256U)
    , max_expression_evaluations(1000000U)
    , max_expression_work_units(268435456U)
    , max_component_nodes(16384U)
    , max_component_depth(64U)
    , max_canonical_output_bytes(256U * 1024U * 1024U)
    , max_canonical_work_units(4000000U)
{
}

JBeamStructuralBeam::JBeamStructuralBeam()
    : node_a_index(INVALID_INDEX)
    , node_b_index(INVALID_INDEX)
    , optional(false)
    , status(JBeamStructuralBeamStatus::ENABLED)
    , jbeam_zero_rest_spring(false)
    , has_spring(false)
    , spring(0.0)
    , has_damping(false)
    , damping(0.0)
    , has_deform(false)
    , deform_unbounded(false)
    , deform(0.0)
    , has_strength(false)
    , strength_unbounded(false)
    , strength(0.0)
    , has_deform_limit_expansion(false)
    , deform_limit_expansion(0.0)
    , has_deformation_trigger_ratio(false)
    , deformation_trigger_ratio(0.0)
    , has_precompression(false)
    , precompression(0.0)
    , has_precompression_range(false)
    , precompression_range(0.0)
    , has_precompression_time(false)
    , precompression_time(0.0)
    , has_long_bound(false)
    , long_bound(1.0)
    , has_short_bound(false)
    , short_bound(1.0)
    , has_long_bound_range(false)
    , long_bound_range(0.0)
    , has_short_bound_range(false)
    , short_bound_range(0.0)
    , has_bound_zone(false)
    , bound_zone(1.0)
    , has_limit_spring(false)
    , limit_spring(1.0)
    , has_limit_damping(false)
    , limit_damping(1.0)
{
}

bool HasSupportedJBeamZeroRestSpringParameters(
    const JBeamStructuralBeam& beam) noexcept
{
    // A finite positive strength is the force at which the spring breaks
    // (BeamNG's `beamStrength`). A rest length of zero has no plastic
    // reading, so a finite deform BELOW that strength is still this role:
    // the lowering keeps the spring elastic to its authored break and names
    // the unmodelled yield (JBeamZeroRestSpring.h). Skipping such a row
    // instead deleted load-bearing structure: the A319 hangs each wing on
    // three of them (deform 120 kN, strength 1.6 MN), and without them both
    // wings folded onto the ground at spawn.
    const bool supported_strength = beam.has_strength &&
        (beam.strength_unbounded ||
            IsFinitePositiveDouble(beam.strength));
    const bool supported_deform = beam.has_deform &&
        (beam.deform_unbounded || IsFinitePositiveDouble(beam.deform));
    return beam.status == JBeamStructuralBeamStatus::ENABLED &&
        beam.beam_type == "NORMAL" &&
        supported_strength && supported_deform &&
        IsFiniteDouble(beam.deform) && IsFiniteDouble(beam.strength) &&
        (!beam.has_spring || IsFiniteNonNegativeDouble(beam.spring)) &&
        (!beam.has_damping || IsFiniteNonNegativeDouble(beam.damping)) &&
        (!beam.has_precompression ||
            (IsFiniteDouble(beam.precompression) && beam.precompression == 1.0)) &&
        !beam.has_precompression_time &&
        (!beam.has_precompression_range ||
            (IsFiniteDouble(beam.precompression_range) &&
             !IsBinary64SubnormalMagnitude(beam.precompression_range) &&
             beam.precompression_range == 0.0)) &&
        (!beam.has_long_bound ||
            (IsFiniteDouble(beam.long_bound) && beam.long_bound == 1.0)) &&
        (!beam.has_short_bound ||
            (IsFiniteDouble(beam.short_bound) && beam.short_bound == 1.0)) &&
        !beam.has_long_bound_range && !beam.has_short_bound_range &&
        !beam.has_bound_zone && !beam.has_limit_spring && !beam.has_limit_damping;
}

bool AreJBeamNodePositionsExactlyCoincident(
    const JBeamStructuralNode& first,
    const JBeamStructuralNode& second) noexcept
{
    const double a[] = {first.x, first.y, first.z};
    const double b[] = {second.x, second.y, second.z};
    for (std::size_t i = 0U; i < 3U; ++i)
    {
        if (!IsFiniteDouble(a[i]) || !IsFiniteDouble(b[i])) return false;
        std::uint64_t bits_a = 0U, bits_b = 0U;
        std::memcpy(&bits_a, &a[i], sizeof(bits_a));
        std::memcpy(&bits_b, &b[i], sizeof(bits_b));
        if (bits_a != bits_b &&
            ((bits_a | bits_b) & UINT64_C(0x7fffffffffffffff)) != 0U)
            return false;
    }
    return true;
}

JBeamStructuralNode::JBeamStructuralNode()
    : x(0.0)
    , y(0.0)
    , z(0.0)
    , node_weight(25.0)
    , node_weight_authored(false)
    , friction_coef(1.0)
    , friction_coef_authored(false)
    , sliding_friction_coef(1.0)
    , sliding_friction_coef_authored(false)
    , collision(true)
    , self_collision(false)
    , static_collision(true)
    , fixed(false)
{
}

JBeamStructuralTriangle::JBeamStructuralTriangle()
    : node_a_index(INVALID_INDEX)
    , node_b_index(INVALID_INDEX)
    , node_c_index(INVALID_INDEX)
    , optional(false)
    , status(JBeamStructuralTriangleStatus::ENABLED)
    , origin(JBeamStructuralTriangleOrigin::TRIANGLE)
    , triangle_type(JBeamStructuralTriangleType::NORMALTYPE)
    , buoyancy(JBeamStructuralTriangleBuoyancy::NONE)
    , authored_row_index(0U)
{
}

JBeamStructuralIR::JBeamStructuralIR()
    : has_ref_frame(false)
    , authored_row_count(0U)
    , retained_byte_count(0U)
    , canonical_output_byte_limit(256U * 1024U * 1024U)
    , canonical_work_unit_limit(4000000U)
{
    ref_frame.reference_index = INVALID_INDEX;
    ref_frame.back_index = INVALID_INDEX;
    ref_frame.left_index = INVALID_INDEX;
    ref_frame.up_index = INVALID_INDEX;
    ref_frame.left_corner_index = INVALID_INDEX;
    ref_frame.right_corner_index = INVALID_INDEX;
}

bool JBeamStructuralIR::IsValid() const
{
    return has_ref_frame && !HasErrorsOrUnknownSeverity(diagnostics);
}

bool EvaluateJBeamStructuralPartNumber(
    const JBeamStructuralIR& ir,
    std::size_t part_index,
    const JBeamValue& value,
    double& output,
    std::string* failure)
{
    const auto refuse = [failure](const std::string& reason)
    {
        if (failure != NULL)
        {
            *failure = reason;
        }
        return false;
    };
    if (value.type == JBeamValueType::NUMBER)
    {
        if (!IsFiniteDouble(value.number_value))
        {
            return refuse("non-finite number");
        }
        output = value.number_value == 0.0 ? 0.0 : value.number_value;
        return true;
    }
    if (!IsExpressionString(value) ||
        value.scalar_text.compare(0U, 2U, "$.") == 0)
    {
        return refuse("neither a number nor a '$' expression");
    }
    if (part_index >= ir.parts.size() ||
        !ir.parts[part_index].expression_variables)
    {
        return refuse("no structural expression scope for this part");
    }
    const std::vector<JBeamExpressionVariable>& own =
        *ir.parts[part_index].expression_variables;
    JBeamExpressionEnvironment environment;
    environment.variables.reserve(
        ir.expression_components.size() + own.size());
    environment.variables.insert(
        environment.variables.end(),
        ir.expression_components.begin(),
        ir.expression_components.end());
    environment.variables.insert(
        environment.variables.end(), own.begin(), own.end());
    const std::string expression =
        value.scalar_text.compare(0U, 2U, "$=") == 0
            ? value.scalar_text
            : std::string("$=") + value.scalar_text;
    const JBeamExpressionResult result = EvaluateJBeamExpression(
        expression, environment, JBeamStructuralLimits().expression_limits);
    if (!result.IsValid() || !result.has_value)
    {
        if (!result.undeclared_variables.empty())
        {
            return refuse(
                "variable '" + result.undeclared_variables[0] +
                "' is not declared");
        }
        return refuse(
            result.diagnostics.empty()
                ? std::string("expression did not evaluate")
                : result.diagnostics[0].message);
    }
    if (result.value.type != JBeamExpressionValueType::NUMBER ||
        !IsFiniteDouble(result.value.number_value))
    {
        return refuse("expression does not evaluate to a finite number");
    }
    output = result.value.number_value == 0.0
        ? 0.0 : result.value.number_value;
    return true;
}

JBeamStructuralIR BuildJBeamStructuralIR(
    const JBeamResolvedGraph& graph,
    const JBeamStructuralLimits& limits)
{
    return StructuralBuilder(graph, limits).Run();
}

std::string SerializeCanonicalJBeamStructuralIR(
    const JBeamStructuralIR& ir)
{
    if (ir.canonical_output_byte_limit == 0U ||
        ir.canonical_work_unit_limit == 0U)
    {
        return std::string();
    }
    std::size_t canonical_work = 1U;
    if (!AddSize(ir.parts.size(), canonical_work) ||
        !AddSize(ir.nodes.size(), canonical_work) ||
        !AddSize(ir.beams.size(), canonical_work) ||
        !AddSize(ir.triangles.size(), canonical_work) ||
        !AddSize(ir.aerodynamic_scales.size(), canonical_work) ||
        !AddSize(ir.diagnostics.size(), canonical_work) ||
        canonical_work > ir.canonical_work_unit_limit)
    {
        return std::string();
    }
    BoundedStringBuffer buffer(ir.canonical_output_byte_limit);
    std::ostream output(&buffer);
    output.imbue(std::locale::classic());
    // v10 binds the explicit, non-welding zero-rest spring admission role.
    // v13 retains deformGroup and deformationTriggerRatio: two vehicles that
    // differ only in which beams trigger which deformation group are not the
    // same simulation and must not share a canonical digest.
    // v14 binds effective structural-node static/sliding friction and their
    // authored presence independently; a single native multiplier is not an
    // implicit claim that distinct authored coefficients are equivalent.
    // v15 resolves the part-level aerodynamic scale against the documented
    // mechanism: a scale multiplies the AUTHORED modifiers, a defaulted
    // coefficient keeps its default unscaled, and a defaulted liftCoef
    // follows the scaled drag. A scaled vehicle therefore carries different
    // effective coefficients than v13's refusal recorded. Written first as
    // v14 and renumbered because the recovery line's own v14 means authored
    // node-friction retention, and two schemas must never claim one version.
    // v16 retains a triangle's `breakGroup`: two vehicles differing only in
    // which triangles share a break group stop producing aerodynamic force at
    // different moments, so they are not the same simulation and must not
    // share a canonical digest.
    // v17 is the union of v14, v15 and v16, which reached this tree down
    // separate lines. A vehicle read with node friction retained, with the
    // aero scale applied to authored modifiers only, and with triangle break
    // groups retained is not the same simulation as one read under any of
    // those alone, so the merged tree takes its own version rather than
    // inheriting either side's.
    // v18: the four progressive damping rates entered the digest above, so a
    // v17 digest no longer describes the same simulation this build runs.
    output << "ror-beamng-structural-ir-v19\n";
    output << "parts\t" << ir.parts.size() << '\n';
    for (std::size_t i = 0U;
         output && i < ir.parts.size();
         ++i)
    {
        output << "P\t";
        AppendProvenance(ir.parts[i].provenance, output);
        for (double component : ir.parts[i].slot_node_offset)
        {
            if (!IsFiniteDouble(component)) return std::string();
            output << '\t';
            AppendDouble(component, output);
        }
        output << '\n';
    }
    if (!output)
    {
        return std::string();
    }
    output << "rows\t" << ir.authored_row_count << '\n';
    output << "nodes\t" << ir.nodes.size() << '\n';
    for (std::size_t i = 0U;
         output && i < ir.nodes.size();
         ++i)
    {
        const JBeamStructuralNode& node = ir.nodes[i];
        if (!IsFiniteNonNegativeDouble(node.friction_coef) ||
            !IsFiniteNonNegativeDouble(node.sliding_friction_coef))
        {
            return std::string();
        }
        output << "N\t" << LengthPrefixed(node.id) << '\t';
        AppendDouble(node.x, output);
        output << '\t';
        AppendDouble(node.y, output);
        output << '\t';
        AppendDouble(node.z, output);
        output << '\t';
        AppendDouble(node.node_weight, output);
        output << '\t' << (node.node_weight_authored ? 1 : 0) << '\t';
        AppendDouble(node.friction_coef, output);
        output << '\t' << (node.friction_coef_authored ? 1 : 0) << '\t';
        AppendDouble(node.sliding_friction_coef, output);
        output << '\t' << (node.sliding_friction_coef_authored ? 1 : 0) << '\t'
               << (node.collision ? 1 : 0) << '\t'
               << (node.self_collision ? 1 : 0) << '\t'
               << (node.static_collision ? 1 : 0) << '\t';
        AppendProvenance(node.provenance, output);
        // Only a pinned node adds a column, so every structure authored
        // before `fixed` was admitted keeps its canonical identity.
        if (node.fixed)
        {
            output << "\tfixed";
        }
        output << '\n';
    }
    if (!output)
    {
        return std::string();
    }
    output << "beams\t" << ir.beams.size() << '\n';
    for (std::size_t i = 0U;
         output && i < ir.beams.size();
         ++i)
    {
        const JBeamStructuralBeam& beam = ir.beams[i];
        output
            << "B\t" << LengthPrefixed(beam.node_a) << '\t'
            << LengthPrefixed(beam.node_b) << '\t'
            << beam.node_a_index << '\t'
            << beam.node_b_index << '\t'
            << LengthPrefixed(beam.beam_type) << '\t'
            << LengthPrefixed(beam.break_group) << '\t'
            << LengthPrefixed(beam.deform_group) << '\t'
            << (beam.optional ? 1 : 0) << '\t'
            << static_cast<int>(beam.status) << '\t'
            << (beam.jbeam_zero_rest_spring ? 1 : 0) << '\t';
        const bool flags[] = {
            beam.has_spring,
            beam.has_damping,
            beam.has_deform,
            beam.has_strength,
            beam.has_precompression,
            beam.has_precompression_range,
            beam.has_precompression_time,
            beam.has_long_bound,
            beam.has_short_bound,
            beam.has_long_bound_range,
            beam.has_short_bound_range,
            beam.has_bound_zone,
            beam.has_limit_spring,
            beam.has_limit_damping,
            beam.has_deform_limit_expansion,
            beam.has_deformation_trigger_ratio,
            beam.has_damp_fast,
            beam.has_damp_rebound,
            beam.has_damp_rebound_fast,
            beam.has_damp_velocity_split
        };
        const double values[] = {
            beam.spring,
            beam.damping,
            beam.deform,
            beam.strength,
            beam.precompression,
            beam.precompression_range,
            beam.precompression_time,
            beam.long_bound,
            beam.short_bound,
            beam.long_bound_range,
            beam.short_bound_range,
            beam.bound_zone,
            beam.limit_spring,
            beam.limit_damping,
            beam.deform_limit_expansion,
            beam.deformation_trigger_ratio,
            beam.damp_fast,
            beam.damp_rebound,
            beam.damp_rebound_fast,
            beam.damp_velocity_split
        };
        // 19 since BeamNG's progressive damping joined: the four
        // velocity-split rates now SELECT the damping coefficient at runtime
        // (ProgressiveBeamDamping), so two vehicles differing only in one of
        // them are no longer the same simulation and must not share a digest.
        // `dampCutoffHz` stays out for exactly the reason the other four came
        // in - nothing consumes it, so it cannot make two vehicles differ.
        for (std::size_t field = 0U; field < 19U; ++field)
        {
            output << (flags[field] ? 1 : 0) << ':';
            AppendDouble(values[field], output);
            output << '\t';
        }
        output
            << (beam.deform_unbounded ? 1 : 0) << '\t'
            << (beam.strength_unbounded ? 1 : 0) << '\t';
        AppendProvenance(beam.provenance, output);
        output << '\n';
    }
    if (!output)
    {
        return std::string();
    }
    output << "aerodynamic-scales\t" << ir.aerodynamic_scales.size() << '\n';
    for (std::size_t i = 0U; output && i < ir.aerodynamic_scales.size(); ++i)
    {
        const JBeamAeroScaleAssignment& scale = ir.aerodynamic_scales[i];
        if (static_cast<unsigned>(scale.parameter) >
                static_cast<unsigned>(JBeamAeroParameter::STALL_ANGLE_RADIANS)) return std::string();
        output << "AS\t" << static_cast<int>(scale.parameter) << '\t';
        if (!AppendAerodynamicScalar(scale.scalar, output,
                canonical_work, ir.canonical_work_unit_limit)) return std::string();
        output << '\n';
    }
    output << "aerodynamic-policy\tpreserved-not-applied;percent-flat-plate-not-physical-Cd-Cl;"
        "authored-ccw-front-both-sides;current-node-area;"
        "part-scale-multiplies-authored-modifiers;default-lift-follows-scaled-drag;"
        "normalized-table-local-scope;cross-part-scope-leakage-unbound;"
        "runtime-break-lifecycle-and-force-law-unbound\n";
    output << "triangles\t" << ir.triangles.size() << '\n';
    for (std::size_t i = 0U;
         output && i < ir.triangles.size();
         ++i)
    {
        const JBeamStructuralTriangle& triangle = ir.triangles[i];
        output
            << "T\t" << LengthPrefixed(triangle.node_a) << '\t'
            << LengthPrefixed(triangle.node_b) << '\t'
            << LengthPrefixed(triangle.node_c) << '\t'
            << triangle.node_a_index << '\t'
            << triangle.node_b_index << '\t'
            << triangle.node_c_index << '\t'
            << (triangle.optional ? 1 : 0) << '\t'
            << static_cast<int>(triangle.status) << '\t'
            << static_cast<int>(triangle.origin) << '\t'
            << static_cast<int>(triangle.triangle_type) << '\t'
            << triangle.authored_row_index << '\t'
            << LengthPrefixed(triangle.break_group) << '\t';
        AppendProvenance(triangle.provenance, output);
        const JBeamTriangleAerodynamics& aero = triangle.aerodynamics;
        const JBeamAeroScalar* const coefficients[] = {
            &aero.drag_percent, &aero.skin_drag_percent, &aero.lift_percent, &aero.stall_angle_rad
        };
        if (static_cast<unsigned>(aero.capability) >
                static_cast<unsigned>(JBeamAeroCapability::UNRESOLVED_PARAMETERS_NOT_APPLIED) ||
            static_cast<unsigned>(aero.area_source) >
                static_cast<unsigned>(JBeamAeroAreaSource::UNAVAILABLE_OPTIONAL_NODE_REFERENCE))
            return std::string();
        output << '\t' << static_cast<int>(aero.capability)
            << '\t' << static_cast<int>(aero.area_source);
        for (std::size_t c = 0U; c < 4U; ++c)
        {
            output << '\t';
            if (!AppendAerodynamicScalar(*coefficients[c], output,
                    canonical_work, ir.canonical_work_unit_limit)) return std::string();
        }
        output << '\n';
    }
    if (!output)
    {
        return std::string();
    }
    output << "ref-frame\t" << (ir.has_ref_frame ? 1 : 0);
    if (ir.has_ref_frame)
    {
        output
            << '\t' << LengthPrefixed(ir.ref_frame.reference)
            << '\t' << LengthPrefixed(ir.ref_frame.back)
            << '\t' << LengthPrefixed(ir.ref_frame.left)
            << '\t' << LengthPrefixed(ir.ref_frame.up)
            << '\t' << LengthPrefixed(ir.ref_frame.left_corner)
            << '\t' << LengthPrefixed(ir.ref_frame.right_corner)
            << '\t' << ir.ref_frame.reference_index
            << '\t' << ir.ref_frame.back_index
            << '\t' << ir.ref_frame.left_index
            << '\t' << ir.ref_frame.up_index
            << '\t' << ir.ref_frame.left_corner_index
            << '\t' << ir.ref_frame.right_corner_index << '\t';
        AppendProvenance(ir.ref_frame.provenance, output);
    }
    output << '\n';
    if (!output)
    {
        return std::string();
    }
    std::vector<std::size_t> diagnostic_order;
    diagnostic_order.reserve(ir.diagnostics.size());
    for (std::size_t i = 0U; i < ir.diagnostics.size(); ++i)
    {
        diagnostic_order.push_back(i);
    }
    std::sort(
        diagnostic_order.begin(),
        diagnostic_order.end(),
        [&ir](std::size_t left_index, std::size_t right_index)
        {
            const JBeamStructuralDiagnostic& left =
                ir.diagnostics[left_index];
            const JBeamStructuralDiagnostic& right =
                ir.diagnostics[right_index];
            if (DiagnosticLess(left, right))
            {
                return true;
            }
            if (DiagnosticLess(right, left))
            {
                return false;
            }
            return left_index < right_index;
        });
    output << "diagnostics\t" << diagnostic_order.size() << '\n';
    for (std::size_t i = 0U;
         output && i < diagnostic_order.size();
         ++i)
    {
        const JBeamStructuralDiagnostic& diagnostic =
            ir.diagnostics[diagnostic_order[i]];
        output
            << "D\t" << static_cast<int>(diagnostic.code) << '\t'
            << static_cast<int>(diagnostic.severity) << '\t';
        AppendProvenance(diagnostic.provenance, output);
        output
            << '\t' << LengthPrefixed(diagnostic.section)
            << '\t' << diagnostic.row_index
            << '\t' << LengthPrefixed(diagnostic.field_name)
            << '\t' << LengthPrefixed(diagnostic.detail)
            << '\t' << (diagnostic.has_preserved_value ? 1 : 0);
        if (diagnostic.has_preserved_value &&
            diagnostic.preserved_value)
        {
            output << '\t';
            std::set<const JBeamValue*> active_values;
            if (!AppendCanonicalValue(
                    *diagnostic.preserved_value,
                    output,
                    active_values,
                    0U,
                    canonical_work,
                    ir.canonical_work_unit_limit))
            {
                return std::string();
            }
        }
        output << '\n';
    }
    if (buffer.Exceeded() || !output)
    {
        return std::string();
    }
    return buffer.Value();
}

const char* JBeamStructuralDiagnosticCodeToString(
    JBeamStructuralDiagnosticCode code)
{
    switch (code)
    {
    case JBeamStructuralDiagnosticCode::INVALID_RESOLVED_GRAPH:
        return "invalid-resolved-graph";
    case JBeamStructuralDiagnosticCode::RESOLVED_PART_LIMIT:
        return "resolved-part-limit";
    case JBeamStructuralDiagnosticCode::ROW_LIMIT:
        return "row-limit";
    case JBeamStructuralDiagnosticCode::NODE_LIMIT:
        return "node-limit";
    case JBeamStructuralDiagnosticCode::BEAM_LIMIT:
        return "beam-limit";
    case JBeamStructuralDiagnosticCode::TRIANGLE_LIMIT:
        return "triangle-limit";
    case JBeamStructuralDiagnosticCode::DIAGNOSTIC_LIMIT:
        return "diagnostic-limit";
    case JBeamStructuralDiagnosticCode::RETAINED_BYTE_LIMIT:
        return "retained-byte-limit";
    case JBeamStructuralDiagnosticCode::PRESERVED_VALUE_LIMIT:
        return "preserved-value-limit";
    case JBeamStructuralDiagnosticCode::NORMALIZATION_ERROR:
        return "normalization-error";
    case JBeamStructuralDiagnosticCode::NORMALIZATION_WARNING:
        return "normalization-warning";
    case JBeamStructuralDiagnosticCode::PART_BODY_NOT_OBJECT:
        return "part-body-not-object";
    case JBeamStructuralDiagnosticCode::DUPLICATE_SECTION:
        return "duplicate-section";
    case JBeamStructuralDiagnosticCode::INVALID_SECTION:
        return "invalid-section";
    case JBeamStructuralDiagnosticCode::INVALID_TABLE_HEADER:
        return "invalid-table-header";
    case JBeamStructuralDiagnosticCode::DUPLICATE_TABLE_HEADER:
        return "duplicate-table-header";
    case JBeamStructuralDiagnosticCode::INVALID_TABLE_ROW:
        return "invalid-table-row";
    case JBeamStructuralDiagnosticCode::MISSING_REQUIRED_FIELD:
        return "missing-required-field";
    case JBeamStructuralDiagnosticCode::AMBIGUOUS_REQUIRED_FIELD:
        return "ambiguous-required-field";
    case JBeamStructuralDiagnosticCode::INVALID_FIELD_TYPE:
        return "invalid-field-type";
    case JBeamStructuralDiagnosticCode::EXPRESSION_ERROR:
        return "expression-error";
    case JBeamStructuralDiagnosticCode::EXPRESSION_LIMIT:
        return "expression-limit";
    case JBeamStructuralDiagnosticCode::EXPRESSION_DISABLED:
        return "expression-disabled";
    case JBeamStructuralDiagnosticCode::UNSUPPORTED_COMPONENT_VALUE:
        return "unsupported-component-value";
    case JBeamStructuralDiagnosticCode::INVALID_COMPONENT_PATH:
        return "invalid-component-path";
    case JBeamStructuralDiagnosticCode::INVALID_VARIABLE_VALUE:
        return "invalid-variable-value";
    case JBeamStructuralDiagnosticCode::NON_FINITE_NUMBER:
        return "non-finite-number";
    case JBeamStructuralDiagnosticCode::INVALID_NODE_WEIGHT:
        return "invalid-node-weight";
    case JBeamStructuralDiagnosticCode::INVALID_NODE_FRICTION:
        return "invalid-node-friction";
    case JBeamStructuralDiagnosticCode::INVALID_BEAM_PARAMETER:
        return "invalid-beam-parameter";
    case JBeamStructuralDiagnosticCode::DUPLICATE_NODE_ID:
        return "duplicate-node-id";
    case JBeamStructuralDiagnosticCode::MISSING_NODE_REFERENCE:
        return "missing-node-reference";
    case JBeamStructuralDiagnosticCode::OPTIONAL_BEAM_SKIPPED:
        return "optional-beam-skipped";
    case JBeamStructuralDiagnosticCode::OPTIONAL_SURFACE_SKIPPED:
        return "optional-surface-skipped";
    case JBeamStructuralDiagnosticCode::DUPLICATE_VERTEX:
        return "duplicate-vertex";
    case JBeamStructuralDiagnosticCode::DEGENERATE_BEAM:
        return "degenerate-beam";
    case JBeamStructuralDiagnosticCode::DEGENERATE_TRIANGLE:
        return "degenerate-triangle";
    case JBeamStructuralDiagnosticCode::UNSUPPORTED_TRIANGLE_TYPE:
        return "unsupported-triangle-type";
    case JBeamStructuralDiagnosticCode::SPECIAL_BEAM_TYPE_DISABLED:
        return "special-beam-type-disabled";
    case JBeamStructuralDiagnosticCode::UNKNOWN_SECTION:
        return "unknown-section";
    case JBeamStructuralDiagnosticCode::UNKNOWN_FIELD:
        return "unknown-field";
    case JBeamStructuralDiagnosticCode::UNSUPPORTED_FIELD:
        return "unsupported-field";
    case JBeamStructuralDiagnosticCode::MISSING_REF_NODES:
        return "missing-ref-nodes";
    case JBeamStructuralDiagnosticCode::DUPLICATE_REF_NODES:
        return "duplicate-ref-nodes";
    case JBeamStructuralDiagnosticCode::DEGENERATE_REF_NODES:
        return "degenerate-ref-nodes";
    case JBeamStructuralDiagnosticCode::MISALIGNED_REF_NODES:
        return "misaligned-ref-nodes";
    case JBeamStructuralDiagnosticCode::MISALIGNED_REF_CORNERS:
        return "misaligned-ref-corners";
    case JBeamStructuralDiagnosticCode::INVALID_SLOT_NODE_OFFSET:
        return "invalid-slot-node-offset";
    case JBeamStructuralDiagnosticCode::AERODYNAMICS_UNRESOLVED:
        return "aerodynamics-unresolved";
    }
    return "unknown";
}

} // namespace BeamNG
} // namespace RoR
