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

#include "JBeamPartResolver.h"

#include <algorithm>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace RoR {
namespace BeamNG {

/// The documented `"none"` sentinel, and an absent default, both mean
/// "deliberately nothing". One definition, because the resolver's own
/// substitution policy and every downstream consumer have to agree on it -
/// and when they did not, a derivation filled a slot an author had
/// explicitly emptied.
///
/// Matched exactly, not case-folded: `"none"` is what the format
/// documents, and treating `"None"` as a decline would be this file
/// inventing a convention rather than reading one.
bool IsDeclinedJBeamSlotSelection(const std::string& selection)
{
    return selection.empty() || selection == "none";
}

const char* JBeamPartResolverPolicyIdentifier() noexcept
{
    return "ror-jbeam-part-resolution-empty-incompatible-default-v2";
}

const std::string& UnresolvedJBeamSlotPart(const JBeamResolvedSlot& slot) noexcept
{
    static const std::string empty;
    if (slot.child ||
        (slot.status != JBeamResolvedSlotStatus::MISSING &&
         slot.status != JBeamResolvedSlotStatus::EMPTY))
    {
        return empty;
    }
    if (slot.explicitly_selected &&
        !IsDeclinedJBeamSlotSelection(slot.unresolved_requested_part))
    {
        return slot.unresolved_requested_part;
    }
    if (slot.status == JBeamResolvedSlotStatus::MISSING &&
        !IsDeclinedJBeamSlotSelection(slot.selected_part))
    {
        return slot.selected_part;
    }
    return empty;
}

namespace {

bool HasErrors(const std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        if (diagnostics[i].severity == JBeamResolveSeverity::ERROR_SEVERITY)
        {
            return true;
        }
    }
    return false;
}

JBeamResolveDiagnostic MakeDiagnostic(
    JBeamResolveDiagnosticCode code,
    JBeamResolveSeverity severity,
    const JBeamSourceSpan& span,
    const std::string& part_name,
    const std::string& slot_name,
    const std::string& detail)
{
    JBeamResolveDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.span = span;
    diagnostic.part_name = part_name;
    diagnostic.slot_name = slot_name;
    diagnostic.detail = detail;
    return diagnostic;
}

bool IsDiagnosticLimit(
    const JBeamResolveDiagnostic& diagnostic)
{
    return diagnostic.code ==
           JBeamResolveDiagnosticCode::DIAGNOSTIC_LIMIT;
}

JBeamResolveDiagnostic MakeDiagnosticLimit()
{
    return MakeDiagnostic(
        JBeamResolveDiagnosticCode::DIAGNOSTIC_LIMIT,
        JBeamResolveSeverity::ERROR_SEVERITY,
        JBeamSourceSpan(),
        std::string(),
        std::string(),
        "Additional diagnostics were deterministically suppressed");
}

void PushDiagnostic(
    std::vector<JBeamResolveDiagnostic>& diagnostics,
    std::size_t configured_limit,
    const JBeamResolveDiagnostic& diagnostic)
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

    const std::size_t retained_limit =
        configured_limit;
    if (diagnostics.size() < retained_limit)
    {
        diagnostics.push_back(diagnostic);
        return;
    }

    // The terminal entry consumes the last retained slot. Replacing the last
    // ordinary entry avoids a transient allocation beyond the configured
    // bound and makes the result independent of allocator capacity.
    diagnostics[retained_limit - 1U] = MakeDiagnosticLimit();
}

bool DiagnosticLess(
    const JBeamResolveDiagnostic& left,
    const JBeamResolveDiagnostic& right)
{
    if (IsDiagnosticLimit(left) != IsDiagnosticLimit(right))
    {
        return !IsDiagnosticLimit(left);
    }
    if (left.span.source_name != right.span.source_name)
    {
        return left.span.source_name < right.span.source_name;
    }
    if (left.span.begin.byte_offset != right.span.begin.byte_offset)
    {
        return left.span.begin.byte_offset < right.span.begin.byte_offset;
    }
    if (left.span.end.byte_offset != right.span.end.byte_offset)
    {
        return left.span.end.byte_offset < right.span.end.byte_offset;
    }
    if (left.code != right.code)
    {
        return static_cast<int>(left.code) < static_cast<int>(right.code);
    }
    if (left.part_name != right.part_name)
    {
        return left.part_name < right.part_name;
    }
    if (left.slot_name != right.slot_name)
    {
        return left.slot_name < right.slot_name;
    }
    return left.detail < right.detail;
}

void SortDiagnostics(std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    std::stable_sort(
        diagnostics.begin(), diagnostics.end(), DiagnosticLess);
}

std::string LengthPrefixed(const std::string& value)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << value.size() << ':' << value;
    return output.str();
}

void AppendCanonicalValue(
    const JBeamValue& value,
    std::ostringstream& output)
{
    switch (value.type)
    {
    case JBeamValueType::NULL_VALUE:
        output << 'n';
        return;
    case JBeamValueType::BOOLEAN:
        output << (value.boolean_value ? "b1" : "b0");
        return;
    case JBeamValueType::NUMBER:
        output << 'd' << LengthPrefixed(value.scalar_text);
        return;
    case JBeamValueType::STRING:
        output << 's' << LengthPrefixed(value.scalar_text);
        return;
    case JBeamValueType::ARRAY:
        output << 'a' << value.array_values.size() << '[';
        for (std::size_t i = 0; i < value.array_values.size(); ++i)
        {
            AppendCanonicalValue(value.array_values[i], output);
        }
        output << ']';
        return;
    case JBeamValueType::OBJECT:
        output << 'o' << value.object_fields.size() << '{';
        for (std::size_t i = 0; i < value.object_fields.size(); ++i)
        {
            const JBeamObjectField& field = value.object_fields[i];
            output << 'k' << LengthPrefixed(field.key);
            if (field.value)
            {
                AppendCanonicalValue(*field.value, output);
            }
            else
            {
                output << 'x';
            }
        }
        output << '}';
        return;
    }
}

std::string CanonicalValue(const JBeamValue& value)
{
    std::ostringstream output;
    output.imbue(std::locale::classic());
    AppendCanonicalValue(value, output);
    return output.str();
}

void AppendSpan(
    const JBeamSourceSpan& span,
    std::ostringstream& output)
{
    output
        << LengthPrefixed(span.source_name) << '\t'
        << span.begin.byte_offset << '\t'
        << span.begin.line << '\t'
        << span.begin.column << '\t'
        << span.end.byte_offset << '\t'
        << span.end.line << '\t'
        << span.end.column;
}

bool IsScalarVariableValue(const JBeamValue& value)
{
    return value.type == JBeamValueType::BOOLEAN ||
           value.type == JBeamValueType::NUMBER ||
           value.type == JBeamValueType::STRING;
}

bool IsVariableName(const std::string& name)
{
    return !name.empty() && name[0] == '$';
}

/// A tuning-variable name is a dictionary key in the authored document, not
/// a C identifier, so every byte after the `$` takes the same run and a
/// leading digit is ordinary: content names variables `$1steer`, `$3fdamp`.
/// The expression tokenizer reads the identical run after its own `$`.
bool IsAuthoredTuningVariableName(const std::string& name)
{
    if (name.size() < 2U || name[0] != '$')
    {
        return false;
    }
    for (std::size_t i = 1U; i < name.size(); ++i)
    {
        const unsigned char value =
            static_cast<unsigned char>(name[i]);
        if (!((value >= static_cast<unsigned char>('a') &&
               value <= static_cast<unsigned char>('z')) ||
              (value >= static_cast<unsigned char>('A') &&
               value <= static_cast<unsigned char>('Z')) ||
              (value >= static_cast<unsigned char>('0') &&
               value <= static_cast<unsigned char>('9')) ||
              value == static_cast<unsigned char>('_')))
        {
            return false;
        }
    }
    return true;
}

bool IsStructuralString(const JBeamValue& value)
{
    if (value.type != JBeamValueType::STRING ||
        value.scalar_text.empty())
    {
        return false;
    }
    return value.scalar_text.compare(0, 2, "$=") != 0 &&
           value.scalar_text[0] != '$';
}

bool IsLiteralStringAllowEmpty(const JBeamValue& value)
{
    return value.type == JBeamValueType::STRING &&
           (value.scalar_text.empty() || IsStructuralString(value));
}

const JBeamNormalizedTable* FindRootTable(
    const JBeamNormalizeResult& normalized)
{
    for (std::size_t i = 0; i < normalized.tables.size(); ++i)
    {
        if (normalized.tables[i].path == "$")
        {
            return &normalized.tables[i];
        }
    }
    return NULL;
}

const JBeamFieldAssignment* RowField(
    const JBeamNormalizedDataRow& row,
    const std::string& name)
{
    return FindEffectiveJBeamField(row, name);
}

JBeamNormalizeResult NormalizeResolverTable(
    const JBeamValue& value,
    const JBeamResolverLimits& limits)
{
    JBeamNormalizeLimits normalize_limits;
    normalize_limits.max_work_units =
        limits.max_table_normalize_work_units;
    normalize_limits.max_retained_bytes =
        limits.max_table_normalize_retained_bytes;
    normalize_limits.max_diagnostics =
        std::max<std::size_t>(1U, limits.max_diagnostics);
    return NormalizeJBeamTables(value, normalize_limits);
}

bool ReadStringArray(
    const JBeamValue& value,
    std::vector<std::string>& output)
{
    if (value.type != JBeamValueType::ARRAY ||
        value.array_values.empty())
    {
        return false;
    }
    for (std::size_t i = 0; i < value.array_values.size(); ++i)
    {
        if (!IsStructuralString(value.array_values[i]))
        {
            return false;
        }
        output.push_back(value.array_values[i].scalar_text);
    }
    return true;
}

void ReadSlotVariables(
    const JBeamFieldAssignment* variables_field,
    const std::string& part_name,
    const std::string& slot_name,
    const JBeamResolverLimits& limits,
    std::vector<JBeamVariableAssignment>& output,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    if (variables_field == NULL)
    {
        return;
    }
    if (variables_field->value->type != JBeamValueType::OBJECT)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_VARIABLE,
            JBeamResolveSeverity::ERROR_SEVERITY,
            variables_field->span,
            part_name,
            slot_name,
            "Slot variables must be a dictionary"));
        return;
    }

    const JBeamValue& object = *variables_field->value;
    if (object.object_fields.size() >
        limits.max_variables_per_node)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::RESOLVED_VARIABLE_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                variables_field->span,
                part_name,
                slot_name,
                "Slot variables exceed the configured per-node limit"));
        return;
    }
    for (std::size_t i = 0; i < object.object_fields.size(); ++i)
    {
        const JBeamObjectField& field = object.object_fields[i];
        if (!IsVariableName(field.key) ||
            !field.value ||
            !IsScalarVariableValue(*field.value))
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_SLOT_VARIABLE,
                JBeamResolveSeverity::ERROR_SEVERITY,
                field.key_span,
                part_name,
                slot_name,
                "Slot variable names must begin with '$' and values must "
                "be number, string, or Boolean scalars"));
            continue;
        }
        JBeamVariableAssignment assignment;
        assignment.name = field.key;
        assignment.value = *field.value;
        assignment.span = field.value->span;
        assignment.origin = JBeamVariableOrigin::SLOT;
        output.push_back(assignment);
    }
}

/// A `nodeOffset` component is a literal number or an unevaluated expression
/// string. Booleans, arrays and objects are shape errors: they cannot become a
/// translation and silently discarding them would place the part at the
/// vehicle origin without saying so.
bool IsNodeOffsetComponent(const JBeamValue& value)
{
    return value.type == JBeamValueType::NUMBER ||
           value.type == JBeamValueType::STRING;
}

void ReadSlotNodeOffset(
    const JBeamFieldAssignment* offset_field,
    const std::string& part_name,
    const std::string& slot_name,
    const JBeamResolverLimits& limits,
    JBeamSlotNodeOffset& output,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    if (offset_field == NULL)
    {
        return;
    }
    if (!offset_field->value ||
        offset_field->value->type != JBeamValueType::OBJECT)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_SLOT_NODE_OFFSET,
                JBeamResolveSeverity::ERROR_SEVERITY,
                offset_field->span,
                part_name,
                slot_name,
                "Slot nodeOffset must be a dictionary of x, y and z "
                "components"));
        return;
    }

    const JBeamValue& object = *offset_field->value;
    JBeamSlotNodeOffset parsed;
    parsed.span = offset_field->span;
    bool valid = true;
    // Duplicate keys keep the package's own last-assignment semantics rather
    // than being rejected, matching every other table this resolver reads.
    for (std::size_t i = 0; i < object.object_fields.size(); ++i)
    {
        const JBeamObjectField& field = object.object_fields[i];
        const bool is_x = field.key == "x";
        const bool is_y = field.key == "y";
        const bool is_z = field.key == "z";
        if (!is_x && !is_y && !is_z)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_SLOT_NODE_OFFSET,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    field.key_span,
                    part_name,
                    slot_name,
                    "Slot nodeOffset accepts only the x, y and z "
                    "components"));
            valid = false;
            continue;
        }
        if (!field.value || !IsNodeOffsetComponent(*field.value))
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_SLOT_NODE_OFFSET,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    field.key_span,
                    part_name,
                    slot_name,
                    "Slot nodeOffset components must be numbers or "
                    "unevaluated expression strings"));
            valid = false;
            continue;
        }
        if (is_x)
        {
            parsed.has_x = true;
            parsed.x = *field.value;
        }
        else if (is_y)
        {
            parsed.has_y = true;
            parsed.y = *field.value;
        }
        else
        {
            parsed.has_z = true;
            parsed.z = *field.value;
        }
    }
    if (!valid)
    {
        return;
    }
    // An empty or all-absent dictionary is a documented no-op, not an offset.
    parsed.present = parsed.has_x || parsed.has_y || parsed.has_z;
    output = parsed;
}

bool ReadCoreSlot(
    const JBeamFieldAssignment* field,
    const std::string& part_name,
    const std::string& slot_name,
    bool& core_slot,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    core_slot = false;
    if (field == NULL)
    {
        return true;
    }
    if (field->value->type != JBeamValueType::BOOLEAN)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD,
            JBeamResolveSeverity::ERROR_SEVERITY,
            field->span,
            part_name,
            slot_name,
            "coreSlot must be Boolean"));
        return false;
    }
    core_slot = field->value->boolean_value;
    return true;
}

bool ReadRequiredString(
    const JBeamFieldAssignment* field,
    const std::string& field_name,
    const std::string& part_name,
    const std::string& slot_name,
    std::string& output,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics,
    bool allow_empty)
{
    if (field == NULL)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::MISSING_SLOT_FIELD,
            JBeamResolveSeverity::ERROR_SEVERITY,
            JBeamSourceSpan(),
            part_name,
            slot_name,
            "Missing required slot field '" + field_name + "'"));
        return false;
    }
    if (field->value->type != JBeamValueType::STRING ||
        (!allow_empty && !IsStructuralString(*field->value)) ||
        (allow_empty &&
         !field->value->scalar_text.empty() &&
         !IsStructuralString(*field->value)))
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD,
            JBeamResolveSeverity::ERROR_SEVERITY,
            field->span,
            part_name,
            slot_name,
            "Slot field '" + field_name +
                "' must be a literal string"));
        return false;
    }
    output = field->value->scalar_text;
    return true;
}

struct ParsedSlotRow
{
    JBeamSlotDefinition definition;
    bool plain_fields;
};

bool IsPlainSlotField(const std::string& name, JBeamSlotKind kind)
{
    return name == "default" || name == "description" || name == "coreSlot" ||
        (kind == JBeamSlotKind::SLOTS ? name == "type" :
            name == "name" || name == "allowTypes" || name == "denyTypes");
}

void ParseSlotTable(
    const JBeamValue& value,
    JBeamSlotKind kind,
    const std::string& part_name,
    const JBeamResolverLimits& limits,
    std::vector<ParsedSlotRow>& output,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    if (value.type == JBeamValueType::ARRAY && value.array_values.empty())
    {
        // Real-world parts declare `"slots": []` for "no child slots".
        // BeamNG's loader accepts the empty table, so it is recorded as a
        // warning rather than refusing the whole package for one part.
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_TABLE,
            JBeamResolveSeverity::WARNING,
            value.span,
            part_name,
            std::string(),
            kind == JBeamSlotKind::SLOTS
                ? "slots is an empty table; the part declares no child slots"
                : "slots2 is an empty table; the part declares no child slots"));
        return;
    }
    const JBeamNormalizeResult normalized =
        NormalizeResolverTable(value, limits);
    const JBeamNormalizedTable* table = FindRootTable(normalized);
    if (table == NULL)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_TABLE,
            JBeamResolveSeverity::ERROR_SEVERITY,
            value.span,
            part_name,
            std::string(),
            kind == JBeamSlotKind::SLOTS
                ? "slots must be a header-row table"
                : "slots2 must be a header-row table"));
        return;
    }
    for (std::size_t diagnostic_index = 0;
         diagnostic_index < normalized.diagnostics.size();
         ++diagnostic_index)
    {
        const JBeamDiagnostic& syntax_diagnostic =
            normalized.diagnostics[diagnostic_index];
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            syntax_diagnostic.code ==
                    JBeamDiagnosticCode::DUPLICATE_TABLE_HEADER
                ? JBeamResolveDiagnosticCode::INVALID_SLOT_TABLE
                : JBeamResolveDiagnosticCode::INVALID_SLOT_ROW,
            JBeamResolveSeverity::ERROR_SEVERITY,
            syntax_diagnostic.span,
            part_name,
            std::string(),
            "Malformed slot table: " + syntax_diagnostic.message));
    }

    // Eligibility must inspect the authored assignments, not only the small
    // public slot projection: an unparsed nodeMove/nodeRotate/other modifier
    // cannot disappear and make two behaviorally different rows equivalent.
    // Scan the shared inherited history once, not once per subsequent row.
    std::size_t inspected_inherited = 0U;
    bool inherited_plain_fields = true;
    for (std::size_t i = 0; i < table->entries.size(); ++i)
    {
        const JBeamNormalizedTableEntry& entry = table->entries[i];
        if (entry.kind ==
            JBeamNormalizedTableEntryKind::DEFAULT_MODIFIER)
        {
            continue;
        }
        if (entry.kind != JBeamNormalizedTableEntryKind::DATA_ROW)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_SLOT_ROW,
                JBeamResolveSeverity::ERROR_SEVERITY,
                entry.raw_value.span,
                part_name,
                std::string(),
                "Slot table entries must be row arrays or inherited "
                "default dictionaries"));
            continue;
        }
        if (output.size() >= limits.max_slots_per_part)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_SLOT_TABLE,
                JBeamResolveSeverity::ERROR_SEVERITY,
                entry.raw_value.span,
                part_name,
                std::string(),
                "Part exceeds the configured slot-count limit"));
            return;
        }

        const JBeamNormalizedDataRow& row = entry.data_row;
        if (row.inherited_assignment_count < inspected_inherited ||
            (row.inherited_assignment_count != 0U &&
             (!row.inherited_assignment_storage ||
              row.inherited_assignment_count > row.inherited_assignment_storage->size())))
        {
            inherited_plain_fields = false;
        }
        else if (row.inherited_assignment_storage)
        {
            for (; inspected_inherited < row.inherited_assignment_count;
                 ++inspected_inherited)
            {
                inherited_plain_fields = IsPlainSlotField(
                    (*row.inherited_assignment_storage)[inspected_inherited].name, kind) &&
                    inherited_plain_fields;
            }
        }
        bool plain_fields = inherited_plain_fields;
        for (const auto& assignment : row.positional_assignments)
            plain_fields = IsPlainSlotField(assignment.name, kind) && plain_fields;
        for (const auto& assignment : row.row_local_assignments)
            plain_fields = IsPlainSlotField(assignment.name, kind) && plain_fields;
        const std::size_t cells = row.raw_row.array_values.size();
        plain_fields = plain_fields &&
            (cells <= table->columns.size() ||
             (cells == table->columns.size() + 1U &&
              row.raw_row.array_values.back().type == JBeamValueType::OBJECT));
        JBeamSlotDefinition slot;
        slot.kind = kind;
        slot.span = row.span;
        const char* name_field =
            kind == JBeamSlotKind::SLOTS ? "type" : "name";
        bool valid = ReadRequiredString(
            RowField(row, name_field),
            name_field,
            part_name,
            std::string(),
            slot.name,
            limits,
            diagnostics,
            false);
        valid = ReadRequiredString(
                    RowField(row, "default"),
                    "default",
                    part_name,
                    slot.name,
                    slot.default_part,
                    limits,
                    diagnostics,
                    true) &&
                valid;
        valid = ReadRequiredString(
                    RowField(row, "description"),
                    "description",
                    part_name,
                    slot.name,
                    slot.description,
                    limits,
                    diagnostics,
                    true) &&
                valid;
        valid = ReadCoreSlot(
                    RowField(row, "coreSlot"),
                    part_name,
                    slot.name,
                    slot.core_slot,
                    limits,
                    diagnostics) &&
                valid;

        if (kind == JBeamSlotKind::SLOTS)
        {
            if (!slot.name.empty())
            {
                slot.allow_types.push_back(slot.name);
            }
        }
        else
        {
            const JBeamFieldAssignment* allow =
                RowField(row, "allowTypes");
            const JBeamFieldAssignment* deny =
                RowField(row, "denyTypes");
            if (allow == NULL ||
                !ReadStringArray(*allow->value, slot.allow_types))
            {
                PushDiagnostic(
                    diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    allow == NULL
                        ? JBeamResolveDiagnosticCode::MISSING_SLOT_FIELD
                        : JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    allow == NULL ? row.span : allow->span,
                    part_name,
                    slot.name,
                    "slots2 allowTypes must be a non-empty array of "
                    "literal strings"));
                valid = false;
            }
            if (deny == NULL ||
                deny->value->type != JBeamValueType::ARRAY)
            {
                PushDiagnostic(
                    diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    deny == NULL
                        ? JBeamResolveDiagnosticCode::MISSING_SLOT_FIELD
                        : JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    deny == NULL ? row.span : deny->span,
                    part_name,
                    slot.name,
                    "slots2 denyTypes must be an array of literal strings"));
                valid = false;
            }
            else
            {
                bool deny_valid = true;
                for (std::size_t deny_index = 0;
                     deny_index < deny->value->array_values.size();
                     ++deny_index)
                {
                    const JBeamValue& deny_value =
                        deny->value->array_values[deny_index];
                    if (!IsStructuralString(deny_value))
                    {
                        deny_valid = false;
                        break;
                    }
                    slot.deny_types.push_back(deny_value.scalar_text);
                }
                if (!deny_valid)
                {
                    PushDiagnostic(
                        diagnostics,
                        limits.max_diagnostics,
                        MakeDiagnostic(
                        JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD,
                        JBeamResolveSeverity::ERROR_SEVERITY,
                        deny->span,
                        part_name,
                        slot.name,
                        "slots2 denyTypes must contain only literal "
                        "strings"));
                    valid = false;
                }
            }
        }

        ReadSlotVariables(
            RowField(row, "variables"),
            part_name,
            slot.name,
            limits,
            slot.variables,
            diagnostics);
        ReadSlotNodeOffset(
            RowField(row, "nodeOffset"),
            part_name,
            slot.name,
            limits,
            slot.node_offset,
            diagnostics);
        if (valid)
        {
            output.push_back({slot, plain_fields});
        }
    }
}

void ParsePartSlotTypes(
    JBeamPartDefinition& part,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    const JBeamObjectField* effective = NULL;
    for (std::size_t i = 0; i < part.body.object_fields.size(); ++i)
    {
        if (part.body.object_fields[i].key == "slotType")
        {
            effective = &part.body.object_fields[i];
        }
    }
    if (effective == NULL || !effective->value)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::MISSING_SLOT_TYPE,
            JBeamResolveSeverity::ERROR_SEVERITY,
            part.name_span,
            part.name,
            std::string(),
            "Part has no slotType"));
        return;
    }

    const JBeamValue& value = *effective->value;
    if (IsStructuralString(value))
    {
        part.slot_types.push_back(value.scalar_text);
    }
    else if (value.type == JBeamValueType::ARRAY &&
             !value.array_values.empty())
    {
        for (std::size_t i = 0; i < value.array_values.size(); ++i)
        {
            if (!IsStructuralString(value.array_values[i]))
            {
                PushDiagnostic(
                    diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_SLOT_TYPE,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    value.array_values[i].span,
                    part.name,
                    std::string(),
                    "slotType arrays must contain literal non-empty "
                    "strings"));
                continue;
            }
            if (std::find(
                    part.slot_types.begin(),
                    part.slot_types.end(),
                    value.array_values[i].scalar_text) !=
                part.slot_types.end())
            {
                PushDiagnostic(
                    diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    JBeamResolveDiagnosticCode::DUPLICATE_SLOT_TYPE,
                    JBeamResolveSeverity::WARNING,
                    value.array_values[i].span,
                    part.name,
                    std::string(),
                    "Duplicate slotType is preserved"));
            }
            part.slot_types.push_back(value.array_values[i].scalar_text);
        }
    }
    else
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::INVALID_SLOT_TYPE,
            JBeamResolveSeverity::ERROR_SEVERITY,
            value.span,
            part.name,
            std::string(),
            "slotType must be a literal string or non-empty array of "
            "literal strings"));
    }
}

// The same selection/mount behavior, ignoring only the part-selector display
// label. The caller additionally requires both complete authored assignment
// histories to be plain; variables, transforms and unknown options never
// qualify even if the public slot projection did not retain them.
bool IsRepeatedPlainSlotRow(
    const JBeamSlotDefinition& first,
    const JBeamSlotDefinition& repeat)
{
    return first.kind == repeat.kind &&
           first.name == repeat.name &&
           first.allow_types == repeat.allow_types &&
           first.deny_types == repeat.deny_types &&
           first.default_part == repeat.default_part &&
           first.core_slot == repeat.core_slot &&
           first.variables.empty() && repeat.variables.empty() &&
           !first.node_offset.present && !repeat.node_offset.present;
}

void ParsePartSlots(
    JBeamPartDefinition& part,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    std::vector<ParsedSlotRow> parsed_slots;
    const JBeamObjectField* slots = NULL;
    const JBeamObjectField* slots2 = NULL;
    for (std::size_t i = 0; i < part.body.object_fields.size(); ++i)
    {
        const JBeamObjectField& field = part.body.object_fields[i];
        if (field.key != "slots" && field.key != "slots2")
        {
            continue;
        }
        const JBeamObjectField*& effective =
            field.key == "slots" ? slots : slots2;
        if (effective != NULL)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::DUPLICATE_SLOT_SECTION,
                JBeamResolveSeverity::WARNING,
                field.key_span,
                part.name,
                field.key,
                "Only the last duplicate slot section is effective; all "
                "source fields remain in the part AST"));
        }
        effective = &field;
    }

    if (slots != NULL && slots->value)
    {
        ParseSlotTable(
            *slots->value,
            JBeamSlotKind::SLOTS,
            part.name,
            limits,
            parsed_slots,
            diagnostics);
    }
    if (slots2 != NULL && slots2->value)
    {
        ParseSlotTable(
            *slots2->value,
            JBeamSlotKind::SLOTS2,
            part.name,
            limits,
            parsed_slots,
            diagnostics);
    }

    std::stable_sort(
        parsed_slots.begin(),
        parsed_slots.end(),
        [](const ParsedSlotRow& left_row,
           const ParsedSlotRow& right_row)
        {
            const JBeamSlotDefinition& left = left_row.definition;
            const JBeamSlotDefinition& right = right_row.definition;
            if (left.span.source_name != right.span.source_name)
            {
                return left.span.source_name < right.span.source_name;
            }
            if (left.span.begin.byte_offset !=
                right.span.begin.byte_offset)
            {
                return left.span.begin.byte_offset <
                       right.span.begin.byte_offset;
            }
            if (left.kind != right.kind)
            {
                return static_cast<int>(left.kind) <
                       static_cast<int>(right.kind);
            }
            return left.name < right.name;
        });

    std::map<std::string, std::size_t> names;
    std::vector<JBeamSlotDefinition> unique_slots;
    std::vector<bool> unique_plain_fields;
    unique_slots.reserve(parsed_slots.size());
    unique_plain_fields.reserve(parsed_slots.size());
    for (std::size_t i = 0; i < parsed_slots.size(); ++i)
    {
        const JBeamSlotDefinition& slot = parsed_slots[i].definition;
        const std::map<std::string, std::size_t>::const_iterator first =
            names.find(slot.name);
        if (first == names.end())
        {
            names.insert(std::make_pair(slot.name, unique_slots.size()));
            unique_slots.push_back(slot);
            unique_plain_fields.push_back(parsed_slots[i].plain_fields);
        }
        else if (unique_plain_fields[first->second] && parsed_slots[i].plain_fields &&
                 IsRepeatedPlainSlotRow(unique_slots[first->second], slot))
        {
            // One selection, not two mounted children. Retain the first UI
            // label; every authored row and label remains in the source AST.
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::DUPLICATE_SLOT,
                JBeamResolveSeverity::WARNING,
                slot.span,
                part.name,
                slot.name,
                unique_slots[first->second].description == slot.description
                    ? "Identical repeated slot declaration is ignored"
                    : "Behavior-equivalent repeated slot has a different display label; "
                      "first display label is retained"));
        }
        else
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::DUPLICATE_SLOT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                slot.span,
                part.name,
                slot.name,
                "Duplicate slot name is ambiguous across slots/slots2"));
            unique_slots.push_back(slot);
            unique_plain_fields.push_back(parsed_slots[i].plain_fields);
        }
    }
    part.slots.swap(unique_slots);
}

bool ReadTuningString(
    const JBeamNormalizedDataRow& row,
    const std::string& field_name,
    const std::string& part_name,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics,
    std::string& output,
    bool required,
    bool allow_variable_name = false)
{
    const JBeamFieldAssignment* field = RowField(row, field_name);
    if (field == NULL)
    {
        if (required)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        MISSING_TUNING_VARIABLE_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    row.span,
                    part_name,
                    field_name,
                    "Tuning-variable row is missing required field '" +
                        field_name + "'"));
            return false;
        }
        return true;
    }
    if (!field->value ||
        (allow_variable_name
             ? field->value->type != JBeamValueType::STRING
             : !IsLiteralStringAllowEmpty(*field->value)))
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    INVALID_TUNING_VARIABLE_FIELD,
                JBeamResolveSeverity::ERROR_SEVERITY,
                field->span,
                part_name,
                field_name,
                "Tuning-variable field '" + field_name +
                    "' must be a literal string"));
        return false;
    }
    output = field->value->scalar_text;
    return true;
}

bool ReadTuningNumber(
    const JBeamNormalizedDataRow& row,
    const std::string& field_name,
    const std::string& part_name,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics,
    JBeamValue& output,
    bool required,
    bool& present)
{
    const JBeamFieldAssignment* field = RowField(row, field_name);
    present = field != NULL;
    if (field == NULL)
    {
        if (required)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        MISSING_TUNING_VARIABLE_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    row.span,
                    part_name,
                    field_name,
                    "Tuning-variable row is missing required field '" +
                        field_name + "'"));
            return false;
        }
        return true;
    }
    if (!field->value ||
        field->value->type != JBeamValueType::NUMBER)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    INVALID_TUNING_VARIABLE_FIELD,
                JBeamResolveSeverity::ERROR_SEVERITY,
                field->span,
                part_name,
                field_name,
                "Tuning-variable field '" + field_name +
                    "' must be a finite literal number"));
        return false;
    }
    output = *field->value;
    return true;
}

bool ReadTuningBoolean(
    const JBeamNormalizedDataRow& row,
    const std::string& field_name,
    const std::string& part_name,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics,
    bool& output)
{
    const JBeamFieldAssignment* field = RowField(row, field_name);
    if (field == NULL)
    {
        return true;
    }
    if (!field->value ||
        field->value->type != JBeamValueType::BOOLEAN)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    INVALID_TUNING_VARIABLE_FIELD,
                JBeamResolveSeverity::ERROR_SEVERITY,
                field->span,
                part_name,
                field_name,
                "Tuning-variable field '" + field_name +
                    "' must be Boolean"));
        return false;
    }
    output = field->value->boolean_value;
    return true;
}

void ParsePartTuningVariables(
    JBeamPartDefinition& part,
    const JBeamResolverLimits& limits,
    std::vector<JBeamResolveDiagnostic>& diagnostics)
{
    const JBeamObjectField* variables = NULL;
    for (std::size_t i = 0U;
         i < part.body.object_fields.size();
         ++i)
    {
        const JBeamObjectField& field = part.body.object_fields[i];
        if (field.key != "variables")
        {
            continue;
        }
        if (variables != NULL)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        DUPLICATE_TUNING_VARIABLE_SECTION,
                    JBeamResolveSeverity::WARNING,
                    field.key_span,
                    part.name,
                    "variables",
                    "Only the last duplicate authored variables table is "
                    "effective; all source fields remain in the part AST"));
        }
        variables = &field;
    }
    if (variables == NULL)
    {
        return;
    }
    if (!variables->value)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_TABLE,
                JBeamResolveSeverity::ERROR_SEVERITY,
                variables->key_span,
                part.name,
                "variables",
                "Authored variables must be a header-row table"));
        return;
    }

    const JBeamNormalizeResult normalized =
        NormalizeResolverTable(*variables->value, limits);
    const JBeamNormalizedTable* table = FindRootTable(normalized);
    if (table == NULL)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_TABLE,
                JBeamResolveSeverity::ERROR_SEVERITY,
                variables->value->span,
                part.name,
                "variables",
                "Authored variables must be a header-row table"));
        return;
    }
    for (std::size_t i = 0U;
         i < normalized.diagnostics.size();
         ++i)
    {
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_TABLE,
                JBeamResolveSeverity::ERROR_SEVERITY,
                normalized.diagnostics[i].span,
                part.name,
                "variables",
                "Malformed authored variables table: " +
                    normalized.diagnostics[i].message));
    }
    if (table->entries.size() > limits.max_variables_per_node)
    {
        const std::size_t rejected = limits.max_variables_per_node;
        PushDiagnostic(
            diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::TUNING_VARIABLE_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                rejected < table->entries.size()
                    ? table->entries[rejected].raw_value.span
                    : variables->value->span,
                part.name,
                "variables",
                "Authored variables exceed the configured per-part "
                "variable limit"));
        return;
    }

    std::set<std::string> names;
    for (std::size_t row_index = 0U;
         row_index < table->entries.size();
         ++row_index)
    {
        const JBeamNormalizedTableEntry& entry =
            table->entries[row_index];
        if (entry.kind != JBeamNormalizedTableEntryKind::DATA_ROW)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_ROW,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    entry.raw_value.span,
                    part.name,
                    "variables",
                    "Authored variables entries must be data-row arrays"));
            continue;
        }

        const JBeamNormalizedDataRow& row = entry.data_row;
        JBeamTuningVariableDefinition definition;
        definition.part_name = part.name;
        definition.package_path = part.package_path;
        definition.span = row.span;
        bool valid = true;
        valid = ReadTuningString(
                    row, "name", part.name, limits, diagnostics,
                    definition.name, true, true) && valid;
        valid = ReadTuningString(
                    row, "type", part.name, limits, diagnostics,
                    definition.type, true) && valid;
        valid = ReadTuningString(
                    row, "unit", part.name, limits, diagnostics,
                    definition.unit, true) && valid;
        valid = ReadTuningString(
                    row, "category", part.name, limits, diagnostics,
                    definition.category, true) && valid;
        bool default_present = false;
        bool min_present = false;
        bool max_present = false;
        valid = ReadTuningNumber(
                    row, "default", part.name, limits, diagnostics,
                    definition.default_value, true, default_present) &&
                valid;
        valid = ReadTuningNumber(
                    row, "min", part.name, limits, diagnostics,
                    definition.min_value, true, min_present) && valid;
        valid = ReadTuningNumber(
                    row, "max", part.name, limits, diagnostics,
                    definition.max_value, true, max_present) && valid;
        valid = ReadTuningString(
                    row, "title", part.name, limits, diagnostics,
                    definition.title, true) && valid;
        valid = ReadTuningString(
                    row, "description", part.name, limits, diagnostics,
                    definition.description, true) && valid;
        valid = ReadTuningString(
                    row, "subCategory", part.name, limits, diagnostics,
                    definition.sub_category, false) && valid;

        JBeamValue optional;
        bool present = false;
        if (ReadTuningNumber(
                row, "stepDis", part.name, limits, diagnostics,
                optional, false, present))
        {
            definition.has_step_dis = present;
            if (present)
            {
                definition.step_dis = optional;
            }
        }
        else
        {
            valid = false;
        }
        present = false;
        if (ReadTuningNumber(
                row, "minDis", part.name, limits, diagnostics,
                optional, false, present))
        {
            definition.has_min_dis = present;
            if (present)
            {
                definition.min_dis = optional;
            }
        }
        else
        {
            valid = false;
        }
        present = false;
        if (ReadTuningNumber(
                row, "maxDis", part.name, limits, diagnostics,
                optional, false, present))
        {
            definition.has_max_dis = present;
            if (present)
            {
                definition.max_dis = optional;
            }
        }
        else
        {
            valid = false;
        }
        valid = ReadTuningBoolean(
                    row, "hideInUI", part.name, limits, diagnostics,
                    definition.hide_in_ui) && valid;

        if (valid &&
            !IsAuthoredTuningVariableName(definition.name))
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        INVALID_TUNING_VARIABLE_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    row.span,
                    part.name,
                    "name",
                    "Tuning-variable names must begin with '$' and "
                    "contain a non-empty identifier"));
            valid = false;
        }
        if (valid && definition.type != "range")
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        INVALID_TUNING_VARIABLE_FIELD,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    row.span,
                    part.name,
                    "type",
                    "Only the documented tuning-variable type 'range' is "
                    "supported"));
            valid = false;
        }
        // Real-world content authors inverted ranges (min > max) to flip
        // the tuning slider's direction; the authored orientation is
        // retained UI metadata while the bound is the closed interval
        // between the two.
        //
        // A default outside that interval is used at the nearer bound, with
        // the bound's exact spelling, exactly as an out-of-range override is
        // (TUNING_VARIABLE_OVERRIDE_CLAMPED): BeamNG bounds a jbeam variable
        // by its declared range. The Frix MF1 declares $toe_R with default
        // 0.6861 in [0.95, 1.05], and that row refused the whole package.
        // The authored row itself stays in the part body.
        if (valid)
        {
            const bool ascending =
                definition.min_value.number_value <
                definition.max_value.number_value;
            const JBeamValue& lower_bound =
                ascending ? definition.min_value : definition.max_value;
            const JBeamValue& upper_bound =
                ascending ? definition.max_value : definition.min_value;
            const JBeamValue* const bound =
                definition.default_value.number_value <
                        lower_bound.number_value
                    ? &lower_bound
                    : definition.default_value.number_value >
                            upper_bound.number_value
                        ? &upper_bound
                        : NULL;
            if (bound != NULL)
            {
                PushDiagnostic(
                    diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            TUNING_VARIABLE_DEFAULT_CLAMPED,
                        JBeamResolveSeverity::WARNING,
                        row.span,
                        part.name,
                        definition.name,
                        "given=" + definition.default_value.scalar_text +
                            ":declared=[" + lower_bound.scalar_text + "," +
                            upper_bound.scalar_text + "]:used=" +
                            bound->scalar_text));
                definition.default_value.number_value = bound->number_value;
                definition.default_value.scalar_text = bound->scalar_text;
            }
        }
        if (!valid)
        {
            continue;
        }
        if (!names.insert(definition.name).second)
        {
            PushDiagnostic(
                diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                    JBeamResolveDiagnosticCode::DUPLICATE_TUNING_VARIABLE,
                    JBeamResolveSeverity::WARNING,
                    definition.span,
                    part.name,
                    definition.name,
                    "Duplicate authored tuning variable uses the last row"));
        }
        part.tuning_variables.push_back(definition);
    }
}

bool HasSlotType(
    const JBeamPartDefinition& part,
    const std::string& slot_type)
{
    return std::find(
               part.slot_types.begin(),
               part.slot_types.end(),
               slot_type) != part.slot_types.end();
}

bool PartLess(
    const JBeamPartDefinition& left,
    const JBeamPartDefinition& right)
{
    if (left.name != right.name)
    {
        return left.name < right.name;
    }
    if (left.package_path != right.package_path)
    {
        return left.package_path < right.package_path;
    }
    if (left.name_span.source_name != right.name_span.source_name)
    {
        return left.name_span.source_name < right.name_span.source_name;
    }
    if (left.name_span.begin.byte_offset !=
        right.name_span.begin.byte_offset)
    {
        return left.name_span.begin.byte_offset <
               right.name_span.begin.byte_offset;
    }
    return CanonicalValue(left.body) < CanonicalValue(right.body);
}

const char* SeverityToString(JBeamResolveSeverity severity)
{
    return severity == JBeamResolveSeverity::ERROR_SEVERITY ? "error" : "warning";
}

const char* SlotKindToString(JBeamSlotKind kind)
{
    return kind == JBeamSlotKind::SLOTS ? "slots" : "slots2";
}

const char* VariableOriginToString(JBeamVariableOrigin origin)
{
    switch (origin)
    {
    case JBeamVariableOrigin::AUTHORED_DEFAULT:
        return "authored-default";
    case JBeamVariableOrigin::CONFIGURATION:
        return "configuration";
    case JBeamVariableOrigin::SLOT:
        return "slot";
    }
    return "unknown";
}

const char* SlotStatusToString(JBeamResolvedSlotStatus status)
{
    switch (status)
    {
    case JBeamResolvedSlotStatus::RESOLVED: return "resolved";
    case JBeamResolvedSlotStatus::EMPTY: return "empty";
    case JBeamResolvedSlotStatus::MISSING: return "missing";
    case JBeamResolvedSlotStatus::NOT_ALLOWED: return "not-allowed";
    case JBeamResolvedSlotStatus::CYCLE: return "cycle";
    case JBeamResolvedSlotStatus::LIMIT_REJECTED:
        return "limit-rejected";
    case JBeamResolvedSlotStatus::SOURCE_REFUSED:
        return "source-refused";
    }
    return "unknown";
}

void AppendDiagnostic(
    const JBeamResolveDiagnostic& diagnostic,
    std::ostringstream& output)
{
    output
        << "D\t"
        << JBeamResolveDiagnosticCodeToString(diagnostic.code) << '\t'
        << SeverityToString(diagnostic.severity) << '\t'
        << LengthPrefixed(diagnostic.part_name) << '\t'
        << LengthPrefixed(diagnostic.slot_name) << '\t'
        << LengthPrefixed(diagnostic.detail) << '\t';
    AppendSpan(diagnostic.span, output);
    output << '\n';
}

void AppendVariable(
    const JBeamVariableAssignment& variable,
    std::ostringstream& output)
{
    output
        << "V\t" << LengthPrefixed(variable.name) << '\t'
        << VariableOriginToString(variable.origin) << '\t';
    AppendSpan(variable.span, output);
    const std::string value = CanonicalValue(variable.value);
    output << '\t' << LengthPrefixed(value) << '\n';
}

void AppendTuningVariable(
    const JBeamTuningVariableDefinition& variable,
    std::ostringstream& output)
{
    output
        << "T\t" << LengthPrefixed(variable.part_name) << '\t'
        << LengthPrefixed(variable.package_path) << '\t'
        << LengthPrefixed(variable.name) << '\t'
        << LengthPrefixed(variable.type) << '\t'
        << LengthPrefixed(variable.unit) << '\t'
        << LengthPrefixed(variable.category) << '\t'
        << LengthPrefixed(CanonicalValue(variable.default_value)) << '\t'
        << LengthPrefixed(CanonicalValue(variable.min_value)) << '\t'
        << LengthPrefixed(CanonicalValue(variable.max_value)) << '\t'
        << LengthPrefixed(variable.title) << '\t'
        << LengthPrefixed(variable.description) << '\t'
        << LengthPrefixed(variable.sub_category) << '\t'
        << (variable.has_step_dis ? 1 : 0) << '\t'
        << (variable.has_step_dis
                ? LengthPrefixed(CanonicalValue(variable.step_dis))
                : LengthPrefixed(std::string())) << '\t'
        << (variable.has_min_dis ? 1 : 0) << '\t'
        << (variable.has_min_dis
                ? LengthPrefixed(CanonicalValue(variable.min_dis))
                : LengthPrefixed(std::string())) << '\t'
        << (variable.has_max_dis ? 1 : 0) << '\t'
        << (variable.has_max_dis
                ? LengthPrefixed(CanonicalValue(variable.max_dis))
                : LengthPrefixed(std::string())) << '\t'
        << (variable.hide_in_ui ? 1 : 0) << '\t';
    AppendSpan(variable.span, output);
    output << '\n';
}

void AppendSlotDefinition(
    const JBeamSlotDefinition& slot,
    std::ostringstream& output)
{
    output
        << SlotKindToString(slot.kind) << '\t'
        << LengthPrefixed(slot.name) << '\t'
        << LengthPrefixed(slot.default_part) << '\t'
        << LengthPrefixed(slot.description) << '\t'
        << (slot.core_slot ? 1 : 0) << '\t';
    AppendSpan(slot.span, output);
    output << "\tallow\t" << slot.allow_types.size();
    for (std::size_t i = 0; i < slot.allow_types.size(); ++i)
    {
        output << '\t' << LengthPrefixed(slot.allow_types[i]);
    }
    output << "\tdeny\t" << slot.deny_types.size();
    for (std::size_t i = 0; i < slot.deny_types.size(); ++i)
    {
        output << '\t' << LengthPrefixed(slot.deny_types[i]);
    }
    output << "\tnode-offset\t" << (slot.node_offset.present ? 1 : 0);
    if (slot.node_offset.present)
    {
        output << '\t' << (slot.node_offset.has_x ? 1 : 0);
        if (slot.node_offset.has_x)
        {
            AppendCanonicalValue(slot.node_offset.x, output);
        }
        output << '\t' << (slot.node_offset.has_y ? 1 : 0);
        if (slot.node_offset.has_y)
        {
            AppendCanonicalValue(slot.node_offset.y, output);
        }
        output << '\t' << (slot.node_offset.has_z ? 1 : 0);
        if (slot.node_offset.has_z)
        {
            AppendCanonicalValue(slot.node_offset.z, output);
        }
        output << '\t';
        AppendSpan(slot.node_offset.span, output);
    }
    output << "\tslot-vars\t" << slot.variables.size() << '\n';
    for (std::size_t i = 0; i < slot.variables.size(); ++i)
    {
        AppendVariable(slot.variables[i], output);
    }
}

bool IsPartAllowed(
    const JBeamSlotDefinition& slot,
    const JBeamPartDefinition& part)
{
    for (std::size_t i = 0; i < slot.deny_types.size(); ++i)
    {
        if (HasSlotType(part, slot.deny_types[i]))
        {
            return false;
        }
    }
    for (std::size_t i = 0; i < slot.allow_types.size(); ++i)
    {
        if (HasSlotType(part, slot.allow_types[i]))
        {
            return true;
        }
    }
    return false;
}

class GraphResolver
{
public:
    GraphResolver(
        const JBeamPackageIndex& index,
        const JBeamResolveRequest& request,
        const JBeamResolverLimits& limits,
        JBeamPackageIndex* selected_index = NULL,
        JBeamExactPartSource* exact_source = NULL)
        : m_index(index)
        , m_request(request)
        , m_limits(limits)
        , m_selected_index(selected_index)
        , m_exact_source(exact_source)
    {
    }

    JBeamResolvedGraph Run()
    {
        if (m_exact_source != NULL &&
            (m_selected_index == NULL || m_request.root_part_name.empty() ||
                m_request.substitute_missing_package_local_defaults))
        {
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::SELECTED_SOURCE_POLICY_REFUSED,
                JBeamResolveSeverity::ERROR_SEVERITY, JBeamSourceSpan(),
                m_request.root_part_name, std::string(),
                "Exact selected source requires an explicit root and no "
                "package-local sibling substitution"));
            return Finish();
        }
        if (!m_index.IsValid())
        {
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::INDEX_INVALID,
                JBeamResolveSeverity::ERROR_SEVERITY,
                JBeamSourceSpan(),
                std::string(),
                std::string(),
                "Cannot resolve a package index containing errors"));
            return Finish();
        }
        if (!PreflightRequest())
        {
            return Finish();
        }
        PrepareLookups();
        ValidateRequest();
        if (HasErrors(m_graph.diagnostics))
        {
            return Finish();
        }
        m_graph.request = m_request;

        const JBeamPartDefinition* root = SelectRoot();
        if (root == NULL)
        {
            return Finish();
        }
        std::vector<JBeamVariableAssignment> variables;
        variables.reserve(m_request.variables.size());
        variables.insert(
            variables.end(),
            m_request.variables.begin(),
            m_request.variables.end());
        m_graph.root = ResolveNode(*root, variables, 0);
        if (m_graph.root && !HasErrors(m_graph.diagnostics))
        {
            ApplyAuthoredTuningDefaults();
        }

        for (std::size_t i = 0;
             i < m_request.part_selections.size();
             ++i)
        {
            if (!m_selection_used[i])
            {
                const JBeamPartSelection& selection =
                    m_request.part_selections[i];
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::UNUSED_PART_SELECTION,
                    JBeamResolveSeverity::WARNING,
                    selection.span,
                    std::string(),
                    selection.slot_name,
                    "Configuration selection did not match a slot in the "
                    "resolved graph"));
            }
        }
        return Finish();
    }

private:
    struct SelectionLookup
    {
        std::size_t effective_index;
        std::vector<std::size_t> occurrence_indices;
        bool marked_used;

        SelectionLookup()
            : effective_index(0U)
            , marked_used(false)
        {
        }
    };

    void Emit(const JBeamResolveDiagnostic& diagnostic)
    {
        PushDiagnostic(
            m_graph.diagnostics,
            m_limits.max_diagnostics,
            diagnostic);
    }

    JBeamResolvedGraph Finish()
    {
        SortDiagnostics(m_graph.diagnostics);
        return m_graph;
    }

    bool PreflightRequest()
    {
        if (m_request.part_selections.size() >
            m_limits.max_request_selections)
        {
            const std::size_t rejected_index =
                m_limits.max_request_selections;
            const JBeamSourceSpan span =
                rejected_index < m_request.part_selections.size()
                    ? m_request.part_selections[rejected_index].span
                    : JBeamSourceSpan();
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::PART_SELECTION_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                span,
                std::string(),
                std::string(),
                "Resolve request exceeds the configured part-selection "
                "limit"));
            return false;
        }
        if (m_request.variables.size() >
            m_limits.max_request_variables)
        {
            const std::size_t rejected_index =
                m_limits.max_request_variables;
            const JBeamSourceSpan span =
                rejected_index < m_request.variables.size()
                    ? m_request.variables[rejected_index].span
                    : JBeamSourceSpan();
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::REQUEST_VARIABLE_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                span,
                std::string(),
                std::string(),
                "Resolve request exceeds the configured variable limit"));
            return false;
        }
        if (m_request.variables.size() >
            m_limits.max_variables_per_node)
        {
            const std::size_t rejected_index =
                m_limits.max_variables_per_node;
            const JBeamSourceSpan span =
                rejected_index < m_request.variables.size()
                    ? m_request.variables[rejected_index].span
                    : JBeamSourceSpan();
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::RESOLVED_VARIABLE_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                span,
                std::string(),
                std::string(),
                "Root variables exceed the configured per-node limit"));
            return false;
        }
        return true;
    }

    /// Number of the candidate's own slots that could resolve inside this
    /// package: the declared default exists, or at least one package-local
    /// part declares the slot type. Depth-1 only, so the scan is bounded by
    /// the candidate's own slot count and cannot recurse.
    std::size_t PackageLocalSlotScore(
        const JBeamPartDefinition& candidate) const
    {
        std::size_t score = 0U;
        for (std::size_t i = 0; i < candidate.slots.size(); ++i)
        {
            const JBeamSlotDefinition& slot = candidate.slots[i];
            if (!slot.default_part.empty() &&
                m_parts.find(slot.default_part) != m_parts.end())
            {
                ++score;
                continue;
            }
            const std::map<std::string, std::vector<std::size_t> >::
                const_iterator by_type = m_slot_type_parts.find(slot.name);
            if (by_type != m_slot_type_parts.end() &&
                !by_type->second.empty())
            {
                ++score;
            }
        }
        return score;
    }

    static std::size_t CommonPrefixLength(
        const std::string& left, const std::string& right)
    {
        const std::size_t bound = std::min(left.size(), right.size());
        std::size_t i = 0U;
        while (i < bound && left[i] == right[i])
        {
            ++i;
        }
        return i;
    }

    /// Deterministic package-local stand-in for a declared default that the
    /// package does not ship. Ranking, in order: longest common prefix with
    /// the missing default name (same authored family first), then the
    /// candidate whose own slots can resolve inside this package, then part
    /// name, then package path. Returns an empty string when substitution is
    /// disabled or when no admissible candidate exists.
    std::string SelectPackageLocalSubstitute(
        const JBeamSlotDefinition& slot,
        const std::string& missing_default) const
    {
        if (!m_request.substitute_missing_package_local_defaults ||
            IsDeclinedJBeamSlotSelection(missing_default))
        {
            return std::string();
        }
        const std::map<std::string, std::vector<std::size_t> >::
            const_iterator by_type = m_slot_type_parts.find(slot.name);
        if (by_type == m_slot_type_parts.end())
        {
            return std::string();
        }
        std::string best_name;
        std::string best_path;
        std::size_t best_prefix = 0U;
        std::size_t best_score = 0U;
        bool have_best = false;
        for (std::size_t i = 0; i < by_type->second.size(); ++i)
        {
            const JBeamPartDefinition& candidate =
                m_index.parts[by_type->second[i]];
            if (candidate.name == missing_default ||
                !IsPartAllowed(slot, candidate))
            {
                continue;
            }
            // A part already on the active resolution stack would close a
            // slot cycle; the cycle guard would refuse it anyway.
            bool on_stack = false;
            for (std::size_t j = 0; j < m_stack.size(); ++j)
            {
                if (m_stack[j] == candidate.name)
                {
                    on_stack = true;
                    break;
                }
            }
            if (on_stack)
            {
                continue;
            }
            const std::size_t prefix =
                CommonPrefixLength(candidate.name, missing_default);
            const std::size_t score = PackageLocalSlotScore(candidate);
            const bool better = !have_best ||
                prefix > best_prefix ||
                (prefix == best_prefix &&
                    (score > best_score ||
                        (score == best_score &&
                            (candidate.name < best_name ||
                                (candidate.name == best_name &&
                                    candidate.package_path < best_path)))));
            if (better)
            {
                have_best = true;
                best_prefix = prefix;
                best_score = score;
                best_name = candidate.name;
                best_path = candidate.package_path;
            }
        }
        return best_name;
    }

    void PrepareLookups()
    {
        m_selection_used.assign(
            m_request.part_selections.size(), false);
        for (std::size_t i = 0; i < m_index.parts.size(); ++i)
        {
            m_parts[m_index.parts[i].name].push_back(i);
            for (std::size_t j = 0;
                 j < m_index.parts[i].slot_types.size();
                 ++j)
            {
                m_slot_type_parts[
                    m_index.parts[i].slot_types[j]].push_back(i);
            }
        }
        for (std::size_t i = 0;
             i < m_request.part_selections.size();
             ++i)
        {
            SelectionLookup& lookup =
                m_selections[
                    m_request.part_selections[i].slot_name];
            lookup.effective_index = i;
            lookup.occurrence_indices.push_back(i);
        }
    }

    void ValidateRequest()
    {
        for (std::size_t i = 0;
             i < m_request.part_selections.size();
             ++i)
        {
            const JBeamPartSelection& selection =
                m_request.part_selections[i];
            if (selection.slot_name.empty())
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_PART_SELECTION,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    selection.span,
                    std::string(),
                    selection.slot_name,
                    "Part-selection slot name cannot be empty"));
            }
            if (!selection.part_name.empty() &&
                (selection.part_name.compare(0, 2, "$=") == 0 ||
                 selection.part_name[0] == '$'))
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INVALID_PART_SELECTION,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    selection.span,
                    std::string(),
                    selection.slot_name,
                    "Structural part selections cannot be unresolved "
                    "variable or expression strings"));
            }
        }
        for (std::map<std::string, SelectionLookup>::const_iterator
                 lookup = m_selections.begin();
             lookup != m_selections.end();
             ++lookup)
        {
            for (std::size_t occurrence = 1U;
                 occurrence <
                     lookup->second.occurrence_indices.size();
                 ++occurrence)
            {
                const std::size_t selection_index =
                    lookup->second.occurrence_indices[occurrence];
                const JBeamPartSelection& selection =
                    m_request.part_selections[selection_index];
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        DUPLICATE_PART_SELECTION,
                    JBeamResolveSeverity::WARNING,
                    selection.span,
                    std::string(),
                    selection.slot_name,
                    "Duplicate selection uses the last request entry"));
            }
        }

        std::map<std::string, JBeamSourceSpan> variables;
        for (std::size_t i = 0; i < m_request.variables.size(); ++i)
        {
            const JBeamVariableAssignment& variable =
                m_request.variables[i];
            if (!IsVariableName(variable.name) ||
                !IsScalarVariableValue(variable.value))
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        INVALID_CONFIGURATION_VARIABLE,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    variable.span,
                    std::string(),
                    std::string(),
                    "Configuration variable names must begin with '$' and "
                    "values must be number, string, or Boolean scalars"));
            }
            if (variables.find(variable.name) != variables.end())
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        DUPLICATE_CONFIGURATION_VARIABLE,
                    JBeamResolveSeverity::WARNING,
                    variable.span,
                    std::string(),
                    std::string(),
                    "Duplicate configuration variable uses "
                    "last-assignment semantics"));
            }
            else
            {
                variables.insert(
                    std::make_pair(variable.name, variable.span));
            }
        }
    }

    std::size_t DefinitionCount(const std::string& name, std::size_t legacy_count) const
    {
        const auto found = m_original_definition_counts.find(name);
        return found == m_original_definition_counts.end() ? legacy_count : found->second;
    }

    bool EnsureExactPart(const std::string& name)
    {
        if (m_exact_source == NULL) return true;
        if (m_attempted_exact_parts.find(name) != m_attempted_exact_parts.end()) return true;
        if (m_attempted_exact_parts.size() >= m_limits.max_input_entries ||
            m_selected_index->parts.size() >= m_limits.max_indexed_parts)
        {
            Emit(MakeDiagnostic(JBeamResolveDiagnosticCode::SELECTED_SOURCE_REFUSED,
                JBeamResolveSeverity::ERROR_SEVERITY, JBeamSourceSpan(), name,
                std::string(), "Exact selected lookup/index limit reached"));
            return false;
        }
        JBeamExactPartLoad loaded = m_exact_source->LoadExactPart(name);
        for (const auto& diagnostic : loaded.diagnostics) Emit(diagnostic);
        if (loaded.status == JBeamExactPartLoadStatus::REFUSED ||
            HasErrors(loaded.diagnostics) ||
            (loaded.status == JBeamExactPartLoadStatus::FOUND &&
                (loaded.part.name != name || loaded.original_definition_count == 0U ||
                    loaded.part.body.type != JBeamValueType::OBJECT)))
        {
            Emit(MakeDiagnostic(JBeamResolveDiagnosticCode::SELECTED_SOURCE_REFUSED,
                JBeamResolveSeverity::ERROR_SEVERITY, loaded.part.name_span, name,
                std::string(), "Exact selected definition was refused before fallback"));
            return false;
        }
        m_attempted_exact_parts[name] = true;
        if (loaded.status == JBeamExactPartLoadStatus::MISSING) return true;
        const std::size_t index = m_selected_index->parts.size();
        m_selected_index->parts.push_back(std::move(loaded.part));
        m_parts[name].push_back(index);
        m_original_definition_counts[name] = loaded.original_definition_count;
        return true;
    }

    const JBeamPartDefinition* SelectRoot()
    {
        if (!m_request.root_part_name.empty())
        {
            if (!EnsureExactPart(m_request.root_part_name)) return NULL;
            const std::map<std::string, std::vector<std::size_t> >::
                const_iterator found =
                    m_parts.find(m_request.root_part_name);
            if (found == m_parts.end())
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::ROOT_PART_NOT_FOUND,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    JBeamSourceSpan(),
                    m_request.root_part_name,
                    std::string(),
                    "Requested root part is not present in the package"));
                return NULL;
            }
            if (DefinitionCount(m_request.root_part_name, found->second.size()) != 1U)
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::AMBIGUOUS_PART,
                    JBeamResolveSeverity::WARNING,
                    JBeamSourceSpan(),
                    m_request.root_part_name,
                    std::string(),
                    "Requested root part has duplicate definitions; the "
                    "canonical last definition in package-path order is "
                    "effective"));
            }
            const JBeamPartDefinition& part =
                m_index.parts[found->second.back()];
            if (!HasSlotType(part, "main"))
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::ROOT_PART_NOT_MAIN,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    part.name_span,
                    part.name,
                    std::string(),
                    "Requested root does not declare slotType 'main'"));
                return NULL;
            }
            return &part;
        }

        const JBeamPartDefinition* root = NULL;
        for (std::size_t i = 0; i < m_index.parts.size(); ++i)
        {
            if (!HasSlotType(m_index.parts[i], "main"))
            {
                continue;
            }
            if (root != NULL)
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::MULTIPLE_MAIN_PARTS,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    m_index.parts[i].name_span,
                    m_index.parts[i].name,
                    std::string(),
                    "Package has more than one slotType 'main' part"));
                return NULL;
            }
            root = &m_index.parts[i];
        }
        if (root == NULL)
        {
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::ROOT_PART_NOT_FOUND,
                JBeamResolveSeverity::ERROR_SEVERITY,
                JBeamSourceSpan(),
                std::string(),
                std::string(),
                "Package has no slotType 'main' part"));
        }
        return root;
    }

    const JBeamPartSelection* FindSelection(const std::string& slot_name)
    {
        const std::map<std::string, SelectionLookup>::iterator found =
            m_selections.find(slot_name);
        if (found == m_selections.end())
        {
            return NULL;
        }
        SelectionLookup& lookup = found->second;
        if (!lookup.marked_used)
        {
            for (std::size_t i = 0;
                 i < lookup.occurrence_indices.size();
                 ++i)
            {
                m_selection_used[lookup.occurrence_indices[i]] = true;
            }
            lookup.marked_used = true;
        }
        return &m_request.part_selections[lookup.effective_index];
    }

    bool IsOnStack(const std::string& part_name) const
    {
        return std::find(
                   m_stack.begin(),
                   m_stack.end(),
                   part_name) != m_stack.end();
    }

    void CollectResolvedNodes(
        JBeamResolvedPartNode& node,
        std::vector<JBeamResolvedPartNode*>& output)
    {
        output.push_back(&node);
        for (std::size_t i = 0U; i < node.slots.size(); ++i)
        {
            if (node.slots[i].child)
            {
                CollectResolvedNodes(*node.slots[i].child, output);
            }
        }
    }

    bool ApplyAuthoredTuningDefaults()
    {
        std::vector<JBeamResolvedPartNode*> nodes;
        nodes.reserve(m_graph.resolved_part_count);
        CollectResolvedNodes(*m_graph.root, nodes);

        std::vector<JBeamTuningVariableDefinition> active;
        std::map<std::string, JBeamSourceSpan> seen;
        for (std::size_t node_index = 0U;
             node_index < nodes.size();
             ++node_index)
        {
            const JBeamPartDefinition& part =
                nodes[node_index]->definition;
            for (std::size_t variable_index = 0U;
                 variable_index < part.tuning_variables.size();
                 ++variable_index)
            {
                const JBeamTuningVariableDefinition& variable =
                    part.tuning_variables[variable_index];
                if (active.size() >= m_limits.max_variables_per_node)
                {
                    Emit(MakeDiagnostic(
                        JBeamResolveDiagnosticCode::TUNING_VARIABLE_LIMIT,
                        JBeamResolveSeverity::ERROR_SEVERITY,
                        variable.span,
                        part.name,
                        variable.name,
                        "Resolved authored variables exceed the configured "
                        "vehicle-variable limit"));
                    return false;
                }
                if (seen.find(variable.name) != seen.end())
                {
                    Emit(MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            DUPLICATE_TUNING_VARIABLE,
                        JBeamResolveSeverity::WARNING,
                        variable.span,
                        part.name,
                        variable.name,
                        "Duplicate active authored tuning variable uses "
                        "deterministic resolved-part preorder and the last "
                        "row"));
                }
                else
                {
                    seen.insert(
                        std::make_pair(variable.name, variable.span));
                }
                active.push_back(variable);
            }
        }

        std::map<
            std::string,
            const JBeamTuningVariableDefinition*> effective;
        for (std::size_t i = 0U; i < active.size(); ++i)
        {
            effective[active[i].name] = &active[i];
        }
        bool overrides_valid = true;
        std::set<std::tuple<
            int,
            std::string,
            std::size_t,
            std::size_t,
            std::string> > checked_overrides;
        // Every resolved node carries its own copy of each inherited
        // assignment, so every copy is bounded; each assignment is reported
        // once. A number outside the declared range is used at the nearer
        // bound, as BeamNG uses it; anything but a number cannot be applied.
        for (std::size_t node_index = 0U;
             node_index < nodes.size();
             ++node_index)
        {
            std::vector<JBeamVariableAssignment>& inherited =
                nodes[node_index]->inherited_variables;
            for (std::size_t i = 0U; i < inherited.size(); ++i)
            {
                const std::map<
                    std::string,
                    const JBeamTuningVariableDefinition*>::const_iterator
                    found = effective.find(inherited[i].name);
                if (found == effective.end())
                {
                    continue;
                }
                const std::tuple<
                    int,
                    std::string,
                    std::size_t,
                    std::size_t,
                    std::string> key(
                        static_cast<int>(inherited[i].origin),
                        inherited[i].span.source_name,
                        inherited[i].span.begin.byte_offset,
                        inherited[i].span.end.byte_offset,
                        inherited[i].name);
                const bool first_report =
                    checked_overrides.insert(key).second;
                const JBeamTuningVariableDefinition& definition =
                    *found->second;
                const bool ascending =
                    definition.min_value.number_value <
                    definition.max_value.number_value;
                const JBeamValue& lower_bound =
                    ascending ? definition.min_value : definition.max_value;
                const JBeamValue& upper_bound =
                    ascending ? definition.max_value : definition.min_value;
                JBeamValue& value = inherited[i].value;
                if (value.type != JBeamValueType::NUMBER)
                {
                    if (first_report)
                    {
                        Emit(MakeDiagnostic(
                            JBeamResolveDiagnosticCode::
                                INVALID_TUNING_VARIABLE_OVERRIDE,
                            JBeamResolveSeverity::ERROR_SEVERITY,
                            inherited[i].span,
                            nodes[node_index]->definition.name,
                            inherited[i].name,
                            std::string("Declared range override from ") +
                                VariableOriginToString(inherited[i].origin) +
                                " must be numeric"));
                    }
                    overrides_valid = false;
                    continue;
                }
                const JBeamValue* const bound =
                    value.number_value < lower_bound.number_value
                        ? &lower_bound
                        : value.number_value > upper_bound.number_value
                            ? &upper_bound
                            : NULL;
                if (bound == NULL)
                {
                    continue;
                }
                if (first_report)
                {
                    Emit(MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            TUNING_VARIABLE_OVERRIDE_CLAMPED,
                        JBeamResolveSeverity::WARNING,
                        inherited[i].span,
                        nodes[node_index]->definition.name,
                        inherited[i].name,
                        "given=" + value.scalar_text + ":declared=[" +
                            lower_bound.scalar_text + "," +
                            upper_bound.scalar_text + "]:used=" +
                            bound->scalar_text));
                }
                value.number_value = bound->number_value;
                value.scalar_text = bound->scalar_text;
            }
        }
        if (!overrides_valid)
        {
            return false;
        }

        for (std::size_t node_index = 0U;
             node_index < nodes.size();
             ++node_index)
        {
            const std::size_t inherited =
                nodes[node_index]->inherited_variables.size();
            if (active.size() > m_limits.max_variables_per_node ||
                inherited > m_limits.max_variables_per_node -
                    active.size())
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::RESOLVED_VARIABLE_LIMIT,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    nodes[node_index]->definition.name_span,
                    nodes[node_index]->definition.name,
                    std::string(),
                    "Authored defaults plus configuration/slot variables "
                    "exceed the configured per-node limit"));
                return false;
            }
        }

        std::vector<JBeamVariableAssignment> defaults;
        defaults.reserve(active.size());
        for (std::size_t i = 0U; i < active.size(); ++i)
        {
            JBeamVariableAssignment assignment;
            assignment.name = active[i].name;
            assignment.value = active[i].default_value;
            assignment.span = active[i].default_value.span;
            assignment.origin = JBeamVariableOrigin::AUTHORED_DEFAULT;
            defaults.push_back(assignment);
        }

        for (std::size_t node_index = 0U;
             node_index < nodes.size();
             ++node_index)
        {
            std::vector<JBeamVariableAssignment> merged;
            merged.reserve(
                defaults.size() +
                nodes[node_index]->inherited_variables.size());
            merged.insert(
                merged.end(), defaults.begin(), defaults.end());
            merged.insert(
                merged.end(),
                nodes[node_index]->inherited_variables.begin(),
                nodes[node_index]->inherited_variables.end());
            nodes[node_index]->inherited_variables.swap(merged);
        }
        m_graph.tuning_variables.swap(active);
        return true;
    }

    std::shared_ptr<JBeamResolvedPartNode> ResolveNode(
        const JBeamPartDefinition& part,
        const std::vector<JBeamVariableAssignment>& variables,
        std::size_t depth)
    {
        if (depth > m_limits.max_depth)
        {
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::RESOLVE_DEPTH_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                part.name_span,
                part.name,
                std::string(),
                "Resolved part depth exceeds the configured limit"));
            return std::shared_ptr<JBeamResolvedPartNode>();
        }
        if (m_graph.resolved_part_count >=
            m_limits.max_resolved_parts)
        {
            Emit(MakeDiagnostic(
                JBeamResolveDiagnosticCode::RESOLVED_PART_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                part.name_span,
                part.name,
                std::string(),
                "Resolved graph exceeds the configured part-count limit"));
            return std::shared_ptr<JBeamResolvedPartNode>();
        }
        ++m_graph.resolved_part_count;

        std::shared_ptr<JBeamResolvedPartNode> node(
            new JBeamResolvedPartNode());
        node->definition = part;
        node->inherited_variables = variables;
        m_stack.push_back(part.name);

        // Lazy loading can grow the private selected index. All active
        // recursion reads its node-owned copy, never a vector element which
        // a later lookup may relocate.
        const JBeamPartDefinition& owned_part = node->definition;

        for (std::size_t i = 0; i < owned_part.slots.size(); ++i)
        {
            if (m_exact_source != NULL && HasErrors(m_graph.diagnostics)) break;
            const JBeamSlotDefinition& slot = owned_part.slots[i];
            JBeamResolvedSlot edge;
            edge.definition = slot;
            const JBeamPartSelection* selection =
                FindSelection(slot.name);
            edge.explicitly_selected = selection != NULL;
            edge.selected_part =
                selection != NULL
                    ? selection->part_name
                    : slot.default_part;

            // A `coreSlot` carrying the empty string has stated no choice.
            //
            // The documented `none` sentinel is how a configuration declines
            // a part, and it still declines one here. The empty string is
            // different: it is what a configuration writes for a slot it
            // never decided, and a slot the author marked REQUIRED is not
            // one a configuration silently empties. Read as a decline, a
            // coreSlot's declared default could never apply to any
            // configuration that lists the slot at all -- and the corpus
            // does list them: 49 blank coreSlot entries across the installed
            // packages, including a motorcycle whose every configuration
            // blanks its paint design, its lower fairing, its tail, its
            // muffler and its rear wheel, each with a default the author
            // declared right there in the slot table.
            //
            // So a blank coreSlot resolves exactly as an omitted entry
            // does, default and all, and everything downstream -- the
            // exact-default retry, allow/deny, the package-local
            // substitution and its receipt -- runs unchanged. A blank
            // NON-core slot is still deliberately empty, because an optional
            // slot is exactly the kind a configuration does empty.
            if (slot.core_slot &&
                selection != NULL &&
                selection->part_name.empty())
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::BLANK_CORE_SLOT_TOOK_DEFAULT,
                    JBeamResolveSeverity::WARNING,
                    slot.span,
                    owned_part.name,
                    slot.name,
                    "Required coreSlot left blank by the configuration; "
                    "resolving it as an omitted entry against the declared "
                    "default '" + slot.default_part +
                    "' (a blank core slot states no choice, unlike 'none')"));
                edge.selected_part = slot.default_part;
                selection = NULL;
            }

            // A missing/incompatible requested part is not permission to
            // choose a sibling. Retry the exact authored default once; the
            // ordinary empty, allow/deny, cycle and quota gates below still
            // apply. A wrong-type default then matches the reference
            // editor's result: the slot stays empty with a NOT_ALLOWED
            // status and a warning, never a whole-graph refusal (policy v2).
            // Keep the request and explicit flag unchanged so a retry cannot
            // impersonate an omitted selection.
            if (selection != NULL &&
                !IsDeclinedJBeamSlotSelection(edge.selected_part))
            {
                if (!EnsureExactPart(edge.selected_part))
                {
                    edge.status = JBeamResolvedSlotStatus::SOURCE_REFUSED;
                    node->slots.push_back(edge);
                    break;
                }
                const auto requested = m_parts.find(edge.selected_part);
                const bool missing = requested == m_parts.end();
                if (missing)
                {
                    edge.unresolved_requested_part = selection->part_name;
                }
                if (edge.selected_part != slot.default_part &&
                    (missing || !IsPartAllowed(
                        slot, m_index.parts[requested->second.back()])))
                {
                    if (!IsDeclinedJBeamSlotSelection(slot.default_part) &&
                        !EnsureExactPart(slot.default_part))
                    {
                        edge.status = JBeamResolvedSlotStatus::SOURCE_REFUSED;
                        node->slots.push_back(edge);
                        break;
                    }
                    Emit(MakeDiagnostic(
                        JBeamResolveDiagnosticCode::RETRIED_AUTHORED_DEFAULT_PART,
                        JBeamResolveSeverity::WARNING,
                        selection->span,
                        owned_part.name,
                        slot.name,
                        "Explicit selection '" + edge.selected_part +
                            (missing ? "' is not present in the package"
                                     : "' is not allowed in this slot") +
                            "; retrying exact authored default '" +
                            slot.default_part + "'"));
                    // If both named choices are absent, keep the actual
                    // request as the missing-name witness; do not turn a
                    // different unavailable default into a selected part.
                    // Empty defaults still resolve empty, with the absent
                    // request separately retained above. A present default
                    // goes through the normal strict allow/deny checks.
                    if (!missing ||
                        IsDeclinedJBeamSlotSelection(slot.default_part) ||
                        m_parts.find(slot.default_part) != m_parts.end())
                    {
                        edge.selected_part = slot.default_part;
                    }
                }
            }

            // THE EMPTY/MISSING OUTCOME IS DECIDED HERE, ONCE.
            //
            // The documented `"none"` sentinel means the author declined a
            // part. It is not a part name the package failed to ship, so it
            // is EMPTY, not MISSING. A failed named request that retries
            // this default keeps its separate unresolved-request witness.
            // Historically the sentinel used to fall through to the
            // lookup below, be reported MISSING, and invite every
            // substitution tier to fill a slot somebody deliberately left
            // alone. One did. The guard was then added downstream, which
            // fixed that consumer and left the conflation in place for the
            // next one; the same conflation is why
            // `FindMissingGearboxSlotSelection`'s comment claims to exclude
            // `"none"` while its predicate never could.
            if (IsDeclinedJBeamSlotSelection(edge.selected_part))
            {
                const bool declined_explicitly = !edge.selected_part.empty() ||
                    (selection != NULL &&
                        IsDeclinedJBeamSlotSelection(selection->part_name));
                edge.status = JBeamResolvedSlotStatus::EMPTY;
                if (slot.core_slot)
                {
                    // Still worth naming: the author marked the slot
                    // required and then put nothing in it. Naming it does
                    // not make it a missing part.
                    if (declined_explicitly)
                    {
                        Emit(MakeDiagnostic(
                            JBeamResolveDiagnosticCode::
                                DECLINED_REQUIRED_PART,
                            JBeamResolveSeverity::WARNING,
                            slot.span,
                            owned_part.name,
                            slot.name,
                            "Required coreSlot explicitly declined with an "
                            "empty or 'none' selection; the slot is "
                            "deliberately empty and is never substituted"));
                    }
                    else
                    {
                        // No default declared at all. That IS a gap the
                        // package left in a slot it called required, and it
                        // keeps the status and the token it always had.
                        edge.status = JBeamResolvedSlotStatus::MISSING;
                        Emit(MakeDiagnostic(
                            JBeamResolveDiagnosticCode::MISSING_REQUIRED_PART,
                            JBeamResolveSeverity::WARNING,
                            slot.span,
                            owned_part.name,
                            slot.name,
                            "Required coreSlot resolved to an empty "
                            "selection; the slot stays empty at the "
                            "declared tier"));
                    }
                }
                node->slots.push_back(edge);
                continue;
            }

            std::map<std::string, std::vector<std::size_t> >::
                // The exact source is consulted before MISSING/derivation
                // can be published, including omitted/core-slot defaults.
                // Declined selections already returned above.
                const_iterator found;
            if (!EnsureExactPart(edge.selected_part))
            {
                edge.status = JBeamResolvedSlotStatus::SOURCE_REFUSED;
                node->slots.push_back(edge);
                break;
            }
            found = m_parts.find(edge.selected_part);
            if (found == m_parts.end())
            {
                // Real-world packages reference base-game parts that are
                // not redistributable with the package. Without a
                // substitution policy the slot stays empty at the declared
                // tier with a truthful diagnostic; nothing is guessed.
                const std::string missing_part = edge.selected_part;
                const std::string substitute =
                    SelectPackageLocalSubstitute(slot, missing_part);
                if (!substitute.empty())
                {
                    found = m_parts.find(substitute);
                }
                if (found == m_parts.end())
                {
                    edge.status = JBeamResolvedSlotStatus::MISSING;
                    Emit(MakeDiagnostic(
                        slot.core_slot
                            ? JBeamResolveDiagnosticCode::
                                MISSING_REQUIRED_PART
                            : JBeamResolveDiagnosticCode::
                                MISSING_OPTIONAL_PART,
                        JBeamResolveSeverity::WARNING,
                        slot.span,
                        owned_part.name,
                        slot.name,
                        "Selected part '" + missing_part +
                            "' is not present in the package; the slot "
                            "stays empty at the declared tier"));
                    node->slots.push_back(edge);
                    continue;
                }
                edge.selected_part = substitute;
                ++m_substituted_slot_count;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::
                        SUBSTITUTED_PACKAGE_LOCAL_PART,
                    JBeamResolveSeverity::WARNING,
                    slot.span,
                    owned_part.name,
                    slot.name,
                    "Declared default part '" + missing_part +
                        "' is not present in the package; the package-local "
                        "part '" + substitute + "' for the same slot type is "
                        "substituted as a declared approximation"));
            }
            if (DefinitionCount(edge.selected_part, found->second.size()) != 1U)
            {
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::AMBIGUOUS_PART,
                    JBeamResolveSeverity::WARNING,
                    slot.span,
                    edge.selected_part,
                    slot.name,
                    "Selected part has duplicate package definitions; "
                    "the canonical last definition in package-path "
                    "order is effective"));
            }

            const JBeamPartDefinition& child_part =
                m_index.parts[found->second.back()];
            if (!IsPartAllowed(slot, child_part))
            {
                // BeamNG parity: a part whose slotType the slot does not
                // allow (or slots2 denies) is simply not fitted. Real-world
                // packages author such defaults (a rear final-drive slot
                // defaulting to a front part); refusing the whole vehicle
                // for one slot was stricter than the reference editor, which
                // leaves the slot empty. The edge keeps its truthful
                // NOT_ALLOWED status and carries no child.
                edge.status = JBeamResolvedSlotStatus::NOT_ALLOWED;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::PART_NOT_ALLOWED_IN_SLOT,
                    JBeamResolveSeverity::WARNING,
                    slot.span,
                    child_part.name,
                    slot.name,
                    "Selected part slotType is not allowed, or is denied "
                    "by slots2; the slot stays empty"));
                node->slots.push_back(edge);
                continue;
            }
            if (IsOnStack(child_part.name))
            {
                edge.status = JBeamResolvedSlotStatus::CYCLE;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::SLOT_CYCLE,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    slot.span,
                    child_part.name,
                    slot.name,
                    "Recursive slot selection forms a part cycle"));
                node->slots.push_back(edge);
                continue;
            }

            // Reject graph-shape limits before constructing the child's
            // inherited-variable vector. A part with many rejected slots must
            // not repeatedly allocate and copy variables after the graph is
            // already known to be at its depth or part-count bound.
            if (depth >= m_limits.max_depth)
            {
                edge.status = JBeamResolvedSlotStatus::LIMIT_REJECTED;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::RESOLVE_DEPTH_LIMIT,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    slot.span,
                    child_part.name,
                    slot.name,
                    "Child would exceed the configured resolve depth"));
                node->slots.push_back(edge);
                continue;
            }
            if (m_graph.resolved_part_count >=
                m_limits.max_resolved_parts)
            {
                edge.status = JBeamResolvedSlotStatus::LIMIT_REJECTED;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::RESOLVED_PART_LIMIT,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    slot.span,
                    child_part.name,
                    slot.name,
                    "Child would exceed the configured resolved-part "
                    "limit"));
                node->slots.push_back(edge);
                continue;
            }
            if (variables.size() >
                    m_limits.max_variables_per_node ||
                slot.variables.size() >
                    m_limits.max_variables_per_node -
                        variables.size())
            {
                edge.status = JBeamResolvedSlotStatus::LIMIT_REJECTED;
                Emit(MakeDiagnostic(
                    JBeamResolveDiagnosticCode::RESOLVED_VARIABLE_LIMIT,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    slot.span,
                    child_part.name,
                    slot.name,
                    "Inherited variables exceed the configured per-node "
                    "limit"));
                node->slots.push_back(edge);
                continue;
            }
            std::vector<JBeamVariableAssignment> child_variables;
            const std::size_t child_variable_count =
                variables.size() + slot.variables.size();
            child_variables.reserve(child_variable_count);
            child_variables.insert(
                child_variables.end(),
                variables.begin(),
                variables.end());
            child_variables.insert(
                child_variables.end(),
                slot.variables.begin(),
                slot.variables.end());

            edge.child =
                ResolveNode(child_part, child_variables, depth + 1);
            edge.status = edge.child
                ? JBeamResolvedSlotStatus::RESOLVED
                : JBeamResolvedSlotStatus::LIMIT_REJECTED;
            if (edge.child)
            {
                edge.unresolved_requested_part.clear();
            }
            node->slots.push_back(edge);
        }
        m_stack.pop_back();
        return node;
    }

    const JBeamPackageIndex& m_index;
    const JBeamResolveRequest& m_request;
    const JBeamResolverLimits& m_limits;
    JBeamPackageIndex* m_selected_index;
    JBeamExactPartSource* m_exact_source;
    std::map<std::string, bool> m_attempted_exact_parts;
    std::map<std::string, std::size_t> m_original_definition_counts;
    std::map<std::string, std::vector<std::size_t> > m_parts;
    /// Package part indices grouped by every declared slotType.
    std::map<std::string, std::vector<std::size_t> > m_slot_type_parts;
    std::size_t m_substituted_slot_count = 0U;
    std::map<std::string, SelectionLookup> m_selections;
    std::vector<bool> m_selection_used;
    std::vector<std::string> m_stack;
    JBeamResolvedGraph m_graph;
};

void AppendResolvedNode(
    const JBeamResolvedPartNode& node,
    std::size_t depth,
    std::ostringstream& output)
{
    output
        << "N\t" << depth << '\t'
        << LengthPrefixed(node.definition.name) << '\t'
        << LengthPrefixed(node.definition.package_path) << '\t';
    AppendSpan(node.definition.name_span, output);
    const std::string body = CanonicalValue(node.definition.body);
    output << "\tbody\t" << LengthPrefixed(body) << '\n';

    output << "node-vars\t" << node.inherited_variables.size() << '\n';
    for (std::size_t i = 0; i < node.inherited_variables.size(); ++i)
    {
        AppendVariable(node.inherited_variables[i], output);
    }
    output << "node-slots\t" << node.slots.size() << '\n';
    for (std::size_t i = 0; i < node.slots.size(); ++i)
    {
        const JBeamResolvedSlot& edge = node.slots[i];
        output
            << "E\t" << depth << '\t'
            << SlotStatusToString(edge.status) << '\t'
            << (edge.explicitly_selected ? 1 : 0) << '\t'
            << LengthPrefixed(edge.selected_part) << '\t'
            << "unresolved-request\t"
            << LengthPrefixed(edge.unresolved_requested_part) << '\t';
        AppendSlotDefinition(edge.definition, output);
        output << "child\t" << (edge.child ? 1 : 0) << '\n';
        if (edge.child)
        {
            AppendResolvedNode(*edge.child, depth + 1, output);
        }
    }
}

} // namespace

JBeamResolveDiagnostic::JBeamResolveDiagnostic()
    : code(JBeamResolveDiagnosticCode::PACKAGE_DOCUMENT_NOT_OBJECT)
    , severity(JBeamResolveSeverity::ERROR_SEVERITY)
{
}

JBeamResolverLimits::JBeamResolverLimits()
    : max_input_entries(200000U)
    , max_indexed_parts(100000U)
    , max_resolved_parts(4096U)
    , max_depth(128U)
    , max_slots_per_part(4096U)
    , max_request_selections(4096U)
    , max_request_variables(4096U)
    , max_variables_per_node(4096U)
    , max_table_normalize_work_units(4000000U)
    , max_table_normalize_retained_bytes(64U * 1024U * 1024U)
    , max_diagnostics(4096U)
{
}

JBeamSlotNodeOffset::JBeamSlotNodeOffset()
    : present(false)
    , has_x(false)
    , has_y(false)
    , has_z(false)
{
}

JBeamSlotDefinition::JBeamSlotDefinition()
    : kind(JBeamSlotKind::SLOTS)
    , core_slot(false)
{
}

JBeamTuningVariableDefinition::JBeamTuningVariableDefinition()
    : has_step_dis(false)
    , has_min_dis(false)
    , has_max_dis(false)
    , hide_in_ui(false)
{
}

bool JBeamPackageIndex::IsValid() const
{
    return !HasErrors(diagnostics);
}

JBeamPackageIndex BuildJBeamPackageIndex(
    const std::vector<JBeamPackageSource>& sources,
    const JBeamResolverLimits& limits)
{
    struct OrderedSource
    {
        const JBeamPackageSource* source;
        std::string body_key;
    };
    std::vector<OrderedSource> ordered;
    ordered.reserve(sources.size());
    for (std::size_t i = 0; i < sources.size(); ++i)
    {
        OrderedSource entry;
        entry.source = &sources[i];
        entry.body_key = CanonicalValue(sources[i].document);
        ordered.push_back(entry);
    }
    std::sort(
        ordered.begin(),
        ordered.end(),
        [](const OrderedSource& left, const OrderedSource& right)
        {
            if (left.source->package_path != right.source->package_path)
            {
                return left.source->package_path <
                       right.source->package_path;
            }
            if (left.source->document.span.source_name !=
                right.source->document.span.source_name)
            {
                return left.source->document.span.source_name <
                       right.source->document.span.source_name;
            }
            return left.body_key < right.body_key;
        });

    JBeamPackageIndex index;
    bool limit_reached = false;
    std::size_t input_entry_count = 0U;
    for (std::size_t source_index = 0;
         source_index < ordered.size() && !limit_reached;
         ++source_index)
    {
        const JBeamPackageSource& source = *ordered[source_index].source;
        if (source.document.type != JBeamValueType::OBJECT)
        {
            PushDiagnostic(
                index.diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::PACKAGE_DOCUMENT_NOT_OBJECT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                source.document.span,
                std::string(),
                std::string(),
                "JBeam package documents must be top-level part "
                "dictionaries"));
            continue;
        }
        for (std::size_t field_index = 0;
             field_index < source.document.object_fields.size();
             ++field_index)
        {
            const JBeamObjectField& field =
                source.document.object_fields[field_index];
            if (input_entry_count >= limits.max_input_entries)
            {
                PushDiagnostic(
                    index.diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            INDEX_INPUT_ENTRY_LIMIT,
                        JBeamResolveSeverity::ERROR_SEVERITY,
                        field.key_span,
                        field.key,
                        std::string(),
                        "Package exceeds the configured top-level "
                        "input-entry limit"));
                limit_reached = true;
                break;
            }
            ++input_entry_count;
            if (field.key.empty())
            {
                PushDiagnostic(
                    index.diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    JBeamResolveDiagnosticCode::EMPTY_PART_NAME,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    field.key_span,
                    field.key,
                    std::string(),
                    "Part name cannot be empty"));
                continue;
            }
            if (!field.value ||
                field.value->type != JBeamValueType::OBJECT)
            {
                // Real-world files with misplaced closing braces leave
                // orphan section arrays at the top level; BeamNG's own
                // loader skips non-dictionary roots. The entry is ignored
                // with a warning instead of refusing the package, and the
                // AST retains the exact source bytes.
                PushDiagnostic(
                    index.diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    JBeamResolveDiagnosticCode::PART_NOT_OBJECT,
                    JBeamResolveSeverity::WARNING,
                    field.value ? field.value->span : field.key_span,
                    field.key,
                    std::string(),
                    "Top-level JBeam part value is not a dictionary and "
                    "is ignored"));
                continue;
            }
            if (index.parts.size() >= limits.max_indexed_parts)
            {
                PushDiagnostic(
                    index.diagnostics,
                    limits.max_diagnostics,
                    MakeDiagnostic(
                    JBeamResolveDiagnosticCode::INDEX_PART_LIMIT,
                    JBeamResolveSeverity::ERROR_SEVERITY,
                    field.key_span,
                    field.key,
                    std::string(),
                    "Package exceeds the configured indexed-part limit"));
                limit_reached = true;
                break;
            }

            JBeamPartDefinition part;
            part.name = field.key;
            part.package_path = source.package_path;
            part.name_span = field.key_span;
            part.body = *field.value;
            ParsePartSlotTypes(part, limits, index.diagnostics);
            ParsePartSlots(part, limits, index.diagnostics);
            ParsePartTuningVariables(part, limits, index.diagnostics);
            index.parts.push_back(part);
        }
    }

    std::sort(index.parts.begin(), index.parts.end(), PartLess);
    for (std::size_t i = 1; i < index.parts.size(); ++i)
    {
        if (index.parts[i - 1].name == index.parts[i].name)
        {
            // Real-world packages ship copy-pasted duplicate part
            // definitions. The canonical last definition in package-path
            // order is effective deterministically; every duplicate stays
            // visible as a warning with its exact source span.
            PushDiagnostic(
                index.diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::DUPLICATE_PART,
                JBeamResolveSeverity::WARNING,
                index.parts[i].name_span,
                index.parts[i].name,
                std::string(),
                "Package contains multiple definitions of the same "
                "case-sensitive part name; the canonical last "
                "definition in package-path order is effective"));
        }
    }

    SortDiagnostics(index.diagnostics);
    return index;
}

std::string SerializeCanonicalJBeamPackageIndex(
    const JBeamPackageIndex& index)
{
    std::vector<JBeamPartDefinition> parts = index.parts;
    std::sort(parts.begin(), parts.end(), PartLess);
    std::vector<JBeamResolveDiagnostic> diagnostics = index.diagnostics;
    SortDiagnostics(diagnostics);

    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "ror-beamng-package-index-v2\n";
    output << "parts\t" << parts.size() << '\n';
    for (std::size_t i = 0; i < parts.size(); ++i)
    {
        const JBeamPartDefinition& part = parts[i];
        output
            << "P\t" << LengthPrefixed(part.name) << '\t'
            << LengthPrefixed(part.package_path) << '\t';
        AppendSpan(part.name_span, output);
        output << "\tslot-types\t" << part.slot_types.size();
        for (std::size_t type_index = 0;
             type_index < part.slot_types.size();
             ++type_index)
        {
            output << '\t'
                   << LengthPrefixed(part.slot_types[type_index]);
        }
        const std::string body = CanonicalValue(part.body);
        output << "\tbody\t" << LengthPrefixed(body) << '\n';
        output << "part-slots\t" << part.slots.size() << '\n';
        for (std::size_t slot_index = 0;
             slot_index < part.slots.size();
             ++slot_index)
        {
            output << "S\t";
            AppendSlotDefinition(part.slots[slot_index], output);
        }
        output << "part-tuning-vars\t"
               << part.tuning_variables.size() << '\n';
        for (std::size_t variable_index = 0U;
             variable_index < part.tuning_variables.size();
             ++variable_index)
        {
            AppendTuningVariable(
                part.tuning_variables[variable_index], output);
        }
    }
    output << "diagnostics\t" << diagnostics.size() << '\n';
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        AppendDiagnostic(diagnostics[i], output);
    }
    return output.str();
}

bool JBeamConfigurationResult::IsValid() const
{
    return !HasErrors(diagnostics);
}

JBeamConfigurationResult ParseJBeamConfiguration(
    const JBeamValue& configuration)
{
    return ParseJBeamConfiguration(
        configuration, JBeamResolverLimits());
}

JBeamConfigurationResult ParseJBeamConfiguration(
    const JBeamValue& configuration,
    const JBeamResolverLimits& limits)
{
    JBeamConfigurationResult result;
    if (configuration.type != JBeamValueType::OBJECT)
    {
        PushDiagnostic(
            result.diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
            JBeamResolveDiagnosticCode::CONFIGURATION_NOT_OBJECT,
            JBeamResolveSeverity::ERROR_SEVERITY,
            configuration.span,
            std::string(),
            std::string(),
            "BeamNG configuration must be a dictionary"));
        return result;
    }

    const JBeamObjectField* parts = NULL;
    const JBeamObjectField* variables = NULL;
    for (std::size_t i = 0;
         i < configuration.object_fields.size();
         ++i)
    {
        const JBeamObjectField& field =
            configuration.object_fields[i];
        if (field.key != "parts" && field.key != "vars")
        {
            continue;
        }
        const JBeamObjectField*& effective =
            field.key == "parts" ? parts : variables;
        if (effective != NULL)
        {
            PushDiagnostic(
                result.diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    DUPLICATE_CONFIGURATION_SECTION,
                JBeamResolveSeverity::WARNING,
                field.key_span,
                std::string(),
                field.key,
                "Only the last duplicate configuration section is "
                "effective"));
        }
        effective = &field;
    }

    bool resource_limit_reached = false;
    if (parts != NULL && parts->value &&
        parts->value->type == JBeamValueType::OBJECT &&
        parts->value->object_fields.size() >
            limits.max_request_selections)
    {
        const std::size_t rejected_index =
            limits.max_request_selections;
        const JBeamSourceSpan span =
            rejected_index < parts->value->object_fields.size()
                ? parts->value->object_fields[rejected_index].key_span
                : parts->key_span;
        PushDiagnostic(
            result.diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::PART_SELECTION_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                span,
                std::string(),
                "parts",
                "Configuration exceeds the configured part-selection "
                "limit"));
        resource_limit_reached = true;
    }
    if (variables != NULL && variables->value &&
        variables->value->type == JBeamValueType::OBJECT &&
        variables->value->object_fields.size() >
            limits.max_request_variables)
    {
        const std::size_t rejected_index =
            limits.max_request_variables;
        const JBeamSourceSpan span =
            rejected_index < variables->value->object_fields.size()
                ? variables->value->object_fields[rejected_index].key_span
                : variables->key_span;
        PushDiagnostic(
            result.diagnostics,
            limits.max_diagnostics,
            MakeDiagnostic(
                JBeamResolveDiagnosticCode::REQUEST_VARIABLE_LIMIT,
                JBeamResolveSeverity::ERROR_SEVERITY,
                span,
                std::string(),
                "vars",
                "Configuration exceeds the configured variable limit"));
        resource_limit_reached = true;
    }
    if (resource_limit_reached)
    {
        SortDiagnostics(result.diagnostics);
        return result;
    }

    if (parts != NULL)
    {
        if (!parts->value ||
            parts->value->type != JBeamValueType::OBJECT)
        {
            PushDiagnostic(
                result.diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    INVALID_CONFIGURATION_SECTION,
                JBeamResolveSeverity::ERROR_SEVERITY,
                parts->value ? parts->value->span : parts->key_span,
                std::string(),
                "parts",
                "Configuration parts must be a dictionary"));
        }
        else
        {
            std::set<std::string> seen;
            for (std::size_t i = 0;
                 i < parts->value->object_fields.size();
                 ++i)
            {
                const JBeamObjectField& field =
                    parts->value->object_fields[i];
                if (!field.value ||
                    field.value->type != JBeamValueType::STRING ||
                    (!field.value->scalar_text.empty() &&
                     !IsStructuralString(*field.value)))
                {
                    PushDiagnostic(
                        result.diagnostics,
                        limits.max_diagnostics,
                        MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            INVALID_PART_SELECTION,
                        JBeamResolveSeverity::ERROR_SEVERITY,
                        field.value
                            ? field.value->span
                            : field.key_span,
                        std::string(),
                        field.key,
                        "Configuration part selections must be strings"));
                    continue;
                }
                if (!seen.insert(field.key).second)
                {
                    PushDiagnostic(
                        result.diagnostics,
                        limits.max_diagnostics,
                        MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            DUPLICATE_PART_SELECTION,
                        JBeamResolveSeverity::WARNING,
                        field.key_span,
                        std::string(),
                        field.key,
                        "Duplicate selection uses last-assignment "
                        "semantics"));
                }
                JBeamPartSelection selection;
                selection.slot_name = field.key;
                selection.part_name = field.value->scalar_text;
                selection.span = field.value->span;
                result.request.part_selections.push_back(selection);
            }
        }
    }

    if (variables != NULL)
    {
        if (!variables->value ||
            variables->value->type != JBeamValueType::OBJECT)
        {
            PushDiagnostic(
                result.diagnostics,
                limits.max_diagnostics,
                MakeDiagnostic(
                JBeamResolveDiagnosticCode::
                    INVALID_CONFIGURATION_SECTION,
                JBeamResolveSeverity::ERROR_SEVERITY,
                variables->value
                    ? variables->value->span
                    : variables->key_span,
                std::string(),
                "vars",
                "Configuration vars must be a dictionary"));
        }
        else
        {
            std::set<std::string> seen;
            for (std::size_t i = 0;
                 i < variables->value->object_fields.size();
                 ++i)
            {
                const JBeamObjectField& field =
                    variables->value->object_fields[i];
                if (!IsVariableName(field.key) ||
                    !field.value ||
                    !IsScalarVariableValue(*field.value))
                {
                    PushDiagnostic(
                        result.diagnostics,
                        limits.max_diagnostics,
                        MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            INVALID_CONFIGURATION_VARIABLE,
                        JBeamResolveSeverity::ERROR_SEVERITY,
                        field.value
                            ? field.value->span
                            : field.key_span,
                        std::string(),
                        std::string(),
                        "Configuration variable names must begin with '$' "
                        "and values must be number, string, or Boolean "
                        "scalars"));
                    continue;
                }
                if (!seen.insert(field.key).second)
                {
                    PushDiagnostic(
                        result.diagnostics,
                        limits.max_diagnostics,
                        MakeDiagnostic(
                        JBeamResolveDiagnosticCode::
                            DUPLICATE_CONFIGURATION_VARIABLE,
                        JBeamResolveSeverity::WARNING,
                        field.key_span,
                        std::string(),
                        std::string(),
                        "Duplicate variable uses last-assignment "
                        "semantics"));
                }
                JBeamVariableAssignment assignment;
                assignment.name = field.key;
                assignment.value = *field.value;
                assignment.span = field.value->span;
                assignment.origin = JBeamVariableOrigin::CONFIGURATION;
                result.request.variables.push_back(assignment);
            }
        }
    }
    SortDiagnostics(result.diagnostics);
    if (!result.diagnostics.empty() &&
        IsDiagnosticLimit(result.diagnostics.back()))
    {
        // A capped report cannot prove which later entries were valid. Do not
        // expose a partially parsed request that a caller could accidentally
        // resolve after ignoring IsValid().
        result.request = JBeamResolveRequest();
    }
    return result;
}

JBeamResolvedSlot::JBeamResolvedSlot()
    : explicitly_selected(false)
    , status(JBeamResolvedSlotStatus::EMPTY)
{
}

JBeamResolvedGraph::JBeamResolvedGraph()
    : resolved_part_count(0)
{
}

bool JBeamResolvedGraph::IsValid() const
{
    return root && !HasErrors(diagnostics);
}

JBeamResolvedGraph ResolveJBeamPartGraph(
    const JBeamPackageIndex& index,
    const JBeamResolveRequest& request,
    const JBeamResolverLimits& limits)
{
    return GraphResolver(index, request, limits).Run();
}

JBeamResolvedGraph ResolveJBeamPartGraphFromExactSource(
    JBeamExactPartSource& source, const JBeamResolveRequest& request,
    const JBeamResolverLimits& limits)
{
    JBeamPackageIndex selected;
    return GraphResolver(selected, request, limits, &selected, &source).Run();
}

std::string SerializeCanonicalJBeamResolvedGraph(
    const JBeamResolvedGraph& graph)
{
    std::vector<JBeamResolveDiagnostic> diagnostics = graph.diagnostics;
    SortDiagnostics(diagnostics);
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << "ror-beamng-resolved-graph-v3\n";
    output << "resolver-policy\t"
           << LengthPrefixed(JBeamPartResolverPolicyIdentifier()) << '\n';
    output
        << "request-root\t"
        << LengthPrefixed(graph.request.root_part_name) << '\n';
    output
        << "request-package-local-default-substitution\t"
        << (graph.request.substitute_missing_package_local_defaults
                ? "1" : "0")
        << '\n';
    output
        << "request-selections\t"
        << graph.request.part_selections.size() << '\n';
    for (std::size_t i = 0;
         i < graph.request.part_selections.size();
         ++i)
    {
        const JBeamPartSelection& selection =
            graph.request.part_selections[i];
        output
            << "Q\t" << LengthPrefixed(selection.slot_name) << '\t'
            << LengthPrefixed(selection.part_name) << '\t';
        AppendSpan(selection.span, output);
        output << '\n';
    }
    output << "request-vars\t" << graph.request.variables.size() << '\n';
    for (std::size_t i = 0; i < graph.request.variables.size(); ++i)
    {
        AppendVariable(graph.request.variables[i], output);
    }
    output << "active-tuning-vars\t"
           << graph.tuning_variables.size() << '\n';
    for (std::size_t i = 0U;
         i < graph.tuning_variables.size();
         ++i)
    {
        AppendTuningVariable(graph.tuning_variables[i], output);
    }
    output << "resolved-parts\t" << graph.resolved_part_count << '\n';
    output << "root\t" << (graph.root ? 1 : 0) << '\n';
    if (graph.root)
    {
        AppendResolvedNode(*graph.root, 0, output);
    }
    output << "diagnostics\t" << diagnostics.size() << '\n';
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        AppendDiagnostic(diagnostics[i], output);
    }
    return output.str();
}

const JBeamVariableAssignment* FindEffectiveJBeamVariable(
    const JBeamResolvedPartNode& node,
    const std::string& name)
{
    for (std::size_t i = node.inherited_variables.size(); i > 0; --i)
    {
        if (node.inherited_variables[i - 1].name == name)
        {
            return &node.inherited_variables[i - 1];
        }
    }
    return NULL;
}

const JBeamTuningVariableDefinition* FindEffectiveJBeamTuningVariable(
    const JBeamResolvedGraph& graph,
    const std::string& name)
{
    for (std::size_t i = graph.tuning_variables.size(); i > 0U; --i)
    {
        if (graph.tuning_variables[i - 1U].name == name)
        {
            return &graph.tuning_variables[i - 1U];
        }
    }
    return NULL;
}

const char* JBeamResolveDiagnosticCodeToString(
    JBeamResolveDiagnosticCode code)
{
    switch (code)
    {
    case JBeamResolveDiagnosticCode::PACKAGE_DOCUMENT_NOT_OBJECT:
        return "package-document-not-object";
    case JBeamResolveDiagnosticCode::PART_NOT_OBJECT:
        return "part-not-object";
    case JBeamResolveDiagnosticCode::EMPTY_PART_NAME:
        return "empty-part-name";
    case JBeamResolveDiagnosticCode::MISSING_SLOT_TYPE:
        return "missing-slot-type";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_TYPE:
        return "invalid-slot-type";
    case JBeamResolveDiagnosticCode::DUPLICATE_SLOT_TYPE:
        return "duplicate-slot-type";
    case JBeamResolveDiagnosticCode::DUPLICATE_PART:
        return "duplicate-part";
    case JBeamResolveDiagnosticCode::MULTIPLE_MAIN_PARTS:
        return "multiple-main-parts";
    case JBeamResolveDiagnosticCode::DUPLICATE_SLOT_SECTION:
        return "duplicate-slot-section";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_TABLE:
        return "invalid-slot-table";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_ROW:
        return "invalid-slot-row";
    case JBeamResolveDiagnosticCode::MISSING_SLOT_FIELD:
        return "missing-slot-field";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_FIELD:
        return "invalid-slot-field";
    case JBeamResolveDiagnosticCode::DUPLICATE_SLOT:
        return "duplicate-slot";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_VARIABLE:
        return "invalid-slot-variable";
    case JBeamResolveDiagnosticCode::INVALID_SLOT_NODE_OFFSET:
        return "invalid-slot-node-offset";
    case JBeamResolveDiagnosticCode::DUPLICATE_TUNING_VARIABLE_SECTION:
        return "duplicate-tuning-variable-section";
    case JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_TABLE:
        return "invalid-tuning-variable-table";
    case JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_ROW:
        return "invalid-tuning-variable-row";
    case JBeamResolveDiagnosticCode::MISSING_TUNING_VARIABLE_FIELD:
        return "missing-tuning-variable-field";
    case JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_FIELD:
        return "invalid-tuning-variable-field";
    case JBeamResolveDiagnosticCode::INVALID_TUNING_VARIABLE_OVERRIDE:
        return "invalid-tuning-variable-override";
    case JBeamResolveDiagnosticCode::DUPLICATE_TUNING_VARIABLE:
        return "duplicate-tuning-variable";
    case JBeamResolveDiagnosticCode::TUNING_VARIABLE_LIMIT:
        return "tuning-variable-limit";
    case JBeamResolveDiagnosticCode::INDEX_INPUT_ENTRY_LIMIT:
        return "index-input-entry-limit";
    case JBeamResolveDiagnosticCode::INDEX_PART_LIMIT:
        return "index-part-limit";
    case JBeamResolveDiagnosticCode::CONFIGURATION_NOT_OBJECT:
        return "configuration-not-object";
    case JBeamResolveDiagnosticCode::INVALID_CONFIGURATION_SECTION:
        return "invalid-configuration-section";
    case JBeamResolveDiagnosticCode::DUPLICATE_CONFIGURATION_SECTION:
        return "duplicate-configuration-section";
    case JBeamResolveDiagnosticCode::INVALID_PART_SELECTION:
        return "invalid-part-selection";
    case JBeamResolveDiagnosticCode::DUPLICATE_PART_SELECTION:
        return "duplicate-part-selection";
    case JBeamResolveDiagnosticCode::PART_SELECTION_LIMIT:
        return "part-selection-limit";
    case JBeamResolveDiagnosticCode::INVALID_CONFIGURATION_VARIABLE:
        return "invalid-configuration-variable";
    case JBeamResolveDiagnosticCode::DUPLICATE_CONFIGURATION_VARIABLE:
        return "duplicate-configuration-variable";
    case JBeamResolveDiagnosticCode::REQUEST_VARIABLE_LIMIT:
        return "request-variable-limit";
    case JBeamResolveDiagnosticCode::DIAGNOSTIC_LIMIT:
        return "diagnostic-limit";
    case JBeamResolveDiagnosticCode::INDEX_INVALID:
        return "index-invalid";
    case JBeamResolveDiagnosticCode::ROOT_PART_NOT_FOUND:
        return "root-part-not-found";
    case JBeamResolveDiagnosticCode::ROOT_PART_NOT_MAIN:
        return "root-part-not-main";
    case JBeamResolveDiagnosticCode::AMBIGUOUS_PART:
        return "ambiguous-part";
    case JBeamResolveDiagnosticCode::MISSING_REQUIRED_PART:
        return "missing-required-part";
    case JBeamResolveDiagnosticCode::DECLINED_REQUIRED_PART:
        return "DECLINED_REQUIRED_PART";
    case JBeamResolveDiagnosticCode::MISSING_OPTIONAL_PART:
        return "missing-optional-part";
    case JBeamResolveDiagnosticCode::SUBSTITUTED_PACKAGE_LOCAL_PART:
        return "substituted-package-local-part";
    case JBeamResolveDiagnosticCode::RETRIED_AUTHORED_DEFAULT_PART:
        return "retried-authored-default-part";
    case JBeamResolveDiagnosticCode::BLANK_CORE_SLOT_TOOK_DEFAULT:
        return "blank-core-slot-took-default";
    case JBeamResolveDiagnosticCode::TUNING_VARIABLE_OVERRIDE_CLAMPED:
        return "tuning-variable-override-clamped";
    case JBeamResolveDiagnosticCode::TUNING_VARIABLE_DEFAULT_CLAMPED:
        return "tuning-variable-default-clamped";
    case JBeamResolveDiagnosticCode::SELECTED_SOURCE_REFUSED:
        return "selected-source-refused";
    case JBeamResolveDiagnosticCode::SELECTED_SOURCE_POLICY_REFUSED:
        return "selected-source-policy-refused";
    case JBeamResolveDiagnosticCode::PART_NOT_ALLOWED_IN_SLOT:
        return "part-not-allowed-in-slot";
    case JBeamResolveDiagnosticCode::SLOT_CYCLE:
        return "slot-cycle";
    case JBeamResolveDiagnosticCode::RESOLVE_DEPTH_LIMIT:
        return "resolve-depth-limit";
    case JBeamResolveDiagnosticCode::RESOLVED_PART_LIMIT:
        return "resolved-part-limit";
    case JBeamResolveDiagnosticCode::RESOLVED_VARIABLE_LIMIT:
        return "resolved-variable-limit";
    case JBeamResolveDiagnosticCode::UNUSED_PART_SELECTION:
        return "unused-part-selection";
    }
    return "unknown";
}

} // namespace BeamNG
} // namespace RoR
