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

/// @file JBeamSyntax.h
/// @brief Dependency-free relaxed-JBeam lexer, parser, and table normalizer.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace RoR {
namespace BeamNG {

/// Source positions are one-based. byte_offset and end positions are
/// half-open; columns count source bytes so diagnostics are locale-independent.
struct JBeamSourcePosition
{
    std::uint64_t byte_offset;
    std::uint64_t line;
    std::uint64_t column;

    JBeamSourcePosition();
};

struct JBeamSourceSpan
{
    std::string source_name;
    JBeamSourcePosition begin;
    JBeamSourcePosition end;

    JBeamSourceSpan();
};

enum class JBeamDiagnosticSeverity
{
    WARNING,
    ERROR_SEVERITY
};

enum class JBeamDiagnosticCode
{
    SOURCE_SIZE_LIMIT,
    TOKEN_LIMIT,
    NODE_LIMIT,
    DEPTH_LIMIT,
    STRING_SIZE_LIMIT,
    DIAGNOSTIC_LIMIT,
    INVALID_CHARACTER,
    UNTERMINATED_BLOCK_COMMENT,
    UNTERMINATED_STRING,
    INVALID_STRING_CHARACTER,
    INVALID_UTF8,
    INVALID_ESCAPE,
    INVALID_UNICODE_ESCAPE,
    INVALID_UNICODE_SURROGATE,
    INVALID_NUMBER,
    NON_FINITE_NUMBER,
    EXPECTED_VALUE,
    EXPECTED_OBJECT_KEY,
    EXPECTED_COLON,
    EXPECTED_OBJECT_END,
    EXPECTED_ARRAY_END,
    UNEXPECTED_COMMA,
    TRAILING_CONTENT,
    CONCATENATED_ROOT_OBJECT,
    DUPLICATE_OBJECT_KEY,
    DUPLICATE_TABLE_HEADER,
    TABLE_ROW_TOO_SHORT,
    TABLE_ROW_TOO_LONG,
    TABLE_INVALID_ENTRY,
    NORMALIZE_WORK_LIMIT,
    NORMALIZE_RETAINED_BYTES_LIMIT,
    // Appended so every earlier code keeps its value.
    SURPLUS_ROOT_CLOSER
};

struct JBeamDiagnostic
{
    JBeamDiagnosticCode code;
    JBeamDiagnosticSeverity severity;
    JBeamSourceSpan span;
    std::string message;

    JBeamDiagnostic();
};

struct JBeamParseLimits
{
    std::size_t max_source_bytes;
    std::size_t max_tokens;
    std::size_t max_nodes;
    std::size_t max_depth;
    std::size_t max_string_bytes;
    std::size_t max_diagnostics;

    JBeamParseLimits();
};

enum class JBeamTokenKind
{
    LEFT_BRACE,
    RIGHT_BRACE,
    LEFT_BRACKET,
    RIGHT_BRACKET,
    COLON,
    COMMA,
    STRING,
    /// An unquoted object key, e.g. `{coreSlot:true}`. Accepted only where a
    /// key may appear; a bare word in a VALUE position is still an error.
    BARE_KEY,
    NUMBER,
    TRUE_VALUE,
    FALSE_VALUE,
    NULL_VALUE,
    END_OF_INPUT,
    INVALID
};

struct JBeamToken
{
    JBeamTokenKind kind;
    /// Decoded UTF-8 for STRING, the exact source spelling for NUMBER, and an
    /// empty string for punctuation and keywords.
    std::string text;
    JBeamSourceSpan span;

    JBeamToken();
};

struct JBeamLexResult
{
    std::vector<JBeamToken> tokens;
    std::vector<JBeamDiagnostic> diagnostics;

    bool IsValid() const;
};

struct JBeamValue;

/// Object fields remain in source order. They intentionally are not stored in
/// a map: duplicate keys are meaningful input and BeamNG-compatible
/// last-assignment behavior must remain observable to later resolution stages.
struct JBeamObjectField
{
    std::string key;
    JBeamSourceSpan key_span;
    std::shared_ptr<const JBeamValue> value;

    JBeamObjectField();
};

enum class JBeamValueType
{
    NULL_VALUE,
    BOOLEAN,
    NUMBER,
    STRING,
    ARRAY,
    OBJECT
};

struct JBeamValue
{
    JBeamValueType type;
    JBeamSourceSpan span;
    bool boolean_value;
    double number_value;
    /// Exact token spelling for NUMBER and decoded UTF-8 for STRING.
    std::string scalar_text;
    std::vector<JBeamValue> array_values;
    std::vector<JBeamObjectField> object_fields;

    JBeamValue();
};

struct JBeamParseResult
{
    JBeamValue root;
    std::vector<JBeamDiagnostic> diagnostics;
    bool has_root;

    JBeamParseResult();
    bool IsValid() const;
};

/// Lexes strict JSON tokens plus JBeam's line/block comments. Comments are not
/// emitted because their source ranges are not semantically observable.
JBeamLexResult LexJBeam(
    const std::string& source,
    const std::string& source_name,
    const JBeamParseLimits& limits = JBeamParseLimits());

/// Parses JSON values with JBeam's optional commas. A repeated comma BETWEEN
/// array values is accepted -- in a dialect whose commas are already optional
/// separators there is nothing between them to be an element, so `[a,,b]` is
/// `[a, b]` -- and a leading zero is accepted, because `00` has exactly one
/// reading. Both are relaxed only because shipped content requires them.
/// An unquoted object KEY is accepted, because `{coreSlot:true}` can only mean
/// the key "coreSlot" -- but only in key position: a bare word where a value
/// belongs is still an error, and so is a bare `true`/`false`/`null` as a key,
/// which lexes as the value it spells. A comma between an object key and its
/// colon is skipped, because `"k",:v` can only mean `"k":v`; the colon must
/// still follow. A closing bracket after the complete root value closes
/// nothing, so it is skipped with a SURPLUS_ROOT_CLOSER warning; any other
/// content there is still TRAILING_CONTENT. Leading commas, repeated commas between OBJECT members, NaN
/// and Infinity remain errors: none of those is merely a stricter spelling of
/// an unambiguous value. Variable references and strings beginning with "$="
/// remain inert strings: the allowlisted, resource-bounded expression
/// evaluator is a separate resolver stage.
JBeamParseResult ParseJBeam(
    const std::string& source,
    const std::string& source_name,
    const JBeamParseLimits& limits = JBeamParseLimits());

/// Returns the last field with this exact case-sensitive key, matching JBeam's
/// assignment behavior while retaining every earlier duplicate in the AST.
const JBeamObjectField* FindLastJBeamObjectField(
    const JBeamValue& object,
    const std::string& key);

enum class JBeamFieldOrigin
{
    INHERITED_DEFAULT,
    POSITIONAL_CELL,
    ROW_LOCAL_OVERRIDE
};

struct JBeamTableColumn
{
    std::string name;
    JBeamSourceSpan span;
};

struct JBeamFieldAssignment
{
    std::string name;
    JBeamFieldOrigin origin;
    JBeamSourceSpan span;
    std::shared_ptr<const JBeamValue> value;
};

struct JBeamInheritedFieldIndex
{
    std::string name;
    /// Ascending indexes into JBeamNormalizedDataRow's shared inherited
    /// assignment storage.
    std::vector<std::size_t> assignment_indices;
};

struct JBeamInheritedAssignmentIndex
{
    /// Sorted by exact case-sensitive field name for deterministic
    /// logarithmic lookup.
    std::vector<JBeamInheritedFieldIndex> fields;
};

struct JBeamNormalizedDataRow
{
    JBeamSourceSpan span;
    /// The original row, including a trailing row-local dictionary.
    JBeamValue raw_row;
    /// All defaults for the containing table share one immutable-after-build
    /// backing store. Only the prefix ending at inherited_assignment_count was
    /// active for this row. This keeps duplicate history without copying an
    /// ever-growing defaults vector into every subsequent row.
    std::shared_ptr<const std::vector<JBeamFieldAssignment> >
        inherited_assignment_storage;
    std::size_t inherited_assignment_count = 0;
    /// Shared table-wide index. A prefix search selects the last assignment
    /// that existed when this row was encountered without scanning all prior
    /// default modifiers.
    std::shared_ptr<const JBeamInheritedAssignmentIndex>
        inherited_assignment_index;
    /// Header/cell assignments in column order, including duplicate headers.
    std::vector<JBeamFieldAssignment> positional_assignments;
    /// Trailing dictionary assignments in source order, including duplicates.
    std::vector<JBeamFieldAssignment> row_local_assignments;
};

enum class JBeamNormalizedTableEntryKind
{
    DEFAULT_MODIFIER,
    DATA_ROW,
    INVALID_ENTRY
};

struct JBeamNormalizedTableEntry
{
    JBeamNormalizedTableEntryKind kind;
    JBeamValue raw_value;
    JBeamNormalizedDataRow data_row;
};

struct JBeamNormalizedTable
{
    /// Stable path with duplicate object-field occurrences encoded as #N.
    std::string path;
    JBeamSourceSpan span;
    std::vector<JBeamTableColumn> columns;
    std::vector<JBeamNormalizedTableEntry> entries;
};

struct JBeamNormalizeLimits
{
    /// Deterministic units charged for discovery, retained-value measurement,
    /// table entries, cells, and assignments.
    std::size_t max_work_units;
    /// Version-1 portable logical bytes retained by normalized tables, copied
    /// value payloads, assignment indexes, strings, and diagnostics. Fixed
    /// record weights are independent of standard-library ABI and iterator
    /// debugging; the source AST and allocator bookkeeping are excluded.
    std::size_t max_retained_bytes;
    std::size_t max_diagnostics;

    JBeamNormalizeLimits();
};

struct JBeamNormalizeResult
{
    std::vector<JBeamNormalizedTable> tables;
    std::vector<JBeamDiagnostic> diagnostics;
    std::size_t work_units = 0;
    std::size_t retained_bytes = 0;

    bool IsValid() const;
};

/// Discovers table-shaped arrays reachable through object fields and preserves
/// their raw entries. An array is treated as a table only when its first item
/// is a non-empty array consisting entirely of strings. Array contents are
/// data, not additional section roots, so discovery deliberately does not
/// recurse into them. This prevents nested row payloads from being normalized
/// repeatedly while retaining the root/object/section contract. A final object
/// in a data row is treated as the documented row-local override dictionary
/// only when it appears after all header columns; an object in a declared
/// positional column remains an ordinary cell.
JBeamNormalizeResult NormalizeJBeamTables(
    const JBeamValue& root,
    const JBeamNormalizeLimits& limits = JBeamNormalizeLimits());

struct JBeamFieldLookupMetrics
{
    std::size_t work_units;

    JBeamFieldLookupMetrics();
};

/// Applies JBeam precedence (defaults, positional cells, row-local overrides)
/// and returns the final exact-case field assignment without discarding any
/// source history from the normalized row. Normalized rows use a shared
/// table-wide index, making inherited-default lookup logarithmic in the number
/// of assignments rather than a reverse scan of the whole defaults prefix.
const JBeamFieldAssignment* FindEffectiveJBeamField(
    const JBeamNormalizedDataRow& row,
    const std::string& name,
    JBeamFieldLookupMetrics* metrics = NULL);

const char* JBeamDiagnosticCodeToString(JBeamDiagnosticCode code);

} // namespace BeamNG
} // namespace RoR
