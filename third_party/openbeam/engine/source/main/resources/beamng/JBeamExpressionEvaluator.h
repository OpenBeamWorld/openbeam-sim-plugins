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

/// @file JBeamExpressionEvaluator.h
/// @brief Deterministic, resource-bounded evaluator for a safe JBeam subset.

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace RoR {
namespace BeamNG {

enum class JBeamExpressionValueType
{
    NIL_VALUE,
    BOOLEAN,
    NUMBER,
    STRING
};

/// Scalar values accepted by the expression evaluator. NUMBER values must be
/// finite IEEE-754 binary64 values. NIL is both a valid explicit value and the
/// value of an absent variable, matching documented JBeam existence checks.
struct JBeamExpressionValue
{
    JBeamExpressionValueType type;
    bool boolean_value;
    double number_value;
    std::string string_value;

    JBeamExpressionValue();

    static JBeamExpressionValue Nil();
    static JBeamExpressionValue Boolean(bool value);
    static JBeamExpressionValue Number(double value);
    static JBeamExpressionValue String(const std::string& value);
};

/// Assignments are searched from back to front. This preserves the existing
/// JBeam resolver's last-assignment-wins contract without requiring a map or
/// exposing locale-dependent ordering.
struct JBeamExpressionVariable
{
    std::string name;
    JBeamExpressionValue value;
};

struct JBeamExpressionEnvironment
{
    std::vector<JBeamExpressionVariable> variables;
};

struct JBeamExpressionLimits
{
    std::size_t max_expression_bytes;
    std::size_t max_tokens;
    std::size_t max_depth;
    /// Per-call argument ceiling. The evaluator also enforces an immutable
    /// allowlist ceiling of 64 arguments even if a caller raises this value.
    std::size_t max_function_arguments;
    /// Deterministic work charged for input bytes, emitted/consumed tokens,
    /// environment validation/lookups, and evaluated operations.
    std::size_t max_work_units;
    std::size_t max_string_bytes;
    std::size_t max_output_string_bytes;
    std::size_t max_variables;
    std::size_t max_variable_name_bytes;
    std::size_t max_environment_string_bytes;

    JBeamExpressionLimits();
};

enum class JBeamExpressionDiagnosticCode
{
    EXPRESSION_SIZE_LIMIT,
    MISSING_EXPRESSION_PREFIX,
    TOKEN_LIMIT,
    DEPTH_LIMIT,
    WORK_LIMIT,
    STRING_SIZE_LIMIT,
    OUTPUT_STRING_SIZE_LIMIT,
    ENVIRONMENT_LIMIT,
    INVALID_ENVIRONMENT_VARIABLE,
    INVALID_ENVIRONMENT_VALUE,
    INVALID_CHARACTER,
    INVALID_UTF8,
    INVALID_NUMBER,
    NON_FINITE_NUMBER,
    UNTERMINATED_STRING,
    INVALID_ESCAPE,
    EXPECTED_VALUE,
    EXPECTED_TOKEN,
    TRAILING_CONTENT,
    UNSUPPORTED_FUNCTION,
    UNSUPPORTED_CASE_SIGNATURE,
    TYPE_MISMATCH,
    DIVISION_BY_ZERO,
    NON_DETERMINISTIC_OPERAND,
    NON_FINITE_RESULT,
    FUNCTION_ARITY,
    FUNCTION_ARGUMENT_LIMIT,
    INVALID_FUNCTION_ARGUMENT
};

struct JBeamExpressionDiagnostic
{
    JBeamExpressionDiagnosticCode code;
    /// Zero-based byte offset in the decoded JBeam string.
    std::size_t byte_offset;
    std::string message;

    JBeamExpressionDiagnostic();
};

struct JBeamExpressionResult
{
    JBeamExpressionValue value;
    std::vector<JBeamExpressionDiagnostic> diagnostics;
    std::size_t token_count;
    std::size_t work_units;
    bool has_value;
    /// Names, in first-read order and deduplicated, of `$variables` the
    /// expression read that the environment did not declare. Reading one is
    /// not an error here - it yields nil, exactly as a variable declared nil
    /// would - but it is the single most common reason a shipped mod's
    /// arithmetic then fails a type check, so the caller is told which name
    /// was missing rather than having to infer it from a byte offset.
    std::vector<std::string> undeclared_variables;

    JBeamExpressionResult();
    bool IsValid() const;
};

/// True when an evaluation failed and every recorded diagnostic is a type
/// mismatch while at least one undeclared variable was read - that is, the
/// failure is fully explained by arithmetic on the nil an undeclared name
/// yields, and not by malformed syntax, a resource limit, or a genuine type
/// error between two declared values.
///
/// Callers use this to keep one unresolvable OPTIONAL field absent instead of
/// rejecting an otherwise admissible vehicle. It must never be used to soften
/// a required field, and limit diagnostics can never satisfy it.
bool JBeamExpressionFailedOnUndeclaredVariableOnly(
    const JBeamExpressionResult& result);

/// Evaluates an explicitly allowlisted subset of BeamNG's documented JBeam
/// expression syntax:
///
///   * a mandatory "$=" prefix;
///   * finite decimal numbers, nil/true/false, single-quoted UTF-8 strings,
///     ASCII variable names matching $[A-Za-z_][A-Za-z0-9_]*, and documented
///     dotted $components paths supplied as flat typed environment keys;
///   * +, -, *, /, %, ^, unary -, ==, ~=, <, <=, >, >=;
///   * Lua-style and/or/not (including their short-circuit, operand-returning
///     ternary idiom), string-string "..", and byte length "#";
///   * documented case(selector, ...) forms: exactly two choices for a Boolean
///     selector, or one-to-63 choices for an exact positive integer selector;
///     an integer past the available choices returns the last choice.
///   * numeric abs(value), square(value), round(value), floor(value),
///     ceil(value), smoothstep(value), smootherstep(value),
///     smootheststep(value), frexp(value), modf(value), rad(value), deg(value),
///     pow(value, integerExponent), fmod(value, divisor),
///     ldexp(value, integerExponent), clamp(value, lower, upper), and
///     one-to-64-argument min(...) and max(...) calls;
///   * exact profile constants pi and huge (documented FLT_MAX).
///
/// The evaluator is not Lua. Dotted component paths perform no dynamic lookup:
/// they are opaque keys in the caller-provided environment. The evaluator
/// deliberately rejects assignment, table values, bracket indexing, method
/// calls, every other function, and all host, file, network, random, clock, and
/// runtime access. Allowlisted function arguments and case() operands are
/// eager, matching the documentation's warning that case cannot protect
/// arithmetic on nil arguments. and/or do short-circuit.
/// To keep canonical binary64 identities independent of the host C library,
/// `%` only accepts exact integers in the inclusive range [-2^53, 2^53], and
/// `^` only accepts exact integer exponents in the inclusive range
/// [-1024, 1024]. The rounding functions use explicit IEEE-754 bit operations;
/// round() resolves exact halves away from zero. The smooth-step family clamps
/// its input to [0, 1] and evaluates one pinned polynomial operation order.
/// pow() has the same integer-exponent boundary as `^`; frexp() returns only
/// the documented usable mantissa and modf() only the documented usable
/// integral result. rad()/deg() use the exact profile pi identity and pinned
/// basic-operation order. fmod() uses an exact integer-significand remainder;
/// ldexp() pins overflow and binary64 subnormal round-to-nearest-even behavior.
JBeamExpressionResult EvaluateJBeamExpression(
    const std::string& expression,
    const JBeamExpressionEnvironment& environment =
        JBeamExpressionEnvironment(),
    const JBeamExpressionLimits& limits = JBeamExpressionLimits());

/// Stable source-independent identity material. The sign of zero is normalized
/// and finite numbers are serialized as their exact binary64 bits; strings are
/// length-prefixed so embedded bytes cannot collide.
std::string SerializeCanonicalJBeamExpressionValue(
    const JBeamExpressionValue& value);

const char* JBeamExpressionDiagnosticCodeToString(
    JBeamExpressionDiagnosticCode code);

} // namespace BeamNG
} // namespace RoR
