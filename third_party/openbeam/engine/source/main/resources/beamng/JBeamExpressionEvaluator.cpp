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

#include "JBeamExpressionEvaluator.h"

#include <cstdint>
#include <cstring>
#include <limits>
#include <locale>
#include <sstream>

namespace RoR {
namespace BeamNG {
namespace {

static_assert(
    sizeof(double) == sizeof(std::uint64_t),
    "The JBeam evaluator requires 64-bit doubles");
static_assert(
    std::numeric_limits<double>::is_iec559,
    "The JBeam evaluator requires IEEE-754 doubles");

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

bool IsVariableStart(unsigned char value)
{
    return IsAsciiAlpha(value) || value == static_cast<unsigned char>('_');
}

bool IsVariableContinue(unsigned char value)
{
    return IsVariableStart(value) || IsAsciiDigit(value);
}

/// A `$name` key is a dictionary key in the authored document, not a C
/// identifier: BeamNG content names tuning variables `$1steer`, `$3fdamp`
/// and so on, and the leading digit carries no meaning the format assigns.
/// It stays unambiguous because the tokenizer only reaches a name after the
/// `$` (or a component-path `.`) has already been consumed, so the maximal
/// run of identifier bytes is the whole name and nothing else could begin
/// there. A bare identifier -- a function name -- keeps IsVariableStart,
/// where a leading digit would be a number instead.
bool IsVariableNameStart(unsigned char value)
{
    return IsVariableStart(value) || IsAsciiDigit(value);
}

bool IsWhitespace(unsigned char value)
{
    return value == static_cast<unsigned char>(' ') ||
        value == static_cast<unsigned char>('\t') ||
        value == static_cast<unsigned char>('\r') ||
        value == static_cast<unsigned char>('\n') ||
        value == static_cast<unsigned char>('\f') ||
        value == static_cast<unsigned char>('\v');
}

std::uint64_t DoubleBits(double value)
{
    // A volatile round trip prevents -ffinite-math-only from replacing the
    // subsequent raw-bit classification with an assumption that all inputs
    // and arithmetic results are finite.
    volatile double observed = value;
    const double snapshot = observed;
    std::uint64_t bits = 0U;
    std::memcpy(&bits, &snapshot, sizeof(bits));
    return bits;
}

bool IsFiniteNumber(double value)
{
    return (DoubleBits(value) & UINT64_C(0x7ff0000000000000)) !=
        UINT64_C(0x7ff0000000000000);
}

double NormalizeNumber(double value)
{
    // Positive and negative zero are indistinguishable in JBeam expressions.
    // Canonicalizing here makes equivalent source spellings serialize alike.
    return (DoubleBits(value) & UINT64_C(0x7fffffffffffffff)) == 0U
        ? 0.0
        : value;
}

double AddNumbers(double left, double right);
double SubtractNumbers(double left, double right);
double MultiplyNumbers(double left, double right);
double DivideNumbers(double left, double right);

double NumberFromBits(std::uint64_t bits)
{
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof(value));
    volatile double observed = value;
    return observed;
}

double TruncateNumber(double value)
{
    const std::uint64_t bits = DoubleBits(value);
    const std::uint64_t exponent_bits =
        (bits >> 52U) & UINT64_C(0x7ff);
    if (exponent_bits < UINT64_C(1023))
    {
        return 0.0;
    }

    const std::uint64_t integral_exponent =
        exponent_bits - UINT64_C(1023);
    if (integral_exponent >= UINT64_C(52))
    {
        return value;
    }

    const unsigned int fractional_bits =
        static_cast<unsigned int>(UINT64_C(52) - integral_exponent);
    const std::uint64_t fractional_mask =
        (UINT64_C(1) << fractional_bits) - UINT64_C(1);
    return NumberFromBits(bits & ~fractional_mask);
}

double FloorNumber(double value)
{
    const double truncated = TruncateNumber(value);
    return value < truncated
        ? SubtractNumbers(truncated, 1.0)
        : truncated;
}

double CeilNumber(double value)
{
    const double truncated = TruncateNumber(value);
    return value > truncated
        ? AddNumbers(truncated, 1.0)
        : truncated;
}

double RoundNumber(double value)
{
    const double truncated = TruncateNumber(value);
    const double fractional = SubtractNumbers(value, truncated);
    if (fractional >= 0.5)
    {
        return AddNumbers(truncated, 1.0);
    }
    if (fractional <= -0.5)
    {
        return SubtractNumbers(truncated, 1.0);
    }
    return truncated;
}

double ClampUnitInterval(double value)
{
    return value < 0.0 ? 0.0 : (value > 1.0 ? 1.0 : value);
}

double SmoothstepNumber(double value)
{
    const double x = ClampUnitInterval(value);
    return MultiplyNumbers(
        MultiplyNumbers(x, x),
        SubtractNumbers(3.0, MultiplyNumbers(2.0, x)));
}

double SmootherstepNumber(double value)
{
    const double x = ClampUnitInterval(value);
    const double x_squared = MultiplyNumbers(x, x);
    const double x_cubed = MultiplyNumbers(x_squared, x);
    const double polynomial = AddNumbers(
        MultiplyNumbers(
            x,
            SubtractNumbers(MultiplyNumbers(6.0, x), 15.0)),
        10.0);
    return MultiplyNumbers(x_cubed, polynomial);
}

double SmootheststepNumber(double value)
{
    const double x = ClampUnitInterval(value);
    const double x_squared = MultiplyNumbers(x, x);
    const double x_fourth = MultiplyNumbers(x_squared, x_squared);
    const double polynomial = AddNumbers(
        35.0,
        MultiplyNumbers(
            x,
            AddNumbers(
                -84.0,
                MultiplyNumbers(
                    x,
                    SubtractNumbers(70.0, MultiplyNumbers(20.0, x))))));
    return MultiplyNumbers(x_fourth, polynomial);
}

double FrexpMantissaNumber(double value)
{
    static const std::uint64_t SIGN_MASK =
        UINT64_C(0x8000000000000000);
    static const std::uint64_t FRACTION_MASK =
        UINT64_C(0x000fffffffffffff);
    static const std::uint64_t HALF_EXPONENT =
        UINT64_C(0x3fe0000000000000);

    const std::uint64_t bits = DoubleBits(value);
    const std::uint64_t magnitude =
        bits & UINT64_C(0x7fffffffffffffff);
    if (magnitude == 0U)
    {
        return 0.0;
    }

    const std::uint64_t sign = bits & SIGN_MASK;
    const std::uint64_t fraction = bits & FRACTION_MASK;
    if ((magnitude >> 52U) != 0U)
    {
        return NumberFromBits(sign | HALF_EXPONENT | fraction);
    }

    unsigned int highest_bit = 0U;
    std::uint64_t remaining = fraction;
    while ((remaining >>= 1U) != 0U)
    {
        ++highest_bit;
    }
    const unsigned int shift = 52U - highest_bit;
    const std::uint64_t normalized_fraction =
        (fraction << shift) & FRACTION_MASK;
    return NumberFromBits(
        sign | HALF_EXPONENT | normalized_fraction);
}

double JBeamPiNumber()
{
    // Exact binary64 identity of the public profile's decimal
    // 3.1415926535898 spelling.
    return NumberFromBits(UINT64_C(0x400921fb54442d28));
}

double JBeamHugeNumber()
{
    // The public JBeam documentation defines huge as FLT_MAX, not DBL_MAX.
    return NumberFromBits(UINT64_C(0x47efffffe0000000));
}

void DecodeMagnitude(
    std::uint64_t magnitude_bits,
    std::uint64_t& significand,
    int& binary_exponent)
{
    static const std::uint64_t FRACTION_MASK =
        UINT64_C(0x000fffffffffffff);
    const std::uint64_t exponent_bits = magnitude_bits >> 52U;
    const std::uint64_t fraction = magnitude_bits & FRACTION_MASK;
    if (exponent_bits == 0U)
    {
        significand = fraction;
        binary_exponent = -1074;
        return;
    }
    significand = UINT64_C(0x0010000000000000) | fraction;
    binary_exponent =
        static_cast<int>(exponent_bits) - 1023 - 52;
}

unsigned int HighestSetBit(std::uint64_t value)
{
    unsigned int highest = 0U;
    while ((value >>= 1U) != 0U)
    {
        ++highest;
    }
    return highest;
}

bool ComposeScaledInteger(
    std::uint64_t significand,
    int binary_exponent,
    std::uint64_t sign,
    double& output)
{
    static const std::uint64_t FRACTION_MASK =
        UINT64_C(0x000fffffffffffff);
    if (significand == 0U)
    {
        output = 0.0;
        return true;
    }

    const unsigned int highest = HighestSetBit(significand);
    const int result_exponent =
        binary_exponent + static_cast<int>(highest);
    if (result_exponent > 1023)
    {
        return false;
    }
    if (result_exponent >= -1022)
    {
        const unsigned int shift = 52U - highest;
        const std::uint64_t normalized = significand << shift;
        const std::uint64_t exponent_bits =
            static_cast<std::uint64_t>(result_exponent + 1023) << 52U;
        output = NumberFromBits(
            sign | exponent_bits | (normalized & FRACTION_MASK));
        return true;
    }

    const int subnormal_shift = binary_exponent + 1074;
    std::uint64_t fraction = 0U;
    if (subnormal_shift >= 0)
    {
        fraction = significand <<
            static_cast<unsigned int>(subnormal_shift);
    }
    else
    {
        const unsigned int right_shift =
            static_cast<unsigned int>(-subnormal_shift);
        if (right_shift < 64U)
        {
            fraction = significand >> right_shift;
            const std::uint64_t mask =
                (UINT64_C(1) << right_shift) - UINT64_C(1);
            const std::uint64_t discarded = significand & mask;
            const std::uint64_t halfway =
                UINT64_C(1) << (right_shift - 1U);
            if (discarded > halfway ||
                (discarded == halfway && (fraction & UINT64_C(1)) != 0U))
            {
                ++fraction;
            }
        }
    }

    if (fraction >= UINT64_C(0x0010000000000000))
    {
        output = NumberFromBits(
            sign | UINT64_C(0x0010000000000000));
        return true;
    }
    output = NumberFromBits(sign | fraction);
    return true;
}

bool InspectFmodWork(
    double left,
    double right,
    std::size_t& work_units)
{
    static const std::uint64_t MAGNITUDE_MASK =
        UINT64_C(0x7fffffffffffffff);
    const std::uint64_t left_bits = DoubleBits(left);
    const std::uint64_t right_bits = DoubleBits(right);
    const std::uint64_t left_magnitude = left_bits & MAGNITUDE_MASK;
    const std::uint64_t right_magnitude = right_bits & MAGNITUDE_MASK;
    work_units = 0U;
    if (right_magnitude == 0U)
    {
        return false;
    }
    if (left_magnitude <= right_magnitude)
    {
        return true;
    }

    std::uint64_t left_significand = 0U;
    std::uint64_t right_significand = 0U;
    int left_exponent = 0;
    int right_exponent = 0;
    DecodeMagnitude(
        left_magnitude, left_significand, left_exponent);
    DecodeMagnitude(
        right_magnitude, right_significand, right_exponent);
    if (left_exponent < right_exponent)
    {
        return false;
    }
    work_units = static_cast<std::size_t>(
        left_exponent - right_exponent);
    return true;
}

bool FmodNumbers(
    double left,
    double right,
    double& output)
{
    static const std::uint64_t SIGN_MASK =
        UINT64_C(0x8000000000000000);
    static const std::uint64_t MAGNITUDE_MASK =
        UINT64_C(0x7fffffffffffffff);
    const std::uint64_t left_bits = DoubleBits(left);
    const std::uint64_t right_bits = DoubleBits(right);
    const std::uint64_t left_magnitude = left_bits & MAGNITUDE_MASK;
    const std::uint64_t right_magnitude = right_bits & MAGNITUDE_MASK;
    if (right_magnitude == 0U)
    {
        return false;
    }
    if (left_magnitude < right_magnitude)
    {
        output = left;
        return true;
    }
    if (left_magnitude == right_magnitude)
    {
        output = 0.0;
        return true;
    }

    std::uint64_t left_significand = 0U;
    std::uint64_t right_significand = 0U;
    int left_exponent = 0;
    int right_exponent = 0;
    DecodeMagnitude(
        left_magnitude, left_significand, left_exponent);
    DecodeMagnitude(
        right_magnitude, right_significand, right_exponent);
    if (left_exponent < right_exponent)
    {
        return false;
    }
    std::uint64_t remainder =
        left_significand % right_significand;
    const std::size_t work_units = static_cast<std::size_t>(
        left_exponent - right_exponent);
    for (std::size_t index = 0U; index < work_units; ++index)
    {
        remainder = (remainder << 1U) % right_significand;
    }
    return ComposeScaledInteger(
        remainder,
        right_exponent,
        left_bits & SIGN_MASK,
        output);
}

bool LdexpNumber(
    double value,
    int exponent,
    double& output)
{
    static const std::uint64_t SIGN_MASK =
        UINT64_C(0x8000000000000000);
    static const std::uint64_t MAGNITUDE_MASK =
        UINT64_C(0x7fffffffffffffff);
    const std::uint64_t bits = DoubleBits(value);
    const std::uint64_t magnitude = bits & MAGNITUDE_MASK;
    if (magnitude == 0U)
    {
        output = 0.0;
        return true;
    }
    std::uint64_t significand = 0U;
    int binary_exponent = 0;
    DecodeMagnitude(magnitude, significand, binary_exponent);
    return ComposeScaledInteger(
        significand,
        binary_exponent + exponent,
        bits & SIGN_MASK,
        output);
}

bool IsValidUtf8(const std::string& value)
{
    std::size_t index = 0U;
    while (index < value.size())
    {
        const unsigned char first =
            static_cast<unsigned char>(value[index]);
        if (first <= 0x7fU)
        {
            ++index;
            continue;
        }

        std::size_t count = 0U;
        std::uint32_t codepoint = 0U;
        std::uint32_t minimum = 0U;
        if (first >= 0xc2U && first <= 0xdfU)
        {
            count = 2U;
            codepoint = first & 0x1fU;
            minimum = 0x80U;
        }
        else if (first >= 0xe0U && first <= 0xefU)
        {
            count = 3U;
            codepoint = first & 0x0fU;
            minimum = 0x800U;
        }
        else if (first >= 0xf0U && first <= 0xf4U)
        {
            count = 4U;
            codepoint = first & 0x07U;
            minimum = 0x10000U;
        }
        else
        {
            return false;
        }

        if (count > value.size() - index)
        {
            return false;
        }
        for (std::size_t offset = 1U; offset < count; ++offset)
        {
            const unsigned char continuation =
                static_cast<unsigned char>(value[index + offset]);
            if ((continuation & 0xc0U) != 0x80U)
            {
                return false;
            }
            codepoint =
                (codepoint << 6U) |
                static_cast<std::uint32_t>(continuation & 0x3fU);
        }
        if (codepoint < minimum ||
            codepoint > 0x10ffffU ||
            (codepoint >= 0xd800U && codepoint <= 0xdfffU))
        {
            return false;
        }
        index += count;
    }
    return true;
}

bool IsValidVariableName(const std::string& name)
{
    if (name.size() < 2U ||
        name[0] != '$' ||
        !IsVariableNameStart(static_cast<unsigned char>(name[1])))
    {
        return false;
    }
    bool has_component_path = false;
    for (std::size_t index = 2U; index < name.size(); ++index)
    {
        if (name[index] == '.')
        {
            has_component_path = true;
            if (index + 1U >= name.size() ||
                !IsVariableNameStart(
                    static_cast<unsigned char>(name[index + 1U])))
            {
                return false;
            }
            continue;
        }
        if (!IsVariableContinue(static_cast<unsigned char>(name[index])))
        {
            return false;
        }
    }
    return !has_component_path ||
        (name.size() > 12U &&
         name.compare(0U, 12U, "$components.") == 0);
}

JBeamExpressionDiagnostic MakeDiagnostic(
    JBeamExpressionDiagnosticCode code,
    std::size_t byte_offset,
    const char* message)
{
    JBeamExpressionDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.byte_offset = byte_offset;
    diagnostic.message = message;
    return diagnostic;
}

class Runtime
{
public:
    Runtime(
        JBeamExpressionResult& result,
        const JBeamExpressionLimits& limits)
        : m_result(result)
        , m_limits(limits)
        , m_failed(false)
    {
    }

    bool Charge(std::size_t units, std::size_t byte_offset)
    {
        if (m_failed)
        {
            return false;
        }
        if (units > m_limits.max_work_units - m_result.work_units)
        {
            Fail(
                JBeamExpressionDiagnosticCode::WORK_LIMIT,
                byte_offset,
                "JBeam expression work exceeds the configured limit");
            return false;
        }
        m_result.work_units += units;
        return true;
    }

    bool Fail(
        JBeamExpressionDiagnosticCode code,
        std::size_t byte_offset,
        const char* message)
    {
        if (!m_failed)
        {
            m_result.diagnostics.push_back(
                MakeDiagnostic(code, byte_offset, message));
            m_failed = true;
        }
        return false;
    }

    bool Failed() const
    {
        return m_failed;
    }

    /// Notes a `$variable` the environment did not declare. Bounded and
    /// deduplicated: a hostile expression may reference undeclared names
    /// without limit, and this must not become an unbounded allocation.
    void RecordUndeclaredVariable(const std::string& name)
    {
        if (m_result.undeclared_variables.size() >=
            kMaxRecordedUndeclaredVariables)
        {
            return;
        }
        for (std::size_t i = 0U;
             i < m_result.undeclared_variables.size();
             ++i)
        {
            if (m_result.undeclared_variables[i] == name)
            {
                return;
            }
        }
        m_result.undeclared_variables.push_back(name);
    }

private:
    static const std::size_t kMaxRecordedUndeclaredVariables = 16U;

    JBeamExpressionResult& m_result;
    const JBeamExpressionLimits& m_limits;
    bool m_failed;
};

enum class TokenKind
{
    NUMBER,
    STRING,
    VARIABLE,
    NIL_VALUE,
    TRUE_VALUE,
    FALSE_VALUE,
    AND,
    OR,
    NOT,
    CASE_VALUE,
    IDENTIFIER,
    PLUS,
    MINUS,
    STAR,
    SLASH,
    PERCENT,
    CARET,
    EQUAL,
    NOT_EQUAL,
    LESS,
    LESS_EQUAL,
    GREATER,
    GREATER_EQUAL,
    CONCATENATE,
    LENGTH,
    LEFT_PAREN,
    RIGHT_PAREN,
    COMMA,
    END_OF_INPUT
};

struct Token
{
    TokenKind kind;
    std::size_t begin;
    std::size_t end;
    JBeamExpressionValue value;
    std::string text;
};

bool ParseFiniteDouble(const std::string& text, double& value)
{
    std::istringstream input(text);
    input.imbue(std::locale::classic());
    input >> std::noskipws >> value;
    return input.eof() && !input.fail() && IsFiniteNumber(value);
}

class Lexer
{
public:
    Lexer(
        const std::string& source,
        const JBeamExpressionLimits& limits,
        Runtime& runtime)
        : m_source(source)
        , m_limits(limits)
        , m_runtime(runtime)
        , m_index(2U)
    {
    }

    bool Run(std::vector<Token>& output)
    {
        while (!m_runtime.Failed())
        {
            while (m_index < m_source.size() &&
                   IsWhitespace(Peek()))
            {
                if (!Advance())
                {
                    return false;
                }
            }
            if (m_index == m_source.size())
            {
                Token token;
                token.kind = TokenKind::END_OF_INPUT;
                token.begin = m_index;
                token.end = m_index;
                return Emit(token, output);
            }

            const unsigned char current = Peek();
            if (IsAsciiDigit(current) ||
                (current == static_cast<unsigned char>('.') &&
                 IsAsciiDigit(Peek(1U))))
            {
                if (!ReadNumber(output))
                {
                    return false;
                }
                continue;
            }
            if (current == static_cast<unsigned char>('\''))
            {
                if (!ReadString(output))
                {
                    return false;
                }
                continue;
            }
            if (current == static_cast<unsigned char>('$'))
            {
                if (!ReadVariable(output))
                {
                    return false;
                }
                continue;
            }
            if (IsVariableStart(current))
            {
                if (!ReadIdentifier(output))
                {
                    return false;
                }
                continue;
            }
            if (!ReadOperator(output))
            {
                return false;
            }
        }
        return false;
    }

private:
    unsigned char Peek(std::size_t lookahead = 0U) const
    {
        if (lookahead >= m_source.size() - m_index)
        {
            return 0U;
        }
        return static_cast<unsigned char>(m_source[m_index + lookahead]);
    }

    bool Advance()
    {
        if (!m_runtime.Charge(1U, m_index))
        {
            return false;
        }
        ++m_index;
        return true;
    }

    bool Emit(const Token& token, std::vector<Token>& output)
    {
        if (output.size() >= m_limits.max_tokens)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TOKEN_LIMIT,
                token.begin,
                "JBeam expression token count exceeds the configured limit");
        }
        if (!m_runtime.Charge(1U, token.begin))
        {
            return false;
        }
        output.push_back(token);
        return true;
    }

    bool ReadNumber(std::vector<Token>& output)
    {
        const std::size_t begin = m_index;
        if (Peek() == static_cast<unsigned char>('.'))
        {
            if (!Advance())
            {
                return false;
            }
            while (IsAsciiDigit(Peek()))
            {
                if (!Advance())
                {
                    return false;
                }
            }
        }
        else
        {
            while (IsAsciiDigit(Peek()))
            {
                if (!Advance())
                {
                    return false;
                }
            }
            if (Peek() == static_cast<unsigned char>('.'))
            {
                if (!Advance())
                {
                    return false;
                }
                while (IsAsciiDigit(Peek()))
                {
                    if (!Advance())
                    {
                        return false;
                    }
                }
            }
        }

        if (Peek() == static_cast<unsigned char>('e') ||
            Peek() == static_cast<unsigned char>('E'))
        {
            if (!Advance())
            {
                return false;
            }
            if (Peek() == static_cast<unsigned char>('+') ||
                Peek() == static_cast<unsigned char>('-'))
            {
                if (!Advance())
                {
                    return false;
                }
            }
            if (!IsAsciiDigit(Peek()))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::INVALID_NUMBER,
                    begin,
                    "JBeam expression exponent requires a decimal digit");
            }
            while (IsAsciiDigit(Peek()))
            {
                if (!Advance())
                {
                    return false;
                }
            }
        }

        const std::string spelling =
            m_source.substr(begin, m_index - begin);
        double number = 0.0;
        if (!ParseFiniteDouble(spelling, number))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::NON_FINITE_NUMBER,
                begin,
                "JBeam expression number must be a finite decimal double");
        }

        Token token;
        token.kind = TokenKind::NUMBER;
        token.begin = begin;
        token.end = m_index;
        token.value = JBeamExpressionValue::Number(
            NormalizeNumber(number));
        return Emit(token, output);
    }

    bool AppendStringByte(
        char byte,
        std::size_t byte_offset,
        std::string& output)
    {
        if (output.size() >= m_limits.max_string_bytes)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::STRING_SIZE_LIMIT,
                byte_offset,
                "JBeam expression string exceeds the configured limit");
        }
        output.push_back(byte);
        return true;
    }

    bool ReadString(std::vector<Token>& output)
    {
        const std::size_t begin = m_index;
        if (!Advance())
        {
            return false;
        }
        std::string decoded;
        while (m_index < m_source.size())
        {
            const std::size_t byte_offset = m_index;
            const unsigned char current = Peek();
            if (current == static_cast<unsigned char>('\''))
            {
                if (!Advance())
                {
                    return false;
                }
                if (!m_runtime.Charge(decoded.size(), begin))
                {
                    return false;
                }
                if (!IsValidUtf8(decoded))
                {
                    return m_runtime.Fail(
                        JBeamExpressionDiagnosticCode::INVALID_UTF8,
                        begin,
                        "JBeam expression string is not valid UTF-8");
                }
                Token token;
                token.kind = TokenKind::STRING;
                token.begin = begin;
                token.end = m_index;
                token.value = JBeamExpressionValue::String(decoded);
                return Emit(token, output);
            }
            if (current < 0x20U || current == 0x7fU)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::INVALID_CHARACTER,
                    byte_offset,
                    "JBeam expression string contains a raw control byte");
            }
            if (current == static_cast<unsigned char>('\\'))
            {
                if (!Advance())
                {
                    return false;
                }
                if (m_index == m_source.size())
                {
                    return m_runtime.Fail(
                        JBeamExpressionDiagnosticCode::UNTERMINATED_STRING,
                        begin,
                        "JBeam expression string is unterminated");
                }
                const unsigned char escaped = Peek();
                char decoded_byte = 0;
                switch (escaped)
                {
                case static_cast<unsigned char>('\''):
                    decoded_byte = '\'';
                    break;
                case static_cast<unsigned char>('\\'):
                    decoded_byte = '\\';
                    break;
                case static_cast<unsigned char>('n'):
                    decoded_byte = '\n';
                    break;
                case static_cast<unsigned char>('r'):
                    decoded_byte = '\r';
                    break;
                case static_cast<unsigned char>('t'):
                    decoded_byte = '\t';
                    break;
                default:
                    return m_runtime.Fail(
                        JBeamExpressionDiagnosticCode::INVALID_ESCAPE,
                        m_index,
                        "JBeam expression string escape is not allowlisted");
                }
                if (!Advance() ||
                    !AppendStringByte(
                        decoded_byte, byte_offset, decoded))
                {
                    return false;
                }
                continue;
            }
            if (!Advance() ||
                !AppendStringByte(
                    static_cast<char>(current), byte_offset, decoded))
            {
                return false;
            }
        }
        return m_runtime.Fail(
            JBeamExpressionDiagnosticCode::UNTERMINATED_STRING,
            begin,
            "JBeam expression string is unterminated");
    }

    bool ReadVariable(std::vector<Token>& output)
    {
        const std::size_t begin = m_index;
        if (!Advance())
        {
            return false;
        }
        if (!IsVariableNameStart(Peek()))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::INVALID_CHARACTER,
                begin,
                "JBeam expression variable name is invalid");
        }
        while (IsVariableContinue(Peek()))
        {
            if (!Advance())
            {
                return false;
            }
        }
        while (Peek() == static_cast<unsigned char>('.') &&
               Peek(1U) != static_cast<unsigned char>('.'))
        {
            if (!IsVariableNameStart(Peek(1U)) ||
                !Advance())
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::INVALID_CHARACTER,
                    m_index,
                    "JBeam component path segment is invalid");
            }
            while (IsVariableContinue(Peek()))
            {
                if (!Advance())
                {
                    return false;
                }
            }
        }
        if (m_index - begin > m_limits.max_variable_name_bytes)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::STRING_SIZE_LIMIT,
                begin,
                "JBeam expression variable name exceeds the configured limit");
        }
        Token token;
        token.kind = TokenKind::VARIABLE;
        token.begin = begin;
        token.end = m_index;
        token.text = m_source.substr(begin, m_index - begin);
        if (!IsValidVariableName(token.text))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::INVALID_CHARACTER,
                begin,
                "Only $components may use dotted JBeam paths");
        }
        return Emit(token, output);
    }

    bool ReadIdentifier(std::vector<Token>& output)
    {
        const std::size_t begin = m_index;
        while (IsVariableContinue(Peek()))
        {
            if (!Advance())
            {
                return false;
            }
        }
        const std::string text =
            m_source.substr(begin, m_index - begin);
        Token token;
        token.begin = begin;
        token.end = m_index;
        token.text = text;
        if (text == "nil")
        {
            token.kind = TokenKind::NIL_VALUE;
        }
        else if (text == "true")
        {
            token.kind = TokenKind::TRUE_VALUE;
        }
        else if (text == "false")
        {
            token.kind = TokenKind::FALSE_VALUE;
        }
        else if (text == "and")
        {
            token.kind = TokenKind::AND;
        }
        else if (text == "or")
        {
            token.kind = TokenKind::OR;
        }
        else if (text == "not")
        {
            token.kind = TokenKind::NOT;
        }
        else if (text == "case")
        {
            token.kind = TokenKind::CASE_VALUE;
        }
        else
        {
            token.kind = TokenKind::IDENTIFIER;
        }
        return Emit(token, output);
    }

    bool EmitOperator(
        TokenKind kind,
        std::size_t begin,
        std::size_t byte_count,
        std::vector<Token>& output)
    {
        for (std::size_t index = 0U; index < byte_count; ++index)
        {
            if (!Advance())
            {
                return false;
            }
        }
        Token token;
        token.kind = kind;
        token.begin = begin;
        token.end = m_index;
        return Emit(token, output);
    }

    bool ReadOperator(std::vector<Token>& output)
    {
        const std::size_t begin = m_index;
        switch (Peek())
        {
        case static_cast<unsigned char>('+'):
            return EmitOperator(TokenKind::PLUS, begin, 1U, output);
        case static_cast<unsigned char>('-'):
            return EmitOperator(TokenKind::MINUS, begin, 1U, output);
        case static_cast<unsigned char>('*'):
            return EmitOperator(TokenKind::STAR, begin, 1U, output);
        case static_cast<unsigned char>('/'):
            return EmitOperator(TokenKind::SLASH, begin, 1U, output);
        case static_cast<unsigned char>('%'):
            return EmitOperator(TokenKind::PERCENT, begin, 1U, output);
        case static_cast<unsigned char>('^'):
            return EmitOperator(TokenKind::CARET, begin, 1U, output);
        case static_cast<unsigned char>('#'):
            return EmitOperator(TokenKind::LENGTH, begin, 1U, output);
        case static_cast<unsigned char>('('):
            return EmitOperator(TokenKind::LEFT_PAREN, begin, 1U, output);
        case static_cast<unsigned char>(')'):
            return EmitOperator(TokenKind::RIGHT_PAREN, begin, 1U, output);
        case static_cast<unsigned char>(','):
            return EmitOperator(TokenKind::COMMA, begin, 1U, output);
        case static_cast<unsigned char>('.'):
            if (Peek(1U) == static_cast<unsigned char>('.'))
            {
                return EmitOperator(
                    TokenKind::CONCATENATE, begin, 2U, output);
            }
            break;
        case static_cast<unsigned char>('='):
            if (Peek(1U) == static_cast<unsigned char>('='))
            {
                return EmitOperator(
                    TokenKind::EQUAL, begin, 2U, output);
            }
            break;
        case static_cast<unsigned char>('~'):
            if (Peek(1U) == static_cast<unsigned char>('='))
            {
                return EmitOperator(
                    TokenKind::NOT_EQUAL, begin, 2U, output);
            }
            break;
        case static_cast<unsigned char>('<'):
            if (Peek(1U) == static_cast<unsigned char>('='))
            {
                return EmitOperator(
                    TokenKind::LESS_EQUAL, begin, 2U, output);
            }
            return EmitOperator(TokenKind::LESS, begin, 1U, output);
        case static_cast<unsigned char>('>'):
            if (Peek(1U) == static_cast<unsigned char>('='))
            {
                return EmitOperator(
                    TokenKind::GREATER_EQUAL, begin, 2U, output);
            }
            return EmitOperator(TokenKind::GREATER, begin, 1U, output);
        default:
            break;
        }
        return m_runtime.Fail(
            JBeamExpressionDiagnosticCode::INVALID_CHARACTER,
            begin,
            "JBeam expression contains a character or operator outside "
            "the allowlist");
    }

    const std::string& m_source;
    const JBeamExpressionLimits& m_limits;
    Runtime& m_runtime;
    std::size_t m_index;
};

bool IsTruthy(const JBeamExpressionValue& value)
{
    return value.type != JBeamExpressionValueType::NIL_VALUE &&
        (value.type != JBeamExpressionValueType::BOOLEAN ||
         value.boolean_value);
}

int CompareStrings(const std::string& left, const std::string& right)
{
    const std::size_t common =
        left.size() < right.size() ? left.size() : right.size();
    for (std::size_t index = 0U; index < common; ++index)
    {
        const unsigned char left_byte =
            static_cast<unsigned char>(left[index]);
        const unsigned char right_byte =
            static_cast<unsigned char>(right[index]);
        if (left_byte < right_byte)
        {
            return -1;
        }
        if (left_byte > right_byte)
        {
            return 1;
        }
    }
    if (left.size() < right.size())
    {
        return -1;
    }
    if (left.size() > right.size())
    {
        return 1;
    }
    return 0;
}

bool ValuesEqual(
    const JBeamExpressionValue& left,
    const JBeamExpressionValue& right)
{
    if (left.type != right.type)
    {
        return false;
    }
    switch (left.type)
    {
    case JBeamExpressionValueType::NIL_VALUE:
        return true;
    case JBeamExpressionValueType::BOOLEAN:
        return left.boolean_value == right.boolean_value;
    case JBeamExpressionValueType::NUMBER:
        return left.number_value == right.number_value;
    case JBeamExpressionValueType::STRING:
        return left.string_value == right.string_value;
    }
    return false;
}

double AddNumbers(double left, double right)
{
    volatile double observed_left = left;
    volatile double observed_right = right;
    volatile double result = observed_left + observed_right;
    return result;
}

double SubtractNumbers(double left, double right)
{
    volatile double observed_left = left;
    volatile double observed_right = right;
    volatile double result = observed_left - observed_right;
    return result;
}

double MultiplyNumbers(double left, double right)
{
    volatile double observed_left = left;
    volatile double observed_right = right;
    volatile double result = observed_left * observed_right;
    return result;
}

double DivideNumbers(double left, double right)
{
    volatile double observed_left = left;
    volatile double observed_right = right;
    volatile double result = observed_left / observed_right;
    return result;
}

bool IsExactIntegerInRange(
    double value,
    double minimum,
    double maximum,
    std::int64_t& integer)
{
    if (value < minimum || value > maximum)
    {
        return false;
    }
    const std::int64_t candidate = static_cast<std::int64_t>(value);
    if (static_cast<double>(candidate) != value)
    {
        return false;
    }
    integer = candidate;
    return true;
}

bool PowerNumbers(double base, int exponent, double& output)
{
    unsigned int remaining = static_cast<unsigned int>(
        exponent < 0 ? -exponent : exponent);
    double factor = base;
    if (exponent < 0)
    {
        factor = DivideNumbers(1.0, factor);
        if (!IsFiniteNumber(factor))
        {
            return false;
        }
    }

    double result = 1.0;
    while (remaining != 0U)
    {
        if ((remaining & 1U) != 0U)
        {
            result = MultiplyNumbers(result, factor);
            if (!IsFiniteNumber(result))
            {
                return false;
            }
        }
        remaining >>= 1U;
        if (remaining != 0U)
        {
            factor = MultiplyNumbers(factor, factor);
            if (!IsFiniteNumber(factor))
            {
                return false;
            }
        }
    }
    output = result;
    return true;
}

double ModuloIntegers(std::int64_t left, std::int64_t right)
{
    std::int64_t remainder = left % right;
    if (remainder != 0 &&
        ((remainder < 0) != (right < 0)))
    {
        remainder += right;
    }
    return static_cast<double>(remainder);
}

class Parser
{
public:
    Parser(
        const std::vector<Token>& tokens,
        const JBeamExpressionEnvironment& environment,
        const JBeamExpressionLimits& limits,
        Runtime& runtime)
        : m_tokens(tokens)
        , m_environment(environment)
        , m_limits(limits)
        , m_runtime(runtime)
        , m_index(0U)
        , m_depth(0U)
    {
    }

    bool Run(JBeamExpressionValue& output)
    {
        if (!ParseOr(true, output))
        {
            return false;
        }
        if (Current().kind != TokenKind::END_OF_INPUT)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TRAILING_CONTENT,
                Current().begin,
                "JBeam expression has trailing content");
        }
        if (!Consume())
        {
            return false;
        }
        return !m_runtime.Failed();
    }

private:
    static const std::size_t MAX_ALLOWLISTED_FUNCTION_ARGUMENTS = 64U;

    const Token& Current() const
    {
        return m_tokens[m_index];
    }

    const Token& Next() const
    {
        return m_tokens[
            m_index + 1U < m_tokens.size() ? m_index + 1U : m_index];
    }

    bool Consume()
    {
        if (!m_runtime.Charge(1U, Current().begin))
        {
            return false;
        }
        if (m_index + 1U < m_tokens.size())
        {
            ++m_index;
        }
        return true;
    }

    bool Match(TokenKind kind)
    {
        if (Current().kind != kind)
        {
            return false;
        }
        return Consume();
    }

    bool Expect(TokenKind kind, const char* message)
    {
        if (Current().kind != kind)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::EXPECTED_TOKEN,
                Current().begin,
                message);
        }
        return Consume();
    }

    bool BeginDepth(std::size_t byte_offset)
    {
        if (m_depth >= m_limits.max_depth)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::DEPTH_LIMIT,
                byte_offset,
                "JBeam expression nesting exceeds the configured limit");
        }
        ++m_depth;
        return true;
    }

    void EndDepth()
    {
        if (m_depth > 0U)
        {
            --m_depth;
        }
    }

    bool ParseOr(bool evaluate, JBeamExpressionValue& output)
    {
        if (!ParseAnd(evaluate, output))
        {
            return false;
        }
        while (Current().kind == TokenKind::OR)
        {
            const std::size_t offset = Current().begin;
            if (!Consume())
            {
                return false;
            }
            const bool use_left = evaluate && IsTruthy(output);
            JBeamExpressionValue right;
            if (!ParseAnd(evaluate && !use_left, right))
            {
                return false;
            }
            if (evaluate)
            {
                if (!m_runtime.Charge(1U, offset))
                {
                    return false;
                }
                if (!use_left)
                {
                    if (!ChargeValueCopy(right, offset))
                    {
                        return false;
                    }
                    output = right;
                }
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseAnd(bool evaluate, JBeamExpressionValue& output)
    {
        if (!ParseComparison(evaluate, output))
        {
            return false;
        }
        while (Current().kind == TokenKind::AND)
        {
            const std::size_t offset = Current().begin;
            if (!Consume())
            {
                return false;
            }
            const bool use_right = evaluate && IsTruthy(output);
            JBeamExpressionValue right;
            if (!ParseComparison(use_right, right))
            {
                return false;
            }
            if (evaluate)
            {
                if (!m_runtime.Charge(1U, offset))
                {
                    return false;
                }
                if (use_right)
                {
                    if (!ChargeValueCopy(right, offset))
                    {
                        return false;
                    }
                    output = right;
                }
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseComparison(bool evaluate, JBeamExpressionValue& output)
    {
        if (!ParseConcatenation(evaluate, output))
        {
            return false;
        }
        while (Current().kind == TokenKind::EQUAL ||
               Current().kind == TokenKind::NOT_EQUAL ||
               Current().kind == TokenKind::LESS ||
               Current().kind == TokenKind::LESS_EQUAL ||
               Current().kind == TokenKind::GREATER ||
               Current().kind == TokenKind::GREATER_EQUAL)
        {
            const Token operation = Current();
            if (!Consume())
            {
                return false;
            }
            JBeamExpressionValue right;
            if (!ParseConcatenation(evaluate, right))
            {
                return false;
            }
            if (evaluate &&
                !ApplyComparison(operation, output, right, output))
            {
                return false;
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseConcatenation(
        bool evaluate,
        JBeamExpressionValue& output)
    {
        if (!ParseAdditive(evaluate, output))
        {
            return false;
        }
        if (Current().kind == TokenKind::CONCATENATE)
        {
            const Token operation = Current();
            if (!Consume() || !BeginDepth(operation.begin))
            {
                return false;
            }
            JBeamExpressionValue right;
            const bool parsed = ParseConcatenation(evaluate, right);
            EndDepth();
            if (!parsed)
            {
                return false;
            }
            if (evaluate &&
                !ApplyConcatenation(operation, output, right, output))
            {
                return false;
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseAdditive(bool evaluate, JBeamExpressionValue& output)
    {
        if (!ParseMultiplicative(evaluate, output))
        {
            return false;
        }
        while (Current().kind == TokenKind::PLUS ||
               Current().kind == TokenKind::MINUS)
        {
            const Token operation = Current();
            if (!Consume())
            {
                return false;
            }
            JBeamExpressionValue right;
            if (!ParseMultiplicative(evaluate, right))
            {
                return false;
            }
            if (evaluate &&
                !ApplyArithmetic(operation, output, right, output))
            {
                return false;
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseMultiplicative(
        bool evaluate,
        JBeamExpressionValue& output)
    {
        if (!ParseUnary(evaluate, output))
        {
            return false;
        }
        while (Current().kind == TokenKind::STAR ||
               Current().kind == TokenKind::SLASH ||
               Current().kind == TokenKind::PERCENT)
        {
            const Token operation = Current();
            if (!Consume())
            {
                return false;
            }
            JBeamExpressionValue right;
            if (!ParseUnary(evaluate, right))
            {
                return false;
            }
            if (evaluate &&
                !ApplyArithmetic(operation, output, right, output))
            {
                return false;
            }
        }
        return !m_runtime.Failed();
    }

    bool ParseUnary(bool evaluate, JBeamExpressionValue& output)
    {
        if (Current().kind == TokenKind::MINUS ||
            Current().kind == TokenKind::NOT ||
            Current().kind == TokenKind::LENGTH)
        {
            const Token operation = Current();
            if (!Consume() || !BeginDepth(operation.begin))
            {
                return false;
            }
            JBeamExpressionValue operand;
            const bool parsed = ParseUnary(evaluate, operand);
            EndDepth();
            if (!parsed)
            {
                return false;
            }
            if (evaluate &&
                !ApplyUnary(operation, operand, output))
            {
                return false;
            }
            return true;
        }
        return ParsePower(evaluate, output);
    }

    bool ParsePower(bool evaluate, JBeamExpressionValue& output)
    {
        if (!ParsePrimary(evaluate, output))
        {
            return false;
        }
        if (Current().kind == TokenKind::CARET)
        {
            const Token operation = Current();
            if (!Consume() || !BeginDepth(operation.begin))
            {
                return false;
            }
            JBeamExpressionValue right;
            const bool parsed = ParseUnary(evaluate, right);
            EndDepth();
            if (!parsed)
            {
                return false;
            }
            if (evaluate &&
                !ApplyArithmetic(operation, output, right, output))
            {
                return false;
            }
        }
        return !m_runtime.Failed();
    }

    bool ParsePrimary(bool evaluate, JBeamExpressionValue& output)
    {
        const Token token = Current();
        switch (token.kind)
        {
        case TokenKind::NUMBER:
        case TokenKind::STRING:
            if (!Consume())
            {
                return false;
            }
            if (evaluate)
            {
                if (!ChargeValueCopy(token.value, token.begin))
                {
                    return false;
                }
                output = token.value;
            }
            else
            {
                output = JBeamExpressionValue::Nil();
            }
            return true;
        case TokenKind::NIL_VALUE:
            if (!Consume())
            {
                return false;
            }
            output = JBeamExpressionValue::Nil();
            return true;
        case TokenKind::TRUE_VALUE:
        case TokenKind::FALSE_VALUE:
            if (!Consume())
            {
                return false;
            }
            output = evaluate
                ? JBeamExpressionValue::Boolean(
                    token.kind == TokenKind::TRUE_VALUE)
                : JBeamExpressionValue::Nil();
            return true;
        case TokenKind::VARIABLE:
            if (!Consume())
            {
                return false;
            }
            if (!evaluate)
            {
                output = JBeamExpressionValue::Nil();
                return true;
            }
            return LookupVariable(token, output);
        case TokenKind::LEFT_PAREN:
            if (!Consume() || !BeginDepth(token.begin))
            {
                return false;
            }
            {
                const bool parsed = ParseOr(evaluate, output);
                EndDepth();
                if (!parsed)
                {
                    return false;
                }
            }
            return Expect(
                TokenKind::RIGHT_PAREN,
                "Expected ')' after JBeam expression group");
        case TokenKind::CASE_VALUE:
            return ParseCase(evaluate, output);
        case TokenKind::IDENTIFIER:
            if ((token.text == "pi" || token.text == "huge") &&
                Next().kind != TokenKind::LEFT_PAREN)
            {
                if (!Consume())
                {
                    return false;
                }
                output = evaluate
                    ? JBeamExpressionValue::Number(
                        token.text == "pi"
                            ? JBeamPiNumber()
                            : JBeamHugeNumber())
                    : JBeamExpressionValue::Nil();
                return true;
            }
            if (Next().kind == TokenKind::LEFT_PAREN &&
                IsAllowlistedScalarFunction(token.text))
            {
                return ParseScalarFunction(evaluate, output);
            }
            return m_runtime.Fail(
                Next().kind == TokenKind::LEFT_PAREN
                    ? JBeamExpressionDiagnosticCode::UNSUPPORTED_FUNCTION
                    : JBeamExpressionDiagnosticCode::EXPECTED_VALUE,
                token.begin,
                Next().kind == TokenKind::LEFT_PAREN
                    ? "JBeam expression function is outside the allowlist"
                    : "Expected an allowlisted JBeam expression value");
        default:
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::EXPECTED_VALUE,
                token.begin,
                "Expected an allowlisted JBeam expression value");
        }
    }

    bool IsAllowlistedScalarFunction(const std::string& name) const
    {
        return name == "abs" ||
            name == "square" ||
            name == "round" ||
            name == "floor" ||
            name == "ceil" ||
            name == "smoothstep" ||
            name == "smootherstep" ||
            name == "smootheststep" ||
            name == "frexp" ||
            name == "modf" ||
            name == "rad" ||
            name == "deg" ||
            name == "pow" ||
            name == "fmod" ||
            name == "ldexp" ||
            name == "clamp" ||
            name == "min" ||
            name == "max";
    }

    bool ParseScalarFunction(
        bool evaluate,
        JBeamExpressionValue& output)
    {
        const Token function = Current();
        if (!Consume() ||
            !Expect(
                TokenKind::LEFT_PAREN,
                "Expected '(' after JBeam scalar function") ||
            !BeginDepth(function.begin))
        {
            return false;
        }

        const std::size_t argument_limit =
            m_limits.max_function_arguments <
                    MAX_ALLOWLISTED_FUNCTION_ARGUMENTS
                ? m_limits.max_function_arguments
                : MAX_ALLOWLISTED_FUNCTION_ARGUMENTS;
        const bool unary =
            function.text == "abs" ||
            function.text == "square" ||
            function.text == "round" ||
            function.text == "floor" ||
            function.text == "ceil" ||
            function.text == "smoothstep" ||
            function.text == "smootherstep" ||
            function.text == "smootheststep" ||
            function.text == "frexp" ||
            function.text == "modf" ||
            function.text == "rad" ||
            function.text == "deg";
        const bool binary =
            function.text == "pow" ||
            function.text == "fmod" ||
            function.text == "ldexp";
        const bool variadic =
            function.text == "min" || function.text == "max";
        std::size_t argument_count = 0U;
        bool all_numeric = true;
        JBeamExpressionValue fixed_arguments[3];
        double extremum = 0.0;
        bool have_extremum = false;
        bool parsed = true;
        if (Current().kind != TokenKind::RIGHT_PAREN)
        {
            while (parsed)
            {
                if (argument_count >= argument_limit)
                {
                    parsed = m_runtime.Fail(
                        JBeamExpressionDiagnosticCode::
                            FUNCTION_ARGUMENT_LIMIT,
                        Current().begin,
                        "JBeam function argument count exceeds the "
                        "configured or allowlisted limit");
                    break;
                }

                JBeamExpressionValue argument;
                parsed = ParseOr(evaluate, argument);
                if (!parsed)
                {
                    break;
                }
                if (evaluate)
                {
                    if (argument.type !=
                        JBeamExpressionValueType::NUMBER)
                    {
                        all_numeric = false;
                    }
                    else if (variadic)
                    {
                        if (!have_extremum)
                        {
                            extremum = argument.number_value;
                            have_extremum = true;
                        }
                        else if (
                            (function.text == "min" &&
                             argument.number_value < extremum) ||
                            (function.text == "max" &&
                             argument.number_value > extremum))
                        {
                            extremum = argument.number_value;
                        }
                    }
                    else if (argument_count < 3U)
                    {
                        fixed_arguments[argument_count] = argument;
                    }
                }
                ++argument_count;
                if (Current().kind != TokenKind::COMMA)
                {
                    break;
                }
                parsed = Consume();
            }
        }
        if (parsed)
        {
            parsed = Expect(
                TokenKind::RIGHT_PAREN,
                "Expected ')' after JBeam scalar function arguments");
        }
        EndDepth();
        if (!parsed)
        {
            return false;
        }

        const bool valid_arity =
            (unary && argument_count == 1U) ||
            (binary && argument_count == 2U) ||
            (function.text == "clamp" && argument_count == 3U) ||
            (variadic &&
             argument_count >= 1U &&
             argument_count <=
                 MAX_ALLOWLISTED_FUNCTION_ARGUMENTS);
        if (!valid_arity)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::FUNCTION_ARITY,
                function.begin,
                "JBeam scalar function has an invalid argument count");
        }
        if (!evaluate)
        {
            output = JBeamExpressionValue::Nil();
            return true;
        }
        if (!m_runtime.Charge(
                1U + argument_count, function.begin))
        {
            return false;
        }
        if (!all_numeric)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                function.begin,
                "JBeam scalar function arguments must all be numbers");
        }

        double result = 0.0;
        if (function.text == "abs")
        {
            volatile double observed =
                fixed_arguments[0].number_value;
            result = observed < 0.0 ? -observed : observed;
        }
        else if (function.text == "square")
        {
            result = MultiplyNumbers(
                fixed_arguments[0].number_value,
                fixed_arguments[0].number_value);
            if (!IsFiniteNumber(result))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                    function.begin,
                    "JBeam square function produced a non-finite value");
            }
        }
        else if (function.text == "round")
        {
            result = RoundNumber(fixed_arguments[0].number_value);
        }
        else if (function.text == "floor")
        {
            result = FloorNumber(fixed_arguments[0].number_value);
        }
        else if (function.text == "ceil")
        {
            result = CeilNumber(fixed_arguments[0].number_value);
        }
        else if (function.text == "smoothstep")
        {
            result = SmoothstepNumber(
                fixed_arguments[0].number_value);
        }
        else if (function.text == "smootherstep")
        {
            result = SmootherstepNumber(
                fixed_arguments[0].number_value);
        }
        else if (function.text == "smootheststep")
        {
            result = SmootheststepNumber(
                fixed_arguments[0].number_value);
        }
        else if (function.text == "frexp")
        {
            result = FrexpMantissaNumber(
                fixed_arguments[0].number_value);
        }
        else if (function.text == "modf")
        {
            result = TruncateNumber(
                fixed_arguments[0].number_value);
        }
        else if (function.text == "rad")
        {
            result = MultiplyNumbers(
                DivideNumbers(
                    fixed_arguments[0].number_value,
                    180.0),
                JBeamPiNumber());
        }
        else if (function.text == "deg")
        {
            result = MultiplyNumbers(
                DivideNumbers(
                    fixed_arguments[0].number_value,
                    JBeamPiNumber()),
                180.0);
        }
        else if (function.text == "pow")
        {
            static const std::int64_t MAX_POWER_EXPONENT = 1024;
            std::int64_t exponent = 0;
            if (!IsExactIntegerInRange(
                    fixed_arguments[1].number_value,
                    -static_cast<double>(MAX_POWER_EXPONENT),
                    static_cast<double>(MAX_POWER_EXPONENT),
                    exponent))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        NON_DETERMINISTIC_OPERAND,
                    function.begin,
                    "JBeam pow exponent must be an exact integer in "
                    "[-1024, 1024]");
            }
            if (fixed_arguments[0].number_value == 0.0 && exponent < 0)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::DIVISION_BY_ZERO,
                    function.begin,
                    "JBeam zero cannot be raised to a negative power");
            }
            if (!PowerNumbers(
                    fixed_arguments[0].number_value,
                    static_cast<int>(exponent),
                    result))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                    function.begin,
                    "JBeam pow function produced a non-finite value");
            }
        }
        else if (function.text == "fmod")
        {
            if (fixed_arguments[1].number_value == 0.0)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::DIVISION_BY_ZERO,
                    function.begin,
                    "JBeam fmod divisor must not be zero");
            }
            std::size_t fmod_work = 0U;
            if (!InspectFmodWork(
                    fixed_arguments[0].number_value,
                    fixed_arguments[1].number_value,
                    fmod_work))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_FUNCTION_ARGUMENT,
                    function.begin,
                    "JBeam fmod operands could not be reduced exactly");
            }
            if (!m_runtime.Charge(fmod_work, function.begin))
            {
                return false;
            }
            if (!FmodNumbers(
                    fixed_arguments[0].number_value,
                    fixed_arguments[1].number_value,
                    result))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_FUNCTION_ARGUMENT,
                    function.begin,
                    "JBeam fmod operands changed after work preflight");
            }
        }
        else if (function.text == "ldexp")
        {
            static const std::int64_t MAX_LDEXP_EXPONENT = 4096;
            std::int64_t exponent = 0;
            if (!IsExactIntegerInRange(
                    fixed_arguments[1].number_value,
                    -static_cast<double>(MAX_LDEXP_EXPONENT),
                    static_cast<double>(MAX_LDEXP_EXPONENT),
                    exponent))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        NON_DETERMINISTIC_OPERAND,
                    function.begin,
                    "JBeam ldexp exponent must be an exact integer in "
                    "[-4096, 4096]");
            }
            if (!LdexpNumber(
                    fixed_arguments[0].number_value,
                    static_cast<int>(exponent),
                    result))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                    function.begin,
                    "JBeam ldexp function overflowed binary64");
            }
        }
        else if (function.text == "clamp")
        {
            const double value =
                fixed_arguments[0].number_value;
            const double lower =
                fixed_arguments[1].number_value;
            const double upper =
                fixed_arguments[2].number_value;
            if (lower > upper)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_FUNCTION_ARGUMENT,
                    function.begin,
                    "JBeam clamp lower bound must not exceed its "
                    "upper bound");
            }
            result = value < lower
                ? lower
                : (value > upper ? upper : value);
        }
        else
        {
            result = extremum;
        }
        if (!IsFiniteNumber(result))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                function.begin,
                "JBeam scalar function produced a non-finite value");
        }
        output = JBeamExpressionValue::Number(
            NormalizeNumber(result));
        return true;
    }

    bool ParseCase(bool evaluate, JBeamExpressionValue& output)
    {
        const Token function = Current();
        if (!Consume() ||
            !Expect(
                TokenKind::LEFT_PAREN,
                "Expected '(' after case") ||
            !BeginDepth(function.begin))
        {
            return false;
        }
        const std::size_t argument_limit =
            m_limits.max_function_arguments <
                    MAX_ALLOWLISTED_FUNCTION_ARGUMENTS
                ? m_limits.max_function_arguments
                : MAX_ALLOWLISTED_FUNCTION_ARGUMENTS;
        if (argument_limit < 2U)
        {
            EndDepth();
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::FUNCTION_ARGUMENT_LIMIT,
                function.begin,
                "JBeam case argument count exceeds the configured limit");
        }

        JBeamExpressionValue selector;
        bool parsed = ParseOr(evaluate, selector);
        std::size_t argument_count = parsed ? 1U : 0U;
        std::size_t choice_count = 0U;
        JBeamExpressionValue selected;
        JBeamExpressionValue last_choice;
        bool have_selected = false;
        std::int64_t numeric_selector = 0;
        const bool numeric_selector_valid =
            evaluate &&
            selector.type == JBeamExpressionValueType::NUMBER &&
            IsExactIntegerInRange(
                selector.number_value,
                1.0,
                9007199254740992.0,
                numeric_selector);

        while (parsed && Current().kind == TokenKind::COMMA)
        {
            if (argument_count >= argument_limit)
            {
                parsed = m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        FUNCTION_ARGUMENT_LIMIT,
                    Current().begin,
                    "JBeam case argument count exceeds the configured "
                    "or allowlisted limit");
                break;
            }
            parsed = Consume();
            JBeamExpressionValue choice;
            if (parsed)
            {
                parsed = ParseOr(evaluate, choice);
            }
            if (!parsed)
            {
                break;
            }
            ++argument_count;
            ++choice_count;
            if (evaluate)
            {
                last_choice = choice;
                const bool boolean_selection =
                    selector.type == JBeamExpressionValueType::BOOLEAN &&
                    ((selector.boolean_value && choice_count == 1U) ||
                     (!selector.boolean_value && choice_count == 2U));
                const bool numeric_selection =
                    numeric_selector_valid &&
                    numeric_selector ==
                        static_cast<std::int64_t>(choice_count);
                if (boolean_selection || numeric_selection)
                {
                    selected = choice;
                    have_selected = true;
                }
            }
        }
        if (parsed)
        {
            parsed = Expect(
                TokenKind::RIGHT_PAREN,
                "Expected ')' after case arguments");
        }
        EndDepth();
        if (!parsed)
        {
            return false;
        }
        if (!evaluate)
        {
            output = JBeamExpressionValue::Nil();
            return true;
        }
        if (!m_runtime.Charge(
                1U + argument_count, function.begin))
        {
            return false;
        }
        if (selector.type == JBeamExpressionValueType::BOOLEAN)
        {
            if (choice_count != 2U)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::FUNCTION_ARITY,
                    function.begin,
                    "Boolean JBeam case requires exactly two choices");
            }
        }
        else if (selector.type == JBeamExpressionValueType::NUMBER)
        {
            if (!numeric_selector_valid)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_FUNCTION_ARGUMENT,
                    function.begin,
                    "Numeric JBeam case selector must be an exact positive "
                    "integer in [1, 2^53]");
            }
            if (choice_count == 0U)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::FUNCTION_ARITY,
                    function.begin,
                    "Numeric JBeam case requires at least one choice");
            }
            if (!have_selected)
            {
                selected = last_choice;
                have_selected = true;
            }
        }
        else
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::UNSUPPORTED_CASE_SIGNATURE,
                function.begin,
                "JBeam case selector must be Boolean or an exact positive "
                "integer");
        }
        if (!have_selected)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::FUNCTION_ARITY,
                function.begin,
                "JBeam case did not provide the selected choice");
        }
        if (!ChargeValueCopy(selected, function.begin))
        {
            return false;
        }
        output = selected;
        return true;
    }

    bool LookupVariable(
        const Token& token,
        JBeamExpressionValue& output)
    {
        for (std::size_t index = m_environment.variables.size();
             index > 0U;
             --index)
        {
            const JBeamExpressionVariable& variable =
                m_environment.variables[index - 1U];
            if (!m_runtime.Charge(1U, token.begin) ||
                !m_runtime.Charge(variable.name.size(), token.begin) ||
                !m_runtime.Charge(token.text.size(), token.begin))
            {
                return false;
            }
            if (variable.name == token.text)
            {
                if (!ChargeValueCopy(variable.value, token.begin))
                {
                    return false;
                }
                output = variable.value;
                if (output.type == JBeamExpressionValueType::NUMBER)
                {
                    output.number_value =
                        NormalizeNumber(output.number_value);
                }
                return true;
            }
        }
        // Not declared. An unset $variable has no value, so this yields nil
        // and succeeds; the failure, if any, comes later when arithmetic meets
        // that nil. Record the name so the caller can say which variable was
        // missing instead of reporting only a byte offset into the expression.
        m_runtime.RecordUndeclaredVariable(token.text);
        output = JBeamExpressionValue::Nil();
        return true;
    }

    bool ChargeValueCopy(
        const JBeamExpressionValue& value,
        std::size_t byte_offset)
    {
        return value.type != JBeamExpressionValueType::STRING ||
            m_runtime.Charge(value.string_value.size(), byte_offset);
    }

    bool RequireNumbers(
        const Token& operation,
        const JBeamExpressionValue& left,
        const JBeamExpressionValue& right)
    {
        if (left.type != JBeamExpressionValueType::NUMBER ||
            right.type != JBeamExpressionValueType::NUMBER)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "JBeam arithmetic operands must both be numbers");
        }
        return true;
    }

    bool ApplyArithmetic(
        const Token& operation,
        const JBeamExpressionValue& left,
        const JBeamExpressionValue& right,
        JBeamExpressionValue& output)
    {
        if (!m_runtime.Charge(1U, operation.begin) ||
            !RequireNumbers(operation, left, right))
        {
            return false;
        }
        if ((operation.kind == TokenKind::SLASH ||
             operation.kind == TokenKind::PERCENT) &&
            right.number_value == 0.0)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::DIVISION_BY_ZERO,
                operation.begin,
                "JBeam expression division by zero is rejected");
        }

        double result = 0.0;
        switch (operation.kind)
        {
        case TokenKind::PLUS:
            result = AddNumbers(left.number_value, right.number_value);
            break;
        case TokenKind::MINUS:
            result =
                SubtractNumbers(left.number_value, right.number_value);
            break;
        case TokenKind::STAR:
            result =
                MultiplyNumbers(left.number_value, right.number_value);
            break;
        case TokenKind::SLASH:
            result =
                DivideNumbers(left.number_value, right.number_value);
            break;
        case TokenKind::PERCENT:
        {
            // Decimal floating-point remainder is delegated to libm by many
            // standard libraries and its last bit is not a cross-platform
            // identity contract. Integer modulo is exact and implements Lua's
            // divisor-signed remainder without any floating-point library.
            static const double MAX_EXACT_INTEGER =
                9007199254740992.0; // 2^53
            std::int64_t left_integer = 0;
            std::int64_t right_integer = 0;
            if (!IsExactIntegerInRange(
                    left.number_value,
                    -MAX_EXACT_INTEGER,
                    MAX_EXACT_INTEGER,
                    left_integer) ||
                !IsExactIntegerInRange(
                    right.number_value,
                    -MAX_EXACT_INTEGER,
                    MAX_EXACT_INTEGER,
                    right_integer))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        NON_DETERMINISTIC_OPERAND,
                    operation.begin,
                    "JBeam '%' operands must be exact integers in "
                    "[-2^53, 2^53]");
            }
            result = ModuloIntegers(left_integer, right_integer);
            break;
        }
        case TokenKind::CARET:
        {
            // std::pow is intentionally avoided: its last-bit result is not
            // specified across C libraries. Integer exponents use this pinned
            // binary exponentiation sequence and the same explicit binary64
            // round trips as the evaluator's other arithmetic.
            static const std::int64_t MAX_POWER_EXPONENT = 1024;
            std::int64_t exponent = 0;
            if (!IsExactIntegerInRange(
                    right.number_value,
                    -static_cast<double>(MAX_POWER_EXPONENT),
                    static_cast<double>(MAX_POWER_EXPONENT),
                    exponent))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        NON_DETERMINISTIC_OPERAND,
                    operation.begin,
                    "JBeam '^' exponent must be an exact integer in "
                    "[-1024, 1024]");
            }
            if (left.number_value == 0.0 && exponent < 0)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::DIVISION_BY_ZERO,
                    operation.begin,
                    "JBeam zero cannot be raised to a negative power");
            }
            if (!PowerNumbers(
                    left.number_value,
                    static_cast<int>(exponent),
                    result))
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                    operation.begin,
                    "JBeam expression exponentiation produced a "
                    "non-finite value");
            }
            break;
        }
        default:
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "Internal JBeam arithmetic operator mismatch");
        }
        if (!IsFiniteNumber(result))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                operation.begin,
                "JBeam expression arithmetic produced a non-finite value");
        }
        output = JBeamExpressionValue::Number(
            NormalizeNumber(result));
        return true;
    }

    bool ApplyUnary(
        const Token& operation,
        const JBeamExpressionValue& operand,
        JBeamExpressionValue& output)
    {
        if (!m_runtime.Charge(1U, operation.begin))
        {
            return false;
        }
        if (operation.kind == TokenKind::NOT)
        {
            output = JBeamExpressionValue::Boolean(!IsTruthy(operand));
            return true;
        }
        if (operation.kind == TokenKind::LENGTH)
        {
            if (operand.type != JBeamExpressionValueType::STRING)
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                    operation.begin,
                    "JBeam length operand must be a string");
            }
            const double length =
                static_cast<double>(operand.string_value.size());
            if (static_cast<std::size_t>(length) !=
                operand.string_value.size())
            {
                return m_runtime.Fail(
                    JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                    operation.begin,
                    "JBeam string length is not exactly representable");
            }
            output = JBeamExpressionValue::Number(length);
            return true;
        }
        if (operand.type != JBeamExpressionValueType::NUMBER)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "JBeam unary minus operand must be a number");
        }
        volatile double observed = operand.number_value;
        volatile double result = -observed;
        if (!IsFiniteNumber(result))
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                operation.begin,
                "JBeam unary arithmetic produced a non-finite value");
        }
        output = JBeamExpressionValue::Number(
            NormalizeNumber(result));
        return true;
    }

    bool ApplyComparison(
        const Token& operation,
        const JBeamExpressionValue& left,
        const JBeamExpressionValue& right,
        JBeamExpressionValue& output)
    {
        if (!m_runtime.Charge(1U, operation.begin))
        {
            return false;
        }
        if (operation.kind == TokenKind::EQUAL ||
            operation.kind == TokenKind::NOT_EQUAL)
        {
            if (left.type == JBeamExpressionValueType::STRING &&
                right.type == JBeamExpressionValueType::STRING)
            {
                const std::size_t compared_bytes =
                    left.string_value.size() < right.string_value.size()
                        ? left.string_value.size()
                        : right.string_value.size();
                if (!m_runtime.Charge(compared_bytes, operation.begin))
                {
                    return false;
                }
            }
            const bool equal = ValuesEqual(left, right);
            output = JBeamExpressionValue::Boolean(
                operation.kind == TokenKind::EQUAL ? equal : !equal);
            return true;
        }

        int comparison = 0;
        if (left.type == JBeamExpressionValueType::NUMBER &&
            right.type == JBeamExpressionValueType::NUMBER)
        {
            comparison = left.number_value < right.number_value
                ? -1
                : (left.number_value > right.number_value ? 1 : 0);
        }
        else if (left.type == JBeamExpressionValueType::STRING &&
                 right.type == JBeamExpressionValueType::STRING)
        {
            const std::size_t compared_bytes =
                left.string_value.size() < right.string_value.size()
                    ? left.string_value.size()
                    : right.string_value.size();
            if (!m_runtime.Charge(compared_bytes, operation.begin))
            {
                return false;
            }
            comparison =
                CompareStrings(left.string_value, right.string_value);
        }
        else
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "JBeam relational operands must be matching numbers "
                "or strings");
        }

        bool result = false;
        switch (operation.kind)
        {
        case TokenKind::LESS:
            result = comparison < 0;
            break;
        case TokenKind::LESS_EQUAL:
            result = comparison <= 0;
            break;
        case TokenKind::GREATER:
            result = comparison > 0;
            break;
        case TokenKind::GREATER_EQUAL:
            result = comparison >= 0;
            break;
        default:
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "Internal JBeam relational operator mismatch");
        }
        output = JBeamExpressionValue::Boolean(result);
        return true;
    }

    bool ApplyConcatenation(
        const Token& operation,
        const JBeamExpressionValue& left,
        const JBeamExpressionValue& right,
        JBeamExpressionValue& output)
    {
        if (!m_runtime.Charge(1U, operation.begin))
        {
            return false;
        }
        if (left.type != JBeamExpressionValueType::STRING ||
            right.type != JBeamExpressionValueType::STRING)
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::TYPE_MISMATCH,
                operation.begin,
                "The safe JBeam concatenation subset requires two strings");
        }
        if (left.string_value.size() >
                m_limits.max_output_string_bytes ||
            right.string_value.size() >
                m_limits.max_output_string_bytes -
                    left.string_value.size())
        {
            return m_runtime.Fail(
                JBeamExpressionDiagnosticCode::OUTPUT_STRING_SIZE_LIMIT,
                operation.begin,
                "JBeam expression output string exceeds the configured limit");
        }
        const std::size_t output_size =
            left.string_value.size() + right.string_value.size();
        if (!m_runtime.Charge(output_size, operation.begin))
        {
            return false;
        }
        std::string joined;
        joined.reserve(output_size);
        joined.append(left.string_value);
        joined.append(right.string_value);
        output = JBeamExpressionValue::String(joined);
        return true;
    }

    const std::vector<Token>& m_tokens;
    const JBeamExpressionEnvironment& m_environment;
    const JBeamExpressionLimits& m_limits;
    Runtime& m_runtime;
    std::size_t m_index;
    std::size_t m_depth;
};

bool ValidateEnvironment(
    const JBeamExpressionEnvironment& environment,
    const JBeamExpressionLimits& limits,
    Runtime& runtime)
{
    if (environment.variables.size() > limits.max_variables)
    {
        return runtime.Fail(
            JBeamExpressionDiagnosticCode::ENVIRONMENT_LIMIT,
            0U,
            "JBeam expression environment exceeds the variable limit");
    }
    std::size_t retained_string_bytes = 0U;
    for (std::size_t index = 0U;
         index < environment.variables.size();
         ++index)
    {
        const JBeamExpressionVariable& variable =
            environment.variables[index];
        if (!runtime.Charge(1U, 0U) ||
            !runtime.Charge(variable.name.size(), 0U))
        {
            return false;
        }
        if (variable.name.size() >
                limits.max_variable_name_bytes ||
            !IsValidVariableName(variable.name))
        {
            return runtime.Fail(
                JBeamExpressionDiagnosticCode::
                    INVALID_ENVIRONMENT_VARIABLE,
                0U,
                "JBeam expression environment has an invalid variable name");
        }
        switch (variable.value.type)
        {
        case JBeamExpressionValueType::NIL_VALUE:
        case JBeamExpressionValueType::BOOLEAN:
            break;
        case JBeamExpressionValueType::NUMBER:
            if (!IsFiniteNumber(variable.value.number_value))
            {
                return runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_ENVIRONMENT_VALUE,
                    0U,
                    "JBeam expression environment number is not finite");
            }
            break;
        case JBeamExpressionValueType::STRING:
            if (variable.value.string_value.size() >
                    limits.max_string_bytes ||
                variable.value.string_value.size() >
                    limits.max_environment_string_bytes -
                        retained_string_bytes)
            {
                return runtime.Fail(
                    JBeamExpressionDiagnosticCode::ENVIRONMENT_LIMIT,
                    0U,
                    "JBeam expression environment strings exceed "
                    "the configured limit");
            }
            if (!runtime.Charge(
                    variable.value.string_value.size(), 0U))
            {
                return false;
            }
            if (!IsValidUtf8(variable.value.string_value))
            {
                return runtime.Fail(
                    JBeamExpressionDiagnosticCode::
                        INVALID_ENVIRONMENT_VALUE,
                    0U,
                    "JBeam expression environment string is not valid UTF-8");
            }
            retained_string_bytes +=
                variable.value.string_value.size();
            break;
        default:
            return runtime.Fail(
                JBeamExpressionDiagnosticCode::INVALID_ENVIRONMENT_VALUE,
                0U,
                "JBeam expression environment value type is invalid");
        }
    }
    return true;
}

void AppendSize(std::size_t value, std::string& output)
{
    char digits[
        std::numeric_limits<std::size_t>::digits10 + 2U];
    std::size_t count = 0U;
    do
    {
        digits[count++] = static_cast<char>(
            '0' + static_cast<char>(value % 10U));
        value /= 10U;
    } while (value > 0U);
    while (count > 0U)
    {
        output.push_back(digits[--count]);
    }
}

} // namespace

JBeamExpressionValue::JBeamExpressionValue()
    : type(JBeamExpressionValueType::NIL_VALUE)
    , boolean_value(false)
    , number_value(0.0)
{
}

JBeamExpressionValue JBeamExpressionValue::Nil()
{
    return JBeamExpressionValue();
}

JBeamExpressionValue JBeamExpressionValue::Boolean(bool value)
{
    JBeamExpressionValue result;
    result.type = JBeamExpressionValueType::BOOLEAN;
    result.boolean_value = value;
    return result;
}

JBeamExpressionValue JBeamExpressionValue::Number(double value)
{
    JBeamExpressionValue result;
    result.type = JBeamExpressionValueType::NUMBER;
    result.number_value = value;
    return result;
}

JBeamExpressionValue JBeamExpressionValue::String(
    const std::string& value)
{
    JBeamExpressionValue result;
    result.type = JBeamExpressionValueType::STRING;
    result.string_value = value;
    return result;
}

JBeamExpressionLimits::JBeamExpressionLimits()
    : max_expression_bytes(65536U)
    , max_tokens(4096U)
    , max_depth(128U)
    , max_function_arguments(64U)
    , max_work_units(4194304U)
    , max_string_bytes(65536U)
    , max_output_string_bytes(65536U)
    , max_variables(4096U)
    , max_variable_name_bytes(256U)
    , max_environment_string_bytes(1048576U)
{
}

JBeamExpressionDiagnostic::JBeamExpressionDiagnostic()
    : code(JBeamExpressionDiagnosticCode::EXPECTED_VALUE)
    , byte_offset(0U)
{
}

JBeamExpressionResult::JBeamExpressionResult()
    : token_count(0U)
    , work_units(0U)
    , has_value(false)
{
}

bool JBeamExpressionResult::IsValid() const
{
    return has_value && diagnostics.empty();
}

bool JBeamExpressionFailedOnUndeclaredVariableOnly(
    const JBeamExpressionResult& result)
{
    if (result.IsValid() ||
        result.diagnostics.empty() ||
        result.undeclared_variables.empty())
    {
        return false;
    }
    // Every diagnostic must be the type mismatch that arithmetic on nil
    // produces. A resource limit, a syntax error, or a mismatch between two
    // *declared* values is a different failure and stays hard, so this can
    // never soften one.
    for (std::size_t i = 0U; i < result.diagnostics.size(); ++i)
    {
        if (result.diagnostics[i].code !=
            JBeamExpressionDiagnosticCode::TYPE_MISMATCH)
        {
            return false;
        }
    }
    return true;
}

JBeamExpressionResult EvaluateJBeamExpression(
    const std::string& expression,
    const JBeamExpressionEnvironment& environment,
    const JBeamExpressionLimits& limits)
{
    JBeamExpressionResult result;
    Runtime runtime(result, limits);
    if (expression.size() > limits.max_expression_bytes)
    {
        runtime.Fail(
            JBeamExpressionDiagnosticCode::EXPRESSION_SIZE_LIMIT,
            0U,
            "JBeam expression exceeds the configured byte limit");
        return result;
    }
    if (expression.size() < 2U ||
        expression[0] != '$' ||
        expression[1] != '=')
    {
        runtime.Fail(
            JBeamExpressionDiagnosticCode::MISSING_EXPRESSION_PREFIX,
            0U,
            "JBeam expression must begin with '$='");
        return result;
    }
    if (!ValidateEnvironment(environment, limits, runtime))
    {
        return result;
    }

    std::vector<Token> tokens;
    const std::size_t reserve_count =
        expression.size() < limits.max_tokens
            ? expression.size()
            : limits.max_tokens;
    tokens.reserve(reserve_count);
    Lexer lexer(expression, limits, runtime);
    if (!lexer.Run(tokens))
    {
        result.token_count = tokens.size();
        return result;
    }
    result.token_count = tokens.size();
    Parser parser(tokens, environment, limits, runtime);
    JBeamExpressionValue value;
    if (!parser.Run(value))
    {
        return result;
    }
    if (value.type == JBeamExpressionValueType::NUMBER)
    {
        if (!IsFiniteNumber(value.number_value))
        {
            runtime.Fail(
                JBeamExpressionDiagnosticCode::NON_FINITE_RESULT,
                0U,
                "JBeam expression result is not finite");
            return result;
        }
        value.number_value = NormalizeNumber(value.number_value);
    }
    if (value.type == JBeamExpressionValueType::STRING &&
        value.string_value.size() > limits.max_output_string_bytes)
    {
        runtime.Fail(
            JBeamExpressionDiagnosticCode::OUTPUT_STRING_SIZE_LIMIT,
            0U,
            "JBeam expression output string exceeds the configured limit");
        return result;
    }
    if (value.type == JBeamExpressionValueType::STRING &&
        !runtime.Charge(value.string_value.size(), 0U))
    {
        return result;
    }
    result.value = value;
    result.has_value = true;
    return result;
}

std::string SerializeCanonicalJBeamExpressionValue(
    const JBeamExpressionValue& value)
{
    switch (value.type)
    {
    case JBeamExpressionValueType::NIL_VALUE:
        return "jbeam-expression-value-v1:nil";
    case JBeamExpressionValueType::BOOLEAN:
        return value.boolean_value
            ? "jbeam-expression-value-v1:boolean:1"
            : "jbeam-expression-value-v1:boolean:0";
    case JBeamExpressionValueType::NUMBER:
    {
        if (!IsFiniteNumber(value.number_value))
        {
            return "jbeam-expression-value-v1:invalid";
        }
        std::uint64_t bits = DoubleBits(value.number_value);
        if ((bits & UINT64_C(0x7fffffffffffffff)) == 0U)
        {
            // Mask the sign in integer space. Under -fno-signed-zeros the
            // optimizer may legally preserve a floating-point -0 even when a
            // helper returns literal +0.
            bits = 0U;
        }
        static const char hex[] = "0123456789abcdef";
        std::string output = "jbeam-expression-value-v1:number:";
        for (unsigned int shift = 60U;; shift -= 4U)
        {
            output.push_back(
                hex[static_cast<std::size_t>((bits >> shift) & 0xfU)]);
            if (shift == 0U)
            {
                break;
            }
        }
        return output;
    }
    case JBeamExpressionValueType::STRING:
    {
        if (!IsValidUtf8(value.string_value))
        {
            return "jbeam-expression-value-v1:invalid";
        }
        std::string output = "jbeam-expression-value-v1:string:";
        AppendSize(value.string_value.size(), output);
        output.push_back(':');
        output.append(value.string_value);
        return output;
    }
    }
    return "jbeam-expression-value-v1:invalid";
}

const char* JBeamExpressionDiagnosticCodeToString(
    JBeamExpressionDiagnosticCode code)
{
    switch (code)
    {
    case JBeamExpressionDiagnosticCode::EXPRESSION_SIZE_LIMIT:
        return "expression-size-limit";
    case JBeamExpressionDiagnosticCode::MISSING_EXPRESSION_PREFIX:
        return "missing-expression-prefix";
    case JBeamExpressionDiagnosticCode::TOKEN_LIMIT:
        return "token-limit";
    case JBeamExpressionDiagnosticCode::DEPTH_LIMIT:
        return "depth-limit";
    case JBeamExpressionDiagnosticCode::WORK_LIMIT:
        return "work-limit";
    case JBeamExpressionDiagnosticCode::STRING_SIZE_LIMIT:
        return "string-size-limit";
    case JBeamExpressionDiagnosticCode::OUTPUT_STRING_SIZE_LIMIT:
        return "output-string-size-limit";
    case JBeamExpressionDiagnosticCode::ENVIRONMENT_LIMIT:
        return "environment-limit";
    case JBeamExpressionDiagnosticCode::INVALID_ENVIRONMENT_VARIABLE:
        return "invalid-environment-variable";
    case JBeamExpressionDiagnosticCode::INVALID_ENVIRONMENT_VALUE:
        return "invalid-environment-value";
    case JBeamExpressionDiagnosticCode::INVALID_CHARACTER:
        return "invalid-character";
    case JBeamExpressionDiagnosticCode::INVALID_UTF8:
        return "invalid-utf8";
    case JBeamExpressionDiagnosticCode::INVALID_NUMBER:
        return "invalid-number";
    case JBeamExpressionDiagnosticCode::NON_FINITE_NUMBER:
        return "non-finite-number";
    case JBeamExpressionDiagnosticCode::UNTERMINATED_STRING:
        return "unterminated-string";
    case JBeamExpressionDiagnosticCode::INVALID_ESCAPE:
        return "invalid-escape";
    case JBeamExpressionDiagnosticCode::EXPECTED_VALUE:
        return "expected-value";
    case JBeamExpressionDiagnosticCode::EXPECTED_TOKEN:
        return "expected-token";
    case JBeamExpressionDiagnosticCode::TRAILING_CONTENT:
        return "trailing-content";
    case JBeamExpressionDiagnosticCode::UNSUPPORTED_FUNCTION:
        return "unsupported-function";
    case JBeamExpressionDiagnosticCode::FUNCTION_ARITY:
        return "function-arity";
    case JBeamExpressionDiagnosticCode::FUNCTION_ARGUMENT_LIMIT:
        return "function-argument-limit";
    case JBeamExpressionDiagnosticCode::INVALID_FUNCTION_ARGUMENT:
        return "invalid-function-argument";
    case JBeamExpressionDiagnosticCode::UNSUPPORTED_CASE_SIGNATURE:
        return "unsupported-case-signature";
    case JBeamExpressionDiagnosticCode::TYPE_MISMATCH:
        return "type-mismatch";
    case JBeamExpressionDiagnosticCode::DIVISION_BY_ZERO:
        return "division-by-zero";
    case JBeamExpressionDiagnosticCode::NON_DETERMINISTIC_OPERAND:
        return "non-deterministic-operand";
    case JBeamExpressionDiagnosticCode::NON_FINITE_RESULT:
        return "non-finite-result";
    }
    return "unknown";
}

} // namespace BeamNG
} // namespace RoR
