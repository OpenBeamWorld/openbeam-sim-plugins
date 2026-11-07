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

#include "JBeamSyntax.h"

#include "JBeamCoordinateTransform.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <utility>

namespace RoR {
namespace BeamNG {

namespace {

bool HasErrors(const std::vector<JBeamDiagnostic>& diagnostics)
{
    for (std::size_t i = 0; i < diagnostics.size(); ++i)
    {
        if (diagnostics[i].severity == JBeamDiagnosticSeverity::ERROR_SEVERITY)
        {
            return true;
        }
    }
    return false;
}

bool DiagnosticLess(
    const JBeamDiagnostic& left,
    const JBeamDiagnostic& right)
{
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
    if (left.severity != right.severity)
    {
        return static_cast<int>(left.severity) <
               static_cast<int>(right.severity);
    }
    if (left.code != right.code)
    {
        return static_cast<int>(left.code) < static_cast<int>(right.code);
    }
    return left.message < right.message;
}

void SortDiagnostics(std::vector<JBeamDiagnostic>& diagnostics)
{
    std::stable_sort(
        diagnostics.begin(), diagnostics.end(), DiagnosticLess);
}

JBeamDiagnostic MakeDiagnostic(
    JBeamDiagnosticCode code,
    JBeamDiagnosticSeverity severity,
    const JBeamSourceSpan& span,
    const std::string& message)
{
    JBeamDiagnostic diagnostic;
    diagnostic.code = code;
    diagnostic.severity = severity;
    diagnostic.span = span;
    diagnostic.message = message;
    return diagnostic;
}

class Lexer
{
public:
    Lexer(
        const std::string& source,
        const std::string& source_name,
        const JBeamParseLimits& limits)
        : m_source(source)
        , m_source_name(source_name)
        , m_limits(limits)
        , m_offset(0)
        , m_line(1)
        , m_column(1)
        , m_stopped(false)
    {
    }

    JBeamLexResult Run()
    {
        if (m_source.size() > m_limits.max_source_bytes)
        {
            const JBeamSourcePosition begin = Position();
            JBeamSourcePosition end = begin;
            end.byte_offset = static_cast<std::uint64_t>(m_source.size());
            m_result.diagnostics.push_back(MakeDiagnostic(
                JBeamDiagnosticCode::SOURCE_SIZE_LIMIT,
                JBeamDiagnosticSeverity::ERROR_SEVERITY,
                Span(begin, end),
                "JBeam source exceeds the configured byte limit"));
            return m_result;
        }

        while (!m_stopped)
        {
            SkipWhitespaceAndComments();
            if (m_stopped)
            {
                break;
            }
            if (AtEnd())
            {
                EmitSimple(JBeamTokenKind::END_OF_INPUT, Position(), Position());
                break;
            }
            LexOne();
        }
        return m_result;
    }

private:
    bool AtEnd() const
    {
        return m_offset >= m_source.size();
    }

    char Peek(std::size_t ahead = 0) const
    {
        const std::size_t index = m_offset + ahead;
        return index < m_source.size() ? m_source[index] : '\0';
    }

    JBeamSourcePosition Position() const
    {
        JBeamSourcePosition position;
        position.byte_offset = static_cast<std::uint64_t>(m_offset);
        position.line = static_cast<std::uint64_t>(m_line);
        position.column = static_cast<std::uint64_t>(m_column);
        return position;
    }

    JBeamSourceSpan Span(
        const JBeamSourcePosition& begin,
        const JBeamSourcePosition& end) const
    {
        JBeamSourceSpan span;
        span.source_name = m_source_name;
        span.begin = begin;
        span.end = end;
        return span;
    }

    char Advance()
    {
        const char value = Peek();
        if (AtEnd())
        {
            return '\0';
        }
        ++m_offset;
        if (value == '\r')
        {
            if (Peek() == '\n')
            {
                ++m_offset;
            }
            ++m_line;
            m_column = 1;
        }
        else if (value == '\n')
        {
            ++m_line;
            m_column = 1;
        }
        else
        {
            ++m_column;
        }
        return value;
    }

    void AddError(
        JBeamDiagnosticCode code,
        const JBeamSourcePosition& begin,
        const std::string& message)
    {
        if (m_result.diagnostics.size() < m_limits.max_diagnostics)
        {
            m_result.diagnostics.push_back(MakeDiagnostic(
                code,
                JBeamDiagnosticSeverity::ERROR_SEVERITY,
                Span(begin, Position()),
                message));
        }
        else
        {
            m_result.diagnostics.push_back(MakeDiagnostic(
                JBeamDiagnosticCode::DIAGNOSTIC_LIMIT,
                JBeamDiagnosticSeverity::ERROR_SEVERITY,
                Span(begin, Position()),
                "JBeam diagnostic count reached the configured limit"));
        }
        // A lexer error cannot be recovered without inventing token
        // boundaries. Failing at the first error is deterministic and keeps
        // hostile malformed sources within the diagnostic memory budget.
        m_stopped = true;
    }

    bool EnsureTokenBudget(const JBeamSourcePosition& begin)
    {
        if (m_result.tokens.size() < m_limits.max_tokens)
        {
            return true;
        }
        AddError(
            JBeamDiagnosticCode::TOKEN_LIMIT,
            begin,
            "JBeam token count exceeds the configured limit");
        m_stopped = true;
        return false;
    }

    void EmitSimple(
        JBeamTokenKind kind,
        const JBeamSourcePosition& begin,
        const JBeamSourcePosition& end)
    {
        if (!EnsureTokenBudget(begin))
        {
            return;
        }
        JBeamToken token;
        token.kind = kind;
        token.span = Span(begin, end);
        m_result.tokens.push_back(token);
    }

    void EmitText(
        JBeamTokenKind kind,
        const std::string& text,
        const JBeamSourcePosition& begin)
    {
        if (!EnsureTokenBudget(begin))
        {
            return;
        }
        JBeamToken token;
        token.kind = kind;
        token.text = text;
        token.span = Span(begin, Position());
        m_result.tokens.push_back(token);
    }

    void SkipWhitespaceAndComments()
    {
        for (;;)
        {
            while (!AtEnd())
            {
                const char value = Peek();
                if (value != ' ' && value != '\t' &&
                    value != '\r' && value != '\n')
                {
                    break;
                }
                Advance();
            }

            if (Peek() == '/' && Peek(1) == '/')
            {
                Advance();
                Advance();
                while (!AtEnd() && Peek() != '\r' && Peek() != '\n')
                {
                    Advance();
                }
                continue;
            }
            if (Peek() == '/' && Peek(1) == '*')
            {
                const JBeamSourcePosition begin = Position();
                Advance();
                Advance();
                bool terminated = false;
                while (!AtEnd())
                {
                    if (Peek() == '*' && Peek(1) == '/')
                    {
                        Advance();
                        Advance();
                        terminated = true;
                        break;
                    }
                    Advance();
                }
                if (!terminated)
                {
                    AddError(
                        JBeamDiagnosticCode::UNTERMINATED_BLOCK_COMMENT,
                        begin,
                        "Unterminated JBeam block comment");
                    m_stopped = true;
                    return;
                }
                continue;
            }
            return;
        }
    }

    static bool IsDigit(char value)
    {
        return value >= '0' && value <= '9';
    }

    static int HexValue(char value)
    {
        if (value >= '0' && value <= '9')
        {
            return value - '0';
        }
        if (value >= 'a' && value <= 'f')
        {
            return 10 + value - 'a';
        }
        if (value >= 'A' && value <= 'F')
        {
            return 10 + value - 'A';
        }
        return -1;
    }

    static void AppendUtf8(std::uint32_t code_point, std::string& output)
    {
        if (code_point <= 0x7fU)
        {
            output.push_back(static_cast<char>(code_point));
        }
        else if (code_point <= 0x7ffU)
        {
            output.push_back(static_cast<char>(0xc0U | (code_point >> 6)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
        }
        else if (code_point <= 0xffffU)
        {
            output.push_back(static_cast<char>(0xe0U | (code_point >> 12)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 6) & 0x3fU)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
        }
        else
        {
            output.push_back(static_cast<char>(0xf0U | (code_point >> 18)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 12) & 0x3fU)));
            output.push_back(
                static_cast<char>(0x80U | ((code_point >> 6) & 0x3fU)));
            output.push_back(static_cast<char>(0x80U | (code_point & 0x3fU)));
        }
    }

    bool ReadHexCodeUnit(std::uint32_t& code_unit)
    {
        code_unit = 0;
        for (int i = 0; i < 4; ++i)
        {
            const int digit = HexValue(Peek());
            if (digit < 0)
            {
                return false;
            }
            code_unit = (code_unit << 4) | static_cast<std::uint32_t>(digit);
            Advance();
        }
        return true;
    }

    bool CheckStringBudget(
        const std::string& output,
        const JBeamSourcePosition& begin)
    {
        if (output.size() <= m_limits.max_string_bytes)
        {
            return true;
        }
        AddError(
            JBeamDiagnosticCode::STRING_SIZE_LIMIT,
            begin,
            "Decoded JBeam string exceeds the configured byte limit");
        return false;
    }

    bool AppendRawUtf8(
        unsigned char first,
        const JBeamSourcePosition& character_begin,
        std::string& output)
    {
        std::size_t continuation_count = 0;
        std::uint32_t code_point = 0;
        std::uint32_t minimum = 0;
        if (first >= 0xc2U && first <= 0xdfU)
        {
            continuation_count = 1;
            code_point = first & 0x1fU;
            minimum = 0x80U;
        }
        else if (first >= 0xe0U && first <= 0xefU)
        {
            continuation_count = 2;
            code_point = first & 0x0fU;
            minimum = 0x800U;
        }
        else if (first >= 0xf0U && first <= 0xf4U)
        {
            continuation_count = 3;
            code_point = first & 0x07U;
            minimum = 0x10000U;
        }
        else
        {
            AddError(
                JBeamDiagnosticCode::INVALID_UTF8,
                character_begin,
                "Invalid UTF-8 leading byte in JBeam string");
            return false;
        }

        for (std::size_t i = 0; i < continuation_count; ++i)
        {
            const unsigned char continuation =
                static_cast<unsigned char>(Peek(i));
            if ((continuation & 0xc0U) != 0x80U)
            {
                AddError(
                    JBeamDiagnosticCode::INVALID_UTF8,
                    character_begin,
                    "Invalid UTF-8 continuation byte in JBeam string");
                return false;
            }
            code_point =
                (code_point << 6) |
                static_cast<std::uint32_t>(continuation & 0x3fU);
        }
        if (code_point < minimum || code_point > 0x10ffffU ||
            (code_point >= 0xd800U && code_point <= 0xdfffU))
        {
            AddError(
                JBeamDiagnosticCode::INVALID_UTF8,
                character_begin,
                "Non-canonical Unicode scalar in JBeam UTF-8 string");
            return false;
        }

        output.push_back(static_cast<char>(first));
        for (std::size_t i = 0; i < continuation_count; ++i)
        {
            output.push_back(Advance());
        }
        return true;
    }

    void LexString()
    {
        const JBeamSourcePosition begin = Position();
        Advance();
        std::string output;
        bool terminated = false;
        while (!AtEnd())
        {
            const JBeamSourcePosition character_begin = Position();
            const char value = Advance();
            if (value == '"')
            {
                terminated = true;
                break;
            }
            const unsigned char unsigned_value =
                static_cast<unsigned char>(value);
            if (unsigned_value < 0x20U)
            {
                AddError(
                    JBeamDiagnosticCode::INVALID_STRING_CHARACTER,
                    begin,
                    "Unescaped control character in JBeam string");
                return;
            }
            if (value != '\\')
            {
                if (unsigned_value < 0x80U)
                {
                    output.push_back(value);
                }
                else if (!AppendRawUtf8(
                             unsigned_value, character_begin, output))
                {
                    return;
                }
                if (!CheckStringBudget(output, begin))
                {
                    return;
                }
                continue;
            }

            if (AtEnd())
            {
                break;
            }
            const char escaped = Advance();
            switch (escaped)
            {
            case '"': output.push_back('"'); break;
            case '\\': output.push_back('\\'); break;
            case '/': output.push_back('/'); break;
            case 'b': output.push_back('\b'); break;
            case 'f': output.push_back('\f'); break;
            case 'n': output.push_back('\n'); break;
            case 'r': output.push_back('\r'); break;
            case 't': output.push_back('\t'); break;
            case 'u':
            {
                std::uint32_t first = 0;
                if (!ReadHexCodeUnit(first))
                {
                    AddError(
                        JBeamDiagnosticCode::INVALID_UNICODE_ESCAPE,
                        begin,
                        "Invalid four-digit Unicode escape in JBeam string");
                    return;
                }
                std::uint32_t code_point = first;
                if (first >= 0xd800U && first <= 0xdbffU)
                {
                    if (Peek() != '\\' || Peek(1) != 'u')
                    {
                        AddError(
                            JBeamDiagnosticCode::INVALID_UNICODE_SURROGATE,
                            begin,
                            "High Unicode surrogate has no low surrogate");
                        return;
                    }
                    Advance();
                    Advance();
                    std::uint32_t second = 0;
                    if (!ReadHexCodeUnit(second) ||
                        second < 0xdc00U || second > 0xdfffU)
                    {
                        AddError(
                            JBeamDiagnosticCode::INVALID_UNICODE_SURROGATE,
                            begin,
                            "Invalid low Unicode surrogate");
                        return;
                    }
                    code_point =
                        0x10000U + ((first - 0xd800U) << 10) +
                        (second - 0xdc00U);
                }
                else if (first >= 0xdc00U && first <= 0xdfffU)
                {
                    AddError(
                        JBeamDiagnosticCode::INVALID_UNICODE_SURROGATE,
                        begin,
                        "Unexpected low Unicode surrogate");
                    return;
                }
                AppendUtf8(code_point, output);
                break;
            }
            default:
                AddError(
                    JBeamDiagnosticCode::INVALID_ESCAPE,
                    begin,
                    "Invalid escape in JBeam string");
                return;
            }
            if (!CheckStringBudget(output, begin))
            {
                return;
            }
        }
        if (!terminated)
        {
            AddError(
                JBeamDiagnosticCode::UNTERMINATED_STRING,
                begin,
                "Unterminated JBeam string");
            return;
        }
        EmitText(JBeamTokenKind::STRING, output, begin);
    }

    void LexNumber()
    {
        const JBeamSourcePosition begin = Position();
        const std::size_t begin_offset = m_offset;
        if (Peek() == '-')
        {
            Advance();
        }
        if (Peek() == '0')
        {
            Advance();
            // Strict JSON forbids a leading zero. JBeam does not, and shipped
            // content relies on it: the Germanwings A319 writes
            // `{"dragCoef": 00, "liftCoef": 00}`, which BeamNG reads as zero.
            // The reading is unambiguous -- `00` is 0, `007` is 7 -- so the
            // digits are consumed and the value parses normally. This relaxes
            // only what content demonstrates; bare keys, NaN and Infinity
            // remain errors, and those are not merely stricter spellings.
            while (IsDigit(Peek()))
            {
                Advance();
            }
        }
        else if (Peek() >= '1' && Peek() <= '9')
        {
            while (IsDigit(Peek()))
            {
                Advance();
            }
        }
        else
        {
            if (!AtEnd())
            {
                Advance();
            }
            AddError(
                JBeamDiagnosticCode::INVALID_NUMBER,
                begin,
                "JBeam number requires an integer component");
            return;
        }

        if (Peek() == '.')
        {
            Advance();
            if (!IsDigit(Peek()))
            {
                AddError(
                    JBeamDiagnosticCode::INVALID_NUMBER,
                    begin,
                    "JBeam number requires digits after the decimal point");
                return;
            }
            while (IsDigit(Peek()))
            {
                Advance();
            }
        }

        if (Peek() == 'e' || Peek() == 'E')
        {
            Advance();
            if (Peek() == '+' || Peek() == '-')
            {
                Advance();
            }
            if (!IsDigit(Peek()))
            {
                AddError(
                    JBeamDiagnosticCode::INVALID_NUMBER,
                    begin,
                    "JBeam number requires exponent digits");
                return;
            }
            while (IsDigit(Peek()))
            {
                Advance();
            }
        }
        EmitText(
            JBeamTokenKind::NUMBER,
            m_source.substr(begin_offset, m_offset - begin_offset),
            begin);
    }

    bool MatchKeyword(const char* keyword)
    {
        std::size_t length = 0;
        while (keyword[length] != '\0')
        {
            if (Peek(length) != keyword[length])
            {
                return false;
            }
            ++length;
        }
        for (std::size_t i = 0; i < length; ++i)
        {
            Advance();
        }
        return true;
    }

    void LexOne()
    {
        const JBeamSourcePosition begin = Position();
        const char value = Peek();
        switch (value)
        {
        case '{':
            Advance();
            EmitSimple(JBeamTokenKind::LEFT_BRACE, begin, Position());
            return;
        case '}':
            Advance();
            EmitSimple(JBeamTokenKind::RIGHT_BRACE, begin, Position());
            return;
        case '[':
            Advance();
            EmitSimple(JBeamTokenKind::LEFT_BRACKET, begin, Position());
            return;
        case ']':
            Advance();
            EmitSimple(JBeamTokenKind::RIGHT_BRACKET, begin, Position());
            return;
        case ':':
            Advance();
            EmitSimple(JBeamTokenKind::COLON, begin, Position());
            return;
        case ',':
            Advance();
            EmitSimple(JBeamTokenKind::COMMA, begin, Position());
            return;
        case '"':
            LexString();
            return;
        case '-':
            LexNumber();
            return;
        default:
            break;
        }

        if (IsDigit(value))
        {
            LexNumber();
            return;
        }
        if (MatchKeyword("true"))
        {
            EmitSimple(JBeamTokenKind::TRUE_VALUE, begin, Position());
            return;
        }
        if (MatchKeyword("false"))
        {
            EmitSimple(JBeamTokenKind::FALSE_VALUE, begin, Position());
            return;
        }
        if (MatchKeyword("null"))
        {
            EmitSimple(JBeamTokenKind::NULL_VALUE, begin, Position());
            return;
        }

        // An unquoted identifier is an object key: BeamNG writes them and the
        // NLR ZX-6R ships `{coreSlot:true}`. It is emitted as its own kind so
        // that only the object parser can accept it -- a bare word where a
        // VALUE belongs stays exactly as invalid as it was.
        const char lead = Peek();
        const bool identifier_start =
            (lead >= 'A' && lead <= 'Z') || (lead >= 'a' && lead <= 'z') ||
            lead == '_';
        if (identifier_start)
        {
            std::string identifier;
            while (!AtEnd())
            {
                const char c = Peek();
                const bool identifier_body =
                    (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_';
                if (!identifier_body)
                {
                    break;
                }
                identifier.push_back(c);
                Advance();
            }
            EmitText(JBeamTokenKind::BARE_KEY, identifier, begin);
            return;
        }

        Advance();
        AddError(
            JBeamDiagnosticCode::INVALID_CHARACTER,
            begin,
            "Invalid character in JBeam source");
    }

    const std::string& m_source;
    const std::string& m_source_name;
    const JBeamParseLimits& m_limits;
    std::size_t m_offset;
    std::size_t m_line;
    std::size_t m_column;
    bool m_stopped;
    JBeamLexResult m_result;
};

class Parser
{
public:
    Parser(
        const std::vector<JBeamToken>& tokens,
        const std::vector<JBeamDiagnostic>& lex_diagnostics,
        const JBeamParseLimits& limits)
        : m_tokens(tokens)
        , m_limits(limits)
        , m_index(0)
        , m_node_count(0)
    {
        m_result.diagnostics = lex_diagnostics;
    }

    JBeamParseResult Run()
    {
        if (HasErrors(m_result.diagnostics) || m_tokens.empty())
        {
            return m_result;
        }
        JBeamValue value;
        if (!ParseValue(0, value))
        {
            return m_result;
        }
        m_result.root = value;
        m_result.has_root = true;

        // Real-world packages occasionally close the root object early and
        // continue with more top-level members or a second concatenated
        // object; BeamNG's own reader tolerates the shape. A bounded
        // recovery merges the continued top-level content into the root
        // object with a warning diagnostic per recovered segment. All
        // ordinary value quotas still apply, and any other trailing
        // content remains a fail-closed error.
        if (m_result.root.type == JBeamValueType::OBJECT)
        {
            while (Current().kind != JBeamTokenKind::END_OF_INPUT)
            {
                if (Match(JBeamTokenKind::COMMA))
                {
                    continue;
                }
                if (Current().kind == JBeamTokenKind::RIGHT_BRACE ||
                    Current().kind == JBeamTokenKind::RIGHT_BRACKET)
                {
                    // A closer after the complete root closes nothing: the
                    // root value is already whole, so the closer has no
                    // reading but a surplus. Shipped content ends a part
                    // file with one -- a Nissan GT-R R35 mod's
                    // r35_rollcage.jbeam closes its root twice -- and one
                    // such file refused the whole archive.
                    if (!AddWarning(
                        JBeamDiagnosticCode::SURPLUS_ROOT_CLOSER,
                        Current().span,
                        "Closing bracket after the complete root JBeam "
                        "value is ignored"))
                    {
                        return m_result;
                    }
                    ++m_index;
                    continue;
                }
                if (Current().kind == JBeamTokenKind::LEFT_BRACE)
                {
                    if (!AddWarning(
                        JBeamDiagnosticCode::CONCATENATED_ROOT_OBJECT,
                        Current().span,
                        "Concatenated top-level object is merged into "
                        "the root object"))
                    {
                        return m_result;
                    }
                    JBeamValue continued;
                    if (!ParseObject(0, continued))
                    {
                        return m_result;
                    }
                    for (std::size_t i = 0;
                         i < continued.object_fields.size();
                         ++i)
                    {
                        m_result.root.object_fields.push_back(
                            continued.object_fields[i]);
                    }
                    continue;
                }
                if (Current().kind == JBeamTokenKind::STRING)
                {
                    if (!AddWarning(
                        JBeamDiagnosticCode::CONCATENATED_ROOT_OBJECT,
                        Current().span,
                        "Top-level member after the root object is "
                        "merged into the root object"))
                    {
                        return m_result;
                    }
                    const JBeamToken key = Current();
                    ++m_index;
                    // The same key-to-colon rule as ParseObject.
                    SkipCommasBeforeColon();
                    if (!Match(JBeamTokenKind::COLON))
                    {
                        AddError(
                            JBeamDiagnosticCode::EXPECTED_COLON,
                            Current().span,
                            "Expected ':' after JBeam object key");
                        return m_result;
                    }
                    JBeamValue member_value;
                    if (!ParseValue(1, member_value))
                    {
                        return m_result;
                    }
                    JBeamObjectField field;
                    field.key = key.text;
                    field.key_span = key.span;
                    field.value = std::shared_ptr<const JBeamValue>(
                        new JBeamValue(member_value));
                    m_result.root.object_fields.push_back(field);
                    continue;
                }
                break;
            }
        }
        if (Current().kind != JBeamTokenKind::END_OF_INPUT)
        {
            AddError(
                JBeamDiagnosticCode::TRAILING_CONTENT,
                Current().span,
                "Unexpected content after the root JBeam value");
        }
        return m_result;
    }

private:
    const JBeamToken& Current() const
    {
        if (m_index < m_tokens.size())
        {
            return m_tokens[m_index];
        }
        return m_tokens.back();
    }

    const JBeamToken& Previous() const
    {
        return m_tokens[m_index - 1];
    }

    bool Match(JBeamTokenKind kind)
    {
        if (Current().kind != kind)
        {
            return false;
        }
        ++m_index;
        return true;
    }

    void AddError(
        JBeamDiagnosticCode code,
        const JBeamSourceSpan& span,
        const std::string& message)
    {
        if (m_result.diagnostics.size() < m_limits.max_diagnostics)
        {
            m_result.diagnostics.push_back(MakeDiagnostic(
                code, JBeamDiagnosticSeverity::ERROR_SEVERITY, span, message));
        }
        else
        {
            m_result.diagnostics.push_back(MakeDiagnostic(
                JBeamDiagnosticCode::DIAGNOSTIC_LIMIT,
                JBeamDiagnosticSeverity::ERROR_SEVERITY,
                span,
                "JBeam diagnostic count reached the configured limit"));
        }
    }

    bool AddWarning(
        JBeamDiagnosticCode code,
        const JBeamSourceSpan& span,
        const std::string& message)
    {
        if (m_result.diagnostics.size() >= m_limits.max_diagnostics)
        {
            m_result.diagnostics.push_back(MakeDiagnostic(
                JBeamDiagnosticCode::DIAGNOSTIC_LIMIT,
                JBeamDiagnosticSeverity::ERROR_SEVERITY,
                span,
                "JBeam diagnostic count reached the configured limit"));
            return false;
        }
        m_result.diagnostics.push_back(MakeDiagnostic(
            code, JBeamDiagnosticSeverity::WARNING, span, message));
        return true;
    }

    bool BeginNode(std::size_t depth, const JBeamSourceSpan& span)
    {
        if (depth > m_limits.max_depth)
        {
            AddError(
                JBeamDiagnosticCode::DEPTH_LIMIT,
                span,
                "JBeam nesting exceeds the configured depth limit");
            return false;
        }
        if (m_node_count >= m_limits.max_nodes)
        {
            AddError(
                JBeamDiagnosticCode::NODE_LIMIT,
                span,
                "JBeam value count exceeds the configured node limit");
            return false;
        }
        ++m_node_count;
        return true;
    }

    // Fast-math-proof finiteness through the shared volatile-byte
    // classification; std::isfinite (and even plain memcpy bit checks)
    // can fold under -ffinite-math-only.
    static bool HasFiniteBits(double value)
    {
        return Detail::IsFiniteBinary64(value);
    }

    static bool ParseFiniteDouble(const std::string& text, double& value)
    {
        std::istringstream input(text);
        input.imbue(std::locale::classic());
        input >> std::noskipws >> value;
        return input.eof() && !input.fail() && HasFiniteBits(value);
    }

    bool ParseValue(std::size_t depth, JBeamValue& output)
    {
        const JBeamToken token = Current();
        if (!BeginNode(depth, token.span))
        {
            return false;
        }
        switch (token.kind)
        {
        case JBeamTokenKind::NULL_VALUE:
            ++m_index;
            output.type = JBeamValueType::NULL_VALUE;
            output.span = token.span;
            return true;
        case JBeamTokenKind::TRUE_VALUE:
        case JBeamTokenKind::FALSE_VALUE:
            ++m_index;
            output.type = JBeamValueType::BOOLEAN;
            output.boolean_value =
                token.kind == JBeamTokenKind::TRUE_VALUE;
            output.span = token.span;
            return true;
        case JBeamTokenKind::STRING:
            ++m_index;
            output.type = JBeamValueType::STRING;
            output.scalar_text = token.text;
            output.span = token.span;
            return true;
        case JBeamTokenKind::NUMBER:
            ++m_index;
            output.type = JBeamValueType::NUMBER;
            output.scalar_text = token.text;
            output.span = token.span;
            if (!ParseFiniteDouble(token.text, output.number_value))
            {
                // The lexer already validated the numeric grammar, so a
                // finite-parse failure here is a binary64 range overflow
                // or underflow. Real-world content carries giant inert
                // values (for example decorative part prices), so the
                // literal parses to the C library's saturated value with
                // a warning; every evaluating consumer still refuses
                // non-finite input at its own boundary, and the exact
                // source text remains the identity-bearing value.
                char* parse_end = NULL;
                const double lenient =
                    std::strtod(token.text.c_str(), &parse_end);
                if (parse_end == NULL ||
                    *parse_end != '\0' ||
                    parse_end == token.text.c_str())
                {
                    AddError(
                        JBeamDiagnosticCode::NON_FINITE_NUMBER,
                        token.span,
                        "JBeam number does not have a finite double "
                        "value");
                    return false;
                }
                if (!HasFiniteBits(lenient) &&
                    !AddWarning(
                        JBeamDiagnosticCode::NON_FINITE_NUMBER,
                        token.span,
                        "JBeam number does not have a finite double "
                        "value; evaluating consumers refuse it"))
                {
                    return false;
                }
                output.number_value = lenient;
            }
            return true;
        case JBeamTokenKind::LEFT_BRACE:
            return ParseObject(depth, output);
        case JBeamTokenKind::LEFT_BRACKET:
            return ParseArray(depth, output);
        default:
            AddError(
                JBeamDiagnosticCode::EXPECTED_VALUE,
                token.span,
                "Expected a JBeam value");
            return false;
        }
    }

    bool ParseObject(std::size_t depth, JBeamValue& output)
    {
        Match(JBeamTokenKind::LEFT_BRACE);
        const JBeamToken opening = Previous();
        output.type = JBeamValueType::OBJECT;
        output.span = opening.span;
        bool after_value = false;
        std::map<std::string, JBeamSourceSpan> first_keys;

        while (Current().kind != JBeamTokenKind::RIGHT_BRACE)
        {
            if (Current().kind == JBeamTokenKind::END_OF_INPUT)
            {
                AddError(
                    JBeamDiagnosticCode::EXPECTED_OBJECT_END,
                    Current().span,
                    "Expected '}' before end of JBeam source");
                return false;
            }
            if (Match(JBeamTokenKind::COMMA))
            {
                if (!after_value)
                {
                    AddError(
                        JBeamDiagnosticCode::UNEXPECTED_COMMA,
                        Previous().span,
                        "Unexpected comma in JBeam object");
                    return false;
                }
                // A repeated comma between members is as vacuous as one
                // between array values, and more plainly so: members are
                // named, so there is no position a hole could shift, and
                // `{"a":1,,"b":2}` reads as `{"a":1,"b":2}`. Shipped content
                // relies on it -- the Frix MF1 writes
                // `"beamDamp":500, ,"beamDeform":85000` in
                // f1f_suspension_F.jbeam, and that one comma refused the
                // whole vehicle. A LEADING comma is still refused above: no
                // member precedes it.
                continue;
            }
            if (Current().kind != JBeamTokenKind::STRING &&
                Current().kind != JBeamTokenKind::BARE_KEY)
            {
                AddError(
                    JBeamDiagnosticCode::EXPECTED_OBJECT_KEY,
                    Current().span,
                    "Expected a JBeam object key");
                return false;
            }

            const JBeamToken key = Current();
            ++m_index;
            // A comma between a key and its colon is vacuous: a comma can be
            // neither a key nor a value, so `"k",:v` has exactly one reading,
            // `"k":v`. Shipped content relies on it -- a Nissan GT-R R35 mod
            // writes `"disp_9",:0.49` -- and BeamNG loads that vehicle. The
            // colon must still follow, so `{"x",}` and `{"x","y":1}` remain
            // EXPECTED_COLON: those would need a missing value.
            SkipCommasBeforeColon();
            if (!Match(JBeamTokenKind::COLON))
            {
                AddError(
                    JBeamDiagnosticCode::EXPECTED_COLON,
                    Current().span,
                    "Expected ':' after JBeam object key");
                return false;
            }
            JBeamValue value;
            if (!ParseValue(depth + 1, value))
            {
                return false;
            }
            JBeamObjectField field;
            field.key = key.text;
            field.key_span = key.span;
            field.value =
                std::shared_ptr<const JBeamValue>(new JBeamValue(value));
            output.object_fields.push_back(field);

            const std::map<std::string, JBeamSourceSpan>::const_iterator found =
                first_keys.find(key.text);
            if (found == first_keys.end())
            {
                first_keys.insert(std::make_pair(key.text, key.span));
            }
            else
            {
                if (!AddWarning(
                    JBeamDiagnosticCode::DUPLICATE_OBJECT_KEY,
                    key.span,
                    "Duplicate case-sensitive JBeam object key; the last "
                    "assignment is effective"))
                {
                    return false;
                }
            }
            after_value = true;
        }
        Match(JBeamTokenKind::RIGHT_BRACE);
        output.span.end = Previous().span.end;
        return true;
    }

    // Consumes every comma between an object key and its colon. Silent, like
    // the repeated array comma: the comma carries no reading to report.
    void SkipCommasBeforeColon()
    {
        while (Match(JBeamTokenKind::COMMA))
        {
        }
    }

    bool ParseArray(std::size_t depth, JBeamValue& output)
    {
        Match(JBeamTokenKind::LEFT_BRACKET);
        const JBeamToken opening = Previous();
        output.type = JBeamValueType::ARRAY;
        output.span = opening.span;
        bool after_value = false;

        while (Current().kind != JBeamTokenKind::RIGHT_BRACKET)
        {
            if (Current().kind == JBeamTokenKind::END_OF_INPUT)
            {
                AddError(
                    JBeamDiagnosticCode::EXPECTED_ARRAY_END,
                    Current().span,
                    "Expected ']' before end of JBeam source");
                return false;
            }
            if (Match(JBeamTokenKind::COMMA))
            {
                if (!after_value)
                {
                    AddError(
                        JBeamDiagnosticCode::UNEXPECTED_COMMA,
                        Previous().span,
                        "Unexpected comma in JBeam array");
                    return false;
                }
                // A repeated comma between array values is vacuous in a
                // dialect whose commas are already optional separators: there
                // is nothing between them to be an element, so `[a,,b]` reads
                // as `[a, b]` and not as a hole. Shipped content relies on it
                // -- the F-22A's missile system writes
                // `{"thrustLimit":40000},,` -- and BeamNG loads that vehicle.
                // A LEADING comma is still refused above: it would have to
                // mean an element before the first, which is not vacuous.
                continue;
            }
            JBeamValue value;
            if (!ParseValue(depth + 1, value))
            {
                return false;
            }
            output.array_values.push_back(value);
            after_value = true;
        }
        Match(JBeamTokenKind::RIGHT_BRACKET);
        output.span.end = Previous().span.end;
        return true;
    }

    const std::vector<JBeamToken>& m_tokens;
    const JBeamParseLimits& m_limits;
    std::size_t m_index;
    std::size_t m_node_count;
    JBeamParseResult m_result;
};

bool IsTableHeader(const JBeamValue& value)
{
    if (value.type != JBeamValueType::ARRAY ||
        value.array_values.empty())
    {
        return false;
    }
    for (std::size_t i = 0; i < value.array_values.size(); ++i)
    {
        if (value.array_values[i].type != JBeamValueType::STRING)
        {
            return false;
        }
    }
    return true;
}

std::shared_ptr<const JBeamValue> CopyValue(const JBeamValue& value)
{
    return std::shared_ptr<const JBeamValue>(new JBeamValue(value));
}

bool AddWithoutOverflow(
    std::size_t left,
    std::size_t right,
    std::size_t& output)
{
    if (right > std::numeric_limits<std::size_t>::max() - left)
    {
        return false;
    }
    output = left + right;
    return true;
}

bool MultiplyWithoutOverflow(
    std::size_t left,
    std::size_t right,
    std::size_t& output)
{
    if (left != 0 &&
        right > std::numeric_limits<std::size_t>::max() / left)
    {
        return false;
    }
    output = left * right;
    return true;
}

namespace JBeamNormalizeRetainedV1 {

// Retained-byte limits are a portable admission contract, not a measurement of
// the active standard library's Debug/Release object layout. In particular,
// MSVC iterator debugging enlarges strings and vectors. Using sizeof() here
// made identical JBeam input cross the same byte limit on Windows Debug while
// remaining admissible on the other supported configurations.
//
// These fixed 64-bit logical weights conservatively cover the ordinary release
// layouts used by the supported x64/arm64 platforms. Dynamic string and vector
// payloads are charged separately below.
const std::size_t DIAGNOSTIC_RECORD = 120U;
const std::size_t VALUE_RECORD = 176U;
const std::size_t OBJECT_FIELD_RECORD = 128U;
const std::size_t FIELD_ASSIGNMENT_RECORD = 136U;
const std::size_t INHERITED_FIELD_INDEX_RECORD = 56U;
const std::size_t INDEX_RECORD = 8U;
const std::size_t TABLE_RECORD = 160U;
const std::size_t TABLE_COLUMN_RECORD = 112U;
const std::size_t TABLE_ENTRY_RECORD = 528U;
const std::size_t VECTOR_RECORD = 24U;
const std::size_t INHERITED_ASSIGNMENT_INDEX_RECORD = 24U;
const std::size_t SHARED_ALLOCATION_OVERHEAD = 64U;

} // namespace JBeamNormalizeRetainedV1

bool IsPathSafeByte(unsigned char value)
{
    return (value >= 'a' && value <= 'z') ||
        (value >= 'A' && value <= 'Z') ||
        (value >= '0' && value <= '9') ||
        value == '_' || value == '-' || value == '.';
}

bool EscapedPathKeySize(
    const std::string& key,
    std::size_t& escaped_size)
{
    escaped_size = key.size();
    for (std::size_t i = 0; i < key.size(); ++i)
    {
        if (!IsPathSafeByte(
                static_cast<unsigned char>(key[i])) &&
            !AddWithoutOverflow(escaped_size, 2U, escaped_size))
        {
            return false;
        }
    }
    return true;
}

void AppendEscapedPathKey(
    const std::string& key,
    std::string& output)
{
    static const char HEX_DIGITS[] = "0123456789ABCDEF";
    for (std::size_t i = 0; i < key.size(); ++i)
    {
        const unsigned char value =
            static_cast<unsigned char>(key[i]);
        if (IsPathSafeByte(value))
        {
            output.push_back(static_cast<char>(value));
        }
        else
        {
            output.push_back('%');
            output.push_back(HEX_DIGITS[value >> 4]);
            output.push_back(HEX_DIGITS[value & 0x0fU]);
        }
    }
}

std::size_t DecimalDigitCount(std::size_t value)
{
    std::size_t count = 1U;
    while (value >= 10U)
    {
        value /= 10U;
        ++count;
    }
    return count;
}

void AppendDecimal(std::size_t value, std::string& output)
{
    char digits[
        std::numeric_limits<std::size_t>::digits10 + 2U];
    std::size_t count = 0U;
    do
    {
        digits[count++] =
            static_cast<char>('0' + value % 10U);
        value /= 10U;
    }
    while (value != 0U);
    while (count > 0U)
    {
        output.push_back(digits[--count]);
    }
}

class TableNormalizer
{
public:
    TableNormalizer(
        const JBeamNormalizeLimits& limits,
        JBeamNormalizeResult& result)
        : m_limits(limits)
        , m_result(result)
        , m_stopped(false)
    {
    }

    bool IsStopped() const
    {
        return m_stopped;
    }

    bool ConsumeWork(
        std::size_t units,
        const JBeamSourceSpan& span)
    {
        if (m_stopped)
        {
            return false;
        }
        if (units > m_limits.max_work_units - std::min(
                m_result.work_units, m_limits.max_work_units))
        {
            Stop(
                JBeamDiagnosticCode::NORMALIZE_WORK_LIMIT,
                span,
                "JBeam table normalization exceeded the configured work "
                "budget");
            return false;
        }
        m_result.work_units += units;
        return true;
    }

    bool Retain(
        std::size_t bytes,
        const JBeamSourceSpan& span)
    {
        if (m_stopped)
        {
            return false;
        }
        if (bytes > m_limits.max_retained_bytes - std::min(
                m_result.retained_bytes, m_limits.max_retained_bytes))
        {
            Stop(
                JBeamDiagnosticCode::NORMALIZE_RETAINED_BYTES_LIMIT,
                span,
                "JBeam table normalization exceeded the configured retained "
                "byte budget");
            return false;
        }
        m_result.retained_bytes += bytes;
        return true;
    }

    bool RetainElements(
        std::size_t count,
        std::size_t element_size,
        const JBeamSourceSpan& span)
    {
        std::size_t bytes = 0;
        if (!MultiplyWithoutOverflow(count, element_size, bytes))
        {
            Stop(
                JBeamDiagnosticCode::NORMALIZE_RETAINED_BYTES_LIMIT,
                span,
                "JBeam table normalization retained-byte accounting "
                "overflowed");
            return false;
        }
        return Retain(bytes, span);
    }

    bool RetainString(
        const std::string& text,
        const JBeamSourceSpan& span)
    {
        // Empty and short strings are normally stored inline. Charging their
        // exact logical size remains deterministic and conservative for
        // payload growth without depending on a standard-library SSO width.
        return Retain(text.size(), span);
    }

    bool AddDiagnostic(
        JBeamDiagnosticCode code,
        JBeamDiagnosticSeverity severity,
        const JBeamSourceSpan& span,
        const std::string& message)
    {
        if (m_stopped)
        {
            return false;
        }
        if (m_result.diagnostics.size() >= m_limits.max_diagnostics)
        {
            Stop(
                JBeamDiagnosticCode::DIAGNOSTIC_LIMIT,
                span,
                "JBeam table normalization reached the configured "
                "diagnostic limit");
            return false;
        }
        std::size_t bytes =
            JBeamNormalizeRetainedV1::DIAGNOSTIC_RECORD;
        if (!AddWithoutOverflow(bytes, span.source_name.size(), bytes) ||
            !AddWithoutOverflow(bytes, message.size(), bytes) ||
            !Retain(bytes, span))
        {
            return false;
        }
        m_result.diagnostics.push_back(
            MakeDiagnostic(code, severity, span, message));
        return true;
    }

    bool MeasureValueCopyDynamicBytes(
        const JBeamValue& value,
        std::size_t& bytes)
    {
        bytes = 0;
        std::vector<const JBeamValue*> pending;
        if (!ConsumeWork(1, value.span))
        {
            return false;
        }
        pending.push_back(&value);
        while (!pending.empty())
        {
            const JBeamValue* current = pending.back();
            pending.pop_back();
            if (!AccumulateString(bytes, current->span.source_name) ||
                !AccumulateString(bytes, current->scalar_text))
            {
                Stop(
                    JBeamDiagnosticCode::NORMALIZE_RETAINED_BYTES_LIMIT,
                    current->span,
                    "JBeam copied-value retained-byte accounting "
                    "overflowed");
                return false;
            }
            if (current->type == JBeamValueType::ARRAY)
            {
                std::size_t child_bytes = 0;
                if (!MultiplyWithoutOverflow(
                        current->array_values.size(),
                        JBeamNormalizeRetainedV1::VALUE_RECORD,
                        child_bytes) ||
                    !AddWithoutOverflow(bytes, child_bytes, bytes))
                {
                    Stop(
                        JBeamDiagnosticCode::
                            NORMALIZE_RETAINED_BYTES_LIMIT,
                        current->span,
                        "JBeam copied-array retained-byte accounting "
                        "overflowed");
                    return false;
                }
                if (!ConsumeWork(
                        current->array_values.size(), current->span))
                {
                    return false;
                }
                for (std::size_t i = 0;
                     i < current->array_values.size();
                     ++i)
                {
                    pending.push_back(&current->array_values[i]);
                }
            }
            else if (current->type == JBeamValueType::OBJECT)
            {
                std::size_t field_bytes = 0;
                if (!MultiplyWithoutOverflow(
                        current->object_fields.size(),
                        JBeamNormalizeRetainedV1::OBJECT_FIELD_RECORD,
                        field_bytes) ||
                    !AddWithoutOverflow(bytes, field_bytes, bytes) ||
                    !ConsumeWork(
                        current->object_fields.size(), current->span))
                {
                    if (!m_stopped)
                    {
                        Stop(
                            JBeamDiagnosticCode::
                                NORMALIZE_RETAINED_BYTES_LIMIT,
                            current->span,
                            "JBeam copied-object retained-byte accounting "
                            "overflowed");
                    }
                    return false;
                }
                for (std::size_t i = 0;
                     i < current->object_fields.size();
                     ++i)
                {
                    const JBeamObjectField& field =
                        current->object_fields[i];
                    if (!AccumulateString(bytes, field.key) ||
                        !AccumulateString(
                            bytes, field.key_span.source_name))
                    {
                        Stop(
                            JBeamDiagnosticCode::
                                NORMALIZE_RETAINED_BYTES_LIMIT,
                            field.key_span,
                            "JBeam copied-field retained-byte accounting "
                            "overflowed");
                        return false;
                    }
                }
            }
        }
        return true;
    }

private:
    bool AccumulateString(
        std::size_t& bytes,
        const std::string& text)
    {
        return AddWithoutOverflow(bytes, text.size(), bytes);
    }

    void Stop(
        JBeamDiagnosticCode code,
        const JBeamSourceSpan& span,
        const std::string& message)
    {
        if (m_stopped)
        {
            return;
        }
        m_stopped = true;
        m_result.diagnostics.push_back(MakeDiagnostic(
            code,
            JBeamDiagnosticSeverity::ERROR_SEVERITY,
            span,
            message));
    }

    const JBeamNormalizeLimits& m_limits;
    JBeamNormalizeResult& m_result;
    bool m_stopped;
};

bool ChargeTreeLookupWork(
    std::size_t element_count,
    const JBeamSourceSpan& span,
    TableNormalizer& normalizer)
{
    std::size_t units = 1;
    while (element_count > 1)
    {
        ++units;
        element_count /= 2;
    }
    return normalizer.ConsumeWork(units, span);
}

bool ChargeAssignment(
    const JBeamObjectField& field,
    TableNormalizer& normalizer)
{
    return normalizer.RetainString(field.key, field.key_span) &&
           normalizer.RetainString(
               field.value->span.source_name, field.value->span);
}

bool AppendObjectAssignments(
    const JBeamValue& object,
    JBeamFieldOrigin origin,
    std::vector<JBeamFieldAssignment>& output,
    std::shared_ptr<JBeamInheritedAssignmentIndex> inherited_index,
    std::map<std::string, std::size_t>* inherited_buckets,
    TableNormalizer& normalizer)
{
    if (!normalizer.ConsumeWork(
            object.object_fields.size(), object.span))
    {
        return false;
    }
    for (std::size_t i = 0; i < object.object_fields.size(); ++i)
    {
        const JBeamObjectField& field = object.object_fields[i];
        if (!field.value)
        {
            if (!normalizer.AddDiagnostic(
                    JBeamDiagnosticCode::TABLE_INVALID_ENTRY,
                    JBeamDiagnosticSeverity::WARNING,
                    field.key_span,
                    "Null JBeam object field value is ignored during table "
                    "normalization"))
            {
                return false;
            }
            continue;
        }
        if (!ChargeAssignment(field, normalizer))
        {
            return false;
        }
        JBeamFieldAssignment assignment;
        assignment.name = field.key;
        assignment.origin = origin;
        assignment.span = field.value->span;
        assignment.value = field.value;
        const std::size_t assignment_index = output.size();
        output.push_back(std::move(assignment));

        if (inherited_index && inherited_buckets != NULL)
        {
            if (!ChargeTreeLookupWork(
                    inherited_buckets->size(),
                    field.key_span,
                    normalizer))
            {
                return false;
            }
            std::map<std::string, std::size_t>::iterator found =
                inherited_buckets->find(field.key);
            std::size_t bucket_index = 0;
            if (found == inherited_buckets->end())
            {
                bucket_index = inherited_index->fields.size();
                if (!normalizer.RetainElements(
                        1,
                        JBeamNormalizeRetainedV1::
                            INHERITED_FIELD_INDEX_RECORD,
                        field.key_span) ||
                    !normalizer.RetainString(field.key, field.key_span))
                {
                    return false;
                }
                JBeamInheritedFieldIndex bucket;
                bucket.name = field.key;
                inherited_index->fields.push_back(std::move(bucket));
                inherited_buckets->insert(
                    std::make_pair(field.key, bucket_index));
            }
            else
            {
                bucket_index = found->second;
            }
            if (!normalizer.RetainElements(
                    1,
                    JBeamNormalizeRetainedV1::INDEX_RECORD,
                    field.key_span))
            {
                return false;
            }
            inherited_index->fields[bucket_index]
                .assignment_indices.push_back(assignment_index);
        }
    }
    return true;
}

bool NormalizeTable(
    const JBeamValue& value,
    const std::string& path,
    JBeamNormalizeResult& result,
    TableNormalizer& normalizer)
{
    if (!normalizer.ConsumeWork(1, value.span) ||
        !normalizer.RetainElements(
            1,
            JBeamNormalizeRetainedV1::TABLE_RECORD,
            value.span) ||
        !normalizer.RetainString(path, value.span) ||
        !normalizer.RetainString(
            value.span.source_name, value.span))
    {
        return false;
    }

    JBeamNormalizedTable table;
    table.path = path;
    table.span = value.span;
    const JBeamValue& header = value.array_values[0];
    if (!normalizer.RetainElements(
            header.array_values.size(),
            JBeamNormalizeRetainedV1::TABLE_COLUMN_RECORD,
            header.span))
    {
        return false;
    }
    table.columns.reserve(header.array_values.size());
    std::map<std::string, std::size_t> header_names;
    for (std::size_t i = 0; i < header.array_values.size(); ++i)
    {
        if (!normalizer.ConsumeWork(1, header.array_values[i].span) ||
            !normalizer.RetainString(
                header.array_values[i].scalar_text,
                header.array_values[i].span) ||
            !normalizer.RetainString(
                header.array_values[i].span.source_name,
                header.array_values[i].span))
        {
            return false;
        }
        JBeamTableColumn column;
        column.name = header.array_values[i].scalar_text;
        column.span = header.array_values[i].span;
        table.columns.push_back(std::move(column));
        if (!ChargeTreeLookupWork(
                header_names.size(),
                table.columns.back().span,
                normalizer))
        {
            return false;
        }
        if (header_names.find(table.columns.back().name) !=
            header_names.end())
        {
            if (!normalizer.AddDiagnostic(
                    JBeamDiagnosticCode::DUPLICATE_TABLE_HEADER,
                    JBeamDiagnosticSeverity::WARNING,
                    table.columns.back().span,
                    "Duplicate case-sensitive JBeam table header is "
                    "preserved"))
            {
                return false;
            }
        }
        else
        {
            header_names.insert(
                std::make_pair(table.columns.back().name, i));
        }
    }

    const std::size_t entry_count = value.array_values.size() - 1;
    std::size_t default_assignment_capacity = 0;
    if (!normalizer.ConsumeWork(entry_count, value.span))
    {
        return false;
    }
    for (std::size_t entry_index = 1;
         entry_index < value.array_values.size();
         ++entry_index)
    {
        const JBeamValue& entry_value = value.array_values[entry_index];
        if (entry_value.type == JBeamValueType::OBJECT)
        {
            if (!AddWithoutOverflow(
                    default_assignment_capacity,
                    entry_value.object_fields.size(),
                    default_assignment_capacity))
            {
                normalizer.RetainElements(
                    std::numeric_limits<std::size_t>::max(),
                    2U,
                    entry_value.span);
                return false;
            }
        }
    }
    if (!normalizer.RetainElements(
            entry_count,
            JBeamNormalizeRetainedV1::TABLE_ENTRY_RECORD,
            value.span) ||
        !normalizer.RetainElements(
            default_assignment_capacity,
            JBeamNormalizeRetainedV1::FIELD_ASSIGNMENT_RECORD,
            value.span) ||
        !normalizer.RetainElements(
            1,
            JBeamNormalizeRetainedV1::VECTOR_RECORD +
                JBeamNormalizeRetainedV1::SHARED_ALLOCATION_OVERHEAD,
            value.span) ||
        !normalizer.RetainElements(
            1,
            JBeamNormalizeRetainedV1::
                    INHERITED_ASSIGNMENT_INDEX_RECORD +
                JBeamNormalizeRetainedV1::SHARED_ALLOCATION_OVERHEAD,
            value.span))
    {
        return false;
    }
    table.entries.reserve(entry_count);
    std::shared_ptr<std::vector<JBeamFieldAssignment> > active_defaults(
        new std::vector<JBeamFieldAssignment>());
    active_defaults->reserve(default_assignment_capacity);
    std::shared_ptr<JBeamInheritedAssignmentIndex> inherited_index(
        new JBeamInheritedAssignmentIndex());
    std::map<std::string, std::size_t> inherited_buckets;

    for (std::size_t entry_index = 1;
         entry_index < value.array_values.size();
         ++entry_index)
    {
        if (!normalizer.ConsumeWork(
                1, value.array_values[entry_index].span))
        {
            return false;
        }
        const JBeamValue& entry_value = value.array_values[entry_index];
        JBeamNormalizedTableEntry entry;
        std::size_t raw_value_bytes = 0;
        if (!normalizer.MeasureValueCopyDynamicBytes(
                entry_value, raw_value_bytes) ||
            !normalizer.Retain(raw_value_bytes, entry_value.span))
        {
            return false;
        }
        entry.raw_value = entry_value;
        if (entry_value.type == JBeamValueType::OBJECT)
        {
            entry.kind = JBeamNormalizedTableEntryKind::DEFAULT_MODIFIER;
            if (!AppendObjectAssignments(
                    entry_value,
                    JBeamFieldOrigin::INHERITED_DEFAULT,
                    *active_defaults,
                    inherited_index,
                    &inherited_buckets,
                    normalizer))
            {
                return false;
            }
        }
        else if (entry_value.type == JBeamValueType::ARRAY)
        {
            entry.kind = JBeamNormalizedTableEntryKind::DATA_ROW;
            if (!normalizer.RetainString(
                    entry_value.span.source_name, entry_value.span) ||
                !normalizer.Retain(raw_value_bytes, entry_value.span))
            {
                return false;
            }
            entry.data_row.span = entry_value.span;
            entry.data_row.raw_row = entry_value;
            entry.data_row.inherited_assignment_storage = active_defaults;
            entry.data_row.inherited_assignment_count =
                active_defaults->size();
            entry.data_row.inherited_assignment_index = inherited_index;

            std::size_t positional_count = entry_value.array_values.size();
            if (positional_count > table.columns.size() &&
                entry_value.array_values[positional_count - 1].type ==
                    JBeamValueType::OBJECT)
            {
                const JBeamValue& overrides =
                    entry_value.array_values[positional_count - 1];
                if (!normalizer.RetainElements(
                        overrides.object_fields.size(),
                        JBeamNormalizeRetainedV1::
                            FIELD_ASSIGNMENT_RECORD,
                        overrides.span))
                {
                    return false;
                }
                entry.data_row.row_local_assignments.reserve(
                    overrides.object_fields.size());
                if (!AppendObjectAssignments(
                        overrides,
                        JBeamFieldOrigin::ROW_LOCAL_OVERRIDE,
                        entry.data_row.row_local_assignments,
                        std::shared_ptr<JBeamInheritedAssignmentIndex>(),
                        NULL,
                        normalizer))
                {
                    return false;
                }
                --positional_count;
            }

            const std::size_t mapped_count =
                std::min(positional_count, table.columns.size());
            if (!normalizer.RetainElements(
                    mapped_count,
                    JBeamNormalizeRetainedV1::
                        FIELD_ASSIGNMENT_RECORD,
                    entry_value.span))
            {
                return false;
            }
            entry.data_row.positional_assignments.reserve(mapped_count);
            for (std::size_t column_index = 0;
                 column_index < mapped_count;
                 ++column_index)
            {
                if (!normalizer.ConsumeWork(
                        1,
                        entry_value.array_values[column_index].span) ||
                    !normalizer.RetainString(
                        table.columns[column_index].name,
                        table.columns[column_index].span) ||
                    !normalizer.RetainString(
                        entry_value.array_values[column_index]
                            .span.source_name,
                        entry_value.array_values[column_index].span))
                {
                    return false;
                }
                std::size_t copied_cell_bytes = 0;
                if (!normalizer.MeasureValueCopyDynamicBytes(
                        entry_value.array_values[column_index],
                        copied_cell_bytes) ||
                    !normalizer.RetainElements(
                        1,
                        JBeamNormalizeRetainedV1::VALUE_RECORD +
                            JBeamNormalizeRetainedV1::
                                SHARED_ALLOCATION_OVERHEAD,
                        entry_value.array_values[column_index].span) ||
                    !normalizer.Retain(
                        copied_cell_bytes,
                        entry_value.array_values[column_index].span))
                {
                    return false;
                }
                JBeamFieldAssignment assignment;
                assignment.name = table.columns[column_index].name;
                assignment.origin = JBeamFieldOrigin::POSITIONAL_CELL;
                assignment.span =
                    entry_value.array_values[column_index].span;
                assignment.value =
                    CopyValue(entry_value.array_values[column_index]);
                entry.data_row.positional_assignments.push_back(
                    std::move(assignment));
            }
            if (positional_count < table.columns.size())
            {
                if (!normalizer.AddDiagnostic(
                        JBeamDiagnosticCode::TABLE_ROW_TOO_SHORT,
                        JBeamDiagnosticSeverity::WARNING,
                        entry_value.span,
                        "JBeam table row has fewer positional cells than "
                        "its header; the raw row is preserved"))
                {
                    return false;
                }
            }
            else if (positional_count > table.columns.size())
            {
                if (!normalizer.AddDiagnostic(
                        JBeamDiagnosticCode::TABLE_ROW_TOO_LONG,
                        JBeamDiagnosticSeverity::WARNING,
                        entry_value.span,
                        "JBeam table row has more positional cells than its "
                        "header; extra raw cells are preserved"))
                {
                    return false;
                }
            }
        }
        else
        {
            entry.kind = JBeamNormalizedTableEntryKind::INVALID_ENTRY;
            if (!normalizer.AddDiagnostic(
                    JBeamDiagnosticCode::TABLE_INVALID_ENTRY,
                    JBeamDiagnosticSeverity::WARNING,
                    entry_value.span,
                    "JBeam table entry is neither a default dictionary nor "
                    "a data-row array; the raw value is preserved"))
            {
                return false;
            }
        }
        table.entries.push_back(std::move(entry));
    }

    for (std::size_t width = inherited_index->fields.size();
         width > 1;
         width = width / 2 + width % 2)
    {
        if (!normalizer.ConsumeWork(
                inherited_index->fields.size(), value.span))
        {
            return false;
        }
    }
    std::sort(
        inherited_index->fields.begin(),
        inherited_index->fields.end(),
        [](const JBeamInheritedFieldIndex& left,
           const JBeamInheritedFieldIndex& right)
        {
            return left.name < right.name;
        });
    result.tables.push_back(std::move(table));
    return true;
}

void DiscoverTables(
    const JBeamValue& value,
    std::string& path,
    JBeamNormalizeResult& result,
    TableNormalizer& normalizer)
{
    if (!normalizer.ConsumeWork(1, value.span))
    {
        return;
    }
    if (value.type == JBeamValueType::ARRAY)
    {
        if (!value.array_values.empty())
        {
            const JBeamValue& candidate_header =
                value.array_values[0];
            if (!normalizer.ConsumeWork(
                    candidate_header.array_values.size(),
                    candidate_header.span))
            {
                return;
            }
            if (IsTableHeader(candidate_header))
            {
                NormalizeTable(value, path, result, normalizer);
            }
        }
        // JBeam section arrays contain rows and arbitrary cell payloads.
        // They are never namespaces for more independently normalized
        // sections.
        return;
    }
    if (value.type != JBeamValueType::OBJECT)
    {
        return;
    }

    std::map<std::string, std::size_t> occurrences;
    for (std::size_t i = 0; i < value.object_fields.size(); ++i)
    {
        if (!normalizer.ConsumeWork(1, value.object_fields[i].key_span))
        {
            return;
        }
        const JBeamObjectField& field = value.object_fields[i];
        // Charge source bytes before the occurrence map can compare or copy
        // the key. This ordering also covers null fields, which still consume
        // an occurrence so later duplicate paths remain deterministic.
        if (!normalizer.ConsumeWork(
                field.key.size(), field.key_span))
        {
            return;
        }
        if (!ChargeTreeLookupWork(
                occurrences.size(), field.key_span, normalizer))
        {
            return;
        }
        const std::size_t occurrence = occurrences[field.key]++;
        if (!field.value)
        {
            if (!normalizer.AddDiagnostic(
                    JBeamDiagnosticCode::TABLE_INVALID_ENTRY,
                    JBeamDiagnosticSeverity::WARNING,
                    field.key_span,
                    "Null JBeam object field value is ignored during table "
                    "discovery"))
            {
                return;
            }
            continue;
        }

        // Charge every source byte before scanning/escaping it. A mutable
        // append/resize path keeps traversal linear in the total segment
        // bytes rather than copying the complete ancestor path at every edge.
        std::size_t escaped_size = 0U;
        if (!EscapedPathKeySize(field.key, escaped_size))
        {
            normalizer.ConsumeWork(
                std::numeric_limits<std::size_t>::max(),
                field.key_span);
            return;
        }
        const std::size_t occurrence_digits =
            DecimalDigitCount(occurrence);
        std::size_t appended_size = 2U;
        if (!AddWithoutOverflow(
                appended_size,
                escaped_size,
                appended_size) ||
            !AddWithoutOverflow(
                appended_size,
                occurrence_digits,
                appended_size))
        {
            normalizer.ConsumeWork(
                std::numeric_limits<std::size_t>::max(),
                field.key_span);
            return;
        }
        const std::size_t escape_expansion =
            escaped_size - field.key.size();
        if (!normalizer.ConsumeWork(
                escape_expansion + occurrence_digits + 2U,
                field.key_span))
        {
            return;
        }

        const std::size_t parent_path_size = path.size();
        if (appended_size >
            path.max_size() -
                std::min(path.size(), path.max_size()))
        {
            normalizer.ConsumeWork(
                std::numeric_limits<std::size_t>::max(),
                field.key_span);
            return;
        }
        path.reserve(parent_path_size + appended_size);
        path.push_back('/');
        AppendEscapedPathKey(field.key, path);
        path.push_back('#');
        AppendDecimal(occurrence, path);
        DiscoverTables(
            *field.value, path, result, normalizer);
        path.resize(parent_path_size);
        if (normalizer.IsStopped())
        {
            return;
        }
    }
}

void ChargeLookupWork(JBeamFieldLookupMetrics* metrics)
{
    if (metrics != NULL &&
        metrics->work_units < std::numeric_limits<std::size_t>::max())
    {
        ++metrics->work_units;
    }
}

const JBeamFieldAssignment* FindLastAssignment(
    const std::vector<JBeamFieldAssignment>& assignments,
    const std::string& name,
    JBeamFieldLookupMetrics* metrics)
{
    for (std::size_t i = assignments.size(); i > 0; --i)
    {
        ChargeLookupWork(metrics);
        if (assignments[i - 1].name == name)
        {
            return &assignments[i - 1];
        }
    }
    return NULL;
}

const JBeamFieldAssignment* FindLastAssignmentPrefix(
    const std::shared_ptr<const std::vector<JBeamFieldAssignment> >&
        assignments,
    std::size_t count,
    const std::string& name,
    JBeamFieldLookupMetrics* metrics)
{
    if (!assignments)
    {
        return NULL;
    }
    const std::size_t bounded_count = std::min(count, assignments->size());
    for (std::size_t i = bounded_count; i > 0; --i)
    {
        ChargeLookupWork(metrics);
        if ((*assignments)[i - 1].name == name)
        {
            return &(*assignments)[i - 1];
        }
    }
    return NULL;
}

const JBeamFieldAssignment* FindIndexedAssignmentPrefix(
    const JBeamNormalizedDataRow& row,
    const std::string& name,
    JBeamFieldLookupMetrics* metrics)
{
    if (!row.inherited_assignment_storage ||
        !row.inherited_assignment_index)
    {
        return FindLastAssignmentPrefix(
            row.inherited_assignment_storage,
            row.inherited_assignment_count,
            name,
            metrics);
    }

    const std::vector<JBeamInheritedFieldIndex>& fields =
        row.inherited_assignment_index->fields;
    std::size_t low = 0;
    std::size_t high = fields.size();
    while (low < high)
    {
        ChargeLookupWork(metrics);
        const std::size_t middle = low + (high - low) / 2;
        if (fields[middle].name < name)
        {
            low = middle + 1;
        }
        else
        {
            high = middle;
        }
    }
    if (low >= fields.size())
    {
        return NULL;
    }
    ChargeLookupWork(metrics);
    if (fields[low].name != name)
    {
        return NULL;
    }

    const std::vector<std::size_t>& indexes =
        fields[low].assignment_indices;
    const std::size_t bounded_count = std::min(
        row.inherited_assignment_count,
        row.inherited_assignment_storage->size());
    std::size_t index_low = 0;
    std::size_t index_high = indexes.size();
    while (index_low < index_high)
    {
        ChargeLookupWork(metrics);
        const std::size_t middle =
            index_low + (index_high - index_low) / 2;
        if (indexes[middle] < bounded_count)
        {
            index_low = middle + 1;
        }
        else
        {
            index_high = middle;
        }
    }
    if (index_low == 0)
    {
        return NULL;
    }
    const std::size_t assignment_index = indexes[index_low - 1];
    if (assignment_index >= bounded_count)
    {
        return NULL;
    }
    return &(*row.inherited_assignment_storage)[assignment_index];
}

} // namespace

JBeamSourcePosition::JBeamSourcePosition()
    : byte_offset(0)
    , line(1)
    , column(1)
{
}

JBeamSourceSpan::JBeamSourceSpan()
{
}

JBeamDiagnostic::JBeamDiagnostic()
    : code(JBeamDiagnosticCode::INVALID_CHARACTER)
    , severity(JBeamDiagnosticSeverity::ERROR_SEVERITY)
{
}

JBeamParseLimits::JBeamParseLimits()
    : max_source_bytes(16U * 1024U * 1024U)
    , max_tokens(1000000U)
    , max_nodes(500000U)
    , max_depth(128U)
    , max_string_bytes(4U * 1024U * 1024U)
    , max_diagnostics(4096U)
{
}

JBeamNormalizeLimits::JBeamNormalizeLimits()
    : max_work_units(12000000U)
    , max_retained_bytes(512U * 1024U * 1024U)
    , max_diagnostics(4096U)
{
}

JBeamFieldLookupMetrics::JBeamFieldLookupMetrics()
    : work_units(0)
{
}

JBeamToken::JBeamToken()
    : kind(JBeamTokenKind::INVALID)
{
}

bool JBeamLexResult::IsValid() const
{
    return !HasErrors(diagnostics);
}

JBeamObjectField::JBeamObjectField()
{
}

JBeamValue::JBeamValue()
    : type(JBeamValueType::NULL_VALUE)
    , boolean_value(false)
    , number_value(0.0)
{
}

JBeamParseResult::JBeamParseResult()
    : has_root(false)
{
}

bool JBeamParseResult::IsValid() const
{
    return has_root && !HasErrors(diagnostics);
}

JBeamLexResult LexJBeam(
    const std::string& source,
    const std::string& source_name,
    const JBeamParseLimits& limits)
{
    return Lexer(source, source_name, limits).Run();
}

JBeamParseResult ParseJBeam(
    const std::string& source,
    const std::string& source_name,
    const JBeamParseLimits& limits)
{
    const JBeamLexResult lexed = LexJBeam(source, source_name, limits);
    JBeamParseResult result =
        Parser(lexed.tokens, lexed.diagnostics, limits).Run();
    SortDiagnostics(result.diagnostics);
    return result;
}

const JBeamObjectField* FindLastJBeamObjectField(
    const JBeamValue& object,
    const std::string& key)
{
    if (object.type != JBeamValueType::OBJECT)
    {
        return NULL;
    }
    for (std::size_t i = object.object_fields.size(); i > 0; --i)
    {
        if (object.object_fields[i - 1].key == key)
        {
            return &object.object_fields[i - 1];
        }
    }
    return NULL;
}

bool JBeamNormalizeResult::IsValid() const
{
    return !HasErrors(diagnostics);
}

JBeamNormalizeResult NormalizeJBeamTables(
    const JBeamValue& root,
    const JBeamNormalizeLimits& limits)
{
    JBeamNormalizeResult result;
    TableNormalizer normalizer(limits, result);
    std::string path("$");
    DiscoverTables(root, path, result, normalizer);
    if (normalizer.IsStopped())
    {
        result.tables.clear();
    }
    SortDiagnostics(result.diagnostics);
    return result;
}

const JBeamFieldAssignment* FindEffectiveJBeamField(
    const JBeamNormalizedDataRow& row,
    const std::string& name,
    JBeamFieldLookupMetrics* metrics)
{
    const JBeamFieldAssignment* assignment =
        FindLastAssignment(row.row_local_assignments, name, metrics);
    if (assignment != NULL)
    {
        return assignment;
    }
    assignment =
        FindLastAssignment(row.positional_assignments, name, metrics);
    if (assignment != NULL)
    {
        return assignment;
    }
    return FindIndexedAssignmentPrefix(row, name, metrics);
}

const char* JBeamDiagnosticCodeToString(JBeamDiagnosticCode code)
{
    switch (code)
    {
    case JBeamDiagnosticCode::SOURCE_SIZE_LIMIT:
        return "source-size-limit";
    case JBeamDiagnosticCode::TOKEN_LIMIT:
        return "token-limit";
    case JBeamDiagnosticCode::NODE_LIMIT:
        return "node-limit";
    case JBeamDiagnosticCode::DEPTH_LIMIT:
        return "depth-limit";
    case JBeamDiagnosticCode::STRING_SIZE_LIMIT:
        return "string-size-limit";
    case JBeamDiagnosticCode::DIAGNOSTIC_LIMIT:
        return "diagnostic-limit";
    case JBeamDiagnosticCode::INVALID_CHARACTER:
        return "invalid-character";
    case JBeamDiagnosticCode::UNTERMINATED_BLOCK_COMMENT:
        return "unterminated-block-comment";
    case JBeamDiagnosticCode::UNTERMINATED_STRING:
        return "unterminated-string";
    case JBeamDiagnosticCode::INVALID_STRING_CHARACTER:
        return "invalid-string-character";
    case JBeamDiagnosticCode::INVALID_UTF8:
        return "invalid-utf8";
    case JBeamDiagnosticCode::INVALID_ESCAPE:
        return "invalid-escape";
    case JBeamDiagnosticCode::INVALID_UNICODE_ESCAPE:
        return "invalid-unicode-escape";
    case JBeamDiagnosticCode::INVALID_UNICODE_SURROGATE:
        return "invalid-unicode-surrogate";
    case JBeamDiagnosticCode::INVALID_NUMBER:
        return "invalid-number";
    case JBeamDiagnosticCode::NON_FINITE_NUMBER:
        return "non-finite-number";
    case JBeamDiagnosticCode::EXPECTED_VALUE:
        return "expected-value";
    case JBeamDiagnosticCode::EXPECTED_OBJECT_KEY:
        return "expected-object-key";
    case JBeamDiagnosticCode::EXPECTED_COLON:
        return "expected-colon";
    case JBeamDiagnosticCode::EXPECTED_OBJECT_END:
        return "expected-object-end";
    case JBeamDiagnosticCode::EXPECTED_ARRAY_END:
        return "expected-array-end";
    case JBeamDiagnosticCode::UNEXPECTED_COMMA:
        return "unexpected-comma";
    case JBeamDiagnosticCode::TRAILING_CONTENT:
        return "trailing-content";
    case JBeamDiagnosticCode::CONCATENATED_ROOT_OBJECT:
        return "concatenated-root-object";
    case JBeamDiagnosticCode::DUPLICATE_OBJECT_KEY:
        return "duplicate-object-key";
    case JBeamDiagnosticCode::DUPLICATE_TABLE_HEADER:
        return "duplicate-table-header";
    case JBeamDiagnosticCode::TABLE_ROW_TOO_SHORT:
        return "table-row-too-short";
    case JBeamDiagnosticCode::TABLE_ROW_TOO_LONG:
        return "table-row-too-long";
    case JBeamDiagnosticCode::TABLE_INVALID_ENTRY:
        return "table-invalid-entry";
    case JBeamDiagnosticCode::NORMALIZE_WORK_LIMIT:
        return "normalize-work-limit";
    case JBeamDiagnosticCode::NORMALIZE_RETAINED_BYTES_LIMIT:
        return "normalize-retained-bytes-limit";
    case JBeamDiagnosticCode::SURPLUS_ROOT_CLOSER:
        return "surplus-root-closer";
    }
    return "unknown";
}

} // namespace BeamNG
} // namespace RoR
