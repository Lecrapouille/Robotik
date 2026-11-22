// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Syntax.hpp"

#include "Colors.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <string>
#include <vector>

namespace
{

enum class Tone
{
    Plain,
    Comment,
    Keyword,
    Key,
    String,
    Number,
    Variable,
    Punct,
};

struct Span
{
    std::size_t begin = 0;
    std::size_t end = 0;
    Tone tone = Tone::Plain;
};

/// Appends a slice, merging it with the previous one of the same tone.
void pushSpan(std::vector<Span>& p_spans,
              std::size_t p_begin,
              std::size_t p_end,
              Tone p_tone)
{
    if (p_begin >= p_end)
    {
        return;
    }
    if (!p_spans.empty() && p_spans.back().tone == p_tone &&
        p_spans.back().end == p_begin)
    {
        p_spans.back().end = p_end;
        return;
    }
    p_spans.emplace_back(p_begin, p_end, p_tone);
}

/// Colour of a tone, or null for the default text colour.
ImVec4 const* toneColor(Tone p_tone)
{
    // Keys and PDDL sections use BLUE, numbers ORANGE, comments GREY.
    // The other tones belong to syntax only.
    static ImVec4 const keyword(0.78F, 0.56F, 0.96F, 1.0F);
    static ImVec4 const string_color(0.86F, 0.75F, 0.52F, 1.0F);
    static ImVec4 const variable(0.45F, 0.82F, 0.78F, 1.0F);
    static ImVec4 const punct(0.58F, 0.62F, 0.68F, 1.0F);
    switch (p_tone)
    {
        case Tone::Plain:
            return nullptr;
        case Tone::Comment:
            return &GREY;
        case Tone::Keyword:
            return &keyword;
        case Tone::Key:
            return &BLUE;
        case Tone::String:
            return &string_color;
        case Tone::Number:
            return &ORANGE;
        case Tone::Variable:
            return &variable;
        case Tone::Punct:
            return &punct;
    }
    return nullptr;
}

/// Draws one line as coloured slices, on the line already started.
void drawSpans(std::string_view p_line, std::vector<Span> const& p_spans)
{
    if (p_spans.empty())
    {
        ImGui::TextUnformatted(p_line.data(), p_line.data() + p_line.size());
        return;
    }
    for (std::size_t i = 0; i < p_spans.size(); ++i)
    {
        if (i != 0)
        {
            ImGui::SameLine(0.0F, 0.0F);
        }
        Span const& span = p_spans[i];
        ImVec4 const* color = toneColor(span.tone);
        if (color != nullptr)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, *color);
        }
        ImGui::TextUnformatted(p_line.data() + span.begin,
                               p_line.data() + span.end);
        if (color != nullptr)
        {
            ImGui::PopStyleColor();
        }
    }
}

/// True when @p_word equals @p_lower, ignoring ASCII case.
bool sameWord(std::string_view p_word, std::string_view p_lower)
{
    if (p_word.size() != p_lower.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < p_word.size(); ++i)
    {
        unsigned char const c = static_cast<unsigned char>(p_word[i]);
        int const lower = static_cast<unsigned char>(p_lower[i]);
        if (std::tolower(c) != lower)
        {
            return false;
        }
    }
    return true;
}

/// True when @p_word is one of @p_words (already lower case).
template <std::size_t N>
bool oneOf(std::string_view p_word, std::string_view const (&p_words)[N])
{
    for (std::string_view word : p_words)
    {
        if (sameWord(p_word, word))
        {
            return true;
        }
    }
    return false;
}

/// Integer, float, hex, or an infinity / nan token.
bool isNumber(std::string_view p_token)
{
    if (p_token.empty())
    {
        return false;
    }
    std::string_view body = p_token;
    if (body.front() == '+' || body.front() == '-')
    {
        body.remove_prefix(1);
    }
    if (body.empty())
    {
        return false;
    }
    if (sameWord(body, ".inf") || sameWord(body, ".nan") ||
        sameWord(body, "inf") || sameWord(body, "nan"))
    {
        return true;
    }
    if (body.size() > 2 && body[0] == '0' && (body[1] == 'x' || body[1] == 'X'))
    {
        for (std::size_t i = 2; i < body.size(); ++i)
        {
            if (std::isxdigit(static_cast<unsigned char>(body[i])) == 0)
            {
                return false;
            }
        }
        return true;
    }
    std::size_t i = 0;
    bool digit = false;
    if (body[i] == '.')
    {
        ++i;
        if (i >= body.size() ||
            std::isdigit(static_cast<unsigned char>(body[i])) == 0)
        {
            return false;
        }
    }
    while (i < body.size() &&
           std::isdigit(static_cast<unsigned char>(body[i])) != 0)
    {
        digit = true;
        ++i;
    }
    if (i < body.size() && body[i] == '.')
    {
        ++i;
        while (i < body.size() &&
               std::isdigit(static_cast<unsigned char>(body[i])) != 0)
        {
            digit = true;
            ++i;
        }
    }
    if (!digit)
    {
        return false;
    }
    if (i < body.size() && (body[i] == 'e' || body[i] == 'E'))
    {
        ++i;
        if (i < body.size() && (body[i] == '+' || body[i] == '-'))
        {
            ++i;
        }
        if (i >= body.size() ||
            std::isdigit(static_cast<unsigned char>(body[i])) == 0)
        {
            return false;
        }
        while (i < body.size() &&
               std::isdigit(static_cast<unsigned char>(body[i])) != 0)
        {
            ++i;
        }
    }
    return i == body.size();
}

/// Language of a text tab, from the file extension.
SyntaxPainter::Language languageOf(std::filesystem::path const& p_path)
{
    std::string ext = p_path.extension().string();
    for (char& c : ext)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    if (ext == ".pddl")
    {
        return SyntaxPainter::Language::Pddl;
    }
    if (ext == ".yaml" || ext == ".yml")
    {
        return SyntaxPainter::Language::Yaml;
    }
    return SyntaxPainter::Language::Plain;
}

/// True when the line is empty or only spaces and tabs.
bool isBlank(std::string_view p_line)
{
    for (char c : p_line)
    {
        if (c != ' ' && c != '\t')
        {
            return false;
        }
    }
    return true;
}

/// Columns of indentation. A tab counts as two spaces.
int leadingIndent(std::string_view p_line)
{
    int indent = 0;
    for (char c : p_line)
    {
        if (c == ' ')
        {
            ++indent;
        }
        else if (c == '\t')
        {
            indent += 2;
        }
        else
        {
            break;
        }
    }
    return indent;
}

/// A colon that ends a YAML key: followed by a space, the end, or a flow mark.
bool isKeyColon(std::string_view p_line, std::size_t p_at)
{
    if (p_at + 1 >= p_line.size())
    {
        return true;
    }
    char const next = p_line[p_at + 1];
    return next == ' ' || next == '\t' || next == '#' || next == ',' ||
           next == '}' || next == ']' || next == '{' || next == '[';
}

/// True when a YAML word stops at @p_at.
bool isYamlBreak(std::string_view p_line, std::size_t p_at)
{
    char const c = p_line[p_at];
    if (c == ' ' || c == '\t' || c == '#' || c == ',' || c == '[' || c == ']' ||
        c == '{' || c == '}' || c == '"' || c == '\'' || c == '(' || c == ')' ||
        c == '!' || c == '&' || c == '*' || c == '|' || c == '>' || c == '<' ||
        c == '=')
    {
        return true;
    }
    return c == ':' && isKeyColon(p_line, p_at);
}

/// Reads a block-scalar mark (`|`, `|-`, `|+2`, `>`). @p_end is one past it.
bool readBlockIndicator(std::string_view p_line,
                        std::size_t p_at,
                        std::size_t& p_end)
{
    if (p_at >= p_line.size() || (p_line[p_at] != '|' && p_line[p_at] != '>'))
    {
        return false;
    }
    std::size_t i = p_at + 1;
    bool chomp = false;
    bool digit = false;
    while (i < p_line.size())
    {
        char const c = p_line[i];
        if (!chomp && (c == '+' || c == '-'))
        {
            chomp = true;
            ++i;
            continue;
        }
        if (!digit && std::isdigit(static_cast<unsigned char>(c)) != 0)
        {
            digit = true;
            ++i;
            continue;
        }
        break;
    }
    if (i < p_line.size() && p_line[i] != ' ' && p_line[i] != '\t' &&
        p_line[i] != '#')
    {
        return false;
    }
    p_end = i;
    return true;
}

/// Scans a double-quoted string from @p_from. The opening quote is already
/// passed.
bool scanDouble(std::string_view p_line, std::size_t p_from, std::size_t& p_end)
{
    std::size_t i = p_from;
    while (i < p_line.size())
    {
        if (p_line[i] == '\\')
        {
            i +=
                ((i + 1 < p_line.size()) ? std::size_t{ 2 } : std::size_t{ 1 });
            continue;
        }
        if (p_line[i] == '"')
        {
            p_end = i + 1;
            return true;
        }
        ++i;
    }
    p_end = p_line.size();
    return false;
}

/// Scans a single-quoted string. A doubled quote is an escaped quote.
bool scanSingle(std::string_view p_line, std::size_t p_from, std::size_t& p_end)
{
    std::size_t i = p_from;
    while (i < p_line.size())
    {
        if (p_line[i] == '\'')
        {
            if (i + 1 < p_line.size() && p_line[i + 1] == '\'')
            {
                i += 2;
                continue;
            }
            p_end = i + 1;
            return true;
        }
        ++i;
    }
    p_end = p_line.size();
    return false;
}

/// Letter, digit, underscore or hyphen.
bool isNameChar(char p_c)
{
    unsigned char const c = static_cast<unsigned char>(p_c);
    return std::isalnum(c) != 0 || p_c == '_' || p_c == '-';
}

/// State a YAML line reads and writes. The painter keeps it between lines.
struct YamlCursor
{
    SyntaxPainter::Mode mode = SyntaxPainter::Mode::Code;
    int block_indent = 0;
    int flow = 0;
};

/// Colours one YAML line that is not inside a string or a block scalar.
void colorYamlCode(std::string_view p_line,
                   std::size_t p_from,
                   YamlCursor& p_cursor,
                   std::vector<Span>& p_spans)
{
    static constexpr std::string_view BOOLS[] = {
        "false", "no", "null", "off", "on", "true", "yes", "~",
    };
    bool block = false;
    // A block mark is accepted at the start of the line or just after a key.
    bool value_slot = p_from == 0;
    std::size_t i = p_from;
    while (i < p_line.size())
    {
        char const c = p_line[i];
        if (c == ' ' || c == '\t')
        {
            std::size_t j = i + 1;
            while (j < p_line.size() && (p_line[j] == ' ' || p_line[j] == '\t'))
            {
                ++j;
            }
            pushSpan(p_spans, i, j, Tone::Plain);
            i = j;
            continue;
        }
        if (c == '#')
        {
            pushSpan(p_spans, i, p_line.size(), Tone::Comment);
            break;
        }
        if (c == '"' || c == '\'')
        {
            std::size_t end = 0;
            bool const closed = c == '"' ? scanDouble(p_line, i + 1, end)
                                         : scanSingle(p_line, i + 1, end);
            pushSpan(p_spans, i, end, Tone::String);
            block = false;
            value_slot = false;
            if (!closed)
            {
                p_cursor.mode = c == '"' ? SyntaxPainter::Mode::DoubleQuote
                                         : SyntaxPainter::Mode::SingleQuote;
                return;
            }
            i = end;
            continue;
        }
        std::size_t mark = i;
        if (value_slot && p_cursor.flow == 0 &&
            readBlockIndicator(p_line, i, mark))
        {
            pushSpan(p_spans, i, mark, Tone::Punct);
            block = true;
            value_slot = false;
            i = mark;
            continue;
        }
        if (c == ':' || c == '(' || c == ')' || c == '[' || c == ']' ||
            c == '{' || c == '}' || c == ',' || c == '<' || c == '=' ||
            c == '|' || c == '>')
        {
            if (c == '[' || c == '{')
            {
                ++p_cursor.flow;
            }
            else if (c == ']' || c == '}')
            {
                p_cursor.flow = std::max(0, p_cursor.flow - 1);
            }
            pushSpan(p_spans, i, i + 1, Tone::Punct);
            block = false;
            value_slot = c == ':';
            ++i;
            continue;
        }
        if (c == '&' || c == '*' || c == '!')
        {
            std::size_t j = i + 1;
            if (c == '!' && j < p_line.size() && p_line[j] == '!')
            {
                ++j;
            }
            std::size_t const name = j;
            while (j < p_line.size() && isNameChar(p_line[j]))
            {
                ++j;
            }
            block = false;
            value_slot = false;
            if (j == name)
            {
                pushSpan(p_spans, i, i + 1, Tone::Punct);
                ++i;
                continue;
            }
            pushSpan(p_spans, i, j, c == '!' ? Tone::Keyword : Tone::Variable);
            i = j;
            continue;
        }
        // A list item may be a block scalar: "- |" or "- >".
        if (c == '-' && (i + 1 >= p_line.size() || p_line[i + 1] == ' ' ||
                         p_line[i + 1] == '\t' || p_line[i + 1] == '#'))
        {
            pushSpan(p_spans, i, i + 1, Tone::Punct);
            block = false;
            value_slot = true;
            ++i;
            continue;
        }
        std::size_t j = i;
        while (j < p_line.size() && !isYamlBreak(p_line, j))
        {
            ++j;
        }
        if (j == i)
        {
            pushSpan(p_spans, i, i + 1, Tone::Punct);
            block = false;
            value_slot = false;
            ++i;
            continue;
        }
        std::string_view const token = p_line.substr(i, j - i);
        bool const key = j < p_line.size() && p_line[j] == ':';
        Tone tone = Tone::Plain;
        if (key)
        {
            tone = Tone::Key;
        }
        else if (token == "---" || token == "...")
        {
            tone = Tone::Keyword;
        }
        else if (isNumber(token))
        {
            tone = Tone::Number;
        }
        else if (oneOf(token, BOOLS))
        {
            tone = Tone::Keyword;
        }
        pushSpan(p_spans, i, j, tone);
        block = false;
        value_slot = false;
        i = j;
    }
    if (block && p_cursor.flow == 0)
    {
        p_cursor.mode = SyntaxPainter::Mode::Block;
        p_cursor.block_indent = leadingIndent(p_line);
    }
}

/// Colours one YAML line and updates @p_cursor for the next one.
void colorYaml(std::string_view p_line,
               YamlCursor& p_cursor,
               std::vector<Span>& p_spans)
{
    if (p_cursor.mode == SyntaxPainter::Mode::DoubleQuote ||
        p_cursor.mode == SyntaxPainter::Mode::SingleQuote)
    {
        bool const doubled = p_cursor.mode == SyntaxPainter::Mode::DoubleQuote;
        std::size_t end = 0;
        bool const closed =
            doubled ? scanDouble(p_line, 0, end) : scanSingle(p_line, 0, end);
        pushSpan(p_spans, 0, end, Tone::String);
        if (!closed)
        {
            return;
        }
        p_cursor.mode = SyntaxPainter::Mode::Code;
        if (end < p_line.size())
        {
            colorYamlCode(p_line, end, p_cursor, p_spans);
        }
        return;
    }
    if (p_cursor.mode == SyntaxPainter::Mode::Block)
    {
        if (isBlank(p_line))
        {
            return;
        }
        if (leadingIndent(p_line) > p_cursor.block_indent)
        {
            pushSpan(p_spans, 0, p_line.size(), Tone::String);
            return;
        }
        p_cursor.mode = SyntaxPainter::Mode::Code;
    }
    colorYamlCode(p_line, 0, p_cursor, p_spans);
}

/// A PDDL structure word (`and`, `define`, `forall`...), not a name.
bool isPddlKeyword(std::string_view p_word)
{
    static constexpr std::string_view WORDS[] = {
        "all",      "and",    "assign", "at",     "decrease", "define",
        "domain",   "either", "end",    "exists", "forall",   "imply",
        "increase", "not",    "or",     "over",   "problem",  "scale-down",
        "scale-up", "start",  "when",
    };
    return oneOf(p_word, WORDS);
}

/// Colours one PDDL line. Comments run from `;` to the end of the line.
void colorPddl(std::string_view p_line, std::vector<Span>& p_spans)
{
    std::size_t i = 0;
    while (i < p_line.size())
    {
        char const c = p_line[i];
        if (c == ' ' || c == '\t')
        {
            std::size_t j = i + 1;
            while (j < p_line.size() && (p_line[j] == ' ' || p_line[j] == '\t'))
            {
                ++j;
            }
            pushSpan(p_spans, i, j, Tone::Plain);
            i = j;
            continue;
        }
        if (c == ';')
        {
            pushSpan(p_spans, i, p_line.size(), Tone::Comment);
            break;
        }
        if (c == '"')
        {
            std::size_t end = 0;
            scanDouble(p_line, i + 1, end);
            pushSpan(p_spans, i, end, Tone::String);
            i = end;
            continue;
        }
        if (c == '(' || c == ')')
        {
            pushSpan(p_spans, i, i + 1, Tone::Punct);
            ++i;
            continue;
        }
        if (c == '?' || c == ':')
        {
            std::size_t j = i + 1;
            while (j < p_line.size() && isNameChar(p_line[j]))
            {
                ++j;
            }
            Tone const tone = j == i + 1
                                  ? Tone::Punct
                                  : (c == '?' ? Tone::Variable : Tone::Key);
            pushSpan(p_spans, i, j, tone);
            i = j;
            continue;
        }
        if (isNameChar(c) || c == '.')
        {
            std::size_t j = i + 1;
            while (j < p_line.size() &&
                   (isNameChar(p_line[j]) || p_line[j] == '.'))
            {
                ++j;
            }
            std::string_view const token = p_line.substr(i, j - i);
            Tone tone = Tone::Plain;
            if (token == "-")
            {
                tone = Tone::Punct;
            }
            else if (isNumber(token))
            {
                tone = Tone::Number;
            }
            else if (isPddlKeyword(token))
            {
                tone = Tone::Keyword;
            }
            pushSpan(p_spans, i, j, tone);
            i = j;
            continue;
        }
        pushSpan(p_spans, i, i + 1, Tone::Punct);
        ++i;
    }
}

/// A plain file: a line whose first visible character is `#` is a comment.
void colorPlain(std::string_view p_line, std::vector<Span>& p_spans)
{
    std::size_t i = 0;
    while (i < p_line.size() && (p_line[i] == ' ' || p_line[i] == '\t'))
    {
        ++i;
    }
    if (i < p_line.size() && p_line[i] == '#')
    {
        pushSpan(p_spans, 0, p_line.size(), Tone::Comment);
    }
    else if (!p_line.empty())
    {
        pushSpan(p_spans, 0, p_line.size(), Tone::Plain);
    }
}

} // namespace

// ----------------------------------------------------------------------------
SyntaxPainter::SyntaxPainter(std::filesystem::path const& p_path)
    : m_language(languageOf(p_path))
{
}

// ----------------------------------------------------------------------------
void SyntaxPainter::reset()
{
    m_mode = Mode::Code;
    m_block_indent = 0;
    m_flow = 0;
}

// ----------------------------------------------------------------------------
void SyntaxPainter::skip(std::string_view p_line)
{
    if (m_language != Language::Yaml)
    {
        return;
    }
    consume(p_line, false);
}

// ----------------------------------------------------------------------------
void SyntaxPainter::draw(std::string_view p_line)
{
    consume(p_line, true);
}

// ----------------------------------------------------------------------------
void SyntaxPainter::consume(std::string_view p_line, bool p_draw)
{
    if (!p_line.empty() && p_line.back() == '\r')
    {
        p_line.remove_suffix(1);
    }
    YamlCursor cursor{ m_mode, m_block_indent, m_flow };
    std::vector<Span> spans;
    switch (m_language)
    {
        case Language::Yaml:
            colorYaml(p_line, cursor, spans);
            break;
        case Language::Pddl:
            colorPddl(p_line, spans);
            break;
        case Language::Plain:
            colorPlain(p_line, spans);
            break;
    }
    m_mode = cursor.mode;
    m_block_indent = cursor.block_indent;
    m_flow = cursor.flow;
    if (p_draw)
    {
        drawSpans(p_line, spans);
    }
}
