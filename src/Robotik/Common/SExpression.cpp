// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Common/SExpression.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

namespace
{

/// Reads an S-expression by recursive descent over a text.
class SExpressionReader
{
public:

    /// @param p_text Text to read; it must outlive the reader.
    explicit SExpressionReader(std::string_view p_text) : m_text(p_text) {}

    /// Reads the whole text as one expression.
    /// @throws std::runtime_error "column N: why".
    SExpression readAll()
    {
        skipBlanks();
        if (m_at == m_text.size())
        {
            fail(m_at, "empty condition");
        }
        SExpression expression = read();
        skipBlanks();
        if (m_at != m_text.size())
        {
            fail(m_at,
                 m_text[m_at] == ')'
                     ? "one ')' too many"
                     : "one expression only (put several in (and ...))");
        }
        return expression;
    }

private:

    /// Throws the error of the character at @p_offset.
    [[noreturn]] static void fail(std::size_t p_offset, std::string const& p_why)
    {
        throw std::runtime_error("column " + std::to_string(p_offset + 1U) +
                                 ": " + p_why);
    }

    /// True for a character that ends a word.
    static bool separator(char p_char)
    {
        return p_char == ' ' || p_char == '\t' || p_char == '\n' ||
               p_char == '\r' || p_char == '(' || p_char == ')' ||
               p_char == '"' || p_char == ';';
    }

    /// Skips spaces and comments.
    void skipBlanks()
    {
        while (m_at < m_text.size())
        {
            char const c = m_text[m_at];
            if (c == ';')
            {
                while (m_at < m_text.size() && m_text[m_at] != '\n')
                {
                    ++m_at;
                }
            }
            else if (c == ' ' || c == '\t' || c == '\n' || c == '\r')
            {
                ++m_at;
            }
            else
            {
                return;
            }
        }
    }

    /// Reads the expression that starts at the cursor (blanks skipped).
    SExpression read()
    {
        SExpression node;
        node.begin = m_at;
        char const c = m_text[m_at];
        if (c == '(')
        {
            node.kind = SExpression::Kind::List;
            ++m_at;
            skipBlanks();
            while (m_at < m_text.size() && m_text[m_at] != ')')
            {
                node.items.push_back(read());
                skipBlanks();
            }
            if (m_at == m_text.size())
            {
                fail(node.begin, "this '(' is not closed");
            }
            if (node.items.empty())
            {
                fail(node.begin, "empty list ()");
            }
            ++m_at;
        }
        else if (c == ')')
        {
            fail(m_at, "')' without '('");
        }
        else if (c == '"')
        {
            node.kind = SExpression::Kind::Text;
            ++m_at;
            std::size_t const close = m_text.find('"', m_at);
            if (close == std::string_view::npos)
            {
                fail(node.begin, "this '\"' is not closed");
            }
            node.text = std::string(m_text.substr(m_at, close - m_at));
            m_at = close + 1U;
        }
        else
        {
            while (m_at < m_text.size() && !separator(m_text[m_at]))
            {
                ++m_at;
            }
            node.text = std::string(m_text.substr(node.begin, m_at - node.begin));
        }
        node.end = m_at;
        return node;
    }

    /// Source text.
    std::string_view m_text;
    /// Cursor in the text.
    std::size_t m_at = 0U;
};

} // namespace

Result<SExpression> parseSExpression(std::string_view p_text)
{
    try
    {
        return SExpressionReader(p_text).readAll();
    }
    catch (std::runtime_error const& error)
    {
        return failure(error.what());
    }
}

/// Number of single-letter edits from @p_a to @p_b (Levenshtein distance).
static std::size_t editDistance(std::string_view p_a, std::string_view p_b)
{
    std::vector<std::size_t> row(p_b.size() + 1U);
    for (std::size_t j = 0; j < row.size(); ++j)
    {
        row[j] = j;
    }
    for (std::size_t i = 1; i <= p_a.size(); ++i)
    {
        std::size_t diagonal = row[0];
        row[0] = i;
        for (std::size_t j = 1; j <= p_b.size(); ++j)
        {
            std::size_t const above = row[j];
            std::size_t const change = p_a[i - 1U] == p_b[j - 1U] ? 0U : 1U;
            row[j] = std::min({ row[j] + 1U, row[j - 1U] + 1U, diagonal + change });
            diagonal = above;
        }
    }
    return row.back();
}

std::string closestWord(std::string_view p_word,
                        std::vector<std::string_view> const& p_known)
{
    std::string_view closest;
    std::size_t best = p_word.size() / 3U + 1U;
    for (std::string_view known : p_known)
    {
        if (std::size_t const d = editDistance(p_word, known); d <= best)
        {
            best = d;
            closest = known;
        }
    }
    return std::string(closest);
}

} // namespace robotik
