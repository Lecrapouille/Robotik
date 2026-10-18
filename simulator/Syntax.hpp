// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Syntax.hpp
/// @brief Syntax colours for the read-only text view (YAML and PDDL).
#pragma once

#include <filesystem>
#include <string_view>

// ****************************************************************************
/// @brief Colours one file in the text view.
///
/// Create one painter per file. Call @c skip for every line above the first
/// visible line, then @c draw for each visible line, in order. YAML strings
/// and block scalars carry their state from one line to the next.
// ****************************************************************************
class SyntaxPainter
{
public:

    explicit SyntaxPainter(std::filesystem::path const& p_path);

    /// @brief Forgets the carried YAML state. The language stays.
    void reset();

    /// @brief Updates the carried state without drawing. No-op except for YAML.
    void skip(std::string_view p_line);

    /// @brief Draws @p_line on the current ImGui line and updates the state.
    void draw(std::string_view p_line);

    /// @brief Which grammar the file uses.
    enum class Language
    {
        Plain,
        Yaml,
        Pddl,
    };

    /// @brief Where a YAML line continues from the line above.
    enum class Mode
    {
        Code,
        DoubleQuote,
        SingleQuote,
        Block,
    };

private:

    Language m_language = Language::Plain;
    Mode m_mode = Mode::Code;
    /// Indent of the line that opened a block scalar. Content is deeper.
    int m_block_indent = 0;
    /// Nesting of `[ ]` and `{ }` still open.
    int m_flow = 0;

    void consume(std::string_view p_line, bool p_draw);
};
