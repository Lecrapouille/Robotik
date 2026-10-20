// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Syntax.hpp"
#include "Ui.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

std::string panelTitle(char const* p_name, char const* p_key)
{
    return std::string(p_name) + "##" + p_key;
}

void badge(char const* p_text, ImVec4 const& p_color)
{
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImVec2 const text = ImGui::CalcTextSize(p_text);
    ImVec2 const pad(6.0F, 1.0F);
    ImVec2 const size(text.x + 2.0F * pad.x,
                      ImGui::GetTextLineHeight() + 2.0F * pad.y);
    ImVec4 fill = p_color;
    fill.w = 0.22F;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(at,
                        ImVec2(at.x + size.x, at.y + size.y),
                        ImGui::GetColorU32(fill),
                        size.y * 0.5F);
    draw->AddText(ImVec2(at.x + pad.x, at.y + pad.y),
                  ImGui::GetColorU32(p_color),
                  p_text);
    ImGui::Dummy(size);
}

void helpMarker(char const* p_text)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0F);
        ImGui::TextUnformatted(p_text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void emptyState(char const* p_text)
{
    ImVec2 const avail = ImGui::GetContentRegionAvail();
    ImVec2 const text = ImGui::CalcTextSize(p_text, nullptr, false, avail.x);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() +
                         std::max(0.0F, (avail.x - text.x) * 0.5F));
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() +
                         std::max(0.0F, (avail.y - text.y) * 0.4F));
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", p_text);
    ImGui::PopStyleColor();
}

// Tables use this fraction of the base font size.
constexpr float TABLE_FONT = 0.87F;

bool beginTable(char const* p_id,
                int p_columns,
                ImGuiTableFlags p_flags,
                ImVec2 const& p_size)
{
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * TABLE_FONT);
    ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(4.0F, 2.0F));
    if (ImGui::BeginTable(p_id, p_columns, p_flags, p_size))
    {
        return true;
    }
    ImGui::PopStyleVar();
    ImGui::PopFont();
    return false;
}

void endTable()
{
    ImGui::EndTable();
    ImGui::PopStyleVar();
    ImGui::PopFont();
}

ImFont* monoFont()
{
    for (ImFont* font : ImGui::GetIO().Fonts->Fonts)
    {
        if (std::string_view(font->GetDebugName()).starts_with("Cousine"))
        {
            return font;
        }
    }
    return nullptr;
}

void textView(std::filesystem::path const& p_path, std::string const& p_text)
{
    if (ImGui::SmallButton("Copy path"))
    {
        ImGui::SetClipboardText(p_path.string().c_str());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy text"))
    {
        ImGui::SetClipboardText(p_text.c_str());
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%s", p_path.string().c_str());
    ImGui::SetItemTooltip("%s", p_path.string().c_str());

    std::vector<std::string_view> lines;
    std::string_view rest(p_text);
    while (!rest.empty())
    {
        std::size_t const end = rest.find('\n');
        lines.push_back(rest.substr(0, end));
        rest = end == std::string_view::npos ? std::string_view{}
                                             : rest.substr(end + 1U);
    }

    SyntaxPainter painter(p_path);
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.08F, 0.09F, 0.10F, 1.0F));
    if (ImGui::BeginChild("##text",
                          ImVec2(0.0F, 0.0F),
                          ImGuiChildFlags_None,
                          ImGuiWindowFlags_HorizontalScrollbar))
    {
        ImGui::PushFont(monoFont(), 0.0F);
        int const digits =
            static_cast<int>(std::to_string(lines.size()).size());
        int seen = 0;
        ImGuiListClipper clipper;
        clipper.Begin(static_cast<int>(lines.size()));
        while (clipper.Step())
        {
            if (clipper.DisplayStart < seen)
            {
                seen = 0;
                painter.reset();
            }
            while (seen < clipper.DisplayStart)
            {
                painter.skip(lines[static_cast<std::size_t>(seen)]);
                ++seen;
            }
            for (int row = clipper.DisplayStart; row < clipper.DisplayEnd;
                 ++row)
            {
                ImGui::TextDisabled("%*d", digits, row + 1);
                ImGui::SameLine();
                painter.draw(lines[static_cast<std::size_t>(row)]);
                ++seen;
            }
        }
        ImGui::PopFont();
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
}

void rangeBar(double p_value,
              double p_lower,
              double p_upper,
              double p_target,
              bool p_has_target,
              char const* p_label,
              ImVec4 const& p_color)
{
    ImVec2 const at = ImGui::GetCursorScreenPos();
    float const width = std::max(ImGui::GetContentRegionAvail().x, 20.0F);
    float const height = ImGui::GetFrameHeight();
    ImGui::Dummy(ImVec2(width, height));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const end(at.x + width, at.y + height);
    draw->AddRectFilled(at, end, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0F);
    double const span = p_upper - p_lower;
    auto place = [&](double p_x)
    {
        double const ratio =
            span > 0.0 ? std::clamp((p_x - p_lower) / span, 0.0, 1.0) : 0.5;
        return at.x + static_cast<float>(ratio) * width;
    };
    // Bars of a signed quantity grow from zero when zero is inside the range.
    double const origin = (p_lower < 0.0 && p_upper > 0.0) ? 0.0 : p_lower;
    float const x0 = place(origin);
    float const x1 = place(p_value);
    ImVec4 fill = p_color;
    fill.w = 0.55F;
    draw->AddRectFilled(ImVec2(std::min(x0, x1), at.y + 2.0F),
                        ImVec2(std::max(x0, x1) + 1.0F, end.y - 2.0F),
                        ImGui::GetColorU32(fill),
                        2.0F);
    if (p_has_target)
    {
        float const x = place(p_target);
        draw->AddLine(ImVec2(x, at.y),
                      ImVec2(x, end.y),
                      ImGui::GetColorU32(ORANGE),
                      2.0F);
    }
    ImVec2 const text = ImGui::CalcTextSize(p_label);
    draw->AddText(
        ImVec2(at.x + (width - text.x) * 0.5F, at.y + (height - text.y) * 0.5F),
        ImGui::GetColorU32(ImGuiCol_Text),
        p_label);
}
