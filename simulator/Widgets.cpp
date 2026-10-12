// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include <algorithm>
#include <cmath>

std::string panelTitle(char const* p_name, char const* p_key)
{
    return std::string(p_name) + "##" + p_key;
}

void badge(char const* p_text, ImVec4 const& p_color)
{
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImVec2 const text = ImGui::CalcTextSize(p_text);
    ImVec2 const pad(6.0f, 1.0f);
    ImVec2 const size(text.x + 2.0f * pad.x, ImGui::GetTextLineHeight() + 2.0f * pad.y);
    ImVec4 fill = p_color;
    fill.w = 0.22f;
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddRectFilled(at, ImVec2(at.x + size.x, at.y + size.y),
                        ImGui::GetColorU32(fill), size.y * 0.5f);
    draw->AddText(ImVec2(at.x + pad.x, at.y + pad.y), ImGui::GetColorU32(p_color), p_text);
    ImGui::Dummy(size);
}

void helpMarker(char const* p_text)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::BeginItemTooltip())
    {
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 30.0f);
        ImGui::TextUnformatted(p_text);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void emptyState(char const* p_text)
{
    ImVec2 const avail = ImGui::GetContentRegionAvail();
    ImVec2 const text = ImGui::CalcTextSize(p_text, nullptr, false, avail.x);
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + std::max(0.0f, (avail.x - text.x) * 0.5f));
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + std::max(0.0f, (avail.y - text.y) * 0.4f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
    ImGui::TextWrapped("%s", p_text);
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
    float const width = std::max(ImGui::GetContentRegionAvail().x, 20.0f);
    float const height = ImGui::GetFrameHeight();
    ImGui::Dummy(ImVec2(width, height));
    ImDrawList* draw = ImGui::GetWindowDrawList();
    ImVec2 const end(at.x + width, at.y + height);
    draw->AddRectFilled(at, end, ImGui::GetColorU32(ImGuiCol_FrameBg), 3.0f);
    double const span = p_upper - p_lower;
    auto place = [&](double p_x)
    {
        double const ratio = span > 0.0 ? std::clamp((p_x - p_lower) / span, 0.0, 1.0) : 0.5;
        return at.x + static_cast<float>(ratio) * width;
    };
    // Bars of a signed quantity grow from zero when zero is inside the range.
    double const origin = (p_lower < 0.0 && p_upper > 0.0) ? 0.0 : p_lower;
    float const x0 = place(origin);
    float const x1 = place(p_value);
    ImVec4 fill = p_color;
    fill.w = 0.55f;
    draw->AddRectFilled(ImVec2(std::min(x0, x1), at.y + 2.0f),
                        ImVec2(std::max(x0, x1) + 1.0f, end.y - 2.0f),
                        ImGui::GetColorU32(fill), 2.0f);
    if (p_has_target)
    {
        float const x = place(p_target);
        draw->AddLine(ImVec2(x, at.y), ImVec2(x, end.y), ImGui::GetColorU32(ORANGE), 2.0f);
    }
    ImVec2 const text = ImGui::CalcTextSize(p_label);
    draw->AddText(ImVec2(at.x + (width - text.x) * 0.5f, at.y + (height - text.y) * 0.5f),
                  ImGui::GetColorU32(ImGuiCol_Text), p_label);
}
