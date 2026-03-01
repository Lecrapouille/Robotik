/**
 * @file BTRenderer.cpp
 * @brief Custom node rendering implementation
 *
 * Based on Oakular from BlackThorn project.
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/BehaviorTree/BTRenderer.hpp"

#include <imgui.h>

#include <cmath>
#include <iomanip>
#include <sstream>

namespace robotik::renderer {

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------
static constexpr float PIN_RADIUS = 5.0f;
static constexpr float NODE_WIDTH = 180.0f;
static constexpr float NODE_PADDING = 8.0f;
static constexpr float NODE_ROUNDING = 4.0f;
static constexpr float LINK_THICKNESS = 2.0f;
static constexpr float GRID_SIZE = 32.0f;
static constexpr size_t MAX_VALUE_DISPLAY_LENGTH = 15;

// ----------------------------------------------------------------------------
// Helper function to get a display string from the blackboard
// ----------------------------------------------------------------------------
static std::string getBlackboardValueString(bt::Blackboard const* blackboard,
                                            const std::string& key)
{
    if (!blackboard)
        return "";

    auto raw_value = blackboard->raw(key);
    if (!raw_value.has_value())
        return "";

    std::string result;
    try
    {
        if (raw_value->type() == typeid(int))
            result = std::to_string(std::any_cast<int>(*raw_value));
        else if (raw_value->type() == typeid(double))
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2)
                << std::any_cast<double>(*raw_value);
            result = oss.str();
        }
        else if (raw_value->type() == typeid(bool))
            result = std::any_cast<bool>(*raw_value) ? "true" : "false";
        else if (raw_value->type() == typeid(std::string))
            result = std::any_cast<std::string>(*raw_value);
        else if (raw_value->type() ==
                 typeid(std::unordered_map<std::string, std::any>))
        {
            const auto& map =
                std::any_cast<const std::unordered_map<std::string, std::any>&>(
                    *raw_value);
            result = "{";
            size_t count = 0;
            for (const auto& [field_key, field_value] : map)
            {
                if (count > 0)
                    result += ", ";
                result += field_key;
                ++count;
                if (count >= 3 && map.size() > 3)
                {
                    result += ", ...";
                    break;
                }
            }
            result += "}";
        }
        else if (raw_value->type() == typeid(std::vector<std::any>))
        {
            const auto& vec =
                std::any_cast<const std::vector<std::any>&>(*raw_value);
            result = "[" + std::to_string(vec.size()) + " items]";
        }
        else if (raw_value->type() == typeid(std::vector<double>))
        {
            const auto& vec =
                std::any_cast<const std::vector<double>&>(*raw_value);
            result = "[" + std::to_string(vec.size()) + " doubles]";
        }
        else
        {
            result = "...";
        }
    }
    catch (...)
    {
        return "";
    }

    if (result.length() > MAX_VALUE_DISPLAY_LENGTH)
    {
        result = result.substr(0, MAX_VALUE_DISPLAY_LENGTH - 3) + "...";
    }

    return result;
}

// ----------------------------------------------------------------------------
// Color lookup table for node types
// ----------------------------------------------------------------------------
const std::unordered_map<std::string, ImU32> s_type_colors = {
    {"Sequence", IM_COL32(100, 150, 200, 255)},
    {"Selector", IM_COL32(200, 150, 100, 255)},
    {"Parallel", IM_COL32(150, 200, 150, 255)},
    {"Decorator", IM_COL32(200, 100, 200, 255)},
    {"Inverter", IM_COL32(200, 100, 200, 255)},
    {"Repeater", IM_COL32(200, 100, 200, 255)},
    {"Action", IM_COL32(100, 200, 100, 255)},
    {"Condition", IM_COL32(200, 200, 100, 255)},
    {"SubTree", IM_COL32(150, 100, 200, 255)},
};

// ----------------------------------------------------------------------------
void BTRenderer::shutdown()
{
    m_node_visuals.clear();
}

// ----------------------------------------------------------------------------
void BTRenderer::drawBehaviorTree(std::unordered_map<ID, BTEditor::Node>& p_nodes,
                                  std::vector<BTEditor::Link> const& p_links,
                                  int p_layout_direction,
                                  bt::Blackboard* p_blackboard,
                                  bool p_read_only)
{
    m_layout_direction = p_layout_direction;
    m_blackboard = p_blackboard;
    m_link_created_this_frame = false;
    m_link_deleted_this_frame = false;
    m_link_dropped_in_void = false;
    m_toggled_subtree_id = -1;
    m_node_visuals.clear();

    ImGui::BeginChild("Canvas",
                      ImVec2(0, 0),
                      true,
                      ImGuiWindowFlags_NoScrollbar |
                          ImGuiWindowFlags_NoScrollWithMouse);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    m_canvas_size = ImGui::GetContentRegionAvail();
    m_canvas_pos = ImGui::GetCursorScreenPos();

    handleCanvasPanAndZoom();
    drawGrid(draw_list, m_canvas_pos);

    draw_list->PushClipRect(m_canvas_pos,
                            ImVec2(m_canvas_pos.x + m_canvas_size.x,
                                   m_canvas_pos.y + m_canvas_size.y),
                            true);

    for (auto const& [id, node] : p_nodes)
    {
        NodeVisual& visual = m_node_visuals[id];
        visual.size = calculateNodeSize(node);
        visual.position = canvasToScreen(node.position);
        visual.bounds = ImRect(visual.position,
                               ImVec2(visual.position.x + visual.size.x,
                                      visual.position.y + visual.size.y));

        bool has_input = true;
        bool has_output = !node.children.empty() || node.type == "Sequence" ||
                          node.type == "Selector" || node.type == "Parallel" ||
                          node.type == "Inverter" || node.type == "Repeater" ||
                          node.type == "SubTree";
        bool is_top_to_bottom = (p_layout_direction == 1);
        calculatePinPositions(visual, has_input, has_output, is_top_to_bottom);
    }

    for (auto const& link : p_links)
    {
        auto from_it = m_node_visuals.find(link.from_node);
        auto to_it = m_node_visuals.find(link.to_node);

        if (from_it != m_node_visuals.end() && to_it != m_node_visuals.end())
        {
            ImVec2 start = from_it->second.output_pin_pos;
            ImVec2 end = to_it->second.input_pin_pos;
            bool is_selected = (link.from_node == m_selected_link_from &&
                                link.to_node == m_selected_link_to);
            bool is_top_to_bottom = (p_layout_direction == 1);
            drawLink(start, end, is_selected, is_top_to_bottom);
        }
    }

    if (m_drag_state.is_dragging_link && m_drag_state.link_start_node >= 0)
    {
        auto it = m_node_visuals.find(m_drag_state.link_start_node);
        if (it != m_node_visuals.end())
        {
            ImVec2 start = it->second.output_pin_pos;
            ImVec2 end = ImGui::GetMousePos();
            bool is_top_to_bottom = (p_layout_direction == 1);
            drawLink(start, end, false, is_top_to_bottom);
        }
    }

    bool is_top_to_bottom =
        (p_layout_direction == static_cast<int>(BTEditor::LayoutDirection::TopToBottom));
    for (auto& [id, node] : p_nodes)
    {
        drawNode(node, is_top_to_bottom);
    }

    if (!p_read_only)
    {
        handleSelection(p_nodes, p_links);

        for (auto& [id, node] : p_nodes)
        {
            handleNodeDrag(node, !p_read_only);
            handleLinkCreation(node, !p_read_only);
        }
    }

    draw_list->PopClipRect();
    ImGui::EndChild();
}

// ----------------------------------------------------------------------------
void BTRenderer::drawNode(BTEditor::Node const& p_node, bool p_is_top_to_bottom)
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    NodeVisual const& visual = m_node_visuals[p_node.id];

    ImVec2 pos = visual.position;
    ImVec2 size = visual.size;

    ImU32 bg_color;
    ImU32 header_color;

    if (p_node.runtime_status != 0)
    {
        switch (p_node.runtime_status)
        {
            case 1:
                bg_color = IM_COL32(80, 60, 20, 255);
                header_color = IM_COL32(255, 180, 0, 255);
                break;
            case 2:
                bg_color = IM_COL32(20, 60, 20, 255);
                header_color = IM_COL32(50, 200, 50, 255);
                break;
            case 3:
                bg_color = IM_COL32(60, 20, 20, 255);
                header_color = IM_COL32(200, 50, 50, 255);
                break;
            default:
                bg_color = IM_COL32(40, 40, 40, 255);
                header_color = IM_COL32(80, 80, 80, 255);
                break;
        }
    }
    else
    {
        bg_color = IM_COL32(45, 45, 48, 255);
        header_color = getNodeColor(p_node.type);
    }

    draw_list->AddRectFilled(
        pos, ImVec2(pos.x + size.x, pos.y + size.y), bg_color, NODE_ROUNDING);

    float header_height = 24.0f;
    draw_list->AddRectFilled(pos,
                             ImVec2(pos.x + size.x, pos.y + header_height),
                             header_color,
                             NODE_ROUNDING,
                             ImDrawFlags_RoundCornersTop);

    auto text_pos = ImVec2(pos.x + NODE_PADDING, pos.y + 4);
    draw_list->AddText(
        text_pos, IM_COL32(255, 255, 255, 255), p_node.type.c_str());

    if (p_node.type == "SubTree" && !p_node.subtree_reference.empty())
    {
        const char* expand_text = p_node.is_expanded ? "[-]" : "[+]";
        auto button_pos = ImVec2(pos.x + size.x - 30, pos.y + 4);
        draw_list->AddText(
            button_pos, IM_COL32(150, 200, 255, 255), expand_text);
    }

    text_pos.y += header_height + NODE_PADDING;
    draw_list->AddText(
        text_pos, IM_COL32(220, 220, 220, 255), p_node.name.c_str());
    text_pos.y += 18;

    if (p_node.type == "SubTree" && !p_node.subtree_reference.empty())
    {
        std::string ref_text = "Ref: " + p_node.subtree_reference;
        draw_list->AddText(
            text_pos, IM_COL32(180, 180, 180, 200), ref_text.c_str());
        text_pos.y += 18;
    }

    if (!p_node.inputs.empty())
    {
        text_pos.y += 4;
        draw_list->AddText(text_pos, IM_COL32(150, 200, 255, 255), "Inputs:");
        text_pos.y += 16;
        for (const auto& input : p_node.inputs)
        {
            std::string value_str =
                getBlackboardValueString(m_blackboard, input);
            std::string input_text = value_str.empty()
                                         ? ("  - " + input)
                                         : ("  - " + input + ": " + value_str);
            draw_list->AddText(
                text_pos, IM_COL32(180, 180, 180, 255), input_text.c_str());
            text_pos.y += 16;
        }
    }

    if (!p_node.outputs.empty())
    {
        text_pos.y += 4;
        draw_list->AddText(text_pos, IM_COL32(255, 200, 150, 255), "Outputs:");
        text_pos.y += 16;
        for (const auto& output : p_node.outputs)
        {
            std::string value_str =
                getBlackboardValueString(m_blackboard, output);
            std::string output_text =
                value_str.empty() ? ("  - " + output)
                                  : ("  - " + output + ": " + value_str);
            draw_list->AddText(
                text_pos, IM_COL32(180, 180, 180, 255), output_text.c_str());
            text_pos.y += 16;
        }
    }

    ImU32 border_color;
    float border_thickness;

    if (p_node.id == m_selected_node_id)
    {
        border_color = IM_COL32(255, 255, 100, 255);
        border_thickness = 3.0f;
    }
    else if (p_node.runtime_status != 0)
    {
        border_color = header_color;
        border_thickness = 2.0f;
    }
    else
    {
        border_color = IM_COL32(100, 100, 100, 255);
        border_thickness = 1.5f;
    }

    draw_list->AddRect(pos,
                       ImVec2(pos.x + size.x, pos.y + size.y),
                       border_color,
                       NODE_ROUNDING,
                       0,
                       border_thickness);

    ImVec2 mouse_pos = ImGui::GetMousePos();

    bool hovered = isPinHovered(visual.input_pin_pos, mouse_pos);
    drawPin(visual.input_pin_pos, true, hovered);

    bool can_have_children =
        !p_node.children.empty() || p_node.type == "Sequence" ||
        p_node.type == "Selector" || p_node.type == "Parallel" ||
        p_node.type == "Inverter" || p_node.type == "Repeater" ||
        p_node.type == "SubTree";
    if (can_have_children)
    {
        hovered = isPinHovered(visual.output_pin_pos, mouse_pos);
        drawPin(visual.output_pin_pos, false, hovered);
    }
}

// ----------------------------------------------------------------------------
void BTRenderer::drawPin(ImVec2 p_position,
                         bool p_is_input,
                         bool p_is_hovered) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    ImU32 color = p_is_input ? IM_COL32(100, 200, 100, 255)
                             : IM_COL32(200, 100, 100, 255);

    if (p_is_hovered)
    {
        color = IM_COL32(255, 255, 100, 255);
    }

    draw_list->AddCircleFilled(p_position, PIN_RADIUS, color, 12);
    draw_list->AddCircle(
        p_position, PIN_RADIUS, IM_COL32(255, 255, 255, 255), 12, 1.5f);
}

// ----------------------------------------------------------------------------
void BTRenderer::drawLink(ImVec2 p_start,
                          ImVec2 p_end,
                          bool p_is_selected,
                          bool p_is_top_to_bottom) const
{
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    ImVec2 cp1;
    ImVec2 cp2;
    if (p_is_top_to_bottom)
    {
        float offset = std::abs(p_end.y - p_start.y) * 0.5f;
        cp1 = ImVec2(p_start.x, p_start.y + offset);
        cp2 = ImVec2(p_end.x, p_end.y - offset);
    }
    else
    {
        float offset = std::abs(p_end.x - p_start.x) * 0.5f;
        cp1 = ImVec2(p_start.x + offset, p_start.y);
        cp2 = ImVec2(p_end.x - offset, p_end.y);
    }

    ImU32 color = p_is_selected ? IM_COL32(255, 200, 0, 255)
                                : IM_COL32(150, 150, 150, 255);

    float thickness = p_is_selected ? 3.0f : LINK_THICKNESS;

    draw_list->AddBezierCubic(p_start, cp1, cp2, p_end, color, thickness);
}

// ----------------------------------------------------------------------------
void BTRenderer::drawGrid(ImDrawList* draw_list, ImVec2 canvas_pos) const
{
    ImU32 grid_color = IM_COL32(60, 60, 60, 100);

    float grid_offset_x = std::fmod(m_canvas_offset.x, GRID_SIZE);
    float grid_offset_y = std::fmod(m_canvas_offset.y, GRID_SIZE);

    for (float x = grid_offset_x; x < m_canvas_size.x; x += GRID_SIZE)
    {
        draw_list->AddLine(
            ImVec2(canvas_pos.x + x, canvas_pos.y),
            ImVec2(canvas_pos.x + x, canvas_pos.y + m_canvas_size.y),
            grid_color);
    }

    for (float y = grid_offset_y; y < m_canvas_size.y; y += GRID_SIZE)
    {
        draw_list->AddLine(
            ImVec2(canvas_pos.x, canvas_pos.y + y),
            ImVec2(canvas_pos.x + m_canvas_size.x, canvas_pos.y + y),
            grid_color);
    }
}

// ----------------------------------------------------------------------------
void BTRenderer::handleCanvasPanAndZoom()
{
    auto const& io = ImGui::GetIO();

    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
    {
        ImVec2 delta = io.MouseDelta;
        m_canvas_offset.x += delta.x;
        m_canvas_offset.y += delta.y;
    }
}

// ----------------------------------------------------------------------------
void BTRenderer::handleNodeDrag(BTEditor::Node& node, bool is_edit_mode)
{
    if (!is_edit_mode)
        return;

    NodeVisual const& visual = m_node_visuals[node.id];
    ImVec2 mouse_pos = ImGui::GetMousePos();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        visual.bounds.Contains(mouse_pos))
    {
        if (!isPinHovered(visual.input_pin_pos, mouse_pos) &&
            !isPinHovered(visual.output_pin_pos, mouse_pos))
        {
            m_drag_state.is_dragging_node = true;
            m_drag_state.dragged_node_id = node.id;
            m_drag_state.mouse_offset = ImVec2(mouse_pos.x - visual.position.x,
                                               mouse_pos.y - visual.position.y);
        }
    }

    if (m_drag_state.is_dragging_node &&
        m_drag_state.dragged_node_id == node.id)
    {
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        {
            auto new_screen_pos =
                ImVec2(mouse_pos.x - m_drag_state.mouse_offset.x,
                       mouse_pos.y - m_drag_state.mouse_offset.y);
            node.position = screenToCanvas(new_screen_pos);
        }
        else if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        {
            m_drag_state.is_dragging_node = false;
            m_drag_state.dragged_node_id = -1;
        }
    }
}

// ----------------------------------------------------------------------------
void BTRenderer::handleLinkCreation(BTEditor::Node const& p_node, bool p_is_edit_mode)
{
    bool can_have_children =
        !p_node.children.empty() || p_node.type == "Sequence" ||
        p_node.type == "Selector" || p_node.type == "Parallel" ||
        p_node.type == "Inverter" || p_node.type == "Repeater" ||
        p_node.type == "SubTree";
    if ((!p_is_edit_mode) || (!can_have_children))
        return;

    NodeVisual const& visual = m_node_visuals[p_node.id];
    ImVec2 mouse_pos = ImGui::GetMousePos();

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        if (isPinHovered(visual.output_pin_pos, mouse_pos))
        {
            m_drag_state.is_dragging_link = true;
            m_drag_state.link_start_node = p_node.id;
        }
    }

    if (m_drag_state.is_dragging_link &&
        ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        bool link_connected = false;

        for (auto const& [id, other_node_visual] : m_node_visuals)
        {
            if (id != m_drag_state.link_start_node)
            {
                if (isPinHovered(other_node_visual.input_pin_pos, mouse_pos))
                {
                    m_created_link_from = m_drag_state.link_start_node;
                    m_created_link_to = id;
                    m_link_created_this_frame = true;
                    link_connected = true;
                    break;
                }
            }
        }

        if (!link_connected)
        {
            m_link_dropped_in_void = true;
            m_link_void_from_node = m_drag_state.link_start_node;
        }

        m_drag_state.is_dragging_link = false;
        m_drag_state.link_start_node = -1;
    }

    if (p_node.type == "SubTree" && !p_node.subtree_reference.empty())
    {
        auto button_pos = ImVec2(visual.position.x + visual.size.x - 30,
                                 visual.position.y + 4);
        auto button_rect =
            ImRect(button_pos, ImVec2(button_pos.x + 30, button_pos.y + 16));

        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
            button_rect.Contains(mouse_pos))
        {
            m_toggled_subtree_id = p_node.id;
        }
    }
}

// ----------------------------------------------------------------------------
void BTRenderer::handleSelection(std::unordered_map<ID, BTEditor::Node> const& nodes,
                                 std::vector<BTEditor::Link> const& p_links)
{
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        ImVec2 mouse_pos = ImGui::GetMousePos();
        bool found = false;

        for (auto& [id, node] : nodes)
        {
            NodeVisual const& visual = m_node_visuals[id];
            if (visual.bounds.Contains(mouse_pos))
            {
                if (!isPinHovered(visual.input_pin_pos, mouse_pos) &&
                    !isPinHovered(visual.output_pin_pos, mouse_pos))
                {
                    m_selected_node_id = id;
                    m_selected_link_from = -1;
                    m_selected_link_to = -1;
                    found = true;
                    break;
                }
            }
        }

        if (!found && !m_drag_state.is_dragging_link)
        {
            m_selected_node_id = -1;
            m_selected_link_from = -1;
            m_selected_link_to = -1;
        }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Delete))
    {
        if ((m_selected_link_from >= 0) && (m_selected_link_to >= 0))
        {
            m_deleted_link_from = m_selected_link_from;
            m_deleted_link_to = m_selected_link_to;
            m_link_deleted_this_frame = true;
            m_selected_link_from = -1;
            m_selected_link_to = -1;
        }
    }
}

// ----------------------------------------------------------------------------
bool BTRenderer::isPinHovered(ImVec2 p_pin_pos, ImVec2 p_mouse_pos) const
{
    float dx = p_mouse_pos.x - p_pin_pos.x;
    float dy = p_mouse_pos.y - p_pin_pos.y;
    float dist_sq = dx * dx + dy * dy;
    return dist_sq <= (PIN_RADIUS + 2) * (PIN_RADIUS + 2);
}

// ----------------------------------------------------------------------------
ImVec2 BTRenderer::calculateNodeSize(const BTEditor::Node& p_node) const
{
    float height = 24.0f + NODE_PADDING;
    height += 18.0f;

    if (p_node.type == "SubTree" && !p_node.subtree_reference.empty())
    {
        height += 18.0f + 18.0f;
    }

    if (!p_node.inputs.empty())
    {
        height += 20.0f;
        height += float(p_node.inputs.size()) * 16.0f;
    }

    if (!p_node.outputs.empty())
    {
        height += 20.0f;
        height += float(p_node.outputs.size()) * 16.0f;
    }

    height += NODE_PADDING * 2;

    return ImVec2(NODE_WIDTH, height);
}

// ----------------------------------------------------------------------------
void BTRenderer::calculatePinPositions(NodeVisual& p_visual,
                                       bool p_has_input,
                                       bool p_has_output,
                                       bool p_is_top_to_bottom) const
{
    if (p_is_top_to_bottom)
    {
        if (p_has_input)
        {
            p_visual.input_pin_pos =
                ImVec2(p_visual.position.x + p_visual.size.x * 0.5f,
                       p_visual.position.y);
        }

        if (p_has_output)
        {
            p_visual.output_pin_pos =
                ImVec2(p_visual.position.x + p_visual.size.x * 0.5f,
                       p_visual.position.y + p_visual.size.y);
        }
    }
    else
    {
        if (p_has_input)
        {
            p_visual.input_pin_pos =
                ImVec2(p_visual.position.x,
                       p_visual.position.y + p_visual.size.y * 0.5f);
        }

        if (p_has_output)
        {
            p_visual.output_pin_pos =
                ImVec2(p_visual.position.x + p_visual.size.x,
                       p_visual.position.y + p_visual.size.y * 0.5f);
        }
    }
}

// ----------------------------------------------------------------------------
ImVec2 BTRenderer::screenToCanvas(ImVec2 p_screen_pos) const
{
    return ImVec2(
        (p_screen_pos.x - m_canvas_pos.x - m_canvas_offset.x) / m_canvas_zoom,
        (p_screen_pos.y - m_canvas_pos.y - m_canvas_offset.y) / m_canvas_zoom);
}

// ----------------------------------------------------------------------------
ImVec2 BTRenderer::canvasToScreen(ImVec2 p_canvas_pos) const
{
    return ImVec2(
        p_canvas_pos.x * m_canvas_zoom + m_canvas_offset.x + m_canvas_pos.x,
        p_canvas_pos.y * m_canvas_zoom + m_canvas_offset.y + m_canvas_pos.y);
}

// ----------------------------------------------------------------------------
ImU32 BTRenderer::getNodeColor(const std::string& p_type) const
{
    auto it = s_type_colors.find(p_type);
    if (it != s_type_colors.end())
    {
        return it->second;
    }
    return IM_COL32(150, 150, 150, 255);
}

// ----------------------------------------------------------------------------
bool BTRenderer::getLinkCreated(ID& p_from_node, ID& p_to_node) const
{
    if (m_link_created_this_frame)
    {
        p_from_node = m_created_link_from;
        p_to_node = m_created_link_to;
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool BTRenderer::getLinkDeleted(ID& p_from_node, ID& p_to_node) const
{
    if (m_link_deleted_this_frame)
    {
        p_from_node = m_deleted_link_from;
        p_to_node = m_deleted_link_to;
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool BTRenderer::getSubTreeToggled(ID& p_node_id) const
{
    if (m_toggled_subtree_id >= 0)
    {
        p_node_id = m_toggled_subtree_id;
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
bool BTRenderer::getLinkDroppedInVoid(ID& p_from_node) const
{
    if (m_link_dropped_in_void)
    {
        p_from_node = m_link_void_from_node;
        return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
BTRenderer::ID BTRenderer::getHoveredNodeId() const
{
    ImVec2 mouse_pos = ImGui::GetMousePos();

    for (const auto& [id, visual] : m_node_visuals)
    {
        if (visual.bounds.Contains(mouse_pos))
        {
            return id;
        }
    }

    return -1;
}

} // namespace robotik::renderer
