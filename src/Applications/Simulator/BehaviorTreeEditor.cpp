/**
 * @file BehaviorTreeEditor.cpp
 * @brief Unified behavior tree editor implementation
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "BehaviorTreeEditor.hpp"

#include "ApplicationController.hpp"
#include "ImGuiFileDialog/ImGuiFileDialog.h"
#include "Robotik/Core/Managers/BehaviorTreeManager.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace robotik::application
{

// ============================================================================
// BTInputPort Implementation
// ============================================================================

BTInputPort::BTInputPort()
{
    m_flag_set.setFlag(PortFlag::Default);
    buildGeometry();
}

void BTInputPort::buildGeometry()
{
    m_rect_pin = ImRect(ImVec2(-m_pin_radius, -m_pin_radius),
                        ImVec2(m_pin_radius, m_pin_radius));
    m_rect_port = ImRect(ImVec2(-m_pin_radius * 2, -m_pin_radius * 2),
                         ImVec2(m_pin_radius * 2, m_pin_radius * 2));
}

void BTInputPort::breakLink()
{
    if (m_target_output != nullptr)
    {
        m_target_output->decreaseLinkCount();
    }
    m_target_node = nullptr;
    m_target_output = nullptr;
}

void BTInputPort::translate(ImVec2 delta)
{
    m_position += delta;
    m_rect_port.Translate(delta);
    m_rect_pin.Translate(delta);
}

void BTInputPort::draw(ImDrawList* draw_list, ImVec2 offset, float scale) const
{
    auto center = ImVec2((m_position * scale) + offset);
    auto radius1 = m_pin_radius * scale;
    auto radius2 = m_pin_radius * scale * 0.5f;
    auto radius3 = m_pin_radius * scale * 0.8f;
    auto thickness = 0.15f * m_pin_radius * scale;

    auto color_bg = ImColor(50, 50, 50);
    auto color_ring = ImColor(100, 150, 200);
    auto color_connected = ImColor(255, 255, 255);
    auto color_highlight = ImColor(255, 165, 0);

    draw_list->AddCircleFilled(center, radius1, color_bg);
    draw_list->AddCircle(center, radius1, color_ring, 0, thickness);

    if (m_target_node == nullptr)
    {
        if (m_flag_set.hasFlag(PortFlag::Hovered))
        {
            draw_list->AddCircleFilled(center, radius2, color_highlight);
        }
        if (m_flag_set.hasFlag(PortFlag::Connectible))
        {
            draw_list->AddCircleFilled(center, radius2, color_highlight);
        }
        if (m_flag_set.hasFlag(PortFlag::Dragging))
        {
            draw_list->AddCircleFilled(center, radius3, color_highlight);
        }
    }
    else
    {
        draw_list->AddCircleFilled(center, radius2, color_connected);
        if (m_flag_set.hasFlag(PortFlag::Hovered))
        {
            draw_list->AddCircleFilled(center, radius3, color_highlight);
        }
    }
}

// ============================================================================
// BTOutputPort Implementation
// ============================================================================

BTOutputPort::BTOutputPort()
{
    m_flag_set.setFlag(PortFlag::Default);
    buildGeometry();
}

void BTOutputPort::buildGeometry()
{
    m_rect_pin = ImRect(ImVec2(-m_pin_radius, -m_pin_radius),
                        ImVec2(m_pin_radius, m_pin_radius));
    m_rect_port = ImRect(ImVec2(-m_pin_radius * 2, -m_pin_radius * 2),
                         ImVec2(m_pin_radius * 2, m_pin_radius * 2));
}

void BTOutputPort::translate(ImVec2 delta)
{
    m_position += delta;
    m_rect_port.Translate(delta);
    m_rect_pin.Translate(delta);
}

void BTOutputPort::draw(ImDrawList* draw_list, ImVec2 offset, float scale) const
{
    auto center = ImVec2((m_position * scale) + offset);
    auto radius1 = m_pin_radius * scale;
    auto radius2 = m_pin_radius * scale * 0.5f;
    auto radius3 = m_pin_radius * scale * 0.8f;
    auto thickness = 0.15f * m_pin_radius * scale;

    auto color_bg = ImColor(50, 50, 50);
    auto color_ring = ImColor(100, 150, 200);
    auto color_connected = ImColor(255, 255, 255);
    auto color_highlight = ImColor(255, 165, 0);

    draw_list->AddCircleFilled(center, radius1, color_bg);
    draw_list->AddCircle(center, radius1, color_ring, 0, thickness);

    if (m_link_count == 0)
    {
        if (m_flag_set.hasFlag(PortFlag::Hovered))
        {
            draw_list->AddCircleFilled(center, radius2, color_highlight);
        }
        if (m_flag_set.hasFlag(PortFlag::Connectible))
        {
            draw_list->AddCircleFilled(center, radius2, color_highlight);
        }
        if (m_flag_set.hasFlag(PortFlag::Dragging))
        {
            draw_list->AddCircleFilled(center, radius3, color_highlight);
        }
    }
    else
    {
        draw_list->AddCircleFilled(center, radius2, color_connected);
        if (m_flag_set.hasFlag(PortFlag::Hovered))
        {
            draw_list->AddCircleFilled(center, radius3, color_highlight);
        }
    }
}

// ============================================================================
// BTNodeVisual Implementation
// ============================================================================

BTNodeVisual::BTNodeVisual(bt::Node* node,
                           BTNodeType type,
                           const std::string& name)
    : m_bt_node(node), m_name(name), m_type(type)
{
    m_flag_set.setFlag(NodeFlag::Default | NodeFlag::Visible);

    // Set colors based on node type
    switch (m_type)
    {
        case BTNodeType::Sequence:
            m_color_node = ImColor(100, 150, 200);
            break;
        case BTNodeType::Selector:
            m_color_node = ImColor(200, 150, 100);
            break;
        case BTNodeType::Parallel:
            m_color_node = ImColor(150, 200, 150);
            break;
        case BTNodeType::Decorator:
            m_color_node = ImColor(200, 100, 200);
            break;
        case BTNodeType::Action:
            m_color_node = ImColor(100, 200, 100);
            break;
        case BTNodeType::Condition:
            m_color_node = ImColor(200, 200, 100);
            break;
        case BTNodeType::Unknown:
        default:
            m_color_node = ImColor(150, 150, 150);
            break;
    }

    m_color_head.Value.x = m_color_node.Value.x * 0.5f;
    m_color_head.Value.y = m_color_node.Value.y * 0.5f;
    m_color_head.Value.z = m_color_node.Value.z * 0.5f;
    m_color_head.Value.w = 1.0f;
    m_color_line = ImColor(106, 4, 15, 191);
    m_color_body = ImColor(0, 0, 0, 191);

    buildGeometry();
}

void BTNodeVisual::buildGeometry()
{
    const float padding = 10.0f;
    const float port_spacing = 20.0f;

    // Calculate title size
    ImVec2 name_size = ImGui::CalcTextSize(m_name.c_str());
    m_title_height = name_size.y + padding * 2;

    // Calculate body height based on number of output ports
    float outputs_height = 0.0f;
    if (!m_output_ports.empty())
    {
        outputs_height = port_spacing * float(m_output_ports.size());
    }
    m_body_height = std::max(40.0f, outputs_height + padding * 2);

    // Calculate node size
    float node_width = std::max(name_size.x + padding * 2, 120.0f);
    float node_height = m_title_height + m_body_height;

    m_rect_node = ImRect(ImVec2(0, 0), ImVec2(node_width, node_height));
    m_rect_title = ImRect(ImVec2(0, 0), ImVec2(node_width, m_title_height));

    // Position input port (top center)
    m_input_port_pos = ImVec2(node_width / 2, 0);
    m_input_port.setPosition(m_input_port_pos);

    // Position output ports (bottom)
    m_output_port_start = ImVec2(0, node_height);
    float x_step = node_width / (float(m_output_ports.size()) + 1.0f);
    for (size_t i = 0; i < m_output_ports.size(); ++i)
    {
        float x = x_step * (float(i) + 1.0f);
        m_output_ports[i].setPosition(ImVec2(x, node_height));
    }
}

void BTNodeVisual::setOutputPortCount(size_t count)
{
    m_output_ports.clear();
    for (size_t i = 0; i < count; ++i)
    {
        BTOutputPort port;
        port.setOrder(static_cast<int>(i));
        m_output_ports.push_back(port);
    }
    buildGeometry();
}

void BTNodeVisual::translate(ImVec2 delta, bool selected_only)
{
    if (selected_only && !m_flag_set.hasAnyFlag(NodeFlag::Selected))
    {
        return;
    }

    m_rect_node.Translate(delta);
    m_rect_title.Translate(delta);
    m_input_port.translate(delta);
    for (auto& port : m_output_ports)
    {
        port.translate(delta);
    }
}

void BTNodeVisual::draw(ImDrawList* draw_list, ImVec2 offset, float scale) const
{
    if (!m_flag_set.hasAnyFlag(NodeFlag::Visible))
    {
        return;
    }

    ImRect rect = m_rect_node;
    rect.Min *= scale;
    rect.Max *= scale;
    rect.Translate(offset);

    const float rounding = m_title_height * scale * 0.3f;

    // Draw body
    draw_list->AddRectFilled(rect.Min, rect.Max, m_color_body, rounding);

    // Draw header
    ImVec2 head_br = rect.GetTR() + ImVec2(0.0f, m_title_height * scale);
    draw_list->AddRectFilled(
        rect.Min, head_br, m_color_head, rounding, ImDrawFlags_RoundCornersTop);

    // Draw separator line
    draw_list->AddLine(ImVec2(rect.Min.x, head_br.y),
                       ImVec2(head_br.x, head_br.y),
                       m_color_line,
                       2.0f);

    // Draw name
    ImVec2 text_pos = rect.GetCenter();
    text_pos.y = rect.Min.y + m_title_height * scale * 0.5f;
    ImVec2 text_size = ImGui::CalcTextSize(m_name.c_str());
    text_pos.x -= text_size.x * 0.5f;
    text_pos.y -= text_size.y * 0.5f;
    ImGui::SetWindowFontScale(scale);
    ImGui::SetCursorScreenPos(text_pos);
    ImGui::Text("%s", m_name.c_str());
    ImGui::SetWindowFontScale(1.0f);

    // Selection highlight
    if (m_flag_set.hasAnyFlag(NodeFlag::Selected))
    {
        draw_list->AddRectFilled(
            rect.Min, rect.Max, ImColor(1.0f, 1.0f, 1.0f, 0.25f), rounding);
    }

    // Outline
    ImColor outline_color = m_color_node;
    float outline_thickness = 2.0f * scale;
    if (m_flag_set.hasAnyFlag(NodeFlag::Highlighted))
    {
        outline_color.Value.x *= 1.5f;
        outline_color.Value.y *= 1.5f;
        outline_color.Value.z *= 1.5f;
        outline_color.Value.w = 1.0f;
        outline_thickness *= 1.5f;
    }
    draw_list->AddRect(
        rect.Min, rect.Max, outline_color, rounding, 0, outline_thickness);

    // Draw ports
    m_input_port.draw(draw_list, offset, scale);
    for (const auto& port : m_output_ports)
    {
        port.draw(draw_list, offset, scale);
    }

    // Draw status indicator if bt_node exists
    if (m_bt_node)
    {
        ImColor status_color;
        switch (m_bt_node->status())
        {
            case bt::Status::SUCCESS:
                status_color = ImColor(0, 255, 0, 150);
                break;
            case bt::Status::FAILURE:
                status_color = ImColor(255, 0, 0, 150);
                break;
            case bt::Status::RUNNING:
                status_color = ImColor(255, 165, 0, 150);
                break;
            case bt::Status::INVALID:
            default:
                status_color = ImColor(100, 100, 100, 50);
                break;
        }
        draw_list->AddRectFilled(rect.Min, rect.Max, status_color, rounding);
    }
}

// ============================================================================
// BehaviorTreeEditor Implementation
// ============================================================================

BehaviorTreeEditor::BehaviorTreeEditor(
    ApplicationController& application_controller,
    const std::function<void()>& halt_callback)
    : m_application_controller(application_controller),
      m_halt_callback(halt_callback)
{
    updateStatusMessage("No behavior tree loaded.");
    subscribeToTreeLoadedSignal();
}

BehaviorTreeEditor::~BehaviorTreeEditor()
{
    disconnectTreeLoadedSignal();
}

void BehaviorTreeEditor::onDrawEditorWindow()
{
    if (ImGui::Begin("Behavior Tree Editor",
                     nullptr,
                     ImGuiWindowFlags_NoScrollbar |
                         ImGuiWindowFlags_NoScrollWithMouse))
    {
        auto const& bt_manager = m_application_controller.getBehaviorTreeManager();
        if (bt_manager.hasTree())
        {
            updateCanvasRect();
            updateCanvasScrollZoom();

            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->PushClipRect(m_canvas_rect.Min, m_canvas_rect.Max);

            drawBackground(dl);
            drawCanvasGrid(dl);
            drawLinks(dl);
            drawNodes(dl);
            drawFreeLink(dl);
            drawSelecting(dl);

            dl->PopClipRect();

            handleActions();
            drawPopupMenu();
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f),
                               "No behavior tree loaded. Load a tree from the "
                               "Controls window.");
        }
    }
    ImGui::End();
}

void BehaviorTreeEditor::onDrawControlsWindow()
{
    if (ImGui::Begin("Behavior Tree Controls"))
    {
        drawControlsPanel();
        ImGui::Separator();
        drawBlackboardPanel();
        ImGui::Separator();
        drawPropertiesPanel();
    }
    ImGui::End();
}

void BehaviorTreeEditor::loadTreePanel()
{
    if (m_file_dialog_open &&
        ImGuiFileDialog::Instance()->Display("ChooseBehaviorTree"))
    {
        if (ImGuiFileDialog::Instance()->IsOk())
        {
            std::string file_path =
                ImGuiFileDialog::Instance()->GetFilePathName();
            loadTreeFromPath(file_path);
        }

        ImGuiFileDialog::Instance()->Close();
        m_file_dialog_open = false;
    }
}

// ============================================================================
// Drawing Methods
// ============================================================================

void BehaviorTreeEditor::updateCanvasRect()
{
    m_mouse_pos = ImGui::GetMousePos();
    m_position = ImGui::GetWindowPos() + ImGui::GetWindowContentRegionMin();
    m_size =
        ImGui::GetWindowContentRegionMax() - ImGui::GetWindowContentRegionMin();
    m_canvas_rect = ImRect(m_position, m_position + m_size);
}

void BehaviorTreeEditor::updateCanvasScrollZoom()
{
    const ImGuiIO& io = ImGui::GetIO();
    bool drag_cond = m_state == EditorState::DraggingInput ||
                     m_state == EditorState::DraggingOutput;

    if (m_state != EditorState::None &&
        (ImGui::IsMouseDown(ImGuiMouseButton_Left) == false || drag_cond) &&
        m_canvas_rect.Contains(m_mouse_pos))
    {
        // Pan with middle mouse button
        if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
        {
            m_scroll += io.MouseDelta;
        }

        // Zoom with mouse wheel
        ImVec2 focus = (m_mouse_pos - m_scroll - m_position) / m_scale;
        auto zoom = static_cast<int>(io.MouseWheel);
        if (zoom < 0)
        {
            while (zoom < 0)
            {
                m_scale = std::max(SCALE_MIN, m_scale - SCALE_STEP);
                zoom += 1;
            }
        }
        if (zoom > 0)
        {
            while (zoom > 0)
            {
                m_scale = std::min(SCALE_MAX, m_scale + SCALE_STEP);
                zoom -= 1;
            }
        }
        ImVec2 shift = m_scroll + (focus * m_scale);
        m_scroll += m_mouse_pos - shift - m_position;
    }
}

void BehaviorTreeEditor::drawCanvasGrid(ImDrawList* draw_list) const
{
    const float grid = 32.0f * m_scale;
    float x = std::fmodf(m_scroll.x, grid);
    float y = std::fmodf(m_scroll.y, grid);
    auto mark_x = static_cast<int>(m_scroll.x / grid);
    auto mark_y = static_cast<int>(m_scroll.y / grid);

    while (x < m_size.x)
    {
        ImColor color = mark_x % 5 ? ImColor(0.5f, 0.5f, 0.5f, 0.1f)
                                   : ImColor(1.0f, 1.0f, 1.0f, 0.1f);
        draw_list->AddLine(ImVec2(x, 0.0f) + m_position,
                           ImVec2(x, m_size.y) + m_position,
                           color,
                           0.1f);
        x += grid;
        mark_x -= 1;
    }

    while (y < m_size.y)
    {
        ImColor color = mark_y % 5 ? ImColor(0.5f, 0.5f, 0.5f, 0.1f)
                                   : ImColor(1.0f, 1.0f, 1.0f, 0.1f);
        draw_list->AddLine(ImVec2(0.0f, y) + m_position,
                           ImVec2(m_size.x, y) + m_position,
                           color,
                           0.1f);
        y += grid;
        mark_y -= 1;
    }
}

void BehaviorTreeEditor::drawBackground(ImDrawList* draw_list) const
{
    draw_list->AddRectFilled(
        m_canvas_rect.Min, m_canvas_rect.Max, ImColor(30, 30, 30));
}

void BehaviorTreeEditor::drawNodes(ImDrawList* draw_list) const
{
    ImVec2 offset = m_position + m_scroll;
    for (const auto& node_visual : m_node_visuals)
    {
        node_visual->draw(draw_list, offset, m_scale);
    }
}

void BehaviorTreeEditor::drawLinks(ImDrawList* draw_list) const
{
    for (const auto& link : m_links)
    {
        if (!link.input_port || !link.output_port)
            continue;

        const ImVec2 offset = m_position + m_scroll;
        ImVec2 p_input = link.input_port->getPosition() * m_scale + offset;
        ImVec2 p_output = link.output_port->getPosition() * m_scale + offset;

        if (isLinkVisible(p_input, p_output))
        {
            drawLinkBezier(link, p_input, p_output, draw_list);
        }
    }
}

void BehaviorTreeEditor::drawFreeLink(ImDrawList* draw_list) const
{
    if (m_input_free_link.x != m_output_free_link.x ||
        m_input_free_link.y != m_output_free_link.y)
    {
        BTLink link;
        link.color = ImColor(255, 165, 0);
        link.thickness = 2.0f * m_scale;
        drawLinkBezier(link, m_input_free_link, m_output_free_link, draw_list);
    }
}

void BehaviorTreeEditor::drawSelecting(ImDrawList* draw_list) const
{
    if (m_state == EditorState::Selecting)
    {
        draw_list->AddRectFilled(m_rect_selecting.Min,
                                 m_rect_selecting.Max,
                                 ImColor(1.0f, 1.0f, 0.0f, 0.05f));
        draw_list->AddRect(m_rect_selecting.Min,
                           m_rect_selecting.Max,
                           ImColor(1.0f, 1.0f, 1.0f, 0.5f));
    }
}

void BehaviorTreeEditor::drawLinkBezier(const BTLink& link,
                                        ImVec2 p_input,
                                        ImVec2 p_output,
                                        ImDrawList* draw_list) const
{
    float distance = std::abs(p_output.y - p_input.y);
    float handle_len = std::min(distance * 0.5f, 50.0f * m_scale);

    ImVec2 p1 = p_input;
    ImVec2 p2 = p_input + ImVec2(0, handle_len);
    ImVec2 p3 = p_output - ImVec2(0, handle_len);
    ImVec2 p4 = p_output;

    draw_list->AddBezierCubic(p1, p2, p3, p4, link.color, link.thickness);
}

// ============================================================================
// Input Handling
// ============================================================================

void BehaviorTreeEditor::updateNodeFlags()
{
    m_hov_node = nullptr;

    if (m_node_visuals.empty())
    {
        return;
    }

    ImVec2 offset = m_position + m_scroll;
    ImRect canvas(m_position, m_position + m_size);

    for (auto it = m_node_visuals.rbegin(); it != m_node_visuals.rend(); ++it)
    {
        BTNodeVisual* node = it->get();

        // Unset hovered flag
        node->getFlagSet().unsetFlag(NodeFlag::Hovered);

        // Node area
        ImRect node_area = node->getRectNode();
        node_area.Min *= m_scale;
        node_area.Max *= m_scale;
        node_area.Translate(offset);
        node_area.ClipWith(canvas);

        // Visibility
        if (canvas.Overlaps(node_area))
        {
            node->getFlagSet().setFlag(NodeFlag::Visible);
        }
        else
        {
            node->getFlagSet().unsetFlag(NodeFlag::Visible);
            continue;
        }

        // Hovered node condition
        if (m_state != EditorState::None && m_hov_node == nullptr &&
            node_area.Contains(m_mouse_pos))
        {
            m_hov_node = node;
            m_hov_node->getFlagSet().setFlag(NodeFlag::Hovered);
        }

        // Selecting
        if (m_state == EditorState::Selecting)
        {
            if (m_rect_selecting.Overlaps(node_area))
            {
                node->getFlagSet().setFlag(NodeFlag::Selected);
            }
            else if (!ImGui::GetIO().KeyShift)
            {
                node->getFlagSet().unsetFlag(NodeFlag::Selected);
            }
        }

        updateInputFlags(node);
        updateOutputFlags(node);
    }
}

void BehaviorTreeEditor::updateInputFlags(BTNodeVisual* node)
{
    BTInputPort& input = node->getInputPort();
    input.getFlagSet().unsetFlag(PortFlag::Hovered | PortFlag::Connectible |
                                 PortFlag::Dragging);

    if (m_state == EditorState::DraggingInput)
    {
        if (m_interaction_input == &input)
        {
            input.getFlagSet().setFlag(PortFlag::Dragging);
        }
        return;
    }

    if (m_state == EditorState::DraggingOutput)
    {
        if (m_interaction_node == node)
        {
            return; // Can't connect to itself
        }
        if (connectionRules(node, m_interaction_node))
        {
            input.getFlagSet().setFlag(PortFlag::Connectible);
        }
    }

    if (m_hov_node != node)
    {
        return;
    }

    if (m_state == EditorState::Selecting)
    {
        return;
    }

    if (m_state != EditorState::DraggingOutput &&
        node->getFlagSet().hasAnyFlag(NodeFlag::Selected))
    {
        return;
    }

    // Hovered input condition
    ImRect input_rect = input.getRectPort();
    input_rect.Min *= m_scale;
    input_rect.Max *= m_scale;
    ImVec2 offset = m_position + m_scroll;
    input_rect.Translate(offset);

    if (m_state != EditorState::None && input_rect.Contains(m_mouse_pos))
    {
        if (m_state != EditorState::DraggingOutput)
        {
            input.getFlagSet().setFlag(PortFlag::Hovered);
        }
        else if (input.getFlagSet().hasFlag(PortFlag::Connectible))
        {
            input.getFlagSet().setFlag(PortFlag::Hovered);
        }
    }
}

void BehaviorTreeEditor::updateOutputFlags(BTNodeVisual* node)
{
    for (auto& output : node->getOutputPorts())
    {
        output.getFlagSet().unsetFlag(
            PortFlag::Hovered | PortFlag::Connectible | PortFlag::Dragging);

        if (m_state == EditorState::DraggingOutput)
        {
            if (m_interaction_output == &output)
            {
                output.getFlagSet().setFlag(PortFlag::Dragging);
            }
            continue;
        }

        if (m_state == EditorState::DraggingInput)
        {
            if (m_interaction_node == node)
            {
                continue; // Can't connect to itself
            }
            if (connectionRules(m_interaction_node, node))
            {
                output.getFlagSet().setFlag(PortFlag::Connectible);
            }
        }

        if (m_hov_node != node)
        {
            continue;
        }

        if (m_state == EditorState::Selecting)
        {
            continue;
        }

        if (m_state != EditorState::DraggingInput &&
            node->getFlagSet().hasAnyFlag(NodeFlag::Selected))
        {
            continue;
        }

        // Hovered output condition
        ImRect output_rect = output.getRectPort();
        output_rect.Min *= m_scale;
        output_rect.Max *= m_scale;
        ImVec2 offset = m_position + m_scroll;
        output_rect.Translate(offset);

        if (m_state != EditorState::None && output_rect.Contains(m_mouse_pos))
        {
            if (m_state != EditorState::DraggingInput)
            {
                output.getFlagSet().setFlag(PortFlag::Hovered);
            }
            else if (output.getFlagSet().hasFlag(PortFlag::Connectible))
            {
                output.getFlagSet().setFlag(PortFlag::Hovered);
            }
        }
    }
}

void BehaviorTreeEditor::handleActions()
{
    updateNodeFlags();

    handleMouseMove();

    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left))
    {
        // Double-click handling (node properties, etc.)
        return;
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        handleMouseLeftClick();
        return;
    }

    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
    {
        handleMouseLeftDrag();
        return;
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        handleMouseLeftRelease();
        return;
    }

    if (ImGui::IsMouseReleased(ImGuiMouseButton_Right))
    {
        handleMouseRightRelease();
        return;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Delete))
    {
        handleKeyboardDelete();
        return;
    }
}

void BehaviorTreeEditor::handleMouseMove()
{
    bool condition = (m_state == EditorState::None) ||
                     (m_state == EditorState::Default) ||
                     (m_state == EditorState::HoveringNode) ||
                     (m_state == EditorState::HoveringInput) ||
                     (m_state == EditorState::HoveringOutput);

    if (condition)
    {
        if (!ImGui::IsWindowHovered())
        {
            m_state = EditorState::None;
            m_interaction_node = nullptr;
            m_interaction_input = nullptr;
            m_interaction_output = nullptr;
        }
        else if (m_hov_node == nullptr)
        {
            m_state = EditorState::Default;
            m_interaction_node = nullptr;
            m_interaction_input = nullptr;
            m_interaction_output = nullptr;
        }
        else
        {
            m_interaction_node = m_hov_node;
            m_interaction_input = nullptr;
            m_interaction_output = nullptr;

            // Check for input port hovering
            BTInputPort& input = m_hov_node->getInputPort();
            if (input.getFlagSet().hasFlag(PortFlag::Hovered))
            {
                m_interaction_input = &input;
                m_state = EditorState::HoveringInput;
            }
            else
            {
                // Check for output port hovering
                for (auto& output : m_hov_node->getOutputPorts())
                {
                    if (output.getFlagSet().hasFlag(PortFlag::Hovered))
                    {
                        m_interaction_output = &output;
                        m_state = EditorState::HoveringOutput;
                        break;
                    }
                }

                if (m_interaction_output == nullptr)
                {
                    m_state = EditorState::HoveringNode;
                }
            }
        }
    }
}

void BehaviorTreeEditor::handleMouseLeftClick()
{
    if (m_state == EditorState::Default)
    {
        // Deselect all nodes
        for (const auto& node : m_node_visuals)
        {
            node->getFlagSet().unsetFlag(NodeFlag::Selected |
                                         NodeFlag::Hovered);
        }
    }
    else if (m_state == EditorState::HoveringNode)
    {
        const ImGuiIO& io = ImGui::GetIO();
        if (io.KeyCtrl)
        {
            m_interaction_node->getFlagSet().flipFlag(NodeFlag::Selected);
        }
        else if (io.KeyShift)
        {
            m_interaction_node->getFlagSet().setFlag(NodeFlag::Selected);
        }
        else if (!m_interaction_node->getFlagSet().hasFlag(NodeFlag::Selected))
        {
            highlightNode(m_interaction_node);
        }
        m_state = EditorState::Dragging;
    }
    else if (m_state == EditorState::HoveringInput)
    {
        if (m_interaction_input->getTargetNode() == nullptr)
        {
            m_state = EditorState::DraggingInput;
        }
        else
        {
            m_state = EditorState::Dragging;
        }
    }
    else if (m_state == EditorState::HoveringOutput)
    {
        m_state = EditorState::DraggingOutput;
    }
}

void BehaviorTreeEditor::handleMouseLeftDrag()
{
    const ImGuiIO& io = ImGui::GetIO();

    if (m_state == EditorState::Default)
    {
        if (!io.KeyShift)
        {
            for (const auto& node : m_node_visuals)
            {
                node->getFlagSet().unsetFlag(NodeFlag::Selected);
            }
        }
        m_state = EditorState::Selecting;
    }
    else if (m_state == EditorState::Selecting)
    {
        const ImVec2 pos =
            m_mouse_pos - ImGui::GetMouseDragDelta(ImGuiMouseButton_Left);
        m_rect_selecting.Min = ImMin(pos, m_mouse_pos);
        m_rect_selecting.Max = ImMax(pos, m_mouse_pos);
    }
    else if (m_state == EditorState::Dragging)
    {
        if (!m_interaction_node->getFlagSet().hasFlag(NodeFlag::Selected))
        {
            dragNodeSingle();
        }
        else
        {
            dragNodeMulti();
        }
    }
    else if (m_state == EditorState::DraggingInput)
    {
        m_interaction_output = nullptr;
        if (m_hov_node)
        {
            for (auto& output : m_hov_node->getOutputPorts())
            {
                if (output.getFlagSet().hasFlag(PortFlag::Hovered |
                                                PortFlag::Connectible))
                {
                    m_interaction_output = &output;
                }
            }
        }

        ImVec2 offset = m_position + m_scroll;
        ImVec2 p1 = offset + (m_interaction_input->getPosition() * m_scale);
        ImVec2 p4 =
            m_interaction_output
                ? (offset + (m_interaction_output->getPosition() * m_scale))
                : m_mouse_pos;
        m_input_free_link = ImVec2(p1.x, p1.y);
        m_output_free_link = ImVec2(p4.x, p4.y);
    }
    else if (m_state == EditorState::DraggingOutput)
    {
        m_interaction_input = nullptr;
        if (m_hov_node)
        {
            BTInputPort& input = m_hov_node->getInputPort();
            if (input.getFlagSet().hasFlag(PortFlag::Hovered |
                                           PortFlag::Connectible))
            {
                m_interaction_input = &input;
            }
        }

        ImVec2 offset = m_position + m_scroll;
        ImVec2 p1 = offset + (m_interaction_output->getPosition() * m_scale);
        ImVec2 p4 =
            m_interaction_input
                ? (offset + (m_interaction_input->getPosition() * m_scale))
                : m_mouse_pos;
        m_output_free_link = ImVec2(p1.x, p1.y);
        m_input_free_link = ImVec2(p4.x, p4.y);
    }
}

void BehaviorTreeEditor::handleMouseLeftRelease()
{
    if (m_state == EditorState::Selecting)
    {
        m_rect_selecting = ImRect();
        sortNodeOrder();
        m_state = EditorState::Default;
    }
    else if (m_state == EditorState::Dragging)
    {
        m_state = EditorState::HoveringNode;
    }
    else if (m_state == EditorState::DraggingInput ||
             m_state == EditorState::DraggingOutput)
    {
        if (m_interaction_input && m_interaction_output)
        {
            // Create connection
            if (m_interaction_input->getTargetOutput())
            {
                m_interaction_input->breakLink();
                eraseLink(m_interaction_input);
            }

            BTLink link;
            if (m_state == EditorState::DraggingInput)
            {
                link.input_node = m_interaction_node;
                link.output_node = m_hov_node;
                m_interaction_input->setTargetNode(m_hov_node);
            }
            else
            {
                link.input_node = m_hov_node;
                link.output_node = m_interaction_node;
                m_interaction_input->setTargetNode(m_interaction_node);
            }

            m_interaction_input->setTargetOutput(m_interaction_output);
            m_interaction_output->increaseLinkCount();

            link.input_port = m_interaction_input;
            link.output_port = m_interaction_output;
            m_links.push_back(link);
        }

        m_input_free_link = ImVec2();
        m_output_free_link = ImVec2();
        m_state = EditorState::Default;
    }
}

void BehaviorTreeEditor::handleMouseRightRelease()
{
    const ImGuiIO& io = ImGui::GetIO();
    if (m_state != EditorState::None && m_canvas_rect.Contains(m_mouse_pos) &&
        !ImGui::IsMouseDown(ImGuiMouseButton_Left) &&
        ImGui::IsMouseReleased(ImGuiMouseButton_Right) &&
        m_interaction_node == nullptr)
    {
        if (io.MouseDragMaxDistanceSqr[1] <
            (io.MouseDragThreshold * io.MouseDragThreshold))
        {
            ImGui::OpenPopup("EditorPopupMenu");
        }
    }
}

void BehaviorTreeEditor::handleKeyboardDelete()
{
    std::vector<BTNodeVisual*> nodes_to_delete;

    for (const auto& node : m_node_visuals)
    {
        if (node->getFlagSet().hasFlag(NodeFlag::Selected))
        {
            nodes_to_delete.push_back(node.get());
        }
    }

    for (BTNodeVisual* node : nodes_to_delete)
    {
        // Remove all connections
        BTInputPort& input = node->getInputPort();
        if (input.getTargetNode() != nullptr)
        {
            input.breakLink();
            eraseLink(&input);
        }

        // Remove connections from other nodes to this node
        for (const auto& other_node : m_node_visuals)
        {
            BTInputPort& other_input = other_node->getInputPort();
            if (other_input.getTargetNode() == node)
            {
                other_input.breakLink();
                eraseLink(&other_input);
            }
        }

        // Remove from visuals
        m_node_visuals.erase(
            std::remove_if(m_node_visuals.begin(),
                           m_node_visuals.end(),
                           [node](const std::unique_ptr<BTNodeVisual>& n)
                           { return n.get() == node; }),
            m_node_visuals.end());

        if (node == m_highlighted_node)
        {
            m_highlighted_node = nullptr;
        }
        if (node == m_hov_node)
        {
            m_hov_node = nullptr;
        }
        if (node == m_interaction_node)
        {
            m_interaction_node = nullptr;
        }
    }
}

void BehaviorTreeEditor::dragNodeSingle()
{
    const ImGuiIO& io = ImGui::GetIO();
    if (m_canvas_rect.Contains(m_mouse_pos))
    {
        m_interaction_node->translate(io.MouseDelta / m_scale, false);
    }
}

void BehaviorTreeEditor::dragNodeMulti()
{
    const ImGuiIO& io = ImGui::GetIO();
    if (m_canvas_rect.Contains(m_mouse_pos))
    {
        for (const auto& node : m_node_visuals)
        {
            node->translate(io.MouseDelta / m_scale, true);
        }
    }
}

// ============================================================================
// Helper Methods
// ============================================================================

void BehaviorTreeEditor::eraseLink(BTInputPort* input)
{
    auto it = m_links.begin();
    while (it != m_links.end())
    {
        if (it->input_port == input)
        {
            it = m_links.erase(it);
            return;
        }
        else
        {
            ++it;
        }
    }
}

bool BehaviorTreeEditor::isLinkVisible(ImVec2 p_input, ImVec2 p_output) const
{
    return m_canvas_rect.Contains(p_input) || m_canvas_rect.Contains(p_output);
}

bool BehaviorTreeEditor::connectionRules(const BTNodeVisual* input_node,
                                         const BTNodeVisual* output_node) const
{
    if (!input_node || !output_node)
        return false;

    // Check if connection already exists
    const BTInputPort& input = input_node->getInputPort();
    if (input.getTargetNode() == output_node)
    {
        return false;
    }

    // Root node cannot have input
    // Other validation rules can be added here

    return true;
}

void BehaviorTreeEditor::highlightNode(BTNodeVisual* node)
{
    if (m_highlighted_node != nullptr)
    {
        m_highlighted_node->getFlagSet().unsetFlag(NodeFlag::Highlighted);
    }
    node->getFlagSet().setFlag(NodeFlag::Highlighted);
    m_highlighted_node = node;

    // Bring to front
    auto it = std::find_if(m_node_visuals.begin(),
                           m_node_visuals.end(),
                           [node](const std::unique_ptr<BTNodeVisual>& n)
                           { return n.get() == node; });
    if (it != m_node_visuals.end() && it != m_node_visuals.end() - 1)
    {
        std::rotate(it, it + 1, m_node_visuals.end());
    }
}

void BehaviorTreeEditor::sortNodeOrder()
{
    std::vector<std::unique_ptr<BTNodeVisual>> unselected;
    std::vector<std::unique_ptr<BTNodeVisual>> selected;

    for (auto& node : m_node_visuals)
    {
        if (node->getFlagSet().hasFlag(NodeFlag::Selected))
        {
            selected.push_back(std::move(node));
        }
        else
        {
            unselected.push_back(std::move(node));
        }
    }

    m_node_visuals.clear();
    m_node_visuals.reserve(unselected.size() + selected.size());
    for (auto& node : unselected)
    {
        m_node_visuals.push_back(std::move(node));
    }
    for (auto& node : selected)
    {
        m_node_visuals.push_back(std::move(node));
    }
}

// ============================================================================
// Tree Integration (Continued in next message)
// ============================================================================

BTNodeType BehaviorTreeEditor::detectNodeType(bt::Node* node) const
{
    if (dynamic_cast<bt::Sequence*>(node))
        return BTNodeType::Sequence;
    if (dynamic_cast<bt::Selector*>(node))
        return BTNodeType::Selector;
    if (dynamic_cast<bt::Parallel*>(node))
        return BTNodeType::Parallel;
    if (dynamic_cast<bt::Decorator*>(node))
        return BTNodeType::Decorator;
    if (dynamic_cast<bt::Action*>(node))
        return BTNodeType::Action;
    if (dynamic_cast<bt::Condition*>(node))
        return BTNodeType::Condition;
    return BTNodeType::Unknown;
}

std::string BehaviorTreeEditor::getNodeDisplayName(bt::Node* node) const
{
    if (!node)
        return "Unknown";

    if (!node->name.empty())
        return node->name;
    if (!node->type().empty())
        return node->type();

    BTNodeType type = detectNodeType(node);
    switch (type)
    {
        case BTNodeType::Sequence:
            return "Sequence";
        case BTNodeType::Selector:
            return "Selector";
        case BTNodeType::Parallel:
            return "Parallel";
        case BTNodeType::Decorator:
            return "Decorator";
        case BTNodeType::Action:
            return "Action";
        case BTNodeType::Condition:
            return "Condition";
        case BTNodeType::Unknown:
        default:
            return "Node";
    }
}

void BehaviorTreeEditor::buildVisualsFromTree(bt::Tree* tree)
{
    clearVisuals();

    if (!tree || !tree->hasRoot())
        return;

    bt::Node& root = tree->getRoot();

    // Simple tree layout algorithm
    float h_spacing = 150.0f;
    float v_spacing = 120.0f;
    float x_offset = 0.0f;
    float y_offset = 50.0f;

    traverseTree(&root, nullptr, x_offset, y_offset, h_spacing, v_spacing);
}

void BehaviorTreeEditor::traverseTree(bt::Node* node,
                                      BTNodeVisual* parent_visual,
                                      float& x_offset,
                                      float y_offset,
                                      float h_spacing,
                                      float v_spacing)
{
    if (!node)
        return;

    // Check if already processed
    if (m_bt_to_visual.find(node) != m_bt_to_visual.end())
        return;

    BTNodeType type = detectNodeType(node);
    std::string name = getNodeDisplayName(node);

    auto visual = std::make_unique<BTNodeVisual>(node, type, name);

    // Determine number of children
    size_t child_count = 0;
    std::vector<bt::Node*> children;

    if (auto composite = dynamic_cast<bt::Composite*>(node))
    {
        for (auto& child : composite->getChildren())
        {
            if (child)
            {
                children.push_back(child.get());
            }
        }
        child_count = children.size();
    }
    else if (auto decorator = dynamic_cast<bt::Decorator*>(node))
    {
        if (decorator->hasChild())
        {
            children.push_back(&decorator->getChild());
            child_count = 1;
        }
    }

    visual->setOutputPortCount(child_count);

    // Position the node
    visual->translate(ImVec2(x_offset, y_offset), false);
    x_offset += h_spacing;

    // Create link to parent
    if (parent_visual)
    {
        BTLink link;
        link.input_node = visual.get();
        link.output_node = parent_visual;
        link.input_port = &visual->getInputPort();

        // Find which output port to use
        size_t port_index = 0; // TODO: determine correct port index
        if (port_index < parent_visual->getOutputPorts().size())
        {
            link.output_port = &parent_visual->getOutputPorts()[port_index];
            link.output_port->increaseLinkCount();

            visual->getInputPort().setTargetNode(parent_visual);
            visual->getInputPort().setTargetOutput(link.output_port);

            m_links.push_back(link);
        }
    }

    BTNodeVisual* visual_ptr = visual.get();
    m_bt_to_visual[node] = visual_ptr;
    m_node_visuals.push_back(std::move(visual));

    // Traverse children
    for (bt::Node* child : children)
    {
        traverseTree(child,
                     visual_ptr,
                     x_offset,
                     y_offset + v_spacing,
                     h_spacing,
                     v_spacing);
    }
}

void BehaviorTreeEditor::clearVisuals()
{
    m_node_visuals.clear();
    m_bt_to_visual.clear();
    m_links.clear();
    m_highlighted_node = nullptr;
    m_hov_node = nullptr;
    m_interaction_node = nullptr;
    m_interaction_input = nullptr;
    m_interaction_output = nullptr;
}

void BehaviorTreeEditor::createNode(BTNodeType type, ImVec2 position)
{
    // TODO: Integrate with NodeFactory for creating actual bt::Node instances
    std::string name = "NewNode";
    auto visual = std::make_unique<BTNodeVisual>(nullptr, type, name);
    visual->translate(position, false);

    // Set output count based on type
    if (type == BTNodeType::Sequence || type == BTNodeType::Selector ||
        type == BTNodeType::Parallel)
    {
        visual->setOutputPortCount(2); // Default 2 children, can be expanded
    }
    else if (type == BTNodeType::Decorator)
    {
        visual->setOutputPortCount(1);
    }
    else
    {
        visual->setOutputPortCount(0);
    }

    m_node_visuals.push_back(std::move(visual));
}

void BehaviorTreeEditor::showNodeCreationMenu()
{
    if (ImGui::MenuItem("Sequence"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Sequence, pos);
    }
    if (ImGui::MenuItem("Selector"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Selector, pos);
    }
    if (ImGui::MenuItem("Parallel"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Parallel, pos);
    }
    if (ImGui::MenuItem("Decorator"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Decorator, pos);
    }
    if (ImGui::MenuItem("Action"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Action, pos);
    }
    if (ImGui::MenuItem("Condition"))
    {
        ImVec2 pos = (m_mouse_pos - m_scroll - m_position) / m_scale;
        createNode(BTNodeType::Condition, pos);
    }
}

// ============================================================================
// UI Panels (Continued in next message due to length)
// ============================================================================

void BehaviorTreeEditor::drawControlsPanel()
{
    ImGui::Text("File Controls");

    if (ImGui::Button("📁 Load Tree"))
    {
        loadBehaviorTree();
    }

    ImGui::SameLine();
    auto& bt_manager = m_application_controller.getBehaviorTreeManager();
    if (bt_manager.hasTree())
    {
        if (ImGui::Button("💾 Save"))
        {
            saveBehaviorTree();
        }
    }

    ImGui::Separator();
    if (!m_status_message.empty())
    {
        ImGui::TextWrapped("%s", m_status_message.c_str());
    }
    if (!m_error_message.empty())
    {
        ImGui::TextColored(
            ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "%s", m_error_message.c_str());
    }

    ImGui::Separator();
    ImGui::Text("Execution Controls");

    if (!bt_manager.hasTree())
    {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No tree loaded");
        return;
    }

    bool is_playing = bt_manager.isPlaying();

    if (!is_playing)
    {
        if (ImGui::Button("▶ Play", ImVec2(100, 0)))
        {
            bt_manager.play();
            std::cout << "▶️ Behavior tree playing" << std::endl;
        }
    }
    else
    {
        if (ImGui::Button("⏸ Stop", ImVec2(100, 0)))
        {
            bt_manager.stop();
            std::cout << "⏸ Behavior tree stopped" << std::endl;
        }
    }

    ImGui::SameLine();

    if (ImGui::Button("⏭ Step", ImVec2(100, 0)))
    {
        bt_manager.step(0.016f); // ~60 FPS
        std::cout << "⏭ Behavior tree step" << std::endl;
    }

    ImGui::SameLine();

    if (ImGui::Button("🔄 Reset", ImVec2(100, 0)))
    {
        bt_manager.reset();
        std::cout << "🔄 Behavior tree reset" << std::endl;
    }

    ImGui::Separator();
    if (is_playing)
    {
        ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Status: Playing");
    }
    else if (bt_manager.hasTree())
    {
        ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Status: Stopped");
    }

    std::string const& error = bt_manager.error();
    if (!error.empty())
    {
        ImGui::Separator();
        ImGui::TextColored(
            ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "Error: %s", error.c_str());
    }
}

void BehaviorTreeEditor::drawBlackboardPanel()
{
    ImGui::Text("Blackboard");

    auto& bt_manager = m_application_controller.getBehaviorTreeManager();
    if (!bt_manager.hasTree())
    {
        return;
    }

    auto blackboard = bt_manager.getBlackboard();
    if (!blackboard)
    {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           "No blackboard available");
        return;
    }

    if (ImGui::BeginTable("BlackboardTable",
                          3,
                          ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg))
    {
        ImGui::TableSetupColumn(
            "Key", ImGuiTableColumnFlags_WidthFixed, 150.0f);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn(
            "Actions", ImGuiTableColumnFlags_WidthFixed, 60.0f);
        ImGui::TableHeadersRow();

        // Known keys from YAML
        std::vector<std::string> known_keys = { "home_joint_positions",
                                                "approach_pose",
                                                "pick_pose",
                                                "place_pose",
                                                "wait_duration" };

        for (auto const& key : known_keys)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::Text("%s", key.c_str());

            ImGui::TableNextColumn();

            // Try different types
            auto found_double = blackboard->get<double>(key);
            if (found_double.has_value())
            {
                double edited_val = found_double.value();
                ImGui::PushItemWidth(-1);
                if (ImGui::InputDouble(("##" + key).c_str(), &edited_val))
                {
                    blackboard->set(key, edited_val);
                }
                ImGui::PopItemWidth();
            }
            else
            {
                auto found_string =
                    blackboard->get<std::string>(key);
                if (found_string.has_value())
                {
                    char buffer[256];
                    strncpy(buffer, found_string.value().c_str(), sizeof(buffer));
                    buffer[sizeof(buffer) - 1] = '\0';

                    ImGui::PushItemWidth(-1);
                    if (ImGui::InputText(
                            ("##" + key).c_str(), buffer, sizeof(buffer)))
                    {
                        blackboard->set(key, std::string(buffer));
                    }
                    ImGui::PopItemWidth();
                }
                else
                {
                    auto found_vec =
                        blackboard->get<std::vector<double>>(key);
                    if (found_vec.has_value())
                    {
                        auto const& vec_value = found_vec.value();
                        std::string vec_str = "[";
                        for (size_t i = 0; i < vec_value.size(); ++i)
                        {
                            vec_str += std::to_string(vec_value[i]);
                            if (i < vec_value.size() - 1)
                                vec_str += ", ";
                        }
                        vec_str += "]";
                        ImGui::Text("%s", vec_str.c_str());
                    }
                    else
                    {
                        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                                           "<unknown type>");
                    }
                }
            }

            ImGui::TableNextColumn();
            if (ImGui::SmallButton(("Del##" + key).c_str()))
            {
                blackboard->remove(key);
            }
        }

        ImGui::EndTable();
    }

    if (ImGui::Button("➕ Add Entry"))
    {
       std::cerr << "Add entry dialog not implemented" << std::endl;
    }
}

void BehaviorTreeEditor::drawPropertiesPanel()
{
    ImGui::Text("Node Properties");

    if (m_highlighted_node)
    {
        ImGui::Text("Name: %s", m_highlighted_node->getName().c_str());
        ImGui::Text("Type: %s",
                    [this]()
                    {
                        switch (m_highlighted_node->getType())
                        {
                            case BTNodeType::Sequence:
                                return "Sequence";
                            case BTNodeType::Selector:
                                return "Selector";
                            case BTNodeType::Parallel:
                                return "Parallel";
                            case BTNodeType::Decorator:
                                return "Decorator";
                            case BTNodeType::Action:
                                return "Action";
                            case BTNodeType::Condition:
                                return "Condition";
                            case BTNodeType::Unknown:
                            default:
                                return "Unknown";
                        }
                    }());

        if (m_highlighted_node->getBTNode())
        {
            bt::Node* node = m_highlighted_node->getBTNode();
            ImGui::Text("Status: %s", bt::to_string(node->status()).c_str());
        }
    }
    else
    {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No node selected");
    }
}

void BehaviorTreeEditor::drawPopupMenu()
{
    if (ImGui::BeginPopup("EditorPopupMenu"))
    {
        ImGui::Text("Add Node");
        ImGui::Separator();
        showNodeCreationMenu();
        ImGui::EndPopup();
    }
}

// ============================================================================
// File Operations
// ============================================================================

void BehaviorTreeEditor::loadBehaviorTree()
{
    IGFD::FileDialogConfig config;
    ImGuiFileDialog::Instance()->OpenDialog("ChooseBehaviorTree",
                                            "Choose Behavior Tree YAML",
                                            "YAML files{.yml,.yaml}",
                                            config);

    m_file_dialog_open = true;
    updateStatusMessage("Select a YAML file...");
    m_error_message.clear();
}

bool BehaviorTreeEditor::loadTreeFromPath(const std::string& file_path)
{
    auto& bt_manager = m_application_controller.getBehaviorTreeManager();

    if (file_path.empty())
    {
        updateErrorMessage("Empty file path.");
        return false;
    }

    std::cout << "📂 Loading behavior tree from: " << file_path << std::endl;

    if (!bt_manager.loadTree(file_path))
    {
        std::string message =
            "❌ Failed to load behavior tree: " + bt_manager.error();
        std::cerr << message << std::endl;
        updateErrorMessage(message);
        updateStatusMessage("No behavior tree loaded.");
        clearVisuals();
        return false;
    }

    std::cout << "✅ Behavior tree loaded successfully" << std::endl;
    updateErrorMessage("");
    updateStatusMessage("Behavior tree loaded: " + file_path);
    return true;
}

void BehaviorTreeEditor::saveBehaviorTree()
{
    // TODO: Implement YAML export functionality
    std::cout << "💾 Save functionality not yet implemented" << std::endl;
    updateStatusMessage("Save functionality coming soon...");
}

void BehaviorTreeEditor::updateStatusMessage(const std::string& message)
{
    m_status_message = message;
}

void BehaviorTreeEditor::updateErrorMessage(const std::string& message)
{
    m_error_message = message;
}

// ============================================================================
// Signal Handling
// ============================================================================

void BehaviorTreeEditor::subscribeToTreeLoadedSignal()
{
    disconnectTreeLoadedSignal();

    auto& manager = m_application_controller.getBehaviorTreeManager();
    auto connection_id = manager.onTreeLoaded.connect(
        [this](bt::Tree* tree)
        {
            if (tree != nullptr)
            {
                buildVisualsFromTree(tree);
                if (m_status_message.empty())
                {
                    updateStatusMessage("Behavior tree loaded from signal.");
                }
                updateErrorMessage("");
            }
            else
            {
                clearVisuals();
                updateErrorMessage("Signal received with null tree.");
                updateStatusMessage("No behavior tree loaded.");
            }
        });

    m_tree_loaded_connection =
        std::make_unique<robotik::ScopedConnection<bt::Tree*>>(
            &manager.onTreeLoaded, connection_id);
}

void BehaviorTreeEditor::disconnectTreeLoadedSignal()
{
    m_tree_loaded_connection.reset();
}

} // namespace robotik::application
