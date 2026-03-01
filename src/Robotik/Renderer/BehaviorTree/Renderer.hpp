/**
 * @file NodeRenderer.hpp
 * @brief Custom node rendering with ImGui
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/Blackboard/Blackboard.hpp"
#include "IDE.hpp"

#include <cstdint>
#include <imgui.h>
#include <imgui_internal.h>
#include <string>
#include <unordered_map>
#include <vector>

// ****************************************************************************
//! \brief Renders behavior tree nodes using pure ImGui
// ****************************************************************************
class Renderer
{
public:

    //! \brief Type alias for node and link IDs
    using ID = int32_t;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    Renderer() = default;

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~Renderer()
    {
        shutdown();
    }

    // ------------------------------------------------------------------------
    //! \brief Cleanup renderer.
    // ------------------------------------------------------------------------
    void shutdown();

    // ------------------------------------------------------------------------
    //! \brief Render the node graph.
    //! \param p_nodes Map of node ID to Node.
    //! \param p_links Vector of links between nodes.
    //! \param p_layout_direction Layout direction (true = top-to-bottom, false
    //! = left-to-right).
    //! \param p_blackboard Pointer to the blackboard for displaying values.
    //! \param p_read_only If true, the graph is read-only (no editing).
    // ------------------------------------------------------------------------
    void drawBehaviorTree(std::unordered_map<ID, IDE::Node>& p_nodes,
                          std::vector<IDE::Link> const& p_links,
                          int p_layout_direction,
                          robotik::bt::Blackboard* p_blackboard = nullptr,
                          bool p_read_only = false);

    // ------------------------------------------------------------------------
    //! \brief Get the last selected node ID.
    //! \return Selected node ID or -1 if none.
    // ------------------------------------------------------------------------
    int getSelectedNodeId() const
    {
        return m_selected_node_id;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the node ID under the mouse cursor.
    //! \return Node ID under cursor or -1 if none.
    // ------------------------------------------------------------------------
    ID getHoveredNodeId() const;

    // ------------------------------------------------------------------------
    //! \brief Get link creation info.
    //! \param p_from_node Output parameter for source node.
    //! \param p_to_node Output parameter for target node.
    //! \return true if a link was created this frame.
    // ------------------------------------------------------------------------
    bool getLinkCreated(ID& p_from_node, ID& p_to_node) const;

    // ------------------------------------------------------------------------
    //! \brief Get link deletion info.
    //! \param p_from_node Output parameter for source node ID.
    //! \param p_to_node Output parameter for target node ID.
    //! \return true if a link was deleted this frame.
    // ------------------------------------------------------------------------
    bool getLinkDeleted(ID& p_from_node, ID& p_to_node) const;

    // ------------------------------------------------------------------------
    //! \brief Get SubTree expand/collapse toggle.
    //! \param p_node_id Output parameter for node ID that was toggled.
    //! \return true if a SubTree node was toggled this frame.
    // ------------------------------------------------------------------------
    bool getSubTreeToggled(ID& p_node_id) const;

    // ------------------------------------------------------------------------
    //! \brief Get link dropped in void info.
    //! \param from_node Output parameter for source node of the dropped link.
    //! \return true if a link was dropped in void (not on any pin) this frame.
    // ------------------------------------------------------------------------
    bool getLinkDroppedInVoid(ID& from_node) const;

    // ------------------------------------------------------------------------
    //! \brief Convert screen coordinates to canvas coordinates.
    //! \param screen_pos The screen position.
    //! \return The canvas position.
    // ------------------------------------------------------------------------
    ImVec2 convertScreenToCanvas(ImVec2 screen_pos) const
    {
        return screenToCanvas(screen_pos);
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Internal structures.
    // ------------------------------------------------------------------------
    struct NodeVisual
    {
        ImVec2 position;       //!< Position of the node.
        ImVec2 size;           //!< Size of the node.
        ImRect bounds;         //!< Bounds of the node.
        ImVec2 input_pin_pos;  //!< Position of input pin.
        ImVec2 output_pin_pos; //!< Position of output pin .
    };

    // ------------------------------------------------------------------------
    //! \brief States for the mouse drag-and-drop.
    // ------------------------------------------------------------------------
    struct DragState
    {
        bool is_dragging_node = false; //!< True if the node is being dragged.
        bool is_dragging_link = false; //!< True if the link is being dragged.
        ID dragged_node_id = -1;       //!< ID of the dragged node.
        ID link_start_node = -1;       //!< ID of the start node of the link.
        ImVec2 drag_start_pos;         //!< Position of the drag start.
        ImVec2 mouse_offset;           //!< Offset of the mouse.
    };

    // ------------------------------------------------------------------------
    //! \brief Rendering a behavior tree node.
    //! \param p_node The node to render.
    //! \param is_top_to_bottom True if the layout is top-to-bottom.
    // ------------------------------------------------------------------------
    void drawNode(IDE::Node const& p_node, bool p_is_top_to_bottom);

    // ------------------------------------------------------------------------
    //! \brief Rendering a pin.
    //! \param pos The position of the pin.
    //! \param is_input True if the pin is an input.
    //! \param is_hovered True if the pin is hovered.
    // ------------------------------------------------------------------------
    void drawPin(ImVec2 pos, bool is_input, bool is_hovered) const;

    // ------------------------------------------------------------------------
    //! \brief Rendering a behavior tree link.
    //! \param start The start position of the link.
    //! \param end The end position of the link.
    //! \param is_selected True if the link is selected.
    //! \param is_top_to_bottom True if the layout is top-to-bottom.
    // ------------------------------------------------------------------------
    void drawLink(ImVec2 start,
                  ImVec2 end,
                  bool is_selected,
                  bool is_top_to_bottom) const;

    // ------------------------------------------------------------------------
    //! \brief Rendering a grid.
    //! \param draw_list The draw list.
    //! \param canvas_pos The position of the canvas.
    // ------------------------------------------------------------------------
    void drawGrid(ImDrawList* draw_list, ImVec2 canvas_pos) const;

    // ------------------------------------------------------------------------
    //! \brief Handling the canvas pan and zoom.
    // ------------------------------------------------------------------------
    void handleCanvasPanAndZoom();

    // ------------------------------------------------------------------------
    //! \brief Handling the node drag.
    //! \param node The node to drag.
    //! \param is_edit_mode True if the editor is in edit mode.
    // ------------------------------------------------------------------------
    void handleNodeDrag(IDE::Node& node, bool is_edit_mode);

    // ------------------------------------------------------------------------
    //! \brief Handling the link creation.
    //! \param node The node to create the link from.
    //! \param is_edit_mode True if the editor is in edit mode.
    // ------------------------------------------------------------------------
    void handleLinkCreation(IDE::Node const& node, bool is_edit_mode);

    // ------------------------------------------------------------------------
    //! \brief Handling the selection.
    //! \param nodes The nodes to select.
    //! \param links The links to select.
    // ------------------------------------------------------------------------
    void handleSelection(std::unordered_map<ID, IDE::Node> const& nodes,
                         std::vector<IDE::Link> const& links);

    // ------------------------------------------------------------------------
    //! \brief Checking if a pin is hovered.
    //! \param pin_pos The position of the pin.
    //! \param mouse_pos The position of the mouse.
    //! \return True if the pin is hovered.
    // ------------------------------------------------------------------------
    bool isPinHovered(ImVec2 pin_pos, ImVec2 mouse_pos) const;

    // ------------------------------------------------------------------------
    //! \brief Calculating the size of a node.
    //! \param node The node to calculate the size of.
    //! \return The size of the node.
    // ------------------------------------------------------------------------
    ImVec2 calculateNodeSize(const IDE::Node& node) const;

    // ------------------------------------------------------------------------
    //! \brief Calculating the positions of the pins.
    //! \param visual The visual of the node.
    //! \param has_input True if the node has an input.
    //! \param has_output True if the node has an output.
    //! \param is_top_to_bottom True if the layout is top-to-bottom.
    // ------------------------------------------------------------------------
    void calculatePinPositions(NodeVisual& visual,
                               bool has_input,
                               bool has_output,
                               bool is_top_to_bottom) const;

    // ------------------------------------------------------------------------
    //! \brief Converting a screen position to a canvas position.
    //! \param screen_pos The screen position.
    //! \return The canvas position.
    // ------------------------------------------------------------------------
    ImVec2 screenToCanvas(ImVec2 screen_pos) const;

    // ------------------------------------------------------------------------
    //! \brief Converting a canvas position to a screen position.
    //! \param canvas_pos The canvas position.
    //! \return The screen position.
    // ------------------------------------------------------------------------
    ImVec2 canvasToScreen(ImVec2 canvas_pos) const;

    // ------------------------------------------------------------------------
    //! \brief Getting the color of a node.
    //! \param type The type of the node.
    //! \return The color of the node.
    // ------------------------------------------------------------------------
    ImU32 getNodeColor(const std::string& type) const;

private:

    // State
    ImVec2 m_canvas_offset = ImVec2(0, 0);
    ImVec2 m_canvas_size = ImVec2(0, 0);
    ImVec2 m_canvas_pos = ImVec2(0, 0);
    float m_canvas_zoom = 1.0f;

    DragState m_drag_state;
    std::unordered_map<ID, NodeVisual> m_node_visuals;
    robotik::bt::Blackboard* m_blackboard =
        nullptr; //!< Pointer to the blackboard for displaying values

    ID m_selected_node_id = -1;
    ID m_selected_link_from = -1;
    ID m_selected_link_to = -1;
    ID m_toggled_subtree_id = -1;
    ID m_created_link_from = -1;
    ID m_created_link_to = -1;
    ID m_deleted_link_from = -1;
    ID m_deleted_link_to = -1;
    ID m_link_void_from_node = -1;
    bool m_link_created_this_frame = false;
    bool m_link_deleted_this_frame = false;
    bool m_link_dropped_in_void = false;
    int m_layout_direction = 1; // 0 = LeftToRight, 1 = TopToBottom
};
