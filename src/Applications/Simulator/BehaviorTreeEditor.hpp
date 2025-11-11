/**
 * @file BehaviorTreeEditor.hpp
 * @brief Unified behavior tree editor with graphical node editing
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 *
 * This editor is based on the core-nodes project:
 * https://github.com/onurae/core-nodes
 * Copyright (c) 2023 Onur AKIN
 * Licensed under the MIT License.
 */

#pragma once

#define IMGUI_DEFINE_MATH_OPERATORS
#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"
#include "Robotik/Core/Common/Signal.hpp"

#include <functional>
#include <imgui.h>
#include <imgui_internal.h>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

// Forward declarations
namespace robotik::application
{
class ApplicationController;
}

namespace robotik::application
{

// ============================================================================
// Enums and Flags
// ============================================================================

//! \brief Node type for behavior tree nodes
enum class BTNodeType
{
    Unknown = 0,
    Sequence,
    Selector,
    Parallel,
    Decorator,
    Action,
    Condition
};

//! \brief Interaction state machine
enum class EditorState
{
    None = 0,
    Default,
    HoveringNode,
    HoveringInput,
    HoveringOutput,
    Dragging,
    DraggingInput,
    DraggingOutput,
    Selecting
};

//! \brief Flag set utility class
class FlagSet
{
private:

    unsigned int m_flags = 0;

public:

    void setFlag(unsigned int flag)
    {
        m_flags |= flag;
    }
    void unsetFlag(unsigned int flag)
    {
        m_flags &= ~flag;
    }
    void flipFlag(unsigned int flag)
    {
        m_flags ^= flag;
    }
    bool hasFlag(unsigned int flag) const
    {
        return (m_flags & flag) == flag;
    }
    bool hasAnyFlag(unsigned int multiFlag) const
    {
        return (m_flags & multiFlag) != 0;
    }
    bool equal(unsigned int multiFlag) const
    {
        return m_flags == multiFlag;
    }
    unsigned int getInt() const
    {
        return m_flags;
    }
    void setInt(unsigned int flags)
    {
        m_flags = flags;
    }
};

//! \brief Node flags
struct NodeFlag
{
    NodeFlag() = delete;
    static constexpr unsigned int Default = 0;
    static constexpr unsigned int Visible = 1 << 0;
    static constexpr unsigned int Hovered = 1 << 1;
    static constexpr unsigned int Selected = 1 << 2;
    static constexpr unsigned int Highlighted = 1 << 3;
};

//! \brief Port flags
struct PortFlag
{
    PortFlag() = delete;
    static constexpr unsigned int Default = 0;
    static constexpr unsigned int Hovered = 1 << 1;
    static constexpr unsigned int Connectible = 1 << 2;
    static constexpr unsigned int Dragging = 1 << 3;
};

// ============================================================================
// Port Classes
// ============================================================================

class BTNodeVisual;
class BTOutputPort;

//! \brief Input port for behavior tree nodes (parent connection)
class BTInputPort
{
private:

    ImVec2 m_position{ 0, 0 };
    ImRect m_rect_pin;
    ImRect m_rect_port;
    FlagSet m_flag_set;
    BTNodeVisual* m_target_node = nullptr;
    BTOutputPort* m_target_output = nullptr;
    float m_pin_radius = 8.0f;

public:

    BTInputPort();
    ~BTInputPort() = default;

    ImVec2 getPosition() const
    {
        return m_position;
    }
    ImRect getRectPort() const
    {
        return m_rect_port;
    }
    FlagSet& getFlagSet()
    {
        return m_flag_set;
    }
    const FlagSet& getFlagSet() const
    {
        return m_flag_set;
    }

    BTNodeVisual* getTargetNode() const
    {
        return m_target_node;
    }
    void setTargetNode(BTNodeVisual* node)
    {
        m_target_node = node;
    }
    BTOutputPort* getTargetOutput() const
    {
        return m_target_output;
    }
    void setTargetOutput(BTOutputPort* output)
    {
        m_target_output = output;
    }

    void breakLink();
    void translate(ImVec2 delta);
    void draw(ImDrawList* draw_list, ImVec2 offset, float scale) const;
    void setPosition(ImVec2 pos)
    {
        m_position = pos;
    }
    void buildGeometry();
};

//! \brief Output port for behavior tree nodes (child connections)
class BTOutputPort
{
private:

    ImVec2 m_position{ 0, 0 };
    ImRect m_rect_pin;
    ImRect m_rect_port;
    FlagSet m_flag_set;
    int m_link_count = 0;
    float m_pin_radius = 8.0f;
    int m_order = 0;

public:

    BTOutputPort();
    ~BTOutputPort() = default;

    ImVec2 getPosition() const
    {
        return m_position;
    }
    ImRect getRectPort() const
    {
        return m_rect_port;
    }
    FlagSet& getFlagSet()
    {
        return m_flag_set;
    }
    const FlagSet& getFlagSet() const
    {
        return m_flag_set;
    }
    int getLinkCount() const
    {
        return m_link_count;
    }
    void increaseLinkCount()
    {
        m_link_count++;
    }
    void decreaseLinkCount()
    {
        m_link_count = std::max(0, m_link_count - 1);
    }
    int getOrder() const
    {
        return m_order;
    }
    void setOrder(int order)
    {
        m_order = order;
    }

    void translate(ImVec2 delta);
    void draw(ImDrawList* draw_list, ImVec2 offset, float scale) const;
    void setPosition(ImVec2 pos)
    {
        m_position = pos;
    }
    void buildGeometry();
};

// ============================================================================
// Node Visual Class
// ============================================================================

//! \brief Visual representation of a behavior tree node
class BTNodeVisual
{
private:

    bt::Node* m_bt_node = nullptr;
    std::string m_name;
    BTNodeType m_type = BTNodeType::Unknown;
    ImColor m_color_node;
    ImColor m_color_head;
    ImColor m_color_line;
    ImColor m_color_body;
    FlagSet m_flag_set;
    ImRect m_rect_node;
    ImRect m_rect_title;
    float m_title_height = 0.0f;
    float m_body_height = 0.0f;
    ImVec2 m_input_port_pos{ 0, 0 };
    ImVec2 m_output_port_start{ 0, 0 };

    BTInputPort m_input_port;
    std::vector<BTOutputPort> m_output_ports;

    void buildGeometry();

public:

    BTNodeVisual() = default;
    BTNodeVisual(bt::Node* node, BTNodeType type, const std::string& name);
    ~BTNodeVisual() = default;

    bt::Node* getBTNode() const
    {
        return m_bt_node;
    }
    void setBTNode(bt::Node* node)
    {
        m_bt_node = node;
    }
    std::string getName() const
    {
        return m_name;
    }
    void setName(const std::string& name)
    {
        m_name = name;
        buildGeometry();
    }
    BTNodeType getType() const
    {
        return m_type;
    }
    FlagSet& getFlagSet()
    {
        return m_flag_set;
    }
    const FlagSet& getFlagSet() const
    {
        return m_flag_set;
    }
    ImRect getRectNode() const
    {
        return m_rect_node;
    }
    ImRect getRectTitle() const
    {
        return m_rect_title;
    }

    BTInputPort& getInputPort()
    {
        return m_input_port;
    }
    const BTInputPort& getInputPort() const
    {
        return m_input_port;
    }
    std::vector<BTOutputPort>& getOutputPorts()
    {
        return m_output_ports;
    }
    const std::vector<BTOutputPort>& getOutputPorts() const
    {
        return m_output_ports;
    }

    void setOutputPortCount(size_t count);
    void translate(ImVec2 delta, bool selected_only = false);
    void draw(ImDrawList* draw_list, ImVec2 offset, float scale) const;
};

// ============================================================================
// Link Structure
// ============================================================================

//! \brief Connection between nodes
struct BTLink
{
    BTNodeVisual* input_node = nullptr;
    BTNodeVisual* output_node = nullptr;
    BTInputPort* input_port = nullptr;
    BTOutputPort* output_port = nullptr;
    ImColor color = ImColor(100, 100, 100);
    float thickness = 2.0f;
};

// ============================================================================
// Main Editor Class
// ============================================================================

//! \brief Unified behavior tree editor with graphical node editing
class BehaviorTreeEditor
{
private:

    // === Application Integration ===
    ApplicationController& m_application_controller;
    std::function<void()> m_halt_callback;
    std::unique_ptr<robotik::ScopedConnection<bt::Tree*>>
        m_tree_loaded_connection;

    // === Canvas State ===
    EditorState m_state = EditorState::Default;
    float m_scale = 1.0f;
    static constexpr float SCALE_MIN = 0.1f;
    static constexpr float SCALE_MAX = 2.0f;
    static constexpr float SCALE_STEP = 0.05f;
    ImVec2 m_position{ 0, 0 };
    ImVec2 m_size{ 0, 0 };
    ImVec2 m_scroll{ 0, 0 };
    ImVec2 m_mouse_pos{ 0, 0 };
    ImRect m_canvas_rect;

    // === Node Management ===
    std::vector<std::unique_ptr<BTNodeVisual>> m_node_visuals;
    std::map<bt::Node*, BTNodeVisual*> m_bt_to_visual;
    BTNodeVisual* m_hov_node = nullptr;
    BTNodeVisual* m_interaction_node = nullptr;
    BTInputPort* m_interaction_input = nullptr;
    BTOutputPort* m_interaction_output = nullptr;
    BTNodeVisual* m_highlighted_node = nullptr;

    // === Link Management ===
    std::vector<BTLink> m_links;
    ImVec2 m_input_free_link{ 0, 0 };
    ImVec2 m_output_free_link{ 0, 0 };

    // === Selection ===
    ImRect m_rect_selecting;

    // === Status Messages ===
    std::string m_status_message;
    std::string m_error_message;
    bool m_file_dialog_open = false;

    // === Drawing ===
    void updateCanvasRect();
    void updateCanvasScrollZoom();
    void drawCanvasGrid(ImDrawList* draw_list) const;
    void drawBackground(ImDrawList* draw_list) const;
    void drawNodes(ImDrawList* draw_list) const;
    void drawLinks(ImDrawList* draw_list) const;
    void drawFreeLink(ImDrawList* draw_list) const;
    void drawSelecting(ImDrawList* draw_list) const;
    void drawLinkBezier(const BTLink& link,
                        ImVec2 p_input,
                        ImVec2 p_output,
                        ImDrawList* draw_list) const;

    // === Input Handling ===
    void updateNodeFlags();
    void updateInputFlags(BTNodeVisual* node);
    void updateOutputFlags(BTNodeVisual* node);
    void handleActions();
    void handleMouseMove();
    void handleMouseLeftClick();
    void handleMouseLeftDrag();
    void handleMouseLeftRelease();
    void handleMouseRightRelease();
    void handleKeyboardDelete();
    void dragNodeSingle();
    void dragNodeMulti();

    // === Node/Link Management ===
    void eraseLink(BTInputPort* input);
    bool isLinkVisible(ImVec2 p_input, ImVec2 p_output) const;
    bool connectionRules(const BTNodeVisual* input_node,
                         const BTNodeVisual* output_node) const;
    void highlightNode(BTNodeVisual* node);
    void sortNodeOrder();

    // === Tree Integration ===
    void buildVisualsFromTree(bt::Tree* tree);
    void clearVisuals();
    BTNodeType detectNodeType(bt::Node* node) const;
    std::string getNodeDisplayName(bt::Node* node) const;
    void traverseTree(bt::Node* node,
                      BTNodeVisual* parent_visual,
                      float& x_offset,
                      float y_offset,
                      float h_spacing,
                      float v_spacing);

    // === UI Panels ===
    void drawControlsPanel();
    void drawBlackboardPanel();
    void drawPropertiesPanel();
    void drawPopupMenu();

    // === File Operations ===
    void loadBehaviorTree();
    bool loadTreeFromPath(const std::string& file_path);
    void saveBehaviorTree();
    void updateStatusMessage(const std::string& message);
    void updateErrorMessage(const std::string& message);

    // === Signal Handling ===
    void subscribeToTreeLoadedSignal();
    void disconnectTreeLoadedSignal();

    // === Node Creation ===
    void createNode(BTNodeType type, ImVec2 position);
    void showNodeCreationMenu();

public:

    explicit BehaviorTreeEditor(
        ApplicationController& application_controller,
        const std::function<void()>& halt_callback = nullptr);
    ~BehaviorTreeEditor();

    //! \brief Draw the main editor window
    void onDrawEditorWindow();

    //! \brief Draw the controls window
    void onDrawControlsWindow();

    //! \brief Handle file dialog (must be called each frame)
    void loadTreePanel();
};

} // namespace robotik::application
