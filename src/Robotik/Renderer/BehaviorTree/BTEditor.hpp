/**
 * @file BTEditor.hpp
 * @brief Behavior tree editor library with graphical node editing
 *
 * Based on Oakular from BlackThorn project.
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/BehaviorTree/BehaviorTree.hpp"
#include "Robotik/Core/Common/Signal.hpp"

#include <imgui.h>

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Forward declarations
namespace YAML {
class Emitter;
class Node;
} // namespace YAML

namespace robotik::renderer {

class BTRenderer;

// ****************************************************************************
//! \brief Behavior tree editor library core class.
//!
//! Provides the core functionality for editing and visualizing behavior trees:
//! - Tree management (nodes, links)
//! - YAML serialization/deserialization
//! - Auto-layout
//!
//! This class is designed to be used as an ImGui component.
// ****************************************************************************
class BTEditor
{
    friend class BTRenderer;

public:

    //! \brief Type alias for node and link IDs
    using ID = int32_t;

    // ------------------------------------------------------------------------
    //! \brief Layout direction for displaying the tree.
    // ------------------------------------------------------------------------
    enum class LayoutDirection
    {
        LeftToRight = 0, //!< Layout direction from left to right
        TopToBottom = 1  //!< Layout direction from top to bottom
    };

    // ------------------------------------------------------------------------
    //! \brief Link between nodes.
    // ------------------------------------------------------------------------
    struct Link
    {
        //! \brief Link ID
        ID id = 0;
        //! \brief From node ID
        ID from_node = 0;
        //! \brief To node ID
        ID to_node = 0;
    };

    // ------------------------------------------------------------------------
    //! \brief Graphical representation of a behavior tree node.
    // ------------------------------------------------------------------------
    struct Node
    {
        //! \brief Node ID
        ID id;
        //! \brief Node type ("Sequence", "Selector", etc.)
        std::string type;
        //! \brief User-defined name
        std::string name;
        //! \brief Node position
        ImVec2 position = ImVec2(0, 0);
        //! \brief Node children
        std::vector<ID> children;
        //! \brief Node parent
        ID parent = -1;
        //! \brief Blackboard input parameters
        std::vector<std::string> inputs;
        //! \brief Blackboard output parameters
        std::vector<std::string> outputs;
        //! \brief SubTree reference (for SubTree nodes)
        std::string subtree_reference;
        //! \brief SubTree expansion state
        bool is_expanded = false;
        //! \brief Optional: reference to actual BT node
        std::shared_ptr<bt::Node> bt_node;
        //! \brief Runtime status (0=INVALID, 1=RUNNING, 2=SUCCESS, 3=FAILURE)
        int runtime_status = 0;
    };

    // ------------------------------------------------------------------------
    //! \brief Tree view for tab management
    // ------------------------------------------------------------------------
    struct TreeView
    {
        //! \brief Name of the tree view
        std::string name;
        //! \brief If the tree view is a subtree
        bool is_subtree;
        //! \brief Root node ID of the tree view
        ID root_id;
        //! \brief Layout direction for displaying the tree
        LayoutDirection layout_direction = LayoutDirection::TopToBottom;
        //! \brief Stored node positions for this view (node_id -> position)
        std::unordered_map<ID, ImVec2> node_positions;
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    BTEditor();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~BTEditor();

    // ------------------------------------------------------------------------
    //! \brief Initialize the editor. Call once before use.
    // ------------------------------------------------------------------------
    void init();

    // ------------------------------------------------------------------------
    //! \brief Shutdown and cleanup.
    // ------------------------------------------------------------------------
    void shutdown();

    // ------------------------------------------------------------------------
    //! \brief Draw the behavior tree editor panel.
    //! \param p_title Window title (optional, nullptr for no window).
    // ------------------------------------------------------------------------
    void draw(const char* p_title = nullptr);

    // ------------------------------------------------------------------------
    //! \brief Reset the editor.
    // ------------------------------------------------------------------------
    void reset();

    // ------------------------------------------------------------------------
    //! \brief Load a tree from a YAML file.
    //! \param p_filepath The path to the YAML file.
    // ------------------------------------------------------------------------
    void loadFromYaml(std::string const& p_filepath);

    // ------------------------------------------------------------------------
    //! \brief Load a tree from a YAML string.
    //! \param p_yaml_content The YAML content as a string.
    // ------------------------------------------------------------------------
    void loadFromYamlString(std::string const& p_yaml_content);

    // ------------------------------------------------------------------------
    //! \brief Save a tree to a YAML file.
    //! \param p_filepath The path to the YAML file.
    // ------------------------------------------------------------------------
    void saveToYaml(std::string const& p_filepath);

    // ------------------------------------------------------------------------
    //! \brief Auto-layout the nodes in the tree.
    // ------------------------------------------------------------------------
    void autoLayoutNodes();

    // ------------------------------------------------------------------------
    //! \brief Check if tree has been modified.
    // ------------------------------------------------------------------------
    bool isModified() const { return m_is_modified; }

    // ------------------------------------------------------------------------
    //! \brief Get the current file path.
    // ------------------------------------------------------------------------
    std::string const& getFilePath() const { return m_behavior_tree_filepath; }

    // ------------------------------------------------------------------------
    //! \brief Get nodes (for external access).
    // ------------------------------------------------------------------------
    std::unordered_map<ID, Node> const& getNodes() const { return m_nodes; }

    // ------------------------------------------------------------------------
    //! \brief Get tree views (for external access).
    // ------------------------------------------------------------------------
    std::map<std::string, TreeView> const& getTreeViews() const { return m_tree_views; }

public: // Node operations

    // ------------------------------------------------------------------------
    //! \brief Add a new node to the tree.
    //! \param p_type The type of the node.
    //! \param p_name The name of the node.
    //! \return The ID of the new node.
    // ------------------------------------------------------------------------
    int addNode(std::string const& p_type, std::string const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Add a node and link it to the pending link from node if any.
    //! \param p_type The type of the node.
    //! \param p_name The name of the node.
    // ------------------------------------------------------------------------
    void addNodeAndLink(std::string const& p_type, std::string const& p_name);

    // ------------------------------------------------------------------------
    //! \brief Delete a node from the tree.
    //! \param p_node_id The ID of the node to delete.
    // ------------------------------------------------------------------------
    void deleteNode(int const p_node_id);

    // ------------------------------------------------------------------------
    //! \brief Create a link between two nodes.
    //! \param p_from_node The source node ID.
    //! \param p_to_node The target node ID.
    // ------------------------------------------------------------------------
    void createLink(int const p_from_node, int const p_to_node);

    // ------------------------------------------------------------------------
    //! \brief Delete a link between two nodes.
    //! \param p_from_node The source node ID.
    //! \param p_to_node The target node ID.
    // ------------------------------------------------------------------------
    void deleteLink(int const p_from_node, int const p_to_node);

    // ------------------------------------------------------------------------
    //! \brief Toggle the expansion state of a SubTree node.
    //! \param node_id The ID of the SubTree node.
    // ------------------------------------------------------------------------
    void toggleSubTreeExpansion(ID node_id);

public: // Signals for synchronization

    //! \brief Signal emitted when a node is modified
    Signal<ID> onNodeModified;
    //! \brief Signal emitted when a link is created
    Signal<ID, ID> onLinkCreated;
    //! \brief Signal emitted when a link is deleted
    Signal<ID> onLinkDeleted;

protected: // TreeView helpers

    //! \brief Get the current tree view (creates one if needed).
    TreeView& getCurrentTreeView();

    //! \brief Find a tree view by its root ID.
    TreeView* findTreeViewByRootId(ID root_id);

    //! \brief Find a node by its ID.
    Node* findNode(ID const p_id);

    //! \brief Get the position of a node in the current view.
    ImVec2 getNodePosition(ID node_id);

    //! \brief Set the position of a node in the current view.
    void setNodePosition(ID node_id, ImVec2 position);

    //! \brief Collect all visible nodes from a root (handles SubTree expansion).
    void collectVisibleNodes(ID root_id, std::unordered_set<ID>& visible_nodes);

protected: // Drawing

    //! \brief Draw the behavior tree using the Renderer.
    void drawBehaviorTree();

    //! \brief Draw the menu bar.
    void drawMenuBar();

    //! \brief Show editor tabs for multiple tree views.
    void showEditorTabs();

    //! \brief Draw a single tree tab.
    void drawTreeTab(const std::string& name, TreeView& view);

    //! \brief Handle edit mode interactions (double-click, right-click, etc.).
    void handleEditModeInteractions();

    //! \brief Show the add node palette popup.
    void showAddNodePalette();

    //! \brief Show the node context menu popup.
    void showNodeContextMenu();

    //! \brief Show the node edit popup.
    void showNodeEditPopup();

    //! \brief Show the blackboard panel.
    void showBlackboardPanel();

private: // SubTree management (internal)

    bool expandSubTree(Node* subtree_node);
    bool collapseSubTree(Node* subtree_node);

private: // Tree conversion (internal)

    void buildTreeFromNodes();
    void buildNodesFromTree(bt::Node& p_root);
    int buildNodesFromTreeRecursive(bt::Node& p_node, ID p_parent_id);
    void serializeNodeToYaml(YAML::Emitter& p_out,
                             Node* p_node,
                             bool is_subtree_definition = false);
    int parseYamlNode(const YAML::Node& p_yaml_node, ID p_parent_id);

    void layoutNodeRecursive(Node* p_node,
                             float p_x,
                             float p_y,
                             float& p_max_extent);

private: // auto-increment unique identifiers

    inline ID getNextNodeId()
    {
        return m_unique_node_id++;
    }

protected:

    //! \brief Indicate how to render trees: from left to right or top to bottom.
    LayoutDirection m_layout_direction = LayoutDirection::TopToBottom;
    //! \brief Renderer for the tree.
    std::unique_ptr<BTRenderer> m_renderer;
    //! \brief Modification flag for unsaved changes.
    bool m_is_modified = false;
    //! \brief Currently opened behavior tree file path.
    std::string m_behavior_tree_filepath;
    //! \brief Available tree views (name -> TreeView)
    std::map<std::string, TreeView> m_tree_views;
    //! \brief Nodes for all trees and subtrees.
    std::unordered_map<ID, Node> m_nodes;
    //! \brief Auto-incremented unique node ID.
    ID m_unique_node_id = 1;
    //! \brief Selected node ID.
    ID m_selected_node_id = -1;
    //! \brief Active tree view name
    std::string m_active_tree_name;
    //! \brief Flag to request programmatic tab change (used once then reset)
    bool m_request_tab_change = false;
    //! \brief Pending link creation from drag-drop
    ID m_pending_link_from_node = -1;
    //! \brief Palettes and popups state
    struct ShowPalettes
    {
        //! \brief Show the node creation palette.
        bool node_creation = false;
        //! \brief Show the node edition popup.
        bool node_edition = false;
        //! \brief Position of the palette (screen coordinates).
        ImVec2 position;
        //! \brief Position for node creation (canvas coordinates).
        ImVec2 canvas_position;
    } m_show_palettes;
    //! \brief Blackboard for storing shared variables.
    std::shared_ptr<bt::Blackboard> m_blackboard;
    //! \brief Flag to show the blackboard panel.
    bool m_show_blackboard_panel = true;
};

} // namespace robotik::renderer
