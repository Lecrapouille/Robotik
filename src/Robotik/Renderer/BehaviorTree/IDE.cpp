/**
 * @file IDE.cpp
 * @brief Oakular - Behavior tree editor library implementation
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "IDE.hpp"
#include "Renderer.hpp"

#include <ImGuiFileDialog.h>
#include <imgui_stdlib.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

// ----------------------------------------------------------------------------
//! \brief Extract the filename without extension from a file path.
//! \param[in] filepath The full file path.
//! \return The filename without extension, or empty string if extraction fails.
// ----------------------------------------------------------------------------
static std::string extractFileNameWithoutExtension(std::string const& filepath)
{
    if (filepath.empty())
        return "";

    try
    {
        std::filesystem::path path(filepath);
        std::string filename = path.stem().string();
        return filename.empty() ? "" : filename;
    }
    catch (...)
    {
        // Fallback: manual extraction if filesystem fails
        size_t last_slash = filepath.find_last_of("/\\");
        size_t start = (last_slash == std::string::npos) ? 0 : last_slash + 1;
        size_t last_dot = filepath.find_last_of('.');
        size_t end = (last_dot == std::string::npos || last_dot < start)
                         ? filepath.length()
                         : last_dot;
        return filepath.substr(start, end - start);
    }
}

// ----------------------------------------------------------------------------
IDE::IDE(size_t const p_width, size_t const p_height)
    : BTApplication(p_width, p_height)
{
}

// ----------------------------------------------------------------------------
bool IDE::onSetup()
{
    setTitle("Oakular - BlackThorn Editor");
    m_renderer = std::make_unique<Renderer>();
    m_server = std::make_unique<Server>();
    reset();
    return true;
}

// ----------------------------------------------------------------------------
void IDE::reset()
{
    m_nodes.clear();
    m_tree_views.clear();
    m_unique_node_id = 1;
    m_selected_node_id = -1;
    m_active_tree_name.clear();
    m_behavior_tree_filepath.clear();
    m_is_modified = false;
    m_show_palettes.node_creation = false;
    m_show_palettes.node_edition = false;
    m_show_palettes.quit_confirmation = false;
    m_show_palettes.position = ImVec2(0, 0);
    m_show_palettes.canvas_position = ImVec2(0, 0);
    m_blackboard = std::make_shared<robotik::bt::Blackboard>();
}

// ----------------------------------------------------------------------------
IDE::~IDE()
{
    onTeardown();
}

// ----------------------------------------------------------------------------
void IDE::onTeardown()
{
    if (m_server)
    {
        m_server->stop();
        m_server.reset();
    }

    if (m_renderer)
    {
        m_renderer->shutdown();
        m_renderer.reset();
    }
}

// ----------------------------------------------------------------------------
void IDE::onDrawMenuBar()
{
    if (ImGui::BeginMenu("File"))
    {
        // Open file dialog to load a YAML file
        if (ImGui::MenuItem("Load Behavior Tree", "Ctrl+O"))
        {
            IGFD::FileDialogConfig config;
            config.path = ".";
            config.countSelectionMax = 1;
            config.flags = ImGuiFileDialogFlags_Modal;
            ImGuiFileDialog::Instance()->OpenDialog(
                "LoadYamlDlgKey", "Choose YAML File", ".yaml,.yml", config);
        }

        // Open file dialog to save a YAML file
        if (ImGui::MenuItem(
                "Save As...", "Ctrl+S", false, m_mode == Mode::Creation))
        {
            IGFD::FileDialogConfig config;
            config.path = ".";
            config.countSelectionMax = 1;
            config.flags = ImGuiFileDialogFlags_Modal |
                           ImGuiFileDialogFlags_ConfirmOverwrite;
            ImGuiFileDialog::Instance()->OpenDialog(
                "SaveYamlDlgKey", "Save YAML File", ".yaml", config);
        }
        ImGui::Separator();

        // Quit the application
        if (ImGui::MenuItem("Quit", "Ctrl+Q"))
        {
            if (m_is_modified)
            {
                m_show_palettes.quit_confirmation = true;
            }
            else
            {
                halt();
            }
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit"))
    {
        // Auto-layout the nodes
        if (ImGui::MenuItem(
                "Auto Layout", "Ctrl+L", false, m_mode == Mode::Creation))
        {
            autoLayoutNodes();
        }

        ImGui::Separator();

        // Define the layout direction to display the tree (left to right)
        if (ImGui::MenuItem("Layout: Left to Right",
                            nullptr,
                            getCurrentTreeView().layout_direction ==
                                LayoutDirection::LeftToRight,
                            m_mode == Mode::Creation))
        {
            getCurrentTreeView().layout_direction =
                LayoutDirection::LeftToRight;
            autoLayoutNodes();
        }

        // Define the layout direction to display the tree (top to bottom)
        if (ImGui::MenuItem("Layout: Top to Bottom",
                            nullptr,
                            getCurrentTreeView().layout_direction ==
                                LayoutDirection::TopToBottom,
                            m_mode == Mode::Creation))
        {
            getCurrentTreeView().layout_direction =
                LayoutDirection::TopToBottom;
            autoLayoutNodes();
        }

        ImGui::Separator();

        // Add a new node
        if (ImGui::MenuItem(
                "Add Node", "Space", false, m_mode == Mode::Creation))
        {
            m_show_palettes.node_creation = true;
            m_show_palettes.position = ImGui::GetMousePos();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
        // Toggle blackboard panel visibility
        if (ImGui::MenuItem("Blackboard", nullptr, m_show_blackboard_panel))
        {
            m_show_blackboard_panel = !m_show_blackboard_panel;
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Mode"))
    {
        // Set the editor mode
        if (ImGui::MenuItem("Editor", nullptr, m_mode == Mode::Creation))
        {
            setMode(Mode::Creation);
        }

        // Set the real-time visualizer mode
        if (ImGui::MenuItem("Visualizer", nullptr, m_mode == Mode::Visualizer))
        {
            setMode(Mode::Visualizer);
        }
        ImGui::EndMenu();
    }
}

// ----------------------------------------------------------------------------
void IDE::setMode(Mode const p_mode)
{
    if (m_mode == p_mode)
        return;

    m_mode = p_mode;
    switch (m_mode)
    {
        case Mode::Visualizer:
            // Clear the current tree when entering visualizer mode
            reset();
            if (m_server && !m_server->isConnected())
            {
                m_server->start();
            }
            break;
        case Mode::Creation:
            if (m_server)
            {
                m_server->stop();
            }
            break;
        default:
            std::cout << "Unknown editor mode" << std::endl;
            break;
    }
}

// ----------------------------------------------------------------------------
void IDE::drawBehaviorTree()
{
    if (!m_renderer)
        return;

    // Get current view (non-const because we'll modify node_positions)
    TreeView& current_view = getCurrentTreeView();

    // Collect visible nodes from current root
    std::unordered_set<int> visible_node_ids;
    int current_root_id = current_view.root_id;

    // Debug: ensure we have a valid root_id
    if (current_root_id < 0)
    {
        // If no root_id, try to find one or use the first node
        // This shouldn't happen, but handle it gracefully
        return;
    }

    collectVisibleNodes(current_root_id, visible_node_ids);

    // Only include orphan nodes when viewing the main tree, not subtrees
    // This allows new unconnected nodes to be visible in the main tree
    if (!current_view.is_subtree)
    {
        // Build set of subtree root IDs (these should not appear in main tree)
        std::unordered_set<int> subtree_root_ids;
        for (const auto& [name, view] : m_tree_views)
        {
            if (view.is_subtree && view.root_id >= 0)
            {
                subtree_root_ids.insert(view.root_id);
            }
        }

        // Include orphan nodes (nodes without parents that aren't the root)
        // BUT exclude subtree definition roots
        for (const auto& [id, node] : m_nodes)
        {
            if (node.parent == -1 && id != current_root_id &&
                subtree_root_ids.find(id) == subtree_root_ids.end())
            {
                visible_node_ids.insert(id);
                // Also collect their children if any
                collectVisibleNodes(id, visible_node_ids);
            }
        }
    }

    // Filter nodes to only visible ones and sync positions from current view
    std::unordered_map<int, Node> visible_nodes;

    // Sync positions from current view's node_positions to nodes (for Renderer)
    // Also update m_nodes so positions are in sync
    for (const auto& [id, node] : m_nodes)
    {
        if (visible_node_ids.count(id) > 0)
        {
            visible_nodes[id] = node;
            // Copy position from current view's node_positions
            auto pos_it = current_view.node_positions.find(id);
            if (pos_it != current_view.node_positions.end())
            {
                visible_nodes[id].position = pos_it->second;
                // Also update the node in m_nodes to keep them in sync
                m_nodes[id].position = pos_it->second;
            }
            else
            {
                // No position stored, use default
                visible_nodes[id].position = ImVec2(0, 0);
                m_nodes[id].position = ImVec2(0, 0);
            }
        }
    }

    // Generate links from parent-child relationships
    std::vector<Link> visible_links;
    int link_id = 0;
    for (const auto& [id, node] : visible_nodes)
    {
        for (int child_id : node.children)
        {
            if (visible_node_ids.count(child_id) > 0)
            {
                visible_links.push_back({link_id++, id, child_id});
            }
        }
    }

    // Render the graph
    bool const is_edit_mode = (m_mode == Mode::Creation);
    IDE::LayoutDirection layout_dir = getCurrentTreeView().layout_direction;
    int layout_dir_int = static_cast<int>(layout_dir);
    m_renderer->drawBehaviorTree(visible_nodes,
                                 visible_links,
                                 layout_dir_int,
                                 m_blackboard.get(),
                                 !is_edit_mode);

    // Save positions back to current view's node_positions (in case Renderer
    // modified them)
    for (auto& [id, node] : visible_nodes)
    {
        current_view.node_positions[id] = node.position;
    }
}

// ----------------------------------------------------------------------------
void IDE::collectVisibleNodes(int root_id,
                              std::unordered_set<int>& visible_nodes)
{
    Node* node = findNode(root_id);
    if (!node)
        return;

    // Add this node to visible set
    visible_nodes.insert(root_id);

    // Process children
    for (int child_id : node->children)
    {
        Node* child = findNode(child_id);
        if (!child)
            continue;

        // Check if this child is from a collapsed SubTree
        if (node->type == "SubTree" && !node->is_expanded)
        {
            // This is a collapsed SubTree, don't include its children
            continue;
        }

        // Recursively collect visible nodes
        collectVisibleNodes(child_id, visible_nodes);
    }
}

// ----------------------------------------------------------------------------
int IDE::addNode(std::string const& p_type, std::string const& p_name)
{
    int id = getNextNodeId();
    IDE::Node node;
    node.id = id;
    node.type = p_type;
    node.name = p_name;

    // Use the palette position if available (stored before palette was shown)
    // Check if we have a pending link to determine if node was created from
    // drag-drop
    m_nodes.emplace(id, std::move(node));

    // Set position in current view's node_positions
    ImVec2 initial_position;
    if (m_pending_link_from_node >= 0)
    {
        // Position at the link drop point (already in canvas coordinates)
        initial_position = m_show_palettes.canvas_position;
    }
    else
    {
        // Normal node creation - use palette position or current mouse position
        if (m_show_palettes.position.x != 0.0f ||
            m_show_palettes.position.y != 0.0f)
        {
            // Convert screen position (from palette window) to canvas
            // coordinates
            initial_position =
                m_renderer->convertScreenToCanvas(m_show_palettes.position);
        }
        else
        {
            // Convert screen mouse position to canvas coordinates
            ImVec2 screen_pos = ImGui::GetMousePos();
            initial_position = m_renderer->convertScreenToCanvas(screen_pos);
        }
    }
    setNodePosition(id, initial_position);

    if (m_nodes.size() == 1u)
    {
        getCurrentTreeView().root_id = id;
    }

    m_is_modified = true;

    std::cout << "Added node: " << p_type << " (" << p_name << ")" << std::endl;

    return id;
}

// ----------------------------------------------------------------------------
void IDE::addNodeAndLink(std::string const& p_type, std::string const& p_name)
{
    int new_node_id = addNode(p_type, p_name);

    // If we have a pending link, complete it now
    if (m_pending_link_from_node >= 0)
    {
        createLink(m_pending_link_from_node, new_node_id);
        m_pending_link_from_node = -1;
    }

    ImGui::CloseCurrentPopup();
}

// ----------------------------------------------------------------------------
void IDE::deleteNode(int const p_node_id)
{
    Node* node = findNode(p_node_id);
    if (node)
    {
        // Remove from parent's children list
        if (node->parent >= 0)
        {
            Node* parent = findNode(node->parent);
            if (parent)
            {
                auto& children = parent->children;
                children.erase(
                    std::remove(children.begin(), children.end(), p_node_id),
                    children.end());
            }
        }

        // Clear parent reference for all children
        for (int child_id : node->children)
        {
            Node* child = findNode(child_id);
            if (child)
            {
                child->parent = -1;
            }
        }
    }

    // Remove the node
    m_nodes.erase(p_node_id);

    // Update the root node if needed
    if (getCurrentTreeView().root_id == p_node_id)
    {
        getCurrentTreeView().root_id = -1;
    }

    m_is_modified = true;
}

// ----------------------------------------------------------------------------
void IDE::createLink(int const p_from_node, int const p_to_node)
{
    Node* from = findNode(p_from_node);
    Node* to = findNode(p_to_node);
    if (!from || !to)
        return;

    // Check if the link already exists
    if (std::find(from->children.begin(), from->children.end(), p_to_node) !=
        from->children.end())
    {
        return;
    }

    // Remove any existing parent of the target node
    // This allows replacing an existing connection by dragging a new link
    if (to->parent >= 0)
    {
        Node* old_parent = findNode(to->parent);
        if (old_parent)
        {
            auto& children = old_parent->children;
            children.erase(
                std::remove(children.begin(), children.end(), p_to_node),
                children.end());
        }
    }

    // Create the parent-child relationship
    from->children.push_back(p_to_node);
    to->parent = p_from_node;

    m_is_modified = true;

    // Emit signal
    onLinkCreated.emit(p_from_node, p_to_node);
}

// ----------------------------------------------------------------------------
void IDE::deleteLink(int const p_from_node, int const p_to_node)
{
    Node* from = findNode(p_from_node);
    Node* to = findNode(p_to_node);

    if (from && to)
    {
        // Remove from parent's children list
        auto& children = from->children;
        auto it = std::find(children.begin(), children.end(), p_to_node);
        if (it != children.end())
        {
            children.erase(it);
            to->parent = -1;

            m_is_modified = true;

            // Emit signal (using a combined ID for backwards compatibility)
            onLinkDeleted.emit(p_from_node * 10000 + p_to_node);
        }
    }
}

// ----------------------------------------------------------------------------
IDE::Node* IDE::findNode(int p_id)
{
    auto it = m_nodes.find(p_id);
    return it != m_nodes.end() ? &it->second : nullptr;
}

// ----------------------------------------------------------------------------
ImVec2 IDE::getNodePosition(ID node_id)
{
    TreeView& current_view = getCurrentTreeView();
    auto it = current_view.node_positions.find(node_id);
    if (it != current_view.node_positions.end())
    {
        return it->second;
    }
    return ImVec2(0, 0); // Default position
}

// ----------------------------------------------------------------------------
void IDE::setNodePosition(ID node_id, ImVec2 position)
{
    TreeView& current_view = getCurrentTreeView();
    current_view.node_positions[node_id] = position;
}

// ----------------------------------------------------------------------------
IDE::TreeView& IDE::getCurrentTreeView()
{
    // Ensure we always have at least one TreeView
    if (m_tree_views.empty())
    {
        // Create a default tree view with the current filepath name or "Main"
        std::string default_name =
            m_behavior_tree_filepath.empty()
                ? "Main"
                : extractFileNameWithoutExtension(m_behavior_tree_filepath);
        m_tree_views[default_name] = {
            default_name, false, -1, LayoutDirection::TopToBottom, {}};
        m_active_tree_name = default_name;
    }

    // Ensure active_tree_name is valid
    auto it = m_tree_views.find(m_active_tree_name);
    if (it == m_tree_views.end())
    {
        // Prefer a non-subtree (main tree) if available
        for (auto& [name, view] : m_tree_views)
        {
            if (!view.is_subtree)
            {
                m_active_tree_name = name;
                return view;
            }
        }
        // Fallback to first available
        m_active_tree_name = m_tree_views.begin()->first;
        it = m_tree_views.begin();
    }

    return it->second;
}

// ----------------------------------------------------------------------------
IDE::TreeView* IDE::findTreeViewByRootId(int root_id)
{
    for (auto& [name, view] : m_tree_views)
    {
        if (view.root_id == root_id)
        {
            return &view;
        }
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
void IDE::autoLayoutNodes()
{
    int root_id = getCurrentTreeView().root_id;
    if (root_id < 0)
        return;

    Node* root = findNode(root_id);
    if (!root)
        return;

    float maxExtent = 0;
    layoutNodeRecursive(root, 100.0f, 100.0f, maxExtent);
}

// ----------------------------------------------------------------------------
void IDE::layoutNodeRecursive(IDE::Node* p_node,
                              float p_x,
                              float p_y,
                              float& p_max_extent)
{
    if (!p_node)
        return;

    // Groot-style layout with proper spacing
    constexpr float NODE_HORIZONTAL_SPACING = 40.0f; // Reduced spacing
    constexpr float NODE_VERTICAL_SPACING = 60.0f;   // Reduced spacing
    constexpr float MIN_NODE_WIDTH = 150.0f;
    constexpr float MIN_NODE_HEIGHT = 80.0f;

    // Use minimum dimensions for layout
    ImVec2 node_size(MIN_NODE_WIDTH, MIN_NODE_HEIGHT);

    // Get layout direction from current view
    LayoutDirection layout_dir = getCurrentTreeView().layout_direction;

    // If the node has no children, set its position and return
    if (p_node->children.empty())
    {
        setNodePosition(p_node->id, ImVec2(p_x, p_y));

        if (layout_dir == LayoutDirection::LeftToRight)
        {
            p_max_extent = std::max(p_max_extent, p_y + node_size.y);
        }
        else // TopToBottom
        {
            p_max_extent = std::max(p_max_extent, p_x + node_size.x);
        }
        return;
    }

    // Layout children recursively first (post-order)
    float child_start_pos =
        (layout_dir == LayoutDirection::LeftToRight) ? p_y : p_x;

    std::vector<ImVec2> child_positions;

    for (size_t i = 0; i < p_node->children.size(); ++i)
    {
        Node* child = findNode(p_node->children[i]);
        if (child)
        {
            float child_extent_before = p_max_extent;

            if (layout_dir == LayoutDirection::LeftToRight)
            {
                // Left to right: parent to children goes right (X increases)
                // Siblings spread vertically (Y increases)
                float child_x = p_x + node_size.x + NODE_VERTICAL_SPACING;
                layoutNodeRecursive(
                    child, child_x, child_start_pos, p_max_extent);
                child_positions.push_back(getNodePosition(child->id));

                // Move to next sibling position vertically (use minimum
                // dimensions)
                child_start_pos += std::max(p_max_extent - child_extent_before,
                                            MIN_NODE_HEIGHT) +
                                   NODE_HORIZONTAL_SPACING;
            }
            else // TopToBottom
            {
                // Top to bottom: parent to children goes down (Y increases)
                // Siblings spread horizontally (X increases)
                float child_y = p_y + node_size.y + NODE_VERTICAL_SPACING;
                layoutNodeRecursive(
                    child, child_start_pos, child_y, p_max_extent);
                child_positions.push_back(getNodePosition(child->id));

                // Move to next sibling position horizontally (use minimum
                // dimensions)
                child_start_pos += std::max(p_max_extent - child_extent_before,
                                            MIN_NODE_WIDTH) +
                                   NODE_HORIZONTAL_SPACING;
            }
        }
    }

    // Center parent relative to children
    if (!child_positions.empty())
    {
        if (layout_dir == LayoutDirection::LeftToRight)
        {
            // Center vertically over children (children spread vertically)
            float min_child_y = child_positions[0].y;
            float max_child_y = child_positions[child_positions.size() - 1].y;
            // Add height of last child (use minimum)
            max_child_y += MIN_NODE_HEIGHT;
            float centerY =
                (min_child_y + max_child_y) / 2.0f - node_size.y / 2.0f;
            ImVec2 pos = ImVec2(p_x, std::max(p_y, centerY));
            setNodePosition(p_node->id, pos);
            p_max_extent = std::max(p_max_extent, pos.x + node_size.x);
        }
        else // TopToBottom
        {
            // Center horizontally over children (children spread horizontally)
            float min_child_x = child_positions[0].x;
            float max_child_x = child_positions[child_positions.size() - 1].x;
            // Add width of last child (use minimum)
            max_child_x += MIN_NODE_WIDTH;
            float centerX =
                (min_child_x + max_child_x) / 2.0f - node_size.x / 2.0f;
            ImVec2 pos = ImVec2(std::max(p_x, centerX), p_y);
            setNodePosition(p_node->id, pos);
            p_max_extent = std::max(p_max_extent, pos.y + node_size.y);
        }
    }
    else
    {
        setNodePosition(p_node->id, ImVec2(p_x, p_y));
    }
}

// ----------------------------------------------------------------------------
void IDE::toggleSubTreeExpansion(int node_id)
{
    IDE::Node* node = findNode(node_id);
    if (!node || node->type != "SubTree")
        return;

    bool success = false;
    if (node->is_expanded)
    {
        success = collapseSubTree(node);
    }
    else
    {
        success = expandSubTree(node);
    }

    // Only toggle the flag if the operation succeeded
    if (success)
    {
        node->is_expanded = !node->is_expanded;
    }
}

// ----------------------------------------------------------------------------
bool IDE::expandSubTree(IDE::Node* subtree_node)
{
    if (!subtree_node || subtree_node->subtree_reference.empty())
        return false;

    // Find the SubTree definition
    auto it = m_tree_views.find(subtree_node->subtree_reference);
    if (it == m_tree_views.end() || !it->second.is_subtree)
        return false;

    int subtree_root_id = it->second.root_id;
    Node* subtree_root = findNode(subtree_root_id);
    if (!subtree_root)
        return false;

    // Link the SubTree root as a child of the SubTree node
    if (std::find(subtree_node->children.begin(),
                  subtree_node->children.end(),
                  subtree_root_id) == subtree_node->children.end())
    {
        subtree_node->children.push_back(subtree_root_id);
        subtree_root->parent = subtree_node->id;
    }

    // Relayout after expansion
    autoLayoutNodes();
    return true;
}

// ----------------------------------------------------------------------------
bool IDE::collapseSubTree(IDE::Node* subtree_node)
{
    if (!subtree_node || subtree_node->subtree_reference.empty())
        return false;

    // Find the SubTree definition
    auto it = m_tree_views.find(subtree_node->subtree_reference);
    if (it == m_tree_views.end() || !it->second.is_subtree)
        return false;

    int subtree_root_id = it->second.root_id;

    // Remove the link between SubTree node and its expanded content
    auto child_it = std::find(subtree_node->children.begin(),
                              subtree_node->children.end(),
                              subtree_root_id);
    if (child_it != subtree_node->children.end())
    {
        subtree_node->children.erase(child_it);
    }

    Node* subtree_root = findNode(subtree_root_id);
    if (subtree_root)
    {
        subtree_root->parent = -1;
    }

    // Relayout after collapse
    autoLayoutNodes();
    return true;
}

// ----------------------------------------------------------------------------
void IDE::loadFromYaml(const std::string& p_filepath)
{
    std::cout << "Loading tree from: " << p_filepath << std::endl;

    try
    {
        YAML::Node yaml = YAML::LoadFile(p_filepath);

        // Clear existing data
        m_nodes.clear();
        m_tree_views.clear();
        m_unique_node_id = 1;
        m_selected_node_id = -1;
        m_active_tree_name.clear();
        m_dfs_node_order.clear();

        // Create a fresh blackboard and parse the Blackboard section
        m_blackboard = std::make_shared<robotik::bt::Blackboard>();
        if (yaml["Blackboard"])
        {
            robotik::bt::BlackboardSerializer::load(*m_blackboard, yaml["Blackboard"]);
            std::cout << "Loaded Blackboard with variables" << std::endl;
        }

        // Parse the BehaviorTree section
        if (yaml["BehaviorTree"])
        {
            YAML::Node tree_node = yaml["BehaviorTree"];
            int root_id = parseYamlNode(tree_node, -1);

            if (root_id < 0)
            {
                std::cerr << "Failed to parse BehaviorTree section"
                          << std::endl;
                return;
            }

            // Extract filename without extension for the tree name
            std::string tree_name = extractFileNameWithoutExtension(p_filepath);

            // Add main tree to views
            m_tree_views[tree_name] = {
                tree_name, false, root_id, LayoutDirection::TopToBottom, {}};
            m_active_tree_name = tree_name;
        }
        else
        {
            std::cerr << "No BehaviorTree section found in YAML" << std::endl;
            return;
        }

        // Parse SubTrees section
        if (yaml["SubTrees"])
        {
            YAML::Node subtrees = yaml["SubTrees"];
            for (auto it = subtrees.begin(); it != subtrees.end(); ++it)
            {
                std::string subtree_name = it->first.as<std::string>();
                YAML::Node subtree_def = it->second;

                // Parse the subtree
                int subtree_root_id = parseYamlNode(subtree_def, -1);

                if (subtree_root_id < 0)
                {
                    std::cerr << "Failed to parse SubTree: " << subtree_name
                              << std::endl;
                    continue;
                }

                // Add subtree to views
                m_tree_views[subtree_name] = {subtree_name,
                                              true,
                                              subtree_root_id,
                                              LayoutDirection::TopToBottom,
                                              {}};

                std::cout << "Loaded SubTree: " << subtree_name << std::endl;
            }
        }

        // Link SubTree nodes to their definitions
        for (auto& [id, node] : m_nodes)
        {
            if (node.type == "SubTree" && !node.subtree_reference.empty())
            {
                // Subtree reference already set during parsing
                std::cout << "SubTree node '" << node.name
                          << "' references: " << node.subtree_reference
                          << std::endl;
            }
        }

        // Auto-layout the nodes for each tree view
        // Save the current active name to restore it later
        std::string saved_active_name = m_active_tree_name;

        for (auto& [name, view] : m_tree_views)
        {
            // Switch to this view temporarily to layout its nodes
            m_active_tree_name = name;
            autoLayoutNodes();

            // Positions are already saved in node_positions by
            // setNodePosition() called in autoLayoutNodes(). We just need to
            // sync node.position for consistency with the Renderer
            std::unordered_set<ID> visible_nodes;
            if (view.root_id >= 0)
            {
                collectVisibleNodes(view.root_id, visible_nodes);
            }
            for (ID id : visible_nodes)
            {
                auto node_it = m_nodes.find(id);
                auto pos_it = view.node_positions.find(id);
                if (node_it != m_nodes.end() &&
                    pos_it != view.node_positions.end())
                {
                    // Sync node.position from node_positions (for Renderer
                    // compatibility)
                    node_it->second.position = pos_it->second;
                }
            }
        }

        // Restore the original active name
        m_active_tree_name = saved_active_name;

        m_is_modified = false;
        m_behavior_tree_filepath = p_filepath;

        // Count subtrees
        size_t subtree_count = 0;
        for (const auto& [name, view] : m_tree_views)
        {
            if (view.is_subtree)
                subtree_count++;
        }

        std::cout << "Tree loaded successfully: " << m_nodes.size()
                  << " nodes, " << subtree_count << " subtrees" << std::endl;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
    }
}

// ----------------------------------------------------------------------------
void IDE::loadFromYamlString(const std::string& p_yaml_content)
{
    std::cout << "Loading tree from YAML string" << std::endl;

    try
    {
        YAML::Node yaml = YAML::Load(p_yaml_content);

        // Clear existing data
        m_nodes.clear();
        m_tree_views.clear();
        m_unique_node_id = 1;
        m_selected_node_id = -1;
        m_active_tree_name.clear();
        m_dfs_node_order.clear();

        // Create a fresh blackboard
        m_blackboard = std::make_shared<robotik::bt::Blackboard>();
        if (yaml["Blackboard"])
        {
            robotik::bt::BlackboardSerializer::load(*m_blackboard, yaml["Blackboard"]);
            std::cout << "Loaded Blackboard with variables" << std::endl;
        }

        // Parse SubTrees section first (so they exist when linking)
        if (yaml["SubTrees"])
        {
            YAML::Node subtrees = yaml["SubTrees"];
            for (auto it = subtrees.begin(); it != subtrees.end(); ++it)
            {
                std::string subtree_name = it->first.as<std::string>();
                YAML::Node subtree_def = it->second;

                // Parse the subtree
                int subtree_root_id = parseYamlNode(subtree_def, -1);

                if (subtree_root_id < 0)
                {
                    std::cerr << "Failed to parse SubTree: " << subtree_name
                              << std::endl;
                    continue;
                }

                // Add subtree to views
                m_tree_views[subtree_name] = {subtree_name,
                                              true,
                                              subtree_root_id,
                                              LayoutDirection::TopToBottom,
                                              {}};

                std::cout << "Loaded SubTree: " << subtree_name << std::endl;
            }
        }

        // Parse the BehaviorTree section
        if (yaml["BehaviorTree"])
        {
            YAML::Node tree_node = yaml["BehaviorTree"];
            int root_id = parseYamlNode(tree_node, -1);

            if (root_id < 0)
            {
                std::cerr << "Failed to parse BehaviorTree section"
                          << std::endl;
                return;
            }

            // Use "Visualizer" as the tree name
            std::string tree_name = "Visualizer";

            // Add main tree to views
            m_tree_views[tree_name] = {
                tree_name, false, root_id, LayoutDirection::TopToBottom, {}};
            m_active_tree_name = tree_name;
        }
        else
        {
            std::cerr << "No BehaviorTree section found in YAML" << std::endl;
            return;
        }

        // Link SubTree nodes to their definitions
        for (auto& [id, node] : m_nodes)
        {
            if (node.type == "SubTree" && !node.subtree_reference.empty())
            {
                std::cout << "SubTree node '" << node.name
                          << "' references: " << node.subtree_reference
                          << std::endl;
            }
        }

        // Auto-layout the nodes
        autoLayoutNodes();

        m_is_modified = false;

        std::cout << "Tree loaded from string: " << m_nodes.size() << " nodes, "
                  << m_dfs_node_order.size() << " in DFS order" << std::endl;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
    }
}

void IDE::saveToYaml(const std::string& p_filepath)
{
    std::cout << "Saving tree to: " << p_filepath << std::endl;

    int root_id = getCurrentTreeView().root_id;
    if (root_id < 0)
    {
        std::cerr << "No root node to save" << std::endl;
        return;
    }

    Node* root = findNode(root_id);
    if (!root)
    {
        std::cerr << "Root node not found" << std::endl;
        return;
    }

    std::ofstream file(p_filepath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << p_filepath
                  << std::endl;
        return;
    }

    YAML::Emitter out;

    // Save main BehaviorTree
    out << YAML::BeginMap;

    // Save Blackboard section using the serializer
    if (m_blackboard)
    {
        YAML::Node blackboard_yaml =
            robotik::bt::BlackboardSerializer::dump(*m_blackboard);
        if (blackboard_yaml.size() > 0)
        {
            out << YAML::Key << "Blackboard";
            out << YAML::Value << blackboard_yaml;
        }
    }

    out << YAML::Key << "BehaviorTree";
    out << YAML::Value;
    serializeNodeToYaml(out, root);

    // Save SubTrees if any
    bool has_subtrees = false;
    for (const auto& [name, view] : m_tree_views)
    {
        if (view.is_subtree)
        {
            has_subtrees = true;
            break;
        }
    }

    if (has_subtrees)
    {
        out << YAML::Key << "SubTrees";
        out << YAML::Value << YAML::BeginMap;

        for (const auto& [name, view] : m_tree_views)
        {
            if (view.is_subtree && view.root_id >= 0)
            {
                Node* subtree_root = findNode(view.root_id);
                if (subtree_root)
                {
                    out << YAML::Key << name;
                    out << YAML::Value;
                    // Pass true to indicate this is a subtree definition
                    serializeNodeToYaml(out, subtree_root, true);
                }
            }
        }

        out << YAML::EndMap;
    }

    out << YAML::EndMap;

    file << out.c_str();
    file.close();

    m_is_modified = false;
    m_behavior_tree_filepath = p_filepath;

    std::cout << "Tree saved successfully to: " << p_filepath << std::endl;
}

// ============================================================================
// Tree Conversion
// ============================================================================

void IDE::buildTreeFromNodes()
{
    // TODO: Pour une future version si nécessaire
    // Pour l'instant, la sauvegarde utilise directement les Node
}

void IDE::buildNodesFromTree(robotik::bt::Node& p_root)
{
    buildNodesFromTreeRecursive(p_root, -1);
    autoLayoutNodes();
}

int IDE::buildNodesFromTreeRecursive(robotik::bt::Node& p_node, int p_parent_id)
{
    int node_id = getNextNodeId();
    IDE::Node editor_node;
    editor_node.id = node_id;
    editor_node.type = p_node.type();
    editor_node.name = p_node.name;
    editor_node.parent = p_parent_id;

    // Pour Composite nodes, récupérer les enfants
    if (auto composite = dynamic_cast<robotik::bt::Composite*>(&p_node))
    {
        for (auto& child : composite->getChildren())
        {
            int child_id = buildNodesFromTreeRecursive(*child, node_id);
            if (child_id >= 0)
            {
                editor_node.children.push_back(child_id);
            }
        }
    }
    // Pour Decorator nodes
    else if (auto decorator = dynamic_cast<robotik::bt::Decorator*>(&p_node))
    {
        if (decorator->hasChild())
        {
            int child_id =
                buildNodesFromTreeRecursive(decorator->getChild(), node_id);
            if (child_id >= 0)
            {
                editor_node.children.push_back(child_id);
            }
        }
    }

    // Utiliser emplace au lieu de operator[] pour éviter le constructeur
    // par défaut
    m_nodes.emplace(node_id, std::move(editor_node));

    if (p_parent_id < 0)
    {
        getCurrentTreeView().root_id = node_id;
    }

    return node_id;
}

void IDE::serializeNodeToYaml(YAML::Emitter& p_out,
                              IDE::Node* p_node,
                              bool is_subtree_definition)
{
    if (!p_node)
        return;

    p_out << YAML::BeginMap;
    p_out << YAML::Key << p_node->type;
    p_out << YAML::Value << YAML::BeginMap;
    p_out << YAML::Key << "name" << YAML::Value << p_node->name;

    // For SubTree nodes, save the reference (only in main tree, not in
    // definitions)
    if (p_node->type == "SubTree" && !p_node->subtree_reference.empty() &&
        !is_subtree_definition)
    {
        p_out << YAML::Key << "reference" << YAML::Value
              << p_node->subtree_reference;
    }

    // Save inputs as parameters with ${variable} reference format
    if (!p_node->inputs.empty())
    {
        p_out << YAML::Key << "inputs" << YAML::Value << YAML::BeginMap;
        for (const auto& input : p_node->inputs)
        {
            p_out << YAML::Key << input << YAML::Value << ("${" + input + "}");
        }
        p_out << YAML::EndMap;
    }

    // Save outputs as separate section
    if (!p_node->outputs.empty())
    {
        p_out << YAML::Key << "outputs" << YAML::Value << YAML::BeginMap;
        for (const auto& output : p_node->outputs)
        {
            p_out << YAML::Key << output << YAML::Value
                  << ("${" + output + "}");
        }
        p_out << YAML::EndMap;
    }

    // Collect children to save
    std::vector<int> children_to_save;
    for (int child_id : p_node->children)
    {
        Node* child = findNode(child_id);
        if (child)
        {
            // When serializing subtree definitions, save all children
            // When serializing main tree, skip expanded inline subtree children
            bool skip_child = false;

            if (!is_subtree_definition && p_node->type == "SubTree" &&
                p_node->is_expanded)
            {
                // Check if this child is the expanded subtree root
                auto subtree_it = m_tree_views.find(p_node->subtree_reference);
                if (subtree_it != m_tree_views.end() &&
                    subtree_it->second.root_id == child_id)
                {
                    skip_child = true;
                }
            }

            if (!skip_child)
            {
                children_to_save.push_back(child_id);
            }
        }
    }

    if (!children_to_save.empty())
    {
        p_out << YAML::Key << "children" << YAML::Value << YAML::BeginSeq;
        for (int child_id : children_to_save)
        {
            Node* child = findNode(child_id);
            if (child)
            {
                // Propagate the is_subtree_definition flag to children
                serializeNodeToYaml(p_out, child, is_subtree_definition);
            }
        }
        p_out << YAML::EndSeq;
    }

    p_out << YAML::EndMap;
    p_out << YAML::EndMap;
}

// ----------------------------------------------------------------------------
int IDE::parseYamlNode(const YAML::Node& p_yaml_node, int p_parent_id)
{
    if (!p_yaml_node.IsMap())
        return -1;

    // The YAML node should have exactly one key which is the node type
    if (p_yaml_node.size() == 0)
        return -1;

    // Get the first (and should be only) key-value pair
    auto it = p_yaml_node.begin();
    std::string node_type = it->first.as<std::string>();
    YAML::Node node_data = it->second;

    // Create the editor node
    int node_id = getNextNodeId();
    std::string node_name = node_type;

    // Extract name if provided
    if (node_data["name"])
    {
        node_name = node_data["name"].as<std::string>();
    }

    IDE::Node editor_node;
    editor_node.id = node_id;
    editor_node.type = node_type;
    editor_node.name = node_name;
    editor_node.parent = p_parent_id;

    // Add to DFS order immediately after creation (for visualizer mode)
    m_dfs_node_order.push_back(node_id);

    // For SubTree nodes, extract reference
    if (editor_node.type == "SubTree" && node_data["reference"])
    {
        editor_node.subtree_reference =
            node_data["reference"].as<std::string>();
    }

    // Extract inputs
    if (node_data["inputs"])
    {
        YAML::Node inputs = node_data["inputs"];
        for (auto input_it = inputs.begin(); input_it != inputs.end();
             ++input_it)
        {
            std::string input_name = input_it->first.as<std::string>();
            editor_node.inputs.push_back(input_name);
        }
    }

    // Extract outputs
    if (node_data["outputs"])
    {
        YAML::Node outputs = node_data["outputs"];
        for (auto output_it = outputs.begin(); output_it != outputs.end();
             ++output_it)
        {
            std::string output_name = output_it->first.as<std::string>();
            editor_node.outputs.push_back(output_name);
        }
    }

    // Legacy: Extract parameters as inputs (for backward compatibility)
    if (node_data["parameters"])
    {
        YAML::Node params = node_data["parameters"];
        for (auto param_it = params.begin(); param_it != params.end();
             ++param_it)
        {
            std::string param_name = param_it->first.as<std::string>();
            // Only add if not already in inputs
            if (std::find(editor_node.inputs.begin(),
                          editor_node.inputs.end(),
                          param_name) == editor_node.inputs.end())
            {
                editor_node.inputs.push_back(param_name);
            }
        }
    }

    // Process children
    if (node_data["children"])
    {
        YAML::Node children = node_data["children"];
        if (children.IsSequence())
        {
            for (size_t i = 0; i < children.size(); ++i)
            {
                int child_id = parseYamlNode(children[i], node_id);
                if (child_id >= 0)
                {
                    editor_node.children.push_back(child_id);
                }
            }
        }
    }

    // Process child (for decorators)
    if (node_data["child"])
    {
        YAML::Node child = node_data["child"];
        if (child.IsSequence() && child.size() > 0)
        {
            int child_id = parseYamlNode(child[0], node_id);
            if (child_id >= 0)
            {
                editor_node.children.push_back(child_id);
            }
        }
    }

    // Add node to the map
    m_nodes.emplace(node_id, std::move(editor_node));

    return node_id;
}
