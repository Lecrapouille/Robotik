/**
 * @file BTEditor.cpp
 * @brief Behavior tree editor library implementation
 *
 * Based on Oakular from BlackThorn project.
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "Robotik/Renderer/BehaviorTree/BTEditor.hpp"
#include "Robotik/Renderer/BehaviorTree/BTRenderer.hpp"

#include <imgui_stdlib.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace robotik::renderer {

// ----------------------------------------------------------------------------
//! \brief Extract the filename without extension from a file path.
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
BTEditor::BTEditor()
{
}

// ----------------------------------------------------------------------------
BTEditor::~BTEditor()
{
    shutdown();
}

// ----------------------------------------------------------------------------
void BTEditor::init()
{
    m_renderer = std::make_unique<BTRenderer>();
    reset();
}

// ----------------------------------------------------------------------------
void BTEditor::shutdown()
{
    if (m_renderer)
    {
        m_renderer->shutdown();
        m_renderer.reset();
    }
}

// ----------------------------------------------------------------------------
void BTEditor::reset()
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
    m_show_palettes.position = ImVec2(0, 0);
    m_show_palettes.canvas_position = ImVec2(0, 0);
    m_blackboard = std::make_shared<bt::Blackboard>();
}

// ----------------------------------------------------------------------------
void BTEditor::draw(const char* p_title)
{
    if (p_title != nullptr)
    {
        ImGui::Begin(p_title, nullptr, ImGuiWindowFlags_MenuBar);
        drawMenuBar();
    }

    showEditorTabs();
    showAddNodePalette();

    if (p_title != nullptr)
    {
        ImGui::End();
    }

    showBlackboardPanel();
}

// ----------------------------------------------------------------------------
void BTEditor::drawMenuBar()
{
    if (!ImGui::BeginMenuBar())
        return;

    if (ImGui::BeginMenu("Edit"))
    {
        if (ImGui::MenuItem("Auto Layout", "Ctrl+L"))
        {
            autoLayoutNodes();
        }

        ImGui::Separator();

        if (ImGui::MenuItem("Layout: Left to Right",
                            nullptr,
                            getCurrentTreeView().layout_direction ==
                                LayoutDirection::LeftToRight))
        {
            getCurrentTreeView().layout_direction =
                LayoutDirection::LeftToRight;
            autoLayoutNodes();
        }

        if (ImGui::MenuItem("Layout: Top to Bottom",
                            nullptr,
                            getCurrentTreeView().layout_direction ==
                                LayoutDirection::TopToBottom))
        {
            getCurrentTreeView().layout_direction =
                LayoutDirection::TopToBottom;
            autoLayoutNodes();
        }

        ImGui::Separator();

        if (ImGui::MenuItem("Add Node", "Space"))
        {
            m_show_palettes.node_creation = true;
            m_show_palettes.position = ImGui::GetMousePos();
        }
        ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
        if (ImGui::MenuItem("Blackboard", nullptr, m_show_blackboard_panel))
        {
            m_show_blackboard_panel = !m_show_blackboard_panel;
        }
        ImGui::EndMenu();
    }

    ImGui::EndMenuBar();
}

// ----------------------------------------------------------------------------
void BTEditor::drawBehaviorTree()
{
    if (!m_renderer)
        return;

    TreeView& current_view = getCurrentTreeView();

    std::unordered_set<int> visible_node_ids;
    int current_root_id = current_view.root_id;

    if (current_root_id < 0)
    {
        return;
    }

    collectVisibleNodes(current_root_id, visible_node_ids);

    if (!current_view.is_subtree)
    {
        std::unordered_set<int> subtree_root_ids;
        for (const auto& [name, view] : m_tree_views)
        {
            if (view.is_subtree && view.root_id >= 0)
            {
                subtree_root_ids.insert(view.root_id);
            }
        }

        for (const auto& [id, node] : m_nodes)
        {
            if (node.parent == -1 && id != current_root_id &&
                subtree_root_ids.find(id) == subtree_root_ids.end())
            {
                visible_node_ids.insert(id);
                collectVisibleNodes(id, visible_node_ids);
            }
        }
    }

    std::unordered_map<int, Node> visible_nodes;

    for (const auto& [id, node] : m_nodes)
    {
        if (visible_node_ids.count(id) > 0)
        {
            visible_nodes[id] = node;
            auto pos_it = current_view.node_positions.find(id);
            if (pos_it != current_view.node_positions.end())
            {
                visible_nodes[id].position = pos_it->second;
                m_nodes[id].position = pos_it->second;
            }
            else
            {
                visible_nodes[id].position = ImVec2(0, 0);
                m_nodes[id].position = ImVec2(0, 0);
            }
        }
    }

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

    bool const is_edit_mode = true;
    LayoutDirection layout_dir = getCurrentTreeView().layout_direction;
    int layout_dir_int = static_cast<int>(layout_dir);
    m_renderer->drawBehaviorTree(visible_nodes,
                                 visible_links,
                                 layout_dir_int,
                                 m_blackboard.get(),
                                 !is_edit_mode);

    for (auto& [id, node] : visible_nodes)
    {
        current_view.node_positions[id] = node.position;
    }
}

// ----------------------------------------------------------------------------
void BTEditor::collectVisibleNodes(int root_id,
                                   std::unordered_set<int>& visible_nodes)
{
    Node* node = findNode(root_id);
    if (!node)
        return;

    visible_nodes.insert(root_id);

    for (int child_id : node->children)
    {
        Node* child = findNode(child_id);
        if (!child)
            continue;

        if (node->type == "SubTree" && !node->is_expanded)
        {
            continue;
        }

        collectVisibleNodes(child_id, visible_nodes);
    }
}

// ----------------------------------------------------------------------------
int BTEditor::addNode(std::string const& p_type, std::string const& p_name)
{
    int id = getNextNodeId();
    Node node;
    node.id = id;
    node.type = p_type;
    node.name = p_name;

    m_nodes.emplace(id, std::move(node));

    ImVec2 initial_position;
    if (m_pending_link_from_node >= 0)
    {
        initial_position = m_show_palettes.canvas_position;
    }
    else
    {
        if (m_show_palettes.position.x != 0.0f ||
            m_show_palettes.position.y != 0.0f)
        {
            initial_position =
                m_renderer->convertScreenToCanvas(m_show_palettes.position);
        }
        else
        {
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
void BTEditor::addNodeAndLink(std::string const& p_type, std::string const& p_name)
{
    int new_node_id = addNode(p_type, p_name);

    if (m_pending_link_from_node >= 0)
    {
        createLink(m_pending_link_from_node, new_node_id);
        m_pending_link_from_node = -1;
    }

    ImGui::CloseCurrentPopup();
}

// ----------------------------------------------------------------------------
void BTEditor::deleteNode(int const p_node_id)
{
    Node* node = findNode(p_node_id);
    if (node)
    {
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

        for (int child_id : node->children)
        {
            Node* child = findNode(child_id);
            if (child)
            {
                child->parent = -1;
            }
        }
    }

    m_nodes.erase(p_node_id);

    if (getCurrentTreeView().root_id == p_node_id)
    {
        getCurrentTreeView().root_id = -1;
    }

    m_is_modified = true;
}

// ----------------------------------------------------------------------------
void BTEditor::createLink(int const p_from_node, int const p_to_node)
{
    Node* from = findNode(p_from_node);
    Node* to = findNode(p_to_node);
    if (!from || !to)
        return;

    if (std::find(from->children.begin(), from->children.end(), p_to_node) !=
        from->children.end())
    {
        return;
    }

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

    from->children.push_back(p_to_node);
    to->parent = p_from_node;

    m_is_modified = true;

    onLinkCreated.emit(p_from_node, p_to_node);
}

// ----------------------------------------------------------------------------
void BTEditor::deleteLink(int const p_from_node, int const p_to_node)
{
    Node* from = findNode(p_from_node);
    Node* to = findNode(p_to_node);

    if (from && to)
    {
        auto& children = from->children;
        auto it = std::find(children.begin(), children.end(), p_to_node);
        if (it != children.end())
        {
            children.erase(it);
            to->parent = -1;

            m_is_modified = true;

            onLinkDeleted.emit(p_from_node * 10000 + p_to_node);
        }
    }
}

// ----------------------------------------------------------------------------
BTEditor::Node* BTEditor::findNode(int p_id)
{
    auto it = m_nodes.find(p_id);
    return it != m_nodes.end() ? &it->second : nullptr;
}

// ----------------------------------------------------------------------------
ImVec2 BTEditor::getNodePosition(ID node_id)
{
    TreeView& current_view = getCurrentTreeView();
    auto it = current_view.node_positions.find(node_id);
    if (it != current_view.node_positions.end())
    {
        return it->second;
    }
    return ImVec2(0, 0);
}

// ----------------------------------------------------------------------------
void BTEditor::setNodePosition(ID node_id, ImVec2 position)
{
    TreeView& current_view = getCurrentTreeView();
    current_view.node_positions[node_id] = position;
}

// ----------------------------------------------------------------------------
BTEditor::TreeView& BTEditor::getCurrentTreeView()
{
    if (m_tree_views.empty())
    {
        std::string default_name =
            m_behavior_tree_filepath.empty()
                ? "Main"
                : extractFileNameWithoutExtension(m_behavior_tree_filepath);
        m_tree_views[default_name] = {
            default_name, false, -1, LayoutDirection::TopToBottom, {}};
        m_active_tree_name = default_name;
    }

    auto it = m_tree_views.find(m_active_tree_name);
    if (it == m_tree_views.end())
    {
        for (auto& [name, view] : m_tree_views)
        {
            if (!view.is_subtree)
            {
                m_active_tree_name = name;
                return view;
            }
        }
        m_active_tree_name = m_tree_views.begin()->first;
        it = m_tree_views.begin();
    }

    return it->second;
}

// ----------------------------------------------------------------------------
BTEditor::TreeView* BTEditor::findTreeViewByRootId(int root_id)
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
void BTEditor::autoLayoutNodes()
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
void BTEditor::layoutNodeRecursive(Node* p_node,
                                   float p_x,
                                   float p_y,
                                   float& p_max_extent)
{
    if (!p_node)
        return;

    constexpr float NODE_HORIZONTAL_SPACING = 40.0f;
    constexpr float NODE_VERTICAL_SPACING = 60.0f;
    constexpr float MIN_NODE_WIDTH = 150.0f;
    constexpr float MIN_NODE_HEIGHT = 80.0f;

    ImVec2 node_size(MIN_NODE_WIDTH, MIN_NODE_HEIGHT);

    LayoutDirection layout_dir = getCurrentTreeView().layout_direction;

    if (p_node->children.empty())
    {
        setNodePosition(p_node->id, ImVec2(p_x, p_y));

        if (layout_dir == LayoutDirection::LeftToRight)
        {
            p_max_extent = std::max(p_max_extent, p_y + node_size.y);
        }
        else
        {
            p_max_extent = std::max(p_max_extent, p_x + node_size.x);
        }
        return;
    }

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
                float child_x = p_x + node_size.x + NODE_VERTICAL_SPACING;
                layoutNodeRecursive(
                    child, child_x, child_start_pos, p_max_extent);
                child_positions.push_back(getNodePosition(child->id));

                child_start_pos += std::max(p_max_extent - child_extent_before,
                                            MIN_NODE_HEIGHT) +
                                   NODE_HORIZONTAL_SPACING;
            }
            else
            {
                float child_y = p_y + node_size.y + NODE_VERTICAL_SPACING;
                layoutNodeRecursive(
                    child, child_start_pos, child_y, p_max_extent);
                child_positions.push_back(getNodePosition(child->id));

                child_start_pos += std::max(p_max_extent - child_extent_before,
                                            MIN_NODE_WIDTH) +
                                   NODE_HORIZONTAL_SPACING;
            }
        }
    }

    if (!child_positions.empty())
    {
        if (layout_dir == LayoutDirection::LeftToRight)
        {
            float min_child_y = child_positions[0].y;
            float max_child_y = child_positions[child_positions.size() - 1].y;
            max_child_y += MIN_NODE_HEIGHT;
            float centerY =
                (min_child_y + max_child_y) / 2.0f - node_size.y / 2.0f;
            ImVec2 pos = ImVec2(p_x, std::max(p_y, centerY));
            setNodePosition(p_node->id, pos);
            p_max_extent = std::max(p_max_extent, pos.x + node_size.x);
        }
        else
        {
            float min_child_x = child_positions[0].x;
            float max_child_x = child_positions[child_positions.size() - 1].x;
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
void BTEditor::toggleSubTreeExpansion(int node_id)
{
    Node* node = findNode(node_id);
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

    if (success)
    {
        node->is_expanded = !node->is_expanded;
    }
}

// ----------------------------------------------------------------------------
bool BTEditor::expandSubTree(Node* subtree_node)
{
    if (!subtree_node || subtree_node->subtree_reference.empty())
        return false;

    auto it = m_tree_views.find(subtree_node->subtree_reference);
    if (it == m_tree_views.end() || !it->second.is_subtree)
        return false;

    int subtree_root_id = it->second.root_id;
    Node* subtree_root = findNode(subtree_root_id);
    if (!subtree_root)
        return false;

    if (std::find(subtree_node->children.begin(),
                  subtree_node->children.end(),
                  subtree_root_id) == subtree_node->children.end())
    {
        subtree_node->children.push_back(subtree_root_id);
        subtree_root->parent = subtree_node->id;
    }

    autoLayoutNodes();
    return true;
}

// ----------------------------------------------------------------------------
bool BTEditor::collapseSubTree(Node* subtree_node)
{
    if (!subtree_node || subtree_node->subtree_reference.empty())
        return false;

    auto it = m_tree_views.find(subtree_node->subtree_reference);
    if (it == m_tree_views.end() || !it->second.is_subtree)
        return false;

    int subtree_root_id = it->second.root_id;

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

    autoLayoutNodes();
    return true;
}

// ----------------------------------------------------------------------------
void BTEditor::loadFromYaml(const std::string& p_filepath)
{
    std::cout << "Loading tree from: " << p_filepath << std::endl;

    try
    {
        YAML::Node yaml = YAML::LoadFile(p_filepath);

        m_nodes.clear();
        m_tree_views.clear();
        m_unique_node_id = 1;
        m_selected_node_id = -1;
        m_active_tree_name.clear();

        m_blackboard = std::make_shared<bt::Blackboard>();
        if (yaml["Blackboard"])
        {
            bt::BlackboardSerializer::load(*m_blackboard, yaml["Blackboard"]);
            std::cout << "Loaded Blackboard with variables" << std::endl;
        }

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

            std::string tree_name = extractFileNameWithoutExtension(p_filepath);

            m_tree_views[tree_name] = {
                tree_name, false, root_id, LayoutDirection::TopToBottom, {}};
            m_active_tree_name = tree_name;
        }
        else
        {
            std::cerr << "No BehaviorTree section found in YAML" << std::endl;
            return;
        }

        if (yaml["SubTrees"])
        {
            YAML::Node subtrees = yaml["SubTrees"];
            for (auto it = subtrees.begin(); it != subtrees.end(); ++it)
            {
                std::string subtree_name = it->first.as<std::string>();
                YAML::Node subtree_def = it->second;

                int subtree_root_id = parseYamlNode(subtree_def, -1);

                if (subtree_root_id < 0)
                {
                    std::cerr << "Failed to parse SubTree: " << subtree_name
                              << std::endl;
                    continue;
                }

                m_tree_views[subtree_name] = {subtree_name,
                                              true,
                                              subtree_root_id,
                                              LayoutDirection::TopToBottom,
                                              {}};

                std::cout << "Loaded SubTree: " << subtree_name << std::endl;
            }
        }

        for (auto& [id, node] : m_nodes)
        {
            if (node.type == "SubTree" && !node.subtree_reference.empty())
            {
                std::cout << "SubTree node '" << node.name
                          << "' references: " << node.subtree_reference
                          << std::endl;
            }
        }

        std::string saved_active_name = m_active_tree_name;

        for (auto& [name, view] : m_tree_views)
        {
            m_active_tree_name = name;
            autoLayoutNodes();

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
                    node_it->second.position = pos_it->second;
                }
            }
        }

        m_active_tree_name = saved_active_name;

        m_is_modified = false;
        m_behavior_tree_filepath = p_filepath;

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
void BTEditor::loadFromYamlString(const std::string& p_yaml_content)
{
    std::cout << "Loading tree from YAML string" << std::endl;

    try
    {
        YAML::Node yaml = YAML::Load(p_yaml_content);

        m_nodes.clear();
        m_tree_views.clear();
        m_unique_node_id = 1;
        m_selected_node_id = -1;
        m_active_tree_name.clear();

        m_blackboard = std::make_shared<bt::Blackboard>();
        if (yaml["Blackboard"])
        {
            bt::BlackboardSerializer::load(*m_blackboard, yaml["Blackboard"]);
            std::cout << "Loaded Blackboard with variables" << std::endl;
        }

        if (yaml["SubTrees"])
        {
            YAML::Node subtrees = yaml["SubTrees"];
            for (auto it = subtrees.begin(); it != subtrees.end(); ++it)
            {
                std::string subtree_name = it->first.as<std::string>();
                YAML::Node subtree_def = it->second;

                int subtree_root_id = parseYamlNode(subtree_def, -1);

                if (subtree_root_id < 0)
                {
                    std::cerr << "Failed to parse SubTree: " << subtree_name
                              << std::endl;
                    continue;
                }

                m_tree_views[subtree_name] = {subtree_name,
                                              true,
                                              subtree_root_id,
                                              LayoutDirection::TopToBottom,
                                              {}};

                std::cout << "Loaded SubTree: " << subtree_name << std::endl;
            }
        }

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

            std::string tree_name = "Main";

            m_tree_views[tree_name] = {
                tree_name, false, root_id, LayoutDirection::TopToBottom, {}};
            m_active_tree_name = tree_name;
        }
        else
        {
            std::cerr << "No BehaviorTree section found in YAML" << std::endl;
            return;
        }

        for (auto& [id, node] : m_nodes)
        {
            if (node.type == "SubTree" && !node.subtree_reference.empty())
            {
                std::cout << "SubTree node '" << node.name
                          << "' references: " << node.subtree_reference
                          << std::endl;
            }
        }

        autoLayoutNodes();

        m_is_modified = false;

        std::cout << "Tree loaded from string: " << m_nodes.size() << " nodes"
                  << std::endl;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML parsing error: " << e.what() << std::endl;
    }
}

// ----------------------------------------------------------------------------
void BTEditor::saveToYaml(const std::string& p_filepath)
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

    out << YAML::BeginMap;

    if (m_blackboard)
    {
        YAML::Node blackboard_yaml =
            bt::BlackboardSerializer::dump(*m_blackboard);
        if (blackboard_yaml.size() > 0)
        {
            out << YAML::Key << "Blackboard";
            out << YAML::Value << blackboard_yaml;
        }
    }

    out << YAML::Key << "BehaviorTree";
    out << YAML::Value;
    serializeNodeToYaml(out, root);

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

void BTEditor::buildTreeFromNodes()
{
}

void BTEditor::buildNodesFromTree(bt::Node& p_root)
{
    buildNodesFromTreeRecursive(p_root, -1);
    autoLayoutNodes();
}

int BTEditor::buildNodesFromTreeRecursive(bt::Node& p_node, int p_parent_id)
{
    int node_id = getNextNodeId();
    Node editor_node;
    editor_node.id = node_id;
    editor_node.type = p_node.type();
    editor_node.name = p_node.name;
    editor_node.parent = p_parent_id;

    if (auto composite = dynamic_cast<bt::Composite*>(&p_node))
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
    else if (auto decorator = dynamic_cast<bt::Decorator*>(&p_node))
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

    m_nodes.emplace(node_id, std::move(editor_node));

    if (p_parent_id < 0)
    {
        getCurrentTreeView().root_id = node_id;
    }

    return node_id;
}

void BTEditor::serializeNodeToYaml(YAML::Emitter& p_out,
                                   Node* p_node,
                                   bool is_subtree_definition)
{
    if (!p_node)
        return;

    p_out << YAML::BeginMap;
    p_out << YAML::Key << p_node->type;
    p_out << YAML::Value << YAML::BeginMap;
    p_out << YAML::Key << "name" << YAML::Value << p_node->name;

    if (p_node->type == "SubTree" && !p_node->subtree_reference.empty() &&
        !is_subtree_definition)
    {
        p_out << YAML::Key << "reference" << YAML::Value
              << p_node->subtree_reference;
    }

    if (!p_node->inputs.empty())
    {
        p_out << YAML::Key << "inputs" << YAML::Value << YAML::BeginMap;
        for (const auto& input : p_node->inputs)
        {
            p_out << YAML::Key << input << YAML::Value << ("${" + input + "}");
        }
        p_out << YAML::EndMap;
    }

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

    std::vector<int> children_to_save;
    for (int child_id : p_node->children)
    {
        Node* child = findNode(child_id);
        if (child)
        {
            bool skip_child = false;

            if (!is_subtree_definition && p_node->type == "SubTree" &&
                p_node->is_expanded)
            {
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
                serializeNodeToYaml(p_out, child, is_subtree_definition);
            }
        }
        p_out << YAML::EndSeq;
    }

    p_out << YAML::EndMap;
    p_out << YAML::EndMap;
}

// ----------------------------------------------------------------------------
int BTEditor::parseYamlNode(const YAML::Node& p_yaml_node, int p_parent_id)
{
    if (!p_yaml_node.IsMap())
        return -1;

    if (p_yaml_node.size() == 0)
        return -1;

    auto it = p_yaml_node.begin();
    std::string node_type = it->first.as<std::string>();
    YAML::Node node_data = it->second;

    int node_id = getNextNodeId();
    std::string node_name = node_type;

    if (node_data["name"])
    {
        node_name = node_data["name"].as<std::string>();
    }

    Node editor_node;
    editor_node.id = node_id;
    editor_node.type = node_type;
    editor_node.name = node_name;
    editor_node.parent = p_parent_id;

    if (editor_node.type == "SubTree" && node_data["reference"])
    {
        editor_node.subtree_reference =
            node_data["reference"].as<std::string>();
    }

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

    if (node_data["parameters"])
    {
        YAML::Node params = node_data["parameters"];
        for (auto param_it = params.begin(); param_it != params.end();
             ++param_it)
        {
            std::string param_name = param_it->first.as<std::string>();
            if (std::find(editor_node.inputs.begin(),
                          editor_node.inputs.end(),
                          param_name) == editor_node.inputs.end())
            {
                editor_node.inputs.push_back(param_name);
            }
        }
    }

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

    m_nodes.emplace(node_id, std::move(editor_node));

    return node_id;
}

// ----------------------------------------------------------------------------
// Node types available for creation
// ----------------------------------------------------------------------------
static const std::array<const char*, 10> c_node_types = {
    "Action",   "Condition", "Failure",  "Inverter", "Parallel",
    "Repeater", "Selector",  "Sequence", "SubTree",  "Success"};

// ----------------------------------------------------------------------------
static bool canHaveBlackboardPorts(const std::string& p_type)
{
    return p_type == "Action" || p_type == "Condition" || p_type == "SubTree";
}

// ----------------------------------------------------------------------------
void BTEditor::showEditorTabs()
{
    if (m_tree_views.size() <= 1)
    {
        drawBehaviorTree();
        handleEditModeInteractions();
        return;
    }

    if (!ImGui::BeginTabBar("TreeTabs"))
        return;

    for (auto& [name, view] : m_tree_views)
    {
        if (!view.is_subtree)
        {
            drawTreeTab(name, view);
        }
    }

    for (auto& [name, view] : m_tree_views)
    {
        if (view.is_subtree)
        {
            drawTreeTab(name, view);
        }
    }

    m_request_tab_change = false;
    ImGui::EndTabBar();
}

// ----------------------------------------------------------------------------
void BTEditor::drawTreeTab(const std::string& name, TreeView& /*view*/)
{
    ImGuiTabItemFlags flags = ImGuiTabItemFlags_None;
    if (m_request_tab_change && m_active_tree_name == name)
    {
        flags = ImGuiTabItemFlags_SetSelected;
    }

    if (m_is_modified)
    {
        flags |= ImGuiTabItemFlags_UnsavedDocument;
    }

    if (ImGui::BeginTabItem(name.c_str(), nullptr, flags))
    {
        if (m_active_tree_name != name)
        {
            m_selected_node_id = -1;
            m_active_tree_name = name;
        }
        drawBehaviorTree();
        handleEditModeInteractions();
        ImGui::EndTabItem();
    }
}

// ----------------------------------------------------------------------------
void BTEditor::handleEditModeInteractions()
{
    if (ImGui::IsPopupOpen("Edit Node") || ImGui::IsPopupOpen("NodeContextMenu"))
    {
        showNodeContextMenu();
        showNodeEditPopup();
        return;
    }

    if (!m_renderer)
        return;

    int from_node, to_node;
    if (m_renderer->getLinkCreated(from_node, to_node))
    {
        createLink(from_node, to_node);
    }

    int link_from, link_to;
    if (m_renderer->getLinkDeleted(link_from, link_to))
    {
        deleteLink(link_from, link_to);
    }

    int toggled_node_id;
    if (m_renderer->getSubTreeToggled(toggled_node_id))
    {
        toggleSubTreeExpansion(toggled_node_id);
    }

    int link_void_from;
    if (m_renderer->getLinkDroppedInVoid(link_void_from))
    {
        m_show_palettes.node_creation = true;
        m_show_palettes.position = ImGui::GetMousePos();
        m_show_palettes.canvas_position =
            m_renderer->convertScreenToCanvas(m_show_palettes.position);
        m_pending_link_from_node = link_void_from;
    }

    int hovered_id = m_renderer->getHoveredNodeId();

    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && hovered_id >= 0)
    {
        m_selected_node_id = hovered_id;
        Node* node = findNode(hovered_id);
        if (node && node->type == "SubTree" && !node->subtree_reference.empty())
        {
            auto it = m_tree_views.find(node->subtree_reference);
            if (it != m_tree_views.end())
            {
                m_active_tree_name = node->subtree_reference;
                m_request_tab_change = true;
                return;
            }
        }
        else if (node)
        {
            m_show_palettes.node_edition = true;
        }
    }
    else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
        m_selected_node_id = hovered_id;
    }

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right) &&
        !ImGui::IsPopupOpen("NodeContextMenu") &&
        !ImGui::IsPopupOpen("AddNodePopup"))
    {
        if (hovered_id >= 0)
        {
            m_selected_node_id = hovered_id;
            ImGui::OpenPopup("NodeContextMenu");
        }
        else
        {
            m_show_palettes.node_creation = true;
            m_show_palettes.position = ImGui::GetMousePos();
            m_pending_link_from_node = -1;
        }
    }

    showNodeContextMenu();
    showNodeEditPopup();

    if (!ImGui::IsPopupOpen("NodeContextMenu") &&
        !ImGui::IsPopupOpen("AddNodePopup"))
    {
        if (m_selected_node_id >= 0 && ImGui::IsKeyPressed(ImGuiKey_Delete))
        {
            deleteNode(m_selected_node_id);
            m_selected_node_id = -1;
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Space))
        {
            m_show_palettes.node_creation = true;
            m_show_palettes.position = ImGui::GetMousePos();
        }
    }
}

// ----------------------------------------------------------------------------
void BTEditor::showAddNodePalette()
{
    if (m_show_palettes.node_creation)
    {
        ImGui::SetNextWindowPos(m_show_palettes.position);
        ImGui::OpenPopup("AddNodePopup");
        m_show_palettes.node_creation = false;
    }

    if (!ImGui::BeginPopup("AddNodePopup",
                           ImGuiWindowFlags_NoResize |
                               ImGuiWindowFlags_AlwaysAutoResize))
    {
        return;
    }

    ImGui::Text("Add Node");
    ImGui::Separator();

    for (const auto& node_type : c_node_types)
    {
        if (node_type && ImGui::Selectable(node_type))
        {
            addNodeAndLink(node_type, node_type);
        }
    }

    ImGui::EndPopup();
}

// ----------------------------------------------------------------------------
void BTEditor::showNodeContextMenu()
{
    if (!ImGui::BeginPopup("NodeContextMenu"))
        return;

    Node* selected = findNode(m_selected_node_id);

    if (!selected)
    {
        ImGui::EndPopup();
        return;
    }

    if (selected->type == "SubTree" && !selected->subtree_reference.empty())
    {
        if (ImGui::MenuItem("Go to Definition"))
        {
            auto it = m_tree_views.find(selected->subtree_reference);
            if (it != m_tree_views.end())
            {
                m_active_tree_name = selected->subtree_reference;
                m_request_tab_change = true;
            }
        }
        ImGui::Separator();
    }

    if (ImGui::MenuItem("Edit", "Double-click"))
    {
        m_show_palettes.node_edition = true;
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::MenuItem("Delete", "Del"))
    {
        deleteNode(m_selected_node_id);
        m_selected_node_id = -1;
        ImGui::CloseCurrentPopup();
    }
    if (ImGui::MenuItem("Set as Root"))
    {
        getCurrentTreeView().root_id = m_selected_node_id;
        m_is_modified = true;
        ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
}

// ----------------------------------------------------------------------------
void BTEditor::showNodeEditPopup()
{
    Node* node = findNode(m_selected_node_id);
    if (!node)
    {
        m_show_palettes.node_edition = false;
        return;
    }

    static Node temp_node;
    static bool temp_node_initialized = false;
    static std::string temp_new_input;
    static std::string temp_new_output;

    if (m_show_palettes.node_edition)
    {
        temp_node = *node;
        temp_node_initialized = true;
        temp_new_input.clear();
        temp_new_output.clear();

        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowSize(ImVec2(400, 0), ImGuiCond_Appearing);

        ImGui::OpenPopup("Edit Node");
        m_show_palettes.node_edition = false;
    }

    if (ImGui::BeginPopupModal("Edit Node",
                               nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize |
                                   ImGuiWindowFlags_NoCollapse |
                                   ImGuiWindowFlags_NoMove))
    {
        if (!temp_node_initialized)
        {
            temp_node = *node;
            temp_node_initialized = true;
        }

        ImGui::Text("Node Type:");
        ImGui::SameLine();

        static int current_type = 0;
        for (size_t i = 0; i < c_node_types.size(); i++)
        {
            if (temp_node.type == c_node_types[i])
            {
                current_type = static_cast<int>(i);
                break;
            }
        }

        if (ImGui::Combo("##Type",
                         &current_type,
                         c_node_types.data(),
                         static_cast<int>(c_node_types.size())))
        {
            temp_node.type = c_node_types[static_cast<size_t>(current_type)];
        }

        ImGui::Spacing();
        ImGui::Text("Name:");
        ImGui::SameLine();
        ImGui::InputText("##Name", &temp_node.name);

        if (temp_node.type == "SubTree")
        {
            ImGui::Spacing();
            ImGui::Text("Reference:");
            ImGui::SameLine();
            ImGui::InputText("##SubTreeRef", &temp_node.subtree_reference);

            if (ImGui::BeginCombo("##AvailableSubTrees",
                                  temp_node.subtree_reference.c_str()))
            {
                for (const auto& [name, view] : m_tree_views)
                {
                    if (view.is_subtree)
                    {
                        bool is_selected = (temp_node.subtree_reference == name);
                        if (ImGui::Selectable(name.c_str(), is_selected))
                        {
                            temp_node.subtree_reference = name;
                        }
                        if (is_selected)
                        {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                }
                ImGui::EndCombo();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        if (canHaveBlackboardPorts(temp_node.type))
        {
            ImGui::Text("Blackboard Inputs:");

            for (size_t i = 0; i < temp_node.inputs.size(); ++i)
            {
                ImGui::PushID(static_cast<int>(i));
                ImGui::BulletText("%s", temp_node.inputs[i].c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton("X"))
                {
                    temp_node.inputs.erase(temp_node.inputs.begin() +
                                           static_cast<std::ptrdiff_t>(i));
                    ImGui::PopID();
                    break;
                }
                ImGui::PopID();
            }

            ImGui::InputText("##NewInput", &temp_new_input);
            ImGui::SameLine();
            if (ImGui::Button("Add Input") && !temp_new_input.empty())
            {
                temp_node.inputs.push_back(temp_new_input);
                temp_new_input.clear();
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();

            ImGui::Text("Blackboard Outputs:");

            for (size_t i = 0; i < temp_node.outputs.size(); ++i)
            {
                ImGui::PushID(1000 + static_cast<int>(i));
                ImGui::BulletText("%s", temp_node.outputs[i].c_str());
                ImGui::SameLine();
                if (ImGui::SmallButton("X"))
                {
                    temp_node.outputs.erase(temp_node.outputs.begin() +
                                            static_cast<std::ptrdiff_t>(i));
                    ImGui::PopID();
                    break;
                }
                ImGui::PopID();
            }

            ImGui::InputText("##NewOutput", &temp_new_output);
            ImGui::SameLine();
            if (ImGui::Button("Add Output") && !temp_new_output.empty())
            {
                temp_node.outputs.push_back(temp_new_output);
                temp_new_output.clear();
            }

            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }
        else
        {
            temp_node.inputs.clear();
            temp_node.outputs.clear();
            ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                               "(This node type cannot have blackboard ports)");
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::Spacing();
        }

        float button_width = 120.0f;
        float spacing = 10.0f;
        float total_width = button_width * 2 + spacing;
        float start_x = (ImGui::GetContentRegionAvail().x - total_width) * 0.5f;

        if (start_x > 0)
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + start_x);

        if (ImGui::Button("Cancel", ImVec2(button_width, 0)))
        {
            ImGui::CloseCurrentPopup();
            temp_node_initialized = false;
            temp_new_input.clear();
            temp_new_output.clear();
        }

        ImGui::SameLine(0, spacing);

        if (ImGui::Button("Apply", ImVec2(button_width, 0)))
        {
            node->type = temp_node.type;
            node->name = temp_node.name;
            node->subtree_reference = temp_node.subtree_reference;
            node->inputs = temp_node.inputs;
            node->outputs = temp_node.outputs;

            m_is_modified = true;
            onNodeModified.emit(m_selected_node_id);
            ImGui::CloseCurrentPopup();
            temp_node_initialized = false;
            temp_new_input.clear();
            temp_new_output.clear();
        }

        ImGui::EndPopup();
    }

    if (!ImGui::IsPopupOpen("Edit Node") && temp_node_initialized)
    {
        temp_node_initialized = false;
        temp_new_input.clear();
        temp_new_output.clear();
    }
}

// ----------------------------------------------------------------------------
void BTEditor::showBlackboardPanel()
{
    if (!m_show_blackboard_panel || !m_blackboard)
        return;

    ImGui::SetNextWindowSize(ImVec2(350, 400), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Blackboard", &m_show_blackboard_panel))
    {
        ImGui::End();
        return;
    }

    static char new_var_name[128] = "";
    static char new_var_value[256] = "";
    static int new_var_type = 0;

    ImGui::Text("Add Variable:");
    ImGui::PushItemWidth(100);
    ImGui::InputText("Name##NewVar", new_var_name, sizeof(new_var_name));
    ImGui::SameLine();
    ImGui::InputText("Value##NewVar", new_var_value, sizeof(new_var_value));
    ImGui::PopItemWidth();

    ImGui::PushItemWidth(80);
    const char* type_names[] = {"string", "int", "double", "bool"};
    ImGui::Combo("Type##NewVar", &new_var_type, type_names, 4);
    ImGui::PopItemWidth();

    ImGui::SameLine();
    if (ImGui::Button("Add") && strlen(new_var_name) > 0)
    {
        std::string name(new_var_name);
        std::string val(new_var_value);

        switch (new_var_type)
        {
            case 0:
                m_blackboard->set(name, val);
                break;
            case 1:
                try
                {
                    m_blackboard->set(name, std::stoi(val));
                }
                catch (...)
                {
                    m_blackboard->set(name, 0);
                }
                break;
            case 2:
                try
                {
                    m_blackboard->set(name, std::stod(val));
                }
                catch (...)
                {
                    m_blackboard->set(name, 0.0);
                }
                break;
            case 3:
                m_blackboard->set(name, val == "true" || val == "1");
                break;
        }

        new_var_name[0] = '\0';
        new_var_value[0] = '\0';
        m_is_modified = true;
    }

    ImGui::Separator();
    ImGui::Text("Variables:");

    auto keys = m_blackboard->keys();
    std::vector<std::string> keys_to_remove;

    for (const auto& key : keys)
    {
        auto raw_value = m_blackboard->raw(key);
        if (!raw_value.has_value())
            continue;

        ImGui::PushID(key.c_str());

        std::string display_value = "<unknown>";
        try
        {
            if (raw_value->type() == typeid(int))
                display_value = std::to_string(std::any_cast<int>(*raw_value));
            else if (raw_value->type() == typeid(double))
                display_value = std::to_string(std::any_cast<double>(*raw_value));
            else if (raw_value->type() == typeid(bool))
                display_value = std::any_cast<bool>(*raw_value) ? "true" : "false";
            else if (raw_value->type() == typeid(std::string))
                display_value = std::any_cast<std::string>(*raw_value);
        }
        catch (...)
        {
            display_value = "<error>";
        }

        ImGui::BulletText("%s: %s", key.c_str(), display_value.c_str());

        ImGui::SameLine();
        if (ImGui::SmallButton("X"))
        {
            keys_to_remove.push_back(key);
        }

        ImGui::PopID();
    }

    for (const auto& key : keys_to_remove)
    {
        m_blackboard->remove(key);
        m_is_modified = true;
    }

    ImGui::End();
}

} // namespace robotik::renderer
