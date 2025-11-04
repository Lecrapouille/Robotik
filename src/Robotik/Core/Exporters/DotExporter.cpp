/**
 * @file DotExporter.cpp
 * @brief Implementation of DotExporter for DOT format export.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Exporters/DotExporter.hpp"
#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/Blueprint/Component.hpp"
#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Blueprint/Node.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>

namespace robotik
{

// ----------------------------------------------------------------------------
DotExporter::DotExporter() : m_error() {}

// ----------------------------------------------------------------------------
bool DotExporter::exportTo(const Robot& robot, const std::string& filename)
{
    m_error.clear();

    // Check if blueprint has a root
    const Blueprint& blueprint = robot.blueprint();
    if (!blueprint.hasRoot())
    {
        m_error = "Blueprint has no root node";
        return false;
    }

    // Create visitor and traverse the blueprint
    DotExportVisitor visitor(robot.name());
    blueprint.root().traverse(visitor);

    // Write DOT code to file
    std::ofstream file(filename);
    if (!file.is_open())
    {
        m_error = "Failed to open file for writing: " + filename;
        return false;
    }

    file << visitor.getDotCode();
    file.close();

    if (!file.good())
    {
        m_error = "Error writing to file: " + filename;
        return false;
    }

    return true;
}

// ============================================================================
// DotExportVisitor implementation
// ============================================================================

// ----------------------------------------------------------------------------
DotExporter::DotExportVisitor::DotExportVisitor(const std::string& robot_name)
    : m_dot_stream(), m_node_ids(), m_node_counter(1), m_robot_name(robot_name)
{
    // Start DOT graph
    m_dot_stream << "digraph " << escapeDotString(m_robot_name) << " {\n";
    m_dot_stream << "    rankdir=TB;\n";
    m_dot_stream << "    node [fontname=\"Arial\"];\n\n";
}

// ----------------------------------------------------------------------------
std::string DotExporter::DotExportVisitor::getDotCode() const
{
    std::string code = m_dot_stream.str();
    code += "}\n";
    return code;
}

// ----------------------------------------------------------------------------
std::string DotExporter::DotExportVisitor::getNodeId(const Node* node)
{
    if (auto it = m_node_ids.find(node); it != m_node_ids.end())
    {
        return it->second;
    }

    const size_t node_index = m_node_counter;
    ++m_node_counter;
    std::string node_id = "node" + std::to_string(node_index);
    m_node_ids.try_emplace(node, node_id);
    return node_id;
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::addEdge(const std::string& parent_id,
                                            const std::string& child_id,
                                            const std::string& label)
{
    m_dot_stream << "    " << parent_id << " -> " << child_id;
    if (!label.empty())
    {
        m_dot_stream << " [label=\"" << escapeDotString(label) << "\"]";
    }
    m_dot_stream << ";\n";
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::addNode(const std::string& node_id,
                                            const std::string& label,
                                            const std::string& shape)
{
    m_dot_stream << "    " << node_id << " [label=\"" << escapeDotString(label)
                 << "\", shape=" << shape << "];\n";
}

// ----------------------------------------------------------------------------
std::string
DotExporter::DotExportVisitor::escapeDotString(const std::string& str) const
{
    std::ostringstream escaped;
    for (char c : str)
    {
        switch (c)
        {
            case '"':
                escaped << "\\\"";
                break;
            case '\\':
                escaped << "\\\\";
                break;
            case '\n':
                escaped << "\\n";
                break;
            case '\r':
                escaped << "\\r";
                break;
            default:
                escaped << c;
                break;
        }
    }
    return escaped.str();
}

// ----------------------------------------------------------------------------
std::string
DotExporter::DotExportVisitor::formatTransform(const Transform& transform) const
{
    constexpr double epsilon = 1e-6;

    Eigen::Vector3d translation = getTranslation(transform);
    Eigen::Matrix3d rotation = getRotation(transform);
    Eigen::Vector3d rpy = rotationToEuler(rotation);

    std::ostringstream label;
    label << std::fixed << std::setprecision(3);

    const bool has_translation = translation.norm() > epsilon;
    const bool has_rotation =
        (rotation - Eigen::Matrix3d::Identity()).norm() > epsilon;

    if (has_translation)
    {
        label << "T(" << translation.x() << "," << translation.y() << ","
              << translation.z() << ")";
        if (has_rotation)
        {
            label << " ";
        }
    }
    if (has_rotation)
    {
        label << "R(" << rpy.x() << "," << rpy.y() << "," << rpy.z() << ")";
    }

    return label.str();
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Joint& joint)
{
    std::string node_id = getNodeId(&joint);

    // Build label with joint name, type, and limits
    std::ostringstream label;
    label << "Joint: " << joint.name() << " (";

    switch (joint.type())
    {
        case Joint::Type::FIXED:
            label << "FIXED";
            break;
        case Joint::Type::PRISMATIC:
            label << "PRISMATIC";
            break;
        case Joint::Type::REVOLUTE:
            label << "REVOLUTE";
            break;
        case Joint::Type::CONTINUOUS:
            label << "CONTINUOUS";
            break;
    }
    label << ")";

    // Add limits for joints that have them (not FIXED or CONTINUOUS)
    if (joint.type() == Joint::Type::REVOLUTE)
    {
        const auto [min, max] = joint.limits();
        label << "\n θ = [" << std::fixed << std::setprecision(3) << min << ", "
              << max << "]";
    }
    else if (joint.type() == Joint::Type::PRISMATIC)
    {
        const auto [min, max] = joint.limits();
        label << "\n d = [" << std::fixed << std::setprecision(3) << min << ", "
              << max << "]";
    }

    addNode(node_id, label.str(), "ellipse");

    // Create edges to all children (we know them via children())
    for (const auto& child : joint.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Link& link)
{
    std::string node_id = getNodeId(&link);

    // Build label with link name
    std::ostringstream label;
    label << "Link: " << link.name();

    addNode(node_id, label.str(), "box");

    // Create edges to all children (we know them via children())
    for (const auto& child : link.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Geometry& geometry)
{
    std::string node_id = getNodeId(&geometry);

    // Build label with geometry name and type
    std::ostringstream label;
    label << "Geometry: " << geometry.name() << " (";

    switch (geometry.type())
    {
        case Geometry::Type::BOX:
            label << "BOX";
            break;
        case Geometry::Type::CYLINDER:
            label << "CYLINDER";
            break;
        case Geometry::Type::SPHERE:
            label << "SPHERE";
            break;
        case Geometry::Type::MESH:
            label << "MESH";
            break;
    }
    label << ")";

    addNode(node_id, label.str(), "diamond");

    // Create edges to all children (we know them via children())
    for (const auto& child : geometry.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Sensor& sensor)
{
    std::string node_id = getNodeId(&sensor);

    // Build label with sensor name
    std::ostringstream label;
    label << "Sensor: " << sensor.name();

    addNode(node_id, label.str(), "octagon");

    // Create edges to all children (we know them via children())
    for (const auto& child : sensor.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Actuator& actuator)
{
    std::string node_id = getNodeId(&actuator);

    // Build label with actuator name
    std::ostringstream label;
    label << "Actuator: " << actuator.name();

    addNode(node_id, label.str(), "octagon");

    // Create edges to all children (we know them via children())
    for (const auto& child : actuator.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

// ----------------------------------------------------------------------------
void DotExporter::DotExportVisitor::visit(const Node& node)
{
    std::string node_id = getNodeId(&node);

    // Build label with node name
    std::ostringstream label;
    label << "Node: " << node.name();

    addNode(node_id, label.str(), "box");

    // Create edges to all children (we know them via children())
    for (const auto& child : node.children())
    {
        std::string child_id = getNodeId(child.get());
        std::string transform_label = formatTransform(child->localTransform());
        addEdge(node_id, child_id, transform_label);
    }
}

} // namespace robotik
