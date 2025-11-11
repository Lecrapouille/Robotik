/**
 * @file DotExporter.hpp
 * @brief Exporter for DOT format (Graphviz) to visualize robot blueprint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Exporters/RobotExporter.hpp"
#include "Robotik/Core/Robot/RobotVisitor.hpp"

#include <sstream>
#include <string>
#include <unordered_map>

namespace robotik
{

// Forward declarations
class Node;
class Joint;
class Link;
class Geometry;
class Sensor;
class Actuator;
class Frame;

// ****************************************************************************
//! \brief Exporter for DOT format (Graphviz) to visualize robot blueprint.
//!
//! This exporter generates a DOT file that can be rendered using Graphviz
//! to visualize the robot's kinematic structure. The graph shows all nodes
//! (Joints, Links, Geometries, Sensors, Actuators) and their hierarchical
//! relationships.
//!
//! Example usage:
//! \code
//!   DotExporter exporter;
//!   if (!exporter.exportTo(robot, "robot.dot")) {
//!       std::cerr << exporter.error() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
class DotExporter: public RobotExporter
{
public:

    DotExporter();
    ~DotExporter() override = default;

    // Non-copyable, non-movable
    DotExporter(const DotExporter&) = delete;
    DotExporter& operator=(const DotExporter&) = delete;
    DotExporter(DotExporter&&) = delete;
    DotExporter& operator=(DotExporter&&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Export a robot to a DOT file.
    //! \param robot The robot to export.
    //! \param filename Path to the output DOT file.
    //! \return True if the export was successful, false otherwise.
    // ------------------------------------------------------------------------
    bool exportTo(const Robot& robot, const std::string& filename) override;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if exportTo() failed.
    //! \return The error message in case of failure, else an empty string.
    // ------------------------------------------------------------------------
    inline std::string error() const override
    {
        return m_error;
    }

private:

    // ****************************************************************************
    //! \brief Visitor for traversing the blueprint and generating DOT code.
    //!
    //! This visitor traverses the blueprint tree and generates DOT graph
    //! definitions for all nodes (Joint, Link, Geometry, Sensor, Actuator).
    // ****************************************************************************
    class DotExportVisitor: public ConstRobotVisitor
    {
    public:

        explicit DotExportVisitor(const std::string& robot_name);
        ~DotExportVisitor() override = default;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Joint node.
        //! \param joint The joint to visit.
        // ------------------------------------------------------------------------
        void visit(const Joint& joint) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Link node.
        //! \param link The link to visit.
        // ------------------------------------------------------------------------
        void visit(const Link& link) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Geometry node.
        //! \param geometry The geometry to visit.
        // ------------------------------------------------------------------------
        void visit(const Geometry& geometry) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Sensor node.
        //! \param sensor The sensor to visit.
        // ------------------------------------------------------------------------
        void visit(const Sensor& sensor) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Actuator node.
        //! \param actuator The actuator to visit.
        // ------------------------------------------------------------------------
        void visit(const Actuator& actuator) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const Frame node.
        //! \param frame The frames to visit.
        // ------------------------------------------------------------------------
        void visit(const Frame& frame) override;

        // ------------------------------------------------------------------------
        //! \brief Visit a const generic Node (fallback).
        //! \param node The node to visit.
        // ------------------------------------------------------------------------
        void visit(const Node& node) override;

        // ------------------------------------------------------------------------
        //! \brief Get the generated DOT code.
        //! \return The complete DOT graph as a string.
        // ------------------------------------------------------------------------
        std::string getDotCode() const;

    private:

        // ------------------------------------------------------------------------
        //! \brief Get or create a unique node ID for the given node.
        //! \param node Pointer to the node.
        //! \return Unique string ID for the node.
        // ------------------------------------------------------------------------
        std::string getNodeId(const Node* node);

        // ------------------------------------------------------------------------
        //! \brief Add an edge from parent to child.
        //! \param parent_id ID of the parent node.
        //! \param child_id ID of the child node.
        //! \param label Optional label for the edge (e.g., transformation).
        // ------------------------------------------------------------------------
        void addEdge(const std::string& parent_id,
                     const std::string& child_id,
                     const std::string& label = "");

        // ------------------------------------------------------------------------
        //! \brief Add a node definition to the DOT graph.
        //! \param node_id Unique ID of the node.
        //! \param label Label text for the node.
        //! \param shape Shape of the node (box, ellipse, diamond, octagon,
        //! etc.).
        // ------------------------------------------------------------------------
        void addNode(const std::string& node_id,
                     const std::string& label,
                     const std::string& shape);

        // ------------------------------------------------------------------------
        //! \brief Escape special characters in a string for DOT format.
        //! \param str The string to escape.
        //! \return The escaped string.
        // ------------------------------------------------------------------------
        std::string escapeDotString(const std::string& str) const;

        // ------------------------------------------------------------------------
        //! \brief Format a local transform for edge label.
        //! \param transform The local transformation matrix.
        //! \return Formatted string representation of the transform.
        // ------------------------------------------------------------------------
        std::string formatTransform(const Transform& transform) const;

    private:

        //! \brief Stream for accumulating DOT code
        std::ostringstream m_dot_stream;
        //! \brief Map from node pointer to unique node ID
        std::unordered_map<const Node*, std::string> m_node_ids;
        //! \brief Counter for generating unique node IDs
        size_t m_node_counter;
        //! \brief Name of the robot (used as graph name)
        std::string m_robot_name;
    };

private:

    //! \brief Error message if export failed
    std::string m_error;
};

} // namespace robotik
