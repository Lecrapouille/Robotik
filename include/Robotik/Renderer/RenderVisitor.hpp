/**
 * @file RenderVisitor.hpp
 * @brief Visitor for rendering robot nodes using OpenGL.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Robot/Blueprint/NodeVisitor.hpp"

#include <Eigen/Dense>
#include <string>

namespace robotik::renderer
{

// Forward declarations
class MeshManager;
class Renderer;
class ShaderManager;

// ****************************************************************************
//! \brief Visitor that renders robot nodes using OpenGL.
//!
//! This visitor traverses the robot blueprint and renders each geometry node
//! by creating or reusing GPU meshes and applying the correct transformations.
//!
//! Example usage:
//! \code
//!   RenderVisitor visitor(mesh_manager, renderer, shader_manager);
//!   robot->blueprint().root().traverse(visitor);
//! \endcode
// ****************************************************************************
class RenderVisitor: public robotik::ConstNodeVisitor
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param mesh_mgr Mesh manager for creating/retrieving GPU meshes.
    //! \param renderer Renderer for drawing meshes.
    //! \param shader_mgr Shader manager for setting uniforms.
    // ------------------------------------------------------------------------
    RenderVisitor(MeshManager& mesh_mgr,
                  Renderer& renderer,
                  ShaderManager& shader_mgr);

    // ------------------------------------------------------------------------
    //! \brief Visit a Joint node (optionally render joint axes for debug).
    // ------------------------------------------------------------------------
    void visit(const robotik::Joint& joint) override;

    // ------------------------------------------------------------------------
    //! \brief Visit a Link node (nothing to render, links have no geometry).
    // ------------------------------------------------------------------------
    void visit(const robotik::Link& link) override;

    // ------------------------------------------------------------------------
    //! \brief Visit a Geometry node and render it.
    // ------------------------------------------------------------------------
    void visit(const robotik::Geometry& geometry) override;

    // ------------------------------------------------------------------------
    //! \brief Visit a Sensor node (not yet implemented).
    // ------------------------------------------------------------------------
    void visit(const robotik::Sensor& sensor) override;

    // ------------------------------------------------------------------------
    //! \brief Visit an Actuator node (not yet implemented).
    // ------------------------------------------------------------------------
    void visit(const robotik::Actuator& actuator) override;

    // ------------------------------------------------------------------------
    //! \brief Visit a generic Node (fallback).
    // ------------------------------------------------------------------------
    void visit(const robotik::Node& node) override;

    // ------------------------------------------------------------------------
    //! \brief Enable/disable rendering of joint axes for debugging.
    // ------------------------------------------------------------------------
    inline void showJointAxes(bool show)
    {
        m_show_joint_axes = show;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Render a geometry with the given transform.
    // ------------------------------------------------------------------------
    void renderGeometry(robotik::Geometry const& geom,
                        Eigen::Matrix4f const& transform);

    // ------------------------------------------------------------------------
    //! \brief Generate a unique mesh name based on geometry parameters.
    // ------------------------------------------------------------------------
    std::string getMeshName(robotik::Geometry const& geom) const;

    MeshManager& m_mesh_manager;
    Renderer& m_renderer;
    ShaderManager& m_shader_manager;
    bool m_show_joint_axes = false;
};

} // namespace robotik::renderer
