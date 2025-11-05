/**
 * @file RenderVisitor.hpp
 * @brief Visitor for rendering robot nodes using OpenGL.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/NodeVisitor.hpp"
#include "Robotik/Renderer/Managers/GeometryManager.hpp"

#include <Eigen/Dense>
#include <string>

// Forward declarations
namespace robotik
{
class Blueprint;
}

namespace robotik::renderer
{

// Forward declarations
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
    //! \param geometry_mgr Geometry manager for creating/retrieving GPU meshes.
    //! \param shader_mgr Shader manager for setting uniforms.
    // ------------------------------------------------------------------------
    RenderVisitor(GeometryManager& geometry_mgr,
                  ShaderManager const& shader_mgr);

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
    //! \brief Visit a Frame node (render coordinate axes).
    // ------------------------------------------------------------------------
    void visit(const robotik::Frame& frame) override;

    // ------------------------------------------------------------------------
    //! \brief Visit a generic Node (fallback).
    // ------------------------------------------------------------------------
    void visit(const robotik::Node& node) override;

    // ------------------------------------------------------------------------
    //! \brief Configure joint axes rendering options.
    //! \param p_enable Enable/disable joint axes rendering globally.
    //! \param p_revolute Show revolute/continuous joint axes.
    //! \param p_prismatic Show prismatic joint axes.
    // ------------------------------------------------------------------------
    inline void
    setJointAxesOptions(bool p_enable, bool p_revolute, bool p_prismatic)
    {
        m_show_joint_axes = p_enable;
        m_show_revolute_axes = p_revolute;
        m_show_prismatic_axes = p_prismatic;
    }

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes (X=red, Y=green, Z=blue).
    //! \param p_axes_mesh Pointer to cylinder mesh for axes (from
    //! GeometryManager).
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_scale Scale factor for axes.
    // ------------------------------------------------------------------------
    void renderAxes(const GeometryManager::GPUMesh* p_axes_mesh,
                    const Eigen::Matrix4f& p_transform,
                    float p_scale) const;

    // ------------------------------------------------------------------------
    //! \brief Render grid.
    //! \param p_grid_mesh Pointer to grid mesh (from GeometryManager).
    //! \param p_color RGB color (0.0-1.0).
    // ------------------------------------------------------------------------
    void renderGrid(const GeometryManager::GPUMesh* p_grid_mesh,
                    const Eigen::Vector3f& p_color) const;

private:

    // ------------------------------------------------------------------------
    //! \brief Render a mesh with transformation and color.
    //! \param p_mesh Pointer to GPU mesh to render.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    // ------------------------------------------------------------------------
    void renderMesh(const GeometryManager::GPUMesh* p_mesh,
                    const Eigen::Matrix4f& p_transform,
                    const Eigen::Vector3f& p_color) const;

    // ------------------------------------------------------------------------
    //! \brief Render a geometry with the given transform.
    // ------------------------------------------------------------------------
    void renderGeometry(robotik::Geometry const& geom,
                        Eigen::Matrix4f const& transform) const;

    // ------------------------------------------------------------------------
    //! \brief Render a joint axis using appropriate mesh.
    //! \param p_joint The joint to render axis for.
    // ------------------------------------------------------------------------
    void renderJointAxis(robotik::Joint const& p_joint) const;

private:

    GeometryManager& m_geometry_manager;
    ShaderManager const& m_shader_manager;
    bool m_show_joint_axes = false;
    bool m_show_revolute_axes = true;
    bool m_show_prismatic_axes = true;
};

} // namespace robotik::renderer
