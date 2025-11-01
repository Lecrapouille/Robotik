/**
 * @file RenderVisitor.hpp
 * @brief Visitor for rendering robot nodes using OpenGL.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include "Robotik/Core/Robot/State.hpp"
#include "Robotik/Renderer/Managers/MeshManager.hpp"

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
class RenderVisitor
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param mesh_mgr Mesh manager for creating/retrieving GPU meshes.
    //! \param shader_mgr Shader manager for setting uniforms.
    // ------------------------------------------------------------------------
    RenderVisitor(MeshManager& mesh_mgr, ShaderManager const& shader_mgr);

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
    //! \brief Preload all geometries from the robot blueprint.
    //! \param p_blueprint The robot blueprint containing all geometries.
    //!
    //! This method creates all GPU meshes needed for rendering the robot
    //! before the first render call. It should be called once after loading
    //! the robot model.
    // ------------------------------------------------------------------------
    void preloadGeometries(robotik::Blueprint const& p_blueprint);

    // ------------------------------------------------------------------------
    //! \brief Render a robot using flat array architecture.
    //! \param p_blueprint The robot blueprint containing geometry data.
    //! \param p_state The robot state containing current transforms.
    //!
    //! This is the new rendering method that iterates over flat arrays instead
    //! of traversing the tree structure. It's more cache-friendly and doesn't
    //! require the tree to be built.
    // ------------------------------------------------------------------------
    void renderBlueprint(robotik::Blueprint const& p_blueprint,
                         robotik::State const& p_state);

    // ------------------------------------------------------------------------
    //! \brief Render a mesh with transformation and color.
    //! \param p_mesh Pointer to GPU mesh to render.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    // ------------------------------------------------------------------------
    void renderMesh(const MeshManager::GPUMesh* p_mesh,
                    const Eigen::Matrix4f& p_transform,
                    const Eigen::Vector3f& p_color) const;

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes (X=red, Y=green, Z=blue).
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_scale Scale factor for axes.
    //! \param p_axes_mesh Pointer to cylinder mesh for axes (from MeshManager).
    // ------------------------------------------------------------------------
    void renderAxes(const MeshManager::GPUMesh* p_axes_mesh,
                    const Eigen::Matrix4f& p_transform,
                    float p_scale) const;

    // ------------------------------------------------------------------------
    //! \brief Render grid.
    //! \param p_grid_mesh Pointer to grid mesh (from MeshManager).
    //! \param p_color RGB color (0.0-1.0).
    // ------------------------------------------------------------------------
    void renderGrid(const MeshManager::GPUMesh* p_grid_mesh,
                    const Eigen::Vector3f& p_color) const;

private:

    MeshManager& m_mesh_manager;
    ShaderManager const& m_shader_manager;
    bool m_show_joint_axes = false;
    bool m_show_revolute_axes = true;
    bool m_show_prismatic_axes = true;
};

} // namespace robotik::renderer
