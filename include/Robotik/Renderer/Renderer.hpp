/**
 * @file Renderer.hpp
 * @brief 3D mesh rendering class for game engine / robot visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Managers/MeshManager.hpp"

#include <Eigen/Dense>
#include <string>

namespace robotik::renderer
{

// Forward declarations
class ShaderManager;

// ****************************************************************************
//! \brief 3D mesh rendering class.
//!
//! This class is responsible for rendering meshes provided by MeshManager.
//! It works with ShaderManager to apply transformations and colors.
//! Designed to be called by scene graph traversal when rendering URDF robots.
//!
//! The Renderer does NOT create or manage meshes - it only renders them.
//! Mesh creation and management is done by MeshManager.
// ****************************************************************************
class Renderer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_shader_manager Reference to shader manager.
    // ------------------------------------------------------------------------
    explicit Renderer(ShaderManager& p_shader_manager);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~Renderer() = default;

    // ------------------------------------------------------------------------
    //! \brief Initialize the renderer.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Render a mesh with transformation and color.
    //! \param p_mesh Pointer to GPU mesh to render.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    // ------------------------------------------------------------------------
    void render(const MeshManager::GPUMesh* p_mesh,
                const Eigen::Matrix4f& p_transform,
                const Eigen::Vector3f& p_color) const;

    // ------------------------------------------------------------------------
    //! \brief Render a grid.
    //! \param p_color RGB color (0.0-1.0).
    //! \param p_size Grid size (number of lines in each direction).
    //! \param p_spacing Spacing between grid lines.
    // ------------------------------------------------------------------------
    void renderGrid(const Eigen::Vector3f& p_color = Eigen::Vector3f(0.7f,
                                                                     0.7f,
                                                                     0.7f),
                    int p_size = 20,
                    float p_spacing = 1.0f) const;

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_scale Scale factor for axes.
    //! \param p_axes_mesh Pointer to cylinder mesh for axes (from MeshManager).
    // ------------------------------------------------------------------------
    void renderAxes(const Eigen::Matrix4f& p_transform,
                    float p_scale,
                    const MeshManager::GPUMesh* p_axes_mesh) const;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize grid geometry.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeGrid();

    // ------------------------------------------------------------------------
    //! \brief Generate grid vertices.
    //! \param p_vertices Output vertex data.
    //! \param p_size Grid size.
    //! \param p_spacing Grid spacing.
    // ------------------------------------------------------------------------
    void generateGrid(std::vector<float>& p_vertices,
                      int p_size,
                      float p_spacing) const;

private:

    //! \brief Shader manager reference
    ShaderManager& m_shader_manager;

    // Grid rendering data (grid is rendered directly, not managed by
    // MeshManager)
    MeshManager::GPUMesh m_grid;
    size_t m_grid_vertex_count = 0;

    // Shader uniform locations
    int m_model_uniform = -1;
    int m_color_uniform = -1;

    std::string m_error;
};

} // namespace robotik::renderer
