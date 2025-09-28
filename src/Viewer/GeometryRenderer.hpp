/**
 * @file GeometryRenderer.hpp
 * @brief OpenGL geometry rendering class for primitives.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <Eigen/Dense>
#include <cstddef>
#include <string>
#include <vector>

namespace robotik::viewer
{

// Forward declarations
class ShaderManager;

// ****************************************************************************
//! \brief OpenGL geometry rendering class for primitives.
//!
//! This class handles rendering of basic 3D primitives like boxes, cylinders,
//! spheres, and grids. It manages OpenGL buffers and provides rendering
//! methods.
// ****************************************************************************
class GeometryRenderer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_shader_manager Reference to shader manager.
    // ------------------------------------------------------------------------
    explicit GeometryRenderer(ShaderManager& p_shader_manager);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~GeometryRenderer();

    // ------------------------------------------------------------------------
    //! \brief Initialize all geometry primitives.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Render a box.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    //! \param p_width Box width.
    //! \param p_height Box height.
    //! \param p_depth Box depth.
    // ------------------------------------------------------------------------
    void renderBox(const Eigen::Matrix4f& p_transform,
                   const Eigen::Vector3f& p_color,
                   float p_width = 1.0f,
                   float p_height = 1.0f,
                   float p_depth = 1.0f) const;

    // ------------------------------------------------------------------------
    //! \brief Render a cylinder.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    //! \param p_radius Cylinder radius.
    //! \param p_height Cylinder height.
    // ------------------------------------------------------------------------
    void renderCylinder(const Eigen::Matrix4f& p_transform,
                        const Eigen::Vector3f& p_color,
                        float p_radius = 1.0f,
                        float p_height = 2.0f) const;

    // ------------------------------------------------------------------------
    //! \brief Render a sphere.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    //! \param p_radius Sphere radius.
    // ------------------------------------------------------------------------
    void renderSphere(const Eigen::Matrix4f& p_transform,
                      const Eigen::Vector3f& p_color,
                      float p_radius = 1.0f) const;

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
    // ------------------------------------------------------------------------
    void renderAxes(const Eigen::Matrix4f& p_transform,
                    float p_scale = 1.0f) const;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& getLastError() const
    {
        return m_last_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize box geometry.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeBox();

    // ------------------------------------------------------------------------
    //! \brief Initialize cylinder geometry.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeCylinder();

    // ------------------------------------------------------------------------
    //! \brief Initialize sphere geometry.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeSphere();

    // ------------------------------------------------------------------------
    //! \brief Initialize grid geometry.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeGrid();

    // ------------------------------------------------------------------------
    //! \brief Generate box vertices and indices.
    //! \param p_vertices Output vertex data.
    //! \param p_indices Output index data.
    // ------------------------------------------------------------------------
    void generateBox(std::vector<float>& p_vertices,
                     std::vector<unsigned int>& p_indices) const;

    // ------------------------------------------------------------------------
    //! \brief Generate cylinder vertices and indices.
    //! \param p_vertices Output vertex data.
    //! \param p_indices Output index data.
    //! \param p_radius Cylinder radius.
    //! \param p_height Cylinder height.
    //! \param p_segments Number of segments around the cylinder.
    // ------------------------------------------------------------------------
    void generateCylinder(std::vector<float>& p_vertices,
                          std::vector<unsigned int>& p_indices,
                          float p_radius = 1.0f,
                          float p_height = 2.0f,
                          size_t p_segments = 16) const;

    // ------------------------------------------------------------------------
    //! \brief Generate sphere vertices and indices.
    //! \param p_vertices Output vertex data.
    //! \param p_indices Output index data.
    //! \param p_radius Sphere radius.
    //! \param p_latitude_segments Number of latitude segments.
    //! \param p_longitude_segments Number of longitude segments.
    // ------------------------------------------------------------------------
    void generateSphere(std::vector<float>& p_vertices,
                        std::vector<unsigned int>& p_indices,
                        float p_radius = 1.0f,
                        size_t p_latitude_segments = 16,
                        size_t p_longitude_segments = 16) const;

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

    // Shader manager reference
    ShaderManager& m_shader_manager;

    // OpenGL buffer objects
    unsigned int m_box_vao = 0;
    unsigned int m_box_vbo = 0;
    unsigned int m_box_ebo = 0;

    unsigned int m_cylinder_vao = 0;
    unsigned int m_cylinder_vbo = 0;
    unsigned int m_cylinder_ebo = 0;
    size_t m_cylinder_index_count = 0;

    unsigned int m_sphere_vao = 0;
    unsigned int m_sphere_vbo = 0;
    unsigned int m_sphere_ebo = 0;
    size_t m_sphere_index_count = 0;

    unsigned int m_grid_vao = 0;
    unsigned int m_grid_vbo = 0;
    size_t m_grid_vertex_count = 0;

    // Shader uniform locations
    int m_model_uniform = -1;
    int m_color_uniform = -1;

    std::string m_last_error;
};

} // namespace robotik::viewer
