/**
 * @file RobotViewer.hpp
 * @brief Robot viewer class in OpenGL.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Viewer/Camera.hpp"
#include "Viewer/GeometryRenderer.hpp"
#include "Viewer/MeshManager.hpp"
#include "Viewer/ShaderManager.hpp"
#include <cstddef>
#include <string>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Robot viewer class for 3D visualization.
//!
//! This class coordinates all the OpenGL components for rendering robots
//! and 3D scenes. It manages shaders, meshes, geometry rendering and camera.
// ****************************************************************************
class RobotViewer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_width Window width.
    //! \param p_height Window height.
    // ------------------------------------------------------------------------
    explicit RobotViewer(size_t p_width = 800, size_t p_height = 600);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~RobotViewer();

    // ------------------------------------------------------------------------
    //! \brief Initialize the viewer.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Render a frame.
    //! \param p_clear_color Background color (RGB).
    // ------------------------------------------------------------------------
    void render(const Eigen::Vector3f& p_clear_color = Eigen::Vector3f(0.2f,
                                                                       0.3f,
                                                                       0.3f));

    // ------------------------------------------------------------------------
    //! \brief Load a mesh from STL file.
    //! \param p_mesh_path Path to the STL file.
    //! \param p_force_reload Force reload even if already cached.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadMesh(const std::string& p_mesh_path, bool p_force_reload = false);

    // ------------------------------------------------------------------------
    //! \brief Render a mesh.
    //! \param p_mesh_path Path to the mesh file.
    //! \param p_transform 4x4 transformation matrix.
    //! \param p_color RGB color (0.0-1.0).
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool renderMesh(const std::string& p_mesh_path,
                    const Eigen::Matrix4f& p_transform,
                    const Eigen::Vector3f& p_color =
                        Eigen::Vector3f(1.0f, 1.0f, 1.0f)) const;

    // ------------------------------------------------------------------------
    //! \brief Set the base path for mesh files.
    //! \param p_base_path Base path for mesh files.
    // ------------------------------------------------------------------------
    void setMeshBasePath(const std::string& p_base_path);

    // ------------------------------------------------------------------------
    //! \brief Get the camera.
    //! \return Reference to camera.
    // ------------------------------------------------------------------------
    Camera& camera()
    {
        return m_camera;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the shader manager.
    //! \return Reference to shader manager.
    // ------------------------------------------------------------------------
    ShaderManager& shaderManager()
    {
        return m_shader_manager;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the mesh manager.
    //! \return Reference to mesh manager.
    // ------------------------------------------------------------------------
    MeshManager& meshManager()
    {
        return m_mesh_manager;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the geometry renderer.
    //! \return Reference to geometry renderer.
    // ------------------------------------------------------------------------
    GeometryRenderer& geometryRenderer()
    {
        return m_geometry_renderer;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& getLastError() const;

private:

    // ------------------------------------------------------------------------
    //! \brief Setup default shaders.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool setupShaders();

private:

    // Core components
    Camera m_camera;
    ShaderManager m_shader_manager;
    MeshManager m_mesh_manager;
    GeometryRenderer m_geometry_renderer;

    // Window properties
    size_t m_width;
    size_t m_height;

    // Error handling
    std::string m_last_error;
};

} // namespace robotik::viewer