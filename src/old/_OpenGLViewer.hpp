#pragma once

#include "Robotik/Robot.hpp"

#include "Robotik/private/Path.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace robotik
{

class Geometry;

// ****************************************************************************
//! \brief Simple OpenGL viewer for robot visualization.
//!
//! This viewer provides basic 3D visualization capabilities:
//! - Camera positioning (top, side, perspective views)
//! - Basic geometry rendering (box, cylinder, sphere)
//! - Grid floor
//! - Robot arm visualization
//! - No lighting (flat shading)
// ****************************************************************************
class OpenGLWindow
{
public:

    using KeyCallback = std::function<void(size_t, int)>;
    using CameraViewType = CameraManager::ViewType;

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_path Path searcher.
    //! \param p_width Window width.
    //! \param p_height Window height.
    //! \param p_title Window title.
    // ------------------------------------------------------------------------
    explicit OpenGLWindow(Path& p_path,
                          size_t p_width,
                          size_t p_height,
                          const std::string& p_title);

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~OpenGLWindow();

    // ------------------------------------------------------------------------
    //! \brief Initialize the viewer.
    //! \return true if initialization successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Set camera view.
    //! \param p_view_type Camera view position.
    //! \param p_camera_target Camera target position (default: origin).
    // ------------------------------------------------------------------------
    void cameraView(CameraViewType p_view_type,
                    const Eigen::Vector3f& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera view.
    //! \param p_view_type Camera view position.
    //! \param p_camera_target Camera target position (default: origin).
    // ------------------------------------------------------------------------
    void cameraView(CameraViewType p_view_type,
                    const Eigen::Vector3d& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Get the current camera view.
    //! \return Current camera view.
    // ------------------------------------------------------------------------
    CameraViewType cameraView() const
    {
        return m_camera_manager.getViewType();
    }

    // ------------------------------------------------------------------------
    //! \brief Render the scene.
    //! \param p_robot Robot arm to render.
    // ------------------------------------------------------------------------
    void render(Robot const& p_robot);

    // ------------------------------------------------------------------------
    //! \brief Check if window should close.
    //! \return true if window should close.
    // ------------------------------------------------------------------------
    bool isHalting() const;

    // ------------------------------------------------------------------------
    //! \brief Process input events.
    //! \param p_key_callback Callback function for key events.
    // ------------------------------------------------------------------------
    void processInput(KeyCallback const& p_key_callback);

    // ------------------------------------------------------------------------
    //! \brief Get the GLFW window.
    //! \return Pointer to GLFW window.
    // ------------------------------------------------------------------------
    GLFWwindow* window() const
    {
        return m_window;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    std::string error() const
    {
        return m_error_message;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if a key is currently pressed.
    //! \param p_key The key to check.
    //! \return true if the key is pressed.
    // ------------------------------------------------------------------------
    bool isKeyPressed(int p_key) const;

private:

    // ------------------------------------------------------------------------
    // Rendering methods (implementation details)
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    //! \brief Render grid.
    // ------------------------------------------------------------------------
    void renderGrid() const;

    // ------------------------------------------------------------------------
    //! \brief Render box.
    //! \param p_transform Transformation matrix.
    //! \param p_color Color.
    // ------------------------------------------------------------------------
    void renderBox(Transform const& p_transform,
                   const Eigen::Vector3f& p_color =
                       Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render cylinder.
    //! \param p_transform Transformation matrix.
    //! \param p_color Color.
    // ------------------------------------------------------------------------
    void renderCylinder(Transform const& p_transform,
                        const Eigen::Vector3f& p_color =
                            Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render sphere.
    //! \param p_transform Transformation matrix.
    //! \param p_color Color.
    // ------------------------------------------------------------------------
    void renderSphere(Transform const& p_transform,
                      const Eigen::Vector3f& p_color =
                          Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render robot arm.
    //! \param p_robot Robot arm to render.
    // ------------------------------------------------------------------------
    void renderRobot(Robot const& p_robot) const;

    // ------------------------------------------------------------------------
    //! \brief Render a joint node.
    //! \param p_joint Pointer to the joint to render.
    //! \param p_world_transform World transformation matrix.
    // ------------------------------------------------------------------------
    void renderJoint(Joint const& p_joint,
                     Transform const& p_world_transform) const;

    // ------------------------------------------------------------------------
    //! \brief Render a link node.
    //! \param p_link Pointer to the link to render.
    //! \param p_world_transform World transformation matrix.
    // ------------------------------------------------------------------------
    void renderLink(Link const& p_link,
                    Transform const& p_world_transform) const;
    void renderGeometry(Geometry const& p_geometry,
                        Transform const& p_world_transform) const;

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes (XYZ) in RGB colors.
    //! \param p_transform Transformation matrix for the axes origin.
    //! \param p_scale Scale factor for the axes length (default: 1.0).
    // ------------------------------------------------------------------------
    void renderAxes(Transform const& p_transform, double p_scale = 1.0) const;

    // ------------------------------------------------------------------------
    //! \brief Render mesh from file.
    //! \param p_transform Transformation matrix.
    //! \param p_mesh_path Path to mesh file.
    //! \param p_color Color.
    // ------------------------------------------------------------------------
    void renderMesh(Transform const& p_transform,
                    const std::string& p_mesh_path,
                    const Eigen::Vector3f& p_color =
                        Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    // Internal initialization and management methods
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    //! \brief Initialize mesh loaders dictionary.
    // ------------------------------------------------------------------------
    void initializeMeshLoaders();

    // ------------------------------------------------------------------------
    //! \brief Initialize OpenGL context.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeGL();

    // ------------------------------------------------------------------------
    //! \brief Initialize shaders.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeShaders();

    // ------------------------------------------------------------------------
    //! \brief Compile shader.
    //! \param p_source Shader source code.
    //! \param p_type Shader type.
    //! \return Shader ID.
    // ------------------------------------------------------------------------
    unsigned int compileShader(const std::string& p_source,
                               unsigned int p_type) const;

    // ------------------------------------------------------------------------
    //! \brief Create shader program.
    //! \param p_vertex_source Vertex shader source.
    //! \param p_fragment_source Fragment shader source.
    //! \return Program ID.
    // ------------------------------------------------------------------------
    unsigned int
    createShaderProgram(const std::string& p_vertex_source,
                        const std::string& p_fragment_source) const;

    // ------------------------------------------------------------------------
    //! \brief Update camera matrices.
    // ------------------------------------------------------------------------
    void updateCamera();

    // ------------------------------------------------------------------------
    //! \brief Setup camera for specific view.
    //! \param p_view_type Camera view.
    //! \param p_camera_target Camera target position.
    // ------------------------------------------------------------------------
    void setupCameraView(CameraViewType p_view_type,
                         const Eigen::Vector3f& p_camera_target);

private:

    // Path searcher
    Path& m_path;

    // Specialized managers
    CameraManager m_camera_manager;
    ShaderManager m_shader_manager;
    GeometryRenderer m_geometry_renderer;
    MeshManager m_mesh_manager;

    // Window properties
    size_t m_width;
    size_t m_height;
    std::string m_title;
    GLFWwindow* m_window = nullptr;

    // Coordinate system conversion matrix
    Eigen::Matrix4f m_urdf_to_opengl_matrix;

    // Error message
    mutable std::string m_error_message;

    // Key state tracking
    std::array<bool, 256> m_keys = { false };
};

} // namespace robotik