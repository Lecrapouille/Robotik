#pragma once

#include "Robotik.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>

namespace robotik
{

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
class Viewer
{
public:

    // ------------------------------------------------------------------------
    //! \brief Camera view positions
    // ------------------------------------------------------------------------
    enum class CameraViewType
    {
        PERSPECTIVE, //!< Default perspective view
        TOP,         //!< Top-down view
        FRONT,       //!< Front view
        SIDE,        //!< Side view
        ISOMETRIC    //!< Isometric view
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor
    //! \param p_width Window width
    //! \param p_height Window height
    //! \param p_title Window title
    // ------------------------------------------------------------------------
    Viewer(int p_width = 800,
           int p_height = 600,
           const std::string& p_title = "Robotik Viewer");

    // ------------------------------------------------------------------------
    //! \brief Destructor
    // ------------------------------------------------------------------------
    ~Viewer();

    // ------------------------------------------------------------------------
    //! \brief Initialize the viewer
    //! \return true if initialization successful
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Set the robot arm to visualize
    //! \param p_robot Pointer to the robot arm
    // ------------------------------------------------------------------------
    void setRobot(const RobotArm& p_robot);

    // ------------------------------------------------------------------------
    //! \brief Set camera view
    //! \param p_view_type Camera view position
    //! \param p_camera_target Camera target position (default: origin)
    // ------------------------------------------------------------------------
    void setCameraView(CameraViewType p_view_type,
                       const Eigen::Vector3f& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera view
    //! \param p_view_type Camera view position
    //! \param p_camera_target Camera target position (default: origin)
    // ------------------------------------------------------------------------
    void setCameraView(CameraViewType p_view_type,
                       const Eigen::Vector3d& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Render the scene
    // ------------------------------------------------------------------------
    void render();

    // ------------------------------------------------------------------------
    //! \brief Check if window should close
    //! \return true if window should close
    // ------------------------------------------------------------------------
    bool shouldClose() const;

    // ------------------------------------------------------------------------
    //! \brief Process input events
    // ------------------------------------------------------------------------
    void processInput();

    // ------------------------------------------------------------------------
    //! \brief Get the GLFW window
    //! \return Pointer to GLFW window
    // ------------------------------------------------------------------------
    GLFWwindow* getWindow() const
    {
        return m_window;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize OpenGL context
    //! \return true if successful
    // ------------------------------------------------------------------------
    bool initializeGL();

    // ------------------------------------------------------------------------
    //! \brief Initialize shaders
    //! \return true if successful
    // ------------------------------------------------------------------------
    bool initializeShaders();

    // ------------------------------------------------------------------------
    //! \brief Initialize geometry buffers
    //! \return true if successful
    // ------------------------------------------------------------------------
    bool initializeGeometry();

    // ------------------------------------------------------------------------
    //! \brief Compile shader
    //! \param p_source Shader source code
    //! \param p_type Shader type
    //! \return Shader ID
    // ------------------------------------------------------------------------
    unsigned int compileShader(const std::string& p_source,
                               unsigned int p_type) const;

    // ------------------------------------------------------------------------
    //! \brief Create shader program
    //! \param p_vertex_source Vertex shader source
    //! \param p_fragment_source Fragment shader source
    //! \return Program ID
    // ------------------------------------------------------------------------
    unsigned int
    createShaderProgram(const std::string& p_vertex_source,
                        const std::string& p_fragment_source) const;

    // ------------------------------------------------------------------------
    //! \brief Render grid
    // ------------------------------------------------------------------------
    void renderGrid() const;

    // ------------------------------------------------------------------------
    //! \brief Render box
    //! \param p_transform Transformation matrix
    //! \param p_color Color
    // ------------------------------------------------------------------------
    void renderBox(const Transform& p_transform,
                   const Eigen::Vector3f& p_color =
                       Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render cylinder
    //! \param p_transform Transformation matrix
    //! \param p_color Color
    // ------------------------------------------------------------------------
    void renderCylinder(const Transform& p_transform,
                        const Eigen::Vector3f& p_color =
                            Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render sphere
    //! \param p_transform Transformation matrix
    //! \param p_color Color
    // ------------------------------------------------------------------------
    void renderSphere(const Transform& p_transform,
                      const Eigen::Vector3f& p_color =
                          Eigen::Vector3f(0.8f, 0.8f, 0.8f)) const;

    // ------------------------------------------------------------------------
    //! \brief Render robot arm
    // ------------------------------------------------------------------------
    void renderRobot() const;

    // ------------------------------------------------------------------------
    //! \brief Update camera matrices
    // ------------------------------------------------------------------------
    void updateCamera();

    // ------------------------------------------------------------------------
    //! \brief Setup camera for specific view
    //! \param p_view_type Camera view
    //! \param p_camera_target Camera target position
    // ------------------------------------------------------------------------
    void setupCameraView(CameraViewType p_view_type,
                         const Eigen::Vector3f& p_camera_target);

private:

    // Window properties
    int m_width;
    int m_height;
    std::string m_title;
    GLFWwindow* m_window = nullptr;

    // Shaders
    unsigned int m_shader_program = 0;

    // Geometry buffers
    unsigned int m_box_vao = 0;
    unsigned int m_box_vbo = 0;
    unsigned int m_box_ebo = 0;
    unsigned int m_cylinder_vao = 0;
    unsigned int m_cylinder_vbo = 0;
    unsigned int m_cylinder_ebo = 0;
    unsigned int m_sphere_vao = 0;
    unsigned int m_sphere_vbo = 0;
    unsigned int m_sphere_ebo = 0;
    unsigned int m_grid_vao = 0;
    unsigned int m_grid_vbo = 0;

    // Camera
    Eigen::Vector3f m_camera_pos = Eigen::Vector3f(8.0f, 3.0f, 8.0f);
    Eigen::Vector3f m_camera_target = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
    float m_fov = 45.0f;
    float m_aspect_ratio = 1.0f;
    float m_near_plane = 0.1f;
    float m_far_plane = 100.0f;

    // Matrices
    Eigen::Matrix4f m_view_matrix;
    Eigen::Matrix4f m_projection_matrix;

    // Robot
    const RobotArm* m_robot = nullptr;

    // Shader sources
    static const std::string s_vertex_shader_source;
    static const std::string s_fragment_shader_source;
};

} // namespace robotik