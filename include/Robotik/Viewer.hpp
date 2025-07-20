#pragma once

#include "Robot.hpp"
#include "Robotik/private/Link.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <functional>

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
    //! \brief Camera view positions.
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
    //! \brief Constructor.
    //! \param p_width Window width.
    //! \param p_height Window height.
    //! \param p_title Window title.
    // ------------------------------------------------------------------------
    Viewer(int p_width = 800,
           int p_height = 600,
           const std::string& p_title = "Robotik Viewer");

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~Viewer();

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
    void setCameraView(CameraViewType p_view_type,
                       const Eigen::Vector3f& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Set camera view.
    //! \param p_view_type Camera view position.
    //! \param p_camera_target Camera target position (default: origin).
    // ------------------------------------------------------------------------
    void setCameraView(CameraViewType p_view_type,
                       const Eigen::Vector3d& p_camera_target);

    // ------------------------------------------------------------------------
    //! \brief Render the scene.
    //! \param p_robot Robot arm to render.
    // ------------------------------------------------------------------------
    void render(Robot const& p_robot);

    // ------------------------------------------------------------------------
    //! \brief Check if window should close.
    //! \return true if window should close.
    // ------------------------------------------------------------------------
    bool shouldClose() const;

    // ------------------------------------------------------------------------
    //! \brief Process input events.
    //! \param p_key_callback Callback function for key events.
    // ------------------------------------------------------------------------
    void
    processInput(std::function<void(int, int)> const& p_key_callback = nullptr);

    // ------------------------------------------------------------------------
    //! \brief Get the GLFW window.
    //! \return Pointer to GLFW window.
    // ------------------------------------------------------------------------
    GLFWwindow* getWindow() const
    {
        return m_window;
    }

private:

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

    // ------------------------------------------------------------------------
    //! \brief Initialize geometry buffers.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initializeGeometry();

    // ------------------------------------------------------------------------
    //! \brief Generate box geometry vertices and indices.
    //! \param vertices Output vector for vertex data.
    //! \param indices Output vector for index data.
    // ------------------------------------------------------------------------
    void generateBox(std::vector<float>& vertices,
                     std::vector<unsigned int>& indices) const;

    // ------------------------------------------------------------------------
    //! \brief Initialize box geometry buffers.
    // ------------------------------------------------------------------------
    void initializeBox();

    // ------------------------------------------------------------------------
    //! \brief Initialize cylinder geometry buffers.
    // ------------------------------------------------------------------------
    void initializeCylinder();

    // ------------------------------------------------------------------------
    //! \brief Initialize sphere geometry buffers.
    // ------------------------------------------------------------------------
    void initializeSphere();

    // ------------------------------------------------------------------------
    //! \brief Initialize grid geometry buffers.
    // ------------------------------------------------------------------------
    void initializeGrid();

    // ------------------------------------------------------------------------
    //! \brief Initialize axes geometry buffers.
    // ------------------------------------------------------------------------

    // ------------------------------------------------------------------------
    //! \brief Generate cylinder geometry vertices and indices.
    //! \param vertices Output vector for vertex data.
    //! \param indices Output vector for index data.
    //! \param radius Cylinder radius.
    //! \param height Cylinder height.
    //! \param segments Number of segments around cylinder.
    // ------------------------------------------------------------------------
    void generateCylinder(std::vector<float>& vertices,
                          std::vector<unsigned int>& indices,
                          float radius,
                          float height,
                          size_t segments) const;

    // ------------------------------------------------------------------------
    //! \brief Generate sphere geometry vertices and indices.
    //! \param vertices Output vector for vertex data.
    //! \param indices Output vector for index data.
    //! \param radius Sphere radius.
    //! \param latitude_segments Number of latitude segments.
    //! \param longitude_segments Number of longitude segments.
    // ------------------------------------------------------------------------
    void generateSphere(std::vector<float>& vertices,
                        std::vector<unsigned int>& indices,
                        float radius,
                        size_t latitude_segments,
                        size_t longitude_segments) const;

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

    // ------------------------------------------------------------------------
    //! \brief Render coordinate axes (XYZ) in RGB colors.
    //! \param p_transform Transformation matrix for the axes origin.
    //! \param p_scale Scale factor for the axes length (default: 1.0).
    // ------------------------------------------------------------------------
    void renderAxes(Transform const& p_transform, double p_scale = 1.0) const;

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

    // Index counts for geometry
    size_t m_cylinder_index_count = 0;
    size_t m_sphere_index_count = 0;

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

    // Shader sources
    static const std::string s_vertex_shader_source;
    static const std::string s_fragment_shader_source;
};

} // namespace robotik