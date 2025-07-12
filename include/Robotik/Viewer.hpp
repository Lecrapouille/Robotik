#pragma once

#include "Robotik.hpp"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <memory>
#include <vector>

namespace robotik
{

//! \brief Structure to hold vertex data for OpenGL rendering
struct Vertex
{
    float position[3];
    float normal[3];
    float color[3];
};

//! \brief Camera controller for 3D navigation
class Camera
{
public:

    Camera();

    void setPosition(const Eigen::Vector3f& position);
    void setTarget(const Eigen::Vector3f& target);
    void setUpVector(const Eigen::Vector3f& up);

    Eigen::Matrix4f getViewMatrix() const;
    Eigen::Matrix4f getProjectionMatrix(float aspect_ratio) const;

    void processInput(GLFWwindow* window, float deltaTime);
    void processMouse(double xpos, double ypos);
    void processScroll(double yoffset);

private:

    Eigen::Vector3f m_position;
    Eigen::Vector3f m_target;
    Eigen::Vector3f m_up;
    Eigen::Vector3f m_front;

    float m_yaw;
    float m_pitch;
    float m_distance;
    float m_sensitivity;
    float m_speed;

    bool m_first_mouse;
    float m_last_x;
    float m_last_y;

    void updateVectors();
};

//! \brief OpenGL mesh for rendering robot components
class Mesh
{
public:

    Mesh();
    ~Mesh();

    void loadData(const std::vector<Vertex>& vertices,
                  const std::vector<unsigned int>& indices);
    void render() const;
    void renderLines() const;
    void cleanup();

private:

    GLuint m_VAO;
    GLuint m_VBO;
    GLuint m_EBO;
    size_t m_index_count;
    bool m_initialized;
};

//! \brief Shader program manager
class ShaderProgram
{
public:

    ShaderProgram();
    ~ShaderProgram();

    bool loadShaders(const std::string& vertex_source,
                     const std::string& fragment_source);
    void use() const;
    void setUniform(const std::string& name,
                    const Eigen::Matrix4f& matrix) const;
    void setUniform(const std::string& name,
                    const Eigen::Vector3f& vector) const;
    void setUniform(const std::string& name, float value) const;
    void cleanup();

private:

    GLuint m_program;
    GLuint compileShader(const std::string& source, GLenum type);
    bool linkProgram(GLuint vertex_shader, GLuint fragment_shader);
};

//! \brief OpenGL Core robot viewer
class RobotViewer
{
public:

    RobotViewer(int width = 1200,
                int height = 800,
                const std::string& title = "Robot Viewer");
    ~RobotViewer();

    // Initialization and cleanup
    bool initialize();
    void cleanup();

    // Main rendering loop
    void render(std::shared_ptr<RobotArm> robot);
    bool shouldClose() const;
    void pollEvents();
    void swapBuffers();

    // Robot visualization settings
    void setShowJoints(bool show)
    {
        m_show_joints = show;
    }
    void setShowLinks(bool show)
    {
        m_show_links = show;
    }
    void setShowCoordinateFrames(bool show)
    {
        m_show_frames = show;
    }
    void setJointRadius(float radius)
    {
        m_joint_radius = radius;
    }
    void setLinkRadius(float radius)
    {
        m_link_radius = radius;
    }

    // Camera control
    Camera& getCamera()
    {
        return m_camera;
    }

    // Window access
    GLFWwindow* getWindow()
    {
        return m_window;
    }

private:

    // Window management
    GLFWwindow* m_window;
    int m_width;
    int m_height;
    std::string m_title;

    // OpenGL objects
    ShaderProgram m_shader_program;
    Mesh m_sphere_mesh;
    Mesh m_cylinder_mesh;
    Mesh m_axis_mesh;

    // Camera
    Camera m_camera;

    // Rendering settings
    bool m_show_joints;
    bool m_show_links;
    bool m_show_frames;
    float m_joint_radius;
    float m_link_radius;

    // Timing
    float m_last_frame_time;
    float m_delta_time;

    // Static callbacks
    static void
    framebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void
    scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

    // Mesh generation
    void generateSphereMesh(float radius, int stacks, int slices);
    void generateCylinderMesh(float radius, float height, int slices);
    void generateAxisMesh(float length);

    // Rendering functions
    void renderRobot(std::shared_ptr<RobotArm> robot);
    void renderNode(std::shared_ptr<Node> node,
                    const Eigen::Matrix4f& parent_transform);
    void renderSphere(const Eigen::Matrix4f& transform,
                      const Eigen::Vector3f& color);
    void renderCylinder(const Eigen::Matrix4f& transform,
                        const Eigen::Vector3f& color);
    void renderAxis(const Eigen::Matrix4f& transform);

    // Utility functions
    Eigen::Matrix4f eigenToGLMatrix(const Transform& transform) const;
    std::string getDefaultVertexShader() const;
    std::string getDefaultFragmentShader() const;

    // Input handling
    void processInput();
};

} // namespace robotik