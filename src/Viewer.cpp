#include "Robotik/Viewer.hpp"
#include <cmath>
#include <iostream>

namespace robotik
{

// Shader sources
const std::string Viewer::s_vertex_shader_source = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
uniform vec3 color;

out vec3 FragColor;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    FragColor = color;
}
)";

const std::string Viewer::s_fragment_shader_source = R"(
#version 330 core
in vec3 FragColor;
out vec4 outColor;

void main()
{
    outColor = vec4(FragColor, 1.0);
}
)";

// ----------------------------------------------------------------------------
Viewer::Viewer(int p_width, int p_height, const std::string& p_title)
    : m_width(p_width), m_height(p_height), m_title(p_title)
{
    m_aspect_ratio = static_cast<float>(p_width) / static_cast<float>(p_height);
}

// ----------------------------------------------------------------------------
Viewer::~Viewer()
{
    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// ----------------------------------------------------------------------------
bool Viewer::initialize()
{
    if (!initializeGL())
    {
        return false;
    }

    if (!initializeShaders())
    {
        return false;
    }

    if (!initializeGeometry())
    {
        return false;
    }

    // Initialize camera matrices with default values
    updateCamera();

    return true;
}

// ----------------------------------------------------------------------------
bool Viewer::initializeGL()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Mac OS X
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

    m_window =
        glfwCreateWindow(m_width, m_height, m_title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        glfwTerminate();
        return false;
    }

    // Make sure OpenGL version 3.3 API is available
    if (!GLEW_VERSION_3_3)
    {
        std::cerr << "OpenGL 3.3 not supported" << std::endl;
        glfwTerminate();
        return false;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    return true;
}

// ----------------------------------------------------------------------------
bool Viewer::initializeShaders()
{
    m_shader_program =
        createShaderProgram(s_vertex_shader_source, s_fragment_shader_source);
    if (m_shader_program == 0)
    {
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------------
bool Viewer::initializeGeometry()
{
    // Create box geometry
    float box_vertices[] = {
        // positions          // normals
        -0.5f, -0.5f, -0.5f, 0.0f,  0.0f,  -1.0f, 0.5f,  -0.5f, -0.5f, 0.0f,
        0.0f,  -1.0f, 0.5f,  0.5f,  -0.5f, 0.0f,  0.0f,  -1.0f, -0.5f, 0.5f,
        -0.5f, 0.0f,  0.0f,  -1.0f, -0.5f, -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,
        0.5f,  -0.5f, 0.5f,  0.0f,  0.0f,  1.0f,  0.5f,  0.5f,  0.5f,  0.0f,
        0.0f,  1.0f,  -0.5f, 0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  -0.5f, -0.5f,
        -0.5f, -1.0f, 0.0f,  0.0f,  -0.5f, 0.5f,  -0.5f, -1.0f, 0.0f,  0.0f,
        -0.5f, 0.5f,  0.5f,  -1.0f, 0.0f,  0.0f,  -0.5f, -0.5f, 0.5f,  -1.0f,
        0.0f,  0.0f,  0.5f,  -0.5f, -0.5f, 1.0f,  0.0f,  0.0f,  0.5f,  0.5f,
        -0.5f, 1.0f,  0.0f,  0.0f,  0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,
        0.5f,  -0.5f, 0.5f,  1.0f,  0.0f,  0.0f,  -0.5f, -0.5f, -0.5f, 0.0f,
        -1.0f, 0.0f,  0.5f,  -0.5f, -0.5f, 0.0f,  -1.0f, 0.0f,  0.5f,  -0.5f,
        0.5f,  0.0f,  -1.0f, 0.0f,  -0.5f, -0.5f, 0.5f,  0.0f,  -1.0f, 0.0f,
        -0.5f, 0.5f,  -0.5f, 0.0f,  1.0f,  0.0f,  0.5f,  0.5f,  -0.5f, 0.0f,
        1.0f,  0.0f,  0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  -0.5f, 0.5f,
        0.5f,  0.0f,  1.0f,  0.0f
    };

    unsigned int box_indices[] = {
        0,  1,  2,  2,  3,  0,  // front
        4,  5,  6,  6,  7,  4,  // back
        8,  9,  10, 10, 11, 8,  // left
        12, 13, 14, 14, 15, 12, // right
        16, 17, 18, 18, 19, 16, // bottom
        20, 21, 22, 22, 23, 20  // top
    };

    glGenVertexArrays(1, &m_box_vao);
    glGenBuffers(1, &m_box_vbo);
    glGenBuffers(1, &m_box_ebo);

    glBindVertexArray(m_box_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_box_vbo);
    glBufferData(
        GL_ARRAY_BUFFER, sizeof(box_vertices), box_vertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_box_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sizeof(box_indices),
                 box_indices,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Create grid geometry
    std::vector<float> grid_vertices;
    const int grid_size = 20;
    const float grid_spacing = 1.0f;

    for (int i = -grid_size; i <= grid_size; ++i)
    {
        // Lines parallel to X axis
        grid_vertices.push_back(-grid_size * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(float(i) * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(1.0f);
        grid_vertices.push_back(0.0f);

        grid_vertices.push_back(grid_size * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(float(i) * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(1.0f);
        grid_vertices.push_back(0.0f);

        // Lines parallel to Z axis
        grid_vertices.push_back(float(i) * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(-grid_size * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(1.0f);
        grid_vertices.push_back(0.0f);

        grid_vertices.push_back(float(i) * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(grid_size * grid_spacing);
        grid_vertices.push_back(0.0f);
        grid_vertices.push_back(1.0f);
        grid_vertices.push_back(0.0f);
    }

    glGenVertexArrays(1, &m_grid_vao);
    glGenBuffers(1, &m_grid_vbo);

    glBindVertexArray(m_grid_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_grid_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 grid_vertices.size() * sizeof(float),
                 grid_vertices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Create simple cylinder (approximated with box for simplicity)
    m_cylinder_vao = m_box_vao; // Reuse box VAO for now
    m_cylinder_vbo = m_box_vbo;
    m_cylinder_ebo = m_box_ebo;

    // Create simple sphere (approximated with box for simplicity)
    m_sphere_vao = m_box_vao; // Reuse box VAO for now
    m_sphere_vbo = m_box_vbo;
    m_sphere_ebo = m_box_ebo;

    return true;
}

// ----------------------------------------------------------------------------
unsigned int Viewer::compileShader(const std::string& p_source,
                                   unsigned int p_type) const
{
    unsigned int shader = glCreateShader(p_type);
    const char* source = p_source.c_str();
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(shader, 512, nullptr, info_log);
        std::cerr << "Shader compilation error: " << info_log << std::endl;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

// ----------------------------------------------------------------------------
unsigned int
Viewer::createShaderProgram(const std::string& p_vertexSource,
                            const std::string& p_fragmentSource) const
{
    unsigned int vertex_shader =
        compileShader(p_vertexSource, GL_VERTEX_SHADER);
    if (vertex_shader == 0)
    {
        return 0;
    }

    unsigned int fragment_shader =
        compileShader(p_fragmentSource, GL_FRAGMENT_SHADER);
    if (fragment_shader == 0)
    {
        glDeleteShader(vertex_shader);
        return 0;
    }

    unsigned int program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    int success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetProgramInfoLog(program, 512, nullptr, info_log);
        std::cerr << "Program linking error: " << info_log << std::endl;
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
        glDeleteProgram(program);
        return 0;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

// ----------------------------------------------------------------------------
void Viewer::setRobot(const RobotArm& p_robot)
{
    m_robot = &p_robot;
}

// ----------------------------------------------------------------------------
void Viewer::setCameraView(CameraViewType p_view,
                           const Eigen::Vector3f& p_camera_target)
{
    setupCameraView(p_view, p_camera_target);
    updateCamera();
}

// ----------------------------------------------------------------------------
void Viewer::setCameraView(CameraViewType p_view,
                           const Eigen::Vector3d& p_camera_target)
{
    setupCameraView(p_view, p_camera_target.cast<float>());
    updateCamera();
}

// ----------------------------------------------------------------------------
void Viewer::render()
{
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(m_shader_program);

    // Set projection matrix
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "projection"),
                       1,
                       GL_FALSE,
                       m_projection_matrix.data());

    // Set view matrix
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "view"),
                       1,
                       GL_FALSE,
                       m_view_matrix.data());

    // Render grid
    renderGrid();

    // Render robot if set
    if (m_robot)
    {
        renderRobot();
    }

    glfwSwapBuffers(m_window);
}

// ----------------------------------------------------------------------------
void Viewer::renderGrid() const
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "model"),
                       1,
                       GL_FALSE,
                       model.data());

    Eigen::Vector3f grid_color(0.7f, 0.7f, 0.7f);
    glUniform3fv(
        glGetUniformLocation(m_shader_program, "color"), 1, grid_color.data());

    glBindVertexArray(m_grid_vao);
    glDrawArrays(GL_LINES, 0,
                 164); // 41 lines * 4 vertices = 164 vertices
}

// ----------------------------------------------------------------------------
void Viewer::renderBox(const Transform& p_transform,
                       const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = p_transform.cast<float>();
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "model"),
                       1,
                       GL_FALSE,
                       model.data());
    glUniform3fv(
        glGetUniformLocation(m_shader_program, "color"), 1, p_color.data());

    glBindVertexArray(m_box_vao);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

// ----------------------------------------------------------------------------
void Viewer::renderCylinder(const Transform& p_transform,
                            const Eigen::Vector3f& p_color) const
{
    // For simplicity, render as box for now
    renderBox(p_transform, p_color);
}

// ----------------------------------------------------------------------------
void Viewer::renderSphere(const Transform& p_transform,
                          const Eigen::Vector3f& p_color) const
{
    // For simplicity, render as box for now
    renderBox(p_transform, p_color);
}

// ----------------------------------------------------------------------------
void Viewer::renderRobot() const
{
    std::cout << "Rendering robot: " << std::endl;
    if (!m_robot)
    {
        return;
    }

    // Traverse the robot tree and render each node
    m_robot->traverseNodes(
        [this](Node& node)
        {
            std::cout << "  " << node.getName() << std::endl;

            // Get the world transform
            Transform world_transform = node.getWorldTransform();

            // Render based on node type
            if (dynamic_cast<Joint*>(&node))
            {
                // Render joint as a small sphere
                Transform joint_transform = world_transform;
                // Scale down the joint representation
                joint_transform.block<3, 3>(0, 0) *= 0.05;
                renderSphere(joint_transform,
                             Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            }
            else
            {
                // Render link as a box
                Transform link_transform = world_transform;
                // Scale the link to be more visible
                link_transform.block<3, 3>(0, 0) *= 0.1;
                renderBox(link_transform, Eigen::Vector3f(0.2f, 0.6f, 0.8f));
            }
        });
}

// ----------------------------------------------------------------------------
void Viewer::updateCamera()
{
    // Update view matrix
    m_view_matrix = Eigen::Matrix4f::Identity();
    Eigen::Vector3f z = (m_camera_pos - m_camera_target).normalized();
    Eigen::Vector3f x = m_camera_up.cross(z).normalized();
    Eigen::Vector3f y = z.cross(x);

    m_view_matrix.block<3, 1>(0, 0) = x;
    m_view_matrix.block<3, 1>(0, 1) = y;
    m_view_matrix.block<3, 1>(0, 2) = z;
    m_view_matrix.block<3, 1>(0, 3) =
        -m_view_matrix.block<3, 3>(0, 0) * m_camera_pos;

    // Update projection matrix
    float tan_half_fov = std::tan(m_fov * 0.5f * float(M_PI) / 180.0f);
    m_projection_matrix = Eigen::Matrix4f::Zero();
    m_projection_matrix(0, 0) = 1.0f / (m_aspect_ratio * tan_half_fov);
    m_projection_matrix(1, 1) = 1.0f / tan_half_fov;
    m_projection_matrix(2, 2) =
        -(m_far_plane + m_near_plane) / (m_far_plane - m_near_plane);
    m_projection_matrix(2, 3) =
        -2.0f * m_far_plane * m_near_plane / (m_far_plane - m_near_plane);
    m_projection_matrix(3, 2) = -1.0f;
}

// ----------------------------------------------------------------------------
void Viewer::setupCameraView(CameraViewType p_view_type,
                             const Eigen::Vector3f& p_camera_target)
{
    // Set the camera target
    m_camera_target = p_camera_target;

    // Calculate appropriate camera distance based on view type
    float camera_distance;

    switch (p_view_type)
    {
        case CameraViewType::PERSPECTIVE:
            camera_distance = 5.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(camera_distance * 0.4f,
                                                  camera_distance * 0.4f,
                                                  camera_distance * 0.4f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case CameraViewType::TOP:
            camera_distance = 10.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(0.0f, camera_distance, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            break;
        case CameraViewType::FRONT:
            camera_distance = 10.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(0.0f, 0.0f, camera_distance);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case CameraViewType::SIDE:
            camera_distance = 10.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(camera_distance, 0.0f, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case CameraViewType::ISOMETRIC:
            camera_distance = 8.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(camera_distance * 0.577f,
                                                  camera_distance * 0.577f,
                                                  camera_distance * 0.577f);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
    }

    // Ensure camera distance is within clipping planes
    float actual_distance = (m_camera_pos - m_camera_target).norm();
    if (actual_distance <= m_near_plane)
    {
        // Move camera further away if too close
        Eigen::Vector3f direction =
            (m_camera_pos - m_camera_target).normalized();
        m_camera_pos = m_camera_target + direction * (m_near_plane * 2.0f);
    }
    else if (actual_distance >= m_far_plane)
    {
        // Move camera closer if too far
        Eigen::Vector3f direction =
            (m_camera_pos - m_camera_target).normalized();
        m_camera_pos = m_camera_target + direction * (m_far_plane * 0.8f);
    }
}

// ----------------------------------------------------------------------------
bool Viewer::shouldClose() const
{
    return glfwWindowShouldClose(m_window);
}

// ----------------------------------------------------------------------------
void Viewer::processInput()
{
    glfwPollEvents();
}

} // namespace robotik