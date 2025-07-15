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
out vec3 Normal;
out vec3 FragPos;

void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);

    // Transform normal to world space
    Normal = mat3(transpose(inverse(model))) * aNormal;

    // Fragment position in world space
    FragPos = vec3(model * vec4(aPos, 1.0));

    FragColor = color;
}
)";

const std::string Viewer::s_fragment_shader_source = R"(
#version 330 core
in vec3 FragColor;
in vec3 Normal;
in vec3 FragPos;

out vec4 outColor;

void main()
{
    // Simple directional light
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 lightColor = vec3(1.0, 1.0, 1.0);

    // Normalize the normal
    vec3 norm = normalize(Normal);

    // Ambient lighting
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse lighting
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Combine lighting
    vec3 result = (ambient + diffuse) * FragColor;
    outColor = vec4(result, 1.0);
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
bool Viewer::shouldClose() const
{
    return glfwWindowShouldClose(m_window);
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
bool Viewer::initializeGeometry()
{
    // Initialize all geometry types using the dedicated functions
    initializeBox();
    initializeGrid();
    initializeCylinder();
    initializeSphere();

    return true;
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
    Eigen::Matrix4f model = p_transform.cast<float>();
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "model"),
                       1,
                       GL_FALSE,
                       model.data());
    glUniform3fv(
        glGetUniformLocation(m_shader_program, "color"), 1, p_color.data());

    glBindVertexArray(m_cylinder_vao);
    glDrawElements(GL_TRIANGLES,
                   GLsizei(m_cylinder_index_count),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void Viewer::renderSphere(const Transform& p_transform,
                          const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = p_transform.cast<float>();
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "model"),
                       1,
                       GL_FALSE,
                       model.data());
    glUniform3fv(
        glGetUniformLocation(m_shader_program, "color"), 1, p_color.data());

    glBindVertexArray(m_sphere_vao);
    glDrawElements(
        GL_TRIANGLES, GLsizei(m_sphere_index_count), GL_UNSIGNED_INT, nullptr);
}

// ----------------------------------------------------------------------------
void Viewer::renderJoint(Joint const& joint,
                         const Transform& world_transform) const
{
    Transform joint_transform = world_transform;

    switch (joint.getType())
    {
        case Joint::Type::REVOLUTE:
        {
            // Render revolute joint as a cylinder (RED)
            joint_transform.block<3, 3>(0, 0) *= 0.05;
            renderCylinder(
                joint_transform,
                Eigen::Vector3f(1.0f, 0.0f, 0.0f)); // Red for revolute joints
            break;
        }
        case Joint::Type::PRISMATIC:
        {
            // Render prismatic joint as a box (GREEN)
            joint_transform.block<3, 3>(0, 0) *= 0.05;
            renderBox(joint_transform,
                      Eigen::Vector3f(
                          0.0f, 1.0f, 0.0f)); // Green for prismatic joints
            break;
        }
        case Joint::Type::FIXED:
        default:
        {
            // Don't render fixed joints
            break;
        }
    }
}

// ----------------------------------------------------------------------------
void Viewer::renderLink(Link const& link,
                        const Transform& world_transform) const
{
    // Apply visual origin transformation
    Transform link_transform = world_transform * link.geometry.visual_origin;

    // Get color from geometry (RGB only, ignore alpha)
    Eigen::Vector3f color = link.geometry.color.head<3>().cast<float>();

    // Check if link has geometry information
    if (!link.geometry.parameters.empty())
    {
        // Use geometry type and parameters for rendering
        switch (link.geometry.type)
        {
            case Geometry::Type::BOX:
            {
                // Scale based on geometry parameters (width, height, depth)
                if (link.geometry.parameters.size() >= 3)
                {
                    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
                    scale(0, 0) = link.geometry.parameters[0];
                    scale(1, 1) = link.geometry.parameters[1];
                    scale(2, 2) = link.geometry.parameters[2];
                    link_transform.block<3, 3>(0, 0) =
                        link_transform.block<3, 3>(0, 0) * scale;
                }
                renderBox(link_transform, color);
                break;
            }
            case Geometry::Type::CYLINDER:
            {
                // Scale based on geometry parameters (radius, height)
                if (link.geometry.parameters.size() >= 2)
                {
                    double radius = link.geometry.parameters[0];
                    double height = link.geometry.parameters[1];
                    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
                    scale(0, 0) = radius * 2;
                    scale(1, 1) = height;
                    scale(2, 2) = radius * 2;
                    link_transform.block<3, 3>(0, 0) =
                        link_transform.block<3, 3>(0, 0) * scale;
                }
                renderCylinder(link_transform, color);
                break;
            }
            case Geometry::Type::SPHERE:
            {
                // Scale based on geometry parameters (radius)
                if (link.geometry.parameters.size() >= 1)
                {
                    double radius = link.geometry.parameters[0];
                    Eigen::Matrix3d scale = Eigen::Matrix3d::Identity();
                    scale(0, 0) = radius * 2;
                    scale(1, 1) = radius * 2;
                    scale(2, 2) = radius * 2;
                    link_transform.block<3, 3>(0, 0) =
                        link_transform.block<3, 3>(0, 0) * scale;
                }
                renderSphere(link_transform, color);
                break;
            }
            case Geometry::Type::MESH:
            {
                // TODO: Implement mesh rendering
                break;
            }
            default:
                // Default to box for unknown geometry
                link_transform.block<3, 3>(0, 0) *= 0.1;
                renderBox(link_transform, color);
                break;
        }
    }
    else
    {
        // No geometry information, render as default box
        link_transform.block<3, 3>(0, 0) *= 0.1;
        renderBox(link_transform, color);
    }
}

// ----------------------------------------------------------------------------
void Viewer::renderRobot() const
{
    if (!m_robot)
    {
        return;
    }

    // Traverse the robot tree and render each node
    m_robot->traverseNodes(
        [this](Node& node)
        {
            // Get the world transform
            Transform world_transform = node.getWorldTransform();

            // Render based on node type
            if (auto joint = dynamic_cast<Joint*>(&node))
            {
                renderJoint(*joint, world_transform);
            }
            else
            {
                // Try to find a corresponding Link object
                Link const* link = m_robot->getLink(node.getName());
                if (link)
                {
                    renderLink(*link, world_transform);
                }
                else
                {
                    // Fallback for unknown node types
                    Transform node_transform = world_transform;
                    node_transform.block<3, 3>(0, 0) *= 0.1;
                    renderBox(
                        node_transform,
                        Eigen::Vector3f(0.5f, 0.5f, 0.5f)); // Gray for unknown
                }
            }
        });
}

// ----------------------------------------------------------------------------
void Viewer::updateCamera()
{
    // Standard lookAt matrix construction
    Eigen::Vector3f f = (m_camera_target - m_camera_pos).normalized();
    Eigen::Vector3f s = f.cross(m_camera_up).normalized();
    Eigen::Vector3f u = s.cross(f);

    m_view_matrix = Eigen::Matrix4f::Identity();

    // Fill the view matrix in column-major order (OpenGL standard)
    m_view_matrix(0, 0) = s.x();
    m_view_matrix(1, 0) = u.x();
    m_view_matrix(2, 0) = -f.x();
    m_view_matrix(3, 0) = 0.0f;

    m_view_matrix(0, 1) = s.y();
    m_view_matrix(1, 1) = u.y();
    m_view_matrix(2, 1) = -f.y();
    m_view_matrix(3, 1) = 0.0f;

    m_view_matrix(0, 2) = s.z();
    m_view_matrix(1, 2) = u.z();
    m_view_matrix(2, 2) = -f.z();
    m_view_matrix(3, 2) = 0.0f;

    m_view_matrix(0, 3) = -s.dot(m_camera_pos);
    m_view_matrix(1, 3) = -u.dot(m_camera_pos);
    m_view_matrix(2, 3) = f.dot(m_camera_pos);
    m_view_matrix(3, 3) = 1.0f;

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
void Viewer::generateBox(std::vector<float>& vertices,
                         std::vector<unsigned int>& indices) const
{
    vertices.clear();
    indices.clear();

    // Box vertices with positions and normals
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

    // Copy vertices data
    vertices.assign(box_vertices,
                    box_vertices + sizeof(box_vertices) / sizeof(float));

    // Box indices
    unsigned int box_indices[] = {
        0,  1,  2,  2,  3,  0,  // front
        4,  5,  6,  6,  7,  4,  // back
        8,  9,  10, 10, 11, 8,  // left
        12, 13, 14, 14, 15, 12, // right
        16, 17, 18, 18, 19, 16, // bottom
        20, 21, 22, 22, 23, 20  // top
    };

    // Copy indices data
    indices.assign(box_indices,
                   box_indices + sizeof(box_indices) / sizeof(unsigned int));
}

// ----------------------------------------------------------------------------
void Viewer::generateCylinder(std::vector<float>& vertices,
                              std::vector<unsigned int>& indices,
                              float radius,
                              float height,
                              size_t segments) const
{
    vertices.clear();
    indices.clear();

    // Generate vertices
    // Bottom center
    vertices.insert(vertices.end(),
                    { 0.0f, -height / 2, 0.0f, 0.0f, -1.0f, 0.0f });
    // Top center
    vertices.insert(vertices.end(),
                    { 0.0f, height / 2, 0.0f, 0.0f, 1.0f, 0.0f });

    // Bottom and top circles
    for (size_t i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(segments);
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);

        // Bottom circle vertex
        vertices.insert(vertices.end(),
                        { x, -height / 2, z, 0.0f, -1.0f, 0.0f });
        // Top circle vertex
        vertices.insert(vertices.end(), { x, height / 2, z, 0.0f, 1.0f, 0.0f });
    }

    // Side vertices with side normals
    for (size_t i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(segments);
        float x = radius * std::cos(angle);
        float z = radius * std::sin(angle);

        // Side vertices with side normals
        float nx = std::cos(angle);
        float nz = std::sin(angle);
        vertices.insert(vertices.end(), { x, -height / 2, z, nx, 0.0f, nz });
        vertices.insert(vertices.end(), { x, height / 2, z, nx, 0.0f, nz });
    }

    // Generate indices
    // Bottom cap
    for (size_t i = 0; i < segments; ++i)
    {
        indices.insert(
            indices.end(),
            { 0u,
              static_cast<unsigned int>(2 + i * 2),
              static_cast<unsigned int>(2 + ((i + 1) % (segments + 1)) * 2) });
    }

    // Top cap
    for (size_t i = 0; i < segments; ++i)
    {
        indices.insert(
            indices.end(),
            { 1u,
              static_cast<unsigned int>(3 + ((i + 1) % (segments + 1)) * 2),
              static_cast<unsigned int>(3 + i * 2) });
    }

    // Side faces
    size_t side_offset = 2 + (segments + 1) * 2;
    for (size_t i = 0; i < segments; ++i)
    {
        size_t curr = side_offset + i * 2;
        size_t next = side_offset + ((i + 1) % (segments + 1)) * 2;

        // Two triangles per side face
        indices.insert(indices.end(),
                       { static_cast<unsigned int>(curr),
                         static_cast<unsigned int>(next),
                         static_cast<unsigned int>(curr + 1) });
        indices.insert(indices.end(),
                       { static_cast<unsigned int>(next),
                         static_cast<unsigned int>(next + 1),
                         static_cast<unsigned int>(curr + 1) });
    }
}

// ----------------------------------------------------------------------------
void Viewer::generateSphere(std::vector<float>& vertices,
                            std::vector<unsigned int>& indices,
                            float radius,
                            size_t latitude_segments,
                            size_t longitude_segments) const
{
    vertices.clear();
    indices.clear();

    // Generate vertices
    for (size_t lat = 0; lat <= latitude_segments; ++lat)
    {
        float theta = M_PIf * float(lat) / float(latitude_segments);
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (size_t lon = 0; lon <= longitude_segments; ++lon)
        {
            float phi = 2.0f * M_PIf * float(lon) / float(longitude_segments);
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            float x = sin_theta * cos_phi;
            float y = cos_theta;
            float z = sin_theta * sin_phi;

            // Position
            vertices.push_back(radius * x);
            vertices.push_back(radius * y);
            vertices.push_back(radius * z);

            // Normal (same as normalized position for sphere)
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    // Generate indices
    for (size_t lat = 0; lat < latitude_segments; ++lat)
    {
        for (size_t lon = 0; lon < longitude_segments; ++lon)
        {
            size_t first = lat * (longitude_segments + 1) + lon;
            size_t second = first + longitude_segments + 1;

            // Two triangles per quad
            indices.insert(indices.end(),
                           { static_cast<unsigned int>(first),
                             static_cast<unsigned int>(second),
                             static_cast<unsigned int>(first + 1) });
            indices.insert(indices.end(),
                           { static_cast<unsigned int>(second),
                             static_cast<unsigned int>(second + 1),
                             static_cast<unsigned int>(first + 1) });
        }
    }
}

// ----------------------------------------------------------------------------
void Viewer::initializeBox()
{
    std::vector<float> box_vertices;
    std::vector<unsigned int> box_indices;
    generateBox(box_vertices, box_indices);

    glGenVertexArrays(1, &m_box_vao);
    glGenBuffers(1, &m_box_vbo);
    glGenBuffers(1, &m_box_ebo);

    glBindVertexArray(m_box_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_box_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 box_vertices.size() * sizeof(float),
                 box_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_box_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 box_indices.size() * sizeof(unsigned int),
                 box_indices.data(),
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
}

// ----------------------------------------------------------------------------
void Viewer::initializeCylinder()
{
    std::vector<float> cylinder_vertices;
    std::vector<unsigned int> cylinder_indices;
    generateCylinder(cylinder_vertices, cylinder_indices, 1.0f, 2.0f, 16);

    glGenVertexArrays(1, &m_cylinder_vao);
    glGenBuffers(1, &m_cylinder_vbo);
    glGenBuffers(1, &m_cylinder_ebo);

    glBindVertexArray(m_cylinder_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_cylinder_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 cylinder_vertices.size() * sizeof(float),
                 cylinder_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cylinder_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 cylinder_indices.size() * sizeof(unsigned int),
                 cylinder_indices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    m_cylinder_index_count = cylinder_indices.size();
}

// ----------------------------------------------------------------------------
void Viewer::initializeSphere()
{
    std::vector<float> sphere_vertices;
    std::vector<unsigned int> sphere_indices;
    generateSphere(sphere_vertices, sphere_indices, 1.0f, 16, 16);

    glGenVertexArrays(1, &m_sphere_vao);
    glGenBuffers(1, &m_sphere_vbo);
    glGenBuffers(1, &m_sphere_ebo);

    glBindVertexArray(m_sphere_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_sphere_vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 sphere_vertices.size() * sizeof(float),
                 sphere_vertices.data(),
                 GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_sphere_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 sphere_indices.size() * sizeof(unsigned int),
                 sphere_indices.data(),
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    m_sphere_index_count = sphere_indices.size();
}

// ----------------------------------------------------------------------------
void Viewer::initializeGrid()
{
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

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
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
            camera_distance = 5.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(0.0f, camera_distance, 0.0f);
            m_camera_up = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
            break;
        case CameraViewType::FRONT:
            camera_distance = 5.0f;
            m_camera_pos =
                p_camera_target + Eigen::Vector3f(0.0f, 0.0f, camera_distance);
            m_camera_up = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
            break;
        case CameraViewType::SIDE:
            camera_distance = 5.0f;
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
void Viewer::processInput(std::function<void(int, int)> key_callback)
{
    glfwPollEvents();

    if (key_callback)
    {
        // Check for camera view keys
        if (glfwGetKey(m_window, GLFW_KEY_1) == GLFW_PRESS)
            key_callback(GLFW_KEY_1, GLFW_PRESS);
        if (glfwGetKey(m_window, GLFW_KEY_2) == GLFW_PRESS)
            key_callback(GLFW_KEY_2, GLFW_PRESS);
        if (glfwGetKey(m_window, GLFW_KEY_3) == GLFW_PRESS)
            key_callback(GLFW_KEY_3, GLFW_PRESS);
        if (glfwGetKey(m_window, GLFW_KEY_4) == GLFW_PRESS)
            key_callback(GLFW_KEY_4, GLFW_PRESS);
        if (glfwGetKey(m_window, GLFW_KEY_5) == GLFW_PRESS)
            key_callback(GLFW_KEY_5, GLFW_PRESS);
    }
}

} // namespace robotik