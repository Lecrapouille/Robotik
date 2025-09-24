#include "OpenGLViewer.hpp"
#include "STLLoader.hpp"

#include "Robotik/private/Path.hpp"

#include <cmath>
#include <iostream>

namespace robotik
{

// Shader sources
const std::string OpenGLViewer::s_vertex_shader_source = R"(
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

const std::string OpenGLViewer::s_fragment_shader_source = R"(
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

static const Eigen::Vector3f red_color(1.0f, 0.0f, 0.0f);
static const Eigen::Vector3f green_color(0.0f, 1.0f, 0.0f);

// ----------------------------------------------------------------------------
OpenGLViewer::OpenGLViewer(Path& p_path,
                           size_t p_width,
                           size_t p_height,
                           const std::string& p_title)
    : m_path(p_path), m_width(p_width), m_height(p_height), m_title(p_title)
{
    m_aspect_ratio = static_cast<float>(p_width) / static_cast<float>(p_height);

    // Initialize URDF to OpenGL coordinate system conversion matrix
    // URDF: X=forward, Y=left, Z=up
    // OpenGL: X=right, Y=up, Z=toward viewer
    // Transformation matrix:
    //   OpenGL_X = -URDF_Y  (left becomes right by negation)
    //   OpenGL_Y = URDF_Z   (up remains up)
    //   OpenGL_Z = -URDF_X  (forward becomes toward viewer by negation)
    m_urdf_to_opengl_matrix = Eigen::Matrix4f::Zero();
    m_urdf_to_opengl_matrix(0, 1) = -1.0f; // OpenGL_X = -URDF_Y
    m_urdf_to_opengl_matrix(1, 2) = 1.0f;  // OpenGL_Y = URDF_Z
    m_urdf_to_opengl_matrix(2, 0) = -1.0f; // OpenGL_Z = -URDF_X
    m_urdf_to_opengl_matrix(3, 3) = 1.0f;  // Homogeneous coordinate
}

// ----------------------------------------------------------------------------
OpenGLViewer::~OpenGLViewer()
{
    // Clean up mesh resources
    for (auto& pair : m_mesh_vaos)
    {
        glDeleteVertexArrays(1, &pair.second);
    }
    for (auto& pair : m_mesh_vbos)
    {
        glDeleteBuffers(1, &pair.second);
    }
    for (auto& pair : m_mesh_ebos)
    {
        glDeleteBuffers(1, &pair.second);
    }

    if (m_window)
    {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::shouldClose() const
{
    return glfwWindowShouldClose(m_window);
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::initialize()
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

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::initializeGL()
{
    if (!glfwInit())
    {
        m_error_message = "Failed to initialize GLFW";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Mac OS X
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing

    m_window = glfwCreateWindow(
        int(m_width), int(m_height), m_title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        m_error_message = "Failed to create GLFW window";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        m_error_message = "Failed to initialize GLEW";
        glfwTerminate();
        return false;
    }

    // Make sure OpenGL version 3.3 API is available
    if (!GLEW_VERSION_3_3)
    {
        m_error_message = "OpenGL 3.3 not supported";
        glfwTerminate();
        return false;
    }

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    return true;
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::initializeShaders()
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
unsigned int OpenGLViewer::compileShader(const std::string& p_source,
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
        // Get the length of the info log
        int info_log_length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_log_length);

        // Allocate buffer with exact size
        std::string info_log(info_log_length, '\0');
        glGetShaderInfoLog(shader, info_log_length, nullptr, &info_log[0]);

        m_error_message = std::string("Shader compilation error: ") + info_log;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

// ----------------------------------------------------------------------------
unsigned int
OpenGLViewer::createShaderProgram(const std::string& p_vertexSource,
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
        // Get the length of the info log
        int info_log_length;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &info_log_length);

        // Allocate buffer with exact size
        std::string info_log(info_log_length, '\0');
        glGetProgramInfoLog(program, info_log_length, nullptr, &info_log[0]);

        m_error_message = "Program linking error: " + info_log;
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
void OpenGLViewer::cameraView(CameraViewType p_view,
                              const Eigen::Vector3f& p_camera_target)
{
    setupCameraView(p_view, p_camera_target);
    updateCamera();
}

// ----------------------------------------------------------------------------
void OpenGLViewer::cameraView(CameraViewType p_view,
                              const Eigen::Vector3d& p_camera_target)
{
    setupCameraView(p_view, p_camera_target.cast<float>());
    updateCamera();
}

// ----------------------------------------------------------------------------
void OpenGLViewer::render(Robot const& p_robot)
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
    renderRobot(p_robot);

    glfwSwapBuffers(m_window);
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::initializeGeometry()
{
    initializeBox();
    initializeGrid();
    initializeCylinder();
    initializeSphere();

    return true;
}

// ----------------------------------------------------------------------------
void OpenGLViewer::renderGrid() const
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
void OpenGLViewer::renderBox(Transform const& p_transform,
                             const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
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
void OpenGLViewer::renderCylinder(Transform const& p_transform,
                                  const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
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
void OpenGLViewer::renderSphere(Transform const& p_transform,
                                const Eigen::Vector3f& p_color) const
{
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
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
void OpenGLViewer::renderMesh(Transform const& p_transform,
                              const std::string& p_mesh_path,
                              const Eigen::Vector3f& p_color) const
{
    if (p_mesh_path.empty())
        return;

    // Check if mesh is already loaded
    auto vao_it = m_mesh_vaos.find(p_mesh_path);
    if (vao_it == m_mesh_vaos.end())
    {
        // Load STL file
        STLLoader::MeshData mesh_data;
        if (!STLLoader::loadSTL(m_path.expand(p_mesh_path), mesh_data))
        {
            std::cerr << "Failed to load STL file: " << p_mesh_path << " - "
                      << STLLoader::getLastError() << std::endl;
            return;
        }

        if (mesh_data.vertices.empty() || mesh_data.indices.empty())
        {
            std::cerr << "STL file contains no geometry: " << p_mesh_path
                      << std::endl;
            return;
        }

        // Generate OpenGL buffers
        unsigned int vao, vbo, ebo;
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);

        // Upload vertex data
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER,
                     mesh_data.vertices.size() * sizeof(float),
                     mesh_data.vertices.data(),
                     GL_STATIC_DRAW);

        // Upload index data
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     mesh_data.indices.size() * sizeof(unsigned int),
                     mesh_data.indices.data(),
                     GL_STATIC_DRAW);

        // Set vertex attributes (position + normal)
        glVertexAttribPointer(
            0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1,
                              3,
                              GL_FLOAT,
                              GL_FALSE,
                              6 * sizeof(float),
                              (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        // Cache the mesh data
        m_mesh_vaos[p_mesh_path] = vao;
        m_mesh_vbos[p_mesh_path] = vbo;
        m_mesh_ebos[p_mesh_path] = ebo;
        m_mesh_index_counts[p_mesh_path] = mesh_data.indices.size();
    }

    // Render the mesh
    Eigen::Matrix4f model = m_urdf_to_opengl_matrix * p_transform.cast<float>();
    glUniformMatrix4fv(glGetUniformLocation(m_shader_program, "model"),
                       1,
                       GL_FALSE,
                       model.data());
    glUniform3fv(
        glGetUniformLocation(m_shader_program, "color"), 1, p_color.data());

    glBindVertexArray(m_mesh_vaos[p_mesh_path]);
    glDrawElements(GL_TRIANGLES,
                   GLsizei(m_mesh_index_counts[p_mesh_path]),
                   GL_UNSIGNED_INT,
                   nullptr);
}

// ----------------------------------------------------------------------------
void OpenGLViewer::renderJoint(Joint const& p_joint,
                               Transform const& p_world_transform) const
{
    // Scale the joint to 5% of the link
    double scaling = 0.025;

    // Create scaling transformation
    Transform scale_transform = Transform::Identity();
    scale_transform(0, 0) = scaling;
    scale_transform(1, 1) = scaling;
    scale_transform(2, 2) = scaling;

    // Apply scaling correctly: base_transform * scale
    Transform joint_transform = p_world_transform * scale_transform;

    switch (p_joint.type())
    {
        case Joint::Type::CONTINUOUS:
        case Joint::Type::REVOLUTE:
        {
            // Render revolute joint as a cylinder in red color
            renderCylinder(joint_transform, red_color);
            break;
        }
        case Joint::Type::PRISMATIC:
        {
            // Render prismatic joint as a box in green color
            renderBox(joint_transform, green_color);
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
void OpenGLViewer::renderLink(Link const& p_link,
                              Transform const& p_world_transform) const
{
    Geometry const& geometry = p_link.geometry();
    std::vector<double> const& parameters = geometry.parameters();

    // Use geometry type and parameters for rendering
    switch (geometry.type())
    {
        case Geometry::Type::BOX:
        {
            // Scale based on geometry parameters (width, height, depth)
            if (parameters.size() >= 3)
            {
                double width = parameters[0];
                double height = parameters[1];
                double depth = parameters[2];

                // Create scaling transformation
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = width;
                scale_transform(1, 1) = height;
                scale_transform(2, 2) = depth;
                Transform link_transform = p_world_transform * scale_transform;

                renderBox(link_transform, geometry.color);
            }
            else
            {
                renderBox(p_world_transform, geometry.color);
            }
            break;
        }
        case Geometry::Type::CYLINDER:
        {
            // Scale based on geometry parameters (radius, height)
            if (parameters.size() >= 2)
            {
                double radius = parameters[0];
                double height = parameters[1];

                // Create scaling transformation
                // Base cylinder is radius 1, height 2, oriented along Z-axis.
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = radius;
                scale_transform(1, 1) = radius;
                scale_transform(2, 2) = height / 2.0;
                Transform link_transform = p_world_transform * scale_transform;

                renderCylinder(link_transform, geometry.color);
            }
            else
            {
                renderCylinder(p_world_transform, geometry.color);
            }
            break;
        }
        case Geometry::Type::SPHERE:
        {
            // Scale based on geometry parameters (radius)
            if (parameters.size() >= 1)
            {
                double radius = parameters[0];

                // Create scaling transformation
                // Base sphere is radius 1
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = radius;
                scale_transform(1, 1) = radius;
                scale_transform(2, 2) = radius;
                Transform link_transform = p_world_transform * scale_transform;

                renderSphere(link_transform, geometry.color);
            }
            else
            {
                renderSphere(p_world_transform, geometry.color);
            }
            break;
        }
        case Geometry::Type::MESH:
        {
            // Render STL mesh
            renderMesh(p_world_transform, geometry.meshPath(), geometry.color);
            break;
        }
        default:
            break;
    }
}

// ----------------------------------------------------------------------------
void OpenGLViewer::renderRobot(Robot const& p_robot) const
{
    if (!p_robot.hasRoot())
        return;

    // Traverse the robot tree and render each node
    p_robot.root().traverse(
        [this](scene::Node const& node, size_t /*p_depth*/)
        {
            Transform world_transform = node.worldTransform();
            if (/*auto joint =*/dynamic_cast<Joint const*>(&node))
            {
                // renderJoint(*joint, world_transform);
            }
            else if (auto geometry = dynamic_cast<Geometry const*>(&node))
            {
                renderGeometry(*geometry, world_transform);
            }
        });

#if 0
    // Traverse the robot tree and render each axis
    p_robot.root().traverse([this](scene::Node const& node, size_t /*p_depth*/)
                            { renderAxes(node.worldTransform(), 0.2); });

    // Render world axes at origin
    renderAxes(p_robot.root().worldTransform(), 0.5);
#endif
}

// ----------------------------------------------------------------------------
void OpenGLViewer::updateCamera()
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
void OpenGLViewer::generateBox(std::vector<float>& vertices,
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
void OpenGLViewer::generateCylinder(std::vector<float>& vertices,
                                    std::vector<unsigned int>& indices,
                                    float radius,
                                    float height,
                                    size_t segments) const
{
    vertices.clear();
    indices.clear();

    // Generate vertices
    // Bottom center (at z = -height/2)
    vertices.insert(vertices.end(),
                    { 0.0f, 0.0f, -height / 2, 0.0f, 0.0f, -1.0f });
    // Top center (at z = height/2)
    vertices.insert(vertices.end(),
                    { 0.0f, 0.0f, height / 2, 0.0f, 0.0f, 1.0f });

    // Bottom and top circles
    for (size_t i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(segments);
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        // Bottom circle vertex
        vertices.insert(vertices.end(),
                        { x, y, -height / 2, 0.0f, 0.0f, -1.0f });
        // Top circle vertex
        vertices.insert(vertices.end(), { x, y, height / 2, 0.0f, 0.0f, 1.0f });
    }

    // Side vertices with side normals
    for (size_t i = 0; i <= segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(segments);
        float x = radius * std::cos(angle);
        float y = radius * std::sin(angle);

        // Side vertices with side normals
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        vertices.insert(vertices.end(), { x, y, -height / 2, nx, ny, 0.0f });
        vertices.insert(vertices.end(), { x, y, height / 2, nx, ny, 0.0f });
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
void OpenGLViewer::generateSphere(std::vector<float>& vertices,
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
void OpenGLViewer::initializeBox()
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
void OpenGLViewer::initializeCylinder()
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
void OpenGLViewer::initializeSphere()
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
void OpenGLViewer::initializeGrid()
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
void OpenGLViewer::setupCameraView(CameraViewType p_view_type,
                                   const Eigen::Vector3f& p_camera_target)
{
    // Set the camera target
    m_camera_target = p_camera_target;
    m_camera_view = p_view_type;

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
void OpenGLViewer::renderAxes(Transform const& p_transform,
                              double p_scale) const
{
    const double radius = 0.02 * p_scale; // Thin cylinder radius
    const double height = p_scale;        // Axis length

    // X axis (red) - cylinder along X direction
    {
        // Create scaling transformation (cylinder default: radius in XY, height
        // in Z)
        Transform scale_transform = Transform::Identity();
        scale_transform(0, 0) = radius;
        scale_transform(1, 1) = radius;
        scale_transform(2, 2) = height / 2.0; // Base cylinder has height 2

        // Create rotation to align cylinder with X axis (rotate 90° around Y)
        Transform rotation_transform = Transform::Identity();
        rotation_transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY())
                .toRotationMatrix();

        // Create translation to position cylinder at half its length along X
        Transform translation_transform = Transform::Identity();
        translation_transform(0, 3) = height * 0.5;

        // Combine: base * translation * rotation * scale
        Transform x_transform = p_transform * translation_transform *
                                rotation_transform * scale_transform;

        renderCylinder(x_transform, red_color);
    }

    // Y axis (green) - cylinder along Y direction
    {
        // Create scaling transformation
        Transform scale_transform = Transform::Identity();
        scale_transform(0, 0) = radius;
        scale_transform(1, 1) = radius;
        scale_transform(2, 2) = height / 2.0; // Base cylinder has height 2

        // Create rotation to align cylinder with Y axis (rotate -90° around X)
        Transform rotation_transform = Transform::Identity();
        rotation_transform.block<3, 3>(0, 0) =
            Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX())
                .toRotationMatrix();

        // Create translation to position cylinder at half its length along Y
        Transform translation_transform = Transform::Identity();
        translation_transform(1, 3) = height * 0.5;

        // Combine: base * translation * rotation * scale
        Transform y_transform = p_transform * translation_transform *
                                rotation_transform * scale_transform;

        renderCylinder(y_transform, green_color);
    }

    // Z axis (blue) - cylinder along Z direction
    {
        // Create scaling transformation
        Transform scale_transform = Transform::Identity();
        scale_transform(0, 0) = radius;
        scale_transform(1, 1) = radius;
        scale_transform(2, 2) = height / 2.0; // Base cylinder has height 2

        // No rotation needed, cylinder is already aligned with Z axis

        // Create translation to position cylinder at half its length along Z
        Transform translation_transform = Transform::Identity();
        translation_transform(2, 3) = height * 0.5;

        // Combine: base * translation * scale
        Transform z_transform =
            p_transform * translation_transform * scale_transform;

        Eigen::Vector3f blue_color(0.0f, 0.0f, 1.0f);
        renderCylinder(z_transform, blue_color);
    }
}

// ----------------------------------------------------------------------------
void OpenGLViewer::processInput(KeyCallback const& p_key_callback)
{
    // Track key states (pressed or released)
    static std::array<bool, 256> keys_old = { false };

    glfwPollEvents();

    for (size_t i = 0; i < m_keys.size(); ++i)
    {
        keys_old[i] = m_keys[i];
        m_keys[i] = (glfwGetKey(m_window, int(i)) == GLFW_PRESS);

        if (m_keys[i] && !keys_old[i])
        {
            p_key_callback(i, GLFW_PRESS);
        }
        else if (!m_keys[i] && keys_old[i])
        {
            p_key_callback(i, GLFW_RELEASE);
        }
    }
}

// ----------------------------------------------------------------------------
bool OpenGLViewer::isKeyPressed(int p_key) const
{
    if (p_key < 0 || p_key >= 256)
        return false;

    return m_keys[p_key];
}

// ----------------------------------------------------------------------------
void OpenGLViewer::renderGeometry(Geometry const& p_geometry,
                                  Transform const& p_world_transform) const
{
    std::vector<double> const& parameters = p_geometry.parameters();

    switch (p_geometry.type())
    {
        case Geometry::Type::BOX:
        {
            // Scale based on geometry parameters (width, height, depth)
            if (parameters.size() >= 3)
            {
                double width = parameters[0];
                double height = parameters[1];
                double depth = parameters[2];

                // Create scaling transformation
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = width;
                scale_transform(1, 1) = height;
                scale_transform(2, 2) = depth;
                Transform final_transform = p_world_transform * scale_transform;

                renderBox(final_transform, p_geometry.color);
            }
            else
            {
                renderBox(p_world_transform, p_geometry.color);
            }
            break;
        }
        case Geometry::Type::CYLINDER:
        {
            // Scale based on geometry parameters (radius, height)
            if (parameters.size() >= 2)
            {
                double radius = parameters[0];
                double height = parameters[1];

                // Create scaling transformation
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = radius;
                scale_transform(1, 1) = radius;
                scale_transform(2, 2) = height / 2.0;
                Transform final_transform = p_world_transform * scale_transform;

                renderCylinder(final_transform, p_geometry.color);
            }
            else
            {
                renderCylinder(p_world_transform, p_geometry.color);
            }
            break;
        }
        case Geometry::Type::SPHERE:
        {
            // Scale based on geometry parameters (radius)
            if (parameters.size() >= 1)
            {
                double radius = parameters[0];

                // Create scaling transformation
                Transform scale_transform = Transform::Identity();
                scale_transform(0, 0) = radius;
                scale_transform(1, 1) = radius;
                scale_transform(2, 2) = radius;
                Transform final_transform = p_world_transform * scale_transform;

                renderSphere(final_transform, p_geometry.color);
            }
            else
            {
                renderSphere(p_world_transform, p_geometry.color);
            }
            break;
        }
        case Geometry::Type::MESH:
        {
            // Render STL mesh
            renderMesh(
                p_world_transform, p_geometry.meshPath(), p_geometry.color);
            break;
        }
        default:
            break;
    }
}

// ----------------------------------------------------------------------------
} // namespace robotik