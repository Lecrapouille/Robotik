#include "Viewer/manager/ShaderManager.hpp"

namespace robotik::viewer
{

// Shader sources (moved from OpenGLWindow)
const std::string ShaderManager::s_vertex_shader_source = R"(
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
        FragPos = vec3(model * vec4(aPos, 1.0));
        Normal = mat3(transpose(inverse(model))) * aNormal;
        FragColor = color;
    }
    )";

const std::string ShaderManager::s_fragment_shader_source = R"(
    #version 330 core
    in vec3 FragColor;
    in vec3 Normal;
    in vec3 FragPos;

    out vec4 color;

    void main()
    {
        // Simple flat shading
        vec3 norm = normalize(Normal);
        vec3 lightDir = normalize(vec3(1.0, 1.0, 1.0));
        float diff = max(dot(norm, lightDir), 0.3);
        vec3 result = diff * FragColor;
        color = vec4(result, 1.0);
    }
    )";

// ----------------------------------------------------------------------------
ShaderManager::ShaderManager() = default;

// ----------------------------------------------------------------------------
ShaderManager::~ShaderManager()
{
    if (m_shader_program != 0)
    {
        glDeleteProgram(m_shader_program);
    }
}

// ----------------------------------------------------------------------------
bool ShaderManager::initialize()
{
    m_shader_program =
        createShaderProgram(s_vertex_shader_source, s_fragment_shader_source);
    return m_shader_program != 0;
}

// ----------------------------------------------------------------------------
void ShaderManager::use() const
{
    glUseProgram(m_shader_program);
}

// ----------------------------------------------------------------------------
unsigned int ShaderManager::compileShader(const std::string& p_source,
                                          unsigned int p_type)
{
    unsigned int shader = glCreateShader(p_type);
    const char* source_cstr = p_source.c_str();
    glShaderSource(shader, 1, &source_cstr, nullptr);
    glCompileShader(shader);

    int success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char info_log[512];
        glGetShaderInfoLog(shader, 512, nullptr, info_log);
        m_error = "Shader compilation failed: " + std::string(info_log);
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

// ----------------------------------------------------------------------------
unsigned int
ShaderManager::createShaderProgram(const std::string& p_vertex_source,
                                   const std::string& p_fragment_source)
{
    unsigned int vertex_shader =
        compileShader(p_vertex_source, GL_VERTEX_SHADER);
    if (vertex_shader == 0)
        return 0;

    unsigned int fragment_shader =
        compileShader(p_fragment_source, GL_FRAGMENT_SHADER);
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
        m_error = "Shader program linking failed: " + std::string(info_log);
        glDeleteProgram(program);
        program = 0;
    }

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    return program;
}

} // namespace robotik::viewer