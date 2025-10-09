/**
 * @file ShaderManager.cpp
 * @brief OpenGL shader management class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Viewer/ShaderManager.hpp"
#include <GL/glew.h>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
ShaderManager::~ShaderManager()
{
    clear();
}

// ----------------------------------------------------------------------------
bool ShaderManager::createProgram(const std::string& p_name,
                                  const std::string& p_vertex_source,
                                  const std::string& p_fragment_source)
{
    // Compile vertex shader
    unsigned int vertex_shader =
        compileShader(p_vertex_source, GL_VERTEX_SHADER);
    if (vertex_shader == 0)
    {
        return false;
    }

    // Compile fragment shader
    unsigned int fragment_shader =
        compileShader(p_fragment_source, GL_FRAGMENT_SHADER);
    if (fragment_shader == 0)
    {
        glDeleteShader(vertex_shader);
        return false;
    }

    // Create program
    unsigned int program = createProgram(vertex_shader, fragment_shader);

    // Clean up shaders
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    if (program == 0)
    {
        return false;
    }

    // Store the program
    m_programs[p_name] = program;
    return true;
}

// ----------------------------------------------------------------------------
bool ShaderManager::useProgram(const std::string& p_name)
{
    auto it = m_programs.find(p_name);
    if (it == m_programs.end())
    {
        m_error = "Shader program '" + p_name + "' not found";
        return false;
    }

    glUseProgram(it->second);
    m_current_program = it->second;
    return true;
}

// ----------------------------------------------------------------------------
unsigned int ShaderManager::getProgram(const std::string& p_name) const
{
    auto it = m_programs.find(p_name);
    return (it != m_programs.end()) ? it->second : 0;
}

// ----------------------------------------------------------------------------
int ShaderManager::getUniformLocation(const std::string& p_name) const
{
    if (m_current_program == 0)
    {
        return -1;
    }

    return glGetUniformLocation(m_current_program, p_name.c_str());
}

// ----------------------------------------------------------------------------
void ShaderManager::setMatrix4f(int p_location, const float* p_matrix) const
{
    if (p_location >= 0)
    {
        glUniformMatrix4fv(p_location, 1, GL_FALSE, p_matrix);
    }
}

// ----------------------------------------------------------------------------
void ShaderManager::setVector3f(int p_location, const float* p_vector) const
{
    if (p_location >= 0)
    {
        glUniform3fv(p_location, 1, p_vector);
    }
}

// ----------------------------------------------------------------------------
bool ShaderManager::hasProgram(const std::string& p_name) const
{
    return m_programs.find(p_name) != m_programs.end();
}

// ----------------------------------------------------------------------------
void ShaderManager::clear()
{
    for (auto const& [name, program] : m_programs)
    {
        glDeleteProgram(program);
    }
    m_programs.clear();
    m_current_program = 0;
}

// ----------------------------------------------------------------------------
unsigned int ShaderManager::compileShader(const std::string& p_source,
                                          unsigned int p_type)
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

        m_error = std::string("Shader compilation error: ") + info_log;
        glDeleteShader(shader);
        return 0;
    }

    return shader;
}

// ----------------------------------------------------------------------------
unsigned int ShaderManager::createProgram(unsigned int p_vertex_shader,
                                          unsigned int p_fragment_shader)
{
    unsigned int program = glCreateProgram();
    glAttachShader(program, p_vertex_shader);
    glAttachShader(program, p_fragment_shader);
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

        m_error = "Program linking error: " + info_log;
        glDeleteProgram(program);
        return 0;
    }

    return program;
}

} // namespace robotik::viewer
