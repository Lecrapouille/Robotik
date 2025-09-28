/**
 * @file ShaderManager.hpp
 * @brief OpenGL shader management class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <string>
#include <unordered_map>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief OpenGL shader management class.
//!
//! This class handles compilation, linking and management of OpenGL shaders.
//! Provides a simple interface for creating and using shader programs.
// ****************************************************************************
class ShaderManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    ShaderManager() = default;

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~ShaderManager();

    // ------------------------------------------------------------------------
    //! \brief Create a shader program from vertex and fragment shader sources.
    //! \param p_name Name identifier for the shader program.
    //! \param p_vertex_source Vertex shader source code.
    //! \param p_fragment_source Fragment shader source code.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createProgram(const std::string& p_name,
                       const std::string& p_vertex_source,
                       const std::string& p_fragment_source);

    // ------------------------------------------------------------------------
    //! \brief Use a shader program.
    //! \param p_name Name of the shader program to use.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool useProgram(const std::string& p_name);

    // ------------------------------------------------------------------------
    //! \brief Get the currently active shader program ID.
    //! \return OpenGL program ID, 0 if none active.
    // ------------------------------------------------------------------------
    unsigned int currentProgram() const
    {
        return m_current_program;
    }

    // ------------------------------------------------------------------------
    //! \brief Get a shader program ID by name.
    //! \param p_name Name of the shader program.
    //! \return OpenGL program ID, 0 if not found.
    // ------------------------------------------------------------------------
    unsigned int getProgram(const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get uniform location for the current program.
    //! \param p_name Name of the uniform.
    //! \return Uniform location, -1 if not found.
    // ------------------------------------------------------------------------
    int getUniformLocation(const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Set a 4x4 matrix uniform.
    //! \param p_location Uniform location.
    //! \param p_matrix Pointer to matrix data.
    // ------------------------------------------------------------------------
    void setMatrix4f(int p_location, const float* p_matrix) const;

    // ------------------------------------------------------------------------
    //! \brief Set a 3D vector uniform.
    //! \param p_location Uniform location.
    //! \param p_vector Pointer to vector data.
    // ------------------------------------------------------------------------
    void setVector3f(int p_location, const float* p_vector) const;

    // ------------------------------------------------------------------------
    //! \brief Check if a shader program exists.
    //! \param p_name Name of the shader program.
    //! \return true if exists.
    // ------------------------------------------------------------------------
    bool hasProgram(const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& getLastError() const
    {
        return m_last_error;
    }

    // ------------------------------------------------------------------------
    //! \brief Clear all shader programs and free OpenGL resources.
    // ------------------------------------------------------------------------
    void clear();

private:

    // ------------------------------------------------------------------------
    //! \brief Compile a shader from source.
    //! \param p_source Shader source code.
    //! \param p_type Shader type (GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, etc.).
    //! \return OpenGL shader ID, 0 if failed.
    // ------------------------------------------------------------------------
    unsigned int compileShader(const std::string& p_source,
                               unsigned int p_type);

    // ------------------------------------------------------------------------
    //! \brief Create a shader program from compiled shaders.
    //! \param p_vertex_shader Vertex shader ID.
    //! \param p_fragment_shader Fragment shader ID.
    //! \return OpenGL program ID, 0 if failed.
    // ------------------------------------------------------------------------
    unsigned int createProgram(unsigned int p_vertex_shader,
                               unsigned int p_fragment_shader);

private:

    // Shader programs storage
    std::unordered_map<std::string, unsigned int> m_programs;
    unsigned int m_current_program = 0;
    std::string m_last_error;
};

} // namespace robotik::viewer
