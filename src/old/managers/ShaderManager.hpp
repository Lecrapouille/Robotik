#pragma once

#include <string>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Shader management for OpenGL rendering.
//!
//! Handles shader compilation, linking, and program management.
// ****************************************************************************
class ShaderManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    ShaderManager();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~ShaderManager();

    // ------------------------------------------------------------------------
    //! \brief Initialize shaders.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool initialize();

    // ------------------------------------------------------------------------
    //! \brief Use the shader program.
    // ------------------------------------------------------------------------
    void use() const;

    // ------------------------------------------------------------------------
    //! \brief Get shader program ID.
    // ------------------------------------------------------------------------
    unsigned int getProgramId() const
    {
        return m_shader_program;
    }

    // ------------------------------------------------------------------------
    //! \brief Get last error message.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Compile shader.
    //! \param p_source Shader source code.
    //! \param p_type Shader type.
    //! \return Shader ID.
    // ------------------------------------------------------------------------
    unsigned int compileShader(const std::string& p_source,
                               unsigned int p_type);

    // ------------------------------------------------------------------------
    //! \brief Create shader program.
    //! \param p_vertex_source Vertex shader source.
    //! \param p_fragment_source Fragment shader source.
    //! \return Program ID.
    // ------------------------------------------------------------------------
    unsigned int createShaderProgram(const std::string& p_vertex_source,
                                     const std::string& p_fragment_source);

private:

    unsigned int m_shader_program = 0;
    std::string m_error;

    // Shader sources
    static const std::string s_vertex_shader_source;
    static const std::string s_fragment_shader_source;
};

} // namespace robotik::viewer