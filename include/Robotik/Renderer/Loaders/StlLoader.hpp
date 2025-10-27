/**
 * @file STLLoader.hpp
 * @brief STL file loader for 3D models.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Renderer/Loaders/MeshLoader.hpp"

#include <cstring>
#include <fstream>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief STL file loader for 3D mesh data.
//!
//! Supports both ASCII and binary STL formats.
//! Loads vertex positions and normals for rendering.
//! Used for loading URDF mesh geometries from STL files.
// ****************************************************************************
class STLLoader: public MeshLoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief Load STL file.
    //! \param p_filename Path to STL file.
    //! \param p_mesh Output CPU mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool load(const std::string& p_filename, CPUMesh& p_mesh) override;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& error() const override
    {
        return m_last_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Load binary STL file.
    //! \param p_file Input file stream.
    //! \param p_mesh Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadBinarySTL(std::ifstream& p_file, CPUMesh& p_mesh);

    // ------------------------------------------------------------------------
    //! \brief Load ASCII STL file.
    //! \param p_file Input file stream.
    //! \param p_mesh Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadAsciiSTL(std::ifstream& p_file, CPUMesh& p_mesh);

    // ------------------------------------------------------------------------
    //! \brief Check if STL file is binary format.
    //! \param p_filename Path to STL file.
    //! \return true if binary format.
    // ------------------------------------------------------------------------
    bool isBinarySTL(const std::string& p_filename);

private:

    std::string m_last_error;
};

} // namespace robotik::renderer
