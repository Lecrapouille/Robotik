/**
 * @file MeshLoader.hpp
 * @brief Base interface for mesh loaders.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <string>
#include <vector>

namespace robotik::renderer
{

// ****************************************************************************
//! \brief Base interface for all mesh loaders.
//!
//! Loaders are responsible for reading mesh data from various file formats
//! (STL, OBJ, etc.) and converting them into a common CPU-side mesh format.
//! This is designed to work with URDF robot geometries.
// ****************************************************************************
class MeshLoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief CPU-side mesh data structure.
    //!
    //! Contains raw mesh data loaded from files. This is the intermediate
    //! format before uploading to GPU. Vertices contain both position and
    //! normal data.
    // ------------------------------------------------------------------------
    struct CPUMesh
    {
        std::vector<float>
            vertices; //!< Vertex data (x,y,z,nx,ny,nz per vertex)
        std::vector<unsigned int> indices; //!< Triangle indices
        size_t triangle_count = 0;         //!< Number of triangles

        //! \brief Clear all mesh data
        void clear()
        {
            vertices.clear();
            indices.clear();
            triangle_count = 0;
        }

        //! \brief Check if mesh is empty
        bool empty() const
        {
            return vertices.empty() || indices.empty();
        }
    };

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~MeshLoader() = default;

    // ------------------------------------------------------------------------
    //! \brief Load mesh from file.
    //! \param p_filename Path to the mesh file.
    //! \param p_mesh Output CPU mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    virtual bool load(const std::string& p_filename, CPUMesh& p_mesh) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message string.
    // ------------------------------------------------------------------------
    virtual const std::string& error() const = 0;
};

} // namespace robotik::renderer
