#pragma once

#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief STL file loader for 3D mesh data.
//!
//! Supports both ASCII and binary STL formats.
//! Loads vertex positions and normals for rendering.
// ****************************************************************************
class STLLoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief Structure to hold mesh data.
    // ------------------------------------------------------------------------
    struct MeshData
    {
        std::vector<float>
            vertices; //!< Vertex positions (x,y,z,nx,ny,nz per vertex)
        std::vector<unsigned int> indices; //!< Triangle indices
        size_t triangle_count = 0;         //!< Number of triangles

        void clear()
        {
            vertices.clear();
            indices.clear();
            triangle_count = 0;
        }
    };

    // ------------------------------------------------------------------------
    //! \brief Load STL file.
    //! \param p_filename Path to STL file.
    //! \param p_mesh_data Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool loadSTL(const std::string& p_filename, MeshData& p_mesh_data);

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    static const std::string& getLastError()
    {
        return s_last_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Load binary STL file.
    //! \param p_file Input file stream.
    //! \param p_mesh_data Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool loadBinarySTL(std::ifstream& p_file, MeshData& p_mesh_data);

    // ------------------------------------------------------------------------
    //! \brief Load ASCII STL file.
    //! \param p_file Input file stream.
    //! \param p_mesh_data Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool loadAsciiSTL(std::ifstream& p_file, MeshData& p_mesh_data);

    // ------------------------------------------------------------------------
    //! \brief Check if STL file is binary format.
    //! \param p_filename Path to STL file.
    //! \return true if binary format.
    // ------------------------------------------------------------------------
    static bool isBinarySTL(const std::string& p_filename);

private:

    static std::string s_last_error;
};

} // namespace robotik
