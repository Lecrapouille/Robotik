#pragma once

#include <functional>
#include <map>
#include <string>
#include <vector>

#include "Robotik/private/Path.hpp"

namespace robotik::viewer
{

// ****************************************************************************
//! \brief Mesh management for 3D models.
//!
//! Handles loading, caching, and rendering of 3D meshes from various formats.
// ****************************************************************************
class MeshManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Common mesh data structure.
    // ------------------------------------------------------------------------
    struct MeshData
    {
        std::vector<float> vertices; //!< Vertex data (format depends on loader)
        std::vector<unsigned int> indices; //!< Triangle indices
        size_t vertex_stride;              //!< Number of floats per vertex

        void clear()
        {
            vertices.clear();
            indices.clear();
            vertex_stride = 0;
        }
    };

    // ------------------------------------------------------------------------
    //! \brief Mesh loader interface.
    // ------------------------------------------------------------------------
    struct MeshLoader
    {
        std::function<bool(const std::string&, MeshData&)> load_function;
        std::function<const std::string&()> error_function;
        size_t vertex_stride; //!< Number of floats per vertex for this format
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    MeshManager();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~MeshManager();

    // ------------------------------------------------------------------------
    //! \brief Load and cache a mesh.
    //! \param p_mesh_path Path to mesh file.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadMesh(const std::string& p_mesh_path);

    // ------------------------------------------------------------------------
    //! \brief Render a cached mesh.
    //! \param p_mesh_path Path to mesh file.
    //! \return true if mesh exists and was rendered.
    // ------------------------------------------------------------------------
    bool renderMesh(const std::string& p_mesh_path) const;

    // ------------------------------------------------------------------------
    //! \brief Check if mesh is loaded.
    //! \param p_mesh_path Path to mesh file.
    //! \return true if mesh is cached.
    // ------------------------------------------------------------------------
    bool isMeshLoaded(const std::string& p_mesh_path) const;

    // ------------------------------------------------------------------------
    //! \brief Get last error message.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Initialize mesh loaders dictionary.
    // ------------------------------------------------------------------------
    void initializeMeshLoaders();

private:

    // ------------------------------------------------------------------------
    //! \brief OpenGL mesh data structure.
    // ------------------------------------------------------------------------
    struct Mesh
    {
        unsigned int vao;   //!< Vertex Array Object
        unsigned int vbo;   //!< Vertex Buffer Object
        unsigned int ebo;   //!< Element Buffer Object
        size_t index_count; //!< Number of indices
    };

private:

    //! \brief Mesh loaders dictionary (STL, DAE, OBJ, etc.).
    std::map<std::string, MeshLoader> m_mesh_loaders;

    //! \brief Mesh rendering cache.
    std::map<std::string, Mesh> m_meshes;

    std::string m_error;
};

} // namespace robotik::viewer