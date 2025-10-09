/**
 * @file MeshManager.hpp
 * @brief OpenGL mesh management class.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Viewer/Mesh.hpp"
#include "Robotik/Viewer/Path.hpp"
#include "Robotik/Viewer/STLLoader.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>

namespace robotik::viewer
{

// ****************************************************************************
//! \brief OpenGL mesh management class.
//!
//! This class handles loading, caching and rendering of 3D meshes.
//! Supports STL files and can be extended for DAE and other formats.
// ****************************************************************************
class MeshManager
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    MeshManager() = default;

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    // ------------------------------------------------------------------------
    ~MeshManager();

    // ------------------------------------------------------------------------
    //! \brief Load a mesh from file.
    //! \param p_mesh_path Path to the mesh file.
    //! \param p_force_reload Force reload even if already cached.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadMesh(const std::string& p_mesh_path, bool p_force_reload = false);

    // ------------------------------------------------------------------------
    //! \brief Load a mesh from STL data.
    //! \param p_mesh_path Unique identifier for the mesh.
    //! \param p_mesh_data STL mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool loadMeshFromData(const std::string& p_mesh_path,
                          const STLLoader::MeshData& p_mesh_data);

    // ------------------------------------------------------------------------
    //! \brief Check if a mesh is loaded.
    //! \param p_mesh_path Path to the mesh file.
    //! \return true if loaded.
    // ------------------------------------------------------------------------
    bool isMeshLoaded(const std::string& p_mesh_path) const;

    // ------------------------------------------------------------------------
    //! \brief Get mesh data.
    //! \param p_mesh_path Path to the mesh file.
    //! \return Pointer to mesh data, nullptr if not found.
    // ------------------------------------------------------------------------
    const Mesh* getMesh(const std::string& p_mesh_path) const;

    // ------------------------------------------------------------------------
    //! \brief Render a mesh.
    //! \param p_mesh_path Path to the mesh file.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool renderMesh(const std::string& p_mesh_path) const;

    // ------------------------------------------------------------------------
    //! \brief Unload a specific mesh.
    //! \param p_mesh_path Path to the mesh file.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool unloadMesh(const std::string& p_mesh_path);

    // ------------------------------------------------------------------------
    //! \brief Clear all meshes and free OpenGL resources.
    // ------------------------------------------------------------------------
    void clear();

    // ------------------------------------------------------------------------
    //! \brief Get the number of loaded meshes.
    //! \return Number of loaded meshes.
    // ------------------------------------------------------------------------
    size_t getMeshCount() const
    {
        return m_meshes.size();
    }

    // ------------------------------------------------------------------------
    //! \brief Get the last error message.
    //! \return Error message.
    // ------------------------------------------------------------------------
    const std::string& error() const
    {
        return m_error;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the search paths for mesh files.
    //! \param p_base_path Base path for mesh files.
    // ------------------------------------------------------------------------
    void setSearchPaths(std::string const& p_base_path)
    {
        m_path.add(p_base_path);
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Create OpenGL buffers from mesh data.
    //! \param p_mesh_data Input mesh data.
    //! \param p_opengl_mesh Output OpenGL mesh.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    bool createOpenGLBuffers(const STLLoader::MeshData& p_mesh_data,
                             Mesh& p_opengl_mesh) const;

    // ------------------------------------------------------------------------
    //! \brief Free OpenGL resources for a mesh.
    //! \param p_opengl_mesh Mesh to free.
    // ------------------------------------------------------------------------
    void freeMeshResources(Mesh& p_opengl_mesh) const;

private:

    //! \brief Mesh storage
    std::unordered_map<std::string, Mesh> m_meshes;
    //! \brief Search paths
    Path m_path;
    //! \brief Last error
    std::string m_error;
};

} // namespace robotik::viewer
