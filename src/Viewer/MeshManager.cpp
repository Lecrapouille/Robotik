/**
 * @file MeshManager.cpp
 * @brief OpenGL mesh management class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/MeshManager.hpp"
#include <GL/glew.h>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
MeshManager::~MeshManager()
{
    clear();
}

// ----------------------------------------------------------------------------
bool MeshManager::loadMesh(const std::string& p_mesh_path, bool p_force_reload)
{
    // Check if already loaded
    if (!p_force_reload && isMeshLoaded(p_mesh_path))
    {
        return true;
    }

    // Load STL file
    STLLoader::MeshData mesh_data;
    const std::string full_path = m_base_path + p_mesh_path;

    if (!STLLoader::loadSTL(full_path, mesh_data))
    {
        m_last_error = "Failed to load STL file: " + p_mesh_path + " - " +
                       STLLoader::error();
        return false;
    }

    if (mesh_data.vertices.empty() || mesh_data.indices.empty())
    {
        m_last_error = "STL file contains no geometry: " + p_mesh_path;
        return false;
    }

    // Create OpenGL mesh
    OpenGLMesh opengl_mesh;
    if (!createOpenGLBuffers(mesh_data, opengl_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_mesh_path] = opengl_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::loadMeshFromData(const std::string& p_mesh_path,
                                   const STLLoader::MeshData& p_mesh_data)
{
    if (p_mesh_data.vertices.empty() || p_mesh_data.indices.empty())
    {
        m_last_error = "Mesh data is empty: " + p_mesh_path;
        return false;
    }

    // Create OpenGL mesh
    OpenGLMesh opengl_mesh;
    if (!createOpenGLBuffers(p_mesh_data, opengl_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_mesh_path] = opengl_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::isMeshLoaded(const std::string& p_mesh_path) const
{
    auto it = m_meshes.find(p_mesh_path);
    return (it != m_meshes.end()) && it->second.is_loaded;
}

// ----------------------------------------------------------------------------
const MeshManager::OpenGLMesh*
MeshManager::getMesh(const std::string& p_mesh_path) const
{
    auto it = m_meshes.find(p_mesh_path);
    return (it != m_meshes.end() && it->second.is_loaded) ? &it->second
                                                          : nullptr;
}

// ----------------------------------------------------------------------------
bool MeshManager::renderMesh(const std::string& p_mesh_path) const
{
    const OpenGLMesh* mesh = getMesh(p_mesh_path);
    if (!mesh)
    {
        // Cannot modify m_last_error in const method, so we'll just return
        // false
        return false;
    }

    glBindVertexArray(mesh->vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(mesh->index_count),
                   GL_UNSIGNED_INT,
                   nullptr);

    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::unloadMesh(const std::string& p_mesh_path)
{
    auto it = m_meshes.find(p_mesh_path);
    if (it == m_meshes.end())
    {
        return false;
    }

    freeMeshResources(it->second);
    m_meshes.erase(it);
    return true;
}

// ----------------------------------------------------------------------------
void MeshManager::clear()
{
    for (auto& [name, mesh] : m_meshes)
    {
        freeMeshResources(mesh);
    }
    m_meshes.clear();
}

// ----------------------------------------------------------------------------
bool MeshManager::createOpenGLBuffers(const STLLoader::MeshData& p_mesh_data,
                                      OpenGLMesh& p_opengl_mesh) const
{
    // Generate OpenGL buffers
    glGenVertexArrays(1, &p_opengl_mesh.vao);
    glGenBuffers(1, &p_opengl_mesh.vbo);
    glGenBuffers(1, &p_opengl_mesh.ebo);

    glBindVertexArray(p_opengl_mesh.vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, p_opengl_mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 p_mesh_data.vertices.size() * sizeof(float),
                 p_mesh_data.vertices.data(),
                 GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p_opengl_mesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 p_mesh_data.indices.size() * sizeof(unsigned int),
                 p_mesh_data.indices.data(),
                 GL_STATIC_DRAW);

    // Set vertex attributes (position + normal)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    p_opengl_mesh.index_count = p_mesh_data.indices.size();
    p_opengl_mesh.is_loaded = true;

    return true;
}

// ----------------------------------------------------------------------------
void MeshManager::freeMeshResources(OpenGLMesh& p_opengl_mesh) const
{
    if (p_opengl_mesh.vao != 0)
    {
        glDeleteVertexArrays(1, &p_opengl_mesh.vao);
    }
    if (p_opengl_mesh.vbo != 0)
    {
        glDeleteBuffers(1, &p_opengl_mesh.vbo);
    }
    if (p_opengl_mesh.ebo != 0)
    {
        glDeleteBuffers(1, &p_opengl_mesh.ebo);
    }
    p_opengl_mesh.clear();
}

} // namespace robotik::viewer
