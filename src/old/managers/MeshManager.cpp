#include "MeshManager.hpp"
#include "../loaders/DAELoader.hpp"
#include "../loaders/STLLoader.hpp"
#include <GL/glew.h>
#include <algorithm>
#include <cctype>

namespace robotik::viewer
{

// ----------------------------------------------------------------------------
// Determine file extension
static std::string get_file_extension(const std::string& p_mesh_path)
{
    std::string file_extension;
    size_t dot_pos = p_mesh_path.find_last_of('.');
    if (dot_pos != std::string::npos)
    {
        file_extension = p_mesh_path.substr(dot_pos + 1);
        std::transform(file_extension.begin(),
                       file_extension.end(),
                       file_extension.begin(),
                       ::tolower);
    }
    return file_extension;
}

// ----------------------------------------------------------------------------
MeshManager::MeshManager()
{
    initializeMeshLoaders();
}

// ----------------------------------------------------------------------------
MeshManager::~MeshManager()
{
    // Clean up mesh resources
    for (auto& pair : m_meshes)
    {
        glDeleteVertexArrays(1, &pair.second.vao);
        glDeleteBuffers(1, &pair.second.vbo);
        glDeleteBuffers(1, &pair.second.ebo);
    }
}

// ----------------------------------------------------------------------------
void MeshManager::initializeMeshLoaders()
{
    // STL Loader
    m_mesh_loaders["stl"] = {
        [](const std::string& filename, MeshData& mesh_data) -> bool
        {
            STLLoader::MeshData stl_data;
            if (STLLoader::loadSTL(filename, stl_data))
            {
                mesh_data.vertices = std::move(stl_data.vertices);
                mesh_data.indices = std::move(stl_data.indices);
                mesh_data.vertex_stride = 6; // position(3) + normal(3)
                return true;
            }
            return false;
        },
        []() -> const std::string& { return STLLoader::error(); },
        6 // vertex stride for STL format
    };

    // DAE Loader
    m_mesh_loaders["dae"] = {
        [](const std::string& filename, MeshData& mesh_data) -> bool
        {
            DAELoader::MeshData dae_data;
            if (DAELoader::loadDAE(filename, dae_data))
            {
                mesh_data.vertices = std::move(dae_data.vertices);
                mesh_data.indices = std::move(dae_data.indices);
                mesh_data.vertex_stride =
                    8; // position(3) + normal(3) + texcoord(2)
                return true;
            }
            return false;
        },
        []() -> const std::string& { return DAELoader::error(); },
        8 // vertex stride for DAE format
    };
}

// ----------------------------------------------------------------------------
bool MeshManager::loadMesh(const std::string& p_mesh_path)
{
    if (isMeshLoaded(p_mesh_path))
        return true;

    // Find appropriate loader
    std::string file_extension = get_file_extension(p_mesh_path);
    auto loader_it = m_mesh_loaders.find(file_extension);
    if (loader_it == m_mesh_loaders.end())
    {
        m_error = "Unsupported mesh file format: " + file_extension;
        return false;
    }

    // Load mesh using the appropriate loader
    MeshData mesh_data;
    if (!loader_it->second.load_function(p_mesh_path, mesh_data))
    {
        m_error = "Failed to load mesh file: " + p_mesh_path + " - " +
                  loader_it->second.error_function();
        return false;
    }

    if (mesh_data.vertices.empty() || mesh_data.indices.empty())
    {
        m_error = "Mesh file contains no geometry: " + p_mesh_path;
        return false;
    }

    // Generate OpenGL buffers
    unsigned int vao, vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 mesh_data.vertices.size() * sizeof(float),
                 mesh_data.vertices.data(),
                 GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 mesh_data.indices.size() * sizeof(unsigned int),
                 mesh_data.indices.data(),
                 GL_STATIC_DRAW);

    // Set vertex attributes based on loader format
    size_t vertex_stride_bytes = mesh_data.vertex_stride * sizeof(float);

    // Position attribute (always present)
    glVertexAttribPointer(0,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          static_cast<GLsizei>(vertex_stride_bytes),
                          nullptr);
    glEnableVertexAttribArray(0);

    // Normal attribute (always present)
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          static_cast<GLsizei>(vertex_stride_bytes),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Texture coordinate attribute (only for formats that support it)
    if (mesh_data.vertex_stride >= 8)
    {
        glVertexAttribPointer(2,
                              2,
                              GL_FLOAT,
                              GL_FALSE,
                              static_cast<GLsizei>(vertex_stride_bytes),
                              (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
    }

    // Cache the mesh data
    m_meshes[p_mesh_path] = { vao, vbo, ebo, mesh_data.indices.size() };

    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::renderMesh(const std::string& p_mesh_path) const
{
    auto mesh_it = m_meshes.find(p_mesh_path);
    if (mesh_it == m_meshes.end())
        return false;

    const Mesh& mesh = mesh_it->second;
    glBindVertexArray(mesh.vao);
    glDrawElements(GL_TRIANGLES,
                   static_cast<GLsizei>(mesh.index_count),
                   GL_UNSIGNED_INT,
                   nullptr);

    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::isMeshLoaded(const std::string& p_mesh_path) const
{
    return m_meshes.find(p_mesh_path) != m_meshes.end();
}

} // namespace robotik::viewer