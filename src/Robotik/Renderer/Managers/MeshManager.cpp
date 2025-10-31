/**
 * @file MeshManager.cpp
 * @brief Unified mesh management class implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Managers/MeshManager.hpp"

#include <GL/glew.h>
#include <cmath>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
MeshManager::~MeshManager()
{
    clear();
}

// ----------------------------------------------------------------------------
bool MeshManager::loadFromFile(const std::string& p_name,
                               const std::string& p_filename,
                               MeshLoader& p_loader,
                               bool p_force_reload)
{
    // Check if already loaded
    if (!p_force_reload && hasMesh(p_name))
    {
        return true;
    }

    // Load CPU mesh data
    MeshLoader::CPUMesh cpu_mesh;
    std::string full_filename = m_path.expand(p_filename);
    if (!p_loader.load(full_filename, cpu_mesh))
    {
        m_error = "Failed to load mesh file: " + full_filename + " - " +
                  p_loader.error();
        return false;
    }

    if (cpu_mesh.empty())
    {
        m_error = "Mesh file contains no geometry: " + full_filename;
        return false;
    }

    // Create GPU mesh
    GPUMesh gpu_mesh;
    if (!createGPUMesh(cpu_mesh, gpu_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_name] = gpu_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::createBox(const std::string& p_name,
                            float p_width,
                            float p_height,
                            float p_depth)
{
    // Generate CPU mesh data
    MeshLoader::CPUMesh cpu_mesh;
    generateBox(cpu_mesh, p_width, p_height, p_depth);

    // Create GPU mesh
    GPUMesh gpu_mesh;
    if (!createGPUMesh(cpu_mesh, gpu_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_name] = gpu_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::createSphere(const std::string& p_name,
                               float p_radius,
                               size_t p_latitude_segments,
                               size_t p_longitude_segments)
{
    // Generate CPU mesh data
    MeshLoader::CPUMesh cpu_mesh;
    generateSphere(
        cpu_mesh, p_radius, p_latitude_segments, p_longitude_segments);

    // Create GPU mesh
    GPUMesh gpu_mesh;
    if (!createGPUMesh(cpu_mesh, gpu_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_name] = gpu_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::createCylinder(const std::string& p_name,
                                 float p_radius,
                                 float p_height,
                                 size_t p_segments)
{
    // Generate CPU mesh data
    MeshLoader::CPUMesh cpu_mesh;
    generateCylinder(cpu_mesh, p_radius, p_height, p_segments);

    // Create GPU mesh
    GPUMesh gpu_mesh;
    if (!createGPUMesh(cpu_mesh, gpu_mesh))
    {
        return false;
    }

    // Store the mesh
    m_meshes[p_name] = gpu_mesh;
    return true;
}

// ----------------------------------------------------------------------------
bool MeshManager::createGrid(const std::string& p_name,
                             int p_size,
                             float p_spacing)
{
    // Generate grid vertices
    std::vector<float> grid_vertices;
    generateGrid(grid_vertices, p_size, p_spacing);

    // Create GPU mesh manually (grid uses GL_LINES, no indices)
    GPUMesh gpu_mesh;

    glGenVertexArrays(1, &gpu_mesh.vao);
    glGenBuffers(1, &gpu_mesh.vbo);

    glBindVertexArray(gpu_mesh.vao);
    glBindBuffer(GL_ARRAY_BUFFER, gpu_mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 grid_vertices.size() * sizeof(float),
                 grid_vertices.data(),
                 GL_STATIC_DRAW);

    // Position attribute (location 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // Normal attribute (location 1) - for grid it's just Z-up
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    gpu_mesh.index_count =
        grid_vertices.size() / 6; // 6 floats per vertex (pos + normal)
    gpu_mesh.is_loaded = true;

    // Store the mesh
    m_meshes[p_name] = gpu_mesh;
    return true;
}

// ----------------------------------------------------------------------------
const MeshManager::GPUMesh*
MeshManager::getMesh(const std::string& p_name) const
{
    auto it = m_meshes.find(p_name);
    return (it != m_meshes.end() && it->second.is_loaded) ? &it->second
                                                          : nullptr;
}

// ----------------------------------------------------------------------------
bool MeshManager::hasMesh(const std::string& p_name) const
{
    auto it = m_meshes.find(p_name);
    return (it != m_meshes.end()) && it->second.is_loaded;
}

// ----------------------------------------------------------------------------
bool MeshManager::unloadMesh(const std::string& p_name)
{
    auto it = m_meshes.find(p_name);
    if (it == m_meshes.end())
    {
        return false;
    }

    freeGPUMesh(it->second);
    m_meshes.erase(it);
    return true;
}

// ----------------------------------------------------------------------------
void MeshManager::clear()
{
    for (auto& [name, mesh] : m_meshes)
    {
        freeGPUMesh(mesh);
    }
    m_meshes.clear();
}

// ----------------------------------------------------------------------------
bool MeshManager::createGPUMesh(const MeshLoader::CPUMesh& p_cpu_mesh,
                                GPUMesh& p_gpu_mesh)
{
    // Generate OpenGL buffers
    glGenVertexArrays(1, &p_gpu_mesh.vao);
    glGenBuffers(1, &p_gpu_mesh.vbo);
    glGenBuffers(1, &p_gpu_mesh.ebo);

    glBindVertexArray(p_gpu_mesh.vao);

    // Upload vertex data
    glBindBuffer(GL_ARRAY_BUFFER, p_gpu_mesh.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 p_cpu_mesh.vertices.size() * sizeof(float),
                 p_cpu_mesh.vertices.data(),
                 GL_STATIC_DRAW);

    // Upload index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, p_gpu_mesh.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 p_cpu_mesh.indices.size() * sizeof(unsigned int),
                 p_cpu_mesh.indices.data(),
                 GL_STATIC_DRAW);

    // Set vertex attributes (position + normal)
    // Position attribute (location = 0)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // Normal attribute (location = 1)
    glVertexAttribPointer(1,
                          3,
                          GL_FLOAT,
                          GL_FALSE,
                          6 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    p_gpu_mesh.index_count = p_cpu_mesh.indices.size();
    p_gpu_mesh.is_loaded = true;

    return true;
}

// ----------------------------------------------------------------------------
void MeshManager::freeGPUMesh(GPUMesh& p_gpu_mesh)
{
    if (p_gpu_mesh.vao != 0)
    {
        glDeleteVertexArrays(1, &p_gpu_mesh.vao);
    }
    if (p_gpu_mesh.vbo != 0)
    {
        glDeleteBuffers(1, &p_gpu_mesh.vbo);
    }
    if (p_gpu_mesh.ebo != 0)
    {
        glDeleteBuffers(1, &p_gpu_mesh.ebo);
    }
    p_gpu_mesh.clear();
}

// ----------------------------------------------------------------------------
// Primitive generation methods
// ----------------------------------------------------------------------------

void MeshManager::generateBox(MeshLoader::CPUMesh& p_mesh,
                              float p_width,
                              float p_height,
                              float p_depth) const
{
    p_mesh.clear();

    // Scale factors
    float sx = p_width * 0.5f;
    float sy = p_height * 0.5f;
    float sz = p_depth * 0.5f;

    // Box vertices with positions and normals
    const float box_vertices[] = {

        // Front face (normal pointing towards +Z)
        -sx,
        -sy,
        sz,
        0.0f,
        0.0f,
        1.0f,
        sx,
        -sy,
        sz,
        0.0f,
        0.0f,
        1.0f,
        sx,
        sy,
        sz,
        0.0f,
        0.0f,
        1.0f,
        -sx,
        sy,
        sz,
        0.0f,
        0.0f,
        1.0f,

        // Back face (normal pointing towards -Z)
        -sx,
        -sy,
        -sz,
        0.0f,
        0.0f,
        -1.0f,
        -sx,
        sy,
        -sz,
        0.0f,
        0.0f,
        -1.0f,
        sx,
        sy,
        -sz,
        0.0f,
        0.0f,
        -1.0f,
        sx,
        -sy,
        -sz,
        0.0f,
        0.0f,
        -1.0f,

        // Left face (normal pointing towards -X)
        -sx,
        -sy,
        -sz,
        -1.0f,
        0.0f,
        0.0f,
        -sx,
        -sy,
        sz,
        -1.0f,
        0.0f,
        0.0f,
        -sx,
        sy,
        sz,
        -1.0f,
        0.0f,
        0.0f,
        -sx,
        sy,
        -sz,
        -1.0f,
        0.0f,
        0.0f,

        // Right face (normal pointing towards +X)
        sx,
        -sy,
        -sz,
        1.0f,
        0.0f,
        0.0f,
        sx,
        sy,
        -sz,
        1.0f,
        0.0f,
        0.0f,
        sx,
        sy,
        sz,
        1.0f,
        0.0f,
        0.0f,
        sx,
        -sy,
        sz,
        1.0f,
        0.0f,
        0.0f,

        // Bottom face (normal pointing towards -Y)
        -sx,
        -sy,
        -sz,
        0.0f,
        -1.0f,
        0.0f,
        sx,
        -sy,
        -sz,
        0.0f,
        -1.0f,
        0.0f,
        sx,
        -sy,
        sz,
        0.0f,
        -1.0f,
        0.0f,
        -sx,
        -sy,
        sz,
        0.0f,
        -1.0f,
        0.0f,

        // Top face (normal pointing towards +Y)
        -sx,
        sy,
        -sz,
        0.0f,
        1.0f,
        0.0f,
        -sx,
        sy,
        sz,
        0.0f,
        1.0f,
        0.0f,
        sx,
        sy,
        sz,
        0.0f,
        1.0f,
        0.0f,
        sx,
        sy,
        -sz,
        0.0f,
        1.0f,
        0.0f
    };

    // Copy vertices
    p_mesh.vertices.assign(box_vertices,
                           box_vertices + 144); // 24 vertices * 6 floats

    // Box indices (2 triangles per face, 6 faces)
    const unsigned int box_indices[] = {
        0,  1,  2,  2,  3,  0,  // Front
        4,  5,  6,  6,  7,  4,  // Back
        8,  9,  10, 10, 11, 8,  // Left
        12, 13, 14, 14, 15, 12, // Right
        16, 17, 18, 18, 19, 16, // Bottom
        20, 21, 22, 22, 23, 20  // Top
    };

    // Copy indices
    p_mesh.indices.assign(box_indices, box_indices + 36);
    p_mesh.triangle_count = 12;
}

// ----------------------------------------------------------------------------
void MeshManager::generateSphere(MeshLoader::CPUMesh& p_mesh,
                                 float p_radius,
                                 size_t p_latitude_segments,
                                 size_t p_longitude_segments) const
{
    p_mesh.clear();

    // Generate vertices
    for (size_t lat = 0; lat <= p_latitude_segments; ++lat)
    {
        float theta = M_PIf * float(lat) / float(p_latitude_segments);
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);

        for (size_t lon = 0; lon <= p_longitude_segments; ++lon)
        {
            float phi = 2.0f * M_PIf * float(lon) / float(p_longitude_segments);
            float sin_phi = std::sin(phi);
            float cos_phi = std::cos(phi);

            float x = sin_theta * cos_phi;
            float y = cos_theta;
            float z = sin_theta * sin_phi;

            // Position
            p_mesh.vertices.push_back(p_radius * x);
            p_mesh.vertices.push_back(p_radius * y);
            p_mesh.vertices.push_back(p_radius * z);

            // Normal (same as normalized position for sphere)
            p_mesh.vertices.push_back(x);
            p_mesh.vertices.push_back(y);
            p_mesh.vertices.push_back(z);
        }
    }

    // Generate indices
    for (size_t lat = 0; lat < p_latitude_segments; ++lat)
    {
        for (size_t lon = 0; lon < p_longitude_segments; ++lon)
        {
            size_t first = lat * (p_longitude_segments + 1) + lon;
            size_t second = first + p_longitude_segments + 1;

            // Two triangles per quad
            p_mesh.indices.push_back(static_cast<unsigned int>(first));
            p_mesh.indices.push_back(static_cast<unsigned int>(second));
            p_mesh.indices.push_back(static_cast<unsigned int>(first + 1));

            p_mesh.indices.push_back(static_cast<unsigned int>(second));
            p_mesh.indices.push_back(static_cast<unsigned int>(second + 1));
            p_mesh.indices.push_back(static_cast<unsigned int>(first + 1));
        }
    }

    p_mesh.triangle_count = p_mesh.indices.size() / 3;
}

// ----------------------------------------------------------------------------
void MeshManager::generateCylinder(MeshLoader::CPUMesh& p_mesh,
                                   float p_radius,
                                   float p_height,
                                   size_t p_segments) const
{
    p_mesh.clear();

    // Generate vertices
    // Bottom center (at z = -height/2)
    p_mesh.vertices.insert(p_mesh.vertices.end(),
                           { 0.0f, 0.0f, -p_height / 2, 0.0f, 0.0f, -1.0f });
    // Top center (at z = height/2)
    p_mesh.vertices.insert(p_mesh.vertices.end(),
                           { 0.0f, 0.0f, p_height / 2, 0.0f, 0.0f, 1.0f });

    // Bottom and top circles
    for (size_t i = 0; i <= p_segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(p_segments);
        float x = p_radius * std::cos(angle);
        float y = p_radius * std::sin(angle);

        // Bottom circle vertex
        p_mesh.vertices.insert(p_mesh.vertices.end(),
                               { x, y, -p_height / 2, 0.0f, 0.0f, -1.0f });
        // Top circle vertex
        p_mesh.vertices.insert(p_mesh.vertices.end(),
                               { x, y, p_height / 2, 0.0f, 0.0f, 1.0f });
    }

    // Side vertices with side normals
    for (size_t i = 0; i <= p_segments; ++i)
    {
        float angle = 2.0f * M_PIf * float(i) / float(p_segments);
        float x = p_radius * std::cos(angle);
        float y = p_radius * std::sin(angle);

        // Side vertices with side normals
        float nx = std::cos(angle);
        float ny = std::sin(angle);
        p_mesh.vertices.insert(p_mesh.vertices.end(),
                               { x, y, -p_height / 2, nx, ny, 0.0f });
        p_mesh.vertices.insert(p_mesh.vertices.end(),
                               { x, y, p_height / 2, nx, ny, 0.0f });
    }

    // Generate indices
    // Bottom cap
    for (size_t i = 0; i < p_segments; ++i)
    {
        p_mesh.indices.push_back(0u);
        p_mesh.indices.push_back(static_cast<unsigned int>(2 + i * 2));
        p_mesh.indices.push_back(
            static_cast<unsigned int>(2 + ((i + 1) % (p_segments + 1)) * 2));
    }

    // Top cap
    for (size_t i = 0; i < p_segments; ++i)
    {
        p_mesh.indices.push_back(1u);
        p_mesh.indices.push_back(
            static_cast<unsigned int>(3 + ((i + 1) % (p_segments + 1)) * 2));
        p_mesh.indices.push_back(static_cast<unsigned int>(3 + i * 2));
    }

    // Side faces
    size_t side_offset = 2 + (p_segments + 1) * 2;
    for (size_t i = 0; i < p_segments; ++i)
    {
        size_t curr = side_offset + i * 2;
        size_t next = side_offset + ((i + 1) % (p_segments + 1)) * 2;

        // Two triangles per side face
        p_mesh.indices.push_back(static_cast<unsigned int>(curr));
        p_mesh.indices.push_back(static_cast<unsigned int>(next));
        p_mesh.indices.push_back(static_cast<unsigned int>(curr + 1));

        p_mesh.indices.push_back(static_cast<unsigned int>(next));
        p_mesh.indices.push_back(static_cast<unsigned int>(next + 1));
        p_mesh.indices.push_back(static_cast<unsigned int>(curr + 1));
    }

    p_mesh.triangle_count = p_mesh.indices.size() / 3;
}

// ----------------------------------------------------------------------------
void MeshManager::generateGrid(std::vector<float>& p_vertices,
                               int p_size,
                               float p_spacing) const
{
    p_vertices.clear();

    // Grid in XY plane (Z=0) for Z-up coordinate system to match URDF standard
    for (int i = -p_size; i <= p_size; ++i)
    {
        // Lines parallel to X axis (varying Y)
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        // Lines parallel to Y axis (varying X)
        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(-float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);

        p_vertices.push_back(float(i) * p_spacing);
        p_vertices.push_back(float(p_size) * p_spacing);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(0.0f);
        p_vertices.push_back(1.0f);
    }
}

} // namespace robotik::renderer
