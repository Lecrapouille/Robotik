/**
 * @file STLLoader.cpp
 * @brief STL file loader for 3D models.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Renderer/Loaders/StlLoader.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <sstream>

namespace robotik::renderer
{

// ----------------------------------------------------------------------------
bool STLLoader::load(const std::string& p_filename, CPUMesh& p_mesh)
{
    p_mesh.clear();
    m_last_error.clear();

    std::ifstream file(p_filename, std::ios::binary);
    if (!file.is_open())
    {
        m_last_error = "Cannot open file: " + p_filename;
        return false;
    }

    // Check if binary or ASCII format
    if (isBinarySTL(p_filename))
    {
        return loadBinarySTL(file, p_mesh);
    }
    else
    {
        // Reopen in text mode for ASCII
        file.close();
        file.open(p_filename, std::ios::in);
        if (!file.is_open())
        {
            m_last_error = "Cannot reopen file in text mode: " + p_filename;
            return false;
        }
        return loadAsciiSTL(file, p_mesh);
    }
}

// ----------------------------------------------------------------------------
bool STLLoader::isBinarySTL(const std::string& p_filename)
{
    std::ifstream file(p_filename, std::ios::binary);
    if (!file.is_open())
        return false;

    // Read first 80 bytes (header)
    char header[80];
    file.read(header, 80);

    // Read triangle count
    uint32_t triangle_count;
    file.read(reinterpret_cast<char*>(&triangle_count), sizeof(uint32_t));

    // Calculate expected file size for binary STL
    // Header (80) + triangle count (4) + triangles (50 bytes each)
    size_t expected_size = 80 + 4 + triangle_count * 50;

    // Get actual file size
    file.seekg(0, std::ios::end);
    size_t actual_size = static_cast<size_t>(file.tellg());

    // If sizes match, it's likely binary
    return (actual_size == expected_size);
}

// ----------------------------------------------------------------------------
bool STLLoader::loadBinarySTL(std::ifstream& p_file, CPUMesh& p_mesh)
{
    // Skip 80-byte header
    p_file.seekg(80);

    // Read triangle count
    uint32_t triangle_count;
    p_file.read(reinterpret_cast<char*>(&triangle_count), sizeof(uint32_t));

    if (triangle_count == 0)
    {
        m_last_error = "STL file contains no triangles";
        return false;
    }

    p_mesh.triangle_count = triangle_count;
    p_mesh.vertices.reserve(triangle_count *
                            18); // 3 vertices * 6 floats per vertex
    p_mesh.indices.reserve(triangle_count * 3);

    for (uint32_t i = 0; i < triangle_count; ++i)
    {
        // Read normal vector (3 floats)
        float normal[3];
        p_file.read(reinterpret_cast<char*>(normal), 3 * sizeof(float));

        // Read 3 vertices (9 floats)
        float vertices[9];
        p_file.read(reinterpret_cast<char*>(vertices), 9 * sizeof(float));

        // Skip attribute byte count (2 bytes)
        p_file.seekg(2, std::ios::cur);

        // Add vertices with normals to mesh data
        for (int j = 0; j < 3; ++j)
        {
            // Position
            p_mesh.vertices.push_back(vertices[j * 3 + 0]);
            p_mesh.vertices.push_back(vertices[j * 3 + 1]);
            p_mesh.vertices.push_back(vertices[j * 3 + 2]);

            // Normal
            p_mesh.vertices.push_back(normal[0]);
            p_mesh.vertices.push_back(normal[1]);
            p_mesh.vertices.push_back(normal[2]);

            // Add index
            p_mesh.indices.push_back(static_cast<unsigned int>(i * 3 + j));
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
bool STLLoader::loadAsciiSTL(std::ifstream& p_file, CPUMesh& p_mesh)
{
    std::string line;
    float normal[3];
    float vertex[3];
    size_t triangle_count = 0;

    while (std::getline(p_file, line))
    {
        // Convert to lowercase for easier parsing
        std::string line_lower = line;
        std::transform(line_lower.begin(),
                       line_lower.end(),
                       line_lower.begin(),
                       ::tolower);

        std::istringstream iss(line_lower);
        std::string token;
        iss >> token;

        if (token == "facet")
        {
            // Read normal vector
            std::string normal_token;
            iss >> normal_token; // should be "normal"
            if (normal_token == "normal")
            {
                iss >> normal[0] >> normal[1] >> normal[2];
            }
        }
        else if (token == "vertex")
        {
            // Read vertex coordinates
            iss >> vertex[0] >> vertex[1] >> vertex[2];

            // Add vertex with normal to mesh data
            p_mesh.vertices.push_back(vertex[0]);
            p_mesh.vertices.push_back(vertex[1]);
            p_mesh.vertices.push_back(vertex[2]);
            p_mesh.vertices.push_back(normal[0]);
            p_mesh.vertices.push_back(normal[1]);
            p_mesh.vertices.push_back(normal[2]);

            // Add index
            p_mesh.indices.push_back(
                static_cast<unsigned int>(p_mesh.indices.size()));
        }
        else if (token == "endfacet")
        {
            triangle_count++;
        }
    }

    p_mesh.triangle_count = triangle_count;

    if (triangle_count == 0)
    {
        m_last_error = "STL file contains no triangles";
        return false;
    }

    return true;
}

} // namespace robotik::renderer
