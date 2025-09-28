/**
 * @file DAELoader.cpp
 * @brief DAE file loader for 3D models.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/loaders/DAELoader.hpp"
#include <fstream>
#include <sstream>

namespace robotik
{

std::string DAELoader::s_last_error;

// ----------------------------------------------------------------------------
bool DAELoader::loadDAE(const std::string& p_filename, MeshData& p_mesh_data)
{
    p_mesh_data.clear();
    s_last_error.clear();

    std::ifstream file(p_filename);
    if (!file.is_open())
    {
        s_last_error = "Cannot open file: " + p_filename;
        return false;
    }

    // Read entire file content
    std::string xml_content((std::istreambuf_iterator<char>(file)),
                            std::istreambuf_iterator<char>());
    file.close();

    if (xml_content.empty())
    {
        s_last_error = "File is empty: " + p_filename;
        return false;
    }

    // Check if it's a valid COLLADA file
    if (xml_content.find("<COLLADA") == std::string::npos)
    {
        s_last_error = "Not a valid COLLADA file: " + p_filename;
        return false;
    }

    return parseGeometry(xml_content, p_mesh_data);
}

// ----------------------------------------------------------------------------
bool DAELoader::parseGeometry(const std::string& p_xml_content,
                              MeshData& p_mesh_data)
{
    // Find geometry section
    std::string geometry_content =
        findElementContent(p_xml_content, "geometry");
    if (geometry_content.empty())
    {
        s_last_error = "No geometry found in COLLADA file";
        return false;
    }

    // Find mesh section within geometry
    std::string mesh_content = findElementContent(geometry_content, "mesh");
    if (mesh_content.empty())
    {
        s_last_error = "No mesh found in geometry";
        return false;
    }

    // Extract vertex positions
    std::vector<float> positions;
    if (!extractFloatArray(mesh_content, "positions", positions))
    {
        s_last_error = "Failed to extract vertex positions";
        return false;
    }

    // Extract normals (optional)
    std::vector<float> normals;
    extractFloatArray(mesh_content, "normals", normals);

    // Extract texture coordinates (optional)
    std::vector<float> texcoords;
    extractFloatArray(mesh_content, "map", texcoords);

    // Extract triangle indices
    std::vector<unsigned int> indices;
    if (!extractTriangles(mesh_content, indices))
    {
        s_last_error = "Failed to extract triangle indices";
        return false;
    }

    if (indices.empty() || positions.empty())
    {
        s_last_error = "No valid mesh data found";
        return false;
    }

    // Build vertex data with interleaved format: position(3) + normal(3) +
    // texcoord(2)
    size_t vertex_count = positions.size() / 3;
    bool has_normals = !normals.empty() && normals.size() >= vertex_count * 3;
    bool has_texcoords =
        !texcoords.empty() && texcoords.size() >= vertex_count * 2;

    p_mesh_data.vertices.reserve(vertex_count *
                                 8); // 3 pos + 3 normal + 2 texcoord

    for (size_t i = 0; i < vertex_count; ++i)
    {
        // Position
        p_mesh_data.vertices.push_back(positions[i * 3 + 0]);
        p_mesh_data.vertices.push_back(positions[i * 3 + 1]);
        p_mesh_data.vertices.push_back(positions[i * 3 + 2]);

        // Normal (default to up if not available)
        if (has_normals)
        {
            p_mesh_data.vertices.push_back(normals[i * 3 + 0]);
            p_mesh_data.vertices.push_back(normals[i * 3 + 1]);
            p_mesh_data.vertices.push_back(normals[i * 3 + 2]);
        }
        else
        {
            p_mesh_data.vertices.push_back(0.0f);
            p_mesh_data.vertices.push_back(0.0f);
            p_mesh_data.vertices.push_back(1.0f);
        }

        // Texture coordinates (default to 0,0 if not available)
        if (has_texcoords)
        {
            p_mesh_data.vertices.push_back(texcoords[i * 2 + 0]);
            p_mesh_data.vertices.push_back(texcoords[i * 2 + 1]);
        }
        else
        {
            p_mesh_data.vertices.push_back(0.0f);
            p_mesh_data.vertices.push_back(0.0f);
        }
    }

    p_mesh_data.indices = indices;
    p_mesh_data.triangle_count = indices.size() / 3;

    return true;
}

// ----------------------------------------------------------------------------
bool DAELoader::extractFloatArray(const std::string& p_xml_content,
                                  const std::string& p_source_id,
                                  std::vector<float>& p_values)
{
    p_values.clear();

    // Find source element with matching ID
    size_t pos = 0;
    while ((pos = p_xml_content.find("<source", pos)) != std::string::npos)
    {
        size_t end_pos = p_xml_content.find(">", pos);
        if (end_pos == std::string::npos)
            break;

        std::string source_tag = p_xml_content.substr(pos, end_pos - pos + 1);
        std::string id = findAttributeValue(source_tag, "id");

        if (id.find(p_source_id) != std::string::npos)
        {
            // Find the float_array within this source
            size_t source_end = p_xml_content.find("</source>", pos);
            if (source_end == std::string::npos)
                break;

            std::string source_content =
                p_xml_content.substr(pos, source_end - pos);
            std::string array_content =
                findElementContent(source_content, "float_array");

            if (!array_content.empty())
            {
                std::istringstream iss(array_content);
                float value;
                while (iss >> value)
                {
                    p_values.push_back(value);
                }
                return !p_values.empty();
            }
        }
        pos = end_pos + 1;
    }

    return false;
}

// ----------------------------------------------------------------------------
bool DAELoader::extractTriangles(const std::string& p_xml_content,
                                 std::vector<unsigned int>& p_indices)
{
    p_indices.clear();

    // Find triangles element
    std::string triangles_content =
        findElementContent(p_xml_content, "triangles");
    if (triangles_content.empty())
    {
        // Try polylist as fallback
        triangles_content = findElementContent(p_xml_content, "polylist");
        if (triangles_content.empty())
        {
            return false;
        }
    }

    // Find the 'p' element containing indices
    std::string p_content = findElementContent(triangles_content, "p");
    if (p_content.empty())
    {
        return false;
    }

    // Parse indices
    std::istringstream iss(p_content);
    unsigned int index;
    while (iss >> index)
    {
        p_indices.push_back(index);
    }

    return !p_indices.empty();
}

// ----------------------------------------------------------------------------
std::string DAELoader::findElementContent(const std::string& p_xml_content,
                                          const std::string& p_tag_name,
                                          size_t p_start_pos)
{
    std::string open_tag = "<" + p_tag_name;
    std::string close_tag = "</" + p_tag_name + ">";

    size_t start = p_xml_content.find(open_tag, p_start_pos);
    if (start == std::string::npos)
        return "";

    // Find the end of the opening tag
    size_t tag_end = p_xml_content.find(">", start);
    if (tag_end == std::string::npos)
        return "";

    // Check if it's a self-closing tag
    if (p_xml_content[tag_end - 1] == '/')
        return "";

    size_t content_start = tag_end + 1;
    size_t content_end = p_xml_content.find(close_tag, content_start);
    if (content_end == std::string::npos)
        return "";

    return trim(
        p_xml_content.substr(content_start, content_end - content_start));
}

// ----------------------------------------------------------------------------
std::string DAELoader::findAttributeValue(const std::string& p_element,
                                          const std::string& p_attribute_name)
{
    std::string attr_pattern = p_attribute_name + "=\"";
    size_t start = p_element.find(attr_pattern);
    if (start == std::string::npos)
        return "";

    start += attr_pattern.length();
    size_t end = p_element.find("\"", start);
    if (end == std::string::npos)
        return "";

    return p_element.substr(start, end - start);
}

// ----------------------------------------------------------------------------
std::string DAELoader::trim(const std::string& p_str)
{
    size_t start = p_str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos)
        return "";

    size_t end = p_str.find_last_not_of(" \t\n\r");
    return p_str.substr(start, end - start + 1);
}

} // namespace robotik
