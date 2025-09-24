#pragma once

#include <cstring>
#include <string>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief DAE (COLLADA) file loader for 3D mesh data.
//!
//! Supports COLLADA DAE format for loading 3D models.
//! Loads vertex positions, normals, and texture coordinates for rendering.
// ****************************************************************************
class DAELoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief Structure to hold mesh data.
    // ------------------------------------------------------------------------
    struct MeshData
    {
        std::vector<float>
            vertices; //!< Vertex positions (x,y,z,nx,ny,nz,u,v per vertex)
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
    //! \brief Load DAE file.
    //! \param p_filename Path to DAE file.
    //! \param p_mesh_data Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool loadDAE(const std::string& p_filename, MeshData& p_mesh_data);

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
    //! \brief Parse geometry data from COLLADA XML.
    //! \param p_xml_content XML content as string.
    //! \param p_mesh_data Output mesh data.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool parseGeometry(const std::string& p_xml_content, MeshData& p_mesh_data);

    // ------------------------------------------------------------------------
    //! \brief Extract float array from XML source element.
    //! \param p_xml_content XML content.
    //! \param p_source_id Source ID to find.
    //! \param p_values Output float values.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool extractFloatArray(const std::string& p_xml_content, 
                                  const std::string& p_source_id,
                                  std::vector<float>& p_values);

    // ------------------------------------------------------------------------
    //! \brief Extract indices from triangles element.
    //! \param p_xml_content XML content.
    //! \param p_indices Output indices.
    //! \return true if successful.
    // ------------------------------------------------------------------------
    static bool extractTriangles(const std::string& p_xml_content,
                                 std::vector<unsigned int>& p_indices);

    // ------------------------------------------------------------------------
    //! \brief Find XML element content by tag name.
    //! \param p_xml_content XML content.
    //! \param p_tag_name Tag name to find.
    //! \param p_start_pos Start position for search.
    //! \return Content between opening and closing tags.
    // ------------------------------------------------------------------------
    static std::string findElementContent(const std::string& p_xml_content,
                                          const std::string& p_tag_name,
                                          size_t p_start_pos = 0);

    // ------------------------------------------------------------------------
    //! \brief Find XML element attribute value.
    //! \param p_element Element string.
    //! \param p_attribute_name Attribute name.
    //! \return Attribute value.
    // ------------------------------------------------------------------------
    static std::string findAttributeValue(const std::string& p_element,
                                          const std::string& p_attribute_name);

    // ------------------------------------------------------------------------
    //! \brief Trim whitespace from string.
    //! \param p_str String to trim.
    //! \return Trimmed string.
    // ------------------------------------------------------------------------
    static std::string trim(const std::string& p_str);

private:

    static std::string s_last_error;
};

} // namespace robotik

