#pragma once

#include "Robotik/Robot.hpp"

// Forward declarations
namespace tinyxml2
{
class XMLElement;
}

namespace robotik
{

// ****************************************************************************
//! \brief Parser for URDF files to automatically build the robot.
// ****************************************************************************
class URDFParser
{
public:

    // ------------------------------------------------------------------------
    //! \brief Load a robot from a URDF file.
    //! \param p_filename The path to the URDF file.
    //! \return A unique pointer to the robot.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> load(const std::string& p_filename);

    // ------------------------------------------------------------------------
    //! \brief Get the error message if the load() method failed.
    //! \return The error message if the load() method failed, else an empty
    //! string.
    // ------------------------------------------------------------------------
    inline std::string getError() const
    {
        return m_error;
    }

private:

    struct Link
    {
        explicit Link(std::string_view const& p_name) : name(p_name) {}

        std::string name;
        Geometry geometry;
        Inertial inertial;
        Joint* parent_joint = nullptr;
        std::vector<Joint*> child_joints;
    };

    std::string findRootLinkName() const;

    void parseLinks(tinyxml2::XMLElement* p_robot_element);
    void parseJoints(tinyxml2::XMLElement* p_robot_element);
    void parseVisualProperties(tinyxml2::XMLElement* p_link_element,
                               Link& p_link) const;
    void parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                 Link& p_link) const;
    void parseMaterial(tinyxml2::XMLElement* p_visual_element,
                       Geometry& p_geometry) const;
    void parseLimits(tinyxml2::XMLElement* p_joint_element,
                     Joint& p_joint) const;
    void parseParentChildLinks(tinyxml2::XMLElement* p_joint_element,
                               Joint& p_joint) const;
    Eigen::Vector3d parseAxis(tinyxml2::XMLElement* p_joint_element) const;
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    parseOriginTransform(tinyxml2::XMLElement* p_element) const;
    Transform parseOriginFromElement(tinyxml2::XMLElement* p_element) const;
    Geometry parseGeometryFromElement(
        tinyxml2::XMLElement const* p_geometry_element) const;
    std::string getRequiredAttribute(tinyxml2::XMLElement const* p_element,
                                     const char* p_attr_name) const;
    std::string getAttributeOrDefault(tinyxml2::XMLElement const* p_element,
                                      const char* p_attr_name,
                                      const std::string& p_default) const;
    Eigen::Vector3d parseVector3(const std::string& p_str) const;
    Eigen::Vector4d parseVector4(const std::string& p_str) const;
    Transform parseOrigin(const std::string& p_xyz,
                          const std::string& p_rpy) const;
    Geometry parseGeometry(const std::string& p_xml) const;
    Inertial parseInertial(const std::string& p_xml) const;
    Joint::Type parseJointType(std::string_view const& p_str_type) const;

    // Scene graph building methods
    bool buildSceneGraph();
    std::unique_ptr<Joint> buildJointTree(Joint* current_joint);

private:

    std::unique_ptr<Robot> m_robot;
    std::unordered_map<std::string, std::unique_ptr<URDFParser::Link>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
