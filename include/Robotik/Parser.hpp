#pragma once

#include "Robotik/Robot.hpp"
#include <optional>

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

    // ------------------------------------------------------------------------
    //! \brief Parsed data for a link.*
    //! \note This is a private class used internally by the URDFParser class.
    //! Do not confuse with robotik::Link which is the definitive class used by
    //! the robot.
    // ------------------------------------------------------------------------
    struct Link
    {
        explicit Link(std::string_view const& p_name) : name(p_name) {}

        std::string name;
        std::optional<Geometry> geometry;
        std::optional<Geometry> collision;
        Transform visual_origin = Transform::Identity();
        Eigen::Vector3f color = Eigen::Vector3f(0.5, 0.5, 0.5);
        Inertial inertial;
        Joint* parent_joint = nullptr;
        std::vector<Joint*> child_joints;
    };

    // ------------------------------------------------------------------------
    //! \brief Find the link that has no parent joint.
    //! \return The name of the root link.
    // ------------------------------------------------------------------------
    std::string findRootLinkName() const;

    // ------------------------------------------------------------------------
    //! \brief Scene graph building methods.
    // ------------------------------------------------------------------------
    bool buildSceneGraph();

    // ------------------------------------------------------------------------
    std::unique_ptr<Joint> buildJointTree(Joint const* p_current_joint);

    // ------------------------------------------------------------------------
    //! \brief Create a Geometry object from URDF Link data.
    //! \param p_urdf_link The URDF Link data containing geometry information.
    //! \param p_name_suffix The suffix to add to the geometry name.
    //! \return A unique pointer to the created Geometry object.
    // ------------------------------------------------------------------------
    std::unique_ptr<Geometry>
    createGeometryFromURDFData(std::optional<Geometry> const& p_geom,
                               std::string_view p_name_suffix,
                               Eigen::Vector3f const& p_color,
                               Transform const& p_transform) const;

    // ------------------------------------------------------------------------
    //! \brief Create a Link object from URDF Link data.
    //! \param p_urdf_link The URDF Link data containing geometry information.
    //! \return A unique pointer to the created Link object.
    // ------------------------------------------------------------------------
    std::unique_ptr<robotik::Link>
    createLinkFromURDFData(const URDFParser::Link& p_urdf_link) const;

    // XML Parsing methods

    void parseLinks(tinyxml2::XMLElement* p_robot_element);
    void parseJoints(tinyxml2::XMLElement* p_robot_element);
    void parseVisualProperties(tinyxml2::XMLElement* p_link_element,
                               URDFParser::Link& p_link) const;
    void parseCollisionProperties(tinyxml2::XMLElement* p_link_element,
                                  URDFParser::Link& p_link) const;
    void parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                 URDFParser::Link& p_link) const;
    void parseMaterial(tinyxml2::XMLElement* p_visual_element,
                       URDFParser::Link& p_link) const;
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
    std::optional<Geometry> parseGeometry(const std::string& p_xml) const;
    Inertial parseInertial(const std::string& p_xml) const;
    Joint::Type parseJointType(std::string_view const& p_str_type) const;

private:

    std::unique_ptr<Robot> m_robot;
    std::unordered_map<std::string, std::unique_ptr<URDFParser::Link>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
