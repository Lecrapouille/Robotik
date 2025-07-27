#pragma once

#include "Robotik/Robot.hpp"

// Forward declarations
namespace tinyxml2
{
class XMLElement;
class XMLDocument;
} // namespace tinyxml2

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
    //! \return A unique pointer to the robot if the file was loaded
    //! successfully, else nullptr.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> load(const std::string& p_filename);

    // ------------------------------------------------------------------------
    //! \brief Get the error message if the load() method failed (returns
    //! nullptr).
    //! \return The error message in case of failure, else an empty string.
    // ------------------------------------------------------------------------
    inline std::string getError() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Temporary data for storing parsed link data. Will be converted to
    //! a robotik::Link object in the buildSceneGraph() method.
    //! \note This is a private class used internally by the URDFParser class.
    //! Do not confuse with robotik::Link which is the definitive class used by
    //! the robot.
    // ------------------------------------------------------------------------
    struct Link
    {
        explicit Link(std::string_view const& p_name) : name(p_name) {}

        std::string name;
        Geometry::Ptr geometry;
        Geometry::Ptr collision;
        Eigen::Vector3f color = Eigen::Vector3f(0.5, 0.5, 0.5);
        Inertial inertial;
        Joint* parent_joint = nullptr;
        std::vector<Joint*> child_joints;
    };

    // ------------------------------------------------------------------------
    //! \brief Create the scene graph from the parsed URDF data.
    //! \param p_robot_name The name of the robot.
    //! \return A unique pointer to the robot if the scene graph was created
    //! successfully, else nullptr.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> buildSceneGraph(std::string_view p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Find the URDFParser::Link that has no parent joint to be used as
    //! the root of the scene graph.
    //! \return A pointer to the root link if found, else nullptr.
    // ------------------------------------------------------------------------
    URDFParser::Link* findRootLink() const;

    // ------------------------------------------------------------------------
    std::unique_ptr<Joint> buildJointTree(Joint const* p_current_joint);

    // ------------------------------------------------------------------------
    //! \brief Parse the robot from the URDF file.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> parseRobot(tinyxml2::XMLElement* p_robot_element);

    // ------------------------------------------------------------------------
    //! \brief Parse the parent and child links of a joint.
    // ------------------------------------------------------------------------
    void parseParentChildLinks(tinyxml2::XMLElement* p_joint_element,
                               Joint& p_joint) const;

    // ------------------------------------------------------------------------
    //! \brief Create a Link object from URDF Link data.
    //! \param p_urdf_link The URDF Link data containing geometry information.
    //! \note p_urdf_link has its collision and geometry moved.
    //! \return A unique pointer to the created Link object.
    // ------------------------------------------------------------------------
    robotik::Link::Ptr
    createLinkFromURDFData(URDFParser::Link& p_urdf_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse an optional geometry (visual or collision) of a link.
    // ------------------------------------------------------------------------
    std::unique_ptr<Geometry>
    parseGeometry(tinyxml2::XMLElement* p_geometry_element,
                  const std::string& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Parse visual geometry of a link (geometry, origin, material).
    // ------------------------------------------------------------------------
    void parseVisualProperties(tinyxml2::XMLElement* p_link_element,
                               URDFParser::Link& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse collision geometry of a link.
    // ------------------------------------------------------------------------
    void parseCollisionProperties(tinyxml2::XMLElement* p_link_element,
                                  URDFParser::Link& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the origin from an element.
    // ------------------------------------------------------------------------
    Transform parseOriginFromElement(tinyxml2::XMLElement* p_element) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the origin transform from the URDF file.
    // ------------------------------------------------------------------------
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    parseOriginTransform(tinyxml2::XMLElement* p_element) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the material of a link.
    // ------------------------------------------------------------------------
    void parseMaterial(tinyxml2::XMLElement* p_visual_element,
                       URDFParser::Link& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the inertial properties from the URDF file.
    // ------------------------------------------------------------------------
    void parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                 URDFParser::Link& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the inertial properties from the URDF file.
    // ------------------------------------------------------------------------
    Inertial parseInertial(tinyxml2::XMLElement* p_inertial_element) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the axis from the URDF file.
    // ------------------------------------------------------------------------
    Eigen::Vector3d parseAxis(tinyxml2::XMLElement* p_joint_element) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the limits from the URDF file.
    // ------------------------------------------------------------------------
    void parseLimits(tinyxml2::XMLElement* p_joint_element,
                     Joint& p_joint) const;

private:

    std::unique_ptr<Robot> m_robot;
    std::unordered_map<std::string, std::unique_ptr<URDFParser::Link>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
