#pragma once

#include "Robotik/Robotik.hpp"

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

    URDFParser();
    std::unique_ptr<RobotArm> load(const std::string& p_filename);
    inline std::string getError() const
    {
        return m_error;
    }

private:

    void parseLinks(tinyxml2::XMLElement* p_robot_element);
    void parseJoints(tinyxml2::XMLElement* p_robot_element);

    // ------------------------------------------------------------------------
    //! \brief Find root and end effector links:
    //!  - The root link is the one that has no parent joint.
    //!  - The end effector link is the one that has no child joint.
    //! We assume that the robot has only one root link and one end effector
    //! link. If this is not the case, the robot will not be setup correctly.
    // ------------------------------------------------------------------------
    std::pair<Link*, Link const*> findRootAndEndEffector();
    void buildSceneGraph(Node* p_node, Link* p_link);
    bool setRootAndEndEffector();
    void setRobotLinks();

    Eigen::Vector3d parseVector3(const std::string& p_str) const;
    Eigen::Vector4d parseVector4(const std::string& p_str) const;
    Transform parseOrigin(const std::string& p_xyz,
                          const std::string& p_rpy) const;
    Geometry parseGeometry(const std::string& p_xml) const;
    Inertial parseInertial(const std::string& p_xml) const;
    Joint::Type parseJointType(const std::string_view& p_str_type) const;

private:

    std::unique_ptr<RobotArm> m_robot;
    std::unordered_map<std::string, std::unique_ptr<Link>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
