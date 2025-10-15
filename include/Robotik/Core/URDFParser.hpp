/**
 * @file URDFParser.hpp
 * @brief Parser for URDF files to create a robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/ClassesFwd.hpp"
#include "Robotik/Core/Types.hpp"

#include <memory>
#include <string>
#include <unordered_map>

// Forward declarations
namespace tinyxml2
{
class XMLElement;
class XMLDocument;
} // namespace tinyxml2

namespace robotik
{

struct URDFParserLink;

// ****************************************************************************
//! \brief Parser for URDF files to automatically build the robot.
// ****************************************************************************
class URDFParser
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    // ------------------------------------------------------------------------
    URDFParser();

    // ------------------------------------------------------------------------
    //! \brief Destructor.
    //! \note Explicitly declared to handle incomplete type URDFParserLink.
    // ------------------------------------------------------------------------
    ~URDFParser();

    // ------------------------------------------------------------------------
    //! \brief Copy constructor and assignment operator are deleted.
    //! \note This is necessary because of the incomplete type URDFParserLink.
    // ------------------------------------------------------------------------
    URDFParser(const URDFParser&) = delete;
    URDFParser& operator=(const URDFParser&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Move constructor and assignment operator.
    // ------------------------------------------------------------------------
    URDFParser(URDFParser&&) noexcept;
    URDFParser& operator=(URDFParser&&) noexcept;

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
    inline std::string error() const
    {
        return m_error;
    }

private:

    // ------------------------------------------------------------------------
    //! \brief Create the scene graph from the parsed URDF data.
    //! \param p_robot_name The name of the robot.
    //! \return A unique pointer to the robot if the scene graph was created
    //! successfully, else nullptr.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> buildSceneGraph(std::string const& p_robot_name);

    // ------------------------------------------------------------------------
    //! \brief Find the URDFParserLink that has no parent joint to be used as
    //! the root of the scene graph.
    //! \return A pointer to the root link if found, else nullptr.
    // ------------------------------------------------------------------------
    URDFParserLink* findRootLink() const;

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
    //! \param p_error The error message in case of failure.
    //! \note p_urdf_link has its collision and geometry moved.
    //! \return A unique pointer to the created Link object.
    // ------------------------------------------------------------------------
    std::unique_ptr<Link> createLinkFromURDFData(URDFParserLink& p_urdf_link,
                                                 std::string& p_error) const;

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
                               URDFParserLink& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse collision geometry of a link.
    // ------------------------------------------------------------------------
    void parseCollisionProperties(tinyxml2::XMLElement* p_link_element,
                                  URDFParserLink& p_link) const;

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
                       URDFParserLink& p_link) const;

    // ------------------------------------------------------------------------
    //! \brief Parse the inertial properties from the URDF file.
    // ------------------------------------------------------------------------
    void parseInertialProperties(tinyxml2::XMLElement* p_link_element,
                                 URDFParserLink& p_link) const;

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

    // ------------------------------------------------------------------------
    //! \brief Compute OBB from mesh file using PCA.
    //! \param p_mesh_path Path to the mesh file.
    //! \param p_center Output: center of the OBB.
    //! \param p_orientation Output: orientation matrix of the OBB.
    //! \param p_params Output: half-extents of the OBB.
    //! \return True if successful, false otherwise.
    // ------------------------------------------------------------------------
    bool computeOBBFromMesh(const std::string& p_mesh_path,
                            Eigen::Vector3d& p_center,
                            Eigen::Matrix3d& p_orientation,
                            std::vector<double>& p_params) const;

    // ------------------------------------------------------------------------
    //! \brief Parse STL file to extract vertices for OBB computation.
    //! \param p_mesh_path Path to the STL file.
    //! \param p_positions Output: vertex positions.
    //! \return True if successful, false otherwise.
    // ------------------------------------------------------------------------
    bool parseSTLVertices(const std::string& p_mesh_path,
                          std::vector<Eigen::Vector3d>& p_positions) const;

    // ------------------------------------------------------------------------
    //! \brief Compute OBB from vertices using PCA.
    //! \param p_positions Input: vertex positions.
    //! \param p_center Output: center of the OBB.
    //! \param p_orientation Output: orientation matrix of the OBB.
    //! \param p_params Output: half-extents of the OBB.
    // ------------------------------------------------------------------------
    void computeOBBFromVertices(const std::vector<Eigen::Vector3d>& p_positions,
                                Eigen::Vector3d& p_center,
                                Eigen::Matrix3d& p_orientation,
                                std::vector<double>& p_params) const;

private:

    std::unique_ptr<Robot> m_robot;
    std::unordered_map<std::string, std::unique_ptr<URDFParserLink>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
