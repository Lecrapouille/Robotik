/**
 * @file UrdfLoader.hpp
 * @brief Parser for URDF files to create a robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Common/Types.hpp"
#include "Robotik/Core/Loaders/RobotLoader.hpp"

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>

// Forward declarations
namespace pugi
{
class xml_node;
} // namespace pugi

namespace robotik
{

struct URDFLoaderLink;
class Joint;
class Geometry;
class Robot;

// ****************************************************************************
//! \brief Parser for URDF files to automatically build the robot.
//!
//! This class implements the RobotLoader interface for URDF format.
//! It also provides a template method loadAs<T>() to create derived Robot
//! types.
//!
//! Example usage:
//! \code
//!   URDFLoader parser;
//!
//!   // Load a standard Robot
//!   auto robot = parser.load("robot.urdf");
//!
//!   // Load a derived Robot type
//!   auto custom_robot = parser.loadAs<MyCustomRobot>("robot.urdf");
//! \endcode
// ****************************************************************************
class URDFLoader: public RobotLoader
{
public:

    URDFLoader();
    ~URDFLoader() override;

    // Non-copyable, non-movable
    URDFLoader(const URDFLoader&) = delete;
    URDFLoader& operator=(const URDFLoader&) = delete;
    URDFLoader(URDFLoader&&) = delete;
    URDFLoader& operator=(URDFLoader&&) = delete;

    // ------------------------------------------------------------------------
    //! \brief Load the blueprint and robot name from a URDF file.
    //! \param p_filename Path to the URDF file.
    //! \param[out] p_robot_name The name of the robot extracted from the file.
    //! \return A Blueprint if successful, or an empty optional on failure.
    // ------------------------------------------------------------------------
    std::optional<Blueprint> loadBlueprint(const std::string& p_filename,
                                           std::string& p_robot_name) override;

    // ------------------------------------------------------------------------
    //! \brief Load a robot from a URDF file.
    //! \param p_filename The path to the URDF file.
    //! \return A unique pointer to the robot if the file was loaded
    //! successfully, else nullptr.
    // ------------------------------------------------------------------------
    std::unique_ptr<Robot> load(const std::string& p_filename) override;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if the load() method failed (returns
    //! nullptr).
    //! \return The error message in case of failure, else an empty string.
    // ------------------------------------------------------------------------
    inline std::string error() const override
    {
        return m_error;
    }

private:

    // Core parsing methods
    bool parseLinks(pugi::xml_node p_robot_element);
    bool parseJoints(pugi::xml_node p_robot_element);
    std::optional<Blueprint> buildBlueprint();
    std::unique_ptr<Robot> buildRobotModel(std::string const& p_robot_name);
    std::unique_ptr<Joint> buildJointTree(Joint const* p_current_joint);

    // Link and geometry parsing
    bool parseLinkGeometries(pugi::xml_node p_link_element,
                             URDFLoaderLink& p_link) const;
    std::unique_ptr<Geometry> parseGeometry(pugi::xml_node p_geometry_element,
                                            const std::string& p_name) const;

private:

    std::unique_ptr<Robot> m_robot;
    std::unordered_map<std::string, std::unique_ptr<URDFLoaderLink>> m_links;
    std::unordered_map<std::string, std::unique_ptr<Joint>> m_joints;
    std::string m_error;
};

} // namespace robotik
