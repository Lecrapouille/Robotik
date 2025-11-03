/**
 * @file RobotLoader.hpp
 * @brief Base interface for robot file loaders.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Blueprint.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

#include <memory>
#include <optional>
#include <string>

namespace robotik
{

// ****************************************************************************
//! \brief Base interface for all robot loaders.
//!
//! This interface allows polymorphic handling of different robot file formats
//! (URDF, SDF, MJCF, etc.). Loaders are responsible for parsing robot
//! description files and creating Robot instances.
//!
//! Example usage:
//! \code
//!   std::unique_ptr<RobotLoader> loader = std::make_unique<URDFLoader>();
//!   auto robot = loader->load("robot.urdf");
//!   if (!robot) {
//!       std::cerr << "Error: " << loader->error() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
class RobotLoader
{
public:

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor.
    // ------------------------------------------------------------------------
    virtual ~RobotLoader() = default;

    // ------------------------------------------------------------------------
    //! \brief Load the blueprint and robot name from a file without creating
    //! a Robot instance.
    //! \param p_filename Path to the robot description file.
    //! \param[out] p_robot_name The name of the robot extracted from the file.
    //! \return A Blueprint if successful, or an empty optional on failure.
    //!
    //! This method allows you to load the kinematic structure and then create
    //! any derived Robot type you want. This is more efficient than loadAs()
    //! because it doesn't create a temporary Robot instance.
    //!
    //! Example:
    //! \code
    //!   std::string robot_name;
    //!   auto blueprint = loader.loadBlueprint("robot.urdf", robot_name);
    //!   if (blueprint) {
    //!       auto robot = std::make_unique<MyCustomRobot>(robot_name,
    //!       std::move(*blueprint));
    //!   }
    //! \endcode
    // ------------------------------------------------------------------------
    virtual std::optional<Blueprint>
    loadBlueprint(const std::string& p_filename, std::string& p_robot_name) = 0;

    // ------------------------------------------------------------------------
    //! \brief Load a robot from a URDF file as a derived Robot type.
    //! \tparam RobotType The derived Robot class to instantiate.
    //! \param p_filename The path to the URDF file.
    //! \return A unique pointer to the derived robot if the file was loaded
    //! successfully, else nullptr.
    //!
    //! This template method allows creating instances of classes derived from
    //! Robot. The derived class must have the same constructor signature as
    //! Robot (name and blueprint).
    //!
    //! NOTE: This is a convenience wrapper around loadBlueprint().
    // ------------------------------------------------------------------------
    template <typename RobotType = Robot>
    std::unique_ptr<RobotType> loadAs(const std::string& p_filename);

    // ------------------------------------------------------------------------
    //! \brief Load a robot from a file.
    //! \param p_filename Path to the robot description file.
    //! \return A unique pointer to the loaded Robot, or nullptr on failure.
    // ------------------------------------------------------------------------
    virtual std::unique_ptr<robotik::Robot>
    load(const std::string& p_filename) = 0;

    // ------------------------------------------------------------------------
    //! \brief Get the error message if load() failed.
    //! \return Error message string, or empty string if no error occurred.
    // ------------------------------------------------------------------------
    virtual std::string error() const = 0;
};

} // namespace robotik

// Template implementation
namespace robotik
{

// ----------------------------------------------------------------------------
// Template implementation
// ----------------------------------------------------------------------------
template <typename RobotType>
std::unique_ptr<RobotType> RobotLoader::loadAs(const std::string& p_filename)
{
    static_assert(std::is_base_of<Robot, RobotType>::value,
                  "RobotType must derive from Robot");

    // Load the blueprint and robot name
    std::string robot_name;
    auto blueprint_opt = loadBlueprint(p_filename, robot_name);

    if (!blueprint_opt)
    {
        return nullptr;
    }

    // Create the robot of the requested type with the loaded blueprint
    return std::make_unique<RobotType>(robot_name, std::move(*blueprint_opt));
}

} // namespace robotik
