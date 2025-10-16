/**
 * @file Robot.hpp
 * @brief Robot class - Extends Hierarchy with kinematic computation methods.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Hierarchy.hpp"

namespace robotik
{

class Robot;

namespace debug
{

// ****************************************************************************
//! \brief Print the robot as hierarchy tree to a string.
//! \param p_robot The robot to print.
//! \param p_detailed If false, show only the tree structure (default).
//!  If true, show detailed information (local and world transforms, geometry).
//! \return A string containing the formatted robot representation.
// ****************************************************************************
std::string printRobot(Robot const& p_robot, bool p_detailed = false);

} // namespace debug

// *********************************************************************************
//! \brief Class representing a complete robotic arm.
//!
//! This class extends Hierarchy with methods for kinematic computations.
//! It maintains backward compatibility with existing code while using
//! the new Hierarchy-based architecture internally.
//!
//! Main capabilities:
//! - Forward kinematics and Jacobian computation
//! - Joint position management
//! - End-effector queries
//!
//! For advanced usage with explicit State management, use Hierarchy directly
//! and create separate State objects.
// *********************************************************************************
class Robot: public Hierarchy
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor with name only.
    //! \param p_name Name of the robot.
    // ------------------------------------------------------------------------
    explicit Robot(std::string_view const& p_name) : Hierarchy(p_name) {}

    // ------------------------------------------------------------------------
    //! \brief Constructor with name and root node.
    //! \param p_name Name of the robot.
    //! \param p_root Root node of the kinematic tree.
    // ------------------------------------------------------------------------
    Robot(std::string_view const& p_name, hierarchy::Node::Ptr p_root)
        : Hierarchy(p_name, std::move(p_root))
    {
    }

    // Inherit all methods from Hierarchy

    // Additional methods for backward compatibility

    // ------------------------------------------------------------------------
    //! \brief Get current joint positions.
    //! \return Vector of joint positions.
    // ------------------------------------------------------------------------
    std::vector<double> jointPositions() const;

    // ------------------------------------------------------------------------
    //! \brief Set joint values and update transforms.
    //! \param p_values Vector of joint values.
    //! \return True if successful, false if size mismatch.
    // ------------------------------------------------------------------------
    bool setJointValues(std::vector<double> const& p_values);

    // ------------------------------------------------------------------------
    //! \brief Set all joints to neutral position (0).
    // ------------------------------------------------------------------------
    void setNeutralPosition();

    // ------------------------------------------------------------------------
    //! \brief Compute Jacobian matrix for end effector.
    //! \param p_end_effector The end effector node.
    //! \return The Jacobian matrix.
    // ------------------------------------------------------------------------
    Jacobian jacobian(hierarchy::Node const& p_end_effector) const;

protected:

    // ------------------------------------------------------------------------
    //! \brief Calculate pose error between target and current pose.
    // ------------------------------------------------------------------------
    Pose calculatePoseError(Pose const& p_target_pose,
                            Pose const& p_current_pose) const;
};

} // namespace robotik
