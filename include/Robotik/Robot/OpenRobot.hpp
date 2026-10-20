// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file OpenRobot.hpp
/// The shortest way to get a moving robot: one call, no world to create, no
/// physics engine to plug.
///
/// @code
/// auto arm = robotik::openRobot("robotik.robots.robot_6axis");
/// if (!arm)
///     return fail(arm.error());              // a message telling what to fix
/// robotik::JointSet& joints = arm.value()->joints();
/// joints.moveTo(joints.revolute("joint2"), robotik::Radians(0.3));
/// for (int i = 0; i < 200; ++i)
///     arm.value()->step(robotik::Seconds(0.01)); // MuJoCo moves the joint
/// @endcode
///
/// This is the first layer of RobotIK: one robot, its joints, sensors and
/// physics. The layers above add a job to do (Task) and files to describe it
/// (Package); nothing here needs them.
///
/// @par Contract
/// - Promises: @c openRobot returns a @c Result — a missing model or a wrong
///   URDF is a message, not an exception.
/// - Expects: a model id (@c pack.key) or a path to a URDF file.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Robot/Robot.hpp"

#include "Compages/World/World.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace robotik
{

// ****************************************************************************
/// How @ref openRobot builds the robot.
///
/// @code
/// robotik::RobotOptions options;
/// options.floating_base = true;  // a walking robot
/// options.floor = true;
/// auto legs = robotik::openRobot("legs.urdf", options);
/// @endcode
// ****************************************************************************
struct RobotOptions
{
    /// Simulate the robot with MuJoCo (gravity, contacts). False: no backend,
    /// the joints only move when written by hand.
    bool physics = true;
    /// The robot is free in space (a mobile or walking robot); false bolts its
    /// base to the world.
    bool floating_base = false;
    /// An infinite floor at z = 0.
    bool floor = false;
    /// Tool URDF mounted on the @c flange link, or empty.
    std::filesystem::path tool;
    /// Joint name to start position (rad or m); the others stay at zero.
    JointPosture home;
};

// ****************************************************************************
/// A robot with the world it lives in. Use it like a pointer to the robot.
// ****************************************************************************
struct OpenedRobot
{
    /// ECS world holding the robot entities.
    std::unique_ptr<compages::world::World> world;
    /// The robot.
    std::unique_ptr<RobotSession> robot;

    /// @return The robot.
    RobotSession* operator->() const
    {
        return robot.get();
    }

    /// @return The robot.
    RobotSession& operator*() const
    {
        return *robot;
    }
};

// ----------------------------------------------------------------------------
/// Loads a robot and plugs its physics.
/// @param p_model URDF file, or a model name of an asset pack
///   (@c robotik.robots.robot_6axis, see @ref AssetResolver).
/// @param p_options Physics, base, floor, tool and start posture.
/// @return The robot, or why it could not be loaded.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<OpenedRobot> openRobot(std::string const& p_model,
                                            RobotOptions const& p_options = {});

} // namespace robotik
