// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file StandaloneRobot.hpp
/// The shortest way to get a moving robot: one call, no world to create, no
/// physics engine to plug.
///
/// From tests/Robot/TestStandaloneRobot.cpp:
/// @code
/// auto arm = robotik::StandaloneRobot::open("robotik.robots.robot_6axis");
/// if (!arm)
///     return fail(arm.error());              // a message telling what to fix
/// robotik::JointSet& joints = arm.value().robot().getJoints();
/// joints.moveTo(joints.requireRevolute("joint2"), robotik::Radians(0.3));
/// for (int i = 0; i < 200; ++i)
///     arm.value().runtime().step(robotik::Seconds(0.01)); // MuJoCo moves it
/// @endcode
///
/// This is the first layer of RobotIK: one robot, its joints, sensors and
/// physics. The layers above add a job to do (Task) and files to describe it
/// (Package); nothing here needs them.
///
/// @par Contract
/// - Promises: @c StandaloneRobot::open returns a @c Result — a missing model or a wrong
///   URDF is a message, not an exception. The world lives longer than the
///   robot, whatever the caller does: both are private.
/// - Expects: a model id (@c pack.key) or a path to a URDF file.
#pragma once

#include "RobotIK/Common/Result.hpp"
#include "RobotIK/Robot/Backends/MujocoBackend.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"

#include "Compages/World/World.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace robotik
{

// ****************************************************************************
/// One robot alone, with the ECS world it lives in: no cell, no other
/// robot, no object. For a program that has its own control loop, a test,
/// a benchmark.
///
/// Why a class: the runtime keeps a reference to the world, so the world
/// must be destroyed after it. Both are private, the world is declared
/// first (so destroyed last), and the class moves but is not assigned (an
/// assignment would destroy the old world before the old robot).
///
/// @code
/// using namespace robotik::literals;
/// // A walking robot: a free base, a floor, MuJoCo with its defaults.
/// auto legs = robotik::StandaloneRobot::open(
///     "legs.urdf",
///     { .physics = robotik::MujocoOptions{ .floating_base = true,
///                                          .floor = true } });
/// // An arm without physics: the joints move when the code places them.
/// auto arm = robotik::StandaloneRobot::open("robotik.robots.robot_6axis",
///                               { .physics = std::nullopt });
/// @endcode
// ****************************************************************************
class StandaloneRobot final
{
public:

    // ************************************************************************
    /// How @ref open builds the robot. Every field has a default: an
    /// empty setup gives a bolted robot, without tool, simulated by MuJoCo.
    // ************************************************************************
    struct Setup
    {
        /// Tool mounted on the robot (URDF, flange and mount links); none
        /// by default (a hexapod, a mobile base).
        ToolMount tool;
        /// Position held at the start and at each reset; the joints not
        /// named stay at zero.
        JointPosture home;
        /// The MuJoCo physics (free base, floor, friction, time step...);
        /// @c std::nullopt for none: the joints then move only when the
        /// code places them (kinematics, tests of sensors).
        std::optional<MujocoOptions> physics = MujocoOptions{};
    };

    // -------------------------------------------------------------------------
    /// Loads a robot, mounts its tool, plugs its physics and holds its home.
    /// @param p_model URDF file, or a model name of an asset pack
    ///   (@c robotik.robots.robot_6axis, see @ref AssetResolver).
    /// @param p_setup Tool, home position and physics.
    /// @return The robot, or why it could not be loaded.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<StandaloneRobot>
    open(std::string const& p_model, Setup const& p_setup);

    /// @copydoc open(std::string const&, Setup const&)
    /// With the default @ref Setup: a bolted robot, no tool, MuJoCo.
    [[nodiscard]] static Result<StandaloneRobot>
    open(std::string const& p_model);

    // -------------------------------------------------------------------------
    /// One control period: the physics moves the joints, then the sensors
    /// read (see @ref RobotRuntime::step).
    // -------------------------------------------------------------------------
    void step(Seconds p_dt)
    {
        m_runtime->step(p_dt);
    }

    StandaloneRobot(StandaloneRobot&&) = default;
    StandaloneRobot& operator=(StandaloneRobot&&) = delete;
    StandaloneRobot(StandaloneRobot const&) = delete;
    StandaloneRobot& operator=(StandaloneRobot const&) = delete;
    ~StandaloneRobot();

    // -------------------------------------------------------------------------
    /// @return The robot as its skills see it: joints, read-only sensors,
    ///   actuators, kinematics.
    // -------------------------------------------------------------------------
    [[nodiscard]] Robot& robot()
    {
        return m_runtime->robot();
    }

    /// @copydoc robot()
    [[nodiscard]] Robot const& robot() const
    {
        return m_runtime->robot();
    }

    // -------------------------------------------------------------------------
    /// @return What runs the robot: @c step, @c reset, the devices to add,
    ///   the truth (see @ref RobotRuntime).
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotRuntime& runtime()
    {
        return *m_runtime;
    }

    /// @copydoc runtime()
    [[nodiscard]] RobotRuntime const& runtime() const
    {
        return *m_runtime;
    }

    // -------------------------------------------------------------------------
    /// @return The ECS world of the robot (its drawn links).
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::World& getWorld()
    {
        return *m_world;
    }

private:

    StandaloneRobot();

    /// ECS world holding the robot entities; declared first, destroyed last.
    std::unique_ptr<compages::world::World> m_world;
    /// The robot.
    std::unique_ptr<RobotRuntime> m_runtime;
};

} // namespace robotik
