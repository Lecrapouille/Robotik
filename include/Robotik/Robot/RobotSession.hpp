// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotSession.hpp
/// The robot as the program that runs it sees it: it steps the physics and
/// the sensors, and it reaches the simulator (ECS world, backend).
///
/// Skills include @c Robot.hpp and receive a @ref robotik::Robot; this header
/// is for the cell, the backends, the sensors and the tests.
///
/// @par Contract
/// - Promises: the skills cannot reach what this class adds (checked by
///   @c make @c check-layers for @c Task/Skills and @c Task/Planning).
/// - Expects: one @ref robotik::RobotSession::step per control period.
/// - Errors: an unknown joint name throws @c std::invalid_argument.
#pragma once

#include "Robotik/Robot/Robot.hpp"

#include "Compages/World/Entity.hpp"

#include <filesystem>
#include <memory>
#include <string_view>

namespace robotik
{

// ****************************************************************************
/// A robot being run: owns the backend and advances time.
///
/// The program that runs the robot (a @ref RoboticCell, a test, @ref openRobot)
/// holds this class; the skills only see its @ref Robot base. A session
/// without a backend still works: the joints then move only when the code
/// writes them, which is enough for kinematics tests.
///
/// @ref step puts the failed actuators in their safe state, advances the
/// backend by one control period, then updates the kinematics and reads the
/// sensors; call it once per period.
/// The backend is the only part that changes between MuJoCo and a real
/// robot (see @ref RobotBackend).
///
/// This class also gives what only the simulator and the program that runs
/// the robot may touch: the ECS world and the entities of the links, the
/// backend, the writable kinematics and the measured base. A skill gets the
/// @ref Robot base, which has none of them, so it cannot read the truth of
/// the simulator nor step it. Why here and not in an adapter: the session
/// already owns the backend, and one type less is one name less to learn.
/// @c make @c check-layers compiles @c Task/Skills and @c Task/Planning
/// without this header, so a cast from @ref Robot is refused there too.
///
/// @code
/// compages::world::World world;
/// robotik::RobotSession robot(world, "arm.urdf");
/// robot.actuators().add<robotik::JointGroup>("arm");
/// auto physics = std::make_unique<robotik::MujocoBackend>();
/// physics->load("arm.urdf");
/// robot.connect(std::move(physics));
/// robot.hold({ { "joint2", 0.3 } });
/// while (running)
///     robot.step(Seconds(0.01));
/// @endcode
// ****************************************************************************
class RobotSession final: public Robot
{
public:

    /// @param p_world ECS world receiving the entities.
    /// @param p_urdf URDF of the robot.
    /// @param p_view Rendering hooks, or null for a headless robot.
    /// @param p_tool URDF of a tool on the flange, or empty.
    /// @param p_resources Shared with the other robots of the world, or null.
    RobotSession(compages::world::World& p_world,
                 std::filesystem::path const& p_urdf,
                 SceneView* p_view = nullptr,
                 std::filesystem::path const& p_tool = {},
                 std::shared_ptr<ResourceManager> p_resources = {});
    ~RobotSession() override;

    // -------------------------------------------------------------------------
    /// Plugs the backend and calls its @c attach. Connect after adding the
    /// sensors a backend may need to know (e.g. force/torque sensors for
    /// MuJoCo).
    /// @param p_backend Backend, now owned by the session.
    // -------------------------------------------------------------------------
    void connect(std::unique_ptr<RobotBackend> p_backend);

    /// @return Backend moving the robot, or null.
    [[nodiscard]] RobotBackend* backend()
    {
        return m_backend.get();
    }

    /// @return Backend moving the robot, read only, or null.
    [[nodiscard]] RobotBackend const* backend() const
    {
        return m_backend.get();
    }

    /// @return The ECS world holding the robot entities.
    [[nodiscard]] compages::world::World& world() const
    {
        return m_world;
    }

    /// @return Root entity, placed at the base pose in the world.
    [[nodiscard]] compages::world::Entity root() const
    {
        return m_root;
    }

    /// @return Link entity named @p_name, or an empty handle.
    [[nodiscard]] compages::world::Entity link(std::string_view p_name) const;

    /// @return Rotation from the URDF frame (Z-up, the world frame of
    ///   Robotik) to the Compages frame (Y-up) the entities are drawn in.
    [[nodiscard]] compages::Quatf const& urdfFrame() const
    {
        return m_world_from_urdf;
    }

    /// @return Analytical model (Pinocchio), writable (a backend or a tool
    ///   change updates it; a skill reads it from @ref Robot).
    [[nodiscard]] PinocchioBackend& kinematics()
    {
        return *m_kinematics;
    }

    using Robot::kinematics; ///< The read-only one stays visible.

    // -------------------------------------------------------------------------
    /// Writes the measured base (backends, or the application for a robot
    /// bolted somewhere else than the world origin).
    /// @param p_base Pose and velocity in the world.
    // -------------------------------------------------------------------------
    void measureBase(BaseState const& p_base)
    {
        m_base = p_base;
    }

    // -------------------------------------------------------------------------
    /// Places the joints of a posture, makes the posture the home and servos
    /// every joint where it is.
    /// @param p_posture Joint name to position; missing joints keep theirs.
    /// @throws std::invalid_argument for an unknown joint name.
    // -------------------------------------------------------------------------
    void hold(JointPosture const& p_posture = {});

    // -------------------------------------------------------------------------
    /// Sets the base pose restored by @ref reset (world origin by default).
    /// @param p_pose Base pose in the world.
    // -------------------------------------------------------------------------
    void startPose(Pose const& p_pose);

    // -------------------------------------------------------------------------
    /// Backend step, kinematics, rendered links, then sensors. Failed
    /// actuators are kept in their safe state. Same as @ref prepare,
    /// @ref advance, then @ref sense.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// The three parts of @ref step. This one puts the failed actuators in
    /// their safe state and advances the clock. Robots sharing a backend do
    /// each part in turn, so a failed actuator is safe before the physics
    /// step and every robot senses after it.
    ///
    /// @code
    /// for (auto* r : robots) r->prepare(dt);
    /// for (auto* r : robots) r->advance(dt); // the first steps the physics
    /// for (auto* r : robots) r->sense();
    /// @endcode
    ///
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void prepare(Seconds p_dt);
    /// Backend step only (see @ref prepare).
    void advance(Seconds p_dt);
    /// Kinematics and rendered links, then the sensors: those whose
    /// resource failed are invalidated (see @ref prepare).
    void sense();

    // -------------------------------------------------------------------------
    /// Clock to zero, joints at home, base at its start pose, sensors
    /// rewound.
    // -------------------------------------------------------------------------
    void reset();

private:

    /// Base pose restored by @ref reset.
    Pose m_start;
};

} // namespace robotik
