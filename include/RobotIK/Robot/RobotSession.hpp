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

#include "RobotIK/Robot/Robot.hpp"

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
/// backend, the writable kinematics, and everything a skill must not write
/// because it is measured or decided outside the program: the joint and
/// vacuum measurements, the measured base, the resources (faults), the
/// devices (@c add) and the id. A skill gets the
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
    ///   RobotIK) to the Compages frame (Y-up) the entities are drawn in.
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

    /// Sets the id of the robot in its world (see @ref id).
    void identify(std::string p_id)
    {
        m_id = std::move(p_id);
    }

    /// @return The sensors, with @c add.
    [[nodiscard]] SensorSet& sensors()
    {
        return m_sensors;
    }

    /// @return The actuators, with @c add.
    [[nodiscard]] ActuatorSet& actuators()
    {
        return m_actuators;
    }

    using Robot::actuators; ///< The read-only ones stay visible.
    using Robot::sensors;   ///< The read-only ones stay visible.

    /// @return Resources, writable: faults fail and restore them, devices
    ///   add theirs.
    [[nodiscard]] ResourceManager& resources()
    {
        return *m_resources;
    }

    using Robot::resources; ///< The read-only one stays visible.

    // -------------------------------------------------------------------------
    /// Writes a joint measurement (the backend, after each step).
    /// @param p_id Joint id.
    /// @param p_position Measured position (rad or m).
    /// @param p_velocity Measured speed.
    // -------------------------------------------------------------------------
    void measureJoint(JointId p_id, double p_position, double p_velocity)
    {
        m_joints.measure(p_id, p_position, p_velocity);
    }

    // -------------------------------------------------------------------------
    /// Teleports a joint at rest: state, reference and command (a backend
    /// that draws a posture computed elsewhere, the reset).
    /// @param p_id Joint id.
    /// @param p_position New position (rad or m).
    // -------------------------------------------------------------------------
    void placeJoint(JointId p_id, double p_position)
    {
        m_joints.place(p_id, p_position);
    }

    // -------------------------------------------------------------------------
    /// Writes the measured vacuum of a cup of this robot (vacuum sensor, or
    /// the grasp model in simulation). Losing the vacuum clears its label.
    /// @param p_gripper Cup of this robot.
    /// @param p_holding True when an object is held.
    // -------------------------------------------------------------------------
    void measureHolding(VacuumGripper& p_gripper, bool p_holding)
    {
        p_gripper.measureHolding(p_holding);
    }

    // -------------------------------------------------------------------------
    /// Writes where the base really is (the backend, after its step: the
    /// truth in simulation). With @ref LocalizationFidelity::Truth, the estimate the
    /// skills read becomes the same.
    /// @param p_base Pose and velocity in the world.
    // -------------------------------------------------------------------------
    void measureBase(BaseState const& p_base)
    {
        m_true_base = p_base;
        if (m_localization == LocalizationFidelity::Truth)
        {
            m_base = p_base;
        }
    }

    /// @return Where the base really is (sensors, rendering, truth queries).
    [[nodiscard]] BaseState const& trueBase() const
    {
        return m_true_base;
    }

    // -------------------------------------------------------------------------
    /// Writes the estimated base, the one a skill reads with @ref base (the
    /// localization of a plugin). With @ref LocalizationFidelity::Truth, the next
    /// @ref measureBase overwrites it.
    /// @param p_base Estimated pose and velocity in the world.
    // -------------------------------------------------------------------------
    void estimateBase(BaseState const& p_base)
    {
        m_base = p_base;
    }

    /// Chooses where the estimated base comes from (see @ref LocalizationFidelity).
    void localization(LocalizationFidelity p_localization)
    {
        m_localization = p_localization;
    }

    /// @return Where the estimated base comes from.
    [[nodiscard]] LocalizationFidelity localization() const
    {
        return m_localization;
    }

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame, from the true base (sensors,
    /// truth queries). A skill reads @ref worldPose, from the estimate.
    /// @param p_link Link found by @ref requireLink on this robot.
    /// @throws std::logic_error if @p_link comes from another robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose trueWorldPose(LinkRef const& p_link) const;

    // -------------------------------------------------------------------------
    /// @ref trueWorldPose by name; empty for the base itself.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireTrueWorldPose(std::string const& p_frame) const;

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
    /// Where the estimated base comes from.
    LocalizationFidelity m_localization = LocalizationFidelity::Truth;
};

} // namespace robotik
