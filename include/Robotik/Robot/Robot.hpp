// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Robot.hpp
/// The robot: its joints, sensors, actuators and kinematics, in one object.
///
/// There are two views of it. Skills see @ref robotik::Robot — they read the
/// joints and the sensors and send commands, nothing more. The program that
/// runs the robot owns a @ref robotik::RobotSession — it steps the physics and
/// the sensors. So a skill cannot step the world by mistake, and the same skill
/// drives a simulated arm or a real one (see doc/Glossary.md: "Backend").
///
/// @par Contract
/// - Promises: a skill sees the same @ref robotik::Robot API in simulation and
///   on a real robot; only the @ref robotik::RobotBackend of the session
///   changes.
/// - Expects: one @ref robotik::RobotSession::step per control period; names of
///   joints and frames that exist in the URDF.
/// - Errors: an unknown joint or frame name is a programming error and throws
///   @c std::invalid_argument. Opening a robot from a file is done by
///   @ref robotik::openRobot, which returns a @c Result.
#pragma once

#include "Robotik/Robot/Backends/RobotBackend.hpp"
#include "Robotik/Robot/Backends/SceneView.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Robot/Devices.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Robot/LinkRef.hpp"
#include "Robotik/Robot/Resources.hpp"
#include "Robotik/Robot/Sensors/Measurements.hpp"
#include "Robotik/Robot/Sensors/Sensor.hpp"

#include "Compages/World/Entity.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;

/// Sensors of a robot.
using SensorSet = DeviceSet<Sensor>;
/// Actuators of a robot.
using ActuatorSet = DeviceSet<Actuator>;

/// Joint name to position, in the SI base unit of the joint (rad or m).
using JointPosture = std::unordered_map<std::string, double>;

// ****************************************************************************
/// Robot API used by skills.
///
/// It cannot be built directly: an application builds a @ref RobotSession,
/// a skill receives it through @ref RobotContext.
///
/// @code
/// robotik::Robot& robot = p_context.robot;
/// robotik::Pose const tool = robot.framePose(robot.tool());     // base frame
/// robotik::Pose const in_world = robot.worldPose(robot.tool()); // world frame
/// if (auto const q = robot.inverseKinematics(robot.tool(), goal))
///     for (robotik::JointId id = 0; id < q->size(); ++id)
///         robot.joints().moveTo(id, (*q)[id]);
/// @endcode
// ****************************************************************************
class Robot
{
public:

    Robot(Robot const&) = delete;
    Robot& operator=(Robot const&) = delete;
    virtual ~Robot();

    /// @return Name of the URDF robot.
    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    // -------------------------------------------------------------------------
    /// @return Id of the robot in its world (the entity id of the world
    ///   file); the URDF name until @ref identify. Two copies of a model
    ///   share the name, not the id.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& id() const
    {
        return m_id;
    }

    /// Sets the id of the robot in its world (see @ref id).
    void identify(std::string p_id)
    {
        m_id = std::move(p_id);
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

    /// @return Work frame: @c tcp when a tool is mounted, otherwise
    ///   @c tool0, @c end_effector, or the last moving link.
    [[nodiscard]] std::string const& tool() const
    {
        return m_tool;
    }

    /// @return The joints: state, commands and limits.
    [[nodiscard]] JointSet& joints()
    {
        return m_joints;
    }

    /// @copydoc joints()
    [[nodiscard]] JointSet const& joints() const
    {
        return m_joints;
    }

    /// @return The sensors.
    [[nodiscard]] SensorSet& sensors()
    {
        return m_sensors;
    }

    /// @copydoc sensors()
    [[nodiscard]] SensorSet const& sensors() const
    {
        return m_sensors;
    }

    /// @return The actuators.
    [[nodiscard]] ActuatorSet& actuators()
    {
        return m_actuators;
    }

    /// @copydoc actuators()
    [[nodiscard]] ActuatorSet const& actuators() const
    {
        return m_actuators;
    }

    /// @return Resources of the robot; shared with the other robots of a
    ///   world (a skill may take an arm of each).
    [[nodiscard]] ResourceManager& resources()
    {
        return *m_resources;
    }

    /// @copydoc resources()
    [[nodiscard]] ResourceManager const& resources() const
    {
        return *m_resources;
    }

    /// @return Robot clock (time since the last reset).
    [[nodiscard]] Seconds time() const
    {
        return m_time;
    }

    // -------------------------------------------------------------------------
    /// @return Base pose and velocity in the world (see @ref BaseState —
    ///   ground truth in simulation, not an estimate).
    // -------------------------------------------------------------------------
    [[nodiscard]] BaseState const& base() const
    {
        return m_base;
    }

    // -------------------------------------------------------------------------
    /// Writes the measured base (backends, or the application for a robot
    /// bolted somewhere else than the world origin).
    /// @param p_base Pose and velocity in the world.
    // -------------------------------------------------------------------------
    void measureBase(BaseState const& p_base)
    {
        m_base = p_base;
    }

    /// @return Backend moving the robot, or null.
    [[nodiscard]] RobotBackend const* backend() const
    {
        return m_backend.get();
    }

    /// @copydoc backend() const
    [[nodiscard]] RobotBackend* backend()
    {
        return m_backend.get();
    }

    /// @return Rotation from the URDF frame (Z-up, the world frame of
    ///   Robotik) to the Compages frame (Y-up) the entities are drawn in.
    [[nodiscard]] compages::core::Quatf const& urdfFrame() const
    {
        return m_world_from_urdf;
    }

    /// @return Link entity named @p_name, or an empty handle.
    [[nodiscard]] compages::world::Entity link(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the robot base frame (measured joints).
    /// @param p_frame Link name.
    /// @return The pose.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose framePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame (base pose times @ref framePose).
    /// @param p_frame Link name; empty for the base itself.
    /// @return The pose.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose worldPose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Finds a link once, for code that reads its pose at every step (see
    /// @ref LinkRef).
    /// @param p_name Link name; empty for the robot base.
    /// @return The handles of the link.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] LinkRef resolve(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame, with no search by name.
    /// @param p_link Link found by @ref resolve.
    /// @return The pose.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose worldPose(LinkRef const& p_link) const;

    // -------------------------------------------------------------------------
    /// Solves the inverse kinematics from the measured joints.
    /// @param p_frame Link to place.
    /// @param p_target Pose wanted, in the robot base frame.
    /// @return Joint positions indexed by @ref JointId in the SI unit of each
    ///   joint, or nothing if the solver does not converge.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::vector<double>>
    inverseKinematics(std::string const& p_frame, Pose const& p_target) const;

    /// @return Analytical model (Pinocchio) for advanced queries.
    [[nodiscard]] PinocchioBackend& kinematics() const
    {
        return *m_kinematics;
    }

protected:

    // -------------------------------------------------------------------------
    /// Loads the URDF into the world, the kinematics and the joints.
    /// @param p_world ECS world receiving the entities.
    /// @param p_urdf URDF of the robot.
    /// @param p_view Rendering hooks, or null for a headless robot.
    /// @param p_tool URDF of a tool on the flange, or empty.
    /// @param p_resources Shared with other robots, or null for its own.
    // -------------------------------------------------------------------------
    Robot(compages::world::World& p_world,
          std::filesystem::path const& p_urdf,
          SceneView* p_view,
          std::filesystem::path const& p_tool = {},
          std::shared_ptr<ResourceManager> p_resources = {});

    /// Copies the joints and the base to the kinematics, then to the
    /// rendered entities.
    void propagate();

protected:

    /// ECS world holding the entities.
    compages::world::World& m_world;
    /// Analytical model for FK and IK.
    std::unique_ptr<PinocchioBackend> m_kinematics;
    /// Backend moving the robot, or null.
    std::unique_ptr<RobotBackend> m_backend;
    /// URDF robot name.
    std::string m_name;
    /// Id in the world.
    std::string m_id;
    /// Work frame.
    std::string m_tool;
    /// Root entity.
    compages::world::Entity m_root;
    /// Joints.
    JointSet m_joints;
    /// Resources, maybe shared with other robots.
    std::shared_ptr<ResourceManager> m_resources;
    /// Sensors.
    SensorSet m_sensors;
    /// Actuators.
    ActuatorSet m_actuators;
    /// Measured base.
    BaseState m_base;
    /// Compages is Y-up; URDF / Pinocchio / MuJoCo stay Z-up. The loader
    /// hangs the robot under this rotation; @ref propagate composes it with
    /// the measured base instead of overwriting it.
    compages::core::Quatf m_world_from_urdf{};
    /// Pinocchio q index of each joint (-1 when not modeled).
    std::vector<int> m_q_indices;
    /// Pinocchio v index of each joint (-1 when not modeled).
    std::vector<int> m_v_indices;
    /// Robot clock.
    Seconds m_time{};
};

// ****************************************************************************
/// A robot being run: owns the backend and advances time.
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
    [[nodiscard]] RobotBackend* backend() const
    {
        return m_backend.get();
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
    /// for (auto* r : robots) r->advance(dt); // the first one steps the
    /// physics for (auto* r : robots) r->sense();
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
