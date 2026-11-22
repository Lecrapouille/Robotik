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
/// runs the robot owns a @ref robotik::RobotSession (RobotSession.hpp) — it
/// steps the physics and the sensors, and it alone reaches the ECS world and
/// the backend. So a skill cannot step the world nor read its truth by
/// mistake, and the same skill drives a simulated arm or a real one (see
/// doc/reference/glossary.md: "Backend").
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

#include "Robotik/Robot/Backends/PinocchioBackend.hpp"
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

/// Sensors of a robot (the session adds them).
using SensorSet = DeviceSet<Sensor>;
/// Actuators of a robot (the session adds them).
using ActuatorSet = DeviceSet<Actuator>;
/// Sensors of a robot, as a skill sees them (no add).
using SensorList = DeviceList<Sensor>;
/// Actuators of a robot, as a skill sees them (no add).
using ActuatorList = DeviceList<Actuator>;

// ****************************************************************************
/// Where the base pose a skill reads comes from (@ref Robot::base).
///
/// The true pose of a mobile base is known in simulation only. A real robot
/// estimates it (odometry, landmarks, a calibration). As for the perception
/// fidelity, the scenario chooses, and the run report says it.
// ****************************************************************************
enum class LocalizationFidelity
{
    /// A mock: the estimate is the true base, copied at each step.
    Truth,
    /// An algorithm of the plugin writes it (RobotSession::estimateBase).
    Plugin,
};

/// @return "truth" or "plugin", the word of the scenario file.
[[nodiscard]] constexpr char const*
toString(LocalizationFidelity p_localization)
{
    return p_localization == LocalizationFidelity::Truth ? "truth" : "plugin";
}

/// Joint name to position, in the SI base unit of the joint (rad or m).
using JointPosture = std::unordered_map<std::string, double>;

// ****************************************************************************
/// Robot API used by skills: read the joints and the sensors, send
/// commands, compute the kinematics.
///
/// A skill must not advance time, swap the physics or read the truth of the
/// simulator: if it could, the same skill would behave differently in a test
/// and on hardware. So this class has no @c step, no backend, no ECS world
/// and a read-only kinematics. It cannot be built directly: an
/// application builds a @ref RobotSession (or calls @ref openRobot), and a
/// skill receives this view through @ref RobotContext.
///
/// The kinematics (@ref requireFramePose, @ref inverseKinematics) use Pinocchio on
/// the measured joints, not the true pose of the simulator, so they are
/// right on a real robot too. Poses are in the base frame of the robot,
/// except @ref requireWorldPose. A joint or frame name that the URDF does not have
/// throws @c std::invalid_argument: it is a mistake in the code. In a hot
/// loop, find a handle once (@ref JointSet::requireRevolute, @ref requireLink) instead
/// of a name at each step.
///
/// @code
/// robotik::Robot& robot = p_context.robot;
/// robotik::Pose const tool = robot.requireFramePose(robot.tool());     // base frame
/// robotik::Pose const in_world = robot.requireWorldPose(robot.tool()); // world frame
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

    /// @return The sensors (to add one: @ref RobotSession::sensors).
    [[nodiscard]] SensorList& sensors()
    {
        return m_sensors;
    }

    /// @copydoc sensors()
    [[nodiscard]] SensorList const& sensors() const
    {
        return m_sensors;
    }

    /// @return The actuators (to add one: @ref RobotSession::actuators).
    [[nodiscard]] ActuatorList& actuators()
    {
        return m_actuators;
    }

    /// @copydoc actuators()
    [[nodiscard]] ActuatorList const& actuators() const
    {
        return m_actuators;
    }

    /// @return Resources of the robot, read only; shared with the other
    ///   robots of a world. The scheduler reserves them; faults and devices
    ///   change them through @ref RobotSession::resources.
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
    /// @return Estimated base pose and velocity in the world (see
    ///   @ref LocalizationFidelity): the true base with @c localization: @c truth,
    ///   what the localization of the plugin wrote otherwise. A robot bolted
    ///   to a table keeps its start pose.
    // -------------------------------------------------------------------------
    [[nodiscard]] BaseState const& base() const
    {
        return m_base;
    }

    // -------------------------------------------------------------------------
    /// Pose of a link in the robot base frame (measured joints).
    /// @param p_frame Link name.
    /// @return The pose.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireFramePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame (estimated @ref base times
    /// @ref requireFramePose).
    /// @param p_frame Link name; empty for the base itself.
    /// @return The pose.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireWorldPose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Finds a link once, for code that reads its pose at every step (see
    /// @ref LinkRef).
    /// @param p_name Link name; empty for the robot base.
    /// @return The handles of the link.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] LinkRef requireLink(std::string const& p_name) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame, with no search by name.
    /// @param p_link Link found by @ref requireLink on this robot.
    /// @return The pose.
    /// @throws std::logic_error if @p_link comes from another robot or was
    ///   never found (a mistake in the code, see @ref LinkRef).
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose worldPose(LinkRef const& p_link) const;

    // -------------------------------------------------------------------------
    /// Solves the inverse kinematics from the measured joints.
    /// @param p_frame Link to place.
    /// @param p_target Pose wanted, in the robot base frame.
    /// @return Joint positions indexed by @ref JointId in the SI unit of each
    ///   joint, or nothing if the solver does not converge.
    /// @param p_goal The whole pose, or only its position or orientation
    ///   (a link with fewer than six joints above it).
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::vector<double>>
    inverseKinematics(std::string const& p_frame,
                      Pose const& p_target,
                      IkGoal p_goal = IkGoal::Whole) const;

    /// @return Analytical model (Pinocchio), read only.
    [[nodiscard]] PinocchioBackend const& kinematics() const
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
    /// rendered entities (at the true base).
    void propagate();

    /// Pose of @p_link in the world from @p_base (estimated or true).
    /// @throws std::logic_error if @p_link comes from another robot.
    [[nodiscard]] Pose poseFrom(BaseState const& p_base,
                                LinkRef const& p_link) const;

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
    /// Estimated base: what the skills read.
    BaseState m_base;
    /// True base, written by the backend (the simulator side reads it).
    BaseState m_true_base;
    /// Compages is Y-up; URDF / Pinocchio / MuJoCo stay Z-up. The loader
    /// hangs the robot under this rotation; @ref propagate composes it with
    /// the measured base instead of overwriting it.
    compages::Quatf m_world_from_urdf{};
    /// Pinocchio q index of each joint (-1 when not modeled).
    std::vector<int> m_q_indices;
    /// Pinocchio v index of each joint (-1 when not modeled).
    std::vector<int> m_v_indices;
    /// Robot clock.
    Seconds m_time{};
    /// Serial number, unique in the process: the owner of its @ref LinkRef.
    std::uint64_t m_serial;
};

} // namespace robotik
