// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Robot.hpp
/// The robot: its joints, sensors, actuators and kinematics, in one object.
///
/// A skill gets a @ref robotik::Robot: it reads the joints and the sensors and
/// sends commands, nothing more. The program that runs the robot owns a
/// @ref robotik::RobotRuntime (RobotRuntime.hpp), which owns the
/// @ref robotik::Robot and lends it to the skills. The runtime alone holds
/// the ECS world, the drawn entities, the backend and the true base; the
/// @ref robotik::Robot does not even store them. So a skill cannot step the
/// world nor read its truth, and the same skill drives a simulated arm or a
/// real one (see doc/reference/glossary.md: "Backend").
///
/// @par Contract
/// - Promises: a skill sees the same @ref robotik::Robot API in simulation and
///   on a real robot; only the @ref robotik::RobotBackend of the runtime
///   changes. The sensors are read only from here.
/// - Expects: one @ref robotik::RobotRuntime::step per control period; names of
///   joints and frames that exist in the URDF.
/// - Errors: an unknown joint or frame name is a programming error and throws
///   @c std::invalid_argument. Opening a robot from a file is done by
///   @ref robotik::StandaloneRobot::open, which returns a @c Result.
#pragma once

#include "RobotIK/Common/Math.hpp"
#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Robot/Devices.hpp"
#include "RobotIK/Robot/Fidelity.hpp"
#include "RobotIK/Robot/Joints.hpp"
#include "RobotIK/Robot/Kinematics.hpp"
#include "RobotIK/Robot/LinkRef.hpp"
#include "RobotIK/Robot/Resources.hpp"
#include "RobotIK/Robot/Sensors/Measurements.hpp"
#include "RobotIK/Robot/Sensors/Sensor.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

/// Sensors of a robot (the runtime adds them).
using SensorSet = DeviceSet<Sensor>;
/// Actuators of a robot (the runtime adds them).
using ActuatorSet = DeviceSet<Actuator>;

// ****************************************************************************
/// Robot API used by skills: read the joints and the sensors, send
/// commands, compute the kinematics.
///
/// A skill must not advance time, swap the physics or read the truth of the
/// simulator: if it could, the same skill would behave differently in a test
/// and on hardware. So this class has no @c step, no backend, no ECS world,
/// no entity, no true base, read-only sensors and a read-only kinematics:
/// it does not store them. It cannot be built directly: an application
/// builds a @ref RobotRuntime (or calls @ref robotik::StandaloneRobot::open), which owns this
/// object and lends it with @ref RobotRuntime::robot; a skill receives it
/// through @ref RobotContext.
///
/// Why owned and not a base class: the runtime is not a kind of robot, it
/// runs one. Owning the @ref Robot keeps the truth out of its storage, and a
/// skill cannot cast its @ref Robot to the runtime.
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
/// robotik::Pose const tool = robot.requireFramePose(robot.getTool());     // base frame
/// robotik::Pose const in_world = robot.requireWorldPose(robot.getTool()); // world frame
/// if (auto const q = robot.inverseKinematics(robot.getTool(), goal))
///     for (robotik::JointId id = 0; id < q->size(); ++id)
///         robot.getJoints().moveTo(id, (*q)[id]);
/// @endcode
// ****************************************************************************
class Robot final
{
    // The runtime builds the robot, writes what is measured, adds devices.
    friend class RobotRuntime;

public:

    // ************************************************************************
    /// What does not change in a robot after it is loaded: its name, its
    /// URDF, its work frame and its moving joints, in the order of their
    /// @ref JointId.
    ///
    /// It is the "model" half of the model and state split of Pinocchio:
    /// the @ref Robot holds the state (measured joints, base) and its
    /// model. Code that keeps data for later (a taught waypoint, a saved
    /// posture) can say for which joints it was made, and check it.
    ///
    /// Guarantees: the runtime writes it once, when it loads the robot;
    /// @ref joints, @ref q_indices and @ref v_indices have one entry per
    /// joint. Limits: a tool is given when the robot is built, so the model
    /// stays the same for the whole life of the robot.
    ///
    /// @code
    /// robotik::Robot::Model const& model = robot.getModel();
    /// std::cout << model.name << " has " << model.joints.size() << " joints\n";
    /// @endcode
    // ************************************************************************
    struct Model
    {
        /// Name of the robot in its URDF (@c <robot name="...">).
        std::string name;
        /// URDF file of the robot.
        std::filesystem::path urdf;
        /// URDF file of the tool mounted on the flange, or empty.
        std::filesystem::path tool_urdf;
        /// Work frame: @c tcp when a tool is mounted, otherwise @c tool0,
        /// @c end_effector, or the last moving link.
        std::string tool;
        /// Names of the moving joints, in the order of their @ref JointId.
        std::vector<std::string> joints;
        /// Pinocchio q index of each joint (-1 when not modeled).
        std::vector<int> q_indices;
        /// Pinocchio v index of each joint (-1 when not modeled).
        std::vector<int> v_indices;
    };

    Robot(Robot const&) = delete;
    Robot& operator=(Robot const&) = delete;
    ~Robot();

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame, with no search by name.
    /// @param p_link Link found by @ref requireLink on this robot.
    /// @return The pose.
    /// @throws std::logic_error if @p_link comes from another robot or was
    ///   never found (a mistake in the code, see @ref LinkRef).
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose getWorldPose(LinkRef const& p_link) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the robot base frame (measured joints).
    /// @param p_frame Link name.
    /// @return The pose.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireFramePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// Pose of a link in the world frame (estimated @ref getBase times
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

    // -------------------------------------------------------------------------
    /// @return The joints: state, commands and limits.
    // -------------------------------------------------------------------------
    [[nodiscard]] JointSet& getJoints()
    {
        return m_joints;
    }

    // -------------------------------------------------------------------------
    /// @copydoc getJoints()
    // -------------------------------------------------------------------------
    [[nodiscard]] JointSet const& getJoints() const
    {
        return m_joints;
    }

    // -------------------------------------------------------------------------
    /// @return The sensors, read only: a skill reads them, it cannot change
    ///   their source, their noise seed nor their reading (the runtime
    ///   does, @ref RobotRuntime::getSensors). A command to a sensor
    ///   (trigger a capture, pan a camera) is an actuator.
    // -------------------------------------------------------------------------
    [[nodiscard]] SensorSet const& getSensors() const
    {
        return m_sensors;
    }

    // -------------------------------------------------------------------------
    /// @return The actuators: a skill commands them (to add one: @ref
    /// RobotRuntime::addActuator).
    // -------------------------------------------------------------------------
    [[nodiscard]] ActuatorSet& getActuators()
    {
        return m_actuators;
    }

    // -------------------------------------------------------------------------
    /// @copydoc getActuators()
    // -------------------------------------------------------------------------
    [[nodiscard]] ActuatorSet const& getActuators() const
    {
        return m_actuators;
    }

    // -------------------------------------------------------------------------
    /// @return Resources of the robot, read only; shared with the other
    /// robots of a world. The scheduler reserves them; faults and devices
    /// change them through @ref RobotRuntime::getResources.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceManager const& getResources() const
    {
        return *m_resources;
    }

    // -------------------------------------------------------------------------
    /// @return Estimated base pose and velocity in the world (see @ref
    /// LocalizationFidelity): the true base with @c localization: @c truth,
    /// what the localization of the plugin wrote otherwise. A robot bolted
    /// to a table keeps its start pose.
    // -------------------------------------------------------------------------
    [[nodiscard]] BaseState const& getBase() const
    {
        return m_base;
    }

    // -------------------------------------------------------------------------
    /// @return Analytical model (Pinocchio), read only.
    // -------------------------------------------------------------------------
    [[nodiscard]] Kinematics const& getKinematics() const
    {
        return m_kinematics;
    }

    // -------------------------------------------------------------------------
    /// @return What does not change: name, URDF, work frame, joint names.
    // -------------------------------------------------------------------------
    [[nodiscard]] Model const& getModel() const
    {
        return m_model;
    }

    // -------------------------------------------------------------------------
    /// @return Name of the URDF robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& getName() const
    {
        return m_model.name;
    }

    // -------------------------------------------------------------------------
    /// @return Id of the robot in its world (the entity id of the world file);
    /// the URDF name until @ref setId. Two copies of a model share the name,
    /// not the id.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& getId() const
    {
        return m_id;
    }

    // -------------------------------------------------------------------------
    /// @return Work frame: @c tcp when a tool is mounted, otherwise @c tool0,
    /// @c end_effector, or the last moving link.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& getTool() const
    {
        return m_model.tool;
    }

private:

    // -------------------------------------------------------------------------
    /// Builds the kinematics; @ref RobotRuntime then adds the joints and the
    /// model.
    /// @param p_urdf URDF of the robot.
    /// @param p_tool Tool mounted on the robot, or none.
    /// @param p_resources Shared with other robots, or null for its own.
    // -------------------------------------------------------------------------
    Robot(std::filesystem::path const& p_urdf,
          ToolMount const& p_tool,
          std::shared_ptr<ResourceManager> p_resources);

    // -------------------------------------------------------------------------
    /// Copies the measured joints to the kinematics.
    // -------------------------------------------------------------------------
    void updateKinematics();

    // -------------------------------------------------------------------------
    /// Pose of @p_link in the world from @p_base (estimated or true).
    /// @throws std::logic_error if @p_link comes from another robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose poseFrom(BaseState const& p_base,
                                LinkRef const& p_link) const;

    /// What does not change (written once by the runtime, when it knows
    /// the joints).
    Model m_model;
    /// Analytical model for FK and IK, on the measured joints.
    Kinematics m_kinematics;
    /// Id in the world.
    std::string m_id;
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
    /// Serial number, unique in the process: the owner of its @ref LinkRef.
    std::uint64_t m_serial;
};

} // namespace robotik
