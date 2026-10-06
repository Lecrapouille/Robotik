// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Robot.hpp
//! @brief The robot as seen by skills (@ref Robot) and by the application that
//! runs it (@ref RobotSession).
//!
//! Skills only see @ref Robot: joints, sensors, actuators, resources and
//! kinematics. Whether a MuJoCo model, a kinematic model or the hardware sits
//! behind is the business of the @ref RobotBackend plugged in the session.
#pragma once

#include "Robotik/Backends/RobotBackend.hpp"
#include "Robotik/Backends/SceneView.hpp"
#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Robot/Devices.hpp"
#include "Robotik/Robot/Joints.hpp"
#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Sensors/Measurements.hpp"
#include "Robotik/Sensors/Sensor.hpp"

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

using SensorSet = DeviceSet<Sensor>;
using ActuatorSet = DeviceSet<Actuator>;

//! @brief Joint name to position, in the SI base unit of the joint (rad or m).
using JointPosture = std::unordered_map<std::string, double>;

// ****************************************************************************
//! @brief Robot API used by skills.
// ****************************************************************************
class Robot
{
public:

    Robot(Robot const&) = delete;
    Robot& operator=(Robot const&) = delete;
    virtual ~Robot();

    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    [[nodiscard]] compages::world::World& world() const
    {
        return m_world;
    }

    //! @brief Root entity, placed at the base pose in the world.
    [[nodiscard]] compages::world::Entity root() const
    {
        return m_root;
    }

    //! @brief Tool frame (tool0, tcp, or the last moving link).
    [[nodiscard]] std::string const& tool() const
    {
        return m_tool;
    }

    [[nodiscard]] JointSet& joints()
    {
        return m_joints;
    }

    [[nodiscard]] JointSet const& joints() const
    {
        return m_joints;
    }

    [[nodiscard]] SensorSet& sensors()
    {
        return m_sensors;
    }

    [[nodiscard]] SensorSet const& sensors() const
    {
        return m_sensors;
    }

    [[nodiscard]] ActuatorSet& actuators()
    {
        return m_actuators;
    }

    [[nodiscard]] ActuatorSet const& actuators() const
    {
        return m_actuators;
    }

    [[nodiscard]] ResourceManager& resources()
    {
        return m_resources;
    }

    [[nodiscard]] ResourceManager const& resources() const
    {
        return m_resources;
    }

    //! @brief Robot clock.
    [[nodiscard]] Seconds time() const
    {
        return m_time;
    }

    // -------------------------------------------------------------------------
    //! @brief Base pose and velocity in the world (see @ref BaseState: ground
    //! truth in simulation, not an estimate).
    // -------------------------------------------------------------------------
    [[nodiscard]] BaseState const& base() const
    {
        return m_base;
    }

    //! @brief Writes the measured base (backends, or the application for a
    //! robot bolted somewhere else than the world origin).
    void measureBase(BaseState const& p_base)
    {
        m_base = p_base;
    }

    //! @brief Backend moving the robot, or null.
    [[nodiscard]] RobotBackend const* backend() const
    {
        return m_backend.get();
    }

    //! @brief Link entity named @p_name, or an empty handle.
    [[nodiscard]] compages::world::Entity link(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Pose of a link in the robot base frame (measured joints).
    //! @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose framePose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    //! @brief Pose of a link in the world frame (base pose times
    //! @ref framePose; empty @p_frame for the base itself).
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose worldPose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    //! @brief Joint positions placing @p_frame at @p_target (base frame),
    //! indexed by @ref JointId in the SI unit of each joint, or nothing if the
    //! solver does not converge.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<std::vector<double>>
    inverseKinematics(std::string const& p_frame, Pose const& p_target) const;

    //! @brief Analytical model (Pinocchio) for advanced queries.
    [[nodiscard]] PinocchioBackend& kinematics() const
    {
        return *m_kinematics;
    }

protected:

    Robot(compages::world::World& p_world,
          std::filesystem::path const& p_urdf,
          SceneView* p_view);

    //! @brief Joints and base to kinematics, then to the rendered entities.
    void propagate();

protected:

    compages::world::World& m_world;
    std::unique_ptr<PinocchioBackend> m_kinematics;
    std::unique_ptr<RobotBackend> m_backend;
    std::string m_name;
    std::string m_tool;
    compages::world::Entity m_root;
    JointSet m_joints;
    ResourceManager m_resources;
    SensorSet m_sensors;
    ActuatorSet m_actuators;
    BaseState m_base;
    //!< Compages is Y-up; URDF / Pinocchio / MuJoCo stay Z-up. The loader
    //!< hangs the robot under this rotation; @ref propagate composes it with
    //!< the measured base instead of overwriting it.
    compages::core::Quatf m_world_from_urdf{};
    //!< Pinocchio q / v index of each joint (-1 when not modeled).
    std::vector<int> m_q_indices;
    std::vector<int> m_v_indices;
    Seconds m_time{};
};

// ****************************************************************************
//! @brief A robot being run: owns the backend and advances time.
//!
//! @code
//! compages::world::World world;
//! robotik::RobotSession robot(world, "arm.urdf");
//! robot.actuators().add<robotik::JointGroup>("arm");
//! robot.connect(std::make_unique<robotik::MujocoBackend>("arm.urdf"));
//! robot.hold({ { "joint2", 0.3 } });
//! while (running)
//!     robot.step(Seconds(0.01));
//! @endcode
// ****************************************************************************
class RobotSession final: public Robot
{
public:

    //! @param p_view Rendering hooks, or null for a headless robot.
    RobotSession(compages::world::World& p_world,
                 std::filesystem::path const& p_urdf,
                 SceneView* p_view = nullptr);
    ~RobotSession() override;

    // -------------------------------------------------------------------------
    //! @brief Plugs the backend. Connect after adding the sensors a backend
    //! may need to know (e.g. force/torque sensors for MuJoCo).
    // -------------------------------------------------------------------------
    void connect(std::unique_ptr<RobotBackend> p_backend);

    [[nodiscard]] RobotBackend* backend() const
    {
        return m_backend.get();
    }

    // -------------------------------------------------------------------------
    //! @brief Places the joints of @p_posture, makes the posture the home and
    //! servos every joint where it is.
    // -------------------------------------------------------------------------
    void hold(JointPosture const& p_posture = {});

    // -------------------------------------------------------------------------
    //! @brief Base pose restored by @ref reset (world origin by default).
    // -------------------------------------------------------------------------
    void startPose(Pose const& p_pose);

    // -------------------------------------------------------------------------
    //! @brief Backend step, kinematics, rendered links, then sensors.
    //! Failed actuators are kept in their safe state.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    //! @brief Clock to zero, joints at home, base at its start pose, sensors
    //! rewound.
    // -------------------------------------------------------------------------
    void reset();

private:

    Pose m_start;
};

} // namespace robotik
