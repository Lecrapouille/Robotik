// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Joints.hpp
//! @brief Dense joint state, commands and low-level PD loop of one robot.
//!
//! Values are SI doubles (rad or m, depending on @ref JointType) stored in
//! parallel arrays indexed by @ref JointId: control loops and backends walk
//! contiguous memory instead of chasing ECS components.
#pragma once

#include "Compages/Core/Units.hpp"
#include "Compages/World/Entity.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

using JointId = std::uint16_t;
inline constexpr JointId NO_JOINT = 0xFFFFu;

enum class JointType : std::uint8_t
{
    Revolute,
    Continuous,
    Prismatic,
};

// ****************************************************************************
//! @brief How the command of a joint is interpreted.
// ****************************************************************************
enum class JointMode : std::uint8_t
{
    Disabled, //!< No effort: the joint is free.
    Position, //!< PD loop toward a rate-limited reference.
    Velocity, //!< Proportional velocity loop.
    Effort,   //!< Raw effort, clamped to the limit.
};

// ****************************************************************************
//! @brief URDF limits (SI). Zero velocity or effort means unlimited.
// ****************************************************************************
struct JointLimits
{
    double lower = 0.0;
    double upper = 0.0;
    double velocity = 0.0;
    double effort = 0.0;

    [[nodiscard]] bool bounded() const
    {
        return upper > lower;
    }
};

// ****************************************************************************
//! @brief Low-level loop tuning.
// ****************************************************************************
struct JointGains
{
    double kp = 150.0;
    double kd = 15.0;
    //!< Max reference slew as a fraction of the velocity limit.
    double speed_ratio = 0.5;
    //!< Gain of the velocity loop.
    double velocity_kp = 5.0;
};

// ****************************************************************************
//! @brief All joints of a robot.
//!
//! @code
//! robotik::JointSet& joints = robot.joints();
//! robotik::JointId const elbow = joints.find("joint3");
//! joints.moveTo(elbow, 0.8);
//! if (joints.reached(elbow, 1e-2)) { ... }
//! @endcode
// ****************************************************************************
class JointSet
{
public:

    // -------------------------------------------------------------------------
    //! @brief Appends a joint; @p_link is the rendered link it moves.
    // -------------------------------------------------------------------------
    JointId add(std::string p_name,
                JointType p_type,
                JointLimits const& p_limits,
                compages::world::Entity p_link = {});

    [[nodiscard]] std::size_t size() const
    {
        return m_names.size();
    }

    //! @brief Joint named @p_name, or @ref NO_JOINT.
    [[nodiscard]] JointId find(std::string_view p_name) const;

    //! @brief Joint named @p_name. @throws std::invalid_argument if unknown.
    [[nodiscard]] JointId require(std::string_view p_name) const;

    // --- Description --------------------------------------------------------

    [[nodiscard]] std::string const& name(JointId p_id) const
    {
        return m_names[p_id];
    }

    [[nodiscard]] JointType type(JointId p_id) const
    {
        return m_types[p_id];
    }

    [[nodiscard]] JointLimits const& limits(JointId p_id) const
    {
        return m_limits[p_id];
    }

    [[nodiscard]] JointGains& gains(JointId p_id)
    {
        return m_gains[p_id];
    }

    [[nodiscard]] compages::world::Entity link(JointId p_id) const
    {
        return m_links[p_id];
    }

    // --- Measured state -----------------------------------------------------

    [[nodiscard]] double position(JointId p_id) const
    {
        return m_positions[p_id];
    }

    [[nodiscard]] double velocity(JointId p_id) const
    {
        return m_velocities[p_id];
    }

    //! @brief Effort produced by the last @ref control call.
    [[nodiscard]] double effort(JointId p_id) const
    {
        return m_efforts[p_id];
    }

    [[nodiscard]] std::span<double const> positions() const
    {
        return m_positions;
    }

    [[nodiscard]] std::span<double const> velocities() const
    {
        return m_velocities;
    }

    [[nodiscard]] std::span<double const> efforts() const
    {
        return m_efforts;
    }

    //! @brief Writes a measurement (backends only).
    void measure(JointId p_id, double p_position, double p_velocity)
    {
        m_positions[p_id] = p_position;
        m_velocities[p_id] = p_velocity;
    }

    //! @brief Teleports a joint at rest: state, reference and command.
    void place(JointId p_id, double p_position);

    // --- Commands -----------------------------------------------------------

    [[nodiscard]] JointMode mode(JointId p_id) const
    {
        return m_modes[p_id];
    }

    //! @brief Setpoint of the current mode (rad, m, rad/s, m/s, N.m or N).
    [[nodiscard]] double target(JointId p_id) const
    {
        return m_targets[p_id];
    }

    void moveTo(JointId p_id, double p_position);
    void spin(JointId p_id, double p_velocity);
    void push(JointId p_id, double p_effort);
    //! @brief Servos the joint where it is.
    void hold(JointId p_id);
    //! @brief Servos every joint where it is.
    void hold();
    //! @brief Removes any effort (the joint becomes free).
    void release(JointId p_id);

    //! @brief True in position mode once |position - target| <= @p_tolerance.
    [[nodiscard]] bool reached(JointId p_id, double p_tolerance) const;

    // --- Home ---------------------------------------------------------------

    [[nodiscard]] double home(JointId p_id) const
    {
        return m_homes[p_id];
    }

    void home(JointId p_id, double p_position)
    {
        m_homes[p_id] = p_position;
    }

    // -------------------------------------------------------------------------
    //! @brief Runs the low-level loop of every joint for @p_dt and stores the
    //! efforts (see @ref efforts). Backends call it at their physics rate.
    // -------------------------------------------------------------------------
    void control(Seconds p_dt);

private:

    // Cold data.
    std::vector<std::string> m_names;
    std::vector<compages::world::Entity> m_links;
    std::vector<JointGains> m_gains;
    std::vector<JointLimits> m_limits;
    std::vector<double> m_homes;
    // Hot data, walked every physics step.
    std::vector<JointType> m_types;
    std::vector<JointMode> m_modes;
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
    std::vector<double> m_efforts;
    std::vector<double> m_targets;
    std::vector<double> m_references;
};

} // namespace robotik
