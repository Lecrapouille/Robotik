// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Joints.hpp
//! @brief Joint state, commands, limits and low-level loops of one robot.
//!
//! Two views of the same joints:
//! @li @ref JointSet — the hot data, one contiguous array per quantity indexed
//!     by @ref JointId, walked by control loops and backends at 1 kHz;
//! @li the Compages joint components (@c compages::world::RevoluteJoint,
//!     @c PrismaticJoint) on the link entities — limits loaded from the URDF,
//!     and a mirror of the measured state (position, velocity, effort) for
//!     rendering and inspection, refreshed after each step.
//!
//! The public API is typed with SI units: a revolute joint is addressed by a
//! @ref Revolute handle and speaks @c Radians, @c AngularVelocity and
//! @c Torque; a prismatic joint by a @ref Prismatic handle and speaks
//! @c Length, @c LinearVelocity and @c Force. Code that walks joints of
//! mixed types (IK solutions, backends, postures) uses @ref JointId and the
//! SI base unit of each joint (rad or m), as Pinocchio and MuJoCo do.
#pragma once

#include "Compages/Core/Units.hpp"
#include "Compages/World/Entity.hpp"
#include "Compages/World/Spatial/Joint.hpp"

#include <cstdint>
#include <limits>
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
    //!< Revolute without position limits (wheels).
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
//! @brief Units of a joint, from its position unit (same unit triplets as the
//! Compages joint states).
// ****************************************************************************
template <class Position>
struct JointUnits;

template <>
struct JointUnits<Radians>
{
    using State = compages::world::RevoluteState;
};

template <>
struct JointUnits<Length>
{
    using State = compages::world::PrismaticState;
};

template <class Position>
using VelocityOf = typename JointUnits<Position>::State::Velocity;
template <class Position>
using EffortOf = typename JointUnits<Position>::State::Effort;

// ****************************************************************************
//! @brief URDF limits of a joint. Infinite values mean unlimited.
// ****************************************************************************
template <class Position>
struct JointLimits
{
    Position lower{ -std::numeric_limits<double>::infinity() };
    Position upper{ std::numeric_limits<double>::infinity() };
    //!< Largest speed, in both directions.
    VelocityOf<Position> velocity{ std::numeric_limits<double>::infinity() };
    //!< Largest torque or force, in both directions.
    EffortOf<Position> effort{ std::numeric_limits<double>::infinity() };

    [[nodiscard]] bool bounded() const
    {
        return upper > lower &&
               upper.value() < std::numeric_limits<double>::infinity();
    }
};

using RevoluteLimits = JointLimits<Radians>;
using PrismaticLimits = JointLimits<Length>;

// ****************************************************************************
//! @brief Typed handle on a joint: the unit is part of the type.
// ****************************************************************************
template <class Position>
struct JointHandle
{
    JointId id = NO_JOINT;

    [[nodiscard]] explicit operator bool() const
    {
        return id != NO_JOINT;
    }
};

//! @brief Revolute or continuous joint (rad, rad/s, N.m).
using Revolute = JointHandle<Radians>;
//! @brief Prismatic joint (m, m/s, N).
using Prismatic = JointHandle<Length>;

// ****************************************************************************
//! @brief Low-level loop tuning (SI, per unit of the joint).
// ****************************************************************************
struct JointGains
{
    //!< Position loop stiffness (N.m/rad or N/m).
    double kp = 150.0;
    //!< Position loop damping (N.m.s/rad or N.s/m).
    double kd = 15.0;
    //!< Largest reference slew as a fraction of the velocity limit.
    double speed_ratio = 0.5;
    //!< Velocity loop gain (N.m.s/rad or N.s/m).
    double velocity_kp = 5.0;
};

// ****************************************************************************
//! @brief All joints of a robot.
//!
//! @code
//! robotik::JointSet& joints = robot.joints();
//! robotik::Revolute const elbow = joints.revolute("joint3");
//! joints.moveTo(elbow, Radians(0.8));
//! if (joints.reached(elbow, Radians(0.01))) { ... }
//! robotik::Revolute const wheel = joints.revolute("left_wheel_joint");
//! joints.spin(wheel, AngularVelocity(3.0));
//! @endcode
// ****************************************************************************
class JointSet
{
public:

    // -------------------------------------------------------------------------
    //! @brief Appends the joint carried by @p_link, reading its type and its
    //! limits from the Compages joint component (filled from the URDF).
    //! @throws std::invalid_argument if @p_link has no joint component or the
    //! name is taken.
    // -------------------------------------------------------------------------
    JointId add(std::string p_name, compages::world::Entity p_link);

    // -------------------------------------------------------------------------
    //! @brief Appends a joint without rendered link (tests, drivers).
    //! @throws std::invalid_argument if the name is taken.
    // -------------------------------------------------------------------------
    Revolute add(std::string p_name,
                 RevoluteLimits const& p_limits,
                 bool p_continuous = false);
    Prismatic add(std::string p_name, PrismaticLimits const& p_limits);

    [[nodiscard]] std::size_t size() const
    {
        return m_names.size();
    }

    //! @brief Joint named @p_name, or @ref NO_JOINT.
    [[nodiscard]] JointId find(std::string_view p_name) const;

    //! @brief Joint named @p_name. @throws std::invalid_argument if unknown.
    [[nodiscard]] JointId require(std::string_view p_name) const;

    //! @brief Typed handles. @throws std::invalid_argument if unknown or of
    //! the other kind.
    [[nodiscard]] Revolute revolute(std::string_view p_name) const;
    [[nodiscard]] Prismatic prismatic(std::string_view p_name) const;
    [[nodiscard]] Revolute revolute(JointId p_id) const;
    [[nodiscard]] Prismatic prismatic(JointId p_id) const;

    // --- Description --------------------------------------------------------

    [[nodiscard]] std::string const& name(JointId p_id) const
    {
        return m_names[p_id];
    }

    [[nodiscard]] JointType type(JointId p_id) const
    {
        return m_types[p_id];
    }

    [[nodiscard]] bool isPrismatic(JointId p_id) const
    {
        return m_types[p_id] == JointType::Prismatic;
    }

    template <class Position>
    [[nodiscard]] JointLimits<Position> limits(JointHandle<Position> p_joint) const
    {
        Bounds const& bounds = m_bounds[p_joint.id];
        return { Position(bounds.lower),
                 Position(bounds.upper),
                 VelocityOf<Position>(bounds.velocity),
                 EffortOf<Position>(bounds.effort) };
    }

    [[nodiscard]] JointGains& gains(JointId p_id)
    {
        return m_gains[p_id];
    }

    [[nodiscard]] compages::world::Entity link(JointId p_id) const
    {
        return m_links[p_id];
    }

    // --- Measured state (typed) ---------------------------------------------

    template <class Position>
    [[nodiscard]] Position position(JointHandle<Position> p_joint) const
    {
        return Position(m_positions[p_joint.id]);
    }

    template <class Position>
    [[nodiscard]] VelocityOf<Position> velocity(JointHandle<Position> p_joint) const
    {
        return VelocityOf<Position>(m_velocities[p_joint.id]);
    }

    //! @brief Effort produced by the last @ref control call.
    template <class Position>
    [[nodiscard]] EffortOf<Position> effort(JointHandle<Position> p_joint) const
    {
        return EffortOf<Position>(m_efforts[p_joint.id]);
    }

    // --- Commands (typed) ---------------------------------------------------

    template <class Position>
    void moveTo(JointHandle<Position> p_joint, Position p_position)
    {
        moveTo(p_joint.id, p_position.value());
    }

    template <class Position>
    void spin(JointHandle<Position> p_joint, VelocityOf<Position> p_velocity)
    {
        spin(p_joint.id, p_velocity.value());
    }

    template <class Position>
    void push(JointHandle<Position> p_joint, EffortOf<Position> p_effort)
    {
        push(p_joint.id, p_effort.value());
    }

    //! @brief True in position mode once |position - target| <= @p_tolerance.
    template <class Position>
    [[nodiscard]] bool reached(JointHandle<Position> p_joint,
                               Position p_tolerance) const
    {
        return reached(p_joint.id, p_tolerance.value());
    }

    template <class Position>
    [[nodiscard]] Position home(JointHandle<Position> p_joint) const
    {
        return Position(m_homes[p_joint.id]);
    }

    template <class Position>
    void home(JointHandle<Position> p_joint, Position p_position)
    {
        m_homes[p_joint.id] = p_position.value();
    }

    // --- Mixed joints: SI base unit of each joint (rad or m) ----------------

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

    [[nodiscard]] double position(JointId p_id) const
    {
        return m_positions[p_id];
    }

    [[nodiscard]] double velocity(JointId p_id) const
    {
        return m_velocities[p_id];
    }

    [[nodiscard]] double effort(JointId p_id) const
    {
        return m_efforts[p_id];
    }

    [[nodiscard]] double home(JointId p_id) const
    {
        return m_homes[p_id];
    }

    void home(JointId p_id, double p_position)
    {
        m_homes[p_id] = p_position;
    }

    [[nodiscard]] JointMode mode(JointId p_id) const
    {
        return m_modes[p_id];
    }

    //! @brief Setpoint of the current mode (position, velocity or effort).
    [[nodiscard]] double target(JointId p_id) const
    {
        return m_targets[p_id];
    }

    void moveTo(JointId p_id, double p_position);
    void spin(JointId p_id, double p_velocity);
    void push(JointId p_id, double p_effort);
    [[nodiscard]] bool reached(JointId p_id, double p_tolerance) const;

    //! @brief Servos the joint where it is.
    void hold(JointId p_id);
    //! @brief Servos every joint where it is.
    void hold();
    //! @brief Removes any effort (the joint becomes free).
    void release(JointId p_id);

    // --- Backend side -------------------------------------------------------

    //! @brief Writes a measurement.
    void measure(JointId p_id, double p_position, double p_velocity)
    {
        m_positions[p_id] = p_position;
        m_velocities[p_id] = p_velocity;
    }

    //! @brief Teleports a joint at rest: state, reference and command.
    void place(JointId p_id, double p_position);

    // -------------------------------------------------------------------------
    //! @brief Runs the low-level loop of every joint for @p_dt and stores the
    //! efforts (see @ref efforts). Backends call it at their physics rate.
    // -------------------------------------------------------------------------
    void control(Seconds p_dt);

    // -------------------------------------------------------------------------
    //! @brief Copies the measured state into the Compages joint components of
    //! the links (rendering and inspection).
    // -------------------------------------------------------------------------
    void publish() const;

private:

    //! @brief Limits in SI base units.
    struct Bounds
    {
        double lower;
        double upper;
        double velocity;
        double effort;
    };

    JointId append(std::string p_name,
                   JointType p_type,
                   Bounds const& p_bounds,
                   compages::world::Entity p_link);

private:

    // Cold data.
    std::vector<std::string> m_names;
    std::vector<compages::world::Entity> m_links;
    std::vector<JointGains> m_gains;
    std::vector<double> m_homes;
    // Hot data, walked every physics step.
    std::vector<JointType> m_types;
    std::vector<JointMode> m_modes;
    std::vector<Bounds> m_bounds;
    std::vector<double> m_positions;
    std::vector<double> m_velocities;
    std::vector<double> m_efforts;
    std::vector<double> m_targets;
    std::vector<double> m_references;
};

} // namespace robotik
