// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Joints.hpp
/// Joint state, commands, limits and low-level loops of one robot.
///
/// Two views of the same joints:
/// @li @ref JointSet — the hot data, one contiguous array per quantity indexed
///     by @ref JointId, walked by control loops and backends at 1 kHz;
/// @li the Compages joint components (@c compages::world::RevoluteJoint,
///     @c PrismaticJoint) on the link entities — limits loaded from the URDF,
///     and a mirror of the measured state (position, velocity, effort) for
///     rendering and inspection, refreshed after each step.
///
/// The public API is typed with SI units: a revolute joint is addressed by a
/// @ref Revolute handle and speaks @c Radians, @c AngularVelocity and
/// @c Torque; a prismatic joint by a @ref Prismatic handle and speaks
/// @c Length, @c LinearVelocity and @c Force. Code that walks joints of
/// mixed types (IK solutions, backends, postures) uses @ref JointId and the
/// SI base unit of each joint (rad or m), as Pinocchio and MuJoCo do.
#pragma once

#include "Compages/World/Entity.hpp"
#include "Compages/World/Spatial/Joint.hpp"
#include "Robotik/Math/Si.hpp"

#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

/// Index of a joint in its @ref JointSet (untyped).
using JointId = std::uint16_t;
/// No joint (e.g. not found).
inline constexpr JointId NO_JOINT = 0xFFFFu;

// ****************************************************************************
/// Kind of a joint, read from the URDF.
// ****************************************************************************
enum class JointType : std::uint8_t
{
    /// Revolute with position limits (an arm joint).
    Revolute,
    /// Revolute without position limits (wheels).
    Continuous,
    /// Sliding joint (a linear axis, a gripper finger).
    Prismatic,
};

// ****************************************************************************
/// How the command of a joint is interpreted.
///
/// The mode is set by the command: @ref JointSet::moveTo selects
/// @c Position, @ref JointSet::spin @c Velocity, @ref JointSet::push
/// @c Effort, @ref JointSet::release @c Disabled.
// ****************************************************************************
enum class JointMode : std::uint8_t
{
    Disabled, ///< No effort: the joint is free.
    Position, ///< PD loop toward a rate-limited reference.
    Velocity, ///< Proportional velocity loop.
    Effort,   ///< Raw effort, clamped to the limit.
};

// ****************************************************************************
/// Units of a joint, from its position unit (same unit triplets as the
/// Compages joint states).
///
/// @code
/// using V = robotik::VelocityOf<Radians>; // AngularVelocity
/// using E = robotik::EffortOf<Length>;    // Force
/// @endcode
///
/// @tparam Position @c Radians or @c Length.
// ****************************************************************************
template <class Position>
struct JointUnits;

/// Units of a revolute joint: rad, rad/s, N.m.
template <>
struct JointUnits<Radians>
{
    using State = compages::world::RevoluteState; ///< Compages state component.
    using Velocity = AngularVelocity;             ///< Speed unit.
    using Effort = Torque;                        ///< Effort unit.
};

/// Units of a prismatic joint: m, m/s, N.
template <>
struct JointUnits<Length>
{
    using State =
        compages::world::PrismaticState; ///< Compages state component.
    using Velocity = LinearVelocity;     ///< Speed unit.
    using Effort = Force;                ///< Effort unit.
};

/// Speed unit of a joint whose position is @p Position.
template <class Position>
using VelocityOf = typename JointUnits<Position>::Velocity;
/// Effort unit of a joint whose position is @p Position.
template <class Position>
using EffortOf = typename JointUnits<Position>::Effort;

// ****************************************************************************
/// URDF limits of a joint. Infinite values mean unlimited.
///
/// @code
/// robotik::RevoluteLimits const l = joints.limits(elbow);
/// Radians const middle = 0.5 * (l.lower + l.upper);
/// @endcode
///
/// @tparam Position @c Radians or @c Length.
// ****************************************************************************
template <class Position>
struct JointLimits
{
    /// Lowest position.
    Position lower{ -std::numeric_limits<double>::infinity() };
    /// Highest position.
    Position upper{ std::numeric_limits<double>::infinity() };
    /// Largest speed, in both directions.
    VelocityOf<Position> velocity{ std::numeric_limits<double>::infinity() };
    /// Largest torque or force, in both directions.
    EffortOf<Position> effort{ std::numeric_limits<double>::infinity() };

    // ------------------------------------------------------------------------
    /// @return True when the position range is finite and not empty.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool bounded() const
    {
        return upper > lower &&
               upper.value() < std::numeric_limits<double>::infinity();
    }
};

/// Limits of a revolute joint.
using RevoluteLimits = JointLimits<Radians>;
/// Limits of a prismatic joint.
using PrismaticLimits = JointLimits<Length>;

// ****************************************************************************
/// Typed handle on a joint: the unit is part of the type, so a revolute
/// joint cannot be given meters by mistake.
///
/// @code
/// robotik::Revolute const elbow = joints.revolute("joint3");
/// if (elbow)
///     joints.moveTo(elbow, Radians(0.8)); // joints.moveTo(elbow, Length(1))
///     does not compile
/// @endcode
///
/// @tparam Position @c Radians or @c Length.
// ****************************************************************************
template <class Position>
struct JointHandle
{
    /// Index in the @ref JointSet, or @ref NO_JOINT.
    JointId id = NO_JOINT;

    // ------------------------------------------------------------------------
    /// @return True when the handle names a joint.
    // ------------------------------------------------------------------------
    [[nodiscard]] explicit operator bool() const
    {
        return id != NO_JOINT;
    }
};

/// Revolute or continuous joint (rad, rad/s, N.m).
using Revolute = JointHandle<Radians>;
/// Prismatic joint (m, m/s, N).
using Prismatic = JointHandle<Length>;

// ****************************************************************************
/// Low-level loop tuning (SI, per unit of the joint).
///
/// @code
/// robotik::JointGains& g = joints.gains(joints.require("joint1"));
/// g.kp = 300.0;   // stiffer
/// g.kd = 20.0;
/// g.speed_ratio = std::numeric_limits<double>::infinity(); // no slew
/// @endcode
// ****************************************************************************
struct JointGains
{
    /// Position loop stiffness (N.m/rad or N/m).
    double kp = 150.0;
    /// Position loop damping (N.m.s/rad or N.s/m).
    double kd = 15.0;
    /// Largest reference slew as a fraction of the velocity limit.
    /// Infinity: no slew, the PD loop aims at the target at once (targets
    /// already smooth, e.g. from a learned policy).
    double speed_ratio = 0.5;
    /// Velocity loop gain (N.m.s/rad or N.s/m).
    double velocity_kp = 5.0;
};

// ****************************************************************************
/// All joints of a robot.
///
/// Skills write commands (@ref moveTo, @ref spin, @ref push); the backend
/// runs @ref control, writes the measurements with @ref measure, and the
/// robot calls @ref publish for the view.
///
/// @code
/// robotik::JointSet& joints = robot.joints();
/// robotik::Revolute const elbow = joints.revolute("joint3");
/// joints.moveTo(elbow, Radians(0.8));
/// if (joints.reached(elbow, Radians(0.01))) { ... }
/// robotik::Revolute const wheel = joints.revolute("left_wheel_joint");
/// joints.spin(wheel, AngularVelocity(3.0));
/// @endcode
// ****************************************************************************
class JointSet
{
public:

    // -------------------------------------------------------------------------
    /// Appends the joint carried by a link, reading its type and its limits
    /// from the Compages joint component (filled from the URDF).
    /// @param p_name Joint name, unique in the robot.
    /// @param p_link Link entity carrying the joint component.
    /// @return The id of the new joint.
    /// @throws std::invalid_argument if @p_link has no joint component or the
    ///   name is taken.
    // -------------------------------------------------------------------------
    JointId add(std::string p_name, compages::world::Entity p_link);

    // -------------------------------------------------------------------------
    /// Appends a revolute joint without rendered link (tests, drivers).
    /// @param p_name Joint name, unique in the robot.
    /// @param p_limits Position, speed and torque limits.
    /// @param p_continuous True for a joint without position limits.
    /// @return The handle of the new joint.
    /// @throws std::invalid_argument if the name is taken.
    // -------------------------------------------------------------------------
    Revolute add(std::string p_name,
                 RevoluteLimits const& p_limits,
                 bool p_continuous = false);

    // -------------------------------------------------------------------------
    /// Appends a prismatic joint without rendered link (tests, drivers).
    /// @param p_name Joint name, unique in the robot.
    /// @param p_limits Position, speed and force limits.
    /// @return The handle of the new joint.
    /// @throws std::invalid_argument if the name is taken.
    // -------------------------------------------------------------------------
    Prismatic add(std::string p_name, PrismaticLimits const& p_limits);

    // -------------------------------------------------------------------------
    /// @return The number of joints.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t size() const
    {
        return m_names.size();
    }

    // -------------------------------------------------------------------------
    /// @param p_name Joint name.
    /// @return The joint, or @ref NO_JOINT.
    // -------------------------------------------------------------------------
    [[nodiscard]] JointId find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// @param p_name Joint name.
    /// @return The joint.
    /// @throws std::invalid_argument if unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] JointId require(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Typed handle of a revolute or continuous joint.
    /// @param p_name Joint name.
    /// @return The handle.
    /// @throws std::invalid_argument if unknown or prismatic.
    // -------------------------------------------------------------------------
    [[nodiscard]] Revolute revolute(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Typed handle of a prismatic joint.
    /// @param p_name Joint name.
    /// @return The handle.
    /// @throws std::invalid_argument if unknown or revolute.
    // -------------------------------------------------------------------------
    [[nodiscard]] Prismatic prismatic(std::string_view p_name) const;

    /// @copydoc revolute(std::string_view) const
    /// @param p_id Joint id.
    [[nodiscard]] Revolute revolute(JointId p_id) const;

    /// @copydoc prismatic(std::string_view) const
    /// @param p_id Joint id.
    [[nodiscard]] Prismatic prismatic(JointId p_id) const;

    // --- Description --------------------------------------------------------

    /// @return The name of joint @p_id.
    [[nodiscard]] std::string const& name(JointId p_id) const
    {
        return m_names[p_id];
    }

    /// @return The kind of joint @p_id.
    [[nodiscard]] JointType type(JointId p_id) const
    {
        return m_types[p_id];
    }

    /// @return True when joint @p_id slides (unit m).
    [[nodiscard]] bool isPrismatic(JointId p_id) const
    {
        return m_types[p_id] == JointType::Prismatic;
    }

    // -------------------------------------------------------------------------
    /// @param p_joint Typed handle.
    /// @return The limits of the joint, in its units.
    // -------------------------------------------------------------------------
    template <class Position>
    [[nodiscard]] JointLimits<Position>
    limits(JointHandle<Position> p_joint) const
    {
        Bounds const& bounds = m_bounds[p_joint.id];
        return { Position(bounds.lower),
                 Position(bounds.upper),
                 VelocityOf<Position>(bounds.velocity),
                 EffortOf<Position>(bounds.effort) };
    }

    /// @return The loop tuning of joint @p_id, to change it in place.
    [[nodiscard]] JointGains& gains(JointId p_id)
    {
        return m_gains[p_id];
    }

    /// @return The link entity of joint @p_id (invalid without rendering).
    [[nodiscard]] compages::world::Entity link(JointId p_id) const
    {
        return m_links[p_id];
    }

    // --- Measured state (typed) ---------------------------------------------

    /// @return The measured position of @p_joint.
    template <class Position>
    [[nodiscard]] Position position(JointHandle<Position> p_joint) const
    {
        return Position(m_positions[p_joint.id]);
    }

    /// @return The measured speed of @p_joint.
    template <class Position>
    [[nodiscard]] VelocityOf<Position>
    velocity(JointHandle<Position> p_joint) const
    {
        return VelocityOf<Position>(m_velocities[p_joint.id]);
    }

    /// @return Effort produced by the last @ref control call.
    template <class Position>
    [[nodiscard]] EffortOf<Position> effort(JointHandle<Position> p_joint) const
    {
        return EffortOf<Position>(m_efforts[p_joint.id]);
    }

    // --- Commands (typed) ---------------------------------------------------

    // -------------------------------------------------------------------------
    /// Servos a joint toward a position (position mode). The reference moves
    /// at @ref JointGains::speed_ratio of the speed limit; the effort is
    /// clamped to the limit.
    /// @param p_joint Typed handle.
    /// @param p_position Target position.
    // -------------------------------------------------------------------------
    template <class Position>
    void moveTo(JointHandle<Position> p_joint, Position p_position)
    {
        moveTo(p_joint.id, p_position.value());
    }

    // -------------------------------------------------------------------------
    /// Turns a joint at a speed (velocity mode), e.g. a wheel.
    /// @param p_joint Typed handle.
    /// @param p_velocity Target speed.
    // -------------------------------------------------------------------------
    template <class Position>
    void spin(JointHandle<Position> p_joint, VelocityOf<Position> p_velocity)
    {
        spin(p_joint.id, p_velocity.value());
    }

    // -------------------------------------------------------------------------
    /// Applies a raw effort (effort mode).
    /// @param p_joint Typed handle.
    /// @param p_effort Torque or force; @ref control clamps it to the limit.
    // -------------------------------------------------------------------------
    template <class Position>
    void push(JointHandle<Position> p_joint, EffortOf<Position> p_effort)
    {
        push(p_joint.id, p_effort.value());
    }

    // -------------------------------------------------------------------------
    /// @param p_joint Typed handle.
    /// @param p_tolerance Largest accepted error.
    /// @return True in position mode once |position - target| <= @p_tolerance.
    // -------------------------------------------------------------------------
    template <class Position>
    [[nodiscard]] bool reached(JointHandle<Position> p_joint,
                               Position p_tolerance) const
    {
        return reached(p_joint.id, p_tolerance.value());
    }

    /// @return The home position of @p_joint (used by the @c Home skill).
    template <class Position>
    [[nodiscard]] Position home(JointHandle<Position> p_joint) const
    {
        return Position(m_homes[p_joint.id]);
    }

    /// Sets the home position of @p_joint to @p_position.
    template <class Position>
    void home(JointHandle<Position> p_joint, Position p_position)
    {
        m_homes[p_joint.id] = p_position.value();
    }

    // --- Mixed joints: SI base unit of each joint (rad or m) ----------------

    /// @return Measured positions, indexed by @ref JointId.
    [[nodiscard]] std::span<double const> positions() const
    {
        return m_positions;
    }

    /// @return Measured speeds, indexed by @ref JointId.
    [[nodiscard]] std::span<double const> velocities() const
    {
        return m_velocities;
    }

    /// @return Efforts of the last @ref control, indexed by @ref JointId.
    [[nodiscard]] std::span<double const> efforts() const
    {
        return m_efforts;
    }

    /// @return Measured position of joint @p_id (rad or m).
    [[nodiscard]] double position(JointId p_id) const
    {
        return m_positions[p_id];
    }

    /// @return Measured speed of joint @p_id.
    [[nodiscard]] double velocity(JointId p_id) const
    {
        return m_velocities[p_id];
    }

    /// @return Effort of joint @p_id at the last @ref control.
    [[nodiscard]] double effort(JointId p_id) const
    {
        return m_efforts[p_id];
    }

    /// @return Home position of joint @p_id.
    [[nodiscard]] double home(JointId p_id) const
    {
        return m_homes[p_id];
    }

    /// Sets the home position of joint @p_id to @p_position.
    void home(JointId p_id, double p_position)
    {
        m_homes[p_id] = p_position;
    }

    /// @return How the command of joint @p_id is read.
    [[nodiscard]] JointMode mode(JointId p_id) const
    {
        return m_modes[p_id];
    }

    /// @return Setpoint of the current mode (position, velocity or effort).
    [[nodiscard]] double target(JointId p_id) const
    {
        return m_targets[p_id];
    }

    /// Untyped @ref moveTo(JointHandle, Position): @p_position in rad or m.
    void moveTo(JointId p_id, double p_position);
    /// Untyped @ref spin(JointHandle, VelocityOf): @p_velocity in rad/s or m/s.
    void spin(JointId p_id, double p_velocity);
    /// Untyped @ref push(JointHandle, EffortOf): @p_effort in N.m or N.
    void push(JointId p_id, double p_effort);
    /// Untyped @ref reached(JointHandle, Position) const.
    [[nodiscard]] bool reached(JointId p_id, double p_tolerance) const;

    /// Servos joint @p_id where it is.
    void hold(JointId p_id);
    /// Servos every joint where it is.
    void hold();
    /// Removes any effort on joint @p_id (the joint becomes free).
    void release(JointId p_id);

    // --- Backend side -------------------------------------------------------

    // -------------------------------------------------------------------------
    /// Writes a measurement. Called by the backend after each step.
    /// @param p_id Joint id.
    /// @param p_position Measured position (rad or m).
    /// @param p_velocity Measured speed.
    // -------------------------------------------------------------------------
    void measure(JointId p_id, double p_position, double p_velocity)
    {
        m_positions[p_id] = p_position;
        m_velocities[p_id] = p_velocity;
    }

    // -------------------------------------------------------------------------
    /// Teleports a joint at rest: state, reference and command (reset).
    /// @param p_id Joint id.
    /// @param p_position New position (rad or m).
    // -------------------------------------------------------------------------
    void place(JointId p_id, double p_position);

    // -------------------------------------------------------------------------
    /// Runs the low-level loop of every joint and stores the efforts (see
    /// @ref efforts). Backends call it at their physics rate.
    /// @param p_dt Duration of the physics step.
    // -------------------------------------------------------------------------
    void control(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Copies the measured state into the Compages joint components of the
    /// links (rendering and inspection).
    // -------------------------------------------------------------------------
    void publish() const;

private:

    // -------------------------------------------------------------------------
    /// Limits in SI base units.
    // -------------------------------------------------------------------------
    struct Bounds
    {
        double lower;    ///< Lowest position.
        double upper;    ///< Highest position.
        double velocity; ///< Largest speed.
        double effort;   ///< Largest effort.
    };

    /// Appends a joint to every array; throws if the name is taken.
    JointId append(std::string p_name,
                   JointType p_type,
                   Bounds const& p_bounds,
                   compages::world::Entity p_link);

private:

    // Cold data.
    /// Joint names.
    std::vector<std::string> m_names;
    /// Link entities (invalid without rendering).
    std::vector<compages::world::Entity> m_links;
    /// Loop tuning.
    std::vector<JointGains> m_gains;
    /// Home positions.
    std::vector<double> m_homes;
    // Hot data, walked every physics step.
    /// Joint kinds.
    std::vector<JointType> m_types;
    /// Command modes.
    std::vector<JointMode> m_modes;
    /// Limits.
    std::vector<Bounds> m_bounds;
    /// Measured positions.
    std::vector<double> m_positions;
    /// Measured speeds.
    std::vector<double> m_velocities;
    /// Efforts of the last control.
    std::vector<double> m_efforts;
    /// Setpoints of the current mode.
    std::vector<double> m_targets;
    /// Rate-limited position references moving toward the targets.
    std::vector<double> m_references;
};

} // namespace robotik
