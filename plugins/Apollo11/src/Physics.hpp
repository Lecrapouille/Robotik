// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Physics.hpp
/// The Earth, the Moon and the laws a spacecraft follows between them: the
/// gravity of both, the air of the Earth, the orbit around one body, and a
/// small propagator the guidance uses to predict where a vehicle goes.
///
/// Frame: geocentric, not rotating, Z along the axis of the Earth. The Moon
/// turns on a circle in the XY plane; the whole mission flies in that plane
/// (no inclination). Earth-centred is not inertial: the gravity adds the
/// pull of the Moon on the Earth (the "indirect" term), which makes it the
/// restricted three-body problem.
///
/// @par Contract
/// SI units. Pure functions, no state but the constants: the guidance may
/// call them as often as it needs, the same inputs give the same outputs.
#pragma once

#include "Robotik/Common/Math.hpp"

#include <functional>
#include <optional>

namespace apollo
{

using robotik::Quaternion;
using robotik::Vector3;

/// Standard gravity, to turn a specific impulse into an exhaust speed.
constexpr double G0 = 9.80665;

// ****************************************************************************
/// A body with gravity: the Earth or the Moon.
// ****************************************************************************
struct Body
{
    double mu = 0.0;            ///< Gravitational parameter (m³/s²).
    double radius = 0.0;        ///< Mean radius (m).
    double rotation = 0.0;      ///< Spin rate about Z (rad/s).
    double density = 0.0;       ///< Air density at the ground (kg/m³), 0: none.
    double pressure = 0.0;      ///< Air pressure at the ground (Pa).
    double scale_height = 1.0;  ///< Height where the air is e times thinner (m).
};

/// Which body.
enum class Place
{
    Earth,
    Moon
};

// ****************************************************************************
/// The Earth at the origin and the Moon on its circular orbit.
// ****************************************************************************
struct Environment
{
    Body earth;
    Body moon;
    double moon_distance = 384.4e6; ///< Radius of the orbit of the Moon (m).
    double moon_phase = 0.0;        ///< Angle of the Moon at t = 0 (rad).

    /// @return Angular rate of the Moon on its orbit (rad/s).
    [[nodiscard]] double moonRate() const;

    /// @return Position of the Moon at time @p_t (s).
    [[nodiscard]] Vector3 moonPosition(double p_t) const;

    /// @return Velocity of the Moon at time @p_t (s).
    [[nodiscard]] Vector3 moonVelocity(double p_t) const;

    /// @return The body @p_place.
    [[nodiscard]] Body const& body(Place p_place) const
    {
        return p_place == Place::Earth ? earth : moon;
    }

    /// @return Position of the centre of @p_place at time @p_t.
    [[nodiscard]] Vector3 centre(Place p_place, double p_t) const;

    /// @return Velocity of the centre of @p_place at time @p_t.
    [[nodiscard]] Vector3 centreVelocity(Place p_place, double p_t) const;

    /// @return Turn of the frame fixed to @p_place at time @p_t: the Moon
    ///   shows the Earth always the same face (its X axis).
    [[nodiscard]] Quaternion spin(Place p_place, double p_t) const;

    /// @return Velocity of the ground (or the air) of @p_place at the
    ///   absolute position @p_r, time @p_t.
    [[nodiscard]] Vector3 groundVelocity(Place p_place,
                                         Vector3 const& p_r,
                                         double p_t) const;

    /// @return Acceleration of gravity at @p_r and time @p_t.
    [[nodiscard]] Vector3 gravity(Vector3 const& p_r, double p_t) const;

    /// @return The body whose sphere of influence holds @p_r.
    [[nodiscard]] Place dominant(Vector3 const& p_r, double p_t) const;

    /// @return Air density at @p_altitude above the Earth (kg/m³).
    [[nodiscard]] double density(double p_altitude) const;

    /// @return Air pressure at @p_altitude above the Earth (Pa).
    [[nodiscard]] double pressure(double p_altitude) const;
};

// ****************************************************************************
/// Position, velocity and mass of a vehicle seen as a point.
// ****************************************************************************
struct Craft
{
    Vector3 r = Vector3::ZERO; ///< Position (m).
    Vector3 v = Vector3::ZERO; ///< Velocity (m/s).
    double m = 1.0;            ///< Mass (kg).
};

// ****************************************************************************
/// The conic a vehicle follows around one body, from its position and
/// velocity relative to that body.
// ****************************************************************************
struct Orbit
{
    Place around = Place::Earth;
    double periapsis = 0.0;   ///< Closest distance to the centre (m).
    double apoapsis = 0.0;    ///< Farthest distance, infinite when open (m).
    double eccentricity = 0.0;
    double semi_major = 0.0;  ///< Negative for a hyperbola (m).
    double radius = 0.0;      ///< Current distance to the centre (m).
    double radial = 0.0;      ///< Speed away from the centre (m/s).
    double spin = 0.0;        ///< Z of the angular momentum: > 0 prograde.
    double period = 0.0;      ///< Period, 0 when open (s).

    [[nodiscard]] bool closed() const
    {
        return eccentricity < 1.0;
    }
};

/// @return The conic of a relative state @p_r, @p_v around a body of
///   gravitational parameter @p_mu.
[[nodiscard]] Orbit conic(Vector3 const& p_r,
                          Vector3 const& p_v,
                          double p_mu,
                          Place p_around);

/// @return The conic of @p_craft around @p_place at time @p_t.
[[nodiscard]] Orbit orbitOf(Environment const& p_environment,
                            Craft const& p_craft,
                            Place p_place,
                            double p_t);

/// @return Speed on a circular orbit of radius @p_r (m/s).
[[nodiscard]] double circularSpeed(double p_mu, double p_r);

// ****************************************************************************
/// Thrust of a burn the propagator simulates: a force along a direction the
/// law gives at each step, until the stop condition.
// ****************************************************************************
struct Burn
{
    double thrust = 0.0; ///< Force (N).
    double flow = 0.0;   ///< Mass flow (kg/s).
    /// Direction of the thrust for a state (unit vector).
    std::function<Vector3(double, Craft const&)> direction;
    /// True when the burn must stop.
    std::function<bool(double, Craft const&)> done;
    double limit = 3000.0; ///< Longest burn (s).
};

// ****************************************************************************
/// Predicts the motion of a point vehicle, as the guidance computer of the
/// vehicle would: same gravity as the truth, no air, perfect pointing.
// ****************************************************************************
class Propagator
{
public:

    explicit Propagator(Environment const& p_environment)
        : m_environment(p_environment)
    {
    }

    /// One Runge-Kutta 4 step, @p_extra an acceleration constant on it.
    void step(Craft& p_craft,
              double p_t,
              double p_h,
              Vector3 const& p_extra = Vector3::ZERO) const;

    /// @return A step suited to the state: small near a body (s).
    [[nodiscard]] double coastStep(Craft const& p_craft, double p_t) const;

    // -------------------------------------------------------------------------
    /// Coasts from @p_t to @p_until.
    /// @return The state at @p_until.
    // -------------------------------------------------------------------------
    [[nodiscard]] Craft coast(Craft p_craft, double p_t, double p_until) const;

    // -------------------------------------------------------------------------
    /// Fires @p_burn from @p_t. @p_t becomes the end of the burn.
    /// @return The state at the end of the burn.
    // -------------------------------------------------------------------------
    [[nodiscard]] Craft burn(Craft p_craft, double& p_t, Burn const& p_burn) const;

    // -------------------------------------------------------------------------
    /// Closest point to a body on the coast from @p_t, searched until
    /// @p_until or until the vehicle goes below its ground.
    // -------------------------------------------------------------------------
    struct Approach
    {
        double time = 0.0;       ///< When (s).
        double distance = 0.0;   ///< Distance to the centre (m).
        double spin = 0.0;       ///< Z of r x v relative to the body.
        Craft craft;             ///< State then.
        bool found = false;      ///< False: still going away at @p_until.
    };
    [[nodiscard]] Approach closest(Craft p_craft,
                                   double p_t,
                                   double p_until,
                                   Place p_place) const;

private:

    Environment const& m_environment;
};

// ****************************************************************************
/// Lambert's problem: the velocities that join two positions in a given
/// time around one body (universal variables, Curtis algorithm 5.2).
// ****************************************************************************
struct LambertSolution
{
    Vector3 v1 = Vector3::ZERO; ///< Velocity leaving @c r1.
    Vector3 v2 = Vector3::ZERO; ///< Velocity arriving at @c r2.
};

/// @param p_prograde Turn in the +Z sense (else -Z).
/// @return The solution, or nothing when it does not converge.
[[nodiscard]] std::optional<LambertSolution> lambert(Vector3 const& p_r1,
                                                     Vector3 const& p_r2,
                                                     double p_time,
                                                     double p_mu,
                                                     bool p_prograde);

/// @return @p_v with a unit length (or zero).
[[nodiscard]] Vector3 unit(Vector3 const& p_v);

/// @return Rotation turning @p_from onto @p_to (unit vectors).
[[nodiscard]] Quaternion turn(Vector3 const& p_from, Vector3 const& p_to);

} // namespace apollo
