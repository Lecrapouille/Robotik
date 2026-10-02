//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
//! @file JointComponents.hpp
//! @brief Joint identity, state, commands, and limits on link entities.
//! Revolute and prismatic joints use distinct unit types; ECS stores a @ref
//! JointState variant. Backends and PD loops use @ref positionSi helpers.
//=============================================================================

#pragma once

#include "Compages/Core/Units.hpp"

#include <cmath>
#include <string>
#include <variant>

namespace robotik::ecs
{

// ****************************************************************************
//! @brief URDF joint category (revolute vs prismatic actuated joints).
// ****************************************************************************
enum class JointMechanism
{
    Revolute,
    Prismatic,
};

// ****************************************************************************
//! @brief How @ref JointCommand is interpreted by @ref ControllerSystem.
// ****************************************************************************
enum class JointControlMode
{
    DISABLED,
    POSITION,
    VELOCITY,
    EFFORT
};

// ****************************************************************************
//! @brief URDF joint name and mechanism on a link entity.
// ****************************************************************************
struct Joint
{
    std::string name;
    JointMechanism mechanism = JointMechanism::Revolute;
};

// ****************************************************************************
//! @brief Measured joint state with SI-typed fields.
// ****************************************************************************
template <typename Position, typename Velocity, typename Effort>
struct BasicJointState
{
    Position position{};
    Velocity velocity{};
    Effort effort{};
};

using RevoluteJointState = BasicJointState<Radians, AngularVelocity, Torque>;
using PrismaticJointState = BasicJointState<Length, LinearVelocity, Force>;

using JointState = std::variant<RevoluteJointState, PrismaticJointState>;

// ****************************************************************************
//! @brief High-level setpoint written by skills.
// ****************************************************************************
template <typename Position, typename Velocity, typename Effort>
struct BasicJointCommand
{
    JointControlMode mode = JointControlMode::DISABLED;
    Position position{};
    Velocity velocity{};
    Effort effort{};
};

using RevoluteJointCommand =
    BasicJointCommand<Radians, AngularVelocity, Torque>;
using PrismaticJointCommand = BasicJointCommand<Length, LinearVelocity, Force>;

using JointCommand = std::variant<RevoluteJointCommand, PrismaticJointCommand>;

// ****************************************************************************
//! @brief URDF limit block copied at load time.
// ****************************************************************************
template <typename Position, typename Velocity, typename Effort>
struct BasicJointLimits
{
    Position lower{};
    Position upper{};
    Velocity max_velocity{};
    Effort max_effort{};
};

using RevoluteJointLimits = BasicJointLimits<Radians, AngularVelocity, Torque>;
using PrismaticJointLimits = BasicJointLimits<Length, LinearVelocity, Force>;

using JointLimits = std::variant<RevoluteJointLimits, PrismaticJointLimits>;

// ****************************************************************************
//! @brief Default posture for @ref HomeSkill and @ref RobotRuntime::hold.
// ****************************************************************************
template <typename Position>
struct BasicHomePosition
{
    Position position{};
};

using RevoluteHomePosition = BasicHomePosition<Radians>;
using PrismaticHomePosition = BasicHomePosition<Length>;

using HomePosition = std::variant<RevoluteHomePosition, PrismaticHomePosition>;

// -------------------------------------------------------------------------
//! @brief |state - goal| > tolerance for a homogeneous unit type.
// -------------------------------------------------------------------------
template <typename Unit>
inline bool exceedsTolerance(Unit const& p_state,
                             Unit const& p_goal,
                             Unit const& p_tolerance)
{
    return std::abs((p_state - p_goal).value()) > p_tolerance.value();
}

inline JointState makeJointState(JointMechanism p_mechanism)
{
    if (p_mechanism == JointMechanism::Prismatic)
    {
        return PrismaticJointState{};
    }
    return RevoluteJointState{};
}

inline JointCommand makeJointCommand(JointMechanism p_mechanism)
{
    if (p_mechanism == JointMechanism::Prismatic)
    {
        return PrismaticJointCommand{};
    }
    return RevoluteJointCommand{};
}

inline HomePosition makeHomePosition(JointMechanism p_mechanism,
                                     double p_position_si = 0.0)
{
    if (p_mechanism == JointMechanism::Prismatic)
    {
        return PrismaticHomePosition{ Length(p_position_si) };
    }
    return RevoluteHomePosition{ Radians(p_position_si) };
}

inline double positionSi(JointState const& p_state)
{
    return std::visit([](auto const& joint) { return joint.position.value(); },
                      p_state);
}

inline void setPosition(JointState& p_state, double p_si)
{
    std::visit([p_si](auto& joint)
               { joint.position = decltype(joint.position)(p_si); },
               p_state);
}

inline double velocitySi(JointState const& p_state)
{
    return std::visit([](auto const& joint) { return joint.velocity.value(); },
                      p_state);
}

inline void setVelocity(JointState& p_state, double p_si)
{
    std::visit([p_si](auto& joint)
               { joint.velocity = decltype(joint.velocity)(p_si); },
               p_state);
}

inline double commandPositionSi(JointCommand const& p_command)
{
    return std::visit([](auto const& command)
                      { return command.position.value(); },
                      p_command);
}

inline void setCommandPosition(JointCommand& p_command, double p_si)
{
    std::visit([p_si](auto& command)
               { command.position = decltype(command.position)(p_si); },
               p_command);
}

inline void setCommandMode(JointCommand& p_command, JointControlMode p_mode)
{
    std::visit([p_mode](auto& command) { command.mode = p_mode; }, p_command);
}

inline JointControlMode commandMode(JointCommand const& p_command)
{
    return std::visit([](auto const& command) { return command.mode; },
                      p_command);
}

inline double homePositionSi(HomePosition const& p_home)
{
    return std::visit([](auto const& home) { return home.position.value(); },
                      p_home);
}

inline void setHomePosition(HomePosition& p_home, double p_si)
{
    std::visit([p_si](auto& home)
               { home.position = decltype(home.position)(p_si); },
               p_home);
}

inline double limitLowerSi(JointLimits const& p_limits)
{
    return std::visit([](auto const& limits) { return limits.lower.value(); },
                      p_limits);
}

inline double limitUpperSi(JointLimits const& p_limits)
{
    return std::visit([](auto const& limits) { return limits.upper.value(); },
                      p_limits);
}

inline double limitMaxVelocitySi(JointLimits const& p_limits)
{
    return std::visit([](auto const& limits)
                      { return limits.max_velocity.value(); },
                      p_limits);
}

inline double limitMaxEffortSi(JointLimits const& p_limits)
{
    return std::visit(
        [](auto const& limits) { return limits.max_effort.value(); }, p_limits);
}

inline JointLimits makeJointLimits(JointMechanism p_mechanism,
                                   double p_lower,
                                   double p_upper,
                                   double p_max_velocity,
                                   double p_max_effort)
{
    if (p_mechanism == JointMechanism::Prismatic)
    {
        return PrismaticJointLimits{ Length(p_lower),
                                     Length(p_upper),
                                     LinearVelocity(p_max_velocity),
                                     Force(p_max_effort) };
    }
    return RevoluteJointLimits{ Radians(p_lower),
                                Radians(p_upper),
                                AngularVelocity(p_max_velocity),
                                Torque(p_max_effort) };
}

} // namespace robotik::ecs
