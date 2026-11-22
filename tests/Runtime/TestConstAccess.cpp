// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// What a const object gives is const: a reader cannot command through it.
// Checked by the compiler; the test body only makes the file a test.

#include "main.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Task/RoboticCell.hpp"

#include <type_traits>
#include <utility>

/// True when the expression type, references and pointers removed, is const.
template <typename T>
inline constexpr bool CONST_RESULT =
    std::is_const_v<std::remove_pointer_t<std::remove_reference_t<T>>>;

using Actuators = robotik::ActuatorSet;

static_assert(CONST_RESULT<decltype(std::declval<Actuators const&>()[0])>);
static_assert(
    CONST_RESULT<decltype(std::declval<Actuators const&>().find("cup"))>);
static_assert(CONST_RESULT<decltype(std::declval<Actuators const&>()
                                        .find<robotik::VacuumGripper>("cup"))>);
static_assert(CONST_RESULT<decltype(std::declval<Actuators const&>()
                                        .first<robotik::VacuumGripper>())>);
static_assert(!CONST_RESULT<decltype(std::declval<Actuators&>()
                                         .first<robotik::VacuumGripper>())>);
static_assert(CONST_RESULT<
              decltype(std::declval<robotik::RoboticCell const&>().planner())>);
static_assert(
    CONST_RESULT<decltype(std::declval<robotik::Robot const&>().kinematics())>);
static_assert(CONST_RESULT<decltype(std::declval<robotik::RobotSession const&>()
                                        .backend())>);

// The Robot of the skills reaches nothing of the simulator: no ECS world,
// no entity, no backend, no writable kinematics, no base to write. The
// session (the program that runs the robot) has them. A requires clause is
// only soft in a template, hence the concept.
template <typename R>
concept ReachesSimulator = requires(R& p_robot) { p_robot.world(); } ||
                           requires(R& p_robot) { p_robot.root(); } ||
                           requires(R& p_robot) { p_robot.link("tcp"); } ||
                           requires(R& p_robot) { p_robot.backend(); } ||
                           requires(R& p_robot) { p_robot.urdfFrame(); } ||
                           requires(R& p_robot) { p_robot.measureBase({}); };

static_assert(!ReachesSimulator<robotik::Robot>);
static_assert(ReachesSimulator<robotik::RobotSession>);
static_assert(
    CONST_RESULT<decltype(std::declval<robotik::Robot&>().kinematics())>);
static_assert(!CONST_RESULT<
              decltype(std::declval<robotik::RobotSession&>().kinematics())>);

TEST(ConstAccess, ConstObjectsGiveConstParts)
{
    SUCCEED() << "checked by the static_asserts of this file";
}
