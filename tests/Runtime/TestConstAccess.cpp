// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// What a const object gives is const: a reader cannot command through it.
// Checked by the compiler; the test body only makes the file a test.

#include "main.hpp"

#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Task/RoboticCell.hpp"

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
static_assert(CONST_RESULT<
              decltype(std::declval<robotik::Robot const&>().getKinematics())>);
static_assert(CONST_RESULT<decltype(std::declval<robotik::RobotRuntime const&>()
                                        .getBackend())>);

// The Robot of the skills reaches nothing of the simulator: no ECS world,
// no entity, no backend, no writable kinematics, no base to write. The
// session (the program that runs the robot) has them. A requires clause is
// only soft in a template, hence the concept.
template <typename R>
concept ReachesSimulator = requires(R& p_robot) { p_robot.getWorld(); } ||
                           requires(R& p_robot) { p_robot.getRoot(); } ||
                           requires(R& p_robot) { p_robot.getLink("tcp"); } ||
                           requires(R& p_robot) { p_robot.getBackend(); } ||
                           requires(R& p_robot) { p_robot.getUrdfFrame(); } ||
                           requires(R& p_robot) { p_robot.measureBase({}); };

static_assert(!ReachesSimulator<robotik::Robot>);
static_assert(ReachesSimulator<robotik::RobotRuntime>);
static_assert(
    CONST_RESULT<decltype(std::declval<robotik::Robot&>().getKinematics())>);
static_assert(!CONST_RESULT<decltype(std::declval<robotik::RobotRuntime&>()
                                         .getKinematics())>);

// What is measured or decided outside the program is written by the
// session only: a skill cannot fake a joint reading, a vacuum, a fault, a new
// device or another id.
template <typename R>
concept WritesMeasures =
    requires(R& p_robot) { p_robot.getJoints().measure(0, 0.0, 0.0); } ||
    requires(R& p_robot) { p_robot.getJoints().place(0, 0.0); } ||
    requires(R& p_robot) { p_robot.getJoints().home(0, 0.0); } ||
    requires(R& p_robot) { p_robot.getResources().fail("arm"); } ||
    requires(R& p_robot) { p_robot.getResources().restoreAll(); } ||
    requires(R& p_robot) {
        p_robot.getActuators().template add<robotik::VacuumGripper>("cup");
    } || requires(R& p_robot) { p_robot.setId("arm"); };

static_assert(!WritesMeasures<robotik::Robot>);
static_assert(WritesMeasures<robotik::RobotRuntime>);
static_assert(requires(robotik::RobotRuntime& p_robot,
                       robotik::VacuumGripper& p_cup) {
    p_robot.measureJoint(robotik::Revolute{ 0 },
                         robotik::Radians(0.0),
                         robotik::AngularVelocity(0.0));
    p_robot.measureHolding(p_cup, true);
});

/// True when a cup lets anyone write its measured vacuum.
template <typename C>
concept CupWritesVacuum = requires(C& p_cup) { p_cup.measureHolding(true); };
static_assert(!CupWritesVacuum<robotik::VacuumGripper>);

TEST(ConstAccess, ConstObjectsGiveConstParts)
{
    SUCCEED() << "checked by the static_asserts of this file";
}
