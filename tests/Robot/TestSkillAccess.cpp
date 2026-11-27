// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// What a skill can reach, checked by the compiler. A skill holds a
// robotik::Robot& or a robotik::RobotContext&: it must not change a sensor,
// add a device, command another robot, reach an entity, nor cast its robot
// to the runtime. The runtime keeps all of these. Each line below fails to
// compile, or turns false, when a header opens one of these doors again.

#include "main.hpp"

#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"
#include "RobotIK/Robot/Sensors/Camera.hpp"
#include "RobotIK/Task/RobotContext.hpp"

#include <type_traits>

/// The holder can give a camera another image source, noise seed, or
/// forget its frame.
template <class R>
concept CanChangeCamera = requires(R& p_robot) {
    p_robot.getSensors().template first<robotik::Camera>()->seed(
        robotik::Seed{ 1U });
    p_robot.getSensors().template first<robotik::Camera>()->source(nullptr);
    p_robot.getSensors().template first<robotik::Camera>()->invalidate();
};

/// The holder can add an actuator.
template <class R>
concept CanAddActuator = requires(R& p_robot) {
    p_robot.getActuators().template add<robotik::JointGroup>("arm");
};

/// The holder can reach the ECS world, the drawn entities or the backend.
template <class R>
concept CanReachWorld = requires(R& p_robot) { p_robot.getWorld(); } ||
                        requires(R& p_robot) { p_robot.getRoot(); } ||
                        requires(R& p_robot) { p_robot.getLink("tool0"); } ||
                        requires(R& p_robot) { p_robot.getBackend(); } ||
                        requires(R& p_robot) { p_robot.getTrueBase(); };

/// The joint set gives the entity of a joint.
template <class J>
concept JointsGiveEntity =
    requires(J& p_joints) { p_joints.link(robotik::JointId{ 0 }); };

static_assert(!CanChangeCamera<robotik::Robot>);
static_assert(CanChangeCamera<robotik::RobotRuntime>);

static_assert(!CanAddActuator<robotik::Robot>);
static_assert(!CanAddActuator<robotik::RobotRuntime>);
static_assert(requires(robotik::RobotRuntime& p_runtime) {
    p_runtime.addActuator<robotik::JointGroup>("arm");
});

static_assert(!CanReachWorld<robotik::Robot>);
static_assert(CanReachWorld<robotik::RobotRuntime>);
static_assert(!JointsGiveEntity<robotik::JointSet>);

// A skill reads the other robots of the team, it does not command them.
static_assert(std::is_const_v<std::remove_pointer_t<std::remove_cvref_t<
                  decltype(std::declval<robotik::RobotContext&>().team[0])>>>);

// The runtime owns the robot: there is no cast from one to the other.
static_assert(!std::is_base_of_v<robotik::Robot, robotik::RobotRuntime>);
static_assert(!std::is_convertible_v<robotik::Robot&, robotik::RobotRuntime&>);
static_assert(
    !std::is_constructible_v<robotik::Robot,
                             std::filesystem::path,
                             std::filesystem::path,
                             std::shared_ptr<robotik::ResourceManager>>);

// ----------------------------------------------------------------------------
TEST(SkillAccess, TheCompilerChecksWhatASkillReaches)
{
    SUCCEED() << "checked by the static_assert of this file";
}
