// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceEnvironment.hpp"

#include "Robotik/Actuators/Actuator.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include <algorithm>
#include <cmath>

#define STEP_M 0.03
#define CONTROL_DT_S 0.01
#define CONTROL_STEPS 10
#define REWARD_GRASP 2.0f
#define REWARD_SUCCESS 10.0f
#define EXPERT_HOVER_M 0.10
#define EXPERT_CARRY_Z_M 0.20
#define EXPERT_DROP_Z_M 0.13
#define EXPERT_ALIGN_M 0.008
#define EXPERT_TOUCH_M 0.006
#define READY_STEPS 20

static char const* const CUBE = "red_cube";
static char const* const BOX = "box";
// Episodes start with the suction cup pointing down above the table. Down
// is Ry(pi): the natural wrist of this arm (joints 4 and 6 near zero, far
// from the limits that trap the IK).
static robotik::Vector3 const READY{ 0.40, 0.0, 0.25 };
static robotik::Quaternion const DOWN{ 0.0, 0.0, 1.0, 0.0 };

// Moves the start posture of the robot to the suction cup at @p_tip pointing
// down. The IK does not reach it from any posture in one go: it follows
// small steps from the current flange pose.
static void ready(robotik::RobotSession& p_robot, robotik::VacuumGripper const& p_gripper, robotik::Vector3 const& p_tip)
{
    robotik::Pose const start = p_robot.framePose(p_robot.tool());
    robotik::Pose const goal{ p_tip + robotik::Vector3{ 0.0, 0.0, p_gripper.length().value() }, DOWN };
    robotik::Quaternion end = goal.rotation;
    if (start.rotation.w * end.w + start.rotation.x * end.x + start.rotation.y * end.y + start.rotation.z * end.z < 0.0)
    {
        end = { -end.w, -end.x, -end.y, -end.z };
    }
    for (int k = 1; k <= READY_STEPS; ++k)
    {
        double const t = static_cast<double>(k) / READY_STEPS;
        robotik::Quaternion const q{ (1.0 - t) * start.rotation.w + t * end.w,
                                     (1.0 - t) * start.rotation.x + t * end.x,
                                     (1.0 - t) * start.rotation.y + t * end.y,
                                     (1.0 - t) * start.rotation.z + t * end.z };
        robotik::Pose const pose{ start.position * (1.0 - t) + goal.position * t, q.normalized() };
        auto solution = p_robot.inverseKinematics(p_robot.tool(), pose);
        if (!solution)
        {
            throw std::runtime_error("No IK solution toward the ready posture");
        }
        robotik::JointPosture posture;
        for (robotik::JointId id = 0; id < p_robot.joints().size(); ++id)
        {
            posture[p_robot.joints().name(id)] = (*solution)[id];
        }
        p_robot.hold(posture);
    }
}

// Reachable part of the table.
static robotik::Vector3 clampWorkspace(robotik::Vector3 const& p_point)
{
    return { std::clamp(p_point.x, 0.20, 0.60),
             std::clamp(p_point.y, -0.40, 0.40),
             std::clamp(p_point.z, 0.02, 0.35) };
}

PickPlaceEnvironment::PickPlaceEnvironment(std::filesystem::path const& p_scenario,
                                           double p_spread,
                                           std::uint32_t p_max_steps)
    : m_max_steps(p_max_steps)
{
    robotik::Scenario scenario = robotik::Scenario::load(p_scenario);
    scenario.behavior_tree.clear();
    scenario.faults.clear();
    scenario.random_faults.clear();
    for (auto& object : scenario.objects)
    {
        if (object.shape.name == CUBE)
        {
            object.randomize[0][0] = -p_spread;
            object.randomize[0][1] = p_spread;
            object.randomize[1][0] = -p_spread;
            object.randomize[1][1] = p_spread;
        }
    }
    m_simulation = std::make_unique<robotik::Simulation>(m_world, std::move(scenario));
    m_arm = m_simulation->robot().actuators().find<robotik::JointGroup>("arm");
    m_gripper = m_simulation->robot().actuators().first<robotik::VacuumGripper>();
    if (m_arm == nullptr || m_gripper == nullptr)
    {
        throw std::runtime_error("The scenario needs an 'arm' joint group and a vacuum gripper");
    }
    ready(m_simulation->robot(), *m_gripper, READY);
}

PickPlaceEnvironment::~PickPlaceEnvironment() = default;

void PickPlaceEnvironment::reset(robotik::Seed p_seed, std::span<float> p_observation)
{
    m_simulation->reset(p_seed);
    m_steps = 0;
    m_grasped = false;
    observe(p_observation);
}

robotik::StepResult PickPlaceEnvironment::step(std::span<float const> p_action, std::span<float> p_observation)
{
    robotik::RobotSession& robot = m_simulation->robot();
    auto axis = [&](std::size_t p_index)
    { return STEP_M * std::clamp(static_cast<double>(p_action[p_index]), -1.0, 1.0); };
    // Relative to the measured tip: a target integrated on its own would run
    // away from an arm that lags behind it.
    robotik::Vector3 const target =
        clampWorkspace(m_gripper->tip(robot) + robotik::Vector3{ axis(0), axis(1), axis(2) });
    m_gripper->suction(p_action[3] > 0.0f);

    // Suction cup pointing down: the flange stays above the tip.
    robotik::Pose const flange{ target + robotik::Vector3{ 0.0, 0.0, m_gripper->length().value() },
                                DOWN };
    if (auto solution = robot.inverseKinematics(robot.tool(), flange))
    {
        std::vector<double> targets;
        targets.reserve(m_arm->joints().size());
        for (robotik::JointId id : m_arm->joints())
        {
            targets.push_back((*solution)[id]);
        }
        m_arm->moveTo(targets);
    }
    for (int i = 0; i < CONTROL_STEPS; ++i)
    {
        m_simulation->step(Seconds(CONTROL_DT_S));
    }
    ++m_steps;
    observe(p_observation);

    robotik::WorldModel const& beliefs = m_simulation->worldModel();
    robotik::Vector3 const cube = beliefs.find(CUBE)->position;
    robotik::Vector3 const box = beliefs.find(BOX)->position;
    robotik::Vector3 const tip = m_gripper->tip(robot);

    robotik::StepResult result;
    if (m_gripper->holding())
    {
        robotik::Vector3 const above{ box.x, box.y, box.z + 0.1 };
        result.reward = 1.0f - static_cast<float>((cube - above).norm());
        if (!m_grasped)
        {
            m_grasped = true;
            result.reward += REWARD_GRASP;
        }
    }
    else
    {
        robotik::Vector3 const top{ cube.x, cube.y, cube.z + 0.02 };
        result.reward = -static_cast<float>((tip - top).norm());
    }
    if (delivered())
    {
        result.reward += REWARD_SUCCESS;
        result.terminated = true;
    }
    result.truncated = !result.terminated && m_steps >= m_max_steps;
    return result;
}

bool PickPlaceEnvironment::delivered() const
{
    if (m_gripper->holding())
    {
        return false;
    }
    robotik::WorldModel const& beliefs = m_simulation->worldModel();
    robotik::WorldObject const* cube = beliefs.find(CUBE);
    robotik::WorldObject const* box = beliefs.find(BOX);
    return std::abs(cube->position.x - box->position.x) < 0.5 * box->size.x &&
           std::abs(cube->position.y - box->position.y) < 0.5 * box->size.y &&
           cube->position.z < box->position.z + 0.5 * box->size.z;
}

void PickPlaceEnvironment::observe(std::span<float> p_observation) const
{
    robotik::WorldModel const& beliefs = m_simulation->worldModel();
    robotik::Vector3 const tip = m_gripper->tip(m_simulation->robot());
    robotik::Vector3 const cube = beliefs.find(CUBE)->position;
    robotik::Vector3 const box = beliefs.find(BOX)->position;
    robotik::Vector3 const relative = cube - tip;
    float* out = p_observation.data();
    for (robotik::Vector3 const& v : { tip, cube, box, relative })
    {
        *out++ = static_cast<float>(v.x);
        *out++ = static_cast<float>(v.y);
        *out++ = static_cast<float>(v.z);
    }
    *out++ = m_gripper->holding() ? 1.0f : 0.0f;
    *out++ = m_gripper->suction() ? 1.0f : 0.0f;
    *out++ = static_cast<float>(m_steps) / static_cast<float>(m_max_steps);
    *out = delivered() ? 1.0f : 0.0f;
}

void expertPolicy(std::span<float const> p_observation, std::span<float> p_action)
{
    robotik::Vector3 const tip{ p_observation[0], p_observation[1], p_observation[2] };
    robotik::Vector3 const cube{ p_observation[3], p_observation[4], p_observation[5] };
    robotik::Vector3 const box{ p_observation[6], p_observation[7], p_observation[8] };
    bool const holding = p_observation[12] > 0.5f;
    bool suction = holding;

    robotik::Vector3 goal;
    if (!holding)
    {
        double const top = cube.z + 0.02;
        if (std::hypot(cube.x - tip.x, cube.y - tip.y) > EXPERT_ALIGN_M)
        {
            goal = { cube.x, cube.y, top + EXPERT_HOVER_M };
        }
        else
        {
            goal = { cube.x, cube.y, top };
            suction = tip.z - top < EXPERT_TOUCH_M;
        }
    }
    else if (std::hypot(box.x - tip.x, box.y - tip.y) > EXPERT_ALIGN_M)
    {
        goal = tip.z < EXPERT_CARRY_Z_M - 0.03 && std::hypot(box.x - tip.x, box.y - tip.y) > 0.1
                   ? robotik::Vector3{ tip.x, tip.y, EXPERT_CARRY_Z_M }
                   : robotik::Vector3{ box.x, box.y, EXPERT_CARRY_Z_M };
    }
    else
    {
        goal = { box.x, box.y, EXPERT_DROP_Z_M };
        suction = tip.z > EXPERT_DROP_Z_M + EXPERT_TOUCH_M;
    }
    robotik::Vector3 const move = (goal - tip) * (1.0 / STEP_M);
    p_action[0] = static_cast<float>(std::clamp(move.x, -1.0, 1.0));
    p_action[1] = static_cast<float>(std::clamp(move.y, -1.0, 1.0));
    p_action[2] = static_cast<float>(std::clamp(move.z, -1.0, 1.0));
    p_action[3] = suction ? 1.0f : -1.0f;
}
