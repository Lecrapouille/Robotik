// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceControl.hpp"

#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#define STEP_M 0.03
#define READY_STEPS 20
#define REWARD_GRASP 2.0f
#define REWARD_SUCCESS 10.0f
#define EXPERT_HOVER_M 0.10
#define EXPERT_CARRY_Z_M 0.22
#define EXPERT_DROP_Z_M 0.12
#define EXPERT_ALIGN_M 0.025
#define EXPERT_TOUCH_M 0.020
#define EXPERT_COMMIT_Z_M 0.06

static robotik::Vector3 clampWorkspace(robotik::Vector3 const& p_point)
{
    return { std::clamp(p_point.x, 0.20, 0.60),
             std::clamp(p_point.y, -0.40, 0.40),
             std::clamp(p_point.z, 0.02, 0.35) };
}

// --- Scripted ready pose before RL episodes -----------------------------------

void readyPosture(robotik::RobotSession& p_robot,
                  robotik::VacuumGripper const& p_gripper,
                  robotik::Vector3 const& p_tip)
{
    robotik::Pose const start = p_robot.framePose(p_robot.tool());
    robotik::Pose const goal{
        p_tip + robotik::Vector3{ 0.0, 0.0, p_gripper.length().value() },
        PICK_PLACE_DOWN
    };
    robotik::Quaternion end = goal.rotation;
    if (start.rotation.w * end.w + start.rotation.x * end.x +
            start.rotation.y * end.y + start.rotation.z * end.z <
        0.0)
    {
        end = { -end.w, -end.x, -end.y, -end.z };
    }
    for (int k = 1; k <= READY_STEPS; ++k)
    {
        double const t = static_cast<double>(k) / READY_STEPS;
        robotik::Quaternion const q{
            (1.0 - t) * start.rotation.w + t * end.w,
            (1.0 - t) * start.rotation.x + t * end.x,
            (1.0 - t) * start.rotation.y + t * end.y,
            (1.0 - t) * start.rotation.z + t * end.z
        };
        robotik::Pose const pose{ start.position * (1.0 - t) +
                                      goal.position * t,
                                  q.normalized() };
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

// --- Observation ----------------------------------------------------------

bool cubeInBox(robotik::Simulation const& p_simulation,
               robotik::VacuumGripper const& p_gripper)
{
    if (p_gripper.holding())
    {
        return false;
    }
    std::optional<double> const inside = p_simulation.query(
        std::string("inside(") + PICK_PLACE_CUBE + ", " + PICK_PLACE_BOX + ")");
    return inside.value_or(0.0) > 0.5;
}

static robotik::VacuumGripper* findGripper(robotik::Simulation& p_simulation)
{
    robotik::VacuumGripper* gripper =
        p_simulation.robot().actuators().first<robotik::VacuumGripper>();
    if (gripper == nullptr)
    {
        throw std::runtime_error("Pick and place learning needs a vacuum gripper");
    }
    return gripper;
}

static robotik::Vector3 believed(robotik::Simulation const& p_simulation, char const* p_name)
{
    robotik::WorldObject const* object = p_simulation.worldModel().find(p_name);
    if (object == nullptr)
    {
        throw std::runtime_error(std::string("The world has no prop '") + p_name + "'");
    }
    return object->position;
}

robotik::Space PickPlaceObservation::space() const
{
    robotik::Space space;
    space.add3("tip", -1.5f, 1.5f, "m")
        .add3("cube", -1.5f, 1.5f, "m")
        .add3("box", -1.5f, 1.5f, "m")
        .add3("cube_from_tip", -3.0f, 3.0f, "m")
        .add("holding", 0.0f, 1.0f)
        .add("suction", 0.0f, 1.0f)
        .add("progress", 0.0f, 1.0f)
        .add("delivered", 0.0f, 1.0f, "", true);
    return space;
}

void PickPlaceObservation::setup(robotik::Simulation& p_simulation)
{
    m_gripper = findGripper(p_simulation);
}

void PickPlaceObservation::reset(robotik::Simulation& p_simulation)
{
    if (p_simulation.oracle())
    {
        p_simulation.observe();
    }
}

void PickPlaceObservation::observe(robotik::Simulation const& p_simulation,
                                   robotik::EpisodeClock const& p_clock,
                                   std::span<float> p_observation)
{
    robotik::Vector3 const tip = m_gripper->tip(p_simulation.robot());
    robotik::Vector3 const cube = believed(p_simulation, PICK_PLACE_CUBE);
    robotik::Vector3 const box = believed(p_simulation, PICK_PLACE_BOX);
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
    *out++ = p_clock.max_steps > 0u ? static_cast<float>(p_clock.steps) /
                                          static_cast<float>(p_clock.max_steps)
                                    : 0.0f;
    *out = cubeInBox(p_simulation, *m_gripper) ? 1.0f : 0.0f;
}

// --- Action ---------------------------------------------------------------

robotik::Space PickPlaceAction::space() const
{
    robotik::Space space;
    // Fractions of STEP_M per control period.
    space.add("move.x", -1.0f, 1.0f).add("move.y", -1.0f, 1.0f).add("move.z", -1.0f, 1.0f);
    space.add("suction", -1.0f, 1.0f);
    return space;
}

void PickPlaceAction::setup(robotik::Simulation& p_simulation)
{
    m_arm = p_simulation.robot().actuators().find<robotik::JointGroup>("arm");
    if (m_arm == nullptr)
    {
        throw std::runtime_error("Pick and place learning needs an 'arm' joint group");
    }
    m_gripper = findGripper(p_simulation);
    // hold() makes the posture the home: every reset starts from it.
    readyPosture(p_simulation.robot(), *m_gripper);
}

void PickPlaceAction::apply(std::span<float const> p_action, robotik::Simulation& p_simulation)
{
    robotik::RobotSession& robot = p_simulation.robot();
    auto axis = [&](std::size_t p_index)
    {
        return STEP_M * std::clamp(static_cast<double>(p_action[p_index]), -1.0, 1.0);
    };
    robotik::Vector3 const target = clampWorkspace(
        m_gripper->tip(robot) + robotik::Vector3{ axis(0), axis(1), axis(2) });
    m_gripper->suction(p_action[3] > 0.0f);

    robotik::Pose const flange{
        target + robotik::Vector3{ 0.0, 0.0, m_gripper->length().value() },
        PICK_PLACE_DOWN
    };
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
}

// --- Reward ---------------------------------------------------------------

void PickPlaceReward::setup(robotik::Simulation const& p_simulation)
{
    m_gripper = p_simulation.robot().actuators().first<robotik::VacuumGripper>();
    if (m_gripper == nullptr)
    {
        throw std::runtime_error("Pick and place learning needs a vacuum gripper");
    }
}

void PickPlaceReward::reset(robotik::Simulation const& /*p_simulation*/)
{
    m_grasped = false;
}

robotik::StepResult PickPlaceReward::evaluate(robotik::Simulation const& p_simulation)
{
    robotik::Vector3 const cube = believed(p_simulation, PICK_PLACE_CUBE);
    robotik::Vector3 const box = believed(p_simulation, PICK_PLACE_BOX);
    robotik::Vector3 const tip = m_gripper->tip(p_simulation.robot());

    robotik::StepResult result;
    if (m_gripper->holding())
    {
        robotik::Vector3 const above{ box.x, box.y, box.z + 0.1 };
        result.reward = 1.0f - static_cast<float>(robotik::norm(cube - above));
        if (!m_grasped)
        {
            m_grasped = true;
            result.reward += REWARD_GRASP;
        }
    }
    else
    {
        robotik::Vector3 const top{ cube.x, cube.y, cube.z + 0.02 };
        result.reward = -static_cast<float>(robotik::norm(tip - top));
    }
    if (cubeInBox(p_simulation, *m_gripper))
    {
        result.reward += REWARD_SUCCESS;
        result.end = robotik::EpisodeEnd::Success;
    }
    return result;
}

robotik::SimulationEnvironment::Parts pickPlaceParts()
{
    robotik::SimulationEnvironment::Parts parts;
    parts.observation = std::make_unique<PickPlaceObservation>();
    parts.action = std::make_unique<PickPlaceAction>();
    parts.reward = std::make_unique<PickPlaceReward>();
    return parts;
}

// --- Expert policy (teacher for PickAndPlaceRL) -------------------------------

void convergedPolicy(std::span<float const> p_observation,
                     std::span<float> p_action)
{
    robotik::Vector3 const tip{ p_observation[0], p_observation[1],
                                p_observation[2] };
    robotik::Vector3 const cube{ p_observation[3], p_observation[4],
                                 p_observation[5] };
    robotik::Vector3 const box{ p_observation[6], p_observation[7],
                                p_observation[8] };
    bool const holding = p_observation[12] > 0.5f;
    bool suction = holding;

    robotik::Vector3 goal;
    if (!holding)
    {
        // Pick: align XY, then descend; latch suction near the cube top.
        double const top = cube.z + 0.02;
        double const xy = std::hypot(cube.x - tip.x, cube.y - tip.y);
        // Commit to the descent once low enough: IK leftover must not send
        // the cup back to hover (that looks like a non-converging wiggle).
        bool const aligned =
            xy < EXPERT_ALIGN_M || tip.z < top + EXPERT_COMMIT_Z_M;
        if (!aligned)
        {
            goal = { cube.x, cube.y, top + EXPERT_HOVER_M };
        }
        else
        {
            goal = { cube.x, cube.y, top };
            suction = tip.z < top + 0.04;
        }
    }
    else if (std::hypot(box.x - tip.x, box.y - tip.y) > EXPERT_ALIGN_M)
    {
        // Carry: stay high while moving toward the box footprint.
        goal = tip.z < EXPERT_CARRY_Z_M - 0.02
                   ? robotik::Vector3{ tip.x, tip.y, EXPERT_CARRY_Z_M }
                   : robotik::Vector3{ box.x, box.y, EXPERT_CARRY_Z_M };
    }
    else
    {
        // Place: lower over the cavity centre, then release suction.
        goal = { box.x, box.y, EXPERT_DROP_Z_M };
        suction = tip.z > EXPERT_DROP_Z_M + 0.015;
    }
    robotik::Vector3 const move = (goal - tip) * (1.0 / STEP_M);
    p_action[0] = static_cast<float>(std::clamp(move.x, -1.0, 1.0));
    p_action[1] = static_cast<float>(std::clamp(move.y, -1.0, 1.0));
    p_action[2] = static_cast<float>(std::clamp(move.z, -1.0, 1.0));
    p_action[3] = suction ? 1.0f : -1.0f;
}

void pickPlacePolicy(std::span<float const> p_observation,
                     std::span<float> p_action,
                     float p_mix,
                     robotik::Random* p_noise)
{
    float const mix = std::clamp(p_mix, 0.0f, 1.0f);
    if (mix >= 1.0f || p_noise == nullptr)
    {
        convergedPolicy(p_observation, p_action);
        return;
    }
    if (mix <= 0.0f)
    {
        for (float& value : p_action)
        {
            value = static_cast<float>(p_noise->uniform(-1.0, 1.0));
        }
        return;
    }
    std::array<float, PICK_PLACE_ACTIONS> teacher{};
    convergedPolicy(p_observation, teacher);
    for (std::size_t i = 0; i < p_action.size(); ++i)
    {
        float const noise = static_cast<float>(p_noise->uniform(-1.0, 1.0));
        p_action[i] = std::clamp((1.0f - mix) * noise + mix * teacher[i], -1.0f, 1.0f);
    }
}

char const* policyPhase(std::span<float const> p_observation)
{
    if (p_observation.size() < PICK_PLACE_OBSERVATIONS)
    {
        return "idle";
    }
    if (p_observation[15] > 0.5f)
    {
        return "delivered";
    }
    robotik::Vector3 const tip{ p_observation[0], p_observation[1],
                                p_observation[2] };
    robotik::Vector3 const cube{ p_observation[3], p_observation[4],
                                 p_observation[5] };
    robotik::Vector3 const box{ p_observation[6], p_observation[7],
                                p_observation[8] };
    bool const holding = p_observation[12] > 0.5f;
    if (!holding)
    {
        return std::hypot(cube.x - tip.x, cube.y - tip.y) > EXPERT_ALIGN_M &&
                       tip.z >= cube.z + 0.02 + EXPERT_COMMIT_Z_M
                   ? "approach cube"
                   : "descend / grasp";
    }
    if (std::hypot(box.x - tip.x, box.y - tip.y) > EXPERT_ALIGN_M)
    {
        return "carry to box";
    }
    return "drop in box";
}
