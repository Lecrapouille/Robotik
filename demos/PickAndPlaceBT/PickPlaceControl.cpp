// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceControl.hpp"

#include "Robotik/Scene/ContainerBounds.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/Robot/Actuators.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string_view>

#define STEP_M 0.03
#define CONTROL_DT_S 0.01
#define CONTROL_STEPS 10
#define READY_STEPS 20
#define EXPERT_HOVER_M 0.10
#define EXPERT_CARRY_Z_M 0.22
#define EXPERT_DROP_Z_M 0.12
#define EXPERT_ALIGN_M 0.025
#define EXPERT_TOUCH_M 0.020
#define EXPERT_COMMIT_Z_M 0.06

// --- Ground-truth layout (same frame as scenario objects) ---------------------

static robotik::Vector3 objectCenter(robotik::Simulation const& p_simulation,
                                     std::string_view p_name)
{
    robotik::Vector3 at{};
    p_simulation.robot().world().each<robotik::ecs::SceneObject>(
        [&](compages::world::Entity p_entity,
            robotik::ecs::SceneObject const& p_object)
        {
            if (p_object.name == p_name)
            {
                auto const p = p_entity.position();
                at = { p.x, p.y, p.z };
            }
        });
    return at;
}

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

// --- Low-level RL action: TCP delta + suction, then physics steps -------------

void applyPickPlaceAction(robotik::Simulation& p_simulation,
                          robotik::JointGroup& p_arm,
                          robotik::VacuumGripper& p_gripper,
                          std::span<float const> p_action)
{
    robotik::RobotSession& robot = p_simulation.robot();
    auto axis = [&](std::size_t p_index)
    {
        return STEP_M *
               std::clamp(static_cast<double>(p_action[p_index]), -1.0, 1.0);
    };
    robotik::Vector3 const target = clampWorkspace(
        p_gripper.tip(robot) + robotik::Vector3{ axis(0), axis(1), axis(2) });
    p_gripper.suction(p_action[3] > 0.0f);

    robotik::Pose const flange{
        target + robotik::Vector3{ 0.0, 0.0, p_gripper.length().value() },
        PICK_PLACE_DOWN
    };
    if (auto solution = robot.inverseKinematics(robot.tool(), flange))
    {
        std::vector<double> targets;
        targets.reserve(p_arm.joints().size());
        for (robotik::JointId id : p_arm.joints())
        {
            targets.push_back((*solution)[id]);
        }
        p_arm.moveTo(targets);
    }
    for (int i = 0; i < CONTROL_STEPS; ++i)
    {
        p_simulation.step(Seconds(CONTROL_DT_S));
    }
}

// --- Reward / observation helpers (mirror scenario inside(box) semantics) ---

bool cubeInBox(robotik::Simulation const& p_simulation,
               robotik::VacuumGripper const& p_gripper)
{
    if (p_gripper.holding())
    {
        return false;
    }
    compages::world::Entity cube;
    compages::world::Entity box;
    p_simulation.robot().world().each<robotik::ecs::SceneObject>(
        [&](compages::world::Entity p_entity,
            robotik::ecs::SceneObject const& p_object)
        {
            if (p_object.name == PICK_PLACE_CUBE)
            {
                cube = p_entity;
            }
            else if (p_object.name == PICK_PLACE_BOX &&
                     p_object.type == robotik::ecs::SceneObject::Type::BOX)
            {
                box = p_entity;
            }
        });
    if (!cube || !box)
    {
        return false;
    }
    auto const cube_at = cube.position();
    auto const box_at = box.position();
    return robotik::scene::restsInside(
        box.get<robotik::ecs::SceneObject>(),
        { box_at.x, box_at.y, box_at.z },
        { cube_at.x, cube_at.y, cube_at.z },
        robotik::scene::halfExtents(cube.get<robotik::ecs::SceneObject>()));
}

void writePickPlaceObservation(robotik::Simulation const& p_simulation,
                               robotik::VacuumGripper const& p_gripper,
                               std::uint32_t p_steps,
                               std::uint32_t p_max_steps,
                               std::span<float> p_observation)
{
    robotik::Vector3 const tip = p_gripper.tip(p_simulation.robot());
    robotik::Vector3 const cube_at = objectCenter(p_simulation, PICK_PLACE_CUBE);
    robotik::Vector3 const box_at = objectCenter(p_simulation, PICK_PLACE_BOX);
    robotik::Vector3 const relative = cube_at - tip;
    float* out = p_observation.data();
    for (robotik::Vector3 const& v : { tip, cube_at, box_at, relative })
    {
        *out++ = static_cast<float>(v.x);
        *out++ = static_cast<float>(v.y);
        *out++ = static_cast<float>(v.z);
    }
    *out++ = p_gripper.holding() ? 1.0f : 0.0f;
    *out++ = p_gripper.suction() ? 1.0f : 0.0f;
    *out++ = static_cast<float>(p_steps) / static_cast<float>(p_max_steps);
    *out = cubeInBox(p_simulation, p_gripper) ? 1.0f : 0.0f;
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
