// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/RobotRuntime.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/Model/RobotLoader.hpp"
#include "Robotik/Systems/ControllerSystem.hpp"
#include "Robotik/Systems/JointProjectionSystem.hpp"
#include "Robotik/Systems/MujocoSyncSystem.hpp"
#include "Robotik/Systems/PinocchioSyncSystem.hpp"

#include "Compages/Core/Frame.hpp"
#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Controllers/ViewFrame.hpp"
#include "Compages/World/World.hpp"

#include <algorithm>
#include <cmath>

#define PHYSICS_STEP_S 0.001

namespace robotik
{

RobotRuntime::RobotRuntime(compages::world::World& p_world,
                           std::filesystem::path const& p_model_file)
    : m_world(p_world)
{
    load(p_model_file, nullptr);
}

RobotRuntime::RobotRuntime(compages::world::World& p_world,
                           std::filesystem::path const& p_model_file,
                           compages::renderer::Scene& p_scene)
    : m_world(p_world)
{
    load(p_model_file, &p_scene);
}

RobotRuntime::~RobotRuntime() = default;

void RobotRuntime::load(std::filesystem::path const& p_model_file,
                        compages::renderer::Scene* p_scene)
{
    m_pinocchio = std::make_unique<PinocchioBackend>(p_model_file);
    m_mujoco = std::make_unique<MujocoBackend>(p_model_file);
    RobotLoader{}.instantiate(
        m_world, p_scene, *m_pinocchio, m_mujoco.get(), p_model_file);
}

void RobotRuntime::hold(JointPosture const& p_posture)
{
    m_world.each<ecs::Joint,
                 ecs::JointState,
                 ecs::JointCommand,
                 ecs::HomePosition,
                 ecs::PositionController,
                 ecs::MujocoJointBinding>(
        [&](compages::world::Entity,
            ecs::Joint const& p_joint,
            ecs::JointState& p_state,
            ecs::JointCommand& p_command,
            ecs::HomePosition& p_home,
            ecs::PositionController& p_controller,
            ecs::MujocoJointBinding const& p_binding)
        {
            // Find the joint in the posture
            if (auto const found = p_posture.find(p_joint.name);
                found != p_posture.end())
            {
                // Convert the goal to SI
                double const q = jointGoalSi(found->second);
                ecs::setHomePosition(p_home, q);
                ecs::setPosition(p_state, q);

                // Set the position in MuJoCo
                if (m_mujoco)
                {
                    m_mujoco->setQpos(p_binding.qpos_index, q);
                }
            }

            // Set the command mode and position
            ecs::setCommandMode(p_command, ecs::JointControlMode::POSITION);
            double const q = ecs::positionSi(p_state);
            ecs::setCommandPosition(p_command, q);
            p_controller.reference = q;
        });

    // Update the Pinocchio backend
    PinocchioSyncSystem{}.update(m_world, *m_pinocchio);
    m_pinocchio->updateKinematics();
    JointProjectionSystem{}.update(m_world);
}

void RobotRuntime::pipeline(Seconds p_dt)
{
    m_dt = p_dt;
    m_time = m_time + p_dt;

    // The PD loop runs at the physics rate, not at the display rate.
    Seconds const physics_step(PHYSICS_STEP_S);

    // Calculate the number of steps
    int const steps = std::clamp(
        static_cast<int>(std::lround(p_dt.value() / physics_step.value())),
        1,
        50);

    // Step the physics
    for (int i = 0; i < steps; ++i)
    {
        ControllerSystem{}.update(m_world, physics_step);
        if (m_mujoco)
        {
            // Write the commands to MuJoCo
            MujocoSyncSystem sync;
            sync.writeCommands(m_world, *m_mujoco);
            m_mujoco->step(physics_step);

            // Read the state from MuJoCo
            sync.readState(m_world, *m_mujoco);
        }
    }

    // Update the Pinocchio backend
    PinocchioSyncSystem{}.update(m_world, *m_pinocchio);
    m_pinocchio->updateKinematics();

    // Update the joint projections
    JointProjectionSystem{}.update(m_world);
}

void RobotRuntime::publish(compages::core::Frame const& p_frame)
{
    m_world.update(p_frame);
}

void RobotRuntime::step(Seconds p_dt)
{
    pipeline(p_dt);
    compages::core::Frame frame;
    frame.elapsed = static_cast<float>(p_dt.value());
    frame.total = static_cast<float>(m_time.value());
    frame.width = 1;
    frame.height = 1;
    publish(frame);
}

void RobotRuntime::step(compages::world::ViewFrame const& p_frame)
{
    pipeline(Seconds(p_frame.elapsed));
    m_world.update(p_frame);
}

RobotContext RobotRuntime::context() const
{
    return RobotContext{ m_world, *m_pinocchio, m_mujoco.get(), m_time, m_dt };
}

PinocchioBackend& RobotRuntime::kinematics()
{
    return *m_pinocchio;
}

MujocoBackend* RobotRuntime::simulation()
{
    return m_mujoco.get();
}

} // namespace robotik
