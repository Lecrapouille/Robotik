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

namespace robotik
{

RobotRuntime::RobotRuntime(compages::world::World& p_world,
                           std::string const& p_model_file)
    : m_world(p_world)
{
    load(p_model_file, nullptr);
}

RobotRuntime::RobotRuntime(compages::world::World& p_world,
                           std::string const& p_model_file,
                           compages::renderer::Scene& p_scene)
    : m_world(p_world)
{
    load(p_model_file, &p_scene);
}

RobotRuntime::~RobotRuntime() = default;

void RobotRuntime::load(std::string const& p_model_file,
                        compages::renderer::Scene* p_scene)
{
    m_pinocchio = std::make_unique<PinocchioBackend>(p_model_file);
    m_mujoco = std::make_unique<MujocoBackend>(p_model_file);
    RobotLoader{}.instantiate(
        m_world, p_scene, *m_pinocchio, m_mujoco.get(), p_model_file);
}

void RobotRuntime::hold(std::unordered_map<std::string, double> const& p_posture)
{
    m_world.each<ecs::Joint, ecs::JointState, ecs::JointCommand, ecs::HomePosition,
                 ecs::PositionController, ecs::MujocoJointBinding>(
        [&](compages::world::Entity,
            ecs::Joint& p_joint,
            ecs::JointState& p_state,
            ecs::JointCommand& p_command,
            ecs::HomePosition& p_home,
            ecs::PositionController& p_controller,
            ecs::MujocoJointBinding& p_binding)
        {
            if (auto const found = p_posture.find(p_joint.name); found != p_posture.end())
            {
                p_home.position = found->second;
                p_state.position = found->second;
                if (m_mujoco)
                {
                    m_mujoco->setQpos(p_binding.qpos_index, found->second);
                }
            }
            p_command.mode = ecs::JointControlMode::Position;
            p_command.position = p_state.position;
            p_controller.reference = p_state.position;
        });
    PinocchioSyncSystem{}.update(m_world, *m_pinocchio);
    m_pinocchio->updateKinematics();
    JointProjectionSystem{}.update(m_world);
}

void RobotRuntime::pipeline(double p_dt)
{
    m_dt = p_dt;
    m_time += p_dt;

    // The PD loop runs at the physics rate, not at the display rate.
    constexpr double kPhysicsStep = 0.001;
    int const steps = std::clamp(static_cast<int>(std::lround(p_dt / kPhysicsStep)), 1, 50);
    for (int i = 0; i < steps; ++i)
    {
        ControllerSystem{}.update(m_world, kPhysicsStep);
        if (m_mujoco)
        {
            MujocoSyncSystem sync;
            sync.writeCommands(m_world, *m_mujoco);
            m_mujoco->step(kPhysicsStep);
            sync.readState(m_world, *m_mujoco);
        }
    }
    PinocchioSyncSystem{}.update(m_world, *m_pinocchio);
    m_pinocchio->updateKinematics();
    JointProjectionSystem{}.update(m_world);
}

void RobotRuntime::publish(compages::core::Frame const& p_frame)
{
    m_world.update(p_frame);
}

void RobotRuntime::step(double p_dt)
{
    pipeline(p_dt);
    compages::core::Frame frame;
    frame.elapsed = static_cast<float>(p_dt);
    frame.total = static_cast<float>(m_time);
    frame.width = 1;
    frame.height = 1;
    publish(frame);
}

void RobotRuntime::step(compages::world::ViewFrame const& p_frame)
{
    pipeline(p_frame.elapsed);
    m_world.update(p_frame);
}

RobotContext RobotRuntime::context() const
{
    return RobotContext{m_world, *m_pinocchio, m_mujoco.get(), m_time, m_dt};
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
