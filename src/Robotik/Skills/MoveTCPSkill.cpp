#include "Robotik/Skills/MoveTCPSkill.hpp"

#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

#include <cmath>

namespace robotik
{

MoveTCPSkill::MoveTCPSkill(std::string p_frame,
                           Pose p_target,
                           double p_joint_tolerance)
    : m_frame(std::move(p_frame)),
      m_target(p_target),
      m_joint_tolerance(p_joint_tolerance)
{
}

void MoveTCPSkill::reset()
{
    m_target_q.clear();
    m_has_target = false;
}

void MoveTCPSkill::setGoal(std::string p_frame, Pose p_target)
{
    if (p_frame != m_frame || p_target.px != m_target.px ||
        p_target.py != m_target.py || p_target.pz != m_target.pz)
    {
        m_frame = std::move(p_frame);
        m_target = p_target;
        reset();
    }
}

Status MoveTCPSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    if (!m_has_target)
    {
        auto solution = p_context.kinematics.solveIK(
            m_frame, m_target, p_context.kinematics.configuration());
        if (!solution)
        {
            return Status::failure;
        }
        m_target_q = std::move(*solution);
        m_has_target = true;
    }

    bool reached = true;
    bool any = false;
    p_context.world
        .each<ecs::JointCommand, ecs::JointState, ecs::PinocchioJointBinding>(
            [&](compages::world::Entity,
                ecs::JointCommand& p_command,
                ecs::JointState& p_state,
                ecs::PinocchioJointBinding& p_binding)
            {
                if (p_binding.q_index < 0 ||
                    p_binding.q_index >= static_cast<int>(m_target_q.size()))
                {
                    return;
                }
                any = true;
                double const goal =
                    m_target_q[static_cast<std::size_t>(p_binding.q_index)];
                p_command.mode = ecs::JointControlMode::Position;
                p_command.position = goal;
                if (std::abs(p_state.position - goal) > m_joint_tolerance)
                {
                    reached = false;
                }
            });

    if (!any)
    {
        return Status::failure;
    }
    return reached ? Status::Success : Status::Running;
}

} // namespace robotik
