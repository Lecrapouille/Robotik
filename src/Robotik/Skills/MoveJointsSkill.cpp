#include "Robotik/Skills/MoveJointsSkill.hpp"

#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include <cmath>

namespace robotik
{

MoveJointsSkill::MoveJointsSkill(Targets p_targets, double p_tolerance)
    : m_targets(std::move(p_targets)), m_tolerance(p_tolerance)
{
}

Status MoveJointsSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    if (m_targets.empty())
    {
        return Status::failure;
    }

    bool reached = true;
    for (auto const& [name, target] : m_targets)
    {
        compages::world::Entity joint = findJoint(p_context.world, name);
        if (!joint || !joint.has<ecs::JointCommand>() ||
            !joint.has<ecs::JointState>())
        {
            return Status::failure;
        }
        ecs::JointCommand& command = joint.get<ecs::JointCommand>();
        command.mode = ecs::JointControlMode::Position;
        command.position = target;
        if (std::abs(joint.get<ecs::JointState>().position - target) >
            m_tolerance)
        {
            reached = false;
        }
    }
    return reached ? Status::Success : Status::Running;
}

} // namespace robotik
