#include "Robotik/Skills/HomeSkill.hpp"

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

#include <cmath>

namespace robotik
{

Status HomeSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    bool reached = true;
    bool any = false;
    p_context.world.each<ecs::JointCommand, ecs::HomePosition, ecs::JointState>(
        [&](compages::world::Entity,
            ecs::JointCommand& p_command,
            ecs::HomePosition& p_home,
            ecs::JointState& p_state)
        {
            any = true;
            p_command.mode = ecs::JointControlMode::Position;
            p_command.position = p_home.position;
            if (std::abs(p_state.position - p_home.position) > m_tolerance)
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
