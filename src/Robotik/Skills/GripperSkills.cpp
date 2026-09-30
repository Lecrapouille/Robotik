#include "Robotik/Skills/GripperSkills.hpp"

#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/ECS/RobotComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

#include <cmath>

namespace robotik
{

namespace
{

Status commandGrippers(RobotContext& p_context, bool p_open, double p_tolerance)
{
    bool reached = true;
    bool any = false;
    p_context.world.each<ecs::Gripper, ecs::JointCommand, ecs::JointState>(
        [&](compages::world::Entity,
            ecs::Gripper& p_gripper,
            ecs::JointCommand& p_command,
            ecs::JointState& p_state)
        {
            any = true;
            double const goal =
                p_open ? p_gripper.max_opening : p_gripper.min_opening;
            p_command.mode = ecs::JointControlMode::Position;
            p_command.position = goal;
            if (std::abs(p_state.position - goal) > p_tolerance)
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

} // namespace

Status OpenGripperSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    return commandGrippers(p_context, true, m_tolerance);
}

Status CloseGripperSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    return commandGrippers(p_context, false, m_tolerance);
}

} // namespace robotik
