#include "Robotik/Skills/MoveJointSkill.hpp"

#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include <cmath>

namespace robotik
{

//------------------------------------------------------------------------------
MoveJointSkill::MoveJointSkill(std::string p_joint_name,
                               double p_target,
                               double p_tolerance)
    : m_joint_name(std::move(p_joint_name)),
      m_target(p_target),
      m_tolerance(p_tolerance)
{
}

//------------------------------------------------------------------------------
void MoveJointSkill::setGoal(std::string p_joint_name, double p_target)
{
    if (p_joint_name != m_joint_name || p_target != m_target)
    {
        m_joint_name = std::move(p_joint_name);
        m_target = p_target;
    }
}

//------------------------------------------------------------------------------
Status MoveJointSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    // Find the joint
    compages::world::Entity joint = findJoint(p_context.world, m_joint_name);
    if (!joint || !joint.has<ecs::JointState>() ||
        !joint.has<ecs::JointCommand>())
    {
        return Status::FAILURE;
    }

    // Set the command mode and position
    ecs::JointCommand& command = joint.get<ecs::JointCommand>();
    ecs::setCommandMode(command, ecs::JointControlMode::POSITION);
    ecs::setCommandPosition(command, m_target);

    // Check if the position is reached
    double const error =
        ecs::positionSi(joint.get<ecs::JointState>()) - m_target;
    return std::abs(error) <= m_tolerance ? Status::SUCCESS : Status::RUNNING;
}

} // namespace robotik
