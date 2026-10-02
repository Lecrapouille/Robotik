#include "Robotik/Skills/MoveTCPSkill.hpp"

#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"
#include "Robotik/Runtime/RobotContext.hpp"

#include "Compages/World/Entity.hpp"

#include <cmath>

namespace robotik
{

//------------------------------------------------------------------------------
//! @brief Apply the IK joint row.
//! @param p_entity The entity.
//! @param p_command The command.
//! @param p_state The state.
//! @param p_binding The binding.
//! @param p_target_q The target q.
//! @param p_joint_tolerance The joint tolerance.
//! @param p_any The any.
//! @param p_reached The reached.
//------------------------------------------------------------------------------
static void applyIkJointRow(compages::world::Entity,
                            ecs::JointCommand& p_command,
                            ecs::JointState const& p_state,
                            ecs::PinocchioJointBinding const& p_binding,
                            std::vector<double> const& p_target_q,
                            double p_joint_tolerance,
                            bool& p_any,
                            bool& p_reached)
{
    // Check if the binding index is valid
    if (p_binding.q_index < 0 ||
        p_binding.q_index >= static_cast<int>(p_target_q.size()))
    {
        return;
    }

    // Set the command mode and position
    p_any = true;
    double const goal = p_target_q[static_cast<std::size_t>(p_binding.q_index)];
    ecs::setCommandMode(p_command, ecs::JointControlMode::POSITION);
    ecs::setCommandPosition(p_command, goal);

    // Check if the position is reached
    if (std::abs(ecs::positionSi(p_state) - goal) > p_joint_tolerance)
    {
        p_reached = false;
    }
}

//------------------------------------------------------------------------------
//! @brief Track the IK configuration.
//! @param p_world The world.
//! @param p_target_q The target q.
//! @param p_joint_tolerance The joint tolerance.
//! @param p_any The any.
//! @param p_reached The reached.
//------------------------------------------------------------------------------
static void trackIkConfiguration(compages::world::World& p_world,
                                 std::vector<double> const& p_target_q,
                                 double p_joint_tolerance,
                                 bool& p_any,
                                 bool& p_reached)
{
    p_world
        .each<ecs::JointCommand, ecs::JointState, ecs::PinocchioJointBinding>(
            [&p_target_q, &p_joint_tolerance, &p_any, &p_reached](
                compages::world::Entity p_entity,
                ecs::JointCommand& p_command,
                ecs::JointState const& p_state,
                ecs::PinocchioJointBinding const& p_binding)
            {
                applyIkJointRow(p_entity,
                                p_command,
                                p_state,
                                p_binding,
                                p_target_q,
                                p_joint_tolerance,
                                p_any,
                                p_reached);
            });
}

//------------------------------------------------------------------------------
MoveTCPSkill::MoveTCPSkill(std::string p_frame,
                           Pose const& p_target,
                           double p_joint_tolerance)
    : m_frame(std::move(p_frame)),
      m_target(p_target),
      m_joint_tolerance(p_joint_tolerance)
{
}

//------------------------------------------------------------------------------
void MoveTCPSkill::reset()
{
    m_target_q.clear();
    m_has_target = false;
}

//------------------------------------------------------------------------------
void MoveTCPSkill::setGoal(std::string p_frame, Pose const& p_target)
{
    if (p_frame != m_frame || p_target.px != m_target.px ||
        p_target.py != m_target.py || p_target.pz != m_target.pz)
    {
        m_frame = std::move(p_frame);
        m_target = p_target;
        reset();
    }
}

//------------------------------------------------------------------------------
Status MoveTCPSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    if (!m_has_target)
    {
        // Solve the IK
        auto solution = p_context.kinematics.solveIK(
            m_frame, m_target, p_context.kinematics.configuration());
        if (!solution)
        {
            return Status::FAILURE;
        }

        // Set the target q
        m_target_q = std::move(*solution);
        m_has_target = true;
    }

    // Track the IK configuration
    bool reached = true;
    bool any = false;
    trackIkConfiguration(
        p_context.world, m_target_q, m_joint_tolerance, any, reached);

    // Check if any joint is commanded
    if (!any)
    {
        return Status::FAILURE;
    }
    return reached ? Status::SUCCESS : Status::RUNNING;
}

} // namespace robotik
