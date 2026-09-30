/**
 * @file GripperSkills.hpp
 * @brief Open and close skills for @ref ecs::Gripper finger joints.
 */

#pragma once

#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

/**
 * @brief Commands all @ref ecs::Gripper entities to @ref ecs::Gripper::max_opening.
 */
class OpenGripperSkill final: public Skill
{
public:

    /**
     * @brief Creates the skill.
     * @param p_tolerance Max |error| on finger joints for success.
     */
    explicit OpenGripperSkill(double p_tolerance = 1e-3)
        : m_tolerance(p_tolerance)
    {
    }

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Position error threshold. */
    double m_tolerance;
};

/**
 * @brief Commands all @ref ecs::Gripper entities to @ref ecs::Gripper::min_opening.
 */
class CloseGripperSkill final: public Skill
{
public:

    /**
     * @brief Creates the skill.
     * @param p_tolerance Max |error| on finger joints for success.
     */
    explicit CloseGripperSkill(double p_tolerance = 1e-3)
        : m_tolerance(p_tolerance)
    {
    }

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Position error threshold. */
    double m_tolerance;
};

} // namespace robotik
