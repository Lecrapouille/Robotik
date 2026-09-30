/**
 * @file Skill.hpp
 * @brief Abstract unit of robot behavior ticked from code or behavior trees.
 */

#pragma once

#include "Robotik/Runtime/Status.hpp"

namespace robotik
{

struct RobotContext;

/**
 * @brief One-shot or continuous action that writes @ref ecs::JointCommand.
 *
 * Skills read state through @ref RobotContext and ECS; they do not call MuJoCo
 * or drivers directly. Register instances with @ref registerSkill for BT use.
 *
 * @example
 * @code
 * class WaveSkill : public robotik::Skill {
 * public:
 *     robotik::Status tick(robotik::RobotContext& ctx, double dt) override {
 *         // set joint commands...
 *         return robotik::Status::Running;
 *     }
 * };
 * @endcode
 */
class Skill
{
public:

    virtual ~Skill() = default;

    /**
     * @brief Clears internal planning state before a new BT action run.
     *
     * Default implementation does nothing.
     */
    virtual void reset() {}

    /**
     * @brief Advances the skill by one simulation step.
     * @param p_context World, backends, time, and dt.
     * @param p_dt Step duration in seconds (often equals @c p_context.dt).
     * @return @ref Status::Running until the goal is met or failed.
     */
    virtual Status tick(RobotContext& p_context, double p_dt) = 0;
};

} // namespace robotik
