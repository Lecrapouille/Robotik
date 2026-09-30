/**
 * @file MoveJointsSkill.hpp
 * @brief Multi-joint simultaneous position skill.
 */

#pragma once

#include "Robotik/Skills/Skill.hpp"

#include <string>
#include <unordered_map>

namespace robotik
{

/**
 * @brief Commands several joints at once; succeeds when all are within tolerance.
 */
class MoveJointsSkill final: public Skill
{
public:

    /** @brief Map of joint name to target position. */
    using Targets = std::unordered_map<std::string, double>;

    /**
     * @brief Creates the skill with a fixed target map.
     * @param p_targets Joint names and goal positions.
     * @param p_tolerance Per-joint success threshold.
     */
    explicit MoveJointsSkill(Targets p_targets, double p_tolerance = 1e-2);

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Joint goals. */
    Targets m_targets;

    /** @brief Position error threshold. */
    double m_tolerance;
};

} // namespace robotik
