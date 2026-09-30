/**
 * @file PickPlaceSkills.hpp
 * @brief Pick-and-place skills for vacuum gripper scenarios.
 */

#pragma once

#include "Robotik/Skills/MoveTCPSkill.hpp"
#include "Robotik/Skills/Skill.hpp"

#include <string>

namespace robotik
{

/**
 * @brief Moves the suction cup above an object with the tool pointing down.
 *
 * @p_clearance is added above the object top plus @ref ecs::VacuumGripper::tool_length.
 */
class ApproachSkill final: public Skill
{
public:

    /**
     * @brief Creates the approach skill.
     * @param p_object @ref ecs::SceneObject name.
     * @param p_clearance Extra height above the object top, in meters.
     */
    ApproachSkill(std::string p_object, double p_clearance);

    void reset() override;
    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Target object name. */
    std::string m_object;

    /** @brief Vertical offset above object top. */
    double m_clearance;

    /** @brief Internal Cartesian mover. */
    MoveTCPSkill m_move;

    /** @brief True after IK goal has been set for this approach. */
    bool m_planned = false;
};

/**
 * @brief Attaches the object to @ref ecs::VacuumGripper when the cup is close enough.
 */
class GraspSkill final: public Skill
{
public:

    /**
     * @brief Creates the grasp skill.
     * @param p_object Object name to grasp.
     */
    explicit GraspSkill(std::string p_object) : m_object(std::move(p_object)) {}

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Object to attach. */
    std::string m_object;
};

/**
 * @brief Releases @ref ecs::VacuumGripper::held and drops the object onto support below.
 */
class ReleaseSkill final: public Skill
{
public:

    Status tick(RobotContext& p_context, double p_dt) override;
};

/**
 * @brief Waits until @ref ecs::DetectedObjects lists @p_object, or succeeds without camera.
 */
class DetectSkill final: public Skill
{
public:

    /**
     * @brief Creates the detect skill.
     * @param p_object Label to look for in detections.
     * @param p_timeout Seconds before @ref Status::failure if never seen.
     */
    explicit DetectSkill(std::string p_object, double p_timeout = 2.0)
        : m_object(std::move(p_object)), m_timeout(p_timeout)
    {
    }

    void reset() override
    {
        m_waited = 0.0;
    }

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Expected detection label. */
    std::string m_object;

    /** @brief Maximum wait time when a camera is present. */
    double m_timeout;

    /** @brief Accumulated wait time in the current run. */
    double m_waited = 0.0;
};

} // namespace robotik
