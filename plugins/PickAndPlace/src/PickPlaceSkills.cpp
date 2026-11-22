// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceSkills.hpp"

#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Task/Grasp/ContainerBounds.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/RobotContext.hpp"

#include "Robotik/Common/Math.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>

namespace robotik
{

static constexpr robotik::Length DEFAULT_CUBE_HALF_HEIGHT{ 0.02 };
static constexpr robotik::Length TRANSIT_CLEARANCE_THRESHOLD{ 0.08 };
static constexpr robotik::Length MAX_FINAL_APPROACH_CLEARANCE{ 0.03 };

/// Half height of the object the gripper holds, from the beliefs of its
/// label; the default cube half height when the size is not known.
static robotik::Length carriedHalfHeight(WorldModel const& p_beliefs,
                                         VacuumGripper const& p_gripper)
{
    WorldObject const* held = p_gripper.holding() && !p_gripper.label().empty()
                                  ? p_beliefs.find(p_gripper.label())
                                  : nullptr;
    if (held == nullptr || held->size.z <= 0.0)
    {
        return DEFAULT_CUBE_HALF_HEIGHT;
    }
    return robotik::Length(held->size.z * 0.5);
}

/// If the clearance is greater than the transit clearance threshold,
/// return the clearance. Otherwise, return the minimum of the clearance and the
/// final approach clearance.
static robotik::Length clearanceAboveCavity(robotik::Length p_clearance)
{
    // Transit keeps full clearance; Reach (≈ 0) caps at 3 cm above the cavity.
    if (p_clearance >= TRANSIT_CLEARANCE_THRESHOLD)
    {
        return p_clearance;
    }
    return std::min(p_clearance, MAX_FINAL_APPROACH_CLEARANCE);
}

/// TCP height over an open box. @p_at is the believed centre (perception or
/// the scenario prior). The wall layout comes from the shape in the world
/// file.
static std::optional<robotik::Length>
tcpHeightOverOpenBox(ecs::Shape const& p_box,
                     WorldModel const& p_beliefs,
                     VacuumGripper const& p_gripper,
                     robotik::Length p_clearance,
                     Vector3 const& p_at)
{
    std::optional<scene::ContainerInner> const inner =
        scene::innerBounds(p_box, p_at);
    if (!inner)
    {
        return std::nullopt;
    }

    robotik::Length const half = carriedHalfHeight(p_beliefs, p_gripper);
    robotik::Length const above = clearanceAboveCavity(p_clearance);
    return robotik::Length(inner->floor_z) + half * 2.0 + above +
           p_gripper.length();
}

VacuumGripper* findGripper(Robot& p_robot, std::string const& p_name)
{
    return p_name.empty() ? p_robot.actuators().first<VacuumGripper>()
                          : p_robot.actuators().find<VacuumGripper>(p_name);
}

VacuumGripper const* findGripper(Robot const& p_robot,
                                 std::string const& p_name)
{
    return p_name.empty() ? p_robot.actuators().first<VacuumGripper>()
                          : p_robot.actuators().find<VacuumGripper>(p_name);
}

ApproachSkill::ApproachSkill(std::string p_object,
                             robotik::Length p_clearance,
                             std::string p_gripper,
                             std::optional<ecs::Shape> p_container)
    : m_object(std::move(p_object)),
      m_clearance(p_clearance),
      m_gripper(std::move(p_gripper)),
      m_container(std::move(p_container))
{
}

void ApproachSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

Status ApproachSkill::tick(RobotContext& p_context, robotik::Seconds p_dt)
{
    if (!m_planned)
    {
        // Believed pose from perception (or oracle), not ECS ground truth.
        WorldObject const* object = p_context.world_model.find(m_object);
        VacuumGripper const* gripper = findGripper(p_context.robot, m_gripper);
        if (object == nullptr || gripper == nullptr)
        {
            return Status::FAILURE;
        }

        // Default target: hover above the object's reported top + clearance.
        Vector3 at = object->position;
        double const yaw = std::atan2(at.y, at.x);
        robotik::Length height = robotik::Length(top(*object)) + m_clearance + gripper->length();

        // If the object is a box, calculate the TCP height above the inner
        // floor.
        if (m_container)
        {
            if (std::optional<robotik::Length> over_box =
                    tcpHeightOverOpenBox(*m_container,
                                         p_context.world_model,
                                         *gripper,
                                         m_clearance,
                                         at))
            {
                height = *over_box;
            }
        }

        // Create the target pose.
        Pose const target{
            { at.x, at.y, height.value() },
            { 0.0, std::cos(yaw * 0.5), std::sin(yaw * 0.5), 0.0 }
        };

        // Create the move skill.
        m_move = MoveTCPSkill(gripper->link(), target);
        m_planned = true;
    }

    // Tick the move skill.
    return m_move.tick(p_context, p_dt);
}

void ApproachSkill::cancel(RobotContext& p_context)
{
    m_move.cancel(p_context);
}

GraspSkill::GraspSkill(std::string p_object,
                       std::string p_gripper,
                       robotik::Seconds p_timeout)
    : m_object(std::move(p_object)),
      m_gripper(std::move(p_gripper)),
      m_timeout(p_timeout)
{
}

Status GraspSkill::tick(RobotContext& p_context, robotik::Seconds p_dt)
{
    // Find the gripper by name.
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        m_why = "no vacuum gripper on the robot";
        return Status::FAILURE;
    }

    // Turn on the vacuum.
    gripper->suction(true);

    // Success when the vacuum holds: the cup took the object it was sent to.
    if (gripper->holding())
    {
        gripper->label(m_object);
        return Status::SUCCESS;
    }

    // If the timeout has been reached, turn off the vacuum and return failure.
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        gripper->suction(false);
        m_why = "suction holds nothing after " +
                std::to_string(m_timeout.value()) + " s";
        return Status::FAILURE;
    }

    // Otherwise, return running.
    return Status::RUNNING;
}

void GraspSkill::cancel(RobotContext& p_context)
{
    if (VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
        gripper != nullptr && !gripper->holding())
    {
        gripper->suction(false);
    }
}

std::string heldObject(Robot const& p_robot, std::string const& p_name)
{
    VacuumGripper const* gripper = findGripper(p_robot, p_name);
    return gripper != nullptr && gripper->holding() ? gripper->label()
                                                    : std::string{};
}

ReleaseSkill::ReleaseSkill(std::string p_gripper, robotik::Seconds p_timeout)
    : m_gripper(std::move(p_gripper)), m_timeout(p_timeout)
{
}

Status ReleaseSkill::tick(RobotContext& p_context, robotik::Seconds p_dt)
{
    // Find the gripper by name.
    VacuumGripper* gripper = findGripper(p_context.robot, m_gripper);
    if (gripper == nullptr)
    {
        m_why = "no vacuum gripper on the robot";
        return Status::FAILURE;
    }

    // Turn off the vacuum.
    gripper->suction(false);
    if (!gripper->holding())
    {
        return Status::SUCCESS;
    }

    // Timeout has been reached ?
    m_waited += p_dt;
    if (m_waited > m_timeout)
    {
        m_why = "the object is still held after the suction stopped";
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

DetectSkill::DetectSkill(std::string p_object, robotik::Seconds p_timeout)
    : m_object(std::move(p_object)), m_timeout(p_timeout)
{
}

Status DetectSkill::tick(RobotContext& p_context, robotik::Seconds /*p_dt*/)
{
    // If the skill has not started, start it and record the observations at the
    // start.
    if (!m_started)
    {
        m_start = p_context.time;
        m_started = true;

        // Ignore detections from before this attempt (e.g. stale camera frame).
        if (WorldObject const* prior = p_context.world_model.find(m_object))
        {
            m_observations_at_start = prior->observations;
        }
    }

    // Find the object by name.
    WorldObject const* object = p_context.world_model.find(m_object);
    if (object == nullptr)
    {
        m_why = m_object + " is not in the world model";
        return Status::FAILURE;
    }

    // Check if the object is fresh and has been seen since the start.
    bool const fresh = object->observations > m_observations_at_start;
    if (fresh && object->seen >= m_start)
    {
        return Status::SUCCESS;
    }

    // Timeout has been reached ?
    if (p_context.time - m_start > m_timeout)
    {
        m_why = m_object + " not seen by the camera within " +
                std::to_string(static_cast<int>(m_timeout.value())) + " s";
        return Status::FAILURE;
    }
    return Status::RUNNING;
}

} // namespace robotik
