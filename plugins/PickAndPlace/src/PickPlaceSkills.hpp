// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PickPlaceSkills.hpp
/// Pick-and-place skills with a vacuum gripper.
///
/// Object positions come from the @ref WorldModel (what the robot believes),
/// never from the simulator.
#pragma once

#include "Robotik/Robot/Components/Objects.hpp"
#include "Robotik/Task/Skills/MotionSkills.hpp"

#include <optional>
#include <string>

namespace robotik
{

class VacuumGripper;

// ****************************************************************************
/// Brings the suction cup @p_clearance above the top of an object,
/// pointing down. Over an open box, the cup goes down to the inner floor
/// plus the height of what it carries.
// ****************************************************************************
class ApproachSkill final: public Skill
{
public:

    /// @param p_gripper @ref VacuumGripper name; empty for the first one.
    /// @param p_container Shape of the object when it is an open box, from
    ///   the world file (known before the run, not measured).
    ApproachSkill(std::string p_object,
                  robotik::Length p_clearance,
                  std::string p_gripper = {},
                  std::optional<ecs::Shape> p_container = {});

    void reset() override;
    Status tick(RobotContext& p_context, robotik::Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_object;
    robotik::Length m_clearance;
    std::string m_gripper;
    std::optional<ecs::Shape> m_container;
    MoveTCPSkill m_move;
    bool m_planned = false;
};

// ****************************************************************************
/// Turns the suction on and waits until the vacuum holds something, then
/// labels the cup with the object (@c VacuumGripper::label): the vacuum
/// cannot say what it holds, the program knows what it went to take.
// ****************************************************************************
class GraspSkill final: public Skill
{
public:

    explicit GraspSkill(std::string p_object,
                        std::string p_gripper = {},
                        robotik::Seconds p_timeout = robotik::Seconds(0.5));

    void reset() override
    {
        m_waited = robotik::Seconds{};
        m_why.clear();
    }

    Status tick(RobotContext& p_context, robotik::Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_object;
    std::string m_gripper;
    robotik::Seconds m_timeout;
    robotik::Seconds m_waited{};
    std::string m_why;
};

// ****************************************************************************
/// Turns the suction off and waits until nothing is held.
// ****************************************************************************
class ReleaseSkill final: public Skill
{
public:

    explicit ReleaseSkill(std::string p_gripper = {},
                          robotik::Seconds p_timeout = robotik::Seconds(0.5));

    void reset() override
    {
        m_waited = robotik::Seconds{};
        m_why.clear();
    }

    Status tick(RobotContext& p_context, robotik::Seconds p_dt) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_gripper;
    robotik::Seconds m_timeout;
    robotik::Seconds m_waited{};
    std::string m_why;
};

// ****************************************************************************
/// Waits for a fresh observation of an object in the @ref WorldModel.
// ****************************************************************************
class DetectSkill final: public Skill
{
public:

    explicit DetectSkill(std::string p_object,
                         robotik::Seconds p_timeout = robotik::Seconds(2.0));

    void reset() override
    {
        m_started = false;
        m_observations_at_start = 0U;
        m_why.clear();
    }

    Status tick(RobotContext& p_context, robotik::Seconds p_dt) override;

    [[nodiscard]] std::string why() const override
    {
        return m_why;
    }

private:

    std::string m_why;

    std::string m_object;
    robotik::Seconds m_timeout;
    robotik::Seconds m_start{};
    bool m_started = false;
    std::uint32_t m_observations_at_start = 0U;
};

/// Gripper named @p_name, or the first one when empty.
[[nodiscard]] VacuumGripper* findGripper(Robot& p_robot,
                                         std::string const& p_name);

/// @copydoc findGripper(Robot&, std::string const&), read only.
[[nodiscard]] VacuumGripper const* findGripper(Robot const& p_robot,
                                               std::string const& p_name);

/// Id of the object the gripper @p_name holds: its label while the vacuum
/// holds, or empty.
[[nodiscard]] std::string heldObject(Robot const& p_robot,
                                     std::string const& p_name = {});

} // namespace robotik
