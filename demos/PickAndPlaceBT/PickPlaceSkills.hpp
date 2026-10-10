// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PickPlaceSkills.hpp
//! @brief Pick-and-place skills with a vacuum gripper.
//!
//! Object positions come from the @ref WorldModel (what the robot believes),
//! never from the simulator.
#pragma once

#include "Robotik/Skills/MotionSkills.hpp"

#include <string>

namespace robotik
{

class VacuumGripper;

// ****************************************************************************
//! @brief Brings the suction cup @p_clearance above the top of an object,
//! pointing down.
// ****************************************************************************
class ApproachSkill final: public Skill
{
public:

    //! @param p_gripper @ref VacuumGripper name; empty for the first one.
    ApproachSkill(std::string p_object,
                  Length p_clearance,
                  std::string p_gripper = {});

    void reset() override;
    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_object;
    Length m_clearance;
    std::string m_gripper;
    MoveTCPSkill m_move;
    bool m_planned = false;
};

// ****************************************************************************
//! @brief Turns the suction on and waits until the object is held.
// ****************************************************************************
class GraspSkill final: public Skill
{
public:

    explicit GraspSkill(std::string p_object,
                        std::string p_gripper = {},
                        Seconds p_timeout = Seconds(0.5));

    void reset() override
    {
        m_waited = Seconds{};
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;
    void cancel(RobotContext& p_context) override;

private:

    std::string m_object;
    std::string m_gripper;
    Seconds m_timeout;
    Seconds m_waited{};
};

// ****************************************************************************
//! @brief Turns the suction off and waits until nothing is held.
// ****************************************************************************
class ReleaseSkill final: public Skill
{
public:

    explicit ReleaseSkill(std::string p_gripper = {},
                          Seconds p_timeout = Seconds(0.5));

    void reset() override
    {
        m_waited = Seconds{};
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    std::string m_gripper;
    Seconds m_timeout;
    Seconds m_waited{};
};

// ****************************************************************************
//! @brief Waits for a fresh observation of an object in the @ref WorldModel.
// ****************************************************************************
class DetectSkill final: public Skill
{
public:

    explicit DetectSkill(std::string p_object, Seconds p_timeout = Seconds(2.0));

    void reset() override
    {
        m_started = false;
        m_observations_at_start = 0u;
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    std::string m_object;
    Seconds m_timeout;
    Seconds m_start{};
    bool m_started = false;
    std::uint32_t m_observations_at_start = 0u;
};

//! @brief Gripper named @p_name, or the first one when empty.
[[nodiscard]] VacuumGripper* findGripper(Robot const& p_robot,
                                         std::string const& p_name);

} // namespace robotik
