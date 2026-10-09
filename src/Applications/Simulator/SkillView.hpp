// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Runtime/Scheduler.hpp"

#include <imgui.h>

#include <algorithm>
#include <string>
#include <vector>

inline ImVec4 const GREY(0.55f, 0.55f, 0.58f, 1.0f);
inline ImVec4 const ORANGE(1.00f, 0.70f, 0.20f, 1.0f);
inline ImVec4 const GREEN(0.35f, 0.85f, 0.40f, 1.0f);
inline ImVec4 const RED(0.95f, 0.35f, 0.35f, 1.0f);
inline ImVec4 const BLUE(0.45f, 0.65f, 1.00f, 1.0f);

inline ImVec4 colorOf(robotik::SkillState p_state)
{
    switch (p_state)
    {
        case robotik::SkillState::Idle:
            return GREY;
        case robotik::SkillState::Waiting:
            return BLUE;
        case robotik::SkillState::Running:
            return ORANGE;
        case robotik::SkillState::Succeeded:
            return GREEN;
        case robotik::SkillState::Failed:
            return RED;
        case robotik::SkillState::Cancelled:
            return GREY;
    }
    return GREY;
}

inline std::string skillOwnerName(robotik::SkillScheduler const& p_skills,
                                  robotik::OwnerId p_owner)
{
    if (p_owner == robotik::NO_OWNER)
    {
        return "-";
    }
    if (p_owner == robotik::ANONYMOUS || p_owner >= p_skills.size())
    {
        return "(external)";
    }
    return p_skills.name(p_owner);
}

namespace skill_detail
{

struct HeldResource
{
    std::string name;
    std::string holder;
    bool unavailable = false;
};

inline std::vector<HeldResource>
blockingResources(robotik::SkillScheduler const& p_skills,
                  robotik::ResourceManager const& p_resources,
                  robotik::SkillId p_id)
{
    std::vector<HeldResource> found;
    for (robotik::ResourceRequirement const& need :
         p_skills.description(p_id).resources)
    {
        if (!p_resources.available(need.id))
        {
            found.push_back({ p_resources.name(need.id), {}, true });
            continue;
        }
        robotik::OwnerId const owner = p_resources.owner(need.id);
        bool const taken = owner != robotik::NO_OWNER && owner != p_id;
        bool const shared = need.access == robotik::Access::Exclusive &&
                            p_resources.users(need.id) > 0u;
        if (!taken && !shared)
        {
            continue;
        }
        found.push_back(
            { p_resources.name(need.id),
              taken ? skillOwnerName(p_skills, owner) : std::string(),
              false });
    }
    return found;
}

inline std::string formatHeld(std::vector<HeldResource> const& p_held)
{
    if (p_held.empty())
    {
        return {};
    }
    std::string const& holder = p_held.front().holder;
    bool const same =
        std::all_of(p_held.begin(),
                    p_held.end(),
                    [&](HeldResource const& p_item)
                    { return !p_item.unavailable && p_item.holder == holder; });
    std::string text;
    if (same)
    {
        for (HeldResource const& item : p_held)
        {
            if (!text.empty())
            {
                text += ", ";
            }
            text += item.name;
        }
        if (!holder.empty())
        {
            text += " (";
            text += holder;
            text += ")";
        }
        return text;
    }
    for (HeldResource const& item : p_held)
    {
        if (!text.empty())
        {
            text += ", ";
        }
        text += item.name;
        if (item.unavailable)
        {
            text += " (unavailable)";
        }
        else if (!item.holder.empty())
        {
            text += " (";
            text += item.holder;
            text += ")";
        }
    }
    return text;
}

} // namespace skill_detail

inline std::string skillWhy(robotik::SkillScheduler const& p_skills,
                            robotik::ResourceManager const& p_resources,
                            robotik::SkillId p_id)
{
    robotik::SkillReason const reason = p_skills.reason(p_id);
    if (reason == robotik::SkillReason::None)
    {
        return {};
    }
    if (reason == robotik::SkillReason::Precondition)
    {
        std::string text = robotik::toString(reason);
        if (robotik::Precondition const* failed = p_skills.precondition(p_id))
        {
            text += ": ";
            text += failed->text;
        }
        return text;
    }
    std::vector<skill_detail::HeldResource> held =
        skill_detail::blockingResources(p_skills, p_resources, p_id);
    if (held.empty())
    {
        robotik::ResourceId const blocker = p_skills.blocker(p_id);
        if (blocker == robotik::NO_RESOURCE)
        {
            return robotik::toString(reason);
        }
        bool const unavailable = reason == robotik::SkillReason::Unavailable ||
                                 reason == robotik::SkillReason::ResourceLost;
        robotik::OwnerId const owner = p_resources.owner(blocker);
        bool const named = !unavailable && owner != robotik::NO_OWNER;
        held.push_back(
            { p_resources.name(blocker),
              named ? skillOwnerName(p_skills, owner) : std::string(),
              unavailable });
    }
    std::string text = robotik::toString(reason);
    text += ": ";
    text += skill_detail::formatHeld(held);
    return text;
}
