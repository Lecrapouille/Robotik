// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.


#include "Robotik/Task/CellDescription.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

/// The robot flange and the tool link that lands on it (see Tools.md).
static constexpr char const* FLANGE = "flange";
static constexpr char const* TOOL_MOUNT = "tool_mount";

EntitySpec const* WorldDescription::find(std::string_view p_id) const
{
    auto const found = std::find_if(entities.begin(),
                                    entities.end(),
                                    [p_id](EntitySpec const& p_entity)
                                    { return p_entity.id == p_id; });
    return found == entities.end() ? nullptr : &*found;
}

EntitySpec const& WorldDescription::robot() const
{
    for (EntitySpec const& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Robot)
        {
            return spec;
        }
    }
    throw std::logic_error("World '" + id + "' has no robot");
}

std::vector<EntitySpec const*> WorldDescription::robots() const
{
    std::vector<EntitySpec const*> result;
    for (EntitySpec const& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Robot)
        {
            result.push_back(&spec);
        }
    }
    return result;
}

EntitySpec const* WorldDescription::mountedTool() const
{
    return mountedTool(robot().id);
}

EntitySpec const* WorldDescription::mountedTool(std::string_view p_robot) const
{
    for (EntitySpec const& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Tool && spec.attach &&
            spec.attach->parent == p_robot)
        {
            return &spec;
        }
    }
    return nullptr;
}

void WorldDescription::mount(std::string_view p_id, std::string_view p_robot)
{
    std::string const robot_id =
        p_robot.empty() ? robot().id : std::string(p_robot);
    EntitySpec const* owner = find(robot_id);
    if (owner == nullptr || owner->type != EntitySpec::Type::Robot)
    {
        throw std::invalid_argument("World '" + id + "' has no robot '" +
                                    robot_id + "'");
    }
    EntitySpec* chosen = nullptr;
    if (!p_id.empty())
    {
        for (EntitySpec& spec : entities)
        {
            if (spec.id == p_id && spec.type == EntitySpec::Type::Tool)
            {
                chosen = &spec;
            }
        }
        if (chosen == nullptr)
        {
            throw std::invalid_argument("World '" + id + "' has no tool '" +
                                        std::string(p_id) + "'");
        }
    }
    for (EntitySpec& spec : entities)
    {
        if (spec.type == EntitySpec::Type::Tool && spec.attach &&
            (spec.attach->parent == robot_id || &spec == chosen))
        {
            spec.attach.reset();
        }
    }
    if (chosen != nullptr)
    {
        chosen->attach = Attachment{
            robot_id, FLANGE, TOOL_MOUNT, Pose{ Vector3::ZERO, Quaternion{} }
        };
    }
}


char const* toString(Fidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case Fidelity::Abstract:
            return "abstract";
        case Fidelity::Rigid:
            return "rigid";
        case Fidelity::Contact:
            return "contact";
    }
    return "?";
}

// ----------------------------------------------------------------------------
char const* toString(PerceptionFidelity p_fidelity)
{
    switch (p_fidelity)
    {
        case PerceptionFidelity::Sensed:
            return "sensed";
        case PerceptionFidelity::NoisyTruth:
            return "noisy_truth";
        case PerceptionFidelity::Truth:
            return "truth";
    }
    return "?";
}

// ----------------------------------------------------------------------------
PerceptionFidelity fidelityOf(PerceptionSpec const& p_perception,
                              std::string_view p_id,
                              bool p_blind)
{
    auto const found = p_perception.overrides.find(p_id);
    PerceptionFidelity const level =
        found == p_perception.overrides.end() ? p_perception.fidelity : found->second;
    return level == PerceptionFidelity::Sensed && p_blind ? p_perception.blind : level;
}

} // namespace robotik
