// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlacePlanning.hpp"

#include "PickPlaceSkills.hpp"

#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/RobotContext.hpp"

#include <cmath>

/// A belief older than this says nothing about where the object is now: the
/// camera may be down since.
static constexpr double FRESH_BELIEF = 0.5;

void registerPlanning(robotik::PlanningDomain& p_domain)
{
    using robotik::Args;
    using robotik::RobotContext;
    using robotik::Truth;

    // --- State, from the beliefs only --------------------------------------
    p_domain.observe("seen",
                     [](RobotContext const& p_context, Args p_args)
                     {
                         robotik::WorldObject const* object =
                             p_context.world_model.find(p_args[0]);
                         return robotik::truth(object != nullptr &&
                                               observed(*object));
                     });
    p_domain.observe("holding",
                     [](RobotContext const& p_context, Args p_args)
                     {
                         return robotik::truth(
                             robotik::heldObject(p_context.robot) == p_args[0]);
                     });
    p_domain.observe(
        "hand_empty",
        [](RobotContext const& p_context, Args)
        {
            if (robotik::findGripper(p_context.robot, {}) == nullptr)
            {
                return Truth::Unknown;
            }
            return robotik::truth(robotik::heldObject(p_context.robot).empty());
        });
    // Inside the footprint of the bin, not held. Height is ignored: right
    // after the release the object is still falling. Not observed lately:
    // unknown, not false.
    p_domain.observe(
        "in",
        [](RobotContext const& p_context, Args p_args)
        {
            robotik::WorldObject const* object =
                p_context.world_model.find(p_args[0]);
            robotik::WorldObject const* bin =
                p_context.world_model.find(p_args[1]);
            if (object == nullptr || bin == nullptr)
            {
                return Truth::Unknown;
            }
            if (robotik::heldObject(p_context.robot) == p_args[0])
            {
                return Truth::False;
            }
            if (!observed(*object) ||
                (p_context.time - object->seen).value() > FRESH_BELIEF)
            {
                return Truth::Unknown;
            }
            bool const inside = std::abs(object->position.x - bin->position.x) <
                                    0.5 * bin->size.x &&
                                std::abs(object->position.y - bin->position.y) <
                                    0.5 * bin->size.y;
            return robotik::truth(inside);
        });

    // --- Actions, as skills of PickPlaceMission -----------------------------
    p_domain.bind("detect", { "Detect({0})" });
    p_domain.bind(
        "pick",
        { "Approach({0})", "Reach({0})", "Grasp({0})", "Approach({0})" });
    p_domain.bind("place", { "Transit({1})", "Approach({1})", "Release" });
}
