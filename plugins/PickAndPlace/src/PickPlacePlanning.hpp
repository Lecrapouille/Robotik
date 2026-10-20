// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PickPlacePlanning.hpp
/// Links the PDDL domain of pick and place
/// (scenarios/pick_and_place.domain.pddl) to the beliefs and the skills.
#pragma once

namespace robotik
{
class PlanningDomain;
} // namespace robotik

// ----------------------------------------------------------------------------
/// Observers of seen, holding, hand_empty and in; bindings of detect,
/// pick and place to the skills of @ref PickPlaceMission.
// ----------------------------------------------------------------------------
void registerPlanning(robotik::PlanningDomain& p_domain);
