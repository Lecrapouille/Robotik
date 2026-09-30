// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file HomeSkill.hpp
 * @brief Moves all joints to their @ref ecs::HomePosition.
 */

#pragma once

#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

/**
 * @brief Commands every joint with @ref ecs::HomePosition to that value.
 */
class HomeSkill final: public Skill
{
public:

    /**
     * @brief Constructs the skill.
     * @param p_tolerance Max |error| in rad or m to declare success.
     */
    explicit HomeSkill(double p_tolerance = 1e-2) : m_tolerance(p_tolerance) {}

    Status tick(RobotContext& p_context, double p_dt) override;

private:

    /** @brief Position error threshold for success. */
    double m_tolerance;
};

} // namespace robotik
