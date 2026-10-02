//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
//! @file HomeSkill.hpp
//! @brief Moves all joints to their @ref ecs::HomePosition.
//=============================================================================

#pragma once

#include "Robotik/Skills/Skill.hpp"

namespace robotik
{

// ****************************************************************************
//! @brief Commands every joint with @ref ecs::HomePosition to that value.
// ****************************************************************************
class HomeSkill final: public Skill
{
public:

    // -------------------------------------------------------------------------
    //! @brief Constructs the skill.
    //! @param p_tolerance Max |error| in rad or m to declare success.
    // -------------------------------------------------------------------------
    explicit HomeSkill(double p_tolerance = 1e-2) noexcept
        : m_tolerance(p_tolerance)
    {
    }

    Status tick(RobotContext& p_context, Seconds p_dt) override;

private:

    //!< Position error threshold for success (units: rad or m).
    double m_tolerance;
};

} // namespace robotik
