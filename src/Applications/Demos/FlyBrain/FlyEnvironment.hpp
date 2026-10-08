// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// @file FlyEnvironment.hpp
// @brief One fly, its arena and its sensors.
//
// The brain is not in here: @ref FlyEnvironment::step only applies an action.
// Headless: no window and no URDF.
#pragma once

#include "FlyController.hpp"
#include "FlyTypes.hpp"
#include "FlyVision.hpp"

#include "Robotik/Environment/Environment.hpp"

#include <vector>

// ****************************************************************************
//! @brief Everything a viewer or a log needs after one step.
// ****************************************************************************
struct FlySnapshot
{
    //!< Thorax after the step.
    FlyPlant plant;
    //!< What the brain just read.
    FlyObservation observation;
    //!< Action that produced this step.
    FlyAction action;
    //!< Joint targets that go with @ref action.
    FlyPosture posture;
    //!< Eye samples and the beam ends.
    FlyVision vision;
    //!< True when the body was pushed out of an obstacle or the arena wall.
    bool collided = false;
    //!< True when the thorax is at the food.
    bool reached = false;
    //!< Reward of this step.
    double reward = 0.0;
    //!< Steps since @ref FlyEnvironment::reset.
    std::uint32_t steps = 0;
};

// ****************************************************************************
//! @brief The fly world an agent acts in.
//!
//! Same seed, same actions: same observations. Obstacle jitter comes from
//! the seed derived as @c "world", sensor noise from @c "sensors".
// ****************************************************************************
class FlyEnvironment final: public robotik::Environment
{
public:

    // ------------------------------------------------------------------------
    //! @brief Keeps @p_scenario. A zero @p_max_steps uses horizon / dt.
    // ------------------------------------------------------------------------
    FlyEnvironment(FlyScenario p_scenario, std::uint32_t p_max_steps);

    // ------------------------------------------------------------------------
    //! @brief @ref FlyObservation::SIZE.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::size_t observationSize() const override
    {
        return FlyObservation::SIZE;
    }

    // ------------------------------------------------------------------------
    //! @brief @ref FlyAction::SIZE.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::size_t actionSize() const override
    {
        return FlyAction::SIZE;
    }

    // ------------------------------------------------------------------------
    //! @brief Jitters the obstacles, parks the fly and writes the first
    //! observation.
    // ------------------------------------------------------------------------
    void reset(robotik::Seed p_seed, std::span<float> p_observation) override;

    // ------------------------------------------------------------------------
    //! @brief Applies @p_action, then writes the next observation.
    //!
    //! Terminated when the food is reached. Truncated at the step limit.
    // ------------------------------------------------------------------------
    robotik::StepResult step(std::span<float const> p_action,
                             std::span<float> p_observation) override;

    // ------------------------------------------------------------------------
    //! @brief File the episode was built from, without the jitter.
    // ------------------------------------------------------------------------
    [[nodiscard]] FlyScenario const& scenario() const
    {
        return m_scenario;
    }

    // ------------------------------------------------------------------------
    //! @brief State after the last @ref reset or @ref step.
    // ------------------------------------------------------------------------
    [[nodiscard]] FlySnapshot const& snapshot() const
    {
        return m_snapshot;
    }

    // ------------------------------------------------------------------------
    //! @brief Obstacles after the jitter of the current episode.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::span<FlyBox const> obstacles() const
    {
        return m_obstacles;
    }

    // ------------------------------------------------------------------------
    //! @brief Thorax positions kept for the trail, oldest first.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::span<robotik::Vector3 const> trail() const
    {
        return m_trail;
    }

    // ------------------------------------------------------------------------
    //! @brief Step length (SI: s), from the scenario.
    // ------------------------------------------------------------------------
    [[nodiscard]] double dt() const
    {
        return m_scenario.dt;
    }

private:

    // ------------------------------------------------------------------------
    //! @brief Fills the vision and the rest of the observation in the snapshot.
    // ------------------------------------------------------------------------
    void sense();

    // ------------------------------------------------------------------------
    //! @brief Pushes the thorax out of walls and boxes. Sets @p_collided.
    // ------------------------------------------------------------------------
    void separate(bool& p_collided);

    // ------------------------------------------------------------------------
    //! @brief Copies the snapshot observation into @p_observation.
    // ------------------------------------------------------------------------
    void write(std::span<float> p_observation) const;

    //!< Scenario as loaded, before jitter.
    FlyScenario m_scenario;
    //!< Step limit of one episode.
    std::uint32_t m_max_steps;
    //!< Obstacles of the current episode.
    std::vector<FlyBox> m_obstacles;
    //!< Body that integrates the action.
    FlyController m_controller;
    //!< Last step.
    FlySnapshot m_snapshot;
    //!< Sensor noise, reseeded on @ref reset.
    robotik::Random m_noise{ robotik::Seed{} };
    //!< Thorax samples, capped.
    std::vector<robotik::Vector3> m_trail;
    //!< Horizontal distance to the food before this step, for the reward.
    double m_previous_distance = 0.0;
};
