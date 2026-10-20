// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file FlyMission.hpp
/// @brief The fly as a regular plugin: its flight model is the backend of
/// the robot, its brain is the agent the mission runs.
///
/// @code
/// Simulation::step
///   ├── robot step ── FlyBackend: one FlyEnvironment step with the last
///   │                 command, then the thorax pose and the posture
///   └── mission step ── FlyMission: the brain reads the new observation
///                       and gives the next command
/// @endcode
#pragma once

#include "FlyBrain.hpp"
#include "FlyEnvironment.hpp"

#include "Robotik/Robot/Backends/RobotBackend.hpp"
#include "Robotik/Task/Mission.hpp"

#include "Compages/World/Entity.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <vector>

// ****************************************************************************
/// @brief Flight dynamics of the fly, as a robot backend. The environment
/// integrates the thorax and the posture; the backend copies them onto the
/// robot (base pose, joint positions).
// ****************************************************************************
class FlyBackend final: public robotik::RobotBackend
{
public:

    explicit FlyBackend(FlyEnvironment& p_environment);

    /// @brief Action applied at the next environment step.
    void command(FlyAction const& p_action)
    {
        m_command = p_action;
    }

    /// @brief Observation written by the last step (or the reset).
    [[nodiscard]] std::span<float const> observation() const
    {
        return m_observation;
    }

    [[nodiscard]] std::span<float> observation()
    {
        return m_observation;
    }

    /// @brief The episode ended: the fly no longer moves.
    [[nodiscard]] bool done() const
    {
        return m_done;
    }

    [[nodiscard]] double episodeReturn() const
    {
        return m_return;
    }

    /// @brief Starts over (the environment was reset by the mission).
    void restart();

    void attach(robotik::Robot& /*p_robot*/) override {}
    void reset(robotik::Robot& p_robot) override;
    void step(robotik::Robot& p_robot, robotik::Seconds p_dt) override;

private:

    void pose(robotik::Robot& p_robot) const;

    FlyEnvironment& m_environment;
    std::array<float, FlyObservation::SIZE> m_observation{};
    FlyAction m_command{};
    double m_lag = 0.0;
    double m_return = 0.0;
    bool m_done = false;
};

// ****************************************************************************
/// @brief Builds the fly world from the scenario, plugs @ref FlyBackend and
/// runs the brain. Draws the three eye beams and the trail when rendered.
// ****************************************************************************
class FlyMission final: public robotik::Mission
{
public:

    void setup(robotik::Cell& p_cell,
               robotik::SceneView* p_view) override;
    void reset(robotik::Cell& p_cell,
               robotik::Seed p_seed) override;
    void step(robotik::Cell& p_cell, robotik::Seconds p_dt) override;
    void measure(robotik::Cell const& p_cell,
                 robotik::Metrics& p_metrics) const override;
    [[nodiscard]] robotik::Status
    status(robotik::Cell const& p_cell) const override;

    // -------------------------------------------------------------------------
    /// @brief Swaps the 13-neuron circuit for the FlyWire connectome (or
    /// back). Keeps the current brain when the tables cannot be read.
    /// @throws std::runtime_error with the reason.
    // -------------------------------------------------------------------------
    void useConnectome(bool p_connectome);

    // -------------------------------------------------------------------------
    /// @brief Builds a brain without touching the mission: safe on another
    /// thread. Loading the connectome reads large files (seconds) from
    /// @p_tables (see @ref tables).
    /// @throws std::runtime_error when the tables cannot be read.
    // -------------------------------------------------------------------------
    [[nodiscard]] static std::unique_ptr<FlyBrain>
    buildBrain(bool p_connectome,
               float p_cruise,
               std::filesystem::path const& p_tables);

    /// @brief Folder of the FlyWire tables: @c flywire/ in the pack
    /// @c robotik.fly. Empty before @c setup.
    [[nodiscard]] std::filesystem::path const& tables() const
    {
        return m_tables;
    }

    /// @brief Cruise height given to a new brain.
    [[nodiscard]] float cruise() const;

    /// @brief Replaces the brain and restarts the episode.
    void installBrain(std::unique_ptr<FlyBrain> p_brain);

    [[nodiscard]] FlyBrain const* brain() const
    {
        return m_brain.get();
    }

    [[nodiscard]] FlyEnvironment const* environment() const
    {
        return m_environment.get();
    }

    [[nodiscard]] FlyAction const& action() const
    {
        return m_action;
    }

private:

    void decide();
    void drawBeamsAndTrail();

    std::unique_ptr<FlyEnvironment> m_environment;
    std::unique_ptr<FlyBrain> m_brain;
    FlyBackend* m_backend = nullptr;
    robotik::Cell* m_simulation = nullptr;
    std::filesystem::path m_tables;
    FlyBrain::Kind m_kind = FlyBrain::Kind::Shiu;
    robotik::Seed m_seed;
    FlyAction m_action{};
    double m_dt = 0.01;
    bool m_collided = false;
    std::array<compages::world::Entity, 3> m_beams;
    std::vector<compages::world::Entity> m_trail;
};
