// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Simulation.hpp
//! @brief Runs a YAML scenario: robot, world, perception, skills, faults,
//! behavior tree and assertions. Headless unless a @ref SceneView is given.
#pragma once

#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Scheduler.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <memory>
#include <string>
#include <vector>

namespace compages::world
{
class World;
}

namespace robotik
{

// ****************************************************************************
//! @brief One scenario instance, replayable from a seed.
//!
//! Each @ref step: faults, behavior tree tick (requests skills), scheduler
//! update (admits and ticks skills), robot step (physics, sensors, perception
//! into the @ref WorldModel), simulated suction. Without a camera image
//! source, an oracle feeds the world model with the ground truth while a
//! camera resource is available.
//!
//! @code
//! compages::world::World world;
//! robotik::Simulation sim(world,
//!     robotik::Scenario::load("data/scenarios/pick_and_place.yml"));
//! sim.reset(robotik::Seed{ 7 });
//! while (!sim.finished())
//!     sim.step(Seconds(0.01));
//! for (auto const& check : sim.checks())
//!     if (!check.passed) { ... }
//! @endcode
// ****************************************************************************
class Simulation
{
public:

    struct Check
    {
        std::string text;
        bool passed = false;
    };

    Simulation(Simulation const&) = delete;
    Simulation& operator=(Simulation const&) = delete;

    //! @param p_view Rendering hooks, or null for a headless run.
    Simulation(compages::world::World& p_world,
               Scenario p_scenario,
               SceneView* p_view = nullptr);
    ~Simulation();

    // -------------------------------------------------------------------------
    //! @brief Starts a new episode: same seed, same episode.
    //!
    //! @p_seed derives the object layout ("world"), the faults ("faults") and
    //! the sensor noise (one stream per sensor name).
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    //! @brief New episode with the scenario seed.
    void reset()
    {
        reset(Seed{ m_scenario.seed });
    }

    void step(Seconds p_dt);

    [[nodiscard]] Scenario const& scenario() const
    {
        return m_scenario;
    }

    [[nodiscard]] Seed seed() const
    {
        return m_seed;
    }

    [[nodiscard]] Seconds time() const
    {
        return m_robot->time();
    }

    [[nodiscard]] RobotSession& robot() const
    {
        return *m_robot;
    }

    [[nodiscard]] WorldModel& worldModel()
    {
        return m_world_model;
    }

    [[nodiscard]] WorldModel const& worldModel() const
    {
        return m_world_model;
    }

    //! @brief Detectors run on every camera frame (empty by default).
    [[nodiscard]] PerceptionPipeline& perception()
    {
        return m_perception;
    }

    [[nodiscard]] PerceptionPipeline const& perception() const
    {
        return m_perception;
    }

    [[nodiscard]] SkillScheduler& skills() const
    {
        return *m_scheduler;
    }

    [[nodiscard]] FaultInjector& faults()
    {
        return m_faults;
    }

    [[nodiscard]] RobotContext& context()
    {
        return m_context;
    }

    //! @brief First camera of the robot, or null.
    [[nodiscard]] Camera* camera() const;

    //! @brief Behavior tree, or null if the scenario defines none.
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_tree.get();
    }

    [[nodiscard]] bt::Status status() const
    {
        return m_status;
    }

    [[nodiscard]] bool finished() const
    {
        return m_status == bt::Status::SUCCESS ||
               m_status == bt::Status::FAILURE;
    }

    //! @brief Peak contact count since the last reset.
    [[nodiscard]] int maxContacts() const
    {
        return m_max_contacts;
    }

    //! @brief Evaluates the @c assert entries of the scenario.
    [[nodiscard]] std::vector<Check> checks() const;

private:

    void spawn(SceneView* p_view);
    void addSkills();
    void loadTree();
    void observe();

private:

    compages::world::World& m_world;
    Scenario m_scenario;
    std::unique_ptr<RobotSession> m_robot;
    WorldModel m_world_model;
    PerceptionPipeline m_perception;
    std::unique_ptr<SkillScheduler> m_scheduler;
    FaultInjector m_faults;
    RobotContext m_context;
    //!< Object entities, in scenario order.
    std::vector<compages::world::Entity> m_objects;
    //!< True when no camera has an image source.
    bool m_oracle = true;

    bt::NodeFactory m_factory;
    bt::Blackboard::Ptr m_blackboard;
    bt::Tree::Ptr m_tree;
    bt::Status m_status = bt::Status::INVALID;

    Seed m_seed;
    int m_max_contacts = 0;
};

} // namespace robotik
