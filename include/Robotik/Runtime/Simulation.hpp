//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
//! @file Simulation.hpp
//! @brief Runs a YAML scenario: world spawn, behavior tree, and assertions.
//=============================================================================

#pragma once

#include "Robotik/Behavior/SkillNodes.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/World/Entity.hpp"

#include <memory>
#include <string>
#include <vector>

namespace compages::renderer
{
class Scene;
}

namespace compages::world
{
struct ViewFrame;
}

namespace robotik
{

class RobotRuntime;

// ****************************************************************************
//! @brief One scenario instance: robot, objects, optional camera, and behavior
//! tree.
//!
//! Skills are registered as BlackThorn actions and ticked once per @ref step.
//! Pass a null scene for headless runs (no meshes, detection may be skipped).
//!
//! @example
//! @code
//! compages::world::World world;
//! compages::renderer::Scene scene(world);
//! robotik::Simulation sim(world, &scene,
//!     robotik::Scenario::load("data/scenarios/pick_and_place.yml"));
//! while (!sim.finished())
//!     sim.step(Seconds(0.01));
//! for (auto const& c : sim.checks())
//!     if (!c.passed) { ... }
//! @endcode
// ****************************************************************************
class Simulation
{
public:

    Simulation(Simulation const&) = delete;
    Simulation& operator=(Simulation const&) = delete;

    // -------------------------------------------------------------------------
    //! @brief Result of evaluating one scenario assertion string.
    // -------------------------------------------------------------------------
    struct Check
    {
        //!< Assertion text from the scenario file.
        std::string text;
        //!< Whether the assertion holds at evaluation time.
        bool passed = false;
    };

    // -------------------------------------------------------------------------
    //! @brief Builds runtime, spawns scenario content, and loads the behavior
    //! tree.
    //! @param p_world World that will own robot and object entities.
    //! @param p_scene Scene for URDF and primitives, or null for headless.
    //! @param p_scenario Parsed scenario (paths already resolved where needed).
    // -------------------------------------------------------------------------
    Simulation(compages::world::World& p_world,
               compages::renderer::Scene* p_scene,
               Scenario p_scenario);

    // -------------------------------------------------------------------------
    //! @brief Releases runtime and behavior tree resources.
    // -------------------------------------------------------------------------
    ~Simulation();

    // -------------------------------------------------------------------------
    // @brief Tick BT and physics with a fixed @p_dt.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    //! @brief Tick BT and physics; update world from @p_frame.
    // -------------------------------------------------------------------------
    void step(compages::world::ViewFrame const& p_frame);

    // -------------------------------------------------------------------------
    //! @brief Scenario description loaded at construction.
    // -------------------------------------------------------------------------
    [[nodiscard]] Scenario const& scenario() const
    {
        return m_scenario;
    }

    // -------------------------------------------------------------------------
    //! @brief Underlying robot runtime (kinematics, simulation, time).
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotRuntime& runtime()
    {
        return *m_runtime;
    }

    // -------------------------------------------------------------------------
    //! @brief Behavior tree instance, or null if the scenario defines none.
    // -------------------------------------------------------------------------
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_tree.get();
    }

    // -------------------------------------------------------------------------
    //! @brief Root status after the last tick.
    // -------------------------------------------------------------------------
    [[nodiscard]] bt::Status status() const
    {
        return m_status;
    }

    // -------------------------------------------------------------------------
    //! @brief True when the tree returned SUCCESS or FAILURE.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool finished() const
    {
        return m_status == bt::Status::SUCCESS ||
               m_status == bt::Status::FAILURE;
    }

    // -------------------------------------------------------------------------
    //! @brief Skill execution log for debugging and UI timelines.
    // -------------------------------------------------------------------------
    [[nodiscard]] SkillTrace const& trace() const
    {
        return m_trace;
    }

    // -------------------------------------------------------------------------
    //! @brief Root entity of the loaded robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity robot() const
    {
        return m_robot;
    }

    // -------------------------------------------------------------------------
    //! @brief Wrist or scenario camera entity; invalid if none configured.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity camera() const
    {
        return m_camera;
    }

    // -------------------------------------------------------------------------
    //! @brief Peak MuJoCo contact count seen since start.
    // -------------------------------------------------------------------------
    [[nodiscard]] int maxContacts() const
    {
        return m_max_contacts;
    }

    // -------------------------------------------------------------------------
    //! @brief Evaluates @c assert entries from the scenario file.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<Check> checks() const;

private:

    void spawn(compages::renderer::Scene* p_scene);
    void buildTree();
    void tick(Seconds p_dt);

private:

    //!< ECS world (not owned).
    compages::world::World& m_world;
    //!< Frozen copy of the loaded scenario.
    Scenario m_scenario;
    //!< Pinocchio + MuJoCo runtime for the robot URDF.
    std::unique_ptr<RobotRuntime> m_runtime;
    //!< Context whose time/dt are updated each tick for skills.
    std::unique_ptr<RobotContext> m_context;
    //!< Robot root entity after spawn.
    compages::world::Entity m_robot;
    //!< Optional onboard camera entity.
    compages::world::Entity m_camera;
    //!< Factory used to instantiate BT action nodes from skills.
    bt::NodeFactory m_factory;
    //!< Blackboard shared by the loaded tree.
    bt::Blackboard::Ptr m_blackboard;
    //!< Loaded behavior tree.
    bt::Tree::Ptr m_tree;
    //!< Last root tick status.
    bt::Status m_status = bt::Status::INVALID;
    //!< Append-only log of skill runs.
    SkillTrace m_trace;
    //!< Maximum @c ncon observed in MuJoCo.
    int m_max_contacts = 0;
};

} // namespace robotik
