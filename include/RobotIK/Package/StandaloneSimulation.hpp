// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file StandaloneSimulation.hpp
/// The shortest way to run a scenario without a window: one call, no world
/// to create.
///
/// A @ref robotik::Simulation lives in an ECS world (Compages) that the
/// simulator shares with its rendering, and its scenario logic often comes
/// from a plugin library. A headless program (the @c RobotIK-Headless host,
/// a test, the application of a demo) does not draw anything:
/// @ref robotik::StandaloneSimulation creates the world, loads the plugin
/// of the scenario when there is one, and keeps both with the simulation,
/// in the order they must be destroyed. It is the third layer of RobotIK,
/// next to @ref robotik::StandaloneRobot (one robot) and
/// @ref robotik::StandaloneCell (a cell in C++).
///
/// @code
/// PickPlaceLogic logic;
/// auto run = robotik::StandaloneSimulation::open(
///     "scenarios/pick_and_place.scenario.yaml", &logic);
/// if (!run)
///     return fail(run.error());          // a bad file is a message
/// robotik::Simulation& simulation = run.value().simulation();
/// while (!simulation.finished())
///     simulation.step(robotik::Seconds(0.01));
/// std::cout << (simulation.passed() ? "pass" : "fail") << "\n";
///
/// // The scenario logic of a plugin package: the library is found from the
/// // file, loaded, and closed with the simulation.
/// auto run = robotik::StandaloneSimulation::openPlugin(
///     "lib/robotik/plugins/HelloWorld/scenarios/reach.scenario.yaml");
/// while (!run.value().simulation().finished())
///     run.value().step(robotik::Seconds(0.01)); // the plugin is called too
/// @endcode
///
/// @par Contract
/// - Promises: @c open returns a @c Result: a scenario file that cannot be
///   read, a world that cannot be built or an assertion that does not
///   compile is a message, not an exception. The world lives longer than
///   the simulation, whatever the caller does: both are private.
/// - Expects: a scenario logic given by @c open outlives the simulation
///   (not owned); the one of a plugin (@c openPlugin) is owned.
/// - Order: the simulation is destroyed first, then the plugin releases
///   its scenario logic and its library is closed, then the world.
#pragma once

#include "RobotIK/Common/Result.hpp"
#include "RobotIK/Package/Plugin/PluginSession.hpp"
#include "RobotIK/Package/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <filesystem>
#include <functional>
#include <memory>

namespace robotik
{

// ****************************************************************************
/// A simulation alone, with the ECS world it lives in, headless (see the
/// file comment).
///
/// Why a class: the simulation keeps a reference to the world, so the
/// world must be destroyed after it. Both are private, the world is
/// declared first (so destroyed last), and the class moves but is not
/// assigned (an assignment would destroy the old world before the old
/// simulation).
// ****************************************************************************
class StandaloneSimulation final
{
public:

    // ************************************************************************
    /// How @ref openPlugin finds the plugin and reads the scenario. Every
    /// field has a default: the standard plugin folders, the scenario file
    /// as it is.
    // ************************************************************************
    struct PluginSetup
    {
        /// One more package, or folder of packages, to search before the
        /// standard ones (@c ROBOTIK_PLUGINS, @c lib/robotik/plugins);
        /// empty for none.
        std::filesystem::path plugins;
        /// Reads the scenario with the asset packs of the plugin package (an
        /// experiment case applies its overrides here); empty reads the
        /// scenario file as it is.
        std::function<Result<Scenario>(AssetResolver const&)> scenario;
    };

    // -------------------------------------------------------------------------
    /// Builds the simulation of a scenario and its world, headless.
    /// @param p_scenario Scenario to run (read from a file, or built in C++).
    /// @param p_logic Scenario logic (skills, detectors, metrics), or null;
    ///   not owned: it must outlive the simulation.
    /// @return The simulation, reset with the scenario seed, or why it could
    ///   not be built.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<StandaloneSimulation>
    open(Scenario p_scenario, ScenarioLogic* p_logic = nullptr);

    // -------------------------------------------------------------------------
    /// Same, from a scenario file; its models are found with
    /// @ref AssetResolver::standard.
    /// @param p_file Scenario file (@c x.scenario.yaml).
    /// @param p_logic Scenario logic, or null; not owned.
    /// @return The simulation, or why the file or the world is wrong.
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<StandaloneSimulation>
    open(std::filesystem::path const& p_file, ScenarioLogic* p_logic = nullptr);

    // -------------------------------------------------------------------------
    /// Builds the simulation of a scenario of a plugin package: finds the
    /// package of @p_file, loads its library, creates its scenario logic,
    /// builds the simulation and starts the plugin. The library is owned:
    /// it is closed with the simulation.
    /// @param p_file Scenario file of a plugin package.
    /// @param p_setup Where else to search, how to read the scenario.
    /// @return The running simulation, or why the plugin, the file or the
    ///   world is wrong (a file outside a package, a missing dependency, a
    ///   plugin that refuses).
    // -------------------------------------------------------------------------
    [[nodiscard]] static Result<StandaloneSimulation>
    openPlugin(std::filesystem::path const& p_file, PluginSetup p_setup);

    /// @copydoc openPlugin(std::filesystem::path const&, PluginSetup)
    /// With the default @ref PluginSetup.
    [[nodiscard]] static Result<StandaloneSimulation>
    openPlugin(std::filesystem::path const& p_file);

    // -------------------------------------------------------------------------
    /// One step: the simulation moves by @p_dt (unless the plugin owns the
    /// clock, @c host.clock: @c plugin), then the plugin is called
    /// (@c Plugin::update). Without a plugin it is
    /// @c simulation().step(p_dt).
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    StandaloneSimulation(StandaloneSimulation&&) = default;
    StandaloneSimulation& operator=(StandaloneSimulation&&) = delete;
    StandaloneSimulation(StandaloneSimulation const&) = delete;
    StandaloneSimulation& operator=(StandaloneSimulation const&) = delete;
    ~StandaloneSimulation();

    /// @return The simulation: step, verdict, report, the cell.
    [[nodiscard]] Simulation& simulation()
    {
        return *m_simulation;
    }

    /// @copydoc simulation()
    [[nodiscard]] Simulation const& simulation() const
    {
        return *m_simulation;
    }

    /// @return The plugin of the scenario (events, keys, its clock), or null
    ///   when the scenario logic was given to @ref open.
    [[nodiscard]] PluginSession* plugin()
    {
        return m_plugin.get();
    }

    /// @return The ECS world of the simulation (its entities).
    [[nodiscard]] compages::world::World& getWorld()
    {
        return *m_world;
    }

private:

    StandaloneSimulation();

    /// ECS world holding the entities; declared first, destroyed last.
    std::unique_ptr<compages::world::World> m_world;
    /// The plugin library and its scenario logic, or null; destroyed after
    /// the simulation that points to the scenario logic.
    std::unique_ptr<PluginSession> m_plugin;
    /// The simulation.
    std::unique_ptr<Simulation> m_simulation;
};

} // namespace robotik
