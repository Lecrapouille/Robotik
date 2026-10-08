// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Fly closed loop: sensors, brain, action, body.
// Without --view there is no window: the same episode can run in N copies.

#include "FlyBrain.hpp"
#include "FlyEnvironment.hpp"
#include "FlyTrace.hpp"
#include "FlyView.hpp"

#include "Robotik/Environment/Environment.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

//! @brief Command line. The scenario seed is used until --seed replaces it.
struct Options
{
    std::filesystem::path scenario = "data/scenarios/fly_obstacle_avoidance.yml";
    std::filesystem::path edges;
    std::filesystem::path binding;
    std::filesystem::path record;
    std::filesystem::path replay;
    std::uint64_t seed = 0;
    bool seed_set = false;
    bool view = false;
    bool reflex = false;
    std::size_t envs = 1;
    std::size_t threads = 0;
    std::size_t episodes = 1;
};

//! @brief Fills @p_options. False prints the usage: unknown flag, missing
//! value, --edges without --binding, or a zero count.
bool parse(int p_argc, char** p_argv, Options& p_options)
{
    for (int i = 1; i < p_argc; ++i)
    {
        std::string const arg = p_argv[i];
        auto next = [&]() -> char const* { return i + 1 < p_argc ? p_argv[++i] : nullptr; };
        char const* value = nullptr;
        if (arg == "--headless")
        {
            p_options.view = false;
        }
        else if (arg == "--view")
        {
            p_options.view = true;
        }
        else if (arg == "--scenario" && (value = next()))
        {
            p_options.scenario = value;
        }
        else if (arg == "--seed" && (value = next()))
        {
            p_options.seed = std::strtoull(value, nullptr, 10);
            p_options.seed_set = true;
        }
        else if (arg == "--brain" && (value = next()))
        {
            if (value == std::string("reflex"))
            {
                p_options.reflex = true;
            }
            else if (value != std::string("shiu"))
            {
                std::cerr << "Cerveau inconnu : " << value << " (reflex|shiu)\n";
                return false;
            }
        }
        else if (arg == "--edges" && (value = next()))
        {
            p_options.edges = value;
        }
        else if (arg == "--binding" && (value = next()))
        {
            p_options.binding = value;
        }
        else if (arg == "--envs" && (value = next()))
        {
            p_options.envs = std::strtoul(value, nullptr, 10);
        }
        else if (arg == "--threads" && (value = next()))
        {
            p_options.threads = std::strtoul(value, nullptr, 10);
        }
        else if (arg == "--episodes" && (value = next()))
        {
            p_options.episodes = std::strtoul(value, nullptr, 10);
        }
        else if (arg == "--record" && (value = next()))
        {
            p_options.record = value;
        }
        else if (arg == "--replay" && (value = next()))
        {
            p_options.replay = value;
        }
        else if (arg == "--help" || arg == "-h")
        {
            return false;
        }
        else
        {
            std::cerr << "Option inconnue : " << arg << '\n';
            return false;
        }
    }
    if (!p_options.edges.empty() && p_options.binding.empty())
    {
        std::cerr << "--edges demande aussi --binding\n";
        return false;
    }
    return p_options.envs > 0u && p_options.episodes > 0u;
}

//! @brief One line of flags, written on stderr.
void usage(char const* p_program)
{
    std::cerr << "Usage : " << p_program
              << " [--scenario file.yml] [--seed S] [--brain reflex|shiu]\n"
              << "       [--edges edges.csv --binding roles.txt]\n"
              << "       [--view] [--envs N] [--threads T] [--episodes E]\n"
              << "       [--record fly_run.json] [--replay fly_run.json]\n";
}

//! @brief Reflex or Shiu. An edge list replaces the embedded circuit and
//! forces the Shiu path inside loadConnectome.
FlyBrain makeBrain(Options const& p_options, FlyScenario const& p_scenario)
{
    FlyBrain::Kind const kind =
        p_options.reflex ? FlyBrain::Kind::Reflex : FlyBrain::Kind::Shiu;
    FlyBrain brain(kind, static_cast<float>(p_scenario.target.z));
    if (!p_options.edges.empty())
    {
        brain.loadConnectome(p_options.edges, p_options.binding);
    }
    return brain;
}

//! @brief One window, one world. A replay file replaces the brain. Recording
//! stores the observation that was read and the action that followed.
int runView(Options const& p_options, FlyScenario const& p_scenario)
{
    std::uint32_t const steps =
        static_cast<std::uint32_t>(p_scenario.horizon / p_scenario.dt);
    FlyEnvironment environment(p_scenario, steps);
    std::vector<float> observation(FlyObservation::SIZE);
    std::vector<float> action(FlyAction::SIZE);
    robotik::Seed const seed{ p_options.seed_set ? p_options.seed : p_scenario.seed };
    environment.reset(seed, observation);
    FlyBrain brain = makeBrain(p_options, p_scenario);
    brain.reset(seed);
    FlyTrace trace;
    trace.seed = seed.value;
    trace.dt = p_scenario.dt;
    FlyTrace replay;
    if (!p_options.replay.empty())
    {
        replay = FlyTrace::load(p_options.replay);
    }

    FlyView view(p_scenario);
    if (!view.open())
    {
        return EXIT_FAILURE;
    }
    bool reached = false;
    std::size_t cursor = 0;
    while (view.frame(environment))
    {
        // The window keeps drawing after the food, but the body stays put.
        if (reached)
        {
            continue;
        }
        FlyObservation const seen = FlyObservation::from(observation);
        FlyAction decided;
        if (!replay.actions.empty())
        {
            if (cursor >= replay.actions.size())
            {
                reached = environment.snapshot().reached;
                continue;
            }
            decided = replay.actions[cursor++];
        }
        else
        {
            decided = brain.update(seen, Seconds(p_scenario.dt));
        }
        decided.write(action);
        if (!p_options.record.empty())
        {
            trace.observations.push_back(seen);
            trace.actions.push_back(decided);
        }
        robotik::StepResult const result = environment.step(action, observation);
        if (result.done())
        {
            reached = result.terminated;
            std::cout << (reached ? "Target reached" : "Time up")
                      << " in " << environment.snapshot().steps << " steps, reward "
                      << environment.snapshot().reward << '\n';
        }
    }
    if (!p_options.record.empty())
    {
        trace.save(p_options.record);
        std::cout << "Trace written to " << p_options.record << '\n';
    }
    return reached ? EXIT_SUCCESS : EXIT_FAILURE;
}

//! @brief N worlds until @c episodes of them have finished. A finished world
//! restarts at once. Recording keeps only the first world's first episode.
int runHeadless(Options const& p_options, FlyScenario const& p_scenario)
{
    std::uint32_t const steps =
        static_cast<std::uint32_t>(p_scenario.horizon / p_scenario.dt);
    robotik::Seed const master{ p_options.seed_set ? p_options.seed : p_scenario.seed };
    std::size_t const threads = p_options.threads > 0u
                                    ? p_options.threads
                                    : std::max<std::size_t>(1u, std::thread::hardware_concurrency());

    // One brain per world. The pool only steps the bodies, so each brain is
    // reset with the seed of the episode that world just started.
    std::vector<FlyBrain> brains;
    brains.reserve(p_options.envs);
    for (std::size_t i = 0; i < p_options.envs; ++i)
    {
        brains.push_back(makeBrain(p_options, p_scenario));
    }
    FlyTrace replay;
    if (!p_options.replay.empty())
    {
        replay = FlyTrace::load(p_options.replay);
    }

    robotik::EnvironmentPool pool(
        p_options.envs,
        [&](std::size_t) { return std::make_unique<FlyEnvironment>(p_scenario, steps); },
        master,
        p_options.envs == 1u ? 1u : threads);
    pool.reset();
    for (std::size_t i = 0; i < pool.size(); ++i)
    {
        brains[i].reset(pool.seed(i));
    }

    FlyTrace trace;
    trace.seed = master.value;
    trace.dt = p_scenario.dt;
    std::size_t replay_cursor = 0;
    while (pool.episodes().size() < p_options.episodes)
    {
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            FlyObservation const seen = FlyObservation::from(pool.observation(i));
            FlyAction decided;
            if (!replay.actions.empty())
            {
                if (replay_cursor < replay.actions.size())
                {
                    decided = replay.actions[replay_cursor++];
                }
            }
            else
            {
                decided = brains[i].update(seen, Seconds(p_scenario.dt));
            }
            decided.write(pool.action(i));
            // The trace is the first episode of world 0, before any restart.
            if (i == 0u && !p_options.record.empty() && pool.episodes().empty())
            {
                trace.observations.push_back(seen);
                trace.actions.push_back(decided);
            }
        }
        pool.step();
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            if (pool.dones()[i] != 0)
            {
                brains[i].reset(pool.seed(i));
            }
        }
    }

    std::size_t successes = 0;
    auto const finished = pool.episodes();
    std::size_t const shown = std::min(finished.size(), p_options.episodes);
    for (std::size_t i = 0; i < shown; ++i)
    {
        if (finished[i].terminated)
        {
            ++successes;
        }
    }
    std::cout << successes << " / " << shown
              << " episodes reached the target (seed " << master.value << ", ";
    if (!p_options.replay.empty())
    {
        std::cout << "replay";
    }
    else if (brains.empty() || brains[0].kind() == FlyBrain::Kind::Reflex)
    {
        std::cout << "reflex brain";
    }
    else
    {
        std::cout << "Shiu brain";
        if (brains[0].connectome())
        {
            std::cout << ", connectome";
        }
    }
    std::cout << ")\n";
    if (!p_options.record.empty())
    {
        trace.save(p_options.record);
        std::cout << "Trace written to " << p_options.record << '\n';
    }
    return successes > 0u ? EXIT_SUCCESS : EXIT_FAILURE;
}

//! @brief Replay forces a single world. --view opens the window, otherwise
//! the pool runs with no OpenGL.
int main(int p_argc, char** p_argv)
{
    Options options;
    if (!parse(p_argc, p_argv, options))
    {
        usage(p_argv[0]);
        return EXIT_FAILURE;
    }
    try
    {
        FlyScenario scenario = FlyScenario::load(options.scenario);
        if (!options.replay.empty())
        {
            options.envs = 1;
        }
        if (options.view)
        {
            return runView(options, scenario);
        }
        return runHeadless(options, scenario);
    }
    catch (std::exception const& failure)
    {
        std::cerr << failure.what() << '\n';
        return EXIT_FAILURE;
    }
}
