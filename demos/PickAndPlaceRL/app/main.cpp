// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Pick and place as a reinforcement learning environment: N simulations step
// in parallel behind one flat action / observation buffer, episodes restart
// on their own with seeds derived from a master seed, and a run replays
// identically whatever the number of threads.
//
// Two policies share the same pool: random (noise) and converged (scripted).
// --train raises a mix from random toward converged, like the Simulator.

#include "PickPlaceEnvironment.hpp"

#include "Robotik/Environment/Environment.hpp"
#include "Robotik/Math/Random.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#define CUBE_SPREAD_M 0.06
#define MAX_STEPS 100u

struct Options
{
    std::filesystem::path scenario = "demos/PickAndPlaceBT/scenarios/pick_and_place.yml";
    std::size_t envs = 8;
    std::size_t threads = 0;
    std::size_t episodes = 32;
    std::uint64_t seed = 1;
    bool random = false;
    bool train = false;
    bool replay = true;
    bool trace = false;
};

struct Report
{
    std::vector<robotik::Episode> episodes;
    std::size_t steps = 0;
    double wall = 0.0;
};

static bool parse(int argc, char** argv, Options& p_options)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string const arg = argv[i];
        auto next = [&]() -> char const* { return i + 1 < argc ? argv[++i] : nullptr; };
        char const* value = nullptr;
        if (arg == "--policy" && (value = next()))
        {
            std::string const policy = value;
            p_options.random = policy == "random";
            if (policy != "random" && policy != "converged" && policy != "expert")
            {
                std::cerr << "Unknown --policy " << policy
                          << " (converged|random)\n";
                return false;
            }
        }
        else if (arg == "--train")
        {
            p_options.train = true;
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
        else if (arg == "--seed" && (value = next()))
        {
            p_options.seed = std::strtoull(value, nullptr, 10);
        }
        else if (arg == "--scenario" && (value = next()))
        {
            p_options.scenario = value;
        }
        else if (arg == "--trace")
        {
            p_options.trace = true;
        }
        else if (arg == "--no-replay")
        {
            p_options.replay = false;
        }
        else
        {
            std::cerr << "Usage: " << argv[0]
                      << " [--policy converged|random] [--train] [--envs N] [--threads T] [--episodes E]"
                         " [--seed S] [--scenario file.yml] [--no-replay] [--trace]\n";
            return false;
        }
    }
    return p_options.envs > 0u;
}

// Steps the pool until @p_options.episodes episodes are over.
static Report run(Options const& p_options, std::size_t p_threads)
{
    robotik::EnvironmentPool pool(
        p_options.envs,
        [&](std::size_t) { return std::make_unique<PickPlaceEnvironment>(p_options.scenario, CUBE_SPREAD_M, MAX_STEPS); },
        robotik::Seed{ p_options.seed },
        p_threads);

    // The random policy draws from its own seed, one stream per environment.
    std::vector<robotik::Random> noise;
    for (std::size_t i = 0; i < pool.size(); ++i)
    {
        noise.emplace_back(robotik::Seed{ p_options.seed }.derive("policy").derive(i));
    }

    Report report;
    float mix = p_options.random ? 0.0f : 1.0f;
    auto const begin = std::chrono::steady_clock::now();
    pool.reset();
    while (pool.episodes().size() < p_options.episodes)
    {
        for (std::size_t i = 0; i < pool.size(); ++i)
        {
            pickPlacePolicy(pool.observation(i),
                            pool.action(i),
                            mix,
                            p_options.random ? &noise[i] : nullptr);
        }
        if (p_options.trace)
        {
            std::cout << "obs[0]";
            for (float value : pool.observation(0))
            {
                std::cout << ' ' << std::setprecision(3) << value;
            }
            std::cout << "  act";
            for (float value : pool.action(0))
            {
                std::cout << ' ' << std::setprecision(2) << value;
            }
            std::cout << '\n';
        }
        pool.step();
        report.steps += pool.size();
        if (p_options.train && p_options.random)
        {
            mix = std::min(1.0f,
                           static_cast<float>(pool.episodes().size()) /
                               static_cast<float>(PICK_PLACE_TRAIN_EPISODES));
        }
    }
    report.wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();
    auto const episodes = pool.episodes();
    report.episodes.assign(episodes.begin(), episodes.begin() + static_cast<std::ptrdiff_t>(p_options.episodes));
    return report;
}

// Bit for bit: a replay must not even differ in rounding.
static bool same(std::vector<robotik::Episode> const& p_a, std::vector<robotik::Episode> const& p_b)
{
    if (p_a.size() != p_b.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < p_a.size(); ++i)
    {
        if (p_a[i].environment != p_b[i].environment || p_a[i].index != p_b[i].index ||
            p_a[i].seed.value != p_b[i].seed.value ||
            std::bit_cast<std::uint64_t>(p_a[i].reward) != std::bit_cast<std::uint64_t>(p_b[i].reward) ||
            p_a[i].steps != p_b[i].steps || p_a[i].terminated != p_b[i].terminated)
        {
            return false;
        }
    }
    return true;
}

int main(int argc, char** argv)
{
    Options options;
    if (!parse(argc, argv, options))
    {
        return EXIT_FAILURE;
    }
    try
    {
        std::size_t const threads = options.threads > 0u ? options.threads : std::max(1u, std::thread::hardware_concurrency());
        Report const report = run(options, threads);

        std::size_t successes = 0;
        double total = 0.0;
        std::cout << " env  episode  seed                  return   steps  success\n";
        for (robotik::Episode const& episode : report.episodes)
        {
            successes += episode.terminated ? 1u : 0u;
            total += episode.reward;
            std::cout << std::setw(4) << episode.environment << std::setw(9) << episode.index << "  "
                      << std::setw(20) << episode.seed.value << std::fixed << std::setprecision(2)
                      << std::setw(9) << episode.reward << std::setw(8) << episode.steps << "  "
                      << (episode.terminated ? "yes" : "no") << '\n';
        }
        double const count = static_cast<double>(report.episodes.size());
        double const simulated = static_cast<double>(report.steps) * 0.1;
        std::cout << (options.random ? (options.train ? "random→converged" : "random")
                                     : "converged")
                  << " policy: " << successes << '/'
                  << report.episodes.size() << " delivered, mean return " << total / count << '\n'
                  << options.envs << " environments on " << threads << " threads: " << report.steps
                  << " steps in " << report.wall << " s (" << static_cast<double>(report.steps) / report.wall
                  << " steps/s, " << simulated / report.wall << "x real time)\n";

        if (options.replay)
        {
            Report const again = run(options, 1u);
            bool const identical = same(report.episodes, again.episodes);
            std::cout << "replay on 1 thread: " << (identical ? "identical" : "DIFFERENT") << '\n';
            if (!identical)
            {
                return EXIT_FAILURE;
            }
        }
        return EXIT_SUCCESS;
    }
    catch (std::exception const& failure)
    {
        std::cerr << "Error: " << failure.what() << '\n';
        return EXIT_FAILURE;
    }
}
