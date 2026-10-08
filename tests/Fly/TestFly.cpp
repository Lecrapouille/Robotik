// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "FlyBrain.hpp"
#include "FlyEnvironment.hpp"
#include "FlyTrace.hpp"
#include "ShiuNetwork.hpp"

#include <filesystem>

namespace
{

std::filesystem::path scenarioFile()
{
    return std::filesystem::path(__FILE__).parent_path() /
           "../../data/scenarios/fly_obstacle_avoidance.yml";
}

robotik::StepResult flyToFood(FlyBrain::Kind p_kind, FlySnapshot& p_end)
{
    FlyScenario const scenario = FlyScenario::load(scenarioFile());
    std::uint32_t const limit =
        static_cast<std::uint32_t>(scenario.horizon / scenario.dt);
    FlyEnvironment environment(scenario, limit);
    FlyBrain brain(p_kind, static_cast<float>(scenario.target.z));
    std::vector<float> observation(FlyObservation::SIZE);
    std::vector<float> action(FlyAction::SIZE);
    robotik::Seed const seed{ scenario.seed };
    environment.reset(seed, observation);
    brain.reset(seed);
    robotik::StepResult result;
    for (std::uint32_t step = 0; step < limit; ++step)
    {
        FlyAction const decided =
            brain.update(FlyObservation::from(observation), Seconds(scenario.dt));
        decided.write(action);
        result = environment.step(action, observation);
        if (result.done())
        {
            break;
        }
    }
    p_end = environment.snapshot();
    return result;
}

} // namespace

TEST(FlyScenario, LoadsObstacleAvoidance)
{
    FlyScenario const scenario = FlyScenario::load(scenarioFile());
    EXPECT_EQ(scenario.seed, 123456u);
    EXPECT_EQ(scenario.obstacles.size(), 2u);
    EXPECT_NEAR(scenario.target.x, 8.0, 1.0e-9);
    EXPECT_TRUE(std::filesystem::exists(scenario.robot_model));
}

TEST(ShiuNetwork, SilentNetworkDoesNotSpike)
{
    ShiuNetwork network;
    network.resize(4);
    network.addDrive(0);
    network.build();
    network.reset(robotik::Seed{ 1 });
    network.step(0.05);
    EXPECT_EQ(network.spikes(0), 0u);
    EXPECT_EQ(network.spikes(1), 0u);
}

TEST(FlyBrain, ShiuTurnsAwayFromTheLeftEye)
{
    FlyBrain brain(FlyBrain::Kind::Shiu, 1.0f);
    brain.reset(robotik::Seed{ 7 });
    FlyObservation observation;
    observation.altitude = 1.0f;
    observation.visual_left = 1.0f;
    FlyAction action;
    for (int step = 0; step < 200; ++step)
    {
        action = brain.update(observation, Seconds(0.01));
    }
    EXPECT_LT(action.turn, -0.3f);
    EXPECT_GT(action.forward, 0.4f);
}

TEST(FlyBrain, ReflexReachesTheFood)
{
    FlySnapshot end;
    robotik::StepResult const result = flyToFood(FlyBrain::Kind::Reflex, end);
    EXPECT_TRUE(result.terminated) << "x=" << end.plant.x << " y=" << end.plant.y
                                   << " yaw=" << end.plant.yaw
                                   << " steps=" << end.steps;
}

TEST(FlyBrain, ShiuReachesTheFood)
{
    FlySnapshot end;
    robotik::StepResult const result = flyToFood(FlyBrain::Kind::Shiu, end);
    EXPECT_TRUE(result.terminated) << "x=" << end.plant.x << " y=" << end.plant.y
                                   << " yaw=" << end.plant.yaw
                                   << " steps=" << end.steps;
}

TEST(FlyEnvironment, SameSeedSameArrival)
{
    FlySnapshot first;
    FlySnapshot second;
    flyToFood(FlyBrain::Kind::Reflex, first);
    flyToFood(FlyBrain::Kind::Reflex, second);
    EXPECT_NEAR(first.plant.x, second.plant.x, 1.0e-9);
    EXPECT_NEAR(first.plant.y, second.plant.y, 1.0e-9);
    EXPECT_EQ(first.steps, second.steps);
    EXPECT_EQ(first.reached, second.reached);
}

TEST(FlyTrace, RoundTrip)
{
    FlyTrace trace;
    trace.seed = 99;
    trace.dt = 0.01;
    trace.observations.push_back(FlyObservation{ 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 1.0f, 2.0f, -0.2f, 3.0f });
    trace.actions.push_back(FlyAction{ 0.8f, -0.2f, 0.1f });
    std::filesystem::path const path =
        std::filesystem::temp_directory_path() / "robotik_fly_trace.json";
    trace.save(path);
    FlyTrace const loaded = FlyTrace::load(path);
    EXPECT_EQ(loaded.seed, 99u);
    ASSERT_EQ(loaded.actions.size(), 1u);
    EXPECT_NEAR(loaded.actions[0].forward, 0.8f, 1.0e-5f);
    EXPECT_NEAR(loaded.actions[0].turn, -0.2f, 1.0e-5f);
    ASSERT_EQ(loaded.observations.size(), 1u);
    EXPECT_NEAR(loaded.observations[0].visual_center, 0.2f, 1.0e-5f);
}
