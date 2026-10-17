// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceControl.hpp"

#include "Robotik/Environment/EpisodeLog.hpp"
#include "Robotik/Environment/SimulationEnvironment.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <sstream>
#include <stdexcept>

namespace
{

robotik::EnvironmentTiming timing(std::uint32_t p_max_steps)
{
    robotik::EnvironmentTiming result;
    result.physics_step = Seconds(PICK_PLACE_PHYSICS_STEP_S);
    result.control_period = Seconds(PICK_PLACE_CONTROL_PERIOD_S);
    result.max_steps = p_max_steps;
    return result;
}

robotik::Scenario scenario()
{
    return robotik::Scenario::load(repoFile("plugins/PickAndPlace/scenarios/pick_and_place.yml"),
                                   testAssets());
}

//! Throws on the third action: a solver that gives up.
class Fragile final: public robotik::ActionAdapter
{
public:

    [[nodiscard]] robotik::Space space() const override
    {
        return robotik::Space::unnamed("a", 4u);
    }

    void apply(std::span<float const> /*p_action*/, robotik::Simulation& /*p_simulation*/) override
    {
        if (++m_calls == 3)
        {
            throw std::runtime_error("no IK solution");
        }
    }

private:

    int m_calls = 0;
};

} // namespace

TEST(SimulationEnvironment, SpacesAreNamedBoundedAndHonest)
{
    robotik::SimulationEnvironment environment(scenario(), pickPlaceParts(), timing(20));
    robotik::Space const observation = environment.observationSpace();
    ASSERT_EQ(environment.observationSize(), PICK_PLACE_OBSERVATIONS);
    EXPECT_EQ(environment.actionSize(), PICK_PLACE_ACTIONS);
    EXPECT_EQ(observation.dimensions[0].name, "tip.x");
    EXPECT_EQ(observation.dimensions[0].unit, "m");
    // Only the delivered flag comes from the ground truth.
    EXPECT_TRUE(observation.privileged());
    EXPECT_TRUE(observation.dimensions.back().privileged);
    EXPECT_FALSE(observation.dimensions[3].privileged);

    std::vector<float> values(environment.observationSize());
    environment.reset(robotik::Seed{ 3 }, values);
    EXPECT_TRUE(observation.contains(values));
}

TEST(SimulationEnvironment, ResetReplaysAndSeedsDiffer)
{
    robotik::SimulationEnvironment environment(scenario(), pickPlaceParts(), timing(20));
    std::vector<float> first(environment.observationSize());
    std::vector<float> again(environment.observationSize());
    std::vector<float> other(environment.observationSize());
    std::vector<float> const action{ 0.5f, -0.5f, 0.0f, -1.0f };
    environment.reset(robotik::Seed{ 5 }, first);
    (void)environment.step(action, first);
    environment.reset(robotik::Seed{ 5 }, again);
    (void)environment.step(action, again);
    EXPECT_EQ(first, again) << "same seed, same action: same observation";
    environment.reset(robotik::Seed{ 6 }, other);
    EXPECT_NE(first[3], other[3]) << "the cube moves with the seed";
}

TEST(SimulationEnvironment, TimeLimitTruncatesWithoutEndingTheTask)
{
    robotik::SimulationEnvironment environment(scenario(), pickPlaceParts(), timing(3));
    std::vector<float> observation(environment.observationSize());
    std::vector<float> const still(environment.actionSize(), 0.0f);
    environment.reset(robotik::Seed{ 1 }, observation);
    robotik::StepResult result;
    for (int i = 0; i < 3; ++i)
    {
        result = environment.step(still, observation);
    }
    EXPECT_EQ(result.end, robotik::EpisodeEnd::TimeLimit);
    EXPECT_TRUE(result.truncated());
    EXPECT_FALSE(result.terminated());
    EXPECT_FLOAT_EQ(observation[14], 1.0f) << "progress";
    EXPECT_NEAR(environment.simulation().time().value(), 0.3, 1e-9) << "three control periods";
}

TEST(SimulationEnvironment, ATechnicalErrorAbortsTheEpisode)
{
    robotik::SimulationEnvironment::Parts parts = pickPlaceParts();
    parts.action = std::make_unique<Fragile>();
    robotik::SimulationEnvironment environment(scenario(), std::move(parts), timing(20));
    std::vector<float> observation(environment.observationSize());
    std::vector<float> const action(environment.actionSize(), 0.0f);
    environment.reset(robotik::Seed{ 1 }, observation);
    EXPECT_EQ(environment.step(action, observation).end, robotik::EpisodeEnd::None);
    EXPECT_EQ(environment.step(action, observation).end, robotik::EpisodeEnd::None);
    robotik::StepResult const result = environment.step(action, observation);
    EXPECT_EQ(result.end, robotik::EpisodeEnd::Aborted);
    EXPECT_TRUE(result.truncated());
    EXPECT_EQ(environment.abortReason(), "no IK solution");
}

TEST(SimulationEnvironment, InvalidPartsOrTimingAreRefused)
{
    robotik::SimulationEnvironment::Parts missing = pickPlaceParts();
    missing.reward.reset();
    EXPECT_THROW(robotik::SimulationEnvironment(scenario(), std::move(missing), timing(10)),
                 std::invalid_argument);
    robotik::EnvironmentTiming odd = timing(10);
    odd.control_period = Seconds(0.015);
    EXPECT_THROW(robotik::SimulationEnvironment(scenario(), pickPlaceParts(), odd),
                 std::invalid_argument);
}

TEST(EpisodeLog, KeepsWhatAReplayNeeds)
{
    robotik::EpisodeLog log;
    log.scenario = "pick_and_place";
    log.policy = "converged";
    log.master_seed = 9;
    log.observation.add("x", -1.0f, 1.0f, "m").add("hint", 0.0f, 1.0f, "", true);
    std::vector<robotik::Episode> episodes(1);
    episodes[0].seed = robotik::Seed{ 42 };
    episodes[0].end = robotik::EpisodeEnd::TimeLimit;
    std::ostringstream json;
    log.write(json, episodes);
    std::string const text = json.str();
    EXPECT_NE(text.find("\"commit\""), std::string::npos);
    EXPECT_NE(text.find("\"master_seed\": 9"), std::string::npos);
    EXPECT_NE(text.find("\"seed\": 42"), std::string::npos);
    EXPECT_NE(text.find("\"end\": \"time limit\""), std::string::npos);
    EXPECT_NE(text.find("\"privileged\": true"), std::string::npos);
}
