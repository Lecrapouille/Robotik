// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Environment/Environment.hpp"

namespace
{

//! @brief Random walk on a line: reach +1 to win, give up after 50 steps.
class WalkEnvironment final: public robotik::Environment
{
public:

    std::size_t observationSize() const override
    {
        return 2u;
    }

    std::size_t actionSize() const override
    {
        return 1u;
    }

    void reset(robotik::Seed p_seed, std::span<float> p_observation) override
    {
        robotik::Random random(p_seed);
        m_position = random.uniform(-0.5, 0.5);
        m_steps = 0;
        write(p_observation);
    }

    robotik::StepResult step(std::span<float const> p_action,
                             std::span<float> p_observation) override
    {
        m_position += 0.1 * static_cast<double>(p_action[0]);
        ++m_steps;
        write(p_observation);
        robotik::StepResult result;
        result.terminated = m_position >= 1.0;
        result.truncated = !result.terminated && m_steps >= 50;
        result.reward = result.terminated ? 1.0f : -0.01f;
        return result;
    }

private:

    void write(std::span<float> p_observation) const
    {
        p_observation[0] = static_cast<float>(m_position);
        p_observation[1] = static_cast<float>(m_steps);
    }

    double m_position = 0.0;
    int m_steps = 0;
};

std::vector<robotik::Episode> play(std::size_t p_threads)
{
    robotik::EnvironmentPool pool(
        8u,
        [](std::size_t) { return std::make_unique<WalkEnvironment>(); },
        robotik::Seed{ 5 },
        p_threads);
    pool.reset();
    robotik::Random policy(robotik::Seed{ 99 });
    for (int t = 0; t < 400; ++t)
    {
        for (float& action : pool.actions())
        {
            action = static_cast<float>(policy.uniform(-0.5, 1.0));
        }
        pool.step();
    }
    return { pool.episodes().begin(), pool.episodes().end() };
}

} // namespace

TEST(EnvironmentPool, AutoResetsWithDerivedSeeds)
{
    robotik::EnvironmentPool pool(
        3u,
        [](std::size_t) { return std::make_unique<WalkEnvironment>(); },
        robotik::Seed{ 1 },
        1u);
    EXPECT_EQ(pool.observationSize(), 2u);
    EXPECT_EQ(pool.actionSize(), 1u);
    pool.reset();
    EXPECT_EQ(pool.seed(2), robotik::Seed{ 1 }.derive(2u).derive(0u));
    for (float& action : pool.actions())
    {
        action = 0.0f;
    }
    for (int t = 0; t < 50; ++t)
    {
        pool.step();
    }
    ASSERT_EQ(pool.episodes().size(), 3u);
    EXPECT_FALSE(pool.episodes()[0].terminated);
    EXPECT_EQ(pool.episodes()[0].steps, 50u);
    EXPECT_EQ(pool.dones()[1], 1u);
    EXPECT_EQ(pool.observation(1)[1], 0.0f) << "restarted at once";
    EXPECT_EQ(pool.seed(1), robotik::Seed{ 1 }.derive(1u).derive(1u));
}

TEST(EnvironmentPool, ThreadsDoNotChangeTheResults)
{
    std::vector<robotik::Episode> const serial = play(1u);
    std::vector<robotik::Episode> const parallel = play(4u);
    ASSERT_EQ(serial.size(), parallel.size());
    ASSERT_FALSE(serial.empty());
    for (std::size_t i = 0; i < serial.size(); ++i)
    {
        EXPECT_EQ(serial[i].environment, parallel[i].environment);
        EXPECT_EQ(serial[i].seed, parallel[i].seed);
        EXPECT_EQ(serial[i].steps, parallel[i].steps);
        EXPECT_DOUBLE_EQ(serial[i].reward, parallel[i].reward);
    }
}
