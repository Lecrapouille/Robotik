// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Package/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <map>
#include <string>

// ----------------------------------------------------------------------------
TEST(SimulationEvents, ToldWhenSkillsEndAndAssertionsChange)
{
    PickPlaceMission mission;
    compages::world::World world;
    robotik::Simulation simulation(
        world,
        robotik::loadScenario(repoFile("plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"),
                              testAssets())
            .take(),
        nullptr,
        &mission);

    std::map<std::string, robotik::SkillState> skills;
    simulation.cell().skills().changed.connect(
        [&](robotik::SkillId p_id, robotik::SkillState p_state)
        { skills[simulation.cell().skills().name(p_id)] = p_state; });
    std::map<std::size_t, bool> colors;
    simulation.assertionChanged.connect([&](std::size_t p_index, bool p_green)
                                        { colors[p_index] = p_green; });
    int ends = 0;
    robotik::Simulation::Outcome outcome = robotik::Simulation::Outcome::Running;
    simulation.ended.connect(
        [&](robotik::Simulation::Outcome p_outcome)
        {
            ++ends;
            outcome = p_outcome;
        });

    simulation.reset(robotik::Seed{ 7 });
    while (!simulation.finished() && simulation.time() < robotik::Seconds(60.0))
    {
        simulation.step(robotik::Seconds(0.01));
    }

    // Every assertion was announced with its last color.
    ASSERT_EQ(colors.size(), simulation.checks().size());
    for (std::size_t i = 0; i < simulation.checks().size(); ++i)
    {
        EXPECT_EQ(colors[i], robotik::ok(simulation.checks()[i])) << simulation.checks()[i].text;
    }
    // Skills ran and ended; the run ended once.
    EXPECT_FALSE(skills.empty());
    EXPECT_EQ(ends, 1);
    EXPECT_EQ(outcome, simulation.outcome());
}
