// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Runtime/Simulation.hpp"

#include "Compages/World/World.hpp"

#include <cmath>

/// The pick-and-place scenario forced to rigid grasp fidelity.
static robotik::Scenario rigidPickAndPlace()
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(
        "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"));
    source.override("scenario.physics.fidelity", robotik::ConfigNode("rigid"));
    return robotik::Scenario::read(source, testAssets());
}

// Rigid level: the cube is a free body, the vacuum welds it, and it falls
// into the box for real on release.
TEST(RigidGrasp, PickAndPlaceDropsTheCubeIntoTheBox)
{
    PickPlaceMission mission(true, false);
    compages::world::World world;
    robotik::Simulation simulation(
        world, rigidPickAndPlace(), nullptr, &mission);
    simulation.reset(robotik::Seed{ 11 });
    bool lifted = false;
    while (!simulation.finished() && simulation.time() < robotik::Seconds(30.0))
    {
        simulation.step(robotik::Seconds(0.01));
        lifted = lifted || simulation.pose("red_cube").position.z > 0.08;
    }
    simulation.conclude();
    for (robotik::Check const& check : simulation.checks())
    {
        EXPECT_EQ(check.verdict, robotik::Check::Verdict::Passed)
            << check.text << ": " << check.detail << " at "
            << check.time.value() << " s";
    }
    EXPECT_EQ(simulation.outcome(),
              robotik::Simulation::Outcome::ControllerSucceeded);
    EXPECT_TRUE(lifted) << "the weld did not carry the cube";

    // It fell where the tool let it go: on the floor of the box (5 mm thick),
    // not put at the centre as the abstract level does.
    robotik::Pose const cube = simulation.pose("red_cube");
    robotik::Pose const box = simulation.pose("box");
    EXPECT_NEAR(cube.position.z, 0.005 + 0.02, 0.003);
    EXPECT_GT(std::hypot(cube.position.x - box.position.x,
                         cube.position.y - box.position.y),
              0.002);
}

TEST(RigidGrasp, ContactLevelIsRefusedUntilItExists)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(
        "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"));
    source.override("scenario.physics.fidelity",
                    robotik::ConfigNode("contact"));
    EXPECT_THROW((void)robotik::Scenario::read(source, testAssets()),
                 std::runtime_error);
}

TEST(RigidGrasp, OverrideOfAnUnknownEntityIsRefused)
{
    robotik::ScenarioSource source = robotik::ScenarioSource::load(repoFile(
        "plugins/PickAndPlace/scenarios/pick_and_place.scenario.yaml"));
    source.override("scenario.physics.overrides.blue_cube",
                    robotik::ConfigNode("rigid"));
    EXPECT_THROW((void)robotik::Scenario::read(source, testAssets()),
                 std::runtime_error);
}
