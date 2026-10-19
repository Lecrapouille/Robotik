// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Package/Scenario/FileRole.hpp"

TEST(FileRole, DoubleExtensionGivesTheRole)
{
    using robotik::FileRole;
    EXPECT_EQ(robotik::fileRole("worlds/arm.world.yaml"), FileRole::World);
    EXPECT_EQ(robotik::fileRole("scenarios/wave.scenario.yaml"),
              FileRole::Scenario);
    EXPECT_EQ(robotik::fileRole("scenarios/pick_rl.rl.yaml"), FileRole::Rl);
    EXPECT_EQ(robotik::fileRole("scenarios/skills.bt.yaml"), FileRole::Tree);
    EXPECT_EQ(robotik::fileRole("scenarios/skills.bt.yml"), FileRole::Tree);
    EXPECT_EQ(robotik::fileRole("scenarios/pick.domain.pddl"),
              FileRole::Domain);
    EXPECT_EQ(robotik::fileRole("trajectories/red.trajectory.yaml"),
              FileRole::Trajectory);
    EXPECT_EQ(robotik::fileRole("experiments/bench.experiment.yaml"),
              FileRole::Experiment);
    EXPECT_EQ(robotik::fileRole("scenarios/wave.yml"), FileRole::Unknown);
    // The role alone is not a name.
    EXPECT_EQ(robotik::fileRole("scenarios/.scenario.yaml"), FileRole::Unknown);
}

TEST(FileRole, IdFromTheFileName)
{
    EXPECT_EQ(robotik::roleStem("scenarios/wave.scenario.yaml"), "wave");
    EXPECT_EQ(robotik::roleStem("scenarios/pick_and_place.rl.yaml"),
              "pick_and_place_rl");
    EXPECT_EQ(robotik::roleStem("experiments/hello.experiment.yaml"),
              "hello_experiment");
    EXPECT_EQ(robotik::roleStem("scenarios/wave.yml"), "wave");
    EXPECT_STREQ(robotik::roleExtension(robotik::FileRole::World),
                 ".world.yaml");
}

TEST(FileRole, OnlyRunnableFilesAreScenarios)
{
    EXPECT_TRUE(robotik::runnableRole("a.scenario.yaml"));
    EXPECT_TRUE(robotik::runnableRole("a.rl.yaml"));
    EXPECT_TRUE(robotik::runnableRole("a.experiment.yaml"));
    EXPECT_TRUE(robotik::runnableRole("a.yml"));
    EXPECT_FALSE(robotik::runnableRole("a.bt.yaml"));
    EXPECT_FALSE(robotik::runnableRole("a.world.yaml"));
    EXPECT_FALSE(robotik::runnableRole("a.trajectory.yaml"));
    EXPECT_FALSE(robotik::runnableRole("a.domain.pddl"));
    EXPECT_FALSE(robotik::runnableRole("notes.txt"));
}
