// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Runtime/RunReport.hpp"
#include "Robotik/Scenario/Experiment.hpp"
#include "Robotik/Scenario/Trajectory.hpp"

#include <fstream>
#include <sstream>

namespace
{

robotik::ExperimentCase const& find(robotik::Experiment const& p_experiment,
                                    std::string const& p_id)
{
    for (robotik::ExperimentCase const& item : p_experiment.cases)
    {
        if (item.id == p_id)
        {
            return item;
        }
    }
    throw std::invalid_argument(p_id);
}

std::filesystem::path write(std::string const& p_name, std::string const& p_text)
{
    std::filesystem::path const path = std::filesystem::temp_directory_path() / p_name;
    std::ofstream(path) << p_text;
    return path;
}

} // namespace

TEST(Experiment, ReadsCasesSeedsAndEvaluation)
{
    std::filesystem::path const file =
        repoFile("plugins/Painter/experiments/painting_benchmark.yaml");
    EXPECT_TRUE(robotik::Experiment::describes(file));
    EXPECT_FALSE(robotik::Experiment::describes(repoFile("plugins/Painter/scenarios/painter.yml")));
    robotik::Experiment const experiment = robotik::Experiment::load(file);
    EXPECT_EQ(experiment.id, "painting_benchmark");
    EXPECT_EQ(experiment.seeds, (std::vector<std::uint64_t>{ 7u, 8u }));
    ASSERT_EQ(experiment.cases.size(), 3u);
    EXPECT_EQ(experiment.cases[1].overrides.size(), 2u);
    EXPECT_EQ(experiment.metrics.size(), 4u);
}

TEST(Experiment, CaseAppliesOverridesFaultsAndAssertions)
{
    robotik::Experiment const painting =
        robotik::Experiment::load(repoFile("plugins/Painter/experiments/painting_benchmark.yaml"));
    robotik::Scenario const slippery =
        painting.scenario(find(painting, "slippery_floor"), testAssets());
    EXPECT_DOUBLE_EQ(slippery.world.floor_friction, 0.1);
    EXPECT_DOUBLE_EQ(slippery.world.find("easel")->friction, 0.1);
    // The case does not leak into the next one.
    robotik::Scenario const nominal = painting.scenario(find(painting, "nominal"), testAssets());
    EXPECT_DOUBLE_EQ(nominal.world.floor_friction, 0.8);

    robotik::Experiment const picking = robotik::Experiment::load(
        repoFile("plugins/PickAndPlaceBT/experiments/pick_and_place_benchmark.yaml"));
    robotik::Scenario const camera =
        picking.scenario(find(picking, "camera_down_at_start"), testAssets());
    ASSERT_EQ(camera.faults.size(), 2u);
    EXPECT_TRUE(camera.faults[0].disable);
    EXPECT_DOUBLE_EQ(camera.faults[1].at.value(), 1.5);
    EXPECT_EQ(camera.assertions.size(), 6u);
    EXPECT_EQ(camera.assertions.back().phase, robotik::AssertionPhase::Always);
    robotik::Scenario const wide = picking.scenario(find(picking, "wide_layout"), testAssets());
    EXPECT_DOUBLE_EQ(wide.world.find("red_cube")->randomize[0][1], 0.05);
}

TEST(Experiment, InvalidFilesAreRefused)
{
    std::filesystem::path const twice = write(
        "robotik_twice.yaml",
        "experiment:\n  id: x\n  cases:\n    - { id: a, scenario: s.yml }\n"
        "    - { id: a, scenario: s.yml }\n");
    EXPECT_THROW((void)robotik::Experiment::load(twice), std::runtime_error);
    std::filesystem::path const empty =
        write("robotik_empty.yaml", "experiment:\n  id: x\n  cases: []\n");
    EXPECT_THROW((void)robotik::Experiment::load(empty), std::runtime_error);
    std::filesystem::path const setup = write(
        "robotik_setup.yaml",
        "experiment:\n  id: x\n  cases:\n    - id: a\n      scenario: s.yml\n"
        "      setup:\n        - reboot: now\n");
    EXPECT_THROW((void)robotik::Experiment::load(setup), std::runtime_error);
}

TEST(RunReport, JsonEscapesAndCountsRuns)
{
    robotik::RunReport report;
    report.experiment = "say \"hi\"";
    robotik::RunRecord run;
    run.case_id = "a\\b";
    run.passed = true;
    run.metrics.emplace_back("x", std::nullopt);
    report.runs.push_back(run);
    EXPECT_TRUE(report.passed());
    std::ostringstream json;
    report.writeJson(json);
    EXPECT_NE(json.str().find("\"say \\\"hi\\\"\""), std::string::npos) << json.str();
    EXPECT_NE(json.str().find("\"a\\\\b\""), std::string::npos);
    EXPECT_NE(json.str().find("\"x\": null"), std::string::npos);
    report.runs.back().passed = false;
    EXPECT_FALSE(report.passed());
}

TEST(Trajectory, ReadsCartesianAndJointPaths)
{
    robotik::Trajectory const stroke = robotik::Trajectory::load(
        repoFile("plugins/Painter/scenarios/trajectories/red_stroke.yaml"));
    EXPECT_EQ(stroke.type, robotik::Trajectory::Type::Cartesian);
    EXPECT_EQ(stroke.frame, "easel.canvas_mount");
    ASSERT_EQ(stroke.poses.size(), 5u);
    EXPECT_DOUBLE_EQ(stroke.poses[1].position.x, 0.08);

    robotik::Trajectory const joints =
        robotik::Trajectory::load(repoFile("tests/Scenario/fixtures/joint_path.yaml"));
    EXPECT_EQ(joints.type, robotik::Trajectory::Type::Joint);
    ASSERT_EQ(joints.configurations.size(), 2u);
    EXPECT_DOUBLE_EQ(joints.configurations[1][0], 0.5);

    EXPECT_THROW((void)robotik::Trajectory::load(repoFile("tests/Scenario/fixtures/bad_path.yaml")),
                 std::runtime_error);
}
