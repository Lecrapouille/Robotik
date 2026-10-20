// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Package/RunReport.hpp"
#include "Robotik/Package/Scenario/Experiment.hpp"
#include "Robotik/Package/Scenario/PackagePath.hpp"
#include "Robotik/Package/Scenario/Trajectory.hpp"

#include <fstream>
#include <sstream>

/// The experiment case with this id.
static robotik::ExperimentCase const&
find(robotik::Experiment const& p_experiment, std::string const& p_id)
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

/// @brief Writes @p_text in experiments/ of a throwaway package.
static std::filesystem::path write(std::string const& p_name,
                                   std::string const& p_text)
{
    std::filesystem::path const package =
        std::filesystem::temp_directory_path() / "robotik_package";
    std::filesystem::create_directories(package / "experiments");
    std::ofstream(package / "plugin.yaml") << "plugin: { id: test }\n";
    std::filesystem::path const path = package / "experiments" / p_name;
    std::ofstream(path) << p_text;
    return path;
}

TEST(Experiment, ReadsCasesSeedsAndEvaluation)
{
    std::filesystem::path const file =
        repoFile("plugins/Painter/experiments/painter.experiment.yaml");
    EXPECT_TRUE(robotik::isExperiment(file));
    EXPECT_FALSE(robotik::isExperiment(
        repoFile("plugins/Painter/scenarios/painter.scenario.yaml")));
    robotik::Experiment const experiment = robotik::loadExperiment(file).take();
    EXPECT_EQ(experiment.id, "painter_experiment");
    EXPECT_EQ(experiment.seeds, (std::vector<std::uint64_t>{ 7u, 8u }));
    ASSERT_EQ(experiment.cases.size(), 3u);
    EXPECT_EQ(experiment.cases[1].overrides.size(), 2u);
    EXPECT_EQ(experiment.metrics.size(), 6u);
}

TEST(Experiment, CaseAppliesOverridesFaultsAndAssertions)
{
    robotik::Experiment const painting = robotik::loadExperiment(
        repoFile("plugins/Painter/experiments/painter.experiment.yaml")).take();
    robotik::Scenario const slippery =
        robotik::scenarioOf(painting, find(painting, "slippery_floor"), testAssets()).take();
    EXPECT_DOUBLE_EQ(slippery.world.floor_friction, 0.1);
    EXPECT_DOUBLE_EQ(robotik::findEntity(slippery.world, "easel")->friction, 0.1);
    // The case does not leak into the next one.
    robotik::Scenario const nominal =
        robotik::scenarioOf(painting, find(painting, "nominal"), testAssets()).take();
    EXPECT_DOUBLE_EQ(nominal.world.floor_friction, 0.8);

    robotik::Experiment const picking = robotik::loadExperiment(repoFile(
        "plugins/PickAndPlace/experiments/pick_and_place.experiment.yaml")).take();
    robotik::Scenario const camera =
        robotik::scenarioOf(picking, find(picking, "camera_down_at_start"), testAssets()).take();
    ASSERT_EQ(camera.faults.size(), 2u);
    EXPECT_TRUE(camera.faults[0].disable);
    EXPECT_DOUBLE_EQ(camera.faults[1].at.value(), 1.5);
    EXPECT_EQ(camera.assertions.size(), 6u);
    EXPECT_EQ(camera.assertions.back().phase, robotik::AssertionPhase::Always);
    robotik::Scenario const wide =
        robotik::scenarioOf(picking, find(picking, "wide_layout"), testAssets()).take();
    EXPECT_DOUBLE_EQ(robotik::findEntity(wide.world, "red_cube")->randomize[0][1], 0.05);
}

TEST(Experiment, InvalidFilesAreRefused)
{
    std::filesystem::path const twice = write(
        "robotik_twice.yaml",
        "experiment:\n  id: x\n  cases:\n    - { id: a, scenario: s.yml }\n"
        "    - { id: a, scenario: s.yml }\n");
    EXPECT_FALSE(robotik::loadExperiment(twice));
    std::filesystem::path const empty =
        write("robotik_empty.yaml", "experiment:\n  id: x\n  cases: []\n");
    EXPECT_FALSE(robotik::loadExperiment(empty));
    std::filesystem::path const setup = write(
        "robotik_setup.yaml",
        "experiment:\n  id: x\n  cases:\n    - id: a\n      scenario: s.yml\n"
        "      setup:\n        - reboot: now\n");
    EXPECT_FALSE(robotik::loadExperiment(setup));
}

TEST(PackagePath, NamesStartAtThePackageRootAndStayInside)
{
    std::filesystem::path const file =
        write("robotik_root.yaml", "experiment: {}\n");
    std::filesystem::path const root =
        std::filesystem::canonical(robotik::packageRoot(file).take());
    EXPECT_EQ(root,
              std::filesystem::canonical(file.parent_path().parent_path()));
    EXPECT_EQ(robotik::packageFile(root, "scenarios/a.yml").take(),
              root / "scenarios" / "a.yml");
    EXPECT_EQ(robotik::packageFile(root, "./worlds/b.yaml").take(),
              root / "worlds" / "b.yaml");
    // ".." is followed: what counts is where the file really is.
    EXPECT_EQ(robotik::packageFile(root, "experiments/../worlds/b.yaml").take(),
              root / "worlds" / "b.yaml");
    EXPECT_TRUE(robotik::packageFile(root, "").take().empty());
    EXPECT_FALSE(robotik::packageFile(root, "../scenarios/a.yml"));
    EXPECT_FALSE(robotik::packageFile(root, "scenarios/../../a.yml"));
    EXPECT_FALSE(robotik::packageFile(root, ".."));
    EXPECT_FALSE(robotik::packageFile(root, "/etc/passwd"));
    EXPECT_FALSE(robotik::packageRoot("/robotik_no_package/a.yml"));

    // A symbolic link is followed too: inside the package it is accepted,
    // pointing out of it, refused.
    std::error_code error;
    std::filesystem::remove(root / "inside", error);
    std::filesystem::remove(root / "outside", error);
    std::filesystem::create_directory_symlink(root / "experiments",
                                              root / "inside");
    std::filesystem::create_directory_symlink(root.parent_path(),
                                              root / "outside");
    EXPECT_EQ(robotik::packageFile(root, "inside/robotik_root.yaml").take(), file);
    EXPECT_FALSE(robotik::packageFile(root, "outside/x.yml"));

    std::filesystem::path const escape =
        write("robotik_escape.yaml",
              "experiment:\n  id: x\n  cases:\n    - { id: a, scenario: "
              "../scenarios/s.yml }\n");
    EXPECT_FALSE(robotik::loadExperiment(escape));
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
    EXPECT_TRUE(robotik::passed(report));
    std::ostringstream json;
    robotik::writeJson(report, json);
    EXPECT_NE(json.str().find("\"say \\\"hi\\\"\""), std::string::npos)
        << json.str();
    EXPECT_NE(json.str().find("\"a\\\\b\""), std::string::npos);
    EXPECT_NE(json.str().find("\"x\": null"), std::string::npos);
    report.runs.back().passed = false;
    EXPECT_FALSE(robotik::passed(report));
}

TEST(Trajectory, ReadsCartesianAndJointPaths)
{
    robotik::Trajectory const stroke = robotik::loadTrajectory(repoFile(
        "plugins/Painter/scenarios/trajectories/red_stroke.trajectory.yaml")).take();
    EXPECT_EQ(stroke.type, robotik::Trajectory::Type::Cartesian);
    EXPECT_EQ(stroke.frame, "easel.canvas_mount");
    ASSERT_EQ(stroke.poses.size(), 5u);
    EXPECT_DOUBLE_EQ(stroke.poses[1].position.x, 0.08);

    robotik::Trajectory const joints = robotik::loadTrajectory(
        repoFile("tests/Scenario/fixtures/joint_path.trajectory.yaml")).take();
    EXPECT_EQ(joints.type, robotik::Trajectory::Type::Joint);
    ASSERT_EQ(joints.configurations.size(), 2u);
    EXPECT_DOUBLE_EQ(joints.configurations[1][0], 0.5);

    EXPECT_FALSE(robotik::loadTrajectory(repoFile(
                     "tests/Scenario/fixtures/bad_path.trajectory.yaml")));
}
