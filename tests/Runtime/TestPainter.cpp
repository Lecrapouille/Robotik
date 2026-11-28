// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Package/StandaloneSimulation.hpp"

#include "PainterLogic.hpp"
#include "PainterScene.hpp"

#include "RobotIK/Package/Simulation.hpp"

#include <cmath>

/// Loads the painter scenario used by these tests.
static robotik::Scenario painterScenario()
{
    return robotik::loadScenario(
        repoFile("demos/Painter/scenarios/painter.scenario.yaml"),
        testAssets()).take();
}

/// Distance between two poses.
static double gap(robotik::Pose const& p_a, robotik::Pose const& p_b)
{
    return robotik::norm(p_a.position - p_b.position);
}

TEST(Painter, StrokesAreDrawnInTheFrameOfTheEasel)
{
    PainterLogic logic;
    auto opened = robotik::require(
        robotik::StandaloneSimulation::open(painterScenario(), &logic));
    robotik::Simulation& simulation = opened.simulation();

    // The easel stands where the world file puts it, its canvas on the mount.
    EXPECT_LT(gap(simulation.cell().requirePose("easel"),
                  robotik::Pose{ robotik::Vector3(0.65, 0.0, 0.0),
                                 robotik::Quaternion{} }),
              1e-3);
    EXPECT_LT(gap(simulation.cell().requireDescribedPose("canvas"),
                  simulation.cell().requirePose("canvas")),
              1e-6);
    EXPECT_EQ(
        simulation.cell().requireQuery("(tool_is_attached brush robot.flange)"),
        1.0);

    // The red stroke paints 4 cm in front of the canvas face, in its upper
    // half.
    std::vector<robotik::Pose> const red =
        simulation.cell().requireWaypoints("red");
    ASSERT_EQ(red.size(), 5U);
    robotik::Vector3 const local =
        simulation.cell().requirePose("canvas").inverse() * red[2].position;
    EXPECT_NEAR(local.x, 0.06 - 0.011, 1e-6);
    EXPECT_NEAR(local.z, 0.16 - 0.251, 1e-6);
    // The brush points into the canvas: its axis is the canvas -x.
    robotik::Vector3 const axis =
        red[2].rotation * robotik::Vector3(0.0, 0.0, 1.0);
    robotik::Vector3 const into =
        simulation.cell().requirePose("canvas").rotation *
        robotik::Vector3(-1.0, 0.0, 0.0);
    EXPECT_NEAR(robotik::norm(axis - into), 0.0, 1e-6);
    EXPECT_THROW((void)simulation.cell().requireWaypoints("purple"),
                 std::invalid_argument);
}

TEST(Painter, CanvasRestsOnTheEasel)
{
    PainterLogic logic;
    auto opened = robotik::require(
        robotik::StandaloneSimulation::open(painterScenario(), &logic));
    robotik::Simulation& simulation = opened.simulation();

    // The mission is suspended: nothing pushes. The canvas may settle 1 mm.
    simulation.cell().suspend(true);
    for (int i = 0; i < 300; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_LT(*simulation.cell().requireQuery("(pose_error easel)"), 2e-3);
    EXPECT_LT(*simulation.cell().requireQuery("(pose_error canvas)"), 5e-3);
    EXPECT_EQ(simulation.cell().requireQuery("(on canvas easel)"), 1.0);
    EXPECT_EQ(simulation.cell().requireQuery("(touching easel canvas)"), 1.0);
    EXPECT_EQ(simulation.cell().requireQuery("(collision_count robot)"), 0.0);
}

TEST(Painter, PortraitDipsAndMarks)
{
    PainterLogic logic;
    auto opened = robotik::require(
        robotik::StandaloneSimulation::open(painterScenario(), &logic));
    robotik::Simulation& simulation = opened.simulation();

    for (int i = 0; i < 20000 && !simulation.finished(); ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    EXPECT_EQ(simulation.outcome(),
              robotik::Simulation::Outcome::ControllerSucceeded)
        << robotik::toString(simulation.outcome());
    for (robotik::Check const& check : simulation.checks())
    {
        EXPECT_EQ(check.verdict, robotik::Check::Verdict::Passed)
            << check.text << ": " << check.detail << " at "
            << check.time.value() << " s";
    }
    EXPECT_TRUE(simulation.passed());
    EXPECT_GT(simulation.cell().requirePose("canvas").position.z, 0.5);
}
