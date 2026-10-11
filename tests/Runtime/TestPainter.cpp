// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PainterMission.hpp"
#include "PainterScene.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Runtime/Simulation.hpp"

#include <cmath>

TEST(Painter, CanvasRestsOnTheEasel)
{
    compages::world::World world;
    PainterMission mission;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(repoFile("plugins/Painter/scenarios/painter.yml"), testAssets()),
        nullptr,
        &mission);

    auto const* physics =
        dynamic_cast<robotik::MujocoBackend const*>(simulation.robot().backend());
    ASSERT_NE(physics, nullptr);
    std::optional<robotik::Pose> const easel0 = physics->bodyPose(painter::kEaselRoot);
    std::optional<robotik::Pose> const canvas0 = physics->bodyPose(painter::kCanvas);
    ASSERT_TRUE(easel0.has_value());
    ASSERT_TRUE(canvas0.has_value());
    EXPECT_NEAR(easel0->position.x, painter::kEaselDistance, 1e-3);
    EXPECT_NEAR(easel0->position.z, 0.0, 1e-3);
    EXPECT_NEAR(canvas0->position.z, painter::canvasPose().position.z, 1e-2);

    // The mission is suspended: nothing pushes. The canvas may settle 1 mm.
    simulation.suspend(true);
    for (int i = 0; i < 300; ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    std::optional<robotik::Pose> const easel = physics->bodyPose(painter::kEaselRoot);
    std::optional<robotik::Pose> const canvas = physics->bodyPose(painter::kCanvas);
    ASSERT_TRUE(easel.has_value());
    ASSERT_TRUE(canvas.has_value());
    EXPECT_NEAR(easel->position.x, easel0->position.x, 2e-3);
    EXPECT_NEAR(easel->position.y, easel0->position.y, 2e-3);
    EXPECT_NEAR(easel->position.z, easel0->position.z, 2e-3);
    double const drift = std::hypot(canvas->position.x - canvas0->position.x,
                                    canvas->position.y - canvas0->position.y,
                                    canvas->position.z - canvas0->position.z);
    EXPECT_LT(drift, 5e-3);
}

TEST(Painter, PortraitDipsAndMarks)
{
    compages::world::World world;
    PainterMission mission;
    robotik::Simulation simulation(
        world,
        robotik::Scenario::load(repoFile("plugins/Painter/scenarios/painter.yml"), testAssets()),
        nullptr,
        &mission);

    for (int i = 0; i < 80000 && !simulation.finished(); ++i)
    {
        simulation.step(robotik::Seconds(0.01));
    }
    if (simulation.status() != bt::Status::SUCCESS)
    {
        robotik::SkillScheduler const& skills = simulation.skills();
        for (robotik::SkillRun const& run : skills.trace())
        {
            ADD_FAILURE() << skills.name(run.skill) << " "
                          << robotik::toString(run.state) << " "
                          << robotik::toString(run.reason)
                          << " contacts=" << simulation.maxContacts();
        }
    }
    EXPECT_EQ(simulation.status(), bt::Status::SUCCESS);

    auto const* physics =
        dynamic_cast<robotik::MujocoBackend const*>(simulation.robot().backend());
    ASSERT_NE(physics, nullptr);
    std::optional<robotik::Pose> const canvas = physics->bodyPose(painter::kCanvas);
    ASSERT_TRUE(canvas.has_value());
    EXPECT_GT(canvas->position.z, 0.5);
}
