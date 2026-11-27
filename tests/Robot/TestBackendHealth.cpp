// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"

#include "Compages/World/World.hpp"

#include <memory>

using namespace robotik::literals;

/// A hardware driver without hardware: the joints follow their targets, and
/// the test cuts the link.
class FakeDriver final: public robotik::RobotBackend
{
public:

    void attach(robotik::RobotRuntime&) override {}
    void reset(robotik::RobotRuntime&) override {}
    void step(robotik::RobotRuntime&, robotik::Seconds) override {}

    [[nodiscard]] Health getHealth() const override
    {
        return link ? Health{} : Health{ false, "link lost" };
    }

    bool link = true; ///< The test cuts it.
};

// ----------------------------------------------------------------------------
TEST(BackendHealth, ALostLinkFailsTheActuatorsUntilItIsBack)
{
    compages::world::World world;
    robotik::RobotRuntime runtime(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    runtime.addActuator<robotik::JointGroup>("arm");
    auto owned = std::make_unique<FakeDriver>();
    FakeDriver& driver = *owned;
    runtime.connect(std::move(owned));
    robotik::ResourceManager const& resources = runtime.robot().getResources();

    // A driver is not a simulated world.
    EXPECT_EQ(runtime.getPhysics(), nullptr);

    runtime.step(0.01_s);
    EXPECT_TRUE(runtime.getHealth().ok);
    EXPECT_TRUE(resources.available("arm"));

    driver.link = false;
    runtime.step(0.01_s);
    EXPECT_FALSE(runtime.getHealth().ok);
    EXPECT_EQ(runtime.getHealth().why, "link lost");
    EXPECT_FALSE(resources.available("arm"));

    driver.link = true;
    runtime.step(0.01_s);
    EXPECT_TRUE(runtime.getHealth().ok);
    EXPECT_TRUE(resources.available("arm"));
}

// ----------------------------------------------------------------------------
TEST(BackendHealth, AFaultOfTheScenarioStaysAfterTheLinkIsBack)
{
    compages::world::World world;
    robotik::RobotRuntime runtime(
        world, assetFile("robotik.robots.simple_revolute_robot"));
    runtime.addActuator<robotik::JointGroup>("arm");
    auto owned = std::make_unique<FakeDriver>();
    FakeDriver& driver = *owned;
    runtime.connect(std::move(owned));

    runtime.getResources().fail("arm"); // a fault, before the loss
    driver.link = false;
    runtime.step(0.01_s);
    driver.link = true;
    runtime.step(0.01_s);
    EXPECT_FALSE(runtime.robot().getResources().available("arm"));
}
