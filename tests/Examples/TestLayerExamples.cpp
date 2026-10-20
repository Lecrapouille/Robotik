// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The smallest program of each layer, kept here so that it always builds and
// runs. The Robot layer one is OpenRobot.MovesAJointInAFewLines
// (tests/Robot/TestOpenRobot.cpp). doc/02-Getting-Started.md shows them.

#include "main.hpp"

#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Task/Cell.hpp"
#include "Robotik/Task/Skills/MotionSkills.hpp"

#include "Compages/World/World.hpp"

// ----------------------------------------------------------------------------
/// Task layer: a robot and a cube described in C++, no file but the URDF, and
/// a skill run by the scheduler.
TEST(LayerExamples, TaskCellInCpp)
{
    robotik::CellDescription cell_description;
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.shape = robotik::ShapeSpec{};
    cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
    cell_description.world.entities = { arm, cube };

    compages::world::World world;
    robotik::Cell cell(world, cell_description);
    robotik::SkillId const raise =
        cell.skills().add<robotik::MoveJointSkill>({ .name = "Raise" }, "joint2", 0.5);
    cell.skills().request(raise);
    while (cell.skills().state(raise) != robotik::SkillState::Succeeded &&
           cell.time() < robotik::Seconds(10.0))
    {
        cell.step(robotik::Seconds(0.01));
    }

    EXPECT_EQ(cell.skills().state(raise), robotik::SkillState::Succeeded);
    robotik::JointSet const& joints = cell.robot().joints();
    EXPECT_NEAR(joints.position(joints.revolute("joint2")).value(), 0.5, 0.02);
    EXPECT_NEAR(cell.pose("cube").position.x, 0.4, 1e-6); // truth, for a measure
}

// ----------------------------------------------------------------------------
/// What the hello scenario needs from C++: count the steps, stop after 2 s.
class HelloMission final: public robotik::Mission
{
public:

    void reset(robotik::Cell& /*p_cell*/, robotik::Seed /*p_seed*/) override
    {
        m_ticks = 0;
    }

    void step(robotik::Cell& /*p_cell*/, robotik::Seconds /*p_dt*/) override
    {
        ++m_ticks;
    }

    void measure(robotik::Cell const& /*p_cell*/, robotik::Metrics& p_metrics) const override
    {
        p_metrics.set("hello.ticks", static_cast<double>(m_ticks));
    }

    [[nodiscard]] robotik::Status status(robotik::Cell const& p_cell) const override
    {
        return p_cell.time() >= robotik::Seconds(2.0) ? robotik::Status::SUCCESS
                                                      : robotik::Status::RUNNING;
    }

private:

    int m_ticks = 0;
};

// ----------------------------------------------------------------------------
/// Package layer: a scenario file builds the cell and judges the run.
TEST(LayerExamples, PackageScenarioFile)
{
    auto scenario = robotik::loadScenario(
        repoFile("plugins/HelloWorld/scenarios/hello.scenario.yaml"), testAssets());
    ASSERT_TRUE(scenario) << scenario.error();

    HelloMission mission;
    compages::world::World world;
    robotik::Simulation simulation(world, scenario.take(), nullptr, &mission);
    while (!simulation.finished())
    {
        simulation.step(robotik::Seconds(0.01));
    }

    EXPECT_TRUE(simulation.passed());
    EXPECT_EQ(simulation.outcome(), robotik::Simulation::Outcome::ControllerSucceeded);
}
