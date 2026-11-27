// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// The smallest program of each layer, kept here so that it always builds and
// runs. The Robot layer one is OpenRobot.MovesAJointInAFewLines
// (tests/Robot/TestOpenRobot.cpp). doc/tutorials/02-three-programs.md shows them.

#include "main.hpp"

#include "PickPlaceLogic.hpp"

#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Task/OpenRoboticCell.hpp"
#include "RobotIK/Task/Skills/MotionSkills.hpp"

#include "Compages/World/World.hpp"

/// Task layer, without a scenario logic: a robot and a cube described in C++, no
/// file but the URDF, and one skill run until it ends.
TEST(LayerExamples, TaskRunsOneSkill)
{
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::EntitySpec cube;
    cube.id = "cube";
    cube.shape = robotik::ShapeSpec{};             // a 4 cm box
    cube.pose.position = robotik::Vector3(0.4, 0.2, 0.02);
    robotik::RoboticCellDescription description;
    description.world.entities = { arm, cube };

    auto opened = robotik::openRoboticCell(std::move(description));
    ASSERT_TRUE(opened) << opened.error();
    robotik::RoboticCell& cell = *opened.value();
    auto const raise = cell.skills().add<robotik::MoveJointSkill>(
        { .name = "Raise" }, "joint2", 0.5);
    EXPECT_EQ(cell.run(raise, robotik::Seconds(10.0)),
              robotik::SkillState::Succeeded);

    robotik::JointSet const& joints = cell.robot().getJoints();
    EXPECT_NEAR(
        joints.position(joints.requireRevolute("joint2")).value(), 0.5, 0.02);
    EXPECT_NEAR(
        cell.requirePose("cube").position.x, 0.4, 1e-6); // truth, for a measure
}

/// The pick-and-place world without its controller, and its scenario logic: the
/// skills and the observers of the planner (PickPlacePlanning.cpp).
static robotik::Result<robotik::OpenedRoboticCell>
pickAndPlace(PickPlaceLogic& p_logic)
{
    auto scenario = robotik::loadScenario(
        repoFile(
            "demos/PickAndPlace/scenarios/pick_and_place_pddl.scenario.yaml"),
        testAssets());
    if (!scenario)
    {
        return robotik::failure(scenario.error());
    }
    robotik::RoboticCellDescription description = scenario.take();
    description.controller = {};
    return robotik::openRoboticCell(std::move(description), &p_logic);
}

/// The goal of the pick and place, in the domain of the package.
static robotik::PlannerSpec cubeInBox(std::string const& p_goal)
{
    robotik::PlannerSpec spec;
    spec.domain =
        repoFile("demos/PickAndPlace/scenarios/pick_and_place.domain.pddl");
    spec.goal = { robotik::parseLiteral(p_goal).take() };
    return spec;
}

/// Task layer, a goal without a sequence of skills: the planner finds detect,
/// pick and place in the PDDL domain, the cell runs them.
TEST(LayerExamples, TaskAchievesAGoal)
{
    PickPlaceLogic logic(true, true);
    auto opened = pickAndPlace(logic);
    ASSERT_TRUE(opened) << opened.error();
    robotik::RoboticCell& cell = *opened.value();

    cell.achieve(cubeInBox("(in red_cube box)"));
    EXPECT_EQ(cell.run(robotik::Seconds(60.0)), robotik::Status::SUCCESS);
    EXPECT_EQ(cell.requireQuery("inside", { "red_cube", "box" }), 1.0);
}

/// A goal that names nothing in the domain is refused before any step.
TEST(LayerExamples, AchieveRefusesAGoalTheDomainDoesNotKnow)
{
    PickPlaceLogic logic(true, true);
    auto opened = pickAndPlace(logic);
    ASSERT_TRUE(opened) << opened.error();
    EXPECT_THROW(opened.value()->achieve(cubeInBox("(flying red_cube)")),
                 std::runtime_error);
}

/// What the hello scenario needs from C++: count the steps, stop after 2 s.
class HelloLogic final: public robotik::ScenarioLogic
{
public:

    void reset(robotik::RoboticCell& /*p_cell*/,
               robotik::Seed /*p_seed*/) override
    {
        m_ticks = 0;
    }

    void act(robotik::RobotContext& /*p_context*/,
             robotik::Seconds /*p_dt*/) override
    {
        ++m_ticks;
    }

    void measure(robotik::RoboticCell const& /*p_cell*/,
                 robotik::Metrics& p_metrics) const override
    {
        p_metrics.set("hello.ticks", static_cast<double>(m_ticks));
    }

    [[nodiscard]] robotik::Status
    status(robotik::RobotContext const& p_context) const override
    {
        return p_context.time >= robotik::Seconds(2.0)
                   ? robotik::Status::SUCCESS
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
        repoFile("doc/tutorials/HelloWorld/scenarios/hello.scenario.yaml"),
        testAssets());
    ASSERT_TRUE(scenario) << scenario.error();

    HelloLogic logic;
    compages::world::World world;
    robotik::Simulation simulation(world, scenario.take(), nullptr, &logic);
    while (!simulation.finished())
    {
        simulation.step(robotik::Seconds(0.01));
    }

    EXPECT_TRUE(simulation.passed());
    EXPECT_EQ(simulation.outcome(),
              robotik::Simulation::Outcome::ControllerSucceeded);
}
