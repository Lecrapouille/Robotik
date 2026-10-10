// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Skills/MotionSkills.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/World.hpp"

#include <stdexcept>

#define TEST_SIM_STEP_S 0.002

TEST(PinocchioBackend, NeutralPoseOfRevoluteArm)
{
    robotik::PinocchioBackend kinematics(dataFile("simple_revolute_robot.urdf"));
    EXPECT_EQ(kinematics.nq(), 1u);
    ASSERT_TRUE(kinematics.hasFrame("arm_link"));
    robotik::Pose const pose = kinematics.framePose("arm_link");
    EXPECT_NEAR(pose.position.z, 0.05, 1e-6);
    EXPECT_NEAR(pose.position.x, 0.0, 1e-6);
}

TEST(RobotSession, LoadsJointsFromUrdf)
{
    compages::world::World world;
    robotik::RobotSession robot(world, dataFile("robot_6axis.urdf"));
    EXPECT_EQ(robot.joints().size(), 6u);
    EXPECT_EQ(robot.joints().find("joint3"), 2u);
    EXPECT_EQ(robot.joints().find("nope"), robotik::NO_JOINT);
    EXPECT_FALSE(robot.tool().empty());
    EXPECT_TRUE(robot.link(robot.tool()));
}

TEST(RobotSession, GripperJointsJoinTheArm)
{
    compages::world::World world;
    robotik::RobotSession robot(world,
                                dataFile("robot_6axis.urdf"),
                                nullptr,
                                dataFile("tool_gripper.urdf"));
    EXPECT_EQ(robot.joints().size(), 8u);
    EXPECT_NE(robot.joints().find("gripper_finger_left_joint"),
              robotik::NO_JOINT);
    EXPECT_NE(robot.joints().find("gripper_finger_right_joint"),
              robotik::NO_JOINT);
    EXPECT_EQ(robot.tool(), "tcp");
}

//! The simulator loads meshes through a scene. That path must keep the fingers.
class SceneToolView : public robotik::SceneView
{
public:

    explicit SceneToolView(compages::renderer::Scene& p_scene) : m_scene(p_scene)
    {
    }

    compages::world::Entity robot(compages::world::World& /*p_world*/,
                                  std::filesystem::path const& p_urdf) override
    {
        return load(p_urdf);
    }

    compages::world::Entity model(compages::world::World& /*p_world*/,
                                  std::filesystem::path const& p_urdf) override
    {
        return load(p_urdf);
    }

private:

    compages::world::Entity load(std::filesystem::path const& p_urdf)
    {
        auto loaded = m_scene.load(p_urdf.string());
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        return loaded.value();
    }

    compages::renderer::Scene& m_scene;
};

TEST(RobotSession, GripperJointsJoinTheArmInTheScene)
{
    compages::world::World world;
    compages::renderer::Scene scene(world);
    SceneToolView view(scene);
    robotik::RobotSession robot(world,
                                dataFile("robot_6axis.urdf"),
                                &view,
                                dataFile("tool_gripper.urdf"));
    EXPECT_EQ(robot.joints().size(), 8u);
    EXPECT_TRUE(robot.joints().isPrismatic(
        robot.joints().find("gripper_finger_left_joint")));
    EXPECT_TRUE(robot.joints().isPrismatic(
        robot.joints().find("gripper_finger_right_joint")));
}

TEST(MujocoBackend, AttachesAndDetachesAToolChain)
{
    compages::world::World world;
    auto physics = std::make_unique<robotik::MujocoBackend>();
    std::string const arm = physics->load(dataFile("robot_6axis.urdf"));
    std::string const tool = physics->load(dataFile("tool_gripper.urdf"));
    EXPECT_THROW(physics->attach(arm, "missing", tool, "tool_mount"),
                 std::runtime_error);
    physics->attach(arm, "flange", tool, "tool_mount");
    robotik::MujocoBackend* const backend = physics.get();

    robotik::RobotSession robot(world, dataFile("robot_6axis.urdf"));
    robot.connect(std::move(physics));
    ASSERT_TRUE(robot.backend()->wrench("tcp").has_value());
    ASSERT_TRUE(robot.backend()->wrench("flange").has_value());

    backend->detach(arm, "flange", tool, "tool_mount");
    EXPECT_THROW(backend->detach(arm, "flange", tool, "tool_mount"),
                 std::runtime_error);
    EXPECT_TRUE(robot.backend()->wrench("tcp").has_value());
    EXPECT_TRUE(robot.backend()->wrench("flange").has_value());
    robot.step(Seconds(0.01));

    backend->attach(arm, "joint6", tool, "tool_mount");
    EXPECT_TRUE(robot.backend()->wrench("tool_mount").has_value());
    robot.step(Seconds(0.01));
}

TEST(RobotSession, MoveJointReachesTheCommand)
{
    compages::world::World world;
    std::filesystem::path const urdf = dataFile("simple_revolute_robot.urdf");
    robotik::RobotSession robot(world, urdf);
    auto physics = std::make_unique<robotik::MujocoBackend>();
    (void)physics->load(urdf);
    robot.connect(std::move(physics));
    robotik::WorldModel beliefs;
    robotik::RobotContext context{ robot, beliefs, {}, {} };
    robotik::MoveJointSkill skill("revolute_joint", 0.5, 0.05);
    skill.reset();

    robotik::Status status = robotik::Status::RUNNING;
    Seconds const step(TEST_SIM_STEP_S);
    for (int i = 0; i < 1500 && status == robotik::Status::RUNNING; ++i)
    {
        context.time = robot.time();
        context.dt = step;
        status = skill.tick(context, step);
        robot.step(step);
    }
    EXPECT_EQ(status, robotik::Status::SUCCESS);
    EXPECT_NEAR(robot.joints().position(0), 0.5, 0.05);
}

TEST(RobotSession, FailedActuatorIsDisabled)
{
    compages::world::World world;
    std::filesystem::path const urdf = dataFile("simple_diff_drive_robot.urdf");
    robotik::RobotSession robot(world, urdf);
    auto& motor = robot.actuators().add<robotik::Motor>(
        "left_motor", robot.joints().name(0));
    EXPECT_TRUE(robot.resources().available("left_motor"));

    motor.spin(AngularVelocity(2.0));
    EXPECT_EQ(robot.joints().mode(motor.joint().id), robotik::JointMode::Velocity);
    robot.resources().fail("left_motor");
    robot.step(Seconds(0.01));
    EXPECT_EQ(robot.joints().mode(motor.joint().id), robotik::JointMode::Disabled);
}
