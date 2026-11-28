// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Robot/Backends/MujocoBackend.hpp"
#include "RobotIK/Robot/Kinematics.hpp"
#include "RobotIK/Robot/RobotRuntime.hpp"
#include "RobotIK/Task/Perception/WorldModel.hpp"
#include "RobotIK/Task/RobotContext.hpp"
#include "RobotIK/Task/Skills/MotionSkills.hpp"

#include "Compages/Renderer/Scene/Scene.hpp"
#include "Compages/World/World.hpp"
#include "RobotIK/Common/Math.hpp"

#include <stdexcept>

#define TEST_SIM_STEP_S 0.002

TEST(Kinematics, NeutralPoseOfRevoluteArm)
{
    robotik::Kinematics kinematics(
        assetFile("robotik.robots.simple_revolute_robot"));
    EXPECT_EQ(kinematics.nq(), 1U);
    ASSERT_TRUE(kinematics.hasFrame("arm_link"));
    robotik::Pose const pose = kinematics.requireFramePose("arm_link");
    EXPECT_NEAR(pose.position.z, 0.05, 1e-6);
    EXPECT_NEAR(pose.position.x, 0.0, 1e-6);
}

TEST(RobotRuntime, LoadsJointsFromUrdf)
{
    compages::world::World world;
    robotik::RobotRuntime robot(world, assetFile("robotik.robots.robot_6axis"));
    EXPECT_EQ(robot.robot().getJoints().size(), 6U);
    EXPECT_EQ(robot.robot().getJoints().find("joint3"), 2U);
    EXPECT_EQ(robot.robot().getJoints().find("nope"), robotik::NO_JOINT);
    EXPECT_FALSE(robot.robot().getTool().empty());
    EXPECT_TRUE(robot.getLink(robot.robot().getTool()));
}

TEST(RobotRuntime, GripperJointsJoinTheArm)
{
    compages::world::World world;
    robotik::RobotRuntime robot(world,
                                assetFile("robotik.robots.robot_6axis"),
                                nullptr,
                                { .urdf = assetFile("robotik.tools.gripper") });
    EXPECT_EQ(robot.robot().getJoints().size(), 8U);
    EXPECT_NE(robot.robot().getJoints().find("gripper_finger_left_joint"),
              robotik::NO_JOINT);
    EXPECT_NE(robot.robot().getJoints().find("gripper_finger_right_joint"),
              robotik::NO_JOINT);
    EXPECT_EQ(robot.robot().getTool(), "tcp");
}

/// The simulator loads meshes through a scene. That path must keep the fingers.
class SceneToolView: public robotik::SceneView
{
public:

    explicit SceneToolView(compages::renderer::Scene& p_scene)
        : m_scene(p_scene)
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
        auto loaded = m_scene.loadFile(p_urdf);
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        return loaded.value();
    }

    compages::renderer::Scene& m_scene;
};

TEST(RobotRuntime, GripperJointsJoinTheArmInTheScene)
{
    compages::world::World world;
    compages::renderer::Scene scene(world);
    SceneToolView view(scene);
    robotik::RobotRuntime robot(world,
                                assetFile("robotik.robots.robot_6axis"),
                                &view,
                                { .urdf = assetFile("robotik.tools.gripper") });
    EXPECT_EQ(robot.robot().getJoints().size(), 8U);
    EXPECT_TRUE(robot.robot().getJoints().isPrismatic(
        robot.robot().getJoints().find("gripper_finger_left_joint")));
    EXPECT_TRUE(robot.robot().getJoints().isPrismatic(
        robot.robot().getJoints().find("gripper_finger_right_joint")));
}

TEST(MujocoBackend, AttachesAndDetachesAToolChain)
{
    compages::world::World world;
    auto physics = std::make_unique<robotik::MujocoBackend>();
    std::string const arm =
        physics->load(assetFile("robotik.robots.robot_6axis"));
    std::string const tool = physics->load(assetFile("robotik.tools.gripper"));
    EXPECT_THROW(physics->attach(arm, "missing", tool, "tool_mount"),
                 std::runtime_error);
    physics->attach(arm, "flange", tool, "tool_mount");
    robotik::MujocoBackend* const backend = physics.get();

    robotik::RobotRuntime robot(world, assetFile("robotik.robots.robot_6axis"));
    robot.connect(std::move(physics));
    ASSERT_TRUE(robot.getPhysics()->wrench("tcp").has_value());
    ASSERT_TRUE(robot.getPhysics()->wrench("flange").has_value());

    backend->detach(arm, "flange", tool, "tool_mount");
    EXPECT_THROW(backend->detach(arm, "flange", tool, "tool_mount"),
                 std::runtime_error);
    EXPECT_TRUE(robot.getPhysics()->wrench("tcp").has_value());
    EXPECT_TRUE(robot.getPhysics()->wrench("flange").has_value());
    robot.step(robotik::Seconds(0.01));

    backend->attach(arm, "joint6", tool, "tool_mount");
    EXPECT_TRUE(robot.getPhysics()->wrench("tool_mount").has_value());
    robot.step(robotik::Seconds(0.01));
}

TEST(RobotRuntime, MoveJointReachesTheCommand)
{
    compages::world::World world;
    std::filesystem::path const urdf =
        assetFile("robotik.robots.simple_revolute_robot");
    robotik::RobotRuntime robot(world, urdf);
    auto physics = std::make_unique<robotik::MujocoBackend>();
    (void)physics->load(urdf);
    robot.connect(std::move(physics));
    robotik::WorldModel beliefs;
    robotik::RobotContext context{ robot.robot(), beliefs, {}, {} };
    robotik::MoveJointSkill skill("revolute_joint", 0.5, 0.05);
    skill.reset();

    robotik::Status status = robotik::Status::RUNNING;
    robotik::Seconds const step(TEST_SIM_STEP_S);
    for (int i = 0; i < 1500 && status == robotik::Status::RUNNING; ++i)
    {
        context.time = robot.getTime();
        context.dt = step;
        status = skill.tick(context, step);
        robot.step(step);
    }
    EXPECT_EQ(status, robotik::Status::SUCCESS);
    EXPECT_NEAR(robot.robot().getJoints().position(0), 0.5, 0.05);
}

TEST(RobotRuntime, FailedActuatorIsDisabled)
{
    compages::world::World world;
    std::filesystem::path const urdf =
        assetFile("robotik.robots.simple_diff_drive_robot");
    robotik::RobotRuntime robot(world, urdf);
    auto& motor = robot.addActuator<robotik::Motor>(
        "left_motor", robot.robot().getJoints().name(0));
    EXPECT_TRUE(robot.getResources().available("left_motor"));

    motor.spin(robotik::AngularVelocity(2.0));
    EXPECT_EQ(robot.robot().getJoints().mode(motor.joint().id),
              robotik::JointMode::Velocity);
    robot.getResources().fail("left_motor");
    robot.step(robotik::Seconds(0.01));
    EXPECT_EQ(robot.robot().getJoints().mode(motor.joint().id),
              robotik::JointMode::Disabled);
}
