#include "main.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Skills/MoveJointSkill.hpp"

#include "Compages/Core/Units.hpp"
#include "Compages/World/World.hpp"

#include <filesystem>
#include <fstream>
#include <string>

#define TEST_SIM_STEP_S 0.002

static std::filesystem::path revoluteUrdf()
{
    std::array<std::filesystem::path, 2> const candidates = {
        "data/simple_revolute_robot.urdf",
        "../data/simple_revolute_robot.urdf",
    };
    for (std::filesystem::path const& candidate : candidates)
    {
        std::ifstream file(candidate);
        if (file)
        {
            return candidate;
        }
    }
    return candidates[0];
}

TEST(PinocchioBackend, NeutralPoseOfRevoluteArm)
{
    robotik::PinocchioBackend kinematics(revoluteUrdf());
    EXPECT_EQ(kinematics.nq(), 1u);
    ASSERT_TRUE(kinematics.hasFrame("arm_link"));
    robotik::Pose const pose = kinematics.framePose("arm_link");
    EXPECT_NEAR(pose.pz, 0.05, 1e-6);
    EXPECT_NEAR(pose.px, 0.0, 1e-6);
}

TEST(RobotRuntime, MoveJointReachesTheCommand)
{
    compages::world::World world;
    robotik::RobotRuntime runtime(world, revoluteUrdf());
    robotik::MoveJointSkill skill("revolute_joint", 0.5, 0.05);

    robotik::Status status = robotik::Status::RUNNING;
    Seconds const sim_step(TEST_SIM_STEP_S);
    for (int i = 0; i < 1500 && status == robotik::Status::RUNNING; ++i)
    {
        robotik::RobotContext context = runtime.context();
        status = skill.tick(context, sim_step);
        runtime.step(sim_step);
    }

    EXPECT_EQ(status, robotik::Status::SUCCESS);
}
