#include "main.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Skills/MoveJointSkill.hpp"

#include "Compages/World/World.hpp"

#include <fstream>
#include <string>

namespace
{

std::string revoluteUrdf()
{
    char const* candidates[] = {
        "data/simple_revolute_robot.urdf",
        "../data/simple_revolute_robot.urdf",
    };
    for (char const* candidate : candidates)
    {
        std::ifstream file(candidate);
        if (file)
        {
            return candidate;
        }
    }
    return candidates[0];
}

} // namespace

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

    robotik::Status status = robotik::Status::Running;
    for (int step = 0; step < 1500 && status == robotik::Status::Running;
         ++step)
    {
        robotik::RobotContext context = runtime.context();
        status = skill.tick(context, 0.002);
        runtime.step(0.002);
    }

    EXPECT_EQ(status, robotik::Status::Success);
}
