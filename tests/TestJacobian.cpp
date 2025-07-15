#include "main.hpp"

#include "Robotik/Parser.hpp"
#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Test Jacobian calculation for Cartesian robot.
// *********************************************************************************
TEST(TestJacobian, JacobianTestCartesian)
{
    URDFParser parser;
    auto robot =
        parser.load("/home/qq/MyGitHub/Robotik/data/cartesian_robot.urdf");
    ASSERT_NE(robot, nullptr);

    auto names = robot->getJointNames();
    ASSERT_EQ(names.size(), 3);
    ASSERT_EQ(names[0], "joint1");
    ASSERT_EQ(names[1], "joint2");
    ASSERT_EQ(names[2], "joint3");

    auto jacobian = robot->calculateJacobian();
    std::cout << jacobian << std::endl;

    // auto end_effector_pose = robot->getEndEffectorPose();
    // std::cout << end_effector_pose << std::endl;
}

#if 0
// *********************************************************************************
//! \brief Test Jacobian calculation for SCARA robot.
// *********************************************************************************
TEST(TestJacobian, JacobianTestScara)
{
    URDFParser parser;
    auto robot = parser.load("/home/qq/MyGitHub/Robotik/data/scara_robot.urdf");
    ASSERT_NE(robot, nullptr);

    auto names = robot->getJointNames();
    for (const auto& name : names)
    {
        std::cout << "Joint: " << name << std::endl;
    }

    auto jacobian = robot->calculateJacobian();
    std::cout << jacobian << std::endl;

    auto end_effector_pose = robot->getEndEffectorPose();
    std::cout << end_effector_pose << std::endl;
}
#endif