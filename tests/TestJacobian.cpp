/**
 * @file TestJacobian.cpp
 * @brief Unit tests for Jacobian matrix computation - Verification of Jacobian
 * calculations, differential inverse kinematics and numerical validation for
 * robots loaded from the folder data/.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Path.hpp"
#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/URDFParser.hpp"

#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Jacobian matrix computation tests.
// *********************************************************************************
class JacobianTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        parser = std::make_unique<URDFParser>();
    }

    std::unique_ptr<Robot> parseRobot(const std::string& p_urdf_file_path)
    {
        Path path(project::info::paths::data);
        auto robot = parser->load(path.expand(p_urdf_file_path));
        if (robot == nullptr)
        {
            std::cerr << "Failed to load URDF file " << p_urdf_file_path << ": "
                      << parser->error();
        }
        return robot;
    }

    // Helper function to compare Jacobian matrices with tolerance
    void expectJacobianNear(const Jacobian& actual,
                            const Jacobian& expected,
                            double tolerance = 1e-6) const
    {
        ASSERT_EQ(actual.rows(), expected.rows())
            << "Jacobian row count mismatch";
        ASSERT_EQ(actual.cols(), expected.cols())
            << "Jacobian column count mismatch";

        for (int i = 0; i < actual.rows(); ++i)
        {
            for (int j = 0; j < actual.cols(); ++j)
            {
                EXPECT_NEAR(actual(i, j), expected(i, j), tolerance)
                    << "Jacobian mismatch at (" << i << ", " << j << ")";
            }
        }
    }

    std::unique_ptr<URDFParser> parser;
};

// *********************************************************************************
//! \brief Test Jacobian calculation for simple revolute robot.
// *********************************************************************************
TEST_F(JacobianTest, SimpleRevoluteRobotJacobian)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    // Find the end effector (arm_link is the end effector for this robot)
    auto end_effector = scene::Node::find(robot->root(), "arm_link");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Set joint to zero position
    robot->setJointValues({ 0.0 });

    // Calculate Jacobian
    Jacobian jacobian = robot->jacobian(*end_effector);

    // Expected Jacobian for simple revolute robot at zero position
    // The joint and end effector are at same Z position (0.05), so no linear
    // velocity Only angular velocity around Z-axis
    Jacobian expected(6, 1);
    expected << 0.0, // vx
        0.0,         // vy
        0.0,         // vz
        0.0,         // ωx
        0.0,         // ωy
        1.0;         // ωz

    expectJacobianNear(jacobian, expected);
}

// *********************************************************************************
//! \brief Test Jacobian calculation for simple prismatic robot.
// *********************************************************************************
TEST_F(JacobianTest, SimplePrismaticRobotJacobian)
{
    auto robot = parseRobot("simple_prismatic_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_prismatic_robot.urdf";

    // Find the end effector
    auto end_effector = scene::Node::find(robot->root(), "end_effector_link");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Set joint to zero position
    robot->setJointValues({ 0.0 });

    // Calculate Jacobian
    Jacobian jacobian = robot->jacobian(*end_effector);

    // Expected Jacobian for simple prismatic robot
    // For prismatic joint: v = axis, ω = 0
    // axis = (0, 0, 1)
    Jacobian expected(6, 1);
    expected << 0.0, // vx
        0.0,         // vy
        1.0,         // vz
        0.0,         // ωx
        0.0,         // ωy
        0.0;         // ωz

    expectJacobianNear(jacobian, expected);
}

// *********************************************************************************
//! \brief Test Jacobian calculation for differential drive robot.
// *********************************************************************************
TEST_F(JacobianTest, DifferentialDriveRobotJacobian)
{
    auto robot = parseRobot("simple_diff_drive_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_diff_drive_robot.urdf";

    // Find a reference point (base_link for this robot)
    auto base_link = scene::Node::find(robot->root(), "base_link");
    ASSERT_NE(base_link, nullptr) << "Base link node not found";

    // Set joints to zero position
    auto joint_names = robot->jointNames();
    std::vector<double> zero_values(joint_names.size(), 0.0);
    robot->setJointValues(zero_values);

    // Calculate Jacobian
    Jacobian jacobian = robot->jacobian(*base_link);

    // Expected Jacobian for differential drive robot (2 continuous joints)
    // Based on actual computed values for the wheel positions
    Jacobian expected(6, 2);
    expected << 0.0249994, 0.0250006, // vx
        -5.50981e-07, -5.50981e-07,   // vy
        0.15, 0.15,                   // vz
        0.0, 0.0,                     // ωx
        -1.0, -1.0,                   // ωy
        -3.67321e-06, -3.67321e-06;   // ωz

    expectJacobianNear(jacobian, expected, 1e-5);
}

// *********************************************************************************
//! \brief Test Jacobian calculation for SCARA robot.
// *********************************************************************************
TEST_F(JacobianTest, SCARArobotJacobian)
{
    auto robot = parseRobot("scara_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load scara_robot.urdf";

    // Find the end effector
    auto end_effector = scene::Node::find(robot->root(), "link_end_effector");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Set joints to zero position
    robot->setJointValues({ 0.0, 0.0, 0.0 });

    // Calculate Jacobian
    Jacobian jacobian = robot->jacobian(*end_effector);

    // Expected Jacobian for SCARA robot at zero configuration
    // Joint 1: revolute at (0, 0, 0.06), end effector at (0.8, 0, 0.06)
    // Joint 2: revolute at (0.35, 0, 0.06), end effector at (0.8, 0, 0.06)
    // Joint 3: prismatic at (0.8, 0, 0.06), moves along -Z axis
    Jacobian expected(6, 3);
    expected << 0.0, 0.0, 0.0, // vx
        0.8, 0.45, 0.0,        // vy
        0.0, 0.0, -1.0,        // vz
        0.0, 0.0, 0.0,         // ωx
        0.0, 0.0, 0.0,         // ωy
        1.0, 1.0, 0.0;         // ωz

    expectJacobianNear(jacobian, expected);
}

// *********************************************************************************
//! \brief Test Jacobian calculation for Cartesian robot.
// *********************************************************************************
TEST_F(JacobianTest, CartesianRobotJacobian)
{
    auto robot = parseRobot("cartesian_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load cartesian_robot.urdf";

    // Find the end effector (link_z is the end effector)
    auto end_effector = scene::Node::find(robot->root(), "link_z");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Set joints to zero position
    robot->setJointValues({ 0.0, 0.0, 0.0 });

    // Calculate Jacobian
    Jacobian jacobian = robot->jacobian(*end_effector);

    // Expected Jacobian for Cartesian robot
    // All joints are prismatic along X, Y, Z axes respectively
    Jacobian expected(6, 3);
    expected << 1.0, 0.0, 0.0, // vx
        0.0, 1.0, 0.0,         // vy
        0.0, 0.0, 1.0,         // vz
        0.0, 0.0, 0.0,         // ωx
        0.0, 0.0, 0.0,         // ωy
        0.0, 0.0, 0.0;         // ωz

    expectJacobianNear(jacobian, expected);
}