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

#include "Robotik/Parser.hpp"
#include "Robotik/Robot.hpp"
#include "Robotik/private/Conversions.hpp"

#include <cmath>

#define TEST_DATA_DIR "/home/qq/MyGitHub/Robotik/data/"

using namespace robotik;

namespace helper
{

#if 0
// Helper function to check if two poses are approximately equal
static bool
posesAreEqual(const Pose& pose1, const Pose& pose2, double tolerance = 1e-3)
{
    return (pose1 - pose2).norm() < tolerance;
}
#endif

} // namespace helper

// *********************************************************************************
//! \brief Test fixture for SCARA robot Jacobian and kinematics tests.
// *********************************************************************************
class ScaraRobotTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        parser = std::make_unique<URDFParser>();
        parseRobot("scara_robot.urdf");
    }

    void parseRobot(const std::string& p_urdf_file_path)
    {
        // Load the robot from URDF
        std::string file_path = std::string(TEST_DATA_DIR) + p_urdf_file_path;
        robot = parser->load(file_path);
        ASSERT_NE(robot, nullptr)
            << "Failed to load " << file_path << ": " << parser->getError();

        // Find the end effector node
        end_effector = scene::Node::find(robot->root(), "link_end_effector");
        ASSERT_NE(end_effector, nullptr) << "End effector node not found";

        // Get the number of joints
        joint_names = robot->jointNames();
        ASSERT_EQ(joint_names.size(), 3) << "Expected 3 joints for SCARA robot";
    }

    // Helper function to set joint values and get end effector pose
    Pose getEndEffectorPose(const std::initializer_list<double>& joint_values)
    {
        robot->setJointValues(joint_values);
        return utils::transformToPose(end_effector->worldTransform());
    }

    std::unique_ptr<URDFParser> parser;
    std::unique_ptr<Robot> robot;
    scene::Node const* end_effector;
    std::vector<std::string> joint_names;
};

// *********************************************************************************
//! \brief Test forward kinematics with all joints at 0 degrees.
// *********************************************************************************
TEST_F(ScaraRobotTest, ForwardKinematicsAllJointsZero)
{
    Pose pose = getEndEffectorPose({ 0.0, 0.0, 0.0 });
    std::cout << utils::printPose("All joints at 0°", pose) << std::endl;

    EXPECT_NEAR(pose(0), 0.8, 0.01);   // x position
    EXPECT_NEAR(pose(1), 0.0, 0.01);   // y position
    EXPECT_NEAR(pose(2), 0.022, 0.01); // z position
    EXPECT_NEAR(pose(3), 0.0, 0.01);   // roll
    EXPECT_NEAR(pose(4), 0.0, 0.01);   // pitch
    EXPECT_NEAR(pose(5), 0.0, 0.01);   // yaw
}

// *********************************************************************************
//! \brief Test forward kinematics with all joints at 90 degrees.
// *********************************************************************************
TEST_F(ScaraRobotTest, ForwardKinematicsAllJoints90Degrees)
{
    Pose pose = getEndEffectorPose({ M_PI / 2, M_PI / 2, 0.0 });
    std::cout << utils::printPose("All joints at 90°", pose) << std::endl;

    EXPECT_NEAR(pose(0), -0.45, 0.01); // x position
    EXPECT_NEAR(pose(1), 0.35, 0.01);  // y position
    EXPECT_NEAR(pose(2), 0.022, 0.01); // z position
    EXPECT_NEAR(pose(3), 0.0, 0.01);   // roll
    EXPECT_NEAR(pose(4), 0.0, 0.01);   // pitch
    EXPECT_NEAR(pose(5), M_PI, 0.01);  // yaw
}

// *********************************************************************************
//! \brief Test forward kinematics with all joints at 180 degrees.
// *********************************************************************************
TEST_F(ScaraRobotTest, ForwardKinematicsAllJoints180Degrees)
{
    Pose pose = getEndEffectorPose({ M_PI, M_PI, 0.0 });
    std::cout << utils::printPose("All joints at 180°", pose) << std::endl;

    EXPECT_NEAR(pose(0), 0.1, 0.1);    // x position
    EXPECT_NEAR(pose(1), 0.0, 0.1);    // y position
    EXPECT_NEAR(pose(2), 0.022, 0.01); // z position
    EXPECT_NEAR(pose(3), M_PI, 0.01);  // roll
    EXPECT_NEAR(pose(4), -M_PI, 0.01); // pitch
    EXPECT_NEAR(pose(5), M_PI, 0.01);  // yaw
}

// *********************************************************************************
//! \brief Test Jacobian calculation for different joint configurations.
// *********************************************************************************
TEST_F(ScaraRobotTest, JacobianCalculation)
{
    // Test Jacobian at zero configuration
    robot->setJointValues({ 0.0, 0.0, 0.0 });
    Jacobian jacobian = robot->calculateJacobian(*end_effector);
    std::cout << "Jacobian with all joints at 0°:" << std::endl
              << jacobian << std::endl;

    // Jacobian should be 6x3
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), 3);

#if 0
    // Test Jacobian at 90 degrees configuration
    robot->setJointValues({ M_PI / 2, M_PI / 2, 0.0 });
    Jacobian jacobian_90 = robot->calculateJacobian(*end_effector);
    std::cout << "Jacobian with all joints at 0°:" << std::endl
              << jacobian << std::endl;

    // Jacobian should have correct dimensions
    EXPECT_EQ(jacobian_90.rows(), 6);
    EXPECT_EQ(jacobian_90.cols(), 3);
#endif
}

#if 0
// *********************************************************************************
//! \brief Test forward kinematics with different prismatic joint values.
// *********************************************************************************
TEST_F(ScaraRobotTest, ForwardKinematicsPrismaticJoint)
{
    // Test with minimum prismatic value
    std::vector<double> joint_values_min = { 0.0, 0.0, 0.038 };
    Pose pose_min = getEndEffectorPose(joint_values_min);

    // Test with maximum prismatic value
    std::vector<double> joint_values_max = { 0.0, 0.0, 0.16 };
    Pose pose_max = getEndEffectorPose(joint_values_max);

    printPose("Prismatic at minimum", pose_min);
    printPose("Prismatic at maximum", pose_max);

    // X and Y positions should be the same (same revolute joint angles)
    EXPECT_NEAR(pose_min(0), pose_max(0), 0.01); // x position
    EXPECT_NEAR(pose_min(1), pose_max(1), 0.01); // y position

    // Z position should be different (prismatic joint affects height)
    // Note: prismatic joint moves in negative Z direction
    EXPECT_LT(
        pose_max(2),
        pose_min(2)); // max z should be less than min z (negative direction)
    EXPECT_NEAR(pose_min(2) - pose_max(2),
                0.16 - 0.038,
                0.01); // difference should match joint range
}

// *********************************************************************************
//! \brief Test inverse kinematics with known reachable poses.
// *********************************************************************************
TEST_F(ScaraRobotTest, InverseKinematicsKnownPoses)
{
    // Test 1: Inverse kinematics for the zero configuration pose
    std::vector<double> target_joints = { 0.0, 0.0, 0.038 };
    Pose target_pose = getEndEffectorPose(target_joints);

    printPose("Target pose for IK", target_pose);

    std::vector<double> solution =
        robot->inverseKinematics(target_pose, *end_effector);

    // Should find a solution
    EXPECT_EQ(solution.size(), 3);

    // Verify the solution by forward kinematics
    robot->setJointValues(solution);
    Pose achieved_pose = utils::transformToPose(end_effector->worldTransform());

    // The achieved pose should be close to the target pose
    EXPECT_TRUE(posesAreEqual(target_pose, achieved_pose, 1e-2));

    printPose("Achieved pose from IK", achieved_pose);

    // Test 2: Inverse kinematics for a different configuration
    target_joints = { M_PI / 4, M_PI / 4, 0.1 };
    target_pose = getEndEffectorPose(target_joints);

    printPose("Target pose for IK (45° each)", target_pose);

    solution = robot->inverseKinematics(target_pose, *end_effector);

    // Should find a solution
    EXPECT_EQ(solution.size(), 3);

    // Verify the solution
    robot->setJointValues(solution);
    achieved_pose = utils::transformToPose(end_effector->worldTransform());

    // The achieved pose should be close to the target pose
    EXPECT_TRUE(posesAreEqual(target_pose, achieved_pose, 1e-2));

    printPose("Achieved pose from IK (45° each)", achieved_pose);
}

// *********************************************************************************
//! \brief Test inverse kinematics with unreachable poses.
// *********************************************************************************
TEST_F(ScaraRobotTest, InverseKinematicsUnreachablePoses)
{
    // Test with a pose that's too far away (outside workspace)
    Pose unreachable_pose;
    unreachable_pose << 10.0, 10.0, 10.0, 0.0, 0.0, 0.0; // Very far position

    std::vector<double> solution =
        robot->inverseKinematics(unreachable_pose, *end_effector);

    // Should return empty solution for unreachable pose
    EXPECT_EQ(solution.size(), 0);

    // Test with a pose that's too high (beyond prismatic joint limits)
    Pose too_high_pose;
    too_high_pose << 0.5, 0.0, 1.0, 0.0, 0.0, 0.0; // Very high z position

    solution = robot->inverseKinematics(too_high_pose, *end_effector);

    // Should return empty solution for pose beyond joint limits
    EXPECT_EQ(solution.size(), 0);
}

// *********************************************************************************
//! \brief Test multiple joint configurations systematically.
// *********************************************************************************
TEST_F(ScaraRobotTest, MultipleJointConfigurations)
{
    std::vector<std::vector<double>> test_configurations = {
        { 0.0, 0.0, 0.038 },           // All zero
        { M_PI / 6, 0.0, 0.038 },      // 30° first joint
        { 0.0, M_PI / 6, 0.038 },      // 30° second joint
        { M_PI / 6, M_PI / 6, 0.038 }, // 30° both joints
        { M_PI / 4, M_PI / 4, 0.1 },   // 45° both joints, middle prismatic
        { M_PI / 2, M_PI / 2, 0.16 },  // 90° both joints, max prismatic
        { M_PI, M_PI, 0.038 },         // 180° both joints
        { -M_PI / 4, M_PI / 4, 0.12 }  // Mixed angles
    };

    for (size_t i = 0; i < test_configurations.size(); ++i)
    {
        const auto& config = test_configurations[i];

        // Test forward kinematics
        Pose pose = getEndEffectorPose(config);

        // Verify pose is valid (not NaN or infinite)
        EXPECT_TRUE(pose.array().isFinite().all())
            << "Configuration " << i << " produced invalid pose";
        EXPECT_FALSE(pose.hasNaN())
            << "Configuration " << i << " produced NaN pose";

        // Test Jacobian calculation
        robot->setJointValues(config);
        Jacobian jacobian = robot->calculateJacobian(*end_effector);

        // Verify Jacobian is valid
        EXPECT_EQ(jacobian.rows(), 6) << "Configuration " << i;
        EXPECT_EQ(jacobian.cols(), 3) << "Configuration " << i;
        EXPECT_TRUE(jacobian.array().isFinite().all())
            << "Configuration " << i << " produced invalid Jacobian";

        // Test inverse kinematics (if pose is reachable)
        std::vector<double> solution =
            robot->inverseKinematics(pose, *end_effector);

        if (solution.size() > 0)
        {
            // Verify the solution
            robot->setJointValues(solution);
            Pose achieved_pose =
                utils::transformToPose(end_effector->worldTransform());

            // The achieved pose should be close to the original pose
            EXPECT_TRUE(posesAreEqual(pose, achieved_pose, 1e-2))
                << "Configuration " << i << " IK solution verification failed";
        }

        std::cout << "Configuration " << i << " [" << config[0] << ", "
                  << config[1] << ", " << config[2] << "] - Pose: [" << pose(0)
                  << ", " << pose(1) << ", " << pose(2) << "]" << std::endl;
    }
}

// *********************************************************************************
//! \brief Test singularity conditions.
// *********************************************************************************
TEST_F(ScaraRobotTest, SingularityConditions)
{
    // Test at a potential singular configuration (arm fully extended)
    std::vector<double> singular_config = { 0.0, 0.0, 0.038 };
    robot->setJointValues(singular_config);

    Jacobian jacobian = robot->calculateJacobian(*end_effector);

    // Check if Jacobian is close to singular (determinant close to zero)
    // For a 6x3 Jacobian, we can check the condition number or singular values
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
    double condition_number =
        svd.singularValues()(0) /
        svd.singularValues()(svd.singularValues().size() - 1);

    std::cout << "Condition number at singular configuration: "
              << condition_number << std::endl;

    // The condition number should be finite
    EXPECT_TRUE(std::isfinite(condition_number));

    // Test inverse kinematics near singularity
    Pose target_pose = utils::transformToPose(end_effector->worldTransform());
    std::vector<double> solution =
        robot->inverseKinematics(target_pose, *end_effector);

    // Should still find a solution (though it might be less accurate)
    EXPECT_EQ(solution.size(), 3);
}

// *********************************************************************************
//! \brief Test joint limits enforcement.
// *********************************************************************************
TEST_F(ScaraRobotTest, JointLimitsEnforcement)
{
    // Test setting values beyond joint limits
    std::vector<double> beyond_limits = { 2 * M_PI,
                                          2 * M_PI,
                                          0.5 }; // Beyond limits

    // The robot should handle this gracefully (either clamp or reject)
    bool success = robot->setJointValues(beyond_limits);

    // Get the actual values that were set
    std::vector<double> actual_values = robot->jointValues();

    // Verify that the values are within reasonable bounds
    for (size_t i = 0; i < actual_values.size(); ++i)
    {
        EXPECT_TRUE(std::isfinite(actual_values[i]))
            << "Joint " << i << " has invalid value";
    }

    // Test with valid values
    std::vector<double> valid_values = { M_PI / 4, M_PI / 4, 0.1 };
    success = robot->setJointValues(valid_values);

    EXPECT_TRUE(success);

    std::vector<double> retrieved_values = robot->jointValues();
    EXPECT_EQ(retrieved_values.size(), 3);

    for (size_t i = 0; i < valid_values.size(); ++i)
    {
        EXPECT_NEAR(retrieved_values[i], valid_values[i], 1e-6)
            << "Joint " << i;
    }
}

#endif