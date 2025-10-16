/**
 * @file TestRobot.cpp
 * @brief Unit tests for the Robot class - Verification of forward and inverse
 * kinematics, Jacobian matrix computation, and end-effector transformations.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Conversions.hpp"
#include "Robotik/Core/Robot.hpp"

#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief End effector node. For this current implementation, a node is enough.
// *********************************************************************************
class EndEffector: public hierarchy::Node
{
public:

    using Node = hierarchy::Node;
    using Node::Node;
};

// *********************************************************************************
//! \brief Test fixture for Robot class.
// *********************************************************************************
class RobotTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Create a simple 2-DOF arm for testing
        robot_arm = std::make_unique<Robot>("test_arm");

        // Create the scene graph
        Joint::Ptr r = hierarchy::Node::create<Joint>(
            "root", Joint::Type::FIXED, Eigen::Vector3d(0, 0, 1));
        joint1 = &(r->createChild<Joint>(
            "joint1", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1)));
        joint2 = &(joint1->createChild<Joint>(
            "joint2", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1)));
        end_effector = &(joint2->createChild<EndEffector>("end_effector"));

        // Set up initial transforms (simple arm with 1 unit links)
        Transform joint1_transform = Eigen::Matrix4d::Identity();
        joint1_transform(2, 3) = 1.0; // 1 unit up
        joint1->localTransform(joint1_transform);

        Transform joint2_transform = Eigen::Matrix4d::Identity();
        joint2_transform(0, 3) = 1.0; // 1 unit forward
        joint2->localTransform(joint2_transform);

        Transform end_effector_transform = Eigen::Matrix4d::Identity();
        end_effector_transform(0, 3) = 1.0; // 1 unit forward
        end_effector->localTransform(end_effector_transform);

        // Set up the robot arm, base frame and end effector
        root = r.get();
        robot_arm->root(std::move(r));
    }

    std::unique_ptr<Robot> robot_arm;
    Joint* root;
    Joint* joint1;
    Joint* joint2;
    EndEffector* end_effector;
};

// *********************************************************************************
//! \brief Test root node access.
// *********************************************************************************
TEST_F(RobotTest, RootSceneNode)
{
    hierarchy::Node const& found_root = robot_arm->root();
    EXPECT_EQ(found_root.name(), "root");
}

// *********************************************************************************
//! \brief Test getting nodes by name.
// *********************************************************************************
TEST_F(RobotTest, SceneNodeSearch)
{
    EXPECT_EQ(hierarchy::Node::find(robot_arm->root(), "root"), root);
    EXPECT_EQ(hierarchy::Node::find(robot_arm->root(), "joint1"), joint1);
    EXPECT_EQ(hierarchy::Node::find(robot_arm->root(), "joint2"), joint2);
    EXPECT_EQ(hierarchy::Node::find(robot_arm->root(), "end_effector"),
              end_effector);
    EXPECT_EQ(hierarchy::Node::find(robot_arm->root(), "nonexistent"), nullptr);
}

// *********************************************************************************
//! \brief Test getting joint by name.
// *********************************************************************************
TEST_F(RobotTest, GetJoints)
{
    std::vector<std::string> joint_names = robot_arm->jointNames();

    // Note: root and end_effector are not actuable joints
    EXPECT_EQ(joint_names.size(), 2);
    EXPECT_EQ(joint_names[0], "joint1");
    EXPECT_EQ(joint_names[1], "joint2");
}

// *********************************************************************************
//! \brief Test setting and getting joint values.
// *********************************************************************************
TEST_F(RobotTest, JointValues)
{
    std::vector<double> values = { M_PI / 4.0, M_PI / 2.0 };
    robot_arm->setJointValues(values);

    std::vector<double> retrieved_values = robot_arm->jointPositions();

    EXPECT_EQ(retrieved_values.size(), 2);
    EXPECT_DOUBLE_EQ(retrieved_values[0], values[0]);
    EXPECT_DOUBLE_EQ(retrieved_values[1], values[1]);
}

// *********************************************************************************
//! \brief Test local and world transforms.
// *********************************************************************************
TEST_F(RobotTest, Transforms)
{
    Transform expected_matrix;

    // Check root node local and world transforms
    Transform world_transform = root->worldTransform();
    Transform local_transform = root->localTransform();
    EXPECT_TRUE(local_transform.isApprox(Eigen::Matrix4d::Identity()));
    EXPECT_TRUE(world_transform.isApprox(Eigen::Matrix4d::Identity()));

    // Check joint1 node local and world transforms
    world_transform = joint1->worldTransform();
    local_transform = joint1->localTransform();
    expected_matrix = Eigen::Matrix4d::Identity();
    expected_matrix(2, 3) = 1.0;
    EXPECT_TRUE(local_transform.isApprox(expected_matrix));
    EXPECT_TRUE(world_transform.isApprox(expected_matrix));

    // Check joint2 node local and world transforms
    world_transform = joint2->worldTransform();
    local_transform = joint2->localTransform();
    expected_matrix = Eigen::Matrix4d::Identity();
    expected_matrix(0, 3) = 1.0;
    EXPECT_TRUE(local_transform.isApprox(expected_matrix));
    expected_matrix(2, 3) = 1.0;
    EXPECT_TRUE(world_transform.isApprox(expected_matrix));

    // Check end effector node local and world transforms
    world_transform = end_effector->worldTransform();
    local_transform = end_effector->localTransform();
    expected_matrix = Eigen::Matrix4d::Identity();
    expected_matrix(0, 3) = 1.0;
    EXPECT_TRUE(local_transform.isApprox(expected_matrix));
    expected_matrix(0, 3) = 2.0;
    expected_matrix(2, 3) = 1.0;
    EXPECT_TRUE(world_transform.isApprox(expected_matrix));

    // Check if the end effector is at the correct pose
    Transform fk_result = end_effector->worldTransform();
    Eigen::Vector3d actual_position = fk_result.block<3, 1>(0, 3);
    Eigen::Matrix3d actual_rotation = fk_result.block<3, 3>(0, 0);
    EXPECT_TRUE(actual_position.isApprox(Eigen::Vector3d(2.0, 0.0, 1.0)));
    EXPECT_TRUE(actual_rotation.isApprox(Eigen::Matrix3d::Identity()));
}

// *********************************************************************************
//! \brief Test forward kinematics.
// *********************************************************************************
TEST_F(RobotTest, ForwardKinematics)
{
    // Test with zero joint values
    std::vector<double> zero_values = { 0.0, 0.0 };
    robot_arm->setJointValues(zero_values);

    Transform fk_result = end_effector->worldTransform();

    // With zero joint values, the end effector should be at specific position
    Eigen::Vector3d actual_position = fk_result.block<3, 1>(0, 3);

    // Position should be reasonable (not at origin due to arm length)
    EXPECT_TRUE(actual_position.norm() > 0.0);

    // Test with different joint values
    std::vector<double> test_values = { M_PI / 2, M_PI / 4 };
    robot_arm->setJointValues(test_values);

    Transform fk_result2 = end_effector->worldTransform();

    // Forward kinematics should provide a valid homogeneous transformation
    EXPECT_DOUBLE_EQ(fk_result2(3, 3), 1.0);
    EXPECT_DOUBLE_EQ(fk_result2(3, 0), 0.0);
    EXPECT_DOUBLE_EQ(fk_result2(3, 1), 0.0);
    EXPECT_DOUBLE_EQ(fk_result2(3, 2), 0.0);

    // Verify that the result is a valid position (not NaN or infinite)
    Eigen::Vector3d actual_position2 = fk_result2.block<3, 1>(0, 3);
    EXPECT_TRUE(actual_position2.array().isFinite().all());
    EXPECT_FALSE(actual_position2.hasNaN());

    // The transform should be valid
    EXPECT_TRUE((fk_result2.block<3, 3>(0, 0).determinant() >
                 0.0)); // Valid rotation matrix

    // Test with extreme joint values
    std::vector<double> extreme_values = { M_PI, -M_PI };
    robot_arm->setJointValues(extreme_values);

    Transform fk_result3 = end_effector->worldTransform();
    EXPECT_NO_THROW(end_effector->worldTransform());

    // Result should still be a valid transformation matrix
    EXPECT_DOUBLE_EQ(fk_result3(3, 3), 1.0);
}

// *********************************************************************************
//! \brief Test end effector pose.
// *********************************************************************************
TEST_F(RobotTest, EndEffectorPose)
{
    std::vector<double> values = { 0.0, 0.0 };
    robot_arm->setJointValues(values);

    Pose pose = utils::transformToPose(end_effector->worldTransform());

    // Check that pose has reasonable values
    EXPECT_EQ(pose.size(), 6); // Should be 6D pose

    // Position should not be at origin (due to arm length)
    Eigen::Vector3d position = pose.head<3>();
    EXPECT_GT(position.norm(), 0.5);
}

// *********************************************************************************
//! \brief Test Jacobian calculation.
// *********************************************************************************
TEST_F(RobotTest, JacobianCalculation)
{
    std::vector<double> values = { 0.0, 0.0 };
    robot_arm->setJointValues(values);

    Jacobian jacobian = robot_arm->jacobian(*end_effector);

    // For a 2-DOF arm, Jacobian should be 6x2
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), 2);

    // Check that Jacobian is not zero (has some values)
    EXPECT_GT(jacobian.norm(), 0.0);
}

#if 0 // FIXME A deplacer
// *********************************************************************************
//! \brief Test inverse kinematics with initial pose.
// *********************************************************************************
TEST_F(RobotTest, InverseKinematicsInitialPose)
{
    // Test if the actual pose is reachable by the target.
    // Current joint positions = [0.0, 0.0]
    Transform target_transform = Eigen::Matrix4d::Identity();
    target_transform(0, 3) = 2.0;
    target_transform(2, 3) = 1.0;
    Pose target_pose = utils::transformToPose(target_transform);

    std::vector<double> solution =
        robot_arm->inverseKinematics(target_pose, *end_effector);

    // Verify the solution by forward kinematics.
    EXPECT_EQ(solution.size(), 2);
    EXPECT_EQ(solution[0], 0.0);
    EXPECT_EQ(solution[1], 0.0);
    robot_arm->setJointValues(solution);

    Transform fk_result = end_effector->worldTransform();
    Eigen::Vector3d actual_position = fk_result.block<3, 1>(0, 3);
    Eigen::Matrix3d actual_rotation = fk_result.block<3, 3>(0, 0);

    EXPECT_TRUE(actual_position.isApprox(Eigen::Vector3d(2.0, 0.0, 1.0)));
    EXPECT_TRUE(actual_rotation.isApprox(Eigen::Matrix3d::Identity()));
}
#endif

#if 0
// *********************************************************************************
//! \brief Test edge cases.
// *********************************************************************************
TEST_F(RobotTest, EdgeCases)
{
    // Test with mismatched joint values size
    std::vector<double> wrong_size_values = { 1.0, 2.0, 3.0 };
    EXPECT_FALSE(robot_arm->setJointValues(wrong_size_values));

    // Test unreachable target for inverse kinematics
    Pose unreachable_pose;
    unreachable_pose << 100.0, 100.0, 100.0, 0.0, 0.0, 0.0;

    std::vector<double> solution =
        robot_arm->inverseKinematics(unreachable_pose, *end_effector);
    EXPECT_EQ(solution.size(), 0);
}
#endif

// *********************************************************************************
//! \brief Test complex kinematic chain.
// *********************************************************************************
TEST_F(RobotTest, ComplexKinematics)
{
    // Test with multiple joint configurations
    std::vector<std::vector<double>> test_configurations = {
        { 0.0, 0.0 },
        { M_PI / 4.0, M_PI / 4.0 },
        { M_PI / 2.0, -M_PI / 2.0 },
        { -M_PI / 3.0, M_PI / 3.0 }
    };

    for (const auto& config : test_configurations)
    {
        robot_arm->setJointValues(config);

        // Jacobian calculation should not crash
        EXPECT_NO_THROW(robot_arm->jacobian(*end_effector));

        // Verify that results are reasonable
        Transform fk_result = end_effector->worldTransform();
        Pose pose = utils::transformToPose(fk_result);
        Jacobian jacobian = robot_arm->jacobian(*end_effector);

        // Transform should be valid homogeneous matrix
        EXPECT_DOUBLE_EQ(fk_result(3, 3), 1.0);
        EXPECT_DOUBLE_EQ(fk_result(3, 0), 0.0);
        EXPECT_DOUBLE_EQ(fk_result(3, 1), 0.0);
        EXPECT_DOUBLE_EQ(fk_result(3, 2), 0.0);

        // Pose should be 6D
        EXPECT_EQ(pose.size(), 6);

        // Jacobian should have correct dimensions
        EXPECT_EQ(jacobian.rows(), 6);
        EXPECT_EQ(jacobian.cols(), 2);
    }
}

// *********************************************************************************
//! \brief Test joint limits interaction.
// *********************************************************************************
TEST_F(RobotTest, JointLimits)
{
    // Set joint limits using direct pointers
    EXPECT_TRUE(joint1 && joint2);
    joint1->limits(-M_PI / 2.0, M_PI / 2.0);
    joint2->limits(-M_PI / 4.0, M_PI / 4.0);

    // Test that values are clamped to limits
    std::vector<double> excessive_values = { M_PI, M_PI };
    robot_arm->setJointValues(excessive_values);

    std::vector<double> actual_values = robot_arm->jointPositions();
    EXPECT_LE(actual_values[0], M_PI / 2.0);
    EXPECT_LE(actual_values[1], M_PI / 4.0);
}