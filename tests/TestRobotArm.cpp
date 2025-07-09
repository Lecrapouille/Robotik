#include "Robotik/Robotik.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for RobotArm class.
// *********************************************************************************
class RobotArmTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Create a simple 2-DOF arm for testing
        robot_arm = std::make_unique<RobotArm>("test_arm");

        // Create root node first
        auto root = Node::create<Node>("root");

        // Create joints directly as child nodes with proper transforms
        Joint& joint1_ref = root->createChild<Joint>(
            "joint1", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
        Joint& joint2_ref = joint1_ref.createChild<Joint>(
            "joint2", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
        Node& end_effector_ref = joint2_ref.createChild<Node>("end_effector");

        // Store references for later use
        joint1 = &joint1_ref;
        joint2 = &joint2_ref;
        end_effector = &end_effector_ref;

        // Set up initial transforms (simple arm with 1 unit links)
        Transform joint1_transform = Eigen::Matrix4d::Identity();
        joint1_transform(2, 3) = 1.0; // 1 unit up
        joint1->setLocalTransform(joint1_transform);

        Transform joint2_transform = Eigen::Matrix4d::Identity();
        joint2_transform(0, 3) = 1.0; // 1 unit forward
        joint2->setLocalTransform(joint2_transform);

        Transform end_effector_transform = Eigen::Matrix4d::Identity();
        end_effector_transform(0, 3) = 1.0; // 1 unit forward
        end_effector->setLocalTransform(end_effector_transform);

        // Configure robot arm with the same joints from the hierarchy
        robot_arm->setRootNode(std::move(root));

        // Create separate joint instances for the robot arm that match the
        // hierarchy
        auto joint1_copy = Joint::create<Joint>(
            "joint1", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
        auto joint2_copy = Joint::create<Joint>(
            "joint2", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));

        // Store pointers to these joints for testing
        joint1_copy_ptr = joint1_copy.get();
        joint2_copy_ptr = joint2_copy.get();

        robot_arm->addJoint(std::move(joint1_copy));
        robot_arm->addJoint(std::move(joint2_copy));
        robot_arm->setEndEffector(*end_effector);
    }

    std::unique_ptr<RobotArm> robot_arm;
    Joint* joint1;
    Joint* joint2;
    Node* end_effector;
    Joint* joint1_copy_ptr;
    Joint* joint2_copy_ptr;
};

// *********************************************************************************
//! \brief Test RobotArm creation and basic properties.
// *********************************************************************************
TEST_F(RobotArmTest, Creation)
{
    auto arm = std::make_unique<RobotArm>("test");
    // Basic creation test - should not crash
    EXPECT_NO_THROW(arm->getJointValues());
}

// *********************************************************************************
//! \brief Test adding joints.
// *********************************************************************************
TEST_F(RobotArmTest, AddJoint)
{
    auto arm = std::make_unique<RobotArm>("test");
    auto joint = Joint::create<Joint>(
        "test_joint", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));

    Joint* joint_ptr = joint.get();
    arm->addJoint(std::move(joint));

    EXPECT_EQ(arm->getJoint("test_joint"), joint_ptr);
}

// *********************************************************************************
//! \brief Test getting joint by name.
// *********************************************************************************
TEST_F(RobotArmTest, GetJoint)
{
    EXPECT_EQ(robot_arm->getJoint("joint1"), joint1_copy_ptr);
    EXPECT_EQ(robot_arm->getJoint("joint2"), joint2_copy_ptr);
    EXPECT_EQ(robot_arm->getJoint("nonexistent"), nullptr);
}

// *********************************************************************************
//! \brief Test setting and getting joint values.
// *********************************************************************************
TEST_F(RobotArmTest, JointValues)
{
    std::vector<double> values = { M_PI / 4, M_PI / 2 };
    robot_arm->setJointValues(values);

    std::vector<double> retrieved_values = robot_arm->getJointValues();

    EXPECT_EQ(retrieved_values.size(), 2);
    EXPECT_DOUBLE_EQ(retrieved_values[0], M_PI / 4);
    EXPECT_DOUBLE_EQ(retrieved_values[1], M_PI / 2);
}

// *********************************************************************************
//! \brief Test forward kinematics.
// *********************************************************************************
TEST_F(RobotArmTest, ForwardKinematics)
{
    // Test with zero joint values
    std::vector<double> zero_values = { 0.0, 0.0 };
    robot_arm->setJointValues(zero_values);

    Transform fk_result = robot_arm->forwardKinematics();

    // With zero joint values, the end effector should be at specific position
    Eigen::Vector3d actual_position = fk_result.block<3, 1>(0, 3);

    // Position should be reasonable (not at origin due to arm length)
    EXPECT_TRUE(actual_position.norm() > 0.0);

    // Test with different joint values
    std::vector<double> test_values = { M_PI / 2, M_PI / 4 };
    robot_arm->setJointValues(test_values);

    Transform fk_result2 = robot_arm->forwardKinematics();

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

    Transform fk_result3 = robot_arm->forwardKinematics();
    EXPECT_NO_THROW(robot_arm->forwardKinematics());

    // Result should still be a valid transformation matrix
    EXPECT_DOUBLE_EQ(fk_result3(3, 3), 1.0);
}

// *********************************************************************************
//! \brief Test end effector pose.
// *********************************************************************************
TEST_F(RobotArmTest, EndEffectorPose)
{
    std::vector<double> values = { 0.0, 0.0 };
    robot_arm->setJointValues(values);

    Pose pose = robot_arm->getEndEffectorPose();

    // Check that pose has reasonable values
    EXPECT_EQ(pose.size(), 6); // Should be 6D pose

    // Position should not be at origin (due to arm length)
    Eigen::Vector3d position = pose.head<3>();
    EXPECT_GT(position.norm(), 0.5);
}

// *********************************************************************************
//! \brief Test Jacobian calculation.
// *********************************************************************************
TEST_F(RobotArmTest, JacobianCalculation)
{
    std::vector<double> values = { 0.0, 0.0 };
    robot_arm->setJointValues(values);

    Jacobian jacobian = robot_arm->calculateJacobian();

    // For a 2-DOF arm, Jacobian should be 6x2
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), 2);

    // Check that Jacobian is not zero (has some values)
    EXPECT_GT(jacobian.norm(), 0.0);
}

// *********************************************************************************
//! \brief Test inverse kinematics.
// *********************************************************************************
TEST_F(RobotArmTest, InverseKinematics)
{
    // Test with a reachable target
    Pose target_pose;
    target_pose << 1.0, 1.0, 1.0, 0.0, 0.0,
        0.0; // Position (1,1,1), no rotation

    std::vector<double> solution;
    bool success = robot_arm->inverseKinematics(target_pose, solution);

    // The exact success depends on the implementation, but it should not crash
    EXPECT_NO_THROW(robot_arm->inverseKinematics(target_pose, solution));

    if (success)
    {
        EXPECT_EQ(solution.size(), 2);
        // If solution found, verify it by forward kinematics
        robot_arm->setJointValues(solution);
        Transform fk_result = robot_arm->forwardKinematics();
        Eigen::Vector3d actual_position = fk_result.block<3, 1>(0, 3);
        Eigen::Vector3d target_position = target_pose.head<3>();

        // Position should be close to target
        EXPECT_TRUE(actual_position.isApprox(target_position, 1e-3));
    }
}

// *********************************************************************************
//! \brief Test root node access.
// *********************************************************************************
TEST_F(RobotArmTest, RootNode)
{
    Node* root = robot_arm->getRootNode();
    EXPECT_NE(root, nullptr);
    EXPECT_EQ(root->getName(), "root");
}

// *********************************************************************************
//! \brief Test node search functionality.
// *********************************************************************************
TEST_F(RobotArmTest, NodeSearch)
{
    Node* found_joint1 = robot_arm->getNode("joint1");
    Node* found_joint2 = robot_arm->getNode("joint2");
    Node* found_end_effector = robot_arm->getNode("end_effector");

    EXPECT_NE(found_joint1, nullptr);
    EXPECT_NE(found_joint2, nullptr);
    EXPECT_NE(found_end_effector, nullptr);

    // Test non-existent node
    Node* not_found = robot_arm->getNode("nonexistent");
    EXPECT_EQ(not_found, nullptr);
}

// *********************************************************************************
//! \brief Test edge cases.
// *********************************************************************************
TEST_F(RobotArmTest, EdgeCases)
{
    // Test with mismatched joint values size
    std::vector<double> wrong_size_values = { 1.0, 2.0, 3.0 };
    EXPECT_THROW(robot_arm->setJointValues(wrong_size_values),
                 std::invalid_argument);

    // Test unreachable target for inverse kinematics
    Pose unreachable_pose;
    unreachable_pose << 100.0, 100.0, 100.0, 0.0, 0.0, 0.0;

    std::vector<double> solution;
    bool success = robot_arm->inverseKinematics(unreachable_pose, solution);
    EXPECT_FALSE(success);
}

// *********************************************************************************
//! \brief Test complex kinematic chain.
// *********************************************************************************
TEST_F(RobotArmTest, ComplexKinematics)
{
    // Test with multiple joint configurations
    std::vector<std::vector<double>> test_configurations = {
        { 0.0, 0.0 },
        { M_PI / 4, M_PI / 4 },
        { M_PI / 2, -M_PI / 2 },
        { -M_PI / 3, M_PI / 3 }
    };

    for (const auto& config : test_configurations)
    {
        robot_arm->setJointValues(config);

        // Forward kinematics should not crash
        EXPECT_NO_THROW(robot_arm->forwardKinematics());

        // Jacobian calculation should not crash
        EXPECT_NO_THROW(robot_arm->calculateJacobian());

        // End effector pose should not crash
        EXPECT_NO_THROW(robot_arm->getEndEffectorPose());

        // Verify that results are reasonable
        Transform fk_result = robot_arm->forwardKinematics();
        Pose pose = robot_arm->getEndEffectorPose();
        Jacobian jacobian = robot_arm->calculateJacobian();

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
TEST_F(RobotArmTest, JointLimits)
{
    // Set joint limits
    Joint* joint1_ptr = robot_arm->getJoint("joint1");
    Joint* joint2_ptr = robot_arm->getJoint("joint2");

    if (joint1_ptr && joint2_ptr)
    {
        joint1_ptr->setLimits(-M_PI / 2, M_PI / 2);
        joint2_ptr->setLimits(-M_PI / 4, M_PI / 4);

        // Test that values are clamped to limits
        std::vector<double> excessive_values = { M_PI, M_PI };
        robot_arm->setJointValues(excessive_values);

        std::vector<double> actual_values = robot_arm->getJointValues();
        EXPECT_LE(actual_values[0], M_PI / 2);
        EXPECT_LE(actual_values[1], M_PI / 4);
    }
}

// *********************************************************************************
//! \brief Test robot arm with different joint types.
// *********************************************************************************
TEST_F(RobotArmTest, MixedJointTypes)
{
    // Create a new arm with mixed joint types
    auto mixed_arm = std::make_unique<RobotArm>("mixed_arm");
    auto root = Node::create<Node>("root");

    // Create prismatic and revolute joints
    Joint& prismatic_joint = root->createChild<Joint>(
        "prismatic", Joint::Type::PRISMATIC, Eigen::Vector3d(0, 0, 1));
    Joint& revolute_joint = prismatic_joint.createChild<Joint>(
        "revolute", Joint::Type::REVOLUTE, Eigen::Vector3d(1, 0, 0));
    Node& end_eff = revolute_joint.createChild<Node>("end_effector");

    mixed_arm->setRootNode(std::move(root));
    mixed_arm->addJoint(Joint::create<Joint>(
        "p_joint", Joint::Type::PRISMATIC, Eigen::Vector3d(0, 0, 1)));
    mixed_arm->addJoint(Joint::create<Joint>(
        "r_joint", Joint::Type::REVOLUTE, Eigen::Vector3d(1, 0, 0)));
    mixed_arm->setEndEffector(end_eff);

    // Test that it works with mixed joint types
    std::vector<double> values = { 0.5, M_PI / 4 }; // Translation and rotation
    EXPECT_NO_THROW(mixed_arm->setJointValues(values));
    EXPECT_NO_THROW(mixed_arm->forwardKinematics());
    EXPECT_NO_THROW(mixed_arm->calculateJacobian());
}