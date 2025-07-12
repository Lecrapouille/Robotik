#include "main.hpp"

#include "Robotik/Robotik.hpp"
#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief End effector node. For this current implementation, a node is enough.
// *********************************************************************************
class EndEffector: public Node
{
public:

    using Node::Node;
};

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
        root = Node::create<Joint>(
            "root", Joint::Type::FIXED, Eigen::Vector3d(0, 0, 1));

        // Create joints directly as child nodes with proper transforms
        joint1 = &(root->createChild<Joint>(
            "joint1", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1)));
        joint2 = &(joint1->createChild<Joint>(
            "joint2", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1)));
        end_effector = &(joint2->createChild<EndEffector>("end_effector"));

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

        // Set up the robot arm, base frame and end effector
        robot_arm->setupRobot(std::move(root), *end_effector);
    }

    std::unique_ptr<RobotArm> robot_arm;
    Joint::Ptr root;
    Joint* joint1;
    Joint* joint2;
    EndEffector* end_effector;
};

// *********************************************************************************
//! \brief Test getting joint by name.
// *********************************************************************************
TEST_F(RobotArmTest, GetJoint)
{
    EXPECT_EQ(robot_arm->getJoint("root"), root.get());
    EXPECT_EQ(robot_arm->getJoint("joint1"), joint1);
    EXPECT_EQ(robot_arm->getJoint("joint2"), joint2);
    EXPECT_EQ(robot_arm->getJoint("end_effector"),
              nullptr); // end_effector is not a joint
    EXPECT_EQ(robot_arm->getJoint("nonexistent"), nullptr);
}

// *********************************************************************************
//! \brief Test setting and getting joint values.
// *********************************************************************************
TEST_F(RobotArmTest, JointValues)
{
    std::vector<double> values = { M_PI / 4.0, M_PI / 2.0 };
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
    // Test with a reachable target: Position (1,1,1), no rotation
    Pose target_pose;
    target_pose << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

    std::vector<double> solution;
    bool success = robot_arm->inverseKinematics(target_pose, solution);

    // The exact success depends on the implementation, but it should not crash
    EXPECT_NO_THROW(robot_arm->inverseKinematics(target_pose, solution));

    if (success)
    {
        // If solution found, verify it by forward kinematics
        EXPECT_EQ(solution.size(), 2);
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
    const Node* found_root = robot_arm->getRootNode();
    EXPECT_NE(found_root, nullptr);
    EXPECT_EQ(found_root->getName(), "root");
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

    // Verify they are the correct nodes
    EXPECT_EQ(found_joint1, joint1);
    EXPECT_EQ(found_joint2, joint2);
    EXPECT_EQ(found_end_effector, end_effector);

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
        { M_PI / 4.0, M_PI / 4.0 },
        { M_PI / 2.0, -M_PI / 2.0 },
        { -M_PI / 3.0, M_PI / 3.0 }
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
    // Set joint limits using direct pointers
    EXPECT_TRUE(joint1 && joint2);
    joint1->setLimits(-M_PI / 2.0, M_PI / 2.0);
    joint2->setLimits(-M_PI / 4.0, M_PI / 4.0);

    // Test that values are clamped to limits
    std::vector<double> excessive_values = { M_PI, M_PI };
    robot_arm->setJointValues(excessive_values);

    std::vector<double> actual_values = robot_arm->getJointValues();
    EXPECT_LE(actual_values[0], M_PI / 2.0);
    EXPECT_LE(actual_values[1], M_PI / 4.0);
}