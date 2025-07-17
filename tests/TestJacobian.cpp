#include "main.hpp"

#include "Robotik/Parser.hpp"
#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Helper function to compare matrices with tolerance
// *********************************************************************************
static auto compareMatrices = [](const robotik::Transform& actual,
                                 const robotik::Transform& expected,
                                 double tolerance = 1e-10)
{
    ASSERT_EQ(actual.rows(), expected.rows());
    ASSERT_EQ(actual.cols(), expected.cols());
    for (int i = 0; i < actual.rows(); ++i)
    {
        for (int j = 0; j < actual.cols(); ++j)
        {
            ASSERT_NEAR(actual(i, j), expected(i, j), tolerance)
                << "Matrix difference at (" << i << "," << j << ")";
        }
    }
};

// *********************************************************************************
//! \brief Helper function to create transformation matrix
// *********************************************************************************
static auto createTransform =
    [](double x, double y, double z) -> robotik::Transform
{
    robotik::Transform T = robotik::Transform::Identity();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
};

// *********************************************************************************
//! \brief Test to check all states of the simple robot with prismatic joint
// *********************************************************************************
TEST(TestJacobian, SimplePrismaticRobotTest)
{
    URDFParser parser;
    auto robot = parser.load(
        "/home/qq/MyGitHub/Robotik/data/simple_prismatic_robot.urdf");
    ASSERT_NE(robot, nullptr);
    ASSERT_EQ(robot->getName(), "simple_prismatic_robot");

    // ==================== CHECK JOINTS ====================

    // Check joint names
    auto const joint_names = robot->getJointNames();
    ASSERT_EQ(joint_names.size(), 1);
    ASSERT_EQ(joint_names[0], "prismatic_joint");

    // Check prismatic joint
    auto prismatic_joint = robot->getJoint("prismatic_joint");
    ASSERT_NE(prismatic_joint, nullptr);
    ASSERT_EQ(prismatic_joint->getName(), "prismatic_joint");
    ASSERT_EQ(prismatic_joint->getType(), Joint::Type::PRISMATIC);
    ASSERT_DOUBLE_EQ(prismatic_joint->getValue(), 0.0);

    // Check translation axis (z-axis)
    auto joint_axis = prismatic_joint->getAxis();
    ASSERT_DOUBLE_EQ(joint_axis.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_axis.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_axis.z(), 1.0);

    // Check origin (0 0 0.05)
    auto joint_origin_xyz = prismatic_joint->getOriginXYZ();
    ASSERT_DOUBLE_EQ(joint_origin_xyz.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_xyz.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_xyz.z(), 0.05);

    // Check origin RPY (0 0 0)
    auto joint_origin_rpy = prismatic_joint->getOriginRPY();
    ASSERT_DOUBLE_EQ(joint_origin_rpy.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_rpy.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_rpy.z(), 0.0);

    // ==================== CHECK LINKS ====================

    // Check base_link
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    ASSERT_EQ(base_link->name, "base_link");
    ASSERT_EQ(base_link->geometry.type, Geometry::Type::BOX);
    ASSERT_EQ(base_link->parent_joint, nullptr);
    ASSERT_NE(base_link->child_joint, nullptr);
    ASSERT_EQ(base_link->child_joint->getName(), "prismatic_joint");

    // Check geometry parameters base_link (0.1 0.1 0.1)
    ASSERT_EQ(base_link->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[0], 0.1);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[1], 0.1);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[2], 0.1);

    // Check color base_link (0.5 0.5 0.5 1)
    ASSERT_DOUBLE_EQ(base_link->geometry.color.x(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.y(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.z(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.w(), 1.0);

    // Check end_effector_link
    auto end_effector_link = robot->getLink("end_effector_link");
    ASSERT_NE(end_effector_link, nullptr);
    ASSERT_EQ(end_effector_link->name, "end_effector_link");
    ASSERT_EQ(end_effector_link->geometry.type, Geometry::Type::CYLINDER);
    ASSERT_NE(end_effector_link->parent_joint, nullptr);
    ASSERT_EQ(end_effector_link->parent_joint->getName(), "prismatic_joint");
    ASSERT_EQ(end_effector_link->child_joint, nullptr);

    // Check geometry parameters end_effector_link (radius=0.02, length=0.1)
    ASSERT_EQ(end_effector_link->geometry.parameters.size(), 2);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.parameters[0], 0.02); // radius
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.parameters[1], 0.1);  // length

    // Check color end_effector_link (1 0 0 1 - red)
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.x(), 1.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.y(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.z(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.w(), 1.0);

    // ==================== CHECK SCENE GRAPH ====================

    // Check scene graph
    auto root_node = robot->getRootNode();
    ASSERT_NE(root_node, nullptr);
    ASSERT_EQ(root_node->getName(), "prismatic_joint");
    ASSERT_EQ(root_node->getChildren().size(), 0);

    // ==================== CHECK TRANSFORMATIONS ====================

    // Check local transformation
    auto const& joint_local_transform = prismatic_joint->getLocalTransform();
    ASSERT_EQ(joint_local_transform.rows(), 4);
    ASSERT_EQ(joint_local_transform.cols(), 4);
    compareMatrices(joint_local_transform, createTransform(0.0, 0.0, 0.05));

    // Check world transformation
    auto const& joint_world_transform = prismatic_joint->getWorldTransform();
    ASSERT_EQ(joint_world_transform.rows(), 4);
    ASSERT_EQ(joint_world_transform.cols(), 4);
    compareMatrices(joint_world_transform, createTransform(0.0, 0.0, 0.05));

#if 0
    // Check end effector pose
    Eigen::Matrix<double, 6, 1> expected_pose;
    expected_pose << 0.0, 0.0, 0.05, 0.0, 0.0, 0.0;

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_DOUBLE_EQ(end_effector_pose(i, 0), expected_pose(i, 0))
            << "End effector pose difference at index " << i;
    }

    // Check Jacobian numerical values for prismatic joint
    robotik::Jacobian expected_jacobian(6, 1);
    expected_jacobian << 0.0, // Linear velocity X
        0.0,                  // Linear velocity Y
        1.0,                  // Linear velocity Z (axis direction)
        0.0,                  // Angular velocity X
        0.0,                  // Angular velocity Y
        0.0;                  // Angular velocity Z

    ASSERT_EQ(jacobian.rows(), expected_jacobian.rows());
    ASSERT_EQ(jacobian.cols(), expected_jacobian.cols());

    for (int i = 0; i < jacobian.rows(); ++i)
    {
        for (int j = 0; j < jacobian.cols(); ++j)
        {
            ASSERT_DOUBLE_EQ(jacobian(i, j), expected_jacobian(i, j))
                << "Jacobian difference at (" << i << "," << j << ")";
        }
    }

    // ==================== CHECK KINEMATICS ====================

    // Check Jacobian (should be 6x1 for 1 joint)
    auto jacobian = robot->calculateJacobian();
    std::cout << "Prismatic Robot Jacobian:" << std::endl;
    std::cout << jacobian << std::endl;

    ASSERT_EQ(jacobian.rows(), 6);
    ASSERT_EQ(jacobian.cols(), 1);

    // Check end effector pose (should be a 6x1 vector)
    auto end_effector_pose = robot->getEndEffectorPose();
    ASSERT_EQ(end_effector_pose.rows(), 6);
    ASSERT_EQ(end_effector_pose.cols(), 1);

    // Check joint values (should be 0 by default)
    auto joint_values = robot->getJointValues();
    ASSERT_EQ(joint_values.size(), 1);
    ASSERT_DOUBLE_EQ(joint_values[0], 0.0);

    // ==================== TEST MOVEMENT ====================

    // Test movement: set joint value to 0.05 and check position
    std::vector<double> new_values = { 0.05 };
    robot->setJointValues(new_values);

    auto new_pose = robot->getEndEffectorPose();
    Eigen::Matrix<double, 6, 1> expected_moved_pose;
    expected_moved_pose << 0.0, 0.0, 0.10, 0.0, 0.0,
        0.0; // 0.05 base + 0.05 movement

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_DOUBLE_EQ(new_pose(i, 0), expected_moved_pose(i, 0))
            << "End effector pose after movement difference at index " << i;
    }
#endif
}

#if 0
// *********************************************************************************
//! \brief Test to check all states of the simple robot with revolute joint
// *********************************************************************************
TEST(TestJacobian, SimpleRevoluteRobotTest)
{
    URDFParser parser;
    auto robot = parser.load(
        "/home/qq/MyGitHub/Robotik/data/simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr);

    // ==================== CHECK JOINTS ====================

    // Check joint names (only actuated joints)
    auto const joint_names = robot->getJointNames();
    ASSERT_EQ(joint_names.size(), 1);
    ASSERT_EQ(joint_names[0], "revolute_joint");

    // Check revolute joint
    auto revolute_joint = robot->getJoint("revolute_joint");
    ASSERT_NE(revolute_joint, nullptr);
    ASSERT_EQ(revolute_joint->getName(), "revolute_joint");
    ASSERT_EQ(revolute_joint->getType(), Joint::Type::REVOLUTE);
    ASSERT_DOUBLE_EQ(revolute_joint->getValue(), 0.0);

    // Check rotation axis (z-axis)
    auto joint_axis = revolute_joint->getAxis();
    ASSERT_DOUBLE_EQ(joint_axis.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_axis.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_axis.z(), 1.0);

    // Check origin (0 0 0.05)
    auto joint_origin_xyz = revolute_joint->getOriginXYZ();
    ASSERT_DOUBLE_EQ(joint_origin_xyz.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_xyz.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_xyz.z(), 0.05);

    // Check origin RPY (0 0 0)
    auto joint_origin_rpy = revolute_joint->getOriginRPY();
    ASSERT_DOUBLE_EQ(joint_origin_rpy.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_rpy.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint_origin_rpy.z(), 0.0);

    // ==================== CHECK LINKS ====================

    // Check base_link
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    ASSERT_EQ(base_link->name, "base_link");
    ASSERT_EQ(base_link->geometry.type, Geometry::Type::BOX);
    ASSERT_EQ(base_link->parent_joint, nullptr);
    ASSERT_NE(base_link->child_joint, nullptr);
    ASSERT_EQ(base_link->child_joint->getName(), "revolute_joint");

    // Check geometry parameters base_link (0.1 0.1 0.1)
    ASSERT_EQ(base_link->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[0], 0.1);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[1], 0.1);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[2], 0.1);

    // Check color base_link (0.5 0.5 0.5 1)
    ASSERT_DOUBLE_EQ(base_link->geometry.color.x(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.y(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.z(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.w(), 1.0);

    // Check arm_link
    auto arm_link = robot->getLink("arm_link");
    ASSERT_NE(arm_link, nullptr);
    ASSERT_EQ(arm_link->name, "arm_link");
    ASSERT_EQ(arm_link->geometry.type, Geometry::Type::BOX);
    ASSERT_NE(arm_link->parent_joint, nullptr);
    ASSERT_EQ(arm_link->parent_joint->getName(), "revolute_joint");
    ASSERT_NE(arm_link->child_joint, nullptr);
    ASSERT_EQ(arm_link->child_joint->getName(), "end_effector_joint");

    // Check geometry parameters arm_link (0.2 0.05 0.05)
    ASSERT_EQ(arm_link->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(arm_link->geometry.parameters[0], 0.2);
    ASSERT_DOUBLE_EQ(arm_link->geometry.parameters[1], 0.05);
    ASSERT_DOUBLE_EQ(arm_link->geometry.parameters[2], 0.05);

    // Check color arm_link (0 0 1 1 - blue)
    ASSERT_DOUBLE_EQ(arm_link->geometry.color.x(), 0.0);
    ASSERT_DOUBLE_EQ(arm_link->geometry.color.y(), 0.0);
    ASSERT_DOUBLE_EQ(arm_link->geometry.color.z(), 1.0);
    ASSERT_DOUBLE_EQ(arm_link->geometry.color.w(), 1.0);

    // Check end_effector_link
    auto end_effector_link = robot->getLink("end_effector_link");
    ASSERT_NE(end_effector_link, nullptr);
    ASSERT_EQ(end_effector_link->name, "end_effector_link");
    ASSERT_EQ(end_effector_link->geometry.type, Geometry::Type::CYLINDER);
    ASSERT_NE(end_effector_link->parent_joint, nullptr);
    ASSERT_EQ(end_effector_link->parent_joint->getName(), "end_effector_joint");
    ASSERT_EQ(end_effector_link->child_joint, nullptr);

    // Check geometry parameters end_effector_link (radius=0.02, length=0.1)
    ASSERT_EQ(end_effector_link->geometry.parameters.size(), 2);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.parameters[0], 0.02); // radius
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.parameters[1], 0.1);  // length

    // Check color end_effector_link (1 0 0 1 - red)
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.x(), 1.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.y(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.z(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_link->geometry.color.w(), 1.0);

    // ==================== CHECK KINEMATICS ====================

    // Check Jacobian (should be 6x1 for 1 actuated joint)
    auto jacobian = robot->calculateJacobian();
    std::cout << "Revolute Robot Jacobian:" << std::endl;
    std::cout << jacobian << std::endl;

    ASSERT_EQ(jacobian.rows(), 6);
    ASSERT_EQ(jacobian.cols(), 1);

    // Check end effector pose (should be a 6x1 vector)
    auto end_effector_pose = robot->getEndEffectorPose();
    ASSERT_EQ(end_effector_pose.rows(), 6);
    ASSERT_EQ(end_effector_pose.cols(), 1);

    // Check joint values (should be 0 by default)
    auto joint_values = robot->getJointValues();
    ASSERT_EQ(joint_values.size(), 1);
    ASSERT_DOUBLE_EQ(joint_values[0], 0.0);

    // ==================== CHECK TRANSFORMATIONS ====================

    // Check local transformation
    auto const& joint_local_transform = revolute_joint->getLocalTransform();
    ASSERT_EQ(joint_local_transform.rows(), 4);
    ASSERT_EQ(joint_local_transform.cols(), 4);

    // Check world transformation
    auto const& joint_world_transform = revolute_joint->getWorldTransform();
    ASSERT_EQ(joint_world_transform.rows(), 4);
    ASSERT_EQ(joint_world_transform.cols(), 4);

    // Check numerical values
    compareMatrices(joint_local_transform, createTransform(0.0, 0.0, 0.05));
    compareMatrices(joint_world_transform, createTransform(0.0, 0.0, 0.05));

    // Check end effector pose (arm extends 0.2 units in x-direction)
    Eigen::Matrix<double, 6, 1> expected_pose;
    expected_pose << 0.2, 0.0, 0.05, 0.0, 0.0, 0.0;

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_DOUBLE_EQ(end_effector_pose(i, 0), expected_pose(i, 0))
            << "End effector pose difference at index " << i;
    }

    // Check Jacobian numerical values for revolute joint
    robotik::Jacobian expected_jacobian(6, 1);
    expected_jacobian
        << 0.0, // Linear velocity X (cross product: axis x (end - joint))
        0.2,    // Linear velocity Y
        0.0,    // Linear velocity Z
        0.0,    // Angular velocity X
        0.0,    // Angular velocity Y
        1.0;    // Angular velocity Z (axis direction)

    ASSERT_EQ(jacobian.rows(), expected_jacobian.rows());
    ASSERT_EQ(jacobian.cols(), expected_jacobian.cols());

    for (int i = 0; i < jacobian.rows(); ++i)
    {
        for (int j = 0; j < jacobian.cols(); ++j)
        {
            ASSERT_DOUBLE_EQ(jacobian(i, j), expected_jacobian(i, j))
                << "Jacobian difference at (" << i << "," << j << ")";
        }
    }

    // ==================== TEST MOVEMENT ====================

    // Test movement: set joint value to π/2 and check position
    std::vector<double> new_values = { M_PI / 2.0 };
    robot->setJointValues(new_values);

    auto new_pose = robot->getEndEffectorPose();
    Eigen::Matrix<double, 6, 1> expected_moved_pose;
    expected_moved_pose << 0.0, 0.2, 0.05, 0.0, 0.0,
        M_PI / 2.0; // Rotated 90 degrees

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_NEAR(new_pose(i, 0), expected_moved_pose(i, 0), 1e-10)
            << "End effector pose after movement difference at index " << i;
    }
}

#    if 0
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
#    endif

// *********************************************************************************
//! \brief Test to check all states of the links and joints of the SCARA robot.
// *********************************************************************************
TEST(TestJacobian, JacobianTestScara)
{
    URDFParser parser;
    auto robot = parser.load("/home/qq/MyGitHub/Robotik/data/scara_robot.urdf");
    ASSERT_NE(robot, nullptr);

    // ==================== CHECK JOINTS ====================

    // Check joint names
    auto const joint_names = robot->getJointNames();
    ASSERT_EQ(joint_names.size(), 3);
    ASSERT_EQ(joint_names[0], "joint1");
    ASSERT_EQ(joint_names[1], "joint2");
    ASSERT_EQ(joint_names[2], "end_effector");

    // Check joint1 (revolute, base_link -> link1)
    auto joint1 = robot->getJoint("joint1");
    ASSERT_NE(joint1, nullptr);
    ASSERT_EQ(joint1->getName(), "joint1");
    ASSERT_EQ(joint1->getType(), Joint::Type::REVOLUTE);
    ASSERT_DOUBLE_EQ(joint1->getValue(), 0.0);

    // Check rotation axis joint1 (z-axis)
    auto joint1_axis = joint1->getAxis();
    ASSERT_DOUBLE_EQ(joint1_axis.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_axis.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_axis.z(), 1.0);

    // Check origin joint1 (0 0 0.06)
    auto joint1_origin_xyz = joint1->getOriginXYZ();
    ASSERT_DOUBLE_EQ(joint1_origin_xyz.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_origin_xyz.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_origin_xyz.z(), 0.06);

    // Check origin RPY joint1 (0 0 0)
    auto joint1_origin_rpy = joint1->getOriginRPY();
    ASSERT_DOUBLE_EQ(joint1_origin_rpy.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_origin_rpy.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint1_origin_rpy.z(), 0.0);

    // Check joint2 (revolute, link1 -> link2)
    auto joint2 = robot->getJoint("joint2");
    ASSERT_NE(joint2, nullptr);
    ASSERT_EQ(joint2->getName(), "joint2");
    ASSERT_EQ(joint2->getType(), Joint::Type::REVOLUTE);
    ASSERT_DOUBLE_EQ(joint2->getValue(), 0.0);

    // Check rotation axis joint2 (z-axis)
    auto joint2_axis = joint2->getAxis();
    ASSERT_DOUBLE_EQ(joint2_axis.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint2_axis.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint2_axis.z(), 1.0);

    // Check origin joint2 (0.35 0 0)
    auto joint2_origin_xyz = joint2->getOriginXYZ();
    ASSERT_DOUBLE_EQ(joint2_origin_xyz.x(), 0.35);
    ASSERT_DOUBLE_EQ(joint2_origin_xyz.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint2_origin_xyz.z(), 0.0);

    // Check origin RPY joint2 (0 0 0)
    auto joint2_origin_rpy = joint2->getOriginRPY();
    ASSERT_DOUBLE_EQ(joint2_origin_rpy.x(), 0.0);
    ASSERT_DOUBLE_EQ(joint2_origin_rpy.y(), 0.0);
    ASSERT_DOUBLE_EQ(joint2_origin_rpy.z(), 0.0);

    // Check end_effector (prismatic, link2 -> link_end_effector)
    auto end_effector_joint = robot->getJoint("end_effector");
    ASSERT_NE(end_effector_joint, nullptr);
    ASSERT_EQ(end_effector_joint->getName(), "end_effector");
    ASSERT_EQ(end_effector_joint->getType(), Joint::Type::PRISMATIC);
    ASSERT_DOUBLE_EQ(end_effector_joint->getValue(), 0.0);

    // Check translation axis end_effector (-z-axis)
    auto end_effector_axis = end_effector_joint->getAxis();
    ASSERT_DOUBLE_EQ(end_effector_axis.x(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_axis.y(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_axis.z(), -1.0);

    // Check origin end_effector (0.45 0 0)
    auto end_effector_origin_xyz = end_effector_joint->getOriginXYZ();
    ASSERT_DOUBLE_EQ(end_effector_origin_xyz.x(), 0.45);
    ASSERT_DOUBLE_EQ(end_effector_origin_xyz.y(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_origin_xyz.z(), 0.0);

    // Check origin RPY end_effector (0 0 0)
    auto end_effector_origin_rpy = end_effector_joint->getOriginRPY();
    ASSERT_DOUBLE_EQ(end_effector_origin_rpy.x(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_origin_rpy.y(), 0.0);
    ASSERT_DOUBLE_EQ(end_effector_origin_rpy.z(), 0.0);

    // ==================== CHECK LINKS ====================

    // Check base_link
    auto base_link = robot->getLink("base_link");
    ASSERT_NE(base_link, nullptr);
    ASSERT_EQ(base_link->name, "base_link");
    ASSERT_EQ(base_link->geometry.type, Geometry::Type::BOX);
    ASSERT_EQ(base_link->parent_joint, nullptr);
    ASSERT_NE(base_link->child_joint, nullptr);
    ASSERT_EQ(base_link->child_joint->getName(), "joint1");

    // Check geometry parameters base_link (0.12 0.12 0.12)
    ASSERT_EQ(base_link->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[0], 0.12);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[1], 0.12);
    ASSERT_DOUBLE_EQ(base_link->geometry.parameters[2], 0.12);

    // Check color base_link (0.5 0.5 0.5 1)
    ASSERT_DOUBLE_EQ(base_link->geometry.color.x(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.y(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.z(), 0.5);
    ASSERT_DOUBLE_EQ(base_link->geometry.color.w(), 1.0);

    // Check link1
    auto link1 = robot->getLink("link1");
    ASSERT_NE(link1, nullptr);
    ASSERT_EQ(link1->name, "link1");
    ASSERT_EQ(link1->geometry.type, Geometry::Type::BOX);
    ASSERT_NE(link1->parent_joint, nullptr);
    ASSERT_EQ(link1->parent_joint->getName(), "joint1");
    ASSERT_NE(link1->child_joint, nullptr);
    ASSERT_EQ(link1->child_joint->getName(), "joint2");

    // Check geometry parameters link1 (0.350 0.09 0.025)
    ASSERT_EQ(link1->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(link1->geometry.parameters[0], 0.350);
    ASSERT_DOUBLE_EQ(link1->geometry.parameters[1], 0.09);
    ASSERT_DOUBLE_EQ(link1->geometry.parameters[2], 0.025);

    // Check color link1 (0 0 1 1 - blue)
    ASSERT_DOUBLE_EQ(link1->geometry.color.x(), 0.0);
    ASSERT_DOUBLE_EQ(link1->geometry.color.y(), 0.0);
    ASSERT_DOUBLE_EQ(link1->geometry.color.z(), 1.0);
    ASSERT_DOUBLE_EQ(link1->geometry.color.w(), 1.0);

    // Check visual origin link1 (0.175 0 0)
    auto link1_visual_origin = link1->geometry.visual_origin;
    ASSERT_DOUBLE_EQ(link1_visual_origin(0, 3), 0.175);
    ASSERT_DOUBLE_EQ(link1_visual_origin(1, 3), 0.0);
    ASSERT_DOUBLE_EQ(link1_visual_origin(2, 3), 0.0);

    // Check link2
    auto link2 = robot->getLink("link2");
    ASSERT_NE(link2, nullptr);
    ASSERT_EQ(link2->name, "link2");
    ASSERT_EQ(link2->geometry.type, Geometry::Type::BOX);
    ASSERT_NE(link2->parent_joint, nullptr);
    ASSERT_EQ(link2->parent_joint->getName(), "joint2");
    ASSERT_NE(link2->child_joint, nullptr);
    ASSERT_EQ(link2->child_joint->getName(), "end_effector");

    // Check geometry parameters link2 (0.450 0.09 0.025)
    ASSERT_EQ(link2->geometry.parameters.size(), 3);
    ASSERT_DOUBLE_EQ(link2->geometry.parameters[0], 0.450);
    ASSERT_DOUBLE_EQ(link2->geometry.parameters[1], 0.09);
    ASSERT_DOUBLE_EQ(link2->geometry.parameters[2], 0.025);

    // Check color link2 (1 0 0 1 - red)
    ASSERT_DOUBLE_EQ(link2->geometry.color.x(), 1.0);
    ASSERT_DOUBLE_EQ(link2->geometry.color.y(), 0.0);
    ASSERT_DOUBLE_EQ(link2->geometry.color.z(), 0.0);
    ASSERT_DOUBLE_EQ(link2->geometry.color.w(), 1.0);

    // Check visual origin link2 (0.225 0 0)
    auto link2_visual_origin = link2->geometry.visual_origin;
    ASSERT_DOUBLE_EQ(link2_visual_origin(0, 3), 0.225);
    ASSERT_DOUBLE_EQ(link2_visual_origin(1, 3), 0.0);
    ASSERT_DOUBLE_EQ(link2_visual_origin(2, 3), 0.0);

    // Check link_end_effector
    auto link_end_effector = robot->getLink("link_end_effector");
    ASSERT_NE(link_end_effector, nullptr);
    ASSERT_EQ(link_end_effector->name, "link_end_effector");
    ASSERT_EQ(link_end_effector->geometry.type, Geometry::Type::CYLINDER);
    ASSERT_NE(link_end_effector->parent_joint, nullptr);
    ASSERT_EQ(link_end_effector->parent_joint->getName(), "end_effector");
    ASSERT_EQ(link_end_effector->child_joint, nullptr);

    // Check geometry parameters link_end_effector (radius=0.02, length=0.2)
    ASSERT_EQ(link_end_effector->geometry.parameters.size(), 2);
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.parameters[0], 0.02); // radius
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.parameters[1], 0.2);  // length

    // Check color link_end_effector (1 0.5 0 1 - orange)
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.color.x(), 1.0);
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.color.y(), 0.5);
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.color.z(), 0.0);
    ASSERT_DOUBLE_EQ(link_end_effector->geometry.color.w(), 1.0);

    // Check visual origin link_end_effector (0 0 0.1)
    auto link_end_effector_visual_origin =
        link_end_effector->geometry.visual_origin;
    ASSERT_DOUBLE_EQ(link_end_effector_visual_origin(0, 3), 0.0);
    ASSERT_DOUBLE_EQ(link_end_effector_visual_origin(1, 3), 0.0);
    ASSERT_DOUBLE_EQ(link_end_effector_visual_origin(2, 3), 0.1);

    // ==================== CHECK RELATIONS ====================

    // Check parent-child relationships
    ASSERT_EQ(base_link->parent_joint, nullptr); // base_link has no parent
    ASSERT_EQ(base_link->child_joint, joint1);   // base_link -> joint1

    ASSERT_EQ(link1->parent_joint, joint1); // joint1 -> link1
    ASSERT_EQ(link1->child_joint, joint2);  // link1 -> joint2

    ASSERT_EQ(link2->parent_joint, joint2);            // joint2 -> link2
    ASSERT_EQ(link2->child_joint, end_effector_joint); // link2 -> end_effector

    ASSERT_EQ(link_end_effector->parent_joint,
              end_effector_joint); // end_effector -> link_end_effector
    ASSERT_EQ(link_end_effector->child_joint,
              nullptr); // link_end_effector has no child

    // ==================== CHECK KINEMATICS ====================

    // Check Jacobian (should be 6x3 for 3 joints)
    auto jacobian = robot->calculateJacobian();
    std::cout << "Jacobian:" << std::endl;
    std::cout << jacobian << std::endl;

    ASSERT_EQ(jacobian.rows(), 6);
    ASSERT_EQ(jacobian.cols(), 3);

    // Check end effector pose (should be a 6x1 vector)
    auto end_effector_pose = robot->getEndEffectorPose();
    ASSERT_EQ(end_effector_pose.rows(), 6);
    ASSERT_EQ(end_effector_pose.cols(), 1);

    // Check joint values (all should be 0 by default)
    auto joint_values = robot->getJointValues();
    ASSERT_EQ(joint_values.size(), 3);
    ASSERT_DOUBLE_EQ(joint_values[0], 0.0); // joint1
    ASSERT_DOUBLE_EQ(joint_values[1], 0.0); // joint2
    ASSERT_DOUBLE_EQ(joint_values[2], 0.0); // end_effector

    // ==================== CHECK TRANSFORMATIONS ====================

    // Check that local transformations are well calculated
    auto const& joint1_local_transform = joint1->getLocalTransform();
    auto const& joint2_local_transform = joint2->getLocalTransform();
    auto const& end_effector_local_transform =
        end_effector_joint->getLocalTransform();

    // The transformations should be 4x4 matrices
    ASSERT_EQ(joint1_local_transform.rows(), 4);
    ASSERT_EQ(joint1_local_transform.cols(), 4);
    ASSERT_EQ(joint2_local_transform.rows(), 4);
    ASSERT_EQ(joint2_local_transform.cols(), 4);
    ASSERT_EQ(end_effector_local_transform.rows(), 4);
    ASSERT_EQ(end_effector_local_transform.cols(), 4);

    // Check that global transformations are well calculated
    auto const& joint1_world_transform = joint1->getWorldTransform();
    auto const& joint2_world_transform = joint2->getWorldTransform();
    auto const& end_effector_world_transform =
        end_effector_joint->getWorldTransform();

    // The transformations should be 4x4 matrices
    ASSERT_EQ(joint1_world_transform.rows(), 4);
    ASSERT_EQ(joint1_world_transform.cols(), 4);
    ASSERT_EQ(joint2_world_transform.rows(), 4);
    ASSERT_EQ(joint2_world_transform.cols(), 4);
    ASSERT_EQ(end_effector_world_transform.rows(), 4);
    ASSERT_EQ(end_effector_world_transform.cols(), 4);

    // ==================== CHECK NUMERICAL VALUES ====================

    // Check local transformations
    compareMatrices(joint1_local_transform, createTransform(0.0, 0.0, 0.06));
    compareMatrices(joint2_local_transform, createTransform(0.35, 0.0, 0.0));
    compareMatrices(end_effector_local_transform,
                    createTransform(0.45, 0.0, 0.0));

    // Check world transformations
    compareMatrices(joint1_world_transform, createTransform(0.0, 0.0, 0.06));
    compareMatrices(joint2_world_transform, createTransform(0.35, 0.0, 0.06));
    compareMatrices(end_effector_world_transform,
                    createTransform(0.8, 0.0, 0.06));

    // Check end effector pose
    Eigen::Matrix<double, 6, 1> expected_pose;
    expected_pose << 0.8, 0.0, 0.06, 0.0, 0.0, 0.0;

    for (int i = 0; i < 6; ++i)
    {
        ASSERT_DOUBLE_EQ(end_effector_pose(i, 0), expected_pose(i, 0))
            << "End effector pose difference at index " << i;
    }

    // Check Jacobian numerical values
    robotik::Jacobian expected_jacobian(6, 3);
    expected_jacobian << 0.0, 0.0, 0.0, // Linear velocity X
        0.8, 0.45, 0.0,                 // Linear velocity Y
        0.0, 0.0, -1.0,                 // Linear velocity Z
        0.0, 0.0, 0.0,                  // Angular velocity X
        0.0, 0.0, 0.0,                  // Angular velocity Y
        1.0, 1.0, 0.0;                  // Angular velocity Z

    ASSERT_EQ(jacobian.rows(), expected_jacobian.rows());
    ASSERT_EQ(jacobian.cols(), expected_jacobian.cols());

    for (int i = 0; i < jacobian.rows(); ++i)
    {
        for (int j = 0; j < jacobian.cols(); ++j)
        {
            ASSERT_DOUBLE_EQ(jacobian(i, j), expected_jacobian(i, j))
                << "Jacobian difference at (" << i << "," << j << ")";
        }
    }
}

#endif