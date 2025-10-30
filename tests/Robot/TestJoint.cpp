/**
 * @file TestJoint.cpp
 * @brief Unit tests for the Joint class - Verification of different joint types
 * (revolute, prismatic, fixed), their properties and transformations.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Robot/Blueprint/Joint.hpp"

#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for different types of joints.
// *********************************************************************************
class JointTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        axis = Eigen::Vector3d(0, 0, 1); // Z-axis
        revolute = Node::create<Joint>("revolute", Joint::Type::REVOLUTE, axis);
        prismatic =
            Node::create<Joint>("prismatic", Joint::Type::PRISMATIC, axis);
        fixed = Node::create<Joint>("fixed", Joint::Type::FIXED, axis);
    }

    Eigen::Vector3d axis;
    Joint::Ptr revolute;
    Joint::Ptr prismatic;
    Joint::Ptr fixed;
};

// *********************************************************************************
//! \brief Test Joint creation and basic properties.
// *********************************************************************************
TEST_F(JointTest, Creation)
{
    EXPECT_EQ(revolute->name(), "revolute");
    EXPECT_EQ(revolute->type(), Joint::Type::REVOLUTE);
    EXPECT_TRUE(revolute->axis().isApprox(axis));
    EXPECT_EQ(revolute->position(), 0.0);
    EXPECT_EQ(revolute->limits().first, -M_PI);
    EXPECT_EQ(revolute->limits().second, M_PI);

    EXPECT_EQ(prismatic->name(), "prismatic");
    EXPECT_EQ(prismatic->type(), Joint::Type::PRISMATIC);
    EXPECT_TRUE(prismatic->axis().isApprox(axis));
    EXPECT_EQ(prismatic->position(), 0.0);
    EXPECT_EQ(prismatic->limits().first, -1.0);
    EXPECT_EQ(prismatic->limits().second, 1.0);

    EXPECT_EQ(fixed->name(), "fixed");
    EXPECT_EQ(fixed->type(), Joint::Type::FIXED);
    EXPECT_TRUE(fixed->axis().isApprox(axis));
    EXPECT_DOUBLE_EQ(fixed->position(), 0.0);
    EXPECT_DOUBLE_EQ(fixed->limits().first, 0.0);
    EXPECT_DOUBLE_EQ(fixed->limits().second, 0.0);
}

// *********************************************************************************
//! \brief Test joint axis normalization.
// *********************************************************************************
TEST_F(JointTest, AxisNormalization)
{
    // Test that axis is normalized during construction
    Eigen::Vector3d unnormalized_axis(2, 0, 0);
    auto joint =
        Node::create<Joint>("test", Joint::Type::REVOLUTE, unnormalized_axis);

    Eigen::Vector3d expected_normalized = unnormalized_axis.normalized();
    EXPECT_TRUE(joint->axis().isApprox(expected_normalized));
    EXPECT_DOUBLE_EQ(joint->axis().norm(), 1.0);
}

// *********************************************************************************
//! \brief Test setting and getting joint values.
// *********************************************************************************
TEST_F(JointTest, SetGetValue)
{
    revolute->position(M_PI / 4.0);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 4.0);

    prismatic->position(0.5);
    EXPECT_DOUBLE_EQ(prismatic->position(), 0.5);

    fixed->position(100.0);
    EXPECT_DOUBLE_EQ(fixed->position(), 0.0);

    fixed->position(1.0);
    EXPECT_DOUBLE_EQ(fixed->position(), 0.0);
}

// *********************************************************************************
//! \brief Test joint limits.
// *********************************************************************************
TEST_F(JointTest, Limits)
{
    revolute->limits(-M_PI, M_PI);

    // Test setting value within limits
    revolute->position(M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 2);

    // Test setting value at limits
    revolute->position(M_PI);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI);

    revolute->position(-M_PI);
    EXPECT_DOUBLE_EQ(revolute->position(), -M_PI);

    // Test setting value beyond limits (should be clamped)
    revolute->position(2 * M_PI);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI);

    revolute->position(-2 * M_PI);
    EXPECT_DOUBLE_EQ(revolute->position(), -M_PI);
}

// *********************************************************************************
//! \brief Test joint limits behavior (we can't test getMin/getMax as they are
//! not public).
// *********************************************************************************
TEST_F(JointTest, LimitsBehavior)
{
    // Test that limits are enforced during value
    revolute->limits(-M_PI / 2, M_PI / 2);

    // Test value within limits
    revolute->position(M_PI / 4);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 4);

    // Test value at limits
    revolute->position(M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 2);

    revolute->position(-M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute->position(), -M_PI / 2);

    // Test prismatic joint limits
    prismatic->limits(0.0, 1.0);

    prismatic->position(0.5);
    EXPECT_DOUBLE_EQ(prismatic->position(), 0.5);

    prismatic->position(1.0);
    EXPECT_DOUBLE_EQ(prismatic->position(), 1.0);
}

// *********************************************************************************
//! \brief Test revolute joint with zero rotation.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransform0Degrees)
{
    // 0 degrees around it axis (Z-axis)
    revolute->position(0.0);
    Transform transform = revolute->localTransform();

    // Should be identity matrix for zero rotation
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test revolute joint transformations.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransform90Degrees)
{
    // 90 degrees around it axis (Z-axis)
    revolute->position(M_PI / 2);
    Transform transform = revolute->localTransform();

    // Check that it's a rotation around Z-axis, and no translation
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    EXPECT_TRUE(rotation.isApprox(expected_rotation));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d::Zero()));
}

// *********************************************************************************
//! \brief Test revolute joint with negative rotation.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransformNegative90Degrees)
{
    // -90 degrees around it axis (Z-axis)
    revolute->position(-M_PI / 2);
    Transform transform = revolute->localTransform();

    // Check that it's a rotation around Z-axis, and no translation
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    EXPECT_TRUE(rotation.isApprox(expected_rotation));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d::Zero()));
}

// *********************************************************************************
//! \brief Test prismatic joint with zero translation.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransformZero)
{
    // 0 meters along it axis (Z-axis)
    prismatic->position(0.0);
    Transform transform = prismatic->localTransform();

    // Should be identity matrix for zero translation
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test prismatic joint transformations.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransform)
{
    // 0.5 meters along it axis (Z-axis)
    prismatic->position(0.5);
    Transform transform = prismatic->localTransform();

    // Check that it's a translation along Z-axis, and no rotation
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d(0, 0, 0.5)));
}

// *********************************************************************************
//! \brief Test prismatic joint with negative translation.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransformNegative)
{
    // -0.3 meters along it axis (Z-axis)
    prismatic->position(-0.3);
    Transform transform = prismatic->localTransform();

    // Check that it's a translation along Z-axis, and no rotation
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);
    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d(0, 0, -0.3)));
}

// *********************************************************************************
//! \brief Test fixed joint transformations.
// *********************************************************************************
TEST_F(JointTest, FixedTransform)
{
    // 100 meters along it axis (Z-axis)
    fixed->position(100.0);
    Transform transform = fixed->localTransform();

    // Fixed joint should always return identity transform
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test automatic transform update when setting value.
// *********************************************************************************
TEST_F(JointTest, AutomaticTransformUpdate)
{
    // Set initial value
    revolute->position(M_PI / 6.0);
    Transform initial_transform = revolute->localTransform();

    // Change value - should automatically update local transform
    revolute->position(M_PI / 3.0);
    Transform updated_transform = revolute->localTransform();

    // Transforms should be different
    EXPECT_FALSE(initial_transform.isApprox(updated_transform));

    // Check that it's a rotation around Z-axis, and no translation
    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    EXPECT_TRUE(expected_rotation.isApprox(expected_rotation));
}

// *********************************************************************************
//! \brief Test different axes for revolute joints.
// *********************************************************************************
TEST_F(JointTest, RevoluteAxes)
{
    // Test X-axis rotation
    Eigen::Vector3d x_axis(1, 0, 0);
    auto x_joint =
        Node::create<Joint>("x_joint", Joint::Type::REVOLUTE, x_axis);
    x_joint->position(M_PI / 2.0);

    // Check that it's a rotation around X-axis, and no translation
    Transform x_transform = x_joint->localTransform();
    Eigen::Matrix3d x_rotation = x_transform.block<3, 3>(0, 0);
    Eigen::Matrix3d expected_x_rotation;
    expected_x_rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    EXPECT_TRUE(x_rotation.isApprox(expected_x_rotation));

    // Test Y-axis rotation
    Eigen::Vector3d y_axis(0, 1, 0);
    auto y_joint =
        Node::create<Joint>("y_joint", Joint::Type::REVOLUTE, y_axis);
    y_joint->position(M_PI / 2.0);

    // Check that it's a rotation around Y-axis, and no translation
    Transform y_transform = y_joint->localTransform();
    Eigen::Matrix3d y_rotation = y_transform.block<3, 3>(0, 0);

    Eigen::Matrix3d expected_y_rotation;
    expected_y_rotation << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    EXPECT_TRUE(y_rotation.isApprox(expected_y_rotation));
}

// *********************************************************************************
//! \brief Test different axes for prismatic joints.
// *********************************************************************************
TEST_F(JointTest, PrismaticAxes)
{
    // Test X-axis translation
    Eigen::Vector3d x_axis(1, 0, 0);
    auto x_joint =
        Node::create<Joint>("x_joint", Joint::Type::PRISMATIC, x_axis);
    x_joint->position(0.5); // 0.5 meters

    // Check that it's a translation along X-axis, and no rotation
    Transform x_transform = x_joint->localTransform();
    Eigen::Vector3d x_translation = x_transform.block<3, 1>(0, 3);
    EXPECT_TRUE(x_translation.isApprox(Eigen::Vector3d(0.5, 0, 0)));

    // Test Y-axis translation
    Eigen::Vector3d y_axis(0, 1, 0);
    auto y_joint =
        Node::create<Joint>("y_joint", Joint::Type::PRISMATIC, y_axis);
    y_joint->position(0.3); // 0.3 meters

    // Check that it's a translation along Y-axis, and no rotation
    Transform y_transform = y_joint->localTransform();
    Eigen::Vector3d y_translation = y_transform.block<3, 1>(0, 3);
    EXPECT_TRUE(y_translation.isApprox(Eigen::Vector3d(0, 0.3, 0)));
}

// *********************************************************************************
//! \brief Test arbitrary axis normalization and rotation.
// *********************************************************************************
TEST_F(JointTest, ArbitraryAxisRotation)
{
    // Test with arbitrary axis
    Eigen::Vector3d arbitrary_axis(1, 1, 1);
    auto joint =
        Node::create<Joint>("arbitrary", Joint::Type::REVOLUTE, arbitrary_axis);

    // Verify axis is normalized
    EXPECT_TRUE(joint->axis().isApprox(arbitrary_axis.normalized()));

    // Test rotation around arbitrary axis
    joint->position(M_PI / 4.0);

    // Check that it's a rotation around arbitrary axis, and no translation
    Transform transform = joint->localTransform();
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

    // Should be close to 1 for rotation matrix
    EXPECT_TRUE((rotation.determinant() > 0.99));
    EXPECT_TRUE((rotation * rotation.transpose())
                    .isApprox(Eigen::Matrix3d::Identity()));
}

// *********************************************************************************
//! \brief Test joint blueprint and transform propagation.
// *********************************************************************************
TEST_F(JointTest, JointBlueprint)
{
    // Create parent joint
    auto parent_joint = Node::create<Joint>(
        "parent", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
    Joint* parent_ptr = parent_joint.get();

    // Create child joint using createChild to establish proper parent-child
    // relationship
    Joint& child_joint = parent_ptr->createChild<Joint>(
        "child", Joint::Type::REVOLUTE, Eigen::Vector3d(1, 0, 0));

    // Set joint values
    parent_ptr->position(M_PI / 2.0); // 90 degrees around Z
    child_joint.position(M_PI / 4.0); // 45 degrees around X

    // Check that child's world transform is composition of parent and child
    Transform expected_parent_world = parent_ptr->localTransform();
    Transform expected_child_world =
        expected_parent_world * child_joint.localTransform();

    EXPECT_TRUE(parent_ptr->worldTransform().isApprox(expected_parent_world));
    EXPECT_TRUE(child_joint.worldTransform().isApprox(expected_child_world));
}

// *********************************************************************************
//! \brief Test edge cases.
// *********************************************************************************
TEST_F(JointTest, EdgeCases)
{
    // Test zero value
    revolute->position(0.0);

    Transform identity_transform = revolute->localTransform();
    EXPECT_TRUE(identity_transform.isApprox(Eigen::Matrix4d::Identity()));

    // Test negative values
    revolute->position(-M_PI / 4.0);
    EXPECT_DOUBLE_EQ(revolute->position(), -M_PI / 4.0);

    // Test very small values
    revolute->position(1e-10);
    EXPECT_DOUBLE_EQ(revolute->position(), 1e-10);

    // Test very large values (should be clamped).
    // Check that it's clamped to max limit.
    revolute->position(100.0);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI);

    // Test prismatic joint with zero axis component (should still work due to
    // normalization).
    // Check this should not crash but behavior is undefined - in practice avoid
    // zero vectors
    Eigen::Vector3d zero_axis(0, 0, 0);
}

// *********************************************************************************
//! \brief Test joint limits with various scenarios.
// *********************************************************************************
TEST_F(JointTest, ExtensiveLimitsTest)
{
    // Test symmetric limits
    revolute->limits(-M_PI / 2.0, M_PI / 2.0);

    revolute->position(M_PI / 3.0);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 3.0);

    revolute->position(M_PI); // Should be clamped to max
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 2.0);

    // Test asymmetric limits
    revolute->limits(-M_PI / 4.0, 3 * M_PI / 4.0);

    revolute->position(M_PI / 2.0);
    EXPECT_DOUBLE_EQ(revolute->position(), M_PI / 2.0);

    revolute->position(-M_PI / 2.0); // Should be clamped to min
    EXPECT_DOUBLE_EQ(revolute->position(), -M_PI / 4.0);

    // Test inverted limits (min > max) - should work but behavior may be
    // undefined
    revolute->limits(M_PI / 2.0, -M_PI / 2.0);
    revolute->position(0.0);
    // Behavior depends on implementation
}

// *********************************************************************************
//! \brief Test performance with many value changes.
// *********************************************************************************
TEST_F(JointTest, PerformanceTest)
{
    const int num_iterations = 1000;

    for (int i = 0; i < num_iterations; ++i)
    {
        double angle = 2.0 * M_PI * i / double(num_iterations);
        revolute->position(angle);

        Transform transform = revolute->localTransform();
        EXPECT_TRUE((transform.block<3, 3>(0, 0).determinant() > 0.99));
    }
}

// *********************************************************************************
//! \brief Test joint naming and identification.
// *********************************************************************************
TEST_F(JointTest, JointNaming)
{
    EXPECT_EQ(revolute->name(), "revolute");
    EXPECT_EQ(prismatic->name(), "prismatic");
    EXPECT_EQ(fixed->name(), "fixed");

    // Test joint with empty name
    auto empty_named_joint =
        Node::create<Joint>("", Joint::Type::REVOLUTE, axis);
    EXPECT_EQ(empty_named_joint->name(), "");

    // Test joint with special characters in name
    auto special_joint =
        Node::create<Joint>("joint_01-test", Joint::Type::REVOLUTE, axis);
    EXPECT_EQ(special_joint->name(), "joint_01-test");
}