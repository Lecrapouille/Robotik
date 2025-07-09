#include "Robotik/Robotik.hpp"
#include <cmath>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Joint class.
// *********************************************************************************
class JointTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Setup test data
        axis = Eigen::Vector3d(0, 0, 1); // Z-axis
        revolute_joint =
            std::make_unique<Joint>("revolute", Joint::Type::REVOLUTE, axis);
        prismatic_joint =
            std::make_unique<Joint>("prismatic", Joint::Type::PRISMATIC, axis);
        fixed_joint =
            std::make_unique<Joint>("fixed", Joint::Type::FIXED, axis);
    }

    Eigen::Vector3d axis;
    std::unique_ptr<Joint> revolute_joint;
    std::unique_ptr<Joint> prismatic_joint;
    std::unique_ptr<Joint> fixed_joint;
};

// *********************************************************************************
//! \brief Test Joint creation and basic properties.
// *********************************************************************************
TEST_F(JointTest, Creation)
{
    EXPECT_EQ(revolute_joint->getName(), "revolute");
    EXPECT_EQ(revolute_joint->getType(), Joint::Type::REVOLUTE);
    EXPECT_TRUE(revolute_joint->getAxis().isApprox(axis));
    EXPECT_EQ(revolute_joint->getValue(), 0.0);

    EXPECT_EQ(prismatic_joint->getName(), "prismatic");
    EXPECT_EQ(prismatic_joint->getType(), Joint::Type::PRISMATIC);
    EXPECT_TRUE(prismatic_joint->getAxis().isApprox(axis));
    EXPECT_EQ(prismatic_joint->getValue(), 0.0);

    EXPECT_EQ(fixed_joint->getName(), "fixed");
    EXPECT_EQ(fixed_joint->getType(), Joint::Type::FIXED);
    EXPECT_TRUE(fixed_joint->getAxis().isApprox(axis));
    EXPECT_EQ(fixed_joint->getValue(), 0.0);
}

// *********************************************************************************
//! \brief Test joint axis normalization.
// *********************************************************************************
TEST_F(JointTest, AxisNormalization)
{
    // Test that axis is normalized during construction
    Eigen::Vector3d unnormalized_axis(2, 0, 0);
    auto joint = std::make_unique<Joint>(
        "test", Joint::Type::REVOLUTE, unnormalized_axis);

    Eigen::Vector3d expected_normalized = unnormalized_axis.normalized();
    EXPECT_TRUE(joint->getAxis().isApprox(expected_normalized));
    EXPECT_DOUBLE_EQ(joint->getAxis().norm(), 1.0);
}

// *********************************************************************************
//! \brief Test setting and getting joint values.
// *********************************************************************************
TEST_F(JointTest, SetGetValue)
{
    revolute_joint->setValue(M_PI / 4);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 4);

    prismatic_joint->setValue(0.5);
    EXPECT_DOUBLE_EQ(prismatic_joint->getValue(), 0.5);

    // Fixed joint also applies limits by default, so 100.0 will be clamped to
    // M_PI
    fixed_joint->setValue(100.0);
    EXPECT_DOUBLE_EQ(fixed_joint->getValue(),
                     M_PI); // Should be clamped to max limit

    // Test with value within limits
    fixed_joint->setValue(1.0);
    EXPECT_DOUBLE_EQ(fixed_joint->getValue(), 1.0);
}

// *********************************************************************************
//! \brief Test joint limits.
// *********************************************************************************
TEST_F(JointTest, Limits)
{
    revolute_joint->setLimits(-M_PI, M_PI);

    // Test setting value within limits
    revolute_joint->setValue(M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 2);

    // Test setting value at limits
    revolute_joint->setValue(M_PI);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI);

    revolute_joint->setValue(-M_PI);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), -M_PI);

    // Test setting value beyond limits (should be clamped)
    revolute_joint->setValue(2 * M_PI);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI);

    revolute_joint->setValue(-2 * M_PI);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), -M_PI);
}

// *********************************************************************************
//! \brief Test joint limits behavior (we can't test getMin/getMax as they are
//! not public).
// *********************************************************************************
TEST_F(JointTest, LimitsBehavior)
{
    // Test that limits are enforced during setValue
    revolute_joint->setLimits(-M_PI / 2, M_PI / 2);

    // Test value within limits
    revolute_joint->setValue(M_PI / 4);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 4);

    // Test value at limits
    revolute_joint->setValue(M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 2);

    revolute_joint->setValue(-M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), -M_PI / 2);

    // Test prismatic joint limits
    prismatic_joint->setLimits(0.0, 1.0);

    prismatic_joint->setValue(0.5);
    EXPECT_DOUBLE_EQ(prismatic_joint->getValue(), 0.5);

    prismatic_joint->setValue(1.0);
    EXPECT_DOUBLE_EQ(prismatic_joint->getValue(), 1.0);
}

// *********************************************************************************
//! \brief Test revolute joint transformations.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransform)
{
    revolute_joint->setValue(M_PI / 2); // 90 degrees

    Transform transform = revolute_joint->getTransform();

    // Check that it's a rotation around Z-axis
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    // For 90 degrees around Z-axis, we should get specific rotation matrix
    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    EXPECT_TRUE(rotation.isApprox(expected_rotation, 1e-10));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d::Zero()));
}

// *********************************************************************************
//! \brief Test revolute joint with zero rotation.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransformZero)
{
    revolute_joint->setValue(0.0);

    Transform transform = revolute_joint->getTransform();

    // Should be identity matrix for zero rotation
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test revolute joint with negative rotation.
// *********************************************************************************
TEST_F(JointTest, RevoluteTransformNegative)
{
    revolute_joint->setValue(-M_PI / 2); // -90 degrees

    Transform transform = revolute_joint->getTransform();
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);

    // For -90 degrees around Z-axis
    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, 1, 0, -1, 0, 0, 0, 0, 1;

    EXPECT_TRUE(rotation.isApprox(expected_rotation, 1e-10));
}

// *********************************************************************************
//! \brief Test prismatic joint transformations.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransform)
{
    prismatic_joint->setValue(0.5);

    Transform transform = prismatic_joint->getTransform();

    // Check that it's a translation along Z-axis
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transform.block<3, 1>(0, 3);

    EXPECT_TRUE(rotation.isApprox(Eigen::Matrix3d::Identity()));
    EXPECT_TRUE(translation.isApprox(Eigen::Vector3d(0, 0, 0.5)));
}

// *********************************************************************************
//! \brief Test prismatic joint with zero translation.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransformZero)
{
    prismatic_joint->setValue(0.0);

    Transform transform = prismatic_joint->getTransform();

    // Should be identity matrix for zero translation
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test prismatic joint with negative translation.
// *********************************************************************************
TEST_F(JointTest, PrismaticTransformNegative)
{
    prismatic_joint->setValue(-0.3);

    Transform transform = prismatic_joint->getTransform();
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
    fixed_joint->setValue(100.0); // Value should be ignored

    Transform transform = fixed_joint->getTransform();

    // Fixed joint should always return identity transform
    EXPECT_TRUE(transform.isApprox(Eigen::Matrix4d::Identity()));
}

// *********************************************************************************
//! \brief Test node transform update.
// *********************************************************************************
TEST_F(JointTest, NodeTransformUpdate)
{
    revolute_joint->setValue(M_PI / 4);

    // Update node transform
    revolute_joint->updateNodeTransform();

    // Check that node's local transform has been updated
    Transform expected_transform = revolute_joint->getTransform();
    EXPECT_TRUE(
        revolute_joint->getLocalTransform().isApprox(expected_transform));
}

// *********************************************************************************
//! \brief Test automatic transform update when setting value.
// *********************************************************************************
TEST_F(JointTest, AutomaticTransformUpdate)
{
    // Set initial value
    revolute_joint->setValue(M_PI / 6);
    Transform initial_transform = revolute_joint->getLocalTransform();

    // Change value - should automatically update local transform
    revolute_joint->setValue(M_PI / 3);
    Transform updated_transform = revolute_joint->getLocalTransform();

    // Transforms should be different
    EXPECT_FALSE(initial_transform.isApprox(updated_transform));

    // Updated transform should match expected
    Transform expected_transform = revolute_joint->getTransform();
    EXPECT_TRUE(updated_transform.isApprox(expected_transform));
}

// *********************************************************************************
//! \brief Test different axes for revolute joints.
// *********************************************************************************
TEST_F(JointTest, RevoluteAxes)
{
    // Test X-axis rotation
    Eigen::Vector3d x_axis(1, 0, 0);
    auto x_joint =
        std::make_unique<Joint>("x_joint", Joint::Type::REVOLUTE, x_axis);
    x_joint->setValue(M_PI / 2);

    Transform x_transform = x_joint->getTransform();
    Eigen::Matrix3d x_rotation = x_transform.block<3, 3>(0, 0);

    Eigen::Matrix3d expected_x_rotation;
    expected_x_rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;

    EXPECT_TRUE(x_rotation.isApprox(expected_x_rotation, 1e-10));

    // Test Y-axis rotation
    Eigen::Vector3d y_axis(0, 1, 0);
    auto y_joint =
        std::make_unique<Joint>("y_joint", Joint::Type::REVOLUTE, y_axis);
    y_joint->setValue(M_PI / 2);

    Transform y_transform = y_joint->getTransform();
    Eigen::Matrix3d y_rotation = y_transform.block<3, 3>(0, 0);

    Eigen::Matrix3d expected_y_rotation;
    expected_y_rotation << 0, 0, 1, 0, 1, 0, -1, 0, 0;

    EXPECT_TRUE(y_rotation.isApprox(expected_y_rotation, 1e-10));
}

// *********************************************************************************
//! \brief Test different axes for prismatic joints.
// *********************************************************************************
TEST_F(JointTest, PrismaticAxes)
{
    // Test X-axis translation
    Eigen::Vector3d x_axis(1, 0, 0);
    auto x_joint =
        std::make_unique<Joint>("x_joint", Joint::Type::PRISMATIC, x_axis);
    x_joint->setValue(0.5);

    Transform x_transform = x_joint->getTransform();
    Eigen::Vector3d x_translation = x_transform.block<3, 1>(0, 3);

    EXPECT_TRUE(x_translation.isApprox(Eigen::Vector3d(0.5, 0, 0)));

    // Test Y-axis translation
    Eigen::Vector3d y_axis(0, 1, 0);
    auto y_joint =
        std::make_unique<Joint>("y_joint", Joint::Type::PRISMATIC, y_axis);
    y_joint->setValue(0.3);

    Transform y_transform = y_joint->getTransform();
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
    auto joint = std::make_unique<Joint>(
        "arbitrary", Joint::Type::REVOLUTE, arbitrary_axis);

    // Verify axis is normalized
    EXPECT_TRUE(joint->getAxis().isApprox(arbitrary_axis.normalized()));

    // Test rotation around arbitrary axis
    joint->setValue(M_PI / 4);
    Transform transform = joint->getTransform();

    // Verify it's a valid rotation matrix
    Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
    EXPECT_TRUE((rotation.determinant() >
                 0.99)); // Should be close to 1 for rotation matrix
    EXPECT_TRUE((rotation * rotation.transpose())
                    .isApprox(Eigen::Matrix3d::Identity(), 1e-10));
}

// *********************************************************************************
//! \brief Test joint hierarchy and transform propagation.
// *********************************************************************************
TEST_F(JointTest, JointHierarchy)
{
    // Create parent joint
    auto parent_joint = std::make_unique<Joint>(
        "parent", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
    Joint* parent_ptr = parent_joint.get();

    // Create child joint using createChild to establish proper parent-child
    // relationship
    Joint& child_joint = parent_ptr->createChild<Joint>(
        "child", Joint::Type::REVOLUTE, Eigen::Vector3d(1, 0, 0));

    // Set joint values
    parent_ptr->setValue(M_PI / 2); // 90 degrees around Z
    child_joint.setValue(M_PI / 4); // 45 degrees around X

    // Update transforms
    parent_ptr->updateWorldTransform();

    // Check that child's world transform is composition of parent and child
    Transform expected_parent_world = parent_ptr->getTransform();
    Transform expected_child_world =
        expected_parent_world * child_joint.getTransform();

    EXPECT_TRUE(
        parent_ptr->getWorldTransform().isApprox(expected_parent_world));
    EXPECT_TRUE(child_joint.getWorldTransform().isApprox(expected_child_world));
}

// *********************************************************************************
//! \brief Test edge cases.
// *********************************************************************************
TEST_F(JointTest, EdgeCases)
{
    // Test zero value
    revolute_joint->setValue(0.0);
    Transform identity_transform = revolute_joint->getTransform();
    EXPECT_TRUE(identity_transform.isApprox(Eigen::Matrix4d::Identity()));

    // Test negative values
    revolute_joint->setValue(-M_PI / 4);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), -M_PI / 4);

    // Test very small values
    revolute_joint->setValue(1e-10);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), 1e-10);

    // Test very large values (should be clamped)
    revolute_joint->setValue(100.0);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(),
                     M_PI); // Should be clamped to max limit

    // Test prismatic joint with zero axis component (should still work due to
    // normalization)
    Eigen::Vector3d zero_axis(0, 0, 0);
    // This should not crash but behavior is undefined - in practice avoid zero
    // vectors
}

// *********************************************************************************
//! \brief Test joint limits with various scenarios.
// *********************************************************************************
TEST_F(JointTest, ExtensiveLimitsTest)
{
    // Test symmetric limits
    revolute_joint->setLimits(-M_PI / 2, M_PI / 2);

    revolute_joint->setValue(M_PI / 3);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 3);

    revolute_joint->setValue(M_PI); // Should be clamped to max
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 2);

    // Test asymmetric limits
    revolute_joint->setLimits(-M_PI / 4, 3 * M_PI / 4);

    revolute_joint->setValue(M_PI / 2);
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), M_PI / 2);

    revolute_joint->setValue(-M_PI / 2); // Should be clamped to min
    EXPECT_DOUBLE_EQ(revolute_joint->getValue(), -M_PI / 4);

    // Test inverted limits (min > max) - should work but behavior may be
    // undefined
    revolute_joint->setLimits(M_PI / 2, -M_PI / 2);
    revolute_joint->setValue(0.0);
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
        double angle = 2.0 * M_PI * i / num_iterations;
        revolute_joint->setValue(angle);

        // Ensure transform is computed correctly each time
        Transform transform = revolute_joint->getTransform();
        EXPECT_TRUE((transform.block<3, 3>(0, 0).determinant() > 0.99));
    }
}

// *********************************************************************************
//! \brief Test joint naming and identification.
// *********************************************************************************
TEST_F(JointTest, JointNaming)
{
    EXPECT_EQ(revolute_joint->getName(), "revolute");
    EXPECT_EQ(prismatic_joint->getName(), "prismatic");
    EXPECT_EQ(fixed_joint->getName(), "fixed");

    // Test joint with empty name
    auto empty_named_joint =
        std::make_unique<Joint>("", Joint::Type::REVOLUTE, axis);
    EXPECT_EQ(empty_named_joint->getName(), "");

    // Test joint with special characters in name
    auto special_joint =
        std::make_unique<Joint>("joint_01-test", Joint::Type::REVOLUTE, axis);
    EXPECT_EQ(special_joint->getName(), "joint_01-test");
}