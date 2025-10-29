/**
 * @file TestConversions.cpp
 * @brief Unit tests for utility functions - Verification of coordinate
 * conversions (Euler angles, quaternions, rotation matrices) and basic
 * mathematical functions.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Common/Conversions.hpp"

#include <cmath>

using namespace robotik;
using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Conversions class.
// *********************************************************************************
class ConversionsTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // Common test values
        tolerance = 1e-10;
    }

    double tolerance;
};

// *********************************************************************************
//! \brief Test Euler angles to rotation matrix conversion.
// *********************************************************************************
TEST_F(ConversionsTest, EulerToRotation)
{
    // Test identity rotation
    Eigen::Matrix3d identity_rot = eulerToRotation(0.0, 0.0, 0.0);
    EXPECT_TRUE(identity_rot.isApprox(Eigen::Matrix3d::Identity()));

    // Test 90 degree rotation around Z-axis
    Eigen::Matrix3d z_rot = eulerToRotation(0.0, 0.0, M_PI / 2);
    Eigen::Matrix3d expected_z_rot;
    expected_z_rot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    EXPECT_TRUE(z_rot.isApprox(expected_z_rot));

    // Test 90 degree rotation around X-axis
    Eigen::Matrix3d x_rot = eulerToRotation(M_PI / 2, 0.0, 0.0);
    Eigen::Matrix3d expected_x_rot;
    expected_x_rot << 1, 0, 0, 0, 0, -1, 0, 1, 0;
    EXPECT_TRUE(x_rot.isApprox(expected_x_rot));

    // Test 90 degree rotation around Y-axis
    Eigen::Matrix3d y_rot = eulerToRotation(0.0, M_PI / 2, 0.0);
    Eigen::Matrix3d expected_y_rot;
    expected_y_rot << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    EXPECT_TRUE(y_rot.isApprox(expected_y_rot));
}

// *********************************************************************************
//! \brief Test rotation matrix to Euler angles conversion.
// *********************************************************************************
TEST_F(ConversionsTest, RotationToEuler)
{
    // Test identity rotation
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Vector3d euler = rotationToEuler(identity);
    EXPECT_NEAR(euler(0), 0.0, tolerance);
    EXPECT_NEAR(euler(1), 0.0, tolerance);
    EXPECT_NEAR(euler(2), 0.0, tolerance);

    // Test known rotations
    Eigen::Matrix3d z_rot;
    z_rot << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    Eigen::Vector3d z_euler = rotationToEuler(z_rot);
    EXPECT_NEAR(z_euler(0), 0.0, tolerance);
    EXPECT_NEAR(z_euler(1), 0.0, tolerance);
    EXPECT_NEAR(z_euler(2), M_PI / 2, tolerance);
}

// *********************************************************************************
//! \brief Test round-trip conversion: Euler -> Rotation -> Euler.
// *********************************************************************************
TEST_F(ConversionsTest, EulerRotationRoundTrip)
{
    std::vector<Eigen::Vector3d> test_angles = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(M_PI / 4, 0.0, 0.0),
        Eigen::Vector3d(0.0, M_PI / 4, 0.0),
        Eigen::Vector3d(0.0, 0.0, M_PI / 4),
        Eigen::Vector3d(M_PI / 6, M_PI / 4, M_PI / 3),
        Eigen::Vector3d(-M_PI / 4, M_PI / 6, -M_PI / 3)
    };

    for (const auto& original_angles : test_angles)
    {
        Eigen::Matrix3d rotation = eulerToRotation(
            original_angles(0), original_angles(1), original_angles(2));
        Eigen::Vector3d recovered_angles = rotationToEuler(rotation);

        // Check that the rotation matrices are the same
        Eigen::Matrix3d recovered_rotation = eulerToRotation(
            recovered_angles(0), recovered_angles(1), recovered_angles(2));
        EXPECT_TRUE(rotation.isApprox(recovered_rotation));
    }
}

// *********************************************************************************
//! \brief Test createTransform with translation and rotation matrix.
// *********************************************************************************
TEST_F(ConversionsTest, CreateTransformWithMatrix)
{
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Matrix3d rotation = eulerToRotation(0.0, 0.0, M_PI / 2);

    Transform transform = createTransform(translation, rotation);

    // Check translation part
    EXPECT_TRUE((transform.block<3, 1>(0, 3).isApprox(translation)));

    // Check rotation part
    EXPECT_TRUE((transform.block<3, 3>(0, 0).isApprox(rotation)));

    // Check homogeneous part
    EXPECT_DOUBLE_EQ(transform(3, 0), 0.0);
    EXPECT_DOUBLE_EQ(transform(3, 1), 0.0);
    EXPECT_DOUBLE_EQ(transform(3, 2), 0.0);
    EXPECT_DOUBLE_EQ(transform(3, 3), 1.0);
}

// *********************************************************************************
//! \brief Test createTransform with translation and Euler angles.
// *********************************************************************************
TEST_F(ConversionsTest, CreateTransformWithEuler)
{
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    double rx = M_PI / 4;
    double ry = M_PI / 6;
    double rz = M_PI / 3;

    Transform transform = createTransform(translation, rx, ry, rz);

    // Check translation part
    EXPECT_TRUE((transform.block<3, 1>(0, 3).isApprox(translation)));

    // Check rotation part
    Eigen::Matrix3d expected_rotation = eulerToRotation(rx, ry, rz);
    EXPECT_TRUE((transform.block<3, 3>(0, 0).isApprox(expected_rotation)));
}

// *********************************************************************************
//! \brief Test getTranslation.
// *********************************************************************************
TEST_F(ConversionsTest, GetTranslation)
{
    Eigen::Vector3d expected_translation(1.0, 2.0, 3.0);
    Transform transform = Transform::Identity();
    transform.block<3, 1>(0, 3) = expected_translation;

    Eigen::Vector3d actual_translation = getTranslation(transform);
    EXPECT_TRUE(actual_translation.isApprox(expected_translation));
}

// *********************************************************************************
//! \brief Test getRotation.
// *********************************************************************************
TEST_F(ConversionsTest, GetRotation)
{
    Eigen::Matrix3d expected_rotation =
        eulerToRotation(M_PI / 4, M_PI / 6, M_PI / 3);
    Transform transform = Transform::Identity();
    transform.block<3, 3>(0, 0) = expected_rotation;

    Eigen::Matrix3d actual_rotation = getRotation(transform);
    EXPECT_TRUE(actual_rotation.isApprox(expected_rotation));
}

// *********************************************************************************
//! \brief Test transformToPose.
// *********************************************************************************
TEST_F(ConversionsTest, TransformToPose)
{
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    double rx = M_PI / 4.0;
    double ry = M_PI / 6.0;
    double rz = M_PI / 3.0;
    Transform transform = createTransform(translation, rx, ry, rz);

    Pose pose = transformToPose(transform);

    // Check position
    EXPECT_DOUBLE_EQ(pose(0), 1.0);
    EXPECT_DOUBLE_EQ(pose(1), 2.0);
    EXPECT_DOUBLE_EQ(pose(2), 3.0);

    // Check orientation (should match input angles)
    EXPECT_NEAR(pose(3), rx, tolerance);
    EXPECT_NEAR(pose(4), ry, tolerance);
    EXPECT_NEAR(pose(5), rz, tolerance);
}

// *********************************************************************************
//! \brief Test poseToTransform.
// *********************************************************************************
TEST_F(ConversionsTest, PoseToTransform)
{
    Pose pose;
    pose << 1.0, 2.0, 3.0, M_PI / 4, M_PI / 6, M_PI / 3;

    Transform transform = poseToTransform(pose);

    // Check translation
    Eigen::Vector3d translation = getTranslation(transform);
    EXPECT_TRUE((translation.isApprox(pose.head<3>())));

    // Check rotation
    Eigen::Matrix3d rotation = getRotation(transform);
    Eigen::Matrix3d expected_rotation =
        eulerToRotation(pose(3), pose(4), pose(5));
    EXPECT_TRUE(rotation.isApprox(expected_rotation));
}

// *********************************************************************************
//! \brief Test round-trip conversion: Transform -> Pose -> Transform.
// *********************************************************************************
TEST_F(ConversionsTest, TransformPoseRoundTrip)
{
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    double rx = M_PI / 4.0;
    double ry = M_PI / 6.0;
    double rz = M_PI / 3.0;
    Transform original_transform = createTransform(translation, rx, ry, rz);

    Pose pose = transformToPose(original_transform);
    Transform recovered_transform = poseToTransform(pose);

    EXPECT_TRUE(original_transform.isApprox(recovered_transform));
}

// *********************************************************************************
//! \brief Test DH (Denavit-Hartenberg) transformation.
// *********************************************************************************
TEST_F(ConversionsTest, DHTransform)
{
    // Test with simple DH parameters
    double a = 1.0;
    double alpha = M_PI / 2.0;
    double d = 0.5;
    double theta = M_PI / 4.0;

    Transform dh_transform = dhTransform(a, alpha, d, theta);

    // DH transform should be a valid homogeneous transformation
    EXPECT_DOUBLE_EQ(dh_transform(3, 0), 0.0);
    EXPECT_DOUBLE_EQ(dh_transform(3, 1), 0.0);
    EXPECT_DOUBLE_EQ(dh_transform(3, 2), 0.0);
    EXPECT_DOUBLE_EQ(dh_transform(3, 3), 1.0);

    // Check that the rotation part is orthogonal (rotation matrix property)
    Eigen::Matrix3d rotation = dh_transform.block<3, 3>(0, 0);
    Eigen::Matrix3d should_be_identity = rotation * rotation.transpose();
    EXPECT_TRUE(should_be_identity.isApprox(Eigen::Matrix3d::Identity()));

    // Check determinant is 1 (proper rotation)
    EXPECT_NEAR(rotation.determinant(), 1.0, tolerance);

    // Test with zero parameters (should give identity)
    Transform identity_dh = dhTransform(0.0, 0.0, 0.0, 0.0);
    EXPECT_TRUE(identity_dh.isApprox(Transform::Identity()));
}

// *********************************************************************************
//! \brief Test DH transform with known values.
// *********************************************************************************
TEST_F(ConversionsTest, DHTransformKnownValues)
{
    // Test simple case: only theta rotation
    Transform theta_only = dhTransform(0.0, 0.0, 0.0, M_PI / 2);

    Eigen::Matrix3d expected_rotation;
    expected_rotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;

    EXPECT_TRUE((theta_only.block<3, 3>(0, 0).isApprox(expected_rotation)));
    EXPECT_TRUE((theta_only.block<3, 1>(0, 3).isApprox(Eigen::Vector3d::Zero(),
                                                       tolerance)));

    // Test simple case: only d translation
    Transform d_only = dhTransform(0.0, 0.0, 1.0, 0.0);

    EXPECT_TRUE((d_only.block<3, 3>(0, 0).isApprox(Eigen::Matrix3d::Identity(),
                                                   tolerance)));
    EXPECT_TRUE((d_only.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(0, 0, 1))));

    // Test simple case: only a translation
    Transform a_only = dhTransform(1.0, 0.0, 0.0, 0.0);

    EXPECT_TRUE((a_only.block<3, 3>(0, 0).isApprox(Eigen::Matrix3d::Identity(),
                                                   tolerance)));
    EXPECT_TRUE((a_only.block<3, 1>(0, 3).isApprox(Eigen::Vector3d(1, 0, 0))));
}

// *********************************************************************************
//! \brief Test edge cases.
// *********************************************************************************
TEST_F(ConversionsTest, EdgeCases)
{
    // Test with very small angles - use larger tolerance for numerical
    // precision
    Eigen::Matrix3d small_rotation = eulerToRotation(1e-10, 1e-10, 1e-10);
    EXPECT_TRUE(small_rotation.isApprox(Eigen::Matrix3d::Identity(), 1e-6));

    // Test with negative angles
    Eigen::Matrix3d neg_rotation =
        eulerToRotation(-M_PI / 4, -M_PI / 6, -M_PI / 3);
    EXPECT_GT(neg_rotation.determinant(),
              0.99); // Should still be a valid rotation

    // Test with large angles
    Eigen::Matrix3d large_rotation =
        eulerToRotation(2 * M_PI, 3 * M_PI, 4 * M_PI);
    // Large angles should be handled correctly (modulo 2π)
    EXPECT_GT(large_rotation.determinant(), 0.99);
}