/**
 * @file TestInverseKinematics.cpp
 * @brief Unit tests for Inverse Kinematics - Verification of IK solving using
 * JacobianIKSolver with poses generated from forward kinematics.
 *
 * This test follows the approach from robot_kinematics.py:
 * 1. Generate 3 reachable poses using forward kinematics with different joint
 *    configurations (neutral, 1/3, 2/3)
 * 2. Test IK solver to reach these poses
 * 3. Verify convergence and accuracy
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Common/Conversions.hpp"
#include "Robotik/Core/Common/Path.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"
#include "Robotik/Core/Robot/Robot.hpp"
#include "Robotik/Core/Solvers/IKSolver.hpp"

#include <cmath>
#include <vector>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Inverse Kinematics tests.
// *********************************************************************************
class InverseKinematicsTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        parser = std::make_unique<URDFLoader>();
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

    // Helper function to compute three joint configurations and corresponding
    // poses Based on robot_kinematics.py compute_three_poses()
    std::pair<std::vector<JointValues>, std::vector<Pose>>
    computeThreePoses(Robot& robot, Node const& end_effector)
    {
        std::vector<JointValues> joint_configurations;
        std::vector<Pose> end_effector_poses;

        // Get joint limits
        std::vector<std::pair<double, double>> limits;
        robot.blueprint().forEachJoint(
            [&limits](Joint const& joint, size_t)
            {
                auto [min, max] = joint.limits();
                limits.emplace_back(min, max);
            });

        // Configuration 0: center (neutral) - middle of limits
        JointValues q0;
        for (auto const& [min_val, max_val] : limits)
        {
            q0.push_back((max_val + min_val) / 2.0);
        }
        setJointValuesAndUpdateFK(robot, q0);
        Transform tf0 = end_effector.worldTransform();
        end_effector_poses.push_back(transformToPose(tf0));
        joint_configurations.push_back(q0);

        // Configuration 1: 1/3 from min
        JointValues q1;
        for (auto const& [min_val, max_val] : limits)
        {
            q1.push_back(min_val + (max_val - min_val) * 1.0 / 3.0);
        }
        setJointValuesAndUpdateFK(robot, q1);
        Transform tf1 = end_effector.worldTransform();
        end_effector_poses.push_back(transformToPose(tf1));
        joint_configurations.push_back(q1);

        // Configuration 2: 2/3 from min
        JointValues q2;
        for (auto const& [min_val, max_val] : limits)
        {
            q2.push_back(min_val + (max_val - min_val) * 2.0 / 3.0);
        }
        setJointValuesAndUpdateFK(robot, q2);
        Transform tf2 = end_effector.worldTransform();
        end_effector_poses.push_back(transformToPose(tf2));
        joint_configurations.push_back(q2);

        return { joint_configurations, end_effector_poses };
    }

    // Helper to set joint values and update forward kinematics
    void setJointValuesAndUpdateFK(Robot& robot, JointValues const& q)
    {
        robot.blueprint().forEachJoint(
            [&q](Joint& joint, size_t index)
            {
                if (index < q.size())
                {
                    joint.position(q[index]);
                }
            });
        robot.forwardKinematics();
    }

    // Helper to verify IK solution by checking forward kinematics result
    bool verifyIKSolution(Robot& robot,
                          Node const& end_effector,
                          Pose const& target_pose,
                          double position_tolerance = 1e-3,
                          double orientation_tolerance = 1e-2)
    {
        robot.forwardKinematics();
        Transform current_transform = end_effector.worldTransform();
        Pose current_pose = transformToPose(current_transform);

        // Check position error
        Eigen::Vector3d pos_error =
            target_pose.head<3>() - current_pose.head<3>();
        if (pos_error.norm() > position_tolerance)
        {
            return false;
        }

        // Check orientation error (convert to rotation matrices)
        Eigen::Matrix3d R_target =
            eulerToRotation(target_pose(3), target_pose(4), target_pose(5));
        Eigen::Matrix3d R_current =
            eulerToRotation(current_pose(3), current_pose(4), current_pose(5));

        Eigen::Matrix3d R_error = R_target * R_current.transpose();
        Eigen::AngleAxisd angle_axis(R_error);
        double orientation_error = angle_axis.angle();

        return orientation_error <= orientation_tolerance;
    }

    std::unique_ptr<URDFLoader> parser;
};

// *********************************************************************************
//! \brief Test inverse kinematics for simple revolute robot (1 DOF).
// *********************************************************************************
TEST_F(InverseKinematicsTest, SimpleRevoluteRobotIK)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    // Find end effector (arm_link is the end effector)
    auto* end_effector = Node::find(robot->blueprint().root(), "arm_link");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Create IK solver
    auto ik_solver = createDefaultIKSolver();

    // Compute three poses using forward kinematics
    auto [_, target_poses] = computeThreePoses(*robot, *end_effector);

    // Test IK for each pose
    for (size_t i = 0; i < target_poses.size(); ++i)
    {
        Pose const& target_pose = target_poses[i];

        // Set robot to neutral position before solving
        robot->setNeutralPosition();

        // Solve IK
        bool solved = ik_solver->solve(*robot, *end_effector, target_pose);

        std::cout << "IK solver: it: " << ik_solver->numIterations()
                  << " error: " << ik_solver->poseError() << std::endl;

        // Verify convergence
        EXPECT_TRUE(solved) << "IK failed to converge for pose " << i;
        EXPECT_TRUE(ik_solver->converged())
            << "IK solver did not converge for pose " << i;
        EXPECT_LT(ik_solver->poseError(), 1e-3)
            << "IK pose error too large for pose " << i;

        // Verify solution by checking forward kinematics
        bool fk_matches = verifyIKSolution(*robot, *end_effector, target_pose);
        EXPECT_TRUE(fk_matches)
            << "Forward kinematics verification failed for pose " << i;

        // Check that joint values are reasonable (within limits)
        robot->blueprint().forEachJoint(
            [](Joint const& joint, size_t index)
            {
                auto [min, max] = joint.limits();
                EXPECT_GE(joint.position(), min)
                    << "Joint " << index << " below lower limit";
                EXPECT_LE(joint.position(), max)
                    << "Joint " << index << " above upper limit";
            });
    }
}

// *********************************************************************************
//! \brief Test inverse kinematics for SCARA robot (3 DOF).
// *********************************************************************************
TEST_F(InverseKinematicsTest, SCARARobotIK)
{
    auto robot = parseRobot("scara_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load scara_robot.urdf";

    // Find end effector
    auto* end_effector =
        Node::find(robot->blueprint().root(), "link_end_effector");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Create IK solver with custom config for better convergence
    JacobianIKSolver::Config config;
    config.max_iterations = 1000;
    config.tolerance = 1e-4;
    config.damping = 0.01;
    auto ik_solver = createJacobianIKSolver(config);

    // Compute three poses using forward kinematics
    auto [_, target_poses] = computeThreePoses(*robot, *end_effector);

    // Test IK for each pose
    for (size_t i = 0; i < target_poses.size(); ++i)
    {
        Pose const& target_pose = target_poses[i];

        // Set robot to neutral position before solving
        robot->setNeutralPosition();

        // Solve IK
        bool solved = ik_solver->solve(*robot, *end_effector, target_pose);

        std::cout << "IK solver: it: " << ik_solver->numIterations()
                  << " error: " << ik_solver->poseError() << std::endl;

        // Verify convergence
        EXPECT_TRUE(solved) << "IK failed to converge for SCARA pose " << i;
        if (solved)
        {
            EXPECT_TRUE(ik_solver->converged());
            EXPECT_LT(ik_solver->poseError(), 1e-3);

            // Verify solution by checking forward kinematics
            bool fk_matches = verifyIKSolution(
                *robot, *end_effector, target_pose, 1e-3, 1e-2);
            EXPECT_TRUE(fk_matches)
                << "Forward kinematics verification failed for SCARA pose "
                << i;
        }
    }
}

// *********************************************************************************
//! \brief Test inverse kinematics for cartesian robot (3 DOF prismatic).
// *********************************************************************************
TEST_F(InverseKinematicsTest, CartesianRobotIK)
{
    auto robot = parseRobot("cartesian_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load cartesian_robot.urdf";

    // Find end effector (link_z is the end effector)
    auto* end_effector = Node::find(robot->blueprint().root(), "link_z");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    // Create IK solver
    auto ik_solver = createDefaultIKSolver();

    // Compute three poses using forward kinematics
    auto [_, target_poses] = computeThreePoses(*robot, *end_effector);

    // Test IK for each pose (prismatic joints should be easier to solve)
    for (size_t i = 0; i < target_poses.size(); ++i)
    {
        Pose const& target_pose = target_poses[i];

        // Set robot to neutral position before solving
        robot->setNeutralPosition();

        // Solve IK
        bool solved = ik_solver->solve(*robot, *end_effector, target_pose);

        // Verify convergence
        EXPECT_TRUE(solved) << "IK failed to converge for cartesian pose " << i;
        EXPECT_TRUE(ik_solver->converged());
        EXPECT_LT(ik_solver->poseError(), 1e-3);

        // Verify solution by checking forward kinematics
        // For prismatic joints, position accuracy should be very high
        bool fk_matches =
            verifyIKSolution(*robot, *end_effector, target_pose, 1e-4, 1e-3);
        EXPECT_TRUE(fk_matches)
            << "Forward kinematics verification failed for cartesian pose "
            << i;
    }
}

// *********************************************************************************
//! \brief Test IK with unreachable pose.
// *********************************************************************************
TEST_F(InverseKinematicsTest, UnreachablePose)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    auto* end_effector = Node::find(robot->blueprint().root(), "arm_link");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    auto ik_solver = createDefaultIKSolver();

    // Create an unreachable pose (very far away)
    Pose unreachable_pose;
    unreachable_pose << 100.0, 100.0, 100.0, 0.0, 0.0, 0.0;

    robot->setNeutralPosition();

    // Solve IK - should either fail or return a pose far from target
    bool solved = ik_solver->solve(*robot, *end_effector, unreachable_pose);

    std::cout << "IK solver: it: " << ik_solver->numIterations()
              << " error: " << ik_solver->poseError() << std::endl;

    // Either IK should not converge, or the error should be very large
    if (solved)
    {
        // If it "converged", the error should still be very large
        EXPECT_GT(ik_solver->poseError(), 1.0)
            << "Unreachable pose should have large error";
    }
    else
    {
        // If it didn't converge, that's also acceptable
        EXPECT_FALSE(ik_solver->converged());
    }
}

// *********************************************************************************
//! \brief Test IK solver respects joint limits.
// *********************************************************************************
TEST_F(InverseKinematicsTest, JointLimitsCompliance)
{
    auto robot = parseRobot("scara_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load scara_robot.urdf";

    auto* end_effector =
        Node::find(robot->blueprint().root(), "link_end_effector");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    auto ik_solver = createDefaultIKSolver();

    // Compute poses
    auto [_, target_poses] = computeThreePoses(*robot, *end_effector);

    // Test with first pose
    Pose const& target_pose = target_poses[0];

    robot->setNeutralPosition();
    bool solved = ik_solver->solve(*robot, *end_effector, target_pose);

    if (solved)
    {
        // Verify all joints are within limits
        robot->blueprint().forEachJoint(
            [](Joint const& joint, size_t)
            {
                auto [min, max] = joint.limits();
                EXPECT_GE(joint.position(), min)
                    << "Joint " << joint.name() << " below limit";
                EXPECT_LE(joint.position(), max)
                    << "Joint " << joint.name() << " above limit";
            });
    }
}

// *********************************************************************************
//! \brief Test IK with different initial configurations.
// *********************************************************************************
TEST_F(InverseKinematicsTest, DifferentInitialConfigurations)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    auto* end_effector = Node::find(robot->blueprint().root(), "arm_link");
    ASSERT_NE(end_effector, nullptr) << "End effector node not found";

    auto ik_solver = createDefaultIKSolver();

    // Generate a target pose
    robot->setNeutralPosition();
    Transform target_transform = end_effector->worldTransform();
    Pose target_pose = transformToPose(target_transform);

    // Try different initial configurations
    std::vector<double> initial_positions = {
        -M_PI / 2.0, 0.0, M_PI / 4.0, M_PI / 2.0
    };

    for (double initial_pos : initial_positions)
    {
        // Set initial configuration
        robot->blueprint().joint("revolute_joint").position(initial_pos);
        robot->forwardKinematics();

        // Solve IK
        bool solved = ik_solver->solve(*robot, *end_effector, target_pose);

        if (solved)
        {
            // Verify solution
            bool fk_matches =
                verifyIKSolution(*robot, *end_effector, target_pose);
            EXPECT_TRUE(fk_matches)
                << "IK solution verification failed for initial position "
                << initial_pos;
        }
    }
}
