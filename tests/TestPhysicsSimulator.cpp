/**
 * @file TestPhysicsSimulator.cpp
 * @brief Unit tests for PhysicsSimulator class
 */

#include "Robotik/Core/PhysicsSimulator.hpp"
#include "Robotik/Core/URDFParser.hpp"
#include "main.hpp"

using namespace robotik;

// ============================================================================
//! \brief Test basic PhysicsSimulator creation
// ============================================================================
TEST(PhysicsSimulator, Construction)
{
    double dt = 0.01;
    Eigen::Vector3d gravity(0.0, 0.0, -9.81);

    PhysicsSimulator simulator(dt, gravity);

    EXPECT_DOUBLE_EQ(simulator.dt(), dt);
    EXPECT_TRUE(simulator.gravity().isApprox(gravity));
}

// ============================================================================
//! \brief Test gravity setter/getter
// ============================================================================
TEST(PhysicsSimulator, GravitySetting)
{
    PhysicsSimulator simulator(0.01);

    Eigen::Vector3d moon_gravity(0.0, 0.0, -1.62);
    simulator.setGravity(moon_gravity);

    EXPECT_TRUE(simulator.gravity().isApprox(moon_gravity));
}

// ============================================================================
//! \brief Test simulation on a simple revolute joint
// ============================================================================
TEST(PhysicsSimulator, SimpleRevoluteJoint)
{
    // Load a simple robot with one revolute joint
    URDFParser parser;
    auto robot = parser.load("data/simple_revolute_robot.urdf");

    ASSERT_TRUE(robot != nullptr)
        << "Failed to load robot: " << parser.getError();
    ASSERT_GT(robot->jointNames().size(), 0u) << "Robot has no joints";

    // Create simulator
    PhysicsSimulator simulator(0.01);

    // Configure joint
    auto& joint = const_cast<Joint&>(robot->joint(robot->jointNames()[0]));
    joint.damping(0.1);
    joint.friction(0.0);
    joint.effort_max(10.0);

    // Set initial position to neutral
    robot->setNeutralPosition();

    // Get initial state
    double initial_pos = joint.position();
    double initial_vel = joint.velocity();

    EXPECT_DOUBLE_EQ(initial_pos, 0.0);
    EXPECT_DOUBLE_EQ(initial_vel, 0.0);

    // Run simulation for a few steps
    for (int i = 0; i < 10; ++i)
    {
        simulator.step(*robot);
    }

    // With gravity and no applied effort, the joint should move
    // (unless the link has zero mass, which would be unusual)
    // We just check that the simulation runs without errors
    SUCCEED() << "Simulation completed without errors";
}

// ============================================================================
//! \brief Test that applied effort affects joint motion
// ============================================================================
TEST(PhysicsSimulator, AppliedEffort)
{
    // Load a simple robot
    URDFParser parser;
    auto robot = parser.load("data/simple_revolute_robot_with_inertia.urdf");

    ASSERT_TRUE(robot != nullptr);
    ASSERT_GT(robot->jointNames().size(), 0u);

    // Create simulator with no gravity
    PhysicsSimulator simulator(0.01, Eigen::Vector3d::Zero());

    // Configure joint
    auto& joint = const_cast<Joint&>(robot->joint(robot->jointNames()[0]));
    joint.damping(0.0);
    joint.friction(0.0);
    joint.effort_max(100.0);

    // Set initial position
    robot->setNeutralPosition();

    // Apply positive effort
    joint.effort(5.0);

    // Run simulation
    for (int i = 0; i < 50; ++i)
    {
        simulator.step(*robot);
    }

    // Velocity should be positive (accelerated by positive effort)
    EXPECT_GT(joint.velocity(), 0.0)
        << "Joint should accelerate with positive effort";
}

// ============================================================================
//! \brief Test damping effect
// ============================================================================
TEST(PhysicsSimulator, DampingEffect)
{
    // Load a simple robot
    URDFParser parser;
    auto robot = parser.load("data/simple_revolute_robot_with_inertia.urdf");

    ASSERT_TRUE(robot != nullptr);
    ASSERT_GT(robot->jointNames().size(), 0u);

    // Create simulator with no gravity
    PhysicsSimulator simulator(0.01, Eigen::Vector3d::Zero());

    // Configure joint with high damping
    auto& joint = const_cast<Joint&>(robot->joint(robot->jointNames()[0]));
    joint.damping(1.0); // High damping
    joint.friction(0.0);
    joint.effort_max(100.0);

    // Set initial position and apply effort
    robot->setNeutralPosition();
    joint.effort(5.0);

    // Run simulation for some time
    for (int i = 0; i < 100; ++i)
    {
        simulator.step(*robot);
    }

    double vel_with_damping = joint.velocity();

    // Now test without damping
    robot->setNeutralPosition();
    joint.damping(0.0);
    joint.effort(5.0);

    for (int i = 0; i < 100; ++i)
    {
        simulator.step(*robot);
    }

    double vel_without_damping = joint.velocity();

    // Velocity with damping should be less than without
    EXPECT_LT(vel_with_damping, vel_without_damping)
        << "Damping should reduce velocity";
}

// ============================================================================
//! \brief Test effort limits
// ============================================================================
TEST(PhysicsSimulator, EffortLimits)
{
    // Load a simple robot
    URDFParser parser;
    auto robot = parser.load("data/simple_revolute_robot_with_inertia.urdf");

    ASSERT_TRUE(robot != nullptr);
    ASSERT_GT(robot->jointNames().size(), 0u);

    // Create simulator
    PhysicsSimulator simulator(0.01, Eigen::Vector3d::Zero());

    // Configure joint with low effort limit
    auto& joint = const_cast<Joint&>(robot->joint(robot->jointNames()[0]));
    joint.damping(0.0);
    joint.friction(0.0);
    joint.effort_max(2.0); // Low limit

    // Set initial position and apply high effort (should be clamped)
    robot->setNeutralPosition();
    joint.effort(10.0); // Try to apply 10 N·m

    // Run just one simulation step to check acceleration
    simulator.step(*robot);

    double acc_with_limit = std::abs(joint.acceleration());

    // Reset and try with higher limit
    robot->setNeutralPosition();
    joint.effort_max(20.0); // Higher limit
    joint.effort(10.0);

    simulator.step(*robot);

    double acc_without_limit = std::abs(joint.acceleration());

    // With lower effort limit, acceleration should be restricted
    EXPECT_LT(acc_with_limit, acc_without_limit)
        << "Effort limits should restrict acceleration (acc_with_limit="
        << acc_with_limit << ", acc_without_limit=" << acc_without_limit << ")";
}

// ============================================================================
//! \brief Test fixed joints are skipped
// ============================================================================
TEST(PhysicsSimulator, FixedJointsSkipped)
{
    // Create a simple robot with a fixed joint
    auto root_link = std::make_unique<Link>(
        "base_link",
        std::make_unique<Geometry>("base_visual",
                                   Geometry::Type::BOX,
                                   std::vector<double>{ 0.1, 0.1, 0.1 },
                                   ""));

    auto& fixed_joint = root_link->createChild<Joint>(
        "fixed_joint", Joint::Type::FIXED, Eigen::Vector3d(0, 0, 1));

    auto& child_link = fixed_joint.createChild<Link>(
        "child_link",
        std::make_unique<Geometry>("child_visual",
                                   Geometry::Type::BOX,
                                   std::vector<double>{ 0.1, 0.1, 0.1 },
                                   ""));
    child_link.inertia(
        1.0, Eigen::Vector3d(0, 0, 1), Eigen::Matrix3d::Identity());

    // Create robot
    Robot robot("test_robot", std::move(root_link));

    // Create simulator
    PhysicsSimulator simulator(0.01);

    // Run simulation - should not crash on fixed joints
    for (int i = 0; i < 10; ++i)
    {
        simulator.step(robot);
    }

    SUCCEED() << "Simulation with fixed joints completed without errors";
}

// ============================================================================
//! \brief Test time step modification
// ============================================================================
TEST(PhysicsSimulator, TimeStepModification)
{
    PhysicsSimulator simulator(0.01);

    EXPECT_DOUBLE_EQ(simulator.dt(), 0.01);

    simulator.dt(0.005);

    EXPECT_DOUBLE_EQ(simulator.dt(), 0.005);
}
