/**
 * @file TestURDFLoader.cpp
 * @brief Unit tests for the URDF parser - Verification of URDF file loading and
 * parsing, robot structure creation and error handling from the folder data/.
 *
 * This test allows to verify the correctness of the URDF file given in the
 * data/ folder and prevent modifying them without updating the tests.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Common/Path.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

#include <cmath>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for URDFLoader class.
// *********************************************************************************
class URDFLoaderTest: public ::testing::Test
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

    std::vector<Geometry const*> getGeometries(Robot const& p_robot) const
    {
        std::vector<Geometry const*> geometry_nodes;
        p_robot.blueprint().root().traverse(
            [&geometry_nodes](const Node& node, size_t)
            {
                if (auto const* link = dynamic_cast<const Link*>(&node))
                {
                    const auto& geom = link->geometry();
                    geometry_nodes.push_back(&geom);
                }
            });
        return geometry_nodes;
    }

    std::unique_ptr<URDFLoader> parser;
};

// *********************************************************************************
//! \brief Test loading non-existent file.
// *********************************************************************************
TEST_F(URDFLoaderTest, NonExistentFile)
{
    std::string fake_path = "/fake/path/robot.urdf";

    Path path(project::info::paths::data);
    auto robot = parser->load(path.expand(fake_path));
    EXPECT_EQ(robot, nullptr);
    EXPECT_EQ(parser->error(),
              "Failed opening '" + fake_path +
                  "'. Reason was 'No such file or directory'");
}

// *********************************************************************************
//! \brief Test parsing simple revolute robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, SimpleRevoluteRobotSceneGraph)
{
    std::string robot_file_path = "simple_revolute_robot.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure
    EXPECT_EQ(robot->blueprint().numJoints(), 1);

    // Get the revolute joint
    auto const& revolute_joint = robot->blueprint().joint("revolute_joint");
    EXPECT_EQ(revolute_joint.type(), Joint::Type::REVOLUTE);

    // Test joint axis - should be Z-axis (0, 0, 1) for revolute joint
    auto axis = revolute_joint.axis();
    EXPECT_DOUBLE_EQ(axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(axis.z(), 1.0);

    // Test joint limits (-3.14159 to 3.14159)
    auto [min_limit, max_limit] = revolute_joint.limits();
    EXPECT_NEAR(min_limit, -3.14159, 1e-4);
    EXPECT_NEAR(max_limit, 3.14159, 1e-4);

    // Test geometry information
    auto geometries = getGeometries(*robot);
    EXPECT_EQ(geometries.size(), 2);

    EXPECT_EQ(geometries[0]->name(), "base_link_geometry");
    EXPECT_EQ(geometries[0]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[0]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[1], 0.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[2], 0.1);
    // meshPath may contain robot directory name or be empty
    EXPECT_EQ(geometries[0]->color, Eigen::Vector3f(0.5, 0.5, 0.5));

    EXPECT_EQ(geometries[1]->name(), "arm_link_geometry");
    EXPECT_EQ(geometries[1]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[1]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[0], 0.2);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[1], 0.05);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[2], 0.05);
    EXPECT_EQ(geometries[1]->color, Eigen::Vector3f(0.0, 0.0, 1.0));
}

// *********************************************************************************
//! \brief Test parsing simple prismatic robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, SimplePrismaticRobotSceneGraph)
{
    std::string robot_file_path = "simple_prismatic_robot.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure
    size_t num_joints = robot->blueprint().numJoints();
    EXPECT_EQ(num_joints, 1);

    // Get the prismatic joint
    auto const& prismatic_joint = robot->blueprint().joint("prismatic_joint");
    EXPECT_EQ(prismatic_joint.type(), Joint::Type::PRISMATIC);

    // Test joint axis - should be Z-axis (0, 0, 1) for prismatic joint
    auto axis = prismatic_joint.axis();
    EXPECT_DOUBLE_EQ(axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(axis.z(), 1.0);

    // Test joint limits (0 to 0.5)
    auto [min_limit, max_limit] = prismatic_joint.limits();
    EXPECT_DOUBLE_EQ(min_limit, 0.0);
    EXPECT_DOUBLE_EQ(max_limit, 0.5);

    // Test geometry information
    auto geometries = getGeometries(*robot);
    EXPECT_EQ(geometries.size(), 2);

    EXPECT_EQ(geometries[0]->name(), "base_link_geometry");
    EXPECT_EQ(geometries[0]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[0]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[0], 0.3);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[1], 0.3);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[2], 0.1);
    EXPECT_EQ(geometries[0]->color, Eigen::Vector3f(0.0, 1.0, 0.0));

    EXPECT_EQ(geometries[1]->name(), "end_effector_link_geometry");
    EXPECT_EQ(geometries[1]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[1]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[0], 0.15);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[1], 0.15);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[2], 0.2);
    EXPECT_EQ(geometries[1]->color, Eigen::Vector3f(0.0, 0.0, 1.0));
}

// *********************************************************************************
//! \brief Test parsing differential drive robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, DifferentialDriveRobotSceneGraph)
{
    std::string robot_file_path = "simple_diff_drive_robot.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure
    size_t num_joints = robot->blueprint().numJoints();
    EXPECT_GE(num_joints, 2);

    // Check joint types
    auto const& left_wheel_joint = robot->blueprint().joint("left_wheel_joint");
    EXPECT_EQ(left_wheel_joint.type(), Joint::Type::CONTINUOUS);
    auto left_axis = left_wheel_joint.axis();
    EXPECT_DOUBLE_EQ(left_axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(left_axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(left_axis.z(), 1.0);

    auto const& right_wheel_joint =
        robot->blueprint().joint("right_wheel_joint");
    EXPECT_EQ(right_wheel_joint.type(), Joint::Type::CONTINUOUS);
    auto right_axis = right_wheel_joint.axis();
    EXPECT_DOUBLE_EQ(right_axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(right_axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(right_axis.z(), 1.0);

    // Test geometry information
    auto geometries = getGeometries(*robot);
    EXPECT_EQ(geometries.size(), 4);

    EXPECT_EQ(geometries[0]->name(), "base_link_geometry");
    EXPECT_EQ(geometries[0]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[0]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[0], 0.5);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[1], 0.3);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[2], 0.1);
    EXPECT_EQ(geometries[0]->color, Eigen::Vector3f(0.6f, 0.6f, 0.6f));

    EXPECT_EQ(geometries[1]->name(), "left_wheel_geometry");
    EXPECT_EQ(geometries[1]->type(), Geometry::Type::CYLINDER);
    EXPECT_EQ(geometries[1]->parameters().size(), 2u);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[1], 0.05);
    EXPECT_EQ(geometries[1]->color, Eigen::Vector3f(1.0f, 1.0f, 1.0f));

    EXPECT_EQ(geometries[2]->name(), "right_wheel_geometry");
    EXPECT_EQ(geometries[2]->type(), Geometry::Type::CYLINDER);
    EXPECT_EQ(geometries[2]->parameters().size(), 2u);
    EXPECT_DOUBLE_EQ(geometries[2]->parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geometries[2]->parameters()[1], 0.05);
    EXPECT_EQ(geometries[2]->color, Eigen::Vector3f(1.0f, 1.0f, 1.0f));

    EXPECT_EQ(geometries[3]->name(), "caster_wheel_geometry");
    EXPECT_EQ(geometries[3]->type(), Geometry::Type::SPHERE);
    EXPECT_EQ(geometries[3]->parameters().size(), 1u);
    EXPECT_DOUBLE_EQ(geometries[3]->parameters()[0], 0.02);
    EXPECT_EQ(geometries[3]->color, Eigen::Vector3f(0.0f, 0.0f, 0.0f));
}

// *********************************************************************************
//! \brief Test parsing SCARA robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, SCARArobotSceneGraph)
{
    std::string robot_file_path = "scara_robot.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure - should have 3 joints
    size_t num_joints = robot->blueprint().numJoints();
    EXPECT_EQ(num_joints, 3);

    // Check specific joints exist
    auto const& joint1 = robot->blueprint().joint("joint1");
    auto const& joint2 = robot->blueprint().joint("joint2");
    auto const& end_effector = robot->blueprint().joint("end_effector");

    // Check joint types
    EXPECT_EQ(joint1.type(), Joint::Type::REVOLUTE);
    EXPECT_EQ(joint2.type(), Joint::Type::REVOLUTE);
    EXPECT_EQ(end_effector.type(), Joint::Type::PRISMATIC);

    // Check joint axes
    auto joint1_axis = joint1.axis();
    auto joint2_axis = joint2.axis();
    auto ee_axis = end_effector.axis();

    // Both revolute joints should rotate around Z-axis
    EXPECT_DOUBLE_EQ(joint1_axis.z(), 1.0);
    EXPECT_DOUBLE_EQ(joint2_axis.z(), 1.0);

    // End effector should translate along negative Z-axis
    EXPECT_DOUBLE_EQ(ee_axis.z(), -1.0);

    // Check joint limits
    auto joint1_limits = joint1.limits();
    auto joint2_limits = joint2.limits();
    auto ee_limits = end_effector.limits();

    EXPECT_NEAR(joint1_limits.first, -3.14, 1e-2);
    EXPECT_NEAR(joint1_limits.second, 3.14, 1e-2);
    EXPECT_NEAR(joint2_limits.first, -3.14, 1e-2);
    EXPECT_NEAR(joint2_limits.second, 3.14, 1e-2);

    EXPECT_NEAR(ee_limits.first, 0.038, 1e-3);
    EXPECT_NEAR(ee_limits.second, 0.16, 1e-3);
}

// *********************************************************************************
//! \brief Test parsing cartesian robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, CartesianRobotSceneGraph)
{
    std::string robot_file_path = "cartesian_robot.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure - should have 3 prismatic joints
    size_t num_joints = robot->blueprint().numJoints();
    EXPECT_EQ(num_joints, 3);

    // Check specific joints exist
    auto const& joint_x = robot->blueprint().joint("joint_x");
    auto const& joint_y = robot->blueprint().joint("joint_y");
    auto const& joint_z = robot->blueprint().joint("joint_z");

    // Check joint types - all should be prismatic
    EXPECT_EQ(joint_x.type(), Joint::Type::PRISMATIC);
    EXPECT_EQ(joint_y.type(), Joint::Type::PRISMATIC);
    EXPECT_EQ(joint_z.type(), Joint::Type::PRISMATIC);

    // Check joint axes
    auto joint_x_axis = joint_x.axis();
    auto joint_y_axis = joint_y.axis();
    auto joint_z_axis = joint_z.axis();

    // X joint should translate along X-axis
    EXPECT_DOUBLE_EQ(joint_x_axis.x(), 1.0);
    EXPECT_DOUBLE_EQ(joint_x_axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(joint_x_axis.z(), 0.0);

    // Y joint should translate along Y-axis
    EXPECT_DOUBLE_EQ(joint_y_axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(joint_y_axis.y(), 1.0);
    EXPECT_DOUBLE_EQ(joint_y_axis.z(), 0.0);

    // Z joint should translate along Z-axis
    EXPECT_DOUBLE_EQ(joint_z_axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(joint_z_axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(joint_z_axis.z(), 1.0);

    // Check joint limits
    auto [joint_x_min, joint_x_max] = joint_x.limits();
    auto [joint_y_min, joint_y_max] = joint_y.limits();
    auto [joint_z_min, joint_z_max] = joint_z.limits();

    EXPECT_DOUBLE_EQ(joint_x_min, -0.5);
    EXPECT_DOUBLE_EQ(joint_x_max, 0.5);
    EXPECT_DOUBLE_EQ(joint_y_min, -0.45);
    EXPECT_DOUBLE_EQ(joint_y_max, 0.45);
    EXPECT_DOUBLE_EQ(joint_z_min, -0.1);
    EXPECT_DOUBLE_EQ(joint_z_max, 0.0);

    // Test geometry information
    auto geometries = getGeometries(*robot);
    EXPECT_EQ(geometries.size(), 4);

    // Check frame2 geometry (grey box)
    EXPECT_EQ(geometries[0]->name(), "frame2_geometry");
    EXPECT_EQ(geometries[0]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[0]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[0], 1.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[1], 0.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[2], 0.1);
    EXPECT_EQ(geometries[0]->color, Eigen::Vector3f(0.5f, 0.5f, 0.5f));

    // Check link_x geometry (red box)
    EXPECT_EQ(geometries[1]->name(), "link_x_geometry");
    EXPECT_EQ(geometries[1]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[1]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[1], 1.0);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[2], 0.1);
    EXPECT_EQ(geometries[1]->color, Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    // Check link_y geometry (green box)
    EXPECT_EQ(geometries[2]->name(), "link_y_geometry");
    EXPECT_EQ(geometries[2]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[2]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[2]->parameters()[0], 0.15);
    EXPECT_DOUBLE_EQ(geometries[2]->parameters()[1], 0.15);
    EXPECT_DOUBLE_EQ(geometries[2]->parameters()[2], 0.15);
    EXPECT_EQ(geometries[2]->color, Eigen::Vector3f(0.0f, 1.0f, 0.0f));

    // Check link_z geometry (blue cylinder)
    EXPECT_EQ(geometries[3]->name(), "link_z_geometry");
    EXPECT_EQ(geometries[3]->type(), Geometry::Type::CYLINDER);
    EXPECT_EQ(geometries[3]->parameters().size(), 2u);
    EXPECT_DOUBLE_EQ(geometries[3]->parameters()[0], 0.04);
    EXPECT_DOUBLE_EQ(geometries[3]->parameters()[1], 0.3);
    EXPECT_EQ(geometries[3]->color, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
}

// *********************************************************************************
//! \brief Test parsing simple revolute robot with inertia URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, SimpleRevoluteRobotWithInertiaSceneGraph)
{
    std::string robot_file_path = "simple_revolute_robot_with_inertia.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure
    EXPECT_EQ(robot->blueprint().numJoints(), 1);

    // Get the revolute joint
    auto const& revolute_joint = robot->blueprint().joint("revolute_joint");
    EXPECT_EQ(revolute_joint.type(), Joint::Type::REVOLUTE);

    // Test joint axis - should be Z-axis (0, 0, 1) for revolute joint
    auto axis = revolute_joint.axis();
    EXPECT_DOUBLE_EQ(axis.x(), 0.0);
    EXPECT_DOUBLE_EQ(axis.y(), 0.0);
    EXPECT_DOUBLE_EQ(axis.z(), 1.0);

    // Test joint limits (-3.14159 to 3.14159)
    auto [min_limit, max_limit] = revolute_joint.limits();
    EXPECT_NEAR(min_limit, -3.14159, 1e-4);
    EXPECT_NEAR(max_limit, 3.14159, 1e-4);

    // Test joint velocity limit
    EXPECT_DOUBLE_EQ(revolute_joint.maxVelocity(), 100.0);

    // Test joint effort limit
    EXPECT_DOUBLE_EQ(revolute_joint.effort_max(), 1000.0);

    // Test geometry information
    auto geometries = getGeometries(*robot);
    EXPECT_EQ(geometries.size(), 2);

    EXPECT_EQ(geometries[0]->name(), "base_link_geometry");
    EXPECT_EQ(geometries[0]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[0]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[1], 0.1);
    EXPECT_DOUBLE_EQ(geometries[0]->parameters()[2], 0.1);
    EXPECT_EQ(geometries[0]->color, Eigen::Vector3f(0.5, 0.5, 0.5));

    EXPECT_EQ(geometries[1]->name(), "arm_link_geometry");
    EXPECT_EQ(geometries[1]->type(), Geometry::Type::BOX);
    EXPECT_EQ(geometries[1]->parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[0], 0.2);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[1], 0.05);
    EXPECT_DOUBLE_EQ(geometries[1]->parameters()[2], 0.05);
    EXPECT_EQ(geometries[1]->color, Eigen::Vector3f(0.0, 0.0, 1.0));

    // Test inertial properties for base_link
    Link const& base_link = robot->blueprint().link("base_link");
    EXPECT_DOUBLE_EQ(base_link.mass(), 1.0);
    Eigen::Vector4d base_com = base_link.centerOfMass();
    EXPECT_NEAR(base_com.x(), 0.0, 1e-6);
    EXPECT_NEAR(base_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(base_com.z(), 0.0, 1e-6);
    Inertial const& base_inertial = base_link.inertia();
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(0, 0), 0.01); // ixx
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(1, 1), 0.01); // iyy
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(2, 2), 0.01); // izz

    // Test inertial properties for arm_link
    Link const& arm_link = robot->blueprint().link("arm_link");
    EXPECT_DOUBLE_EQ(arm_link.mass(), 0.5);
    Eigen::Vector4d arm_com = arm_link.centerOfMass();
    EXPECT_NEAR(arm_com.x(), 0.1, 1e-6);
    EXPECT_NEAR(arm_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(arm_com.z(), 0.0, 1e-6);
    Inertial const& arm_inertial = arm_link.inertia();
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(0, 0), 0.001); // ixx
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(1, 1), 0.001); // iyy
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(2, 2), 0.001); // izz
}

// *********************************************************************************
//! \brief Test parsing 6-axis robot URDF kinematic tree.
// *********************************************************************************
TEST_F(URDFLoaderTest, Robot6AxisSceneGraph)
{
    std::string robot_file_path = "robot_6axis.urdf";
    auto robot = parseRobot(robot_file_path);
    ASSERT_NE(robot, nullptr) << "Failed to load " << robot_file_path;

    // Check robot has root node
    EXPECT_TRUE(robot->blueprint().hasRoot());

    // Check joint structure - should have 6 joints
    size_t num_joints = robot->blueprint().numJoints();
    EXPECT_EQ(num_joints, 6);

    // Check link structure - should have 7 links (base_link + 6 links)
    EXPECT_EQ(robot->blueprint().numLinks(), 7);

    // Test joint1 limits, velocity, effort
    auto const& joint1 = robot->blueprint().joint("joint1");
    EXPECT_EQ(joint1.type(), Joint::Type::REVOLUTE);
    auto [j1_min, j1_max] = joint1.limits();
    EXPECT_NEAR(j1_min, -3.14159, 1e-4);
    EXPECT_NEAR(j1_max, 3.14159, 1e-4);
    EXPECT_DOUBLE_EQ(joint1.maxVelocity(), 2.0);
    EXPECT_DOUBLE_EQ(joint1.effort_max(), 150.0);

    // Test joint2 limits, velocity, effort
    auto const& joint2 = robot->blueprint().joint("joint2");
    EXPECT_EQ(joint2.type(), Joint::Type::REVOLUTE);
    auto [j2_min, j2_max] = joint2.limits();
    EXPECT_NEAR(j2_min, -1.57079, 1e-4);
    EXPECT_NEAR(j2_max, 1.57079, 1e-4);
    EXPECT_DOUBLE_EQ(joint2.maxVelocity(), 1.8);
    EXPECT_DOUBLE_EQ(joint2.effort_max(), 120.0);

    // Test joint3 limits, velocity, effort
    auto const& joint3 = robot->blueprint().joint("joint3");
    EXPECT_EQ(joint3.type(), Joint::Type::REVOLUTE);
    auto [j3_min, j3_max] = joint3.limits();
    EXPECT_NEAR(j3_min, -2.35619, 1e-4);
    EXPECT_NEAR(j3_max, 2.35619, 1e-4);
    EXPECT_DOUBLE_EQ(joint3.maxVelocity(), 2.2);
    EXPECT_DOUBLE_EQ(joint3.effort_max(), 100.0);

    // Test joint4 limits, velocity, effort
    auto const& joint4 = robot->blueprint().joint("joint4");
    EXPECT_EQ(joint4.type(), Joint::Type::REVOLUTE);
    auto [j4_min, j4_max] = joint4.limits();
    EXPECT_NEAR(j4_min, -3.14159, 1e-4);
    EXPECT_NEAR(j4_max, 3.14159, 1e-4);
    EXPECT_DOUBLE_EQ(joint4.maxVelocity(), 2.5);
    EXPECT_DOUBLE_EQ(joint4.effort_max(), 50.0);

    // Test joint5 limits, velocity, effort
    auto const& joint5 = robot->blueprint().joint("joint5");
    EXPECT_EQ(joint5.type(), Joint::Type::REVOLUTE);
    auto [j5_min, j5_max] = joint5.limits();
    EXPECT_NEAR(j5_min, -1.74533, 1e-4);
    EXPECT_NEAR(j5_max, 1.74533, 1e-4);
    EXPECT_DOUBLE_EQ(joint5.maxVelocity(), 2.8);
    EXPECT_DOUBLE_EQ(joint5.effort_max(), 40.0);

    // Test joint6 limits, velocity, effort
    auto const& joint6 = robot->blueprint().joint("joint6");
    EXPECT_EQ(joint6.type(), Joint::Type::REVOLUTE);
    auto [j6_min, j6_max] = joint6.limits();
    EXPECT_NEAR(j6_min, -3.14159, 1e-4);
    EXPECT_NEAR(j6_max, 3.14159, 1e-4);
    EXPECT_DOUBLE_EQ(joint6.maxVelocity(), 3.0);
    EXPECT_DOUBLE_EQ(joint6.effort_max(), 30.0);

    // Test inertial properties for base_link
    Link const& base_link = robot->blueprint().link("base_link");
    EXPECT_DOUBLE_EQ(base_link.mass(), 5.0);
    Eigen::Vector4d base_com = base_link.centerOfMass();
    EXPECT_NEAR(base_com.x(), 0.0, 1e-6);
    EXPECT_NEAR(base_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(base_com.z(), 0.05, 1e-6);
    Inertial const& base_inertial = base_link.inertia();
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(0, 0), 0.05); // ixx
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(1, 1), 0.05); // iyy
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(2, 2), 0.08); // izz

    // Test inertial properties for link1
    Link const& link1 = robot->blueprint().link("link1");
    EXPECT_DOUBLE_EQ(link1.mass(), 3.0);
    Eigen::Vector4d link1_com = link1.centerOfMass();
    EXPECT_NEAR(link1_com.x(), 0.0, 1e-6);
    EXPECT_NEAR(link1_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(link1_com.z(), 0.15, 1e-6);
    Inertial const& link1_inertial = link1.inertia();
    EXPECT_DOUBLE_EQ(link1_inertial.inertia_matrix(0, 0), 0.025); // ixx
    EXPECT_DOUBLE_EQ(link1_inertial.inertia_matrix(1, 1), 0.025); // iyy
    EXPECT_DOUBLE_EQ(link1_inertial.inertia_matrix(2, 2), 0.01);  // izz

    // Test inertial properties for link2
    Link const& link2 = robot->blueprint().link("link2");
    EXPECT_DOUBLE_EQ(link2.mass(), 2.5);
    Eigen::Vector4d link2_com = link2.centerOfMass();
    EXPECT_NEAR(link2_com.x(), 0.0, 1e-6);
    EXPECT_NEAR(link2_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(link2_com.z(), 0.175, 1e-6);

    // Test inertial properties for link6 (end effector)
    Link const& link6 = robot->blueprint().link("link6");
    EXPECT_DOUBLE_EQ(link6.mass(), 0.5);
    Eigen::Vector4d link6_com = link6.centerOfMass();
    EXPECT_NEAR(link6_com.x(), 0.0, 1e-6);
    EXPECT_NEAR(link6_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(link6_com.z(), 0.03, 1e-6);
    Inertial const& link6_inertial = link6.inertia();
    EXPECT_DOUBLE_EQ(link6_inertial.inertia_matrix(0, 0), 0.001);  // ixx
    EXPECT_DOUBLE_EQ(link6_inertial.inertia_matrix(1, 1), 0.001);  // iyy
    EXPECT_DOUBLE_EQ(link6_inertial.inertia_matrix(2, 2), 0.0005); // izz
}