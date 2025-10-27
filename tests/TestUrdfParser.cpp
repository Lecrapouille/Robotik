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

#include "Robotik/Core/Path.hpp"
#include "Robotik/Core/Robot.hpp"
#include "Robotik/Core/UrdfLoader.hpp"

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
//! \brief Test parsing simple revolute robot URDF scene graph.
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
//! \brief Test parsing simple prismatic robot URDF scene graph.
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
//! \brief Test parsing differential drive robot URDF scene graph.
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
//! \brief Test parsing SCARA robot URDF scene graph.
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
//! \brief Test parsing cartesian robot URDF scene graph.
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