#include "main.hpp"

#include "Robotik/Parser.hpp"

#include <cmath>
#include <fstream>
#include <gtest/gtest.h>

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for URDFParser class.
// *********************************************************************************
class URDFParserTest: public ::testing::Test
{
protected:

    void SetUp() override
    {
        parser = std::make_unique<URDFParser>();
        urdf_file_path = "/home/qq/MyGitHub/Robotik/data/scara_robot.urdf";
    }

    bool createSimpleURDF(std::string& p_temp_file)
    {
        // Create a simple URDF string for testing
        std::string simple_urdf = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>)";

        std::ofstream file(p_temp_file);
        file << simple_urdf;
        file.close();
        return true;
    }

    std::unique_ptr<URDFParser> parser;
    std::string urdf_file_path;
};

// *********************************************************************************
//! \brief Test loading non-existent file.
// *********************************************************************************
TEST_F(URDFParserTest, NonExistentFile)
{
    std::string fake_path = "/fake/path/robot.urdf";

    auto robot = parser->load(fake_path);
    EXPECT_EQ(robot, nullptr);
    EXPECT_EQ(parser->getError(),
              "Failed opening '" + fake_path +
                  "'. Reason was 'No such file or directory'");
}

// *********************************************************************************
//! \brief Test parsing a simple URDF file.
// *********************************************************************************
TEST_F(URDFParserTest, ParseSimpleURDF)
{
    std::string temp_file = "/tmp/test_simple.urdf";
    ASSERT_TRUE(createSimpleURDF(temp_file));
    std::ifstream file(temp_file);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << temp_file;
    }

    EXPECT_NO_THROW({
        auto robot = parser->load(temp_file);
        EXPECT_NE(robot, nullptr);
    });

    // Clean up
    std::remove(temp_file.c_str());
}

// *********************************************************************************
//! \brief Test loading URDF from file.
// *********************************************************************************
TEST_F(URDFParserTest, LoadURDFFromFile)
{
    // Skip test if file doesn't exist
    std::ifstream file(urdf_file_path);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << urdf_file_path;
    }
    file.close();

    EXPECT_NO_THROW({
        auto robot = parser->load(urdf_file_path);
        EXPECT_NE(robot, nullptr);
    });
}

// *********************************************************************************
//! \brief Test parsing simple robot URDF.
// *********************************************************************************
TEST_F(URDFParserTest, ParseSimpleRobot)
{
    std::string temp_file = "/tmp/test_simple.urdf";
    createSimpleURDF(temp_file);
    std::ifstream file(temp_file);

    auto robot = parser->load(temp_file);
    ASSERT_NE(robot, nullptr);

    auto joint_names = robot->jointNames();
    EXPECT_EQ(joint_names.size(), 1);
    EXPECT_EQ(joint_names[0], "joint1");

    auto joint1 = robot->joint("joint1");
    EXPECT_EQ(joint1->type(), Joint::Type::REVOLUTE);
    EXPECT_EQ(joint1->axis(), Eigen::Vector3d(0, 0, 1));
}

// *********************************************************************************
//! \brief Test parsing SCARA robot URDF.
// *********************************************************************************
TEST_F(URDFParserTest, ParseSCARArobot)
{
    // Skip test if file doesn't exist
    std::ifstream file(urdf_file_path);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << urdf_file_path;
    }
    file.close();

    auto robot = parser->load(urdf_file_path);
    ASSERT_NE(robot, nullptr);

    // Check that the robot has the expected number of joints
    auto joint_names = robot->jointNames();
    EXPECT_EQ(joint_names.size(), 3);

    // Check specific joints exist
    EXPECT_NE(robot->joint("joint1"), nullptr);
    EXPECT_NE(robot->joint("joint2"), nullptr);
    EXPECT_NE(robot->joint("end_effector"), nullptr);
    EXPECT_EQ(robot->joint("joint4"), nullptr);

    // Check joint types
    auto root = robot->joint("joint1");
    auto joint2 = robot->joint("joint2");
    auto end_effector = robot->joint("end_effector");

    EXPECT_EQ(root->type(), Joint::Type::REVOLUTE);
    EXPECT_EQ(joint2->type(), Joint::Type::REVOLUTE);
    EXPECT_EQ(end_effector->type(), Joint::Type::PRISMATIC);

    // Check end effector and root
    std::cout << "root: " << root << std::endl;
    std::cout << "joint2: " << joint2 << std::endl;
    std::cout << "end_effector: " << end_effector << std::endl;

    EXPECT_EQ(robot->endEffector(), end_effector);
    EXPECT_EQ(robot->getRootNode(), root);
}

// *********************************************************************************
//! \brief Test forward kinematics with parsed SCARA robot.
// *********************************************************************************
TEST_F(URDFParserTest, SCARAForwardKinematics)
{
    // Skip test if file doesn't exist
    std::ifstream file(urdf_file_path);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << urdf_file_path;
    }
    file.close();

    auto robot = parser->load(urdf_file_path);
    ASSERT_NE(robot, nullptr);

    // Test forward kinematics with zero joint values
    std::vector<double> zero_values(4, 0.0); // 4 movable joints
    robot->setJointValues(zero_values);

    Transform fk_result = robot->forwardKinematics();

    // Verify that the result is a valid transformation matrix
    EXPECT_DOUBLE_EQ(fk_result(3, 3), 1.0);
    EXPECT_DOUBLE_EQ(fk_result(3, 0), 0.0);
    EXPECT_DOUBLE_EQ(fk_result(3, 1), 0.0);
    EXPECT_DOUBLE_EQ(fk_result(3, 2), 0.0);

    // Test with different joint values
    std::vector<double> test_values = { M_PI / 4, M_PI / 6, M_PI / 8, 0.05 };
    robot->setJointValues(test_values);

    Transform fk_result2 = robot->forwardKinematics();
    EXPECT_NO_THROW(robot->forwardKinematics());

    // Verify the position is reasonable
    Eigen::Vector3d position = fk_result2.block<3, 1>(0, 3);
    EXPECT_TRUE(position.array().isFinite().all());
    EXPECT_FALSE(position.hasNaN());
}

// *********************************************************************************
//! \brief Test joint limits from URDF.
// *********************************************************************************
TEST_F(URDFParserTest, JointLimits)
{
    // Skip test if file doesn't exist
    std::ifstream file(urdf_file_path);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << urdf_file_path;
    }
    file.close();

    auto robot = parser->load(urdf_file_path);
    ASSERT_NE(robot, nullptr);

    // Test joint limits for revolute joints
    auto joint1 = robot->joint("joint1");
    ASSERT_NE(joint1, nullptr);

    // Test that joint limits are properly set
    // The URDF specifies -3.14 to 3.14 for revolute joints
    EXPECT_NO_THROW(joint1->value(3.0));
    EXPECT_NO_THROW(joint1->value(-3.0));

    // Test prismatic joint limits
    auto joint4 = robot->joint("joint4");
    ASSERT_NE(joint4, nullptr);
    EXPECT_EQ(joint4->type(), Joint::Type::PRISMATIC);

    // Test that prismatic joint accepts values within limits
    EXPECT_NO_THROW(joint4->value(0.05));
    EXPECT_NO_THROW(joint4->value(-0.05));
}

// *********************************************************************************
//! \brief Test complete SCARA robot functionality.
// *********************************************************************************
TEST_F(URDFParserTest, CompleteSCARAFunctionality)
{
    // Skip test if file doesn't exist
    std::ifstream file(urdf_file_path);
    if (!file.is_open())
    {
        GTEST_FAIL() << "URDF file not found: " << urdf_file_path;
    }
    file.close();

    auto robot = parser->load(urdf_file_path);
    ASSERT_NE(robot, nullptr);

    // Test typical SCARA movements
    std::vector<double> scara_config = { 0.0, 0.0, 0.0, 0.0 };
    robot->setJointValues(scara_config);

    // Test forward kinematics
    Transform fk_result = robot->forwardKinematics();
    std::cout << "fk_result: " << fk_result << std::endl;
    EXPECT_NO_THROW(robot->forwardKinematics());

    // Test end effector pose
    Pose end_effector_pose = robot->endEffectorPose();
    EXPECT_EQ(end_effector_pose.size(), 6);

    // Test Jacobian calculation
    Jacobian jacobian = robot->calculateJacobian();
    EXPECT_EQ(jacobian.rows(), 6);
    EXPECT_EQ(jacobian.cols(), 4); // 4 movable joints

    // Test joint value retrieval
    std::vector<double> retrieved_values = robot->jointValues();
    EXPECT_EQ(retrieved_values.size(), 4);

    for (size_t i = 0; i < scara_config.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(retrieved_values[i], scara_config[i]);
    }
}

// *********************************************************************************
//! \brief Test parsing geometry with different link types.
// *********************************************************************************
TEST_F(URDFParserTest, ParseGeometryURDF)
{
    std::string geom_urdf = R"(<?xml version="1.0"?>
<robot name="geometry_test">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.2 0.3"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <link name="cylinder_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="cylinder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>)";

    // Create a temporary URDF file
    std::string temp_file = "/tmp/test_geometry.urdf";
    std::ofstream file(temp_file);
    file << geom_urdf;
    file.close();

    EXPECT_NO_THROW({
        auto robot = parser->load(temp_file);
        EXPECT_NE(robot, nullptr);
    });

    // Clean up
    std::remove(temp_file.c_str());
}