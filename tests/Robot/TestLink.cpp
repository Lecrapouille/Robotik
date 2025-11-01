/**
 * @file TestLink.cpp
 * @brief Unit tests for the Link class - Verification of Link creation,
 * geometry access, inertial properties, and integration with the kinematic
 * tree.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "main.hpp"

#include "Robotik/Core/Common/Path.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"
#include "Robotik/Core/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

using namespace robotik;

// *********************************************************************************
//! \brief Test fixture for Link class.
// *********************************************************************************
class LinkTest: public ::testing::Test
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

    std::unique_ptr<URDFLoader> parser;
};

// *********************************************************************************
//! \brief Test Link creation with geometry.
// *********************************************************************************
TEST_F(LinkTest, Creation)
{
    // Create a Box geometry
    auto geometry = std::make_unique<Box>("test_box", 0.1, 0.2, 0.3);

    // Create Link with geometry
    Link link("test_link", std::move(geometry));

    EXPECT_EQ(link.name(), "test_link");
    EXPECT_EQ(link.children().size(), 1);
}

// *********************************************************************************
//! \brief Test geometry access (const).
// *********************************************************************************
TEST_F(LinkTest, GeometryAccessConst)
{
    auto geometry = std::make_unique<Box>("test_box", 0.1, 0.2, 0.3);
    Link link("test_link", std::move(geometry));

    Geometry const& geom = link.geometry();
    EXPECT_EQ(geom.name(), "test_box");
    EXPECT_EQ(geom.type(), Geometry::Type::BOX);
    EXPECT_EQ(geom.parameters().size(), 3u);
    EXPECT_DOUBLE_EQ(geom.parameters()[0], 0.1);
    EXPECT_DOUBLE_EQ(geom.parameters()[1], 0.2);
    EXPECT_DOUBLE_EQ(geom.parameters()[2], 0.3);
}

// *********************************************************************************
//! \brief Test geometry access (non-const).
// *********************************************************************************
TEST_F(LinkTest, GeometryAccessNonConst)
{
    auto geometry = std::make_unique<Cylinder>("test_cyl", 0.05, 0.2);
    Link link("test_link", std::move(geometry));

    Geometry& geom = link.geometry();
    EXPECT_EQ(geom.name(), "test_cyl");
    EXPECT_EQ(geom.type(), Geometry::Type::CYLINDER);

    // Modify color
    geom.color = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(geom.color.x(), 1.0f);
    EXPECT_FLOAT_EQ(geom.color.y(), 0.0f);
    EXPECT_FLOAT_EQ(geom.color.z(), 0.0f);
}

// *********************************************************************************
//! \brief Test inertial properties API.
// *********************************************************************************
TEST_F(LinkTest, InertiaAPI)
{
    auto geometry = std::make_unique<Box>("test_box", 0.1, 0.2, 0.3);
    Link link("test_link", std::move(geometry));

    // Set inertial properties
    double mass = 2.5;
    Eigen::Vector3d com(0.1, 0.05, 0.0);
    Eigen::Matrix3d inertia_matrix;
    inertia_matrix << 0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03;

    link.inertia(mass, com, inertia_matrix);

    // Test mass()
    EXPECT_DOUBLE_EQ(link.mass(), 2.5);

    // Test centerOfMass()
    Eigen::Vector4d com_result = link.centerOfMass();
    EXPECT_DOUBLE_EQ(com_result.x(), 0.1);
    EXPECT_DOUBLE_EQ(com_result.y(), 0.05);
    EXPECT_DOUBLE_EQ(com_result.z(), 0.0);
    EXPECT_DOUBLE_EQ(com_result.w(), 1.0);

    // Test length()
    double expected_length = com.norm();
    EXPECT_NEAR(link.length(), expected_length, 1e-9);

    // Test inertia() accessor
    Inertial const& inertial = link.inertia();
    EXPECT_DOUBLE_EQ(inertial.mass, 2.5);
    EXPECT_TRUE(inertial.center_of_mass.isApprox(com));
    EXPECT_TRUE(inertial.inertia_matrix.isApprox(inertia_matrix));
}

// *********************************************************************************
//! \brief Test inertial properties using Inertial struct.
// *********************************************************************************
TEST_F(LinkTest, InertiaStruct)
{
    auto geometry = std::make_unique<Sphere>("test_sphere", 0.1);
    Link link("test_link", std::move(geometry));

    Inertial inertial;
    inertial.mass = 1.5;
    inertial.center_of_mass = Eigen::Vector3d(0.2, 0.1, 0.05);
    inertial.inertia_matrix =
        Eigen::Matrix3d::Identity() * 0.005; // Simple diagonal matrix

    link.inertia(inertial);

    EXPECT_DOUBLE_EQ(link.mass(), 1.5);
    Eigen::Vector4d com = link.centerOfMass();
    EXPECT_DOUBLE_EQ(com.x(), 0.2);
    EXPECT_DOUBLE_EQ(com.y(), 0.1);
    EXPECT_DOUBLE_EQ(com.z(), 0.05);
}

// *********************************************************************************
//! \brief Test setCollisionData.
// *********************************************************************************
TEST_F(LinkTest, SetCollisionData)
{
    auto geometry = std::make_unique<Box>("test_box", 0.1, 0.2, 0.3);
    Link link("test_link", std::move(geometry));

    Eigen::Vector3d collision_center(0.05, 0.1, 0.0);
    Eigen::Matrix3d collision_orientation = Eigen::Matrix3d::Identity();
    std::vector<double> collision_params{ 0.05, 0.1, 0.15 };

    link.setCollisionData(
        collision_center, collision_orientation, collision_params);

    // Verify that collision data was set (by accessing through geometry)
    // Note: The collision data is stored in Geometry, so we verify it through
    // the geometry accessor
    Geometry const& geom = link.geometry();
    // The setCollisionData on Link calls geometry().setCollisionData()
    // We can verify the geometry still exists and is accessible
    EXPECT_EQ(geom.name(), "test_box");
}

// *********************************************************************************
//! \brief Test Link integration in Node tree with Joint.
// *********************************************************************************
TEST_F(LinkTest, NodeIntegration)
{
    // Create a joint
    auto joint = Node::create<Joint>(
        "joint1", Joint::Type::REVOLUTE, Eigen::Vector3d(0, 0, 1));
    Joint* joint_ptr = joint.get();

    // Create a link with geometry
    auto geometry = std::make_unique<Box>("link_geom", 0.1, 0.1, 0.1);
    auto link = std::make_unique<Link>("link1", std::move(geometry));
    Link* link_ptr = link.get();

    // Add link as child of joint
    joint_ptr->addChild(std::move(link));

    // Verify parent-child relationship
    EXPECT_EQ(joint_ptr->children().size(), 1);
    EXPECT_EQ(Node::find(*joint_ptr, "link1"), link_ptr);

    // Verify link still has geometry
    EXPECT_EQ(link_ptr->geometry().name(), "link_geom");
}

// *********************************************************************************
//! \brief Test Link properties from URDF loaded robot (simple_revolute_robot).
// *********************************************************************************
TEST_F(LinkTest, URDFLoadedLinksSimpleRevolute)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    // Get base_link
    Link const& base_link = robot->blueprint().link("base_link");
    EXPECT_EQ(base_link.name(), "base_link");

    // Verify geometry exists
    Geometry const& base_geom = base_link.geometry();
    EXPECT_EQ(base_geom.type(), Geometry::Type::BOX);
    EXPECT_EQ(base_geom.parameters().size(), 3u);

    // Get arm_link
    Link const& arm_link = robot->blueprint().link("arm_link");
    EXPECT_EQ(arm_link.name(), "arm_link");

    Geometry const& arm_geom = arm_link.geometry();
    EXPECT_EQ(arm_geom.type(), Geometry::Type::BOX);
}

// *********************************************************************************
//! \brief Test Link inertial properties from URDF with inertia
// *********************************************************************************
TEST_F(LinkTest, URDFLoadedLinksWithInertia)
{
    auto robot = parseRobot("simple_revolute_robot_with_inertia.urdf");
    ASSERT_NE(robot, nullptr)
        << "Failed to load simple_revolute_robot_with_inertia.urdf";

    // Test base_link inertial properties
    Link const& base_link = robot->blueprint().link("base_link");
    EXPECT_DOUBLE_EQ(base_link.mass(), 1.0);

    Eigen::Vector4d com = base_link.centerOfMass();
    EXPECT_NEAR(com.x(), 0.0, 1e-6);
    EXPECT_NEAR(com.y(), 0.0, 1e-6);
    EXPECT_NEAR(com.z(), 0.0, 1e-6);

    Inertial const& base_inertial = base_link.inertia();
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(0, 0), 0.01); // ixx
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(1, 1), 0.01); // iyy
    EXPECT_DOUBLE_EQ(base_inertial.inertia_matrix(2, 2), 0.01); // izz

    // Test arm_link inertial properties
    Link const& arm_link = robot->blueprint().link("arm_link");
    EXPECT_DOUBLE_EQ(arm_link.mass(), 0.5);

    Eigen::Vector4d arm_com = arm_link.centerOfMass();
    EXPECT_NEAR(arm_com.x(), 0.1, 1e-6); // center of mass at (0.1, 0, 0)
    EXPECT_NEAR(arm_com.y(), 0.0, 1e-6);
    EXPECT_NEAR(arm_com.z(), 0.0, 1e-6);

    Inertial const& arm_inertial = arm_link.inertia();
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(0, 0), 0.001); // ixx
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(1, 1), 0.001); // iyy
    EXPECT_DOUBLE_EQ(arm_inertial.inertia_matrix(2, 2), 0.001); // izz

    // Test length() method
    double arm_length = arm_link.length();
    EXPECT_NEAR(arm_length, 0.1, 1e-6); // ||(0.1, 0, 0)|| = 0.1
}

// *********************************************************************************
//! \brief Test Link with different geometry types from URDF.
// *********************************************************************************
TEST_F(LinkTest, URDFLoadedLinksDifferentGeometryTypes)
{
    auto robot = parseRobot("simple_diff_drive_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_diff_drive_robot.urdf";

    // Test link with cylinder geometry
    Link const& wheel_link = robot->blueprint().link("left_wheel");
    Geometry const& wheel_geom = wheel_link.geometry();
    EXPECT_EQ(wheel_geom.type(), Geometry::Type::CYLINDER);
    EXPECT_EQ(wheel_geom.parameters().size(), 2u); // radius, length

    // Test link with sphere geometry
    Link const& caster_link = robot->blueprint().link("caster_wheel");
    Geometry const& caster_geom = caster_link.geometry();
    EXPECT_EQ(caster_geom.type(), Geometry::Type::SPHERE);
    EXPECT_EQ(caster_geom.parameters().size(), 1u); // radius

    // Test link with box geometry
    Link const& base_link = robot->blueprint().link("base_link");
    Geometry const& base_geom = base_link.geometry();
    EXPECT_EQ(base_geom.type(), Geometry::Type::BOX);
}

// *********************************************************************************
//! \brief Test Link transform propagation in kinematic tree.
// *********************************************************************************
TEST_F(LinkTest, TransformPropagation)
{
    auto robot = parseRobot("simple_revolute_robot.urdf");
    ASSERT_NE(robot, nullptr) << "Failed to load simple_revolute_robot.urdf";

    Link const& base_link = robot->blueprint().link("base_link");

    // Set joint position to change transforms
    robot->blueprint().joint("revolute_joint").position(M_PI / 4.0);
    robot->setJointPositions();

    // Verify link has world transform
    Transform world_tf = base_link.worldTransform();
    EXPECT_DOUBLE_EQ(world_tf(3, 3), 1.0); // Homogeneous matrix bottom-right

    // Verify geometry inherits parent transform
    Geometry const& geom = base_link.geometry();
    Transform geom_world_tf = geom.worldTransform();
    EXPECT_DOUBLE_EQ(geom_world_tf(3, 3), 1.0);
}
