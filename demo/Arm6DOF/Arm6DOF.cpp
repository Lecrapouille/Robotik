#include "Robotik/Robotik.hpp"
#include <iomanip>
#include <iostream>

using namespace robotik;

void printTransform(const Transform& transform, const std::string& name)
{
    std::cout << "=== " << name << " ===" << std::endl;
    Eigen::IOFormat format(4, 0, ", ", "\n", "  ");
    std::cout << transform.format(format) << std::endl;

    // Extract position and orientation
    Eigen::Vector3d pos = utils::getTranslation(transform);
    Eigen::Vector3d euler =
        utils::rotationToEuler(utils::getRotation(transform));

    std::cout << "Position: [" << pos.x() << ", " << pos.y() << ", " << pos.z()
              << "]" << std::endl;
    std::cout << "Orientation (rad): [" << euler.x() << ", " << euler.y()
              << ", " << euler.z() << "]" << std::endl;
    std::cout << "Orientation (deg): [" << euler.x() * 180.0 / M_PI << ", "
              << euler.y() * 180.0 / M_PI << ", " << euler.z() * 180.0 / M_PI
              << "]" << std::endl;
    std::cout << std::endl;
}

// Create a 6DOF robot arm inspired by a standard industrial robot
int main()
{
    // Create the robot arm
    auto robot = std::make_shared<RobotArm>("6DOF_Robot");

    // Create the nodes of the scene graph
    // We need separate nodes for geometry offsets and joint rotations
    auto base = std::make_shared<Node>("base");
    auto joint1_node = std::make_shared<Node>("joint1_node");
    auto link1 = std::make_shared<Node>("link1");
    auto joint2_node = std::make_shared<Node>("joint2_node");
    auto link2 = std::make_shared<Node>("link2");
    auto joint3_node = std::make_shared<Node>("joint3_node");
    auto link3 = std::make_shared<Node>("link3");
    auto joint4_node = std::make_shared<Node>("joint4_node");
    auto link4 = std::make_shared<Node>("link4");
    auto joint5_node = std::make_shared<Node>("joint5_node");
    auto link5 = std::make_shared<Node>("link5");
    auto joint6_node = std::make_shared<Node>("joint6_node");
    auto link6 = std::make_shared<Node>("link6");
    auto gripper = std::make_shared<Node>("gripper");

    // Build the hierarchy: base -> joint1 -> link1 -> joint2 -> link2 -> ...
    base->addChild(joint1_node);
    joint1_node->addChild(link1);
    link1->addChild(joint2_node);
    joint2_node->addChild(link2);
    link2->addChild(joint3_node);
    joint3_node->addChild(link3);
    link3->addChild(joint4_node);
    joint4_node->addChild(link4);
    link4->addChild(joint5_node);
    joint5_node->addChild(link5);
    link5->addChild(joint6_node);
    joint6_node->addChild(link6);
    link6->addChild(gripper);

    // Configure the fixed geometry of the gripper (offset by the last joint)
    Transform gripperOffset = Transform::Identity();
    gripperOffset.block<3, 1>(0, 3) =
        Eigen::Vector3d(0, 0, 0.1); // 10cm d'offset
    gripper->setLocalTransform(gripperOffset);

    // Create the joints
    auto joint1 = std::make_shared<Joint>(
        "joint1", JointType::REVOLUTE, Eigen::Vector3d::UnitZ());
    auto joint2 = std::make_shared<Joint>(
        "joint2", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint3 = std::make_shared<Joint>(
        "joint3", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint4 = std::make_shared<Joint>(
        "joint4", JointType::REVOLUTE, Eigen::Vector3d::UnitX());
    auto joint5 = std::make_shared<Joint>(
        "joint5", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint6 = std::make_shared<Joint>(
        "joint6", JointType::REVOLUTE, Eigen::Vector3d::UnitZ());

    // Associate the joints to the joint nodes (not the link nodes)
    joint1->setNode(joint1_node);
    joint2->setNode(joint2_node);
    joint3->setNode(joint3_node);
    joint4->setNode(joint4_node);
    joint5->setNode(joint5_node);
    joint6->setNode(joint6_node);

    // Configure the limits of the joints (in radians)
    joint1->setLimits(-M_PI, M_PI);         // ±180°
    joint2->setLimits(-M_PI / 2, M_PI / 2); // ±90°
    joint3->setLimits(-M_PI / 2, M_PI / 2); // ±90°
    joint4->setLimits(-M_PI, M_PI);         // ±180°
    joint5->setLimits(-M_PI / 2, M_PI / 2); // ±90°
    joint6->setLimits(-M_PI, M_PI);         // ±180°

    // Configure the robot arm
    robot->setRootNode(base);
    robot->addJoint(joint1);
    robot->addJoint(joint2);
    robot->addJoint(joint3);
    robot->addJoint(joint4);
    robot->addJoint(joint5);
    robot->addJoint(joint6);
    robot->setEndEffector(gripper);

    // Define the geometric offsets between the joints
    // In a real robot, these values would come from the CAD model or the DH
    // parameters
    Transform link1Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.2), 0, 0, 0);
    Transform link2Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.3), M_PI / 2, 0, 0);
    Transform link3Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.3), 0, 0, 0);
    Transform link4Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI / 2, 0, 0);
    Transform link5Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI / 2, 0, 0);
    Transform link6Offset =
        utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI / 2, 0, 0);

    // Apply the geometric offsets to the link nodes
    link1->setLocalTransform(link1Offset);
    link2->setLocalTransform(link2Offset);
    link3->setLocalTransform(link3Offset);
    link4->setLocalTransform(link4Offset);
    link5->setLocalTransform(link5Offset);
    link6->setLocalTransform(link6Offset);

    // Test of forward kinematics
    std::cout << "=== Test of forward kinematics ===" << std::endl;

    // Repos position
    robot->setJointValues({ 0, 0, 0, 0, 0, 0 });
    printTransform(robot->forwardKinematics(), "Position de repos");

    // Test position 1
    robot->setJointValues(
        { M_PI / 4, M_PI / 6, -M_PI / 6, M_PI / 4, M_PI / 3, -M_PI / 4 });
    printTransform(robot->forwardKinematics(), "Position de test 1");

    // Test of inverse kinematics
    std::cout << "=== Test of inverse kinematics ===" << std::endl;

    // Target position - use the position from test position 1 (known to be
    // reachable)
    Pose targetPose;
    targetPose << 0.153692, -0.140951, 0.479251, -2.02747, -2.07754,
        2.04553; // x, y, z, rx, ry, rz

    std::cout << "Target position: [" << targetPose.segment<3>(0).transpose()
              << "], [" << targetPose.segment<3>(3).transpose() << "]"
              << std::endl;

    // Start from the configuration that produces the target position
    robot->setJointValues(
        { M_PI / 4, M_PI / 6, -M_PI / 6, M_PI / 4, M_PI / 3, -M_PI / 4 });

    // Print initial pose
    Pose currentPose = robot->getEndEffectorPose();
    std::cout << "Initial pose: [" << currentPose.segment<3>(0).transpose()
              << "], [" << currentPose.segment<3>(3).transpose() << "]"
              << std::endl;

    // Calculate initial error
    Pose error = targetPose - currentPose;
    std::cout << "Initial error norm: " << error.norm() << std::endl;

    // Solve the inverse kinematics
    if (std::vector<double> solution;
        robot->inverseKinematics(targetPose, solution))
    {
        std::cout << "Inverse kinematics succeeded!" << std::endl;
        std::cout << "Joint values: ";
        for (double val : solution)
        {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        // Check the solution
        robot->setJointValues(solution);
        printTransform(robot->forwardKinematics(), "Resulting position");
    }
    else
    {
        std::cerr << "Inverse kinematics failed" << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
