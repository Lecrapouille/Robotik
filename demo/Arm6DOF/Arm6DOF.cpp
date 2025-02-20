#include "Robotik/Robotik.hpp"
#include <iostream>
#include <iomanip>

using namespace RobotKinematics;

void printTransform(const Transform& transform, const std::string& name)
{
    std::cout << "=== " << name << " ===" << std::endl;
    Eigen::IOFormat format(4, 0, ", ", "\n", "  ");
    std::cout << transform.format(format) << std::endl;

    // Extraire position et orientation
    Eigen::Vector3d pos = Utils::getTranslation(transform);
    Eigen::Vector3d euler = Utils::rotationToEuler(Utils::getRotation(transform));

    std::cout << "Position: [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]" << std::endl;
    std::cout << "Orientation (rad): [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]" << std::endl;
    std::cout << "Orientation (deg): ["
              << euler.x() * 180.0/M_PI << ", "
              << euler.y() * 180.0/M_PI << ", "
              << euler.z() * 180.0/M_PI << "]" << std::endl;
    std::cout << std::endl;
}

// Création d'un bras robot 6DOF inspiré par un bras industriel standard
int main()
{
    // Créer le bras robot
    auto robot = std::make_shared<RobotArm>("6DOF_Robot");

    // Créer les nœuds du graphe de scène
    auto base = std::make_shared<Node>("base");
    auto link1 = std::make_shared<Node>("link1");
    auto link2 = std::make_shared<Node>("link2");
    auto link3 = std::make_shared<Node>("link3");
    auto link4 = std::make_shared<Node>("link4");
    auto link5 = std::make_shared<Node>("link5");
    auto link6 = std::make_shared<Node>("link6");
    auto gripper = std::make_shared<Node>("gripper");

    // Construire la hiérarchie
    base->addChild(link1);
    link1->addChild(link2);
    link2->addChild(link3);
    link3->addChild(link4);
    link4->addChild(link5);
    link5->addChild(link6);
    link6->addChild(gripper);

    // Configurer la géométrie fixe du gripper (offset par rapport au dernier joint)
    Transform gripperOffset = Transform::Identity();
    gripperOffset.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 0.1);  // 10cm d'offset
    gripper->setLocalTransform(gripperOffset);

    // Créer les joints
    auto joint1 = std::make_shared<Joint>("joint1", JointType::REVOLUTE, Eigen::Vector3d::UnitZ());
    auto joint2 = std::make_shared<Joint>("joint2", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint3 = std::make_shared<Joint>("joint3", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint4 = std::make_shared<Joint>("joint4", JointType::REVOLUTE, Eigen::Vector3d::UnitX());
    auto joint5 = std::make_shared<Joint>("joint5", JointType::REVOLUTE, Eigen::Vector3d::UnitY());
    auto joint6 = std::make_shared<Joint>("joint6", JointType::REVOLUTE, Eigen::Vector3d::UnitZ());

    // Associer les joints aux nœuds
    joint1->setNode(link1);
    joint2->setNode(link2);
    joint3->setNode(link3);
    joint4->setNode(link4);
    joint5->setNode(link5);
    joint6->setNode(link6);

    // Configurer les limites des joints (en radians)
    joint1->setLimits(-M_PI, M_PI);         // ±180°
    joint2->setLimits(-M_PI/2, M_PI/2);     // ±90°
    joint3->setLimits(-M_PI/2, M_PI/2);     // ±90°
    joint4->setLimits(-M_PI, M_PI);         // ±180°
    joint5->setLimits(-M_PI/2, M_PI/2);     // ±90°
    joint6->setLimits(-M_PI, M_PI);         // ±180°

    // Configurer le bras
    robot->setRootNode(base);
    robot->addJoint(joint1);
    robot->addJoint(joint2);
    robot->addJoint(joint3);
    robot->addJoint(joint4);
    robot->addJoint(joint5);
    robot->addJoint(joint6);
    robot->setEndEffector(gripper);

    // Définir les offsets géométriques entre les joints
    // Dans un vrai robot, ces valeurs viendraient du modèle CAD ou des paramètres DH
    Transform link1Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.2), 0, 0, 0);
    Transform link2Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.3), M_PI/2, 0, 0);
    Transform link3Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.3), 0, 0, 0);
    Transform link4Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI/2, 0, 0);
    Transform link5Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI/2, 0, 0);
    Transform link6Offset = Utils::createTransform(Eigen::Vector3d(0, 0, 0.1), M_PI/2, 0, 0);

    // Test de cinématique directe
    std::cout << "=== Test de cinématique directe ===" << std::endl;

    // Position de repos
    robot->setJointValues({0, 0, 0, 0, 0, 0});
    printTransform(robot->forwardKinematics(), "Position de repos");

    // Position de test 1
    robot->setJointValues({M_PI/4, M_PI/6, -M_PI/6, M_PI/4, M_PI/3, -M_PI/4});
    printTransform(robot->forwardKinematics(), "Position de test 1");

    // Test de cinématique inverse
    std::cout << "=== Test de cinématique inverse ===" << std::endl;

    // Position cible
    Pose targetPose;
    targetPose << 0.5, 0.3, 0.7, 0, M_PI/2, 0;  // x, y, z, rx, ry, rz

    std::cout << "Position cible: ["
              << targetPose.segment<3>(0).transpose() << "], ["
              << targetPose.segment<3>(3).transpose() << "]" << std::endl;

    // Partir d'une configuration de repos
    robot->setJointValues({0, 0, 0, 0, 0, 0});

    // Résoudre la cinématique inverse
    std::vector<double> solution;
    bool success = robot->inverseKinematics(targetPose, solution);

    if (success)
    {
        std::cout << "Cinématique inverse réussie!" << std::endl;
        std::cout << "Valeurs des joints: ";
        for (double val : solution) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        // Vérifier la solution
        robot->setJointValues(solution);
        printTransform(robot->forwardKinematics(), "Position résultante");
    }
    else
    {
        std::cout << "Échec de la cinématique inverse" << std::endl;
    }

    return 0;
}
