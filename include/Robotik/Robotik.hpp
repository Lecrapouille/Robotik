#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <Eigen/Dense>

namespace robotik {

// Représentation d'une transformation homogène 4x4
using Transform = Eigen::Matrix4d;
// Vecteur 6D pour représenter une position/orientation (x,y,z,rx,ry,rz)
using Pose = Eigen::Matrix<double, 6, 1>;
// Jacobienne
using Jacobian = Eigen::MatrixXd;

// Classe pour les différents types de joints
enum class JointType {
    REVOLUTE,   // Joint rotatif
    PRISMATIC,  // Joint prismatique (linéaire)
    FIXED       // Joint fixe (pas de mouvement)
};

// Classe représentant un nœud dans le graphe de scène
class Node
{
private:
    std::string name;
    Transform localTransform;
    Transform worldTransform;
    std::vector<std::shared_ptr<Node>> children;
    std::weak_ptr<Node> parent;
    bool isDirty;

public:
    Node(const std::string& name);

    // Gestion de la hiérarchie
    void addChild(std::shared_ptr<Node> child);
    void removeChild(const std::string& childName);
    std::shared_ptr<Node> getChild(const std::string& childName);

    // Gestion des transformations
    void setLocalTransform(const Transform& transform);
    const Transform& getLocalTransform() const;
    const Transform& getWorldTransform();

    // Mise à jour des transformations
    void updateWorldTransform();
    void markDirty();

    const std::string& getName() const;
};

// Classe représentant un joint robotique
class Joint
{
private:
    std::string name;
    JointType type;
    double value;
    double min;
    double max;
    Eigen::Vector3d axis;
    std::shared_ptr<Node> node;

public:
    Joint(const std::string& name, JointType type, const Eigen::Vector3d& axis);

    void setValue(double val);
    double getValue() const;
    void setLimits(double minVal, double maxVal);

    Transform getTransform() const;
    void updateNodeTransform();

    JointType getType() const;
    const Eigen::Vector3d& getAxis() const;
    void setNode(std::shared_ptr<Node> node);
    std::shared_ptr<Node> getNode();
};

// Classe pour un bras robotique complet
class RobotArm
{
private:
    std::string name;
    std::shared_ptr<Node> rootNode;
    std::vector<std::shared_ptr<Joint>> joints;
    std::unordered_map<std::string, std::shared_ptr<Joint>> jointMap;
    std::shared_ptr<Node> endEffector;

public:
    RobotArm(const std::string& name);

    // Configuration du robot
    void setRootNode(std::shared_ptr<Node> root);
    void addJoint(std::shared_ptr<Joint> joint);
    void setEndEffector(std::shared_ptr<Node> node);

    // Cinématique directe
    Transform forwardKinematics();
    Pose getEndEffectorPose();

    // Cinématique inverse
    bool inverseKinematics(const Pose& targetPose, std::vector<double>& solution);

    // Calcul de la jacobienne
    Jacobian calculateJacobian();

    // Accès aux joints
    std::shared_ptr<Joint> getJoint(const std::string& name);
    std::vector<double> getJointValues();
    void setJointValues(const std::vector<double>& values);
};

// Fonctions utilitaires pour les conversions
namespace utils
{
    // Conversion entre matrices de rotation et représentations d'angle
    Eigen::Matrix3d eulerToRotation(double rx, double ry, double rz);
    Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& rot);

    // Conversions pour transformations homogènes
    Transform createTransform(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation);
    Transform createTransform(const Eigen::Vector3d& translation, double rx, double ry, double rz);

    // Extraction de données des transformations
    Eigen::Vector3d getTranslation(const Transform& transform);
    Eigen::Matrix3d getRotation(const Transform& transform);
    Pose transformToPose(const Transform& transform);
    Transform poseToTransform(const Pose& pose);

    // Conversion DH (Denavit-Hartenberg)
    Transform dhTransform(double a, double alpha, double d, double theta);
} // namespace utils
} // namespace robotik
