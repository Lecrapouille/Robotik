#include "Robotik/Robotik.hpp"
#include <cmath>
#include <stdexcept>

namespace robotik {

//------------------------------------------------------------------------------
// Implémentation de Node
//------------------------------------------------------------------------------

Node::Node(const std::string& name)
    : name(name), isDirty(true)
{
    localTransform = Transform::Identity();
    worldTransform = Transform::Identity();
}

void Node::addChild(std::shared_ptr<Node> child)
{
    children.push_back(child);
    child->parent = std::weak_ptr<Node>(shared_from_this());
    child->markDirty();
}

void Node::removeChild(const std::string& childName)
{
    auto it = std::find_if(children.begin(), children.end(),
                         [&childName](const std::shared_ptr<Node>& child)
                         {
                             return child->getName() == childName;
                         });

    if (it != children.end())
    {
        children.erase(it);
    }
}

std::shared_ptr<Node> Node::getChild(const std::string& childName)
{
    auto it = std::find_if(children.begin(), children.end(),
                         [&childName](const std::shared_ptr<Node>& child)
                         {
                             return child->getName() == childName;
                         });

    if (it != children.end())
    {
        return *it;
    }
    return nullptr;
}

void Node::setLocalTransform(const Transform& transform)
{
    localTransform = transform;
    markDirty();
}

const Transform& Node::getLocalTransform() const
{
    return localTransform;
}

const Transform& Node::getWorldTransform()
{
    if (isDirty)
    {
        updateWorldTransform();
    }
    return worldTransform;
}

void Node::updateWorldTransform()
{
    if (auto parentNode = parent.lock())
    {
        worldTransform = parentNode->getWorldTransform() * localTransform;
    }
    else
    {
        worldTransform = localTransform;
    }

    isDirty = false;

    // Mettre à jour les transformations des enfants
    for (auto& child : children)
    {
        child->updateWorldTransform();
    }
}

void Node::markDirty()
{
    isDirty = true;
    for (auto& child : children)
    {
        child->markDirty();
    }
}

const std::string& Node::getName() const
{
    return name;
}

//------------------------------------------------------------------------------
// Implémentation de Joint
//------------------------------------------------------------------------------

Joint::Joint(const std::string& name, JointType type, const Eigen::Vector3d& axis)
    : name(name), type(type), value(0.0), min(-M_PI), max(M_PI), axis(axis.normalized())
{}

void Joint::setValue(double val)
{
    // Appliquer les limites
    if (val < min) val = min;
    if (val > max) val = max;

    value = val;

    // Mettre à jour la transformation du nœud
    if (node)
    {
        updateNodeTransform();
    }
}

double Joint::getValue() const
{
    return value;
}

void Joint::setLimits(double minVal, double maxVal)
{
    min = minVal;
    max = maxVal;
}

Transform Joint::getTransform() const
{
    Transform transform = Transform::Identity();

    switch (type)
    {
        case JointType::REVOLUTE:
        {
            // Créer la matrice de rotation autour de l'axe
            Eigen::AngleAxisd rotation(value, axis);
            transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            break;
        }
        case JointType::PRISMATIC:
        {
            // Translation le long de l'axe
            transform.block<3, 1>(0, 3) = axis * value;
            break;
        }
        case JointType::FIXED:
            // Pas de transformation pour les joints fixes
            break;
    }

    return transform;
}

void Joint::updateNodeTransform()
{
    node->setLocalTransform(getTransform());
}

JointType Joint::getType() const
{
    return type;
}

const Eigen::Vector3d& Joint::getAxis() cons
 {
    return axis;
}

void Joint::setNode(std::shared_ptr<Node> n)
{
    node = n;
    updateNodeTransform();
}

std::shared_ptr<Node> Joint::getNode()
{
    return node;
}

//------------------------------------------------------------------------------
// Implémentation de RobotArm
//------------------------------------------------------------------------------

RobotArm::RobotArm(const std::string& name)
    : name(name)
{}

void RobotArm::setRootNode(std::shared_ptr<Node> root)
{
    rootNode = root;
}

void RobotArm::addJoint(std::shared_ptr<Joint> joint)
{
    joints.push_back(joint);
    jointMap[joint->getName()] = joint;
}

void RobotArm::setEndEffector(std::shared_ptr<Node> node)
{
    endEffector = node;
}

Transform RobotArm::forwardKinematics()
{
    if (!endEffector)
    {
        throw std::runtime_error("End effector not set");
    }

    // S'assurer que toutes les transformations sont à jour
    rootNode->updateWorldTransform();

    return endEffector->getWorldTransform();
}

Pose RobotArm::getEndEffectorPose()
{
    Transform transform = forwardKinematics();
    return Utils::transformToPose(transform);
}

bool RobotArm::inverseKinematics(const Pose& targetPose, std::vector<double>& solution)
{
    // Implémentation de la cinématique inverse par la méthode de Jacobienne inverse
    const int maxIterations = 100;
    const double epsilon = 1e-6;
    const double damping = 0.1;  // Facteur d'amortissement pour la stabilité

    // Initialiser avec les valeurs actuelles
    solution = getJointValues();

    Transform targetTransform = Utils::poseToTransform(targetPose);

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Position actuelle de l'effecteur
        Transform currentTransform = forwardKinematics();
        Pose currentPose = Utils::transformToPose(currentTransform);

        // Différence de position/orientation
        Pose error = targetPose - currentPose;

        // Si l'erreur est suffisamment petite, on considère la solution comme trouvée
        if (error.norm() < epsilon)
        {
            return true;
        }

        // Calculer la jacobienne
        Jacobian J = calculateJacobian();

        // Calculer la pseudo-inverse amortie (méthode de Levenberg-Marquardt)
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::MatrixXd damped = JtJ + damping * Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
        Eigen::MatrixXd invJtJ = damped.inverse();
        Eigen::MatrixXd Jpinv = invJtJ * J.transpose();

        // Calculer l'incrément des angles articulaires
        Eigen::VectorXd dTheta = Jpinv * error;

        // Mettre à jour les angles
        for (size_t i = 0; i < joints.size(); ++i)
        {
            solution[i] += dTheta(i);
        }

        // Appliquer les nouvelles valeurs
        setJointValues(solution);
    }

    // Si on arrive ici, c'est qu'on n'a pas trouvé de solution
    return false;
}

Jacobian RobotArm::calculateJacobian()
{
    const int numJoints = joints.size();
    Jacobian J(6, numJoints);

    // Position de l'effecteur
    Transform endEffectorTransform = endEffector->getWorldTransform();
    Eigen::Vector3d endPos = endEffectorTransform.block<3, 1>(0, 3);

    for (int i = 0; i < numJoints; ++i)
    {
        auto joint = joints[i];
        auto jointNode = joint->getNode();

        // Transformation du joint dans l'espace global
        Transform jointTransform = jointNode->getWorldTransform();

        // Position du joint
        Eigen::Vector3d jointPos = jointTransform.block<3, 1>(0, 3);

        // Orientation de l'axe du joint dans l'espace global
        Eigen::Vector3d jointAxis = jointTransform.block<3, 3>(0, 0) * joint->getAxis();

        if (joint->getType() == JointType::REVOLUTE)
        {
            // Contribution à la vélocité linéaire: cross(axis, (end - joint))
            Eigen::Vector3d r = endPos - jointPos;
            Eigen::Vector3d v = jointAxis.cross(r);

            // Remplir la jacobienne
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = jointAxis;
        }
        else if (joint->getType() == JointType::PRISMATIC)
        {
            // Contribution à la vélocité linéaire: axis
            J.block<3, 1>(0, i) = jointAxis;
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
    }

    return J;
}

std::shared_ptr<Joint> RobotArm::getJoint(const std::string& name)
{
    auto it = jointMap.find(name);
    if (it != jointMap.end())
    {
        return it->second;
    }
    return nullptr;
}

std::vector<double> RobotArm::getJointValues()
{
    std::vector<double> values;
    values.reserve(joints.size());

    for (const auto& joint : joints)
    {
        values.push_back(joint->getValue());
    }

    return values;
}

void RobotArm::setJointValues(const std::vector<double>& values)
{
    if (values.size() != joints.size())
    {
        throw std::invalid_argument("Number of values doesn't match number of joints");
    }

    for (size_t i = 0; i < joints.size(); ++i)
    {
        joints[i]->setValue(values[i]);
    }

    // Mettre à jour toutes les transformations
    rootNode->updateWorldTransform();
}

//------------------------------------------------------------------------------
// Implémentation des fonctions utilitaires
//------------------------------------------------------------------------------

namespace utils {

Eigen::Matrix3d eulerToRotation(double rx, double ry, double rz)
{
    Eigen::AngleAxisd rollAngle(rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rz, Eigen::Vector3d::UnitZ());

    return yawAngle.toRotationMatrix() * pitchAngle.toRotationMatrix() * rollAngle.toRotationMatrix();
}

Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& rot)
{
    return rot.eulerAngles(2, 1, 0).reverse();
}

Transform createTransform(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
{
    Transform transform = Transform::Identity();
    transform.block<3, 3>(0, 0) = rotation;
    transform.block<3, 1>(0, 3) = translation;
    return transform;
}

Transform createTransform(const Eigen::Vector3d& translation, double rx, double ry, double rz)
{
    return createTransform(translation, eulerToRotation(rx, ry, rz));
}

Eigen::Vector3d getTranslation(const Transform& transform)
{
    return transform.block<3, 1>(0, 3);
}

Eigen::Matrix3d getRotation(const Transform& transform)
{
    return transform.block<3, 3>(0, 0);
}

Pose transformToPose(const Transform& transform)
{
    Pose pose;
    pose.segment<3>(0) = getTranslation(transform);
    pose.segment<3>(3) = rotationToEuler(getRotation(transform));
    return pose;
}

Transform poseToTransform(const Pose& pose)
{
    return createTransform(pose.segment<3>(0), pose(3), pose(4), pose(5));
}

Transform dhTransform(double a, double alpha, double d, double theta)
{
    Transform transform = Transform::Identity();

    double cosTheta = cos(theta);
    double sinTheta = sin(theta);
    double cosAlpha = cos(alpha);
    double sinAlpha = sin(alpha);

    transform(0, 0) = cosTheta;
    transform(0, 1) = -sinTheta * cosAlpha;
    transform(0, 2) = sinTheta * sinAlpha;
    transform(0, 3) = a * cosTheta;

    transform(1, 0) = sinTheta;
    transform(1, 1) = cosTheta * cosAlpha;
    transform(1, 2) = -cosTheta * sinAlpha;
    transform(1, 3) = a * sinTheta;

    transform(2, 0) = 0;
    transform(2, 1) = sinAlpha;
    transform(2, 2) = cosAlpha;
    transform(2, 3) = d;

    return transform;
}

} // namespace utils
} // namespace robotik
