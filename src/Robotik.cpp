#include "Robotik/Robotik.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robotik
{

// ----------------------------------------------------------------------------
Node::Node(const std::string& p_name) : m_name(p_name)
{
    m_local_transform = Transform::Identity();
    m_world_transform = Transform::Identity();
}

// ----------------------------------------------------------------------------
Node* Node::getChild(const std::string& p_child_name)
{
    auto it = std::find_if(m_children.begin(),
                           m_children.end(),
                           [&p_child_name](const std::unique_ptr<Node>& p_child)
                           { return p_child->getName() == p_child_name; });

    if (it != m_children.end())
    {
        return it->get();
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
void Node::setLocalTransform(const Transform& p_transform)
{
    m_local_transform = p_transform;
    markDirty();
}

// ----------------------------------------------------------------------------
const Transform& Node::getLocalTransform() const
{
    return m_local_transform;
}

// ----------------------------------------------------------------------------
const Transform& Node::getWorldTransform()
{
    if (m_is_dirty)
    {
        updateWorldTransform();
    }
    return m_world_transform;
}

// ----------------------------------------------------------------------------
void Node::updateWorldTransform()
{
    if (m_parent)
    {
        m_world_transform = m_parent->getWorldTransform() * m_local_transform;
    }
    else
    {
        m_world_transform = m_local_transform;
    }

    m_is_dirty = false;

    // Update the transformations of the children
    for (auto const& child : m_children)
    {
        child->updateWorldTransform();
    }
}

// ----------------------------------------------------------------------------
void Node::markDirty()
{
    m_is_dirty = true;
    for (auto const& child : m_children)
    {
        child->markDirty();
    }
}

// ----------------------------------------------------------------------------
const std::string& Node::getName() const
{
    return m_name;
}

// ----------------------------------------------------------------------------
const std::vector<std::unique_ptr<Node>>& Node::getChildren() const
{
    return m_children;
}

// ----------------------------------------------------------------------------
Joint::Joint(const std::string& p_name,
             JointType p_type,
             const Eigen::Vector3d& p_axis)
    : m_name(p_name),
      m_type(p_type),
      m_value(0.0),
      m_min(-M_PI),
      m_max(M_PI),
      m_axis(p_axis.normalized())
{
}

// ----------------------------------------------------------------------------
void Joint::setValue(double p_value)
{
    // Apply the limits
    if (p_value < m_min)
        p_value = m_min;
    if (p_value > m_max)
        p_value = m_max;

    m_value = p_value;

    // Update the transformation of the node
    if (m_node)
    {
        updateNodeTransform();
    }
}

// ----------------------------------------------------------------------------
double Joint::getValue() const
{
    return m_value;
}

// ----------------------------------------------------------------------------
void Joint::setLimits(double p_min, double p_max)
{
    m_min = p_min;
    m_max = p_max;
}

// ----------------------------------------------------------------------------
Transform Joint::getTransform() const
{
    Transform transform = Transform::Identity();

    switch (m_type)
    {
        case JointType::REVOLUTE:
        {
            // Create the rotation matrix around the axis
            Eigen::AngleAxisd rotation(m_value, m_axis);
            transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            break;
        }
        case JointType::PRISMATIC:
        {
            // Translation along the axis
            transform.block<3, 1>(0, 3) = m_axis * m_value;
            break;
        }
        case JointType::FIXED:
            // No transformation for fixed joints
            break;
    }

    return transform;
}

// ----------------------------------------------------------------------------
void Joint::updateNodeTransform()
{
    m_node->setLocalTransform(getTransform());
}

// ----------------------------------------------------------------------------
JointType Joint::getType() const
{
    return m_type;
}

// ----------------------------------------------------------------------------
const Eigen::Vector3d& Joint::getAxis() const
{
    return m_axis;
}

// ----------------------------------------------------------------------------
void Joint::setNode(Node* p_node)
{
    m_node = p_node;
    updateNodeTransform();
}

// ----------------------------------------------------------------------------
Node* Joint::getNode() const
{
    return m_node;
}

// ----------------------------------------------------------------------------
const std::string& Joint::getName() const
{
    return m_name;
}

// ----------------------------------------------------------------------------
RobotArm::RobotArm(const std::string& p_name) : m_name(p_name) {}

// ----------------------------------------------------------------------------
void RobotArm::setRootNode(std::unique_ptr<Node> p_root)
{
    m_root_node = std::move(p_root);
}

// ----------------------------------------------------------------------------
void RobotArm::addJoint(std::unique_ptr<Joint> p_joint)
{
    m_joint_map[p_joint->getName()] = p_joint.get();
    m_joints.push_back(std::move(p_joint));
}

// ----------------------------------------------------------------------------
void RobotArm::setEndEffector(Node* p_node)
{
    m_end_effector = p_node;
}

// ----------------------------------------------------------------------------
Transform RobotArm::forwardKinematics() const
{
    if (!m_end_effector)
    {
        throw std::runtime_error("End effector not set");
    }

    // Ensure that all transformations are up to date
    m_root_node->updateWorldTransform();

    return m_end_effector->getWorldTransform();
}

// ----------------------------------------------------------------------------
Pose RobotArm::getEndEffectorPose() const
{
    Transform transform = forwardKinematics();
    return utils::transformToPose(transform);
}

// ----------------------------------------------------------------------------
bool RobotArm::inverseKinematics(const Pose& p_target_pose,
                                 std::vector<double>& p_solution)
{
    // Implementation of the inverse kinematics by the Jacobian method
    // inverse
    const int maxIterations =
        500; // Increased iterations for better convergence
    const double epsilon =
        1e-4; // More reasonable tolerance for robotics applications
    const double damping = 0.01; // Reduced damping for better convergence

    // Initialize with the current values
    p_solution = getJointValues();

    for (int iter = 0; iter < maxIterations; ++iter)
    {
        // Current position of the end effector
        Transform current_transform = forwardKinematics();
        Pose current_pose = utils::transformToPose(current_transform);

        // Difference of position/orientation
        Pose error = p_target_pose - current_pose;

        // If the error is small enough, we consider the solution as found
        if (error.norm() < epsilon)
        {
            return true;
        }

        // Calculate the Jacobian
        Jacobian J = calculateJacobian();

        // Calculate the damped pseudo-inverse (Levenberg-Marquardt method)
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::MatrixXd damped =
            JtJ + damping * Eigen::MatrixXd::Identity(JtJ.rows(),
                                                      JtJ.cols()); // NOLINT
        Eigen::MatrixXd inv_JtJ = damped.inverse();
        Eigen::MatrixXd Jpinv = inv_JtJ * J.transpose();

        // Calculate the increment of the joint angles
        Eigen::VectorXd dTheta = Jpinv * error;

        // Update the angles
        for (size_t i = 0; i < m_joints.size(); ++i)
        {
            p_solution[i] += dTheta(i);
        }

        // Apply the new values
        setJointValues(p_solution);
    }

    // If we arrive here, it means we haven't found a solution
    return false;
}

// ----------------------------------------------------------------------------
Jacobian RobotArm::calculateJacobian() const
{
    const size_t num_joints = m_joints.size();
    Jacobian J(6, num_joints);

    // Position of the end effector
    Transform end_effector_transform = m_end_effector->getWorldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < num_joints; ++i)
    {
        auto joint = m_joints[i].get();
        auto joint_node = joint->getNode();

        // Transformation of the joint in the global space
        Transform joint_transform = joint_node->getWorldTransform();

        // Position of the joint
        Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

        // Orientation of the joint axis in the global space
        Eigen::Vector3d joint_axis =
            joint_transform.block<3, 3>(0, 0) * joint->getAxis();

        if (joint->getType() == JointType::REVOLUTE)
        {
            // Contribution to linear velocity: cross(axis, (end - joint))
            Eigen::Vector3d r = end_pos - joint_pos;
            Eigen::Vector3d v = joint_axis.cross(r);

            // Fill the Jacobian
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = joint_axis;
        }
        else if (joint->getType() == JointType::PRISMATIC)
        {
            // Contribution to linear velocity: axis
            J.block<3, 1>(0, i) = joint_axis;
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
    }

    return J;
}

// ----------------------------------------------------------------------------
Joint* RobotArm::getJoint(const std::string& p_name) const
{
    auto it = m_joint_map.find(p_name);
    if (it != m_joint_map.end())
    {
        return it->second;
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
std::vector<double> RobotArm::getJointValues() const
{
    std::vector<double> values;
    values.reserve(m_joints.size());

    for (const auto& joint : m_joints)
    {
        values.push_back(joint->getValue());
    }

    return values;
}

// ----------------------------------------------------------------------------
void RobotArm::setJointValues(const std::vector<double>& p_values)
{
    if (p_values.size() != m_joints.size())
    {
        throw std::invalid_argument(
            "Number of values doesn't match number of joints");
    }

    for (size_t i = 0; i < m_joints.size(); ++i)
    {
        m_joints[i]->setValue(p_values[i]);
    }

    // Update all transformations
    m_root_node->updateWorldTransform();
}

// ----------------------------------------------------------------------------
Node* RobotArm::getRootNode() const
{
    return m_root_node.get();
}

namespace utils
{

// ----------------------------------------------------------------------------
Eigen::Matrix3d eulerToRotation(double p_rx, double p_ry, double p_rz)
{
    Eigen::AngleAxisd rollAngle(p_rx, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(p_ry, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(p_rz, Eigen::Vector3d::UnitZ());

    return yawAngle.toRotationMatrix() * pitchAngle.toRotationMatrix() *
           rollAngle.toRotationMatrix();
}

// ----------------------------------------------------------------------------
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& p_rot)
{
    return p_rot.eulerAngles(2, 1, 0).reverse();
}

// ----------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          const Eigen::Matrix3d& p_rotation)
{
    Transform transform = Transform::Identity();
    transform.block<3, 3>(0, 0) = p_rotation;
    transform.block<3, 1>(0, 3) = p_translation;
    return transform;
}

// ----------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          double p_rx,
                          double p_ry,
                          double p_rz)
{
    return createTransform(p_translation, eulerToRotation(p_rx, p_ry, p_rz));
}

// ----------------------------------------------------------------------------
Eigen::Vector3d getTranslation(const Transform& p_transform)
{
    return p_transform.block<3, 1>(0, 3);
}

// ----------------------------------------------------------------------------
Eigen::Matrix3d getRotation(const Transform& p_transform)
{
    return p_transform.block<3, 3>(0, 0);
}

// ----------------------------------------------------------------------------
Pose transformToPose(const Transform& p_transform)
{
    Pose pose;
    pose.segment<3>(0) = getTranslation(p_transform);
    pose.segment<3>(3) = rotationToEuler(getRotation(p_transform));
    return pose;
}

// ----------------------------------------------------------------------------
Transform poseToTransform(const Pose& p_pose)
{
    return createTransform(
        p_pose.segment<3>(0), p_pose(3), p_pose(4), p_pose(5));
}

// ----------------------------------------------------------------------------
Transform dhTransform(double p_a, double p_alpha, double p_d, double p_theta)
{
    Transform transform = Transform::Identity();

    double cos_theta = cos(p_theta);
    double sin_theta = sin(p_theta);
    double cos_alpha = cos(p_alpha);
    double sin_alpha = sin(p_alpha);

    transform(0, 0) = cos_theta;
    transform(0, 1) = -sin_theta * cos_alpha;
    transform(0, 2) = sin_theta * sin_alpha;
    transform(0, 3) = p_a * cos_theta;

    transform(1, 0) = sin_theta;
    transform(1, 1) = cos_theta * cos_alpha;
    transform(1, 2) = -cos_theta * sin_alpha;
    transform(1, 3) = p_a * sin_theta;

    transform(2, 0) = 0;
    transform(2, 1) = sin_alpha;
    transform(2, 2) = cos_alpha;
    transform(2, 3) = p_d;

    return transform;
}

} // namespace utils
} // namespace robotik
