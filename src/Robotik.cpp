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
Node* Node::getNode(const std::string& p_name)
{
    // Check if any direct child has the name
    auto it = std::find_if(m_children.begin(),
                           m_children.end(),
                           [&p_name](const std::unique_ptr<Node>& p_node)
                           { return p_node->getName() == p_name; });

    if (it != m_children.end())
    {
        return it->get();
    }

    // If not found in direct children, search recursively
    for (const auto& child : m_children)
    {
        Node* result = child->getNode(p_name);
        if (result != nullptr)
        {
            return result;
        }
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
Joint::Joint(const std::string& p_name,
             Joint::Type p_type,
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
void Joint::setValue(Radian p_value)
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
Transform Joint::getTransform() const
{
    Transform transform = Transform::Identity();

    switch (m_type)
    {
        case Joint::Type::REVOLUTE:
        {
            // Create the rotation matrix around the axis
            Eigen::AngleAxisd rotation(m_value.to<double>(), m_axis);
            transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            break;
        }
        case Joint::Type::PRISMATIC:
        {
            // Translation along the axis
            transform.block<3, 1>(0, 3) = m_axis * m_value.to<double>();
            break;
        }
        case Joint::Type::FIXED:
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
void Joint::setNode(Node& p_node)
{
    m_node = &p_node;
    updateNodeTransform();
}

// ----------------------------------------------------------------------------
void RobotArm::setRootNode(Node::Ptr p_root)
{
    m_root_node = std::move(p_root);
}

// ----------------------------------------------------------------------------
void RobotArm::addJoint(Joint::Ptr p_joint)
{
    m_joint_map[p_joint->getName()] = p_joint.get();
    m_joints.push_back(std::move(p_joint));
}

// ----------------------------------------------------------------------------
void RobotArm::setEndEffector(Node& p_node)
{
    m_end_effector = &p_node;
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
                                 std::vector<Radian>& p_solution,
                                 size_t const p_max_iterations,
                                 double const p_epsilon,
                                 double const p_damping)
{
    // Initialize with the current values
    p_solution = getJointValues();

    for (size_t iter = 0; iter < p_max_iterations; ++iter)
    {
        // Current position of the end effector
        Transform current_transform = forwardKinematics();
        Pose current_pose = utils::transformToPose(current_transform);

        // Difference of position/orientation
        Pose error = p_target_pose - current_pose;

        // If the error is small enough, we consider the solution as found
        if (error.norm() < p_epsilon)
        {
            return true;
        }

        // Calculate the Jacobian
        Jacobian J = calculateJacobian();

        // Calculate the damped pseudo-inverse (Levenberg-Marquardt method)
        Eigen::MatrixXd JtJ = J.transpose() * J;
        Eigen::MatrixXd damped =
            JtJ + p_damping * Eigen::MatrixXd::Identity(JtJ.rows(),
                                                        JtJ.cols()); // NOLINT
        Eigen::MatrixXd inv_JtJ = damped.inverse();
        Eigen::MatrixXd Jpinv = inv_JtJ * J.transpose();

        // Calculate the increment of the joint angles
        Eigen::VectorXd dTheta = Jpinv * error;

        // Update the angles
        for (size_t i = 0; i < m_joints.size(); ++i)
        {
            p_solution[i] += Radian(dTheta(i));
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

        if (joint->getType() == Joint::Type::REVOLUTE)
        {
            // Contribution to linear velocity: cross(axis, (end - joint))
            Eigen::Vector3d r = end_pos - joint_pos;
            Eigen::Vector3d v = joint_axis.cross(r);

            // Fill the Jacobian
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = joint_axis;
        }
        else if (joint->getType() == Joint::Type::PRISMATIC)
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
    if (auto it = m_joint_map.find(p_name); it != m_joint_map.end())
    {
        return it->second;
    }
    return nullptr;
}

// ----------------------------------------------------------------------------
std::vector<Radian> RobotArm::getJointValues() const
{
    std::vector<Radian> values;
    values.reserve(m_joints.size());

    for (const auto& joint : m_joints)
    {
        values.push_back(joint->getValue());
    }

    return values;
}

// ----------------------------------------------------------------------------
void RobotArm::setJointValues(const std::vector<Radian>& p_values)
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

// ----------------------------------------------------------------------------
Node* RobotArm::getNode(const std::string& p_name) const
{
    if (!m_root_node)
    {
        return nullptr;
    }

    return m_root_node->getNode(p_name);
}

namespace utils
{

// ----------------------------------------------------------------------------
Eigen::Matrix3d eulerToRotation(Radian p_rx, Radian p_ry, Radian p_rz)
{
    Eigen::AngleAxisd roll_angle(p_rx.to<double>(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch_angle(p_ry.to<double>(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw_angle(p_rz.to<double>(), Eigen::Vector3d::UnitZ());

    return yaw_angle.toRotationMatrix() * pitch_angle.toRotationMatrix() *
           roll_angle.toRotationMatrix();
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
                          Radian p_rx,
                          Radian p_ry,
                          Radian p_rz)
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
    return createTransform(p_pose.segment<3>(0),
                           Radian(p_pose(3)),
                           Radian(p_pose(4)),
                           Radian(p_pose(5)));
}

// ----------------------------------------------------------------------------
Transform dhTransform(double p_a, Radian p_alpha, double p_d, Radian p_theta)
{
    Transform transform = Transform::Identity();

    double cos_theta = std::cos(p_theta.to<double>());
    double sin_theta = std::sin(p_theta.to<double>());
    double cos_alpha = std::cos(p_alpha.to<double>());
    double sin_alpha = std::sin(p_alpha.to<double>());

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
