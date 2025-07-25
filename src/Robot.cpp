#include "Robotik/Robot.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Exception.hpp"

#include <cmath>
namespace robotik
{

// ----------------------------------------------------------------------------
Robot::Robot(std::string_view const& p_name, scene::Node::Ptr p_root)
    : m_name(p_name)
{
    root(std::move(p_root));
}

// ----------------------------------------------------------------------------
void Robot::root(scene::Node::Ptr p_root)
{
    m_root_node = std::move(p_root);
    cacheListOfJoints();
}

// ----------------------------------------------------------------------------
void Robot::cacheListOfJoints()
{
    m_joints.clear();
    if (m_root_node == nullptr)
    {
        return;
    }

    m_root_node->traverse(
        [this](scene::Node& p_node, size_t /*p_depth*/)
        {
            if (auto joint = dynamic_cast<Joint*>(&p_node))
            {
                // Only cache actuable joints (REVOLUTE and PRISMATIC)
                if (joint->type() == Joint::Type::REVOLUTE ||
                    joint->type() == Joint::Type::PRISMATIC)
                {
                    m_joints.push_back(joint);
                }
            }
        });
}

// ----------------------------------------------------------------------------
std::vector<double> Robot::inverseKinematics(Pose const& p_target_pose,
                                             scene::Node const& p_end_effector,
                                             size_t const p_max_iterations,
                                             double const p_epsilon,
                                             double const p_damping)
{
    // Initialize with the current values
    std::vector<double> solution = jointValues();

    for (size_t iter = 0; iter < p_max_iterations; ++iter)
    {
        // Current position of the end effector
        Transform current_transform = p_end_effector.worldTransform();
        Pose current_pose = utils::transformToPose(current_transform);

        // Difference of position/orientation
        Pose error = p_target_pose - current_pose;

        // If the error is small enough, we consider the solution as found
        if (error.norm() < p_epsilon)
        {
            return solution;
        }

        // Calculate the Jacobian
        Jacobian J = calculateJacobian(p_end_effector);

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
            solution[i] += double(dTheta(i));
        }

        // Apply the new values
        setJointValues(solution);
    }

    // If we arrive here, it means we haven't found a solution
    return {};
}

// ----------------------------------------------------------------------------
Jacobian Robot::calculateJacobian(scene::Node const& p_end_effector) const
{
    const size_t num_joints = m_joints.size();
    Jacobian J(6, num_joints);

    // Position of the end effector
    Transform end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < num_joints; ++i)
    {
        auto const& joint = *m_joints[i];

        // Transformation of the joint in the global space
        Transform joint_transform = joint.worldTransform();

        // Position of the joint
        Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

        // Orientation of the joint axis in the global space
        Eigen::Vector3d joint_axis =
            joint_transform.block<3, 3>(0, 0) * joint.axis();

        if (joint.type() == Joint::Type::REVOLUTE)
        {
            // Contribution to linear velocity: cross(axis, (end - joint))
            Eigen::Vector3d r = end_pos - joint_pos;
            Eigen::Vector3d v = joint_axis.cross(r);

            // Fill the Jacobian
            J.block<3, 1>(0, i) = v;
            J.block<3, 1>(3, i) = joint_axis;
        }
        else if (joint.type() == Joint::Type::PRISMATIC)
        {
            // Contribution to linear velocity: axis
            J.block<3, 1>(0, i) = joint_axis;
            J.block<3, 1>(3, i) = Eigen::Vector3d::Zero();
        }
    }

    return J;
}

// ----------------------------------------------------------------------------
std::vector<double> Robot::jointValues() const
{
    std::vector<double> values;
    values.reserve(m_joints.size());

    for (const auto& joint : m_joints)
    {
        values.push_back(joint->position());
    }

    return values;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::jointNames() const
{
    std::vector<std::string> names;
    names.reserve(m_joints.size());

    for (const auto& joint : m_joints)
    {
        names.push_back(joint->name());
    }

    return names;
}

// ----------------------------------------------------------------------------
bool Robot::setJointValues(std::vector<double> const& p_values)
{
    if (p_values.size() != m_joints.size())
    {
        return false;
    }

    // FIXME: to be optimized: each children node will be updated.
    // Could be optimized to prevent updating and just call the root to be
    // updated.
    for (size_t i = 0; i < m_joints.size(); ++i)
    {
        m_joints[i]->position(p_values[i]);
    }

    return true;
}

} // namespace robotik
