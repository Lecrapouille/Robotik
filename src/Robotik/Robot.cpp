#include "Robotik/Robot.hpp"
#include "Robotik/private/Conversions.hpp"
#include "Robotik/private/Exception.hpp"
#include "Robotik/private/Joint.hpp"
#include "Robotik/private/Link.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

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
    markJointsCacheDirty();
}

// ----------------------------------------------------------------------------
void Robot::cacheListOfJoints() const
{
    if (!m_joints_cache_dirty)
    {
        return; // Cache is still valid
    }

    m_joints.clear();
    m_joints_map.clear();

    if (m_root_node == nullptr)
    {
        m_joints_cache_dirty = false;
        return;
    }

    m_root_node->traverse(
        [this](scene::Node& p_node, size_t /*p_depth*/)
        {
            if (auto joint = dynamic_cast<Joint*>(&p_node))
            {
                // Only cache actuable joints
                if (joint->type() == Joint::Type::REVOLUTE ||
                    joint->type() == Joint::Type::CONTINUOUS ||
                    joint->type() == Joint::Type::PRISMATIC)
                {
                    m_joints_map.try_emplace(joint->name(), std::ref(*joint));
                    m_joints.emplace_back(std::ref(*joint));
                }
            }
        });

    m_joints_cache_dirty = false;
}

// ----------------------------------------------------------------------------
Pose Robot::calculatePoseError(Pose const& p_target_pose,
                               Pose const& p_current_pose) const
{
    Pose error;

    // Position error is straightforward
    error.head<3>() = p_target_pose.head<3>() - p_current_pose.head<3>();

    // Orientation error: convert to rotation matrices for proper subtraction
    Eigen::Vector3d target_euler = p_target_pose.tail<3>();
    Eigen::Vector3d current_euler = p_current_pose.tail<3>();

    Eigen::Matrix3d target_rot = utils::eulerToRotation(
        target_euler(0), target_euler(1), target_euler(2));
    Eigen::Matrix3d current_rot = utils::eulerToRotation(
        current_euler(0), current_euler(1), current_euler(2));

    // Compute rotation error: R_error = R_target * R_current^T
    Eigen::Matrix3d error_rot = target_rot * current_rot.transpose();

    // Convert back to Euler angles (axis-angle would be better but this is
    // simpler)
    Eigen::Vector3d error_euler = utils::rotationToEuler(error_rot);
    error.tail<3>() = error_euler;

    return error;
}

// ----------------------------------------------------------------------------
std::vector<double> Robot::inverseKinematics(Pose const& p_target_pose,
                                             scene::Node const& p_end_effector,
                                             size_t const p_max_iterations,
                                             double const p_epsilon,
                                             double const p_damping)
{
    // Validate input parameters
    if (p_max_iterations == 0)
    {
        throw RobotikException("Invalid max_iterations: must be > 0");
    }
    if (p_epsilon <= 0.0)
    {
        throw RobotikException("Invalid epsilon: must be > 0");
    }
    if (p_damping < 0.0)
    {
        throw RobotikException("Invalid damping: must be >= 0");
    }

    // Ensure joints cache is up to date
    cacheListOfJoints();

    if (m_joints.empty())
    {
        throw RobotikException("No actuable joints found in robot");
    }

    // Initialize with the current values
    std::vector<double> solution = jointValues();
    const size_t num_joints = m_joints.size();

    for (size_t iter = 0; iter < p_max_iterations; ++iter)
    {
        // Current position of the end effector
        Transform current_transform = p_end_effector.worldTransform();
        Pose current_pose = utils::transformToPose(current_transform);

        // Calculate pose error (proper handling of orientation)
        Pose error = calculatePoseError(p_target_pose, current_pose);

        // If the error is small enough, we consider the solution as found
        if (error.norm() < p_epsilon)
        {
            return solution;
        }

        // Calculate the Jacobian
        Jacobian J = calculateJacobian(p_end_effector);

        // Use SVD for robust pseudo-inverse calculation
        Eigen::JacobiSVD<Jacobian> svd(
            J, Eigen::ComputeThinU | Eigen::ComputeThinV);

        // Handle singularities by filtering small singular values
        const double singular_threshold = 1e-6;
        Eigen::VectorXd singular_values = svd.singularValues();
        Eigen::VectorXd inv_singular_values(singular_values.size());

        for (int i = 0; i < singular_values.size(); ++i)
        {
            if (singular_values(i) > singular_threshold)
            {
                inv_singular_values(i) =
                    1.0 / (singular_values(i) * singular_values(i) +
                           p_damping * p_damping);
                inv_singular_values(i) *= singular_values(i);
            }
            else
            {
                inv_singular_values(i) = 0.0;
            }
        }

        // Compute damped pseudo-inverse: J+ = V * S+ * U^T
        Jacobian Jpinv = svd.matrixV() * inv_singular_values.asDiagonal() *
                         svd.matrixU().transpose();

        // Calculate the increment of the joint angles
        Eigen::VectorXd dTheta = Jpinv * error;

        // Apply joint limits and update angles
        bool limits_violated = false;
        for (size_t i = 0; i < num_joints; ++i)
        {
            double new_value = solution[i] + double(dTheta(i));
            Joint const& joint = m_joints[i].get();
            auto [min_limit, max_limit] = joint.limits();

            // Check joint limits for non-continuous joints
            if (joint.type() != Joint::Type::CONTINUOUS)
            {
                if (new_value < min_limit || new_value > max_limit)
                {
                    // Clamp to limits
                    new_value = std::clamp(new_value, min_limit, max_limit);
                    limits_violated = true;
                }
            }

            solution[i] = new_value;
        }

        // Apply the new values
        if (!setJointValues(solution))
        {
            throw RobotikException(
                "Failed to set joint values during IK iteration");
        }

        // If we hit joint limits, we might need more iterations or different
        // approach
        if (limits_violated && iter > p_max_iterations / 2)
        {
            // Try to continue but with reduced step size
            for (size_t i = 0; i < num_joints; ++i)
            {
                solution[i] = solution[i] - 0.5 * double(dTheta(i));
            }
            setJointValues(solution);
        }
    }

    // If we arrive here, it means we haven't found a solution
    return {};
}

// ----------------------------------------------------------------------------
Jacobian Robot::calculateJacobian(scene::Node const& p_end_effector) const
{
    // Ensure joints cache is up to date
    cacheListOfJoints();

    const size_t num_joints = m_joints.size();
    Jacobian J(6, num_joints);

    // Position of the end effector
    Transform end_effector_transform = p_end_effector.worldTransform();
    Eigen::Vector3d end_pos = end_effector_transform.block<3, 1>(0, 3);

    for (size_t i = 0; i < num_joints; ++i)
    {
        Joint const& joint = m_joints[i].get();

        // Transformation of the joint in the global space
        Transform joint_transform = joint.worldTransform();

        // Position of the joint
        Eigen::Vector3d joint_pos = joint_transform.block<3, 1>(0, 3);

        // Orientation of the joint axis in the global space
        Eigen::Vector3d joint_axis =
            joint_transform.block<3, 3>(0, 0) * joint.axis();

        if ((joint.type() == Joint::Type::REVOLUTE) ||
            (joint.type() == Joint::Type::CONTINUOUS))
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
Joint const& Robot::joint(std::string_view const& p_name) const
{
    // Ensure joints cache is up to date
    cacheListOfJoints();

    auto const& it = m_joints_map.find(std::string(p_name));
    if (it == m_joints_map.end())
    {
        throw RobotikException("Joint not found: " + std::string(p_name));
    }
    return it->second.get();
}

// ----------------------------------------------------------------------------
std::vector<double> Robot::jointValues() const
{
    // Ensure joints cache is up to date
    cacheListOfJoints();

    std::vector<double> values;
    values.reserve(m_joints.size());

    for (const auto& joint_ref : m_joints)
    {
        values.push_back(joint_ref.get().position());
    }

    return values;
}

// ----------------------------------------------------------------------------
std::vector<std::string> Robot::jointNames() const
{
    // Ensure joints cache is up to date
    cacheListOfJoints();

    std::vector<std::string> names;
    names.reserve(m_joints.size());

    for (const auto& joint_ref : m_joints)
    {
        names.push_back(joint_ref.get().name());
    }

    return names;
}

// ----------------------------------------------------------------------------
bool Robot::setJointValues(std::vector<double> const& p_values)
{
    // Ensure joints cache is up to date
    cacheListOfJoints();

    if (p_values.size() != m_joints.size())
    {
        return false;
    }

    // Set all joint values first (avoid multiple transform updates)
    for (size_t i = 0; i < m_joints.size(); ++i)
    {
        m_joints[i].get().position(p_values[i]);
    }

    return true;
}

// ----------------------------------------------------------------------------
Joint const& Robot::findEndEffector() const
{
    if (!hasRoot())
    {
        throw RobotikException("Robot has no root");
    }

    Joint const* end_effector_joint = nullptr;
    size_t max_depth = 0;

    // Traverse the scene graph to find the joint at maximum depth
    m_root_node->traverse(
        [&](scene::Node const& p_node, size_t p_depth)
        {
            // Check if this is a Joint node
            if (auto joint = dynamic_cast<Joint const*>(&p_node))
            {
                // Check if this joint has no child joints (it's a terminal
                // joint)
                bool has_child_joints = false;
                p_node.traverse(
                    [&has_child_joints](scene::Node const& p_child,
                                        size_t /*p_child_depth*/)
                    {
                        if (dynamic_cast<Joint const*>(&p_child))
                        {
                            has_child_joints = true;
                        }
                    });

                // If this joint has no child joints, it's a candidate for end
                // effector
                if (!has_child_joints && p_depth > max_depth)
                {
                    max_depth = p_depth;
                    end_effector_joint = joint;
                }
            }
        });

    // If no terminal joint found, look for the parent joint of the deepest link
    if (!end_effector_joint)
    {
        Link const* deepest_link = nullptr;
        max_depth = 0;

        m_root_node->traverse(
            [&](scene::Node const& p_node, size_t p_depth)
            {
                if (auto link = dynamic_cast<Link const*>(&p_node))
                {
                    // Check if this link has no child joints
                    bool has_child_joints = false;
                    p_node.traverse(
                        [&has_child_joints](scene::Node const& p_child,
                                            size_t /*p_child_depth*/)
                        {
                            if (dynamic_cast<Joint const*>(&p_child))
                            {
                                has_child_joints = true;
                            }
                        });

                    if (!has_child_joints && p_depth > max_depth)
                    {
                        max_depth = p_depth;
                        deepest_link = link;
                    }
                }
            });

        // Find the joint that connects to this deepest link
        if (deepest_link)
        {
            m_root_node->traverse(
                [&](scene::Node const& p_node, size_t /*p_depth*/)
                {
                    if (auto joint = dynamic_cast<Joint const*>(&p_node))
                    {
                        // Check if this joint has the deepest link as a child
                        for (auto const& child : joint->children())
                        {
                            if (child.get() == deepest_link)
                            {
                                end_effector_joint = joint;
                                return;
                            }
                        }
                    }
                });
        }
    }

    if (!end_effector_joint)
    {
        throw RobotikException("End effector not found");
    }

    return *end_effector_joint;
}

} // namespace robotik
