#pragma once

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

//! \brief Representation of a homogeneous transformation 4x4
using Transform = Eigen::Matrix4d;
//! \brief Vector 6D to represent a position/orientation (x,y,z,rx,ry,rz)
using Pose = Eigen::Matrix<double, 6, 1>;
//! \brief Jacobian
using Jacobian = Eigen::MatrixXd;

// *********************************************************************************
//! \brief Class for the different types of joints.
// *********************************************************************************
enum class JointType
{
    REVOLUTE,  // Revolute joint
    PRISMATIC, // Prismatic joint
    FIXED      // Fixed joint (no movement)
};

// *********************************************************************************
//! \brief Class representing a node in the scene graph.
// *********************************************************************************
class Node: public std::enable_shared_from_this<Node>
{
public:

    explicit Node(const std::string& p_name);

    // Hierarchy management
    void addChild(std::shared_ptr<Node> p_child);
    void removeChild(const std::string& p_child_name);
    std::shared_ptr<Node> getChild(const std::string& p_child_name);

    // Transformation management
    void setLocalTransform(const Transform& p_transform);
    const Transform& getLocalTransform() const;
    const Transform& getWorldTransform();

    // Update transformations
    void updateWorldTransform();
    void markDirty();

    const std::string& getName() const;

private:

    std::string m_name;
    Transform m_local_transform;
    Transform m_world_transform;
    std::vector<std::shared_ptr<Node>> m_children;
    std::weak_ptr<Node> m_parent;
    bool m_is_dirty = true;
};

// *********************************************************************************
//! \brief Class representing a robotic joint.
// *********************************************************************************
class Joint
{
public:

    Joint(const std::string& p_name,
          JointType p_type,
          const Eigen::Vector3d& p_axis);

    void setValue(double p_value);
    double getValue() const;
    void setLimits(double p_min, double p_max);

    Transform getTransform() const;
    void updateNodeTransform();

    JointType getType() const;
    const Eigen::Vector3d& getAxis() const;
    void setNode(std::shared_ptr<Node> p_node);
    std::shared_ptr<Node> getNode() const;
    const std::string& getName() const;

private:

    std::string m_name;
    JointType m_type;
    double m_value;
    double m_min;
    double m_max;
    Eigen::Vector3d m_axis;
    std::shared_ptr<Node> m_node;
};

// *********************************************************************************
//! \brief Class representing a complete robotic arm.
// *********************************************************************************
class RobotArm
{
public:

    explicit RobotArm(const std::string& p_name);

    // Robot configuration
    void setRootNode(std::shared_ptr<Node> p_root);
    void addJoint(std::shared_ptr<Joint> p_joint);
    void setEndEffector(std::shared_ptr<Node> p_node);

    // Forward kinematics
    Transform forwardKinematics() const;
    Pose getEndEffectorPose() const;

    // Inverse kinematics
    bool inverseKinematics(const Pose& targetPose,
                           std::vector<double>& solution);

    // Jacobian calculation
    Jacobian calculateJacobian() const;

    // Access to joints
    std::shared_ptr<Joint> getJoint(const std::string& p_name) const;
    std::vector<double> getJointValues() const;
    void setJointValues(const std::vector<double>& p_values);

private:

    std::string m_name;
    std::shared_ptr<Node> m_root_node;
    std::vector<std::shared_ptr<Joint>> m_joints;
    std::unordered_map<std::string, std::shared_ptr<Joint>> m_joint_map;
    std::shared_ptr<Node> m_end_effector;
};

//  Utility functions for conversions.
namespace utils
{

// Conversion between rotation matrices and angle representations.
Eigen::Matrix3d eulerToRotation(double p_rx, double p_ry, double p_rz);
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& p_rot);

// Conversions for homogeneous transformations
Transform createTransform(const Eigen::Vector3d& p_translation,
                          const Eigen::Matrix3d& p_rotation);
Transform createTransform(const Eigen::Vector3d& p_translation,
                          double p_rx,
                          double p_ry,
                          double p_rz);

// Extraction of data from transformations
Eigen::Vector3d getTranslation(const Transform& p_transform);
Eigen::Matrix3d getRotation(const Transform& p_transform);
Pose transformToPose(const Transform& p_transform);
Transform poseToTransform(const Pose& p_pose);

// DH (Denavit-Hartenberg) conversion
Transform dhTransform(double p_a, double p_alpha, double p_d, double p_theta);
} // namespace utils
} // namespace robotik
