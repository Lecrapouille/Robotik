#pragma once

#include <Eigen/Dense>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ----------------------------------------------------------------------------
//! \brief Homogeneous transformation matrix (4x4) for 3D spatial
//! transformations.
//!
//! This 4x4 matrix encodes both rotation and translation in a single
//! representation: [R t] where R is 3x3 rotation matrix, t is 3x1 translation
//! vector [0 1] bottom row is always [0 0 0 1] for homogeneous coordinates
//!
//! Used for: Forward kinematics, coordinate frame transformations, pose
//! composition.
// ----------------------------------------------------------------------------
using Transform = Eigen::Matrix4d;

// ----------------------------------------------------------------------------
//! \brief 6D pose vector representing position and orientation
//! (x,y,z,rx,ry,rz).
//!
//! This compact representation combines:
//! - Position: [x,y,z] - 3D coordinates in meters
//! - Orientation: [rx,ry,rz] - Euler angles in radians (roll, pitch, yaw)
//!
//! Used for: End-effector targets, trajectory waypoints, inverse kinematics
//! goals.
// ----------------------------------------------------------------------------
using Pose = Eigen::Matrix<double, 6, 1>;

// ----------------------------------------------------------------------------
//! \brief Jacobian matrix relating joint velocities to end-effector velocity.
//!
//! This matrix (typically 6×n for n joints) provides the linear relationship:
//! end_effector_velocity = J * joint_velocities
//!
//! Used for: Velocity control, force control, singularity analysis, inverse
//! kinematics
// ----------------------------------------------------------------------------
using Jacobian = Eigen::MatrixXd;

// ****************************************************************************
//! \brief Class representing a node in the scene graph.
//!
//! In robotics, a scene graph is a hierarchical data structure that represents
//! the spatial relationships between different components of a robot. Each node
//! in this graph represents a physical or logical component that has:
//! - A position and orientation in 3D space (transform)
//! - A relationship to a parent component (inheritance of transforms)
//! - Potentially multiple child components
//!
//! Physical representation in robotics:
//! - Robot links (rigid bodies connecting joints)
//! - Joint frames (coordinate systems at joint locations)
//! - Tool frames (end-effector coordinate systems)
//! - Sensor mounts (camera, lidar attachment points)
//! - Collision geometries (for path planning)
//!
//! The scene graph enables:
//! - Forward kinematics calculation (computing end-effector pose)
//! - Coordinate frame transformations between robot components
//! - Hierarchical motion propagation (parent motion affects all children)
//! - Efficient spatial queries and collision detection
//!
//! Example robot hierarchy:
//! Base -> Shoulder -> Upper_Arm -> Elbow -> Forearm -> Wrist -> End_Effector
// ****************************************************************************
class Node
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor. Initializes the node with a name and transformation
    //! to identity matrix.
    //! \param p_name Name of the node.
    // ------------------------------------------------------------------------
    explicit Node(const std::string& p_name);

    // ------------------------------------------------------------------------
    //! \brief Create and store a new child node.
    //! \tparam T Type of the child node should inherit from Node.
    //! \param p_args Arguments for the child node constructor.
    //! \return Reference to the added child node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    T& addChild(Args&&... p_args)
    {
        static_assert(std::is_base_of_v<Node, T>, "T must inherit from Node");

        auto child = std::make_unique<T>(std::forward<Args>(p_args)...);
        T& child_ref = *child;

        m_children.push_back(std::move(child));
        child_ref.m_parent = this;
        child_ref.markDirty();

        return child_ref;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children of the node.
    //! \return Vector of child nodes.
    // ------------------------------------------------------------------------
    const std::vector<std::unique_ptr<Node>>& getChildren() const;

    // ------------------------------------------------------------------------
    //! \brief Set the local transform of the node.
    //!
    //! The local transform defines the position and orientation of this node
    //! relative to its parent in the hierarchy. In the context of robotics,
    //! this typically corresponds to:
    //! - For a link: its position/orientation relative to the parent joint
    //! - For a joint: the transformation it applies (rotation/translation)
    //!
    //! This 4x4 homogeneous matrix encodes:
    //! - Translation (first 3 elements of the 4th column)
    //! - Rotation (3x3 upper left submatrix)
    //! - Scale factor (typically 1 in robotics)
    //!
    //! \param p_transform Local homogeneous transformation matrix 4x4
    // ------------------------------------------------------------------------
    void setLocalTransform(const Transform& p_transform);

    // ------------------------------------------------------------------------
    //! \brief Get the local transform of the node.
    //!
    //! The local transform describes the position and orientation of this node
    //! relative to its parent in the hierarchy. This transformation is
    //! expressed in the parent's coordinate frame.
    //!
    //! In robotics, this transformation represents:
    //! - The geometric offset between two consecutive links
    //! - The transformation applied by a joint (rotation or translation)
    //! - The physical dimensions of the robot's links
    //!
    //! \return Constant reference to the local transformation matrix 4x4
    // ------------------------------------------------------------------------
    const Transform& getLocalTransform() const;

    // ------------------------------------------------------------------------
    //! \brief Get the world transform of the node.
    //!
    //! The world transform describes the absolute position and orientation of
    //! this node in the global/base coordinate frame. This transformation is
    //! calculated by multiplying all local transforms from the root to this
    //! node.
    //!
    //! In robotics, this transformation represents:
    //! - Position/orientation of a link in the robot's workspace
    //! - End-effector pose relative to the robot's base
    //! - Absolute configuration of any part of the robot
    //!
    //! The world transform = T_base * T_joint1 * ... * T_jointN * T_local
    //! where each T_jointX is
    // ------------------------------------------------------------------------
    const Transform& getWorldTransform();

    // ------------------------------------------------------------------------
    //! \brief Update the world transform of the node.
    //!
    //! Updates the world transform of this node and all its children
    //! recursively. This method implements the propagation of transforms in
    //! the robot's kinematic chain.
    //!
    //! The calculation follows the formula:
    //! - If root node: T_world = T_local
    //! - Otherwise: T_world = T_world_parent * T_local
    //!
    //! In robotics, this update is necessary when:
    //! - A joint changes value (angle or position)
    //! - The robot's configuration is modified
    //! - The direct kinematics is recalculated
    //!
    //! The algorithm uses a cache system ("dirty flag") to avoid unnecessary
    //! recalculations and optimize performance.
    // ------------------------------------------------------------------------
    void updateWorldTransform();

    // ------------------------------------------------------------------------
    //! \brief Mark the node as dirty.
    //!
    //! Marks the node as dirty, indicating that its world transform needs to be
    //! recalculated. This is used to optimize performance by avoiding
    //! unnecessary recalculations.
    // ------------------------------------------------------------------------
    void markDirty();

    // ------------------------------------------------------------------------
    //! \brief Get the name of the node.
    //! \return Name of the node.
    // ------------------------------------------------------------------------
    const std::string& getName() const;

private:

    std::string m_name;
    Transform m_local_transform;
    Transform m_world_transform;
    std::vector<std::unique_ptr<Node>> m_children;
    Node* m_parent = nullptr;
    bool m_is_dirty = true;
};

// *********************************************************************************
//! \brief Class representing a robotic joint.
//!
//! In robotics, a joint is a mechanical connection between two rigid bodies
//! (links) that allows controlled motion between them. Joints are the actuated
//! components that give robots their ability to move and manipulate objects.
//!
//! Physical characteristics:
//! - REVOLUTE joints: Allow rotation around a fixed axis (like elbow, shoulder)
//!   * Typical range: -π to +π radians (or limited by mechanical constraints)
//!   * Examples: Robot arm joints, wheel steering mechanisms
//!
//! - PRISMATIC joints: Allow linear translation along a fixed axis (like
//! pistons)
//!   * Typical range: 0 to max_extension (in meters)
//!   * Examples: Linear actuators, elevator mechanisms, grippers
//!
//! - FIXED joints: No movement, rigid connection between links
//!   * Used for: Tool attachments, sensor mounts, structural connections
//!
//! Each joint has:
//! - A motion axis (3D vector defining rotation/translation direction)
//! - Current position/angle value (the joint's configuration)
//! - Motion limits (safety and mechanical constraints)
//! - Associated Node for transform propagation in the scene graph
//!
//! The joint transforms the coordinate frame from parent link to child link
//! based on its current value and type, enabling forward kinematics
//! computation.
// *********************************************************************************
class Joint
{
public:

    // ------------------------------------------------------------------------
    //! \brief Enumeration for the different types of robotic joints.
    //!
    //! This defines the fundamental types of mechanical connections in
    //! robotics:
    //!
    //! REVOLUTE: Rotational joint around a fixed axis
    //! - Degrees of freedom: 1 (rotation angle)
    //! - Motion: Circular around axis vector
    //! - Examples: Elbow, shoulder, wrist rotation
    //! - Typical actuators: Servo motors, stepper motors
    //!
    //! PRISMATIC: Linear translation joint along a fixed axis
    //! - Degrees of freedom: 1 (linear position)
    //! - Motion: Straight line along axis vector
    //! - Examples: Linear actuators, pneumatic pistons, telescoping arms
    //! - Typical actuators: Linear motors, hydraulic cylinders
    //!
    //! FIXED: Rigid connection with no movement
    //! - Degrees of freedom: 0 (completely constrained)
    //! - Motion: None (permanent rigid attachment)
    //! - Examples: Tool mounting, sensor brackets, structural connections
    //! - Purpose: Maintain fixed spatial relationships between components
    // ------------------------------------------------------------------------
    enum class JointType
    {
        REVOLUTE,  // Revolute joint - rotation around axis
        PRISMATIC, // Prismatic joint - translation along axis
        FIXED      // Fixed joint - no movement, rigid connection
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor.
    //! \param p_name Name of the joint.
    //! \param p_type Type of the joint.
    //! \param p_axis Axis of the joint.
    // ------------------------------------------------------------------------
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
    void setNode(Node* p_node);
    Node* getNode() const;
    const std::string& getName() const;

private:

    std::string m_name;
    JointType m_type;
    double m_value;
    double m_min;
    double m_max;
    Eigen::Vector3d m_axis;
    Node* m_node = nullptr;
};

// *********************************************************************************
//! \brief Class representing a complete robotic arm.
//!
//! This class encapsulates an entire robotic manipulator system, combining
//! the kinematic chain (joints and links) with high-level control and analysis
//! capabilities. It serves as the main interface for robot control and
//! simulation.
//!
//! Key components:
//! - Kinematic chain: Hierarchical structure of joints and links
//! - Root node: Base frame of the robot (typically fixed to ground/table)
//! - End-effector: Final link where tools/grippers are attached
//! - Joint collection: All actuated joints that define robot configuration
//!
//! Main capabilities:
//!
//! 1. FORWARD KINEMATICS: Given joint angles → compute end-effector pose
//!    * Essential for: Path planning, collision avoidance, visualization
//!    * Formula: T_end = T_base * ∏(T_joint_i) from base to end-effector
//!
//! 2. INVERSE KINEMATICS: Given desired end-effector pose → compute joint
//! angles
//!    * Essential for: Task-space control, trajectory following
//!    * Challenging due to: Multiple solutions, singularities, joint limits
//!
//! 3. JACOBIAN COMPUTATION: Relationship between joint velocities and
//! end-effector velocity
//!    * Essential for: Velocity control, force control, singularity analysis
//!    * J = ∂pose/∂joints (6×n matrix for n joints)
//!
//! 4. CONFIGURATION MANAGEMENT: Set/get joint positions, enforce limits
//!    * Essential for: Robot control, safety, trajectory execution
//!
//! This class bridges the gap between low-level joint control and high-level
//! task planning, providing the mathematical foundation for robot manipulation.
// *********************************************************************************
class RobotArm
{
public:

    explicit RobotArm(const std::string& p_name);

    // Robot configuration
    void setRootNode(std::unique_ptr<Node> p_root);
    void addJoint(std::unique_ptr<Joint> p_joint);
    void setEndEffector(Node* p_node);

    // Forward kinematics
    Transform forwardKinematics() const;
    Pose getEndEffectorPose() const;

    // Inverse kinematics
    bool inverseKinematics(const Pose& targetPose,
                           std::vector<double>& solution);

    // Jacobian calculation
    Jacobian calculateJacobian() const;

    // Access to joints
    Joint* getJoint(const std::string& p_name) const;
    std::vector<double> getJointValues() const;
    void setJointValues(const std::vector<double>& p_values);
    Node* getRootNode() const;

private:

    std::string m_name;
    std::unique_ptr<Node> m_root_node;
    std::vector<std::unique_ptr<Joint>> m_joints;
    std::unordered_map<std::string, Joint*> m_joint_map;
    Node* m_end_effector = nullptr;
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
