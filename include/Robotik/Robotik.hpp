#pragma once

// #include "units.h"

#include <Eigen/Dense>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// ****************************************************************************
//! \brief Base exception class for all Robotik library exceptions.
// ****************************************************************************
class RobotikException: public std::runtime_error
{
public:

    explicit RobotikException(const std::string& message)
        : std::runtime_error("Robotik: " + message)
    {
    }
};

namespace robotik
{
// using namespace units::literals;
// using Radian = units::angle::double_t;
// using Meter = units::length::meter_t;

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
//! A scene graph is a hierarchical data structure that represents the spatial
//! relationships between its different components. Each node in this graph
//! represents a physical or logical component that has:
//! - A position and orientation in 3D space (transform).
//! - A relationship to a parent component (inheritance of transforms).
//! - Potentially multiple child components.
//!
//! Physical representation in robotics:
//! - Robot links (rigid bodies connecting joints).
//! - Joint frames (coordinate systems at joint locations).
//! - Tool frames (end-effector coordinate systems).
//! - Sensor mounts (camera, lidar attachment points).
//! - Collision geometries (for path planning).
//!
//! The scene graph enables:
//! - Forward kinematics calculation (computing end-effector pose).
//! - Coordinate frame transformations between robot components.
//! - Hierarchical motion propagation (parent motion affects all children).
//! - Efficient spatial queries and collision detection.
//!
//! Example robot hierarchy:
//! Base -> Shoulder -> Upper_Arm -> Elbow -> Forearm -> Wrist -> End_Effector
// ****************************************************************************
class Node
{
public:

    using Ptr = std::unique_ptr<Node>;

    // ------------------------------------------------------------------------
    //! \brief Constructor. Initializes the node with a name and transformation
    //! to identity matrix.
    //! \param p_name Name of the node.
    // ------------------------------------------------------------------------
    explicit Node(const std::string_view& p_name);

    // ------------------------------------------------------------------------
    //! \brief Virtual destructor to ensure proper cleanup of derived classes.
    // ------------------------------------------------------------------------
    virtual ~Node() = default;

    // ------------------------------------------------------------------------
    //! \brief Create a new node.
    //! \tparam T Type of the child node should inherit from Node.
    //! \param p_args Arguments for the child node constructor.
    //! \return Pointer to the new node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    static std::unique_ptr<T> create(Args&&... p_args)
    {
        static_assert(std::is_base_of<Node, T>::value,
                      "T must inherit from Node");
        return std::make_unique<T>(std::forward<Args>(p_args)...);
    }

    // ------------------------------------------------------------------------
    //! \brief Create and store a new child node.
    //! \tparam T Type of the child node should inherit from Node.
    //! \param p_args Arguments for the child node constructor.
    //! \return Reference to the added child node.
    // ------------------------------------------------------------------------
    template <typename T, typename... Args>
    T& createChild(Args&&... p_args)
    {
        auto child = Node::create<T>(std::forward<Args>(p_args)...);
        T& child_ref = *child;

        m_children.push_back(std::move(child));
        child_ref.m_parent = this;
        child_ref.markDirty();

        return child_ref;
    }

    // ------------------------------------------------------------------------
    //! \brief Add a child node created outside of this class.
    //! \param p_child Pointer to the child node.
    // ------------------------------------------------------------------------
    void addChild(Node::Ptr p_child)
    {
        m_children.push_back(std::move(p_child));
    }

    // ------------------------------------------------------------------------
    //! \brief Get the children of the node.
    //! \return Vector of child nodes.
    // ------------------------------------------------------------------------
    inline const std::vector<std::unique_ptr<Node>>& getChildren() const
    {
        return m_children;
    }

    // ------------------------------------------------------------------------
    //! \brief Get a node by name. This method is recursive and will search
    //! through the entire subtree rooted at this node.
    //! \param p_name Name of the node.
    //! \return Pointer to the node, or nullptr if not found.
    // ------------------------------------------------------------------------
    Node* getNode(const std::string_view& p_name);

    // ------------------------------------------------------------------------
    //! \brief Traverse the node tree recursively and apply a function to each
    //! node.
    //!
    //! This method performs a depth-first traversal of the node tree starting
    //! from this node. It applies the provided function to each node in the
    //! tree, including the current node and all its descendants.
    //!
    //! The traversal order is:
    //! 1. Apply function to current node
    //! 2. Recursively traverse all children
    //!
    //! This is useful for operations that need to be applied to all nodes
    //! in a subtree, such as:
    //! - Collecting all joints in a robot arm
    //! - Updating properties of all nodes
    //! - Searching for specific node types
    //!
    //! \tparam Function Type of the function to apply (lambda, function
    //! pointer, etc.)
    //! \param p_function Function to apply to each node. Should accept Node&
    //! parameter.
    // ------------------------------------------------------------------------
    template <typename Function>
    void traverse(Function&& p_function)
    {
        // Apply the function to the current node
        p_function(*this);

        // Recursively traverse all children
        for (const auto& child : m_children)
        {
            child->traverse(std::forward<Function>(p_function));
        }
    }

    // ------------------------------------------------------------------------
    //! \brief Set the local transform of the node.
    //!
    //! The local transform defines the position and orientation of this
    //! node relative to its parent in the hierarchy. In the context of
    //! robotics, this typically corresponds to:
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
    //! The local transform describes the position and orientation of this
    //! node relative to its parent in the hierarchy. This transformation is
    //! expressed in the parent's coordinate frame.
    //!
    //! In robotics, this transformation represents:
    //! - The geometric offset between two consecutive links
    //! - The transformation applied by a joint (rotation or translation)
    //! - The physical dimensions of the robot's links
    //!
    //! \return Constant reference to the local transformation matrix 4x4
    // ------------------------------------------------------------------------
    inline const Transform& getLocalTransform() const
    {
        return m_local_transform;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the world transform of the node.
    //!
    //! The world transform describes the absolute position and orientation
    //! of this node in the global/base coordinate frame. This
    //! transformation is calculated by multiplying all local transforms
    //! from the root to this node.
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
    //! The algorithm uses a cache system ("dirty flag") to avoid
    //! unnecessary recalculations and optimize performance.
    // ------------------------------------------------------------------------
    void updateWorldTransform();

    // ------------------------------------------------------------------------
    //! \brief Get the name of the node.
    //! \return Name of the node.
    // ------------------------------------------------------------------------
    inline const std::string& getName() const
    {
        return m_name;
    }

protected:

    // ------------------------------------------------------------------------
    //! \brief Mark the node as dirty.
    //!
    //! Marks the node as dirty, indicating that its world transform needs
    //! to be recalculated. This is used to optimize performance by avoiding
    //! unnecessary recalculations.
    // ------------------------------------------------------------------------
    void markDirty();

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
//! (links) that allows controlled motion between them. Joints are the
//! actuated components that give robots their ability to move and
//! manipulate objects.
//!
//! Physical characteristics:
//! - REVOLUTE joints: Allow rotation around a fixed axis (like elbow,
//! shoulder)
//!   * Typical range: -π to +π radians (or limited by mechanical
//!   constraints)
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
class Joint: public Node
{
public:

    using Ptr = std::unique_ptr<Joint>;

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
    enum class Type
    {
        REVOLUTE,  // Revolute joint - rotation around axis
        PRISMATIC, // Prismatic joint - translation along axis
        FIXED      // Fixed joint - no movement, rigid connection
    };

    // ------------------------------------------------------------------------
    //! \brief Constructor for robotic joint initialization.
    //!
    //! PHYSICS: Joint initialization establishes the kinematic and dynamic
    //! properties of a mechanical connection between two rigid bodies in a
    //! robotic system. The joint parameters define how motion is transmitted
    //! through the kinematic chain.
    //!
    //! For REVOLUTE joints:
    //! - Axis defines the instantaneous axis of rotation (Rodrigues' theorem)
    //! - Angular displacement follows θ(t) = θ₀ + ω*t for constant velocity
    //! - Kinetic energy: KE = ½*I*ω² where I is moment of inertia
    //! - Torque-angle relationship: τ = I*α (α = angular acceleration)
    //!
    //! For PRISMATIC joints:
    //! - Axis defines the direction of linear translation
    //! - Linear displacement follows x(t) = x₀ + v*t for constant velocity
    //! - Kinetic energy: KE = ½*m*v² where m is mass
    //! - Force-displacement relationship: F = m*a (a = linear acceleration)
    //!
    //! The axis vector must be normalized as it represents a pure direction
    //! in 3D space. The joint's configuration space is one-dimensional for
    //! both revolute and prismatic joints.
    //!
    //! \param p_name Unique identifier for the joint in the kinematic chain
    //! \param p_type Mechanical type defining the motion constraint
    //! \param p_axis Normalized 3D vector defining the motion axis
    // ------------------------------------------------------------------------
    Joint(const std::string_view& p_name,
          Type p_type,
          const Eigen::Vector3d& p_axis);

    // ------------------------------------------------------------------------
    //! \brief Set the joint's configuration value.
    //!
    //! PHYSICS: This method updates the joint's generalized coordinate,
    //! which represents the current state in the joint's configuration space.
    //!
    //! For REVOLUTE joints:
    //! - Value represents angular displacement θ from reference position
    //! - Units: radians (SI base unit for plane angles)
    //! - Range: typically [-π, π] or limited by mechanical constraints
    //! - Physical meaning: rotation angle about the joint axis
    //! - Transformation: T = Rot(axis, θ) using Rodrigues' rotation formula
    //!
    //! For PRISMATIC joints:
    //! - Value represents linear displacement d from reference position
    //! - Units: meters (SI base unit for length)
    //! - Range: [0, d_max] limited by actuator stroke length
    //! - Physical meaning: translation distance along the joint axis
    //! - Transformation: T = Trans(axis * d)
    //!
    //! The joint value directly affects the local transformation matrix
    //! that propagates through the kinematic chain during forward kinematics.
    //!
    //! \param p_value New joint configuration value in appropriate units
    // ------------------------------------------------------------------------
    void setValue(double p_value);

    // ------------------------------------------------------------------------
    //! \brief Get the current joint configuration value.
    //!
    //! PHYSICS: Returns the current generalized coordinate representing
    //! the joint's position in its configuration space.
    //!
    //! For REVOLUTE joints:
    //! - Returns angular position θ relative to reference configuration
    //! - Represents accumulated rotation about the joint axis
    //! - Used in forward kinematics: T = T_ref * Rot(axis, θ)
    //!
    //! For PRISMATIC joints:
    //! - Returns linear position d relative to reference configuration
    //! - Represents extension/retraction along the joint axis
    //! - Used in forward kinematics: T = T_ref * Trans(axis * d)
    //!
    //! This value is fundamental for:
    //! - Forward kinematics computation
    //! - Inverse kinematics solutions
    //! - Jacobian matrix calculation
    //! - Motion planning and control
    //!
    //! \return Current joint configuration value
    // ------------------------------------------------------------------------
    inline double getValue() const
    {
        return m_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Set joint motion limits for safety and mechanical constraints.
    //!
    //! PHYSICS: Joint limits represent physical and safety constraints that
    //! prevent damage to the robotic system and ensure safe operation.
    //!
    //! MECHANICAL LIMITS:
    //! - Hard stops: Physical barriers preventing further motion
    //! - Actuator limits: Maximum torque/force output capabilities
    //! - Structural limits: Material stress and fatigue considerations
    //!
    //! For REVOLUTE joints:
    //! - Angular limits prevent over-rotation and cable wrapping
    //! - Common ranges: [-π, π] for continuous rotation
    //! - Limited ranges: [-π/2, π/2] for elbow-like joints
    //! - Singularity avoidance: Limits may prevent kinematic singularities
    //!
    //! For PRISMATIC joints:
    //! - Linear limits prevent over-extension and collision
    //! - Stroke limits: [0, L_max] where L_max is actuator stroke
    //! - Workspace limits: Physical boundaries of the robot's reach
    //!
    //! SAFETY CONSIDERATIONS:
    //! - Soft limits: Programming boundaries with safety margins
    //! - Hard limits: Hardware switches and mechanical stops
    //! - Velocity limits: Maximum safe motion speeds
    //! - Acceleration limits: Maximum safe motion accelerations
    //!
    //! \param p_min Minimum allowed joint value
    //! \param p_max Maximum allowed joint value
    // ------------------------------------------------------------------------
    inline void setLimits(double p_min, double p_max)
    {
        m_min = p_min;
        m_max = p_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Compute the joint's local transformation matrix.
    //!
    //! PHYSICS: This method computes the 4x4 homogeneous transformation
    //! matrix that represents the spatial relationship between the joint's
    //! parent and child coordinate frames.
    //!
    //! MATHEMATICAL FOUNDATION:
    //! The transformation encodes both rotation and translation:
    //! T = [R  t]  where R is 3x3 rotation matrix, t is 3x1 translation
    //!     [0  1]
    //!
    //! For REVOLUTE joints:
    //! - R = Rot(axis, θ) using Rodrigues' rotation formula
    //! - R = I + sin(θ)*[axis]× + (1-cos(θ))*[axis]×²
    //! - Where [axis]× is the skew-symmetric matrix of the axis vector
    //! - Translation t = 0 (pure rotation about axis)
    //!
    //! For PRISMATIC joints:
    //! - R = I (identity matrix, no rotation)
    //! - Translation t = d * axis (linear displacement along axis)
    //!
    //! KINEMATIC CHAIN PROPAGATION:
    //! The joint transformation is multiplied with parent transformations
    //! to compute the forward kinematics: T_world = T_parent * T_joint
    //!
    //! \return 4x4 homogeneous transformation matrix
    // ------------------------------------------------------------------------
    Transform getTransform() const;

    // ------------------------------------------------------------------------
    //! \brief Update the associated node's transformation.
    //!
    //! PHYSICS: This method propagates the joint's motion through the
    //! kinematic chain by updating the spatial transformation of the
    //! associated scene graph node.
    //!
    //! KINEMATIC CHAIN DYNAMICS:
    //! When a joint moves, it affects the position and orientation of all
    //! downstream links in the kinematic chain. This propagation follows
    //! the composition of transformations:
    //!
    //! T_end = T_base * T_joint1 * T_joint2 * ... * T_jointN
    //!
    //! The update process:
    //! 1. Compute joint's local transformation T_joint
    //! 2. Set node's local transform to T_joint
    //! 3. Mark node as dirty for world transform recalculation
    //! 4. Trigger cascading updates through child nodes
    //!
    //! This mechanism ensures that:
    //! - Forward kinematics remains consistent
    //! - All dependent coordinate frames are updated
    //! - Performance is optimized through dirty flagging
    //!
    //! Called automatically when joint value changes via setValue()
    // ------------------------------------------------------------------------
    void updateLocalTransform();

    // ------------------------------------------------------------------------
    //! \brief Get the mechanical type of the joint.
    //! \return The mechanical type of the joint
    // ------------------------------------------------------------------------
    inline Joint::Type getType() const
    {
        return m_type;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the joint's motion axis vector.
    //! \return Constant reference to the normalized motion axis vector
    //!
    //! For REVOLUTE joints:
    //! - Axis represents the instantaneous rotation axis
    //! - Right-hand rule: Positive angles follow right-hand convention
    //! - Screw axis: Defines helical motion if combined with translation
    //! - Angular velocity: ω = θ̇ * axis (where θ̇ is joint velocity)
    //!
    //! For PRISMATIC joints:
    //! - Axis represents the translation direction
    //! - Unit direction: Positive values move in +axis direction
    //! - Linear velocity: v = ḋ * axis (where ḋ is joint velocity)
    // ------------------------------------------------------------------------
    inline const Eigen::Vector3d& getAxis() const
    {
        return m_axis;
    }

private:

    //! \brief Mechanical constraint type (revolute/prismatic/fixed)
    Joint::Type m_type;
    //! \brief Current joint configuration value
    double m_value;
    //! \brief Minimum allowable joint value (safety limit)
    double m_min;
    //! \brief Maximum allowable joint value (safety limit)
    double m_max;
    //! \brief Normalized motion axis in 3D space
    Eigen::Vector3d m_axis;
};

// *********************************************************************************
//! \brief Class representing a complete robotic arm.
//!
//! This class encapsulates an entire robotic manipulator system, combining
//! the kinematic chain (joints and links) with high-level control and
//! analysis capabilities. It serves as the main interface for robot control
//! and simulation.
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
//!    * Essential for: Velocity control, force control, singularity
//!    analysis
//!    * J = ∂pose/∂joints (6×n matrix for n joints)
//!
//! 4. CONFIGURATION MANAGEMENT: Set/get joint positions, enforce limits
//!    * Essential for: Robot control, safety, trajectory execution
//!
//! This class bridges the gap between low-level joint control and
//! high-level task planning, providing the mathematical foundation for
//! robot manipulation.
// *********************************************************************************
class RobotArm
{
public:

    // ------------------------------------------------------------------------
    //! \brief Constructor. Perform no action, just set the name.
    //! \param p_name Name of the robot arm.
    // ------------------------------------------------------------------------
    explicit RobotArm(const std::string_view& p_name) : m_name(p_name) {}

    // ------------------------------------------------------------------------
    //! \brief Setup the robot arm with a new root node and end effector.
    //! \tparam T Type of the root node. Must be a subclass of Node.
    //! \param p_root Unique pointer to the root node.
    //! \param p_end_effector Reference to the end effector node.
    //! \return Reference to the root node.
    // ------------------------------------------------------------------------
    void setupRobot(Node::Ptr p_root, Node& p_end_effector);

    // ------------------------------------------------------------------------
    //! \brief Define the given node as the end effector of the robot arm.
    //! \param p_node Pointer to the end effector node.
    // ------------------------------------------------------------------------
    void setEndEffector(Node& p_node);

    // ------------------------------------------------------------------------
    //! \brief Define the given node as the end effector of the robot arm.
    //! \param p_name Name of the end effector node.
    // ------------------------------------------------------------------------
    Node* setEndEffector(std::string_view p_name);

    // ------------------------------------------------------------------------
    //! \brief Get a const pointer to the root node of the robot arm.
    //! \note The root node is the base frame of the robot arm. We return a
    //! const pointer to it to avoid modifying the scene graph that will make
    //! this class not aware of the changes and not able to update its cache.
    //! If you need to modify the root node, use the setRootNode() method.
    //! \return Pointer to the root node.
    // ------------------------------------------------------------------------
    Node const* getRootNode() const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a node by its name.
    //! \param p_name Name of the node.
    //! \return Pointer to the node, or nullptr if not found.
    // ------------------------------------------------------------------------
    Node* getNode(const std::string_view& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Find and return a joint by its name.
    //! \param p_name Name of the joint.
    //! \return Pointer to the joint, or nullptr if not found.
    // ------------------------------------------------------------------------
    Joint* getJoint(const std::string_view& p_name) const;

    // ------------------------------------------------------------------------
    //! \brief Get the joint values of the robot arm.
    //!
    //! PHYSICS: Returns the current configuration of all joints in the
    //! kinematic chain. This vector represents the robot's pose in joint
    //! space, where each element corresponds to a joint's generalized
    //! coordinate.
    //!
    //! For a serial manipulator with n joints:
    //! - Joint space: Q = [q₁, q₂, ..., qₙ]
    //! - Each qᵢ represents the i-th joint's angular position (revolute)
    //!   or linear position (prismatic)
    //! - The configuration space is typically n-dimensional
    //!
    //! This joint configuration uniquely determines:
    //! - End-effector position and orientation (via forward kinematics)
    //! - Robot's workspace accessibility
    //! - Potential singularities and joint limits
    //!
    //! \return Vector of joint configuration values in order
    // ------------------------------------------------------------------------
    std::vector<double> getJointValues() const;

    // ------------------------------------------------------------------------
    //! \brief Get the names of the actuable joints in the order they appear
    //! in the joint values vector.
    //!
    //! This method provides a mapping between joint indices and joint names,
    //! allowing you to know which joint corresponds to which index in the
    //! joint values vector returned by getJointValues().
    //!
    //! \return Vector of joint names in the same order as joint values
    // ------------------------------------------------------------------------
    std::vector<std::string> getJointNames() const;

    // ------------------------------------------------------------------------
    //! \brief Set joint values for all actuable joints in order.
    //!
    //! This method provides a convenient way to set joint values using a simple
    //! vector of doubles, where each value corresponds to the joint in the same
    //! order as returned by getJointNames().
    //!
    //! \param p_values Vector of joint values to set
    //! \return True if successful, false if wrong number of values
    //! \throw std::invalid_argument if the number of values doesn't match the
    //! number of joints
    // ------------------------------------------------------------------------
    bool setJointValues(const std::vector<double>& p_values);

    // ------------------------------------------------------------------------
    //! \brief Get joint values by specifying joint names in desired order.
    //!
    //! This method allows you to retrieve joint values for specific joints
    //! in a custom order, providing more control over the joint configuration.
    //!
    //! \param p_joint_names Vector of joint names in the desired order
    //! \return Vector of joint values in the same order as the names
    //! \throw std::invalid_argument if any joint name is not found
    // ------------------------------------------------------------------------
    std::vector<double>
    getJointValuesByName(const std::vector<std::string>& p_joint_names) const;

    // ------------------------------------------------------------------------
    //! \brief Set joint values by specifying joint names and their values.
    //! \param p_joint_names Vector of joint names.
    //! \param p_values Vector of joint values in the same order as names.
    //! \return True if the joint values were set successfully.
    // ------------------------------------------------------------------------
    bool setJointValuesByName(const std::vector<std::string>& p_joint_names,
                              const std::vector<double>& p_values);

    // ------------------------------------------------------------------------
    //! \brief Compute the forward kinematics of the robot arm.
    //! \note Call setEndEffector(Node&) before calling this method.
    //! \return The transformation matrix from the root to the end effector.
    //! \throw std::runtime_error if the end effector is not set.
    // ------------------------------------------------------------------------
    Transform forwardKinematics() const;

    // ------------------------------------------------------------------------
    //! \brief Get the pose of the end effector.
    //! \note Call setEndEffector(Node&) before calling this method.
    //! \return The pose of the end effector.
    //! \throw std::runtime_error if the end effector is not set.
    // ------------------------------------------------------------------------
    Pose getEndEffectorPose() const;

    // ------------------------------------------------------------------------
    //! \brief Compute the inverse kinematics of the robot arm by the Jacobian
    //! method.
    //! \note Call setEndEffector(Node&) before calling this method.
    //! \throw std::runtime_error if the end effector is not set.
    //!
    //! PHYSICS: Inverse kinematics solves for the joint configuration that
    //! achieves a desired end-effector pose. This is mathematically complex
    //! as it involves solving a system of nonlinear equations.
    //!
    //! MATHEMATICAL FORMULATION:
    //! Given target pose P_target, find joint values Q such that:
    //! P_target = f(Q) where f is the forward kinematics function
    //!
    //! SOLUTION CHALLENGES:
    //! - Multiple solutions: A given pose may have several valid joint configs
    //! - No solutions: Target pose may be outside robot's workspace
    //! - Infinite solutions: At kinematic singularities
    //! - Numerical stability: Iterative methods may not converge
    //!
    //! COMMON ALGORITHMS:
    //! - Jacobian-based methods (Newton-Raphson, Levenberg-Marquardt)
    //! - Geometric methods (for specific robot geometries)
    //! - Optimization-based approaches
    //!
    //! WORKSPACE CONSTRAINTS:
    //! - Reachable workspace: All poses that can be achieved
    //! - Dexterous workspace: Poses achievable with arbitrary orientations
    //! - Joint limits, singularities, and obstacles affect solvability
    //!
    //! \param p_target_pose Desired 6D pose of the end-effector
    //! [x,y,z,rx,ry,rz].
    //! \param solution Output vector containing the joint configuration.
    //! \param p_max_iterations Maximum number of iterations.
    //! \param p_epsilon Tolerance for convergence.
    //! \param p_damping Damping factor.
    //!
    //! \return True if a valid solution is found, false otherwise.
    // ------------------------------------------------------------------------
    bool inverseKinematics(const Pose& p_target_pose,
                           std::vector<double>& solution,
                           size_t const p_max_iterations = 500,
                           double const p_epsilon = 1e-4,
                           double const p_damping = 0.01);

    // ------------------------------------------------------------------------
    //! \brief Compute the Jacobian matrix of the robot arm.
    //! \note Call setEndEffector(Node&) before calling this method.
    //! \return The Jacobian matrix.
    // ------------------------------------------------------------------------
    Jacobian calculateJacobian() const;

protected:

    // ------------------------------------------------------------------------
    //! \brief Check if the robot arm is setup correctly.
    //! \throw std::runtime_error if the robot arm is not setup correctly.
    // ------------------------------------------------------------------------
    void checkRobotSetupValidity() const;

    // ------------------------------------------------------------------------
    //! \brief Cache the list of joints in the robot arm. Shall be called
    //! each time the scene graph is modified.
    //! \note This method is called automatically when needed, but can be
    //! called explicitly to force cache update.
    // ------------------------------------------------------------------------
    void cacheListOfJoints();

private:

    std::string m_name;
    Node::Ptr m_root_node;
    std::vector<Joint*> m_joints;
    Node* m_end_effector = nullptr;
};

//  Utility functions for conversions.
namespace utils
{

// ------------------------------------------------------------------------
//! \brief Convert Euler angles to rotation matrix.
//!
//! PHYSICS: Euler angles represent a sequence of rotations about coordinate
//! axes. This function implements the ZYX (yaw-pitch-roll) convention,
//! which is common in robotics.
//!
//! MATHEMATICAL FOUNDATION:
//! R = R_z(yaw) * R_y(pitch) * R_x(roll)
//! Where each R_i(θ) is a rotation matrix about axis i by angle θ
//!
//! ROTATION SEQUENCE:
//! 1. Roll (rx): Rotation about X-axis (bank angle)
//! 2. Pitch (ry): Rotation about Y-axis (elevation angle)
//! 3. Yaw (rz): Rotation about Z-axis (azimuth angle)
//!
//! GIMBAL LOCK WARNING:
//! This representation suffers from gimbal lock when pitch = ±π/2
//! For critical applications, consider using quaternions instead.
//!
//! \param p_rx Roll angle around X-axis (doubles)
//! \param p_ry Pitch angle around Y-axis (doubles)
//! \param p_rz Yaw angle around Z-axis (doubles)
//! \return 3x3 rotation matrix
// ------------------------------------------------------------------------
Eigen::Matrix3d eulerToRotation(double p_rx, double p_ry, double p_rz);

// ------------------------------------------------------------------------
//! \brief Extract Euler angles from rotation matrix.
//!
//! PHYSICS: Decomposes a 3x3 rotation matrix into three sequential
//! rotations about coordinate axes using ZYX convention.
//!
//! MATHEMATICAL PROCESS:
//! Given rotation matrix R, extract angles such that:
//! R = R_z(yaw) * R_y(pitch) * R_x(roll)
//!
//! SINGULARITY HANDLING:
//! - At gimbal lock (pitch = ±π/2), roll and yaw become interdependent
//! - Solution: Set roll to zero and compute yaw accordingly
//! - This ensures unique representation but may not preserve original angles
//!
//! ANGLE RANGES:
//! - Roll: [-π, π]
//! - Pitch: [-π/2, π/2]
//! - Yaw: [-π, π]
//!
//! \param p_rot 3x3 rotation matrix (must be orthogonal)
//! \return Vector containing [roll, pitch, yaw] angles
// ------------------------------------------------------------------------
Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& p_rot);

// ------------------------------------------------------------------------
//! \brief Create homogeneous transformation from translation and rotation.
//!
//! PHYSICS: Combines separate translation and rotation into a single
//! 4x4 homogeneous transformation matrix for efficient computation.
//!
//! MATHEMATICAL STRUCTURE:
//! T = [R  t]  where R is 3x3 rotation matrix, t is 3x1 translation
//!     [0  1]
//!
//! GEOMETRIC INTERPRETATION:
//! - First applies rotation R to a point
//! - Then adds translation t
//! - Composition: T(p) = R*p + t
//!
//! \param p_translation 3D translation vector [x, y, z]
//! \param p_rotation 3x3 rotation matrix
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          const Eigen::Matrix3d& p_rotation);

// ------------------------------------------------------------------------
//! \brief Create homogeneous transformation from translation and Euler angles.
//!
//! PHYSICS: Convenience function that combines translation with rotation
//! specified as Euler angles using ZYX convention.
//!
//! PROCESS:
//! 1. Convert Euler angles to rotation matrix
//! 2. Combine with translation into homogeneous form
//!
//! APPLICATIONS:
//! - Robot link transformations
//! - Camera pose specification
//! - Object placement in 3D space
//!
//! \param p_translation 3D translation vector [x, y, z]
//! \param p_rx Roll angle around X-axis (doubles)
//! \param p_ry Pitch angle around Y-axis (doubles)
//! \param p_rz Yaw angle around Z-axis (doubles)
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform createTransform(const Eigen::Vector3d& p_translation,
                          double p_rx,
                          double p_ry,
                          double p_rz);

// Extraction of data from transformations
Eigen::Vector3d getTranslation(const Transform& p_transform);
Eigen::Matrix3d getRotation(const Transform& p_transform);
Pose transformToPose(const Transform& p_transform);
Transform poseToTransform(const Pose& p_pose);

// ------------------------------------------------------------------------
//! \brief Create transformation matrix using Denavit-Hartenberg parameters.
//!
//! PHYSICS: The Denavit-Hartenberg (DH) convention is a systematic method
//! for describing the geometry of serial robotic manipulators. It provides
//! a standardized way to represent the relationship between consecutive
//! joint coordinate frames.
//!
//! DH PARAMETERS:
//! - a (link length): Distance between joint axes along common normal
//! - α (link twist): Angle between joint axes about common normal
//! - d (link offset): Distance between common normals along joint axis
//! - θ (joint angle): Angle between common normals about joint axis
//!
//! TRANSFORMATION SEQUENCE:
//! 1. Rotate about Z-axis by θ (joint angle)
//! 2. Translate along Z-axis by d (link offset)
//! 3. Translate along X-axis by a (link length)
//! 4. Rotate about X-axis by α (link twist)
//!
//! MATHEMATICAL FORMULA:
//! T = Rot_z(θ) * Trans_z(d) * Trans_x(a) * Rot_x(α)
//!
//! APPLICATIONS:
//! - Forward kinematics computation
//! - Robot modeling and simulation
//! - Kinematic calibration
//! - Workspace analysis
//!
//! \param p_a Link length parameter (meters)
//! \param p_alpha Link twist parameter (doubles)
//! \param p_d Link offset parameter (meters)
//! \param p_theta Joint angle parameter (doubles)
//! \return 4x4 homogeneous transformation matrix
// ------------------------------------------------------------------------
Transform dhTransform(double p_a, double p_alpha, double p_d, double p_theta);

} // namespace utils

} // namespace robotik
