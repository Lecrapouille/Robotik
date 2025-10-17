/**
 * @file Joint.hpp
 * @brief Joint class - Representation of a robotic joint.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Node.hpp"

namespace robotik
{

// *********************************************************************************
//! \brief Class representing a robotic joint.
//!
//! In robotics, a joint is a mechanical connection between two rigid bodies
//! (links) that allows controlled motion between them. Joints are the actuated
//! components that give robots their ability to move and manipulate objects.
//!
//! - REVOLUTE joints: Allow rotation around a fixed axis (like elbow,
//! shoulder).
//!   * Examples: Robot arm joints, wheel steering mechanisms.
//!   * Example of range: -π to +π radians (or limited by mechanical
//!   constraints).
//!
//! - CONTINUOUS joints: Allow rotation around a fixed axis with no limits.
//!   * Examples: wheel steering mechanisms.
//!
//! - PRISMATIC joints: Allow linear translation along a fixed axis (like
//! pistons).
//!   * Examples: Linear actuators, elevator mechanisms, grippers.
//!   * Example of range: 0 to max_extension (in meters).
//!
//! - FIXED joints: No movement, rigid connection between links
//!   * Used for: Tool attachments, sensor mounts, structural connections.
//!
//! Each joint has:
//! - A motion axis (3D vector defining rotation/translation direction).
//! - Current position/angle value (the joint's configuration).
//! - Motion limits (safety and mechanical constraints).
//! - Associated Node for transform propagation in the scene graph.
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
    //! REVOLUTE, CONTINUOUS: Rotational joint around a fixed axis.
    //! CONTINUOUS is a special case of REVOLUTE with no limits.
    //! - Degrees of freedom: 1 (rotation angle).
    //! - Motion: Circular around axis vector.
    //! - Examples: Elbow, shoulder, wrist rotation.
    //! - Typical actuators: Servo motors, stepper motors.
    //!
    //! PRISMATIC: Linear translation joint along a fixed axis.
    //! - Degrees of freedom: 1 (linear position).
    //! - Motion: Straight line along axis vector.
    //! - Examples: Linear actuators, pneumatic pistons, telescoping arms.
    //! - Typical actuators: Linear motors, hydraulic cylinders.
    //!
    //! FIXED: Rigid connection with no movement.
    //! - Degrees of freedom: 0 (completely constrained).
    //! - Motion: None (permanent rigid attachment).
    //! - Examples: Tool mounting, sensor brackets, structural connections.
    //! - Purpose: Maintain fixed spatial relationships between components.
    // ------------------------------------------------------------------------
    enum class Type
    {
        //! \brief Fixed joint - no movement, rigid connection
        FIXED,
        //! \brief Prismatic joint - translation along axis with limits
        PRISMATIC,
        //! \brief Revolute joint - rotation around axis with limits
        REVOLUTE,
        //! \brief Continuous joint - rotation around axis with no limits
        CONTINUOUS,
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
    //! - Axis defines the instantaneous axis of rotation (Rodrigues' theorem).
    //! - Angular displacement follows θ(t) = θ₀ + ω*t for constant velocity.
    //! - Kinetic energy: KE = ½*I*ω² where I is moment of inertia.
    //! - Torque-angle relationship: τ = I*α (α = angular acceleration).
    //!
    //! For PRISMATIC joints:
    //! - Axis defines the direction of linear translation.
    //! - Linear displacement follows x(t) = x₀ + v*t for constant velocity.
    //! - Kinetic energy: KE = ½*m*v² where m is mass.
    //! - Force-displacement relationship: F = m*a (a = linear acceleration).
    //!
    //! The axis vector must be normalized as it represents a pure direction
    //! in 3D space. The joint's configuration space is one-dimensional for
    //! both revolute and prismatic joints.
    //!
    //! \param p_name Unique identifier for the joint in the kinematic chain.
    //! \param p_index Index of the joint in the hierarchy to access it from a
    //! std::vector<Joint>.
    //! \param p_type Mechanical type defining the motion constraint (REVOLUTE,
    //! PRISMATIC, FIXED, CONTINUOUS).
    //! \param p_axis Normalized 3D vector defining the motion axis.
    // ------------------------------------------------------------------------
    Joint(std::string const& p_name,
          Type p_type,
          Eigen::Vector3d const& p_axis,
          size_t p_index = 0);

    // ------------------------------------------------------------------------
    //! \brief Get the joint's index in the hierarchy.
    //! \return The joint's index in the hierarchy.
    // ------------------------------------------------------------------------
    inline size_t index() const
    {
        return m_index;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the joint's index in the hierarchy.
    //!
    //! This method is called by Hierarchy::cacheHierarchyTree() to assign
    //! indices to joints after they are added to the joints vector.
    //!
    //! \param p_index The index to assign to this joint.
    // ------------------------------------------------------------------------
    void setIndex(size_t p_index);

    // ------------------------------------------------------------------------
    //! \brief Get the joint's motion axis vector.
    //! \return Constant reference to the normalized motion axis vector.
    //!
    //! For REVOLUTE and CONTINUOUS joints:
    //! - Axis represents the instantaneous rotation axis.
    //! - Right-hand rule: Positive angles follow right-hand convention.
    //! - Screw axis: Defines helical motion if combined with translation.
    //! - Angular velocity: ω = θ̇' * axis (where θ̇' is joint velocity).
    //!
    //! For PRISMATIC joints:
    //! - Axis represents the translation direction.
    //! - Unit direction: Positive values move in +axis direction.
    //! - Linear velocity: v = d' * axis (where d' is joint velocity).
    // ------------------------------------------------------------------------
    inline Eigen::Vector3d const& axis() const
    {
        return m_axis;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the mechanical type of the joint (REVOLUTE, PRISMATIC ...).
    //! \return The mechanical type of the joint
    // ------------------------------------------------------------------------
    inline Joint::Type type() const
    {
        return m_type;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the joint's position (angular or linear) value and compute
    //! the joint's local transformation matrix. The joint value directly
    //! affects the local transformation matrix that propagates through the
    //! kinematic chain during forward kinematics.
    //!
    //! PHYSICS: This method computes the 4x4 homogeneous transformation
    //! matrix that represents the spatial relationship between the joint's
    //! parent and child coordinate frames.
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
    // ------------------------------------------------------------------------
    void position(double p_position); // FIXME: utiliser S.I.

    // ------------------------------------------------------------------------
    //! \brief Set the current joint configuration value.
    //! \param p_velocity The current joint velocity value.
    //! \param p_dt The time step.
    // ------------------------------------------------------------------------
    void position(double p_velocity, double p_dt);

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
    inline double position() const
    {
        return m_position;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current joint velocity value.
    //! \return The current joint velocity value
    // ------------------------------------------------------------------------
    inline double velocity() const
    {
        return m_velocity;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the maximum joint velocity value.
    // ------------------------------------------------------------------------
    inline double maxVelocity() const
    {
        return m_velocity_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the maximum joint velocity value.
    //! \param p_velocity_max The maximum joint velocity value
    // ------------------------------------------------------------------------
    inline void maxVelocity(double p_velocity_max)
    {
        m_velocity_max = p_velocity_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the current joint velocity value constrained by the maximum
    //! velocity.
    //! \param p_velocity The current joint velocity value
    // ------------------------------------------------------------------------
    void velocity(double p_acceleration, double p_dt);

    // ------------------------------------------------------------------------
    //! \brief Get the current joint acceleration value.
    //! \return The current joint acceleration value
    // ------------------------------------------------------------------------
    inline double acceleration() const
    {
        return m_acceleration;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the current joint acceleration value.
    //! \param p_acceleration The current joint acceleration value
    // ------------------------------------------------------------------------
    inline void acceleration(double p_acceleration)
    {
        m_acceleration = p_acceleration;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the current joint effort (force or torque) value.
    //! \return The current joint effort value
    // ------------------------------------------------------------------------
    inline double effort() const
    {
        return m_effort;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the current joint effort (force or torque) value.
    //! \param p_effort The current joint effort value
    // ------------------------------------------------------------------------
    inline void effort(double p_effort)
    {
        m_effort = p_effort;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the maximum joint effort (force or torque) value.
    //! \return The maximum joint effort value
    // ------------------------------------------------------------------------
    inline double effort_max() const
    {
        return m_effort_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the maximum joint effort (force or torque) value.
    //! \param p_effort_max The maximum joint effort value
    // ------------------------------------------------------------------------
    inline void effort_max(double p_effort_max)
    {
        m_effort_max = p_effort_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the joint damping coefficient.
    //! \return The joint damping coefficient
    // ------------------------------------------------------------------------
    inline double damping() const
    {
        return m_damping;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the joint damping coefficient.
    //! \param p_damping The joint damping coefficient
    // ------------------------------------------------------------------------
    inline void damping(double p_damping)
    {
        m_damping = p_damping;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the joint friction coefficient.
    //! \return The joint friction coefficient
    // ------------------------------------------------------------------------
    inline double friction() const
    {
        return m_friction;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the joint friction coefficient.
    //! \param p_friction The joint friction coefficient
    // ------------------------------------------------------------------------
    inline void friction(double p_friction)
    {
        m_friction = p_friction;
    }

    // ------------------------------------------------------------------------
    //! \brief Set joint motion limits for safety and mechanical constraints.
    //!
    //! Joint limits represent physical and safety constraints that prevent
    //! damage to the robotic system and ensure safe operation:
    //! - Hard stops: Physical barriers preventing further motion.
    //! - Actuator limits: Maximum torque/force output capabilities.
    //! - Structural limits: Material stress and fatigue considerations.
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
    //! \param p_position_min Minimum allowed joint value.
    //! \param p_position_max Maximum allowed joint value.
    //! \note The limits have no effect for CONTINUOUS joints and FIXED joints.
    // ------------------------------------------------------------------------
    inline void limits(double p_position_min, double p_position_max)
    {
        // Note: the if is redundant since the setter position() has security.
        // This code is for the sake of clarity.
        if (m_type == Joint::Type::CONTINUOUS || m_type == Joint::Type::FIXED)
            return;

        m_position_min = p_position_min;
        m_position_max = p_position_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the joint's motion limits.
    //! \return The joint's motion limits
    // ------------------------------------------------------------------------
    inline std::pair<double, double> limits() const
    {
        return { m_position_min, m_position_max };
    }

    // ------------------------------------------------------------------------
    //! \brief Set the local transform of the joint.
    // ------------------------------------------------------------------------
    void localTransform(Transform const& p_transform) override;

    // ------------------------------------------------------------------------
    //! \brief Get the local transform of the joint.
    //!
    //! PHYSICS: This method overrides the base class localTransform() to
    //! properly combine the static local transform (from URDF parsing) with
    //! the dynamic joint transform (computed from joint position/type).
    //!
    //! The combined transformation follows the composition:
    //! T_local_combined = T_static * T_joint
    //!
    //! Where:
    //! - T_static: Static transformation from URDF (Node::m_local_transform)
    //! - T_joint: Dynamic transformation based on joint position/type
    //!
    //! This ensures proper kinematic chain propagation while maintaining
    //! clean separation between static and dynamic transformations.
    //!
    //! \return Combined local transformation matrix
    // ------------------------------------------------------------------------
    Transform const& localTransform() const override;

    // ------------------------------------------------------------------------
    //! \brief Accept a visitor (Visitor pattern override).
    //! \param visitor The visitor to accept.
    // ------------------------------------------------------------------------
    void accept(NodeVisitor& visitor) override;

    // ------------------------------------------------------------------------
    //! \brief Accept a const visitor (Visitor pattern override).
    //! \param visitor The const visitor to accept.
    // ------------------------------------------------------------------------
    void accept(ConstNodeVisitor& visitor) const override;

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
    //! Called automatically when joint value changes via value()
    // ------------------------------------------------------------------------
    void updateTransforms();

private:

    //! \brief Mechanical constraint type (revolute/prismatic/fixed)
    Joint::Type m_type;
    //! \brief Index of the joint in the hierarchy.
    size_t m_index = 0;
    //! \brief Normalized motion axis in 3D space
    Eigen::Vector3d m_axis;
    //! \brief Cached joint transformation matrix.
    Transform m_joint_transform;
    //! \brief Cached combined local transformation (static + joint).
    Transform m_combined_local_transform;
    //! \brief Current joint configuration value (radians or meters)
    double m_position;
    //! \brief safety limit: minimum allowable joint value (safety limit)
    //! (radians or meters)
    double m_position_min;
    //! \brief safety limit: maximum allowable joint value (safety limit)
    //! (radians or meters)
    double m_position_max;
    //! \brief Current joint velocity value (radians/second or meters/second)
    double m_velocity = 0.0;
    //! \brief Maximum joint velocity value (radians/second or meters/second)
    double m_velocity_max = 0.0;
    //! \brief Current joint acceleration value (radians/second² or
    //! meters/second²)
    double m_acceleration = 0.0;
    //! \brief Current force/torque value (newtons or newton-meters)
    double m_effort = 0.0;
    //! \brief Maximum joint force/torque value (newtons or newton-meters)
    double m_effort_max = 0.0;
    //! \brief Damping parameter (no unit)
    double m_damping = 0.0;
    //! \brief Friction parameter (no unit)
    double m_friction = 0.0;
};

} // namespace robotik