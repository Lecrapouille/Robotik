#pragma once

#include "Robotik/private/Geometry.hpp"
#include "Robotik/private/Inertial.hpp"
#include "Robotik/private/SceneNode.hpp"

namespace robotik
{

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
class Joint: public scene::Node
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
        REVOLUTE,   // Revolute joint - rotation around axis
        PRISMATIC,  // Prismatic joint - translation along axis
        CONTINUOUS, // Continuous joint - rotation around axis
        FIXED       // Fixed joint - no movement, rigid connection
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
    Joint(std::string_view const& p_name,
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
    //! \param p_position New joint configuration value in appropriate units
    // ------------------------------------------------------------------------
    void position(double p_position); // FIXME: utiliser S.I.

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
    inline void limits(double p_min, double p_max)
    {
        m_min = p_min;
        m_max = p_max;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the joint's motion limits.
    //! \return The joint's motion limits
    // ------------------------------------------------------------------------
    inline std::pair<double, double> limits() const
    {
        return { m_min, m_max };
    }

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

    // ------------------------------------------------------------------------
    //! \brief Get the mechanical type of the joint.
    //! \return The mechanical type of the joint
    // ------------------------------------------------------------------------
    inline Joint::Type type() const
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
    inline const Eigen::Vector3d& axis() const
    {
        return m_axis;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the geometry of the joint.
    //! \param p_geometry The geometry to set
    // ------------------------------------------------------------------------
    inline void setGeometry(const Geometry& p_geometry)
    {
        m_geometry = p_geometry;
    }

    // ------------------------------------------------------------------------
    //! \brief Set the inertial properties of the joint.
    //! \param p_inertial The inertial properties to set
    // ------------------------------------------------------------------------
    inline void setInertial(const Inertial& p_inertial)
    {
        m_inertial = p_inertial;
    }

private:

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
    Transform computeJointTransform(Joint::Type p_type,
                                    double p_position,
                                    const Eigen::Vector3d& p_axis) const;

private:

    //! \brief Mechanical constraint type (revolute/prismatic/fixed)
    Joint::Type m_type;
    //! \brief Current joint configuration value
    double m_position;
    //! \brief Minimum allowable joint value (safety limit)
    double m_min;
    //! \brief Maximum allowable joint value (safety limit)
    double m_max;
    //! \brief Normalized motion axis in 3D space
    Eigen::Vector3d m_axis;
    //! \brief Geometry of the link connected to the joint
    Geometry m_geometry;
    //! \brief Inertial properties of the joint
    Inertial m_inertial;
    //! \brief Cached joint transformation matrix.
    // FIXME Transform m_joint_transform;
};

} // namespace robotik