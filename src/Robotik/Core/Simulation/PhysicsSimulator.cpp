/**
 * @file PhysicsSimulator.cpp
 * @brief PhysicsSimulator class - Apply physics to the robot.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Simulation/PhysicsSimulator.hpp"
#include "Robotik/Core/Robot/Blueprint/Joint.hpp"
#include "Robotik/Core/Robot/Blueprint/Link.hpp"
#include "Robotik/Core/Robot/Robot.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
PhysicsSimulator::PhysicsSimulator(double p_dt,
                                   Eigen::Vector3d const& p_gravity)
    : m_dt(p_dt), m_gravity(p_gravity)
{
}

// ----------------------------------------------------------------------------
Eigen::Vector3d PhysicsSimulator::computeLinkForce(Link const& p_link) const
{
    return m_gravity * p_link.mass();
}

// ----------------------------------------------------------------------------
void PhysicsSimulator::step(Robot& p_robot)
{
    // TODO: Reimplement physics simulation using flat arrays instead of tree
    // structure The old tree-based implementation has been removed as part of
    // the refactoring. For now, physics simulation is disabled.
    (void)p_robot; // Suppress unused parameter warning
}

// ----------------------------------------------------------------------------
//! \brief Compute the dynamics for a joint (Newton-Euler).
//! \param p_joint The joint to compute the dynamics for.
//! \param p_parent_vel The velocity of the parent node.
//! \param p_parent_acc The acceleration of the parent node.
// ----------------------------------------------------------------------------
void PhysicsSimulator::computeDynamics(Joint& p_joint,
                                       const Eigen::Vector3d& p_parent_vel,
                                       const Eigen::Vector3d& p_parent_acc)
{
    // Skip fixed joints
    if (p_joint.type() == Joint::Type::FIXED)
    {
        // Propagate to children joints
        for (const auto& child : p_joint.children())
        {
            // Find the link child, then its joint children
            if (auto* link = dynamic_cast<Link*>(child.get()); link)
            {
                for (const auto& grandchild : link->children())
                {
                    auto* joint = dynamic_cast<Joint*>(grandchild.get());
                    if (joint)
                    {
                        computeDynamics(*joint, p_parent_vel, p_parent_acc);
                    }
                }
            }
        }
        return;
    }

    // Find the child link to get its mass and center of mass
    Link* child_link = nullptr;
    for (const auto& child : p_joint.children())
    {
        child_link = dynamic_cast<Link*>(child.get());
        if (child_link)
            break;
    }

    // If no child link, nothing to compute
    if (!child_link)
        return;

    // Global center of mass position
    Eigen::Vector3d com_global =
        (child_link->worldTransform() * child_link->centerOfMass()).head<3>();

    // Applied forces (gravity)
    Eigen::Vector3d force = computeLinkForce(*child_link);

    // Joint acceleration calculation
    double inertia_equiv =
        child_link->mass() * child_link->length() * child_link->length() / 3.0;

    // Gravity torque/force
    double gravity_effort = 0.0;
    if (p_joint.type() == Joint::Type::REVOLUTE ||
        p_joint.type() == Joint::Type::CONTINUOUS)
    {
        // For revolute joints, compute torque from gravity
        Eigen::Vector3d r =
            com_global - p_joint.worldTransform().block<3, 1>(0, 3);
        Eigen::Vector3d torque = r.cross(force);
        gravity_effort = torque.dot(p_joint.axis());
    }
    else if (p_joint.type() == Joint::Type::PRISMATIC)
    {
        // For prismatic joints, use force projection
        gravity_effort = force.dot(p_joint.axis());
    }

    // Joint dynamics: a = (τ_applied + τ_gravity - damping - friction) /
    // inertia
    double total_effort =
        p_joint.effort() + gravity_effort -
        p_joint.damping() * p_joint.velocity() -
        p_joint.friction() * std::copysign(1.0, p_joint.velocity());

    // Clamping of the effort
    if (p_joint.effort_max() > 0.0)
    {
        total_effort = std::max(-p_joint.effort_max(),
                                std::min(p_joint.effort_max(), total_effort));
    }

    // Compute acceleration (avoid division by zero)
    p_joint.acceleration(total_effort / (inertia_equiv + 1e-6));

    // Propagate to children joints
    Eigen::Vector3d vel = p_parent_vel;
    Eigen::Vector3d acc = p_parent_acc;

    for (const auto& grandchild : child_link->children())
    {
        auto* joint = dynamic_cast<Joint*>(grandchild.get());
        if (joint)
        {
            computeDynamics(*joint, vel, acc);
        }
    }
}

// ----------------------------------------------------------------------------
void PhysicsSimulator::integrate(Joint& p_joint)
{
    // Skip fixed joints
    if (p_joint.type() == Joint::Type::FIXED)
    {
        // Propagate to children
        for (const auto& child : p_joint.children())
        {
            auto* link = dynamic_cast<Link*>(child.get());
            if (link)
            {
                for (const auto& grandchild : link->children())
                {
                    auto* joint = dynamic_cast<Joint*>(grandchild.get());
                    if (joint)
                    {
                        integrate(*joint);
                    }
                }
            }
        }
        return;
    }

    // Update the velocity with clamping
    p_joint.velocity(p_joint.acceleration(), m_dt);

    // Update the position with clamping
    p_joint.position(p_joint.velocity(), m_dt);

    // Propagate to children
    for (const auto& child : p_joint.children())
    {
        auto* link = dynamic_cast<Link*>(child.get());
        if (link)
        {
            for (const auto& grandchild : link->children())
            {
                auto* joint = dynamic_cast<Joint*>(grandchild.get());
                if (joint)
                {
                    integrate(*joint);
                }
            }
        }
    }
}

} // namespace robotik
