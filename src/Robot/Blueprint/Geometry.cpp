/**
 * @file Geometry.cpp
 * @brief Geometry classes for collision detection and/or visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Robot/Blueprint/Geometry.hpp"
#include "Robotik/Robot/Blueprint/NodeVisitor.hpp"

#include <algorithm>
#include <cmath>

namespace robotik
{

// ----------------------------------------------------------------------------
bool Geometry::collide(const Geometry& p_other) const
{
    // Dispatch based on geometry types
    if (m_type == Type::SPHERE && p_other.m_type == Type::SPHERE)
    {
        return collideSpheres(p_other);
    }
    else if (m_type == Type::CYLINDER && p_other.m_type == Type::CYLINDER)
    {
        return collideCylinders(p_other);
    }
    else if (m_type == Type::BOX && p_other.m_type == Type::BOX)
    {
        return collideOBBs(p_other);
    }
    else if (m_type == Type::MESH && p_other.m_type == Type::MESH)
    {
        return collideOBBs(p_other);
    }
    else if (m_type == Type::BOX && p_other.m_type == Type::MESH)
    {
        return collideOBBs(p_other);
    }
    else if (m_type == Type::MESH && p_other.m_type == Type::BOX)
    {
        return collideOBBs(p_other);
    }
    else if ((m_type == Type::SPHERE && p_other.m_type == Type::CYLINDER) ||
             (m_type == Type::CYLINDER && p_other.m_type == Type::SPHERE))
    {
        const Geometry& sphere = (m_type == Type::SPHERE) ? *this : p_other;
        const Geometry& cylinder = (m_type == Type::CYLINDER) ? *this : p_other;
        return collideSphereCylinder(sphere, cylinder);
    }
    else if ((m_type == Type::SPHERE && p_other.m_type == Type::BOX) ||
             (m_type == Type::BOX && p_other.m_type == Type::SPHERE))
    {
        const Geometry& sphere = (m_type == Type::SPHERE) ? *this : p_other;
        const Geometry& obb = (m_type == Type::BOX) ? *this : p_other;
        return collideSphereOBB(sphere, obb);
    }
    else if ((m_type == Type::SPHERE && p_other.m_type == Type::MESH) ||
             (m_type == Type::MESH && p_other.m_type == Type::SPHERE))
    {
        const Geometry& sphere = (m_type == Type::SPHERE) ? *this : p_other;
        const Geometry& obb = (m_type == Type::MESH) ? *this : p_other;
        return collideSphereOBB(sphere, obb);
    }
    else if ((m_type == Type::CYLINDER && p_other.m_type == Type::BOX) ||
             (m_type == Type::BOX && p_other.m_type == Type::CYLINDER))
    {
        const Geometry& cylinder = (m_type == Type::CYLINDER) ? *this : p_other;
        const Geometry& obb = (m_type == Type::BOX) ? *this : p_other;
        return collideCylinderOBB(cylinder, obb);
    }
    else if ((m_type == Type::CYLINDER && p_other.m_type == Type::MESH) ||
             (m_type == Type::MESH && p_other.m_type == Type::CYLINDER))
    {
        const Geometry& cylinder = (m_type == Type::CYLINDER) ? *this : p_other;
        const Geometry& obb = (m_type == Type::MESH) ? *this : p_other;
        return collideCylinderOBB(cylinder, obb);
    }

    // Unknown combination, no collision
    return false;
}

// ----------------------------------------------------------------------------
bool Geometry::collideSpheres(const Geometry& p_other) const
{
    // Transform centers to world space
    Eigen::Vector4d center1_h(m_collision_center.x(),
                              m_collision_center.y(),
                              m_collision_center.z(),
                              1.0);
    Eigen::Vector4d center2_h(p_other.m_collision_center.x(),
                              p_other.m_collision_center.y(),
                              p_other.m_collision_center.z(),
                              1.0);

    Eigen::Vector3d center1 = (worldTransform() * center1_h).head<3>();
    Eigen::Vector3d center2 = (p_other.worldTransform() * center2_h).head<3>();

    double radius1 = m_collision_params[0];
    double radius2 = p_other.m_collision_params[0];

    double distance = (center2 - center1).norm();
    return distance <= (radius1 + radius2);
}

// ----------------------------------------------------------------------------
bool Geometry::collideCylinders(const Geometry& p_other) const
{
    // Transform to world space
    Eigen::Vector4d center1_h(m_collision_center.x(),
                              m_collision_center.y(),
                              m_collision_center.z(),
                              1.0);
    Eigen::Vector4d center2_h(p_other.m_collision_center.x(),
                              p_other.m_collision_center.y(),
                              p_other.m_collision_center.z(),
                              1.0);

    Eigen::Vector3d center1 = (worldTransform() * center1_h).head<3>();
    Eigen::Vector3d center2 = (p_other.worldTransform() * center2_h).head<3>();

    Eigen::Matrix3d rot1 =
        worldTransform().block<3, 3>(0, 0) * m_collision_orientation;
    Eigen::Matrix3d rot2 = p_other.worldTransform().block<3, 3>(0, 0) *
                           p_other.m_collision_orientation;

    // Cylinder axis is along z-axis in local frame
    Eigen::Vector3d axis1 = rot1.col(2);
    Eigen::Vector3d axis2 = rot2.col(2);

    double radius1 = m_collision_params[0];
    double halfHeight1 = m_collision_params[1] / 2.0;
    double radius2 = p_other.m_collision_params[0];
    double halfHeight2 = p_other.m_collision_params[1] / 2.0;

    // Simplified cylinder-cylinder collision: treat as capsules
    // Project centers onto axes
    Eigen::Vector3d p1_start = center1 - axis1 * halfHeight1;
    Eigen::Vector3d p1_end = center1 + axis1 * halfHeight1;
    Eigen::Vector3d p2_start = center2 - axis2 * halfHeight2;
    Eigen::Vector3d p2_end = center2 + axis2 * halfHeight2;

    // Find closest points on line segments
    Eigen::Vector3d d1 = p1_end - p1_start;
    Eigen::Vector3d d2 = p2_end - p2_start;
    Eigen::Vector3d r = p1_start - p2_start;

    double a = d1.dot(d1);
    double b = d1.dot(d2);
    double c = d2.dot(d2);
    double d = d1.dot(r);
    double e = d2.dot(r);

    double denom = a * c - b * b;
    double s = 0.0, t = 0.0;

    if (denom < 1e-10)
    {
        s = 0.0;
        t = e / c;
    }
    else
    {
        s = (b * e - c * d) / denom;
        t = (a * e - b * d) / denom;
    }

    s = std::clamp(s, 0.0, 1.0);
    t = std::clamp(t, 0.0, 1.0);

    Eigen::Vector3d closest1 = p1_start + s * d1;
    Eigen::Vector3d closest2 = p2_start + t * d2;

    double distance = (closest2 - closest1).norm();
    return distance <= (radius1 + radius2);
}

// ----------------------------------------------------------------------------
bool Geometry::collideOBBs(const Geometry& p_other) const
{
    // Transform to world space
    Eigen::Vector4d center1_h(m_collision_center.x(),
                              m_collision_center.y(),
                              m_collision_center.z(),
                              1.0);
    Eigen::Vector4d center2_h(p_other.m_collision_center.x(),
                              p_other.m_collision_center.y(),
                              p_other.m_collision_center.z(),
                              1.0);

    Eigen::Vector3d center1 = (worldTransform() * center1_h).head<3>();
    Eigen::Vector3d center2 = (p_other.worldTransform() * center2_h).head<3>();

    Eigen::Matrix3d rot1 =
        worldTransform().block<3, 3>(0, 0) * m_collision_orientation;
    Eigen::Matrix3d rot2 = p_other.worldTransform().block<3, 3>(0, 0) *
                           p_other.m_collision_orientation;

    Eigen::Vector3d half1(
        m_collision_params[0], m_collision_params[1], m_collision_params[2]);
    Eigen::Vector3d half2(p_other.m_collision_params[0],
                          p_other.m_collision_params[1],
                          p_other.m_collision_params[2]);

    // SAT (Separating Axis Theorem) test with 15 axes
    Eigen::Vector3d T = center2 - center1;

    // Test 15 axes
    for (int i = 0; i < 3; ++i)
    {
        // Axes from OBB1
        Eigen::Vector3d axis = rot1.col(i);
        double r1 = half1[i];
        double r2 = std::abs(half2[0] * rot2.col(0).dot(axis)) +
                    std::abs(half2[1] * rot2.col(1).dot(axis)) +
                    std::abs(half2[2] * rot2.col(2).dot(axis));
        if (std::abs(T.dot(axis)) > r1 + r2)
            return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        // Axes from OBB2
        Eigen::Vector3d axis = rot2.col(i);
        double r1 = std::abs(half1[0] * rot1.col(0).dot(axis)) +
                    std::abs(half1[1] * rot1.col(1).dot(axis)) +
                    std::abs(half1[2] * rot1.col(2).dot(axis));
        double r2 = half2[i];
        if (std::abs(T.dot(axis)) > r1 + r2)
            return false;
    }

    // Cross product axes
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            Eigen::Vector3d axis = rot1.col(i).cross(rot2.col(j));
            if (axis.squaredNorm() < 1e-10)
                continue;

            axis.normalize();
            double r1 =
                std::abs(half1[(i + 1) % 3] * rot1.col((i + 1) % 3).dot(axis)) +
                std::abs(half1[(i + 2) % 3] * rot1.col((i + 2) % 3).dot(axis));
            double r2 =
                std::abs(half2[(j + 1) % 3] * rot2.col((j + 1) % 3).dot(axis)) +
                std::abs(half2[(j + 2) % 3] * rot2.col((j + 2) % 3).dot(axis));
            if (std::abs(T.dot(axis)) > r1 + r2)
                return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------
bool Geometry::collideSphereCylinder(const Geometry& p_sphere,
                                     const Geometry& p_cylinder) const
{
    // Transform to world space
    Eigen::Vector4d sphere_center_h(p_sphere.m_collision_center.x(),
                                    p_sphere.m_collision_center.y(),
                                    p_sphere.m_collision_center.z(),
                                    1.0);
    Eigen::Vector4d cyl_center_h(p_cylinder.m_collision_center.x(),
                                 p_cylinder.m_collision_center.y(),
                                 p_cylinder.m_collision_center.z(),
                                 1.0);

    Eigen::Vector3d sphere_center =
        (p_sphere.worldTransform() * sphere_center_h).head<3>();
    Eigen::Vector3d cyl_center =
        (p_cylinder.worldTransform() * cyl_center_h).head<3>();

    Eigen::Matrix3d cyl_rot = p_cylinder.worldTransform().block<3, 3>(0, 0) *
                              p_cylinder.m_collision_orientation;
    Eigen::Vector3d cyl_axis = cyl_rot.col(2);

    double sphere_radius = p_sphere.m_collision_params[0];
    double cyl_radius = p_cylinder.m_collision_params[0];
    double cyl_half_height = p_cylinder.m_collision_params[1] / 2.0;

    // Find closest point on cylinder axis to sphere center
    Eigen::Vector3d to_sphere = sphere_center - cyl_center;
    double projection = to_sphere.dot(cyl_axis);
    projection = std::clamp(projection, -cyl_half_height, cyl_half_height);

    Eigen::Vector3d closest_on_axis = cyl_center + projection * cyl_axis;
    double distance = (sphere_center - closest_on_axis).norm();

    return distance <= (sphere_radius + cyl_radius);
}

// ----------------------------------------------------------------------------
bool Geometry::collideSphereOBB(const Geometry& p_sphere,
                                const Geometry& p_obb) const
{
    // Transform to world space
    Eigen::Vector4d sphere_center_h(p_sphere.m_collision_center.x(),
                                    p_sphere.m_collision_center.y(),
                                    p_sphere.m_collision_center.z(),
                                    1.0);
    Eigen::Vector4d obb_center_h(p_obb.m_collision_center.x(),
                                 p_obb.m_collision_center.y(),
                                 p_obb.m_collision_center.z(),
                                 1.0);

    Eigen::Vector3d sphere_center =
        (p_sphere.worldTransform() * sphere_center_h).head<3>();
    Eigen::Vector3d obb_center =
        (p_obb.worldTransform() * obb_center_h).head<3>();

    Eigen::Matrix3d obb_rot = p_obb.worldTransform().block<3, 3>(0, 0) *
                              p_obb.m_collision_orientation;
    Eigen::Vector3d half_extents(p_obb.m_collision_params[0],
                                 p_obb.m_collision_params[1],
                                 p_obb.m_collision_params[2]);
    double sphere_radius = p_sphere.m_collision_params[0];

    // Transform sphere center to OBB local space
    Eigen::Vector3d local_sphere =
        obb_rot.transpose() * (sphere_center - obb_center);

    // Find closest point on OBB to sphere center
    Eigen::Vector3d closest;
    for (int i = 0; i < 3; ++i)
    {
        closest[i] =
            std::clamp(local_sphere[i], -half_extents[i], half_extents[i]);
    }

    // Transform back to world space
    Eigen::Vector3d closest_world = obb_center + obb_rot * closest;
    double distance = (sphere_center - closest_world).norm();

    return distance <= sphere_radius;
}

// ----------------------------------------------------------------------------
bool Geometry::collideCylinderOBB(const Geometry& p_cylinder,
                                  const Geometry& p_obb) const
{
    // Simplified: treat cylinder as OBB
    // In a more sophisticated implementation, we could do proper cylinder-OBB
    // SAT For now, approximate cylinder with its bounding box

    Eigen::Vector4d cyl_center_h(p_cylinder.m_collision_center.x(),
                                 p_cylinder.m_collision_center.y(),
                                 p_cylinder.m_collision_center.z(),
                                 1.0);
    Eigen::Vector3d cyl_center =
        (p_cylinder.worldTransform() * cyl_center_h).head<3>();
    Eigen::Matrix3d cyl_rot = p_cylinder.worldTransform().block<3, 3>(0, 0) *
                              p_cylinder.m_collision_orientation;

    double radius = p_cylinder.m_collision_params[0];
    double half_height = p_cylinder.m_collision_params[1] / 2.0;

    // Create temporary OBB for cylinder
    Eigen::Vector3d cyl_half_extents(radius, radius, half_height);

    // Use OBB-OBB test
    Eigen::Vector4d obb_center_h(p_obb.m_collision_center.x(),
                                 p_obb.m_collision_center.y(),
                                 p_obb.m_collision_center.z(),
                                 1.0);
    Eigen::Vector3d obb_center =
        (p_obb.worldTransform() * obb_center_h).head<3>();
    Eigen::Matrix3d obb_rot = p_obb.worldTransform().block<3, 3>(0, 0) *
                              p_obb.m_collision_orientation;
    Eigen::Vector3d obb_half_extents(p_obb.m_collision_params[0],
                                     p_obb.m_collision_params[1],
                                     p_obb.m_collision_params[2]);

    // SAT test
    Eigen::Vector3d T = obb_center - cyl_center;

    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d axis = cyl_rot.col(i);
        double r1 = cyl_half_extents[i];
        double r2 = std::abs(obb_half_extents[0] * obb_rot.col(0).dot(axis)) +
                    std::abs(obb_half_extents[1] * obb_rot.col(1).dot(axis)) +
                    std::abs(obb_half_extents[2] * obb_rot.col(2).dot(axis));
        if (std::abs(T.dot(axis)) > r1 + r2)
            return false;
    }

    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d axis = obb_rot.col(i);
        double r1 = std::abs(cyl_half_extents[0] * cyl_rot.col(0).dot(axis)) +
                    std::abs(cyl_half_extents[1] * cyl_rot.col(1).dot(axis)) +
                    std::abs(cyl_half_extents[2] * cyl_rot.col(2).dot(axis));
        double r2 = obb_half_extents[i];
        if (std::abs(T.dot(axis)) > r1 + r2)
            return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
void Geometry::accept(NodeVisitor& visitor)
{
    visitor.visit(*this);
}

// ----------------------------------------------------------------------------
void Geometry::accept(ConstNodeVisitor& visitor) const
{
    visitor.visit(*this);
}

} // namespace robotik