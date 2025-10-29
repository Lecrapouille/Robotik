/**
 * @file Geometry.hpp
 * @brief Geometry classes for collision detection and/or visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include "Robotik/Core/Robot/Blueprint/Node.hpp"

#include <iostream>
namespace robotik
{

// ****************************************************************************
//! \brief Geometry for visualization and collision detection. Geometry are
//! stored in the blueprint tree as robot model. Geometry type and parameters
//! come from URDF file.
//!
//! This class represents a geometric shape for displaying a robot link. It can
//! be a box, a cylinder, a sphere, or a mesh. In the case of a mesh, a bounding
//! box is computed and used for collision detection. For other types, the
//! geometry is directly used for collision detection.
// ****************************************************************************
class Geometry: public Node
{
public:

    enum class Type
    {
        BOX,
        CYLINDER,
        SPHERE,
        MESH
    };

    // ------------------------------------------------------------------------
    //! \brief Common constructor for any kind of geometry with name, type,
    //! parameters, and mesh path.
    // ------------------------------------------------------------------------
    Geometry(std::string const& p_name,
             Type p_type,
             std::vector<double>&& p_parameters,
             std::string p_mesh_path)
        : Node(p_name),
          m_type(p_type),
          m_parameters(std::move(p_parameters)),
          m_mesh_path(p_mesh_path)
    {
    }

    // ------------------------------------------------------------------------
    //! \brief Get the geometry's type.
    //! \return The geometry's type.
    // ------------------------------------------------------------------------
    Type type() const
    {
        return m_type;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the geometry's parameters (if type is BOX, CYLINDER, or
    //! SPHERE).
    //! \return The geometry's parameters.
    //!  for box: size (x, y, z)
    //!  for cylinder: radius, length
    //!  for sphere: radius
    // ------------------------------------------------------------------------
    std::vector<double> const& parameters() const
    {
        return m_parameters;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the geometry's mesh path (if type is MESH).
    //! \return The geometry's mesh path.
    // ------------------------------------------------------------------------
    std::string const& meshPath() const
    {
        return m_mesh_path;
    }

    // ------------------------------------------------------------------------
    //! \brief Set collision data (called by Link constructor).
    //! \param p_center Center of the collision volume in local frame.
    //! \param p_orientation Orientation of the collision volume in local frame.
    //! \param p_params Collision parameters (dimensions).
    // ------------------------------------------------------------------------
    void setCollisionData(const Eigen::Vector3d& p_center,
                          const Eigen::Matrix3d& p_orientation,
                          const std::vector<double>& p_params)
    {
        m_collision_center = p_center;
        m_collision_orientation = p_orientation;
        m_collision_params = p_params;
    }

    // ------------------------------------------------------------------------
    //! \brief Check collision with another geometry.
    //! \param p_other The other geometry to check collision with.
    //! \return True if collision detected, false otherwise.
    // ------------------------------------------------------------------------
    bool collide(const Geometry& p_other) const;

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

private:

    // Helper methods for collision detection
    bool collideSpheres(const Geometry& p_other) const;
    bool collideCylinders(const Geometry& p_other) const;
    bool collideOBBs(const Geometry& p_other) const;
    bool collideSphereCylinder(const Geometry& p_sphere,
                               const Geometry& p_cylinder) const;
    bool collideSphereOBB(const Geometry& p_sphere,
                          const Geometry& p_obb) const;
    bool collideCylinderOBB(const Geometry& p_cylinder,
                            const Geometry& p_obb) const;

public:

    //! \brief Color of the geometry (RGB values).
    Eigen::Vector3f color = Eigen::Vector3f(0.5, 0.5, 0.5);

protected:

    //! \brief Type of geometry (box, cylinder, sphere, mesh).
    Type m_type;
    //! \brief Parameters of the geometry (dimensions for box, radius for
    //! cylinder, radius for sphere).
    std::vector<double> m_parameters;
    //! \brief Path to the mesh file (if type is MESH).
    std::string m_mesh_path;
    // Collision data (OBB representation)
    //! \brief Center of the collision volume in local frame.
    Eigen::Vector3d m_collision_center;
    //! \brief Orientation of the collision volume in local frame (for OBB).
    Eigen::Matrix3d m_collision_orientation;
    //! \brief Collision parameters (half-extents for box/OBB, radius+length for
    //! cylinder, radius for sphere).
    std::vector<double> m_collision_params;
};

// ****************************************************************************
//! \brief Box geometry.
// ****************************************************************************
class Box: public Geometry
{
public:

    Box(std::string const& p_name, double p_x, double p_y, double p_z)
        : Geometry(p_name, Type::BOX, std::vector<double>{ p_x, p_y, p_z }, "")
    {
    }
};

// ****************************************************************************
//! \brief Cylinder geometry.
// ****************************************************************************
class Cylinder: public Geometry
{
public:

    Cylinder(std::string const& p_name, double p_radius, double p_length)
        : Geometry(p_name,
                   Type::CYLINDER,
                   std::vector<double>{ p_radius, p_length },
                   "")
    {
    }
};

// ****************************************************************************
//! \brief Sphere geometry.
// ****************************************************************************
class Sphere: public Geometry
{
public:

    Sphere(std::string const& p_name, double p_radius)
        : Geometry(p_name, Type::SPHERE, std::vector<double>{ p_radius }, "")
    {
    }
};

// ****************************************************************************
//! \brief Mesh geometry.
// ****************************************************************************
class Mesh: public Geometry
{
public:

    Mesh(std::string const& p_name, std::string p_mesh_path)
        : Geometry(p_name, Type::MESH, {}, p_mesh_path)
    {
        std::cout << "Mesh: " << p_name << " " << p_mesh_path << std::endl;
    }
};

} // namespace robotik