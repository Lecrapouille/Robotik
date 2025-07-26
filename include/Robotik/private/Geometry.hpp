#pragma once

#include "Robotik/private/SceneNode.hpp"
#include "Robotik/private/Types.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Geometry data for collision detection and/or visualization.
// ****************************************************************************
class Geometry: public scene::Node
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
    //! \brief Constructor with name and type.
    // ------------------------------------------------------------------------
    explicit Geometry(std::string_view const& p_name,
                      Type p_type,
                      std::vector<double>&& p_parameters,
                      std::string_view p_mesh_path)
        : scene::Node(p_name),
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
    //! \brief Get the geometry's debug string.
    //! \param p_detailed Whether to include detailed information
    //! \return The geometry's debug string
    // ------------------------------------------------------------------------
    // std::string debug(bool p_detailed) const override;

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
};

// ****************************************************************************
//! \brief Box geometry.
// ****************************************************************************
class Box: public Geometry
{
public:

    Box(std::string_view const& p_name, std::vector<double>&& p_parameters)
        : Geometry(p_name, Type::BOX, std::move(p_parameters), "")
    {
    }
};

// ****************************************************************************
//! \brief Cylinder geometry.
// ****************************************************************************
class Cylinder: public Geometry
{
public:

    Cylinder(std::string_view const& p_name, std::vector<double>&& p_parameters)
        : Geometry(p_name, Type::CYLINDER, std::move(p_parameters), "")
    {
    }
};

// ****************************************************************************
//! \brief Sphere geometry.
// ****************************************************************************
class Sphere: public Geometry
{
public:

    Sphere(std::string_view const& p_name, std::vector<double>&& p_parameters)
        : Geometry(p_name, Type::SPHERE, std::move(p_parameters), "")
    {
    }
};

// ****************************************************************************
//! \brief Mesh geometry.
// ****************************************************************************
class Mesh: public Geometry
{
public:

    Mesh(std::string_view const& p_name, std::string_view p_mesh_path)
        : Geometry(p_name, Type::MESH, {}, p_mesh_path)
    {
    }
};

} // namespace robotik