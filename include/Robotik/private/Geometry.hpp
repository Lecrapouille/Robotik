#pragma once

#include "Robotik/private/Types.hpp"

namespace robotik
{

// ****************************************************************************
//! \brief Geometry data for collision detection and visualization.
// ****************************************************************************
struct Geometry
{
    enum class Type
    {
        BOX,
        CYLINDER,
        SPHERE,
        MESH
    };

    //! \brief Type of geometry (box, cylinder, sphere, mesh).
    Type type;
    //! \brief Parameters of the geometry (dimensions for box, radius for
    //! cylinder, radius for sphere).
    std::vector<double> parameters;
    //! \brief Path to the mesh file (if type is MESH).
    std::string mesh_path;
    //! \brief Color of the geometry (RGBA values).
    Eigen::Vector4d color;
    //! \brief Visual origin transformation (xyz and rpy).
    robotik::Transform visual_origin = robotik::Transform::Identity();
};

} // namespace robotik