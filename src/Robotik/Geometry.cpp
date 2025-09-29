/**
 * @file Geometry.cpp
 * @brief Geometry classes for collision detection and/or visualization.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/private/Geometry.hpp"

#include <iomanip>
#include <sstream>

namespace robotik
{

#if 0
// ----------------------------------------------------------------------------
std::string Geometry::debug(bool /*p_detailed*/) const
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    switch (m_type)
    {
        case Type::BOX:
            oss << "box";
            if (m_parameters.size() >= 3)
            {
                oss << "(x=" << m_parameters[0] << ", y=" << m_parameters[1]
                    << ", z=" << m_parameters[2] << ")";
            }
            break;
        case Type::CYLINDER:
            oss << "cylinder";
            if (m_parameters.size() >= 2)
            {
                oss << "(r=" << m_parameters[0] << ", h=" << m_parameters[1]
                    << ")";
            }
            break;
        case Type::SPHERE:
            oss << "sphere";
            if (m_parameters.size() >= 1)
            {
                oss << "(r=" << m_parameters[0] << ")";
            }
            break;
        case Type::MESH:
            oss << "mesh";
            if (!m_mesh_path.empty())
            {
                oss << "(" << m_mesh_path << ")";
            }
            break;
        default:
            break;
    }

    oss << " color(" << color.x() << ", " << color.y() << ", " << color.z()
        << ")";

    return oss.str();
}
#endif

} // namespace robotik