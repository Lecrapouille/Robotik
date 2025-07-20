#include "Robotik/private/Link.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Link::Link(std::string_view const& p_name,
           Geometry const& p_geometry,
           Inertial const& p_inertial)
    : Node(p_name), m_geometry(p_geometry), m_inertial(p_inertial)
{
}

} // namespace robotik