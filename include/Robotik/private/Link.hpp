#pragma once

#include "Robotik/private/Geometry.hpp"
#include "Robotik/private/Inertial.hpp"
#include "Robotik/private/Node.hpp"

namespace robotik
{

class Link: public Node
{
public:

    using Ptr = std::unique_ptr<Link>;

    Link(std::string_view const& p_name,
         Geometry const& p_geometry,
         Inertial const& p_inertial);

    inline Geometry const& geometry() const
    {
        return m_geometry;
    }

    inline Inertial const& inertial() const
    {
        return m_inertial;
    }

private:

    Geometry m_geometry;
    Inertial m_inertial;
};

} // namespace robotik