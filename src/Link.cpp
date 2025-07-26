#include "Robotik/private/Link.hpp"

namespace robotik
{

// ----------------------------------------------------------------------------
Geometry const& Link::geometry() const
{
    // Return the first child that shall exist.
    auto const& children = scene::Node::children();
    return *dynamic_cast<Geometry const*>(children[0].get());
}

// ----------------------------------------------------------------------------
Geometry const& Link::collision() const
{
    // Return the second child if it exists and is a geometry.
    if (auto const& children = scene::Node::children(); children.size() > 1u)
    {
        return *dynamic_cast<Geometry const*>(children[1].get());
    }

    // Fallback: return the visual geometry (1st child).
    return geometry();
}

} // namespace robotik