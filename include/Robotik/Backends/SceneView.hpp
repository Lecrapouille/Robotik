// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file SceneView.hpp
//! @brief Rendering hooks, implemented by graphical applications.
#pragma once

#include "Compages/Renderer/Assets/UrdfLoader.hpp"
#include "Compages/World/Entity.hpp"

#include <filesystem>
#include <stdexcept>

namespace compages::world
{
class World;
}

namespace robotik
{

class Camera;
class FrameSource;
class Image;

namespace ecs
{
struct SceneObject;
}

// ****************************************************************************
//! @brief What a renderer adds to the ECS world built by Robotik.
//!
//! The library stays headless: without a view, robots and objects are plain
//! entities and cameras have no image source. The simulator implements these
//! hooks with Compages meshes and offscreen rendering.
// ****************************************************************************
class SceneView
{
public:

    virtual ~SceneView() = default;

    //! @brief Loads the URDF with its meshes. @throws std::runtime_error.
    virtual compages::world::Entity robot(compages::world::World& p_world,
                                          std::filesystem::path const& p_urdf) = 0;

    //! @brief Loads a URDF on its own. The caller parents @c tool_mount.
    virtual compages::world::Entity model(compages::world::World& p_world,
                                          std::filesystem::path const& p_urdf)
    {
        auto loaded = compages::renderer::loadUrdf(p_world, p_urdf.string());
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        return loaded.value();
    }

    //! @brief Adds the meshes of a scenario object to @p_entity.
    virtual void object(compages::world::Entity /*p_entity*/,
                        ecs::SceneObject const& /*p_object*/)
    {
        /* no-op */
    }

    //! @brief Image source of @p_camera mounted on @p_link, or null.
    virtual FrameSource* camera(Camera& /*p_camera*/,
                                compages::world::Entity /*p_link*/)
    {
        return nullptr;
    }

    // -------------------------------------------------------------------------
    //! @brief Paints @p_image on the ground, centered on the world origin.
    //! @param p_width Extent along the world X axis (m).
    //! @param p_height Extent along the world Y axis (m); row 0 of the image
    //! is the +Y edge.
    // -------------------------------------------------------------------------
    virtual void ground(Image const& /*p_image*/,
                        double /*p_width*/,
                        double /*p_height*/)
    {
        /* no-op */
    }

    [[nodiscard]] virtual bool hasGround() const
    {
        return false;
    }
};

} // namespace robotik
