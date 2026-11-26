// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file SceneView.hpp
/// What a window adds to a simulation: 3D models, camera images, a ground
/// texture.
///
/// The simulation works without it (headless): a graphical host implements
/// these hooks, the library only calls them when a view is given.
///
/// @par Contract
/// - Promises: every call may be skipped: a null @ref robotik::SceneView is a
///   headless run and the simulation does not change.
/// - Expects: the simulation calls the view after it changed the world, never
///   the reverse.
/// - Errors: a 3D file that cannot be loaded throws @c std::runtime_error.
#pragma once

#include "Compages/World/Import/UrdfLoader.hpp"
#include "Compages/World/Entity.hpp"
#include "RobotIK/Common/Math.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

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
struct Shape;
}

// ****************************************************************************
/// What a renderer adds to the ECS world built by RobotIK.
///
/// The library stays headless: without a view, robots and objects are plain
/// entities and cameras have no image source. The simulator implements these
/// hooks with Compages meshes and offscreen rendering.
///
/// Code that gets a @c SceneView* must accept null (headless run).
///
/// @code
/// class MyView final : public robotik::SceneView
/// {
/// public:
///     compages::world::Entity robot(compages::world::World& p_world,
///                                   std::filesystem::path const& p_urdf)
///                                   override
///     {
///         return loadWithMeshes(p_world, p_urdf);
///     }
/// };
///
/// void MyLogic::setup(robotik::RoboticCell& p_cell, robotik::SceneView*
/// p_view)
/// {
///     if (p_view != nullptr)
///         p_view->openCanvas(board, 0.6, 0.4);
/// }
/// @endcode
// ****************************************************************************
class SceneView
{
public:

    virtual ~SceneView() = default;

    // -------------------------------------------------------------------------
    /// Loads a robot URDF with its meshes.
    /// @param p_world World receiving the entities.
    /// @param p_urdf URDF file.
    /// @return The root entity of the robot.
    /// @throws std::runtime_error when the file cannot be loaded.
    // -------------------------------------------------------------------------
    virtual compages::world::Entity
    robot(compages::world::World& p_world,
          std::filesystem::path const& p_urdf) = 0;

    // -------------------------------------------------------------------------
    /// Loads a URDF on its own (a tool, a prop). The caller parents it, for
    /// example to @c tool_mount.
    /// @param p_world World receiving the entities.
    /// @param p_urdf URDF file.
    /// @return The root entity of the model.
    /// @throws std::runtime_error when the file cannot be loaded.
    // -------------------------------------------------------------------------
    virtual compages::world::Entity model(compages::world::World& p_world,
                                          std::filesystem::path const& p_urdf)
    {
        auto loaded = compages::world::loadUrdf(p_world, p_urdf);
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        return loaded.value();
    }

    // -------------------------------------------------------------------------
    /// Adds the meshes of a scenario object to its entity.
    /// @param p_entity Entity of the object.
    /// @param p_object Shape, size and color to draw.
    // -------------------------------------------------------------------------
    virtual void object(compages::world::Entity /*p_entity*/,
                        ecs::Shape const& /*p_object*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Gives a camera its images.
    /// @param p_camera Camera sensor.
    /// @param p_link Link entity carrying the camera.
    /// @return The image source, or null when the view renders nothing.
    // -------------------------------------------------------------------------
    virtual FrameSource* camera(Camera& /*p_camera*/,
                                compages::world::Entity /*p_link*/)
    {
        return nullptr;
    }

    // -------------------------------------------------------------------------
    /// Paints an image on the ground, centered on the world origin (a track
    /// to follow, a map).
    /// @param p_image Image to paint.
    /// @param p_width Extent along the world X axis (m).
    /// @param p_height Extent along the world Y axis (m); row 0 of the image
    ///   is the +Y edge.
    // -------------------------------------------------------------------------
    virtual void
    ground(Image const& /*p_image*/, double /*p_width*/, double /*p_height*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// @return True when a ground image was painted.
    // -------------------------------------------------------------------------
    [[nodiscard]] virtual bool hasGround() const
    {
        return false;
    }

    // -------------------------------------------------------------------------
    /// Opens a paintable sheet on the +X face of an entity. Headless views
    /// ignore it.
    /// @param p_parent Entity carrying the sheet.
    /// @param p_width Y extent of the parent (m).
    /// @param p_height Z extent of the parent (m).
    /// @return True when the sheet was opened.
    // -------------------------------------------------------------------------
    virtual bool openCanvas(compages::world::Entity /*p_parent*/,
                            double /*p_width*/,
                            double /*p_height*/)
    {
        return false;
    }

    // -------------------------------------------------------------------------
    /// Stamps ink at a point of the sheet opened by @ref openCanvas.
    /// @param p_y Metres from the parent origin along its Y axis.
    /// @param p_z Metres from the parent origin along its Z axis.
    /// @param p_red Red in [0, 1].
    /// @param p_green Green in [0, 1].
    /// @param p_blue Blue in [0, 1].
    // -------------------------------------------------------------------------
    virtual void ink(double /*p_y*/,
                     double /*p_z*/,
                     float /*p_red*/,
                     float /*p_green*/,
                     float /*p_blue*/)
    {
        // Nothing to do by default.
    }
    // --- Actors (people, vehicles moved by a story) -------------------------

    // -------------------------------------------------------------------------
    /// Draws an actor with a glTF model that has clips (Idle, Walk...).
    /// The model hangs from @p_entity, which faces +X with Z up; glTF files
    /// are Y up and face +Z, so the view turns the model to match, then by
    /// @p_yaw about Z. Headless views draw nothing.
    /// @param p_entity Entity of the actor, moved by the story.
    /// @param p_model glTF or GLB file.
    /// @param p_yaw Extra turn of the model about Z (rad).
    /// @param p_scale Scale of the model.
    /// @throws std::runtime_error when the file cannot be loaded.
    // -------------------------------------------------------------------------
    virtual void actor(compages::world::Entity /*p_entity*/,
                       std::filesystem::path const& /*p_model*/,
                       double /*p_yaw*/,
                       double /*p_scale*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Plays a clip on an actor; called at each step, so it must be cheap
    /// when nothing changed.
    /// @param p_entity Entity of the actor.
    /// @param p_clip Name of the clip in the model file; empty: none.
    /// @param p_fade Time for the clip before to fade out.
    /// @param p_loop The clip starts again at its end.
    /// @param p_ground_speed Speed of the body (m/s) for a walk or a run:
    ///   the view plays the clip so the feet do not slide; 0 plays it at
    ///   its own speed.
    // -------------------------------------------------------------------------
    virtual void animate(compages::world::Entity /*p_entity*/,
                         std::string const& /*p_clip*/,
                         Seconds /*p_fade*/,
                         bool /*p_loop*/,
                         double /*p_ground_speed*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Shows what an actor says, above its head, for a time.
    /// @param p_entity Entity of the actor.
    /// @param p_text The sentence; empty hides it.
    /// @param p_height Where to write it above the entity (m).
    // -------------------------------------------------------------------------
    virtual void say(compages::world::Entity /*p_entity*/,
                     std::string const& /*p_text*/,
                     double /*p_height*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Sets a face expression of an actor (morph targets of the model).
    /// @param p_entity Entity of the actor.
    /// @param p_name Expression (smile, angry...).
    /// @param p_weight Weight in [0, 1]; 0 removes it.
    // -------------------------------------------------------------------------
    virtual void expression(compages::world::Entity /*p_entity*/,
                            std::string const& /*p_name*/,
                            double /*p_weight*/)
    {
        /* no-op */
    }

    // -------------------------------------------------------------------------
    /// Shows or hides an actor (it entered or left the world).
    /// @param p_entity Entity of the actor.
    /// @param p_shown True to draw it.
    // -------------------------------------------------------------------------
    virtual void shown(compages::world::Entity /*p_entity*/, bool /*p_shown*/)
    {
        /* no-op */
    }
};

} // namespace robotik
