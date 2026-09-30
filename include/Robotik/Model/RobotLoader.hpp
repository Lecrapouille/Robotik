// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file RobotLoader.hpp
 * @brief URDF load into Compages and ECS binding to Pinocchio and MuJoCo.
 */

#pragma once

#include <string>

namespace compages::renderer
{
class Scene;
}

namespace compages::world
{
class World;
}

namespace robotik
{

class MujocoBackend;
class PinocchioBackend;

/**
 * @brief Instantiates one robot URDF and wires joint entities to both backends.
 *
 * Joint names are the single key: Pinocchio and MuJoCo indices are stored
 * separately on each link. Gripper and end-effector components are inferred
 * from URDF naming heuristics.
 *
 * @example
 * @code
 * compages::world::World world;
 * compages::renderer::Scene scene(world);
 * robotik::PinocchioBackend pin("arm.urdf");
 * robotik::MujocoBackend mujoco("arm.urdf");
 * robotik::RobotLoader{}.instantiate(world, &scene, pin, &mujoco, "arm.urdf");
 * @endcode
 */
class RobotLoader
{
public:

    /**
     * @brief Loads URDF into @p_scene or headless @p_world and attaches ECS data.
     * @param p_world Target world.
     * @param p_scene Scene for meshes, or null for headless load only.
     * @param p_pinocchio Already constructed backend for the same URDF.
     * @param p_mujoco MuJoCo backend, or null to skip dynamics bindings.
     * @param p_filename URDF path.
     * @throws std::runtime_error on parse failure or missing links.
     */
    void instantiate(compages::world::World& p_world,
                     compages::renderer::Scene* p_scene,
                     PinocchioBackend& p_pinocchio,
                     MujocoBackend* p_mujoco,
                     std::string const& p_filename);
};

} // namespace robotik
