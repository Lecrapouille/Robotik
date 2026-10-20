// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file World.hpp
/// Reads a world file: what physically exists at the start of a run.
///
/// The data it fills (@ref robotik::WorldDescription) is plain C++ in the Task
/// layer; this file only knows the YAML format below.
///
/// A world file lists entities. The URDF of a model describes its structure;
/// the world says which instance is present, where, and what it hangs from.
///
/// @code
/// world:
///   id: painting_lab
///   floor: { friction: 0.8 }
///   entities:
///     - id: robot
///       type: robot                       # the controlled robot (one)
///       model: robotik.robots.robot_6axis
///       home: { joint2: 0.3, joint3: 1.3, joint5: 1.54 }
///       sensors:
///         wrist_camera: { type: camera, parent: link6, fov: 70 }
///       actuators:
///         arm: { type: joint_group }
///     - id: brush
///       type: tool                        # keeps its identity once mounted
///       model: robotik.tools.default
///       attach: { parent: robot.flange, frame: tool_mount }
///     - id: easel
///       type: body                        # simulated rigid body
///       model: robotik.painter.easel
///       physics: free                     # free | fixed
///       pose: { position: [0.65, 0.0, 0.0], rpy: [0.0, 0.0, 3.14159] }
///     - id: canvas
///       type: body
///       shape: { type: box, size: [0.02, 0.40, 0.50] }
///       mass: 0.5
///       attach:
///         parent: easel.canvas_mount
///         transform: { position: [0.011, 0.0, 0.251] }
///     - id: red_cube
///       type: prop                        # moved by the grasp system only
///       shape: { type: box, size: [0.04, 0.04, 0.04] }
///       pose: { position: [0.40, 0.20, 0.02] }
///       randomize: { x: [-0.02, 0.02], y: [-0.02, 0.02] }
/// @endcode
///
/// Frames are Z-up, in metres and radians. A tool that is not attached is in
/// storage: it is not built, but a host may mount it instead of another.
///
/// @par Contract
/// - Promises: @c loadWorld and @c readWorld return a @c Result.
/// - Expects: paths start at the package root.
#pragma once

#include "Robotik/Common/Result.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Task/CellDescription.hpp"

#include <filesystem>

namespace robotik
{

// ----------------------------------------------------------------------------
/// Reads the @c world: node of a file.
/// @param p_world The @c world: node.
/// @param p_directory Package root; file names are relative to it.
/// @param p_assets Resolves the model names.
/// @return The world; the error names the entity and the field at fault.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<WorldDescription> readWorld(ConfigNode const& p_world,
                                         std::filesystem::path const& p_directory,
                                         AssetResolver const& p_assets);

// ----------------------------------------------------------------------------
/// Reads a world file (its root holds @c world:).
/// @param p_path A @c x.world.yaml file.
/// @param p_assets Resolves the model names.
/// @return The world; the error names the file and the field at fault.
// ----------------------------------------------------------------------------
[[nodiscard]] Result<WorldDescription> loadWorld(std::filesystem::path const& p_path,
                                         AssetResolver const& p_assets);

} // namespace robotik
