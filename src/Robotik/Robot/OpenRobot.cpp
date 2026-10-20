// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Robot/OpenRobot.hpp"

#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Robot/Backends/MujocoBackend.hpp"

#include <exception>

namespace robotik
{

// ----------------------------------------------------------------------------
Result<OpenedRobot> openRobot(std::string const& p_model, RobotOptions const& p_options)
{
    // The URDF and MuJoCo loaders report a bad file by throwing: this is the
    // border where it becomes a message.
    try
    {
        std::filesystem::path urdf = p_model;
        if (!std::filesystem::exists(urdf))
        {
            Result<std::filesystem::path> resolved = AssetResolver::standard().resolve(p_model, ".");
            if (!resolved)
            {
                return failure(p_model + ": " + resolved.error());
            }
            urdf = resolved.take();
        }
        OpenedRobot opened;
        opened.world = std::make_unique<compages::world::World>();
        opened.robot = std::make_unique<RobotSession>(*opened.world, urdf, nullptr, p_options.tool);
        if (p_options.physics)
        {
            MujocoOptions physics;
            physics.floating_base = p_options.floating_base;
            physics.floor = p_options.floor;
            auto backend = std::make_unique<MujocoBackend>(physics);
            std::string const arm = backend->load(urdf);
            if (!p_options.tool.empty())
            {
                backend->attach(arm, "flange", backend->load(p_options.tool), "tool_mount");
            }
            opened.robot->connect(std::move(backend));
        }
        opened.robot->hold(p_options.home);
        return opened;
    }
    catch (std::exception const& error)
    {
        return failure(p_model + ": " + error.what());
    }
}

} // namespace robotik
