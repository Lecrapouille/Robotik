// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Robot/StandaloneRobot.hpp"

#include "RobotIK/Robot/Assets/AssetResolver.hpp"

#include <exception>

namespace robotik
{

// ----------------------------------------------------------------------------
StandaloneRobot::StandaloneRobot()
    : m_world(std::make_unique<compages::world::World>())
{
}

// ----------------------------------------------------------------------------
StandaloneRobot::~StandaloneRobot()
{
    // The runtime refers to the world: it goes first.
    m_runtime.reset();
}

// ----------------------------------------------------------------------------
Result<StandaloneRobot> StandaloneRobot::open(std::string const& p_model)
{
    return open(p_model, Setup{});
}

// ----------------------------------------------------------------------------
Result<StandaloneRobot> StandaloneRobot::open(std::string const& p_model,
                                              Setup const& p_setup)
{
    // The URDF and MuJoCo loaders report a bad file by throwing: this is the
    // border where it becomes a message.
    try
    {
        std::filesystem::path urdf = p_model;
        if (!std::filesystem::exists(urdf))
        {
            Result<std::filesystem::path> resolved =
                AssetResolver::standard().resolve(p_model, ".");
            if (!resolved)
            {
                return failure(p_model + ": " + resolved.error());
            }
            urdf = resolved.take();
        }
        StandaloneRobot robot;
        robot.m_runtime = std::make_unique<RobotRuntime>(
            *robot.m_world, urdf, nullptr, p_setup.tool);
        if (p_setup.physics)
        {
            auto backend = std::make_unique<MujocoBackend>(*p_setup.physics);
            (void)backend->loadRobot(urdf, p_setup.tool);
            robot.m_runtime->connect(std::move(backend));
        }
        robot.m_runtime->hold(p_setup.home);
        return robot;
    }
    catch (std::exception const& error)
    {
        return failure(p_model + ": " + error.what());
    }
}

} // namespace robotik
