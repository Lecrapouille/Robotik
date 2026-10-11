// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Scenario/Trajectory.hpp"

#include "Robotik/Scenario/Config.hpp"

#include <stdexcept>

namespace robotik
{

namespace
{

std::vector<double> numbers(ConfigNode const& p_node)
{
    std::vector<double> result;
    for (ConfigNode const& item : p_node.items())
    {
        std::optional<double> const value = item.asNumber();
        if (!value)
        {
            throw std::runtime_error("'" + item.scalar() + "' is not a number");
        }
        result.push_back(*value);
    }
    return result;
}

} // namespace

Trajectory Trajectory::load(std::filesystem::path const& p_path)
{
    Trajectory trajectory;
    trajectory.file = std::filesystem::absolute(p_path).lexically_normal();
    try
    {
        ConfigNode const root = ConfigNode::load(trajectory.file);
        ConfigNode const& node = root.at("trajectory");
        if (!node.isMap())
        {
            throw std::runtime_error("no 'trajectory:' section");
        }
        if (!node.at("points").isSequence() || node.at("points").items().empty())
        {
            throw std::runtime_error("points must be a non-empty list");
        }
        std::string const type = node.text("type", "cartesian");
        if (type == "cartesian")
        {
            trajectory.type = Type::Cartesian;
            trajectory.frame = node.text("frame");
            Quaternion const orientation = node.pose("orientation").rotation;
            for (ConfigNode const& point : node.at("points").items())
            {
                if (point.isMap())
                {
                    trajectory.poses.push_back(point.asPose());
                    continue;
                }
                std::vector<double> const xyz = numbers(point);
                if (xyz.size() != 3u)
                {
                    throw std::runtime_error("a cartesian point is [x, y, z] or a pose");
                }
                trajectory.poses.push_back(Pose{ Vector3(xyz[0], xyz[1], xyz[2]), orientation });
            }
        }
        else if (type == "joint")
        {
            trajectory.type = Type::Joint;
            trajectory.joints = node.strings("joints");
            if (trajectory.joints.empty())
            {
                throw std::runtime_error("a joint trajectory names its joints");
            }
            for (ConfigNode const& point : node.at("points").items())
            {
                std::vector<double> configuration = numbers(point);
                if (configuration.size() != trajectory.joints.size())
                {
                    throw std::runtime_error("every point gives one value per joint");
                }
                trajectory.configurations.push_back(std::move(configuration));
            }
        }
        else
        {
            throw std::runtime_error("type must be cartesian or joint");
        }
    }
    catch (std::runtime_error const& error)
    {
        throw std::runtime_error(trajectory.file.string() + ": " + error.what());
    }
    return trajectory;
}

} // namespace robotik
