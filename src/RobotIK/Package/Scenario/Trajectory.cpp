// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "RobotIK/Package/Scenario/Trajectory.hpp"

#include "RobotIK/Common/Config.hpp"

namespace robotik
{

/// Reads a YAML sequence of numbers.
static Result<std::vector<double>> numbers(ConfigNode const& p_node)
{
    std::vector<double> result;
    for (ConfigNode const& item : p_node.items())
    {
        std::optional<double> const value = item.asNumber();
        if (!value)
        {
            return failure("'" + item.scalar() + "' is not a number");
        }
        result.push_back(*value);
    }
    return result;
}

/// Reads the trajectory section; errors are prefixed by the caller.
static Result<void> readTrajectory(Trajectory& p_trajectory)
{
    Trajectory& trajectory = p_trajectory;
    ConfigNode root;
    ROBOTIK_TRY_ASSIGN(root, ConfigNode::load(trajectory.file));
    ROBOTIK_TRY(root.checkKeys({ "trajectory" }, "file"));
    ConfigNode const& node = root.at("trajectory");
    ROBOTIK_TRY(node.checkKeys(
        { "type", "frame", "orientation", "joints", "points" }, "trajectory"));
    if (!node.isMap())
    {
        return failure("no 'trajectory:' section");
    }
    if (!node.at("points").isSequence() || node.at("points").items().empty())
    {
        return failure("points must be a non-empty list");
    }
    std::string const type = node.text("type", "cartesian");
    if (type == "cartesian")
    {
        trajectory.type = Trajectory::Type::Cartesian;
        trajectory.frame = node.text("frame");
        Pose orientation;
        ROBOTIK_TRY_ASSIGN(orientation, node.pose("orientation"));
        for (ConfigNode const& point : node.at("points").items())
        {
            Pose pose{ Vector3::ZERO, orientation.rotation };
            if (point.isMap())
            {
                ROBOTIK_TRY_ASSIGN(pose, point.asPose());
                trajectory.poses.push_back(pose);
                continue;
            }
            std::vector<double> xyz;
            ROBOTIK_TRY_ASSIGN(xyz, numbers(point));
            if (xyz.size() != 3U)
            {
                return failure("a cartesian point is [x, y, z] or a pose");
            }
            pose.position = Vector3(xyz[0], xyz[1], xyz[2]);
            trajectory.poses.push_back(pose);
        }
    }
    else if (type == "joint")
    {
        trajectory.type = Trajectory::Type::Joint;
        trajectory.joints = node.strings("joints");
        if (trajectory.joints.empty())
        {
            return failure("a joint trajectory names its joints");
        }
        for (ConfigNode const& point : node.at("points").items())
        {
            std::vector<double> configuration;
            ROBOTIK_TRY_ASSIGN(configuration, numbers(point));
            if (configuration.size() != trajectory.joints.size())
            {
                return failure("every point gives one value per joint");
            }
            trajectory.configurations.push_back(std::move(configuration));
        }
    }
    else
    {
        return failure("type must be cartesian or joint");
    }
    return success();
}

Result<Trajectory> loadTrajectory(std::filesystem::path const& p_path)
{
    Trajectory trajectory;
    trajectory.file = std::filesystem::absolute(p_path).lexically_normal();
    if (Result<void> const read = readTrajectory(trajectory); !read)
    {
        return failure(trajectory.file.string() + ": " + read.error());
    }
    return trajectory;
}

} // namespace robotik
