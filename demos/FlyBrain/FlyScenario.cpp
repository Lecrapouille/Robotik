// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyTypes.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

namespace
{

//! @brief Scalar child, or @p_default when the key is missing.
double
number(bt::YamlNode const& p_node, std::string_view p_key, double p_default)
{
    return p_node.child(p_key).asDouble().value_or(p_default);
}

//! @brief Scalar child, or an empty string when the key is missing.
std::string text(bt::YamlNode const& p_node, std::string_view p_key)
{
    bt::YamlNode const child = p_node.child(p_key);
    return child.valid() ? child.scalar() : std::string{};
}

//! @brief Three-number sequence, or @p_default when the node is not one.
robotik::Vector3 vector3(bt::YamlNode const& p_node, robotik::Vector3 p_default)
{
    if (!p_node.isSeq() || p_node.size() != 3u)
    {
        return p_default;
    }
    return { p_node.child(0u).asDouble().value_or(p_default.x),
             p_node.child(1u).asDouble().value_or(p_default.y),
             p_node.child(2u).asDouble().value_or(p_default.z) };
}

} // namespace

FlyObservation FlyObservation::from(std::span<float const> p_values)
{
    if (p_values.size() < SIZE)
    {
        throw std::invalid_argument("Fly observation is shorter than 9 values");
    }

    // Create the observation.
    FlyObservation observation;
    observation.visual_left = p_values[0];
    observation.visual_center = p_values[1];
    observation.visual_right = p_values[2];
    observation.angular_velocity = p_values[3];
    observation.forward_velocity = p_values[4];
    observation.altitude = p_values[5];
    observation.distance_to_obstacle = p_values[6];
    observation.target_bearing = p_values[7];
    observation.target_distance = p_values[8];

    return observation;
}

void FlyObservation::write(std::span<float> p_values) const
{
    if (p_values.size() < SIZE)
    {
        throw std::invalid_argument(
            "Fly observation buffer is shorter than 9 values");
    }

    // Write the observation.
    p_values[0] = visual_left;
    p_values[1] = visual_center;
    p_values[2] = visual_right;
    p_values[3] = angular_velocity;
    p_values[4] = forward_velocity;
    p_values[5] = altitude;
    p_values[6] = distance_to_obstacle;
    p_values[7] = target_bearing;
    p_values[8] = target_distance;
}

FlyAction FlyAction::from(std::span<float const> p_values)
{
    if (p_values.size() < SIZE)
    {
        throw std::invalid_argument("Fly action is shorter than 3 values");
    }

    return { p_values[0], p_values[1], p_values[2] };
}

void FlyAction::write(std::span<float> p_values) const
{
    if (p_values.size() < SIZE)
    {
        throw std::invalid_argument(
            "Fly action buffer is shorter than 3 values");
    }

    // Write the action.
    p_values[0] = forward;
    p_values[1] = turn;
    p_values[2] = lift;
}

FlyScenario FlyScenario::load(std::filesystem::path const& p_path)
{
    // Parse the YAML file.
    auto parsed = bt::YamlDocument::parseFile(p_path.string());
    if (!parsed)
    {
        throw std::runtime_error("Cannot read scenario '" + p_path.string() +
                                 "': " + parsed.getError());
    }

    // Get the root node and the directory.
    bt::YamlNode const root = parsed.getValue().root();
    std::filesystem::path const directory =
        std::filesystem::absolute(p_path).parent_path();

    // Create the scenario.
    FlyScenario scenario;
    scenario.name = text(root, "scenario");
    if (scenario.name.empty())
    {
        scenario.name = "fly_obstacle_avoidance";
    }

    // Set the scenario parameters.
    scenario.seed = static_cast<std::uint64_t>(number(root, "seed", 123456.0));
    scenario.dt = number(root, "dt", 0.01);
    scenario.horizon = number(root, "horizon", 40.0);
    scenario.sensor_noise = number(root, "sensor_noise", 0.01);
    scenario.layout_jitter = number(root, "layout_jitter", 0.02);

    // Set the agent type.
    bt::YamlNode const agent = root.child("agent");
    if (agent.valid())
    {
        std::string const type = text(agent, "type");
        if (!type.empty())
        {
            scenario.agent = type;
        }
    }

    // Arena, start pose, obstacles and food live under "environment".
    bt::YamlNode const environment = root.child("environment");
    if (environment.valid())
    {
        scenario.arena = vector3(environment.child("size"), scenario.arena);
        bt::YamlNode const fly = environment.child("fly");
        if (fly.valid())
        {
            // Set the fly position and orientation.
            robotik::Vector3 const position =
                vector3(fly.child("position"),
                        { scenario.fly.x, scenario.fly.y, scenario.fly.z });
            robotik::Vector3 const orientation =
                vector3(fly.child("orientation"), robotik::zero3());

            scenario.fly.x = position.x;
            scenario.fly.y = position.y;
            scenario.fly.z = position.z;
            scenario.fly.yaw = orientation.z;
        }

        // Set the obstacles.
        if (environment.hasKey("obstacles"))
        {
            environment.child("obstacles")
                .forEachSeq(
                    [&](bt::YamlNode p_node)
                    {
                        FlyBox box;
                        box.position =
                            vector3(p_node.child("position"), robotik::zero3());
                        box.size =
                            vector3(p_node.child("size"), { 1.0, 1.0, 1.0 });
                        scenario.obstacles.push_back(box);
                    });
        }

        // Set the target.
        bt::YamlNode const target = environment.child("target");
        if (target.valid())
        {
            scenario.target =
                vector3(target.child("position"), scenario.target);
        }
    }

    // Bare file name: walk up to the repository data/ directory. A relative
    // path with a slash stays beside the scenario file.
    bt::YamlNode const robot = root.child("robot");
    std::filesystem::path model = text(robot, "model");
    if (model.empty())
    {
        model = "drosophila_x100.urdf";
    }
    if (model.is_relative() && model.filename() == model)
    {
        std::filesystem::path cursor = directory;
        std::filesystem::path found;
        for (int hop = 0; hop < 8 && !cursor.empty(); ++hop)
        {
            std::filesystem::path const candidate = cursor / "data" / model;
            std::error_code error;
            if (std::filesystem::is_regular_file(candidate, error))
            {
                found = candidate;
                break;
            }
            std::filesystem::path const parent = cursor.parent_path();
            if (parent == cursor)
            {
                break;
            }
            cursor = parent;
        }
        model = found.empty() ? directory / model : found;
    }
    else if (model.is_relative())
    {
        model = directory / model;
    }
    scenario.robot_model = std::filesystem::weakly_canonical(model);
    return scenario;
}
