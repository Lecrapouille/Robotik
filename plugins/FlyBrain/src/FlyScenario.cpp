// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyTypes.hpp"

#include <filesystem>
#include <stdexcept>
#include <string>

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

FlyScenario FlyScenario::from(robotik::CellDescription const& p_scenario)
{
    FlyScenario scenario;
    scenario.name = p_scenario.id;
    scenario.seed = p_scenario.seed;
    robotik::ConfigNode const& parameters = p_scenario.parameters;
    scenario.horizon = parameters.number("horizon", scenario.horizon);
    scenario.agent = parameters.text("brain", scenario.agent);
    if (scenario.agent != "shiu" && scenario.agent != "reflex")
    {
        throw std::runtime_error("parameters.brain must be shiu or reflex");
    }
    scenario.dt = parameters.number("dt", scenario.dt);
    scenario.sensor_noise =
        parameters.number("sensor_noise", scenario.sensor_noise);
    scenario.layout_jitter =
        parameters.number("layout_jitter", scenario.layout_jitter);
    scenario.arena = parameters.vector3("arena", scenario.arena);

    robotik::EntitySpec const& fly = p_scenario.world.robot();
    scenario.robot_model = fly.model;
    scenario.fly.x = fly.pose.position.x;
    scenario.fly.y = fly.pose.position.y;
    scenario.fly.z = fly.pose.position.z;
    scenario.fly.yaw = robotik::yaw(fly.pose.rotation).value();

    bool food = false;
    for (robotik::EntitySpec const& entity : p_scenario.world.entities)
    {
        if (entity.type != robotik::EntitySpec::Type::Prop)
        {
            continue;
        }
        if (entity.id == "food")
        {
            scenario.target = entity.pose.position;
            food = true;
        }
        else if (entity.id.starts_with("obstacle"))
        {
            scenario.obstacles.push_back(
                { entity.pose.position, entity.shape->size });
        }
    }
    if (!food)
    {
        throw std::runtime_error("The fly world needs a prop named food");
    }
    return scenario;
}
