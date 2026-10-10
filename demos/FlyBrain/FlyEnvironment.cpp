// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyEnvironment.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{

constexpr double BODY_RADIUS_M = 0.16;
constexpr double REACH_M = 0.45;
constexpr double REACH_Z_M = 0.4;
constexpr double GROUND_Z_M = 0.19;
constexpr std::size_t TRAIL_CAP = 240;

//! @brief Brings an angle back to (-pi, pi], for the food bearing.
double wrapPi(double p_angle)
{
    constexpr double pi = 3.14159265358979323846;
    while (p_angle > pi)
    {
        p_angle -= 2.0 * pi;
    }
    while (p_angle < -pi)
    {
        p_angle += 2.0 * pi;
    }
    return p_angle;
}

//! @brief Distance in the ground plane. Height is checked apart from this.
double horizontal(FlyPlant const& p_plant, robotik::Vector3 const& p_target)
{
    double const dx = p_target.x - p_plant.x;
    double const dy = p_target.y - p_plant.y;
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

FlyEnvironment::FlyEnvironment(FlyScenario p_scenario,
                               std::uint32_t p_max_steps)
    : m_scenario(std::move(p_scenario)), m_max_steps(p_max_steps)
{
    if (m_scenario.dt <= 0.0)
    {
        throw std::invalid_argument("Fly scenario dt must be positive");
    }
    if (m_max_steps == 0u)
    {
        m_max_steps =
            static_cast<std::uint32_t>(m_scenario.horizon / m_scenario.dt);
    }
}

void FlyEnvironment::reset(robotik::Seed p_seed, std::span<float> p_observation)
{
    // "world" and "sensors" are separate streams, so one does not shift the
    // other.
    m_obstacles = m_scenario.obstacles;
    robotik::Random layout(p_seed.derive("world"));
    double const jitter = m_scenario.layout_jitter;

    // Jitter the obstacles.
    for (FlyBox& box : m_obstacles)
    {
        box.position.x += layout.uniform(-jitter, jitter);
        box.position.y += layout.uniform(-jitter, jitter);
    }

    // Create the noise generator.
    m_noise = robotik::Random(p_seed.derive("sensors"));

    // Reset the controller and the snapshot.
    m_controller.reset();
    m_snapshot.observation = {};
    m_snapshot.action = {};
    m_snapshot.posture = {};
    m_snapshot.vision.left = 0.0f;
    m_snapshot.vision.center = 0.0f;
    m_snapshot.vision.right = 0.0f;
    m_snapshot.vision.distance = 0.0f;

    // Reset the vision origins and ends.
    for (int ray = 0; ray < 3; ++ray)
    {
        m_snapshot.vision.origins[ray] = robotik::zero3();
        m_snapshot.vision.ends[ray] = robotik::zero3();
    }

    // Reset the collision and reach flags, the reward, the steps, the plant,
    // the trail and the previous distance.
    m_snapshot.collided = false;
    m_snapshot.reached = false;
    m_snapshot.reward = 0.0;
    m_snapshot.steps = 0;
    m_snapshot.plant = m_scenario.fly;

    m_trail.clear();
    m_trail.push_back(robotik::Vector3(
        m_snapshot.plant.x, m_snapshot.plant.y, m_snapshot.plant.z));
    m_previous_distance = horizontal(m_snapshot.plant, m_scenario.target);
    sense();
    write(p_observation);
}

//! @brief Eyes first, then the quantities the eyes do not measure.
void FlyEnvironment::sense()
{
    // Sense the vision.
    FlyPlant const& plant = m_snapshot.plant;
    m_snapshot.vision = senseFly(plant,
                                 m_obstacles,
                                 &m_noise,
                                 m_scenario.sensor_noise,
                                 m_snapshot.posture.neck_yaw);

    // Calculate the target bearing.
    double const dx = m_scenario.target.x - plant.x;
    double const dy = m_scenario.target.y - plant.y;

    // Write the observation.
    FlyObservation& observation = m_snapshot.observation;
    observation.visual_left = m_snapshot.vision.left;
    observation.visual_center = m_snapshot.vision.center;
    observation.visual_right = m_snapshot.vision.right;
    observation.angular_velocity = static_cast<float>(plant.yaw_rate);
    observation.forward_velocity = static_cast<float>(plant.speed);
    observation.altitude = static_cast<float>(plant.z);
    observation.distance_to_obstacle = m_snapshot.vision.distance;
    observation.target_bearing =
        static_cast<float>(wrapPi(std::atan2(dy, dx) - plant.yaw));
    observation.target_distance =
        static_cast<float>(horizontal(plant, m_scenario.target));
}

void FlyEnvironment::separate(bool& p_collided)
{
    FlyPlant& plant = m_snapshot.plant;

    // Keep the thorax inside the arena and outside every box.
    // Sets @p_collided when a limit had to push it.
    // x runs along the arena. y is centred, so the side walls sit at ±width/2.
    double const margin = m_scenario.arena.y * 0.5;
    double const limits[4] = { -1.0, m_scenario.arena.x, -margin, margin };
    double* coordinates[2] = { &plant.x, &plant.y };

    // Check the x and y coordinates.
    for (int axis = 0; axis < 2; ++axis)
    {
        double& value = *coordinates[axis];
        if (value < limits[axis * 2] + BODY_RADIUS_M)
        {
            value = limits[axis * 2] + BODY_RADIUS_M;
            p_collided = true;
            plant.speed *= 0.2;
        }
        else if (value > limits[axis * 2 + 1] - BODY_RADIUS_M)
        {
            value = limits[axis * 2 + 1] - BODY_RADIUS_M;
            p_collided = true;
            plant.speed *= 0.2;
        }
    }

    // Check the z coordinate.
    if (plant.z > m_scenario.arena.z)
    {
        plant.z = m_scenario.arena.z;
        plant.vertical_speed = std::min(0.0, plant.vertical_speed);
    }

    // Check the z coordinate.
    if (plant.z < GROUND_Z_M)
    {
        plant.z = GROUND_Z_M;
        plant.vertical_speed = std::max(0.0, plant.vertical_speed);
    }

    // Check the obstacles.
    for (FlyBox const& box : m_obstacles)
    {
        robotik::Vector3 const half(
            box.size.x * 0.5, box.size.y * 0.5, box.size.z * 0.5);
        double const lower[3] = { box.position.x - half.x,
                                  box.position.y - half.y,
                                  box.position.z - half.z };
        double const upper[3] = { box.position.x + half.x,
                                  box.position.y + half.y,
                                  box.position.z + half.z };
        double point[3] = { plant.x, plant.y, plant.z };

        double closest[3];
        // closest is the point of the box nearest the thorax. inside means
        // the thorax centre is in the box, so that point is the centre itself
        // until the nearest face replaces it.
        bool inside = true;
        for (int axis = 0; axis < 3; ++axis)
        {
            closest[axis] = std::clamp(point[axis], lower[axis], upper[axis]);
            inside = inside && point[axis] > lower[axis] &&
                     point[axis] < upper[axis];
        }

        // Calculate the closest point and whether the thorax is inside the box.
        if (inside)
        {
            int best = 0;
            double penetration = point[0] - lower[0];
            double face = lower[0];
            for (int axis = 0; axis < 3; ++axis)
            {
                if (point[axis] - lower[axis] < penetration)
                {
                    penetration = point[axis] - lower[axis];
                    best = axis;
                    face = lower[axis];
                }
                if (upper[axis] - point[axis] < penetration)
                {
                    penetration = upper[axis] - point[axis];
                    best = axis;
                    face = upper[axis];
                }
            }
            closest[0] = point[0];
            closest[1] = point[1];
            closest[2] = point[2];
            closest[best] = face;
        }
        double const delta[3] = { point[0] - closest[0],
                                  point[1] - closest[1],
                                  point[2] - closest[2] };
        double const distance = std::sqrt(
            delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (!inside && distance >= BODY_RADIUS_M)
        {
            continue;
        }

        // If the thorax is outside the box, set the collision flag and reduce
        // the speed.
        p_collided = true;
        plant.speed *= 0.3;

        // Outside the box, delta points out of it. Inside, the way out is
        // toward the nearest face, so the opposite direction.
        double const sign = inside ? -1.0 : 1.0;
        double const travel =
            inside ? distance + BODY_RADIUS_M : BODY_RADIUS_M - distance;
        if (distance > 1.0e-6)
        {
            double const scale = sign * travel / distance;
            plant.x += delta[0] * scale;
            plant.y += delta[1] * scale;
            plant.z += delta[2] * scale;
        }
        else
        {
            plant.z += BODY_RADIUS_M;
        }
    }
}

robotik::StepResult FlyEnvironment::step(std::span<float const> p_action,
                                         std::span<float> p_observation)
{
    // Apply the action.
    m_snapshot.action = FlyAction::from(p_action);
    m_controller.apply(m_snapshot.action, m_snapshot.plant, m_scenario.dt);
    m_snapshot.posture = m_controller.posture();

    // Separate the thorax from the obstacles.
    bool collided = false;
    separate(collided);
    m_snapshot.collided = collided;

    // Sense the vision.
    sense();

    // Calculate the step.
    robotik::Vector3 const here(
        m_snapshot.plant.x, m_snapshot.plant.y, m_snapshot.plant.z);
    robotik::Vector3 const& last = m_trail.back();
    double const step_x = here.x - last.x;
    double const step_y = here.y - last.y;
    double const step_z = here.z - last.z;

    // A crumb every 8 cm. The cap drops the oldest sample.
    if (step_x * step_x + step_y * step_y + step_z * step_z > 0.08 * 0.08)
    {
        m_trail.push_back(here);
        if (m_trail.size() > TRAIL_CAP)
        {
            m_trail.erase(m_trail.begin());
        }
    }
    ++m_snapshot.steps;

    // Calculate the distance to the target and the progress toward it.
    double const distance = horizontal(m_snapshot.plant, m_scenario.target);
    double const progress = m_previous_distance - distance;
    m_previous_distance = distance;

    // Check if the thorax has reached the target.
    bool const reached =
        distance < REACH_M &&
        std::fabs(m_snapshot.plant.z - m_scenario.target.z) < REACH_Z_M;
    m_snapshot.reached = reached;

    // Progress toward the food, a small time cost, a collision penalty, a
    // bonus once the thorax is there.
    double reward = progress - 0.002;
    if (collided)
    {
        reward -= 0.15;
    }
    if (reached)
    {
        reward += 5.0;
    }
    m_snapshot.reward = reward;
    write(p_observation);

    // Create the result.
    robotik::StepResult result;
    result.reward = static_cast<float>(reward);
    result.terminated = reached;
    result.truncated = !reached && m_snapshot.steps >= m_max_steps;
    return result;
}

void FlyEnvironment::write(std::span<float> p_observation) const
{
    m_snapshot.observation.write(p_observation);
}
