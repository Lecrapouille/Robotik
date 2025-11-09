/**
 * @file WaypointManager.cpp
 * @brief Implementation of WaypointManager.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Managers/WaypointManager.hpp"
#include "Robotik/Core/Common/Conversions.hpp"

#include <iostream>

namespace robotik
{

// Initialize static members
std::vector<WaypointManager::Waypoint> const WaypointManager::s_empty_waypoints;
std::vector<Eigen::Matrix4f> const WaypointManager::s_empty_cache;

// ----------------------------------------------------------------------------
size_t WaypointManager::addWaypoint(Robot const& p_robot,
                                    Node const& p_end_effector,
                                    std::string const& p_label,
                                    double p_duration)
{
    // Create a waypoint with the current robot state
    Waypoint waypoint;
    waypoint.position = p_robot.states().joint_positions;
    waypoint.velocity.resize(waypoint.position.size(), 0.0);
    waypoint.acceleration.resize(waypoint.position.size(), 0.0);
    waypoint.duration = p_duration;
    waypoint.pose = transformToPose(p_end_effector.worldTransform());

    // Get or create the waypoints vector for this end effector
    auto& waypoints = m_waypoints[&p_end_effector];
    waypoints.push_back(waypoint);

    // Set label (default to auto-generated name if empty)
    size_t waypoint_index = waypoints.size() - 1;
    waypoints[waypoint_index].label =
        p_label.empty() ? ("waypoint_" + std::to_string(waypoint_index))
                        : p_label;

    std::cout << "📍 Recorded waypoint " << waypoint_index << ": "
              << waypoints[waypoint_index].label << " (duration: " << p_duration
              << "s)" << std::endl;

    // Update render cache
    updateRenderCache(p_end_effector);

    return waypoint_index;
}

// ----------------------------------------------------------------------------
void WaypointManager::deleteWaypoint(Node const& p_end_effector, size_t p_index)
{
    auto it = m_waypoints.find(&p_end_effector);
    if (it == m_waypoints.end() || p_index >= it->second.size())
        return;

    it->second.erase(it->second.begin() + static_cast<long>(p_index));

    std::cout << "🗑️ Deleted waypoint " << p_index << std::endl;

    // Update render cache
    updateRenderCache(p_end_effector);
}

// ----------------------------------------------------------------------------
void WaypointManager::clearWaypoints(Node const& p_end_effector)
{
    auto it = m_waypoints.find(&p_end_effector);
    if (it != m_waypoints.end())
    {
        it->second.clear();
        std::cout << "🗑️ Cleared all waypoints for end effector" << std::endl;

        // Update render cache
        updateRenderCache(p_end_effector);
    }
}

// ----------------------------------------------------------------------------
void WaypointManager::clearAllWaypoints()
{
    m_waypoints.clear();
    m_render_cache.clear();
    std::cout << "🗑️ Cleared all waypoints" << std::endl;
}

// ----------------------------------------------------------------------------
std::vector<WaypointManager::Waypoint> const&
WaypointManager::getWaypoints(Node const& p_end_effector) const
{
    auto it = m_waypoints.find(&p_end_effector);
    if (it == m_waypoints.end())
        return s_empty_waypoints;

    return it->second;
}

// ----------------------------------------------------------------------------
size_t WaypointManager::size(Node const& p_end_effector) const
{
    auto it = m_waypoints.find(&p_end_effector);
    if (it == m_waypoints.end())
        return 0;

    return it->second.size();
}

// ----------------------------------------------------------------------------
bool WaypointManager::hasWaypoints(Node const& p_end_effector) const
{
    auto it = m_waypoints.find(&p_end_effector);
    return it != m_waypoints.end() && !it->second.empty();
}

// ----------------------------------------------------------------------------
bool WaypointManager::isWaypointReached(Robot const& p_robot,
                                        Node const& p_end_effector,
                                        Waypoint const& p_waypoint,
                                        double p_position_tolerance,
                                        double p_orientation_tolerance)
{
    // Check that waypoint has valid joint positions
    if (p_waypoint.position.size() != p_robot.states().joint_positions.size())
        return false;

    // Get current end-effector pose
    Transform current_transform = p_end_effector.worldTransform();
    Pose current_pose = transformToPose(current_transform);

    // Calculate pose error
    Pose error = calculatePoseError(p_waypoint.pose, current_pose);

    // Check if error is within tolerance
    double position_error = error.head<3>().norm();
    double orientation_error = error.tail<3>().norm();

    return (position_error < p_position_tolerance &&
            orientation_error < p_orientation_tolerance);
}

// ----------------------------------------------------------------------------
void WaypointManager::updateRenderCache(Node const& p_end_effector)
{
    auto it = m_waypoints.find(&p_end_effector);
    if (it == m_waypoints.end())
    {
        m_render_cache.erase(&p_end_effector);
        return;
    }

    auto const& waypoints = it->second;
    auto& cache = m_render_cache[&p_end_effector];
    cache.clear();
    cache.reserve(waypoints.size());

    // Convert each waypoint pose to a transformation matrix
    for (auto const& wp : waypoints)
    {
        auto transform = poseToTransform(wp.pose);
        cache.emplace_back(transform.cast<float>());
    }
}

// ----------------------------------------------------------------------------
std::vector<Eigen::Matrix4f> const&
WaypointManager::getRenderCache(Node const& p_end_effector) const
{
    auto it = m_render_cache.find(&p_end_effector);
    if (it == m_render_cache.end())
        return s_empty_cache;

    return it->second;
}

// ----------------------------------------------------------------------------
void WaypointManager::onRobotAdded(Robot* p_robot)
{
    if (p_robot)
    {
        std::cout << "WaypointManager: Robot added: " << p_robot->name()
                  << std::endl;
        // Track the robot (for future cleanup if needed)
        // Currently we don't need to do anything specific here
    }
}

// ----------------------------------------------------------------------------
void WaypointManager::onRobotRemoved(std::string const& p_robot_name)
{
    std::cout << "WaypointManager: Robot removed: " << p_robot_name
              << std::endl;

    // Note: We don't have direct robot pointer mapping anymore
    // Waypoints are keyed by Node*, so they'll naturally be invalidated
    // when the robot is destroyed. For now, we rely on proper cleanup
    // when robots are removed.
}

} // namespace robotik
