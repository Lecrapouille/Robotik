/**
 * @file WaypointManager.cpp
 * @brief Implementation of WaypointManager.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Robot/WaypointManager.hpp"
#include "Robotik/Core/Common/Conversions.hpp"

#include <iostream>

namespace robotik
{

// Initialize static member
std::vector<WaypointManager::Waypoint> const WaypointManager::s_empty_waypoints;

// ----------------------------------------------------------------------------
size_t WaypointManager::addWaypoint(Robot const& p_robot,
                                    Node const* p_end_effector,
                                    std::string const& p_label,
                                    double p_duration)
{
    if (!p_end_effector)
    {
        std::cerr << "⚠️ Cannot add waypoint: end effector is null" << std::endl;
        return 0;
    }

    // Create a waypoint with the current robot state
    Waypoint waypoint;
    waypoint.position = p_robot.states().joint_positions;
    waypoint.velocity.resize(waypoint.position.size(), 0.0);
    waypoint.acceleration.resize(waypoint.position.size(), 0.0);
    waypoint.duration = p_duration;

    // Get or create the waypoints vector for this end effector
    auto& waypoints = m_waypoints[p_end_effector];
    waypoints.push_back(waypoint);

    // Set label (default to auto-generated name if empty)
    size_t waypoint_index = waypoints.size() - 1;
    waypoints[waypoint_index].label =
        p_label.empty() ? ("waypoint_" + std::to_string(waypoint_index))
                        : p_label;

    std::cout << "📍 Recorded waypoint " << waypoint_index << ": "
              << waypoints[waypoint_index].label << " (duration: " << p_duration
              << "s)" << std::endl;

    return waypoint_index;
}

// ----------------------------------------------------------------------------
void WaypointManager::deleteWaypoint(Node const* p_end_effector, size_t p_index)
{
    if (!p_end_effector)
        return;

    auto it = m_waypoints.find(p_end_effector);
    if (it == m_waypoints.end() || p_index >= it->second.size())
        return;

    it->second.erase(it->second.begin() + static_cast<long>(p_index));

    std::cout << "🗑️ Deleted waypoint " << p_index << std::endl;
}

// ----------------------------------------------------------------------------
void WaypointManager::clearWaypoints(Node const* p_end_effector)
{
    if (!p_end_effector)
        return;

    auto it = m_waypoints.find(p_end_effector);
    if (it != m_waypoints.end())
    {
        it->second.clear();
        std::cout << "🗑️ Cleared all waypoints for end effector" << std::endl;
    }
}

// ----------------------------------------------------------------------------
void WaypointManager::clearAllWaypoints()
{
    m_waypoints.clear();
    std::cout << "🗑️ Cleared all waypoints" << std::endl;
}

// ----------------------------------------------------------------------------
std::vector<WaypointManager::Waypoint> const&
WaypointManager::getWaypoints(Node const* p_end_effector) const
{
    if (!p_end_effector)
        return s_empty_waypoints;

    auto it = m_waypoints.find(p_end_effector);
    if (it == m_waypoints.end())
        return s_empty_waypoints;

    return it->second;
}

// ----------------------------------------------------------------------------
size_t WaypointManager::size(Node const* p_end_effector) const
{
    if (!p_end_effector)
        return 0;

    auto it = m_waypoints.find(p_end_effector);
    if (it == m_waypoints.end())
        return 0;

    return it->second.size();
}

// ----------------------------------------------------------------------------
bool WaypointManager::hasWaypoints(Node const* p_end_effector) const
{
    if (!p_end_effector)
        return false;

    auto it = m_waypoints.find(p_end_effector);
    return it != m_waypoints.end() && !it->second.empty();
}

// ----------------------------------------------------------------------------
bool WaypointManager::isWaypointReached(Robot const& p_robot,
                                        Node const* p_end_effector,
                                        Waypoint const& p_waypoint,
                                        double p_position_tolerance,
                                        double p_orientation_tolerance)
{
    if (!p_end_effector)
        return false;

    // Check that waypoint has valid joint positions
    if (p_waypoint.position.size() != p_robot.states().joint_positions.size())
        return false;

    // Get current end-effector pose
    Transform current_transform = p_end_effector->worldTransform();
    Pose current_pose = transformToPose(current_transform);

    // Calculate target pose by temporarily applying waypoint positions
    // We need a non-const robot to do this, but we'll restore the state
    Robot& mutable_robot = const_cast<Robot&>(p_robot);

    // Save current joint positions
    auto current_joint_positions = p_robot.states().joint_positions;

    // Apply waypoint joint positions temporarily
    mutable_robot.setJointPositions(p_waypoint.position);

    // Calculate target pose
    Transform target_transform = p_end_effector->worldTransform();
    Pose target_pose = transformToPose(target_transform);

    // Restore original joint positions
    mutable_robot.setJointPositions(current_joint_positions);

    // Calculate pose error
    Pose error = calculatePoseError(target_pose, current_pose);

    // Check if error is within tolerance
    double position_error = error.head<3>().norm();
    double orientation_error = error.tail<3>().norm();

    return (position_error < p_position_tolerance &&
            orientation_error < p_orientation_tolerance);
}

} // namespace robotik
