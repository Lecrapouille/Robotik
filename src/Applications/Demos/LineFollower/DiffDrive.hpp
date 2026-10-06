// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Robot/Robot.hpp"

#include <string>

// Planar pose of the robot base in the map.
struct Pose2
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

// Wheel layout of a differential drive robot.
struct DriveGeometry
{
    double radius = 0.1;
    double track = 0.3;
    // Axle midpoint along the base X axis (m).
    double axle = -0.15;
    // Base frame origin above the floor (m).
    double height = 0.075;
    // The URDF wheel axes point to -Y: positive joint speed drives backward.
    double sign = -1.0;

    // Forward speed (m/s) and yaw rate (rad/s) of wheel speeds (rad/s).
    void twist(double p_left, double p_right, double& p_speed, double& p_yaw_rate) const;
    // Wheel speeds (rad/s) of a forward speed and a yaw rate.
    void wheels(double p_speed, double p_yaw_rate, double& p_left, double& p_right) const;
    // Base pose after @p_dt at constant twist (exact arc of the axle midpoint).
    [[nodiscard]] Pose2 integrate(Pose2 const& p_pose,
                                  double p_speed,
                                  double p_yaw_rate,
                                  double p_dt) const;
};

// Kinematic backend: wheels follow their velocity targets through a first
// order lag, the base rolls without slipping. Holds the ground truth pose.
class DiffDriveBackend final: public robotik::RobotBackend
{
public:

    DiffDriveBackend(std::string p_left, std::string p_right, DriveGeometry p_geometry = {});

    void attach(robotik::Robot& p_robot) override;
    void reset(robotik::Robot& p_robot) override;
    void step(robotik::Robot& p_robot, Seconds p_dt) override;

    void start(Pose2 const& p_pose)
    {
        m_start = p_pose;
    }

    [[nodiscard]] Pose2 const& truth() const
    {
        return m_truth;
    }

    // Base frame in the map, at its height above the floor.
    [[nodiscard]] robotik::Pose pose() const;

    [[nodiscard]] DriveGeometry const& geometry() const
    {
        return m_geometry;
    }

private:

    std::string m_left_name;
    std::string m_right_name;
    DriveGeometry m_geometry;
    robotik::JointId m_left = robotik::NO_JOINT;
    robotik::JointId m_right = robotik::NO_JOINT;
    double m_speeds[2] = { 0.0, 0.0 };
    Pose2 m_start;
    Pose2 m_truth;
};
