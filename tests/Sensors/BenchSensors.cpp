// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Robot/OpenRobot.hpp"
#include "Robotik/Robot/Sensors/Imu.hpp"
#include "Robotik/Robot/Sensors/RangeScanner.hpp"

#include <chrono>
#include <iostream>
#include <string>

// ----------------------------------------------------------------------------
/// How long the sensors of a robot take per step. Disabled: run it by hand
/// with --gtest_also_run_disabled_tests --gtest_filter='SensorBench.*'.
TEST(SensorBench, DISABLED_ManySensorsOnOneRobot)
{
    robotik::RobotOptions options;
    options.physics = false; // measure the sensors, not MuJoCo
    auto arm = robotik::openRobot(repoFile("data/packs/robotik.robots/robot_6axis.urdf").string(),
                                  options);
    ASSERT_TRUE(arm) << arm.error();
    for (int i = 0; i < 64; ++i)
    {
        robotik::ImuConfig config;
        config.parent = "link6";
        config.frequency = 0.0; // every step
        (void)arm.value()->sensors().add<robotik::Imu>("imu" + std::to_string(i), config);
    }
    for (int i = 0; i < 8; ++i)
    {
        robotik::RangeScannerConfig config;
        config.beams = 32;
        config.frequency = 0.0;
        (void)arm.value()->sensors().add<robotik::RangeScanner>("lidar" + std::to_string(i), config);
    }
    int constexpr STEPS = 2000;
    auto const start = std::chrono::steady_clock::now();
    for (int i = 0; i < STEPS; ++i)
    {
        arm.value()->step(robotik::Seconds(0.001));
    }
    double const us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
    std::cout << "[BENCH] 64 IMU + 8 lidars: " << us / STEPS << " us per step\n";
}
