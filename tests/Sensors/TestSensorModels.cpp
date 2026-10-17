// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "PickPlaceMission.hpp"

#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Config.hpp"
#include "Robotik/Scenario/World.hpp"
#include "Robotik/Sensors/Imu.hpp"
#include "Robotik/Sensors/Noise.hpp"

#include "Compages/World/World.hpp"

TEST(NoiseModel, IdealChannelDrawsNothing)
{
    robotik::Random used(robotik::Seed{ 3 });
    robotik::Random untouched(robotik::Seed{ 3 });
    robotik::NoiseModel const ideal;
    EXPECT_TRUE(ideal.ideal());
    EXPECT_DOUBLE_EQ(ideal.apply(1.25, used), 1.25);
    // The stream was not consumed: both generators still agree.
    EXPECT_DOUBLE_EQ(used.uniform(), untouched.uniform());
}

TEST(NoiseModel, BiasResolutionAndSaturation)
{
    robotik::Random random(robotik::Seed{ 1 });
    robotik::NoiseModel model;
    model.bias = 0.3;
    model.resolution = 0.25;
    model.min = -1.0;
    model.max = 1.0;
    EXPECT_DOUBLE_EQ(model.apply(0.1, random), 0.5); // 0.4 rounded to 0.25 steps
    EXPECT_DOUBLE_EQ(model.apply(5.0, random), 1.0);
    EXPECT_DOUBLE_EQ(model.apply(-5.0, random), -1.0);

    robotik::NoiseModel const noisy = robotik::NoiseModel::gaussian(0.1);
    double sum = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        sum += noisy.apply(2.0, random);
    }
    EXPECT_NEAR(sum / 2000.0, 2.0, 0.02);
}

TEST(LatencyBuffer, DeliversAfterTheLatencyWithTheCaptureStamp)
{
    robotik::LatencyBuffer<robotik::ImuReading> buffer;
    buffer.latency(robotik::Seconds(0.05));
    robotik::ImuReading reading;
    reading.stamp = robotik::Seconds(0.01);
    reading.valid = true;
    buffer.push(reading.stamp, reading);
    EXPECT_FALSE(buffer.advance(robotik::Seconds(0.05)));
    EXPECT_FALSE(buffer.current().valid);
    EXPECT_TRUE(buffer.advance(robotik::Seconds(0.06)));
    EXPECT_TRUE(buffer.current().valid);
    EXPECT_DOUBLE_EQ(buffer.current().stamp.value(), 0.01);
}

TEST(Imu, ReadingHasAHeaderALatencyAndGoesInvalidOnFailure)
{
    compages::world::World world;
    robotik::RobotSession robot(world, assetFile("robotik.robots.simple_revolute_robot"));
    robotik::ImuConfig config;
    config.parent = "base_link";
    config.frequency = 100.0;
    config.latency = robotik::Seconds(0.03);
    robotik::Imu& imu = robot.sensors().add<robotik::Imu>("imu", config);

    robot.step(robotik::Seconds(0.01));
    robot.step(robotik::Seconds(0.01));
    EXPECT_FALSE(imu.reading().valid) << "nothing delivered within the latency";
    robot.step(robotik::Seconds(0.01));
    robot.step(robotik::Seconds(0.01));
    ASSERT_TRUE(imu.reading().valid);
    EXPECT_EQ(imu.reading().frame, "imu");
    EXPECT_EQ(imu.reading().sequence, 1u);
    EXPECT_DOUBLE_EQ(imu.reading().stamp.value(), 0.01);
    // At rest the accelerometer reads the reaction to gravity, upward.
    EXPECT_NEAR(imu.reading().acceleration.z, 9.80665, 1e-6);

    robot.resources().fail("imu");
    robot.step(robotik::Seconds(0.01));
    EXPECT_FALSE(imu.reading().valid);
}

TEST(World, EverySensorTypeIsDeclaredInYaml)
{
    robotik::ConfigNode const world = robotik::ConfigNode::parse(R"(
id: lab
entities:
  - id: robot
    type: robot
    model: robotik.robots.robot_6axis
    sensors:
      eye: { type: rgbd, parent: link6, fov: 60, resolution: [64, 48] }
      imu:
        type: imu
        parent: base_link
        frequency: 200
        latency: 0.002
        accelerometer: { sigma: 0.02, bias: 0.05, saturation: [-160, 160] }
        gyroscope: 0.001
      lidar: { type: range, parent: base_link, beams: 90, fov: 180, max_range: 5,
               noise: { sigma: 0.01, resolution: 0.001 } }
      wrist: { type: force_torque, link: link6, force: 0.5, torque: 0.02 }
)");
    robotik::WorldDescription const description =
        robotik::WorldDescription::read(world, ".", testAssets());
    std::vector<robotik::SensorSpec> const& sensors = description.robot().sensors;
    ASSERT_EQ(sensors.size(), 4u);
    EXPECT_EQ(sensors[0].type, robotik::SensorSpec::Type::Camera);
    EXPECT_TRUE(sensors[0].camera.depth);
    EXPECT_EQ(sensors[1].type, robotik::SensorSpec::Type::Imu);
    EXPECT_DOUBLE_EQ(sensors[1].imu.latency.value(), 0.002);
    EXPECT_DOUBLE_EQ(sensors[1].imu.accelerometer.bias, 0.05);
    EXPECT_DOUBLE_EQ(sensors[1].imu.accelerometer.max, 160.0);
    EXPECT_DOUBLE_EQ(sensors[1].imu.gyroscope.sigma, 0.001);
    EXPECT_EQ(sensors[2].type, robotik::SensorSpec::Type::Range);
    EXPECT_EQ(sensors[2].range.beams, 90u);
    EXPECT_DOUBLE_EQ(sensors[2].range.noise.resolution, 0.001);
    EXPECT_EQ(sensors[3].type, robotik::SensorSpec::Type::ForceTorque);
    EXPECT_EQ(sensors[3].force_torque.link, "link6");

    robotik::ConfigNode bad = world;
    bad.set("entities.robot.sensors.wrist.link", robotik::ConfigNode(""));
    EXPECT_THROW((void)robotik::WorldDescription::read(bad, ".", testAssets()), std::runtime_error);
    bad = world;
    bad.set("entities.robot.sensors.eye.latency", robotik::ConfigNode("0.1"));
    EXPECT_THROW((void)robotik::WorldDescription::read(bad, ".", testAssets()), std::runtime_error);
    bad = world;
    bad.set("entities.robot.sensors.eye.type", robotik::ConfigNode("sonar"));
    EXPECT_THROW((void)robotik::WorldDescription::read(bad, ".", testAssets()), std::runtime_error);
}

TEST(Simulation, GroundTruthIsNotGivenUnlessTheScenarioAsks)
{
    auto build = [](char const* p_policy)
    {
        robotik::ScenarioSource source = robotik::ScenarioSource::load(
            repoFile("plugins/PickAndPlace/scenarios/pick_and_place.yml"));
        source.override("scenario.perception.oracle", robotik::ConfigNode(p_policy));
        return robotik::Scenario::read(source, testAssets());
    };
    {
        PickPlaceMission mission;
        compages::world::World world;
        robotik::Simulation blind(world, build("never"), nullptr, &mission);
        EXPECT_FALSE(blind.oracle());
        // Headless, no image: the robot never sees the cube.
        for (int i = 0; i < 300 && !blind.finished(); ++i)
        {
            blind.step(robotik::Seconds(0.01));
        }
        blind.conclude();
        EXPECT_FALSE(blind.passed());
        EXPECT_FALSE(blind.worldModel().find("red_cube")->observed());
    }
    {
        PickPlaceMission mission;
        compages::world::World world;
        robotik::Simulation oracle(world, build("when_blind"), nullptr, &mission);
        EXPECT_TRUE(oracle.oracle());
    }
}
