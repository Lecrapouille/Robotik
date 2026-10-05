// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Line follower demo: a differential drive robot localizes itself on AprilTags
// printed on the floor, reaches a black line and follows it for a lap.
//
// The camera is simulated by casting rays on a procedural floor map (no GPU).
// OpenCV (line detection, --view windows) and the AprilRobotics library are
// only used here, never in the Robotik library.

#include "DiffDrive.hpp"
#include "Floor.hpp"
#include "Skills.hpp"
#include "Track.hpp"
#include "Vision.hpp"

#include "Robotik/Behavior/SkillNodes.hpp"
#include "Robotik/Math/Random.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Scheduler.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include "Compages/World/World.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

#define DEMO_DT_S 0.01
#define DEMO_MAX_TIME_S 150.0
#define TAG_SIZE_M 0.16
#define TAG_OFFSET_M 0.30
#define FOLLOW_SPEED 0.30
#define CROSS_TRACK_LIMIT_M 0.08
// Time to settle on the line before the cross track error counts.
#define CROSS_TRACK_SETTLE_S 2.0
#define CAMERA_PITCH_DEG 40.0
#define MAP_VIEW_SCALE 0.5

static char const* const TREE = R"(
BehaviorTree:
  Sequence:
    name: LineFollower
    children:
      - Action:
          name: Localize
      # Losing the line fails FollowLine: reach it again and go on.
      - UntilSuccess:
          attempts: 3
          child:
            - Sequence:
                children:
                  - Action:
                      name: ReachLine
                  - Action:
                      name: FollowLine
)";

struct Options
{
    std::uint64_t seed = 1;
    double laps = 1.0;
    bool view = false;
    std::filesystem::path save;
    std::filesystem::path urdf = "data/simple_diff_drive_robot.urdf";
};

struct Statistic
{
    double sum = 0.0;
    double max = 0.0;
    std::size_t count = 0;

    void add(double p_value)
    {
        sum += p_value;
        max = std::max(max, p_value);
        ++count;
    }

    [[nodiscard]] double mean() const
    {
        return count > 0u ? sum / static_cast<double>(count) : 0.0;
    }
};

static bool parse(int argc, char** argv, Options& p_options)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string const arg = argv[i];
        if (arg == "--view")
        {
            p_options.view = true;
        }
        else if (arg == "--seed" && i + 1 < argc)
        {
            p_options.seed = std::strtoull(argv[++i], nullptr, 10);
        }
        else if (arg == "--laps" && i + 1 < argc)
        {
            p_options.laps = std::strtod(argv[++i], nullptr);
        }
        else if (arg == "--save" && i + 1 < argc)
        {
            p_options.save = argv[++i];
        }
        else if (arg == "--urdf" && i + 1 < argc)
        {
            p_options.urdf = argv[++i];
        }
        else
        {
            std::cerr << "Usage: " << argv[0] << " [--seed N] [--laps X] [--view] [--save map.png] [--urdf path]\n";
            return false;
        }
    }
    return true;
}

// Tags 30 cm beside the line, alternately on its right (outside of the
// loop) and on its left (inside).
static std::vector<FloorTag> placeTags(Track const& p_track)
{
    std::vector<FloorTag> tags;
    int const count = 12;
    for (int id = 0; id < count; ++id)
    {
        Track::Point const p = p_track.at((id + 0.5) * p_track.perimeter() / count);
        double const side = (id % 2 == 0) ? TAG_OFFSET_M : -TAG_OFFSET_M;
        tags.push_back({ id, p.x + side * std::sin(p.heading), p.y - side * std::cos(p.heading) });
    }
    return tags;
}

static robotik::CameraConfig frontCamera()
{
    double const pitch = CAMERA_PITCH_DEG * std::numbers::pi / 180.0;
    double const c = std::cos(pitch);
    double const s = std::sin(pitch);
    robotik::CameraConfig config;
    config.parent = "base_link";
    // Optical frame: Z forward and tilted down, X to the right of the robot.
    config.mount = robotik::Pose{ { 0.2, 0.0, 0.2 },
                                  robotik::Quaternion::basis({ 0.0, -1.0, 0.0 }, { -s, 0.0, -c }, { c, 0.0, -s }) };
    config.intrinsics = robotik::CameraIntrinsics::fromFov(320u, 240u, Radians(60.0 * std::numbers::pi / 180.0));
    config.frequency = 20.0;
    config.noise = 0.02;
    return config;
}

static robotik::SkillDescription describe(std::string p_name, robotik::ResourceManager const& p_resources)
{
    robotik::SkillDescription description;
    description.name = std::move(p_name);
    description.resources = { p_resources.require("left_wheel"),
                              p_resources.require("right_wheel"),
                              p_resources.require("front_camera", robotik::Access::Shared) };
    return description;
}

// --view: the camera picture with what perception found.
static void showCamera(robotik::CameraFrame const& p_frame, robotik::Detections const& p_detections)
{
    cv::Mat picture;
    cv::cvtColor(cv::Mat(static_cast<int>(p_frame.rgb.height()), static_cast<int>(p_frame.rgb.width()), CV_8UC3,
                         const_cast<std::uint8_t*>(p_frame.rgb.bytes().data())),
                 picture, cv::COLOR_RGB2BGR);
    for (robotik::Detection const& item : p_detections.items)
    {
        cv::Scalar const color = item.id >= 0 ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255);
        cv::rectangle(picture, { item.box[0], item.box[1] }, { item.box[2], item.box[3] }, color, 1);
        cv::circle(picture, { static_cast<int>(item.center[0]), static_cast<int>(item.center[1]) }, 3, color, -1);
        if (item.id >= 0)
        {
            cv::putText(picture, std::to_string(item.id), { item.box[0], item.box[1] - 3 },
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
        }
    }
    cv::resize(picture, picture, {}, 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("Robotik camera", picture);
}

// The floor map with the true (green) and estimated (red) paths.
static cv::Mat drawMap(FloorMap const& p_floor, std::vector<Pose2> const& p_truth, std::vector<Pose2> const& p_estimate)
{
    robotik::Image const& image = p_floor.image();
    cv::Mat map;
    cv::cvtColor(cv::Mat(static_cast<int>(image.height()), static_cast<int>(image.width()), CV_8UC3,
                         const_cast<std::uint8_t*>(image.bytes().data())),
                 map, cv::COLOR_RGB2BGR);
    cv::resize(map, map, {}, MAP_VIEW_SCALE, MAP_VIEW_SCALE, cv::INTER_AREA);
    auto draw = [&](std::vector<Pose2> const& p_path, cv::Scalar const& p_color)
    {
        std::vector<cv::Point> points;
        for (Pose2 const& pose : p_path)
        {
            auto const [u, v] = p_floor.pixel(pose.x, pose.y);
            points.emplace_back(static_cast<int>(u * MAP_VIEW_SCALE), static_cast<int>(v * MAP_VIEW_SCALE));
        }
        if (points.empty())
        {
            return;
        }
        cv::polylines(map, points, false, p_color, 2);
        Pose2 const& last = p_path.back();
        cv::Point const tip(points.back().x + static_cast<int>(20.0 * std::cos(last.yaw)),
                            points.back().y - static_cast<int>(20.0 * std::sin(last.yaw)));
        cv::arrowedLine(map, points.back(), tip, p_color, 2);
    };
    draw(p_truth, cv::Scalar(0, 180, 0));
    draw(p_estimate, cv::Scalar(0, 0, 230));
    return map;
}

int main(int argc, char** argv)
{
    Options options;
    if (!parse(argc, argv, options))
    {
        return EXIT_FAILURE;
    }

    try
    {
        Track const track;
        FloorMap const floor(track, placeTags(track), TAG_SIZE_M);
        robotik::Seed const master{ options.seed };

        // Random start near the line, unknown to the robot.
        robotik::Random start(master.derive("start"));
        Track::Point const near = track.at(start.uniform(0.0, track.perimeter()));
        double const offset = start.uniform(-0.3, 0.3);
        auto backend = std::make_unique<DiffDriveBackend>("left_wheel_joint", "right_wheel_joint");
        DiffDriveBackend& drive = *backend;
        drive.start({ near.x - offset * std::sin(near.heading),
                      near.y + offset * std::cos(near.heading),
                      near.heading + start.uniform(-0.8, 0.8) });

        compages::world::World world;
        robotik::RobotSession robot(world, options.urdf);
        robot.connect(std::move(backend));
        auto& left = robot.actuators().add<robotik::Motor>("left_wheel", "left_wheel_joint");
        auto& right = robot.actuators().add<robotik::Motor>("right_wheel", "right_wheel_joint");
        auto& camera = robot.sensors().add<robotik::Camera>("front_camera", frontCamera());
        FloorCamera source(floor, drive);
        camera.source(&source);
        camera.seed(master.derive("front_camera"));

        robotik::PerceptionPipeline perception;
        perception.add<AprilTagDetector>(TAG_SIZE_M);
        perception.add<LineDetector>();

        // Odometry believes in a slightly wrong wheel radius and track.
        robotik::Random calibration(master.derive("odometry"));
        DriveGeometry model = drive.geometry();
        model.radius *= 1.0 + calibration.uniform(-0.03, 0.03);
        model.track *= 1.0 + calibration.uniform(-0.05, 0.05);
        Navigation navigation(track, model);

        Statistic fix_error;
        std::vector<Pose2> truth_path;
        std::vector<Pose2> estimate_path;
        camera.onFrame([&](robotik::CameraFrame const& p_frame)
        {
            robotik::Detections const& detections = perception.process(p_frame);
            if (auto fix = robotik::localize(detections, floor.landmarks());
                fix && navigation.fix(*fix, p_frame.stamp))
            {
                Pose2 const& truth = drive.truth();
                fix_error.add(std::hypot(fix->pose.position.x - truth.x, fix->pose.position.y - truth.y));
            }
            if (options.view)
            {
                showCamera(p_frame, detections);
                cv::imshow("Robotik map", drawMap(floor, truth_path, estimate_path));
                cv::waitKey(1);
            }
        });

        robotik::SkillScheduler skills(robot.resources());
        Wheels const wheels{ left, right, navigation.model };
        skills.add<LocalizeSkill>(describe("Localize", robot.resources()), wheels, navigation);
        skills.add<ReachLineSkill>(describe("ReachLine", robot.resources()), wheels, navigation);
        robotik::SkillId const follow_id = skills.add<FollowLineSkill>(
            describe("FollowLine", robot.resources()), wheels, navigation, perception, options.laps, FOLLOW_SPEED);
        auto const& follow = static_cast<FollowLineSkill const&>(skills.skill(follow_id));

        bt::NodeFactory factory;
        robotik::registerSkills(factory, skills);
        auto built = bt::Builder::fromText(factory, TREE);
        if (!built)
        {
            std::cerr << built.getError() << '\n';
            return EXIT_FAILURE;
        }
        bt::Tree::Ptr tree = std::move(built.getValue());

        robotik::WorldModel world_model;
        robotik::RobotContext context{ robot, world_model, {}, {} };
        Seconds const dt(DEMO_DT_S);
        Statistic cross_track;
        double following = 0.0;
        bool was_following = false;
        bt::Status status = bt::Status::RUNNING;
        auto wall = std::chrono::steady_clock::now();
        auto const begin = wall;
        std::size_t steps = 0;
        while (status == bt::Status::RUNNING && robot.time().value() < DEMO_MAX_TIME_S)
        {
            context.time = robot.time();
            context.dt = dt;
            status = tree->tick();
            skills.update(context);
            robot.step(dt);
            navigation.odometry(left.velocity(), right.velocity(), dt.value());

            Pose2 const& truth = drive.truth();
            if (skills.state(follow_id) == robotik::SkillState::Running)
            {
                if (!was_following)
                {
                    cross_track = {};
                }
                was_following = true;
                following += dt.value();
            }
            else
            {
                following = 0.0;
                was_following = false;
            }
            if (following > CROSS_TRACK_SETTLE_S)
            {
                double const ax = truth.x + model.axle * std::cos(truth.yaw);
                double const ay = truth.y + model.axle * std::sin(truth.yaw);
                Track::Point const on = track.closest(ax, ay);
                cross_track.add(std::hypot(ax - on.x, ay - on.y));
            }
            if (++steps % 5u == 0u)
            {
                truth_path.push_back(truth);
                estimate_path.push_back(navigation.estimate);
            }
            if (options.view)
            {
                wall += std::chrono::microseconds(static_cast<long>(DEMO_DT_S * 1e6));
                std::this_thread::sleep_until(wall);
            }
        }
        double const elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count();

        for (robotik::SkillRun const& run : skills.trace())
        {
            std::cout << "  " << run.start.value() << " s  " << skills.name(run.skill) << "  "
                      << robotik::toString(run.state) << "  (" << (run.end - run.start).value() << " s)\n";
        }
        if (!options.save.empty())
        {
            cv::imwrite(options.save.string(), drawMap(floor, truth_path, estimate_path));
        }
        Pose2 const& truth = drive.truth();
        double const final_error = std::hypot(navigation.estimate.x - truth.x, navigation.estimate.y - truth.y);
        bool const on_line = cross_track.count > 0u && cross_track.max < CROSS_TRACK_LIMIT_M;
        bool const passed = status == bt::Status::SUCCESS && on_line;
        std::cout << "tag fixes      " << navigation.fixes << " (" << navigation.rejected
                  << " rejected)  error mean " << 1000.0 * fix_error.mean()
                  << " mm  max " << 1000.0 * fix_error.max << " mm\n"
                  << "cross track    (last run, after " << CROSS_TRACK_SETTLE_S << " s on the line) mean " << 1000.0 * cross_track.mean() << " mm  max "
                  << 1000.0 * cross_track.max << " mm\n"
                  << "laps           " << follow.progress() / track.perimeter() << '\n'
                  << "final estimate error " << 1000.0 * final_error << " mm\n"
                  << (passed ? "[PASS]" : "[FAIL]") << " seed=" << options.seed << " time=" << robot.time().value()
                  << " s (wall " << elapsed << " s)\n";
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    catch (std::exception const& failure)
    {
        std::cerr << "Error: " << failure.what() << '\n';
        return EXIT_FAILURE;
    }
}
