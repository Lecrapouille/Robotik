// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Line follower: localize on AprilTags, reach the line, follow it.
// OpenCV and AprilRobotics stay in this demo, never in the library.

#include "Floor.hpp"
#include "LineFollowerMission.hpp"

#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Skills/SkillNodes.hpp"

#include "Compages/World/World.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <thread>
#include <vector>

#define DEMO_DT_S 0.01
#define DEMO_MAX_TIME_S 150.0
#define MAP_VIEW_SCALE 0.5

struct Options
{
    std::uint64_t seed = 1;
    double laps = 1.0;
    bool view = false;
    std::filesystem::path save;
    std::filesystem::path scenario = "demos/LineFollower/scenarios/line_follower.yml";
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
        else if (arg == "--scenario" && i + 1 < argc)
        {
            p_options.scenario = argv[++i];
        }
        else
        {
            std::cerr << "Usage: " << argv[0]
                      << " [--seed N] [--laps X] [--view] [--save map.png]"
                         " [--scenario path]\n";
            return false;
        }
    }
    return true;
}

static void showCamera(robotik::CameraFrame const& p_frame,
                       robotik::Detections const& p_detections)
{
    cv::Mat picture;
    cv::cvtColor(cv::Mat(static_cast<int>(p_frame.rgb.height()),
                         static_cast<int>(p_frame.rgb.width()),
                         CV_8UC3,
                         const_cast<std::uint8_t*>(p_frame.rgb.bytes().data())),
                 picture,
                 cv::COLOR_RGB2BGR);
    for (robotik::Detection const& item : p_detections.items)
    {
        cv::Scalar const color =
            item.id >= 0 ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255);
        cv::rectangle(picture,
                      { item.box[0], item.box[1] },
                      { item.box[2], item.box[3] },
                      color,
                      1);
        cv::circle(picture,
                   { static_cast<int>(item.center[0]),
                     static_cast<int>(item.center[1]) },
                   3,
                   color,
                   -1);
        if (item.id >= 0)
        {
            cv::putText(picture,
                        std::to_string(item.id),
                        { item.box[0], item.box[1] - 3 },
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1);
        }
    }
    cv::resize(picture, picture, {}, 2.0, 2.0, cv::INTER_NEAREST);
    cv::imshow("Robotik camera", picture);
}

static cv::Mat drawMap(FloorMap const& p_floor,
                       std::vector<Pose2> const& p_truth,
                       std::vector<Pose2> const& p_estimate)
{
    robotik::Image const& image = p_floor.image();
    cv::Mat map;
    cv::cvtColor(cv::Mat(static_cast<int>(image.height()),
                         static_cast<int>(image.width()),
                         CV_8UC3,
                         const_cast<std::uint8_t*>(image.bytes().data())),
                 map,
                 cv::COLOR_RGB2BGR);
    cv::resize(map, map, {}, MAP_VIEW_SCALE, MAP_VIEW_SCALE, cv::INTER_AREA);
    auto draw = [&](std::vector<Pose2> const& p_path, cv::Scalar const& p_color)
    {
        std::vector<cv::Point> points;
        for (Pose2 const& pose : p_path)
        {
            auto const [u, v] = p_floor.pixel(pose.x, pose.y);
            points.emplace_back(static_cast<int>(u * MAP_VIEW_SCALE),
                                static_cast<int>(v * MAP_VIEW_SCALE));
        }
        if (points.empty())
        {
            return;
        }
        cv::polylines(map, points, false, p_color, 2);
        Pose2 const& last = p_path.back();
        cv::Point const tip(
            points.back().x + static_cast<int>(20.0 * std::cos(last.yaw)),
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
        LineFollowerMission mission(options.laps);
        compages::world::World world;
        robotik::Simulation simulation(
            world,
            robotik::Scenario::load(options.scenario),
            nullptr,
            &mission);
        simulation.reset(robotik::Seed{ options.seed });

        robotik::Camera* camera = simulation.camera();
        if (options.view && camera != nullptr)
        {
            camera->onFrame(
                [&](robotik::CameraFrame const& p_frame)
                {
                    showCamera(p_frame, simulation.perception().detections());
                    cv::imshow("Robotik map",
                               drawMap(mission.floor(),
                                       mission.truthPath(),
                                       mission.estimatePath()));
                    cv::waitKey(1);
                });
        }

        Seconds const dt(DEMO_DT_S);
        auto wall = std::chrono::steady_clock::now();
        auto const begin = wall;
        while (!simulation.finished() &&
               simulation.time().value() < DEMO_MAX_TIME_S)
        {
            simulation.step(dt);
            if (options.view)
            {
                wall += std::chrono::microseconds(
                    static_cast<long>(DEMO_DT_S * 1e6));
                std::this_thread::sleep_until(wall);
            }
        }
        double const elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - begin)
                                   .count();

        robotik::SkillScheduler const& skills = simulation.skills();
        for (robotik::SkillRun const& run : skills.trace())
        {
            std::cout << "  " << run.start.value() << " s  "
                      << skills.name(run.skill) << "  "
                      << robotik::toString(run.state) << "  ("
                      << (run.end - run.start).value() << " s)\n";
        }
        if (!options.save.empty())
        {
            cv::imwrite(options.save.string(),
                        drawMap(mission.floor(),
                                mission.truthPath(),
                                mission.estimatePath()));
        }

        bool passed = true;
        for (auto const& check : simulation.checks())
        {
            std::cout << (check.passed ? "[PASS] " : "[FAIL] ") << check.text;
            if (!check.detail.empty())
            {
                std::cout << "  (" << check.detail << ')';
            }
            std::cout << '\n';
            passed = passed && check.passed;
        }
        Pose2 const& truth = mission.drive()->truth();
        double const final_error =
            std::hypot(mission.navigation().estimate.x - truth.x,
                       mission.navigation().estimate.y - truth.y);
        std::cout << "tag fixes      " << mission.navigation().fixes << " ("
                  << mission.navigation().rejected << " rejected)  error mean "
                  << 1000.0 * mission.fixErrorMean() << " mm  max "
                  << 1000.0 * mission.fixErrorMax() << " mm\n"
                  << "laps           "
                  << mission.followProgress() / mission.track().perimeter()
                  << '\n'
                  << "final estimate error " << 1000.0 * final_error << " mm\n"
                  << (passed ? "[PASS]" : "[FAIL]") << " seed=" << options.seed
                  << " time=" << simulation.time().value() << " s (wall "
                  << elapsed << " s)\n";
        return passed ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    catch (std::exception const& failure)
    {
        std::cerr << "Error: " << failure.what() << '\n';
        return EXIT_FAILURE;
    }
}
