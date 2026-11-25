// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// Line follower: localize on AprilTags, reach the line, follow it.
// OpenCV and AprilRobotics stay in this demo, never in the library.
//
// This program runs without a window. To watch the run, open the simulator
// (Demos -> Line follower): its Cameras panel shows what the robot sees,
// with the detections, and the plugin panel the estimate against the truth.
// Here, --save writes the map of the run in a PNG file.

#include "Floor.hpp"
#include "LineFollowerLogic.hpp"

#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Task/Skills/SkillNodes.hpp"

#include "Compages/World/World.hpp"

#include <opencv2/imgproc.hpp>

#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

#define DEMO_DT_S 0.01
#define DEMO_MAX_TIME_S 150.0
#define MAP_VIEW_SCALE 0.5

struct Options
{
    std::uint64_t seed = 1;
    double laps = 1.0;
    std::filesystem::path save;
    std::filesystem::path scenario =
        "demos/LineFollower/scenarios/line_follower.scenario.yaml";
};

static bool parse(int argc, char** argv, Options& p_options)
{
    for (int i = 1; i < argc; ++i)
    {
        std::string const arg = argv[i];
        if (arg == "--seed" && i + 1 < argc)
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
                      << " [--seed N] [--laps X] [--save map.png]"
                         " [--scenario path]\n";
            return false;
        }
    }
    return true;
}

static cv::Mat drawMap(FloorMap const& p_floor,
                       std::vector<Pose2> const& p_truth,
                       std::vector<Pose2> const& p_estimate)
{
    robotik::Image const& image = p_floor.image();
    cv::Mat map;
    cv::resize(cv::Mat(static_cast<int>(image.height()),
                       static_cast<int>(image.width()),
                       CV_8UC3,
                       const_cast<std::uint8_t*>(image.bytes().data())),
               map, {}, MAP_VIEW_SCALE, MAP_VIEW_SCALE, cv::INTER_AREA);
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
    draw(p_truth, cv::Scalar(0, 180, 0));      // RGB: green
    draw(p_estimate, cv::Scalar(230, 0, 0));   // RGB: red
    return map;
}

/// Writes an RGB picture in a PNG file, or throws.
static void savePng(std::filesystem::path const& p_path, cv::Mat const& p_rgb)
{
    if (stbi_write_png(p_path.string().c_str(), p_rgb.cols, p_rgb.rows, 3,
                       p_rgb.data, static_cast<int>(p_rgb.step)) == 0)
    {
        throw std::runtime_error("cannot write " + p_path.string());
    }
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
        LineFollowerLogic logic(options.laps);
        robotik::Result<robotik::Scenario> loaded =
            robotik::loadScenario(options.scenario, robotik::AssetResolver::standard());
        if (!loaded)
        {
            throw std::runtime_error(loaded.error());
        }
        compages::world::World world;
        robotik::Simulation simulation(world, loaded.take(), nullptr, &logic);
        simulation.reset(robotik::Seed{ options.seed });

        robotik::Seconds const dt(DEMO_DT_S);
        auto const begin = std::chrono::steady_clock::now();
        while (!simulation.finished() &&
               simulation.time().value() < DEMO_MAX_TIME_S)
        {
            simulation.step(dt);
        }
        double const elapsed = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - begin)
                                   .count();

        robotik::SkillScheduler const& skills = simulation.cell().skills();
        for (robotik::SkillRun const& run : skills.trace())
        {
            std::cout << "  " << run.start.value() << " s  "
                      << skills.name(run.skill) << "  "
                      << robotik::toString(run.state) << "  ("
                      << (run.end - run.start).value() << " s)\n";
        }
        if (!options.save.empty())
        {
            savePng(options.save,
                    drawMap(logic.floor(),
                            logic.truthPath(),
                            logic.estimatePath()));
        }

        simulation.conclude();
        for (robotik::Check const& check : simulation.checks())
        {
            std::cout << '[' << robotik::toString(check.verdict) << "] "
                      << check.text;
            if (!check.detail.empty())
            {
                std::cout << "  (" << check.detail << ')';
            }
            std::cout << '\n';
        }
        bool const passed = simulation.passed();
        Pose2 const& truth = logic.drive()->truth();
        double const final_error =
            std::hypot(logic.navigation().estimate.x - truth.x,
                       logic.navigation().estimate.y - truth.y);
        std::cout << "tag fixes      " << logic.navigation().fixes << " ("
                  << logic.navigation().rejected << " rejected)  error mean "
                  << 1000.0 * logic.fixErrorMean() << " mm  max "
                  << 1000.0 * logic.fixErrorMax() << " mm\n"
                  << "laps           "
                  << logic.followProgress() / logic.track().perimeter() << '\n'
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
