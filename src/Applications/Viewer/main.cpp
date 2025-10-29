/**
 * @file main.cpp
 * @brief Main entry point for the Robot 3D viewer application.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Application.hpp"

// Generated file holding project information
#include "project_info.hpp"

#include <iostream>

// ----------------------------------------------------------------------------
static void display_usage(const std::string& program_name)
{
    std::cout << "Usage: " << program_name << " [OPTIONS] [urdf_files...]"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  -h, --help              Display this help message and exit"
              << std::endl;
    std::cout << "  --width <pixels>    Set window width (default: 1024)"
              << std::endl;
    std::cout << "  --height <pixels>       Set window height (default: 768)"
              << std::endl;
    std::cout << "  --fps <fps>         Set target frame rate (default: 60)"
              << std::endl;
    std::cout << "  --physics <hz>      Set physics update rate (default: 15)"
              << std::endl;
    std::cout << "  --gravity <value> Set physics gravity (default: -9.81)"
              << std::endl;
}

// ----------------------------------------------------------------------------
static bool parse_command_line(size_t const p_argc,
                               char* const p_argv[],
                               robotik::application::Configuration& p_config)
{
    std::string urdf_file;
    size_t i = 1;

    p_config.window_title = "Robot Viewer";
    p_config.search_paths = project::info::paths::data;

    try
    {
        for (i = 1; i < p_argc; ++i)
        {
            std::string arg = p_argv[i];

            if (arg == "--help" || arg == "-h")
            {
                display_usage(p_argv[0]);
                return false;
            }
            else if (arg == "--width")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --width requires a value" << std::endl;
                    return false;
                }

                p_config.window_width = std::stoul(p_argv[++i]);
            }
            else if (arg == "--height")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --height requires a value"
                              << std::endl;
                    return false;
                }

                p_config.window_height = std::stoul(p_argv[++i]);
            }
            else if (arg == "--fps")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --fps requires a value" << std::endl;
                    return false;
                }

                p_config.target_fps = std::stoul(p_argv[++i]);
            }
            else if (arg == "--physics")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --physics requires a value"
                              << std::endl;
                    return false;
                }

                p_config.target_physics_hz = std::stoul(p_argv[++i]);
            }
            else if (arg == "--gravity")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --gravity requires a value"
                              << std::endl;
                    return false;
                }

                p_config.physics_gravity =
                    Eigen::Vector3d(0.0, 0.0, std::stod(p_argv[++i]));
            }
            else if (arg[0] == '-')
            {
                std::cerr << "Error: Unknown option: " << arg << std::endl;
                return false;
            }
            else
            {
                p_config.urdf_files.push_back(arg);
            }
        }
    }
    catch (const std::exception&)
    {
        std::cerr << "Error: Invalid width value: " << p_argv[i] << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    robotik::application::Configuration config;
    if (!parse_command_line(argc, argv, config))
    {
        return EXIT_FAILURE;
    }

    if (robotik::application::Application app(config); !app.run())
    {
        std::cerr << "Failed to run the application. Reason: " << app.error()
                  << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}