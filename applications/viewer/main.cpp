/**
 * @file main.cpp
 * @brief Main entry point for the Robot 3D viewer application.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "RobotViewerApplication.hpp"

// Generated file holding project information
#include "project_info.hpp"

#include <iostream>
#include <sstream>

using namespace robotik::viewer;

// ----------------------------------------------------------------------------
static void display_usage(const std::string& program_name)
{
    std::cout << "Usage: " << program_name << " [OPTIONS] [urdf_file]"
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
    std::cout << "  --control-joint <joint>  Set control joint for IK "
                 "(default: auto-detect the end-effector)"
              << std::endl;
    std::cout << "  --camera-target <joint>  Set camera target joint "
                 "(default: auto-detect the base link)"
              << std::endl;
    std::cout << "  --joint-positions <values>  Set initial joint positions "
                 "(comma-separated values, e.g., 0.1,0.2,0.3)"
              << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  urdf_file               Path to the URDF file to load"
              << std::endl;
}

// ----------------------------------------------------------------------------
static bool parse_command_line(size_t const p_argc,
                               char* const p_argv[],
                               Configuration& p_config)
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
            else if (arg == "--control-joint")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --control-joint requires a value"
                              << std::endl;
                    return false;
                }

                p_config.control_joint = p_argv[++i];
            }
            else if (arg == "--camera-target")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --camera-target requires a value"
                              << std::endl;
                    return false;
                }

                p_config.camera_target = p_argv[++i];
            }
            else if (arg == "--joint-positions")
            {
                if (i + 1 >= p_argc)
                {
                    std::cerr << "Error: --joint-positions requires a value"
                              << std::endl;
                    return false;
                }

                std::string positions_str = p_argv[++i];
                p_config.joint_positions.clear();

                // Parse comma-separated values
                std::stringstream ss(positions_str);
                std::string value;
                while (std::getline(ss, value, ','))
                {
                    try
                    {
                        // Remove leading/trailing whitespace
                        value.erase(0, value.find_first_not_of(" \t"));
                        value.erase(value.find_last_not_of(" \t") + 1);

                        if (!value.empty())
                        {
                            p_config.joint_positions.push_back(
                                std::stod(value));
                        }
                    }
                    catch (const std::exception&)
                    {
                        std::cerr
                            << "Error: Invalid joint position value: " << value
                            << std::endl;
                        return false;
                    }
                }
            }
            else if (arg == "--profile")
            {
                p_config.enable_profiling = true;
            }
            else if (arg[0] == '-')
            {
                std::cerr << "Error: Unknown option: " << arg << std::endl;
                return false;
            }
            else
            {
                if (!p_config.urdf_file.empty())
                {
                    std::cerr << "Error: Multiple URDF files specified"
                              << std::endl;
                    return false;
                }
                p_config.urdf_file = arg;
            }
        }
    }
    catch (const std::exception&)
    {
        std::cerr << "Error: Invalid width value: " << p_argv[i] << std::endl;
        return false;
    }

    // Check if a URDF file was provided
    if (p_config.urdf_file.empty())
    {
        std::cerr
            << "Error: No URDF file provided. Use --help for usage information."
            << std::endl;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    Configuration config;
    if (!parse_command_line(argc, argv, config))
    {
        return EXIT_FAILURE;
    }

    if (RobotViewerApplication app(config); !app.run())
    {
        std::cerr << "Failed to run the application. Reason: " << app.error()
                  << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}