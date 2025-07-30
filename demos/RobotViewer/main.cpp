#include "RobotViewerApplication.hpp"
#include <iostream>

using namespace robotik;

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
    std::cout << "  --physics <hz>      Set physics update rate (default: 1000)"
              << std::endl;
    std::cout << "  --control-joint <joint>  Set control joint for IK "
                 "(default: auto-detect)"
              << std::endl;
    std::cout << "  --camera-target <joint>  Set camera target joint (default: "
                 "auto-detect)"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  urdf_file               Path to the URDF file to load"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  1-5            Change camera view" << std::endl;
    std::cout << "  M              Toggle between Animation and Inverse "
                 "Kinematics mode"
              << std::endl;
    std::cout << "  Z/S            Move target +/-X (IK mode)" << std::endl;
    std::cout << "  Q/D            Move target +/-Y (IK mode)" << std::endl;
    std::cout << "  A/E            Move target +/-Z (IK mode)" << std::endl;
}

// ----------------------------------------------------------------------------
static bool parse_command_line(size_t const p_argc,
                               char* const p_argv[],
                               RobotViewerApplication::Configuration& p_config)
{
    std::string urdf_file;
    size_t i = 1;

    p_config.window_title = "Robot Viewer";

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
    RobotViewerApplication::Configuration config;
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