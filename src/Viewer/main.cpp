/**
 * @file main.cpp
 * @brief Main entry point for the Robot 3D viewer application.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Viewer/RobotViewerApplication.hpp"

#include <iostream>

using namespace robotik::viewer;

// ----------------------------------------------------------------------------
int main(int argc, char* argv[])
{
    RobotViewerApplication::Configuration config;
    if (RobotViewerApplication app(config); !app.run())
    {
        std::cerr << "Failed to run the application. Reason: " << app.error()
                  << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}