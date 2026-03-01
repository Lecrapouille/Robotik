/**
 * @file main.cpp
 * @brief Entry point for Oakular - BlackThorn behavior tree editor
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#include "IDE.hpp"

#include <iostream>

int main()
{
    try
    {
        // Create and run Oakular editor
        IDE ide(1600, 900);

        if (!ide.run())
        {
            std::cerr << "Failed to run ide: " << ide.error() << std::endl;
            return EXIT_FAILURE;
        }

        return EXIT_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
