/**
 * @file Mesh.hpp
 * @brief OpenGL mesh data structure.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <cstddef>

namespace robotik::viewer
{
// ------------------------------------------------------------------------
//! \brief Structure to hold OpenGL mesh data.
// ------------------------------------------------------------------------
struct Mesh
{
    //! \brief Vertex Array Object
    unsigned int vao = 0;
    //! \brief Vertex Buffer Object
    unsigned int vbo = 0;
    //! \brief Element Buffer Object
    unsigned int ebo = 0;
    //! \brief Number of indices
    size_t index_count = 0;
    //! \brief Load status
    bool is_loaded = false;

    //! \brief Clear the mesh data
    void clear()
    {
        vao = 0;
        vbo = 0;
        ebo = 0;
        index_count = 0;
        is_loaded = false;
    }
};

} // namespace robotik::viewer