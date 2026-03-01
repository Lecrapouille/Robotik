/**
 * @file Status.hpp
 * @brief Status enum for behavior tree nodes.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 */

#pragma once

#include <algorithm>
#include <array>
#include <string>

namespace robotik::bt {

// ****************************************************************************
//! \brief Enum representing the status of a node in the behavior tree.
// ****************************************************************************
enum class Status
{
    INVALID = 0, //!< The node is invalid (internal use only).
    RUNNING = 1, //!< The node is running.
    SUCCESS = 2, //!< The node is successful.
    FAILURE = 3, //!< The node is failed.
};

// ****************************************************************************
//! \brief Convert a node status to a string.
//! \param[in] p_status The status to convert.
//! \return The string representation of the status.
// ****************************************************************************
inline std::string const& to_string(Status p_status)
{
    static std::array<std::string, 5> const names = {
        "INVALID", "RUNNING", "SUCCESS", "FAILURE", "???"};
    const auto idx = std::min<std::size_t>(static_cast<std::size_t>(p_status),
                                           names.size() - 1);
    return names[idx];
}

} // namespace robotik::bt
