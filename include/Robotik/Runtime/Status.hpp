//=============================================================================
// Source: https://github.com/Robotik-Core/Robotik
// Copyright (c) 2020-2026 Quentin Quadrat.
// Distributed under MIT License.
//=============================================================================
// @file Status.hpp
// @brief Execution status shared by skills and behavior-tree adapters.
//=============================================================================

#pragma once

namespace robotik
{

// -------------------------------------------------------------------------
// @brief Outcome of a single skill tick or an aggregated skill run.
// -------------------------------------------------------------------------
enum class Status
{
    IDLE,     //!< Not started or explicitly idle.
    RUNNING,  //!< Still working toward the goal.
    SUCCESS,  //!< Goal reached.
    FAILURE,  //!< Goal cannot be reached or was aborted.
};

} // namespace robotik
