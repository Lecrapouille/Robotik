/**
 * @file Status.hpp
 * @brief Execution status shared by skills and behavior-tree adapters.
 */

#pragma once

namespace robotik
{

/**
 * @brief Outcome of a single skill tick or an aggregated skill run.
 */
enum class Status
{
    Idle,    ///< Not started or explicitly idle.
    Running, ///< Still working toward the goal.
    Success, ///< Goal reached.
    failure  ///< Goal cannot be reached or was aborted.
};

} // namespace robotik
