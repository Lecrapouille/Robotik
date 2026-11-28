// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Fidelity.hpp
/// How true to life a run is, on three separate axes: how the physics holds
/// an object, where the beliefs of the robot come from, and where its
/// estimated base comes from.
///
/// A simpler level is not cheating when the scenario chooses it and the run
/// report says it: a test of the plan does not need a detector, nor a
/// friction model. The three axes are independent (a scenario can have a
/// rigid physics, beliefs copied from the truth and a base estimated by the
/// plugin), so they are three enums, side by side in one header.
///
/// @par Contract
/// - Promises: each @c toString gives the word of the scenario file.
/// - Expects: nothing; plain enums.
#pragma once

#include <cstdint>

namespace robotik
{

// ****************************************************************************
/// How finely the physics simulates an interaction (see
/// doc/reference/scenario-file.md, "Physics fidelity").
// ****************************************************************************
enum class PhysicsFidelity : std::uint8_t
{
    /// A held object follows the tool, a released one is put down where it
    /// belongs: no contact, no fall. Checks the logic and the plan.
    Abstract,
    /// Objects are rigid bodies of the physics engine: a grasp welds the
    /// object to the tool, a release lets it fall for real.
    Rigid,
    /// No weld: the tool holds by force (suction, friction). Not built yet.
    Contact,
};

// ----------------------------------------------------------------------------
/// @param p_fidelity Level to name.
/// @return "abstract", "rigid" or "contact".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(PhysicsFidelity p_fidelity);

// ****************************************************************************
/// Where the robot's beliefs about an object come from (@c perception:).
///
/// Giving the robot the true pose of an object is not cheating when it is
/// chosen, said and recorded: like a simpler physics, it lets a test focus
/// on what it checks. A planning test can skip image processing; a car
/// simulator can hand over the lane marks and the pedestrians instead of
/// running vision on photo-realistic images.
///
/// @code{.yaml}
/// perception:
///   fidelity: sensed          # truth | noisy_truth | sensed (default)
///   overrides: { red_cube: truth }
///   blind: truth              # when no camera has an image (a headless run)
///   noise: { sigma: 0.005, dropout: 0.1, range: 2.0 }   # for noisy_truth
///   max_age: 2.0              # older beliefs are unknown to the planner (s)
/// @endcode
// ****************************************************************************
enum class PerceptionFidelity : std::uint8_t
{
    /// The perception pipeline (detectors on camera images) fills the
    /// beliefs.
    Sensed,
    /// The simulator writes the true position, degraded by
    /// the @c noise of the @c perception: section (noise, missed observations, range).
    NoisyTruth,
    /// The simulator writes the true position (a mock of perception).
    Truth,
};

// ----------------------------------------------------------------------------
/// @param p_fidelity Level to name.
/// @return "sensed", "noisy_truth" or "truth".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(PerceptionFidelity p_fidelity);

// ****************************************************************************
/// Where the base pose a skill reads comes from (@ref Robot::getBase).
///
/// The true pose of a mobile base is known in simulation only. A real robot
/// estimates it (odometry, landmarks, a calibration). As for the perception
/// fidelity, the scenario chooses, and the run report says it.
// ****************************************************************************
enum class LocalizationFidelity
{
    /// A mock: the estimate is the true base, copied at each step.
    Truth,
    /// An algorithm of the plugin writes it (RobotRuntime::estimateBase).
    Plugin,
};

/// @return "truth" or "plugin", the word of the scenario file.
[[nodiscard]] constexpr char const*
toString(LocalizationFidelity p_localization)
{
    return p_localization == LocalizationFidelity::Truth ? "truth" : "plugin";
}

} // namespace robotik
