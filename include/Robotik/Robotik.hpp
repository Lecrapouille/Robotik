// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file Robotik.hpp
 * @brief Umbrella header of the Robotik public API.
 *
 * A @ref robotik::Robot owns its joints, sensors, actuators and resources;
 * skills run under the @ref robotik::SkillScheduler; a
 * @ref robotik::RobotBackend (MuJoCo in simulation) moves the joints. Kinematics
 * come from Pinocchio and the scene graph from Compages. Rendering stays out of
 * the library (@ref robotik::SceneView).
 */

#pragma once

#include "Robotik/Actuators/Actuator.hpp"
#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Behavior/SkillNodes.hpp"
#include "Robotik/Environment/Environment.hpp"
#include "Robotik/Math/Pose.hpp"
#include "Robotik/Math/Random.hpp"
#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Runtime/Scheduler.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Scenario.hpp"
#include "Robotik/Sensors/Camera.hpp"
#include "Robotik/Skills/MotionSkills.hpp"
#include "Robotik/Skills/PickPlaceSkills.hpp"
