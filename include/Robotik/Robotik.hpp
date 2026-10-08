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
 * the library (@ref robotik::SceneView). A @ref robotik::Simulation runs a
 * scenario and the @ref robotik::Mission that a demo plugs in. An
 * @ref robotik::Agent turns an @ref robotik::Observation into an
 * @ref robotik::Action without knowing the simulator.
 */

#pragma once

#include "Robotik/Agents/Agent.hpp"
#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Environment/Environment.hpp"
#include "Robotik/Math/Geometry.hpp"
#include "Robotik/Math/Random.hpp"
#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/Localization.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/Resources.hpp"
#include "Robotik/Runtime/Scheduler.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Scenario.hpp"
#include "Robotik/Sensors/Camera.hpp"
#include "Robotik/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Sensors/Imu.hpp"
#include "Robotik/Sensors/RangeScanner.hpp"
#include "Robotik/Skills/MotionSkills.hpp"
#include "Robotik/Skills/SkillNodes.hpp"
