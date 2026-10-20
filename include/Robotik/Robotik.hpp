// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Robotik.hpp
/// One include for the whole public API of RobotIK.
///
/// RobotIK has three layers; each one works without the ones above it
/// (see doc/Glossary.md):
/// @li Robot: one robot, its joints, sensors, actuators and physics
///     (@ref robotik::openRobot, @ref robotik::Robot);
/// @li Task: robots and objects doing a job, with a controller (behavior
///     tree, PDDL planner or mission code) and beliefs
///     (@ref robotik::Cell, @ref robotik::SkillScheduler);
/// @li Package: the YAML files that describe a job, the plugins, and the
///     verdict of a run (@ref robotik::Simulation, @ref robotik::RunRecord).
///
/// Include it for a small application or a test. Inside the library and the
/// plugins, include the precise header: it builds faster.
///
/// @code
/// #include "Robotik/Robotik.hpp"
///
/// auto scenario = robotik::loadScenario(path, assets);
/// if (!scenario)
///     return fail(scenario.error());
/// compages::world::World world;
/// robotik::Simulation simulation(world, scenario.take());
/// while (!simulation.finished())
///     simulation.step(robotik::Seconds(0.01));
/// @endcode
///
/// @par Contract
/// - Promises: the same API on a simulated and a real robot; files are read
///   with a @c Result, never an exception.
/// - Expects: skills read beliefs, never the ground truth.

#pragma once

#include "Robotik/Task/Agents/Agent.hpp"
#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Task/Learning/Environment.hpp"
#include "Robotik/Common/Math.hpp"
#include "Robotik/Common/Random.hpp"
#include "Robotik/Task/Perception/Detector.hpp"
#include "Robotik/Task/Perception/Localization.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Mission.hpp"
#include "Robotik/Robot/Resources.hpp"
#include "Robotik/Task/Scheduler.hpp"
#include "Robotik/Package/Simulation.hpp"
#include "Robotik/Package/Scenario/Scenario.hpp"
#include "Robotik/Robot/Sensors/Camera.hpp"
#include "Robotik/Robot/Sensors/ForceTorqueSensor.hpp"
#include "Robotik/Robot/Sensors/Imu.hpp"
#include "Robotik/Robot/Sensors/RangeScanner.hpp"
#include "Robotik/Task/Skills/MotionSkills.hpp"
#include "Robotik/Task/Skills/SkillNodes.hpp"
