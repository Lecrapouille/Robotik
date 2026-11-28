// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotIK.hpp
/// One include for the whole public API of RobotIK.
///
/// RobotIK has three layers; each one works without the ones above it
/// (see doc/reference/glossary.md):
/// @li Robot: one robot, its joints, sensors, actuators and physics
///     (@ref robotik::StandaloneRobot::open, @ref robotik::Robot);
/// @li Task: robots and objects doing a job, with a controller (behavior
///     tree, PDDL planner or scenario logic) and beliefs
///     (@ref robotik::RoboticCell, @ref robotik::SkillScheduler);
/// @li Package: the YAML files that describe a job, the plugins, and the
///     verdict of a run (@ref robotik::Simulation, @ref robotik::RunRecord).
///
/// Include it for a small application or a test. Inside the library and the
/// plugins, include the precise header: it builds faster.
///
/// @code
/// #include "RobotIK/RobotIK.hpp"
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

#include "RobotIK/Common/Math.hpp"
#include "RobotIK/Common/Random.hpp"
#include "RobotIK/Package/Scenario/Scenario.hpp"
#include "RobotIK/Package/Simulation.hpp"
#include "RobotIK/Robot/Assets/AssetResolver.hpp"
#include "RobotIK/Robot/Backends/MujocoBackend.hpp"
#include "RobotIK/Robot/Resources.hpp"
#include "RobotIK/Robot/Robot.hpp"
#include "RobotIK/Robot/Sensors/Camera.hpp"
#include "RobotIK/Robot/Sensors/ForceTorqueSensor.hpp"
#include "RobotIK/Robot/Sensors/Imu.hpp"
#include "RobotIK/Robot/Sensors/RangeScanner.hpp"
#include "RobotIK/Task/Agents/Agent.hpp"
#include "RobotIK/Task/Faults.hpp"
#include "RobotIK/Task/Learning/Environment.hpp"
#include "RobotIK/Task/Metrics.hpp"
#include "RobotIK/Task/Perception/Detector.hpp"
#include "RobotIK/Task/Perception/Localization.hpp"
#include "RobotIK/Task/Perception/WorldModel.hpp"
#include "RobotIK/Task/ScenarioLogic.hpp"
#include "RobotIK/Task/Scheduler.hpp"
#include "RobotIK/Task/Skills/MotionSkills.hpp"
#include "RobotIK/Task/Skills/SkillNodes.hpp"
