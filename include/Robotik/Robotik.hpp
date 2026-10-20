// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file Robotik.hpp
 * Umbrella header of the Robotik public API.
 *
 * A @ref robotik::Robot owns its joints, sensors, actuators and resources;
 * skills run under the @ref robotik::SkillScheduler; a
 * @ref robotik::RobotBackend (MuJoCo in simulation) moves the joints.
 * Kinematics come from Pinocchio and the scene graph from Compages. Rendering
 * stays out of the library (@ref robotik::SceneView). A @ref
 * robotik::Simulation runs a scenario and the @ref robotik::Mission that a demo
 * plugs in. An
 * @ref robotik::Agent turns an @ref robotik::Observation into an
 * @ref robotik::Action without knowing the simulator.
 *
 * Include it when you want everything at once (a small application, a test).
 * Inside the library and the plugins, prefer the precise header: it builds
 * faster.
 *
 * @code
 * #include "Robotik/Robotik.hpp"
 *
 * compages::world::World world;
 * robotik::Simulation simulation(world, robotik::loadScenario(path, assets));
 * simulation.reset(robotik::Seed(42));
 * @endcode
 */

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
