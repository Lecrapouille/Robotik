// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/**
 * @file Robotik.hpp
 * @brief Umbrella header: backends, runtime, skills, and ECS queries.
 *
 * Kinematics come from Pinocchio, dynamics from MuJoCo, and the scene graph
 * from Compages. Behavior trees use BlackThorn via @ref registerSkill.
 */

#pragma once

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/Behavior/SkillNodes.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotRuntime.hpp"
#include "Robotik/Skills/GripperSkills.hpp"
#include "Robotik/Skills/HomeSkill.hpp"
#include "Robotik/Skills/MoveJointSkill.hpp"
#include "Robotik/Skills/MoveJointsSkill.hpp"
#include "Robotik/Skills/MoveTCPSkill.hpp"
