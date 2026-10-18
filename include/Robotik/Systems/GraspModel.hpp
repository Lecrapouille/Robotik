// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file GraspModel.hpp
/// @brief How the simulation makes a gripper hold an object, per level of
/// physics fidelity (@ref Fidelity).
///
/// Skills never see the model: they turn the suction of a
/// @ref VacuumGripper on or off and read @c holding(). The model decides
/// what that does to the world after each physics step:
/// - @ref AbstractGrasp: the object follows the tool, and is put down where
///   it belongs on release (no contact, no fall);
/// - @ref RigidGrasp: the object is a rigid body of the physics engine; a
///   weld holds it, and it falls for real on release.
/// The models share the grippers: each one only touches its own objects.
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
/// @brief Grasps, carries and releases objects for the grippers of a robot.
// ****************************************************************************
class GraspModel
{
public:

    virtual ~GraspModel() = default;

    /// @brief Updates every gripper of @p_robot after a physics step.
    virtual void update(Robot& p_robot) = 0;
};

// ****************************************************************************
/// @brief Abstract level: objects without physics (@ref ecs::Shape without
/// @ref ecs::PhysicsBody). Suction near the top or bottom face of a box
/// attaches it; no suction drops it onto the floor of the container below,
/// or the ground.
// ****************************************************************************
class AbstractGrasp final: public GraspModel
{
public:

    void update(Robot& p_robot) override;
};

// ****************************************************************************
/// @brief Rigid level: objects are free bodies of the physics backend.
/// Suction near the top or the bottom face of one turns on the weld
/// declared between the gripper and it; no suction turns it off.
// ****************************************************************************
class RigidGrasp final: public GraspModel
{
public:

    /// @brief An object the grippers may hold, and its weld per gripper.
    struct Graspable
    {
        /// World entity id, also the body name in the backend.
        std::string id;
        /// Half height of the object (m): the top and bottom faces are
        /// this far from its centre.
        double half_height = 0.0;
    };

    explicit RigidGrasp(std::vector<Graspable> p_objects) : m_objects(std::move(p_objects))
    {
    }

    /// @brief Name of the weld between @p_gripper and @p_object, to declare
    /// in the backend before it is built.
    [[nodiscard]] static std::string weldName(std::string const& p_gripper,
                                              std::string const& p_object);

    void update(Robot& p_robot) override;

private:

    std::vector<Graspable> m_objects;
};

} // namespace robotik
