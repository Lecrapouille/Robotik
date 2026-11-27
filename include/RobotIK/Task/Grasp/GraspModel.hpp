// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file GraspModel.hpp
/// How the simulation makes a gripper hold an object, per level of physics
/// fidelity (@ref robotik::Fidelity).
///
/// Skills never see the model: they turn the suction of a
/// @ref robotik::VacuumGripper on or off and read @c holding(). The model
/// decides what that does to the world after each physics step:
/// - @ref robotik::AbstractGrasp — the object follows the tool, and is put down
///   where it belongs on release (no contact, no fall);
/// - @ref robotik::RigidGrasp — the object is a rigid body of the physics
///   engine; a weld holds it, and it falls for real on release.
/// The models share the grippers: each one only touches its own objects.
/// Which entity a gripper holds is truth: it stays in @ref robotik::HeldObjects,
/// owned by the cell, and the gripper only gets the measured vacuum.
///
/// @par Contract
/// - Promises: skills never see the model nor the entity held; each model
///   only moves its own objects.
/// - Expects: one @c update after each physics step.
#pragma once

#include "Compages/World/EntityId.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robotik
{

class RobotRuntime;
class VacuumGripper;

// ****************************************************************************
/// Which entity each gripper of the cell holds: the truth behind
/// @c VacuumGripper::holding().
///
/// Why not in the gripper: a skill sees the gripper, and the entity held is
/// the truth (on hardware, nobody knows it). The cell owns one instance; the
/// grasp models write it, and the truth queries and the simulator read it.
/// @ref hold and @ref drop also write the measured vacuum of the gripper, so
/// both cannot disagree.
///
/// @code
/// robotik::HeldObjects held;
/// held.hold(robot, cup, cube.id()); // cup.holding() is now true
/// compages::world::EntityId const what = held.of(cup);
/// held.drop(robot, cup);            // cup.holding() is false again
/// @endcode
// ****************************************************************************
class HeldObjects
{
public:

    /// @return Entity held by @p_gripper, or an invalid id.
    [[nodiscard]] compages::world::EntityId
    of(VacuumGripper const& p_gripper) const
    {
        auto const found = m_held.find(&p_gripper);
        return found == m_held.end() ? compages::world::EntityId{}
                                     : found->second;
    }

    // ------------------------------------------------------------------------
    /// @p_gripper holds @p_object (an invalid id is the same as @ref drop).
    /// @param p_robot Runtime of the robot carrying the gripper: it writes
    ///   the measured vacuum.
    // ------------------------------------------------------------------------
    void hold(RobotRuntime& p_robot,
              VacuumGripper& p_gripper,
              compages::world::EntityId p_object);

    /// @p_gripper of @p_robot holds nothing.
    void drop(RobotRuntime& p_robot, VacuumGripper& p_gripper);

private:

    /// Gripper to entity held; a gripper holding nothing is not a key.
    std::unordered_map<VacuumGripper const*, compages::world::EntityId> m_held;
};

// ****************************************************************************
/// Grasps, carries and releases objects for the grippers of a robot.
///
/// The @ref Simulation owns the models and calls @ref update after each
/// physics step; a plugin does not need to touch them.
///
/// @code
/// std::vector<std::unique_ptr<robotik::GraspModel>> grasps;
/// grasps.push_back(std::make_unique<robotik::AbstractGrasp>());
/// robotik::HeldObjects held;
/// // after each physics step:
/// for (auto& grasp : grasps)
///     grasp->update(robot, held);
/// @endcode
// ****************************************************************************
class GraspModel
{
public:

    virtual ~GraspModel() = default;

    // ------------------------------------------------------------------------
    /// Updates every gripper of a robot after a physics step: attaches,
    /// carries or releases the objects.
    /// @param p_robot Robot whose grippers are updated.
    /// @param p_held What each gripper holds, shared by the models.
    // ------------------------------------------------------------------------
    virtual void update(RobotRuntime& p_robot, HeldObjects& p_held) = 0;
};

// ****************************************************************************
/// Abstract level: objects without physics (@ref ecs::Shape without
/// @ref ecs::PhysicsBody). Suction near the top or bottom face of a box
/// attaches it; no suction drops it onto the floor of the container below,
/// or the ground.
///
/// Fast and stable: good for planning tests and RL.
///
/// @code
/// robotik::AbstractGrasp grasp;
/// grasp.update(robot, held);
/// @endcode
// ****************************************************************************
class AbstractGrasp final: public GraspModel
{
public:

    /// Attaches, carries or drops the shapes without physics.
    void update(RobotRuntime& p_robot, HeldObjects& p_held) override;
};

// ****************************************************************************
/// Rigid level: objects are free bodies of the physics backend. Suction near
/// the top or the bottom face of one turns on the weld declared between the
/// gripper and it; no suction turns it off.
///
/// The welds must exist in the physics model before it is built: declare one
/// per gripper and object, named by @ref weldName.
///
/// @code
/// robotik::RigidGrasp grasp({ { "box", 0.025 } });
/// physics.addWeld(robotik::RigidGrasp::weldName("vacuum", "box"), tool_link,
/// "box");
/// // after each physics step:
/// grasp.update(robot, held);
/// @endcode
// ****************************************************************************
class RigidGrasp final: public GraspModel
{
public:

    // ------------------------------------------------------------------------
    /// An object the grippers may hold.
    // ------------------------------------------------------------------------
    struct Graspable
    {
        /// World entity id, also the body name in the backend.
        std::string id;
        /// Half height of the object (m): the top and bottom faces are
        /// this far from its centre.
        double half_height = 0.0;
    };

    // ------------------------------------------------------------------------
    /// @param p_objects Objects the grippers may hold.
    // ------------------------------------------------------------------------
    explicit RigidGrasp(std::vector<Graspable> p_objects)
        : m_objects(std::move(p_objects))
    {
    }

    // ------------------------------------------------------------------------
    /// Names the weld between a gripper and an object, to declare in the
    /// backend before it is built.
    /// @param p_gripper Name of the gripper actuator.
    /// @param p_object Id of the object.
    /// @return The weld name.
    // ------------------------------------------------------------------------
    [[nodiscard]] static std::string weldName(std::string const& p_gripper,
                                              std::string const& p_object);

    /// Turns the welds on or off from the suction of each gripper.
    void update(RobotRuntime& p_robot, HeldObjects& p_held) override;

private:

    /// Objects the grippers may hold.
    std::vector<Graspable> m_objects;
};

} // namespace robotik
