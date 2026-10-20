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
///     decides
/// what that does to the world after each physics step:
/// - @ref robotik::AbstractGrasp — the object follows the tool, and is put down
///   where it belongs on release (no contact, no fall);
/// - @ref robotik::RigidGrasp — the object is a rigid body of the physics
///   engine; a weld holds it, and it falls for real on release.
/// The models share the grippers: each one only touches its own objects.
///
/// @par Contract
/// - Promises: skills never see the model; each model only moves its own
///   objects.
/// - Expects: one @c update after each physics step.
#pragma once

#include <string>
#include <utility>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
/// Grasps, carries and releases objects for the grippers of a robot.
///
/// The @ref Simulation owns the models and calls @ref update after each
/// physics step; a plugin does not need to touch them.
///
/// @code
/// std::vector<std::unique_ptr<robotik::GraspModel>> grasps;
/// grasps.push_back(std::make_unique<robotik::AbstractGrasp>());
/// // after each physics step:
/// for (auto& grasp : grasps)
///     grasp->update(robot);
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
    // ------------------------------------------------------------------------
    virtual void update(Robot& p_robot) = 0;
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
/// grasp.update(robot);
/// @endcode
// ****************************************************************************
class AbstractGrasp final: public GraspModel
{
public:

    /// Attaches, carries or drops the shapes without physics.
    void update(Robot& p_robot) override;
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
/// grasp.update(robot);
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
    void update(Robot& p_robot) override;

private:

    /// Objects the grippers may hold.
    std::vector<Graspable> m_objects;
};

} // namespace robotik
