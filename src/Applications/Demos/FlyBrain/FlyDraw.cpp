// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "FlyDraw.hpp"

#include "Robotik/Robot/Robot.hpp"

#include "Compages/Renderer/Scene.hpp"
#include "Compages/World/Components/Camera.hpp"

#include <cmath>
#include <string>

namespace
{

constexpr float PI = 3.14159265358979323846f;

//! @brief Euclidean length. The beam math stays in float, the view's unit.
float lengthOf(compages::core::Vector3f const& p_vector)
{
    return std::sqrt(p_vector.x * p_vector.x + p_vector.y * p_vector.y +
                     p_vector.z * p_vector.z);
}

//! @brief Places one joint when the URDF has it. A missing name is skipped,
//! so a shorter model still draws.
void poseJoint(robotik::Robot& p_robot,
               std::string const& p_name,
               double p_angle)
{
    robotik::JointId const id = p_robot.joints().find(p_name);
    if (id != robotik::NO_JOINT)
    {
        p_robot.joints().place(id, p_angle);
    }
}

} // namespace

compages::core::Vector3f flyToView(robotik::Vector3 const& p_z_up)
{
    return { static_cast<float>(p_z_up.x),
             static_cast<float>(p_z_up.z),
             static_cast<float>(-p_z_up.y) };
}

void poseFlyBody(robotik::Robot& p_robot, FlySnapshot const& p_snapshot)
{
    FlyPosture const& posture = p_snapshot.posture;

    // Compages is Y-up. The fly stays Z-up until this conversion.
    // Left and right share the posture: the URDF mirrors the signs.
    for (char const* leg : FLY_LEG_NAMES)
    {
        std::string const name(leg);
        poseJoint(p_robot, name + "_coxa_pitch", posture.coxa_pitch);
        poseJoint(p_robot, name + "_coxa_roll", 0.0);
        poseJoint(p_robot, name + "_coxa_yaw", 0.0);
        poseJoint(p_robot, name + "_femur_pitch", posture.femur_pitch);
        poseJoint(p_robot, name + "_femur_roll", 0.0);
        poseJoint(p_robot, name + "_tibia_pitch", posture.tibia_pitch);
        poseJoint(p_robot, name + "_tarsus_pitch", posture.tarsus_pitch);
    }

    // Left and right share the posture: the URDF mirrors the signs.
    for (char const* side : { "L", "R" })
    {
        std::string const wing = std::string("wing_") + side;
        poseJoint(p_robot, wing + "_sweep", posture.wing_sweep);
        poseJoint(p_robot, wing + "_deviation", posture.wing_deviation);
        poseJoint(p_robot, wing + "_pitch", posture.wing_pitch);
        poseJoint(
            p_robot, std::string("haltere_") + side + "_flap", posture.haltere);
    }
    poseJoint(p_robot, "neck_yaw", posture.neck_yaw);
    poseJoint(p_robot, "neck_pitch", posture.neck_pitch);
    poseJoint(p_robot, "neck_roll", 0.0);
    poseJoint(p_robot, "abdomen_pitch", posture.abdomen_pitch);

    // Measure the base pose.
    robotik::Pose pose;
    pose.position = robotik::Vector3(
        p_snapshot.plant.x, p_snapshot.plant.y, p_snapshot.plant.z);
    pose.rotation = robotik::rpy(0.0, 0.0, p_snapshot.plant.yaw);
    p_robot.measureBase(robotik::BaseState{ pose, {} });
}

void aimFlyBeam(compages::world::Entity p_beam,
                compages::world::Entity p_start,
                compages::world::Entity p_end,
                robotik::Vector3 const& p_from,
                robotik::Vector3 const& p_to,
                float p_strength)
{
    // Convert the positions to the view frame.
    compages::core::Vector3f const from = flyToView(p_from);
    compages::core::Vector3f const to = flyToView(p_to);
    p_start.position(from.x, from.y, from.z).scale(0.06f);
    float const tip = 0.08f + 0.1f * p_strength;
    p_end.position(to.x, to.y, to.z).scale(tip);

    // Calculate the span of the beam.
    compages::core::Vector3f const delta(
        to.x - from.x, to.y - from.y, to.z - from.z);
    float const span = lengthOf(delta);
    if (span < 1.0e-4f)
    {
        p_beam.position(from.x, from.y, from.z).scale(0.001f);
        return;
    }

    // Calculate the direction of the beam.
    compages::core::Vector3f const direction(
        delta.x / span, delta.y / span, delta.z / span);

    // Built-in cone: tip at local +Y, base at local -Y, one unit tall.
    // The centre is the midpoint, so the tip lands on the sample and the
    // base on the eye.
    p_beam.position(
        (from.x + to.x) * 0.5f, (from.y + to.y) * 0.5f, (from.z + to.z) * 0.5f);

    // cross(+Y, direction) is the axis that carries the cone's +Y onto the
    // beam.
    compages::core::Vector3f const axis(direction.z, 0.0f, -direction.x);
    float const sine = lengthOf(axis);
    if (sine < 1.0e-4f)
    {
        if (direction.y < 0.0f)
        {
            p_beam.rotation(Radians(PI),
                            compages::core::Vector3f(1.0f, 0.0f, 0.0f));
        }
        else
        {
            p_beam.rotation(compages::core::Quatf(1.0f, 0.0f, 0.0f, 0.0f));
        }
    }
    else
    {
        p_beam.rotation(Radians(std::atan2(sine, direction.y)), axis);
    }
    float const diameter = 0.025f + 0.04f * p_strength;
    p_beam.scale(diameter, span, diameter);
}

compages::world::Entity mountFlyEye(compages::renderer::Scene& p_scene,
                                    robotik::Robot& p_robot,
                                    std::string_view p_link,
                                    std::string_view p_name)
{
    compages::world::Entity const link = p_robot.link(p_link);
    if (!link)
    {
        return {};
    }

    // Columns are the camera axes in the eye frame: +X image-right, +Y up,
    // +Z opposite the look direction. -Z then falls on the optical +X.
    robotik::Quaternion const lens =
        robotik::basis(robotik::Vector3(0.0, -1.0, 0.0),
                       robotik::Vector3(0.0, 0.0, 1.0),
                       robotik::Vector3(-1.0, 0.0, 0.0));

    // Create the camera.
    compages::world::Entity camera =
        p_scene.camera(std::string(p_name))
            .parent(link)
            .position(0.04f, 0.0f, 0.0f)
            .rotation(compages::core::Quatf(static_cast<float>(lens.w),
                                            static_cast<float>(lens.x),
                                            static_cast<float>(lens.y),
                                            static_cast<float>(lens.z)));

    // Set the camera properties.
    auto& eye = camera.get<compages::world::Camera>();
    eye.fov = units::angle::degree_t(70.0);
    eye.near_plane = 0.02f;
    eye.far_plane = 30.0f;

    return camera;
}
