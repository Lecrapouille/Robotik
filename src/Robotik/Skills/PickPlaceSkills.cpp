#include "Robotik/Skills/PickPlaceSkills.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/PerceptionComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include <cmath>
#include <string_view>

#define CONTACT_DISTANCE_M 0.02

namespace robotik
{

//------------------------------------------------------------------------------
//! @brief Get the vacuum gripper from the world.
//! @param p_world The world.
//! @return The vacuum gripper.
//------------------------------------------------------------------------------
static ecs::VacuumGripper* gripperOf(compages::world::World& p_world)
{
    compages::world::Entity tool = findTool(p_world);
    return tool ? tool.find<ecs::VacuumGripper>() : nullptr;
}

//------------------------------------------------------------------------------
//! @brief Get the top of the object.
//! @param p_object The object.
//! @return The top of the object.
//------------------------------------------------------------------------------
static double topOf(compages::world::Entity p_object)
{
    return static_cast<double>(p_object.position().z) +
           p_object.get<ecs::SceneObject>().size[2].value() * 0.5;
}

//------------------------------------------------------------------------------
//! @brief Update the container floor row.
//! @param p_other The other object.
//! @param p_object The object.
//! @param p_at The position of the object.
//! @param p_floor The floor.
//------------------------------------------------------------------------------
static void updateContainerFloorRow(compages::world::Entity p_other,
                                    ecs::SceneObject& p_object,
                                    compages::core::Vector3f p_at,
                                    double& p_floor)
{
    auto const center = p_other.position();
    if (p_object.type == ecs::SceneObject::Type::BOX &&
        std::abs(p_at.x - center.x) <
            static_cast<float>(p_object.size[0].value() * 0.5) &&
        std::abs(p_at.y - center.y) <
            static_cast<float>(p_object.size[1].value() * 0.5))
    {
        p_floor =
            static_cast<double>(center.z) - p_object.size[2].value() * 0.5;
    }
}

//------------------------------------------------------------------------------
//! @brief Find the container floor.
//! @param p_world The world.
//! @param p_at The position of the object.
//! @return The container floor.
//------------------------------------------------------------------------------
static double findContainerFloor(compages::world::World& p_world,
                                 compages::core::Vector3f p_at)
{
    double floor = 0.0;
    p_world.each<ecs::SceneObject>(
        [&p_at, &floor](compages::world::Entity p_other,
                        ecs::SceneObject& p_object)
        { updateContainerFloorRow(p_other, p_object, p_at, floor); });
    return floor;
}

//------------------------------------------------------------------------------
//! @brief Merge the detection row.
//! @param p_detected The detected objects.
//! @param p_label The label.
//! @param p_camera The camera.
//! @param p_seen The seen.
//------------------------------------------------------------------------------
static void mergeDetectionRow(compages::world::Entity,
                              ecs::DetectedObjects const& p_detected,
                              std::string_view p_label,
                              bool& p_camera,
                              bool& p_seen)
{
    p_camera = true;
    for (ecs::Detection const& detection : p_detected.items)
    {
        p_seen = p_seen || detection.label == p_label;
    }
}

//------------------------------------------------------------------------------
//! @brief Scan the detections.
//! @param p_world The world.
//! @param p_label The label.
//! @param p_camera The camera.
//! @param p_seen The seen.
//------------------------------------------------------------------------------
static void scanDetections(compages::world::World& p_world,
                           std::string_view p_label,
                           bool& p_camera,
                           bool& p_seen)
{
    p_world.each<ecs::DetectedObjects>(
        [&p_label, &p_camera, &p_seen](compages::world::Entity p_entity,
                                       ecs::DetectedObjects const& p_detected)
        {
            mergeDetectionRow(p_entity, p_detected, p_label, p_camera, p_seen);
        });
}

//------------------------------------------------------------------------------
ApproachSkill::ApproachSkill(std::string p_object, Length p_clearance)
    : m_object(std::move(p_object)),
      m_clearance(p_clearance),
      m_move("", Pose{})
{
}

//------------------------------------------------------------------------------
void ApproachSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

//------------------------------------------------------------------------------
Status ApproachSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    if (!m_planned)
    {
        // Find the object and the tool
        compages::world::Entity object = findObject(p_context.world, m_object);
        compages::world::Entity tool = findTool(p_context.world);
        ecs::VacuumGripper const* gripper = gripperOf(p_context.world);
        if (!object || gripper == nullptr)
        {
            return Status::FAILURE;
        }

        // Calculate the target pose.
        // Flange above the cup, cup pointing down: half a turn about X, then
        // facing the object from the base so the wrist stays mid range.
        Pose target;
        target.px = object.position().x;
        target.py = object.position().y;
        target.pz =
            topOf(object) + (m_clearance + gripper->tool_length).value();
        double const yaw = std::atan2(target.py, target.px);
        target.qw = 0.0;
        target.qx = std::cos(yaw * 0.5);
        target.qy = std::sin(yaw * 0.5);
        target.qz = 0.0;
        m_move.setGoal(tool.get<ecs::EndEffector>().name, target);
        m_planned = true;
    }
    return m_move.tick(p_context, p_dt);
}

//------------------------------------------------------------------------------
Status GraspSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    // Find the object and the tool
    compages::world::Entity object = findObject(p_context.world, m_object);
    ecs::VacuumGripper* gripper = gripperOf(p_context.world);
    if (!object || gripper == nullptr)
    {
        return Status::FAILURE;
    }

    // Calculate the gap between the tool tip and the object top.
    auto const tip = toolTip(p_context.world, p_context.kinematics);
    auto const at = object.position();
    double const gap = std::hypot(tip[0] - static_cast<double>(at.x),
                                  tip[1] - static_cast<double>(at.y),
                                  tip[2] - topOf(object));
    if (gap > Length(CONTACT_DISTANCE_M).value())
    {
        return Status::FAILURE;
    }
    gripper->held = object;
    return Status::SUCCESS;
}

//------------------------------------------------------------------------------
Status ReleaseSkill::tick(RobotContext& p_context, Seconds /*p_dt*/)
{
    // Find the gripper and the held object
    ecs::VacuumGripper* gripper = gripperOf(p_context.world);
    if (gripper == nullptr || !p_context.world.alive(gripper->held))
    {
        return Status::FAILURE;
    }

    // Release the object
    compages::world::Entity held(p_context.world, gripper->held);
    gripper->held = {};

    // No physics for the objects: it lands on the floor of the container below.
    auto const at = held.position();
    double const floor = findContainerFloor(p_context.world, at);

    // Set the held object position
    auto z = floor + held.get<ecs::SceneObject>().size[2].value() * 0.5;
    held.position(at.x, at.y, static_cast<float>(z));
    return Status::SUCCESS;
}

//------------------------------------------------------------------------------
Status DetectSkill::tick(RobotContext& p_context, Seconds p_dt)
{
    bool camera = false;
    bool seen = false;
    scanDetections(p_context.world, m_object, camera, seen);

    // Headless runs have no renderer, hence no image: trust the scenario.
    if (!camera || seen)
    {
        return Status::SUCCESS;
    }

    // Check if the timeout has been reached
    m_waited += p_dt;
    return m_waited > m_timeout ? Status::FAILURE : Status::RUNNING;
}

} // namespace robotik
