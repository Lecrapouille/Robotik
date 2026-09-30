#include "Robotik/Skills/PickPlaceSkills.hpp"

#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/PerceptionComponents.hpp"
#include "Robotik/ECS/Queries.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Systems/GraspSystem.hpp"

#include <cmath>

namespace robotik
{

namespace
{

constexpr double kContactDistance = 0.02;

ecs::VacuumGripper* gripperOf(compages::world::World& p_world)
{
    compages::world::Entity tool = findTool(p_world);
    return tool ? tool.find<ecs::VacuumGripper>() : nullptr;
}

double topOf(compages::world::Entity p_object)
{
    return p_object.position().z + p_object.get<ecs::SceneObject>().size[2] * 0.5f;
}

} // namespace

ApproachSkill::ApproachSkill(std::string p_object, double p_clearance)
    : m_object(std::move(p_object)), m_clearance(p_clearance), m_move("", Pose{})
{
}

void ApproachSkill::reset()
{
    m_planned = false;
    m_move.reset();
}

Status ApproachSkill::tick(RobotContext& p_context, double p_dt)
{
    if (!m_planned)
    {
        compages::world::Entity object = findObject(p_context.world, m_object);
        compages::world::Entity tool = findTool(p_context.world);
        ecs::VacuumGripper* gripper = gripperOf(p_context.world);
        if (!object || gripper == nullptr)
        {
            return Status::failure;
        }
        // Flange above the cup, cup pointing down: half a turn about X, then
        // facing the object from the base so the wrist stays mid range.
        Pose target;
        target.px = object.position().x;
        target.py = object.position().y;
        target.pz = topOf(object) + m_clearance + gripper->tool_length;
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

Status GraspSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    compages::world::Entity object = findObject(p_context.world, m_object);
    ecs::VacuumGripper* gripper = gripperOf(p_context.world);
    if (!object || gripper == nullptr)
    {
        return Status::failure;
    }
    auto const tip = toolTip(p_context.world, p_context.kinematics);
    double const gap = std::hypot(tip[0] - object.position().x,
                                  tip[1] - object.position().y, tip[2] - topOf(object));
    if (gap > kContactDistance)
    {
        return Status::failure;
    }
    gripper->held = object;
    return Status::Success;
}

Status ReleaseSkill::tick(RobotContext& p_context, double /*p_dt*/)
{
    ecs::VacuumGripper* gripper = gripperOf(p_context.world);
    if (gripper == nullptr || !p_context.world.alive(gripper->held))
    {
        return Status::failure;
    }
    compages::world::Entity held(p_context.world, gripper->held);
    gripper->held = {};

    // No physics for the objects: it lands on the floor of the container below.
    float floor = 0.0f;
    auto const at = held.position();
    p_context.world.each<ecs::SceneObject>(
        [&](compages::world::Entity p_other, ecs::SceneObject& p_object)
        {
            auto const center = p_other.position();
            if (p_object.type == ecs::SceneObject::Type::Box &&
                std::abs(at.x - center.x) < p_object.size[0] * 0.5f &&
                std::abs(at.y - center.y) < p_object.size[1] * 0.5f)
            {
                floor = center.z - p_object.size[2] * 0.5f;
            }
        });
    held.position(at.x, at.y, floor + held.get<ecs::SceneObject>().size[2] * 0.5f);
    return Status::Success;
}

Status DetectSkill::tick(RobotContext& p_context, double p_dt)
{
    bool camera = false;
    bool seen = false;
    p_context.world.each<ecs::DetectedObjects>(
        [&](compages::world::Entity, ecs::DetectedObjects& p_detected)
        {
            camera = true;
            for (ecs::Detection const& detection : p_detected.items)
            {
                seen = seen || detection.label == m_object;
            }
        });
    // Headless runs have no renderer, hence no image: trust the scenario.
    if (!camera || seen)
    {
        return Status::Success;
    }
    m_waited += p_dt;
    return m_waited > m_timeout ? Status::failure : Status::Running;
}

} // namespace robotik
