#include "Robotik/Systems/GraspSystem.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/ObjectComponents.hpp"
#include "Robotik/ECS/Queries.hpp"

#include <Eigen/Geometry>

namespace robotik
{

namespace
{

// A point at p_distance along the Z axis of the tool flange.
Eigen::Vector3d alongTool(compages::world::Entity p_tool,
                          PinocchioBackend const& p_kinematics,
                          double p_distance)
{
    Pose const flange = p_kinematics.framePose(p_tool.get<ecs::EndEffector>().name);
    Eigen::Quaterniond const rotation(flange.qw, flange.qx, flange.qy, flange.qz);
    return Eigen::Vector3d(flange.px, flange.py, flange.pz) +
           rotation * Eigen::Vector3d(0.0, 0.0, p_distance);
}

} // namespace

std::array<double, 3> toolTip(compages::world::World& p_world,
                              PinocchioBackend const& p_kinematics)
{
    compages::world::Entity tool = findTool(p_world);
    if (!tool)
    {
        return { 0.0, 0.0, 0.0 };
    }
    ecs::VacuumGripper const* gripper = tool.find<ecs::VacuumGripper>();
    Eigen::Vector3d const tip =
        alongTool(tool, p_kinematics, gripper != nullptr ? gripper->tool_length : 0.0);
    return { tip.x(), tip.y(), tip.z() };
}

void GraspSystem::update(compages::world::World& p_world,
                         PinocchioBackend const& p_kinematics)
{
    compages::world::Entity tool = findTool(p_world);
    ecs::VacuumGripper const* gripper = tool ? tool.find<ecs::VacuumGripper>() : nullptr;
    if (gripper == nullptr || !p_world.alive(gripper->held))
    {
        return;
    }
    compages::world::Entity held(p_world, gripper->held);
    float const half = held.get<ecs::SceneObject>().size[2] * 0.5f;
    Eigen::Vector3d const center =
        alongTool(tool, p_kinematics, gripper->tool_length + half);
    held.position(static_cast<float>(center.x()), static_cast<float>(center.y()),
                  static_cast<float>(center.z()));
}

} // namespace robotik
