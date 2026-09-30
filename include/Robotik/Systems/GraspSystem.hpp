/**
 * @file GraspSystem.hpp
 * @brief Kinematic follow for vacuum-held objects and tool tip query.
 */

#pragma once

#include <array>

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;

/**
 * @brief Suction cup tip position in the robot base frame, meters.
 * @param p_world ECS world containing the tool and @ref ecs::VacuumGripper.
 * @param p_kinematics Kinematics after configuration sync.
 * @return @c {x, y, z}; zeros if no tool is found.
 */
[[nodiscard]] std::array<double, 3> toolTip(compages::world::World& p_world,
                                            PinocchioBackend const& p_kinematics);

/**
 * @brief Moves the held @ref ecs::SceneObject entity with the tool each frame.
 */
class GraspSystem
{
public:

    /**
     * @brief Updates grasped object transform from flange FK and gripper offset.
     * @param p_world ECS world.
     * @param p_kinematics Pinocchio backend with up-to-date @c q.
     */
    void update(compages::world::World& p_world, PinocchioBackend const& p_kinematics);
};

} // namespace robotik
