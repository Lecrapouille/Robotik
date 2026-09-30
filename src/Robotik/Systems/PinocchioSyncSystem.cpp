#include "Robotik/Systems/PinocchioSyncSystem.hpp"

#include "Robotik/Backends/PinocchioBackend.hpp"
#include "Robotik/ECS/BackendComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

namespace robotik
{

void PinocchioSyncSystem::update(compages::world::World& p_world,
                                 PinocchioBackend& p_pinocchio)
{
    std::vector<double> q = p_pinocchio.configuration();
    std::vector<double> v = p_pinocchio.velocity();

    p_world.each<ecs::JointState, ecs::PinocchioJointBinding>(
        [&](compages::world::Entity,
            ecs::JointState& p_state,
            ecs::PinocchioJointBinding& p_binding)
        {
            if (p_binding.q_index >= 0 &&
                p_binding.q_index < static_cast<int>(q.size()))
            {
                q[static_cast<std::size_t>(p_binding.q_index)] = p_state.position;
            }
            if (p_binding.v_index >= 0 &&
                p_binding.v_index < static_cast<int>(v.size()))
            {
                v[static_cast<std::size_t>(p_binding.v_index)] = p_state.velocity;
            }
        });

    p_pinocchio.setConfiguration(q);
    p_pinocchio.setVelocity(v);
}

} // namespace robotik
