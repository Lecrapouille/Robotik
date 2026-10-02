#include "Robotik/Systems/ControllerSystem.hpp"

#include "Robotik/ECS/ActuatorComponents.hpp"
#include "Robotik/ECS/JointComponents.hpp"

#include "Compages/World/Entity.hpp"

#include <algorithm>

namespace robotik
{

//------------------------------------------------------------------------------
void ControllerSystem::update(compages::world::World& p_world,
                              Seconds p_dt) const
{
    double const dt = p_dt.value();
    p_world.each<ecs::JointState,
                 ecs::JointCommand,
                 ecs::PositionController,
                 ecs::ActuatorCommand>(
        [&dt](compages::world::Entity p_entity,
              ecs::JointState const& p_state,
              ecs::JointCommand const& p_command,
              ecs::PositionController& p_controller,
              ecs::ActuatorCommand& p_output)
        {
            // Write the reference position and effort to 0 if not in position
            // mode
            if (ecs::commandMode(p_command) != ecs::JointControlMode::POSITION)
            {
                p_controller.reference = ecs::positionSi(p_state);
                p_output.effort = 0.0;
                return;
            }

            // Find the joint limits
            ecs::JointLimits const* limits = p_entity.find<ecs::JointLimits>();
            double const max_velocity =
                limits != nullptr ? ecs::limitMaxVelocitySi(*limits) : 0.0;
            double const speed = max_velocity > 0.0
                                     ? max_velocity * p_controller.speed_ratio
                                     : 1.0;
            double const step = speed * dt;

            // Update the reference position
            double const command_position = ecs::commandPositionSi(p_command);
            p_controller.reference += std::clamp(
                command_position - p_controller.reference, -step, step);

            // Update the effort
            double effort = p_controller.kp * (p_controller.reference -
                                               ecs::positionSi(p_state)) -
                            p_controller.kd * ecs::velocitySi(p_state);
            double const max_effort =
                limits != nullptr ? ecs::limitMaxEffortSi(*limits) : 0.0;
            if (max_effort > 0.0)
            {
                effort = std::clamp(effort, -max_effort, max_effort);
            }
            p_output.effort = effort;
        });
}

} // namespace robotik
