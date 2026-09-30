/**
 * @file RobotRuntime.hpp
 * @brief Owns Pinocchio and MuJoCo and advances the robot one frame at a time.
 */

#pragma once

#include "Robotik/Runtime/RobotContext.hpp"

#include <memory>
#include <string>
#include <unordered_map>

namespace compages::renderer
{
class Scene;
}

namespace compages::core
{
struct Frame;
}

namespace compages::world
{
class World;
struct ViewFrame;
}

namespace robotik
{

class MujocoBackend;
class PinocchioBackend;

/**
 * @brief Loads one URDF, binds ECS joints, and runs the control / physics pipeline.
 *
 * Compages @c World remains the single source of truth for poses and components.
 * Pinocchio updates after MuJoCo each step; joint angles are projected onto the
 * Compages hierarchy for rendering.
 *
 * @example
 * @code
 * compages::world::World world;
 * compages::renderer::Scene scene(world);
 * robotik::RobotRuntime runtime(world, "robot.urdf", scene);
 * runtime.hold({{"joint1", 0.0}, {"joint2", 0.3}});
 * while (running) {
 *     robotik::MoveJointSkill skill("joint1", 0.5);
 *     skill.tick(runtime.context(), 0.01);
 *     runtime.step(0.01);
 * }
 * @endcode
 */
class RobotRuntime
{
public:

    /**
     * @brief Headless runtime: URDF entities live in @p_world only.
     * @param p_world ECS world to populate.
     * @param p_model_file Path to a URDF file.
     */
    RobotRuntime(compages::world::World& p_world, std::string const& p_model_file);

    /**
     * @brief Runtime with meshes and lights loaded into @p_scene.
     * @param p_world ECS world to populate.
     * @param p_model_file Path to a URDF file.
     * @param p_scene Compages scene used for visualization.
     */
    RobotRuntime(compages::world::World& p_world,
                 std::string const& p_model_file,
                 compages::renderer::Scene& p_scene);

    /** @brief Releases backends and MuJoCo temporary URDF copies. */
    ~RobotRuntime();

    RobotRuntime(RobotRuntime const&) = delete;
    RobotRuntime& operator=(RobotRuntime const&) = delete;

    /**
     * @brief Sets joint positions, stores them as home, and servos all joints.
     * @param p_posture Map of joint name to position (rad or m). Omitted joints
     *        keep their current state.
     */
    void hold(std::unordered_map<std::string, double> const& p_posture = {});

    /**
     * @brief Advances simulation by @p_dt without a full view frame.
     * @param p_dt Step size in seconds.
     */
    void step(double p_dt);

    /**
     * @brief Advances simulation and updates Compages controllers from @p_frame.
     * @param p_frame Viewport size, elapsed time, and input for the world.
     */
    void step(compages::world::ViewFrame const& p_frame);

    /** @brief Snapshot for skill ticks (time and dt filled by the caller). */
    [[nodiscard]] RobotContext context() const;

    /** @brief Current simulation time in seconds. */
    [[nodiscard]] double time() const
    {
        return m_time;
    }

    /** @brief Pinocchio backend for FK / IK. */
    [[nodiscard]] PinocchioBackend& kinematics();

    /** @brief MuJoCo backend, or null if constructed headless without dynamics. */
    [[nodiscard]] MujocoBackend* simulation();

private:

    void load(std::string const& p_model_file, compages::renderer::Scene* p_scene);
    void pipeline(double p_dt);
    void publish(compages::core::Frame const& p_frame);

    /** @brief ECS world reference (not owned). */
    compages::world::World& m_world;

    /** @brief Analytical model for the loaded URDF. */
    std::unique_ptr<PinocchioBackend> m_pinocchio;

    /** @brief Physics simulator; null only when MuJoCo is not used. */
    std::unique_ptr<MujocoBackend> m_mujoco;

    /** @brief Accumulated simulation time in seconds. */
    double m_time = 0.0;

    /** @brief Last pipeline step duration in seconds. */
    double m_dt = 0.0;
};

} // namespace robotik
