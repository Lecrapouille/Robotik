// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RobotRuntime.hpp
/// What runs a robot (the cell, a test, a driver), as opposed to what its
/// skills see. The runtime makes time pass: at each step it moves the
/// physics (or reads the real robot) and the sensors. It owns the
/// @ref robotik::Robot that the skills get, and it alone holds what a skill
/// must not touch: the simulated world and its entities, the backend, the
/// true position of the robot, and the values that only the sensors write.
///
/// A skill includes @c Robot.hpp and gets a @ref robotik::Robot from
/// @ref robotik::RobotRuntime::robot: it reads the measures and sends
/// commands.
///
/// @par Contract
/// - Promises: a skill cannot reach what this class holds: the
///   @ref robotik::Robot does not store it, and @c make @c check-layers
///   compiles @c Task/Skills and @c Task/Planning without this header.
/// - Expects: one @ref robotik::RobotRuntime::step per control period.
/// - Errors: an unknown joint name throws @c std::invalid_argument; a URDF
///   that cannot be read throws @c std::runtime_error (use
///   @ref robotik::StandaloneRobot::open for a @c Result).
#pragma once

#include "RobotIK/Robot/Backends/PhysicsWorld.hpp"
#include "RobotIK/Robot/Backends/RobotBackend.hpp"
#include "RobotIK/Robot/Robot.hpp"
#include "RobotIK/Robot/SceneView.hpp"

#include "Compages/World/Entity.hpp"

#include <filesystem>
#include <memory>
#include <string_view>
#include <vector>

namespace compages::world
{
class World;
}

namespace robotik
{

// ****************************************************************************
/// Runs a robot. At each step, it moves the joints (in the simulator or on
/// the real robot), then reads the sensors.
///
/// Why: a skill must behave the same in a test, in the simulator and on a
/// real robot. So a skill gets only the @ref Robot (read the measures,
/// send commands). Everything that would let it cheat (read the true
/// position of the robot, write a joint measure, step the physics, change a
/// sensor) is here, in a class that only the program that runs the robot
/// holds: a @ref RoboticCell, a test, @ref robotik::StandaloneRobot::open.
///
/// How: @ref connect plugs a backend, the part that really moves the
/// joints (MuJoCo in simulation, a driver on hardware, see
/// @ref RobotBackend). Then @ref step, once per control period, keeps the
/// failed actuators safe, moves the backend, and reads the sensors. A
/// runtime without a backend still works: the joints move only when the
/// code places them, which is enough for kinematics tests.
///
/// Why it owns a @ref Robot instead of deriving from it: the runtime is not
/// a kind of robot, it runs one. Owning it keeps the world, the entities,
/// the backend and the true base out of what a skill can reach, and the
/// call @c runtime.robot() shows in the code where the skill side starts.
///
/// @code
/// using namespace robotik::literals;
/// compages::world::World world;
/// robotik::RobotRuntime runtime(world, "arm.urdf");
/// // An actuator gives the skills a name ("arm") for the joints they move.
/// runtime.addActuator<robotik::JointGroup>("arm");
/// // The physics: MuJoCo reads the same URDF.
/// auto physics = std::make_unique<robotik::MujocoBackend>();
/// physics->load("arm.urdf");
/// runtime.connect(std::move(physics));
/// // Start pose: joint2 at 0.3 rad, the others at 0; the arm holds it.
/// runtime.hold({ .angles = { { "joint2", 0.3_rad } } });
/// while (running)
///     runtime.step(0.01_s); // 10 ms of physics, then the sensors
/// robotik::Robot& robot = runtime.robot(); // what a skill gets
/// @endcode
// ****************************************************************************
class RobotRuntime final
{
public:

    // -------------------------------------------------------------------------
    /// Loads the URDF into the world, the kinematics and the joints.
    /// @param p_world ECS world receiving the entities.
    /// @param p_urdf URDF of the robot.
    /// @param p_view Rendering hooks, or null for a headless robot.
    /// @param p_tool Tool mounted on the robot (URDF, flange and mount
    ///   links), or none.
    /// @param p_resources Shared with the other robots of the world, or null.
    /// @throws std::runtime_error if the URDF or the tool cannot be loaded.
    // -------------------------------------------------------------------------
    RobotRuntime(compages::world::World& p_world,
                 std::filesystem::path const& p_urdf,
                 SceneView* p_view = nullptr,
                 ToolMount const& p_tool = {},
                 std::shared_ptr<ResourceManager> p_resources = {});

    RobotRuntime(RobotRuntime const&) = delete;
    RobotRuntime& operator=(RobotRuntime const&) = delete;
    ~RobotRuntime();

    // -------------------------------------------------------------------------
    /// @return The robot as a skill sees it: joints, read-only sensors,
    ///   actuators, kinematics, estimated base. Give this to the skills.
    // -------------------------------------------------------------------------
    [[nodiscard]] Robot& robot()
    {
        return m_robot;
    }

    // -------------------------------------------------------------------------
    /// @copydoc robot()
    // -------------------------------------------------------------------------
    [[nodiscard]] Robot const& robot() const
    {
        return m_robot;
    }

    // -------------------------------------------------------------------------
    /// Gives the robot its backend: what moves the joints from now on.
    /// Add the sensors first: a backend reads them when it is plugged
    /// (MuJoCo makes a force/torque sensor for each one, for example).
    /// @param p_backend Backend, now owned by the runtime.
    // -------------------------------------------------------------------------
    void connect(std::unique_ptr<RobotBackend> p_backend);

    // -------------------------------------------------------------------------
    /// Makes one control period pass: failed actuators go to their safe
    /// state, the backend moves the joints, then the robot updates its
    /// kinematics, its drawing and its sensors. It is @ref prepare,
    /// @ref advance and @ref sense in a row; several robots that share one
    /// physics call the three parts themselves.
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// First part of @ref step: puts the failed actuators in their safe
    /// state. Robots that share one physics do each part for all of them in
    /// turn, so every failed actuator is safe before the physics moves, and
    /// every robot reads its sensors after it.
    ///
    /// @code
    /// for (auto* r : robots) r->prepare(dt);
    /// for (auto* r : robots) r->advance(dt); // the first steps the physics
    /// for (auto* r : robots) r->sense();
    /// @endcode
    ///
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void prepare(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Second part of @ref step: the backend moves the joints, then the
    /// clock of the robot takes the time of the backend (a real robot has
    /// its own clock), or adds @p p_dt when the backend has none.
    // -------------------------------------------------------------------------
    void advance(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Last part of @ref step: computes where each link is, moves the drawn
    /// links, then reads the sensors. A sensor whose resource failed gives
    /// no reading (see @ref prepare).
    // -------------------------------------------------------------------------
    void sense();

    // -------------------------------------------------------------------------
    /// Starts a new run: clock at zero, joints at their home position (see
    /// @ref hold), base at its start pose (see @ref setStartPose), sensors
    /// without old readings.
    // -------------------------------------------------------------------------
    void reset();

    // -------------------------------------------------------------------------
    /// Puts the robot in a posture at once and keeps it there: the motors
    /// hold every joint where it is. The posture also becomes the home
    /// position that @ref reset goes back to.
    /// @param p_posture Angles of the joints that turn, lengths of those
    ///   that slide; a joint not in the posture keeps its position.
    /// @throws std::invalid_argument for an unknown joint name, or a joint
    ///   in the wrong table (an angle for a joint that slides).
    // -------------------------------------------------------------------------
    void hold(JointPosture const& p_posture = {});

    // -------------------------------------------------------------------------
    /// Builds a sensor on the robot and declares its resource.
    /// @tparam T Sensor type (@ref Camera, @ref Imu, ...).
    /// @param p_args Arguments of the constructor of @p T (name first).
    /// @return The new sensor, owned by the robot.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<Sensor> T, typename... Args>
    T& addSensor(Args&&... p_args)
    {
        return m_robot.m_sensors.add<T>(std::forward<Args>(p_args)...);
    }

    // -------------------------------------------------------------------------
    /// Builds an actuator on the robot and declares its resource.
    /// @tparam T Actuator type (@ref JointGroup, @ref VacuumGripper, ...).
    /// @param p_args Arguments of the constructor of @p T (name first).
    /// @return The new actuator, owned by the robot.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<Actuator> T, typename... Args>
    T& addActuator(Args&&... p_args)
    {
        return m_robot.m_actuators.add<T>(std::forward<Args>(p_args)...);
    }

    // -------------------------------------------------------------------------
    /// Writes what the encoder of a joint reads: the backend calls it after
    /// each physics step, a driver after each reading. It is the only way to
    /// change what the skills read as the joint position: a skill cannot
    /// write it.
    /// @param p_joint Joint that turns (@c Revolute) or slides
    ///   (@c Prismatic): the unit of the values follows.
    /// @param p_position Measured position (@c Radians or @c Length).
    /// @param p_velocity Measured speed (rad/s or m/s).
    // -------------------------------------------------------------------------
    template <class Position>
    void measureJoint(JointHandle<Position> p_joint,
                      Position p_position,
                      VelocityOf<Position> p_velocity)
    {
        m_robot.m_joints.measure(
            p_joint.id, p_position.value(), p_velocity.value());
    }

    // -------------------------------------------------------------------------
    /// Puts a joint at a position at once, at rest, as if it had always been
    /// there: measure, target and command become the same. For a reset, or
    /// a backend that draws a posture computed elsewhere; not a move (use
    /// @ref JointSet::moveTo for that).
    /// @param p_joint Joint that turns or slides.
    /// @param p_position New position (@c Radians or @c Length).
    // -------------------------------------------------------------------------
    template <class Position>
    void placeJoint(JointHandle<Position> p_joint, Position p_position)
    {
        m_robot.m_joints.place(p_joint.id, p_position.value());
    }

    // -------------------------------------------------------------------------
    /// Writes what the vacuum sensor of a suction cup reads: is something
    /// held? In simulation the grasp model calls it. When the vacuum is
    /// lost, the cup forgets the name of what it held.
    /// @param p_gripper Cup of this robot.
    /// @param p_holding True when an object is held.
    // -------------------------------------------------------------------------
    void measureHolding(VacuumGripper& p_gripper, bool p_holding)
    {
        p_gripper.measureHolding(p_holding);
    }

    // -------------------------------------------------------------------------
    /// Writes where the base really is: the backend calls it after its step
    /// (in simulation it is the truth). With @ref LocalizationFidelity::Truth
    /// the skills read this same pose; otherwise they read what
    /// @ref estimateBase wrote.
    /// @param p_base Pose and velocity in the world.
    // -------------------------------------------------------------------------
    void measureBase(BaseState const& p_base)
    {
        m_true_base = p_base;
        if (m_localization == LocalizationFidelity::Truth)
        {
            m_robot.m_base = p_base;
        }
    }

    // -------------------------------------------------------------------------
    /// Writes where the robot thinks its base is: the result of a
    /// localization algorithm of a plugin (odometry, landmarks). The skills
    /// read it with @ref Robot::getBase. With @ref LocalizationFidelity::Truth
    /// the next @ref measureBase overwrites it.
    /// @param p_base Estimated pose and velocity in the world.
    // -------------------------------------------------------------------------
    void estimateBase(BaseState const& p_base)
    {
        m_robot.m_base = p_base;
    }

    // -------------------------------------------------------------------------
    /// Where a link really is in the world, from the true base. For the
    /// sensors (a camera sees from where it really is) and the checks of a
    /// scenario; a skill reads @ref Robot::getWorldPose, from the estimated
    /// base.
    /// @param p_link Link found by @ref Robot::requireLink on this robot.
    /// @throws std::logic_error if @p_link comes from another robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose getTrueWorldPose(LinkRef const& p_link) const;

    // -------------------------------------------------------------------------
    /// Same as @ref getTrueWorldPose, found by name each time: slower, for
    /// code that runs once. An empty name gives the base itself.
    /// @throws std::invalid_argument if the frame is unknown.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireTrueWorldPose(std::string const& p_frame) const;

    // -------------------------------------------------------------------------
    /// @return The drawn entity of a link, to hang something on it (a
    ///   marker, a camera); an empty handle when there is no such link.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity
    getLink(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Chooses where @ref reset puts the base of the robot in the world: a
    /// mobile robot starts there, an arm is bolted there. Default: the
    /// world origin.
    /// @param p_pose Base pose in the world.
    // -------------------------------------------------------------------------
    void setStartPose(Pose const& p_pose);

    // -------------------------------------------------------------------------
    /// Gives the robot its name in the world file ("left_arm"), so two
    /// copies of the same URDF are told apart (see @ref Robot::getId).
    // -------------------------------------------------------------------------
    void setId(std::string p_id)
    {
        m_robot.m_id = std::move(p_id);
    }

    // -------------------------------------------------------------------------
    /// Chooses who writes the base pose that the skills read: a copy of the
    /// truth (a mock, the default) or a localization algorithm of the
    /// plugin (see @ref LocalizationFidelity).
    // -------------------------------------------------------------------------
    void setLocalization(LocalizationFidelity p_localization)
    {
        m_localization = p_localization;
    }

    // -------------------------------------------------------------------------
    /// @return Who writes the base pose that the skills read.
    // -------------------------------------------------------------------------
    [[nodiscard]] LocalizationFidelity getLocalization() const
    {
        return m_localization;
    }

    // -------------------------------------------------------------------------
    /// @return Robot clock: the time of the backend after the last step
    ///   (simulated time, or the stamp of the last hardware reading), zero
    ///   at @ref reset. A skill reads the same time in
    ///   @c RobotContext::time, given at each tick.
    // -------------------------------------------------------------------------
    [[nodiscard]] Seconds getTime() const
    {
        return m_time;
    }

    // -------------------------------------------------------------------------
    /// @return Where the base really is (sensors, rendering, truth queries).
    // -------------------------------------------------------------------------
    [[nodiscard]] BaseState const& getTrueBase() const
    {
        return m_true_base;
    }

    // -------------------------------------------------------------------------
    /// @return The sensors, writable: the runtime gives them their image
    ///   source, their noise seed, and forgets their readings. A skill gets
    ///   them read only (@ref Robot::getSensors).
    // -------------------------------------------------------------------------
    [[nodiscard]] SensorSet& getSensors()
    {
        return m_robot.m_sensors;
    }

    // -------------------------------------------------------------------------
    /// @copydoc getSensors()
    // -------------------------------------------------------------------------
    [[nodiscard]] SensorSet const& getSensors() const
    {
        return m_robot.m_sensors;
    }

    // -------------------------------------------------------------------------
    /// @return The resources, writable: a fault marks one as failed and
    ///   back, a new device adds its own.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceManager& getResources()
    {
        return *m_robot.m_resources;
    }

    // -------------------------------------------------------------------------
    /// @return The geometry model of the robot (Pinocchio), writable: a
    ///   backend or a tool change updates it; a skill only reads it.
    // -------------------------------------------------------------------------
    [[nodiscard]] Kinematics& getKinematics()
    {
        return m_robot.m_kinematics;
    }

    // -------------------------------------------------------------------------
    /// @return What moves the joints (MuJoCo, a driver), or null before
    ///   @ref connect.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotBackend* getBackend()
    {
        return m_backend.get();
    }

    // -------------------------------------------------------------------------
    /// @copydoc getBackend()
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotBackend const* getBackend() const
    {
        return m_backend.get();
    }

    // -------------------------------------------------------------------------
    /// @return The simulated world of the backend (rays, contacts, bodies,
    ///   welds), or null on hardware and without a backend. Simulated
    ///   sensors, the truth and the grasp models read it; a skill never.
    // -------------------------------------------------------------------------
    [[nodiscard]] PhysicsWorld* getPhysics()
    {
        return m_physics;
    }

    // -------------------------------------------------------------------------
    /// @copydoc getPhysics()
    // -------------------------------------------------------------------------
    [[nodiscard]] PhysicsWorld const* getPhysics() const
    {
        return m_physics;
    }

    // -------------------------------------------------------------------------
    /// @return Whether the backend drives the robot, read after the last
    ///   step (see @ref RobotBackend::Health). While it is not, every
    ///   actuator is in its safe state and its resource is failed.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotBackend::Health const& getHealth() const
    {
        return m_health;
    }

    // -------------------------------------------------------------------------
    /// @return The world of entities that draws the robot and its objects.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::World& getWorld() const
    {
        return m_world;
    }

    // -------------------------------------------------------------------------
    /// @return The entity of the robot base: the links hang under it, and
    ///   it follows the true base.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity getRoot() const
    {
        return m_root;
    }

    // -------------------------------------------------------------------------
    /// @return The drawn link entity that carries joint @p p_joint (empty
    ///   when it has none).
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity getJointLink(JointId p_joint) const
    {
        return m_links[p_joint];
    }

    // -------------------------------------------------------------------------
    /// @return Rotation from the URDF frame (Z-up, the world frame of
    ///   RobotIK) to the Compages frame (Y-up) the entities are drawn in.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::Quatf const& getUrdfFrame() const
    {
        return m_world_from_urdf;
    }

private:

    // -------------------------------------------------------------------------
    /// Updates the kinematics, then the drawn entities (joints and base, at
    /// the true base).
    // -------------------------------------------------------------------------
    void propagate();

    // -------------------------------------------------------------------------
    /// Writes the measured joints to the joint components of the links
    /// (rendering and inspection).
    // -------------------------------------------------------------------------
    void publishJoints() const;

    // -------------------------------------------------------------------------
    /// Reads the health of the backend: fails the actuator resources when
    /// it is lost, restores them when it is back.
    // -------------------------------------------------------------------------
    void checkHealth();

private:

    /// ECS world holding the entities.
    compages::world::World& m_world;
    /// Root entity (the links hang under it).
    compages::world::Entity m_root;
    /// Compages is Y-up; URDF / Pinocchio / MuJoCo stay Z-up. The loader
    /// hangs the robot under this rotation; @ref propagate composes it with
    /// the true base instead of overwriting it.
    compages::Quatf m_world_from_urdf{};
    /// The robot that the skills get.
    Robot m_robot;
    /// Link entity of each joint, indexed by @ref JointId.
    std::vector<compages::world::Entity> m_links;
    /// Backend moving the robot, or null.
    std::unique_ptr<RobotBackend> m_backend;
    /// The same backend seen as a simulated world, or null (hardware).
    PhysicsWorld* m_physics = nullptr;
    /// Health of the backend after the last step.
    RobotBackend::Health m_health;
    /// Actuator resources failed because the backend is not healthy, to
    /// restore when it is again.
    std::vector<ResourceId> m_health_failed;
    /// True base, written by the backend (the simulator side reads it).
    BaseState m_true_base;
    /// Base pose restored by @ref reset.
    Pose m_start;
    /// Robot clock (see @ref getTime).
    Seconds m_time{};
    /// Where the estimated base comes from.
    LocalizationFidelity m_localization = LocalizationFidelity::Truth;
};

} // namespace robotik
