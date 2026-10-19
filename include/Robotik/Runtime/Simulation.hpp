// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Simulation.hpp
/// Runs a scenario: builds its world, drives the controller, applies the
/// faults and judges the assertions. Headless unless a @ref SceneView is
/// given.
#pragma once

#include "Robotik/Backends/MujocoBackend.hpp"
#include "Robotik/Perception/Detector.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Planning/PlanExecutor.hpp"
#include "Robotik/Planning/PlanningDomain.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Runtime/Metrics.hpp"
#include "Robotik/Runtime/Mission.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Scheduler.hpp"
#include "Robotik/Scenario/Assertions.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

namespace compages::world
{
class World;
}

namespace robotik
{

class PinocchioBackend;
class MujocoBackend;
class GraspModel;

namespace ecs
{
struct Shape;
}

// ****************************************************************************
/// One scenario instance, replayable from a seed.
///
/// Each @ref step: faults, behavior tree tick (requests skills), scheduler
/// update (admits and ticks skills), robot step (physics, sensors, perception
/// into the @ref WorldModel), simulated suction, mission, assertions. When
/// the scenario allows it (@ref OraclePolicy), an oracle feeds the world
/// model with the ground truth while a sensor resource is available.
///
/// The run ends when the controller finishes, the @c stop_when condition
/// holds, the @c timeout elapses or an assertion marked @c stop fails. The
/// physics keeps running afterwards; the verdicts are frozen.
///
/// The ground truth (@ref pose, @ref query) is for evaluation: skills only
/// see the @ref RobotContext.
///
/// @code
/// compages::world::World world;
/// robotik::Simulation sim(world, robotik::Scenario::load(path, assets));
/// sim.reset(robotik::Seed{ 7 });
/// while (!sim.finished())
///     sim.step(Seconds(0.01));
/// for (robotik::Check const& check : sim.checks())
///     if (check.verdict != robotik::Check::Verdict::Passed) { ... }
/// @endcode
// ****************************************************************************
class Simulation
{
public:

    /// Why a run ended.
    enum class Outcome : std::uint8_t
    {
        Running,             ///< Not ended.
        ControllerSucceeded, ///< The tree, the planner or the mission
                             ///< succeeded.
        ControllerFailed,    ///< The tree, the planner or the mission failed.
        StopCondition,       ///< The @c stop_when condition holds.
        Timeout,             ///< The @c timeout elapsed.
        AssertionFailed,     ///< An assertion marked @c stop failed.
        /// The host ended the run (@ref conclude).
        Interrupted,
    };

    Simulation(Simulation const&) = delete;
    Simulation& operator=(Simulation const&) = delete;

    // -------------------------------------------------------------------------
    /// Builds the world of the scenario (robots, sensors, objects, physics),
    /// calls @ref Mission::setup, then resets with the scenario seed.
    /// @param p_world ECS world receiving the entities.
    /// @param p_scenario Scenario to run.
    /// @param p_view Rendering hooks, or null for a headless run.
    /// @param p_mission Task plugin (skills, detectors, metrics); not owned.
    /// @throws std::runtime_error when the world cannot be built.
    // -------------------------------------------------------------------------
    Simulation(compages::world::World& p_world,
               Scenario p_scenario,
               SceneView* p_view = nullptr,
               Mission* p_mission = nullptr);
    ~Simulation();

    // -------------------------------------------------------------------------
    /// Starts a new episode: same seed, same episode.
    /// @param p_seed Derives the prop layout ("world"), the faults
    ///   ("faults") and the sensor noise (one stream per sensor name).
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    /// New episode with the scenario seed.
    void reset()
    {
        reset(Seed{ m_scenario.seed });
    }

    // -------------------------------------------------------------------------
    /// Advances the run by one step (see the class comment for the order).
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Operator mode: physics and sensors keep running, the behavior tree,
    /// the scheduler, the mission and the assertions do not. Joint commands
    /// then come from the teach pendant.
    /// @param p_on True to suspend, false to resume.
    // -------------------------------------------------------------------------
    void suspend(bool p_on)
    {
        m_suspended = p_on;
    }

    /// @return True in operator mode.
    [[nodiscard]] bool suspended() const
    {
        return m_suspended;
    }

    /// @return The scenario being run.
    [[nodiscard]] Scenario const& scenario() const
    {
        return m_scenario;
    }

    /// @return Seed of the current episode.
    [[nodiscard]] Seed seed() const
    {
        return m_seed;
    }

    /// @return Simulated time since the reset.
    [[nodiscard]] Seconds time() const
    {
        return m_robot->time();
    }

    /// @return The first robot of the world file.
    [[nodiscard]] RobotSession& robot() const
    {
        return *m_robot;
    }

    // -------------------------------------------------------------------------
    /// Finds a robot. The robots of a world share one physics, one resource
    /// manager, one scheduler and the beliefs.
    /// @param p_id Entity id in the world file.
    /// @return The robot, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotSession* robot(std::string_view p_id) const;

    /// @return Every robot, in world file order (the first is @ref robot()).
    [[nodiscard]] std::vector<std::unique_ptr<RobotSession>> const&
    robots() const
    {
        return m_robots;
    }

    // -------------------------------------------------------------------------
    /// @return Frame of the beliefs in the world: the base of the robot when
    ///   it is alone, the world itself when the robots share them (a skill
    ///   then uses its own base: @c robot.base().pose.inverse() * belief).
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose beliefFrame() const;

    /// @return The beliefs shared by the robots.
    [[nodiscard]] WorldModel& worldModel()
    {
        return m_world_model;
    }

    // -------------------------------------------------------------------------
    /// @return Entity of the Z-up frame of the world, at its origin: the
    ///   parent of the props. Hang on it what stays in the world (a floor),
    ///   not on the root of a robot, which moves with a floating base.
    // -------------------------------------------------------------------------
    [[nodiscard]] compages::world::Entity worldFrame() const
    {
        return m_frame;
    }

    // -------------------------------------------------------------------------
    /// Options of the physics the core builds after @ref Mission::setup: a
    /// mission may tune them there (a floating base, no gravity
    /// compensation, the step and the rotors of a legged robot). The world
    /// still adds its floor. The robots, tools, bodies and rigid props of the
    /// world are loaded as usual.
    ///
    /// @code
    /// void Walk::setup(robotik::Simulation& p_sim, robotik::SceneView*)
    /// {
    ///     p_sim.physicsOptions().floating_base = true;
    ///     p_sim.physicsOptions().gravity_compensation = false;
    /// }
    /// @endcode
    ///
    /// @return The options, to change in place.
    // -------------------------------------------------------------------------
    [[nodiscard]] MujocoOptions& physicsOptions()
    {
        return m_physics_options;
    }

    /// @copydoc worldModel()
    [[nodiscard]] WorldModel const& worldModel() const
    {
        return m_world_model;
    }

    /// @return Detectors run on every camera frame (empty by default).
    [[nodiscard]] PerceptionPipeline& perception()
    {
        return m_perception;
    }

    /// @copydoc perception()
    [[nodiscard]] PerceptionPipeline const& perception() const
    {
        return m_perception;
    }

    /// @return The scheduler of every skill of the world.
    [[nodiscard]] SkillScheduler& skills() const
    {
        return *m_scheduler;
    }

    /// @return Factory the behavior tree is built from. Skills registered
    ///   later under the same name replace a node added here.
    [[nodiscard]] bt::NodeFactory& factory()
    {
        return m_factory;
    }

    /// @return The faults of the scenario.
    [[nodiscard]] FaultInjector& faults()
    {
        return m_faults;
    }

    /// @return Context of the first robot.
    [[nodiscard]] RobotContext& context()
    {
        return m_contexts.front();
    }

    // -------------------------------------------------------------------------
    /// @param p_id Robot entity id.
    /// @return Context of the robot.
    /// @throws std::invalid_argument if there is no such robot.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotContext& context(std::string_view p_id);

    /// @return First camera of the robots, or null.
    [[nodiscard]] Camera* camera() const;

    /// @return True when the ground truth feeds the world model (see
    ///   @ref OraclePolicy).
    [[nodiscard]] bool oracle() const
    {
        return m_oracle;
    }

    /// @return Behavior tree, or null if the controller has none. With a
    ///   planner: the tree of the current plan.
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_executor ? m_executor->tree() : m_tree.get();
    }

    // -------------------------------------------------------------------------
    /// @return Observers and bindings of the PDDL planner. The mission fills
    ///   them in @ref Mission::setup; they are used when the scenario
    ///   controller is a @c planner (see @ref PlanningDomain).
    // -------------------------------------------------------------------------
    [[nodiscard]] PlanningDomain& planning()
    {
        return m_planning;
    }

    /// @return The planner controller, or null when the scenario has none.
    [[nodiscard]] PlanExecutor* planner() const
    {
        return m_executor.get();
    }

    /// @return Controller status (behavior tree, or mission without one).
    [[nodiscard]] bt::Status status() const
    {
        return m_status;
    }

    // --- End of run
    // -----------------------------------------------------------

    /// @return Why the run ended, or @c Running.
    [[nodiscard]] Outcome outcome() const
    {
        return m_outcome;
    }

    /// @return True once the run ended (the physics may keep running).
    [[nodiscard]] bool finished() const
    {
        return m_outcome != Outcome::Running;
    }

    // -------------------------------------------------------------------------
    /// Ends the run now (the host's own limit): pending assertions get their
    /// final verdict. No effect on a finished run.
    // -------------------------------------------------------------------------
    void conclude();

    /// @return Assertions with their verdicts (live while running).
    [[nodiscard]] std::vector<Check> const& checks() const
    {
        return m_assertions.checks();
    }

    /// @return True when the run finished and every assertion passed.
    [[nodiscard]] bool passed() const
    {
        return m_assertions.passed();
    }

    /// @return Peak count of simultaneous contacts (floor excluded) since
    ///   the last reset, whatever the bodies.
    [[nodiscard]] int maxContacts() const
    {
        return m_max_contacts;
    }

    // --- Ground truth (world frame; for evaluation, not for skills) ----------

    // -------------------------------------------------------------------------
    /// True pose of an entity or one of its frames.
    /// @param p_reference @c entity or @c entity.frame: a robot link, a link
    ///   of a body model, or the origin of a prop or a primitive body.
    /// @return The pose in the world frame.
    /// @throws std::invalid_argument for an unknown entity or frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose pose(std::string_view p_reference) const;

    /// @return Pose of entity @p_id when the episode started.
    [[nodiscard]] Pose initialPose(std::string_view p_id) const;

    /// @return ECS entity of @p_id (props, primitive bodies, rendered
    ///   models), or an empty handle.
    [[nodiscard]] compages::world::Entity entity(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// Adds a primitive drawn for the eye only (beam, trail), in the Z-up
    /// world frame: its position and rotation are world coordinates. It is
    /// not an entity of the world: nothing collides with it, and assertions
    /// cannot name it. The mission moves it.
    /// @param p_object Shape, size and color.
    /// @return The new entity.
    // -------------------------------------------------------------------------
    compages::world::Entity decorate(ecs::Shape const& p_object);

    // -------------------------------------------------------------------------
    /// Value of a metric or of a world query, as used in conditions:
    /// @c time, @c controller.success, mission metrics, @c pose_error(e),
    /// @c orientation_error(e), @c distance(a, b), @c inside(a, b),
    /// @c on(a, b), @c touching(a, b), @c collision_count(e),
    /// @c tool_is_attached(tool, robot.flange), @c joint_limits_respected(r),
    /// @c holding(gripper[, object]), @c skill_started(name),
    /// @c skill_succeeded(name), @c skill_failed(name).
    ///
    /// @code
    /// double const d = sim.query("distance(robot.tcp,
    /// red_cube)").value_or(-1.0);
    /// @endcode
    ///
    /// @param p_term Metric name or query.
    /// @return The value; nothing for an unknown name.
    /// @throws std::invalid_argument for a known query on an unknown entity.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double> query(std::string_view p_term) const;

    // -------------------------------------------------------------------------
    /// Evaluates a condition (see @ref evaluate in Metrics.hpp) on the
    /// metrics and the queries.
    /// @param p_condition E.g. @c "inside(red_cube, box)".
    /// @return What was measured and whether it holds.
    // -------------------------------------------------------------------------
    [[nodiscard]] Evaluation evaluate(std::string_view p_condition) const;

    // --- Described world (what the robot was told) ---------------------------

    // -------------------------------------------------------------------------
    /// Pose of an entity as the world file describes it at the start, not
    /// as it is now: a prior the robot may plan from. Robot frames come from
    /// its own kinematics.
    /// @param p_reference @c entity or @c entity.frame.
    /// @return The pose in the world frame.
    /// @throws std::invalid_argument for an unknown entity or frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose described(std::string_view p_reference) const;

    // -------------------------------------------------------------------------
    /// Points of a cartesian trajectory of the scenario, in the robot base
    /// frame (its frame placed where the world file says).
    /// @param p_name Key under @c trajectories: in the scenario.
    /// @return The poses, in order.
    /// @throws std::invalid_argument for an unknown or non cartesian one.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<Pose> waypoints(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Copies the ground-truth prop positions into the @ref WorldModel. Used
    /// by the RL overlay (same observations as the headless env).
    // -------------------------------------------------------------------------
    void observe();

    /// @return Task plugin, or null.
    [[nodiscard]] Mission* mission() const
    {
        return m_mission;
    }

private:

    /// A world entity once built.
    struct Record
    {
        EntitySpec const* spec = nullptr;
        /// ECS entity: props, primitive bodies, rendered models. May be empty.
        compages::world::Entity entity;
        /// Root body in the physics backend (bodies).
        std::string body;
        /// Backend bodies of the entity, for contacts.
        std::unordered_set<std::string> links;
        /// Frames of a body model, at its root (neutral configuration).
        std::shared_ptr<PinocchioBackend> frames;
        /// Where the entity starts (world frame) before randomization.
        Pose nominal{ Vector3::ZERO, Quaternion{} };
        /// Pose captured at the start of the episode.
        Pose initial{ Vector3::ZERO, Quaternion{} };
        /// Peak contacts with other entities (floor excluded).
        int peak_contacts = 0;
    };

    /// Builds the robots, the entities and the physics of the world.
    void build(SceneView* p_view);
    /// Actuators and sensors of one robot and of its mounted tool.
    /// @p_blind turns false when a camera gets a render source.
    void buildDevices(RobotSession& p_robot,
                      EntitySpec const& p_spec,
                      EntitySpec const* p_tool,
                      SceneView* p_view,
                      bool& p_blind);
    /// Builds the ECS entity of one world entity (shape or model).
    void buildRecord(Record& p_record, SceneView* p_view);
    /// True when @p_prop is a body of the physics backend: its level is
    /// rigid, or it is a box a rigid gripper may take, or a container a
    /// rigid object may fall into (FIDELITY.md).
    [[nodiscard]] bool rigidProp(EntitySpec const& p_prop) const;
    /// Free boxes, container walls and grasp welds of the rigid props.
    void addRigidProps(MujocoBackend& p_physics);
    /// Builds the shared MuJoCo physics and connects every robot to it.
    void connectBackend();
    /// Adds the generic skills (@c Home, @c Stop...) to the scheduler.
    void addSkills();
    /// Robot that carries @p_spec: a robot itself, the robot a tool is
    /// mounted on, else the first robot.
    [[nodiscard]] RobotSession& ownerOf(EntitySpec const& p_spec) const;
    /// MuJoCo prefix of the names of robot @p_id (empty for the first).
    [[nodiscard]] std::string prefixOf(std::string_view p_id) const;
    /// Builds the behavior tree of the scenario, if it has one.
    void loadTree();
    /// Copies the physics poses of the bodies to their ECS entities.
    void syncBodies();
    /// Counts the contacts of each entity after a step.
    void track();
    /// Updates the assertions and ends the run when a stop rule holds.
    void settle();
    /// Ends the run: final verdicts, frozen outcome.
    void finish(Outcome p_outcome);

    /// Record of entity @p_id, or null.
    [[nodiscard]] Record const* record(std::string_view p_id) const;
    /// Record of entity @p_id; throws std::invalid_argument if none.
    [[nodiscard]] Record const& require(std::string_view p_id) const;
    /// World pose of an entity from the file, its attachment resolved.
    [[nodiscard]] Pose resolveNominal(EntitySpec const& p_spec) const;
    /// True pose of a frame of an entity (empty frame: its origin).
    [[nodiscard]] Pose framePose(Record const& p_record,
                                 std::string_view p_frame) const;
    /// Contacts of the entity with other entities, floor excluded.
    [[nodiscard]] int contactsOf(Record const& p_record,
                                 std::vector<Contact> const& p_contacts) const;
    /// @c time, @c controller.success and the mission metrics.
    [[nodiscard]] Metrics metrics() const;
    /// Answers a world query with arguments (see @ref query).
    [[nodiscard]] std::optional<double>
    call(std::string const& p_name,
         std::vector<std::string> const& p_args) const;

private:

    /// ECS world of the entities.
    compages::world::World& m_world;
    /// The scenario being run.
    Scenario m_scenario;
    /// Task plugin, not owned, or null.
    Mission* m_mission = nullptr;
    /// Options of the physics built by connectBackend (see physicsOptions).
    MujocoOptions m_physics_options;
    /// Rendering hooks, or null.
    SceneView* m_view = nullptr;
    /// One for all the robots of the world.
    std::shared_ptr<ResourceManager> m_resources;
    /// Every robot, in world file order.
    std::vector<std::unique_ptr<RobotSession>> m_robots;
    /// The first robot.
    RobotSession* m_robot = nullptr;
    /// @ref RobotContext::team: every robot when there are several.
    std::vector<Robot*> m_team;
    /// Beliefs shared by the robots.
    WorldModel m_world_model;
    /// Detectors run on every camera frame.
    PerceptionPipeline m_perception;
    /// Scheduler of every skill.
    std::unique_ptr<SkillScheduler> m_scheduler;
    /// Faults of the scenario.
    FaultInjector m_faults;
    /// One per robot, in the order of @ref m_robots.
    std::vector<RobotContext> m_contexts;
    /// Judges the assertions.
    AssertionEngine m_assertions;
    /// Z-up frame of the world entities (Compages is Y-up).
    compages::world::Entity m_frame;
    /// One per world entity, in file order (a tool in storage has none).
    std::vector<Record> m_records;
    /// The ground truth feeds the world model.
    bool m_oracle = false;

    /// Factory of the behavior tree nodes.
    bt::NodeFactory m_factory;
    /// Clock of the behavior trees: the simulated time.
    [[nodiscard]] bt::Tree::ClockFunction simulatedClock() const;

    /// Blackboard of the behavior tree.
    bt::Blackboard::Ptr m_blackboard;
    /// Behavior tree of the scenario, or null.
    bt::Tree::Ptr m_tree;
    /// Last controller status.
    bt::Status m_status = bt::Status::INVALID;
    /// Why the run ended.
    Outcome m_outcome = Outcome::Running;

    /// Seed of the episode.
    Seed m_seed;
    /// Peak count of simultaneous contacts.
    int m_max_contacts = 0;
    /// Robots (ids) that carried a prop through container walls. Sticky
    /// until reset.
    std::unordered_set<std::string> m_prop_penetration;
    /// How the grippers hold objects, one model per level in use.
    std::vector<std::unique_ptr<GraspModel>> m_grasps;
    /// Teach pendant owns the joints; the mission is frozen.
    bool m_suspended = false;
    /// Observers and bindings given by the mission.
    PlanningDomain m_planning;
    /// After the factory and the scheduler it uses.
    std::unique_ptr<PlanExecutor> m_executor;
};

// ----------------------------------------------------------------------------
/// @param p_outcome Outcome to name.
/// @return For example "timeout" or "controller succeeded".
// ----------------------------------------------------------------------------
[[nodiscard]] char const* toString(Simulation::Outcome p_outcome);

} // namespace robotik
