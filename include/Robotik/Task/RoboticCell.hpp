// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file RoboticCell.hpp
/// A robot cell running in simulation: robots and objects sharing one
/// physics, a controller deciding what to do, and the true state of the world
/// for measures.
///
/// A *cell* is the place where robots work, as in a factory (see
/// doc/reference/glossary.md). It is the Task layer: a C++ program builds one
/// from a @ref robotik::RoboticCellDescription, without any file. Headless unless a
/// @ref robotik::SceneView is given.
///
/// @par Contract
/// - Promises: same description and same seed, same run. Skills, observers and
///   scenario logics read beliefs; the truth API (@c pose, @c query) is for metrics,
///   assertions and RL rewards (@c make @c check-layers checks it).
/// - Expects: one @c step per control period; entities are found by id each
///   time, never kept by a plugin.
/// - Errors: a description that cannot be built throws @c std::runtime_error in
///   the constructor; an unknown entity or robot in a query throws
///   @c std::invalid_argument.
#pragma once

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Condition.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Perception/Detector.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/RoboticCellDescription.hpp"
#include "Robotik/Task/ScenarioLogic.hpp"
#include "Robotik/Task/Scheduler.hpp"

#include "Robotik/Task/Status.hpp"
#include "Robotik/Task/TreeTypes.hpp"

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
/// Robots doing a job together, replayable from a seed.
///
/// A robot alone (@ref Robot) knows its joints and its sensors, but a job
/// needs more: objects to move, a controller that chooses the next skill, a
/// scheduler that shares the devices, faults, and the true state of the
/// world to measure the result. The cell puts these pieces together around
/// one physics. Use it when a program runs a job, with or without files: a
/// test or an application builds it from a @ref RoboticCellDescription (or calls
/// @ref openRoboticCell), and the Package layer builds it from a scenario file.
/// To move one robot without a job, @ref openRobot is enough.
///
/// Each @ref step runs, in this order: the faults, the controller tick (the
/// behavior tree or the plan requests skills), the scheduler (it admits and
/// ticks the skills), the robot step (physics, sensors, perception into the
/// @ref WorldModel), the simulated suction, and the scenario logic. When the
/// perception fidelity of an object is @c truth or @c noisy_truth
/// (@ref PerceptionSpec), the cell writes its belief from the ground truth,
/// as a mock of the detectors, and the run report says so.
///
/// The same description and the same seed give the same run. The
/// controller stops being ticked once it succeeds or fails (see
/// @ref status), or when @ref stopController is called; the physics keeps
/// running. The cell does not judge a run (time limit, assertions): that is
/// the job of the @c Simulation of the Package layer.
///
/// The cell is the only class that holds both the decisions and the ground
/// truth. The truth (@ref requirePose, @ref requireQuery) is for measures, assertions and
/// rewards; skills see only the @ref RobotContext, and @c make
/// @c check-layers compiles the skills and the planner without this header,
/// so that they cannot call it.
///
/// From tests/Examples/TestLayerExamples.cpp (TaskRunsOneSkill):
/// @code
/// robotik::EntitySpec arm;
/// arm.id = "arm";
/// arm.type = robotik::EntitySpec::Type::Robot;
/// arm.model = "robot_6axis.urdf";
/// robotik::RoboticCellDescription description;
/// description.world.entities = { arm };
/// auto opened = robotik::openRoboticCell(std::move(description));
/// robotik::RoboticCell& cell = *opened.value();
/// auto const raise = cell.skills().add<robotik::MoveJointSkill>(
///     { .name = "Raise" }, "joint2", 0.5);
/// cell.run(raise, robotik::Seconds(10.0));   // SkillState::Succeeded
/// @endcode
// ****************************************************************************
class RoboticCell
{
public:

    RoboticCell(RoboticCell const&) = delete;
    RoboticCell& operator=(RoboticCell const&) = delete;

    // -------------------------------------------------------------------------
    /// Builds the robots, sensors, objects and physics of the description,
    /// calls @ref ScenarioLogic::setup, then resets with the description seed.
    /// @param p_world ECS world receiving the entities.
    /// @param p_description What to build.
    /// @param p_view Rendering hooks, or null for a headless run.
    /// @param p_logic Task plugin (skills, detectors, metrics); not owned.
    /// @throws std::runtime_error when the description breaks a rule of
    ///   @ref validate (message of validate) or a model cannot be loaded.
    // -------------------------------------------------------------------------
    RoboticCell(compages::world::World& p_world,
                RoboticCellDescription p_description,
                SceneView* p_view = nullptr,
                ScenarioLogic* p_logic = nullptr);
    ~RoboticCell();

    // -------------------------------------------------------------------------
    /// Starts a new episode: same seed, same episode.
    /// @param p_seed Derives the prop layout ("world"), the faults
    ///   ("faults") and the sensor noise (one stream per sensor name).
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    /// New episode with the seed of the description.
    void reset()
    {
        reset(Seed{ m_description.seed });
    }

    // -------------------------------------------------------------------------
    /// Advances the run by one step (see the class comment for the order).
    /// @param p_dt Duration of the step.
    // -------------------------------------------------------------------------
    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    /// Operator mode: physics and sensors keep running, the behavior tree,
    /// the scheduler, the scenario logic and the assertions do not. Joint commands
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

    /// @return What the cell was built from.
    [[nodiscard]] RoboticCellDescription const& description() const
    {
        return m_description;
    }

    /// @return Seed of the current episode.
    [[nodiscard]] Seed seed() const
    {
        return m_seed;
    }

    /// @return Simulated time since the reset.
    // -------------------------------------------------------------------------
    /// Requests a skill and steps until it ends: run one skill without a
    /// scenario logic, a tree or a planner.
    ///
    /// @code
    /// auto const raise = cell.skills().add<robotik::MoveJointSkill>(
    ///     { .name = "Raise" }, "joint2", 0.5);
    /// cell.run(raise, robotik::Seconds(10.0));   // SkillState::Succeeded
    /// @endcode
    ///
    /// @param p_skill Skill registered on @ref skills.
    /// @param p_limit Simulated time allowed, from now.
    /// @param p_dt Step.
    /// @return The final state, or @c Waiting / @c Running at the limit.
    // -------------------------------------------------------------------------
    SkillState run(SkillId p_skill, Seconds p_limit, Seconds p_dt = Seconds(0.01));

    // -------------------------------------------------------------------------
    /// Steps until the controller ends (behavior tree, plan, or the status of
    /// the scenario logic), or until the limit.
    /// @param p_limit Simulated time allowed, from now.
    /// @param p_dt Step.
    /// @return @ref status at the end: @c RUNNING at the limit.
    // -------------------------------------------------------------------------
    Status run(Seconds p_limit, Seconds p_dt = Seconds(0.01));

    // -------------------------------------------------------------------------
    /// Reaches a goal without writing a sequence of skills: the planner finds
    /// the actions in the PDDL domain, the cell runs them with the skills of
    /// the bindings (@ref planning) and plans again when the beliefs
    /// disagree. The @c planner block of a scenario calls it.
    ///
    /// @code
    /// robotik::PlannerSpec goal;
    /// goal.domain = "pick_and_place.domain.pddl";
    /// goal.goal = { robotik::parseLiteral("(in red_cube box)").take() };
    /// cell.achieve(goal);
    /// robotik::Status const done = cell.run(robotik::Seconds(60.0));
    /// @endcode
    ///
    /// @param p_spec Domain file, goal and settings of the planner.
    /// @throws std::runtime_error when the domain, the goal, the objects and
    ///   the bindings do not agree (every problem is listed), or when the
    ///   controller is a behavior tree.
    // -------------------------------------------------------------------------
    void achieve(PlannerSpec p_spec);

    [[nodiscard]] Seconds time() const
    {
        return m_robot->time();
    }

    /// @return The first robot of the world file.
    [[nodiscard]] RobotSession& robot()
    {
        return *m_robot;
    }

    /// @return The first robot, read only (a const cell commands nothing).
    [[nodiscard]] RobotSession const& robot() const
    {
        return *m_robot;
    }

    // -------------------------------------------------------------------------
    /// Finds a robot. The robots of a world share one physics, one resource
    /// manager, one scheduler and the beliefs.
    /// @param p_id Entity id in the world file.
    /// @return The robot, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] RobotSession* robot(std::string_view p_id);

    /// @copydoc robot(std::string_view)
    [[nodiscard]] RobotSession const* robot(std::string_view p_id) const;

    /// @return Every robot, in world file order (the first is @ref robot()).
    [[nodiscard]] std::vector<std::unique_ptr<RobotSession>> const& robots()
    {
        return m_robots;
    }

    /// @return Every robot, read only, in world file order.
    [[nodiscard]] std::vector<RobotSession const*> const& robots() const
    {
        return m_robot_views;
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
    /// Options of the physics the core builds after @ref ScenarioLogic::setup — a
    /// scenario logic may tune them there (a floating base, no gravity
    /// compensation, the step and the rotors of a legged robot). The world
    /// still adds its floor. The robots, tools, bodies and rigid props of the
    /// world are loaded as usual.
    ///
    /// @code
    /// void Walk::setup(robotik::RoboticCell& p_cell, robotik::SceneView*)
    /// {
    ///     p_cell.physicsOptions().floating_base = true;
    ///     p_cell.physicsOptions().gravity_compensation = false;
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
    [[nodiscard]] SkillScheduler& skills()
    {
        return *m_scheduler;
    }

    /// @return The scheduler, read only: states and trace, no request.
    [[nodiscard]] SkillScheduler const& skills() const
    {
        return *m_scheduler;
    }

    /// @return Factory the behavior tree is built from. Skills registered
    ///   later under the same name replace a node added here. Include
    ///   @c Robotik/Task/Skills/SkillNodes.hpp to add your own nodes.
    [[nodiscard]] bt::NodeFactory& factory();

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
    [[nodiscard]] RobotContext& requireContext(std::string_view p_id);

    /// @return First camera of the robots, or null.
    [[nodiscard]] Camera* camera();

    /// @copydoc camera()
    [[nodiscard]] Camera const* camera() const;

    /// @return True when the simulator feeds the beliefs of a prop or more
    ///   (see @ref PerceptionFidelity).
    [[nodiscard]] bool oracle() const
    {
        return m_oracle;
    }

    // -------------------------------------------------------------------------
    /// @param p_id Entity id of a prop.
    /// @return Where the beliefs about it come from.
    // -------------------------------------------------------------------------
    [[nodiscard]] PerceptionFidelity perceptionOf(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// @return The perception level of most props, then the exceptions, e.g.
    ///   "truth, red_cube: sensed", for reports.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string perceptionSummary() const;

    /// @return Behavior tree, or null if the controller has none. With a
    ///   planner: the tree of the current plan. Include
    ///   @c Robotik/Task/Skills/SkillNodes.hpp to walk it.
    [[nodiscard]] bt::Tree const* tree() const;

    // -------------------------------------------------------------------------
    /// @return Observers and bindings of the PDDL planner. The scenario logic fills
    ///   them in @ref ScenarioLogic::setup; they are used when the scenario
    ///   controller is a @c planner (see @ref PlanningDomain).
    // -------------------------------------------------------------------------
    [[nodiscard]] PlanningDomain& planning()
    {
        return m_planning;
    }

    /// @return The planner controller, or null when the scenario has none.
    [[nodiscard]] PlanExecutor* planner()
    {
        return m_executor.get();
    }

    /// @return The planner controller, read only (reports, panels).
    [[nodiscard]] PlanExecutor const* planner() const
    {
        return m_executor.get();
    }

    /// @return Controller status (behavior tree, planner, or scenario logic without
    ///   one); @c IDLE before the first tick.
    [[nodiscard]] Status status() const
    {
        return m_status;
    }

    // --- End of run
    // -----------------------------------------------------------

    // -------------------------------------------------------------------------
    /// Stops ticking the controller (behavior tree, plan); physics, sensors
    /// and scenario logic go on. Called when a run is judged over. Undone by
    /// @ref reset.
    // -------------------------------------------------------------------------
    void stopController()
    {
        m_controller_stopped = true;
    }

    /// @return True once the controller is no longer ticked.
    [[nodiscard]] bool controllerStopped() const
    {
        return m_controller_stopped;
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
    [[nodiscard]] Pose requirePose(std::string_view p_reference) const;

    /// @return Pose of entity @p_id when the episode started.
    [[nodiscard]] Pose initialPose(std::string_view p_id) const;

    /// @return ECS entity of @p_id (props, primitive bodies, rendered
    ///   models), or an empty handle.
    [[nodiscard]] compages::world::Entity entity(std::string_view p_id);

    // -------------------------------------------------------------------------
    /// Adds a primitive drawn for the eye only (beam, trail), in the Z-up
    /// world frame: its position and rotation are world coordinates. It is
    /// not an entity of the world: nothing collides with it, and assertions
    /// cannot name it. The scenario logic moves it.
    /// @param p_object Shape, size and color.
    /// @return The new entity.
    // -------------------------------------------------------------------------
    compages::world::Entity decorate(ecs::Shape const& p_object);

    // -------------------------------------------------------------------------
    /// Value of a metric or of a world query, written as in a condition:
    /// @c time, @c controller.success, plugin metrics, or a query among
    /// @c (pose_error e), @c (orientation_error e), @c (distance a b),
    /// @c (inside a b), @c (on a b), @c (touching a b),
    /// @c (collision_count e), @c (tool_is_attached tool robot.flange),
    /// @c (joint_limits_respected r), @c (holding gripper [object]),
    /// @c (skill_started name), @c (skill_succeeded name),
    /// @c (skill_failed name), and the queries added by @ref addQuery.
    ///
    /// @code
    /// double const d = cell.requireQuery("(distance robot.tcp red_cube)").value();
    /// @endcode
    ///
    /// @param p_term Metric name or query.
    /// @return The value; nothing when it has no value now.
    /// @throws std::invalid_argument for an unknown name, a wrong number of
    ///   arguments or an unknown entity.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double>
    requireQuery(std::string_view p_term) const;

    // -------------------------------------------------------------------------
    /// Asks a world query from C++, without writing text.
    ///
    /// @code
    /// bool const done = cell.requireQuery("inside", { "red_cube", "box" }) == 1.0;
    /// @endcode
    ///
    /// @param p_name Query name, e.g. @c "inside".
    /// @param p_args Its arguments.
    /// @return The answer.
    /// @throws std::invalid_argument for an unknown query, a wrong number of
    ///   arguments or an unknown entity.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double>
    requireQuery(std::string_view p_name,
                 std::vector<std::string> const& p_args) const;

    // -------------------------------------------------------------------------
    /// Adds a world query that conditions may use, e.g. a measure only a
    /// plugin knows. Call it in @ref ScenarioLogic::setup, before the scenario
    /// conditions are read.
    /// @param p_query Name, number of arguments and answer; it reads the
    ///   truth and must not change the world.
    /// @throws std::invalid_argument when the name is already a query.
    // -------------------------------------------------------------------------
    void addQuery(ConditionQuery p_query);

    // -------------------------------------------------------------------------
    /// @return The metrics published now and the queries, for
    ///   @ref compileCondition.
    // -------------------------------------------------------------------------
    [[nodiscard]] ConditionVocabulary vocabulary() const;

    // -------------------------------------------------------------------------
    /// Reads a condition once, to evaluate it at each step.
    /// @param p_text E.g. @c "(inside red_cube box)".
    /// @return The condition; fails on an unknown name (see Condition.hpp).
    // -------------------------------------------------------------------------
    [[nodiscard]] Result<Condition> compile(std::string_view p_text) const;

    // -------------------------------------------------------------------------
    /// Reads and evaluates a condition now (a one-off question; at each step,
    /// @ref compile once and call @ref robotik::evaluate).
    /// @param p_condition E.g. @c "(inside red_cube box)".
    /// @return What was measured and whether it holds; @c known false and
    ///   the reason when the condition does not compile.
    // -------------------------------------------------------------------------
    [[nodiscard]] Evaluation evaluate(std::string_view p_condition) const;

    /// @return @c time, @c controller.success, the planner counters and the
    ///   plugin metrics, measured now.
    [[nodiscard]] Metrics metrics() const;

    // --- Described world (what the robot was told) ---------------------------

    // -------------------------------------------------------------------------
    /// Pose of an entity as the world file describes it at the start, not
    /// as it is now: a prior the robot may plan from. Robot frames come from
    /// its own kinematics.
    /// @param p_reference @c entity or @c entity.frame.
    /// @return The pose in the world frame.
    /// @throws std::invalid_argument for an unknown entity or frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose requireDescribedPose(std::string_view p_reference) const;

    // -------------------------------------------------------------------------
    /// Points of a cartesian trajectory of the scenario, in the robot base
    /// frame (its frame placed where the world file says).
    /// @param p_name Key under @c trajectories: in the scenario.
    /// @return The poses, in order.
    /// @throws std::invalid_argument for an unknown or non cartesian one.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<Pose>
    requireWaypoints(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Whether a prop is a body of the physics backend: its level is rigid,
    /// or it is a box a rigid gripper may take, or a container a rigid
    /// object may fall into (doc/reference/scenario-file.md, "Physics
    /// fidelity"). Read from the description only.
    /// @param p_prop An entity of the world.
    /// @return False for anything else than a prop with a shape.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool rigidProp(EntitySpec const& p_prop) const;

    // -------------------------------------------------------------------------
    /// Copies the ground-truth prop positions into the @ref WorldModel. Used
    /// by the RL overlay (same observations as the headless env).
    // -------------------------------------------------------------------------
    void observe();

private:

    /// Writes the beliefs of the props at the truth or noisy_truth level.
    void perceiveTruth();

public:

    /// @return Task plugin, or null.
    [[nodiscard]] ScenarioLogic* logic() const
    {
        return m_logic;
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
        /// Where the beliefs about this prop come from.
        PerceptionFidelity perception = PerceptionFidelity::Sensed;
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
    /// Free boxes, container walls and grasp welds of the rigid props.
    void addRigidProps(MujocoBackend& p_physics);
    /// Builds the shared MuJoCo physics and connects every robot to it.
    void connectBackend();
    /// Adds the generic skills (@c Home, @c Stop...) to the scheduler.
    void addSkills();
    /// Robot that carries @p_spec: a robot itself, the robot a tool is
    /// mounted on, else the first robot.
    [[nodiscard]] RobotSession const& ownerOf(EntitySpec const& p_spec) const;
    /// MuJoCo prefix of the names of robot @p_id (empty for the first).
    [[nodiscard]] std::string prefixOf(std::string_view p_id) const;
    /// Builds the behavior tree of the scenario, if it has one.
    void loadTree();
    /// Copies the physics poses of the bodies to their ECS entities.
    void syncBodies();
    /// Counts the contacts of each entity after a step.
    void track();

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
    /// The queries every cell answers (see @ref requireQuery).
    [[nodiscard]] std::vector<ConditionQuery> builtinQueries() const;

private:

    /// ECS world of the entities.
    compages::world::World& m_world;
    /// What the cell was built from.
    RoboticCellDescription m_description;
    /// Task plugin, not owned, or null.
    ScenarioLogic* m_logic = nullptr;
    /// Options of the physics built by connectBackend (see physicsOptions).
    MujocoOptions m_physics_options;
    /// Rendering hooks, or null.
    SceneView* m_view = nullptr;
    /// One for all the robots of the world.
    std::shared_ptr<ResourceManager> m_resources;
    /// Every robot, in world file order.
    std::vector<std::unique_ptr<RobotSession>> m_robots;
    /// The same robots, read only, for @ref robots() const.
    std::vector<RobotSession const*> m_robot_views;
    /// The first robot.
    RobotSession* m_robot = nullptr;
    /// @ref RobotContext::team — every robot when there are several.
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
    /// Z-up frame of the world entities (Compages is Y-up).
    compages::world::Entity m_frame;
    /// One per world entity, in file order (a tool in storage has none).
    std::vector<Record> m_records;
    /// The simulator feeds the beliefs of a prop or more.
    bool m_oracle = false;
    /// A prop or more is seen by the perception pipeline (gate its updates).
    bool m_any_sensed = true;
    /// Noise and misses of the noisy_truth level.
    Random m_perception_random;

    /// Factory of the behavior tree nodes.
    std::unique_ptr<bt::NodeFactory> m_factory;
    /// Clock of the behavior trees: the simulated time.
    [[nodiscard]] TreeClock simulatedClock() const;

    /// Blackboard of the behavior tree.
    std::shared_ptr<bt::Blackboard> m_blackboard;
    /// Behavior tree of the scenario, or null.
    std::unique_ptr<bt::Tree> m_tree;
    /// Last controller status.
    Status m_status = Status::IDLE;
    /// The controller is no longer ticked.
    bool m_controller_stopped = false;

    /// World queries of the conditions: built-in, then added ones.
    std::vector<ConditionQuery> m_queries;

    /// Seed of the episode.
    Seed m_seed;
    /// Peak count of simultaneous contacts.
    int m_max_contacts = 0;
    /// Robots (ids) that carried a prop through container walls. Sticky
    /// until reset.
    std::unordered_set<std::string> m_prop_penetration;
    /// How the grippers hold objects, one model per level in use.
    std::vector<std::unique_ptr<GraspModel>> m_grasps;
    /// Teach pendant owns the joints; the scenario logic is frozen.
    bool m_suspended = false;
    /// Observers and bindings given by the scenario logic.
    PlanningDomain m_planning;
    /// After the factory and the scheduler it uses.
    std::unique_ptr<PlanExecutor> m_executor;
};



} // namespace robotik
