// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Cell.hpp
/// A robot cell running in simulation: robots and objects sharing one
/// physics, a controller deciding what to do, and the true state of the world
/// for measures.
///
/// A *cell* is the place where robots work, as in a factory (see
/// doc/Glossary.md). It is the Task layer: a C++ program builds one from a
/// @ref robotik::CellDescription, without any file. Headless unless a @ref
///     robotik::SceneView is
/// given.
///
/// @par Contract
/// - Promises: same description and same seed, same run. Skills, observers and
///   missions read beliefs; the truth API (@c pose, @c query) is for metrics,
///   assertions and RL rewards (@c make @c check-layers checks it).
/// - Expects: one @c step per control period; entities are found by id each
///   time, never kept by a plugin.
/// - Errors: a description that cannot be built throws @c std::runtime_error in
///   the constructor; an unknown entity or robot in a query throws
///   @c std::invalid_argument.
#pragma once

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Task/Perception/Detector.hpp"
#include "Robotik/Task/Perception/WorldModel.hpp"
#include "Robotik/Task/Planning/PlanExecutor.hpp"
#include "Robotik/Task/Planning/PlanningDomain.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Task/Faults.hpp"
#include "Robotik/Task/Metrics.hpp"
#include "Robotik/Task/Mission.hpp"
#include "Robotik/Task/RobotContext.hpp"
#include "Robotik/Task/Scheduler.hpp"
#include "Robotik/Task/CellDescription.hpp"

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
/// Robots doing a job together, replayable from a seed.
///
/// Built from a @ref CellDescription, written in C++ or read from YAML by the
/// Package layer. Each @ref step — faults, controller tick (the behavior tree
/// or the plan requests skills), scheduler update (admits and ticks skills),
/// robot step (physics, sensors, perception into the @ref WorldModel),
/// simulated suction, mission. When the perception fidelity of an object is
/// @c truth or @c noisy_truth (@ref PerceptionSpec), the cell writes its
/// belief from the ground truth, as a mock of the detectors.
///
/// The controller stops being ticked once it succeeds or fails (see
/// @ref status), or when @ref stopController is called; the physics keeps
/// running. How a run is judged (time limit, assertions) is not the job of
/// the cell: see the @c Simulation of the Package layer.
///
/// The ground truth (@ref pose, @ref query) is for measures and rewards:
/// skills only see the @ref RobotContext.
///
/// @code
/// robotik::CellDescription description;
/// description.world.entities.push_back(
///     { .id = "arm", .type = robotik::EntitySpec::Type::Robot, .model = "arm.urdf" });
/// description.controller.behavior_tree = "wave.bt.yaml";
/// compages::world::World world;
/// robotik::Cell cell(world, description);
/// while (cell.status() == bt::Status::RUNNING || cell.status() == bt::Status::INVALID)
///     cell.step(robotik::Seconds(0.01));
/// @endcode
// ****************************************************************************
class Cell
{
public:

    Cell(Cell const&) = delete;
    Cell& operator=(Cell const&) = delete;

    // -------------------------------------------------------------------------
    /// Builds the robots, sensors, objects and physics of the description,
    /// calls @ref Mission::setup, then resets with the description seed.
    /// @param p_world ECS world receiving the entities.
    /// @param p_description What to build.
    /// @param p_view Rendering hooks, or null for a headless run.
    /// @param p_mission Task plugin (skills, detectors, metrics); not owned.
    /// @throws std::runtime_error when the cell cannot be built.
    // -------------------------------------------------------------------------
    Cell(compages::world::World& p_world,
         CellDescription p_description,
         SceneView* p_view = nullptr,
         Mission* p_mission = nullptr);
    ~Cell();

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

    /// @return What the cell was built from.
    [[nodiscard]] CellDescription const& description() const
    {
        return m_description;
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
    /// Options of the physics the core builds after @ref Mission::setup — a
    /// mission may tune them there (a floating base, no gravity
    /// compensation, the step and the rotors of a legged robot). The world
    /// still adds its floor. The robots, tools, bodies and rigid props of the
    /// world are loaded as usual.
    ///
    /// @code
    /// void Walk::setup(robotik::Cell& p_cell, robotik::SceneView*)
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

    // -------------------------------------------------------------------------
    /// Stops ticking the controller (behavior tree, plan); physics, sensors
    /// and mission go on. Called when a run is judged over. Undone by
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

private:

    /// Writes the beliefs of the props at the truth or noisy_truth level.
    void perceiveTruth();

public:

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
    /// What the cell was built from.
    CellDescription m_description;
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
    bt::NodeFactory m_factory;
    /// Clock of the behavior trees: the simulated time.
    [[nodiscard]] bt::Tree::ClockFunction simulatedClock() const;

    /// Blackboard of the behavior tree.
    bt::Blackboard::Ptr m_blackboard;
    /// Behavior tree of the scenario, or null.
    bt::Tree::Ptr m_tree;
    /// Last controller status.
    bt::Status m_status = bt::Status::INVALID;
    /// The controller is no longer ticked.
    bool m_controller_stopped = false;

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



} // namespace robotik
