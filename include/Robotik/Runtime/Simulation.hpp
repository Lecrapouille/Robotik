// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Simulation.hpp
//! @brief Runs a scenario: builds its world, drives the controller, applies
//! the faults and judges the assertions. Headless unless a @ref SceneView is
//! given.
#pragma once

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

namespace ecs
{
struct SceneObject;
}

// ****************************************************************************
//! @brief One scenario instance, replayable from a seed.
//!
//! Each @ref step: faults, behavior tree tick (requests skills), scheduler
//! update (admits and ticks skills), robot step (physics, sensors, perception
//! into the @ref WorldModel), simulated suction, mission, assertions. When
//! the scenario allows it (@ref OraclePolicy), an oracle feeds the world
//! model with the ground truth while a sensor resource is available.
//!
//! The run ends when the controller finishes, the @c stop_when condition
//! holds, the @c timeout elapses or an assertion marked @c stop fails. The
//! physics keeps running afterwards; the verdicts are frozen.
//!
//! The ground truth (@ref pose, @ref query) is for evaluation: skills only
//! see the @ref RobotContext.
//!
//! @code
//! compages::world::World world;
//! robotik::Simulation sim(world, robotik::Scenario::load(path, assets));
//! sim.reset(robotik::Seed{ 7 });
//! while (!sim.finished())
//!     sim.step(Seconds(0.01));
//! for (robotik::Check const& check : sim.checks())
//!     if (check.verdict != robotik::Check::Verdict::Passed) { ... }
//! @endcode
// ****************************************************************************
class Simulation
{
public:

    //! @brief Why a run ended.
    enum class Outcome : std::uint8_t
    {
        Running,
        ControllerSucceeded,
        ControllerFailed,
        StopCondition,
        Timeout,
        AssertionFailed,
        //!< The host ended the run (@ref conclude).
        Interrupted,
    };

    Simulation(Simulation const&) = delete;
    Simulation& operator=(Simulation const&) = delete;

    //! @param p_view Rendering hooks, or null for a headless run.
    //! @param p_mission Task plugin (skills, detectors, metrics); not owned.
    Simulation(compages::world::World& p_world,
               Scenario p_scenario,
               SceneView* p_view = nullptr,
               Mission* p_mission = nullptr);
    ~Simulation();

    // -------------------------------------------------------------------------
    //! @brief Starts a new episode: same seed, same episode.
    //!
    //! @p_seed derives the prop layout ("world"), the faults ("faults") and
    //! the sensor noise (one stream per sensor name).
    // -------------------------------------------------------------------------
    void reset(Seed p_seed);

    //! @brief New episode with the scenario seed.
    void reset()
    {
        reset(Seed{ m_scenario.seed });
    }

    void step(Seconds p_dt);

    // -------------------------------------------------------------------------
    //! @brief Operator mode: physics and sensors keep running, the behavior
    //! tree, the scheduler, the mission and the assertions do not. Joint
    //! commands then come from the teach pendant.
    // -------------------------------------------------------------------------
    void suspend(bool p_on)
    {
        m_suspended = p_on;
    }

    [[nodiscard]] bool suspended() const
    {
        return m_suspended;
    }

    [[nodiscard]] Scenario const& scenario() const
    {
        return m_scenario;
    }

    [[nodiscard]] Seed seed() const
    {
        return m_seed;
    }

    [[nodiscard]] Seconds time() const
    {
        return m_robot->time();
    }

    [[nodiscard]] RobotSession& robot() const
    {
        return *m_robot;
    }

    [[nodiscard]] WorldModel& worldModel()
    {
        return m_world_model;
    }

    [[nodiscard]] WorldModel const& worldModel() const
    {
        return m_world_model;
    }

    //! @brief Detectors run on every camera frame (empty by default).
    [[nodiscard]] PerceptionPipeline& perception()
    {
        return m_perception;
    }

    [[nodiscard]] PerceptionPipeline const& perception() const
    {
        return m_perception;
    }

    [[nodiscard]] SkillScheduler& skills() const
    {
        return *m_scheduler;
    }

    //! @brief Factory the behavior tree is built from. Skills registered
    //! later under the same name replace a node added here.
    [[nodiscard]] bt::NodeFactory& factory()
    {
        return m_factory;
    }

    [[nodiscard]] FaultInjector& faults()
    {
        return m_faults;
    }

    [[nodiscard]] RobotContext& context()
    {
        return m_context;
    }

    //! @brief First camera of the robot, or null.
    [[nodiscard]] Camera* camera() const;

    //! @brief The ground truth feeds the world model (see @ref OraclePolicy).
    [[nodiscard]] bool oracle() const
    {
        return m_oracle;
    }

    //! @brief Behavior tree, or null if the controller has none. With a
    //! planner: the tree of the current plan.
    [[nodiscard]] bt::Tree const* tree() const
    {
        return m_executor ? m_executor->tree() : m_tree.get();
    }

    // -------------------------------------------------------------------------
    //! @brief Observers and bindings of the PDDL planner. The mission fills
    //! them in @ref Mission::setup; they are used when the scenario
    //! controller is a @c planner (see @ref PlanningDomain).
    // -------------------------------------------------------------------------
    [[nodiscard]] PlanningDomain& planning()
    {
        return m_planning;
    }

    //! @brief The planner controller, or null when the scenario has none.
    [[nodiscard]] PlanExecutor* planner() const
    {
        return m_executor.get();
    }

    //! @brief Controller status (behavior tree, or mission without one).
    [[nodiscard]] bt::Status status() const
    {
        return m_status;
    }

    // --- End of run -----------------------------------------------------------

    [[nodiscard]] Outcome outcome() const
    {
        return m_outcome;
    }

    [[nodiscard]] bool finished() const
    {
        return m_outcome != Outcome::Running;
    }

    // -------------------------------------------------------------------------
    //! @brief Ends the run now (the host's own limit): pending assertions get
    //! their final verdict. No effect on a finished run.
    // -------------------------------------------------------------------------
    void conclude();

    //! @brief Assertions with their verdicts (live while running).
    [[nodiscard]] std::vector<Check> const& checks() const
    {
        return m_assertions.checks();
    }

    //! @brief Run finished and every assertion passed.
    [[nodiscard]] bool passed() const
    {
        return m_assertions.passed();
    }

    //! @brief Peak count of simultaneous contacts (floor excluded) since the
    //! last reset, whatever the bodies.
    [[nodiscard]] int maxContacts() const
    {
        return m_max_contacts;
    }

    // --- Ground truth (world frame; for evaluation, not for skills) ----------

    // -------------------------------------------------------------------------
    //! @brief Pose of @c entity or @c entity.frame: a robot link, a link of a
    //! body model, or the origin of a prop or a primitive body.
    //! @throws std::invalid_argument for an unknown entity or frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose pose(std::string_view p_reference) const;

    //! @brief Pose of @p_id when the episode started.
    [[nodiscard]] Pose initialPose(std::string_view p_id) const;

    //! @brief ECS entity of @p_id (props, primitive bodies, rendered models),
    //! or an empty handle.
    [[nodiscard]] compages::world::Entity entity(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    //! @brief A primitive drawn for the eye only (beam, trail), in the Z-up
    //! world frame: its position and rotation are world coordinates. It is
    //! not an entity of the world: nothing collides with it, and assertions
    //! cannot name it. The mission moves it.
    // -------------------------------------------------------------------------
    compages::world::Entity decorate(ecs::SceneObject const& p_object);

    // -------------------------------------------------------------------------
    //! @brief Value of a metric or of a world query, as used in conditions:
    //! @c time, @c controller.success, mission metrics, @c pose_error(e),
    //! @c orientation_error(e), @c distance(a, b), @c inside(a, b),
    //! @c on(a, b), @c touching(a, b), @c collision_count(e),
    //! @c tool_is_attached(tool, robot.flange), @c joint_limits_respected(r),
    //! @c holding(gripper[, object]), @c skill_started(name),
    //! @c skill_succeeded(name), @c skill_failed(name).
    //! @return Nothing for an unknown name.
    //! @throws std::invalid_argument for a known query on an unknown entity.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::optional<double> query(std::string_view p_term) const;

    //! @brief Evaluates a condition (see @ref evaluate in Metrics.hpp).
    [[nodiscard]] Evaluation evaluate(std::string_view p_condition) const;

    // --- Described world (what the robot was told) ---------------------------

    // -------------------------------------------------------------------------
    //! @brief Pose of @c entity or @c entity.frame as the world file
    //! describes it at the start, not as it is now: a prior the robot may
    //! plan from. Robot frames come from its own kinematics.
    //! @throws std::invalid_argument for an unknown entity or frame.
    // -------------------------------------------------------------------------
    [[nodiscard]] Pose described(std::string_view p_reference) const;

    // -------------------------------------------------------------------------
    //! @brief Points of the cartesian trajectory @p_name of the scenario, in
    //! the robot base frame (its frame placed where the world file says).
    //! @throws std::invalid_argument for an unknown or non cartesian one.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<Pose> waypoints(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Copies the ground-truth prop positions into the @ref WorldModel.
    //! Used by the RL overlay (same observations as the headless env).
    // -------------------------------------------------------------------------
    void observe();

    //! @brief Task plugin, or null.
    [[nodiscard]] Mission* mission() const
    {
        return m_mission;
    }

private:

    //! @brief A world entity once built.
    struct Record
    {
        EntitySpec const* spec = nullptr;
        //!< ECS entity: props, primitive bodies, rendered models. May be empty.
        compages::world::Entity entity;
        //!< Root body in the physics backend (bodies).
        std::string body;
        //!< Backend bodies of the entity, for contacts.
        std::unordered_set<std::string> links;
        //!< Frames of a body model, at its root (neutral configuration).
        std::shared_ptr<PinocchioBackend> frames;
        //!< Where the entity starts (world frame) before randomization.
        Pose nominal{ zero3(), Quaternion{} };
        //!< Pose captured at the start of the episode.
        Pose initial{ zero3(), Quaternion{} };
        //!< Peak contacts with other entities (floor excluded).
        int peak_contacts = 0;
    };

    void build(SceneView* p_view);
    void buildRecord(Record& p_record, SceneView* p_view);
    void connectBackend();
    void addSkills();
    void loadTree();
    void syncBodies();
    void track();
    void settle();
    void finish(Outcome p_outcome);

    [[nodiscard]] Record const* record(std::string_view p_id) const;
    [[nodiscard]] Record const& require(std::string_view p_id) const;
    [[nodiscard]] Pose resolveNominal(EntitySpec const& p_spec) const;
    [[nodiscard]] Pose framePose(Record const& p_record, std::string_view p_frame) const;
    [[nodiscard]] int contactsOf(Record const& p_record,
                                 std::vector<Contact> const& p_contacts) const;
    [[nodiscard]] Metrics metrics() const;
    [[nodiscard]] std::optional<double> call(std::string const& p_name,
                                             std::vector<std::string> const& p_args) const;

private:

    compages::world::World& m_world;
    Scenario m_scenario;
    Mission* m_mission = nullptr;
    SceneView* m_view = nullptr;
    std::unique_ptr<RobotSession> m_robot;
    WorldModel m_world_model;
    PerceptionPipeline m_perception;
    std::unique_ptr<SkillScheduler> m_scheduler;
    FaultInjector m_faults;
    RobotContext m_context;
    AssertionEngine m_assertions;
    //!< Z-up frame of the world entities (Compages is Y-up).
    compages::world::Entity m_frame;
    //!< One per world entity, in file order (a tool in storage has none).
    std::vector<Record> m_records;
    //!< The ground truth feeds the world model.
    bool m_oracle = false;

    bt::NodeFactory m_factory;
    bt::Blackboard::Ptr m_blackboard;
    bt::Tree::Ptr m_tree;
    bt::Status m_status = bt::Status::INVALID;
    Outcome m_outcome = Outcome::Running;

    Seed m_seed;
    int m_max_contacts = 0;
    //!< A carried prop went through container walls. Sticky until reset.
    bool m_prop_penetration = false;
    //!< Teach pendant owns the joints; the mission is frozen.
    bool m_suspended = false;
    PlanningDomain m_planning;
    //!< After the factory and the scheduler it uses.
    std::unique_ptr<PlanExecutor> m_executor;
};

[[nodiscard]] char const* toString(Simulation::Outcome p_outcome);

} // namespace robotik
