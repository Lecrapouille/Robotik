// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include <limits>

#include "RobotIK/Robot/Actuators.hpp"
#include "RobotIK/Robot/RobotSession.hpp"
#include "RobotIK/Task/Perception/WorldModel.hpp"
#include "RobotIK/Task/RobotContext.hpp"
#include "RobotIK/Task/Scheduler.hpp"
#include "RobotIK/Task/Skills/MotionSkills.hpp"
#include "RobotIK/Task/Skills/SkillNodes.hpp"

#include "Compages/World/World.hpp"

using robotik::Access;
using robotik::SkillReason;
using robotik::SkillState;
using robotik::Status;

/// Runs for a fixed number of ticks, records cancellations.
class CountdownSkill final: public robotik::Skill
{
public:

    explicit CountdownSkill(int p_ticks, Status p_end = Status::SUCCESS)
        : m_ticks(p_ticks), m_end(p_end)
    {
    }

    void reset() override
    {
        m_left = m_ticks;
        ++resets;
    }

    Status tick(robotik::RobotContext&, robotik::Seconds) override
    {
        ++ticks;
        return --m_left > 0 ? Status::RUNNING : m_end;
    }

    void cancel(robotik::RobotContext&) override
    {
        ++cancels;
    }

    int ticks = 0;
    int resets = 0;
    int cancels = 0;

private:

    int m_ticks;
    Status m_end;
    int m_left = 0;
};

/// Runs until cancelled, then needs @p p_ticks updates to be safe.
class SlowStopSkill final: public robotik::Skill
{
public:

    explicit SlowStopSkill(int p_ticks) : m_ticks(p_ticks) {}

    Status tick(robotik::RobotContext&, robotik::Seconds) override
    {
        return Status::RUNNING;
    }

    void cancel(robotik::RobotContext&) override
    {
        m_left = m_ticks;
    }

    Status stopping(robotik::RobotContext&, robotik::Seconds) override
    {
        ++stops;
        return --m_left > 0 ? Status::RUNNING : Status::SUCCESS;
    }

    int stops = 0;

private:

    int m_ticks;
    int m_left = 0;
};

/// Fixture that drives a scheduler with countdown skills.
class SchedulerTest: public ::testing::Test
{
protected:

    SchedulerTest()
        : robot(world, assetFile("robotik.robots.simple_revolute_robot")),
          context{ robot, beliefs, {}, robotik::Seconds(0.01) },
          skills(robot.resources())
    {
        robot.resources().add("arm");
        robot.resources().add("gripper");
        robot.resources().add("camera");
    }

    robotik::SkillDescription
    describe(std::string p_name,
             std::vector<robotik::ResourceRequirement> p_resources,
             robotik::Priority p_priority = 0)
    {
        robotik::SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = p_priority;
        return description;
    }

    robotik::ResourceRequirement arm() const
    {
        return robot.resources().require("arm");
    }

    void update(int p_times = 1)
    {
        for (int i = 0; i < p_times; ++i)
        {
            skills.update(context);
            context.time = context.time + context.dt;
        }
    }

    compages::world::World world;
    robotik::RobotSession robot;
    robotik::WorldModel beliefs;
    robotik::RobotContext context;
    robotik::SkillScheduler skills;
};

TEST_F(SchedulerTest, RunsUntilSuccessAndReleases)
{
    auto const id = skills.add<CountdownSkill>(describe("Move", { arm() }), 3);
    EXPECT_EQ(skills.find("Move"), id);
    EXPECT_EQ(skills.state(id), SkillState::Idle);
    skills.request(id);
    EXPECT_EQ(skills.state(id), SkillState::Waiting);
    update();
    EXPECT_EQ(skills.state(id), SkillState::Running);
    EXPECT_EQ(robot.resources().owner(arm().id), id);
    update(2);
    EXPECT_EQ(skills.state(id), SkillState::Succeeded);
    EXPECT_EQ(robot.resources().owner(arm().id), robotik::NO_OWNER);
    ASSERT_EQ(skills.trace().size(), 1U);
    EXPECT_EQ(skills.trace()[0].state, SkillState::Succeeded);
}

TEST_F(SchedulerTest, BusyResourceMakesWait)
{
    auto const first = skills.add<CountdownSkill>(describe("A", { arm() }), 3);
    auto const second = skills.add<CountdownSkill>(describe("B", { arm() }), 1);
    skills.request(first);
    skills.request(second);
    update();
    EXPECT_EQ(skills.state(first), SkillState::Running);
    EXPECT_EQ(skills.state(second), SkillState::Waiting);
    EXPECT_EQ(skills.reason(second), SkillReason::Busy);
    EXPECT_EQ(skills.blocker(second), arm().id);
    update(3);
    EXPECT_EQ(skills.state(first), SkillState::Succeeded);
    EXPECT_EQ(skills.state(second), SkillState::Succeeded);
    bool waited = false;
    bool succeeded = false;
    for (robotik::SkillRun const& run : skills.trace())
    {
        if (run.skill != second)
        {
            continue;
        }
        waited = waited || run.state == SkillState::Waiting;
        succeeded = succeeded || run.state == SkillState::Succeeded;
    }
    EXPECT_TRUE(waited);
    EXPECT_TRUE(succeeded);
}

TEST_F(SchedulerTest, SharedResourcesRunTogether)
{
    auto const camera = robot.resources().require("camera", Access::Shared);
    auto const a = skills.add<CountdownSkill>(describe("A", { camera }), 5);
    auto const b = skills.add<CountdownSkill>(describe("B", { camera }), 5);
    skills.request(a);
    skills.request(b);
    update();
    EXPECT_EQ(skills.state(a), SkillState::Running);
    EXPECT_EQ(skills.state(b), SkillState::Running);
}

TEST_F(SchedulerTest, HigherPriorityPreempts)
{
    auto const low =
        skills.add<CountdownSkill>(describe("Low", { arm() }, 10), 100);
    auto const stop =
        skills.add<CountdownSkill>(describe("Stop", { arm() }, 1000), 2);
    skills.request(low);
    update();
    skills.request(stop);
    update();
    // A preempted run is not over: it waits for the arm again.
    EXPECT_EQ(skills.state(low), SkillState::Waiting);
    EXPECT_EQ(skills.reason(low), SkillReason::Preempted);
    EXPECT_EQ(static_cast<CountdownSkill&>(skills.skill(low)).cancels, 1);
    EXPECT_EQ(skills.state(stop), SkillState::Running);
}

TEST_F(SchedulerTest, PreemptedSkillResumes)
{
    auto const low =
        skills.add<CountdownSkill>(describe("Low", { arm() }, 10), 100);
    auto const stop =
        skills.add<CountdownSkill>(describe("Stop", { arm() }, 1000), 2);
    robotik::RunId const run = skills.request(low);
    update();
    skills.request(stop);
    update(3); // preempted; Stop runs two ticks and gives the arm back
    EXPECT_EQ(skills.state(stop), SkillState::Succeeded);
    EXPECT_EQ(skills.state(run), SkillState::Running);
    EXPECT_EQ(static_cast<CountdownSkill&>(skills.skill(low)).resets, 2);
}

TEST_F(SchedulerTest, PreemptedSkillThatMustNotWaitIsCancelled)
{
    auto description = describe("Low", { arm() }, 10);
    description.wait = false;
    auto const low = skills.add<CountdownSkill>(std::move(description), 100);
    auto const stop =
        skills.add<CountdownSkill>(describe("Stop", { arm() }, 1000), 2);
    skills.request(low);
    update();
    skills.request(stop);
    update();
    EXPECT_EQ(skills.state(low), SkillState::Cancelled);
    EXPECT_EQ(skills.reason(low), SkillReason::Preempted);
}

TEST_F(SchedulerTest, NonCancellableIsNotPreempted)
{
    auto description = describe("Critical", { arm() }, 0);
    description.cancellable = false;
    auto const critical = skills.add<CountdownSkill>(std::move(description), 3);
    auto const urgent =
        skills.add<CountdownSkill>(describe("Urgent", { arm() }, 50), 1);
    skills.request(critical);
    update();
    skills.request(urgent);
    update();
    EXPECT_EQ(skills.state(critical), SkillState::Running);
    EXPECT_EQ(skills.state(urgent), SkillState::Waiting);
}

TEST_F(SchedulerTest, PreemptionIsAllOrNothing)
{
    // Wave holds the arm and may be preempted; Hold keeps the gripper and
    // may not. Grasp needs both: it must wait, and Wave must go on.
    auto const wave =
        skills.add<CountdownSkill>(describe("Wave", { arm() }, 0), 100);
    auto hold_description =
        describe("Hold", { robot.resources().require("gripper") }, 0);
    hold_description.cancellable = false;
    auto const hold =
        skills.add<CountdownSkill>(std::move(hold_description), 100);
    auto const grasp = skills.add<CountdownSkill>(
        describe("Grasp",
                 { arm(), robot.resources().require("gripper") },
                 50),
        1);
    skills.request(wave);
    skills.request(hold);
    update();
    skills.request(grasp);
    update();
    EXPECT_EQ(skills.state(wave), SkillState::Running);
    EXPECT_EQ(static_cast<CountdownSkill&>(skills.skill(wave)).cancels, 0);
    EXPECT_EQ(skills.state(hold), SkillState::Running);
    EXPECT_EQ(skills.state(grasp), SkillState::Waiting);
    EXPECT_EQ(skills.reason(grasp), SkillReason::Busy);
    EXPECT_EQ(skills.blocker(grasp), robot.resources().find("gripper"));
    EXPECT_EQ(skills.blocker(wave), robotik::NO_RESOURCE);
}

TEST_F(SchedulerTest, LeaseOutsideTheSchedulerIsNotPreempted)
{
    // A shared lease taken by the scenario logic is not a skill.
    robotik::ResourceLease logic = robot.resources().acquire(
        { robot.resources().require("camera", robotik::Access::Shared) });
    ASSERT_TRUE(logic);
    auto const urgent = skills.add<CountdownSkill>(
        describe("Urgent", { robot.resources().require("camera") }, 1000), 1);
    skills.request(urgent);
    update();
    EXPECT_EQ(skills.state(urgent), SkillState::Waiting);
    EXPECT_EQ(skills.reason(urgent), SkillReason::Busy);
}

TEST_F(SchedulerTest, LostResourceAbortsTheRun)
{
    auto const id =
        skills.add<CountdownSkill>(describe("Move", { arm() }), 100);
    skills.request(id);
    update();
    robot.resources().fail("arm");
    update();
    EXPECT_EQ(skills.state(id), SkillState::Failed);
    EXPECT_EQ(skills.reason(id), SkillReason::ResourceLost);
    EXPECT_EQ(static_cast<CountdownSkill&>(skills.skill(id)).cancels, 1);

    skills.request(id);
    update();
    EXPECT_EQ(skills.state(id), SkillState::Failed);
    EXPECT_EQ(skills.reason(id), SkillReason::Unavailable);
}

TEST_F(SchedulerTest, PreconditionWaitsOrFails)
{
    bool ready = false;
    auto waiting = describe("Waiting", {});
    waiting.preconditions.push_back(
        { "ready",
          [&ready](robotik::RobotContext const&) noexcept { return ready; } });
    auto failing = waiting;
    failing.name = "Failing";
    failing.wait = false;
    auto const a = skills.add<CountdownSkill>(std::move(waiting), 1);
    auto const b = skills.add<CountdownSkill>(std::move(failing), 1);
    skills.request(a);
    skills.request(b);
    update();
    EXPECT_EQ(skills.state(a), SkillState::Waiting);
    ASSERT_NE(skills.precondition(a), nullptr);
    EXPECT_EQ(skills.precondition(a)->text, "ready");
    EXPECT_EQ(skills.state(b), SkillState::Failed);
    ready = true;
    update();
    EXPECT_EQ(skills.state(a), SkillState::Succeeded);
}

TEST_F(SchedulerTest, CancelIsCooperative)
{
    auto const id =
        skills.add<CountdownSkill>(describe("Move", { arm() }), 100);
    robotik::RunId const run = skills.request(id);
    update();
    skills.cancel(run);
    EXPECT_EQ(skills.state(id), SkillState::Running);
    update();
    EXPECT_EQ(skills.state(id), SkillState::Cancelled);
    EXPECT_EQ(skills.reason(id), SkillReason::Cancelled);
    EXPECT_EQ(robot.resources().owner(arm().id), robotik::NO_OWNER);
}

TEST_F(SchedulerTest, ResourcesHeldWhileStopping)
{
    auto const slow =
        skills.add<SlowStopSkill>(describe("Slow", { arm() }, 0), 3);
    auto const next =
        skills.add<CountdownSkill>(describe("Next", { arm() }, 50), 100);
    skills.request(slow);
    update();
    skills.request(next);
    update(); // preempted: stopping 1/3, the arm stays held
    EXPECT_EQ(skills.state(slow), SkillState::Stopping);
    EXPECT_EQ(robot.resources().owner(arm().id), slow);
    EXPECT_EQ(skills.state(next), SkillState::Waiting);
    EXPECT_EQ(skills.reason(next), SkillReason::Preempting);
    update(2); // stopping 2/3, then 3/3: safe, the arm goes to Next
    // Slow waits again for the arm, now held by Next.
    EXPECT_EQ(skills.state(slow), SkillState::Waiting);
    EXPECT_EQ(skills.reason(slow), SkillReason::Busy);
    EXPECT_EQ(skills.state(next), SkillState::Running);
    EXPECT_EQ(robot.resources().owner(arm().id), next);
}

TEST_F(SchedulerTest, StopTimeoutForcesTheEnd)
{
    auto description = describe("Slow", { arm() }, 0);
    description.stop_timeout = robotik::Seconds(0.025);
    auto const slow = skills.add<SlowStopSkill>(std::move(description), 1000);
    robotik::RunId const run = skills.request(slow);
    update();
    skills.cancel(run);
    update(3); // 0, 0.01 and 0.02 s of stopping
    EXPECT_EQ(skills.state(slow), SkillState::Stopping);
    update(); // 0.03 s > 0.025 s
    EXPECT_EQ(skills.state(slow), SkillState::Cancelled);
    EXPECT_TRUE(skills.trace().back().forced);
    EXPECT_EQ(robot.resources().owner(arm().id), robotik::NO_OWNER);
}

TEST_F(SchedulerTest, UrgentSkillDoesNotWaitForStopping)
{
    auto const slow =
        skills.add<SlowStopSkill>(describe("Slow", { arm() }, 0), 100);
    auto stop_description = describe("Stop", { arm() }, 1000);
    stop_description.urgent = true;
    auto const stop =
        skills.add<CountdownSkill>(std::move(stop_description), 100);
    skills.request(slow);
    update();
    skills.request(stop);
    update();
    EXPECT_EQ(skills.state(slow), SkillState::Waiting);
    EXPECT_EQ(skills.reason(slow), SkillReason::Preempted);
    EXPECT_EQ(static_cast<SlowStopSkill&>(skills.skill(slow)).stops, 0);
    EXPECT_EQ(skills.state(stop), SkillState::Running);
}

TEST_F(SchedulerTest, TwoRequestsAreTwoRuns)
{
    auto const id =
        skills.add<CountdownSkill>(describe("Move", { arm() }), 2);
    robotik::RunId const first = skills.request(id);
    robotik::RunId const second = skills.request(id);
    EXPECT_NE(first, second);
    EXPECT_EQ(skills.state(second), SkillState::Waiting);
    EXPECT_EQ(skills.reason(second), SkillReason::Queued);
    update(2); // admitted and ticked twice: the first one succeeds
    EXPECT_EQ(skills.state(first), SkillState::Succeeded);
    EXPECT_EQ(skills.state(second), SkillState::Waiting);
    EXPECT_EQ(skills.reason(second), SkillReason::None);
    update(2);
    EXPECT_EQ(skills.state(second), SkillState::Succeeded);
    EXPECT_EQ(skills.state(first), SkillState::Succeeded);
}

TEST_F(SchedulerTest, CancellingOneTicketKeepsTheOther)
{
    auto const id =
        skills.add<CountdownSkill>(describe("Move", { arm() }), 100);
    robotik::RunId const first = skills.request(id);
    robotik::RunId const second = skills.request(id);
    update();
    // The second caller gives up: the run of the first one goes on.
    skills.cancel(second);
    EXPECT_EQ(skills.state(second), SkillState::Cancelled);
    update();
    EXPECT_EQ(skills.state(first), SkillState::Running);
    // The first caller gives up: only its run ends.
    robotik::RunId const third = skills.request(id);
    skills.cancel(first);
    EXPECT_EQ(skills.state(third), SkillState::Waiting);
    EXPECT_EQ(skills.reason(third), SkillReason::Queued);
    update(); // the cancel frees the arm: the third run starts at once
    EXPECT_EQ(skills.state(first), SkillState::Cancelled);
    EXPECT_EQ(skills.state(third), SkillState::Running);
}

TEST_F(SchedulerTest, OldTicketIsUnknownAfterReset)
{
    auto const id =
        skills.add<CountdownSkill>(describe("Move", { arm() }), 1);
    robotik::RunId const old = skills.request(id);
    update(2);
    skills.reset();
    EXPECT_EQ(skills.state(old), SkillState::Idle);
    robotik::RunId const fresh = skills.request(id);
    EXPECT_NE(old, fresh);
    EXPECT_EQ(skills.state(old), SkillState::Idle);
}

TEST_F(SchedulerTest, BehaviorTreeRequestsSkills)
{
    skills.add<CountdownSkill>(describe("First", { arm() }), 2);
    skills.add<CountdownSkill>(
        describe("Second", { arm() }), 2, Status::FAILURE);
    bt::NodeFactory factory;
    robotik::registerSkills(factory, skills);
    auto built = bt::Builder::fromText(factory, R"(
BehaviorTree:
  Sequence:
    children:
      - Action:
          name: First
      - Action:
          name: Second
)");
    ASSERT_TRUE(built) << built.getError();
    bt::Tree::Ptr tree = std::move(built.getValue());
    bt::Status status = bt::Status::RUNNING;
    for (int i = 0; i < 20 && status == bt::Status::RUNNING; ++i)
    {
        status = tree->tick();
        update();
    }
    EXPECT_EQ(status, bt::Status::FAILURE);
    EXPECT_EQ(skills.state(skills.find("First")), SkillState::Succeeded);
    EXPECT_EQ(skills.state(skills.find("Second")), SkillState::Failed);
}

TEST_F(SchedulerTest, TreeNodeWritesWhyItsRunEnded)
{
    skills.add<CountdownSkill>(
        describe("Grasp", { arm() }), 2, Status::FAILURE);
    bt::NodeFactory factory;
    robotik::registerSkills(factory, skills);
    auto board = std::make_shared<bt::Blackboard>();
    auto built = bt::Builder::fromText(factory, R"(
BehaviorTree:
  Action:
    name: Grasp
)",
                                       board);
    ASSERT_TRUE(built) << built.getError();
    bt::Tree::Ptr tree = std::move(built.getValue());
    bt::Status status = bt::Status::RUNNING;
    for (int i = 0; i < 20 && status == bt::Status::RUNNING; ++i)
    {
        status = tree->tick();
        update();
    }
    EXPECT_EQ(status, bt::Status::FAILURE);
    EXPECT_EQ(board->get<std::string>("Grasp.state"), "FAILED");
    EXPECT_EQ(board->get<std::string>("Grasp.reason"), "failed");
}

/// Moves joint 0 at each tick, to check that writes follow the leases.
class PushJointSkill final: public robotik::Skill
{
public:

    Status tick(robotik::RobotContext& p_context, robotik::Seconds) override
    {
        p_context.robot.joints().moveTo(0, 0.3);
        return Status::RUNNING;
    }
};

/// A robot whose joints are guarded by the joint group "body".
class GuardedJoints: public ::testing::Test
{
protected:

    GuardedJoints()
        : robot(world, assetFile("robotik.robots.simple_revolute_robot")),
          context{ robot, beliefs, {}, robotik::Seconds(0.01) },
          skills(robot.resources())
    {
        robot.actuators().add<robotik::JointGroup>("body");
    }

    robotik::ResourceRequirement body() const
    {
        return robot.resources().require("body");
    }

    compages::world::World world;
    robotik::RobotSession robot;
    robotik::WorldModel beliefs;
    robotik::RobotContext context;
    robotik::SkillScheduler skills;
};

TEST_F(GuardedJoints, FreeJointIsWritable)
{
    EXPECT_NO_THROW(robot.joints().moveTo(0, 0.2));
}

TEST_F(GuardedJoints, SkillWritesUnderItsLease)
{
    robotik::SkillDescription description;
    description.name = "Push";
    description.resources = { body() };
    auto const push = skills.add<PushJointSkill>(std::move(description));
    skills.request(push);
    EXPECT_NO_THROW(skills.update(context));
    EXPECT_EQ(skills.state(push), SkillState::Running);
    EXPECT_DOUBLE_EQ(robot.joints().target(0), 0.3);
}

TEST_F(GuardedJoints, WriteToHeldJointThrows)
{
    // The scenario logic writes after the scheduler: with a skill holding
    // the joints, the last writer would win. It is refused.
    robotik::SkillDescription description;
    description.name = "Push";
    description.resources = { body() };
    auto const push = skills.add<PushJointSkill>(std::move(description));
    skills.request(push);
    skills.update(context);
    EXPECT_THROW(robot.joints().moveTo(0, 0.2), std::logic_error);
    EXPECT_THROW(robot.joints().hold(), std::logic_error);
}

TEST_F(GuardedJoints, LogicLeaseBlocksSkill)
{
    robotik::ResourceLease logic = robot.resources().acquire({ body() });
    ASSERT_TRUE(logic);
    EXPECT_NO_THROW(robot.joints().moveTo(0, 0.2));
    robotik::SkillDescription description;
    description.name = "Push";
    description.resources = { body() };
    auto const push = skills.add<PushJointSkill>(std::move(description));
    skills.request(push);
    skills.update(context);
    EXPECT_EQ(skills.state(push), SkillState::Waiting);
    EXPECT_EQ(skills.reason(push), SkillReason::Busy);
}

TEST_F(GuardedJoints, SafeStateIsWrittenWhoeverHoldsTheJoints)
{
    robotik::SkillDescription description;
    description.name = "Push";
    description.resources = { body() };
    auto const push = skills.add<PushJointSkill>(std::move(description));
    skills.request(push);
    skills.update(context);
    // The group fails: the robot puts it in its safe state at the next step.
    robot.resources().fail("body");
    EXPECT_NO_THROW(robot.step(robotik::Seconds(0.01)));
}

TEST_F(SchedulerTest, MoveJointWaitsForTheJointToSlowDown)
{
    std::string const joint = robot.joints().name(0);
    robotik::MoveJointSkill settle(joint, 0.5);
    robotik::MoveJointSkill pass(
        joint, 0.5, 1e-2, std::numeric_limits<double>::infinity());
    settle.reset();
    pass.reset();
    robot.measureJoint(0, 0.5, 1.0); // on the target, still at 1 rad/s
    EXPECT_EQ(settle.tick(context, context.dt), Status::RUNNING);
    EXPECT_EQ(pass.tick(context, context.dt), Status::SUCCESS);
    robot.measureJoint(0, 0.5, 0.01);
    EXPECT_EQ(settle.tick(context, context.dt), Status::SUCCESS);
}
