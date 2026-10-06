// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Skills/SkillNodes.hpp"
#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Robot/Robot.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Scheduler.hpp"

#include "Compages/World/World.hpp"

using robotik::Access;
using robotik::SkillReason;
using robotik::SkillState;
using robotik::Status;

namespace
{

//! @brief Runs for a fixed number of ticks, records cancellations.
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

    Status tick(robotik::RobotContext&, Seconds) override
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

class SchedulerTest: public ::testing::Test
{
protected:

    SchedulerTest()
        : robot(world, dataFile("simple_revolute_robot.urdf")),
          context{ robot, beliefs, {}, Seconds(0.01) },
          skills(robot.resources())
    {
        robot.resources().add("arm");
        robot.resources().add("gripper");
        robot.resources().add("camera");
    }

    robotik::SkillDescription describe(std::string p_name,
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

} // namespace

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
    ASSERT_EQ(skills.trace().size(), 1u);
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
    auto const low = skills.add<CountdownSkill>(describe("Low", { arm() }, 10), 100);
    auto const stop = skills.add<CountdownSkill>(describe("Stop", { arm() }, 1000), 2);
    skills.request(low);
    update();
    skills.request(stop);
    update();
    EXPECT_EQ(skills.state(low), SkillState::Cancelled);
    EXPECT_EQ(skills.reason(low), SkillReason::Preempted);
    EXPECT_EQ(static_cast<CountdownSkill&>(skills.skill(low)).cancels, 1);
    EXPECT_EQ(skills.state(stop), SkillState::Running);
}

TEST_F(SchedulerTest, NonCancellableIsNotPreempted)
{
    auto description = describe("Critical", { arm() }, 0);
    description.cancellable = false;
    auto const critical = skills.add<CountdownSkill>(std::move(description), 3);
    auto const urgent = skills.add<CountdownSkill>(describe("Urgent", { arm() }, 50), 1);
    skills.request(critical);
    update();
    skills.request(urgent);
    update();
    EXPECT_EQ(skills.state(critical), SkillState::Running);
    EXPECT_EQ(skills.state(urgent), SkillState::Waiting);
}

TEST_F(SchedulerTest, LostResourceAbortsTheRun)
{
    auto const id = skills.add<CountdownSkill>(describe("Move", { arm() }), 100);
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
        { "ready", [&ready](robotik::RobotContext const&) noexcept { return ready; } });
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
    auto const id = skills.add<CountdownSkill>(describe("Move", { arm() }), 100);
    skills.request(id);
    update();
    skills.cancel(id);
    EXPECT_EQ(skills.state(id), SkillState::Running);
    update();
    EXPECT_EQ(skills.state(id), SkillState::Cancelled);
    EXPECT_EQ(skills.reason(id), SkillReason::Cancelled);
    EXPECT_EQ(robot.resources().owner(arm().id), robotik::NO_OWNER);
}

TEST_F(SchedulerTest, BehaviorTreeRequestsSkills)
{
    skills.add<CountdownSkill>(describe("First", { arm() }), 2);
    skills.add<CountdownSkill>(describe("Second", { arm() }), 2, Status::FAILURE);
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
