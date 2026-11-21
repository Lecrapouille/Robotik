// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

// A mission has three sides per step: act (robot and beliefs), simulate (the
// world) and observe (the truth, read only). The types keep them apart.

#include "main.hpp"

#include "Robotik/Task/OpenRoboticCell.hpp"

#include <string>
#include <type_traits>
#include <utility>

// A const cell commands nothing: what observe() gets is read only.
static_assert(
    std::is_const_v<std::remove_reference_t<
        decltype(std::declval<robotik::RoboticCell const&>().skills())>>);
static_assert(
    std::is_const_v<std::remove_reference_t<
        decltype(std::declval<robotik::RoboticCell const&>().robot())>>);
static_assert(
    std::is_const_v<std::remove_pointer_t<
        decltype(std::declval<robotik::RoboticCell const&>().camera())>>);
static_assert(std::is_same_v<
              decltype(std::declval<robotik::RoboticCell const&>().robots()),
              std::vector<robotik::RobotSession const*> const&>);

/// Writes the order of its calls.
class SidesMission final: public robotik::Mission
{
public:

    void act(robotik::RobotContext& p_context, robotik::Seconds) override
    {
        calls += "a";
        last_act = p_context.time;
    }

    void simulate(robotik::RoboticCell&, robotik::Seconds) override
    {
        calls += "s";
    }

    void observe(robotik::RoboticCell const& p_cell, robotik::Seconds) override
    {
        calls += "o";
        last_observe = p_cell.time();
    }

    [[nodiscard]] robotik::Status
    status(robotik::RobotContext const& p_context) const override
    {
        return p_context.time >= robotik::Seconds(0.03) ? robotik::Status::SUCCESS
                                                       : robotik::Status::RUNNING;
    }

    std::string calls;
    robotik::Seconds last_act{ -1.0 };
    robotik::Seconds last_observe{ -1.0 };
};

TEST(MissionSides, ActThenSimulateThenObserveAtTheSameTime)
{
    robotik::EntitySpec arm;
    arm.id = "arm";
    arm.type = robotik::EntitySpec::Type::Robot;
    arm.model = repoFile("data/packs/robotik.robots/robot_6axis.urdf");
    robotik::RoboticCellDescription description;
    description.world.entities = { arm };

    SidesMission mission;
    auto opened = robotik::openRoboticCell(std::move(description), &mission);
    ASSERT_TRUE(opened) << opened.error();
    robotik::RoboticCell& cell = *opened.value();

    EXPECT_EQ(cell.run(robotik::Seconds(1.0)), robotik::Status::SUCCESS);
    EXPECT_EQ(mission.calls, "asoasoaso") << "one act, simulate, observe per step";
    EXPECT_EQ(mission.last_act, mission.last_observe)
        << "the robot side sees the time after the physics step";
}
