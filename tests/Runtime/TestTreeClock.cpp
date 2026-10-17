// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "BlackThorn/BlackThorn.hpp"

#include <chrono>

// The timed nodes of a tree read the clock given to the tree: in a
// simulation, the simulated time, whatever the wall clock does.
TEST(TreeClock, WaitFollowsTheSimulatedTime)
{
    double simulated = 0.0;
    bt::NodeFactory factory;
    auto built = bt::Builder::fromText(factory,
                                       "BehaviorTree:\n"
                                       "  Sequence:\n"
                                       "    children:\n"
                                       "      - Wait:\n"
                                       "          milliseconds: 500\n",
                                       std::make_shared<bt::Blackboard>());
    ASSERT_TRUE(built) << built.getError();
    bt::Tree::Ptr tree = std::move(built.getValue());
    tree->clock(
        [&simulated]
        {
            return std::chrono::steady_clock::time_point(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(simulated)));
        });

    EXPECT_EQ(tree->tick(), bt::Status::RUNNING);
    simulated = 0.49;
    EXPECT_EQ(tree->tick(), bt::Status::RUNNING);
    simulated = 0.50;
    EXPECT_EQ(tree->tick(), bt::Status::SUCCESS);
}

TEST(TreeClock, TimeoutDoesNotExpireWhileTheSimulationIsPaused)
{
    double simulated = 1.0;
    bt::NodeFactory factory;
    auto built = bt::Builder::fromText(factory,
                                       "BehaviorTree:\n"
                                       "  Timeout:\n"
                                       "    milliseconds: 100\n"
                                       "    child:\n"
                                       "      - Wait:\n"
                                       "          milliseconds: 1000\n",
                                       std::make_shared<bt::Blackboard>());
    ASSERT_TRUE(built) << built.getError();
    bt::Tree::Ptr tree = std::move(built.getValue());
    tree->clock(
        [&simulated]
        {
            return std::chrono::steady_clock::time_point(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(simulated)));
        });
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_EQ(tree->tick(), bt::Status::RUNNING);
    }
    simulated = 1.1;
    EXPECT_EQ(tree->tick(), bt::Status::FAILURE);
}
