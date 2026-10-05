// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Runtime/Faults.hpp"
#include "Robotik/Runtime/Resources.hpp"

using robotik::Access;
using robotik::ResourceManager;

TEST(ResourceManager, ExclusiveLeaseBlocksOthers)
{
    ResourceManager resources;
    auto const arm = resources.add("arm");
    EXPECT_EQ(resources.add("arm"), arm);
    EXPECT_EQ(resources.find("arm"), arm);

    robotik::ResourceLease lease = resources.acquire({ resources.require("arm") }, 1u);
    ASSERT_TRUE(lease);
    EXPECT_EQ(resources.owner(arm), 1u);

    robotik::Conflict const conflict =
        resources.conflict(std::vector{ resources.require("arm") });
    ASSERT_TRUE(conflict);
    EXPECT_EQ(conflict.resource, arm);
    EXPECT_EQ(conflict.owner, 1u);
    EXPECT_FALSE(resources.acquire({ resources.require("arm") }, 2u));

    lease.release();
    EXPECT_EQ(resources.owner(arm), robotik::NO_OWNER);
    EXPECT_TRUE(resources.acquire({ resources.require("arm") }, 2u));
}

TEST(ResourceManager, SharedLeasesCoexist)
{
    ResourceManager resources;
    auto const camera = resources.add("camera");
    auto a = resources.acquire({ resources.require("camera", Access::Shared) }, 1u);
    auto b = resources.acquire({ resources.require("camera", Access::Shared) }, 2u);
    EXPECT_TRUE(a);
    EXPECT_TRUE(b);
    EXPECT_EQ(resources.users(camera), 2u);
    EXPECT_FALSE(resources.acquire({ resources.require("camera") }, 3u));
    {
        robotik::ResourceLease moved = std::move(a);
        EXPECT_FALSE(a);
    }
    EXPECT_EQ(resources.users(camera), 1u);
}

TEST(ResourceManager, FailedResourceIsUnavailable)
{
    ResourceManager resources;
    resources.add("gripper");
    resources.fail("gripper");
    EXPECT_FALSE(resources.available("gripper"));
    robotik::Conflict const conflict =
        resources.conflict(std::vector{ resources.require("gripper") });
    EXPECT_TRUE(conflict.unavailable);
    EXPECT_FALSE(resources.acquire({ resources.require("gripper") }));
    resources.restoreAll();
    EXPECT_TRUE(resources.available("gripper"));
    EXPECT_THROW((void)resources.require("unknown"), std::invalid_argument);
}

TEST(FaultInjector, ScheduledAndRandomFaultsAreReplayable)
{
    auto run = [](robotik::Seed p_seed)
    {
        ResourceManager resources;
        resources.add("camera");
        resources.add("gripper");
        robotik::FaultInjector faults(
            { { Seconds(0.5), "camera", true }, { Seconds(1.0), "camera", false } },
            { { "gripper", 0.5 } });
        faults.reset(p_seed);
        std::vector<int> history;
        Seconds const dt(0.01);
        for (int i = 1; i <= 300; ++i)
        {
            faults.update(resources, Seconds(i * 0.01), dt);
            history.push_back((resources.available("camera") ? 1 : 0) +
                              (resources.available("gripper") ? 2 : 0));
        }
        return history;
    };
    std::vector<int> const first = run(robotik::Seed{ 3 });
    EXPECT_EQ(first, run(robotik::Seed{ 3 }));
    EXPECT_EQ(first[10] & 1, 1);
    EXPECT_EQ(first[60] & 1, 0);
    EXPECT_EQ(first[150] & 1, 1);
    EXPECT_EQ(first.back() & 2, 0) << "rate 0.5/s over 3 s should fail";
}
