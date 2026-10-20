// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Common/Signal.hpp"

#include <string>
#include <vector>

// ----------------------------------------------------------------------------
TEST(Signal, CallsSlotsInConnectionOrder)
{
    robotik::Signal<int> signal;
    std::vector<std::string> calls;
    signal.connect([&](int p_value) { calls.push_back("a" + std::to_string(p_value)); });
    signal.connect([&](int p_value) { calls.push_back("b" + std::to_string(p_value)); });
    signal(1);
    signal.emit(2);
    EXPECT_EQ(calls, (std::vector<std::string>{ "a1", "b1", "a2", "b2" }));
    EXPECT_EQ(signal.size(), 2U);
}

// ----------------------------------------------------------------------------
TEST(Signal, DisconnectDuringEmission)
{
    robotik::Signal<> signal;
    int first = 0;
    int second = 0;
    robotik::Signal<>::Id second_id = 0;
    signal.connect(
        [&]
        {
            ++first;
            signal.disconnect(second_id);
        });
    second_id = signal.connect([&] { ++second; });
    signal();
    signal();
    EXPECT_EQ(first, 2);
    EXPECT_EQ(second, 0); // disconnected before its turn
    EXPECT_EQ(signal.size(), 1U);
}

// ----------------------------------------------------------------------------
TEST(Signal, ConnectDuringEmissionWaitsForTheNextOne)
{
    robotik::Signal<> signal;
    int late = 0;
    signal.connect([&] { signal.connect([&] { ++late; }); });
    signal();
    EXPECT_EQ(late, 0);
    signal();
    EXPECT_EQ(late, 1);
}

// ----------------------------------------------------------------------------
TEST(Signal, ScopedConnectionEndsWithItsScope)
{
    robotik::Signal<int> signal;
    int last = 0;
    {
        robotik::ScopedConnection<int> link(signal, [&](int p_value) { last = p_value; });
        signal(3);
        EXPECT_EQ(signal.size(), 1U);
    }
    signal(4);
    EXPECT_EQ(last, 3);
    EXPECT_EQ(signal.size(), 0U);
}
