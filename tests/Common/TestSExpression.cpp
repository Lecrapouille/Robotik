// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "RobotIK/Common/SExpression.hpp"

TEST(SExpression, ReadsAtomsTextsAndLists)
{
    robotik::SExpression const e =
        robotik::parseSExpression("(and (inside cube \"big box\") ; why\n 3)").take();
    ASSERT_EQ(e.kind, robotik::SExpression::Kind::List);
    ASSERT_EQ(e.items.size(), 3U);
    EXPECT_EQ(e.items[0].text, "and");
    EXPECT_EQ(e.items[1].items[2].kind, robotik::SExpression::Kind::Text);
    EXPECT_EQ(e.items[1].items[2].text, "big box");
    EXPECT_EQ(e.items[2].text, "3");
    EXPECT_EQ(robotik::parseSExpression("laps").take().text, "laps");
}

TEST(SExpression, NamesTheColumnOfAnError)
{
    EXPECT_EQ(robotik::parseSExpression("(< time 3").error(),
              "column 1: this '(' is not closed");
    EXPECT_EQ(robotik::parseSExpression("(< time 3))").error(),
              "column 11: one ')' too many");
    EXPECT_EQ(robotik::parseSExpression("a b").error(),
              "column 3: one expression only (put several in (and ...))");
    EXPECT_EQ(robotik::parseSExpression("()").error(), "column 1: empty list ()");
    EXPECT_EQ(robotik::parseSExpression("  ").error(), "column 3: empty condition");
}

TEST(SExpression, ClosestWordSuggestsATypo)
{
    EXPECT_EQ(robotik::closestWord("phse", { "name", "phase", "at" }), "phase");
    EXPECT_EQ(robotik::closestWord("zzzzzz", { "name", "phase" }), "");
}
