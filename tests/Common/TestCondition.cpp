// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Task/Condition.hpp"

#include <stdexcept>

/// A vocabulary with two metrics and a query on two known boxes.
static robotik::ConditionVocabulary vocabulary(int& p_calls)
{
    robotik::ConditionVocabulary words;
    words.metrics = { "time", "laps" };
    words.queries.push_back(
        { "inside", 2U, 2U,
          [&p_calls](std::vector<std::string> const& p_args) -> std::optional<double>
          {
              ++p_calls;
              for (std::string const& arg : p_args)
              {
                  if (arg != "cube" && arg != "box")
                  {
                      throw std::invalid_argument("unknown entity '" + arg + "'");
                  }
              }
              return 1.0;
          } });
    return words;
}

/// Metrics of one step.
static robotik::Metrics metrics(double p_time, double p_laps)
{
    robotik::Metrics table;
    table.set("time", p_time);
    table.set("laps", p_laps);
    return table;
}

TEST(Condition, ComparesAndCombines)
{
    int calls = 0;
    robotik::ConditionVocabulary const words = vocabulary(calls);
    robotik::Condition const c =
        robotik::compileCondition("(and (inside cube box) (< time 30) (not (= laps 0)))",
                                  words).take();
    EXPECT_EQ(calls, 1) << "the query is checked once at compile time";
    EXPECT_TRUE(robotik::evaluate(c, metrics(10.0, 1.0)).holds);
    robotik::Evaluation const late = robotik::evaluate(c, metrics(40.0, 1.0));
    EXPECT_TRUE(late.known);
    EXPECT_FALSE(late.holds);
    EXPECT_EQ(late.detail, "time = 40") << "and shows the first child that fails";
    EXPECT_EQ(late.expected, "< 30");
    EXPECT_FALSE(robotik::evaluate(c, metrics(10.0, 0.0)).holds);

    robotik::Condition const either =
        robotik::compileCondition("(or (> laps 2) (< time 5))", words).take();
    EXPECT_TRUE(robotik::evaluate(either, metrics(1.0, 0.0)).holds);
    EXPECT_FALSE(robotik::evaluate(either, metrics(9.0, 0.0)).holds);

    robotik::Condition const two =
        robotik::compileCondition("(>= laps time)", words).take();
    robotik::Evaluation const e = robotik::evaluate(two, metrics(2.0, 3.0));
    EXPECT_TRUE(e.holds);
    EXPECT_DOUBLE_EQ(e.value, 3.0);
    EXPECT_EQ(e.expected, ">= time (2)");
}

TEST(Condition, BareValueHoldsWhenNotZero)
{
    int calls = 0;
    robotik::Condition const c =
        robotik::compileCondition("laps", vocabulary(calls)).take();
    EXPECT_FALSE(robotik::evaluate(c, metrics(0.0, 0.0)).holds);
    robotik::Evaluation const e = robotik::evaluate(c, metrics(0.0, 2.0));
    EXPECT_TRUE(e.holds);
    EXPECT_EQ(e.detail, "laps = 2");
    EXPECT_EQ(e.expected, "!= 0");
}

TEST(Condition, RefusesWhatItCannotMeasure)
{
    int calls = 0;
    robotik::ConditionVocabulary const words = vocabulary(calls);
    auto error = [&](std::string_view p_text)
    { return robotik::compileCondition(p_text, words).error(); };
    EXPECT_EQ(error("(< tme 30)"),
              "condition '(< tme 30)': column 4: unknown value 'tme' (did you "
              "mean 'time'?)");
    EXPECT_EQ(error("(insid cube box)"),
              "condition '(insid cube box)': column 2: unknown operator or query "
              "'insid' (did you mean 'inside'?)");
    EXPECT_EQ(error("(inside cube)"),
              "condition '(inside cube)': column 1: (inside ...) takes 2 "
              "argument(s), not 1");
    EXPECT_EQ(error("(inside cube bin)"),
              "condition '(inside cube bin)': column 1: unknown entity 'bin'");
    EXPECT_EQ(error("(< time)"),
              "condition '(< time)': column 1: (< a b) compares two values");
    EXPECT_EQ(error("inside"),
              "condition 'inside': column 1: 'inside' is a query: write (inside "
              "...) with its arguments");
    EXPECT_NE(error("time < 30").find("one expression only"), std::string::npos);
}

TEST(Condition, MissingMetricIsUnknownNotZero)
{
    int calls = 0;
    robotik::Condition const c =
        robotik::compileCondition("(< laps 1)", vocabulary(calls)).take();
    robotik::Metrics only_time;
    only_time.set("time", 1.0);
    robotik::Evaluation const e = robotik::evaluate(c, only_time);
    EXPECT_FALSE(e.known);
    EXPECT_FALSE(e.holds);
    EXPECT_EQ(e.detail, "metric 'laps' is not published at this step");
}
