// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Core/Common/Path.hpp"

using namespace robotik;

class PathTests: public ::testing::Test
{
protected:

    void SetUp() override
    {
        // No setup needed for Path tests
    }

    void TearDown() override
    {
        // No cleanup needed for Path tests
    }
};

TEST_F(PathTests, EmptyConstructor)
{
    Path path;

    EXPECT_EQ(0, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:");
}

TEST_F(PathTests, SplitConstructor)
{
    Path path("/a/b:c/d");

    EXPECT_EQ(2, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:/a/b:c/d:");
    path.clear();
    EXPECT_EQ(0, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:");
    path.add("g/g");
    EXPECT_EQ(1, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:g/g:");
    path.reset("a/b");
    EXPECT_EQ(1, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:a/b:");
}

TEST_F(PathTests, SplitDir)
{
    Path path("/a//b\\d/:e\\d:");
    EXPECT_TRUE(path.toString() == ".:/a//b\\d:e\\d:");
    EXPECT_EQ(2, path.paths().size());
    path.remove("incorrect/path");
    EXPECT_EQ(2, path.paths().size());
    path.remove("/a//b\\d");
    EXPECT_EQ(2, path.paths().size());
    path.remove("/a//b\\d/");
    EXPECT_EQ(1, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:e\\d:");
    path.remove("e\\d/");
    EXPECT_EQ(0, path.paths().size());
    path.remove("");
    EXPECT_EQ(0, path.paths().size());
    path.remove("incorrect/path");
    EXPECT_EQ(0, path.paths().size());
    path.add("g/g");
    EXPECT_EQ(1, path.paths().size());
    EXPECT_TRUE(path.toString() == ".:g/g:");
}

TEST_F(PathTests, FindAndExpand)
{
    Path path("/bin:/usr/bin:/usr/local/bin");
    EXPECT_TRUE(path.expand("ls") != "ls");
    auto [path1, found1] = path.find("ls");

    EXPECT_TRUE(found1);
    EXPECT_TRUE(path1 != "ls");
    EXPECT_TRUE(path1 != "");

    auto [path3, found3] = path.find(path1);
    EXPECT_TRUE(found3);
    EXPECT_TRUE(path3 == path1);

    path.clear();
    EXPECT_TRUE(path.expand("ls") == "ls");
    auto [path2, found2] = path.find("ls");
    EXPECT_FALSE(found2);
    EXPECT_TRUE(path2 == "");
}
