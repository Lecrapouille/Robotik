// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Assets/AssetResolver.hpp"

namespace
{

std::filesystem::path fixture(char const* p_name)
{
    return repoFile(std::filesystem::path("tests/Assets/fixtures") / p_name);
}

} // namespace

TEST(Version, ParsesShortFormsAndRejectsGarbage)
{
    EXPECT_EQ(robotik::Version::parse("1")->toString(), "1.0.0");
    EXPECT_EQ(robotik::Version::parse("1.2")->toString(), "1.2.0");
    EXPECT_EQ(robotik::Version::parse("1.2.3")->toString(), "1.2.3");
    EXPECT_FALSE(robotik::Version::parse(""));
    EXPECT_FALSE(robotik::Version::parse("1."));
    EXPECT_FALSE(robotik::Version::parse("1.2.3.4"));
    EXPECT_FALSE(robotik::Version::parse("v1"));
}

TEST(Version, SameMajorAndNotOlderSatisfies)
{
    robotik::Version const required = *robotik::Version::parse("1.2");
    EXPECT_TRUE(robotik::Version::parse("1.2.0")->satisfies(required));
    EXPECT_TRUE(robotik::Version::parse("1.9.1")->satisfies(required));
    EXPECT_FALSE(robotik::Version::parse("1.1.9")->satisfies(required));
    EXPECT_FALSE(robotik::Version::parse("2.0.0")->satisfies(required));
}

TEST(AssetResolver, LongestPackIdWins)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("roots1"));
    ASSERT_EQ(assets.packs().size(), 2u);
    EXPECT_EQ(assets.model("robotik.robots.arm").filename(), "arm.urdf");
    EXPECT_EQ(assets.model("robotik.robots.abb.irb").filename(), "irb.urdf");
    EXPECT_THROW((void)assets.model("robotik.robots.abb"), std::runtime_error);
    EXPECT_THROW((void)assets.model("robotik.nothing.arm"), std::runtime_error);
}

TEST(AssetResolver, PathsStayRelativeToTheReferencingFile)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("roots1"));
    std::filesystem::path const base("/tmp/scenarios");
    EXPECT_EQ(assets.resolve("../robots/x.urdf", base), "/tmp/robots/x.urdf");
    EXPECT_EQ(assets.resolve("local.urdf", base), "/tmp/scenarios/local.urdf");
    EXPECT_EQ(assets.resolve("robotik.robots.arm", base).filename(), "arm.urdf");
    EXPECT_TRUE(assets.resolve("", base).empty());
}

TEST(AssetResolver, FirstRootWinsAndAnotherVersionIsAConflict)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("roots1"));
    assets.addRoot(fixture("roots2"));
    ASSERT_EQ(assets.errors().size(), 1u);
    EXPECT_NE(assets.errors().front().find("conflicts"), std::string::npos);
    EXPECT_EQ(assets.find("robotik.robots")->version.toString(), "1.2.0");
    // Scanning the same root twice is not a conflict.
    assets.addRoot(fixture("roots1"));
    EXPECT_EQ(assets.errors().size(), 1u);
}

TEST(AssetResolver, RequireChecksTheVersion)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("roots1"));
    std::string why;
    EXPECT_NE(assets.require("robotik.robots", *robotik::Version::parse("1.1"), why), nullptr);
    EXPECT_EQ(assets.require("robotik.robots", *robotik::Version::parse("2.0"), why), nullptr);
    EXPECT_NE(why.find("1.2.0"), std::string::npos);
    EXPECT_EQ(assets.require("robotik.tools", *robotik::Version::parse("1"), why), nullptr);
    EXPECT_NE(why.find("not installed"), std::string::npos);
}

TEST(AssetPack, UnverifiedLicenseIsFlagged)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("roots1"));
    EXPECT_TRUE(assets.find("robotik.robots")->licenseVerified());
    EXPECT_FALSE(assets.find("robotik.robots.abb")->licenseVerified());
}

TEST(AssetPack, ModelLeavingThePackIsRejected)
{
    robotik::AssetResolver assets;
    assets.addRoot(fixture("escape"));
    EXPECT_TRUE(assets.packs().empty());
    ASSERT_EQ(assets.errors().size(), 1u);
    EXPECT_NE(assets.errors().front().find("passwd"), std::string::npos);
}

TEST(AssetResolver, RepositoryPacksHaveTheirModels)
{
    for (robotik::AssetPack const& pack : testAssets().packs())
    {
        for (auto const& [key, file] : pack.models)
        {
            EXPECT_TRUE(std::filesystem::is_regular_file(pack.model(key)))
                << pack.id << "." << key;
        }
    }
    EXPECT_TRUE(testAssets().errors().empty())
        << (testAssets().errors().empty() ? "" : testAssets().errors().front());
    EXPECT_NE(testAssets().find("robotik.painter"), nullptr);
}
