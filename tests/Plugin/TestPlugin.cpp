// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/World/World.hpp"

#include <cstdlib>
#include <string>

namespace
{

std::filesystem::path repo(std::filesystem::path const& p_relative)
{
    for (char const* root : { ".", ".." })
    {
        std::filesystem::path const candidate = std::filesystem::path(root) / p_relative;
        if (std::filesystem::exists(candidate))
        {
            return candidate;
        }
    }
    return p_relative;
}

std::filesystem::path buildRoot()
{
    for (char const* root : { "build", "../build" })
    {
        if (std::filesystem::is_directory(root))
        {
            return root;
        }
    }
    return "build";
}

std::string drawPanel(robotik::PluginHost const& p_host)
{
    if (p_host.panels().empty() || p_host.panels().front().draw == nullptr)
    {
        return {};
    }
    std::string text;
    RobotikCanvas canvas{};
    canvas.size = static_cast<std::uint32_t>(sizeof(canvas));
    canvas.impl = &text;
    canvas.text = [](RobotikCanvas const* p_canvas, char const* p_line) {
        auto* buffer = static_cast<std::string*>(p_canvas->impl);
        if (p_line != nullptr)
        {
            buffer->append(p_line);
        }
    };
    p_host.panels().front().draw(p_host.panels().front().user, &canvas);
    return text;
}

} // namespace

TEST(PluginCatalog, OnePluginListsSeveralScenarios)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("demos/Manipulator") };
    catalog.scan(roots);
    ASSERT_EQ(catalog.packages().size(), 1u);
    robotik::PluginPackage const& package = catalog.packages().front();
    EXPECT_EQ(package.id, "robotik.manipulator");
    ASSERT_EQ(package.scenarios.size(), 3u);
    EXPECT_EQ(package.scenarios[0].id, "forward_kinematics");
    EXPECT_EQ(package.scenarios[1].id, "inverse_kinematics");
    EXPECT_EQ(package.scenarios[2].id, "joint_limits");
    EXPECT_EQ(package.library.filename(), "librobotik_manipulator.so");
    EXPECT_EQ(catalog.findScenario(package.scenarios[0].file), &package);
    EXPECT_EQ(catalog.findScenario(package.scenarios[2].file)->id, package.id);
}

TEST(PluginCatalog, ProbeOwnsTwoScenarios)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("demos/Probe") };
    catalog.scan(roots);
    ASSERT_EQ(catalog.packages().size(), 1u);
    EXPECT_EQ(catalog.packages().front().scenarios.size(), 2u);
}

TEST(PluginCatalog, RejectsPathsThatLeaveThePackage)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("tests/Plugin/fixtures/escape"),
                                                    repo("tests/Plugin/fixtures/slash") };
    catalog.scan(roots);
    EXPECT_TRUE(catalog.packages().empty());
    EXPECT_GE(catalog.errors().size(), 2u);
}

TEST(Scenario, BareModelNameResolvesFromPluginScenario)
{
    robotik::Scenario const scenario = robotik::Scenario::load(
        repo("demos/Manipulator/scenarios/forward_kinematics.yml"));
    EXPECT_EQ(scenario.name, "forward_kinematics");
    EXPECT_EQ(scenario.robot_model.filename(), "robot_6axis.urdf");
    EXPECT_TRUE(std::filesystem::exists(scenario.robot_model));
}

TEST(PluginManager, MissingLibraryAndIncompatibleAbi)
{
    std::filesystem::path const directory = buildRoot() / "plugins-fixtures";
    std::filesystem::create_directories(directory);
    std::filesystem::path const empty = directory / "empty.so";
    std::filesystem::path const bad = directory / "bad_abi.so";
    std::string const compile_empty =
        "gcc -shared -fPIC -o " + empty.string() + " " +
        repo("tests/Plugin/empty.c").string();
    std::string const compile_bad =
        "gcc -shared -fPIC -I" + repo("include").string() + " -o " + bad.string() + " " +
        repo("tests/Plugin/bad_abi.c").string();
    ASSERT_EQ(std::system(compile_empty.c_str()), 0);
    ASSERT_EQ(std::system(compile_bad.c_str()), 0);

    robotik::PluginManager missing;
    EXPECT_EQ(missing.load((directory / "absent.so").string()), ROBOTIK_PLUGIN_ERR_NOT_FOUND);
    EXPECT_FALSE(missing.error().empty());

    robotik::PluginManager symbols;
    EXPECT_EQ(symbols.load(empty.string()), ROBOTIK_PLUGIN_ERR_NOT_FOUND);
    EXPECT_FALSE(symbols.error().empty());

    robotik::PluginManager abi;
    EXPECT_EQ(abi.load(bad.string()), ROBOTIK_PLUGIN_ERR_ABI);
    EXPECT_EQ(abi.state(), robotik::PluginState::Unloaded);
}

TEST(PluginSession, ProbeLifecycleKeepsTheLibraryAcrossScenarios)
{
    std::filesystem::path const library =
        buildRoot() / "plugins" / "Probe" / "librobotik_probe.so";
    if (!std::filesystem::is_regular_file(library))
    {
        GTEST_SKIP() << "probe plugin is not built";
    }
    robotik::PluginSession session;
    session.scan(buildRoot() / "plugins" / "Probe");
    std::filesystem::path const alpha =
        buildRoot() / "plugins" / "Probe" / "scenarios" / "alpha.yml";
    std::filesystem::path const beta =
        buildRoot() / "plugins" / "Probe" / "scenarios" / "beta.yml";
    ASSERT_EQ(session.prepare(alpha), robotik::PluginPrepare::Ready);
    EXPECT_EQ(session.start(), ROBOTIK_PLUGIN_OK);
    session.publishKey('H');
    session.afterStep(0.01);
    EXPECT_NE(drawPanel(session.host()).find("alpha"), std::string::npos);
    EXPECT_NE(drawPanel(session.host()).find("keys=1"), std::string::npos);
    ASSERT_FALSE(session.host().menus().empty());
    session.host().menus().front().callback(session.host().menus().front().user);
    EXPECT_NE(drawPanel(session.host()).find("homes=1"), std::string::npos);

    ASSERT_EQ(session.prepare(beta), robotik::PluginPrepare::Ready);
    EXPECT_EQ(session.scenarioName(), "Beta");
    EXPECT_EQ(drawPanel(session.host()).find("alpha"), std::string::npos);
    session.host().menus().front().callback(session.host().menus().front().user);

    robotik::PluginSession broken;
    broken.scan(buildRoot() / "plugins" / "Probe");
    // Setup failure is requested through the manager after a normal load.
    robotik::PluginManager manager;
    ASSERT_EQ(manager.load(library.string()), ROBOTIK_PLUGIN_OK);
    robotik::PluginHost host;
    ASSERT_EQ(manager.create(host.api()), ROBOTIK_PLUGIN_OK);
    EXPECT_EQ(manager.setup("broken"), ROBOTIK_PLUGIN_ERR_SETUP);
    EXPECT_EQ(host.error(), "broken scenario");
    EXPECT_TRUE(host.menus().empty());
    manager.shutdown();
    manager.destroy();
    EXPECT_EQ(manager.state(), robotik::PluginState::Loaded);
    EXPECT_EQ(manager.unload(), ROBOTIK_PLUGIN_OK);
    EXPECT_EQ(manager.state(), robotik::PluginState::Unloaded);
}

TEST(PluginSession, InverseKinematicsAssertionTurnsGreenWithoutStoppingEarly)
{
    std::filesystem::path const scenario =
        buildRoot() / "plugins" / "Manipulator" / "scenarios" / "inverse_kinematics.yml";
    if (!std::filesystem::is_regular_file(
            buildRoot() / "plugins" / "Manipulator" / "librobotik_manipulator.so"))
    {
        GTEST_SKIP() << "manipulator plugin is not built";
    }
    robotik::PluginSession session;
    session.setHeadless(true);
    session.scan(buildRoot() / "plugins");
    ASSERT_EQ(session.prepare(scenario), robotik::PluginPrepare::Ready) << session.error();

    compages::world::World world;
    robotik::Simulation simulation(
        world, robotik::Scenario::load(scenario), nullptr, session.mission());
    session.attach(&simulation);
    ASSERT_EQ(session.start(), ROBOTIK_PLUGIN_OK);

    bool saw_failure = false;
    EXPECT_FALSE(simulation.finished());
    for (auto const& check : simulation.checks())
    {
        if (check.text.find("ik.error") != std::string::npos && !check.passed)
        {
            saw_failure = true;
        }
    }
    EXPECT_TRUE(saw_failure);

    for (int step = 0; step < 800 && !simulation.finished(); ++step)
    {
        simulation.step(Seconds(0.01));
        session.afterStep(0.01);
    }
    EXPECT_TRUE(simulation.finished());
    bool recovered = false;
    for (auto const& check : simulation.checks())
    {
        if (check.text.find("ik.error") != std::string::npos)
        {
            recovered = check.passed;
        }
    }
    EXPECT_TRUE(recovered);
    session.detach();
}
