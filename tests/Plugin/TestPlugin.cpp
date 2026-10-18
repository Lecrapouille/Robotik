// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "main.hpp"

#include <algorithm>

#include "Robotik/Plugin/PluginSession.hpp"
#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Scenario/Scenario.hpp"

#include "Compages/World/World.hpp"

#include <cstdlib>
#include <string>

/// Resolves a path from the repository root, from build/ or from src.
static std::filesystem::path repo(std::filesystem::path const& p_relative)
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

/// Directory of the build, next to the sources or one level above.
static std::filesystem::path buildRoot()
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

/// Draws the first plugin panel and returns the text it wrote.
static std::string drawPanel(robotik::PluginHost const& p_host)
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

TEST(PluginCatalog, ReadsTheLibrariesAndTheirLicense)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("plugins/LineFollower"),
                                                    repo("plugins/HumanoidWalk") };
    catalog.scan(roots);
    robotik::PluginPackage const* follower = catalog.findPackage("robotik.line_follower");
    ASSERT_NE(follower, nullptr);
    ASSERT_EQ(follower->system.size(), 1u);
    EXPECT_EQ(follower->system[0].name, "opencv4");
    EXPECT_EQ(follower->system[0].license, "Apache-2.0");
    ASSERT_EQ(follower->thirdparties.size(), 1u);
    EXPECT_EQ(follower->thirdparties[0].name, "AprilRobotics/apriltag");
    EXPECT_EQ(follower->thirdparties[0].url, "https://github.com/AprilRobotics/apriltag");
    EXPECT_EQ(follower->thirdparties[0].license, "BSD-2-Clause");

    robotik::PluginPackage const* walk = catalog.findPackage("robotik.humanoid_walk");
    ASSERT_NE(walk, nullptr);
    ASSERT_EQ(walk->thirdparties.size(), 3u);
    // A quoted value keeps its colon; an entry without repo keeps its url.
    EXPECT_EQ(walk->thirdparties[0].description, "LimX HU_D04 humanoid: URDF and meshes");
    EXPECT_EQ(walk->thirdparties[2].name, "onnxruntime");
    EXPECT_EQ(walk->thirdparties[2].url, "https://github.com/microsoft/onnxruntime");
    EXPECT_EQ(walk->thirdparties[2].license, "MIT");
}

TEST(PluginCatalog, OnePluginListsSeveralScenarios)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("plugins/HelloWorld") };
    catalog.scan(roots);
    ASSERT_EQ(catalog.packages().size(), 1u);
    robotik::PluginPackage const& package = catalog.packages().front();
    EXPECT_EQ(package.id, "robotik.hello_world");
    ASSERT_EQ(package.scenarios.size(), 4u);
    EXPECT_EQ(package.scenarios[0].id, "hello");
    EXPECT_EQ(package.scenarios[1].id, "wave");
    EXPECT_EQ(package.scenarios[2].id, "reach");
    EXPECT_EQ(package.scenarios[3].id, "skills");
    EXPECT_EQ(package.library.filename(), "librobotik_hello_world.so");
    EXPECT_EQ(catalog.findScenario(package.scenarios[0].file), &package);
    EXPECT_EQ(catalog.findScenario(package.scenarios[3].file)->id, package.id);
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

TEST(Scenario, ModelIdResolvesThroughTheAssetPacks)
{
    robotik::Scenario const scenario = robotik::Scenario::load(
        repo("plugins/HelloWorld/scenarios/hello.scenario.yaml"), testAssets());
    EXPECT_EQ(scenario.id, "hello");
    EXPECT_EQ(scenario.world.robot().model.filename(), "robot_6axis.urdf");
    EXPECT_TRUE(std::filesystem::exists(scenario.world.robot().model));
}

TEST(PluginCatalog, ReadsTheResourcesAndHostSections)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("plugins/Painter"),
                                                    repo("plugins/PickAndPlace") };
    catalog.scan(roots);
    ASSERT_EQ(catalog.packages().size(), 2u) << (catalog.errors().empty() ? "" : catalog.errors().front());
    robotik::PluginPackage const* painter = catalog.findPackage("robotik.painter");
    ASSERT_NE(painter, nullptr);
    EXPECT_EQ(painter->api_version, ROBOTIK_PLUGIN_ABI_VERSION);
    EXPECT_EQ(painter->version.toString(), "1.0.0");
    EXPECT_EQ(painter->resources.filename(), "data");
    ASSERT_EQ(painter->packs.size(), 2u);
    EXPECT_EQ(painter->packs[0].id, "robotik.robots");
    EXPECT_EQ(painter->packs[0].version.toString(), "1.0.0");
    EXPECT_FALSE(painter->owns_clock);
    robotik::PluginPackage const* pick = catalog.findPackage("robotik.pick_and_place");
    ASSERT_NE(pick, nullptr);
    EXPECT_FALSE(pick->owns_clock);
    // The clock and the submenu are chosen per scenario.
    ASSERT_FALSE(pick->scenarios.empty());
    EXPECT_FALSE(pick->scenarios.front().owns_clock);
    EXPECT_EQ(pick->scenarios.front().menu, "Behavior tree");
    auto const rl = std::find_if(pick->scenarios.begin(), pick->scenarios.end(),
                                 [](robotik::PluginScenario const& p_scenario) {
                                     return p_scenario.id == "pick_and_place_rl";
                                 });
    ASSERT_NE(rl, pick->scenarios.end());
    EXPECT_TRUE(rl->owns_clock);
    EXPECT_EQ(rl->menu, "Reinforcement learning");
}

TEST(PluginCatalog, RejectsAnotherPluginApi)
{
    robotik::PluginCatalog catalog;
    std::vector<std::filesystem::path> const roots{ repo("tests/Plugin/fixtures/api") };
    catalog.scan(roots);
    EXPECT_TRUE(catalog.packages().empty());
    ASSERT_EQ(catalog.errors().size(), 1u);
    EXPECT_NE(catalog.errors().front().find("plugin API 99"), std::string::npos);
}

TEST(PluginSession, RefusesAPackageWhosePackIsMissing)
{
    robotik::PluginSession session;
    session.scan(repo("tests/Plugin/fixtures/nopack"));
    std::filesystem::path const scenario =
        repo("tests/Plugin/fixtures/nopack/scenarios/one.scenario.yaml");
    EXPECT_EQ(session.prepare(scenario), robotik::PluginPrepare::Failed);
    EXPECT_NE(session.error().find("robotik.unknown"), std::string::npos) << session.error();
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

TEST(PluginSession, LifecycleKeepsTheLibraryAcrossScenarios)
{
    std::filesystem::path const package = buildRoot() / "plugins" / "HelloWorld";
    std::filesystem::path const library = package / "librobotik_hello_world.so";
    if (!std::filesystem::is_regular_file(library))
    {
        GTEST_SKIP() << "hello world plugin is not built";
    }
    robotik::PluginSession session;
    session.scan(package);
    std::filesystem::path const hello = package / "scenarios" / "hello.scenario.yaml";
    std::filesystem::path const wave = package / "scenarios" / "wave.scenario.yaml";
    ASSERT_EQ(session.prepare(hello), robotik::PluginPrepare::Ready);
    EXPECT_EQ(session.start(), ROBOTIK_PLUGIN_OK);
    // 'H' is a key press and a home request.
    session.publishKey('H');
    session.afterStep(0.01);
    EXPECT_NE(drawPanel(session.host()).find("scenario hello"), std::string::npos);
    EXPECT_NE(drawPanel(session.host()).find("updates=1 keys=1 homes=1"), std::string::npos);
    ASSERT_FALSE(session.host().menus().empty());
    session.host().menus().front().callback(session.host().menus().front().user);
    EXPECT_NE(drawPanel(session.host()).find("homes=2"), std::string::npos);

    ASSERT_EQ(session.prepare(wave), robotik::PluginPrepare::Ready);
    EXPECT_EQ(session.scenarioName(), "2. Move a joint");
    EXPECT_EQ(drawPanel(session.host()).find("hello"), std::string::npos);
    session.host().menus().front().callback(session.host().menus().front().user);

    // Setup failure is requested through the manager after a normal load.
    robotik::PluginManager manager;
    ASSERT_EQ(manager.load(library.string()), ROBOTIK_PLUGIN_OK);
    robotik::PluginHost host;
    ASSERT_EQ(manager.create(host.api()), ROBOTIK_PLUGIN_OK);
    EXPECT_EQ(manager.setup("broken"), ROBOTIK_PLUGIN_ERR_SETUP);
    EXPECT_EQ(host.error(), "unknown hello world scenario: broken");
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
        buildRoot() / "plugins" / "HelloWorld" / "scenarios" / "reach.scenario.yaml";
    if (!std::filesystem::is_regular_file(
            buildRoot() / "plugins" / "HelloWorld" / "librobotik_hello_world.so"))
    {
        GTEST_SKIP() << "hello world plugin is not built";
    }
    robotik::PluginSession session;
    session.setHeadless(true);
    session.scan(buildRoot() / "plugins");
    ASSERT_EQ(session.prepare(scenario), robotik::PluginPrepare::Ready) << session.error();

    compages::world::World world;
    robotik::Simulation simulation(
        world, robotik::Scenario::load(scenario, session.assets()), nullptr, session.mission());
    session.attach(&simulation);
    ASSERT_EQ(session.start(), ROBOTIK_PLUGIN_OK);

    bool saw_failure = false;
    EXPECT_FALSE(simulation.finished());
    for (auto const& check : simulation.checks())
    {
        if (check.text.find("reach.error") != std::string::npos && !check.ok())
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
        if (check.text.find("reach.error") != std::string::npos)
        {
            recovered = check.verdict == robotik::Check::Verdict::Passed;
        }
    }
    EXPECT_TRUE(recovered);
    session.detach();
}
