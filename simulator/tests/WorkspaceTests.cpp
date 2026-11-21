// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. See LICENSING.md for details.

/// @file WorkspaceTests.cpp
/// Checks editor identity, expansion and saved layouts without a GPU.

#include "Ui.hpp"
#include <gtest/gtest.h>
#include <imgui_internal.h>

/// Draws one frame without a renderer or a loaded scenario.
static void frame(App& p_app)
{
    updatePanelLayout(p_app);
    ImGui::NewFrame();
    drawPanels(p_app);
    ImGui::Render();
}

TEST(SimulatorEditors, PreserveIdentityAndLayout)
{
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1600, 900);
    io.DeltaTime = 1.0F / 60;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;
    unsigned char* pixels;
    int width, height;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
    {
        App app;
        initializePanelSettings(app);
        frame(app);
        frame(app);
        frame(app);
        auto* timeline = ImGui::FindWindowByName("Timeline");
        ASSERT_TRUE(timeline && timeline->DockId);
        auto const original_id = timeline->ID;
        auto const original_dock = timeline->DockId;
        auto const original_pos = timeline->Pos;
        auto const original_size = timeline->Size;
        for (int i = 0; i < 5; ++i)
        {
            toggleMaximized(app, "Timeline");
            frame(app);
            frame(app);
            ASSERT_TRUE(app.ui.maximized_window == "Timeline");
            ASSERT_TRUE(timeline->ID == original_id);
            ASSERT_TRUE(timeline->DockId == 0);
            ASSERT_TRUE(timeline->Size.x == io.DisplaySize.x);
            toggleMaximized(app, "Timeline");
            frame(app);
            frame(app);
            frame(app);
            ASSERT_TRUE(app.ui.maximized_window.empty());
            ASSERT_TRUE(timeline->DockId == original_dock);
            ASSERT_TRUE(timeline->Pos.x == original_pos.x &&
                        timeline->Pos.y == original_pos.y);
            ASSERT_TRUE(timeline->Size.x == original_size.x &&
                        timeline->Size.y == original_size.y);
        }
        // Opening another tool restores the layout and leaves Timeline open.
        toggleMaximized(app, "Timeline");
        frame(app);
        showPanel(app, PanelKind::Inspector);
        frame(app);
        frame(app);
        ASSERT_TRUE(app.ui.maximized_window.empty());
        ASSERT_TRUE(
            app.ui.panels[static_cast<std::size_t>(PanelKind::Timeline)]);
        // Closing while expanded survives restoration and preset changes.
        toggleMaximized(app, "Timeline");
        frame(app);
        app.ui.panels[static_cast<std::size_t>(PanelKind::Timeline)] = false;
        app.ui.workspace = Workspace::Robot;
        app.ui.pending_maximized = std::string();
        app.ui.reset_layout = true;
        frame(app);
        frame(app);
        ASSERT_TRUE(app.ui.workspace == Workspace::Robot);
        ASSERT_TRUE(
            !app.ui.panels[static_cast<std::size_t>(PanelKind::Timeline)]);
        auto* inspector = ImGui::FindWindowByName("Inspector");
        auto const inspector_id = inspector->ID;
        app.ui.workspace = Workspace::Plugin;
        app.ui.reset_layout = true;
        frame(app);
        frame(app);
        showPanel(app, PanelKind::Inspector);
        frame(app);
        ASSERT_TRUE(app.ui.workspace == Workspace::Plugin);
        ASSERT_TRUE(inspector->ID == inspector_id);
        // All registered editors can be expanded with the same window identity.
        char const* names[] = {
            "3D scene",        "Execution graph", "Plugin",    "Files",
            "Package details", "Assets",          "Inspector", "Joints",
            "Joint plots",     "Teach Pendant",   "Cameras",   "Assertions",
            "Timeline",        "Events",          "Resources", "Planner",
            "World model",     "Perception"
        };
        for (std::size_t i = 0; i < PANEL_COUNT; ++i)
        {
            showPanel(app, static_cast<PanelKind>(i));
            frame(app);
            frame(app);
            auto* window = ImGui::FindWindowByName(names[i]);
            ASSERT_TRUE(window);
            auto id = window->ID;
            toggleMaximized(app, names[i]);
            frame(app);
            frame(app);
            ASSERT_TRUE(app.ui.maximized_window == names[i]);
            ASSERT_TRUE(window->ID == id && window->DockId == 0);
            app.ui.pending_maximized = std::string();
            frame(app);
            frame(app);
        }
        // A requested editor wins over default focus during layout recovery.
        showPanel(app, PanelKind::Cameras);
        app.ui.reset_layout = true;
        frame(app);
        frame(app);
        frame(app);
        auto* cameras = ImGui::FindWindowByName("Cameras");
        ASSERT_TRUE(cameras && cameras->DockTabIsVisible);
        // A stale focus or a closed editor must never create an empty expansion.
        app.ui.panels[static_cast<std::size_t>(PanelKind::Timeline)] = false;
        toggleMaximized(app, "Timeline");
        frame(app);
        ASSERT_TRUE(app.ui.maximized_window.empty());
        // Plugin editors have namespaced identities and the same lifecycle.
        int draws = 0;
        auto const* api = app.plugins.host().api();
        api->add_panel(
            api->host,
            "test_panel",
            "Inspector",
            [](void* p_user, RobotikCanvas const*)
            { ++*static_cast<int*>(p_user); },
            &draws);
        frame(app);
        frame(app);
        std::string const plugin_title =
            pluginWindowTitle(app, app.plugins.host().panels().front());
        auto* plugin_window = ImGui::FindWindowByName(plugin_title.c_str());
        ASSERT_TRUE(plugin_window && plugin_window->ID != inspector_id);
        toggleMaximized(app, plugin_title);
        frame(app);
        frame(app);
        ASSERT_TRUE(app.ui.maximized_window == plugin_title);
        ASSERT_TRUE(plugin_window->DockId == 0);
        app.ui.plugin_panels["test_panel"] = false;
        app.ui.pending_maximized = std::string();
        frame(app);
        frame(app);
        int const stopped = draws;
        frame(app);
        ASSERT_EQ(draws, stopped);
        app.ui.workspace = Workspace::Mission;
        app.ui.reset_layout = true;
        frame(app);
        frame(app);
        ASSERT_FALSE(app.ui.plugin_panels["test_panel"]);
        // Closed plugin editors are saved and restored alongside standard ones.
        std::string const settings = ImGui::SaveIniSettingsToMemory();
        app.ui.plugin_panels["test_panel"] = true;
        ImGui::LoadIniSettingsFromMemory(settings.c_str());
        ASSERT_FALSE(app.ui.plugin_panels["test_panel"]);
        // Use the same action as the top buttons, menu and keyboard shortcuts.
        // A workspace never left opens its defaults, without window copies.
        ASSERT_EQ(app.ui.workspace, Workspace::Mission);
        PanelVisibility const mission = app.ui.panels;
        for (Workspace const preset :
             { Workspace::Robot, Workspace::Plugin, Workspace::Autonomy })
        {
            selectWorkspace(app, preset);
            frame(app);
            frame(app);
            ASSERT_EQ(app.ui.workspace, preset);
            ASSERT_EQ(app.ui.panels, defaultPanels(preset));
            ASSERT_EQ(inspector->ID, inspector_id);
            for (std::size_t i = 0; i < PANEL_COUNT; ++i)
            {
                auto* window = ImGui::FindWindowByName(
                    panelName(static_cast<PanelKind>(i)));
                ASSERT_EQ(window != nullptr && window->Active, app.ui.panels[i])
                    << panelName(static_cast<PanelKind>(i));
            }
        }
        // A workspace left gets back its editors and where they were docked.
        selectWorkspace(app, Workspace::Mission);
        frame(app);
        frame(app);
        ASSERT_EQ(app.ui.panels, mission);
        selectWorkspace(app, Workspace::Robot);
        frame(app);
        frame(app);
        showPanel(app, PanelKind::Files);
        frame(app);
        frame(app);
        auto* files = ImGui::FindWindowByName("Files");
        ImGuiID const files_dock = files->DockId;
        ASSERT_NE(files_dock, 0U);
        selectWorkspace(app, Workspace::Autonomy);
        frame(app);
        frame(app);
        ASSERT_FALSE(files->Active);
        selectWorkspace(app, Workspace::Robot);
        frame(app);
        frame(app);
        ASSERT_TRUE(app.ui.panels[static_cast<std::size_t>(PanelKind::Files)]);
        ASSERT_TRUE(files->Active);
        ASSERT_EQ(files->DockId, files_dock);
        // The editors of each workspace are saved with the settings.
        std::string const saved = ImGui::SaveIniSettingsToMemory();
        ASSERT_NE(saved.find("Workspace2="), std::string::npos);
        // Editors saved by an older layout version are not read back.
        PanelVisibility const before = app.ui.panels;
        char const old_settings[] = "[RobotikEditors1][Visibility]\nOpen=0\n";
        ImGui::LoadIniSettingsFromMemory(old_settings);
        ASSERT_EQ(app.ui.panels, before);
        // Clicking the active preset recovers its default contents.
        selectWorkspace(app, Workspace::Robot);
        frame(app);
        ASSERT_EQ(app.ui.panels, defaultPanels(Workspace::Robot));
        // Switching from an expanded view restores the layout first.
        frame(app);
        toggleMaximized(app, "Joints");
        frame(app);
        ASSERT_EQ(app.ui.maximized_window, "Joints");
        selectWorkspace(app, Workspace::Mission);
        frame(app);
        frame(app);
        ASSERT_TRUE(app.ui.maximized_window.empty());
        ASSERT_EQ(app.ui.workspace, Workspace::Mission);
        ASSERT_EQ(app.ui.panels, mission);
        // Shutdown restores a maximized editor before persistence.
        toggleMaximized(app, "Inspector");
        frame(app);
        shutdownPanelSettings(app);
        ASSERT_TRUE(app.ui.maximized_window.empty());
        ASSERT_TRUE(inspector->DockId != 0);
    }
    ImGui::DestroyContext();
}
