// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "SimulatorDisplay.hpp"
#include "Ui.hpp"

#include "Robotik/Package/Plugin/PluginABI.h"
#include "Robotik/Package/Scenario/Scenario.hpp"

#include "Compages/World/Controllers/Controls.hpp"

#include <imgui.h>

#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

/// Two-column "key  value" rows.
static void field(char const* p_key, char const* p_format, ...) IM_FMTARGS(2);
static void field(char const* p_key, char const* p_format, ...)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", p_key);
    ImGui::TableNextColumn();
    va_list args;
    va_start(args, p_format);
    ImGui::TextWrappedV(p_format, args);
    va_end(args);
}

/// Opens the two-column table the field rows share.
static bool beginFields(char const* p_id)
{
    if (!beginTable(p_id, 2, ImGuiTableFlags_SizingStretchProp))
    {
        return false;
    }
    ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
    return true;
}

/// The license of a pack: green when somebody wrote it down.
static void licenseBadge(robotik::AssetPack const& p_pack)
{
    if (robotik::licenseVerified(p_pack))
    {
        ImGui::TextColored(GREEN, "%s", p_pack.license.c_str());
    }
    else
    {
        badge("not checked", ORANGE);
        ImGui::SetItemTooltip(
            "Nobody wrote down the license of these files (%s). Check it "
            "before you share or ship them.",
            p_pack.license.empty() ? "empty" : p_pack.license.c_str());
    }
}

/// True when @p_file is inside the folder @p_root.
static bool inside(std::filesystem::path const& p_root,
                   std::filesystem::path const& p_file)
{
    auto const mismatch = std::mismatch(
        p_root.begin(), p_root.end(), p_file.begin(), p_file.end());
    return !p_root.empty() && mismatch.first == p_root.end();
}

/// Packs the running scenario takes a model from, or the package
/// requires, or the package owns.
static bool usedByRun(App const& p_app,
                      robotik::PluginPackage const* p_package,
                      robotik::AssetPack const& p_pack)
{
    if (p_package != nullptr)
    {
        if (!p_package->resources.empty() &&
            inside(p_package->resources, p_pack.root))
        {
            return true;
        }
        for (robotik::PackRequirement const& requirement : p_package->packs)
        {
            if (requirement.id == p_pack.id)
            {
                return true;
            }
        }
    }
    if (p_app.simulation)
    {
        for (robotik::EntitySpec const& entity :
             p_app.simulation->scenario().world.entities)
        {
            if (!entity.model.empty() && inside(p_pack.root, entity.model))
            {
                return true;
            }
        }
    }
    return false;
}

/// Case-insensitive substring test against the file filter.
static bool matches(std::string const& p_text, char const* p_filter)
{
    if (p_filter[0] == '\0')
    {
        return true;
    }
    std::string text = p_text;
    std::string filter = p_filter;
    auto lower = [](std::string& p_s)
    {
        std::ranges::transform(
            p_s,
            p_s.begin(),
            [](unsigned char p_c)
            { return static_cast<char>(std::tolower(p_c)); });
    };
    lower(text);
    lower(filter);
    return text.find(filter) != std::string::npos;
}

/// Description, timing and assertions of the loaded scenario.
static void scenarioDetails(App& p_app, robotik::PluginPackage const* p_package)
{
    robotik::Scenario const& scenario = p_app.simulation->scenario();
    if (!scenario.description.empty())
    {
        ImGui::TextWrapped("%s", scenario.description.c_str());
        ImGui::Spacing();
    }
    if (!beginFields("scenario"))
    {
        return;
    }
    field("id", "%s", scenario.id.c_str());
    field("world", "%s", scenario.world.id.c_str());
    field("robot", "%s", robotik::firstRobot(scenario.world).model_ref.c_str());
    std::string const controller =
        scenario.controller.planner
            ? "PDDL planner, " +
                  scenario.controller.planner->domain.filename().string()
        : scenario.controller.behavior_tree.empty()
            ? std::string("plugin mission (C++)")
            : scenario.controller.behavior_tree.filename().string();
    field("controller", "%s", controller.c_str());
    field("seed", "%llu", static_cast<unsigned long long>(scenario.seed));
    field("timeout", "%.0f s", scenario.timeout.value());
    if (!scenario.stop_when.empty())
    {
        field("stop when", "%s", scenario.stop_when.c_str());
    }
    field("faults",
          "%zu planned, %zu random",
          scenario.faults.size(),
          scenario.random_faults.size());
    field("perception",
          "%s%s",
          p_app.simulation->cell().perceptionSummary().c_str(),
          p_app.simulation->cell().oracle()
              ? " (the simulator gives the true poses)"
              : "");
    if (p_package != nullptr)
    {
        field("clock",
              "%s",
              p_app.owns_clock ? "the plugin advances time" : "the simulation");
        field("graphics",
              "%s",
              p_package->graphics ? "needs a window" : "works headless");
    }
    endTable();
}

/// Packs, plugins and third parties the package declares.
static void dependencies(App& p_app, robotik::PluginPackage const& p_package)
{
    ImGui::SeparatorText("Dependencies");
    robotik::AssetResolver const& assets = p_app.plugins.assets();
    robotik::PluginCatalog const& catalog = p_app.plugins.catalog();
    bool const none = p_package.resources.empty() && p_package.packs.empty() &&
                      p_package.plugins.empty() && p_package.system.empty() &&
                      p_package.thirdparties.empty();
    if (none)
    {
        ImGui::TextDisabled("None: the package only uses the core.");
        return;
    }
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg |
                                  ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingStretchProp;
    if (!beginTable("dependencies", 3, flags))
    {
        return;
    }
    ImGui::TableSetupColumn("Kind", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch, 1.4F);
    ImGui::TableSetupColumn(
        "License / status", ImGuiTableColumnFlags_WidthStretch, 1.0F);
    ImGui::TableHeadersRow();

    if (!p_package.resources.empty())
    {
        for (robotik::AssetPack const& pack : assets.packs())
        {
            if (inside(p_package.resources, pack.root))
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::TextDisabled("own pack");
                ImGui::TableNextColumn();
                ImGui::Text("%s %s",
                            pack.id.c_str(),
                            robotik::toString(pack.version).c_str());
                ImGui::TableNextColumn();
                licenseBadge(pack);
            }
        }
    }
    for (robotik::PackRequirement const& requirement : p_package.packs)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("asset pack");
        ImGui::TableNextColumn();
        ImGui::Text("%s >= %s",
                    requirement.id.c_str(),
                    robotik::toString(requirement.version).c_str());
        ImGui::TableNextColumn();
        std::string why;
        if (robotik::AssetPack const* pack =
                assets.require(requirement.id, requirement.version, why))
        {
            licenseBadge(*pack);
        }
        else
        {
            badge("missing", RED);
            ImGui::SetItemTooltip("%s", why.c_str());
        }
    }
    for (std::string const& plugin : p_package.plugins)
    {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("plugin");
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(plugin.c_str());
        ImGui::TableNextColumn();
        bool const found = catalog.findPackage(plugin) != nullptr;
        badge(found ? "found" : "missing", found ? GREEN : RED);
    }
    auto libraries =
        [](std::vector<robotik::ThirdParty> const& p_list, char const* p_kind)
    {
        for (robotik::ThirdParty const& library : p_list)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", p_kind);
            ImGui::TableNextColumn();
            if (library.url.empty())
            {
                ImGui::TextUnformatted(library.name.c_str());
            }
            else
            {
                ImGui::TextLinkOpenURL(library.name.c_str(),
                                       library.url.c_str());
            }
            if (!library.description.empty() || !library.reference.empty())
            {
                ImGui::SetItemTooltip("%s%s%s",
                                      library.description.c_str(),
                                      library.reference.empty() ? ""
                                                                : "\nVersion: ",
                                      library.reference.c_str());
            }
            ImGui::TableNextColumn();
            if (library.license.empty())
            {
                badge("not checked", ORANGE);
            }
            else
            {
                ImGui::TextColored(GREEN, "%s", library.license.c_str());
            }
        }
    };
    libraries(p_package.system, "system");
    libraries(p_package.thirdparties, "library");
    endTable();
    if (!p_package.system.empty() || !p_package.thirdparties.empty())
    {
        ImGui::TextDisabled(
            "Libraries are listed in dependencies: of plugin.yaml.");
    }
}

/// Prints a byte count as B, kB or MB.
static std::string sizeText(std::uintmax_t p_bytes)
{
    char text[32];
    if (p_bytes < 1024U)
    {
        std::snprintf(text, sizeof(text), "%ju B", p_bytes);
    }
    else if (p_bytes < 1024U * 1024U)
    {
        std::snprintf(text,
                      sizeof(text),
                      "%.1f KB",
                      static_cast<double>(p_bytes) / 1024.0);
    }
    else
    {
        std::snprintf(text,
                      sizeof(text),
                      "%.1f MB",
                      static_cast<double>(p_bytes) / (1024.0 * 1024.0));
    }
    return text;
}

/// Folders first, then files, each sorted by name.
static PackageEntry readFolder(std::filesystem::path const& p_folder,
                               std::string p_name,
                               int p_depth)
{
    PackageEntry entry;
    entry.name = std::move(p_name);
    entry.path = p_folder;
    entry.folder = true;
    std::error_code error;
    std::filesystem::directory_iterator cursor(p_folder, error);
    for (; !error && cursor != std::filesystem::directory_iterator();
         cursor.increment(error))
    {
        std::filesystem::path const path = cursor->path();
        std::string name = path.filename().string();
        if (cursor->is_directory(error))
        {
            if (p_depth < 6)
            {
                entry.children.push_back(
                    readFolder(path, std::move(name), p_depth + 1));
            }
        }
        else
        {
            PackageEntry file;
            file.name = std::move(name);
            file.path = path;
            file.size = cursor->file_size(error);
            entry.children.push_back(std::move(file));
        }
    }
    std::ranges::sort(entry.children,
                      [](PackageEntry const& p_a, PackageEntry const& p_b)
                      {
                          return p_a.folder != p_b.folder ? p_a.folder
                                                          : p_a.name < p_b.name;
                      });
    for (PackageEntry const& child : entry.children)
    {
        entry.size += child.size;
    }
    return entry;
}

/// Files of the loaded scenario and experiment, as canonical paths,
/// to mark them in the tree.
static std::vector<std::filesystem::path> loadedFiles(App const& p_app)
{
    std::vector<std::filesystem::path> files;
    auto add = [&files](std::filesystem::path const& p_path)
    {
        std::error_code error;
        if (!p_path.empty())
        {
            files.push_back(std::filesystem::weakly_canonical(p_path, error));
        }
    };
    add(p_app.scenario_path);
    add(p_app.world_path);
    if (p_app.simulation)
    {
        add(p_app.simulation->scenario().controller.behavior_tree);
    }
    add(p_app.ui.experiment_path);
    return files;
}

/// One file or folder of the package tree.
static void fileRow(App& p_app,
                    PackageEntry const& p_entry,
                    std::vector<std::filesystem::path> const& p_loaded,
                    int p_depth)
{
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    if (p_entry.folder)
    {
        // The content folders open at first; data/ may hold hundreds of meshes.
        bool const content = p_depth == 0 && p_entry.name != "data" &&
                             p_entry.name != "external";
        ImGui::SetNextItemOpen(content, ImGuiCond_Once);
        bool const open = ImGui::TreeNodeEx(p_entry.path.string().c_str(),
                                            ImGuiTreeNodeFlags_SpanFullWidth,
                                            "%s/",
                                            p_entry.name.c_str());
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", sizeText(p_entry.size).c_str());
        if (open)
        {
            for (PackageEntry const& child : p_entry.children)
            {
                fileRow(p_app, child, p_loaded, p_depth + 1);
            }
            ImGui::TreePop();
        }
        return;
    }
    bool const loaded =
        std::ranges::find(p_loaded, p_entry.path) != p_loaded.end();
    if (loaded)
    {
        ImGui::PushStyleColor(ImGuiCol_Text, GREEN);
    }
    ImGui::TreeNodeEx(p_entry.path.string().c_str(),
                      ImGuiTreeNodeFlags_Leaf |
                          ImGuiTreeNodeFlags_NoTreePushOnOpen |
                          ImGuiTreeNodeFlags_SpanFullWidth,
                      "%s",
                      p_entry.name.c_str());
    if (loaded)
    {
        ImGui::PopStyleColor();
    }
    if (ImGui::IsItemClicked(ImGuiMouseButton_Left))
    {
        showFile(p_app, p_entry.path);
    }
    if (ImGui::BeginPopupContextItem())
    {
        if (ImGui::MenuItem("Copy path"))
        {
            ImGui::SetClipboardText(p_entry.path.string().c_str());
        }
        ImGui::EndPopup();
    }
    ImGui::SetItemTooltip("%sClick: a scenario is loaded, any other file is "
                          "shown in its tab of Files.",
                          loaded ? "In use (green).\n" : "");
    ImGui::TableNextColumn();
    ImGui::TextDisabled("%s", sizeText(p_entry.size).c_str());
}

/// The package tree and the filter above it.
static void packageFiles(App& p_app, robotik::PluginPackage const& p_package)
{
    ImGui::SeparatorText("Files");
    if (p_app.ui.package_root != p_package.root)
    {
        p_app.ui.package_root = p_package.root;
        p_app.ui.package_tree =
            readFolder(p_package.root, p_package.root.filename().string(), 0);
    }
    if (ImGui::SmallButton("Copy folder"))
    {
        ImGui::SetClipboardText(p_package.root.string().c_str());
    }
    ImGui::SetItemTooltip("%s", p_package.root.string().c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Read again"))
    {
        p_app.ui.package_tree =
            readFolder(p_package.root, p_package.root.filename().string(), 0);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("read only");
    ImGui::SameLine();
    helpMarker(
        "The package as the simulator reads it. In the build tree this is a "
        "copy "
        "made by make: change the files in plugins/<Name>/, not here.\n\n"
        "Green: the files of the loaded scenario and of the chosen "
        "experiment.");
    ImGui::TextDisabled("%s", p_package.root.string().c_str());
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV;
    if (beginTable("files", 2, flags))
    {
        ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Size", ImGuiTableColumnFlags_WidthFixed);
        std::vector<std::filesystem::path> const loaded = loadedFiles(p_app);
        for (PackageEntry const& entry : p_app.ui.package_tree.children)
        {
            fileRow(p_app, entry, loaded, 0);
        }
        endTable();
    }
}

/// A value of an override, written inline: 0.05, [a, b], {x: 1}.
static std::string inlineText(robotik::ConfigNode const& p_node)
{
    if (p_node.isScalar())
    {
        return p_node.scalar();
    }
    std::string text;
    if (p_node.isSequence())
    {
        for (robotik::ConfigNode const& item : p_node.items())
        {
            text += (text.empty() ? "" : ", ") + inlineText(item);
        }
        return "[" + text + "]";
    }
    for (auto const& [key, value] : p_node.entries())
    {
        text += (text.empty() ? "" : ", ") + key + ": " + inlineText(value);
    }
    return "{" + text + "}";
}

/// Table of the cases of the loaded experiment.
static void experimentCases(App& p_app, robotik::Experiment const& p_experiment)
{
    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_SizingStretchProp;
    if (!beginTable("cases", 4, flags))
    {
        return;
    }
    ImGui::TableSetupColumn("Case", ImGuiTableColumnFlags_WidthStretch, 1.0F);
    ImGui::TableSetupColumn(
        "Scenario", ImGuiTableColumnFlags_WidthStretch, 1.0F);
    ImGui::TableSetupColumn(
        "Controller", ImGuiTableColumnFlags_WidthStretch, 0.9F);
    ImGui::TableSetupColumn(
        "Changes", ImGuiTableColumnFlags_WidthStretch, 1.4F);
    ImGui::TableHeadersRow();
    for (robotik::ExperimentCase const& item : p_experiment.cases)
    {
        ImGui::PushID(item.id.c_str());
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(item.id.c_str());
        ImGui::TableNextColumn();
        if (ImGui::TextLink(item.scenario.filename().string().c_str()))
        {
            showFile(p_app, item.scenario);
        }
        ImGui::SetItemTooltip(
            "%s\nClick to load it alone, without the changes of this case.",
            item.scenario.string().c_str());
        ImGui::TableNextColumn();
        if (item.behavior_tree.empty())
        {
            ImGui::TextDisabled("the scenario's");
        }
        else if (ImGui::TextLink(
                     item.behavior_tree.filename().string().c_str()))
        {
            showFile(p_app, item.behavior_tree);
        }
        ImGui::TableNextColumn();
        std::vector<std::string> changes;
        for (auto const& [path, value] : item.overrides)
        {
            changes.push_back(path + " = " + inlineText(value));
        }
        for (robotik::Fault const& fault : item.faults)
        {
            char line[160];
            std::snprintf(line,
                          sizeof(line),
                          "%s %s at %.2f s",
                          fault.disable ? "disable" : "restore",
                          fault.resource.c_str(),
                          fault.at.value());
            changes.emplace_back(line);
        }
        for (robotik::RandomFault const& fault : item.random_faults)
        {
            char line[160];
            std::snprintf(line,
                          sizeof(line),
                          "%s fails at %.3g per second",
                          fault.resource.c_str(),
                          fault.rate);
            changes.emplace_back(line);
        }
        if (changes.empty())
        {
            ImGui::TextDisabled("none");
        }
        for (std::string const& change : changes)
        {
            ImGui::TextWrapped("%s", change.c_str());
        }
        ImGui::PopID();
    }
    endTable();
}

/// Package that owns the scenario currently loaded.
static robotik::PluginPackage const* loadedPackage(App const& p_app)
{
    return p_app.plugins.catalog().matchScenario(p_app.scenario_path).package;
}

/// A new package: its first experiment becomes the chosen one.
static void followPackage(App& p_app, robotik::PluginPackage const* p_package)
{
    UiState& ui = p_app.ui;
    if (p_package == nullptr || ui.experiment_root == p_package->root)
    {
        return;
    }
    ui.experiment_root = p_package->root;
    ui.experiment_path.clear();
    ui.experiment.reset();
    ui.experiment_error.clear();
    ui.shown_files[static_cast<std::size_t>(FileTab::Experiment)] = {};
    if (!p_package->experiments.empty())
    {
        selectExperiment(p_app, p_package->experiments.front().file, false);
    }
}

/// Title of a file tab.
static char const* viewName(FileTab p_view)
{
    switch (p_view)
    {
        case FileTab::Scenario:
            return "Scenario";
        case FileTab::World:
            return "World";
        case FileTab::Tree:
            return "Behavior tree";
        case FileTab::Planner:
            return "Planner";
        case FileTab::Experiment:
            return "Experiment";
        case FileTab::Asset:
        case FileTab::Other:
            break;
    }
    return "";
}

void showView(App& p_app, FileTab p_view, bool p_editor)
{
    // Asset and Other have no view: only the tab of Files changes.
    if (p_view == FileTab::Asset || p_view == FileTab::Other)
    {
        if (p_editor)
        {
            p_app.ui.select_tab = p_view;
        }
        return;
    }
    p_app.ui.view = p_view;
    if (p_editor)
    {
        p_app.ui.select_tab = p_view;
    }
}

void scenarioView(App& p_app)
{
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        return;
    }
    scenarioDetails(p_app, loadedPackage(p_app));
    ImGui::SeparatorText("Assertions");
    assertionsView(p_app);
}

void viewPanel(App& p_app)
{
    FileTab const view = p_app.ui.view;
    // The title follows the view; "###View" keeps the window and its place.
    std::string const title = std::string(viewName(view)) + "###View";
    if (!ImGui::Begin(title.c_str()))
    {
        ImGui::End();
        return;
    }
    for (FileTab const tab : { FileTab::Scenario,
                               FileTab::World,
                               FileTab::Tree,
                               FileTab::Planner,
                               FileTab::Experiment })
    {
        if (tab != FileTab::Scenario)
        {
            ImGui::SameLine();
        }
        bool const active = tab == view;
        if (active)
        {
            ImGui::PushStyleColor(
                ImGuiCol_Button,
                ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
        }
        if (ImGui::SmallButton(viewName(tab)) && !active)
        {
            showView(p_app, tab, true);
        }
        if (active)
        {
            ImGui::PopStyleColor();
        }
    }
    ImGui::Separator();
    switch (view)
    {
        case FileTab::Scenario:
            scenarioView(p_app);
            break;
        case FileTab::World:
            worldView(p_app);
            break;
        case FileTab::Tree:
            treeView(p_app);
            break;
        case FileTab::Planner:
            plannerView(p_app);
            break;
        case FileTab::Experiment:
            experimentView(p_app);
            break;
        case FileTab::Asset:
        case FileTab::Other:
            break;
    }
    ImGui::End();
}

void pluginPanel(App& p_app)
{
    if (!ImGui::Begin("Plugin"))
    {
        ImGui::End();
        return;
    }
    robotik::PluginPackage const* package = loadedPackage(p_app);
    followPackage(p_app, package);
    std::vector<std::string> const& broken = p_app.plugins.catalog().errors();
    if (!p_app.error.empty() || !broken.empty())
    {
        // A broken plugin.yaml leaves its package out of the Demos menu:
        // this is the only place that says why.
        ImGui::SeparatorText("Errors");
        ImGui::PushStyleColor(ImGuiCol_Text, RED);
        if (!p_app.error.empty())
        {
            ImGui::TextWrapped("%s", p_app.error.c_str());
        }
        for (std::string const& error : broken)
        {
            ImGui::TextWrapped("%s", error.c_str());
        }
        ImGui::PopStyleColor();
        ImGui::Spacing();
    }
    if (package == nullptr)
    {
        emptyState("No plugin loaded.\nChoose a scenario in the Demos menu.");
        ImGui::End();
        return;
    }

    ImGui::SeparatorText("Description");
    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.35F);
    ImGui::TextUnformatted(package->name.c_str());
    ImGui::PopFont();
    badge(package->category.c_str(), BLUE);
    if (!package->description.empty())
    {
        ImGui::TextWrapped("%s", package->description.c_str());
    }
    if (beginFields("description"))
    {
        field("id", "%s", package->id.c_str());
        field("version", "%s", robotik::toString(package->version).c_str());
        field("plugin API",
              "%u%s",
              package->api_version,
              package->api_version == ROBOTIK_PLUGIN_ABI_VERSION
                  ? ""
                  : " (not this host)");
        endTable();
    }
    ImGui::Spacing();
    packageFiles(p_app, *package);
    ImGui::Spacing();
    dependencies(p_app, *package);
    ImGui::End();
}

/// Lower-case extension of @p_path, with its dot.
static std::string extensionOf(std::filesystem::path const& p_path)
{
    std::string ext = p_path.extension().string();
    for (char& c : ext)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return ext;
}

/// A world of its own holding the model of @p_path, with a camera that sees
/// all of it; null with @p_error set when the file cannot be drawn.
static std::unique_ptr<ModelView> openModel(std::filesystem::path const& p_path,
                                            std::string& p_error)
{
    auto view = std::make_unique<ModelView>();
    view->world = std::make_unique<compages::world::World>();
    view->scene = std::make_unique<compages::renderer::Scene>(*view->world);
    auto const clear = viewClearColor();
    view->scene->background(clear.x, clear.y, clear.z);
    view->camera = view->scene->camera("PreviewCamera");
    view->scene->activeCamera(view->camera);
    auto loaded = view->scene->load(p_path.string());
    if (!loaded)
    {
        p_error = loaded.error();
        return nullptr;
    }
    // The camera and the sun are placed from the size of the model; the rig
    // then turns around its middle.
    compages::core::Vector3f const middle = view->scene->frameAll();
    view->camera.add<compages::world::Rig>(middle);
    if (compages::world::Rig* rig =
            view->world->behavior<compages::world::Rig>(view->camera))
    {
        // A tool of a few centimetres as well as a whole cell.
        rig->controller.min_distance = 0.01F;
        rig->controller.max_distance = 50.0F;
    }
    if (auto prepared = view->scene->prepare(); !prepared)
    {
        p_error = prepared.error();
        return nullptr;
    }
    return view;
}

/// Shows @p_path in the Asset tab of Files from the next frame.
static void openPreview(App& p_app, std::filesystem::path const& p_path)
{
    p_app.ui.asset_preview.pending = p_path;
    p_app.ui.select_tab = FileTab::Asset;
}

void openClickedAsset(App& p_app)
{
    AssetPreview& preview = p_app.ui.asset_preview;
    if (preview.pending.empty())
    {
        return;
    }
    std::filesystem::path const path = std::move(preview.pending);
    preview.pending.clear();
    preview.path = path;
    preview.text.clear();
    preview.error.clear();
    preview.image = compages::gpu::Texture();
    preview.model.reset();
    preview.show_text = false;
    std::string const ext = extensionOf(path);
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" ||
        ext == ".tga")
    {
        // ImGui draws the first row of the file at the top: no flip.
        compages::gpu::LoadOptions options;
        options.flip_vertically = false;
        compages::Status const loaded =
            preview.image.load(path.string(), options);
        if (!loaded)
        {
            preview.error = loaded.error();
        }
    }
    else if (ext == ".urdf" || ext == ".stl" || ext == ".gltf" ||
             ext == ".glb")
    {
        preview.model = openModel(path, preview.error);
        if (ext == ".urdf")
        {
            preview.text = readShown(path);
        }
    }
    else if (ext == ".obj" || ext == ".dae")
    {
        std::error_code error;
        preview.text = "# 3D mesh, " +
                       sizeText(std::filesystem::file_size(path, error)) +
                       ". This format has no preview.";
    }
    else
    {
        preview.text = readShown(path);
    }
}

/// The 3D view of the model of the preview; the main loop renders it.
static void modelView(ModelView& p_model)
{
    ImVec2 const size(std::max(ImGui::GetContentRegionAvail().x, 1.0F),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0F));
    ImVec2 const scale = ImGui::GetIO().DisplayFramebufferScale;
    auto const width = static_cast<std::uint32_t>(size.x * scale.x);
    auto const height = static_cast<std::uint32_t>(size.y * scale.y);
    if (width != p_model.target.width || height != p_model.target.height)
    {
        // A new texture holds nothing until the main loop renders into it.
        p_model.rendered = false;
        p_model.target.resize(width, height);
    }
    ImVec2 const at = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##model",
                           size,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    p_model.area = ViewArea{ .x = at.x,
                             .y = at.y,
                             .w = size.x,
                             .h = size.y,
                             .hovered = ImGui::IsItemHovered() ||
                                        ImGui::IsItemActive() };
    if (p_model.target.width == 0 || !p_model.rendered)
    {
        return;
    }
    // The device puts the first row at the bottom.
    ImDrawList* draw = ImGui::GetWindowDrawList();
    draw->AddImage(
        ImTextureRef(static_cast<ImTextureID>(p_model.target.color.nativeId())),
        at,
        ImVec2(at.x + size.x, at.y + size.y),
        ImVec2(0.0F, 1.0F),
        ImVec2(1.0F, 0.0F));
    char const* hint = "Left drag: move   Right drag: turn   Wheel: zoom";
    ImVec2 const text = ImGui::CalcTextSize(hint);
    ImVec2 const corner(at.x + 8.0F, at.y + size.y - text.y - 10.0F);
    draw->AddRectFilled(
        corner,
        ImVec2(corner.x + text.x + 12.0F, corner.y + text.y + 6.0F),
        IM_COL32(20, 22, 27, 170),
        4.0F);
    draw->AddText(ImVec2(corner.x + 6.0F, corner.y + 3.0F),
                  ImGui::GetColorU32(ImGuiCol_TextDisabled),
                  hint);
}

void assetView(App& p_app)
{
    AssetPreview& preview = p_app.ui.asset_preview;
    preview.visible = true;
    if (preview.model && !preview.text.empty())
    {
        if (ImGui::SmallButton(preview.show_text ? "3D" : "Text"))
        {
            preview.show_text = !preview.show_text;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%s", preview.path.string().c_str());
    }
    if (!preview.error.empty())
    {
        ImGui::TextColored(RED, "%s", preview.error.c_str());
    }
    if (preview.image.valid())
    {
        auto const width = static_cast<float>(preview.image.width());
        auto const height = static_cast<float>(preview.image.height());
        ImGui::TextDisabled(
            "%.0f x %.0f px  %s", width, height, preview.path.string().c_str());
        // Fit the tab, never larger than the image.
        ImVec2 const room = ImGui::GetContentRegionAvail();
        float const scale = std::min({ 1.0F, room.x / width, room.y / height });
        if (scale > 0.0F)
        {
            ImGui::Image(ImTextureRef(static_cast<ImTextureID>(
                             preview.image.nativeId())),
                         ImVec2(width * scale, height * scale));
        }
    }
    else if (preview.model && !preview.show_text)
    {
        modelView(*preview.model);
    }
    else if (!preview.text.empty())
    {
        textView(preview.path, preview.text);
    }
}

/// Files of @p_pack, relative to its folder and sorted, read once.
static std::vector<std::filesystem::path> const&
packFiles(App& p_app, robotik::AssetPack const& p_pack)
{
    auto [found, added] = p_app.ui.pack_files.try_emplace(p_pack.root);
    if (added)
    {
        std::error_code error;
        std::filesystem::recursive_directory_iterator cursor(p_pack.root,
                                                             error);
        for (; !error && cursor != std::filesystem::recursive_directory_iterator();
             cursor.increment(error))
        {
            if (cursor->is_regular_file(error))
            {
                found->second.push_back(
                    cursor->path().lexically_relative(p_pack.root));
            }
        }
        std::ranges::sort(found->second);
    }
    return found->second;
}

/// Rows of the files of an open pack, under a node closed by default: a
/// click shows one in the Asset tab of Files.
static void assetFiles(App& p_app,
                       robotik::AssetPack const& p_pack,
                       char const* p_filter)
{
    std::vector<std::filesystem::path> const& files = packFiles(p_app, p_pack);
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    bool const open = ImGui::TreeNodeEx("##files",
                                        ImGuiTreeNodeFlags_SpanFullWidth,
                                        "Files (%zu)",
                                        files.size());
    if (!open)
    {
        return;
    }
    for (std::filesystem::path const& file : files)
    {
        std::string const name = file.string();
        if (!matches(name, p_filter) && !matches(p_pack.id, p_filter))
        {
            continue;
        }
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TreeNodeEx(name.c_str(),
                          ImGuiTreeNodeFlags_Leaf |
                              ImGuiTreeNodeFlags_NoTreePushOnOpen |
                              ImGuiTreeNodeFlags_SpanFullWidth);
        if (ImGui::IsItemClicked())
        {
            openPreview(p_app, p_pack.root / file);
        }
        ImGui::SetItemTooltip("Click to see the file.");
        ImGui::TableNextColumn();
        ImGui::TableNextColumn();
        ImGui::TableNextColumn();
        std::error_code error;
        std::uintmax_t const size =
            std::filesystem::file_size(p_pack.root / file, error);
        ImGui::TextDisabled("%s", error ? "?" : sizeText(size).c_str());
    }
    ImGui::TreePop();
}

void assetsPanel(App& p_app)
{
    if (!ImGui::Begin("Assets"))
    {
        ImGui::End();
        return;
    }
    robotik::AssetResolver const& assets = p_app.plugins.assets();
    robotik::PluginPackage const* package =
        p_app.plugins.catalog().matchScenario(p_app.scenario_path).package;
    if (assets.packs().empty())
    {
        emptyState(
            "No asset pack found.\nSet ROBOTIK_DATA or build the project.");
        ImGui::End();
        return;
    }

    std::size_t models = 0;
    std::size_t unchecked = 0;
    for (robotik::AssetPack const& pack : assets.packs())
    {
        models += pack.models.size();
        unchecked += robotik::licenseVerified(pack) ? 0U : 1U;
    }
    ImGui::Text("%zu packs, %zu models", assets.packs().size(), models);
    if (unchecked > 0U)
    {
        ImGui::SameLine();
        ImGui::TextColored(ORANGE,
                           "  %zu license%s not checked",
                           unchecked,
                           unchecked > 1U ? "s" : "");
    }
    ImGui::SameLine();
    helpMarker(
        "An asset pack is a versioned folder of models (URDF and meshes) with "
        "a "
        "pack.yaml: id, version, license, source. A world names a model by "
        "<pack id>.<key>, e.g. robotik.tools.vacuum.\n\nPacks are searched in "
        "ROBOTIK_DATA, data/packs, then in the data/ folder of each plugin.");

    char* const filter = p_app.ui.asset_filter.data();
    ImGui::SetNextItemWidth(
        std::min(220.0F, ImGui::GetContentRegionAvail().x * 0.5F));
    ImGui::InputTextWithHint("##filter",
                             "Filter packs and models",
                             filter,
                             p_app.ui.asset_filter.size());
    ImGui::SameLine();
    ImGui::Checkbox("Used by this run", &p_app.ui.assets_in_use);
    bool const only_used = p_app.ui.assets_in_use;

    ImGuiTableFlags const flags =
        ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
        ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY;
    float const errors_height =
        assets.errors().empty() ? 0.0F
                                : ImGui::GetTextLineHeightWithSpacing() * 4.0F;
    if (beginTable("packs", 4, flags, ImVec2(0.0F, -errors_height)))
    {
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn(
            "Pack / model", ImGuiTableColumnFlags_WidthStretch, 1.6F);
        ImGui::TableSetupColumn("Version", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableSetupColumn(
            "License", ImGuiTableColumnFlags_WidthStretch, 1.2F);
        ImGui::TableSetupColumn("Models", ImGuiTableColumnFlags_WidthFixed);
        ImGui::TableHeadersRow();
        for (robotik::AssetPack const& pack : assets.packs())
        {
            bool const used = usedByRun(p_app, package, pack);
            if (only_used && !used)
            {
                continue;
            }
            bool model_match = false;
            for (auto const& [key, file] : pack.models)
            {
                model_match =
                    model_match || matches(pack.id + "." + key, filter);
            }
            if (!matches(pack.id, filter) &&
                !matches(pack.description, filter) && !model_match)
            {
                continue;
            }
            ImGui::PushID(pack.id.c_str());
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            // A filter opens the packs, to show the models that match.
            if (filter[0] != '\0')
            {
                ImGui::SetNextItemOpen(true, ImGuiCond_Always);
            }
            bool const open =
                ImGui::TreeNodeEx("##pack",
                                  ImGuiTreeNodeFlags_SpanFullWidth,
                                  "%s",
                                  pack.id.c_str());
            if (!pack.description.empty())
            {
                ImGui::SetItemTooltip("%s", pack.description.c_str());
            }
            if (used)
            {
                ImGui::SameLine();
                badge("in use", GREEN);
            }
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(robotik::toString(pack.version).c_str());
            ImGui::TableNextColumn();
            licenseBadge(pack);
            ImGui::TableNextColumn();
            ImGui::Text("%zu", pack.models.size());
            if (open)
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                if (!pack.description.empty())
                {
                    ImGui::TextWrapped("%s", pack.description.c_str());
                }
                if (!pack.source.empty())
                {
                    ImGui::TextDisabled("Source:");
                    ImGui::SameLine();
                    ImGui::TextLinkOpenURL(pack.source.c_str());
                }
                ImGui::TextDisabled("Folder:");
                ImGui::SameLine();
                if (ImGui::SmallButton("Copy path"))
                {
                    ImGui::SetClipboardText(pack.root.string().c_str());
                }
                ImGui::SetItemTooltip("%s", pack.root.string().c_str());
                for (auto const& [key, file] : pack.models)
                {
                    std::string const id = pack.id + "." + key;
                    if (!matches(id, filter) && !matches(pack.id, filter))
                    {
                        continue;
                    }
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::TreeNodeEx(id.c_str(),
                                      ImGuiTreeNodeFlags_Leaf |
                                          ImGuiTreeNodeFlags_Bullet |
                                          ImGuiTreeNodeFlags_NoTreePushOnOpen |
                                          ImGuiTreeNodeFlags_SpanFullWidth);
                    if (ImGui::IsItemClicked())
                    {
                        openPreview(p_app,
                                    file.is_absolute() ? file
                                                       : pack.root / file);
                    }
                    if (ImGui::BeginPopupContextItem())
                    {
                        if (ImGui::MenuItem("Copy model id"))
                        {
                            ImGui::SetClipboardText(id.c_str());
                        }
                        ImGui::EndPopup();
                    }
                    ImGui::SetItemTooltip(
                        "Click to see the file, right click to copy the id.");
                    ImGui::TableNextColumn();
                    ImGui::TableNextColumn();
                    ImGui::TextDisabled("%s", file.string().c_str());
                }
                assetFiles(p_app, pack, filter);
                ImGui::TreePop();
            }
            ImGui::PopID();
        }
        endTable();
    }
    if (!assets.errors().empty())
    {
        ImGui::SeparatorText("Problems");
        for (std::string const& error : assets.errors())
        {
            ImGui::TextColored(RED, "%s", error.c_str());
        }
    }
    ImGui::End();
}

void experimentView(App& p_app)
{
    UiState& ui = p_app.ui;
    if (ui.experiment_path.empty())
    {
        emptyState("No experiment.\nAn experiment runs variants of the "
                   "scenarios over several "
                   "seeds. Click one in the Files of the Plugin panel.");
        return;
    }
    if (!ui.experiment)
    {
        ImGui::TextColored(RED, "%s", ui.experiment_error.c_str());
        return;
    }
    robotik::Experiment const& experiment = *ui.experiment;

    ImGui::PushFont(nullptr, ImGui::GetStyle().FontSizeBase * 1.2F);
    ImGui::TextUnformatted(experiment.id.c_str());
    ImGui::PopFont();
    ImGui::SameLine();
    if (ImGui::SmallButton("Copy command"))
    {
        std::string const command = "./build/Robotik-Headless " +
                                    experiment.file.string() + " --report " +
                                    experiment.id + ".json";
        ImGui::SetClipboardText(command.c_str());
    }
    ImGui::SameLine();
    helpMarker("An experiment runs every case with every seed, without window, "
               "and writes a "
               "report. Copy the command and run it in a terminal.");
    if (!experiment.description.empty())
    {
        ImGui::TextWrapped("%s", experiment.description.c_str());
    }
    if (beginFields("experiment"))
    {
        std::string seeds;
        for (std::uint64_t const seed : experiment.seeds)
        {
            seeds += (seeds.empty() ? "" : ", ") + std::to_string(seed);
        }
        field("seeds",
              "%s",
              seeds.empty() ? "the seed of each scenario" : seeds.c_str());
        field("runs",
              "%zu",
              experiment.cases.size() *
                  std::max<std::size_t>(experiment.seeds.size(), 1U));
        if (!experiment.world.empty())
        {
            field("world",
                  "%s (replaces the world of every scenario)",
                  experiment.world.filename().string().c_str());
        }
        endTable();
    }

    ImGui::SeparatorText("Cases");
    experimentCases(p_app, experiment);

    if (!experiment.assertions.empty() || !experiment.metrics.empty())
    {
        ImGui::SeparatorText("Evaluation");
        for (robotik::AssertionSpec const& assertion : experiment.assertions)
        {
            std::string text = assertion.condition;
            for (std::string const& step : assertion.sequence)
            {
                text += (text.empty() ? "" : " then ") + step;
            }
            ImGui::BulletText("%s", text.c_str());
            ImGui::SameLine();
            ImGui::TextDisabled("(%s)", robotik::toString(assertion.phase));
        }
        if (!experiment.metrics.empty())
        {
            std::string metrics;
            for (std::string const& metric : experiment.metrics)
            {
                metrics += (metrics.empty() ? "" : ", ") + metric;
            }
            ImGui::TextDisabled("Recorded:");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", metrics.c_str());
        }
    }
}
