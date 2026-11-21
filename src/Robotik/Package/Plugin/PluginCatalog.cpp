// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Package/Plugin/PluginCatalog.hpp"

#include "Robotik/Robot/Assets/AssetResolver.hpp"
#include "Robotik/Common/Config.hpp"
#include "Robotik/Common/File.hpp"
#include "Robotik/Package/Plugin/PluginABI.h"
#include "Robotik/Package/Scenario/FileRole.hpp"


#include <algorithm>
#include <cstdlib>
#include <optional>
#include <system_error>

namespace robotik
{

/// True when the map @p_node writes the key @p_key, even with no value.
static bool writes(ConfigNode const& p_node, std::string_view p_key)
{
    return std::ranges::any_of(p_node.entries(),
                               [p_key](auto const& p_entry)
                               { return p_entry.first == p_key; });
}

/// Refuses the keys of a plugin.yaml that the catalog does not read (a typo
/// would silently drop a scenario, a pack or a library).
static Result<void> checkManifestKeys(ConfigNode const& p_root)
{
    ROBOTIK_TRY(p_root.checkKeys({ "plugin",
                                   "library",
                                   "resources",
                                   "dependencies",
                                   "host",
                                   "scenarios",
                                   "experiments" },
                                 "plugin.yaml"));
    ROBOTIK_TRY(p_root.at("plugin").checkKeys(
        { "id", "name", "category", "description", "version", "api_version" },
        "plugin"));
    ROBOTIK_TRY(p_root.at("library").checkKeys({ "entrypoint" }, "library"));
    ConfigNode const& resources = p_root.at("resources");
    ROBOTIK_TRY(resources.checkKeys({ "directory", "packs" }, "resources"));
    for (ConfigNode const& pack : resources.at("packs").items())
    {
        ROBOTIK_TRY(pack.checkKeys({ "id", "version" }, "resources.packs"));
    }
    ConfigNode const& dependencies = p_root.at("dependencies");
    ROBOTIK_TRY(dependencies.checkKeys({ "plugins", "system", "thirdparties" },
                                       "dependencies"));
    for (char const* list : { "system", "thirdparties" })
    {
        for (ConfigNode const& item : dependencies.at(list).items())
        {
            ROBOTIK_TRY(item.checkKeys(
                { "name", "repo", "url", "license", "description" },
                std::string("dependencies.") + list));
        }
    }
    ConfigNode const& host = p_root.at("host");
    ROBOTIK_TRY(host.checkKeys({ "graphics", "clock", "view" }, "host"));
    ROBOTIK_TRY(host.at("view").checkKeys(
        { "eye", "target", "distance", "clip", "floor", "background" },
        "host.view"));
    for (char const* section : { "scenarios", "experiments" })
    {
        ConfigNode const& node = p_root.at(section);
        ROBOTIK_TRY(node.checkKeys({ "directory", "entries" }, section));
        for (ConfigNode const& entry : node.at("entries").items())
        {
            ROBOTIK_TRY(entry.checkKeys(
                { "id", "name", "file", "menu", "clock" },
                std::string(section) + " entry '" + entry.text("id") + "'"));
        }
    }
    return success();
}

/// True when the text is a plausible shared-library file name.
static bool libraryName(std::string const& p_name)
{
    if (p_name.size() < 4U || !p_name.ends_with(".so"))
    {
        return false;
    }
    for (char const character : p_name)
    {
        bool const ok = (character >= 'a' && character <= 'z') ||
                        (character >= 'A' && character <= 'Z') ||
                        (character >= '0' && character <= '9') ||
                        character == '.' || character == '_' ||
                        character == '+' || character == '-';
        if (!ok)
        {
            return false;
        }
    }
    return true;
}

/// A relative name. Whether it stays in the package is checked on its
/// canonical path (".." and symbolic links are followed).
static bool relativeName(std::string const& p_file)
{
    if (p_file.empty())
    {
        return false;
    }
    std::filesystem::path const path(p_file);
    if (path.is_absolute())
    {
        return false;
    }
    return !path.has_root_name() && !path.has_root_directory();
}

/// Canonical path, or the absolute path when canonicalization fails.
static std::filesystem::path canonicalOf(std::filesystem::path const& p_path)
{
    std::error_code error;
    std::filesystem::path const canonical =
        std::filesystem::weakly_canonical(p_path, error);
    return error ? std::filesystem::absolute(p_path) : canonical;
}

/// True when a file stays inside a directory once links are resolved.
static bool contained(std::filesystem::path const& p_root,
                      std::filesystem::path const& p_file)
{
    std::filesystem::path const root = canonicalOf(p_root);
    std::filesystem::path const file = canonicalOf(p_file);
    std::filesystem::path const relative = file.lexically_relative(root);
    if (relative.empty())
    {
        return false;
    }
    for (std::filesystem::path const& part : relative)
    {
        if (part == "..")
        {
            return false;
        }
    }
    return true;
}

/// Beside the manifest (staged package), else next to the executable (a
/// package read from the sources while its library sits in build/).
static std::filesystem::path resolveLibrary(std::filesystem::path const& p_root,
                                            std::string const& p_name)
{
    std::filesystem::path const beside = p_root / p_name;
    std::error_code error;
    if (std::filesystem::is_regular_file(beside, error))
    {
        return beside;
    }
    if (std::filesystem::path const exe = executableDirectory(); !exe.empty())
    {
        std::filesystem::path const candidate = exe / p_name;
        if (std::filesystem::is_regular_file(candidate, error))
        {
            return candidate;
        }
    }
    return beside;
}

/// Turns a file stem into a title; underscores and dashes become spaces.
static std::string titleFromStem(std::string p_stem)
{
    for (char& character : p_stem)
    {
        if (character == '_' || character == '-')
        {
            character = ' ';
        }
    }
    return p_stem;
}

/// Entries of a "scenarios:" or "experiments:" section: listed, or every
/// runnable file of the folder (*.scenario.yaml, *.rl.yaml,
/// *.experiment.yaml, or a YAML file without a role). Files must stay in
/// the package. False, with @p_why set, on an invalid section or file.
static bool readEntries(ConfigNode const& p_section,
                        std::filesystem::path const& p_root,
                        bool p_owns_clock,
                        std::vector<PluginScenario>& p_out,
                        std::string& p_why)
{
    if (p_section.isNull())
    {
        return true;
    }
    std::string const folder_name = p_section.text("directory");
    if (!folder_name.empty() && folder_name != "." &&
        (!relativeName(folder_name) ||
         !contained(p_root, p_root / folder_name)))
    {
        p_why = "a directory must stay inside the package";
        return false;
    }
    std::filesystem::path const folder =
        folder_name.empty() ? p_root
                            : (p_root / folder_name).lexically_normal();
    if (writes(p_section, "entries"))
    {
        for (ConfigNode const& item : p_section.at("entries").items())
        {
            PluginScenario entry;
            entry.id = item.text("id");
            entry.name = item.text("name");
            std::string const file = item.text("file");
            if (entry.id.empty() || entry.name.empty() || file.empty())
            {
                p_why = "invalid entry (id, name and file are required)";
                return false;
            }
            entry.file = (folder / file).lexically_normal();
            entry.menu = item.text("menu");
            std::string const clock = item.text("clock");
            entry.owns_clock = clock.empty() ? p_owns_clock : clock == "plugin";
            p_out.push_back(std::move(entry));
        }
    }
    else
    {
        std::vector<std::filesystem::path> files;
        std::error_code error;
        std::filesystem::directory_iterator cursor(folder, error);
        for (; !error && cursor != std::filesystem::directory_iterator();
             cursor.increment(error))
        {
            std::filesystem::path const file = cursor->path();
            // Trees, domains and trajectories live beside the scenarios but
            // are not runnable (FileRole.hpp).
            if (runnableRole(file))
            {
                files.push_back(file);
            }
        }
        std::ranges::sort(files);
        for (std::filesystem::path const& file : files)
        {
            PluginScenario entry;
            entry.id = roleStem(file);
            entry.name = titleFromStem(entry.id);
            entry.file = file;
            entry.owns_clock = p_owns_clock;
            p_out.push_back(std::move(entry));
        }
    }
    for (PluginScenario const& entry : p_out)
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(entry.file, error) ||
            !contained(p_root, entry.file))
        {
            p_why = entry.file.string() +
                    " is outside the plugin package or missing";
            return false;
        }
    }
    return true;
}

/// An entry of dependencies.system or dependencies.thirdparties: a map, or
/// a scalar (the repo or the name).
static ThirdParty libraryOf(ConfigNode const& p_item)
{
    ThirdParty entry;
    std::string repo = p_item.isMap() ? p_item.text("repo") : std::string{};
    entry.name = p_item.isMap() ? p_item.text("name") : p_item.scalar();
    if (!repo.empty())
    {
        if (std::size_t const at = repo.find('@'); at != std::string::npos)
        {
            entry.reference = repo.substr(at + 1U);
            repo.resize(at);
        }
        if (std::size_t const option = repo.find(':');
            option != std::string::npos)
        {
            repo.resize(option);
        }
        entry.name = repo;
        entry.url = "https://github.com/" + repo;
    }
    if (p_item.isMap())
    {
        if (std::string const url = p_item.text("url"); !url.empty())
        {
            entry.url = url;
        }
        entry.license = p_item.text("license");
        entry.description = p_item.text("description");
    }
    return entry;
}

void PluginCatalog::scan(std::filesystem::path const& p_extra)
{
    std::vector<std::filesystem::path> roots;
    if (!p_extra.empty())
    {
        roots.push_back(p_extra);
    }
    if (char const* env = std::getenv("ROBOTIK_PLUGINS"); env != nullptr)
    {
        std::string_view paths(env);
        while (!paths.empty())
        {
            std::size_t const colon = paths.find(':');
            if (std::string_view const item = paths.substr(0, colon);
                !item.empty())
            {
                roots.emplace_back(item);
            }
            paths = colon == std::string_view::npos ? std::string_view{}
                                                    : paths.substr(colon + 1U);
        }
    }
    // Build tree (build/Robotik-Headless, build/plugins) and install prefix
    // (bin/, lib/robotik/plugins).
    if (std::filesystem::path const exe = executableDirectory(); !exe.empty())
    {
        roots.push_back(exe / "plugins");
        roots.push_back(exe.parent_path() / "lib" / "robotik" / "plugins");
    }
    scan(roots);
}

PluginPackage const* PluginCatalog::add(std::filesystem::path const& p_scenario)
{
    std::filesystem::path directory = canonicalOf(p_scenario).parent_path();
    for (int hop = 0; hop < 3 && !directory.empty(); ++hop)
    {
        std::error_code error;
        if (std::filesystem::is_regular_file(directory / "plugin.yaml", error))
        {
            loadPackage(directory);
            return findScenario(p_scenario);
        }
        directory = directory.parent_path();
    }
    return nullptr;
}

void PluginCatalog::scan(std::vector<std::filesystem::path> const& p_roots)
{
    m_packages.clear();
    m_errors.clear();
    m_roots_seen.clear();
    for (std::filesystem::path const& root : p_roots)
    {
        consider(root);
    }
}

PluginPackage const* PluginCatalog::findPackage(std::string_view p_id) const
{
    for (PluginPackage const& package : m_packages)
    {
        if (package.id == p_id)
        {
            return &package;
        }
    }
    return nullptr;
}

PluginPackage const*
PluginCatalog::findScenario(std::filesystem::path const& p_scenario) const
{
    std::filesystem::path const wanted = canonicalOf(p_scenario);
    for (PluginPackage const& package : m_packages)
    {
        for (PluginScenario const& scenario : package.scenarios)
        {
            if (canonicalOf(scenario.file) == wanted)
            {
                return &package;
            }
        }
    }
    return nullptr;
}

PluginMatch
PluginCatalog::matchScenario(std::filesystem::path const& p_scenario) const
{
    if (PluginPackage const* exact = findScenario(p_scenario))
    {
        return { exact, scenarioOf(*exact, p_scenario) };
    }
    std::string const stem = roleStem(p_scenario);
    if (stem.empty())
    {
        return {};
    }
    PluginMatch found;
    int matches = 0;
    for (PluginPackage const& package : m_packages)
    {
        for (PluginScenario const& scenario : package.scenarios)
        {
            if (scenario.id == stem)
            {
                found = { &package, &scenario };
                ++matches;
            }
        }
    }
    return matches == 1 ? found : PluginMatch{};
}

PluginScenario const*
PluginCatalog::scenarioOf(PluginPackage const& p_package,
                          std::filesystem::path const& p_scenario) const
{
    std::filesystem::path const wanted = canonicalOf(p_scenario);
    for (PluginScenario const& scenario : p_package.scenarios)
    {
        if (canonicalOf(scenario.file) == wanted)
        {
            return &scenario;
        }
    }
    return nullptr;
}

void PluginCatalog::consider(std::filesystem::path const& p_directory)
{
    std::error_code error;
    if (!std::filesystem::is_directory(p_directory, error))
    {
        return;
    }
    if (std::filesystem::is_regular_file(p_directory / "plugin.yaml", error))
    {
        loadPackage(p_directory);
        return;
    }
    std::filesystem::directory_iterator cursor(p_directory, error);
    if (error)
    {
        return;
    }
    for (; cursor != std::filesystem::directory_iterator();
         cursor.increment(error))
    {
        if (error)
        {
            break;
        }
        std::filesystem::path const child = cursor->path();
        if (std::filesystem::is_directory(child, error) &&
            std::filesystem::is_regular_file(child / "plugin.yaml", error))
        {
            loadPackage(child);
        }
    }
}

void PluginCatalog::loadPackage(std::filesystem::path const& p_root)
{
    std::filesystem::path const root = canonicalOf(p_root);
    for (std::filesystem::path const& seen : m_roots_seen)
    {
        if (seen == root)
        {
            return;
        }
    }

    std::filesystem::path const manifest = p_root / "plugin.yaml";
    auto reject = [&](std::string const& p_why)
    { m_errors.push_back(manifest.string() + ": " + p_why); };
    Result<ConfigNode> const parsed = ConfigNode::load(manifest);
    if (!parsed)
    {
        m_errors.push_back(parsed.error());
        return;
    }
    ConfigNode const& node = parsed.value();
    if (Result<void> const checked = checkManifestKeys(node); !checked)
    {
        reject(checked.error());
        return;
    }

    // --- plugin: identity ---------------------------------------------------
    ConfigNode const& plugin = node.at("plugin");
    PluginPackage package;
    package.root = root;
    package.id = plugin.text("id");
    package.name = plugin.text("name");
    package.category = plugin.text("category");
    package.description = plugin.text("description");
    if (package.category.empty())
    {
        package.category = "Demo";
    }
    std::optional<Version> const version = parseVersion(plugin.text("version"));
    if (package.id.empty() || package.name.empty() || !version)
    {
        reject("plugin.id, plugin.name and plugin.version are required");
        return;
    }
    package.version = *version;
    package.api_version = static_cast<std::uint32_t>(
        plugin.at("api_version").asNumber().value_or(0.0));
    if (package.api_version != ROBOTIK_PLUGIN_ABI_VERSION)
    {
        reject("built for plugin API " + std::to_string(package.api_version) +
               ", this host speaks " +
               std::to_string(ROBOTIK_PLUGIN_ABI_VERSION));
        return;
    }

    // --- library ------------------------------------------------------------
    std::string const library = node.at("library").text("entrypoint");
    if (!libraryName(library))
    {
        reject("library.entrypoint must be a file name ending in .so");
        return;
    }
    package.library = resolveLibrary(p_root, library);

    // --- resources: own pack and required packs ------------------------------
    ConfigNode const& resources = node.at("resources");
    if (std::string const directory = resources.text("directory");
        !directory.empty())
    {
        if (!relativeName(directory) || !contained(p_root, p_root / directory))
        {
            reject("resources.directory must stay inside the package");
            return;
        }
        // Canonical, like the root of the packs it holds.
        package.resources = canonicalOf(p_root / directory);
    }
    for (ConfigNode const& item : resources.at("packs").items())
    {
        PackRequirement requirement;
        requirement.id = item.isMap() ? item.text("id") : item.scalar();
        std::string const wanted = item.isMap() ? item.text("version") : "0";
        std::optional<Version> const minimum =
            parseVersion(wanted.empty() ? "0" : wanted);
        if (requirement.id.empty() || !minimum)
        {
            reject("resources.packs entries need an id and a version such as "
                   "\"1.0\"");
            return;
        }
        requirement.version = *minimum;
        package.packs.push_back(std::move(requirement));
    }

    // --- dependencies -------------------------------------------------------
    ConfigNode const& dependencies = node.at("dependencies");
    for (ConfigNode const& item : dependencies.at("plugins").items())
    {
        package.plugins.push_back(item.scalar());
    }
    for (ConfigNode const& item : dependencies.at("system").items())
    {
        package.system.push_back(libraryOf(item));
    }
    for (ConfigNode const& item : dependencies.at("thirdparties").items())
    {
        package.thirdparties.push_back(libraryOf(item));
    }

    // --- host hints ---------------------------------------------------------
    ConfigNode const& host = node.at("host");
    Result<bool> const graphics = host.flag("graphics", false);
    if (!graphics)
    {
        reject("host." + graphics.error());
        return;
    }
    package.graphics = graphics.value();
    package.owns_clock = host.text("clock") == "plugin";
    if (host.has("view"))
    {
        ConfigNode const& view = host.at("view");
        auto read =
            [](ConfigNode const& p_node, float* p_out, std::size_t p_size)
        {
            if (!p_node.isSequence())
            {
                return false;
            }
            std::size_t index = 0;
            for (ConfigNode const& item : p_node.items())
            {
                if (index < p_size)
                {
                    p_out[index] = static_cast<float>(
                        item.asNumber().value_or(p_out[index]));
                    ++index;
                }
            }
            return true;
        };
        read(view.at("eye"), package.eye, 3U);
        read(view.at("target"), package.target, 3U);
        read(view.at("distance"), package.distance, 2U);
        read(view.at("clip"), package.clip, 2U);
        Result<bool> const floor = view.flag("floor", true);
        if (!floor)
        {
            reject("host.view." + floor.error());
            return;
        }
        package.floor = floor.value();
        package.has_background =
            read(view.at("background"), package.background, 3U);
        package.has_view = true;
    }

    // --- scenarios and experiments -----------------------------------------
    std::string why;
    if (!readEntries(node.at("scenarios"),
                     p_root,
                     package.owns_clock,
                     package.scenarios,
                     why) ||
        !readEntries(node.at("experiments"),
                     p_root,
                     package.owns_clock,
                     package.experiments,
                     why))
    {
        if (!why.empty())
        {
            reject(why);
        }
        return;
    }
    if (package.scenarios.empty())
    {
        reject("no scenario file");
        return;
    }
    m_roots_seen.push_back(root);
    m_packages.push_back(std::move(package));
}

} // namespace robotik
