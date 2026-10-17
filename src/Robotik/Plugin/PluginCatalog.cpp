// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginCatalog.hpp"

#include "Robotik/Assets/AssetResolver.hpp"
#include "Robotik/Plugin/PluginABI.h"

#include "BlackThorn/Builder/Yaml.hpp"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <system_error>

namespace robotik
{

namespace
{

std::string field(bt::YamlNode const& p_node, std::string_view p_key)
{
    bt::YamlNode const child = p_node.child(p_key);
    return child.valid() ? child.scalar() : std::string{};
}

bool libraryName(std::string const& p_name)
{
    if (p_name.size() < 4u || !p_name.ends_with(".so"))
    {
        return false;
    }
    for (char const character : p_name)
    {
        bool const ok = (character >= 'a' && character <= 'z') ||
                        (character >= 'A' && character <= 'Z') ||
                        (character >= '0' && character <= '9') || character == '.' ||
                        character == '_' || character == '+' || character == '-';
        if (!ok)
        {
            return false;
        }
    }
    return true;
}

//! A relative name. Whether it stays in the package is checked on its
//! canonical path (".." and symbolic links are followed).
bool relativeName(std::string const& p_file)
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

std::filesystem::path canonicalOf(std::filesystem::path const& p_path)
{
    std::error_code error;
    std::filesystem::path const canonical = std::filesystem::weakly_canonical(p_path, error);
    return error ? std::filesystem::absolute(p_path) : canonical;
}

bool contained(std::filesystem::path const& p_root, std::filesystem::path const& p_file)
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

//! Beside the manifest (staged package), else next to the executable (a
//! package read from the sources while its library sits in build/).
std::filesystem::path resolveLibrary(std::filesystem::path const& p_root,
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

std::string titleFromStem(std::string p_stem)
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

//! Entries of a "scenarios:" or "experiments:" section: listed, or every
//! *.yml / *.yaml of the folder (behavior trees excepted). Files must stay in
//! the package. False, with @p_why set, on an invalid section or file.
bool readEntries(bt::YamlNode const& p_section,
                 std::filesystem::path const& p_root,
                 bool p_owns_clock,
                 std::vector<PluginScenario>& p_out,
                 std::string& p_why)
{
    if (!p_section.valid())
    {
        return true;
    }
    std::string const folder_name = field(p_section, "directory");
    if (!folder_name.empty() && folder_name != "." &&
        (!relativeName(folder_name) || !contained(p_root, p_root / folder_name)))
    {
        p_why = "a directory must stay inside the package";
        return false;
    }
    std::filesystem::path const folder =
        folder_name.empty() ? p_root : (p_root / folder_name).lexically_normal();
    if (p_section.hasKey("entries"))
    {
        bool entries_ok = true;
        p_section.child("entries").forEachSeq([&](bt::YamlNode p_item) {
            PluginScenario entry;
            entry.id = field(p_item, "id");
            entry.name = field(p_item, "name");
            std::string const file = field(p_item, "file");
            if (entry.id.empty() || entry.name.empty() || file.empty())
            {
                entries_ok = false;
                return;
            }
            entry.file = (folder / file).lexically_normal();
            entry.menu = field(p_item, "menu");
            std::string const clock = field(p_item, "clock");
            entry.owns_clock = clock.empty() ? p_owns_clock : clock == "plugin";
            p_out.push_back(std::move(entry));
        });
        if (!entries_ok)
        {
            p_why = "invalid entry (id, name and file are required)";
            return false;
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
            std::string const name = file.filename().string();
            bool const yaml = name.ends_with(".yml") || name.ends_with(".yaml");
            // Behavior trees live beside the scenarios but are not runnable.
            if (yaml && !name.ends_with(".bt.yml") && !name.ends_with(".bt.yaml"))
            {
                files.push_back(file);
            }
        }
        std::sort(files.begin(), files.end());
        for (std::filesystem::path const& file : files)
        {
            PluginScenario entry;
            entry.id = file.stem().string();
            entry.name = titleFromStem(entry.id);
            entry.file = file;
            entry.owns_clock = p_owns_clock;
            p_out.push_back(std::move(entry));
        }
    }
    for (PluginScenario const& entry : p_out)
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(entry.file, error) || !contained(p_root, entry.file))
        {
            p_why = entry.file.string() + " is outside the plugin package or missing";
            return false;
        }
    }
    return true;
}

//! An entry of dependencies.system or dependencies.thirdparties: a map, or
//! a scalar (the repo or the name).
ThirdParty libraryOf(bt::YamlNode const& p_item)
{
    ThirdParty entry;
    std::string repo = p_item.isMap() ? field(p_item, "repo") : std::string{};
    entry.name = p_item.isMap() ? field(p_item, "name") : p_item.scalar();
    if (!repo.empty())
    {
        if (std::size_t const at = repo.find('@'); at != std::string::npos)
        {
            entry.reference = repo.substr(at + 1u);
            repo.resize(at);
        }
        if (std::size_t const option = repo.find(':'); option != std::string::npos)
        {
            repo.resize(option);
        }
        entry.name = repo;
        entry.url = "https://github.com/" + repo;
    }
    if (p_item.isMap())
    {
        if (std::string const url = field(p_item, "url"); !url.empty())
        {
            entry.url = url;
        }
        entry.license = field(p_item, "license");
        entry.description = field(p_item, "description");
    }
    return entry;
}

} // namespace

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
            if (std::string_view const item = paths.substr(0, colon); !item.empty())
            {
                roots.emplace_back(item);
            }
            paths = colon == std::string_view::npos ? std::string_view{}
                                                    : paths.substr(colon + 1u);
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

PluginPackage const* PluginCatalog::findScenario(std::filesystem::path const& p_scenario) const
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

PluginMatch PluginCatalog::matchScenario(std::filesystem::path const& p_scenario) const
{
    if (PluginPackage const* exact = findScenario(p_scenario))
    {
        return { exact, scenarioOf(*exact, p_scenario) };
    }
    std::string const stem = p_scenario.stem().string();
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

PluginScenario const* PluginCatalog::scenarioOf(PluginPackage const& p_package,
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
    for (; cursor != std::filesystem::directory_iterator(); cursor.increment(error))
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
    auto parsed = bt::YamlDocument::parseFile(manifest.string());
    if (!parsed)
    {
        m_errors.push_back(manifest.string() + ": " + parsed.getError());
        return;
    }
    auto reject = [&](std::string const& p_why) {
        m_errors.push_back(manifest.string() + ": " + p_why);
    };
    bt::YamlNode const node = parsed.getValue().root();

    // --- plugin: identity ---------------------------------------------------
    bt::YamlNode const plugin = node.child("plugin");
    PluginPackage package;
    package.root = root;
    package.id = field(plugin, "id");
    package.name = field(plugin, "name");
    package.category = field(plugin, "category");
    package.description = field(plugin, "description");
    if (package.category.empty())
    {
        package.category = "Demo";
    }
    std::optional<Version> const version = Version::parse(field(plugin, "version"));
    if (package.id.empty() || package.name.empty() || !version)
    {
        reject("plugin.id, plugin.name and plugin.version are required");
        return;
    }
    package.version = *version;
    package.api_version =
        static_cast<std::uint32_t>(plugin.child("api_version").asDouble().value_or(0.0));
    if (package.api_version != ROBOTIK_PLUGIN_ABI_VERSION)
    {
        reject("built for plugin API " + std::to_string(package.api_version) +
               ", this host speaks " + std::to_string(ROBOTIK_PLUGIN_ABI_VERSION));
        return;
    }

    // --- library ------------------------------------------------------------
    std::string const library = field(node.child("library"), "entrypoint");
    if (!libraryName(library))
    {
        reject("library.entrypoint must be a file name ending in .so");
        return;
    }
    package.library = resolveLibrary(p_root, library);

    // --- resources: own pack and required packs ------------------------------
    bt::YamlNode const resources = node.child("resources");
    if (std::string const directory = field(resources, "directory"); !directory.empty())
    {
        if (!relativeName(directory) || !contained(p_root, p_root / directory))
        {
            reject("resources.directory must stay inside the package");
            return;
        }
        // Canonical, like the root of the packs it holds.
        package.resources = canonicalOf(p_root / directory);
    }
    bool packs_ok = true;
    resources.child("packs").forEachSeq([&](bt::YamlNode p_item) {
        PackRequirement requirement;
        requirement.id = p_item.isMap() ? field(p_item, "id") : p_item.scalar();
        std::string const wanted = p_item.isMap() ? field(p_item, "version") : "0";
        std::optional<Version> const minimum = Version::parse(wanted.empty() ? "0" : wanted);
        if (requirement.id.empty() || !minimum)
        {
            packs_ok = false;
            return;
        }
        requirement.version = *minimum;
        package.packs.push_back(std::move(requirement));
    });
    if (!packs_ok)
    {
        reject("resources.packs entries need an id and a version such as \"1.0\"");
        return;
    }

    // --- dependencies -------------------------------------------------------
    bt::YamlNode const dependencies = node.child("dependencies");
    dependencies.child("plugins").forEachSeq(
        [&](bt::YamlNode p_item) { package.plugins.push_back(p_item.scalar()); });
    dependencies.child("system").forEachSeq(
        [&](bt::YamlNode p_item) { package.system.push_back(libraryOf(p_item)); });
    dependencies.child("thirdparties").forEachSeq(
        [&](bt::YamlNode p_item) { package.thirdparties.push_back(libraryOf(p_item)); });

    // --- host hints ---------------------------------------------------------
    bt::YamlNode const host = node.child("host");
    package.graphics = host.child("graphics").asBool().value_or(false);
    package.owns_clock = field(host, "clock") == "plugin";
    if (host.hasKey("view"))
    {
        bt::YamlNode const view = host.child("view");
        auto read3 = [](bt::YamlNode const& p_node, float p_out[3]) {
            if (!p_node.isSeq())
            {
                return;
            }
            int index = 0;
            p_node.forEachSeq([&](bt::YamlNode p_item) {
                if (index < 3)
                {
                    p_out[index] = static_cast<float>(p_item.asDouble().value_or(p_out[index]));
                    ++index;
                }
            });
        };
        read3(view.child("eye"), package.eye);
        read3(view.child("target"), package.target);
        package.has_view = true;
    }

    // --- scenarios and experiments -----------------------------------------
    std::string why;
    if (!readEntries(node.child("scenarios"), p_root, package.owns_clock, package.scenarios, why) ||
        !readEntries(node.child("experiments"), p_root, package.owns_clock, package.experiments, why))
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
