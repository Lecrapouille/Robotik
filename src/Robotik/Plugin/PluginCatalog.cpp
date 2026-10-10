// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Plugin/PluginCatalog.hpp"

#include "BlackThorn/Builder/Yaml.hpp"

#include <algorithm>
#include <cstdlib>
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

bool relativeInside(std::string const& p_file)
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
    for (std::filesystem::path const& part : path)
    {
        if (part == "..")
        {
            return false;
        }
    }
    return true;
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

std::filesystem::path resolveLibrary(std::filesystem::path const& p_root,
                                     std::string const& p_name)
{
    std::filesystem::path const beside = p_root / p_name;
    std::error_code error;
    if (std::filesystem::is_regular_file(beside, error))
    {
        return beside;
    }
    for (char const* build : { "build", "../build" })
    {
        std::filesystem::path const candidate(std::filesystem::path(build) / p_name);
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

} // namespace

void PluginCatalog::scan(std::filesystem::path const& p_extra)
{
    std::vector<std::filesystem::path> roots;
    if (!p_extra.empty())
    {
        roots.push_back(p_extra);
    }
    if (char const* env = std::getenv("ROBOTIK_PLUGINS"))
    {
        if (env[0] != '\0')
        {
            roots.emplace_back(env);
        }
    }
    roots.emplace_back("build/plugins");
    roots.emplace_back("plugins");
    roots.emplace_back("../build/plugins");
    scan(roots);
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
    bt::YamlNode const node = parsed.getValue().root();
    PluginPackage package;
    package.root = root;
    package.id = field(node, "id");
    package.name = field(node, "name");
    package.version = field(node, "version");
    package.category = field(node, "category");
    if (package.category.empty())
    {
        package.category = "Demo";
    }
    if (node.hasKey("graphics"))
    {
        package.graphics = node.child("graphics").asBool().value_or(false);
    }
    if (field(node, "clock") == "plugin")
    {
        package.owns_clock = true;
    }
    if (node.hasKey("view"))
    {
        bt::YamlNode const view = node.child("view");
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
    std::string const library = field(node, "library");
    if (package.id.empty() || package.name.empty() || package.version.empty())
    {
        m_errors.push_back(manifest.string() + ": id, name and version are required");
        return;
    }
    if (!libraryName(library))
    {
        m_errors.push_back(manifest.string() +
                           ": library must be a file name ending in .so");
        return;
    }
    package.library = resolveLibrary(p_root, library);

    if (node.hasKey("scenarios"))
    {
        bt::YamlNode const scenarios = node.child("scenarios");
        if (!scenarios.isSeq())
        {
            m_errors.push_back(manifest.string() + ": scenarios must be a list");
            return;
        }
        scenarios.forEachSeq([&](bt::YamlNode p_item) {
            PluginScenario scenario;
            scenario.id = field(p_item, "id");
            scenario.name = field(p_item, "name");
            std::string const file = field(p_item, "file");
            if (scenario.id.empty() || scenario.name.empty() || !relativeInside(file))
            {
                m_errors.push_back(manifest.string() + ": invalid scenario entry");
                return;
            }
            scenario.file = (p_root / file).lexically_normal();
            package.scenarios.push_back(std::move(scenario));
        });
    }
    else
    {
        std::filesystem::path const folder = p_root / "scenarios";
        std::error_code error;
        if (std::filesystem::is_directory(folder, error))
        {
            std::vector<std::filesystem::path> files;
            std::filesystem::directory_iterator cursor(folder, error);
            for (; !error && cursor != std::filesystem::directory_iterator();
                 cursor.increment(error))
            {
                std::filesystem::path const file = cursor->path();
                std::string const extension = file.extension().string();
                if (extension == ".yml" || extension == ".yaml")
                {
                    files.push_back(file);
                }
            }
            std::sort(files.begin(), files.end());
            for (std::filesystem::path const& file : files)
            {
                PluginScenario scenario;
                scenario.id = file.stem().string();
                scenario.name = titleFromStem(scenario.id);
                scenario.file = file;
                package.scenarios.push_back(std::move(scenario));
            }
        }
    }

    if (package.scenarios.empty())
    {
        m_errors.push_back(manifest.string() + ": no scenario file");
        return;
    }
    for (PluginScenario const& scenario : package.scenarios)
    {
        std::error_code error;
        if (!std::filesystem::is_regular_file(scenario.file, error) ||
            !contained(p_root, scenario.file))
        {
            m_errors.push_back(scenario.file.string() +
                               " is outside the plugin package or missing");
            return;
        }
    }
    m_roots_seen.push_back(root);
    m_packages.push_back(std::move(package));
}

} // namespace robotik
