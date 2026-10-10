// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginCatalog.hpp
//! @brief Discovers plugin packages without loading their libraries.
//!
//! A package is a directory with @c plugin.yaml, the shared library and a
//! @c scenarios/ folder. One plugin owns several scenario files. The catalog
//! reads the manifest only; the library is opened later, for the chosen
//! scenario.
#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

struct PluginScenario
{
    std::string id;
    std::string name;
    std::filesystem::path file;
};

struct PluginPackage
{
    std::filesystem::path root;
    std::string id;
    std::string name;
    std::string version;
    std::string category;
    bool graphics = false;
    //! @brief The plugin update replaces @c Simulation::step for this package.
    bool owns_clock = false;
    bool has_view = false;
    float eye[3] = { 1.6f, 1.2f, 1.6f };
    float target[3] = { 0.35f, 0.15f, 0.0f };
    std::filesystem::path library;
    std::vector<PluginScenario> scenarios;
};

struct PluginMatch
{
    PluginPackage const* package = nullptr;
    PluginScenario const* scenario = nullptr;
};

// ****************************************************************************
//! @brief In-memory list of plugin packages.
// ****************************************************************************
class PluginCatalog
{
public:

    //! @brief Default search roots: @c ROBOTIK_PLUGINS, @c build/plugins, @c plugins.
    void scan(std::filesystem::path const& p_extra = {});

    //! @brief Scans exactly these directories. Each one is either a package
    //! or a folder of packages.
    void scan(std::vector<std::filesystem::path> const& p_roots);

    [[nodiscard]] std::vector<PluginPackage> const& packages() const
    {
        return m_packages;
    }

    [[nodiscard]] std::vector<std::string> const& errors() const
    {
        return m_errors;
    }

    [[nodiscard]] PluginPackage const* findPackage(std::string_view p_id) const;

    //! @brief Package that ships @p_scenario, compared as a canonical path.
    [[nodiscard]] PluginPackage const* findScenario(std::filesystem::path const& p_scenario) const;

    //! @brief Exact path, or the only scenario whose id is the file stem.
    [[nodiscard]] PluginMatch matchScenario(std::filesystem::path const& p_scenario) const;

    [[nodiscard]] PluginScenario const* scenarioOf(PluginPackage const& p_package,
                                                   std::filesystem::path const& p_scenario) const;

private:

    void consider(std::filesystem::path const& p_directory);
    void loadPackage(std::filesystem::path const& p_root);

    std::vector<PluginPackage> m_packages;
    std::vector<std::string> m_errors;
    std::vector<std::filesystem::path> m_roots_seen;
};

} // namespace robotik
