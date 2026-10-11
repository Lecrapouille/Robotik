// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginCatalog.hpp
//! @brief Discovers plugin packages without loading their libraries.
//!
//! A package is a directory with @c plugin.yaml, the shared library, its
//! scenario files and optionally a @c data/ asset pack. The catalog reads the
//! manifest only; the library is opened later, for the chosen scenario.
//!
//! @code
//! plugin:
//!   id: robotik.painter
//!   name: Robotic painter
//!   version: 1.0.0
//!   api_version: 1            # ROBOTIK_PLUGIN_ABI_VERSION it was built for
//!   category: Manipulation
//! library:
//!   entrypoint: librobotik_painter.so
//! resources:                  # runtime dependencies
//!   directory: data           # this package's own asset pack
//!   packs:
//!     - { id: robotik.robots, version: "1.0" }
//! dependencies:
//!   plugins: [ robotik.other ]  # must be in the catalog to run
//!   thirdparties: [ opencv ]    # build time, resolved by the Makefile
//! host:                       # hints for the simulator
//!   graphics: false
//!   clock: simulation         # or plugin: its update advances time
//!   view: { eye: [1.6, 1.2, 1.6], target: [0.35, 0.15, 0.0] }
//! scenarios:
//!   directory: scenarios
//!   entries:                  # optional: every *.yml of the folder otherwise
//!     - { id: painter, name: Robotic painter, file: painter.yml }
//! experiments:                # optional, same format
//!   directory: experiments
//! @endcode
//!
//! The manifest describes the package; it does not build it.
#pragma once

#include "Robotik/Assets/Version.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

//! @brief A runnable file of a package: a scenario or an experiment.
struct PluginScenario
{
    std::string id;
    std::string name;
    std::filesystem::path file;
};

//! @brief An asset pack the package needs at run time.
struct PackRequirement
{
    std::string id;
    //!< Oldest compatible version (same major).
    Version version;
};

struct PluginPackage
{
    std::filesystem::path root;
    std::string id;
    std::string name;
    Version version;
    std::string category;
    //!< @c ROBOTIK_PLUGIN_ABI_VERSION the library was built against.
    std::uint32_t api_version = 0;
    //!< The package's own asset pack, or empty.
    std::filesystem::path resources;
    std::vector<PackRequirement> packs;
    //!< Plugin ids that must be in the catalog for this one to run.
    std::vector<std::string> plugins;
    //!< Build-time libraries, informative only.
    std::vector<std::string> thirdparties;
    bool graphics = false;
    //! @brief The plugin update replaces @c Simulation::step for this package.
    bool owns_clock = false;
    bool has_view = false;
    float eye[3] = { 1.6f, 1.2f, 1.6f };
    float target[3] = { 0.35f, 0.15f, 0.0f };
    std::filesystem::path library;
    std::vector<PluginScenario> scenarios;
    //!< Experiment files (cases over the scenarios), same entry format.
    std::vector<PluginScenario> experiments;
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

    //! @brief @p_extra, then the default roots: each directory of
    //! @c ROBOTIK_PLUGINS (colon separated), @c plugins/ next to the
    //! executable, @c lib/robotik/plugins of its install prefix.
    void scan(std::filesystem::path const& p_extra = {});

    //! @brief Adds the package holding @p_scenario (its folder or one of the
    //! two above) and returns it, or null when there is none.
    PluginPackage const* add(std::filesystem::path const& p_scenario);

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
