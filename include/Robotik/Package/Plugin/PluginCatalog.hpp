// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginCatalog.hpp
/// Discovers plugin packages without loading their libraries.
///
/// A package is a directory with @c plugin.yaml, the shared library, its
/// scenario files and optionally a @c data/ asset pack. The catalog reads the
/// manifest only; the library is opened later, for the chosen scenario.
///
/// @code
/// plugin:
///   id: robotik.painter
///   name: Robotic painter
///   version: 1.0.0
///   api_version: 1            # ROBOTIK_PLUGIN_ABI_VERSION it was built for
///   category: Manipulation
///   description: A robot arm paints strokes on an easel.   # optional
/// library:
///   entrypoint: librobotik_painter.so
/// resources:                  # runtime dependencies
///   directory: data           # this package's own asset pack
///   packs:
///     - { id: robotik.robots, version: "1.0" }
/// dependencies:
///   plugins: [ robotik.other ]  # must be in the catalog to run
///   system: [ { name: opencv4, license: Apache-2.0 } ]
///   thirdparties: [ { repo: AprilRobotics/apriltag, license: BSD-2-Clause } ]
/// host:                       # hints for the simulator
///   graphics: false
///   clock: simulation         # or plugin: its update advances time
///   view: { eye: [1.6, 1.2, 1.6], target: [0.35, 0.15, 0.0] }
/// scenarios:
///   directory: scenarios
///   entries:                  # optional: every runnable file of the folder
///   otherwise
///     - { id: painter, name: Robotic painter, file: painter.scenario.yaml }
///     # optional: submenu of the Demos menu, clock overriding host.clock
///     - { id: rl, name: RL, file: rl.rl.yaml, menu: Learning, clock: plugin }
/// experiments:                # optional, same format
///   directory: experiments
/// @endcode
///
/// The manifest describes the package; it does not build it. Its
/// @c dependencies (see @ref ThirdParty) are read by the Makefile of the
/// plugin too (plugins/dependencies.awk): it clones and builds them.
#pragma once

#include "Robotik/Common/Version.hpp"

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// A runnable file of a package: a scenario or an experiment.
///
/// @code
/// for (robotik::PluginScenario const& s : package.scenarios)
///     std::cout << s.id << " (" << s.name << "): " << s.file << '\n';
/// @endcode
// ****************************************************************************
struct PluginScenario
{
    /// Id given to @c Plugin::setup.
    std::string id;
    /// Name shown in the Demos menu.
    std::string name;
    /// Absolute path of the file.
    std::filesystem::path file;
    /// Submenu of the Demos menu, or empty (directly under the package).
    std::string menu;
    /// The plugin update advances time for this scenario: its @c clock
    /// field, else @c host.clock of the package.
    bool owns_clock = false;
};

// ****************************************************************************
/// An asset pack the package needs at run time (@c resources.packs).
///
/// @code
/// std::string why;
/// if (assets.require(need.id, need.version, why) == nullptr)
///     std::cerr << why << '\n';
/// @endcode
// ****************************************************************************
struct PackRequirement
{
    /// Pack id.
    std::string id;
    /// Oldest compatible version (same major).
    Version version;
};

// ****************************************************************************
/// A library the plugin is built with, from @c dependencies.system or
/// @c dependencies.thirdparties of @c plugin.yaml. The simulator lists them
/// in the credits:
/// @code
/// dependencies:
///   system:
///     - { name: opencv4, license: Apache-2.0 }           # pkg-config name
///   thirdparties:
///     - { repo: AprilRobotics/apriltag@v3.4.2, license: BSD-2-Clause,
///         description: AprilTag fiducial detection }     # cloned and built
///     - { name: onnxruntime, url: "https://...", license: MIT }  # no repo
/// @endcode
// ****************************************************************************
struct ThirdParty
{
    /// GitHub @c user/repo (without @c :recurse nor @c @ref), or a name.
    std::string name;
    /// Branch, tag or commit after @c @ in @c repo, or empty.
    std::string reference;
    /// Home page: the GitHub page of a repo, or the @c url field.
    std::string url;
    /// SPDX license (required).
    std::string license;
    /// What the library does, for the credits.
    std::string description;
};

// ****************************************************************************
/// Everything @c plugin.yaml says about a package (see the file comment).
///
/// @code
/// for (robotik::PluginPackage const& p : catalog.packages())
///     std::cout << p.id << ' ' << p.version.toString() << ": "
///               << p.scenarios.size() << " scenarios\n";
/// @endcode
// ****************************************************************************
struct PluginPackage
{
    /// Folder holding @c plugin.yaml.
    std::filesystem::path root;
    /// Plugin id; equals @c describe().id of the library.
    std::string id;
    /// Name shown to the user.
    std::string name;
    /// Version of the package.
    Version version;
    /// Group in the Demos menu.
    std::string category;
    /// One or two sentences for the user, or empty.
    std::string description;
    /// @c ROBOTIK_PLUGIN_ABI_VERSION the library was built against.
    std::uint32_t api_version = 0;
    /// The package's own asset pack, or empty.
    std::filesystem::path resources;
    /// Asset packs needed at run time.
    std::vector<PackRequirement> packs;
    /// Plugin ids that must be in the catalog for this one to run.
    std::vector<std::string> plugins;
    /// Libraries of the system (pkg-config), for the credits.
    std::vector<ThirdParty> system;
    /// Libraries cloned and built with the plugin, for the credits.
    std::vector<ThirdParty> thirdparties;
    /// Needs a graphical host (@c host.graphics); refused headless.
    bool graphics = false;
    /// Default of @ref PluginScenario::owns_clock (@c host.clock).
    bool owns_clock = false;
    /// @c host.view is given: use @ref eye and @ref target.
    bool has_view = false;
    /// Camera position of the 3D view (m).
    float eye[3] = { 1.6F, 1.2F, 1.6F };
    /// Point the camera of the 3D view looks at (m).
    float target[3] = { 0.35F, 0.15F, 0.0F };
    /// Absolute path of the shared library.
    std::filesystem::path library;
    /// Scenario files.
    std::vector<PluginScenario> scenarios;
    /// Experiment files (cases over the scenarios), same entry format.
    std::vector<PluginScenario> experiments;
};

// ****************************************************************************
/// A package and one of its scenarios, found by @ref
/// PluginCatalog::matchScenario.
///
/// @code
/// robotik::PluginMatch const m = catalog.matchScenario("wave.scenario.yaml");
/// if (m.package != nullptr && m.scenario != nullptr)
///     std::cout << m.package->id << " / " << m.scenario->id << '\n';
/// @endcode
// ****************************************************************************
struct PluginMatch
{
    /// Package found, or null.
    PluginPackage const* package = nullptr;
    /// Scenario of the package, or null.
    PluginScenario const* scenario = nullptr;
};

// ****************************************************************************
/// In-memory list of plugin packages.
///
/// A broken manifest does not stop the scan: its message goes to
/// @ref errors.
///
/// @code
/// robotik::PluginCatalog catalog;
/// catalog.scan();
/// for (std::string const& e : catalog.errors())
///     std::cerr << e << '\n';
/// robotik::PluginPackage const* hello =
/// catalog.findPackage("robotik.hello_world");
/// @endcode
// ****************************************************************************
class PluginCatalog
{
public:

    // -------------------------------------------------------------------------
    /// Scans an extra folder, then the default roots: each directory of
    /// @c ROBOTIK_PLUGINS (colon separated), @c plugins/ next to the
    /// executable, @c lib/robotik/plugins of its install prefix.
    /// @param p_extra Extra package or folder of packages; empty for none.
    // -------------------------------------------------------------------------
    void scan(std::filesystem::path const& p_extra = {});

    // -------------------------------------------------------------------------
    /// Adds the package holding a scenario (its folder or one of the two
    /// above).
    /// @param p_scenario Scenario file.
    /// @return The package, or null when there is none.
    // -------------------------------------------------------------------------
    PluginPackage const* add(std::filesystem::path const& p_scenario);

    // -------------------------------------------------------------------------
    /// Scans exactly these directories.
    /// @param p_roots Each one is either a package or a folder of packages.
    // -------------------------------------------------------------------------
    void scan(std::vector<std::filesystem::path> const& p_roots);

    // -------------------------------------------------------------------------
    /// @return Every package found, in scan order.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<PluginPackage> const& packages() const
    {
        return m_packages;
    }

    // -------------------------------------------------------------------------
    /// @return The manifest errors met while scanning.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::vector<std::string> const& errors() const
    {
        return m_errors;
    }

    // -------------------------------------------------------------------------
    /// @param p_id Plugin id.
    /// @return The package, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] PluginPackage const* findPackage(std::string_view p_id) const;

    // -------------------------------------------------------------------------
    /// @param p_scenario Scenario file, compared as a canonical path.
    /// @return The package that ships it, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] PluginPackage const*
    findScenario(std::filesystem::path const& p_scenario) const;

    // -------------------------------------------------------------------------
    /// Finds a scenario by its exact path, or else the only scenario whose
    /// id is the id of the file name (see @ref roleStem).
    /// @param p_scenario Scenario file.
    /// @return The package and the scenario; both null when nothing or
    ///   several scenarios match.
    // -------------------------------------------------------------------------
    [[nodiscard]] PluginMatch
    matchScenario(std::filesystem::path const& p_scenario) const;

    // -------------------------------------------------------------------------
    /// @param p_package Package to search.
    /// @param p_scenario Scenario file, compared as a canonical path.
    /// @return The scenario entry of the package, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] PluginScenario const*
    scenarioOf(PluginPackage const& p_package,
               std::filesystem::path const& p_scenario) const;

private:

    /// Loads @p_directory if it is a package, else the packages one level
    /// below it.
    void consider(std::filesystem::path const& p_directory);
    /// Reads one @c plugin.yaml; errors go to @ref m_errors.
    void loadPackage(std::filesystem::path const& p_root);

    /// Packages found.
    std::vector<PluginPackage> m_packages;
    /// Manifest errors.
    std::vector<std::string> m_errors;
    /// Folders already scanned, to load a package once.
    std::vector<std::filesystem::path> m_roots_seen;
};

} // namespace robotik
