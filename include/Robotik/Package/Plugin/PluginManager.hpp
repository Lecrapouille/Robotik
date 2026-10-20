// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file PluginManager.hpp
/// Loads one plugin library and drives a single scenario instance.
///
/// The library stays loaded when the scenario instance is destroyed, so
/// another scenario of the same plugin can be prepared without @c dlopen.
///
/// @par Contract
/// - Promises: each call returns a status; @c error() tells why the last one
///   failed.
/// - Expects: the order load, create, setup, start, then stop, shutdown,
///   destroy, unload.
#pragma once

#include "Robotik/Package/Plugin/PluginABI.h"

#include <cstdint>
#include <string>

namespace robotik
{

// ****************************************************************************
/// Life cycle of a plugin, in the order of the calls of @ref PluginManager.
// ****************************************************************************
enum class PluginState : std::uint8_t
{
    Unloaded, ///< No library open.
    Loaded,   ///< Library open, no instance.
    Created,  ///< Instance created, no scenario.
    Setup,    ///< Scenario prepared, not started.
    Running,  ///< Updated every step.
    Paused,   ///< Started, but not updated.
    Stopped,  ///< Run ended; can be shut down.
    Shutdown, ///< Instance released its resources; can be destroyed.
};

// ****************************************************************************
/// Owner of the @c dlopen handle and of the instance pointer.
///
/// Each call checks the state and returns an error status (with the text in
/// @ref error) instead of throwing, as the C border requires.
///
/// @code
/// robotik::PluginManager plugin;
/// if (plugin.load("build/plugins/HelloWorld/librobotik_hello_world.so") !=
/// ROBOTIK_PLUGIN_OK)
///     std::cerr << plugin.error() << '\n';
/// plugin.create(&host_api);
/// plugin.setup("wave");
/// plugin.start();
/// while (running)
///     plugin.update(0.01);
/// plugin.stop();
/// plugin.shutdown();
/// plugin.unload();
/// @endcode
// ****************************************************************************
class PluginManager
{
public:

    PluginManager() = default;
    PluginManager(PluginManager const&) = delete;
    PluginManager& operator=(PluginManager const&) = delete;

    /// Destroys the instance and closes the library if still open.
    ~PluginManager();

    // -------------------------------------------------------------------------
    /// @return Where the plugin is in its life cycle.
    // -------------------------------------------------------------------------
    [[nodiscard]] PluginState state() const
    {
        return m_state;
    }

    // -------------------------------------------------------------------------
    /// @return The message of the last failed call, or empty.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    // -------------------------------------------------------------------------
    /// @return The plugin id given by its library, once loaded.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::string const& id() const
    {
        return m_id;
    }

    // -------------------------------------------------------------------------
    /// @return True when a library is open.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool loaded() const
    {
        return m_state != PluginState::Unloaded;
    }

    // -------------------------------------------------------------------------
    /// Opens the library, checks the ABI and reads the vtable.
    /// @param p_path Path of the @c .so file.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus load(std::string const& p_path);

    // -------------------------------------------------------------------------
    /// Creates the instance.
    /// @param p_host Functions the plugin may call. It must outlive the
    ///   instance.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus create(RobotikHostAPI const* p_host);

    // -------------------------------------------------------------------------
    /// Prepares one scenario of the plugin.
    /// @param p_scenario_id Scenario id, as listed in @c plugin.yaml.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus setup(std::string const& p_scenario_id);

    /// Starts the prepared scenario. @return The status.
    RobotikPluginStatus start();

    // -------------------------------------------------------------------------
    /// Advances the plugin by one step.
    /// @param p_dt Duration of the step (s).
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus update(double p_dt);

    // -------------------------------------------------------------------------
    /// Forwards an event of the host (key pressed, scenario started, skill
    /// completed...) to the plugin.
    /// @param p_event Event to forward.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus onEvent(RobotikEvent const& p_event);

    // -------------------------------------------------------------------------
    /// Pauses or resumes the run.
    /// @param p_paused True to pause, false to resume.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus pause(bool p_paused);

    /// Ends the run. @return The status.
    RobotikPluginStatus stop();

    /// Lets the instance release its resources. @return The status.
    RobotikPluginStatus shutdown();

    // -------------------------------------------------------------------------
    /// Destroys the instance and keeps the library open, ready for another
    /// @ref create.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus destroy();

    // -------------------------------------------------------------------------
    /// Destroys a leftover instance, then closes the library.
    /// @return The status; see @ref error on failure.
    // -------------------------------------------------------------------------
    RobotikPluginStatus unload();

private:

    /// Stores @p_message in @ref m_error and returns @p_status.
    RobotikPluginStatus fail(RobotikPluginStatus p_status,
                             std::string p_message);
    /// Forgets the functions read from the library.
    void clearSymbols();

    /// Life cycle state.
    PluginState m_state = PluginState::Unloaded;
    /// Message of the last failed call.
    std::string m_error;
    /// Plugin id given by its library.
    std::string m_id;
    /// @c dlopen handle, or null.
    void* m_library = nullptr;
    /// Instance created by the plugin, or null.
    RobotikPlugin* m_instance = nullptr;
    /// Functions of the instance.
    RobotikPluginVTable m_api{};
    /// Factory read from the library.
    RobotikPlugin* (*m_create)(RobotikHostAPI const*) = nullptr;
    /// Destructor read from the library.
    void (*m_destroy)(RobotikPlugin*) = nullptr;
};

} // namespace robotik
