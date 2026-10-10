// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file PluginManager.hpp
//! @brief Loads one plugin library and drives a single scenario instance.
//!
//! The library stays loaded when the scenario instance is destroyed, so
//! another scenario of the same plugin can be prepared without @c dlopen.
#pragma once

#include "Robotik/Plugin/PluginABI.h"

#include <cstdint>
#include <string>

namespace robotik
{

enum class PluginState : std::uint8_t
{
    Unloaded,
    Loaded,
    Created,
    Setup,
    Running,
    Paused,
    Stopped,
    Shutdown,
};

// ****************************************************************************
//! @brief Owner of the @c dlopen handle and of the instance pointer.
// ****************************************************************************
class PluginManager
{
public:

    PluginManager() = default;
    PluginManager(PluginManager const&) = delete;
    PluginManager& operator=(PluginManager const&) = delete;
    ~PluginManager();

    [[nodiscard]] PluginState state() const
    {
        return m_state;
    }

    [[nodiscard]] std::string const& error() const
    {
        return m_error;
    }

    [[nodiscard]] std::string const& id() const
    {
        return m_id;
    }

    [[nodiscard]] bool loaded() const
    {
        return m_state != PluginState::Unloaded;
    }

    //! @brief Opens the library, checks the ABI and reads the vtable.
    RobotikPluginStatus load(std::string const& p_path);

    //! @brief Creates the instance. The host API must outlive it.
    RobotikPluginStatus create(RobotikHostAPI const* p_host);

    RobotikPluginStatus setup(std::string const& p_scenario_id);
    RobotikPluginStatus start();
    RobotikPluginStatus update(double p_dt);
    RobotikPluginStatus onEvent(RobotikEvent const& p_event);
    RobotikPluginStatus pause(bool p_paused);
    RobotikPluginStatus stop();
    RobotikPluginStatus shutdown();

    //! @brief Destroys the instance and keeps the library open.
    RobotikPluginStatus destroy();

    //! @brief Destroys a leftover instance, then closes the library.
    RobotikPluginStatus unload();

private:

    RobotikPluginStatus fail(RobotikPluginStatus p_status, std::string p_message);
    void clearSymbols();

    PluginState m_state = PluginState::Unloaded;
    std::string m_error;
    std::string m_id;
    void* m_library = nullptr;
    RobotikPlugin* m_instance = nullptr;
    RobotikPluginVTable m_api{};
    RobotikPlugin* (*m_create)(RobotikHostAPI const*) = nullptr;
    void (*m_destroy)(RobotikPlugin*) = nullptr;
};

} // namespace robotik
