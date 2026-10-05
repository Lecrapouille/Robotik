// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Devices.hpp
//! @brief Registry of the sensors or the actuators of a robot.
#pragma once

#include "Robotik/Runtime/Resources.hpp"

#include <concepts>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace robotik
{

class Robot;

// ****************************************************************************
//! @brief Owns devices and declares each one as a resource of the same name.
//!
//! @code
//! auto& camera = robot.sensors().add<robotik::Camera>("wrist_camera", config);
//! auto* gripper = robot.actuators().first<robotik::VacuumGripper>();
//! bool ok = robot.resources().available("wrist_camera");
//! @endcode
// ****************************************************************************
template <typename Base>
class DeviceSet
{
public:

    DeviceSet(Robot& p_robot, ResourceManager& p_resources)
        : m_robot(p_robot), m_resources(p_resources)
    {
    }

    // -------------------------------------------------------------------------
    //! @brief Builds a device in place and registers its resource.
    //! @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<Base> T, typename... Args>
    T& add(Args&&... p_args)
    {
        auto device = std::make_unique<T>(std::forward<Args>(p_args)...);
        if (find(device->name()) != nullptr)
        {
            throw std::invalid_argument("Duplicated device '" +
                                        device->name() + "'");
        }
        if constexpr (requires { device->bind(m_robot); })
        {
            device->bind(m_robot);
        }
        T& added = *device;
        m_ids.push_back(m_resources.add(added.name()));
        m_devices.push_back(std::move(device));
        return added;
    }

    [[nodiscard]] std::size_t size() const
    {
        return m_devices.size();
    }

    [[nodiscard]] Base& operator[](std::size_t p_index) const
    {
        return *m_devices[p_index];
    }

    //! @brief Resource of the device at @p_index.
    [[nodiscard]] ResourceId resource(std::size_t p_index) const
    {
        return m_ids[p_index];
    }

    [[nodiscard]] bool available(std::size_t p_index) const
    {
        return m_resources.available(m_ids[p_index]);
    }

    [[nodiscard]] Base* find(std::string_view p_name) const
    {
        for (auto const& device : m_devices)
        {
            if (device->name() == p_name)
            {
                return device.get();
            }
        }
        return nullptr;
    }

    template <std::derived_from<Base> T>
    [[nodiscard]] T* find(std::string_view p_name) const
    {
        return dynamic_cast<T*>(find(p_name));
    }

    //! @brief First device of type @p T, or null.
    template <std::derived_from<Base> T>
    [[nodiscard]] T* first() const
    {
        for (auto const& device : m_devices)
        {
            if (auto* found = dynamic_cast<T*>(device.get()))
            {
                return found;
            }
        }
        return nullptr;
    }

private:

    Robot& m_robot;
    ResourceManager& m_resources;
    std::vector<std::unique_ptr<Base>> m_devices;
    std::vector<ResourceId> m_ids;
};

} // namespace robotik
