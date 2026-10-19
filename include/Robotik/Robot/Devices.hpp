// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Devices.hpp
/// Registry of the sensors or the actuators of a robot.
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
/// Owns devices and declares each one as a resource of the same name.
///
/// So a skill that needs the camera reserves the resource @c wrist_camera,
/// and a fault on @c wrist_camera stops the camera.
///
/// @code
/// auto& camera = robot.sensors().add<robotik::Camera>("wrist_camera", config);
/// auto* gripper = robot.actuators().first<robotik::VacuumGripper>();
/// bool ok = robot.resources().available("wrist_camera");
/// @endcode
///
/// @tparam Base @ref Sensor or @ref Actuator.
// ****************************************************************************
template <typename Base>
class DeviceSet
{
public:

    // -------------------------------------------------------------------------
    /// @param p_robot Robot carrying the devices (given to their @c bind).
    /// @param p_resources Manager receiving one resource per device.
    // -------------------------------------------------------------------------
    DeviceSet(Robot& p_robot, ResourceManager& p_resources)
        : m_robot(p_robot), m_resources(p_resources)
    {
    }

    // -------------------------------------------------------------------------
    /// Builds a device in place and registers its resource. A device with a
    /// @c bind(Robot&) method is bound to the robot.
    /// @tparam T Concrete device type.
    /// @param p_args Arguments of the constructor of @p T (name first).
    /// @return The new device, owned by the set.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<Base> T, typename... Args>
    T& add(Args&&... p_args)
    {
        auto device = std::make_unique<T>(std::forward<Args>(p_args)...);
        if (find(device->name()) != nullptr)
        {
            throw std::invalid_argument("Duplicated device '" + device->name() +
                                        "'");
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

    // -------------------------------------------------------------------------
    /// @return The number of devices.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t size() const
    {
        return m_devices.size();
    }

    // -------------------------------------------------------------------------
    /// @param p_index Index in insertion order, below @ref size.
    /// @return The device.
    // -------------------------------------------------------------------------
    [[nodiscard]] Base& operator[](std::size_t p_index) const
    {
        return *m_devices[p_index];
    }

    // -------------------------------------------------------------------------
    /// @param p_index Index of the device.
    /// @return The resource of the device.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceId resource(std::size_t p_index) const
    {
        return m_ids[p_index];
    }

    // -------------------------------------------------------------------------
    /// @param p_index Index of the device.
    /// @return False when the resource of the device has failed.
    // -------------------------------------------------------------------------
    [[nodiscard]] bool available(std::size_t p_index) const
    {
        return m_resources.available(m_ids[p_index]);
    }

    // -------------------------------------------------------------------------
    /// Finds a device by its name.
    /// @param p_name Device name.
    /// @return The device, or null.
    // -------------------------------------------------------------------------
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

    // -------------------------------------------------------------------------
    /// Finds a device by its name and type.
    /// @tparam T Expected device type.
    /// @param p_name Device name.
    /// @return The device, or null when absent or of another type.
    // -------------------------------------------------------------------------
    template <std::derived_from<Base> T>
    [[nodiscard]] T* find(std::string_view p_name) const
    {
        return dynamic_cast<T*>(find(p_name));
    }

    // -------------------------------------------------------------------------
    /// Finds the first device of a type.
    /// @tparam T Wanted device type.
    /// @return The first device of type @p T, or null.
    // -------------------------------------------------------------------------
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

    /// Robot carrying the devices.
    Robot& m_robot;
    /// Manager of the device resources.
    ResourceManager& m_resources;
    /// Devices, in insertion order.
    std::vector<std::unique_ptr<Base>> m_devices;
    /// Resource of each device, aligned with @ref m_devices.
    std::vector<ResourceId> m_ids;
};

} // namespace robotik
