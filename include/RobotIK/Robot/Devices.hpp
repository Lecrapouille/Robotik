// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Devices.hpp
/// The list of the sensors, or of the actuators, of one robot.
///
/// It owns the devices and declares each one as a resource (see
/// doc/reference/glossary.md: "Resource"), so a skill reserves the camera by
/// its name and a fault on that name stops the camera.
///
/// @par Contract
/// - Promises: each device is a resource of the same name; a failed resource
///   stops its device at the next step.
/// - Expects: device names are unique in the world (several robots share one
///   resource manager).
/// - Errors: a duplicated name throws @c std::invalid_argument.
#pragma once

#include "RobotIK/Robot/Resources.hpp"

#include <concepts>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace robotik
{

class Robot;
class RobotRuntime;

// ****************************************************************************
/// The devices of a robot: find, read, command; only the runtime adds one.
///
/// Why: a skill must not grow a camera nor a gripper: a real robot does not
/// change because a program asks. So @c add is private: @ref RobotRuntime
/// calls it (@ref RobotRuntime::addSensor, @ref RobotRuntime::addActuator),
/// a skill gets the set from @ref Robot and only finds and uses.
///
/// Each device is a resource of the same name (see
/// doc/reference/glossary.md: "Resource"): a skill that needs the camera
/// reserves @c wrist_camera, and a fault on @c wrist_camera stops it.
///
/// Why one class and not a list seen by skills plus a set that adds: the
/// right to add is one method, a @c friend says it without a second type.
///
/// @code
/// auto& camera = runtime.addSensor<robotik::Camera>("wrist_camera", config);
/// auto* gripper = robot.getActuators().first<robotik::VacuumGripper>();
/// auto const* imu = robot.getSensors().find<robotik::Imu>("imu");
/// @endcode
///
/// @tparam Base @ref Sensor or @ref Actuator.
// ****************************************************************************
template <typename Base>
class DeviceSet final
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

    DeviceSet(DeviceSet const&) = delete;
    DeviceSet& operator=(DeviceSet const&) = delete;

    // -------------------------------------------------------------------------
    /// Finds a device by its name.
    /// @param p_name Device name.
    /// @return The device, or null.
    // -------------------------------------------------------------------------
    [[nodiscard]] Base* find(std::string_view p_name)
    {
        return const_cast<Base*>(std::as_const(*this).find(p_name));
    }

    /// @copydoc find(std::string_view)
    [[nodiscard]] Base const* find(std::string_view p_name) const
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
    [[nodiscard]] T* find(std::string_view p_name)
    {
        return dynamic_cast<T*>(find(p_name));
    }

    /// @copydoc find(std::string_view)
    template <std::derived_from<Base> T>
    [[nodiscard]] T const* find(std::string_view p_name) const
    {
        return dynamic_cast<T const*>(find(p_name));
    }

    // -------------------------------------------------------------------------
    /// Finds the first device of a type.
    /// @tparam T Wanted device type.
    /// @return The first device of type @p T, or null.
    // -------------------------------------------------------------------------
    template <std::derived_from<Base> T>
    [[nodiscard]] T* first()
    {
        return const_cast<T*>(std::as_const(*this).template first<T>());
    }

    /// @copydoc first()
    template <std::derived_from<Base> T>
    [[nodiscard]] T const* first() const
    {
        for (auto const& device : m_devices)
        {
            if (auto const* found = dynamic_cast<T const*>(device.get()))
            {
                return found;
            }
        }
        return nullptr;
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
    /// @param p_index Index in insertion order, below @ref size.
    /// @return The device.
    // -------------------------------------------------------------------------
    [[nodiscard]] Base& operator[](std::size_t p_index)
    {
        return *m_devices[p_index];
    }

    /// @copydoc operator[](std::size_t)
    [[nodiscard]] Base const& operator[](std::size_t p_index) const
    {
        return *m_devices[p_index];
    }

    // -------------------------------------------------------------------------
    /// @return The number of devices.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::size_t size() const
    {
        return m_devices.size();
    }

private:

    friend class RobotRuntime;

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
        // The resource exists before bind: an actuator guards its joints
        // with it.
        ResourceId const resource = m_resources.add(device->name());
        if constexpr (requires { device->bind(m_robot); })
        {
            device->bind(m_robot);
        }
        T& added = *device;
        m_ids.push_back(resource);
        m_devices.push_back(std::move(device));
        return added;
    }

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
