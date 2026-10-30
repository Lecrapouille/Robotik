// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file CellDevices.hpp
/// Devices of the cell that belong to no robot: a conveyor belt, later the
/// machines of a factory. A device has a name, also the name of its resource
/// in the resource manager the robots share, and the world entity it stands
/// for.
///
/// @code
/// # a world entity: a belt of 1 m, the parts on it go along +x at 0.3 m/s
/// - id: feeder
///   type: body
///   shape: { type: box, size: [1.0, 0.2, 0.05] }
///   actuators:
///     feeder_belt: { type: conveyor, speed: 0.3, direction: [1, 0, 0] }
/// @endcode
///
/// @par Contract
/// - Promises: one resource per device, registered when it is added; a belt
///   only says what is commanded, the @ref Cell moves the parts on it.
/// - Errors: a second device of the same name throws (a mistake in the code;
///   the world reader refuses it first).
#pragma once

#include "Robotik/Common/Math.hpp"
#include "Robotik/Robot/Resources.hpp"

#include <concepts>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// A device of the cell, not of a robot.
// ****************************************************************************
class CellDevice
{
public:

    virtual ~CellDevice() = default;

    /// @return Device name, also the name of its resource.
    [[nodiscard]] std::string const& name() const
    {
        return m_name;
    }

    /// @return World entity the device stands for.
    [[nodiscard]] std::string const& entity() const
    {
        return m_entity;
    }

    /// The resource failed, or a new episode: back to a safe state.
    virtual void stop() = 0;

protected:

    /// @param p_name Device name, unique in the world.
    /// @param p_entity World entity it stands for.
    CellDevice(std::string p_name, std::string p_entity)
        : m_name(std::move(p_name)), m_entity(std::move(p_entity))
    {
    }

private:

    std::string m_name;
    std::string m_entity;
};

// ****************************************************************************
/// A conveyor belt: the top face of a box body. While it runs, the parts
/// resting on it go at its speed, in its direction.
///
/// @code
/// auto* belt = cell.devices().find<robotik::ConveyorBelt>("feeder_belt");
/// belt->run(true);   // or request the skill "feeder_belt.Run"
/// @endcode
// ****************************************************************************
class ConveyorBelt final: public CellDevice
{
public:

    // -------------------------------------------------------------------------
    /// @param p_name Belt name, also its resource name.
    /// @param p_entity Box body whose top face is the belt.
    /// @param p_direction Direction of the parts, in the frame of the body
    ///   (normalized here; z is ignored).
    /// @param p_speed Speed of the parts.
    /// @throws std::invalid_argument for a direction without x nor y.
    // -------------------------------------------------------------------------
    ConveyorBelt(std::string p_name, std::string p_entity, Vector3 p_direction,
                 LinearVelocity p_speed);

    /// Runs (true) or stops (false) the belt.
    void run(bool p_on)
    {
        m_running = p_on;
    }

    /// Stops the belt.
    void stop() override
    {
        m_running = false;
    }

    /// @return True while the belt is commanded to run.
    [[nodiscard]] bool running() const
    {
        return m_running;
    }

    /// @return Unit direction of the parts in the frame of the body (z = 0).
    [[nodiscard]] Vector3 const& direction() const
    {
        return m_direction;
    }

    /// @return Speed of the parts while it runs.
    [[nodiscard]] LinearVelocity speed() const
    {
        return m_speed;
    }

private:

    Vector3 m_direction;
    LinearVelocity m_speed;
    bool m_running = false;
};

// ****************************************************************************
/// The devices of a cell, each with its resource.
// ****************************************************************************
class CellDevices
{
public:

    /// @param p_resources Manager receiving one resource per device.
    explicit CellDevices(ResourceManager& p_resources) : m_resources(p_resources) {}

    // -------------------------------------------------------------------------
    /// Builds a device in place and registers its resource.
    /// @tparam T Concrete device type.
    /// @param p_args Arguments of the constructor of @p T (name first).
    /// @return The new device, owned by the set.
    /// @throws std::invalid_argument on a duplicated name.
    // -------------------------------------------------------------------------
    template <std::derived_from<CellDevice> T, typename... Args>
    T& add(Args&&... p_args)
    {
        auto device = std::make_unique<T>(std::forward<Args>(p_args)...);
        if (find(device->name()) != nullptr)
        {
            throw std::invalid_argument("Duplicated device '" + device->name() + "'");
        }
        T& added = *device;
        m_ids.push_back(m_resources.add(added.name()));
        m_devices.push_back(std::move(device));
        return added;
    }

    /// @return The number of devices.
    [[nodiscard]] std::size_t size() const
    {
        return m_devices.size();
    }

    /// @param p_index Index in insertion order, below @ref size.
    /// @return The device.
    [[nodiscard]] CellDevice& operator[](std::size_t p_index) const
    {
        return *m_devices[p_index];
    }

    /// @param p_index Index of the device.
    /// @return The resource of the device.
    [[nodiscard]] ResourceId resource(std::size_t p_index) const
    {
        return m_ids[p_index];
    }

    /// @param p_index Index of the device.
    /// @return False when the resource of the device has failed.
    [[nodiscard]] bool available(std::size_t p_index) const
    {
        return m_resources.available(m_ids[p_index]);
    }

    /// @param p_name Device name.
    /// @return The device, or null.
    [[nodiscard]] CellDevice* find(std::string_view p_name) const
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

    /// @tparam T Expected device type.
    /// @param p_name Device name.
    /// @return The device, or null when absent or of another type.
    template <std::derived_from<CellDevice> T>
    [[nodiscard]] T* find(std::string_view p_name) const
    {
        return dynamic_cast<T*>(find(p_name));
    }

private:

    ResourceManager& m_resources;
    std::vector<std::unique_ptr<CellDevice>> m_devices;
    std::vector<ResourceId> m_ids;
};

} // namespace robotik
