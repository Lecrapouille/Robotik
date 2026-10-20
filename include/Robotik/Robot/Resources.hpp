// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Resources.hpp
/// What skills reserve to run (arm, gripper, camera...), and failures.
///
/// A failure is just an unavailable resource: no skill-specific fault code.
/// Skills needing a failed resource cannot start; running ones are stopped by
/// the @ref SkillScheduler. Storage is structure-of-arrays indexed by a small
/// @ref ResourceId resolved once from the name.
#pragma once

#include <array>
#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

/// Index of a resource in its @ref ResourceManager.
using ResourceId = std::uint16_t;
/// No resource (e.g. not found, no conflict).
inline constexpr ResourceId NO_RESOURCE = 0xFFFFU;

/// Identifies who holds a lease (a skill id, or any user number).
using OwnerId = std::uint32_t;
/// No owner.
inline constexpr OwnerId NO_OWNER = 0xFFFFFFFFU;
/// Owner of a lease taken outside the scheduler.
inline constexpr OwnerId ANONYMOUS = 0xFFFFFFFEU;

// ****************************************************************************
/// Access mode of a reservation.
// ****************************************************************************
enum class Access : std::uint8_t
{
    Shared,    ///< Several holders at once (a camera stream).
    Exclusive, ///< One holder (an arm, a gripper).
};

// ****************************************************************************
/// One resource a skill needs, with its access mode.
///
/// Build it with @ref ResourceManager::require, which checks the name.
///
/// @code
/// robotik::ResourceRequirement const arm = resources.require("arm");
/// robotik::ResourceRequirement const eye =
///     resources.require("camera", robotik::Access::Shared);
/// @endcode
// ****************************************************************************
struct ResourceRequirement
{
    /// Resource needed.
    ResourceId id = NO_RESOURCE;
    /// Alone or shared with other holders.
    Access access = Access::Exclusive;
};

// ****************************************************************************
/// Why a set of requirements cannot be acquired right now.
///
/// @code
/// if (robotik::Conflict const c = resources.conflict(needs))
///     std::cout << resources.name(c.resource)
///               << (c.unavailable ? " failed" : " is busy") << '\n';
/// @endcode
// ****************************************************************************
struct Conflict
{
    /// First blocking resource, @ref NO_RESOURCE when there is no conflict.
    ResourceId resource = NO_RESOURCE;
    /// Exclusive holder of that resource, or @ref NO_OWNER.
    OwnerId owner = NO_OWNER;
    /// True when the resource failed rather than being busy.
    bool unavailable = false;

    /// @return True when there is a conflict.
    [[nodiscard]] explicit operator bool() const
    {
        return resource != NO_RESOURCE;
    }
};

class ResourceManager;

// ****************************************************************************
/// RAII reservation: releases its resources when destroyed.
///
/// Move it to keep the reservation longer; it cannot be copied.
///
/// @code
/// {
///     robotik::ResourceLease lease = resources.acquire({
///     resources.require("arm") }); if (!lease)
///         return; // busy or failed
///     // ... use the arm
/// } // the arm is free again
/// @endcode
// ****************************************************************************
class ResourceLease
{
public:

    /// Largest number of resources in one lease.
    static constexpr std::size_t CAPACITY = 8U;

    /// Empty lease (holds nothing).
    ResourceLease() = default;
    /// Takes the reservation of @p_other, which becomes empty.
    ResourceLease(ResourceLease&& p_other) noexcept;
    /// Releases the current reservation, then takes the one of @p_other.
    ResourceLease& operator=(ResourceLease&& p_other) noexcept;
    ResourceLease(ResourceLease const&) = delete;
    ResourceLease& operator=(ResourceLease const&) = delete;
    /// Releases the resources.
    ~ResourceLease();

    // -------------------------------------------------------------------------
    /// @return True when the lease holds its resources.
    // -------------------------------------------------------------------------
    [[nodiscard]] explicit operator bool() const
    {
        return m_manager != nullptr;
    }

    // -------------------------------------------------------------------------
    /// @return Who holds the lease.
    // -------------------------------------------------------------------------
    [[nodiscard]] OwnerId owner() const
    {
        return m_owner;
    }

    // -------------------------------------------------------------------------
    /// @return The resources held, with their access mode.
    // -------------------------------------------------------------------------
    [[nodiscard]] std::span<ResourceRequirement const> resources() const
    {
        return { m_items.data(), m_count };
    }

    // -------------------------------------------------------------------------
    /// Gives the resources back now; the lease becomes empty.
    // -------------------------------------------------------------------------
    void release();

private:

    friend class ResourceManager;

    /// Manager that granted the lease, or null when empty.
    ResourceManager* m_manager = nullptr;
    /// Who holds the lease.
    OwnerId m_owner = NO_OWNER;
    /// Resources held (the first @ref m_count).
    std::array<ResourceRequirement, CAPACITY> m_items{};
    /// Number of resources held.
    std::uint8_t m_count = 0;
};

// ****************************************************************************
/// Registry of the robot resources, their holders and their health.
///
/// Sensors and actuators register themselves under their own name (see
/// @ref DeviceSet); a mission may add more (a table, a shared zone).
///
/// @code
/// robotik::ResourceManager resources;
/// resources.add("arm");
/// resources.add("camera");
/// auto lease = resources.acquire({ resources.require("arm"),
///     resources.require("camera", robotik::Access::Shared) });
/// resources.fail("camera"); // fault injection or real failure
/// @endcode
// ****************************************************************************
class ResourceManager
{
public:

    // -------------------------------------------------------------------------
    /// Declares a resource.
    /// @param p_name Resource name, unique in the world.
    /// @return Its id; the existing id for a known name.
    // -------------------------------------------------------------------------
    ResourceId add(std::string p_name);

    // -------------------------------------------------------------------------
    /// @param p_name Resource name.
    /// @return Its id, or @ref NO_RESOURCE.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceId find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    /// Builds a requirement on a known resource.
    /// @param p_name Resource name.
    /// @param p_access Alone or shared.
    /// @return The requirement.
    /// @throws std::invalid_argument for an unknown name.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceRequirement
    require(std::string_view p_name, Access p_access = Access::Exclusive) const;

    /// @return The number of resources.
    [[nodiscard]] std::size_t size() const
    {
        return m_names.size();
    }

    /// @return The name of resource @p_id.
    [[nodiscard]] std::string const& name(ResourceId p_id) const
    {
        return m_names[p_id];
    }

    // --- Health -------------------------------------------------------------

    /// Marks a resource failed: skills needing it stop or cannot start.
    void fail(ResourceId p_id);
    /// Marks a resource working again.
    void restore(ResourceId p_id);
    /// @copydoc fail(ResourceId)
    void fail(std::string_view p_name);
    /// @copydoc restore(ResourceId)
    void restore(std::string_view p_name);
    /// Restores every resource.
    void restoreAll();

    /// @return False when resource @p_id failed or is unknown.
    [[nodiscard]] bool available(ResourceId p_id) const
    {
        return p_id < m_failed.size() && m_failed[p_id] == 0U;
    }

    /// @return False when resource @p_name failed or is unknown.
    [[nodiscard]] bool available(std::string_view p_name) const
    {
        return available(find(p_name));
    }

    // --- Ownership ----------------------------------------------------------

    /// @return Exclusive holder of @p_id, or @ref NO_OWNER.
    [[nodiscard]] OwnerId owner(ResourceId p_id) const
    {
        return m_owners[p_id];
    }

    /// @return Number of shared holders of @p_id.
    [[nodiscard]] std::uint16_t users(ResourceId p_id) const
    {
        return m_users[p_id];
    }

    // -------------------------------------------------------------------------
    /// Tells why requirements cannot be acquired now, without acquiring.
    /// @param p_requirements Resources wanted.
    /// @return The first reason; an empty conflict when all are free.
    // -------------------------------------------------------------------------
    [[nodiscard]] Conflict
    conflict(std::span<ResourceRequirement const> p_requirements) const;

    // -------------------------------------------------------------------------
    /// Reserves all the requirements at once, or nothing.
    /// @param p_requirements Resources wanted (at most
    ///   @ref ResourceLease::CAPACITY).
    /// @param p_owner Who holds the lease (a skill id).
    /// @return The lease; an empty lease on conflict.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceLease
    acquire(std::span<ResourceRequirement const> p_requirements,
            OwnerId p_owner = ANONYMOUS);

    /// @copydoc acquire(std::span<ResourceRequirement const>, OwnerId)
    [[nodiscard]] ResourceLease
    acquire(std::initializer_list<ResourceRequirement> p_requirements,
            OwnerId p_owner = ANONYMOUS)
    {
        return acquire(std::span<ResourceRequirement const>(
                           p_requirements.begin(), p_requirements.size()),
                       p_owner);
    }

private:

    friend class ResourceLease;
    /// Gives back the resources of @p_lease (called by the lease).
    void release(ResourceLease& p_lease);

    /// Resource names.
    std::vector<std::string> m_names;
    /// 1 when the resource failed.
    std::vector<std::uint8_t> m_failed;
    /// Exclusive holder of each resource.
    std::vector<OwnerId> m_owners;
    /// Number of shared holders of each resource.
    std::vector<std::uint16_t> m_users;
};

} // namespace robotik
