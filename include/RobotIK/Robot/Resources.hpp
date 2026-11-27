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
/// the @ref robotik::SkillScheduler. Storage is structure-of-arrays indexed by
/// a small
/// @ref robotik::ResourceId resolved once from the name.
///
/// @par Contract
/// - Promises: a resource is held by one skill at most; a failure is seen by
///   every skill that needs the resource, at the next scheduling.
/// - Expects: names resolved once to a @ref robotik::ResourceId.
/// - Errors: an unknown name throws @c std::invalid_argument; an unknown id
///   in a requirement is a conflict (nothing is acquired).
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
/// Owner of a lease taken outside the scheduler, and the writer of a
/// command sent outside a skill (scenario logic, teach pendant, test).
inline constexpr OwnerId ANONYMOUS = 0xFFFFFFFEU;
/// Writer of the safe states (a failed actuator, a reset): it may write to
/// any resource.
inline constexpr OwnerId SAFETY = 0xFFFFFFFDU;

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
    /// True when the requirement names no known resource (for example
    /// @ref NO_RESOURCE). Then @ref resource may be @ref NO_RESOURCE too.
    bool invalid = false;

    /// @return True when there is a conflict.
    [[nodiscard]] explicit operator bool() const
    {
        return resource != NO_RESOURCE || invalid;
    }
};

// ----------------------------------------------------------------------------
/// Sorts requirements by id and merges the duplicates: a resource asked
/// twice is kept once, and exclusive access wins over shared access.
/// @param p_requirements Requirements, maybe with duplicates.
/// @return One requirement per resource, sorted by id.
// ----------------------------------------------------------------------------
[[nodiscard]] std::vector<ResourceRequirement>
normalize(std::span<ResourceRequirement const> p_requirements);

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
    /// Gives the resources back now; the lease becomes empty.
    // -------------------------------------------------------------------------
    void release();

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
/// @ref DeviceSet); a scenario logic may add more (a table, a shared zone).
///
/// A resource is a part that two skills must not use at once in conflicting
/// ways: one arm, one gripper, a zone of the table. Exclusive access has
/// one holder; shared access (a camera stream) has many. A resource also has
/// a health: a fault makes it unavailable, and the scheduler stops the
/// skills that hold it. The manager only answers and records; it never
/// stops a skill itself (the @ref SkillScheduler does). With several robots,
/// one manager holds the resources of the whole world, so names are unique
/// in the world. A @c lease gives the resources back when it is released or
/// destroyed.
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
    /// Reserves all the requirements at once, or nothing. Duplicates are
    /// merged first (see @ref normalize).
    /// @param p_requirements Resources wanted (at most
    ///   @ref ResourceLease::CAPACITY different ones).
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

    // -------------------------------------------------------------------------
    /// Tells why requirements cannot be acquired now, without acquiring.
    /// @param p_requirements Resources wanted.
    /// @return The first reason; an empty conflict when all are free.
    // -------------------------------------------------------------------------
    [[nodiscard]] Conflict
    conflict(std::span<ResourceRequirement const> p_requirements) const;

    // --- Writes -------------------------------------------------------------

    // -------------------------------------------------------------------------
    /// Checks that the current @ref writer may command resource @p_id: the
    /// resource is free, or held exclusively by the writer. An actuator
    /// calls it before each command. One compare when the resource is free.
    /// @param p_id Resource of the actuator, or @ref NO_RESOURCE (no check).
    /// @param p_what What is written, for the message ("joint 'joint1'").
    /// @throws std::logic_error when another owner holds the resource: a
    ///   command that bypasses the scheduler is a mistake in the code.
    // -------------------------------------------------------------------------
    void checkWrite(ResourceId p_id, std::string_view p_what) const
    {
        if (p_id < m_owners.size() && m_owners[p_id] != NO_OWNER &&
            m_owners[p_id] != m_writer && m_writer != SAFETY)
        {
            refuseWrite(p_id, p_what);
        }
    }

    /// @return Who sends the commands now: the skill ticked by the
    ///   scheduler, else @ref ANONYMOUS (see @ref WriterScope).
    [[nodiscard]] OwnerId writer() const
    {
        return m_writer;
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

private:

    friend class ResourceLease;
    friend class WriterScope;
    /// Gives back the resources of @p_lease (called by the lease).
    void release(ResourceLease& p_lease);
    /// Throws the error of @ref checkWrite.
    [[noreturn]] void refuseWrite(ResourceId p_id,
                                  std::string_view p_what) const;

    /// Resource names.
    std::vector<std::string> m_names;
    /// 1 when the resource failed.
    std::vector<std::uint8_t> m_failed;
    /// Exclusive holder of each resource.
    std::vector<OwnerId> m_owners;
    /// Number of shared holders of each resource.
    std::vector<std::uint16_t> m_users;
    /// Who sends the commands now (see @ref WriterScope).
    OwnerId m_writer = ANONYMOUS;
};

// ****************************************************************************
/// Says who sends the commands during a scope, then restores the previous
/// writer.
///
/// The scheduler opens one around each call of a skill, with the skill id:
/// the actuators then refuse a command to a resource that another skill
/// holds (@ref ResourceManager::checkWrite). The robot opens one with
/// @ref SAFETY when it puts a failed actuator in its safe state. A skill
/// cannot open one: it gets the resources read only.
///
/// @code
/// {
///     robotik::WriterScope const scope(resources, skill_id);
///     skill.tick(context, dt); // its commands are checked against its lease
/// }
/// @endcode
// ****************************************************************************
class WriterScope
{
public:

    /// @param p_resources Manager of the resources written.
    /// @param p_writer Who writes during the scope.
    WriterScope(ResourceManager& p_resources, OwnerId p_writer)
        : m_resources(p_resources), m_previous(p_resources.m_writer)
    {
        p_resources.m_writer = p_writer;
    }

    /// Restores the previous writer.
    ~WriterScope()
    {
        m_resources.m_writer = m_previous;
    }

    WriterScope(WriterScope const&) = delete;
    WriterScope& operator=(WriterScope const&) = delete;

private:

    /// Manager whose writer is set.
    ResourceManager& m_resources;
    /// Writer before the scope.
    OwnerId m_previous;
};

} // namespace robotik
