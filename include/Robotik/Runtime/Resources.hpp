// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Resources.hpp
//! @brief What skills reserve to run (arm, gripper, camera...), and failures.
//!
//! A failure is just an unavailable resource: no skill-specific fault code.
//! Skills needing a failed resource cannot start; running ones are stopped by
//! the @ref SkillScheduler. Storage is structure-of-arrays indexed by a small
//! @ref ResourceId resolved once from the name.
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

using ResourceId = std::uint16_t;
inline constexpr ResourceId NO_RESOURCE = 0xFFFFu;

//! @brief Identifies who holds a lease (a skill id, or any user number).
using OwnerId = std::uint32_t;
inline constexpr OwnerId NO_OWNER = 0xFFFFFFFFu;
//! @brief Owner of a lease taken outside the scheduler.
inline constexpr OwnerId ANONYMOUS = 0xFFFFFFFEu;

// ****************************************************************************
//! @brief Access mode of a reservation.
// ****************************************************************************
enum class Access : std::uint8_t
{
    Shared,    //!< Several holders at once (a camera stream).
    Exclusive, //!< One holder (an arm, a gripper).
};

// ****************************************************************************
//! @brief One resource a skill needs, with its access mode.
// ****************************************************************************
struct ResourceRequirement
{
    ResourceId id = NO_RESOURCE;
    Access access = Access::Exclusive;
};

// ****************************************************************************
//! @brief Why a set of requirements cannot be acquired right now.
// ****************************************************************************
struct Conflict
{
    //!< First blocking resource, @ref NO_RESOURCE when there is no conflict.
    ResourceId resource = NO_RESOURCE;
    //!< Exclusive holder of that resource, or @ref NO_OWNER.
    OwnerId owner = NO_OWNER;
    //!< True when the resource failed rather than being busy.
    bool unavailable = false;

    [[nodiscard]] explicit operator bool() const
    {
        return resource != NO_RESOURCE;
    }
};

class ResourceManager;

// ****************************************************************************
//! @brief RAII reservation: releases its resources when destroyed.
// ****************************************************************************
class ResourceLease
{
public:

    static constexpr std::size_t CAPACITY = 8u;

    ResourceLease() = default;
    ResourceLease(ResourceLease&& p_other) noexcept;
    ResourceLease& operator=(ResourceLease&& p_other) noexcept;
    ResourceLease(ResourceLease const&) = delete;
    ResourceLease& operator=(ResourceLease const&) = delete;
    ~ResourceLease();

    //! @brief True when the lease holds its resources.
    [[nodiscard]] explicit operator bool() const
    {
        return m_manager != nullptr;
    }

    [[nodiscard]] OwnerId owner() const
    {
        return m_owner;
    }

    [[nodiscard]] std::span<ResourceRequirement const> resources() const
    {
        return { m_items.data(), m_count };
    }

    //! @brief Gives the resources back now.
    void release();

private:

    friend class ResourceManager;

    ResourceManager* m_manager = nullptr;
    OwnerId m_owner = NO_OWNER;
    std::array<ResourceRequirement, CAPACITY> m_items{};
    std::uint8_t m_count = 0;
};

// ****************************************************************************
//! @brief Registry of the robot resources, their holders and their health.
//!
//! @code
//! robotik::ResourceManager resources;
//! resources.add("arm");
//! resources.add("camera");
//! auto lease = resources.acquire({ resources.require("arm"),
//!     resources.require("camera", robotik::Access::Shared) });
//! resources.fail("camera"); // fault injection or real failure
//! @endcode
// ****************************************************************************
class ResourceManager
{
public:

    // -------------------------------------------------------------------------
    //! @brief Declares a resource; returns the existing id for a known name.
    // -------------------------------------------------------------------------
    ResourceId add(std::string p_name);

    //! @brief Id of @p_name, or @ref NO_RESOURCE.
    [[nodiscard]] ResourceId find(std::string_view p_name) const;

    // -------------------------------------------------------------------------
    //! @brief Requirement on a known resource.
    //! @throws std::invalid_argument for an unknown name.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceRequirement
    require(std::string_view p_name, Access p_access = Access::Exclusive) const;

    [[nodiscard]] std::size_t size() const
    {
        return m_names.size();
    }

    [[nodiscard]] std::string const& name(ResourceId p_id) const
    {
        return m_names[p_id];
    }

    // --- Health -------------------------------------------------------------

    void fail(ResourceId p_id);
    void restore(ResourceId p_id);
    void fail(std::string_view p_name);
    void restore(std::string_view p_name);
    //! @brief Restores every resource.
    void restoreAll();

    [[nodiscard]] bool available(ResourceId p_id) const
    {
        return p_id < m_failed.size() && m_failed[p_id] == 0u;
    }

    [[nodiscard]] bool available(std::string_view p_name) const
    {
        return available(find(p_name));
    }

    // --- Ownership ----------------------------------------------------------

    //! @brief Exclusive holder of @p_id, or @ref NO_OWNER.
    [[nodiscard]] OwnerId owner(ResourceId p_id) const
    {
        return m_owners[p_id];
    }

    //! @brief Number of shared holders of @p_id.
    [[nodiscard]] std::uint16_t users(ResourceId p_id) const
    {
        return m_users[p_id];
    }

    //! @brief First reason preventing the acquisition (none when free).
    [[nodiscard]] Conflict
    conflict(std::span<ResourceRequirement const> p_requirements) const;

    // -------------------------------------------------------------------------
    //! @brief Reserves all the requirements at once, or nothing.
    //! @return An empty lease on conflict.
    // -------------------------------------------------------------------------
    [[nodiscard]] ResourceLease
    acquire(std::span<ResourceRequirement const> p_requirements,
            OwnerId p_owner = ANONYMOUS);

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
    void release(ResourceLease& p_lease);

    std::vector<std::string> m_names;
    std::vector<std::uint8_t> m_failed;
    std::vector<OwnerId> m_owners;
    std::vector<std::uint16_t> m_users;
};

} // namespace robotik
